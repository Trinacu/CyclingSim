#include "sim.h"
#include "drafting.h"
#include "group.h"
#include "lateral_solver.h"
#include "rider.h"
#include "snapshot.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <unordered_map>

// Member initialisation order note:
// params is declared before lateral_solver_ in sim.h, so params is
// default-initialised before lateral_solver_ is constructed from it.
// This is correct regardless of the order listed in the initialiser list.
PhysicsEngine::PhysicsEngine(const Course* c)
    : course(c), lateral_solver_(params), group_tracker_(group_params_) {}

bool PhysicsEngine::add_rider(const RiderConfig cfg) {
  auto r = std::make_unique<Rider>(cfg);

  // could even raise here?
  if (riders.count(r->get_id()) > 0) {
    SDL_Log("PhysicsEngine::add_rider: Tried to add rider who is already in "
            "the list! %s",
            r->name.c_str());
    return false;
  }
  std::lock_guard<std::mutex> lock(frame_mtx);
  r->set_course(course);
  riders.emplace(cfg.rider_id, std::move(r));
  teams_.register_rider(cfg.rider_id, cfg.team_id);
  return true;
}

// --- Update pipeline ---
//
// Ordering guarantee for the snapshot:
//   step_and_snapshot() calls update(dt), then fill_snapshot().
//   update() calls all four phases, including step_lateral_apply() which writes
//   new lat_pos into each Rider.
//   fill_snapshot() reads lat_pos via Rider::snapshot() after the apply.
//   Therefore the snapshot always reflects the fully-resolved lateral state for
//   the current step — no off-by-one between longitudinal and lateral.

void PhysicsEngine::update(double dt) {
  step_group_classify();
  step_group_role_apply();
  step_draft_apply();
  step_rotation_apply(dt);
  step_follow_apply(dt);
  step_longitudinal(dt);
  step_lateral_behavior();
  step_lateral_solve(dt);
  step_lateral_apply();
}

void PhysicsEngine::step_and_snapshot(double dt, FrameSnapshot& out) {
  std::lock_guard<std::mutex> lock(frame_mtx);
  update(dt);
  fill_snapshot(out);
}

void PhysicsEngine::fill_snapshot(FrameSnapshot& out) const {
  // this needs to be called under phys_lock, but we lock in sim::step_fixed
  out.riders.clear();
  for (const auto& [id, r] : riders) {
    auto snap = r->snapshot();
    snap.group_id = group_tracker_.get_group_id(id);
    snap.group_role = group_tracker_.get_role(id);
    out.riders.emplace(id, std::move(snap));
  }
  out.groups = group_tracker_.get_snapshot(); // value copy; small at N<=20
}

const std::unordered_map<RiderId, std::unique_ptr<Rider>>&
PhysicsEngine::get_riders() const {
  return riders;
}

// this is (now) only used to set camera to first rider... kinda useless if
// fixed
const Rider* PhysicsEngine::get_rider_by_id(RiderId id) const {
  auto it = riders.find(id);
  if (it == riders.end()) {
    SDL_Log("Engine::get_rider_by_id: id %d not found", id);
    return nullptr;
  }
  return it->second.get();
}

// Physics-thread-only: reached via Simulation's command queue or the
// effort-schedule loop in step_fixed(), never directly from the UI.
void PhysicsEngine::set_rider_effort(int id, double effort) {
  auto it = riders.find(id);
  if (it == riders.end()) {
    SDL_Log("Engine::set_rider_effort: id %d not found", id);
    return;
  }
  it->second->set_effort(effort);
}

// --- Behavior management ---

void PhysicsEngine::set_rider_behavior(
    RiderId id, std::shared_ptr<ILateralBehavior> behavior) {
  if (behavior)
    behaviors_[id] = std::move(behavior);
  else
    behaviors_.erase(id);
}

void PhysicsEngine::clear_rider_behavior(RiderId id) { behaviors_.erase(id); }

// --- Follow-target management (physics-thread-only, like behaviors) ---

void PhysicsEngine::set_follow_target(RiderId rider, RiderId leader) {
  auto it = riders.find(rider);
  if (it == riders.end() || riders.count(leader) == 0 || rider == leader) {
    SDL_Log("Engine::set_follow_target: invalid pair rider %d -> leader %d",
            rider, leader);
    return;
  }
  // Bootstrap the integrator from the current effort so the controller takes
  // over smoothly instead of collapsing effort to ~0 and rebuilding it.
  double integ = it->second->get_target_effort();
  const double max_effort = it->second->get_config().max_effort;
  if (integ < 0.0)
    integ = 0.0;
  else if (integ > max_effort)
    integ = max_effort;
  follow_states_[rider] = FollowState{.leader = leader, .integrator = integ};
}

void PhysicsEngine::clear_follow_target(RiderId rider) {
  if (follow_states_.erase(rider) > 0) {
    // Drop the wake-axis steering along with the effort controller, or the
    // rider would keep springing toward the ex-leader's line forever.
    auto it = riders.find(rider);
    if (it != riders.end())
      it->second->clear_lat_target();
  }
}

void PhysicsEngine::clear_follow_targets() {
  for (const auto& [id, fs] : follow_states_) {
    auto it = riders.find(id);
    if (it != riders.end())
      it->second->clear_lat_target();
  }
  follow_states_.clear();
}

// --- Paceline rotation (physics-thread-only) ---

void PhysicsEngine::set_paceline_rotation(
    const std::vector<RotationMember>& roster, const RotationParams& params) {
  rotation_ = std::make_unique<PacelineRotation>(roster, params);
}

void PhysicsEngine::clear_paceline_rotation() { rotation_.reset(); }

void PhysicsEngine::clear_auto_rotations() {
  for (const auto& rot : auto_rotations_)
    for (RiderId id : rot->members())
      clear_follow_target(id);
  auto_rotations_.clear();
}

bool PhysicsEngine::promote_sitter(RiderId id) {
  if (rotation_ && rotation_->is_member(id))
    return rotation_->promote_sitter(id);
  for (auto& rot : auto_rotations_)
    if (rot->is_member(id))
      return rot->promote_sitter(id);
  return false;
}

const PacelineRotation* PhysicsEngine::get_rotation_for(RiderId id) const {
  if (rotation_ && rotation_->is_member(id))
    return rotation_.get();
  for (const auto& rot : auto_rotations_)
    if (rot->is_member(id))
      return rot.get();
  return nullptr;
}

// C2 reconcile: one rotation per group from declared Paceline roles.
// Deterministic by construction: the group snapshot is ordered front-to-back,
// members within a group are position-sorted, and auto_rotations_ keeps
// creation order.
void PhysicsEngine::reconcile_rotations() {
  const GroupSnapshot& groups = group_tracker_.get_snapshot();

  // Rotations whose members should stay this round (by rotation pointer).
  std::vector<PacelineRotation*> touched;

  for (const Group& g : groups) {
    // Declared set for this group: rider intent (get_group_role), skipping
    // the manual rotation's riders — that roster is API-owned.
    std::vector<RiderId> declared;
    for (const GroupMember& m : g.all_members()) {
      auto it = riders.find(m.id);
      if (it == riders.end())
        continue;
      if (it->second->get_group_role() != GroupRole::Paceline)
        continue;
      if (rotation_ && rotation_->is_member(m.id))
        continue;
      declared.push_back(m.id);
    }

    // Existing rotation for this group: first with any declared member.
    PacelineRotation* rot = nullptr;
    for (auto& r : auto_rotations_) {
      for (RiderId id : declared)
        if (r->is_member(id)) {
          rot = r.get();
          break;
        }
      if (rot)
        break;
    }

    if (!rot) {
      if (declared.size() < 2)
        continue; // nothing to form
      // Roster in position order, front first (same-group => already near).
      std::sort(declared.begin(), declared.end(),
                [this](RiderId a, RiderId b) {
                  return riders.at(a)->get_pos() > riders.at(b)->get_pos();
                });
      std::vector<RotationMember> roster;
      for (RiderId id : declared)
        roster.push_back({id, false});
      auto_rotations_.push_back(
          std::make_unique<PacelineRotation>(roster, auto_rotation_params_));
      touched.push_back(auto_rotations_.back().get());
      continue;
    }
    touched.push_back(rot);

    // Remove ex-declarers (left the group, un-declared, or went manual).
    for (RiderId id : rot->members()) {
      if (std::find(declared.begin(), declared.end(), id) != declared.end())
        continue;
      rot->remove_member(id);
      clear_follow_target(id);
    }
    // Admit new declarers within detach_gap of a current member (interim
    // proximity gate — C4's join maneuver replaces this with a real
    // approach).
    for (RiderId id : declared) {
      if (rot->is_member(id))
        continue;
      const double pos = riders.at(id)->get_pos();
      double nearest = std::numeric_limits<double>::infinity();
      for (RiderId mid : rot->members())
        nearest = std::min(nearest,
                           std::fabs(riders.at(mid)->get_pos() - pos));
      if (nearest <= auto_rotation_params_.detach_gap)
        rot->add_member(id, false);
    }
  }

  // Dissolve rotations that lost their group's declarers or fell below 2.
  auto_rotations_.erase(
      std::remove_if(auto_rotations_.begin(), auto_rotations_.end(),
                     [&](std::unique_ptr<PacelineRotation>& r) {
                       const bool keep =
                           std::find(touched.begin(), touched.end(),
                                     r.get()) != touched.end() &&
                           r->member_count() >= 2;
                       if (!keep)
                         for (RiderId id : r->members())
                           clear_follow_target(id);
                       return !keep;
                     }),
      auto_rotations_.end());
}

// Rotation phase: feed each coordinator flat member state, apply its
// directives to the follow subsystem.  Runs right before step_follow_apply
// so this tick's controllers see this tick's follow graph.
void PhysicsEngine::step_rotation_apply(double dt) {
  if (rotation_)
    apply_rotation(*rotation_, dt);
  for (auto& rot : auto_rotations_)
    apply_rotation(*rot, dt);
}

void PhysicsEngine::apply_rotation(PacelineRotation& rot, double dt) {
  rotation_inputs_.clear();
  for (const auto& [id, r] : riders) {
    if (!rot.is_member(id))
      continue;
    const auto [wind_dir, wind_speed] = course->get_wind(r->get_pos());
    rotation_inputs_.push_back(RotationInput{
        .id = id,
        .lon_pos = r->get_pos(),
        .speed = r->get_speed(),
        .bike_len = r->get_bike_len(),
        .crosswind =
            wind_speed * std::sin(wind_dir - r->get_heading()),
        .target_effort = r->get_target_effort(),
    });
  }

  const auto directives = rot.tick(dt, rotation_inputs_);
  for (const auto& d : directives) {
    auto it = riders.find(d.id);
    if (it == riders.end())
      continue;

    if (d.pulling) {
      clear_follow_target(d.id); // also drops the wake-axis lat target
      if (d.set_effort)
        it->second->set_effort(*d.set_effort);
      continue;
    }
    if (d.follow < 0)
      continue; // line fully depleted — leave the rider be

    auto fit = follow_states_.find(d.id);
    if (fit == follow_states_.end()) {
      set_follow_target(d.id, d.follow); // bootstraps the gap integrator
      fit = follow_states_.find(d.id);
      if (fit == follow_states_.end())
        continue; // set_follow_target rejected the pair
    } else {
      fit->second.leader = d.follow; // dynamic retarget keeps the integrator
    }

    if (d.swing_side != 0.0) {
      fit->second.side = d.swing_side;
      // Seed the drift speed-hold from the current effort so the swing-off
      // eases from the pull rather than dipping to zero and rebuilding.
      double seed = it->second->get_target_effort();
      const double max_effort = it->second->get_config().max_effort;
      if (seed < 0.0)
        seed = 0.0;
      else if (seed > max_effort)
        seed = max_effort;
      fit->second.drift_integrator = seed;
    }

    if (d.move_up_side != 0.0) {
      fit->second.approach_side = d.move_up_side;
      // Move-up effort cap (C-pre-b): 20% above the power needed to hold the
      // line's speed with the rider's current draft, floored at threshold —
      // enough to gain ground, never a sprint.  Recomputed every tick: as the
      // rider pulls out of the shelter its P_hold (and so the cap) rises.
      auto tit = riders.find(d.follow);
      const Rider& me = *it->second;
      if (tit != riders.end() && me.get_ftp() > 0.0) {
        const double p_hold = me.cruise_power(tit->second->get_speed());
        fit->second.effort_cap = std::max(1.0, 1.2 * p_hold / me.get_ftp());
      }
    }
  }

  for (RiderId id : rot.removed_last_tick())
    clear_follow_target(id);
}

// Follow phase: each gap controller writes its rider's target_effort — the
// single active effort writer for riders in Follow mode.  Runs before
// step_longitudinal() so the command applies this tick; positions are
// one tick stale, same as drafting.
void PhysicsEngine::step_follow_apply(double dt) {
  for (auto& [id, fs] : follow_states_) {
    auto rit = riders.find(id);
    auto lit = riders.find(fs.leader);
    if (rit == riders.end() || lit == riders.end())
      continue; // stale entry — rider or leader removed; hold last effort

    Rider& r = *rit->second;
    const Rider& leader = *lit->second;
    const FollowInput in{
        .gap = (leader.get_pos() - r.get_pos()) - leader.get_bike_len(),
        .rel_speed = leader.get_speed() - r.get_speed(),
        .own_speed = r.get_speed(),
        .max_effort = r.get_config().max_effort,
    };
    double effort = follow_effort(in, dt, fs.integrator, follow_params_);

    // Steer to the leader's wake axis (apparent-wind direction — the same
    // axis the shelter test scores against), not to the leader's line:
    // with crosswind the sweet spot is offset leeward, and this is what
    // forms echelons once B2 lands real wind.  An assigned ILateralBehavior
    // overrides this (it runs later in the tick) — that's the D3 swing-off
    // hook.
    double lat = wake_axis_lat(build_draft_state(fs.leader, leader),
                               r.get_pos());

    // Merging back after a pull (rotation, D3): hold v_leader - drift_delta
    // via the speed-hold PI, max-combined with the gap controller — far from
    // the tail the gap controller wants ~0 and the drift term paces the
    // fall-back; near the wheel the gap controller rises through it.  Ride
    // offset to the swing side, full while overlapped (gap <= 0), fading
    // linearly to sit exactly on the axis at the gap setpoint.
    if (fs.side != 0.0) {
      const double setpoint =
          follow_params_.d0 + follow_params_.h * in.own_speed;
      if (in.gap >= setpoint) {
        // Merge complete — the offset has already faded to 0 here, so
        // clearing causes no lateral step.  Hand the drift integrator's held
        // cruise effort to the gap integrator (same bootstrap discipline as
        // set_follow_target) so the takeover doesn't dip effort and reopen
        // the gap.
        fs.integrator = std::max(fs.integrator, fs.drift_integrator);
        fs.side = 0.0;
        fs.drift_integrator = 0.0;
      } else {
        const double v_err = (leader.get_speed() - follow_params_.drift_delta)
                             - r.get_speed();
        effort = std::max(effort,
                          drift_effort(v_err, dt, fs.drift_integrator,
                                       in.max_effort, follow_params_));
        const double fade =
            (in.gap <= 0.0) ? 1.0 : 1.0 - in.gap / setpoint;
        lat += fs.side * follow_params_.swing_offset_radii * r.get_radius() *
               fade;
      }
    }

    // Move-up transit (sitter promotion, C-pre-b; the C4 join reuses this):
    // approach the tail from behind riding offset to the advance side, effort
    // clamped to the cap set by the rotation phase.  The offset fades to 0
    // over the last approach_fade_len metres above the setpoint, so the
    // cut-in ends exactly on the wake axis; at the setpoint the transit is
    // complete and clearing causes no lateral step.
    if (fs.approach_side != 0.0) {
      const double setpoint =
          follow_params_.d0 + follow_params_.h * in.own_speed;
      if (in.gap <= setpoint) {
        fs.approach_side = 0.0;
        fs.effort_cap = -1.0;
      } else {
        if (fs.effort_cap >= 0.0)
          effort = std::min(effort, fs.effort_cap);
        const double fade = std::min(
            1.0, (in.gap - setpoint) / follow_params_.approach_fade_len);
        lat += fs.approach_side * follow_params_.swing_offset_radii *
               r.get_radius() * fade;
      }
    }

    r.set_effort(effort);
    r.set_lat_target(lat);
  }
}

// Drafting phase: compute per-rider CdA multipliers from formation geometry
// and write them into the riders.  Runs after the group phases (so
// roles/groups are current for this tick) and before step_longitudinal(),
// whose drag terms consume cda_factor.  Positions are one tick stale — fine
// at 100 Hz.
DraftRiderState PhysicsEngine::build_draft_state(RiderId id,
                                                 const Rider& r) const {
  const auto [wind_dir, wind_speed] = course->get_wind(r.get_pos());
  const double heading = r.get_heading();
  return DraftRiderState{
      .id = id,
      .group_id = group_tracker_.get_group_id(id),
      .role = group_tracker_.get_role(id),
      .lon_pos = r.get_pos(),
      .lat_pos = r.get_lat_pos(),
      .speed = r.get_speed(),
      .radius = r.get_radius(),
      .bike_len = r.get_bike_len(),
      .crosswind = wind_speed * std::sin(wind_dir - heading),
      .headwind = wind_speed * std::cos(wind_dir - heading),
  };
}

void PhysicsEngine::step_draft_apply() {
  draft_states_.clear();
  draft_states_.reserve(riders.size());

  for (const auto& [id, r] : riders)
    draft_states_.push_back(build_draft_state(id, *r));

  const std::vector<double> factors =
      compute_draft_factors(draft_states_, drafting_params_);
  for (size_t i = 0; i < draft_states_.size(); ++i) {
    auto it = riders.find(draft_states_[i].id);
    if (it != riders.end())
      it->second->set_cda_factor(factors[i]);
  }
}

// Phase 1: advance each rider's longitudinal physics independently.
void PhysicsEngine::step_longitudinal(double dt) {
  for (const auto& [id, r] : riders)
    r->update(dt);
}

// Phase 2: query each assigned behavior for an optional lateral target and
// write the result into the rider.  Also builds lat_states_ for reuse by
// step_lateral_solve().
//
// Why build lat_states_ here rather than in step_lateral_solve():
//   build_context() needs to filter nearby riders from lat_states_ while
//   behaviors are running.  Building the vector first, then calling behaviors,
//   means all proximity queries are consistent within the same step.
void PhysicsEngine::step_lateral_behavior() {
  // Build the shared state snapshot from post-longitudinal rider state.
  lat_states_.clear();
  lat_states_.reserve(riders.size());

  for (const auto& [id, r] : riders) {
    lat_states_.push_back(LateralRiderState{
        .id = id,
        .lon_pos = r->get_pos(),
        .speed = r->get_speed(),
        .lat_pos = r->get_lat_pos(),
        .lat_vel = r->get_lat_vel(),
        .lat_target = r->get_lat_target(),
        .w_prime_frac = r->get_energy_fraction(),
        .surplus_power = compute_surplus_power(*r),
        .mass = r->get_total_mass(),
        .rider_radius = r->get_radius(),
        .bike_length = r->get_bike_len(),
        .road_width = course->get_road_width(r->get_pos()),
    });
  }

  // Call each assigned behavior.
  for (const auto& [id, behavior] : behaviors_) {
    auto rider_it = riders.find(id);
    if (rider_it == riders.end())
      continue; // stale entry — rider was removed

    const LateralContext ctx = build_context(id);
    const std::optional<double> target = behavior->compute_lat_target(ctx);

    Rider& r = *rider_it->second;
    if (target.has_value())
      r.set_lat_target(target.value());
    else
      r.clear_lat_target();

    // Keep lat_states_ in sync so subsequent build_context() calls within
    // this step see the updated target.
    for (auto& s : lat_states_) {
      if (s.id == id) {
        s.lat_target = r.get_lat_target();
        break;
      }
    }
  }
}

// Phase 3: run the lateral solver over lat_states_ built in Phase 2.
void PhysicsEngine::step_lateral_solve(double dt) {
  // lat_states_ was populated (and behavior-updated) in
  // step_lateral_behavior(). If there are no assigned behaviors, it still
  // contains all riders with their current lat_target (typically nullopt) — the
  // solver handles that correctly.
  lat_updates_ = lateral_solver_.solve(lat_states_, dt);
}

// Phase 4: write solver output back into Rider objects.
//
// speed_penalty is passed through to Rider::apply_lateral_update(), but the
// state.speed *= speed_penalty line there is currently commented out — the
// penalty is computed and plumbed but intentionally disabled until tuned.
void PhysicsEngine::step_lateral_apply() {
  for (const auto& upd : lat_updates_) {
    auto it = riders.find(upd.id);
    if (it == riders.end())
      continue;
    it->second->apply_lateral_update(upd.new_lat_pos, upd.new_lat_vel,
                                     upd.speed_penalty);
  }
}

void PhysicsEngine::build_group_input() {
  group_input_.clear();
  group_input_.reserve(riders.size());
  for (const auto& [id, r] : riders) {
    group_input_.push_back(GroupMember{
        .id = id,
        .lon_pos = r->get_pos(),
        .speed = r->get_speed(),
        .role = GroupRole::Unassigned,
    });
  }
}

void PhysicsEngine::step_group_classify() {
  build_group_input();
  group_tracker_.update(group_input_);

  // for (const auto& g : group_tracker_.get_snapshot())
  //   SDL_Log("Group %d (%s): %d riders, front %.0f m, span %.0f m", g.ordinal,
  //           g.display_name.c_str(), g.size(), g.front_pos(), g.back_pos());
}

void PhysicsEngine::step_group_role_apply() {
  role_decls_.clear();
  for (const auto& [id, r] : riders) {
    const GroupRole role = r->get_group_role();
    if (role != GroupRole::Unassigned)
      role_decls_[id] = role;
  }
  group_tracker_.apply_role_declarations(role_decls_);
}

// --- Private helpers ---

// Approximate power consumed by longitudinal resistance at current speed.
// Uses a simplified model intentionally — surplus_power is used only to size
// the shove budget, not to drive physics.  Exact values are not required.
double PhysicsEngine::compute_surplus_power(const Rider& r) const {
  const double v = r.get_speed();
  const double rho = 1.2234;
  const double g = 9.80665;
  const double cda = r.get_config().cda * r.get_cda_factor();
  const double m = r.get_total_mass();
  const double slope = course->get_slope(r.get_pos());

  const double P_aero = 0.5 * rho * cda * v * v * v;
  const double P_roll = 0.006 * m * g * v; // crr ≈ 0.006
  const double P_grav = m * g * slope * v;

  return std::max(0.0, r.get_power() - (P_aero + P_roll + P_grav));
}

// Build a LateralContext for one rider from the current lat_states_ snapshot.
LateralContext PhysicsEngine::build_context(RiderId id) const {
  const LateralRiderState* own = nullptr;
  for (const auto& s : lat_states_) {
    if (s.id == id) {
      own = &s;
      break;
    }
  }
  if (!own)
    return {};

  LateralContext ctx;
  ctx.own_lat_pos = own->lat_pos;
  ctx.own_lat_vel = own->lat_vel;
  ctx.own_speed = own->speed;
  ctx.own_w_prime_frac = own->w_prime_frac;
  ctx.road_width = own->road_width;

  for (const auto& s : lat_states_) {
    if (s.id == id)
      continue;
    const double lon_offset = s.lon_pos - own->lon_pos;
    if (std::fabs(lon_offset) <= s.bike_length) {
      ctx.nearby.push_back(NearbyRider{
          .lon_offset = lon_offset,
          .lat_pos = s.lat_pos,
          .speed = s.speed,
          .w_prime_frac = s.w_prime_frac,
      });
    }
  }

  return ctx;
}

GroupContext PhysicsEngine::build_group_context(RiderId id) const {
  GroupContext ctx;

  const GroupId gid = group_tracker_.get_group_id(id);
  if (gid == kNoGroup)
    return ctx; // rider not in snapshot; return default

  const GroupSnapshot& snap = group_tracker_.get_snapshot();

  // gid == ordinal == index into snap (guaranteed by GroupTracker)
  if (gid < 0 || gid >= static_cast<int>(snap.size()))
    return ctx;

  const Group& group = snap[gid];

  ctx.own_group_id = gid;
  ctx.own_role = group_tracker_.get_role(id);
  ctx.group_ordinal = group.ordinal;
  ctx.group_size = group.size();
  ctx.paceline_size = static_cast<int>(group.paceline.size());
  ctx.body_size = static_cast<int>(group.body.size());

  // Paceline position: index of this rider in group.paceline
  // (already sorted front-to-back by apply_role_declarations)
  ctx.paceline_position = -1;
  for (int i = 0; i < static_cast<int>(group.paceline.size()); ++i) {
    if (group.paceline[i].id == id) {
      ctx.paceline_position = i;
      break;
    }
  }
  ctx.is_paceline_front = (ctx.paceline_position == 0);

  // is_group_front: true if this rider has the highest lon_pos in the group
  const double own_pos = [&]() -> double {
    for (const auto& m : group.all_members())
      if (m.id == id)
        return m.lon_pos;
    return -1.0;
  }();
  ctx.is_group_front = (own_pos >= group.front_pos() - 1e-6);

  // gap_to_group_ahead: distance from own group's front to the rear
  // of the group with ordinal (gid - 1)
  if (gid == 0) {
    ctx.gap_to_group_ahead = -1.0; // we are the front group
  } else {
    const Group& ahead = snap[gid - 1];
    ctx.gap_to_group_ahead = ahead.back_pos() - group.front_pos();
    // Clamp to zero — a negative value means groups are overlapping,
    // which the classifier should prevent but guard against anyway.
    if (ctx.gap_to_group_ahead < 0.0)
      ctx.gap_to_group_ahead = 0.0;
  }

  return ctx;
}

// SIMULATION

Simulation::Simulation(const Course* c) : engine(c), decision_(c) {}

// C0: derive race-style time gaps for the snapshot.  Groups are ordered
// front-to-back (ordinal 0 leads); each chasing group's gap is measured
// against the *rearmost* rider of the group ahead crossing this group's
// front position.
static void fill_time_gaps(FrameSnapshot& snap, const RaceClock& clock,
                           double now) {
  for (size_t gi = 1; gi < snap.groups.size(); ++gi) {
    Group& g = snap.groups[gi];
    const Group& ahead = snap.groups[gi - 1];

    RiderId rear = -1;
    double rear_pos = std::numeric_limits<double>::infinity();
    auto scan = [&](const std::vector<GroupMember>& v) {
      for (const auto& m : v)
        if (m.lon_pos < rear_pos) {
          rear_pos = m.lon_pos;
          rear = m.id;
        }
    };
    scan(ahead.paceline);
    scan(ahead.body);
    if (rear < 0)
      continue; // empty group ahead (shouldn't happen after update)

    const auto gap = clock.time_gap(rear, g.front_pos(), now);
    g.time_gap_ahead = gap.value_or(-1.0);
  }
}

void Simulation::add_riders(const std::vector<RiderConfig>& configs) {
  for (const auto& cfg : configs) {
    assert(cfg.rider_id >= 0);
    if (!engine.add_rider(cfg))
      SDL_Log("add_riders: engine rejected rider_id %d", cfg.rider_id);
  }
}

void Simulation::publish_snapshot() {
  snap_back.real_time = SDL_GetTicks() / 1000.0;

  std::scoped_lock lock(snapshot_swap_mtx);

  if (snap_back.sim_time <= snap_curr.sim_time)
    return; // physics didn't advance (shouldn't happen in step_fixed, but safe)

  snap_prev = std::move(snap_curr);
  snap_curr = std::move(snap_back);
}

bool Simulation::consume_latest_frame_pair(FrameSnapshot& out_prev,
                                           FrameSnapshot& out_curr) {
  std::scoped_lock lock(snapshot_swap_mtx);

  if (snap_curr.sim_time < 0.0)
    return false; // nothing published yet

  out_prev = snap_prev;
  out_curr = snap_curr;
  return true;
}

// Runs queued UI commands on the physics thread.  Swap under the lock,
// execute outside it, so command bodies can't deadlock against the queue.
void Simulation::drain_commands() {
  std::vector<std::function<void()>> cmds;
  {
    std::scoped_lock lock(commands_mtx);
    cmds.swap(pending_commands);
  }
  for (auto& cmd : cmds)
    cmd();
}

void Simulation::step_fixed(double dt) {
  drain_commands();

  // Schedules drive effort only when they are the active source — a follow
  // target takes precedence (EffortSource::Follow > Schedule).
  for (auto& [id, sched] : effort_schedules)
    if (!engine.has_follow_target(id))
      engine.set_rider_effort(id, sched->effort_at(sim_seconds));

  engine.step_and_snapshot(dt, snap_back);

  sim_seconds += dt;

  // Perception feed + snapshot post-processing (C0): the RaceClock sees the
  // post-step positions at the post-step time, then the group time gaps are
  // stamped into the outgoing frame.
  decision_.observe(engine, sim_seconds);
  fill_time_gaps(snap_back, decision_.race_clock(), sim_seconds);

  // Decision tick (C2): after the step and the perception feed, so contexts
  // read fully-resolved state and outputs take effect from the next step —
  // one tick of reaction delay, by design.
  decision_accum_ += dt;
  if (decision_accum_ >= decision_.decision_period()) {
    decision_accum_ = 0.0;
    decision_.decide(*this);
  }

  // C2: stamp effort ownership into the frame (only Simulation knows about
  // schedules and policies).
  for (auto& [id, snap] : snap_back.riders) {
    snap.effort_source = get_effort_source(id);
    if (const IRiderPolicy* p = decision_.get_policy(id))
      snap.policy = p->name();
  }

  snap_back.sim_time = sim_seconds;
  snap_back.sim_dt = dt;
  snap_back.time_factor = time_factor;

  publish_snapshot(); // acquires snapshot_swap_mtx
}

void Simulation::set_effort_schedule(int rider_id,
                                     std::shared_ptr<EffortSchedule> schedule) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this, rider_id, s = std::move(schedule)]() {
    effort_schedules[rider_id] = s;
    decision_.clear_policy(rider_id); // schedule replaces any policy (C2)
  });
}

void Simulation::set_rider_policy(RiderId rider_id,
                                  std::shared_ptr<IRiderPolicy> policy) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this, rider_id, p = std::move(policy)]() {
    effort_schedules.erase(rider_id); // policy replaces any schedule (C2)
    decision_.set_policy(rider_id, p);
  });
}

void Simulation::clear_rider_policy(RiderId rider_id) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back(
      [this, rider_id]() { decision_.clear_policy(rider_id); });
}

void Simulation::clear_effort_schedule(RiderId rider_id) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back(
      [this, rider_id]() { effort_schedules.erase(rider_id); });
}

void Simulation::set_rider_effort(RiderId rider_id, double effort) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this, rider_id, effort]() {
    // The slider acts only in Manual mode; Follow and Schedule own the
    // rider's effort exclusively while active.
    if (get_effort_source(rider_id) == EffortSource::Manual)
      engine.set_rider_effort(rider_id, effort);
  });
}

void Simulation::set_follow_target(RiderId rider, RiderId leader) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back(
      [this, rider, leader]() { engine.set_follow_target(rider, leader); });
}

void Simulation::clear_follow_target(RiderId rider) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back(
      [this, rider]() { engine.clear_follow_target(rider); });
}

void Simulation::set_paceline_rotation(std::vector<RotationMember> roster,
                                       RotationParams params) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this, roster = std::move(roster), params]() {
    engine.set_paceline_rotation(roster, params);
  });
}

void Simulation::clear_paceline_rotation() {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this]() { engine.clear_paceline_rotation(); });
}

void Simulation::promote_sitter(RiderId id) {
  std::scoped_lock lock(commands_mtx);
  pending_commands.push_back([this, id]() { engine.promote_sitter(id); });
}

EffortSource Simulation::get_effort_source(RiderId rider_id) const {
  if (engine.has_follow_target(rider_id))
    return EffortSource::Follow;
  if (decision_.has_policy(rider_id))
    return EffortSource::Policy;
  if (effort_schedules.count(rider_id) > 0)
    return EffortSource::Schedule;
  return EffortSource::Manual;
}

// Must be called only while no driver is stepping the sim (e.g. after
// RealtimeSimRunner::stop()). No locking needed because there's no
// concurrent access.
void Simulation::reset() {
  sim_seconds = 0.0;
  decision_accum_ = 0.0;
  effort_schedules.clear();
  engine.clear_paceline_rotation();
  engine.clear_auto_rotations();
  engine.clear_follow_targets();
  decision_.reset(); // drops traces and policies
  {
    std::scoped_lock lock(commands_mtx);
    pending_commands.clear();
  }

  // Clear the published snapshot buffers back to the "nothing published"
  // sentinel (sim_time -1).  Without this, publish_snapshot()'s monotonicity
  // guard (snap_back.sim_time <= snap_curr.sim_time) silently drops every
  // frame of the new run until its clock passes the old run's final time —
  // the display stays frozen for exactly that long.
  {
    std::scoped_lock lock(snapshot_swap_mtx);
    snap_prev = FrameSnapshot{};
    snap_curr = FrameSnapshot{};
    snap_back = FrameSnapshot{};
  }

  // get_riders() returns const ref, but unique_ptr<Rider> still lets us
  // call non-const methods on the Rider through the pointer.
  for (const auto& [id, r] : engine.get_riders())
    r->reset();
}

double Simulation::get_sim_seconds() const { return sim_seconds; }

const PhysicsEngine* Simulation::get_engine() const { return &engine; }
PhysicsEngine* Simulation::get_engine() { return &engine; }
