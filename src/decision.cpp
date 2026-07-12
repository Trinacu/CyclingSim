#include "decision.h"
#include "sim.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr int kLastTableSlot =
    static_cast<int>(sizeof(DraftingParams::paceline_table) / sizeof(double)) -
    1;
} // namespace

DecisionSystem::DecisionSystem(const Course* course, DecisionParams params)
    : params_(params),
      clock_(course->get_total_length(), course->get_checkpoints(),
             params.grid_spacing),
      intel_(*course) {}

void DecisionSystem::observe(const PhysicsEngine& engine, double t) {
  // Per-rider traces are independent — iteration order is irrelevant here
  // (unlike the C2 decision phase, which must iterate in sorted id order).
  for (const auto& [id, r] : engine.get_riders())
    clock_.record(id, r->get_pos(), t);
}

void DecisionSystem::reset() {
  clock_.reset();
  policies_.clear();
  policy_follow_.clear();
}

// --- C2: policy management + the decision tick ---

void DecisionSystem::set_policy(RiderId id,
                                std::shared_ptr<IRiderPolicy> policy) {
  if (policy)
    policies_[id] = std::move(policy);
  else
    clear_policy(id);
}

void DecisionSystem::clear_policy(RiderId id) {
  policies_.erase(id);
  policy_follow_.erase(id);
}

const IRiderPolicy* DecisionSystem::get_policy(RiderId id) const {
  auto it = policies_.find(id);
  return it == policies_.end() ? nullptr : it->second.get();
}

void DecisionSystem::decide(Simulation& sim) {
  PhysicsEngine& eng = *sim.get_engine();

  // Sorted id order: cross-rider decisions must not depend on the riders
  // map's unspecified iteration order.
  std::vector<RiderId> ids;
  ids.reserve(policies_.size());
  for (const auto& [id, p] : policies_)
    ids.push_back(id);
  std::sort(ids.begin(), ids.end());

  for (RiderId id : ids) {
    if (!eng.get_rider_by_id(id)) {
      clear_policy(id); // rider left the sim
      continue;
    }
    const DecisionContext ctx = build_context(sim, id);
    const PolicyOutput out = policies_.at(id)->decide(ctx);

    // The rider's group role is policy-owned (declares paceline intent the
    // reconcile below consumes).
    eng.get_riders().at(id)->set_group_role(out.role_decl);

    // Follow: skip for rotation members (the rotation resolves the follow
    // graph every tick — a policy shapes it via roles/maneuvers instead).
    if (!eng.get_rotation_for(id)) {
      if (out.follow) {
        eng.set_follow_target(id, *out.follow);
        policy_follow_.insert(id);
      } else if (policy_follow_.count(id)) {
        eng.clear_follow_target(id);
        policy_follow_.erase(id);
      }
    }

    if (out.maneuver && out.maneuver->type == Maneuver::Type::PromoteSitter)
      eng.promote_sitter(id);

    // Held effort: lands on the ordinary path; a live follow controller
    // simply overwrites it every tick (Follow > Policy, by mechanism).
    if (out.target_effort)
      eng.set_rider_effort(id, *out.target_effort);
  }

  eng.reconcile_rotations();
}

// --- C1c: pace estimation helpers ---

PaceEstimate estimate_wprime_pace(const Rider& rider, double dist,
                                  double avg_gradient, double headwind,
                                  double wbal, double draft_factor) {
  const double ftp = rider.get_ftp();
  const double p_max = rider.get_config().max_effort * ftp;

  PaceEstimate est;
  est.power = ftp;
  if (dist <= 0.0 || ftp <= 0.0)
    return est;

  for (int i = 0; i < 4; ++i) {
    est.speed =
        rider.cruise_speed_at(est.power, avg_gradient, headwind, draft_factor);
    if (est.speed <= 0.0)
      break;
    est.duration = dist / est.speed;
    est.power = std::clamp(ftp + std::max(0.0, wbal) / est.duration, ftp,
                           p_max);
  }
  return est;
}

double rotation_avg_draft_factor(int n, const DraftingParams& p) {
  if (n <= 1)
    return 1.0;
  double sum = 0.0;
  for (int k = 0; k < n; ++k)
    sum += p.paceline_table[std::min(k, kLastTableSlot)];
  return sum / n;
}

double line_slot_draft_factor(int depth, const DraftingParams& p) {
  if (depth < 0)
    return 1.0;
  return p.paceline_table[std::min(depth, kLastTableSlot)];
}

// --- C3: W′-budgeted pacing policy ---

namespace {

// Positive climbing (m of elevation) inside [from, to], summed over the
// intel's climb index — overlap portions only.
double climbing_between(const CourseIntel& intel, double from, double to) {
  double elev = 0.0;
  for (const Climb& c : intel.climbs()) {
    const double lo = std::max(from, c.start);
    const double hi = std::min(to, c.crest_pos);
    if (hi > lo)
      elev += (hi - lo) * c.avg_gradient;
  }
  return elev;
}

} // namespace

WPrimePacingPolicy::WPrimePacingPolicy(WPrimePacingParams params)
    : params_(params) {}

PolicyOutput WPrimePacingPolicy::decide(const DecisionContext& ctx) {
  PolicyOutput out;
  out.role_decl = params_.role_decl;
  if (!ctx.intel || !ctx.self || ctx.ftp <= 0.0)
    return out;
  const CourseIntel& intel = *ctx.intel;

  // Horizon: the next crest if its climb starts within horizon_m (a rider
  // already mid-climb gets that crest), else the finish.
  const std::optional<Climb> climb = intel.next_climb(ctx.pos);
  const bool crest_horizon =
      climb && (climb->start - ctx.pos) <= params_.horizon_m;
  const bool on_climb = climb && ctx.pos >= climb->start;

  // Descent check on a short forward window (the point slope isn't in the
  // context, and the window smooths segment edges).  Suppressed mid-climb:
  // a short dip inside a merged climb must not drop the pace.
  const double look_end =
      std::min(ctx.pos + params_.descent_lookahead, intel.total_length());
  const double grad_here =
      look_end > ctx.pos ? intel.avg_gradient(ctx.pos, look_end) : 0.0;
  if (!on_climb && grad_here < params_.descent_gradient) {
    out.target_effort = std::min(params_.recovery_effort, ctx.effort_limit);
    return out;
  }

  // Approach at threshold: with a crest horizon but the climb not yet under
  // the wheels, hold FTP and open the budget at the foot — constant power
  // over a mixed flat+climb window would leak budget into the (aero-bound,
  // poor joules-per-second-gained) approach.
  if (crest_horizon && !on_climb) {
    out.target_effort = std::min(1.0, ctx.effort_limit);
    return out;
  }

  const double target_pos =
      crest_horizon ? climb->crest_pos : intel.total_length();
  const double dist = target_pos - ctx.pos;
  const double avg_grad =
      dist > 0.0 ? intel.avg_gradient(ctx.pos, target_pos) : 0.0;

  // Budget = wbal above the reserve floor, scaled by the horizon's share of
  // the climbing left on the whole course: a joule above FTP buys the most
  // time where gravity, not aero, sets the speed, so an early kicker must
  // not hijack the budget from the big climbs behind it.  A lone climb —
  // and the finish horizon, which contains everything left — gets share 1,
  // preserving the crest-at-the-floor property.
  const double elev_horizon = climbing_between(intel, ctx.pos, target_pos);
  const double elev_rest =
      climbing_between(intel, target_pos, intel.total_length());
  const double share =
      elev_rest > 0.0 ? elev_horizon / (elev_horizon + elev_rest) : 1.0;
  const double w_prime = ctx.self->get_config().w_prime_base;
  const double budget =
      share * std::max(0.0, ctx.wbal - params_.wbal_floor_frac * w_prime);
  double draft = 1.0;
  if (ctx.in_rotation)
    draft = ctx.sitting_in
                ? line_slot_draft_factor(ctx.rotation_size - 1, draft_)
                : rotation_avg_draft_factor(ctx.rotation_size, draft_);

  const PaceEstimate est =
      estimate_wprime_pace(*ctx.self, dist, avg_grad, 0.0, budget, draft);
  // Feasibility clamp: the energy model throttles realized effort anyway,
  // but the request should stay honest (it's what the UI shows).
  out.target_effort = std::min(est.power / ctx.ftp, ctx.effort_limit);
  return out;
}

// --- C1d: context construction ---

namespace {

// Rearmost member of a group (min lon_pos across both sub-containers).
RiderId rearmost_member(const Group& g) {
  RiderId rear = -1;
  double rear_pos = std::numeric_limits<double>::infinity();
  auto scan = [&](const std::vector<GroupMember>& v) {
    for (const auto& m : v)
      if (m.lon_pos < rear_pos) {
        rear_pos = m.lon_pos;
        rear = m.id;
      }
  };
  scan(g.paceline);
  scan(g.body);
  return rear;
}

} // namespace

DecisionContext DecisionSystem::build_context(const Simulation& sim,
                                              RiderId id) const {
  DecisionContext c;
  const PhysicsEngine& eng = *sim.get_engine();
  const Rider* r = eng.get_rider_by_id(id);
  if (!r)
    return c;

  c.id = id;
  c.team = r->get_team_id();
  c.pos = r->get_pos();
  c.speed = r->get_speed();
  c.heading = r->get_heading();
  c.wbal = r->get_energy();
  c.wbal_frac = r->get_energy_fraction();
  c.ftp = r->get_ftp();
  c.effort_limit = r->get_effort_limit();
  c.target_effort = r->get_target_effort();
  c.effort_source = sim.get_effort_source(id);

  if (const auto* rot = eng.get_rotation_for(id)) {
    c.in_rotation = true;
    c.rotation_size = rot->member_count();
    c.line_depth = rot->line_depth(id);
    c.sitting_in = rot->is_sitting(id);
  }

  c.group = eng.build_group_context(id);

  // Rider window: everyone within ±perception_horizon, rear-to-front.
  const GroupSnapshot& groups = eng.get_group_tracker().get_snapshot();
  for (const auto& [oid, other] : eng.get_riders()) {
    if (oid == id)
      continue;
    const double off = other->get_pos() - c.pos;
    if (std::fabs(off) > params_.perception_horizon)
      continue;
    PerceivedRider nb;
    nb.id = oid;
    nb.lon_offset = off;
    nb.speed = other->get_speed();
    const GroupId gid = eng.get_group_tracker().get_group_id(oid);
    for (const auto& g : groups)
      if (g.id == gid) {
        nb.group_ordinal = g.ordinal;
        nb.group_size = g.size();
        break;
      }
    c.nearby.push_back(nb);
  }
  std::sort(c.nearby.begin(), c.nearby.end(),
            [](const PerceivedRider& a, const PerceivedRider& b) {
              return a.lon_offset < b.lon_offset;
            });

  // Race-style time gaps via the C0 traces.  Snapshot index == ordinal
  // (front-to-back).  Ahead: when did the group ahead's rearmost rider cross
  // *my group's front*; behind: when did *my* rearmost cross theirs.
  c.now = sim.get_sim_seconds();
  const int ord = c.group.group_ordinal;
  if (c.group.own_group_id != kNoGroup && !groups.empty()) {
    if (ord > 0 && static_cast<size_t>(ord) < groups.size()) {
      const RiderId rear = rearmost_member(groups[ord - 1]);
      if (rear >= 0)
        c.time_gap_to_group_ahead =
            clock_.time_gap(rear, groups[ord].front_pos(), c.now)
                .value_or(-1.0);
    }
    if (static_cast<size_t>(ord + 1) < groups.size()) {
      const RiderId own_rear = rearmost_member(groups[ord]);
      if (own_rear >= 0)
        c.time_gap_to_group_behind =
            clock_.time_gap(own_rear, groups[ord + 1].front_pos(), c.now)
                .value_or(-1.0);
    }
  }

  c.intel = &intel_;
  c.clock = &clock_;
  c.self = r;
  return c;
}
