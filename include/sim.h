// sim.h
#ifndef SIM_H
#define SIM_H

#include "collision_params.h"
#include "course.h"
#include "decision.h"
#include "drafting.h"
#include "drafting_params.h"
#include "effortschedule.h"
#include "follow.h"
#include "follow_params.h"
#include "group.h"
#include "grouping_params.h"
#include "lateral_behavior.h"
#include "lateral_solver.h"
#include "rider.h"
#include "rotation.h"
#include "rotation_params.h"
#include "snapshot.h"
#include "team.h"
#include <functional>
#include <memory>
#include <mutex>

#include <atomic>
#include <unordered_map>
#include <vector>

class PhysicsEngine {
private:
  const Course* course;
  mutable std::mutex frame_mtx;
  std::unordered_map<RiderId, std::unique_ptr<Rider>> riders;

  void fill_snapshot(FrameSnapshot& out) const;

  CollisionParams params;
  LateralSolver lateral_solver_; // stateless; holds a copy of params
                                 //
  GroupingParams group_params_;
  GroupTracker group_tracker_;

  // Static team entities (C-pre-a); populated at setup, read by the decision
  // phase.  add_rider registers each rider with its config's team_id.
  TeamRegistry teams_;

  DraftingParams drafting_params_;

  // Pre-allocated buffers — cleared and refilled each tick, never
  // heap-allocated in the hot path.  Same pattern as lat_states_ and
  // lat_updates_.
  std::vector<GroupMember> group_input_;
  std::unordered_map<RiderId, GroupRole> role_decls_;

  // Optional per-rider behavior: absent → purely force-driven (lat_target
  // nullopt).
  // shared_ptr because behavior instances may be shared across riders
  // (e.g. every team member uses the same BlockBehavior instance).
  std::unordered_map<RiderId, std::shared_ptr<ILateralBehavior>> behaviors_;

  // lat_states_ is built once in step_lateral_behavior() and reused by
  // step_lateral_solve().  lat_updates_ is written by the solver and consumed
  // by step_lateral_apply().
  std::vector<LateralRiderState> lat_states_;
  std::vector<LateralUpdate> lat_updates_;

  // draft_states_ is rebuilt each tick by step_draft_apply().
  std::vector<DraftRiderState> draft_states_;

  FollowParams follow_params_;
  // Follow targets: presence of an entry means the rider is in Follow mode
  // (see EffortSource below) and this controller owns its target_effort.
  std::unordered_map<RiderId, FollowState> follow_states_;

  // Optional paceline rotation (D3).  Its directives drive the follow
  // subsystem each tick; members' follow targets are owned by the rotation
  // while it exists — don't mix with manual set_follow_target on the same
  // riders.
  std::unique_ptr<PacelineRotation> rotation_;
  std::vector<RotationInput> rotation_inputs_; // rebuilt each tick

  // Reconciled rotations (C2): formed per group from riders declaring
  // GroupRole::Paceline, by reconcile_rotations() at the decision cadence.
  // The manual rotation_ wins — its members are never reconciled.
  std::vector<std::unique_ptr<PacelineRotation>> auto_rotations_;
  RotationParams auto_rotation_params_;

  // One rotation's directives -> follow subsystem (the body shared by the
  // manual and reconciled rotations in step_rotation_apply).
  void apply_rotation(PacelineRotation& rot, double dt);

  // One rider's flat drafting input (position, extent, apparent-wind
  // components).  Used by step_draft_apply for every rider and by
  // step_follow_apply for each follower's leader.
  DraftRiderState build_draft_state(RiderId id, const Rider& r) const;

  void step_draft_apply(); // computes and writes per-rider cda_factor
  void step_rotation_apply(double dt); // rotation directives -> follow states
  void step_follow_apply(double dt); // gap controllers write target_effort
                                     // and wake-axis lat_target
  void step_longitudinal(double dt);
  void step_lateral_behavior();       // builds lat_states_, queries behaviors
  void step_lateral_solve(double dt); // calls lateral_solver_.solve()
  void step_lateral_apply();          // writes lat_updates_ back into riders

  void build_group_input();   // populates group_input_ from current rider state
  void step_group_classify(); // calls group_tracker_.update()
  void
  step_group_role_apply(); // calls group_tracker_.apply_role_declarations()

  // Approximate surplus power: rider output minus resistive losses at current
  // speed.  Used to populate LateralRiderState::surplus_power.
  // Intentionally a rough estimate — it is used only to size the shove budget,
  // not to drive longitudinal physics.
  double compute_surplus_power(const Rider& r) const;

  // Build a LateralContext for one rider from the current lat_states_ snapshot.
  // Nearby riders are filtered to those within one bike_length longitudinally.
  LateralContext build_context(RiderId id) const;
public:
  // Build a GroupContext for one rider from the current GroupTracker
  // snapshot — reflects the fully-resolved state of the last completed tick.
  // C1: consumed by DecisionSystem::build_context (this is what it was kept
  // for since the group phase landed).
  GroupContext build_group_context(RiderId id) const;

  // Read access for the decision layer's rider window (C1).  Snapshot state
  // of the last completed tick, physics-thread-only like everything else.
  const GroupTracker& get_group_tracker() const { return group_tracker_; }

  explicit PhysicsEngine(const Course* c);
  bool add_rider(const RiderConfig cfg);
  void update(double dt);

  // Setup-time (before stepping starts) — create teams first, then add
  // riders whose configs carry the returned TeamId.
  TeamId add_team(std::string name) { return teams_.add_team(std::move(name)); }
  const TeamRegistry& get_teams() const { return teams_; }

  const Course* get_course() const { return course; }
  double get_course_length() const { return course->get_total_length(); }

  // do these returns need to/should be const?
  const std::unordered_map<RiderId, std::unique_ptr<Rider>>& get_riders() const;
  const Rider* get_rider_by_id(RiderId id) const;

  // physicsengine mutates rider state
  void set_rider_effort(int id, double effort);

  void step_and_snapshot(double dt, FrameSnapshot& out);

  // Replaces any previously assigned behavior.  nullptr → clear_rider_behavior.
  void set_rider_behavior(RiderId id,
                          std::shared_ptr<ILateralBehavior> behavior);
  void clear_rider_behavior(RiderId id);

  // Follow targets (physics-thread-only, like set_rider_behavior: reached via
  // Simulation's command queue or test/setup code before stepping starts).
  // Assigning replaces any previous target and bootstraps the controller's
  // integrator from the rider's current target_effort, so the takeover never
  // steps effort discontinuously.
  void set_follow_target(RiderId rider, RiderId leader);
  void clear_follow_target(RiderId rider);
  void clear_follow_targets(); // all — used by Simulation::reset()
  bool has_follow_target(RiderId rider) const {
    return follow_states_.count(rider) > 0;
  }

  // Paceline rotation (physics-thread-only, like follow targets).  Replaces
  // any existing rotation; roster order is the initial line order (first
  // rotator = initial puller).
  void set_paceline_rotation(const std::vector<RotationMember>& roster,
                             const RotationParams& params);
  void clear_paceline_rotation();
  const PacelineRotation* get_paceline_rotation() const {
    return rotation_.get();
  }

  // SittingIn -> InLine promotion (C-pre-b; physics-thread-only).  First
  // sitter: pure roster bookkeeping.  Deeper sitter: enters move-up transit —
  // rides the advance side past the sitters ahead with effort capped at
  // max(1.0, 1.2 * P_hold / ftp), joining the InLine tail on arrival.
  // False when the rider is not a sitter of any rotation.
  bool promote_sitter(RiderId id);

  // C2 reconcile (physics-thread-only, decision cadence): form/update one
  // rotation per group from riders declaring GroupRole::Paceline.  The
  // manual rotation_ wins — its members are skipped.  A declarer is admitted
  // only when within detach_gap of an existing member (interim gate; C4's
  // join maneuver replaces it); an ex-declarer is removed; a rotation
  // shrinking below 2 dissolves.
  void reconcile_rotations();

  // The rotation (manual or reconciled) this rider belongs to, else nullptr.
  const PacelineRotation* get_rotation_for(RiderId id) const;

  void set_auto_rotation_params(const RotationParams& p) {
    auto_rotation_params_ = p;
  }
  int auto_rotation_count() const {
    return static_cast<int>(auto_rotations_.size());
  }

  void clear_auto_rotations();

  ~PhysicsEngine() = default;
};

// Passive fixed-step simulation engine: step_fixed + command queue +
// snapshot double-buffering.  It owns no thread — a driver calls step_fixed:
// RealtimeSimRunner (realtime_runner.h) paces it against the wall clock,
// OfflineSimulationRunner (analysis.h) steps it as fast as possible.
class Simulation {
private:
  PhysicsEngine engine;

  // Perception & decision layer (workstream C).  observe() feeds it every
  // step; decide() fires every decision_period of sim time (C2).
  DecisionSystem decision_;
  double decision_accum_ = 0.0;

  // written by the UI thread (via a driver), read by the physics loop
  std::atomic<double> time_factor{1.0};
  double sim_seconds = 0.0;

  // UI -> sim command funnel.  Public mutators push closures here; step_fixed
  // drains them on the physics thread, so engine/rider/schedule mutation is
  // single-threaded.
  mutable std::mutex commands_mtx;
  std::vector<std::function<void()>> pending_commands;
  void drain_commands(); // called at the top of step_fixed()

  double dt = 0.01; // 100 Hz physics

  std::unordered_map<int, std::shared_ptr<EffortSchedule>> effort_schedules;

  // Double buffers
  FrameSnapshot snap_prev; // published previous
  FrameSnapshot snap_curr; // published current
  FrameSnapshot snap_back; // build buffer
                           //
  // Called at the end of step_fixed(), while frame_mtx is still held
  void publish_snapshot();

  mutable std::mutex snapshot_swap_mtx;

public:
  Simulation(const Course* c);

  void add_riders(const std::vector<RiderConfig>& configs);

  void reset();

  void step_fixed(double dt);

  void set_time_factor(double f) { time_factor = f; }
  double get_time_factor() const { return time_factor; }

  double get_dt() { return dt; }
  void set_dt(double dt_) { dt = dt_; }

  double get_sim_seconds() const;
  const PhysicsEngine* get_engine() const;
  PhysicsEngine* get_engine();

  // Physics-thread state (like get_effort_source): call from the physics
  // thread or while no driver is stepping.
  const DecisionSystem& get_decision() const { return decision_; }

  // Queued: applied on the physics thread at the start of the next step.
  // set_rider_effort is a no-op unless the rider's EffortSource is Manual.
  // Policies and schedules are mutually exclusive: assigning either replaces
  // the other (C2).
  void set_effort_schedule(int rider_id,
                           std::shared_ptr<EffortSchedule> schedule);
  void clear_effort_schedule(RiderId rider_id);
  void set_rider_policy(RiderId rider_id,
                        std::shared_ptr<IRiderPolicy> policy);
  void clear_rider_policy(RiderId rider_id);
  void set_rider_effort(RiderId rider_id, double effort);
  void set_follow_target(RiderId rider, RiderId leader);
  void clear_follow_target(RiderId rider);
  void set_paceline_rotation(std::vector<RotationMember> roster,
                             RotationParams params);
  void clear_paceline_rotation();
  void promote_sitter(RiderId id);

  // Reads physics-thread state — call from the physics thread or while no
  // driver is stepping (tests, debug UI via snapshot preferred).
  EffortSource get_effort_source(RiderId rider_id) const;

  // Called by the renderer each frame; returns false if no new frame
  bool consume_latest_frame_pair(FrameSnapshot& out_prev,
                                 FrameSnapshot& out_curr);
};

class SimulationEndCondition {
public:
  virtual ~SimulationEndCondition() = default;
  virtual bool should_stop(const Simulation& sim) const = 0;
};

class FinishLineCondition : public SimulationEndCondition {
public:
  FinishLineCondition() {}

  bool should_stop(const Simulation& sim) const override {
    for (const auto& [id, r] : sim.get_engine()->get_riders()) {
      if (!r->finished()) {
        // SDL_Log("%s: %.1f", r->name.c_str(), r->pos);
        return false;
      }
    }
    SDL_Log("all riders finished");
    return true;
  }
};

class TimeLimitCondition : public SimulationEndCondition {
public:
  TimeLimitCondition(double max_time) : t(max_time) {}

  bool should_stop(const Simulation& sim) const override {
    return sim.get_sim_seconds() >= t;
  }

private:
  double t;
};

class OrEndCondition : public SimulationEndCondition {
public:
  OrEndCondition(std::unique_ptr<SimulationEndCondition> a,
                 std::unique_ptr<SimulationEndCondition> b)
      : lhs(std::move(a)), rhs(std::move(b)) {}

  bool should_stop(const Simulation& sim) const override {
    return lhs->should_stop(sim) || rhs->should_stop(sim);
  }

private:
  std::unique_ptr<SimulationEndCondition> lhs, rhs;
};

#endif
