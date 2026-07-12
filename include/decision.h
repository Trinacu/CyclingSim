// decision.h — perception & decision layer (workstream C).
//
// C0 gave the skeleton (DecisionSystem + RaceClock feed); C1 the perception
// products (CourseIntel — owned here, one shared const digest — the
// per-rider DecisionContext, the W′-budget pace estimator); C2 the decision
// cadence: decide() runs every decision_period seconds of sim time, builds
// each policy rider's context (sorted id order — cross-rider decisions must
// be deterministic), lets its IRiderPolicy answer, applies the outputs
// through the engine's public API, then reconciles declared-role rotations.
// Outputs are *held* between ticks (nothing else writes a Policy rider's
// effort), and a decision made at tick N takes effect from N+1 — one tick of
// reaction delay is intended.  Team directors land in C4.
//
// Owned by Simulation (not PhysicsEngine): the engine stays pure mechanics,
// the decision layer reads it through its public API.  Everything runs on
// the physics thread (whichever thread calls Simulation::step_fixed) —
// decision *cadence* is lower, the thread is the same.

#ifndef DECISION_H
#define DECISION_H

#include "course.h"
#include "course_intel.h"
#include "drafting_params.h"
#include "group.h"
#include "mytypes.h"
#include "race_clock.h"
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

class PhysicsEngine;
class Rider;
class Simulation;

struct DecisionParams {
  double grid_spacing = 100.0;       // m, RaceClock crossing-time grid
  double perception_horizon = 200.0; // m, rider window is ±this
  double decision_period = 1.0;      // s of sim time between decide() ticks
};

// One rider the context owner can see (±perception_horizon).  Deliberately
// *no* W′ of strangers — you can't see another rider's legs; teammates' W′
// flows through the team director (C4), not through perception.
struct PerceivedRider {
  RiderId id = -1;
  double lon_offset = 0.0; // their pos - own pos (m), + = ahead
  double speed = 0.0;      // m/s
  int group_ordinal = -1;  // -1 = ungrouped
  int group_size = 0;
};

// Team-director command (C4 fills these; the inbox exists from C1 so
// policies can compile against the final context shape).
struct Directive {
  enum class Type { Free, Pull, SitIn, Chase, ProtectLeader };
  Type type = Type::Free;
  RiderId target = -1; // e.g. the leader to protect
};

// Everything one rider knows when deciding, rebuilt per rider per decision
// tick by DecisionSystem::build_context.
struct DecisionContext {
  // --- own state ---
  RiderId id = -1;
  TeamId team = kNoTeam;
  double pos = 0.0, speed = 0.0, heading = 0.0;
  double wbal = 0.0;          // J
  double wbal_frac = 0.0;     // [0, 1]
  double ftp = 0.0;           // current (degradable) FTP, W
  double effort_limit = 0.0;  // energy model's current cap
  double target_effort = 0.0; // FTP-relative
  EffortSource effort_source = EffortSource::Manual;

  // --- rotation membership (from the engine's PacelineRotation, if any) ---
  bool in_rotation = false;
  int rotation_size = 0; // total members
  int line_depth = -1;   // 0 = puller; -1 = not InLine
  bool sitting_in = false;

  // --- group topology (engine's GroupContext, finally wired) ---
  GroupContext group;

  // --- rider window, sorted rear-to-front by lon_offset ---
  std::vector<PerceivedRider> nearby;

  // --- race-style time gaps, seconds; -1 = none/unknown (C0 RaceClock) ---
  double time_gap_to_group_ahead = -1.0;
  double time_gap_to_group_behind = -1.0;

  // --- world knowledge handles + clock time for ad-hoc queries ---
  const CourseIntel* intel = nullptr;
  const RaceClock* clock = nullptr;
  // Own rider, const: the estimator's entry point (cruise_speed_at) lives on
  // Rider, and a rider knows their own body.  Strangers stay behind the
  // perception boundary above — never reach through this to the engine.
  const Rider* self = nullptr;
  double now = 0.0;

  // --- directive inbox (filled by the team director from C4 on) ---
  std::optional<Directive> directive;
};

// --- C1c: W′-budget pace estimation (pure helpers) ---

// The constant power that spends `wbal` joules exactly over `dist` metres of
// the given *average* gradient/headwind at `draft_factor`.  Fixed point of
// P = ftp + wbal / (dist / cruise_speed(P)), ~4 evaluations.  Deliberately
// rough (the honesty is the feature): averaged window instead of
// segment-by-segment, linear W′ depletion above FTP (matches the core), no
// FTP degradation or altitude — it reads as a rider's mental model, not an
// oracle.  Runs fine every decision tick: rolling re-planning self-corrects
// the roughness.
struct PaceEstimate {
  double power = 0.0;    // W, clamped to [ftp, max_effort * ftp]
  double speed = 0.0;    // m/s at that power
  double duration = 0.0; // s over dist
};
PaceEstimate estimate_wprime_pace(const Rider& rider, double dist,
                                  double avg_gradient, double headwind,
                                  double wbal, double draft_factor);

// Average CdA factor over one full rotation cycle of n riders: slot k takes
// paceline_table[min(k, last)] — entries clamp at the last value, so n
// beyond the table averages in more of it.  Assumes ideal alignment
// (falloff·align ≈ 1) — the right kind of optimism for a plan-ahead
// estimate.  n <= 1 → 1.0 (solo, nobody on the wheel).
double rotation_avg_draft_factor(int n, const DraftingParams& p);

// Steady CdA factor sitting at line depth d (0 = front, assumes a follower
// behind — in a line with sitters there is one).  d < 0 → 1.0.
double line_slot_draft_factor(int depth, const DraftingParams& p);

// --- C2: policies ---

// A physical action the policy asks the engine to start (grows in C4 with
// the paceline join).
struct Maneuver {
  enum class Type { PromoteSitter };
  Type type = Type::PromoteSitter;
};

// One decision tick's outputs.  Everything is *operated through* the
// existing arbitration, not around it: a follow request installs a normal
// follow target (whose controller then owns the effort — Follow outranks
// Policy); target_effort lands on the engine's ordinary effort path and
// simply holds until the next tick.
struct PolicyOutput {
  std::optional<double> target_effort; // FTP-relative
  std::optional<RiderId> follow; // set/keep a follow target; nullopt clears
                                 // a policy-installed one
  // Behind = follow the rider ahead; Ahead = protect them (C4) — `follow` is
  // then the ward.  Meaningful only when `follow` is set.
  FollowRelation follow_relation = FollowRelation::Behind;
  GroupRole role_decl = GroupRole::Unassigned; // policy owns the rider's role
  std::optional<Maneuver> maneuver;
};

// The meta-controller above EffortSource: selects modes and targets rather
// than competing as another 100 Hz writer.  Stateful (pull timers, hysteresis)
// — hence non-const decide; one instance per rider unless deliberately shared.
class IRiderPolicy {
public:
  virtual ~IRiderPolicy() = default;
  virtual PolicyOutput decide(const DecisionContext& ctx) = 0;
  virtual const char* name() const = 0;
};

// --- C3: W′-budgeted pacing policy ---

// Rolling re-plan at decision cadence: pick a horizon — the next crest if a
// climb lies within horizon_m, else the finish — and hold the constant power
// that spends the W′ budget (wbal minus a reserve) exactly over it
// (estimate_wprime_pace).  Descents are recovery, not spending: pushing W′
// into aero at descent speed is waste, so ride recovery_effort and let the
// next re-plan fold the recharge back into the budget.  Draft assumption
// from own rotation membership (the C1c helpers).  The estimator's
// roughness is self-correcting: every tick re-plans from actual wbal/pos.
struct WPrimePacingParams {
  double horizon_m = 3000.0; // a climb starting within this owns the horizon
  double wbal_floor_frac = 0.15; // reserve: W′ fraction never planned away
  double recovery_effort = 0.6;  // FTP-relative, on descents (W′ recharges)
  double descent_gradient = -0.01; // avg gradient below this reads as descent
  double descent_lookahead = 100.0; // m window for the descent check
  // Declared through PolicyOutput every tick (a policy rider's role is
  // policy-owned); Paceline makes the rider join chase rotations (C4-era).
  GroupRole role_decl = GroupRole::Unassigned;

  // --- C4 tactics (bounded deltas on the pacing baseline) ---
  double chase_delta = 0.15; // effort added over the baseline while chasing
  // Feasibility clamp: below this W′ fraction a Chase order is refused — the
  // rider-side clamp is the final authority over any directive.
  double chase_reserve_frac = 0.25;
  // > 0 arms the *own-initiative* chase rule (no directive needed): chase
  // while the group ahead trails by less than this many seconds.  A Chase
  // directive works regardless.  Off by default — un-directed riders pace
  // their own race (and the C3 gate scenario stays a pacing-only gate).
  double chase_gap_max = 0.0; // s
};

class WPrimePacingPolicy : public IRiderPolicy {
public:
  explicit WPrimePacingPolicy(WPrimePacingParams params = {});
  PolicyOutput decide(const DecisionContext& ctx) override;
  const char* name() const override { return "wp-pace"; }

private:
  // The C3 baseline: W′-budgeted pace toward the current horizon.
  double pacing_effort(const DecisionContext& ctx) const;
  // C4: directive obedience + own-initiative chase, applied over the
  // baseline.  Every order passes the feasibility clamp here.
  void apply_tactics(const DecisionContext& ctx, PolicyOutput& out) const;

  WPrimePacingParams params_;
  DraftingParams draft_;
};

// --- C4: race plan + team director ---

// Per-team race plan, set at scenario setup (Simulation::set_race_plan).
// Static intent, like the team roster itself — survives Simulation::reset.
struct RacePlan {
  RiderId leader = -1; // ProtectLeader assignments target this rider
  // Standing per-rider orders; a rider missing from the map rides Free.
  std::unordered_map<RiderId, Directive::Type> assignments;
  // > 0 enables the chase rule: a Free-assigned rider (never the leader) is
  // ordered to Chase while its group trails the group ahead by less than
  // this many seconds.
  double chase_gap_max = 0.0; // s
};

// One per team, inside DecisionSystem; runs *before* rider policies each
// decision tick (teams iterated in TeamId order).  Sees its own riders' full
// DecisionContexts including W′ — radio, deliberately the only cross-rider
// W′ visibility in the system — and emits one Directive per rider into the
// context inbox.  Directives are commands with a rider-side clamp: policies
// obey but clamp to feasibility (a cooked rider can't chase), which gives
// suggestion-like softness with no arbitration machinery.  Radio = team
// membership, not policy state: riders in human modes receive directives too
// (surfaced, not obeyed — the C-UI badge renders them).
class TeamDirector {
public:
  explicit TeamDirector(RacePlan plan) : plan_(std::move(plan)) {}
  const RacePlan& plan() const { return plan_; }

  // Contexts arrive sorted by id (decide() builds them that way); one
  // directive per context, appended to `out`.  Deterministic and stateless —
  // v1 rules are pure functions of the plan and the contexts.
  void direct(const std::vector<DecisionContext>& team,
              std::unordered_map<RiderId, Directive>& out) const;

private:
  RacePlan plan_;
};

// --- The system ---

class DecisionSystem {
public:
  explicit DecisionSystem(const Course* course, DecisionParams params = {});

  // Perception feed — call every physics step, after the engine stepped, with
  // the post-step sim time.  Per rider this is one gridline-index comparison
  // unless a gridline/checkpoint was actually crossed.
  void observe(const PhysicsEngine& engine, double t);

  // Drop all perception state (Simulation::reset).  CourseIntel is static
  // course knowledge and survives.
  void reset();

  // Build one rider's view of the world from the last completed tick.
  // Physics-thread-only (reads engine + Simulation state).  decide() calls
  // this per policy rider at the decision cadence.
  DecisionContext build_context(const Simulation& sim, RiderId id) const;

  // One decision tick (C2): contexts -> policies -> apply -> rotation
  // reconcile.  Called by Simulation::step_fixed every decision_period of
  // sim time, after the engine stepped and observe() ran.
  void decide(Simulation& sim);

  // Per-rider policy assignment (physics-thread-only; Simulation's queued
  // set_rider_policy is the UI-safe path).  Assigning replaces any previous
  // policy; nullptr clears.  A policy rider's group role, follow target and
  // held effort are policy-owned from the next tick.
  void set_policy(RiderId id, std::shared_ptr<IRiderPolicy> policy);
  void clear_policy(RiderId id);
  bool has_policy(RiderId id) const { return policies_.count(id) > 0; }
  const IRiderPolicy* get_policy(RiderId id) const;

  // Per-team race plan -> director (C4; physics-thread-only, Simulation's
  // queued set_race_plan is the UI-safe path).  Setting replaces any previous
  // plan for the team.  Plans are scenario configuration like the team
  // registry itself: reset() keeps them (it drops per-run state only).
  void set_race_plan(TeamId team, RacePlan plan);
  void clear_race_plan(TeamId team) { directors_.erase(team); }

  // The directive the last decision tick issued to this rider (surfaced for
  // tests and the C-UI badge; non-policy riders receive but don't obey).
  std::optional<Directive> last_directive(RiderId id) const;

  double decision_period() const { return params_.decision_period; }

  const RaceClock& race_clock() const { return clock_; }
  const CourseIntel& course_intel() const { return intel_; }

private:
  DecisionParams params_;
  RaceClock clock_;
  CourseIntel intel_;

  std::unordered_map<RiderId, std::shared_ptr<IRiderPolicy>> policies_;
  // Follow targets this layer installed (vs. rotation/UI ones): only these
  // are cleared when a policy stops emitting `follow`.
  std::set<RiderId> policy_follow_;

  // C4: one director per planned team; directives_ is rebuilt every decision
  // tick by the director phase (rider -> this tick's order).
  std::unordered_map<TeamId, TeamDirector> directors_;
  std::unordered_map<RiderId, Directive> directives_;
};

#endif
