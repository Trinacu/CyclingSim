# CyclingSim — Feature Roadmap (TODO #7, #6 + drafting)

Status: **outline for discussion** — decisions marked ⚖️ are open; everything else is the
current recommendation. Ordering: B1 → D1 → (B2 ∥ D2) → D3 → C, see Dependencies at the
bottom.

## Context

The audit-fix phase is done (see git history for the old PLAN.md): the sim core is
warning-free, fully tested, single-solver (ACCEL_FORCE), and the realtime driver is
extracted. Workstream A (lateral model, TODO #9+10) is also done — see below. Remaining
feature areas:

- **#7** — wind: `Course::get_wind()` is a stub returning a constant `Wind{0,1}` (a
  permanent 1 m/s wind!), rider `heading` is never set from the course, and there is no
  crosswind concept. Longitudinal drag already includes rider speed correctly
  (`v_air = v + env->headwind` in sim_core.c).
- **#6** — riders have no decision layer: effort comes from static `EffortSchedule`s or
  the UI slider. Riders should "see" the course (full profile — the course book) and the
  riders around them (±200 m only) and decide effort/tactics from that.

---

## Workstream A — Lateral model: penalty targeting + centering (#9, #10) — DONE

Completed 2026-07-07. Penalty applies only to a blocked squeezer, ramped by longitudinal
offset (side-by-side is free); ambient centering via `CollisionParams::ambient_center_k`
(not W′-scaled); penalty re-enabled in `Rider::apply_lateral_update`. Constants
(`kPenaltyScale` etc.) stayed as locals in `compute_shove` — tune by recompile.
**Still open:** interactive feel-check / tuning of `kPenaltyScale` / `kMaxPenaltyRate`.

---

## Workstream B — Wind (#7), phased

**Goal (B1):** real wind exists per course and affects longitudinal physics correctly.
**Goal (B2):** crosswind exerts lateral force via the lateral solver.

### B1. Longitudinal wind (data + projection)

What's already right: sim_core's `v_air = v + env->headwind` (all three resistance
paths) — drag correctly depends on rider speed. The scalar-headwind contract into the C
core can stay for B1.

What's missing / wrong:
1. **Data source**: `Course::get_wind` returns constant `Wind{0,1}`. Give `Course` a real
   wind field: a course-level base `Wind` set by the factories (`create_endulating` etc.,
   default speed 0). `Segment` can carry an optional override later (sheltered sections)
   — not in B1.
2. **Rider heading**: `Rider::heading` is initialized to 0 and never updated, but
   `Segment` already has a `heading` field. Expose `ICourseView::get_heading(pos)`
   (include/course.h) and set `rider.heading` from it in `Rider::update`. The existing
   projection `headwind = wind_speed * cos(wind_dir − heading)` (src/rider.cpp) then
   becomes meaningful on courses with heading changes.
3. Default all factory courses to `speed = 0` — removes today's phantom 1 m/s wind
   (expect tiny speed changes in plots/tests that compare absolute speeds).

### B2. Crosswind → lateral force

Key architectural point: lateral physics lives in the C++ `LateralSolver`, **not** in the
C core — so crosswind force belongs there, not in `EnvState`.

- Engine computes the signed crosswind component per rider
  (`wind_speed * sin(wind_dir − heading)`) and passes it into `LateralRiderState`
  (new field, e.g. `crosswind`).
- `free_movement` adds a lateral acceleration term: side force ≈
  `0.5 * rho * CdA_side * (crosswind)² * sign`, mass-normalized; fights the
  spring/damper like any other force. `CdA_side` as a `CollisionParams` (or rider)
  constant to start.
- ⚖️ Optional extra (separate decision): yaw-angle drag increase in the longitudinal
  model — would need crosswind in `EnvState` (C ABI change). Not needed for the lateral
  feel; park it.
- Future note: crosswind + drafting is what makes echelons emerge. The drafting side is
  now Workstream D; D1's wake-axis abstraction is the hook where crosswind plugs in.

### B. Files & tests

- B1: `include/course.h`, `src/course.cpp` (wind field + `get_heading`), `src/rider.cpp`
  (set heading), course factories.
- B2: `include/lateral_solver.h` (`LateralRiderState::crosswind`),
  `src/lateral_solver.cpp` (force term), `src/sim.cpp` (fill field in
  `step_lateral_behavior`'s state build).
- Tests: core-level — terminal velocity with head/tailwind asymmetry
  (`tests/core/test_terminal_velocity.c` pattern); C++ — projection correctness on a
  course with varied segment headings; B2 — rider drifts leeward without target, holds
  line (with effort) when target set.

---

## Workstream D — Drafting

**Goal:** riders in formation pay less aero power via the existing `cda_factor` hook
(sim_core.c already multiplies it into all three drag paths — no C ABI change).
Precise TTT-style drafting inside a paceline; coarser, more pronounced sheltering in
the group body. Drafting lands **before** the decision layer (C): it is the physical
reality that tactics later reason about.

Drafting decomposes into three separable problems, implemented in order:

1. **D1 — aero model**: who shelters whom, how much (`cda_factor` per rider per tick).
   Purely physical, no decisions; testable alone with effort schedules.
2. **D2 — gap-holding**: a follow controller so a rider can *stay* on a wheel.
   Without it pacelines drift apart; with it they hold together at matched effort.
3. **D3 — rotation**: pull policy at the front, swing off, drop back, reattach.

### D1. Aero model — DONE

Completed 2026-07-08. Amended same day by D3.0 (best-wheel link selection — see D3). Pure solver in `include/drafting.h` / `src/drafting.cpp`
(`compute_draft_factors`), params in `include/drafting_params.h`; wired via
`PhysicsEngine::step_draft_apply()` (before `step_longitudinal`, one-tick-stale
positions). `Rider::set_cda_factor` writes the core's `cda_factor`; the factor is in
`RiderSnapshot`; `compute_surplus_power` includes it. Tests in
`tests/test_drafting.cpp` incl. the first engine-level integration test.

Model as landed (scope decision: **chain model applies to every non-`Body` rider** —
nothing declares roles yet; the body heuristic is implemented but dormant behind the
`GroupRole::Body` gate):
- `paceline_table = {0.98, 0.61, 0.50, 0.44, 0.42, 0.41}`, saturating from P6 (deeper
  clamps to 0.41); entry 0 is the front rider's ~2% push from a follower on the wheel.
- Gap falloff (wheel-to-wheel): benefit ×1.0 at contact → ×0.7 at 5 m → 0 at 8 m
  (piecewise linear; 8 m is also the chain-link cutoff).
- Lateral alignment: benefit fades linearly to 0 at 3 × leader radius of displacement
  from the leader's *wake axis*; the axis trails along the leader's apparent wind
  (straight behind with today's stub wind — B2's crosswind rotates it → echelons).
- `cda_factor = 1 − (1 − table(depth))·falloff·align`, with **continuous chain depth**
  (`depth = 1 + s_leader·depth_leader`, table linearly interpolated) and the front-rider
  push weighted by (1 − own link strength) — both so a chain splitting/reforming never
  steps CdA discontinuously (100 Hz jitter rule).
- Body heuristic (dormant): `body_curve = {0.90, 0.60, 0.50, 0.47}` by riders ahead
  within 10 m, same group. ⚖️ Still open: curve/floor values and smoothing — tune when
  roles go live.

### D2. Gap-holding (follow controller) — design settled 2026-07-08

A paceline is only a paceline if followers hold the wheel. Runs at physics cadence
(100 Hz), not decision cadence.

**Effort ownership (not modulation).** `target_effort` has exactly **one writer per
rider at any time**, selected by a per-rider *effort source* mode:
`Schedule` (existing `EffortSchedule`) | `Manual` (`set_rider_effort` / UI slider) |
`Follow` (this controller). The mode is the arbiter — no blending rule exists. The UI
slider is **inert in Follow mode**; it acts only in Manual. Modes are set via engine
API (command queue) for now; the decision layer (C2/C4) selects them later — this is
the deliberate thin manual slice of C. Broader "behavior modes" that bundle effort
source + lateral behavior + `GroupRole` (e.g. *rotate in paceline*) stay a C2 concept;
D2 ships only the effort-source arbitration.

**Follow target.** Per-rider optional (analogous to `lat_target`): leader `RiderId` +
controller state. Engine API `set_follow_target(rider, leader)` /
`clear_follow_target(rider)`, mirroring `set_rider_behavior`, queued like all
cross-thread commands.

**Gap = D1's definition**, verbatim: wheel-to-wheel,
`gap = lon_sep − leader.bike_len` (`bike_len = wheelbase + 2·wheel_r`) — the same
measure the drafting falloff uses, so "0.25 m" means the same thing to both systems.

**Setpoint:** `target_gap = d0 + h·v`, with `d0 = 0.25 m`, `h = 0 s` default. `h` is
the string-stability escape hatch: if the accordion test shows oscillation growing down
the chain, a tiny headway (~0.02–0.05 s) damps it, and D1's falloff (≈ full benefit
under 1 m) makes the cost negligible. ⚖️ Open: final gains and whether `h` stays 0 —
settled empirically by the accordion test.

**Controller:** PI(D) on `e = gap − target_gap`, output written to `target_effort`.
- **I is mandatory** — at converged steady state (e = 0) it holds the entire cruise
  effort; P alone sags permanently behind the target.
- **D on relative speed** (= de/dt, exact in a sim — no noise) provides damping.
- **Windup: integrator hard-clamped to `[0, max_effort]`** (static rider param —
  `rider.h` `max_effort`). No realized-effort read-back, no back-calculation, and
  explicitly **no zeroing on overshoot** (the integrator holds cruise effort; wiping it
  when the gap dips below target produces a surge–coast limit cycle as the *normal*
  operating mode). The `≥ 0` clamp kills overlap windup: during an overrun the
  integrator bleeds toward 0 but never negative, so recovery when the leader resumes is
  immediate. Known bounded residual: a fatigued rider whose dynamic `effort_limit` sits
  below `max_effort` recovers slightly sluggishly — acceptable (reads as realistic);
  upgrade path is local back-calculation if tests say otherwise.

**Effort limit stays out of the controller** — it belongs to the energy model, which
already clamps realized effort to `[0, energy_effort_limit]` every step
(`core/src/sim_core.c`, effort resolution). A dying rider getting dropped is emergent;
the controller never learns why its request wasn't met.

**No braking.** Effort floors at 0; deceleration is drag-only. Wheel overlap is
tolerated (the lateral/collision model handles contact). Failure mode to watch at low
speed where drag is weak.

**Plumbing (mirrors D1):** pure controller in new `include/follow.h` /
`src/follow.cpp` + `include/follow_params.h` (`d0`, `h`, gains — sibling of
`DraftingParams`); engine phase `step_follow_apply()` next to `step_draft_apply()`
(before `step_longitudinal`, one-tick-stale positions fine); per-rider mode + follow
state in `PhysicsEngine` beside `behaviors_`.

**Tests (`tests/test_follow.cpp`):**
- Convergence: leader on constant schedule, follower converges to 0.25 m and holds
  (integral carries cruise effort at e ≈ 0, no sawtooth).
- Leader slowdown → overlap → resume: follower recovers immediately (integrator never
  went negative — no dead-follower delay), no huge overshoot.
- **Accordion test (tuning gate):** 5-rider chain, leader steps effort up/down; gap
  oscillation amplitude must *decay* down the chain, not grow. Decides gains and `h`.
- Weak follower (low FTP) gets dropped emergently: gap grows past D1's 8 m cutoff,
  draft lost — no controller knowledge of `effort_limit` needed.
- Mode arbitration: `set_rider_effort` is ignored while in Follow mode; mode switch
  back to Manual restores the slider path.
- Determinism / dt-independence of the controller step.

### D3. Rotation — design settled 2026-07-08, implemented 2026-07-09

Implementation notes (beyond the design below): `PacelineRotation` in
`include/rotation.h` / `src/rotation.cpp`, params in `include/rotation_params.h`;
merge mechanics (drift speed-hold PI max-combined with the gap controller, swing
offset fading over [0, setpoint] wheel gap) live in `FollowState`/`follow.cpp` and
`step_follow_apply`; on merge completion the drift integrator seeds the gap
integrator so the takeover doesn't dip effort. Tests in `tests/test_rotation.cpp`.
Accepted finding from the multi-drifter stress test: sustained deep flight (short
`pull_time` drains the line to 2) can swap adjacent drifters' merge order —
physically induced, deterministic, pack coheres; order is only guaranteed for
single-drifter flight.

Scoped to paceline mechanics, not tactics (C4 later decides *whether/who* participates
by editing the roster). Swing-off is **not** an `ILateralBehavior` (earlier sketch
dropped): the drift lateral needs the follow gap as its signal, which behaviors can't
see — it lives in the follow subsystem. The behavior-override hook stays reserved for
real tactics.

#### D3.0 Prerequisite — best-wheel link selection (D1 amendment)

D1's "nearest strictly-ahead draftable wheel" rule breaks under rotation merges:
(1) a barely-aligned near wheel beats a well-aligned far wheel (wrong shelter), and
(2) when the near wheel's alignment fades to exactly 0 it stops being a candidate and
the link *snaps* to the farther wheel — a step of up to ~0.2 in `cda_factor`, violating
the 100 Hz continuity rule. A tidy single chain never exercises this; a rotating
paceline does every cycle.

Fix: evaluate the full benefit `(1 − table(depth_via_j)) · falloff · align` for the
**longitudinally closest `link_candidates` (= 3) in-range riders ahead** and link to the
argmax. Max of continuous functions → continuous across wheel switches; clean chains
are unchanged (nearest = best there). Accepted residuals, documented in the header:
tiny step in the ~2% front-push transfer at a switch; candidate-set changes beyond the
top-3 can step (rare — a rider whose 3 nearest wheels are all offset gets no draft from
a 4th aligned one).

#### Roster & roles

`PacelineRotation` (new `include/rotation.h` / `src/rotation.cpp`), engine-owned,
ticked in `step_rotation_apply()` before the follow phase. It owns an explicit ordered
**roster** — a membership contract, deliberately separate from `GroupTracker` (which
stays the emergent proximity classifier for aero/display). Members are `Rotator`
(take pulls) or `SittingIn` (never pull, ride the rear). Rotator states: `Pulling`,
`InLine`, `Drifting`.

#### Follow graph — resolved dynamically every tick (no event rewiring)

- Puller: no follow target; own effort source is live (each rider decides its own
  pace — on promotion the new puller inherits the previous puller's `target_effort`
  for now; a C3 policy replaces that later. ⚖️ C-era: group pace negotiation.)
- `InLine` rotator: follows the previous rider in InLine order.
- `Drifting` rotator: follows the **last InLine rider** (dynamic).
- First `SittingIn` rider: follows the **last InLine rider** (same rule — this is what
  makes the merge choreography emerge); the rest chain behind each other.

#### Rotation event & drift mechanics

- Pull trigger: time-based (`pull_time` param); W′-based variants slot in behind the
  same seam later (note: W′ triggers only work above FTP pace). Guard: no new rotation
  if the InLine count would drop below 2.
- Swing side: **windward**, from the same crosswind component the wake axis uses;
  `default_side` param while crosswind ≈ 0 (until B1 lands real wind/heading).
- Drift lateral: wake axis ± `3 · rider_radius`; full offset while the wheel gap to the
  rider ahead in line is ≤ 0, then fades linearly to sit exactly on the axis when the
  gap reaches the setpoint.
- Drift longitudinal: speed-hold PI at `v_leader − drift_delta` (~0.4 m/s), combined
  with the D2 gap controller as **max(drift effort, gap effort)** — smooth handover,
  kd gives the anticipatory speed-up, arrival speed bounded → asymptotic approach, no
  overshoot-sprint (energy-honest).
- Attach: when the drifter's **position** drops below the last InLine rider's
  (pos delta > 0 — not wheel gap), append to InLine order. One-shot and monotone → no
  timer/hysteresis. "InLine" is bookkeeping; the lateral fade finishes the merge
  physically. At that instant the first SittingIn rider's dynamic target flips to the
  new tail and computes to ≈ the setpoint gap already (geometry) — its controller then
  opens the slot naturally as the merger drops in.
- Accepted quirk: the SittingIn rider wiggles ~0.5 m toward the merging rider's wake
  (D2 steering targets the leader's actual axis). Ship it; upgrade path if it looks
  twitchy: key the follower's axis on the leader's lat *target* (intent) instead of
  position. D3.0 guarantees the wiggle at least prices the aero correctly.
- Dropped members: an InLine/SittingIn member whose gap to the rider ahead exceeds
  `max_draft_gap` persistently (a few seconds) is removed from the roster — physics
  already dropped them; smarter role handling is C4-era.

#### D3 files & tests

- `include/rotation.h`, `src/rotation.cpp`; `include/follow.h`, `src/follow.cpp`
  (Drifting mode: side, `drift_delta`, speed-hold + max-combine); `include/sim.h`,
  `src/sim.cpp` (`step_rotation_apply`, roster API); `src/drafting.cpp` (D3.0).
- Tests: D3.0 in `tests/test_drafting.cpp` (best-wheel beats nearest-wheel; factor
  continuous under a lateral sweep of the merging rider; candidate-cap semantics);
  `tests/test_rotation.cpp` (front sequence cycles through all rotators repeatedly;
  line holds through swaps; multi-drifter flight with short `pull_time`; SittingIn
  slot-open on merge; weak member removed from roster; determinism).
- Demo: appstate becomes a rotating paceline (~6 rotators + 1 SittingIn).

### D. Files & tests

- D1 landed; **D2 designed (see above)** — files: new `include/follow.h`,
  `src/follow.cpp`, `include/follow_params.h`, `tests/test_follow.cpp`; touch
  `include/sim.h`, `src/sim.cpp` (mode map, `step_follow_apply`, queued APIs).
  D3 gets its test plan when designed.

---

## Workstream C — Perception & decision layer (#6)

**Goal:** a per-rider `DecisionContext` (course window + rider window) driving two first
consumers: effort pacing and group tactics. Same architecture pattern as the lateral
pipeline (engine builds context → pure strategy object decides → engine applies).

### C1. DecisionContext (new `include/decision.h`)

Two windows, both from the start:

- **Course knowledge — perfect, whole course** ("the course book"): riders may query the
  full profile. Practical API: not raw segments but digested queries the consumers need —
  `distance_to_finish`, `avg_gradient(from, to)`, `next_climb()` (start/length/avg %),
  `distance_to_crest()`. Backed by `ICourseView` (already on every rider); consider
  precomputing a climb index once per course.
- **Rider awareness — limited, ±200 m** (perception horizon, parameter): descriptors like
  `NearbyRider` but for the decision scale: `lon_offset`, `speed`, `group ordinal/size`
  (from the group snapshot), and (⚖️ open) whether `w_prime_frac` of others is visible —
  recommend **no** (you can't see another rider's legs; maybe a noisy "looks fresh /
  struggling" signal later).
- Reuses: `GroupContext` + `PhysicsEngine::build_group_context` (include/sim.h — built,
  never wired; this is what it was kept for), `GroupTracker` snapshot for group data.
- **Race-style time gaps** ("chase at 0:45") as a *derived perception quantity* here
  (and in the UI) — e.g. `gap_to_group_ahead / group_speed` from the group snapshot.
  Confirmed 2026-07-08: nothing time-gap-shaped exists in the codebase (all gap
  concepts are metres); it belongs here, not in D2's controller.

### C2. Decision cadence

Decisions don't need 100 Hz. New engine phase `step_decision()` runs every N physics
steps (e.g. 1 Hz sim-time; parameter). Outputs are *held* between ticks:
- `target_effort` (feeds existing `PhysicsEngine::set_rider_effort` path)
- lateral behavior selection (assign/clear `ILateralBehavior` — mechanism exists:
  `set_rider_behavior`)
- `GroupRole` declaration (mechanism exists: role declarations in the group phase)

Note: the per-rider *effort source* mode (Schedule/Manual/Follow) lands with D2 — C2's
job becomes selecting modes/targets, not inventing the arbitration.

### C3. First consumer 1 — effort pacing (`IEffortPolicy`)

Interface parallel to `ILateralBehavior`: `double compute_target_effort(const
DecisionContext&) const`. First concrete policy: **W′-budgeted climb pacing** — push
above FTP on gradients steep enough to matter, sized so W′ hits its floor near the
crest; recover on descents. Coexists with `EffortSchedule` (scripted TT riders keep
schedules; AI riders get policies — per-rider choice).

### C4. First consumer 2 — group tactics

Uses the rider window + group data: sit-in vs. chase vs. drift-back decision as effort
deltas on top of the pacing policy (e.g. "gap to group ahead < X and W′ > Y → chase").
Declares `GroupRole` intent (paceline participation) — finally exercising
`build_group_context`.

⚖️ Open: how pacing (C3) and tactics (C4) compose — recommend tactics as a bounded
modifier on the pacing baseline (`effort = clamp(pacing + tactic_delta, …)`), decided
per tick.

### C. Files & tests

- New: `include/decision.h`, `src/decision.cpp` (context build + policies).
- Touch: `include/sim.h`, `src/sim.cpp` (`step_decision()` phase, cadence), rider/engine
  glue for per-rider policy assignment.
- Tests: context construction (windows, climb digest) against a synthetic course; policy
  unit tests (W′ budget honored, crest detection); determinism (same inputs → same
  decisions).

---

## Dependencies & suggested order

```
B1 (wind data)      ──►  small; no prerequisites
B2 (crosswind)      ──►  adds force into the free_movement/solver code A settled;
                         also feeds D1's wake axis (echelons)
D1 (draft aero)     ──►  DONE (2026-07-08)
D2 (gap-holding)    ──►  makes formations hold at matched effort
D3 (rotation)       ──►  after D2; D3.0 amends D1's link rule first
C  (decision layer) ──►  biggest; C4 declares roles that drafting rewards
```

Rough sizes: B1 ≈ half a session; B2 ≈ half–one; D2 ≈ one;
D3 ≈ one+; C ≈ several, with its own design checkpoints (C1 API review before C3/C4).

## Verification (all workstreams)

Per landing: `cmake --build build -j` (0 warnings) + `ctest` (all green) + headless
smoke run (SIGTERM → exit 0). B2 additionally needs an interactive check (leeward
drift), as does A's still-open penalty-feel tuning; C needs a scripted scenario run (e.g. plot screen: one AI rider
with climb-pacing policy vs. one scheduled rider on `create_endulating`).
