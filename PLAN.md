# CyclingSim — Feature Roadmap (TODO #7, #6 + drafting)

Status: **Workstreams D (drafting) and B (wind) are DONE** — D committed through
6092bc1 (2026-07-09), B landed 2026-07-10 in the two-commit split suggested in §B.
Decisions marked ⚖️ are open; everything else is the current recommendation.
Remaining: workstream C, see Dependencies at the bottom.

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

## Workstream B — Wind (#7), phased — DONE (2026-07-10)

Implemented 2026-07-10 as designed below (design settled 2026-07-09). Landing
notes: the core wind test lives at `tests/core/test_wind_core.c` — the planned
`test_wind.c` name would collide with the C++ `tests/test_wind.cpp` CMake target;
the ⚖️ yaw constants landed at `kYawDragGain = 1.0`, `kMinApparentLon = 1.0 m/s`,
`kYawFactorCap = 3.0` as rider.cpp locals. Echelon integration test confirms the
design anchor (aligned follower total cda ≈ 0.83 vs exposed ≈ 1.35 under 5 m/s
crosswind; exposed chaser burns W′, sheltered one doesn't). **Still open ⚖️:**
the interactive feel-check of the demo wind (echelon stagger, windward swings)
that finalises those constants — shares a session with workstream A's
penalty-feel tuning.

**Goal (B1):** real wind exists per course and affects longitudinal physics correctly.
**Goal (B2):** crosswind costs energy via yaw-dependent longitudinal drag.
(**Redesigned 2026-07-09** — replaces the earlier "crosswind → lateral force" B2; see
the dropped/parked list below for the rationale record.)

Motivation for the redesign: a real rider counters crosswind by leaning, which is
quasi-static and energetically ~free — and our lateral spring/damper control is also
free — so a lateral force term would be exactly cancelled at zero cost and produce
nothing. Echelon *geometry* already emerges from D1's wake axis rotating with apparent
wind + D2 followers steering to it + the road-width clamp (gutter). The *energetic*
reality of crosswind is longitudinal: apparent-wind magnitude and yaw-dependent CdA
increase drag. That is what B2 models. Outcome: exposed riders in crosswind pay
~15–25% more drag power while riders sheltered on the rotated wake axis don't —
echelons become energetically load-bearing, not just geometric.

### Architecture (signal flow)

```
Course (base Wind, per-segment heading)
  └─ get_wind(pos), get_heading(pos)          [B1: real data, was stub]
       └─ Rider::update (src/rider.cpp)
            heading   = course->get_heading(pos)                  [B1]
            u = speed + wind·cos(dir − heading)  (long. apparent) [exists]
            c =         wind·sin(dir − heading)  (crosswind)      [B2]
            yaw_factor = CdA_ratio(yaw) · V_a / |u|  (capped)     [B2]
            state.cda_factor = draft_factor_ · yaw_factor_        [B2]
            env.headwind = u − speed  → sim_step_rider(...)
       └─ PhysicsEngine::build_draft_state (src/sim.cpp)
            headwind/crosswind per rider → wake axis rotates      [exists, D1]
            rotation swing side = windward                        [exists, D3]
```

Key invariant: **no C ABI change**. The core keeps its scalar `env->headwind` and
`cda_factor` contract; crosswind enters purely through the C++ layer as a multiplier
on `cda_factor` (same one-tick-staleness class as drafting, fine at 100 Hz).

### B1. Longitudinal wind (data + projection)

What's already right: sim_core's `v_air = v + env->headwind` (all three resistance
paths) — drag correctly depends on rider speed. The scalar-headwind contract into the C
core stays.

1. **Wind field on `Course`** (`include/course.h`, `src/course.cpp`): member
   `Wind wind_{0.0, 0.0}` returned by `get_wind(pos)` (the `pos` parameter stays —
   per-segment overrides are future scope) + `set_wind(Wind)` for appstate/tests.
   Factories keep wind 0 — removes today's phantom 1 m/s wind (expect tiny absolute
   speed shifts; existing suites assert relative quantities, verified).
   **Sign convention, documented on `struct Wind`:** `heading` is the direction the
   wind blows *from* — `wind.heading == rider heading` ⇒ full headwind (positive
   `env.headwind`, consistent with the existing `cos` projection in rider.cpp);
   the crosswind `sin` sign is what D1's wake axis and D3's windward swing already
   consume, so those stay untouched.
2. **Rider heading**: `ICourseView::get_heading(pos)` + `Course` impl
   (`segments[find_segment(pos)].heading` — field already exists); `Rider::update`
   sets `heading` from it before the wind projection. Factory courses keep heading 0;
   varied-heading courses are built in tests via `Course::from_segments`.
3. **Core tailwind sign fix** (`core/src/sim_core.c`, arithmetic only, no ABI change):
   all three drag paths square `v_air`, so a tailwind stronger than rider speed
   (`v_air < 0`) *resists* instead of pushes. Fix `v_air * v_air` →
   `v_air * fabs(v_air)` in `pow_speed` (P_aero), `resistive_force` (drag term), and
   the matching derivative in `pow_speed_prime`
   (`2·v_air·v + v_air²` → `2·fabs(v_air)·v + v_air·fabs(v_air)`). This also makes
   B2's multiplier algebra exact.

### B2. Crosswind → yaw drag (replaces "lateral force")

**Model** (standard cycling-aero form; per rider per tick, one-tick-stale speed `v`):

```
u   = v + wind·cos(dir − heading)      // longitudinal apparent wind (= v + env.headwind)
c   =     wind·sin(dir − heading)      // crosswind component
V_a = sqrt(u² + c²)                    // apparent wind magnitude
CdA_ratio = 1 + k_yaw · c²/V_a²        // = 1 + k_yaw·sin²(yaw); smooth, no trig
yaw_factor = CdA_ratio · V_a / max(|u|, u_min),  clamped to m_cap
```

- Target force `F = ½ρ·CdA·CdA_ratio·V_a·u`; the core (post-B1.3) computes
  `½ρ·CdA·cda_factor·v_air·|v_air|`, so multiplying `cda_factor` by `yaw_factor`
  reproduces the target exactly, signs included (tailwind push amplified by side
  wind — physical).
- `u_min`/`m_cap` guard the standing-start spike (`u → 0` while the true force → 0;
  the cap only matters below ~walking pace).
- ⚖️ Constants `k_yaw` (≈1.0: +10–15% CdA at ~20° yaw), `u_min` (≈1 m/s), `m_cap`
  (≈3) as file-local constants in rider.cpp, tune-by-recompile — same precedent as
  workstream A's `kPenaltyScale` locals. Empirical knobs; settle at the feel-check.
- Design anchor: 5 m/s pure crosswind at 12 m/s ⇒ `V_a/u ≈ 1.083`, yaw ≈ 23°,
  `CdA_ratio ≈ 1.15` ⇒ ~+25% drag power exposed; a rider aligned on the rotated wake
  axis recovers most of it through the D1 shelter multiplier.

**Factor split in `Rider`** (`include/rider.h`, `src/rider.cpp`): `state.cda_factor`
becomes the *product* of two named factors so shelter and wind stay separable:
- New members `draft_factor_ = 1.0`, `yaw_factor_ = 1.0`; `set_cda_factor(f)` writes
  `draft_factor_` (drafting callers unchanged); `get_cda_factor()` returns the product
  (existing tests stay valid).
- `Rider::update` computes `yaw_factor_` right after the wind projection and writes
  `state.cda_factor = draft_factor_ · yaw_factor_` before `sim_step_rider`. Engine
  tick ordering already works (`step_draft_apply` runs before `step_longitudinal`) —
  no sim.cpp change.
- `RiderSnapshot`: `cda_factor` stays the product (what physics sees); add
  `yaw_factor` so plots/UI can distinguish "sheltered" from "in crosswind".
- `compute_surplus_power` and all three core drag paths pick the product up for free.

**Dropped / parked (decision record):**
- **Dropped: lateral crosswind force in `LateralSolver`** — cancelled for free by
  position control; leaning is free in reality too; no behavioral or energetic value.
  `LateralRiderState` gets no `crosswind` field.
- **Dropped: energy cost for lateral movement** — not physical (steering displacement
  is near-free); the contested-gap case is already covered by workstream A's squeeze
  penalty.
- **Parked: windward tactical positioning of exposed riders** (riding the windward
  edge to shrink the echelon) — a decision, not a force → C-era (C4).
- **Parked:** per-segment wind overrides (sheltered sections); equipment-dependent yaw
  curves / sail effect (deep wheels: `CdA_ratio < 1` at moderate yaw).

### B. Files & tests

- B1: `include/course.h`, `src/course.cpp` (wind field + `get_heading`),
  `src/rider.cpp` (set heading), `core/src/sim_core.c` (tailwind sign).
- B2: `include/rider.h`, `src/rider.cpp` (factor split + yaw multiplier), snapshot,
  `src/appstate.cpp` (demo wind).
- Tests, core (`tests/core/test_wind.c`, `test_terminal_velocity.c` pattern):
  terminal speed ordering headwind < still < tailwind; strong tailwind accelerates a
  zero-effort rider from rest (sign-fix proof).
- Tests, C++ B1: `get_heading` per segment; projection correctness — rider on a
  `from_segments` course with headings {0, π/2, π} under `Wind{0, w}` is slowest on
  the headwind leg, fastest on the tailwind leg.
- Tests, C++ B2 (`tests/test_wind.cpp`): pure crosswind lowers terminal speed vs
  still air, symmetric in ±c; still air ⇒ `yaw_factor == 1` exactly (drafting suites
  unaffected — guard test); standing start under strong crosswind stays finite and
  respects the cap; determinism. **Echelon integration test** (the payoff): with
  crosswind, a follower's `lat_target` sits leeward on the rotated wake axis;
  an aligned follower's total `cda_factor` ≪ an exposed rider's; an exposed chaser
  burns W′ faster than the sheltered one at matched speed.
- Demo / feel-check: appstate sets a modest angled wind (3–4 m/s at ~60° to the
  course) on the rotation demo — expect echelon stagger, consistently windward
  swings, slightly lower line speed. Interactive gate for the ⚖️ constants.

Suggested commit split: (1) B1 wind data + heading + sign fix + tests;
(2) B2 yaw factor + factor split + tests + demo wind.

---

## Workstream D — Drafting — DONE (2026-07-09)

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

### D2. Gap-holding (follow controller) — DONE

Designed and implemented 2026-07-08, committed d749b88. Landed as specced below
(gains kp=2.8 ki=0.35 kd=5.0, h kept at 0 — accordion test passed with mild ~12%/link
amplification that reconverges); plus an addendum beyond the spec: followers also
steer laterally to the leader's wake axis (`wake_axis_lat`, shared with D1), so D2
steering and D1 shelter rotate together when B2 lands crosswind.

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

### D3. Rotation — DONE

Design settled 2026-07-08; implemented 2026-07-09, committed 6092bc1, visual check
passed. Implementation notes (beyond the design below): `PacelineRotation` in
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

All landed: D1 in `drafting.h/.cpp` + `drafting_params.h` (`tests/test_drafting.cpp`),
D2 in `follow.h/.cpp` + `follow_params.h` (`tests/test_follow.cpp`), D3 in
`rotation.h/.cpp` + `rotation_params.h` (`tests/test_rotation.cpp`), engine phases
`step_draft_apply` → `step_rotation_apply` → `step_follow_apply` in `sim.h/.cpp`.
Still open across D (deferred, not blockers): body-heuristic tuning when
`GroupRole::Body` goes live (D1 ⚖️), and UI exposure of follow/rotation modes
(C2-era — no interactive control yet beyond the appstate demo).

---

## Workstream C — Perception & decision layer (#6)

**Goal:** AI riders that perceive the race (course, groups, time gaps) and decide
(pacing, tactics, cooperation) on top of the complete A/B/D machinery. Same
architecture pattern as the lateral pipeline: engine builds context → pure strategy
object decides → engine applies.

Design converged 2026-07-10 (supersedes the earlier C1–C4 sketch). Key amendments:
teams become real entities (C-pre); time gaps are trace-based, position-indexed (C0);
course knowledge is a separate `CourseIntel` object plus a deliberately rough W′-pace
estimator backed by a core `cruise_speed` helper (C1); decisions live in a
`DecisionSystem` owned by `Simulation`, not more `PhysicsEngine` members, with
policies as *meta-controllers* above the `EffortSource` arbitration (C2); the decision
hierarchy is race plan → per-team director (commands + rider-side feasibility clamp) →
rider policy → existing 100 Hz controllers, while groups stay emergent — no "group
brain", group cooperation is role declaration + rotation reconcile (C4).

### C-pre. Prerequisites: team registry + sitter promotion — DONE

**Landed 2026-07-10, commit c94c0f4** (gate: 0 warnings, 17/17 ctest, headless
smoke exit 0). As-built notes / deviations from the text below:
- `team.h` is **header-only** (TeamRegistry methods inline, the effortschedule.h
  precedent) — no `team.cpp`, so the CMake source glob stays untouched.
- `sim_cruise_power` turned out to be a three-line wrapper: the core already had
  the force math factored as `resistive_force()` shared with the ACCEL_FORCE
  solver, so the no-drift property came for free.  Round-trip test
  (`tests/core/test_cruise_power.c`) is exact at terminal velocity on flat,
  3 % climb, head- and tailwind.
- **Attach criterion amended**: roster attach at "wheel gap ≤ engage_gap" could
  admit a promoter still physically *behind* the first sitter, making that
  sitter retarget to a tail behind itself and brake.  As built, roster attach is
  positional like the drifter rule — past the first sitter (engage_gap fallback
  when no sitters remain) — while the follow-side transit (offset + cap) clears
  independently at the setpoint, same decoupling as the drifter merge.
- The generalized transit lives in the **follow subsystem**: `FollowState` gains
  `approach_side`/`effort_cap` (refreshed each tick by the `move_up_side`
  directive; cap recomputed as the rider's draft changes), FollowParams gains
  `approach_fade_len = 2 m`.  This is the piece C4's MoveUp join reuses.
- Measured (engine test, depth-2 promotion into a 3-line + 2 sitters at 0.85
  effort): transit ≈ 10 s, max effort 1.017 (cap active — uncapped controller
  would demand max_effort), full 1.5 m advance-side offset, cut-in to 0.23 m,
  queue reforms at ≈ 0.25 m gaps; in a live rotation the promoted rider takes
  its first pull while true sitters never do.

**C-pre-a. Team registry.** The current `Team` in rider.h is a stub — `id` always 0,
copied by value into every rider, so there is no shared entity for a director to
live on.

- New `include/team.h`: `TeamId = int` + `kNoTeam = -1` (add in mytypes.h for
  symmetry with `RiderId`/`GroupId`); `struct Team { TeamId id; std::string name;
  std::vector<RiderId> roster; }` (race-plan fields land in C4); `class TeamRegistry`
  (`add_team(name) → TeamId`, `get`, `team_of(RiderId)`).
- `Rider`/`RiderConfig` hold `TeamId team_id` instead of a by-value `Team`; delete the
  old class. `PhysicsEngine` owns the registry (the decision phase reads it on the
  physics thread); `add_rider` registers the rider with its team. Update
  `appstate.cpp` (`Team("Team1")` → registry) and the snapshot fill
  (`RiderSnapshot::team_id` sourced from the new id).
- Gate: build + existing suite green (registry assertions land with the C2 tests).
  This landing also carries this PLAN.md rewrite.

**C-pre-b. SittingIn → InLine promotion.** Completes the D3 rotation mechanics:
a roster member can drift back and sit in, but there is no way back into the
rotation — promotion is rotation-internal (sub-container change for an existing
member), decision-free, driven by an engine API exactly like D2/D3 (tests/UI call
it now; C4's directives call the same API later). Not to be confused with the
*MoveUp join* (a non-member from the pack joining the paceline) — that is
decision-layer and lands in C4, reusing the transit mechanics built here.

- **First-sitter fast path**: already on the last InLine wheel (rotation.h: first
  SittingIn follows last InLine), so promotion is pure roster bookkeeping — moved
  from `sitting_` to the `inline_` tail, no maneuver.
- **Deeper sitters** physically overtake the sitters ahead: ride up the
  **rotation's advance side** (same crosswind-sign rule as the D3 swing side) to
  the InLine tail; the sitting queue behind re-links via the existing dynamic
  follow graph, closing the vacated gap automatically.
- **Transit state machine** (write it generalized — C4's MoveUp join reuses it):
  while in transit the maneuver owns `lat_target` (one lane out on the advance
  side); effort cap `max(1.0 · ftp, 1.2 · P_hold)` with
  `P_hold = sim_cruise_power(v_group, current draft factor)`. Completion when the
  wheel gap to the target slot reaches the follow setpoint → enters `inline_` at
  the tail, lat_target handed back to the follow subsystem's wake-axis logic.
- Needs pulled forward from C1a: **`sim_cruise_power(rider, env, v)`** in the C core —
  direct evaluation of the resistive force terms at speed v (aero incl. `cda_factor`,
  rolling, gravity, bearings, drivetrain), factored out of the ACCEL_FORCE solver's
  force math so the two can never drift. (The Newton inverse `sim_cruise_speed`
  stays in C1a.)
- Engine surface (physics-thread-only, like follow/rotation): `promote_sitter(RiderId)`.
- Tests (tests/test_rotation.cpp + a core round-trip check for cruise_power):
  promotion from sitting index 2 (passes two riders, effort never exceeds cap,
  enters InLine at tail, then gets cycled into pulling); first-sitter fast path is
  instant; sitting queue closes the vacated gap.

### C0. RaceClock — race-style time gaps & checkpoints — DONE

**Landed 2026-07-10, commit 56962f6** (gate: 0 warnings, 18/18 ctest, headless
smoke exit 0). As-built notes:
- `record(id, pos, t)` keeps the previous sample internally (anchor + latest
  per trace) — no `pos_before/dt` plumbing; DecisionSystem just feeds
  positions.  Queries interpolate between cell gridlines *or the trace's live
  endpoints*, so the first cell and the stretch just behind a rider's current
  position answer exactly too.  Stalls are ignored → stored times are always
  *first* crossings.
- Measured roughness matches the prediction: a 15→5 m/s step mid-cell gives
  +1.67 s query error; gridlines/checkpoints stay exact (finish margins
  resolve to ~50 µs).
- UI is a `GroupBoardDrawable` (display.h/.cpp, top-right overlay: colour
  swatch matching the halos + "name (size) +m:ss"), registered in screen.cpp —
  there were no existing group labels to attach to, so the board is new.
- Test-design note: near-equal riders started together draft each other to a
  wheel-to-wheel finish, so the finish-order test separates the pair by
  effort, not ftp.

New `include/race_clock.h`, `src/race_clock.cpp` — pure and engine-free, same
isolation as drafting/follow/rotation. Replaces the old "gap/speed" idea: a real
trace-based measurement (time gap = now − when the rider ahead crossed my position),
robust on gradients where the instantaneous estimate is wrong.

- **Crossing-time grid, per rider**: spacing parameter `grid_spacing = 100 m`;
  `std::vector<double>` of `ceil(course_len/spacing)+1` entries, unset = NaN.
  `record(id, pos_before, pos_after, t, dt)` writes every gridline crossed this step
  with sub-step linear interpolation. O(1) lookups (index + lerp), memory bounded by
  course length, whole-course history for free.
- **Named checkpoints** at arbitrary positions — **course data**: `Course` gains a
  `struct Checkpoint { double pos; std::string label; }` list set at construction
  (finish implicit at `total_length`; TT timechecks / KOM lines explicit — this is
  what the commented-out `isCheckpoint(pos)` stub in ICourseView was groping at).
  RaceClock reads the list from `Course` at setup: same interpolation, exact
  per-rider capture stored permanently. Finish ordering can be decided by
  hundredths, so checkpoints never go through the grid.
- **Queries**: `crossing_time(id, pos) → optional<double>` (nullopt if not yet
  there); `time_gap(ahead, behind_pos, now)`. Group-level gap ("chase at 0:45") is
  derived by the caller from the `GroupSnapshot`: `ahead` = rearmost member of the
  group ahead (`Group::back_pos`).
- Accepted roughness (state in header): lerp assumes constant speed within a cell —
  worst case ≈ ±1 s across a sharp gradient transition; reads as race-radio precision.
- **DecisionSystem skeleton** so RaceClock has a home from day one: new
  `include/decision.h`, `src/decision.cpp` with `class DecisionSystem` holding the
  RaceClock; only entry point `observe(const PhysicsEngine&, t, dt)` (per rider one
  gridline-index comparison per step). Owned by `Simulation`, called from
  `step_fixed` after `engine.update(dt)`. `decide()` arrives in C2.
- **UI**: `Group`/`FrameSnapshot` gains `double time_gap_ahead` (−1 = leading group),
  filled at snapshot time; render next to the existing group labels.
- Tests `tests/test_race_clock.cpp`: constant-speed pair → exact gaps everywhere;
  speed step inside a cell → bounded error; checkpoint capture exact vs analytic;
  query ahead of a rider's progress → nullopt.

### C1. CourseIntel + core cruise helpers + W′-pace estimator + DecisionContext — DONE

**Landed 2026-07-10, commit 15631fd** (gate: 0 warnings, 21/21 ctest, headless
smoke exit 0). As-built notes / deviations:
- `sim_cruise_speed` agrees with the ACCEL_FORCE terminal velocity to 1e-4 in
  every env (shared force model did its job).  Fixed a latent `find_segment`
  edge on the way: `x == total_length` (exactly the finish — climbs crest
  there) fell through the binary search and threw.
- Estimator entry point is `Rider::cruise_speed_at(power, slope, headwind,
  cda_factor)` — a what-if copy of the rider's state/env — so
  `estimate_wprime_pace` needs no RiderState/EnvState plumbing in the decision
  layer.  End-to-end check: riding the estimated pace up a 2.5 km 6 % climb
  from a standing start lands at **wbal_frac 0.008 at the crest, duration
  within 1 %** of the estimate — comfortably inside the planned 10–15 %
  tolerance (the energy model's effort-limit throttle below 20 % W′bal is the
  soft landing).
- The decision-scale rider descriptor is **`PerceivedRider`** — `NearbyRider`
  already names the lateral-scale one (lateral_behavior.h).
- `EffortSource` moved sim.h → mytypes.h so DecisionContext can carry it
  without an include cycle.  `PacelineRotation` gained `line_depth(id)` /
  `is_sitting(id)` introspection for the context's rotation block.
- `build_group_context` is now public and consumed (what it was kept for);
  engine also exposes `get_group_tracker()` for the rider window.  Context
  time gaps come as both precomputed ahead/behind fields and const
  CourseIntel/RaceClock handles for ad-hoc queries.  The C4 `Directive`
  struct + inbox exist now so C2/C3 compile against the final context shape.
- DecisionSystem owns the shared-const `CourseIntel` (built in its ctor;
  survives reset — static course knowledge).

**C1a. Core cruise-speed helper** (`core/include/sim_core.h`, `core/src/sim_core.c`):
- `sim_cruise_speed(rider, env, power)` — Newton (bisection fallback) on
  `sim_cruise_power(v) = power` (~5–10 iterations), inverting the direct-evaluation
  helper that already landed with C-pre-b.
- Test `tests/core/test_cruise_speed.c` (no C++ target-name collision — the
  `test_wind_core.c` precedent): cruise_speed vs solver terminal velocity;
  `cruise_power(cruise_speed(P)) ≈ P` round-trip; slope/headwind variants.

**C1b. CourseIntel** (new `include/course_intel.h`, `src/course_intel.cpp`):
- Built **once** from `Course` at sim start; one shared `const` instance (perfect
  knowledge; per-team noisy digests are a future hook). Needs
  `const std::vector<Segment>& get_segments() const` added to `Course` (private today).
- Climb index: merge consecutive uphill segments (min avg gradient ~2 %, tolerate
  dips shorter than ~200 m) → `struct Climb { start, length, avg_gradient, crest_pos }`.
- Queries: `distance_to_finish(pos)`, `next_climb(pos)`, `distance_to_crest(pos)`
  (nullopt when none ahead), `avg_gradient(from, to)` (O(1) via `get_altitude`).
  (Checkpoints are *not* CourseIntel's job — they are `Course` data, consumed by
  RaceClock in C0; CourseIntel only digests the profile.)
- Tests `tests/test_course_intel.cpp`: climb digest on a synthetic segment list
  (known merges/dips) and on `create_endulating`; edge cases (final descent, past
  last crest, pos 0).

**C1c. W′-budget pace estimator** (pure free function in decision.h/.cpp):
`estimate_wprime_pace(dist, avg_gradient, avg_wind, wbal_J, ftp_W, draft_factor,
phys) → power` — the constant power that spends the W′ budget exactly over `dist`.
- Fixed point: `P₀ = ftp`; ~3×: `v = sim_cruise_speed(P)`, `T = dist/v`,
  `P = ftp + wbal/T`; clamp `[ftp, max]`. Sub-µs per rider — runs every decision
  tick (rolling re-plan self-corrects as reality diverges).
- Deliberately rough (the honesty is the feature): averaged gradient/wind over the
  window; ignores FTP degradation and altitude; linear W′ depletion above FTP
  (matches `energy_update`).
- Draft factor input: for a rotation of n riders,
  `avg over k=0..n−1 of paceline_table[min(k, 5)]` (entries clamp at the last value,
  so n > 6 averages in multiple 0.41s); assumes ideal alignment (falloff·align ≈ 1).
  Solo → 1.0; sitting in at line depth d → `paceline_table[min(d, 5)]`.
  **No steep-gradient cutoff**: at climbing speeds the aero term inside cruise_speed
  is already small, so draft influence vanishes by itself — branch-free.
- Tests (start `tests/test_decision.cpp`): offline sim (`OfflineSimulationRunner`)
  holding the estimated pace over a synthetic climb → W′ within ~10–15 % of the
  floor at the crest; draft averaging incl. the n > 6 clamp.

**C1d. DecisionContext** (decision.h; built per rider per decision tick):
- Own state: pos, speed, heading, wbal (J + fraction), ftp, `effort_limit`, current
  `target_effort` + EffortSource, rotation membership (member / slot depth / size —
  via `get_paceline_rotation`).
- Group: `GroupContext` via `PhysicsEngine::build_group_context` (built, never
  wired — this is what it was kept for; make public or expose a bulk builder).
- Rider window ±200 m (perception-horizon param): per nearby rider
  `{ id, lon_offset, speed, group ordinal & size }` — **no** `w_prime` of strangers
  (decided; teammate W′ flows through the team director in C4, not the context).
- Time gaps: `time_gap_to_group_ahead`/`behind` precomputed from RaceClock +
  snapshot; const pointers to `CourseIntel`/`RaceClock` for ad-hoc queries.
- Directive inbox (filled by the team director from C4 on; empty until then).

### C2. Decision cadence, `decide()`, policy-as-meta-controller — DONE

**Landed 2026-07-10, commit bc1afd5** (gate: 0 warnings, 21/21 ctest, headless
smoke exit 0). As-built notes / deviations:
- "Follow > Policy" needs no dedicated guard — it holds *by mechanism*: a live
  follow controller rewrites target_effort every physics step, so a policy's
  1 Hz held effort simply never sticks.  Likewise the planned "dedicated
  engine path bypassing the no-op-unless-Manual guard" wasn't needed: that
  guard lives only in Simulation's queued UI path; decide() runs on the
  physics thread and calls `PhysicsEngine::set_rider_effort` directly.
- Policies and schedules are mutually exclusive in *both* directions
  (assigning either replaces the other), so their relative arbitration order
  is unobservable; `EffortSource::Policy` reports whenever a policy is
  assigned and no follow target is live.
- Policy-installed follow targets are tracked (`policy_follow_` set) so a
  policy emitting nullopt clears only its own target — a manual or
  rotation-owned one survives (tested).
- The reconcile made the engine **multi-rotation**: `auto_rotations_` vector
  beside the manual `rotation_`, shared `apply_rotation()` body, overlap
  matching (a group split re-shapes rosters on the next tick), interim
  `detach_gap` proximity admission gate, dissolve-below-2.
  `get_rotation_for(id)` is the new universal lookup; `promote_sitter`
  searches all rotations.  `PacelineRotation` gained
  `add_member`/`remove_member`/`members`.
- Test-design lesson (kept as a comment in test_decision.cpp): declaring
  Paceline while the bunch is still together forms one rotation whose follow
  controllers *glue the pack* — the slow riders can hold any wheel, they were
  just unwilling.  Stage splits first, declare after.  Behavioral separation
  (unwillingness vs. inability) is exactly C3/C4 territory.
- Appstate demo unaffected: its riders all declare Paceline but belong to the
  manual rotation, which the reconcile skips.
- UI: new `RiderBoardDrawable` (bottom-left; name, M/S/F/P mode letter,
  effort, policy name), snapshot fields stamped by Simulation at frame time —
  closes the deferred D-era "UI exposure of follow/rotation modes".

**Next before C3: the interactive feel-check milestone below.**

- `DecisionSystem::decide(PhysicsEngine&)` fired from `step_fixed` via sim-time
  accumulator every `decision_period` (param, default **1.0 s**). Cadence ≠ thread:
  everything stays on the physics thread; the single-writer model (command queue,
  physics-thread-only APIs) is untouched.
- **Determinism**: iterate riders in **sorted RiderId order** (`riders` is an
  `unordered_map` — unspecified order would make cross-rider decisions
  nondeterministic). Test: two identical runs → identical decision streams.
- **`IRiderPolicy`**: `PolicyOutput decide(const DecisionContext&)` with
  `{ optional<double> target_effort, optional<RiderId> follow, GroupRole role_decl,
  optional<Maneuver> maneuver }`. Outputs **held** between ticks; decisions at tick N
  apply from N+1 (one-tick reaction delay is intended).
- **Arbitration**: policies sit *above* `EffortSource` and operate it — they may
  set/clear follow targets and write effort. Add `EffortSource::Policy`; derived
  priority **Follow > Schedule > Policy > Manual**; assigning a policy replaces any
  schedule (mutually exclusive — TT riders keep schedules, AI riders get policies).
  Policy effort writes use a dedicated engine path, not the queued UI path (decide()
  already runs on the physics thread) and bypass the "no-op unless Manual" guard.
- **Rotation reconcile** (decision cadence): form/update one `PacelineRotation` per
  group from riders declaring `GroupRole::Paceline`. Manual `set_paceline_rotation`
  wins — reconcile skips riders in a manually installed rotation. New members are
  admitted **only when physically arrived** (via the MoveUp join, C4) —
  declared-but-pending until then; until C4 lands, reconcile only forms rotations
  from riders already physically in line (the C2 test scenario).
  `PacelineRotation` gains `add_member(RiderId, sits_in)` for arrived joiners.
- **Snapshot/UI**: `RiderSnapshot` += policy name/state + effective EffortSource —
  closes the deferred D-leftover "UI exposure of follow/rotation modes".
- Tests (test_decision.cpp): cadence (fires every N steps, held between);
  determinism; arbitration transitions (policy↔schedule↔follow, slider inert when a
  policy is assigned); reconcile respects manual rotations.

**Milestone between C2 and C3 — interactive feel-check session** (user present;
blocks C3/C4 sign-off, not their start): A's `kPenaltyScale`/`kMaxPenaltyRate`
(believable lateral speed dip + natural centering) and B2's yaw constants
(`kYawDragGain` etc., rider.cpp locals — few-% CdA at 3.5 m/s @ 60°, sensible
windward swings, echelon worth forming) against the appstate rotating-paceline demo.
Do it before AI behavior lands so "the paceline looks weird" stays attributable to
constants, not decisions.

### C3. First consumer — W′-budgeted pacing policy

`WPrimePacingPolicy : IRiderPolicy`:
- Horizon each tick: next crest if a climb lies within ~`horizon_km` (CourseIntel),
  else the finish.
- `target_power = estimate_wprime_pace(...)` with a **reserve**: budget
  `wbal − wbal_floor_frac · w_prime` (param, default 0.15). Effort conversion:
  `target_effort = target_power / ftp` (effort is FTP-relative throughout the core).
- Descents / no horizon: recovery effort (param, ~0.6) so W′ recharges.
- Draft assumption from own rotation membership (C1c rules). Coexists with
  `EffortSchedule` per the C2 arbitration.
- **Verification scenario** (the C gate): offline run on `create_endulating` — one
  policy rider vs one `StepEffortSchedule` rider at equal average power; assert the
  policy rider crests with `wbal_fraction ≈ floor ± tol` and arrives no later. Also
  eyeball on the plot screen.
- Unit tests: budget honored on synthetic climbs (steep-short vs long-shallow);
  horizon handoff at the crest; recovery below FTP on descents.

### C4. Team director, tactics, MoveUp maneuver

**Directives & director:**
- `struct Directive { enum Type { Free, Pull, SitIn, Chase, ProtectLeader }; … }`;
  `struct RacePlan { RiderId leader; per-rider default roles; simple rules }` — per
  team, set at scenario setup (C-pre registry).
- `TeamDirector` (one per team, inside DecisionSystem; runs **before** rider policies
  each decision tick, teams iterated in TeamId order): sees its own riders' full
  DecisionContexts **including W′** (radio — deliberately the only cross-rider W′
  visibility in the system); emits one Directive per rider into the context inbox.
- **Commands with rider-side clamp**: policies obey directives but always clamp to
  feasibility (`effort_limit`, W′ floor) — a cooked rider can't chase; the clamp is
  the final authority, giving suggestion-like softness with no arbitration machinery.

**Tactics (rider policy layer):**
- Sit-in / chase / drift as a **bounded delta** on the pacing baseline:
  `effort = clamp(pacing + tactic_delta, 0, effort_limit)`.
- Chase rule uses C0 gaps: `time_gap_to_group_ahead < gap_max && wbal_frac > reserve
  → chase`; declares `GroupRole::Paceline` to join the chase rotation.

**MoveUp join** (Body → paceline; decision-triggered — this is what makes it
C4-era, unlike C-pre-b's rotation-internal sitter promotion):
- A non-member who decides to participate (own tactic, Chase declaration, or a
  Pull/SitIn directive) physically rides up from the pack to the line tail before
  the C2 reconcile admits them (`add_member`).
- Mechanics: **reuses the C-pre-b transit state machine** verbatim — advance-side
  `lat_target`, effort cap `max(1.0 · ftp, 1.2 · P_hold)`, completion at the follow
  setpoint. New here is only the engine entry point for non-members
  (`request_paceline_join(RiderId, sits_in)`) and the decision-side callers.
- A Pull directive to a current sitter resolves to C-pre-b's `promote_sitter`
  instead.

Tests: director determinism + clamp (exhausted rider ignores Chase); directive flow
(Pull → rider ends up puller within N ticks, offline — exercising join/promote
end-to-end from a directive); join-from-pack arrives and merges without a lateral
step.

### C. Order, sizes, files

```
C-pre  teams + sitter promotion    DONE         2026-07-10 (c94c0f4): team.h (header-only), mytypes.h,
                                                rider.*, sim.* (promote API), sim_core.* (cruise_power),
                                                rotation.*, follow.*, appstate.cpp
C0     RaceClock + skeleton        DONE         2026-07-10 (56962f6): race_clock.*, decision.* (observe),
                                                course.* (checkpoints), sim.*, group.h (time_gap_ahead),
                                                display.* (GroupBoardDrawable), screen.cpp
C1     intel+core+estimator+ctx    DONE         2026-07-10 (15631fd): course_intel.*, sim_core.*
                                                (cruise_speed), course.* (get_segments + find_segment fix),
                                                decision.*, rider.* (cruise_speed_at), rotation.*
                                                (line_depth/is_sitting), mytypes.h (EffortSource), sim.h
C2     cadence+policies+reconcile  DONE         2026-07-10 (bc1afd5): decision.*, sim.* (multi-rotation +
                                                reconcile), rotation.*, mytypes.h (EffortSource::Policy),
                                                snapshot.h, simrenderer.cpp, display.* (RiderBoardDrawable),
                                                screen.cpp
       — feel-check session (interactive, A + B2 constants) —
C3     pacing policy               ~1           decision.*, analysis scenario
C4     director+tactics+MoveUp join ~1.5        decision.*, team.h, sim.* (join API), rotation.*
```

Existing pieces reused (do not rebuild): `build_group_context` (sim.h),
`GroupContext` (group.h), `paceline_table` (drafting_params.h), follow API
(`set_follow_target`, sim.h), `PacelineRotation` (rotation.h), `EffortSource`
arbitration (sim.h), `wake_axis_lat` (drafting.h), `OfflineSimulationRunner`
(analysis.h), `energy_wbal`/`energy_update` (sim_core), `create_endulating`.
decision.h/.cpp starts as one pair; split (policy.h, director.h) only past ~500
lines.

---

## Dependencies & suggested order

```
B1 (wind data)      ──►  DONE (2026-07-10)
B2 (crosswind)      ──►  DONE (2026-07-10; yaw drag into cda_factor — echelons are
                         now energetically load-bearing)
D1 (draft aero)     ──►  DONE (2026-07-08)
D2 (gap-holding)    ──►  DONE (2026-07-08)
D3 (rotation)       ──►  DONE (2026-07-09; D3.0 link-rule amendment included)
C  (decision layer) ──►  in progress; C-pre, C0, C1, C2 DONE (2026-07-10:
                         c94c0f4, 56962f6, 15631fd, bc1afd5) → next:
                         **feel-check session** (interactive) → C3 → C4;
                         C4 declares roles that drafting rewards
```

Rough sizes: B1 ≈ half a session; B2 ≈ half–one; C ≈ 6–8 sessions (per-phase
breakdown in "C. Order, sizes, files" above).

## Verification (all workstreams)

Per landing: `cmake --build build -j` (0 warnings) + `ctest` (all green) + headless
smoke run (SIGTERM → exit 0). B2 additionally needs an interactive check (echelon
stagger + windward swings under angled wind; gates the ⚖️ yaw constants), as does A's
still-open penalty-feel tuning (both bundled into the C2→C3 feel-check milestone);
C needs scripted scenario runs — the C3 offline scenario (policy vs. schedule on
`create_endulating`) and a C4 chase scenario (two groups, director orders a chase,
gap closes and the time-gap readout falls), both runnable headless — plus one
interactive plot-screen/appstate eyeball at C3 and C4.
