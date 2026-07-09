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
B1 (wind data)      ──►  DONE (2026-07-10)
B2 (crosswind)      ──►  DONE (2026-07-10; yaw drag into cda_factor — echelons are
                         now energetically load-bearing)
D1 (draft aero)     ──►  DONE (2026-07-08)
D2 (gap-holding)    ──►  DONE (2026-07-08)
D3 (rotation)       ──►  DONE (2026-07-09; D3.0 link-rule amendment included)
C  (decision layer) ──►  biggest; C4 declares roles that drafting rewards
```

Rough sizes: B1 ≈ half a session; B2 ≈ half–one;
C ≈ several, with its own design checkpoints (C1 API review before C3/C4).

## Verification (all workstreams)

Per landing: `cmake --build build -j` (0 warnings) + `ctest` (all green) + headless
smoke run (SIGTERM → exit 0). B2 additionally needs an interactive check (echelon
stagger + windward swings under angled wind; gates the ⚖️ yaw constants), as does A's
still-open penalty-feel tuning; C needs a scripted scenario run (e.g. plot screen: one
AI rider with climb-pacing policy vs. one scheduled rider on `create_endulating`).
