# CyclingSim — Feature Roadmap (TODO #9+10, #7, #6 + drafting)

Status: **outline for discussion** — decisions marked ⚖️ are open; everything else is the
current recommendation. Ordering: A → B1 → (B2 ∥ C), see Dependencies at the bottom.

## Context

The audit-fix phase is done (see git history for the old PLAN.md): the sim core is
warning-free, fully tested, single-solver (ACCEL_FORCE), and the realtime driver is
extracted. Three feature areas remain from TODO:

- **#9 + #10** — the lateral model is half-wired: collision speed penalty is computed but
  disabled (`Rider::apply_lateral_update`, commented out "until tuned") because it
  penalizes the wrong riders; `lat_target` semantics (nullopt = no spring) leave riders
  with no natural tendency toward open road.
- **#7** — wind: `Course::get_wind()` is a stub returning a constant `Wind{0,1}` (a
  permanent 1 m/s wind!), rider `heading` is never set from the course, and there is no
  crosswind concept. Longitudinal drag already includes rider speed correctly
  (`v_air = v + env->headwind` in sim_core.c).
- **#6** — riders have no decision layer: effort comes from static `EffortSchedule`s or
  the UI slider. Riders should "see" the course (full profile — the course book) and the
  riders around them (±200 m only) and decide effort/tactics from that.

---

## Workstream A — Lateral model: penalty targeting + centering (#9, #10)

**Status: DONE** (2026-07-07). Penalty applies only to a blocked squeezer, ramped by
longitudinal offset (side-by-side is free); ambient centering via
`CollisionParams::ambient_center_k` (not W′-scaled); penalty re-enabled in
`Rider::apply_lateral_update`. Constants (`kPenaltyScale` etc.) stayed as locals in
`compute_shove` — tune by recompile. Interactive feel-check / tuning still open.

**Goal:** the collision speed penalty punishes only riders squeezing into occupied space;
riders have a gentle default tendency toward open road; `speed_penalty` is re-enabled.

### A1. Penalty targeting rule (#9)

Current: `compute_shove` (src/lateral_solver.cpp) assigns penalty rates to *both* riders
of a contact pair, weighted by resistance fractions. That punishes the rider in front for
being rammed from behind.

Proposed rule, using machinery that already exists:
- `ContactPair` construction sorts by `lon_pos`, so **B is ahead of A by construction**
  (`lon_sep >= 0`). The rider *behind* (A) is the "squeezer" candidate.
- Penalty applies **only to A**, and only when A is actually squeezing:
  modulate by `is_blocked(a_idx, riders)` (already implemented, src/lateral_solver.cpp) —
  full penalty when no passable gap ahead, zero (or small) when a free lane exists.
- The rider ahead (B) is never speed-penalized; it still receives its lateral shove.

⚖️ Open: side-by-side pairs (`lon_sep ≈ 0`, currently tie-broken by id) — no penalty for
either (recommended: pure lateral jostling shouldn't slow anyone) or split penalty.

### A2. `lat_target` concept review (#10)

Current semantics: `std::optional<double>` — behaviors return a target, `nullopt` means
*no spring at all* (`HoldLineBehavior` returns nullopt → rider is purely force-driven and
stays wherever shoves left it).

Recommendation: **ambient centering, separate from behavior targets**:
- In `LateralSolver::free_movement`, when `lat_target == nullopt`, apply a weak spring
  toward 0 with a new `CollisionParams::ambient_center_k` (≪ `lat_spring_k`).
- Explicit behavior targets keep using `lat_spring_k` (strong, intentional steering).
- Result: empty road → riders drift to the middle; contact/overtaking → shoves and
  behavior targets easily overpower the ambient pull. No behavior interface change.

Why not "default target = 0 with the existing spring": uniform strong pull to the
centerline fights the shove model — packs get wedged single-file into the middle and
every overtake decays back to center.

⚖️ Open: center toward road centre (0) vs. "riding line" (e.g. slight offset / future
lane concept). Start with 0; revisit when group behavior lands.

### A3. Re-enable the penalty

- Uncomment `state.speed *= speed_penalty` in `Rider::apply_lateral_update`
  (src/rider.cpp), after A1 makes it fair.
- Tune `kPenaltyScale` / `kMaxPenaltyRate` (compute_shove) by eye in the game: a blocked
  rider grinding against a wall of wheels should visibly stall, a clean overtake should
  cost nothing.

### A. Files & tests

- `src/lateral_solver.cpp`, `include/lateral_solver.h` (penalty rule, ambient spring,
  `CollisionParams::ambient_center_k` in `include/collision_params.h`)
- `src/rider.cpp` (re-enable penalty)
- `tests/test_lateral_physics.cpp`: front rider never penalized; blocked squeezer is;
  unblocked overtaker isn't; ambient centering converges to 0 with no contacts;
  dt-independence (existing test) still holds.

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

### D1. Aero model — geometric chain for paceline, depth heuristic for body

Role (from the group phase) selects the model; geometry or depth sets the value.

**Paceline riders — hybrid: position table × gap falloff.** For each `Paceline`-role
rider, find the nearest rider ahead within a lateral tolerance (~rider width) and
within `max_draft_gap`; walk that chain to get *chain depth*, then:

```
cda_factor = paceline_table[min(depth, N-1)] * gap_falloff(wheel_gap) (× alignment(Δlat))
```

- `paceline_table` — TTT numbers by chain position, saturating around P4–P6.
  ⚖️ Open: exact values (numbers to be supplied; literature baseline ≈
  {1.00, 0.70, 0.64, 0.60, …} as placeholder).
- `gap_falloff` — smooth (e.g. smoothstep): full effect at sub-wheel gaps, decaying to
  1.0 over a few metres. **Every term smooth, no hard cutoffs** — at 100 Hz a
  discontinuous CdA step when a rider crosses a threshold causes visible speed jitter.
- The lateral-tolerance test *is* the "one rider wide" definition: a rider swinging off
  loses the draft continuously — exactly what D3 rotation needs.
- **Multiple pacelines cost nothing**: side-by-side sprint trains each form their own
  chain because lateral alignment separates them. Which lane a team rides in is an
  *assignment* problem (perception/tactics era), not an aero-model problem.
- Write the shelter test as *offset from a wake axis* (axis = straight behind for now);
  B2's crosswind later rotates the axis → echelons, with no model rewrite.

**Body riders — depth heuristic, deliberately approximate and more pronounced.**
`cda_factor = body_curve(riders_ahead_in_group)` (count within a longitudinal window,
any role): exposed at the front edge (~0.9), one row back (~0.6), buried (floor
~0.45–0.5 for small groups). A blob shelters better than a single wheel; false
precision here isn't worth per-rider geometry. ⚖️ Open: floor value, and whether the
floor should deepen with group size (big-peloton literature goes much lower).

**Plumbing:**
- New `include/drafting_params.h` (`DraftingParams`: table, `max_draft_gap`,
  `lat_tolerance`, falloff shape, body curve/floor) — sibling of `GroupingParams`.
- `PhysicsEngine::step_draft_apply()` — the slot already reserved in `update()`
  (src/sim.cpp:48), between `step_group_role_apply()` and `step_longitudinal()`.
  One-tick-stale positions are fine at 100 Hz.
- `Rider::set_cda_factor(double)` → writes `state.cda_factor` (hook exists, always 1.0
  today).
- Add the factor to `RiderSnapshot` for debug/UI (draft visualisation later).
- Fix `compute_surplus_power` (src/sim.cpp) to include the factor — otherwise sheltered
  riders undercount their shove budget.

### D2. Gap-holding (follow controller)

A paceline is only a paceline if followers hold the wheel. Per-rider optional *follow
target* (analogous to `lat_target`): PD controller on gap error modulating
`target_effort`, **capped by the rider's own `effort_limit`** — a dying rider still
gets dropped, which is the emergent behavior we want. ⚖️ Open: effort modulation
(physical, W′-honest — recommended) vs. direct speed governor (stable but fakes
physics). Runs at physics cadence, not decision cadence.

### D3. Rotation

For `paceline_position == 0` (GroupContext already provides it): pull policy (time- or
W′-based), then swing off as an `ILateralBehavior` (interface fits as-is), ease off,
rejoin at the rear of the paceline. D1's continuous lateral falloff makes the handoff
smooth; D2 makes the reformed line hold. First genuinely decision-flavored piece —
scoped to paceline mechanics, not tactics (C4 decides *whether* to participate).

### D. Files & tests

- D1: new `include/drafting_params.h`; `include/sim.h`, `src/sim.cpp`
  (`step_draft_apply`), `src/rider.cpp`/`include/rider.h` (`set_cda_factor`),
  `include/snapshot.h`.
- Tests (D1): two riders nose-to-tail → follower factor = `table[1] * falloff(gap)`;
  gap beyond `max_draft_gap` → 1.0; lateral offset beyond tolerance → 1.0; two
  side-by-side chains stay independent; factor is continuous under small position
  perturbations; at equal effort the follower gains speed / spends less W′.
- D2/D3 get their own test plans when designed in detail.

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

### C2. Decision cadence

Decisions don't need 100 Hz. New engine phase `step_decision()` runs every N physics
steps (e.g. 1 Hz sim-time; parameter). Outputs are *held* between ticks:
- `target_effort` (feeds existing `PhysicsEngine::set_rider_effort` path)
- lateral behavior selection (assign/clear `ILateralBehavior` — mechanism exists:
  `set_rider_behavior`)
- `GroupRole` declaration (mechanism exists: role declarations in the group phase)

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
A  (lateral tuning) ──►  small, self-contained, do first; finishes the collision model
B1 (wind data)      ──►  small; independent of A
B2 (crosswind)      ──►  after A (adds force into the same free_movement/solver code)
D1 (draft aero)     ──►  independent of A/B; before C — tactics reason about drafting
D2 (gap-holding)    ──►  after D1; makes formations hold at matched effort
D3 (rotation)       ──►  after D2 and A (swing-off is an ILateralBehavior A tunes)
C  (decision layer) ──►  biggest; after D1 (C4 declares roles that drafting rewards);
                         benefits from A being settled
```

Rough sizes: A ≈ a session; B1 ≈ half; B2 ≈ half–one; D1 ≈ a session; D2 ≈ one;
D3 ≈ one+; C ≈ several, with its own design checkpoints (C1 API review before C3/C4).

## Verification (all workstreams)

Per landing: `cmake --build build -j` (0 warnings) + `ctest` (all green) + headless
smoke run (SIGTERM → exit 0). A and B2 additionally need interactive checks (penalty
feel, leeward drift); C needs a scripted scenario run (e.g. plot screen: one AI rider
with climb-pacing policy vs. one scheduled rider on `create_endulating`).
