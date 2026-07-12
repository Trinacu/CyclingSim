# Adjustable parameters — lateral shove + C3.0 feel-check

Reference for the interactive tuning session (PLAN.md § C3.0).  All of these
are tune-by-recompile: `CollisionParams` fields are engine defaults
(`PhysicsEngine::params`, sim.h — `from_config` is declared but never
implemented, so the struct defaults are what runs); the rest are named
file-local constants.

## 1. Lateral shove (contact push, riders intersecting)

Mechanics (`LateralSolver::compute_shove`, src/lateral_solver.cpp): a contact
pair is any two riders with longitudinal separation < bike length and lateral
overlap of their radii.  Per contact, per tick, a **separation rate** (m/s)
is computed as

```
total_sep = clamp( contact_floor + active_a + active_b, 0, max_lat_correction )
contact_floor = overlap_frac · k_contact            (overlap_frac: 0 at touch → 1 at full overlap)
active_x      = clamp(surplus_power · shove_kJ, 0, J_max) · w_prime_frac
```

then split between the two riders (direction: away from each other, tiebreak
toward the side with more road space) and integrated by the solver's single
`· dt`.

| Param | File | Value | Mechanics |
|---|---|---|---|
| `k_contact` | include/collision_params.h | 0.07 | Overlap-proportional **baseline push**: full overlap separates at `k_contact` m/s regardless of power. Guarantees riders always diverge. This is the knob for "how hard does the pack push apart". |
| `shove_kJ` | include/collision_params.h | 1e-8 | Converts **surplus power** (W, watts beyond what current speed consumes — `compute_surplus_power`, src/sim.cpp) into extra separation rate. **Currently vestigial**: at 1e-8, a 500 W surplus adds ~5e-6 m/s — the push is effectively `k_contact` only. Raise toward ~1e-3–1e-2 to make strong riders actively shove. |
| `J_max` | include/collision_params.h | 30.0 | **Cap on one rider's active (power-funded) push component**, applied before the `w_prime_frac` scaling. Header comment says "N·s impulse per step" but the code clamps a *rate* (m/s) — the comment is stale. Never binds while `shove_kJ` is tiny. |
| `max_lat_correction` | include/collision_params.h | 10.0 | **Cap on the total separation rate of one contact** (floor + both active terms), m/s. The overall "maximum force of the push". Also never binds today (floor maxes at 0.07). |
| `shove_asymmetry` | include/collision_params.h | 0.4 | How unevenly the separation splits between the pair. 0 = always 50/50; 1 = fully by the resistance ratio `mass · (0.5 + 0.5·surplus_power)` (hardcoded blend inside `compute_shove`). Note: surplus is raw watts, so any meaningful surplus dominates the mass term — with asymmetry > 0 the fresher rider barely gives way. |

Fatigue enters twice: `w_prime_frac` scales the active push a rider can fund,
and (via surplus power) the resistance split — a cooked rider both pushes
less and gets displaced more.

## 2. C3.0 feel-check — A: lateral feel

Squeeze penalty (`compute_shove` locals, src/lateral_solver.cpp): when a
contact's shove rate exceeds a threshold *and* the rear rider has no passable
gap ahead (`is_blocked`), that rider loses speed — `speed *= (1 − penalty·dt)`.

| Param | File | Value | Mechanics |
|---|---|---|---|
| `kPenaltyScale` | src/lateral_solver.cpp | 0.5 | m⁻¹ — converts the rear rider's shove rate into a speed-penalty rate. The main "cutting through a closed gap costs speed" knob. |
| `kMaxPenaltyRate` | src/lateral_solver.cpp | 5.0 | 1/s — ceiling on the penalty rate; speed decays like `exp(−rate·t)` while squeezing, so 5.0 ⇒ up to ~5 % loss per 10 ms tick. Caps how brutal a squeeze can get. |
| `kPenaltyThresholdRate` | src/lateral_solver.cpp | 0.1 | m/s — shove rates below this are penalty-free (adjacent jostling costs nothing). |
| `kSqueezeRampFrac` | src/lateral_solver.cpp | 0.5 | Fraction of bike length of longitudinal offset at which the penalty reaches full strength — side-by-side contact is free, penalty ramps in as the squeezer sits further behind. |

Free-movement spring-damper (`CollisionParams`, integrated in
`free_movement`, src/lateral_solver.cpp):

| Param | File | Value | Mechanics |
|---|---|---|---|
| `lat_spring_k` | include/collision_params.h | 4.0 | 1/s² — spring toward `lat_target`, scaled by `w_prime_frac` (tired riders steer sluggishly). "Magnetized vs sluggish" line-holding feel. |
| `lat_damping` | include/collision_params.h | 8.0 | 1/s — exact exponential velocity decay; the system is deliberately overdamped (`lat_damping² ≫ 4·lat_spring_k`). Lower ⇒ floatier, faster lateral moves. |
| `ambient_center_k` | include/collision_params.h | 0.5 | 1/s² — weak pull toward road centre when a rider has **no** `lat_target` (not W′-scaled). Must stay well below `lat_spring_k` so shoves/targets overpower it. |

## 3. C3.0 feel-check — B2: crosswind yaw drag

Applied in `Rider::update` (src/rider.cpp): `yaw_factor = min(cap,
(1 + gain·c²/V_a²) · V_a / max(|u|, floor))`, multiplied into `cda_factor`.
Eyeball via the cda_factor / yaw_factor plots under the demo wind
(3.5 m/s @ 60°): a few % CdA on exposed riders, swings consistently windward,
echelon energetically worth forming.

| Param | File | Value | Mechanics |
|---|---|---|---|
| `kYawDragGain` | src/rider.cpp | 1.0 | `k_yaw` in `CdA_ratio = 1 + k_yaw·sin²(yaw)`; ~1.0 ⇒ +10–15 % CdA at ~20° yaw. Magnitude of the crosswind drag penalty. |
| `kMinApparentLon` | src/rider.cpp | 1.0 | m/s — floor on the longitudinal apparent wind `|u|` in the divisor; guards the standing-start spike (u → 0 while the true force → 0). Only matters below ~walking pace. |
| `kYawFactorCap` | src/rider.cpp | 3.0 | Hard cap on the whole yaw multiplier — bounds the standing-start regime the floor doesn't catch. |

## Sign-off record (PLAN.md § C3.0)

- [ ] Shove push signed off (values: ____)
- [ ] A constants signed off (values: ____)
- [ ] B2 constants signed off (values: ____)
