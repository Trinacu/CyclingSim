  ---
  CyclingSim — Architecture & Code Review

  Verdict up front: the macro-architecture is genuinely good — a pure C99 physics core under a C++ engine, snapshot-based decoupling
  between physics and rendering, and strategy/solver separation for the lateral model. The main problems are (1) two silently-dead 
  physics features caused by fields that are never initialized (aero drag and the brand-new FTP fatigue degradation are both no-ops
  right now), (2) a threading model where UI→sim mutations are unsynchronized, and (3) a testing pattern that copy-pastes the code
  under test instead of linking it, which is how bug #1-adjacent drift keeps slipping through.

  ---
  Part 1 — Architecture

  What's working well (keep these decisions)

  - The C core boundary (core/ as C99, extern "C", own include dir, strict warnings + -Werror, its own C-only tests) is the
  strongest decision in the codebase. It keeps the physics deterministic, ABI-stable, and testable without SDL.
  - Snapshot pipeline (PhysicsEngine::fill_snapshot → Simulation's triple buffer → SimulationRenderer interpolating
  frame_prev/frame_curr) is the right shape for a threaded sim. The renderer never touches live Rider objects.
  - The lateral stack (ILateralBehavior → LateralContext / LateralSolver → LateralUpdate) has clean data contracts with explicitly
  documented dependency policies (lateral_solver.h:5-22). Behaviors are anonymous and stateless; the solver is const and injectable.
  Textbook.
  - The 4-phase engine pipeline in sim.cpp:45-53 with the ordering-guarantee comment is exactly the kind of documentation that pays
  off.

  A1. Threading: the write path from UI to sim is unsynchronized (high)

  The read path (snapshots under snapshot_swap_mtx) is correct. The write path isn't:

  - Simulation::set_effort_schedule (sim.cpp:447) inserts into effort_schedules from the UI thread while step_fixed (sim.cpp:434)
  iterates the same map on the physics thread. Mutating an unordered_map during iteration is UB — this will eventually crash when
  start_realtime_tt or a future UI action installs a schedule mid-run.
  - EffortSlider::on_change → Simulation::set_rider_effort → Rider::set_effort writes state.target_effort with no lock while the
  physics thread reads it. set_time_factor writes a non-atomic double read by the physics loop. Torn doubles are unlikely on x86 but
  it's still a data race (UB), and TSan will light up.
  - PhysicsEngine has frame_mtx and locks it in add_rider and step_and_snapshot, but set_rider_effort (sim.cpp:89) doesn't — the
  mutex protects only half the mutations it exists for.

  Recommendation: route all UI→sim mutations through one funnel. Either take frame_mtx in every PhysicsEngine mutator, or (better,
  and more scalable) a small command queue: UI pushes {rider_id, effort} / {schedule} / {time_factor} into a mutex-guarded vector,
  and step_fixed drains it at the top of each step. That makes the physics thread the sole owner of sim state and eliminates the
  whole class of bug.

  A2. Shutdown order destroys the renderer before the textures (high)

  ~AppState (appstate.cpp:97-111) destroys the renderer/window and calls SDL_Quit() in the destructor body. Members (screens,
  resources) are destroyed after the body — so every widget destructor (Stopwatch, MinimapWidget, RiderPanel, …) and TextureManager
  call SDL_DestroyTexture on a dead renderer, after SDL_Quit. That's use-after-free at every shutdown; it's "working" by luck. Fix
  by resetting members explicitly before tearing down SDL (screens.reset(); resources.reset(); SDL_DestroyRenderer(...); ...), or by
  wrapping the SDL context in its own RAII object declared first in AppState.

  A3. Layering: the UI depends on the whole simulation

  widget.h includes sim.h; sliders and buttons hold Simulation* and call set_rider_effort / set_time_factor / pause() directly.
  Combined with pch.hpp pulling Eigen into everything, a change to collision_params.h recompiles the widgets. Two concrete moves:

  - Widgets already read only via RenderContext (good). Give them a matching narrow write interface — a SimControl struct of
  std::functions or a small abstract interface with set_effort/set_time_factor/toggle_pause — so widget.h needs neither sim.h nor
  the command-queue details from A1. Note the two fixes compose: the control interface's implementation is the command queue.
  - Rider (the physics-side class) includes texturemanager.h, visualmodel.h, and holds const SDL_Texture* image (rider.h:97,
  apparently unused). Presentation identity belongs in the snapshot/render layer; the sim Rider shouldn't know SDL exists. This
  matters because it currently blocks compiling the engine headless.

  A4. Testing strategy: tests that mirror instead of link (high, structural)

  tests/test_ftp_degrade.cpp:60-95 copy-pastes the functions under test ("Copy of the functions under test (inlined so we can
  compile without the full sim_core.c translation unit)"). Two consequences, both already visible:

  1. The test file's constants are in Pa while sim_core.c uses kPa — they only agree because the saturation ratio happens to be
  scale-invariant. Nothing enforces that.
  2. You just fixed the 1.0 - bug in both places by hand. The test passing tells you the copy is right, not the shipped code — and
  indeed the shipped feature is broken in a way this test can't see (see B2).

  The infrastructure for doing it right already exists: tests/core/*.c link core_lib directly. Move the FTP-factor tests there,
  declare altitude_ftp_factor / fatigue_ftp_factor / saturation in sim_core.h (they're already non-static, i.e. exported but
  undeclared — accidental API), and delete the mirror.

  Also: the C++ tests each link the full game_lib (SDL, ImGui, ImPlot) even when testing pure math like the lateral solver.
  lateral_solver.cpp + group.cpp could form a tiny sim_logic library that tests link cheaply.

  A5. Ownership & lifetime nits

  - Rider::course (rider.h:82) is an uninitialized raw pointer; finished() dereferences it unconditionally. add_rider happens to
  call set_course immediately, but one reordering away from UB. Initialize to nullptr or pass the course through the constructor
  (it's required anyway — make invalid states unrepresentable).
  - SimulationScreen::selected_rider (screen.h:82) is never initialized; cycle_rider reads it on first arrow-key press.
  - Camera as shared_ptr + weak_ptr in RenderContext is ceremony without benefit — the renderer owns it and outlives every render
  call; a reference or raw pointer with a lifetime comment would be simpler and removes the per-drawable .lock() failure paths.
  - Config data is triplicated: Rider stores config (which contains a Bike and Team), plus separate bike/team members, plus the same
  numbers copied into the C RiderState. Pick one source of truth (config immutable, RiderState as the working copy) and drop the
  middle layer.

  A6. Build system

  - No warning flags at all on the C++ side — -Wall -Wextra exists only for core_lib (CMakeLists.txt:123). Given the number of
  uninitialized-member bugs below, -Wall -Wextra (and ideally -Wuninitialized, or a one-off cppcheck/clang-tidy run) on game_lib
  would have caught several of them.
  - Eigen is vendored inside include/, mixed with your own headers. Move it to vendor/eigen and mark it SYSTEM so it doesn't pollute
  your include space or warnings.
  - run_tests depends on ${test_targets} which is never populated (CMakeLists.txt:218-221).
  - Repo hygiene: scripts/.venv (megabytes of generated C), files_contents.txt, imgui.ini, both todo and TODO are tracked or lying
  at the root. .gitignore candidates.

  ---
  Part 2 — Implementation findings

  High severity

  B1. Aero drag is zero for every rider — cda_factor is never set.
  rider_state_init (sim_core.c:155) does memset(r, 0, ...) and then assigns every field except cda_factor. All three force/power
  functions compute cda_total = (cda_rider + cda_wheel_drag) * r->cda_factor (sim_core.c:208,230,248) — which is × 0.0. So drag is
  identically zero in all three solvers. The declared-but-never-defined Rider::set_cda_factor (rider.h:84) suggests this was meant
  to be a position/drafting modifier defaulting to 1.0. Fix: r->cda_factor = 1.0; in rider_state_init, and add a core test asserting
  terminal velocity on a flat course is finite/plausible — that test would also have caught this.

  B2. The FTP fatigue-degradation feature is inert — ftp_degrade_rate never reaches the core.
  RiderConfig has ftp_degrade_rate (rider.h:53), but RiderInitParams (sim_core.h:127) has no such field, Rider::Rider never copies
  it, and energy_init never assigns e->ftp_degrade_rate. The memset leaves it 0, so fatigue_ftp_factor (sim_core.c:355) always
  returns 1.0 in the real sim. The feature added in commit 38668d8 and just bug-fixed in your working tree has never actually run.
  This is the direct cost of the mirror-test pattern (A4). Fix: add the field to RiderInitParams, thread it through rider_state_init
  → energy_init.

  B3. fatigue_ftp_factor is unclamped and can go negative. Once B2 is fixed, a long enough ride drives the factor below zero →
  negative FTP → negative power target. Clamp to a floor (e.g. fmax(factor, 0.5) or whatever the physiology should be).

  B4. Threading races — covered as A1; listed here because they're concrete bugs, not just architecture.

  B5. Shutdown use-after-free — covered as A2.

  Medium severity

  B6. Rider::lat_target starts engaged at 0.0. std::optional<double> lat_target = 0.0; (rider.h:76) is an engaged optional targeting
  the centerline — so from the first tick every rider's spring pulls them to lane center, while reset() sets nullopt. Initial and
  post-reset states differ. You almost certainly want = std::nullopt.

  B7. The three solvers disagree on rolling resistance. resistive_force uses (r->crr + env->crr) (sim_core.c:252), but
  pow_speed/pow_speed_prime use only r->crr (sim_core.c:215). So the Newton POWER_BALANCE solver ignores course surface crr while
  the force/energy solvers include it — and Newton's fallback (step_acceleration) uses the other model, so a non-converged step
  changes physics model mid-flight. Factor the resistive terms into one shared function used by all three.

  B8. TimeControlPanel::w is never initialized (widget.h:314, ctor at widget.cpp:614 sets x,y,h only) and is used to draw the
  background rect (widget.cpp:688). Garbage width every run. get_preferred_size computes the correct value — assign it in the
  constructor.

  B9. Solver selection by rider name. rider.cpp:56-61 picks the physics solver from magic strings ("Power", "AccelEnergy"). Put
  SimSolverType in RiderConfig.

  B10. Global hotkeys eat text input. main.cpp handles SDLK_S/P/T/R on raw KEY_DOWN before the screen sees the event, so typing "s"
  in EditableStringField switches screens. Also nothing consults ImGuiIO::WantCaptureKeyboard/Mouse. Route keys through the screen
  first (it returns a consumed flag that main.cpp currently ignores), and gate globals on ImGui capture.

  B11. Grouping hysteresis is unimplemented. GroupingParams defines form_gap/break_gap but GroupTracker::update uses only
  gap_threshold — groups will flicker at the boundary, and group IDs are re-derived from ordinal each tick so "Group 2" is not a
  stable identity. Fine for now, but either implement the hysteresis or delete the params so the struct doesn't lie.

  B12. Doc/code drift in the lateral stack. Comments reference params.x_lookahead in four files, but CollisionParams has no such
  field — the actual window is riders[bi].bike_length. lateral_solver.h:44-46 claims rider_radius is "NOT duplicated here" while the
  struct has the field and the engine fills it. sim.cpp:181-187 documents speed_penalty being applied, but
  Rider::apply_lateral_update has it commented out (rider.cpp:122) — the solver computes penalties nobody consumes. And
  compute_shove's resist = mass * (0.5 + 0.5 * surplus_power) (lateral_solver.cpp:240) multiplies kg by watts — the 0.5 + 0.5·x
  shape strongly suggests x was meant to be a normalized [0,1] fraction, not hundreds of watts (as written, resistance is ~entirely
  power-determined and mass is irrelevant). Given this and shove_kJ = 1e-8 producing micrometre displacements, the shove model reads
  as untuned placeholder — worth a comment saying so, or a pass with test_lateral_physics asserting realistic separations.

  B13. Dead code accumulating. PhysicsEngine::build_group_context is never called;
  Rider::pow_speed/newton/householder/update_power_breakdown are declared but not defined (or fully commented out); is_close exists
  as an unused global in rider.cpp:10 duplicating the static one in the core (ODR hazard for future TUs);
  interp_alpha/get_interp_alpha never updated; ~80 lines of commented ImPlot code in RiderPanel::render_imgui; and
  widget.cpp:222-238 contains a leftover AI-assistant monologue as a comment ("That is a bug in the provided code snippet, but I
  will implement it as requested…"). Deleting all of this costs nothing — it's in git history.

  as an unused global in rider.cpp:10 duplicating the static one in the core (ODR hazard for future TUs);
  interp_alpha/get_interp_alpha never updated; ~80 lines of commented ImPlot code in RiderPanel::render_imgui; and
  widget.cpp:222-238 contains a leftover AI-assistant monologue as a comment ("That is a bug in the provided code snippet, but I
  will implement it as requested…"). Deleting all of this costs nothing — it's in git history.

  Low severity / polish

  - Rider::snapshot() copies name (a std::string) per rider per physics step at 100 Hz, and consume_latest_frame_pair deep-copies
  both snapshots per render frame. Fine at N≤8; if N grows, put immutable identity (name, team) in a separate registry keyed by
  RiderId and keep snapshots POD.
  - piecewise in the C core reports errors via printf and a -1.0 sentinel checked by an assert after use (sim_core.c:120-121) — in
  release builds a bad threshold silently propagates. Make the contract an assert inside the function.
  - energy_update's magic 0.8 floor and piecewise(wbal_frac, 0.2) threshold (sim_core.c:120-126) deserve named constants in
  EnergyState next to the tau params.
  - Course::find_segment throws for x < 0 but silently clamps for x > length — pick one behavior; and get_wind returns a hardcoded 1
  m/s headwind (course.cpp:73, TODO'd).
  - env.rho is constant 1.2234 while the course starts at 1000 m altitude — since you now model altitude for FTP, air density should
  follow (rho ≈ rho0·exp(-alt/H), you already have rel_press).
  - The dynamic_cast<ILayoutWidget*> pattern in every layout container: since effectively all widgets implement it, fold
  get_preferred_size/set_bounds into Widget with defaults and delete the casts.
  - RiderConfig positional brace-init with 13 fields (appstate.cpp:48-77) is a bug factory (the commented-out "Buggy" rider has the
  wrong field count). Until C++20 designated initializers, a RiderConfigBuilder or named-setter chain is worth the boilerplate.
  - Naming: create_endulating → create_undulating; Rider::_pos2d leading underscore is nonstandard style for this codebase.
  - The sim_core.h comment cda_rider /* already includes cda_factor */ contradicts the code (which multiplies by cda_factor) — fix
  alongside B1.

  ---
  Suggested order of attack

  1. B1 + B2 + B3 — one small core patch plus a tests/core/test_terminal_velocity.c; these change every number the sim produces.
  2. A2 shutdown order (5-line fix) and the trivial uninitialized members (B6, B8, selected_rider, course = nullptr).
  3. A1 + A3 together — command queue as the single UI→sim funnel, widgets get the narrow control interface.
  4. A4 — move FTP tests onto core_lib, delete the mirror.
  5. -Wall -Wextra on game_lib, then sweep the dead code (B13).

  The foundations here are better than most hobby sims — the C core and snapshot pipeline are the hard parts and they're right. The
  failure mode to guard against is exactly what B1/B2 show: features wired 90% of the way through the config→core plumbing with no
  end-to-end test that would notice the last missing 10%. A handful of "simulate 60 s on a flat course, assert speed is between 30
  and 55 km/h"–style integration tests on core_lib would make that whole class of bug impossible to ship.
