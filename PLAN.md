# CyclingSim — Fix Plan (from AUDIT.md "Suggested order of attack")

## Context

The architecture review (AUDIT.md) found dead physics features from never-initialized fields,
a shutdown-order use-after-free, unsynchronized UI→sim writes, and mirror-tests that don't link
the code under test. Steps 1–3 are **done** (uncommitted). During review of Step 3 the user and
I agreed on a follow-up: the command queue stays in `Simulation::step_fixed()` (it drains in both
realtime and offline modes and guarantees mutations only at step boundaries), but `Simulation`
still conflates the passive stepping engine with the realtime thread driver. **Step 3.5 (new)
extracts the driver**, making it symmetric with the existing `OfflineSimulationRunner`
(`include/analysis.h`).

## Progress

- [x] Step 1 — core physics: `cda_factor=1.0`, `ftp_degrade_rate` threaded config→core,
      fatigue floor, `tests/core/test_terminal_velocity.c`. Done 2026-07-06.
- [x] Step 2 — shutdown order + ImGui shutdown in `~AppState`; uninit members
      (`course`, `lat_target`, `selected_rider`, `TimeControlPanel::w`). Done 2026-07-06.
- [x] Step 3 — command queue in `Simulation` (drained in `step_fixed`), `ISimControl`
      (`include/simcontrol.h`), widgets decoupled from sim.h. Done 2026-07-06.
- [x] Step 3.5 — RealtimeSimRunner extracted (`include/realtime_runner.h`,
      `src/realtime_runner.cpp`); `Simulation` is now passive. Done 2026-07-06.
- [x] Step 4 — `tests/core/test_ftp_factors.c` links core_lib (kPa units);
      mirror `tests/test_ftp_degrade.cpp` deleted. Done 2026-07-06.
- [x] Step 5 — -Wall -Wextra on game_lib (0 warnings); dead-code sweep;
      comment-drift fixes. Bonus: warnings exposed a real bug — `Bike::wheelbase`
      was never initialized from the ctor param. Done 2026-07-06.

Known pre-existing test failures (fail at HEAD too, out of scope): `test_lateral_physics`
(assert tests/test_lateral_physics.cpp:95), `test_solver_compare` (0.1 m/s startup transient).

---

## Step 3.5 — Extract RealtimeSimRunner from Simulation

**Goal:** `Simulation` becomes a passive stepping engine (step_fixed + queue + snapshots);
all thread/pacing/pause/error concerns move to a new `RealtimeSimRunner`, symmetric with
`OfflineSimulationRunner`. The command queue **stays** in `Simulation::step_fixed()` —
it is the step-boundary mutation guarantee for every driver.

### New files: `include/realtime_runner.h`, `src/realtime_runner.cpp`

```cpp
class RealtimeSimRunner : public ISimControl {
public:
  explicit RealtimeSimRunner(Simulation* sim);
  ~RealtimeSimRunner();            // calls stop()

  void start();                    // spawns thread_ running loop()
  void stop();                     // running=false, joins thread_

  // ISimControl (UI talks to the runner, not to Simulation)
  void set_rider_effort(RiderId id, double effort) override; // delegates to sim
  void set_time_factor(double f) override;                   // delegates to sim
  void pause() override;  void resume() override;
  bool is_paused() const override;
  void toggle_pause();

  std::atomic<bool> physics_error{false};
  std::string physics_error_message;

private:
  void loop();                     // body of current Simulation::start_realtime()
  Simulation* sim;                 // not owned
  std::thread thread_;
  std::atomic<bool> running{false};
  std::atomic<bool> paused{false};
};
```
`loop()` = current `sim.cpp:350-401` verbatim, with `time_factor` read via
`sim->get_time_factor()`, dt via `sim->get_dt()`, try/catch setting the runner's
`physics_error*`. `start()` guards double-start (`if (thread_.joinable()) return;`).

### `Simulation` slimming (`include/sim.h`, `src/sim.cpp`)
- Remove: `ISimControl` inheritance (+ `override` markers), `running`, `paused`,
  `start_realtime()`, `pause/resume/toggle_pause/is_paused/stop`, `physics_error`,
  `physics_error_message`, dead `run_max_speed()` decl + `SimulationCondition` fwd decl.
- Keep: `time_factor` as `std::atomic<double>` with `set_time_factor`/new `get_time_factor()`
  (it is stamped into snapshots in `step_fixed`; the runner reads it for pacing;
  offline default 1.0). Keep queue, schedules, snapshots, `reset()` unchanged.
- `reset()` no longer touches `paused`/`physics_error` (moved to runner; add
  runner-side reset of those in `start()`).

### Call-site updates (all found via grep, complete list)
- `include/appstate.h` — replace `std::thread physics_thread` with
  `std::unique_ptr<RealtimeSimRunner> runner;` (declared **after** `sim`).
- `src/appstate.cpp` — ctor: create runner after sim; `runner->set_time_factor(0.2)`
  (replaces `sim->set_time_factor`). Dtor: `runner->stop()` replaces the join block
  (before `screens.reset()`). `start_realtime_tt()`: `runner->stop(); sim->reset(); …;
  runner->start();` (replaces manual thread juggling).
- `src/main.cpp:22-23` — `SDL_AppInit`: `state->runner->start();` replaces thread creation.
  `main.cpp:92-94` — `state->runner->physics_error` / `physics_error_message`;
  on error call `state->runner->stop()`.
- `src/screen.cpp:57,69` — pass `s->runner.get()` (ISimControl*) to
  `add_effort_slider` / `TimeControlPanel`. `screen.cpp:200` PlotScreen pause button:
  `state->runner->toggle_pause()`.
- `src/widget.cpp` PauseButton — already ISimControl*, no change.
- Offline paths (`timetrial.cpp`, `plotting.cpp`, `analysis.cpp`) — no changes; they use
  the passive `Simulation` directly.

### Verification
Build clean; ctest unchanged (only the 2 pre-existing failures); game runs and exits 0 on
SIGTERM (background-run + kill pattern used in Steps 2–3); PlotScreen (P key) still runs its
offline sim. Manual next time the user runs it: pause button, time-factor slider, R-key TT.

---

## Step 4 — Test architecture: FTP tests link the real core

- `core/include/sim_core.h` — declare `saturation`, `altitude_ftp_factor`,
  `fatigue_ftp_factor`; make `rel_press`/`alv_press` static in sim_core.c unless needed;
  delete unused `rel_alv_press`.
- New `tests/core/test_ftp_factors.c` — port assertions from `tests/test_ftp_degrade.cpp`,
  calling the linked functions. **Units: kPa** (p50 ≈ 2.5–4.5, not Pa). Add a case asserting
  the `FTP_FATIGUE_FLOOR` clamp from Step 1.
- Delete `tests/test_ftp_degrade.cpp`.

## Step 5 — Warnings + dead-code sweep

- `CMakeLists.txt` — `target_compile_options(game_lib PRIVATE -Wall -Wextra)` (no -Werror);
  fix surfaced warnings.
- Delete: global `is_close` (`src/rider.cpp:10`); undefined decls in `rider.h`
  (`pow_speed*`, `newton`, `householder`, `set_cda_factor`, `set_mass`,
  `update_power_breakdown` + commented body); `interp_alpha`/`get_interp_alpha`;
  commented blocks at bottom of sim.cpp; AI-monologue comment `widget.cpp` Stopwatch::render;
  unused `row_height` in `RiderPanel::add_bar`; commented ImPlot block in `render_imgui`.
  Also: duplicate `screens = std::make_unique<ScreenManager>(this)` in `appstate.cpp`
  (constructed twice, lines 43 & 82).
- **Keep** `PhysicsEngine::build_group_context` with a "not yet wired" comment.
- Comment-drift: remove `x_lookahead` references (sim.h, lateral_solver.h/.cpp,
  lateral_behavior.h); fix `rider_radius` "NOT duplicated" claim; note disabled
  `speed_penalty` at sim.cpp phase-4 comment.

## Verification (overall)

After each step: `cmake --build build -j && ctest --test-dir build` — expect only the two
pre-existing failures. Smoke test: background-run `build/game`, SIGTERM after ~5 s, expect
exit 0. Update the memory progress file
(`~/.claude/projects/-home-aleks-Documents-cpp-CyclingSim/memory/cyclingsim-audit-fix-progress.md`)
after each completed step.
