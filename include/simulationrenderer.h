#ifndef SIMULATION_RENDERER_H
#define SIMULATION_RENDERER_H

#include "corerenderer.h"
#include "display.h"
#include "sim.h"
#include "widget.h"
#include <memory>

// This renderer extends CoreRenderer with:
// - Camera
// - World-to-screen transform
// - Automatic SnapshotMap generation
// - Course + Rider drawing
class SimulationRenderer : public CoreRenderer {
public:
  SimulationRenderer(SDL_Renderer* r, GameResources* resources, Simulation* sim,
                     std::shared_ptr<Camera> camera);

  ~SimulationRenderer() override = default;

  // Main render function: adds simulation info + world drawables.
  void render_frame() override;
  void update();

  RiderPanel* get_rider_panel() const { return rider_panel; }
  void set_rider_panel(RiderPanel* p) { rider_panel = p; }

  // World drawables
  void add_world_drawable(std::unique_ptr<Drawable> d);

  std::shared_ptr<Camera> get_camera() const { return camera; }

  int pick_rider(double screen_x, double screen_y) const;

  const SnapshotMap& get_snapshot_map() const { return snapshot_front; }
  void build_and_swap_snapshots();

private:
  Simulation* sim;                // Not owned
  std::shared_ptr<Camera> camera; // Owned here
  std::vector<std::unique_ptr<Drawable>> world_drawables;

  RiderPanel* rider_panel = nullptr;

  // Double buffers
  SnapshotMap snapshot_front; // used by render + UI
  SnapshotMap snapshot_back;  // temporary build buffer
  mutable std::mutex snapshot_swap_mtx;
};

#endif
