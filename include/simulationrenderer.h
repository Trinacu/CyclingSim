#ifndef SIMULATION_RENDERER_H
#define SIMULATION_RENDERER_H

#include "corerenderer.h"
#include "display.h"
#include "sim.h"
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

  void set_target(int uid);

  // Snapshot handling
  void build_snapshot_map();
  const SnapshotMap& get_snapshot_map() const { return snapshotMap; }

  // World drawables
  void add_world_drawable(std::unique_ptr<Drawable> d);

  std::shared_ptr<Camera> get_camera() const { return camera; }

  int pick_rider(double screen_x, double screen_y) const;

private:
  Simulation* sim;                // Not owned
  std::shared_ptr<Camera> camera; // Owned here
  std::vector<std::unique_ptr<Drawable>> world_drawables;

  // A map rider_uid → RiderSnapshot created each frame
  SnapshotMap snapshotMap;
  int target_id;
};

#endif
