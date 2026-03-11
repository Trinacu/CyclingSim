#ifndef SIMRENDERER_H
#define SIMRENDERER_H

#include "corerenderer.h"
#include "display.h"
#include "snapshot.h"
#include "ui_layout.h"
#include <memory>

class Simulation;
class Camera;
class RiderPanel;

class SimulationRenderer : public CoreRenderer {
public:
  SimulationRenderer(SDL_Renderer* r, GameResources* resources, Simulation* sim,
                     std::shared_ptr<Camera> camera);

  ~SimulationRenderer() override = default;

  void render_frame() override;
  void reset();
  void update();

  void add_world_drawable(std::unique_ptr<Drawable> d);
  std::shared_ptr<Camera> get_camera() const { return camera; }

  RiderId pick_rider(double screen_x, double screen_y) const;
  std::vector<RiderId> get_rider_ids() const;
  void build_and_swap_snapshots();

  void set_ui_root(std::unique_ptr<UIRoot> ui_root);

  bool handle_event(const SDL_Event* e);

private:
  // sim and camera are owned by AppState !
  Simulation* sim;                // Not owned
  std::shared_ptr<Camera> camera; // Owned here
  std::vector<std::unique_ptr<Drawable>> world_drawables;

  std::unique_ptr<UIRoot> ui_root;

  FrameSnapshot frame_prev; // published previous
  FrameSnapshot frame_curr; // published current
};

#endif
