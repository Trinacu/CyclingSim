// sliders.h
#ifndef SLIDERS_H
#define SLIDERS_H

#include "display.h"
#include "layout_types.h"
#include "widget.h"
#include <SDL3/SDL.h>
#include <functional>

//  The exponential segment uses:
//    value = pow(exp_base, s * exp_scale)
//  where s = (t - neutral_point) / neutral_point, so s ∈ [0, 1]
//  when t goes from neutral_point to 2*neutral_point.
struct PiecewiseMappingConfig {
  double neutral_point = 0.3; // t-coordinate of the linear/exp join
  double value_lo = 0.1;      // domain value at t = 0
  double value_mid = 1.0;     // domain value at t = neutral_point
  double value_hi = 100.0;    // domain value at t = 1
};

class PiecewiseSlider : public Widget, public ILayoutWidget {
public:
  // Callbacks — set these immediately after construction.
  std::function<void(double)> on_change;
  std::function<double(const RenderContext*)> get_value;

  PiecewiseSlider(int x, int y, int w, int h, PiecewiseMappingConfig cfg = {});

  void set_config(PiecewiseMappingConfig cfg);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  int x, y, w, h;
  PiecewiseMappingConfig cfg;
  bool dragging = false;
  int marker_width = 4;

  // Bidirectional piecewise math — private, pure functions.
  double slider_to_value(double t) const;
  double value_to_slider(double v) const;
};

//  Usage:
//    auto es = std::make_unique<EffortSlider>(x, y, w, h, sim);
//    es->set_rider_id(id);   // or let RiderPanel do this
//    rider_panel->add(std::move(es));
class EffortSlider : public Widget, public ILayoutWidget, public IRiderWidget {
public:
  EffortSlider(int x, int y, int w, int h, Simulation* sim);

  // IRiderWidget — called by RiderPanel::set_rider_id
  void set_rider_id(RiderId id) override;

  // ILayoutWidget — delegates to inner PiecewiseSlider
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget — delegates to inner PiecewiseSlider
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  RiderId rider_id = -1;
  Simulation* sim;
  double cached_max_effort = -1.0; // sentinel: forces update on first render
  std::unique_ptr<PiecewiseSlider> slider;

  // Rebuilds on_change / get_value with the current rider_id
  // captured by value.  Called from the constructor and every
  // time set_rider_id changes the id.
  void rewire();

  static PiecewiseMappingConfig make_config(double max_effort);
};

#endif
