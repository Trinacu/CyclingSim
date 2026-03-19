#include "sliders.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cassert>
#include <cmath>

// ============================================================
//  Construction / layout
// ============================================================

PiecewiseSlider::PiecewiseSlider(int x_, int y_, int w_, int h_,
                                 PiecewiseMappingConfig cfg_)
    : x(x_), y(y_), w(w_), h(h_), cfg(cfg_) {}

void PiecewiseSlider::set_config(PiecewiseMappingConfig new_cfg) {
  cfg = new_cfg;
}

LayoutSize PiecewiseSlider::get_preferred_size() const { return {w, h}; }

void PiecewiseSlider::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  w = r.w;
  h = r.h;
}

// ============================================================
//  Piecewise linear math
//
//  The track [0, 1] is split at neutral_point into two segments.
//  Each maps linearly between its value endpoints.
//
//  Segment 1: t in [0,             neutral_point]
//             v in [value_lo,      value_mid    ]
//
//  Segment 2: t in [neutral_point, 1            ]
//             v in [value_mid,     value_hi     ]
//
//  slider_to_value and value_to_slider are exact inverses for
//  all v in [value_lo, value_hi].
// ============================================================

double PiecewiseSlider::slider_to_value(double t) const {
  if (t <= cfg.neutral_point) {
    double s = t / cfg.neutral_point;
    return cfg.value_lo + s * (cfg.value_mid - cfg.value_lo);
  } else {
    double s = (t - cfg.neutral_point) / (1.0 - cfg.neutral_point);
    return cfg.value_mid + s * (cfg.value_hi - cfg.value_mid);
  }
}

double PiecewiseSlider::value_to_slider(double v) const {
  if (v <= cfg.value_mid) {
    // Guard against degenerate first segment (value_lo == value_mid).
    if (cfg.value_mid <= cfg.value_lo)
      return 0.0;
    double s = (v - cfg.value_lo) / (cfg.value_mid - cfg.value_lo);
    return std::clamp(s, 0.0, 1.0) * cfg.neutral_point;
  } else {
    // Guard against degenerate second segment (value_hi == value_mid).
    if (cfg.value_hi <= cfg.value_mid)
      return cfg.neutral_point;
    double s = (v - cfg.value_mid) / (cfg.value_hi - cfg.value_mid);
    return cfg.neutral_point +
           std::clamp(s, 0.0, 1.0) * (1.0 - cfg.neutral_point);
  }
}
// ============================================================
//  Render
// ============================================================

void PiecewiseSlider::render(const RenderContext* ctx) {
  assert(get_value && "PiecewiseSlider::get_value must be set before render");

  SDL_Renderer* r = ctx->renderer;

  // Track
  SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
  SDL_FRect bar{(float)x, (float)y + h * 0.4f, (float)w, h * 0.2f};
  SDL_RenderFillRect(r, &bar);

  // Neutral-point marker — shows where the mapping crosses linear_hi (1.0)
  float marker_x =
      (float)x + (float)(cfg.neutral_point * w) - marker_width / 2.0f;
  SDL_FRect marker{marker_x, (float)y, (float)marker_width, (float)h};
  SDL_RenderFillRect(r, &marker);

  // Knob — position is derived from the live value read via get_value
  double current = get_value(ctx);
  double t = value_to_slider(current);
  float knob_x = (float)(x + t * w - h / 4.0);
  SDL_SetRenderDrawColor(r, 240, 240, 240, 255);
  SDL_FRect knob{knob_x, (float)(y + h / 4.0), (float)h / 2.0f,
                 (float)h / 2.0f};
  SDL_RenderFillRect(r, &knob);
}

// ============================================================
//  Event handling
// ============================================================

bool PiecewiseSlider::handle_event(const SDL_Event* e) {
  switch (e->type) {

  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (e->button.x >= x && e->button.x <= x + w && e->button.y >= y &&
        e->button.y <= y + h) {
      dragging = true;
      return true;
    }
    break;

  case SDL_EVENT_MOUSE_BUTTON_UP:
    // Always release drag regardless of pointer position.
    if (dragging) {
      dragging = false;
      return true;
    }
    break;

  case SDL_EVENT_MOUSE_MOTION:
    if (dragging) {
      double local = std::clamp((e->motion.x - x) / (double)w, 0.0, 1.0);
      if (on_change)
        on_change(slider_to_value(local));
      return true;
    }
    break;
  }

  return false;
}

PiecewiseMappingConfig EffortSlider::make_config(double max_effort) {
  PiecewiseMappingConfig cfg;
  cfg.neutral_point = 0.5;
  cfg.value_lo = 0.0;
  cfg.value_mid = 1.0;
  cfg.value_hi = max_effort;
  return cfg;
}

EffortSlider::EffortSlider(int x, int y, int w, int h, Simulation* sim_)
    : sim(sim_) {
  slider = std::make_unique<PiecewiseSlider>(x, y, w, h, make_config(2.0));
  rewire();
}

void EffortSlider::set_rider_id(RiderId id) {
  rider_id = id;
  cached_max_effort = -1.0;
  rewire();
}

void EffortSlider::rewire() {
  // Snapshot rider_id and sim by value so the closures are
  // independent of this object's future state.  If set_rider_id
  // is called again, rewire() replaces both lambdas atomically;
  // the old closures (with the old id) are simply discarded.
  RiderId id = rider_id;
  Simulation* sim_ptr = sim;

  slider->on_change = [sim_ptr, id](double v) {
    if (id >= 0)
      sim_ptr->set_rider_effort(id, v);
  };

  slider->get_value = [id](const RenderContext* ctx) -> double {
    auto it = ctx->riders.find(id);
    return (it != ctx->riders.end()) ? it->second.effort : 0.0;
  };
}

void EffortSlider::render(const RenderContext* ctx) {
  auto it = ctx->riders.find(rider_id);
  if (it != ctx->riders.end()) {
    const double max_effort = it->second.max_effort;
    if (max_effort != cached_max_effort) {
      cached_max_effort = max_effort;
      slider->set_config(make_config(max_effort));
    }
  }
  slider->render(ctx);
}

LayoutSize EffortSlider::get_preferred_size() const {
  return slider->get_preferred_size();
}

void EffortSlider::set_bounds(LayoutRect r) { slider->set_bounds(r); }

bool EffortSlider::handle_event(const SDL_Event* e) {
  return slider->handle_event(e);
}
