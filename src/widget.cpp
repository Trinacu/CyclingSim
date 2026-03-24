#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "SDL3/SDL_log.h"
#include "SDL3/SDL_pixels.h"
#include "SDL3/SDL_rect.h"
#include "SDL3/SDL_render.h"
#include "SDL3/SDL_surface.h"
#include "display.h"
#include "imgui.h"
#include "implot.h"
#include "sim.h"
#include "simrenderer.h"
#include "sliders.h"
#include "snapshot.h"
#include "widget.h"

std::string format_number(double value, int precision) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std ::string(buffer);
}

void format_time(double seconds, char* text) {
  int totalTenths = static_cast<int>(round(seconds * 10));
  int tenths = totalTenths % 10;
  int totalSeconds = totalTenths / 10;
  int secs = totalSeconds % 60;
  int totalMinutes = totalSeconds / 60;
  int mins = totalMinutes % 60;
  int hours = totalMinutes / 60;

  // Format with leading zeros and fixed positions
  snprintf(text, 11, "%02d:%02d:%02d.%d", hours, mins, secs, tenths);
}

// ======================= PROGRESS BAR =======================

ProgressBar::ProgressBar(int x_, int y_, int w_, int h_,
                         const double* value_ptr, SDL_Color bg_color_,
                         SDL_Color fill_color_, double min, double max)
    : x(x_), y(y_), w(w_), h(h_), min_val(min), max_val(max),
      source([value_ptr]() { return *value_ptr; }), fill_color(fill_color_),
      bg_color(bg_color_) {}

ProgressBar::ProgressBar(int x_, int y_, int w_, int h_, ValueFn getter,
                         SDL_Color bg_color_, SDL_Color fill_color_, double min,
                         double max)
    : x(x_), y(y_), w(w_), h(h_), min_val(min), max_val(max),
      source(std::move(getter)), fill_color(fill_color_), bg_color(bg_color_) {}

ProgressBar::ProgressBar(int x_, int y_, int w_, int h_, RiderId id,
                         RiderDataFn getter, SDL_Color bg_color_,
                         SDL_Color fill_color_, double min, double max)
    : x(x_), y(y_), w(w_), h(h_), min_val(min), max_val(max),
      rider_binding(RiderBinding{id, std::move(getter)}),
      fill_color(fill_color_), bg_color(bg_color_) {}

ProgressBar::~ProgressBar() { SDL_DestroyTexture(label_tex); }

void ProgressBar::set_label(std::string text, TTF_Font* font) {
  label_str = std::move(text);
  label_font = font;
  // Invalidate any previously baked texture if called after first render
  SDL_DestroyTexture(label_tex);
  label_tex = nullptr;
}

void ProgressBar::set_fill_color(SDL_Color c) { fill_color = c; }
void ProgressBar::set_color_fn(ColorFn fn) { color_fn = std::move(fn); }

LayoutSize ProgressBar::get_preferred_size() const { return {w, h}; }

void ProgressBar::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  w = r.w;
  h = r.h;
  // label is centered inside the bar; position changes invalidate it
  SDL_DestroyTexture(label_tex);
  label_tex = nullptr;
}

void ProgressBar::set_rider_id(RiderId id) {
  if (rider_binding)
    rider_binding->id = id;
}

void ProgressBar::build_label_tex(SDL_Renderer* r) {
  if (!label_font || label_str.empty())
    return;
  SDL_Surface* surf = TTF_RenderText_Blended(label_font, label_str.c_str(), 0,
                                             {255, 255, 255, 255});
  if (!surf)
    return;
  label_tex = SDL_CreateTextureFromSurface(r, surf);
  SDL_DestroySurface(surf);
}

void ProgressBar::render(const RenderContext* ctx) {
  double raw = 0.0;

  if (rider_binding) {
    auto it = ctx->riders.find(rider_binding->id);
    if (it == ctx->riders.end())
      return;
    raw = rider_binding->getter(it->second);
  } else if (source) {
    raw = source();
  } else {
    return;
  }

  double range = max_val - min_val;
  double t =
      (range != 0.0) ? std::clamp((raw - min_val) / range, 0.0, 1.0) : 0.0;

  draw(ctx->renderer, t);
}

void ProgressBar::draw(SDL_Renderer* r, double t) {
  SDL_SetRenderDrawColor(r, bg_color.r, bg_color.g, bg_color.b, bg_color.a);
  SDL_FRect bg{(float)x, (float)y, (float)w, (float)h};
  SDL_RenderFillRect(r, &bg);

  // Fill
  SDL_Color fc = color_fn ? color_fn(t) : fill_color;
  SDL_SetRenderDrawColor(r, fc.r, fc.g, fc.b, fc.a);
  SDL_FRect fill{(float)x, (float)y, (float)(w * t), (float)h};
  SDL_RenderFillRect(r, &fill);

  // Label — baked once, centered in the bar
  if (!label_str.empty()) {
    if (!label_tex)
      build_label_tex(r);
    if (label_tex) {
      float tw, th;
      SDL_GetTextureSize(label_tex, &tw, &th);
      SDL_FRect dst{x + (w - tw) * 0.5f, y + (h - th) * 0.5f, tw, th};
      SDL_RenderTexture(r, label_tex, nullptr, &dst);
    }
  }
}

LayoutSize Stopwatch::get_preferred_size() const {
  // If the background has already been created by a prior render pass,
  // return the actual baked dimensions.
  if (bg_width > 0)
    return {bg_width, bg_height};

  // Otherwise measure the fixed-width reference string with the font so
  // UIRoot::resolve() gets accurate dimensions even before first render.
  int w = 0, h = 0;
  TTF_GetStringSize(font, "00:00:00.0", 0, &w, &h);
  return {w + 2 * content_offset, h + 2 * content_offset};
}

void Stopwatch::set_bounds(LayoutRect r) {
  screen_x = r.x;
  screen_y = r.y;
  // Width/height are determined by font metrics, not imposed from outside.
}

SDL_Texture* Stopwatch::render_time(SDL_Renderer* renderer, const char* s,
                                    int& out_w, int& out_h) {
  SDL_Surface* surf =
      TTF_RenderText_LCD(font, s, 0, text_color, SDL_Color{0, 0, 0, 200});
  if (!surf) {
    SDL_Log("TTF_RenderText_LCD failed: %s", SDL_GetError());
    return nullptr;
  }
  out_w = surf->w;
  out_h = surf->h;

  SDL_Texture* new_texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);

  if (!new_texture) {
    SDL_Log("SDL_CreateTextureFromSurface failed: %s", SDL_GetError());
    return nullptr;
  }
  return new_texture;
}

void Stopwatch::update_texture(const RenderContext* ctx) {
  format_time(ctx->sim_time, text);

  if (texture) {
    SDL_DestroyTexture(texture);
    texture = nullptr;
  }
  texture = render_time(ctx->renderer, text, width, height);
}

void Stopwatch::render(const RenderContext* ctx) {
  // 1) Check if we need to build the background frame
  if (bg_texture == nullptr) {
    bg_texture = create_base(ctx->renderer);
  }

  // 2) Check if we need to update the time digits
  Uint32 now_ticks = SDL_GetTicks();
  if (texture == nullptr || (now_ticks - last_update_ticks) >=
                                static_cast<Uint32>(update_interval_ms)) {
    update_texture(ctx);
    last_update_ticks = now_ticks;
  }

  // 3) Draw Background Frame
  if (bg_texture) {
    // Note: The background includes padding, so it is larger than the text
    SDL_FRect dst{static_cast<float>(screen_x), static_cast<float>(screen_y),
                  static_cast<float>(bg_width), static_cast<float>(bg_height)};
    SDL_RenderTexture(ctx->renderer, bg_texture, nullptr, &dst);
  }

  // 4) Draw Text (inset by content_offset)
  if (texture) {
    // Note: width/height here refers to the BACKGROUND dimensions calculated in
    // create_base. We use texture size from the text itself for the source? No,
    // render_time set width/height? Wait, render_time sets 'width' and 'height'
    // MEMBER variables which overwrites the background size? FIX: The member
    // variables `width` and `height` should likely store the total widget size.
    // But render_time takes `width` and `height` by reference as `out_w`,
    // `out_h`.

    // Let's inspect the logic:
    // create_base sets `width` and `height` to the padded size.
    // update_texture calls render_time(..., width, height).
    // This effectively overwrites the widget size with the text size every
    // update! That is a bug in the provided code snippet, but I will implement
    // it as requested while fixing the overwrite to use local variables for the
    // text size so the background doesn't shrink.

    float txt_w, txt_h;
    SDL_GetTextureSize(texture, &txt_w, &txt_h);

    SDL_FRect dst = {static_cast<float>(screen_x + content_offset),
                     static_cast<float>(screen_y + content_offset),
                     static_cast<float>(txt_w), static_cast<float>(txt_h)};
    SDL_RenderTexture(ctx->renderer, texture, nullptr, &dst);
  }
}

SDL_Texture* Stopwatch::create_base(SDL_Renderer* renderer) {
  int w, h;
  // Calculate size based on a dummy string "00:00:00.0" to ensure fixed width
  TTF_GetStringSize(font, "00:00:00.0", 0, &w, &h);

  int padded_w = w + 2 * content_offset;
  int padded_h = h + 2 * content_offset;
  bg_width = padded_w;
  bg_height = padded_h;

  SDL_Surface* surf =
      SDL_CreateSurface(padded_w, padded_h, SDL_PIXELFORMAT_XRGB8888);
  if (!surf) {
    SDL_Log("SDL_CreateSurface failed: %s", SDL_GetError());
    return nullptr;
  }

  // Store the total size in member variables
  width = surf->w;
  height = surf->h;

  const SDL_PixelFormatDetails* fmt_details =
      SDL_GetPixelFormatDetails(surf->format);
  Uint32 transparent = SDL_MapRGBA(fmt_details, nullptr, 0, 0, 0, 0);
  Uint32 hlPix = SDL_MapRGBA(fmt_details, nullptr, 177, 177, 177, 255);
  Uint32 shPix = SDL_MapRGBA(fmt_details, nullptr, 77, 77, 77, 255);

  SDL_Rect r{0, 0, padded_w, padded_h};
  SDL_FillSurfaceRect(surf, &r, transparent);

  SDL_Rect topEdge = {0, 0, padded_w, edge_thickness};
  SDL_FillSurfaceRect(surf, &topEdge, hlPix);

  SDL_Rect leftEdge = {0, 0, edge_thickness, padded_h};
  SDL_FillSurfaceRect(surf, &leftEdge, hlPix);

  SDL_Rect bottomEdge = {0, padded_h - edge_thickness, padded_w,
                         edge_thickness};
  SDL_FillSurfaceRect(surf, &bottomEdge, shPix);

  SDL_Rect rightEdge = {padded_w - edge_thickness, 0, edge_thickness, padded_h};
  SDL_FillSurfaceRect(surf, &rightEdge, shPix);

  SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
  return tex;
}

LateralOverview::LateralOverview(int w_, int h_, const Course* c)
    : course(c), w_(w_), h_(h_) {}

LateralOverview::~LateralOverview() { SDL_DestroyTexture(tex); }

LayoutSize LateralOverview::get_preferred_size() const { return {w_, h_}; }

void LateralOverview::set_bounds(LayoutRect r) {
  x_ = r.x;
  y_ = r.y;
  // w and h are fixed at construction and don't change.
}

void LateralOverview::ensure_target(SDL_Renderer* r) {
  if (tex)
    return;
  tex = SDL_CreateTexture(r, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET,
                          w_, h_);
  SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
}

void LateralOverview::draw_into_target(const RenderContext* ctx) {
  SDL_SetRenderTarget(ctx->renderer, tex);
  SDL_SetRenderDrawColor(ctx->renderer, 0, 0, 0, 0);
  SDL_RenderClear(ctx->renderer);

  SDL_SetRenderDrawColor(ctx->renderer, 40, 40, 40, 255);
  SDL_RenderLine(ctx->renderer, w_ / 2.0, 0, w_ / 2.0, h_);
  SDL_SetRenderDrawColor(ctx->renderer, 255, 255, 255, 255);
  SDL_RenderLine(ctx->renderer, w_ - kPad, 0, w_ - kPad, h_);
  SDL_RenderLine(ctx->renderer, kPad, 0, kPad, h_);

  auto cam = ctx->camera_weak.lock();
  if (!cam) {
    SDL_SetRenderTarget(ctx->renderer, nullptr);
    return;
  }

  double lon_min = cam->get_pos()[0] - cam->get_world_width() / 2;
  double lon_max = cam->get_pos()[0] + cam->get_world_width() / 2;

  // for (const auto& [_, rs] : ctx->riders) {
  //   SDL_FPoint pos = to_widget(rs.pos, rs.lat_pos, lon_min, lon_max,
  //                              course->get_road_width(rs.pos) / 2);
  //   SDL_FRect rect = {pos.x, pos.y, 3, 3};
  //   SDL_RenderRect(ctx->renderer, &rect);
  // }

  for (const auto& [_, rs] : ctx->riders) {
    SDL_FPoint pos = to_widget(rs.pos, rs.lat_pos, lon_min, lon_max,
                               course->get_road_width(rs.pos) / 2);
    SDL_FRect rect = {pos.x - 2, pos.y - 2, 4, 4};
    SDL_RenderRect(ctx->renderer, &rect);
  }

  SDL_SetRenderTarget(ctx->renderer, nullptr);
}

SDL_FPoint LateralOverview::to_widget(double lon_pos, double lat_pos,
                                      double lon_min, double lon_max,
                                      double road_half) const {
  float y = (lon_max - lon_pos) / (lon_max - lon_min) * h_;
  float x = w_ / 2.0 + lat_pos / road_half * w_;
  return SDL_FPoint{x, y};
}

void LateralOverview::render(const RenderContext* ctx) {
  if (ctx->riders.empty())
    return;

  ensure_target(ctx->renderer);
  draw_into_target(ctx);

  // background
  SDL_SetRenderDrawColor(ctx->renderer, 20, 20, 20, 200);
  SDL_FRect bg{(float)x_, (float)y_, (float)w_, (float)h_};
  SDL_RenderFillRect(ctx->renderer, &bg);

  SDL_RenderTexture(ctx->renderer, tex, nullptr, &bg);
}

/* MINIMAP */
// TODO - have the vertical scale be 1 so it shows proportional
// except when small vert changes, give a minimum, and with big
// changes give a maximum

MinimapWidget::MinimapWidget(int x, int y, int w, int h, const Course* course)
    : course(course), x(x), y(y), w(w), h(h) {
  world_x_min = world_x_max = course->points[0].x;
  world_y_min = world_y_max = course->points[0].y;

  for (const auto& p : course->points) {
    world_x_min = std::min(world_x_min, p.x);
    world_x_max = std::max(world_x_max, p.x);
    world_y_min = std::min(world_y_min, p.y);
    world_y_max = std::max(world_y_max, p.y);
  }

  // prevent degenerate range on flat courses
  if (world_y_max - world_y_min < 5.0) {
    world_y_min -= 2.5;
    world_y_max += 2.5;
  }
}

LayoutSize MinimapWidget::get_preferred_size() const { return {w, h}; }

void MinimapWidget::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  // w and h are fixed at construction and don't change.
}

static SDL_FColor slope_to_color(double slope) {
  // flat=green, uphill->red, downhill->blue
  if (slope > 0.0) {
    float t = std::min((float)(slope / 0.15), 1.0f); // saturates at 15%
    return {1.0f * t, 1.0f * (1.0f - t), 0.0f, 0.8f};
  } else {
    float t = std::min((float)(-slope / 0.08), 1.0f); // saturates at 8%
    return {0.0f, 1.0f * (1.0f - t), 1.0f * t, 0.8f};
  }
}

void MinimapWidget::bake_profile(SDL_Renderer* r) {
  profile_tex = SDL_CreateTexture(r, SDL_PIXELFORMAT_RGBA8888,
                                  SDL_TEXTUREACCESS_TARGET, w, h);
  SDL_SetTextureBlendMode(profile_tex, SDL_BLENDMODE_BLEND);
  SDL_SetRenderTarget(r, profile_tex);
  SDL_SetRenderDrawColor(r, 0, 0, 0, 0);
  SDL_RenderClear(r);

  // draw filled polygon below the line (optional, nicer look)
  // ...

  // draw the profile line
  const auto& pts = course->points;
  std::vector<SDL_FPoint> screen_pts;
  for (auto& p : pts)
    screen_pts.push_back(world_to_mini(p.x, p.y));

  float bottom_y = (float)(h - pad);

  std::vector<SDL_Vertex> verts;
  verts.reserve((pts.size() - 1) * 6);

  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double slope = (pts[i + 1].y - pts[i].y) / (pts[i + 1].x - pts[i].x);
    SDL_FColor col = slope_to_color(slope);

    SDL_FPoint tl = screen_pts[i];
    SDL_FPoint tr = screen_pts[i + 1];
    SDL_FPoint bl = {tl.x, bottom_y};
    SDL_FPoint br = {tr.x, bottom_y};

    // two triangles: tl-tr-br and tl-br-bl
    SDL_Vertex quad[6] = {
        {{tl.x, tl.y}, col, {0, 0}}, {{tr.x, tr.y}, col, {0, 0}},
        {{br.x, br.y}, col, {0, 0}},

        {{tl.x, tl.y}, col, {0, 0}}, {{br.x, br.y}, col, {0, 0}},
        {{bl.x, bl.y}, col, {0, 0}},
    };
    for (auto& v : quad)
      verts.push_back(v);
  }

  SDL_RenderGeometry(r, nullptr, verts.data(), (int)verts.size(), nullptr, 0);

  SDL_SetRenderDrawColor(r, 180, 220, 255, 255);
  SDL_RenderLines(r, screen_pts.data(), screen_pts.size());

  SDL_SetRenderTarget(r, nullptr); // restore
}

SDL_FPoint MinimapWidget::world_to_mini(double wx, double wy) const {
  float nx = (wx - world_x_min) / (world_x_max - world_x_min);
  float ny = (wy - world_y_min) / (world_y_max - world_y_min);
  return {
      pad + nx * (w - 2 * pad),
      (h - pad) - ny * (h - 2 * pad) // flip Y
  };
}

void MinimapWidget::render(const RenderContext* ctx) {
  if (!profile_tex)
    bake_profile(ctx->renderer);

  // background
  SDL_SetRenderDrawColor(ctx->renderer, 20, 20, 20, 200);
  SDL_FRect bg{(float)x, (float)y, (float)w, (float)h};
  SDL_RenderFillRect(ctx->renderer, &bg);

  SDL_FRect dst = bg;
  SDL_RenderTexture(ctx->renderer, profile_tex, nullptr, &dst);

  // rider dots — pos2d.y() is already altitude
  // for (auto& [uid, snap] : ctx->curr_frame->riders) {
  //     auto pt = world_to_mini(snap.pos2d.x(), snap.pos2d.y());
  //     SDL_SetRenderDrawColor(ctx->renderer, 255, 80, 80, 255);
  //     SDL_FRect dot{pt.x - 3, pt.y - 3, 6, 6};
  //     SDL_RenderFillRect(ctx->renderer, &dot);
  // }
}

/* UI elements */

Button::Button(int x_, int y_, int w_, int h_, std::string label_,
               TTF_Font* font_, Callback cb)
    : x(x_), y(y_), w(w_), h(h_), label(std::move(label_)), font(font_),
      on_click(std::move(cb)) {}

Button::~Button() { SDL_DestroyTexture(label_tex); }

void Button::build_label_text(SDL_Renderer* r) {
  SDL_Surface* surf = TTF_RenderText_Blended(font, label.c_str(), 0,
                                             SDL_Color{255, 255, 255, 255});
  if (!surf)
    return;
  label_tex = SDL_CreateTextureFromSurface(r, surf);
  SDL_DestroySurface(surf);
}

void Button::set_label(std::string new_label) {
  label = new_label;
  SDL_DestroyTexture(label_tex);
  label_tex = nullptr;
}

LayoutSize Button::get_preferred_size() const { return {w, h}; }

void Button::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  w = r.w;
  h = r.h;
}

void Button::render(const RenderContext* ctx) {
  SDL_Renderer* r = ctx->renderer;

  // Colors
  if (pressed)
    SDL_SetRenderDrawColor(r, 60, 60, 60, 255);
  else if (hovered)
    SDL_SetRenderDrawColor(r, 90, 90, 90, 255);
  else
    SDL_SetRenderDrawColor(r, 70, 70, 70, 255);

  SDL_FRect bg{(float)x, (float)y, (float)w, (float)h};
  SDL_RenderFillRect(r, &bg);

  if (!label_tex)
    build_label_text(r);

  if (label_tex) {
    float tw, th;
    SDL_GetTextureSize(label_tex, &tw, &th);
    SDL_FRect dst{x + (w - tw) * 0.5f, y + (h - th) * 0.5f, tw, th};
    SDL_RenderTexture(r, label_tex, nullptr, &dst);
  }
}

bool Button::handle_event(const SDL_Event* e) {
  int mx, my;

  switch (e->type) {

  case SDL_EVENT_MOUSE_MOTION:
    mx = e->motion.x;
    my = e->motion.y;
    hovered = (mx >= x && mx <= x + w && my >= y && my <= y + h);
    return hovered;

  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (e->button.button == SDL_BUTTON_LEFT && hovered) {
      pressed = true;
      return true;
    }
    break;

  case SDL_EVENT_MOUSE_BUTTON_UP:
    if (e->button.button == SDL_BUTTON_LEFT) {
      if (pressed && hovered)
        if (on_click)
          on_click(); // <-- fire callback

      pressed = false;
      return hovered; // consumed if inside
    }
    break;
  }

  return false;
}

PauseButton::PauseButton(int x, int y, int w, int h, Simulation* sim,
                         TTF_Font* font)
    : Button(x, y, w, h, sim->is_paused() ? "Resume" : "Pause", font,
             [sim, this]() {
               if (sim->is_paused()) {
                 sim->resume();
                 set_label("Pause");
               } else {
                 sim->pause();
                 set_label("Resume");
               }
             }) {}

void PauseButton::render(const RenderContext* ctx) {
  // Update label before drawing
  // BUT NOT BY READING STRAIGHT FROM SIM!
  // set_label(sim->is_paused() ? "Resume" : "Pause");

  Button::render(ctx);
}

TimeControlPanel::TimeControlPanel(int x_, int y_, int h_, TTF_Font* font,
                                   Simulation* sim_)
    : x(x_), y(y_), h(h_), sim(sim_) {
  int widget_h = h_ / 2;
  int widget_y = h_ / 4; // relative to panel top

  // -- ValueField (time factor readout) --
  child_rel_positions.push_back({next_x - x, 2});
  auto tf =
      std::make_unique<ValueField>(next_x, y + 2, valfield_w, h - 4, font);
  time_factor_field = tf.get();
  children.push_back(std::move(tf));
  next_x += valfield_w + gap_w;

  child_rel_positions.push_back({next_x - x, widget_y});
  PiecewiseMappingConfig time_cfg;
  auto slider = std::make_unique<PiecewiseSlider>(next_x, widget_y, slider_w,
                                                  widget_h, time_cfg);
  slider->on_change = [sim_](double v) { sim_->set_time_factor(v); };
  slider->get_value = [](const RenderContext* ctx) { return ctx->time_factor; };
  children.push_back(std::move(slider));
  next_x += slider_w + gap_w;

  auto make_factor_button = [&](double val) {
    child_rel_positions.push_back({next_x - x, widget_y});
    children.push_back(std::make_unique<TimeFactorButton>(
        next_x, widget_y, button_w, widget_h, val, sim_, font));
    next_x += gap_w + button_w;
  };

  make_factor_button(0.5);
  make_factor_button(1.0);
  make_factor_button(20.0);

  child_rel_positions.push_back({next_x - x, widget_y});
  children.push_back(std::make_unique<PauseButton>(next_x, widget_y, button_w,
                                                   widget_h, sim, font));
}

LayoutSize TimeControlPanel::get_preferred_size() const {
  // Right edge of the PauseButton relative to panel origin:
  //   gap_w (initial offset)
  //   + valfield_w + gap_w
  //   + slider_w + gap_w
  //   + 3 * (gap_w + button_w)   <- factor buttons
  //   + (gap_w + button_w)       <- existing extra skip (preserved)
  //   + button_w                 <- pause button width
  //   + gap_w                    <- right margin
  int w =
      gap_w + valfield_w + gap_w + slider_w + gap_w + 4 * (gap_w + button_w);
  return {w, h};
}

void TimeControlPanel::do_layout(int base_x, int base_y) {
  for (int i = 0; i < static_cast<int>(children.size()); ++i) {
    if (auto* lw = dynamic_cast<ILayoutWidget*>(children[i].get())) {
      auto [rx, ry] = child_rel_positions[i];
      LayoutSize cs = lw->get_preferred_size();
      lw->set_bounds({base_x + rx, base_y + ry, cs.w, cs.h});
    }
  }
}

void TimeControlPanel::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  do_layout(x, y);
}

void TimeControlPanel::render(const RenderContext* ctx) {
  SDL_Renderer* r = ctx->renderer;

  // (Optional) panel background
  SDL_SetRenderDrawColor(r, 40, 40, 40, 200);
  SDL_FRect bg{(float)x, (float)y, static_cast<float>(w),
               static_cast<float>(h)};
  SDL_RenderFillRect(r, &bg);

  char buf[8];
  snprintf(buf, sizeof(buf), "%.2f", ctx->time_factor);
  time_factor_field->set_text(buf);

  // Children
  for (auto& w : children)
    w->render(ctx);
}

bool TimeControlPanel::handle_event(const SDL_Event* e) {
  for (auto& w : children)
    if (w->handle_event(e))
      return true;

  return false;
}

ValueField::ValueField(int x_, int y_, int w_, int h_, TTF_Font* font_)
    : x(x_), y(y_), w(w_), h(h_), font(font_) {}

ValueField::~ValueField() {
  if (texture)
    SDL_DestroyTexture(texture);
  if (bg_texture)
    SDL_DestroyTexture(bg_texture);
}

LayoutSize ValueField::get_preferred_size() const { return {w, h}; }

void ValueField::set_bounds(LayoutRect r) {
  // If the layout changes the size, invalidate the background texture
  // so it is recreated at the new dimensions next render.
  if (r.w != w || r.h != h) {
    if (bg_texture) {
      SDL_DestroyTexture(bg_texture);
      bg_texture = nullptr;
    }
  }
  set_position(r.x, r.y);
  w = r.w;
  h = r.h;
}

void ValueField::set_text(const std::string& text) {
  if (text != current_text) {
    current_text = text;
    if (texture) {
      SDL_DestroyTexture(texture);
      texture = nullptr;
    }
  }
}

void ValueField::create_bg(SDL_Renderer* renderer) {
  SDL_Surface* surf = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_RGBA32);
  SDL_FillSurfaceRect(surf, nullptr,
                      SDL_MapRGBA(SDL_GetPixelFormatDetails(surf->format),
                                  nullptr, 100, 100, 100, 200));
  bg_texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
}

void ValueField::update_texture(SDL_Renderer* renderer) {
  if (current_text.empty())
    return;

  SDL_Surface* surf =
      TTF_RenderText_Blended(font, current_text.c_str(), 0, text_color);
  if (!surf)
    return;

  texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
}

void ValueField::render(const RenderContext* ctx) {
  SDL_Renderer* renderer = ctx->renderer;

  if (!bg_texture)
    create_bg(renderer);

  if (!texture && !current_text.empty())
    update_texture(renderer);

  SDL_FRect rect{(float)x, (float)y, (float)w, (float)h};
  SDL_RenderTexture(renderer, bg_texture, nullptr, &rect);

  if (texture) {
    float tex_w, tex_h;
    SDL_GetTextureSize(texture, &tex_w, &tex_h);

    rect = SDL_FRect{(float)x + w - tex_w - 4, (float)y + (h - tex_h) * 0.5f,
                     (float)tex_w, (float)tex_h};

    SDL_RenderTexture(renderer, texture, nullptr, &rect);
  }
}

RiderValueField::RiderValueField(int x, int y, int w, int h, TTF_Font* font,
                                 DataGetter getter_)
    : ValueField(x, y, w, h, font), getter(getter_) {}

void RiderValueField::render_with_snapshot(const RenderContext* ctx,
                                           const RiderRenderState* snap) {
  if (!snap)
    return;

  std::string new_text = getter(*snap);
  set_text(new_text);

  // Use base render method
  render(ctx);
}

// ======================= METRIC ROW =======================

MetricRow::MetricRow(int x_, int y_, TTF_Font* f, RiderId id, std::string label,
                     std::string unit, RiderValueField::DataGetter getter)
    : x(x_), y(y_), font(f), rider_id(id), label_txt(std::move(label)),
      unit_txt(std::move(unit)) {
  field = std::make_unique<RiderValueField>(x + label_width, y, field_w,
                                            field_h, font, getter);
}

MetricRow::~MetricRow() {
  SDL_DestroyTexture(label_tex);
  SDL_DestroyTexture(unit_tex);
}

void MetricRow::set_rider_id(RiderId id) { rider_id = id; }

LayoutSize MetricRow::get_preferred_size() const {
  // label_width + field_w + padding + approx unit text width
  return {label_width + field_w + 5 + 30, row_h};
}

void MetricRow::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  field->set_position(x + label_width, y);
}

void MetricRow::render(const RenderContext* ctx) {
  auto it = ctx->riders.find(rider_id);
  if (it == ctx->riders.end())
    return;
  const RiderRenderState& rs = it->second;

  if (!label_tex) {
    SDL_Surface* s = TTF_RenderText_Blended(font, label_txt.c_str(), 0,
                                            {200, 200, 200, 255});
    label_tex = SDL_CreateTextureFromSurface(ctx->renderer, s);
    SDL_DestroySurface(s);
  }
  if (!unit_tex && !unit_txt.empty()) {
    SDL_Surface* s =
        TTF_RenderText_Blended(font, unit_txt.c_str(), 0, {150, 150, 150, 255});
    unit_tex = SDL_CreateTextureFromSurface(ctx->renderer, s);
    SDL_DestroySurface(s);
  }

  if (label_tex) {
    float w, h;
    SDL_GetTextureSize(label_tex, &w, &h);
    SDL_FRect r = {(float)x, (float)y + (field_h - h) / 2.f, w, h};
    SDL_RenderTexture(ctx->renderer, label_tex, nullptr, &r);
  }

  field->render_with_snapshot(ctx, &rs);

  if (unit_tex) {
    float w, h;
    SDL_GetTextureSize(unit_tex, &w, &h);
    SDL_FRect r = {(float)(x + label_width + field->get_width() + 5),
                   (float)y + (field_h - h) / 2.f, w, h};
    SDL_RenderTexture(ctx->renderer, unit_tex, nullptr, &r);
  }
}

// ======================= RIDER PANEL =======================
// WARNING - this assumes the font isnt deallocated
RiderPanel::RiderPanel(int x_, int y_, TTF_Font* f) : x(x_), y(y_), font(f) {
  build_fields();
}

void RiderPanel::set_rider_id(RiderId new_id) {
  id = new_id;
  for (auto& child : children) {
    if (auto* rw = dynamic_cast<IRiderWidget*>(child.get()))
      rw->set_rider_id(new_id);
  }
}

void RiderPanel::build_fields() {
  add_row("Speed", "km/h", [](const RiderRenderState& s) {
    return format_number(3.6 * s.speed, 2);
  });
  add_row("Power", "W", [](const RiderRenderState& s) {
    return format_number(s.power, 0); // Precision 0 for watts
  });
  add_row("Dist", "km", [](const RiderRenderState& s) {
    return format_number(s.pos / 1000.0, 3);
  });
  add_row("Grad", "%", [](const RiderRenderState& s) {
    return format_number(s.slope * 100.0);
  });

  add_bar("W' bal",
          [](const RiderRenderState& rs) { return rs.wbal_fraction; });
  // TODO - seems like now we have to pass sim into RiderPanel so that we can
  // get it to EffortSlider?
  // add_effort_slider(sim, )
}

LayoutSize RiderPanel::get_preferred_size() const {
  int total_h = 30; // title
  int max_w = 200;  // minimum panel width
  for (const auto& child : children) {
    if (const auto* lw = dynamic_cast<const ILayoutWidget*>(child.get())) {
      LayoutSize s = lw->get_preferred_size();
      total_h += s.h;
      max_w = std::max(max_w, s.w);
    }
  }
  return {max_w, total_h};
}

void RiderPanel::set_bounds(LayoutRect r) {
  x = r.x;
  y = r.y;
  dirty = true;
  SDL_DestroyTexture(title_tex);
  title_tex = nullptr;
}

void RiderPanel::do_layout() {
  int cursor_y = y + 30; // 30px reserved for title
  for (auto& child : children) {
    if (auto* lw = dynamic_cast<ILayoutWidget*>(child.get())) {
      LayoutSize s = lw->get_preferred_size();
      lw->set_bounds({x, cursor_y, s.w, s.h});
      cursor_y += s.h;
    }
  }
  dirty = false;
}

RiderPanel::~RiderPanel() {
  if (title_tex)
    SDL_DestroyTexture(title_tex);
}

void RiderPanel::add_row(std::string label, std::string unit,
                         RiderValueField::DataGetter getter) {
  children.push_back(std::make_unique<MetricRow>(
      x, y, font, id, std::move(label), std::move(unit), getter));
  dirty = true;
}

void RiderPanel::add_bar(std::string label, ProgressBar::RiderDataFn getter,
                         SDL_Color bg_color, SDL_Color fill_color, double min,
                         double max) {
  auto bar = std::make_unique<ProgressBar>(x, y, 160, 16, id, std::move(getter),
                                           bg_color, fill_color, min, max);
  bar->set_label(std::move(label), font);
  bar->set_color_fn([](double t) -> SDL_Color {
    if (t > 0.5)
      return {80, 200, 80, 255};
    if (t > 0.2)
      return {220, 180, 0, 255};
    return {220, 60, 60, 255};
  });
  children.push_back(std::move(bar));
  dirty = true;
  int row_height = 30;
}

void RiderPanel::add_effort_slider(Simulation* sim, double max_effort) {
  auto es = std::make_unique<EffortSlider>(x, y, 160, 20, sim);
  es->set_rider_id(id);
  children.push_back(std::move(es));
  dirty = true;
}

void RiderPanel::render(const RenderContext* ctx) {
  if (id < 0)
    return;

  if (dirty)
    do_layout();

  auto it = ctx->riders.find(id);
  if (it == ctx->riders.end())
    return;

  const RiderRenderState& rs = it->second;
  if (title != rs.name) {
    title = rs.name;
    if (title_tex)
      SDL_DestroyTexture(title_tex);

    SDL_Surface* s =
        TTF_RenderText_Blended(font, title.c_str(), 0, {255, 255, 255});
    title_tex = SDL_CreateTextureFromSurface(ctx->renderer, s);
    SDL_DestroySurface(s);
  }

  if (title_tex) {
    float tw, th;
    SDL_GetTextureSize(title_tex, &tw, &th);
    SDL_FRect r = {(float)x, (float)y, tw, th};
    SDL_RenderTexture(ctx->renderer, title_tex, nullptr, &r);
  }

  // HACK for now: Iterate rows, find their internal fields, and update their
  // ID? Proper way: Refactor ValueField::render to take a snapshot, not look
  // it up itself. Draw all rows
  for (auto& child : children)
    child->render(ctx);
}

bool RiderPanel::handle_event(const SDL_Event* e) {
  // if we have overlapping widgets, we need to iterate in reverse
  for (auto& child : children)
    if (child->handle_event(e))
      return true;
  return false;
}

void RiderPanel::render_imgui(const RenderContext* ctx) {
  // if (id < 0)
  //   return;
  //
  // auto it = ctx->riders.find(id);
  // if (it == ctx->riders.end())
  //   return;
  //
  // const RiderRenderState& rs = it->second;
  //
  // // compute where the plot goes based on panel position
  // ImVec2 pos((float)x, (float)y + 140);
  // ImVec2 size(300, 300);
  //
  // ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
  // ImGui::SetNextWindowSize(size, ImGuiCond_Always);
  //
  // ImGuiWindowFlags win_flags =
  //     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
  //     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing
  //     | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground;
  //
  // static double data1[static_cast<size_t>(PowerTerm::COUNT)];
  // for (size_t i = 0; i < static_cast<size_t>(PowerTerm::COUNT); ++i) {
  //   data1[i] = static_cast<float>(rs.power_breakdown[i]);
  // }
  // // clamp so we don't show negative inertia term
  // // thought ideally we'd only show the output of the other terms
  // // but that'd require subtracting the change from before or sth similar?
  // data1[static_cast<size_t>(PowerTerm::Inertia)] =
  //     std::max(0.0, data1[static_cast<size_t>(PowerTerm::Inertia)]);
  // static ImPlotPieChartFlags flags = 0;
  // bool open = ImGui::Begin("##riderplot", nullptr, win_flags);
  //
  // // DragFloat controls
  // // ImGui::SetNextItemWidth(250);
  // // for (size_t i = 0; i < static_cast<size_t>(PowerTerm::COUNT); ++i) {
  // //   char label[32];
  // //   snprintf(label, sizeof(label), POWER_LABELS[i], i);
  // //   ImGui::DragFloat(label, &data1[i], 0.01f, 0, 1);
  // // }
  //
  // if (open) {
  //   if (ImGui::Button(show_plot ? "Hide plot" : "Show plot")) {
  //     show_plot = !show_plot;
  //   }
  //
  //   if (show_plot) {
  //     if (ImPlot::BeginPlot("##Pie1",
  //                           ImVec2(ImGui::GetTextLineHeight() * 16,
  //                                  ImGui::GetTextLineHeight() * 16),
  //                           ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
  //       ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations,
  //                         ImPlotAxisFlags_NoDecorations);
  //       ImPlot::SetupAxesLimits(0, 1, 0, 1);
  //       ImPlot::PlotPieChart(POWER_LABELS, data1,
  //                            static_cast<int>(PowerTerm::COUNT), 0.5, 0.5,
  //                            0.4,
  //                            "%.2f", 90, flags);
  //       ImPlot::EndPlot();
  //     }
  //   }
  // }
  //
  // ImGui::End();
}

bool BaseEditableField::handle_event(const SDL_Event* e) {
  switch (e->type) {

  // ================================
  // CLICK EVENTS
  // ================================
  case SDL_EVENT_MOUSE_BUTTON_DOWN: {
    int mx = e->button.x;
    int my = e->button.y;
    bool inside = (mx >= x && mx <= x + w && my >= y && my <= y + h);

    if (inside) {
      editing = true;
      buffer = current_text;
      SDL_StartTextInput(window);
      last_cursor_blink = SDL_GetTicks();
      cursor_visible = true;
      return true;
    } else if (editing) {
      commit();
      return true;
    }
    break;
  }

  // ================================
  // KEY EVENTS
  // ================================
  case SDL_EVENT_KEY_DOWN:
    if (!editing)
      break;

    if (e->key.key == SDLK_BACKSPACE) {
      if (!buffer.empty())
        buffer.pop_back();
      return true;
    }

    if (e->key.key == SDLK_RETURN || e->key.key == SDLK_KP_ENTER) {
      commit();
      return true;
    }

    break;

  // ================================
  // TEXT INPUT (UTF-8)
  // ================================
  case SDL_EVENT_TEXT_INPUT:
    if (!editing)
      break;

    for (char c : std::string(e->text.text)) {
      if (accept_char(c, buffer))
        buffer.push_back(c);
      SDL_Log("%s", buffer.c_str());
    }
    return true;
  }

  return false;
}

void BaseEditableField::render(const RenderContext* ctx) {
  if (editing) {
    Uint32 now = SDL_GetTicks();
    if (now - last_cursor_blink >= BLINK_MS) {
      cursor_visible = !cursor_visible;
      last_cursor_blink = now;
    }
  }

  SDL_Renderer* renderer = ctx->renderer;

  // if (!bg_texture)
  //   create_bg(renderer);

  std::string saved = current_text;
  ValueField::set_text(current_display_text());

  ValueField::render(ctx);

  current_text = saved;
  //
  // // regenerate texture every frame while editing
  // if (texture) {
  //   SDL_DestroyTexture(texture);
  //   texture = nullptr;
  // }
  // if (!displayed.empty()) {
  //   SDL_Surface* surf =
  //       TTF_RenderText_Blended(font, displayed.c_str(), 0, text_color);
  //   texture = SDL_CreateTextureFromSurface(renderer, surf);
  //   SDL_DestroySurface(surf);
  // }
  //
  // SDL_FRect rect{(float)x, (float)y, (float)w, (float)h};
  // SDL_RenderTexture(renderer, bg_texture, nullptr, &rect);
  //
  // if (texture) {
  //   float tw, th;
  //   SDL_GetTextureSize(texture, &tw, &th);
  //   SDL_FRect dst{(float)x, (float)y, tw, th};
  //   SDL_RenderTexture(renderer, texture, nullptr, &dst);
  // }
}

void BaseEditableField::commit() {
  editing = false;
  SDL_StopTextInput(window);
  set_text(buffer);
  if (on_commit)
    on_commit(buffer);
}

std::string BaseEditableField::current_display_text() const {
  if (!editing)
    return current_text;

  if (cursor_visible)
    return buffer + "|";

  return buffer;
}

bool EditableNumberField::accept_char(char c, const std::string& before) {
  if (c >= '0' && c <= '9')
    return true;
  if (c == '.' && before.find('.') == std::string::npos)
    return true;
  if (c == '-' && before.empty())
    return true;
  return false;
}
