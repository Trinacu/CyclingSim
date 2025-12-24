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
#include "snapshot.h"
#include "widget.h"
// for std::setprecision
// #include <iomanip>

std::string format_number(double value, int precision) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std ::string(buffer);
}

void format_time(double seconds, char* text) {
  // Break down into components
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

// std::pair<int, int> Widget::get_texture_size() const {
//   return {widget_w, widget_h};
// }

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
  double sim_time = sim->get_sim_seconds();
  format_time(sim_time, text);

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

Button::Button(int x_, int y_, int w_, int h_, std::string label_,
               TTF_Font* font_, Callback cb)
    : x(x_), y(y_), w(w_), h(h_), label(std::move(label_)), font(font_),
      on_click(std::move(cb)) {}

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

  // Render text
  SDL_Surface* surf = TTF_RenderText_Blended(font, label.c_str(), 0,
                                             SDL_Color{255, 255, 255, 255});
  if (!surf)
    return;

  SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
  if (!tex) {
    SDL_DestroySurface(surf);
    return;
  }

  float tx = x + (w - surf->w) * 0.5f;
  float ty = y + (h - surf->h) * 0.5f;
  SDL_FRect dst{tx, ty, (float)surf->w, (float)surf->h};
  SDL_RenderTexture(r, tex, nullptr, &dst);

  SDL_DestroyTexture(tex);
  SDL_DestroySurface(surf);
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
    : Button(x, y, w, h, sim->is_paused() ? "Resume" : "Pause", font, [sim]() {
        if (sim->is_paused())
          sim->resume();
        else
          sim->pause();
      }) {}

void PauseButton::render(const RenderContext* ctx) {
  // Update label before drawing
  // set_label(sim->is_paused() ? "Resume" : "Pause");

  Button::render(ctx);
}

double TimeFactorSlider::slider_to_factor(double t) {
  if (t <= neutral_point) {
    double s = t / neutral_point;
    return 0.1 + s * (1.0 - 0.1);
  } else {
    double s = (t - neutral_point) / neutral_point;
    return std::pow(10.0, s * 2.0); // 1 → 100
  }
}

double TimeFactorSlider::factor_to_slider(double f) {
  if (f <= 1.0) {
    double s = (f - 0.1) / (1.0 - 0.1);
    return s * neutral_point;
  } else {
    double s = std::log10(f) / 2.0;
    return neutral_point + s * neutral_point;
  }
}

void TimeFactorSlider::render(const RenderContext* ctx) {
  SDL_Renderer* r = ctx->renderer;

  // background bar
  SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
  SDL_FRect bar{(float)x, (float)y + h * 0.4f, (float)w, h * 0.2f};
  SDL_RenderFillRect(r, &bar);

  // neutral point - factor = 1.0
  float markerX = x + neutral_point * w - marker_width / 2.0;
  SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
  SDL_FRect marker{markerX, (float)y, static_cast<float>(marker_width),
                   (float)h};
  SDL_RenderFillRect(r, &marker);

  // knob
  double t = factor_to_slider(sim->get_time_factor());
  float knobX = x + t * w - h / 4.0;
  SDL_SetRenderDrawColor(r, 240, 240, 240, 255);
  SDL_FRect knob{knobX, static_cast<float>(y + h / 4.0), (float)h / 2,
                 (float)h / 2};
  SDL_RenderFillRect(r, &knob);
}

bool TimeFactorSlider::handle_event(const SDL_Event* e) {
  switch (e->type) {
  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (e->button.x >= x && e->button.x <= x + w && e->button.y >= y &&
        e->button.y <= y + h) {
      dragging = true;
      return true;
    }
    break;

  case SDL_EVENT_MOUSE_BUTTON_UP:
    dragging = false;
    break;

  case SDL_EVENT_MOUSE_MOTION:
    if (dragging) {
      double local = (e->motion.x - x) / (double)w;
      local = std::clamp(local, 0.0, 1.0);
      double f = slider_to_factor(local);
      sim->set_time_factor(f);
      return true;
    }
    break;
  }
  return false;
}

TimeControlPanel::TimeControlPanel(int x_, int y_, int h_, TTF_Font* font,
                                   Simulation* sim_)
    : x(x_), y(y_), h(h_), sim(sim_) {
  auto tf =
      std::make_unique<ValueField>(next_x, y + 2, valfield_w, h - 4, font);
  time_factor_field = tf.get();
  children.push_back(std::move(tf));
  next_x += valfield_w + gap_w;

  int widget_h = h_ / 2;
  int widget_y = y + h_ / 4;

  children.push_back(std::make_unique<TimeFactorSlider>(
      next_x, widget_y, slider_w, widget_h, sim));
  next_x += slider_w + gap_w;

  auto make_factor_button = [&](double val) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%.1f", val);
    std::string str = buffer; // "3.1"
    // children.push_back(std::make_unique<Button>(
    //     next_x, y + h_ / 4, button_w, h_ / 2, str + " ×", font,
    //     [sim_, val]() { sim_->set_time_factor(val); }));
    children.push_back(std::make_unique<TimeFactorButton>(
        next_x, widget_y, button_w, widget_h, val, sim_, font));
    next_x += gap_w + button_w;
  };

  make_factor_button(0.5);
  make_factor_button(1.0);
  make_factor_button(20.0);

  next_x += gap_w + button_w;
  children.push_back(std::make_unique<PauseButton>(next_x, widget_y, button_w,
                                                   widget_h, sim, font));
}

void TimeControlPanel::render(const RenderContext* ctx) {
  SDL_Renderer* r = ctx->renderer;

  // (Optional) panel background
  SDL_SetRenderDrawColor(r, 40, 40, 40, 200);
  SDL_FRect bg{
      (float)x, (float)y,
      static_cast<float>(valfield_w + slider_w + 3 * button_w + 6 * gap_w),
      static_cast<float>(h)};
  SDL_RenderFillRect(r, &bg);

  char buf[8];
  snprintf(buf, sizeof(buf), "%.2f", sim->get_time_factor());
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
                                 size_t rider_id, DataGetter getter_)
    : ValueField(x, y, w, h, font), target_rider_id(rider_id), getter(getter_) {
}

void RiderValueField::render_with_snapshot(const RenderContext* ctx,
                                           const RiderSnapshot* snap) {
  if (!snap)
    return;

  std::string new_text = getter(*snap);
  set_text(new_text);

  // Use base render method
  render(ctx);
}

// ======================= METRIC ROW =======================

MetricRow::MetricRow(int x_, int y_, TTF_Font* f, size_t id, std::string label,
                     std::string unit, RiderValueField::DataGetter getter)
    : x(x_), y(y_), font(f), label_txt(label), unit_txt(unit) {

  // Create the child ValueField (Width 80, Height 24)
  // We position it relative to our X + label_width
  field = std::make_unique<RiderValueField>(x + label_width, y, 80, 24, font,
                                            id, getter);
}

MetricRow::~MetricRow() {
  if (label_tex)
    SDL_DestroyTexture(label_tex);
  if (unit_tex)
    SDL_DestroyTexture(unit_tex);
}

void MetricRow::set_position(int new_x, int new_y) {
  x = new_x;
  y = new_y;
  // Update child position
  field->set_position(x + label_width, y);
}

void MetricRow::render_for_rider(const RenderContext* ctx,
                                 const RiderSnapshot* snap) {
  // 1. Render static labels once
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

  // 2. Draw Label (Left)
  if (label_tex) {
    float w, h;
    SDL_GetTextureSize(label_tex, &w, &h);
    // Vertically center label relative to the row height (approx 24)
    SDL_FRect r = {(float)x, (float)y + (24 - h) / 2, w, h};
    SDL_RenderTexture(ctx->renderer, label_tex, nullptr, &r);
  }

  // Call the specific render on the field
  field->render_with_snapshot(ctx, snap);

  if (unit_tex) {
    float w, h;
    SDL_GetTextureSize(unit_tex, &w, &h);
    // Position: X + label + field_width + padding
    SDL_FRect r = {(float)(x + label_width + field->get_width() + 5),
                   (float)y + (24 - h) / 2, w, h};
    SDL_RenderTexture(ctx->renderer, unit_tex, nullptr, &r);
  }
}

// ======================= RIDER PANEL =======================
// WARNING - this assumes the font isnt deallocated
RiderPanel::RiderPanel(int x_, int y_, TTF_Font* f) : x(x_), y(y_), font(f) {}

void RiderPanel::set_rider_id(int uid) { rider_uid = uid; }

RiderPanel::~RiderPanel() {
  if (title_tex)
    SDL_DestroyTexture(title_tex);
}

void RiderPanel::add_row(std::string label, std::string unit,
                         RiderValueField::DataGetter getter) {
  int row_height = 30;                                // height + spacing
  int current_offset = rows.size() * row_height + 30; // +30 for title space

  // pass 0 as dummy, we overrider it
  // Ideally, MetricRow should also be refactored, but here is a
  // quick inheritance trick:

  // Better yet, let's make MetricRow dynamic too.
  // But to save refactoring EVERYTHING, let's use a "Dynamic ID" constant?
  // No, cleaner to just pass the ID in render.
  auto row = std::make_unique<MetricRow>(x, y + current_offset, font, 0, label,
                                         unit, getter);
  rows.push_back(std::move(row));
}

void RiderPanel::render(const RenderContext* ctx) {

  const RiderSnapshot* snap = ctx->get_snapshot(rider_uid);
  if (!snap)
    return;

  std::string title_text = snap->name;

  if (title != title_text) {
    title = title_text;
    if (title_tex)
      SDL_DestroyTexture(title_tex);
    SDL_Surface* s =
        TTF_RenderText_Blended(font, title.c_str(), 0, {255, 255, 255});
    title_tex = SDL_CreateTextureFromSurface(ctx->renderer, s);
    SDL_DestroySurface(s);
  }
  // Draw Title
  if (!title_tex) {
    SDL_Surface* s = TTF_RenderText_Blended(font, title.c_str(), 0,
                                            {255, 255, 0, 255}); // Yellow title
    title_tex = SDL_CreateTextureFromSurface(ctx->renderer, s);
    SDL_DestroySurface(s);
  }

  float w, h;
  SDL_GetTextureSize(title_tex, &w, &h);
  SDL_FRect r = {(float)x, (float)y, w, h};
  SDL_RenderTexture(ctx->renderer, title_tex, nullptr, &r);

  // HACK for now: Iterate rows, find their internal fields, and update their
  // ID? Proper way: Refactor ValueField::render to take a snapshot, not look
  // it up itself. Draw all rows
  for (auto& row : rows) {
    row->render_for_rider(ctx, snap);
  }
}

void RiderPanel::render_imgui(const RenderContext* ctx) {
  if (!visible || rider_uid == -1)
    return;

  const RiderSnapshot* snap = ctx->get_snapshot(rider_uid);

  // compute where the plot goes based on panel position
  ImVec2 pos((float)x, (float)y + 140);
  ImVec2 size(300, 300);

  ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(size, ImGuiCond_Always);

  ImGuiWindowFlags win_flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground;

  static double data1[static_cast<size_t>(PowerTerm::COUNT)];
  for (size_t i = 0; i < static_cast<size_t>(PowerTerm::COUNT); ++i) {
    data1[i] = static_cast<float>(snap->power_breakdown[i]);
  }
  // clamp so we don't show negative inertia term
  // thought ideally we'd only show the output of the other terms
  // but that'd require subtracting the change from before or sth similar?
  data1[static_cast<size_t>(PowerTerm::Inertia)] =
      std::max(0.0, data1[static_cast<size_t>(PowerTerm::Inertia)]);
  static ImPlotPieChartFlags flags = 0;
  bool open = ImGui::Begin("##riderplot", nullptr, win_flags);

  // DragFloat controls
  // ImGui::SetNextItemWidth(250);
  // for (size_t i = 0; i < static_cast<size_t>(PowerTerm::COUNT); ++i) {
  //   char label[32];
  //   snprintf(label, sizeof(label), POWER_LABELS[i], i);
  //   ImGui::DragFloat(label, &data1[i], 0.01f, 0, 1);
  // }

  if (open) {
    if (ImGui::Button(show_plot ? "Hide plot" : "Show plot")) {
      show_plot = !show_plot;
    }

    if (show_plot) {
      if (ImPlot::BeginPlot("##Pie1",
                            ImVec2(ImGui::GetTextLineHeight() * 16,
                                   ImGui::GetTextLineHeight() * 16),
                            ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations,
                          ImPlotAxisFlags_NoDecorations);
        ImPlot::SetupAxesLimits(0, 1, 0, 1);
        ImPlot::PlotPieChart(POWER_LABELS, data1,
                             static_cast<int>(PowerTerm::COUNT), 0.5, 0.5, 0.4,
                             "%.2f", 90, flags);
        ImPlot::EndPlot();
      }
    }
  }

  ImGui::End();
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
