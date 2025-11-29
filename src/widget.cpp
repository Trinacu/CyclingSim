#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <cstdio>
#include <string>

#include "SDL3/SDL_render.h"
#include "SDL3/SDL_surface.h"
#include "display.h"
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

ValueField::ValueField(int x_, int y_, int w, int h, TTF_Font* f, size_t id,
                       DataGetter g)
    : x(x_), y(y_), width(w), height(h), font(f), target_rider_id(id),
      getter(g) {}

ValueField::~ValueField() {
  if (texture)
    SDL_DestroyTexture(texture);
  if (bg_texture)
    SDL_DestroyTexture(bg_texture);
}

void ValueField::create_bg(SDL_Renderer* renderer) {
  // Create a dark gray box background
  SDL_Surface* surf =
      SDL_CreateSurface(width, height, SDL_PIXELFORMAT_RGBA8888);
  SDL_FillSurfaceRect(surf, nullptr,
                      SDL_MapRGBA(SDL_GetPixelFormatDetails(surf->format),
                                  nullptr, 50, 50, 50, 255));
  bg_texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
}

void ValueField::update_texture(SDL_Renderer* renderer,
                                const RiderSnapshot& snap) {
  // 1. Get string from lambda
  std::string new_text = getter(snap);

  // 2. Only re-render if text changed (Optimization)
  if (new_text == current_text && texture)
    return;

  current_text = new_text;
  if (texture)
    SDL_DestroyTexture(texture);

  // 3. Render Text
  SDL_Surface* surf =
      TTF_RenderText_Blended(font, current_text.c_str(), 0, text_color);
  if (surf) {
    texture = SDL_CreateTextureFromSurface(renderer, surf);
    SDL_DestroySurface(surf);
  }
}

void ValueField::render_with_snapshot(const RenderContext* ctx,
                                      const RiderSnapshot* snap) {
  if (!snap)
    return;

  if (!bg_texture)
    create_bg(ctx->renderer);
  update_texture(ctx->renderer, *snap);
  // 3. Draw Background
  SDL_FRect bg_rect = {(float)x, (float)y, (float)width, (float)height};
  SDL_RenderTexture(ctx->renderer, bg_texture, nullptr, &bg_rect);

  // 4. Draw Text (Centered or Right Aligned)
  if (texture) {
    float tex_w, tex_h;
    SDL_GetTextureSize(texture, &tex_w, &tex_h);
    // Align right inside the box with padding
    float txt_x = x + width - tex_w - 5;
    float txt_y = y + (height - tex_h) / 2;
    SDL_FRect txt_rect = {txt_x, txt_y, tex_w, tex_h};
    SDL_RenderTexture(ctx->renderer, texture, nullptr, &txt_rect);
  }
}

// ======================= METRIC ROW =======================

MetricRow::MetricRow(int x_, int y_, TTF_Font* f, size_t id, std::string label,
                     std::string unit, ValueField::DataGetter getter)
    : x(x_), y(y_), font(f), label_txt(label), unit_txt(unit) {

  // Create the child ValueField (Width 80, Height 24)
  // We position it relative to our X + label_width
  field = std::make_unique<ValueField>(x + label_width, y, 80, 24, font, id,
                                       getter);
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
                         ValueField::DataGetter getter) {
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
  // ID? Proper way: Refactor ValueField::render to take a snapshot, not look it
  // up itself.
  // Draw all rows
  for (auto& row : rows) {
    row->render_for_rider(ctx, snap);
  }
}
