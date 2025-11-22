#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <cstdio>
#include <string>

#include "display.h"
#include "rider.h"
#include "widget.h"
// for std::setprecision
// #include <iomanip>

std::string format_number(double value, int precision = 1) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std ::string(buffer);
}

char* format_time(double seconds) {
  int len = 11;
  char* buffer = (char*)malloc(len * sizeof(char));
  if (!buffer)
    return nullptr;

  // Break down into components
  int totalTenths = static_cast<int>(round(seconds * 10));
  int tenths = totalTenths % 10;
  int totalSeconds = totalTenths / 10;
  int secs = totalSeconds % 60;
  int totalMinutes = totalSeconds / 60;
  int mins = totalMinutes % 60;
  int hours = totalMinutes / 60;

  // Format with leading zeros and fixed positions
  snprintf(buffer, len, "%02d:%02d:%02d.%d", hours, mins, secs, tenths);

  return buffer;
}

std::pair<int, int> Widget::get_texture_size() const {
  return {widget_w, widget_h};
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

  // Create a texture from that surface
  SDL_Texture* new_texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);

  if (!new_texture) {
    SDL_Log("SDL_CreateTextureFromSurface failed: %s", SDL_GetError());
    return nullptr;
  }
  return new_texture;
}

void Stopwatch::update_texture(const RenderContext* ctx) {
  double sim_time = sim->get_sim_seconds(); // e.g. 12.3456
  char* buffer = format_time(sim_time);
  // Free old texture first:
  free_texture();
  texture = render_time(ctx->renderer, buffer, tex_w, tex_h);
  free(buffer);
  if (!texture) {
    return; // failed to build; abort drawing this frame
  }
}

void Stopwatch::render(const RenderContext* ctx) {
  // 1) Check whether it's time to rebuild the texture:
  if (base_texture == nullptr) {
    base_texture = create_base(ctx->renderer);
  }
  Uint32 now_ticks = SDL_GetTicks();
  if (texture == nullptr || (now_ticks - last_update_ticks) >=
                                static_cast<Uint32>(update_interval_ms)) {
    update_texture(ctx);
    last_update_ticks = now_ticks;
  }

  // 2) Draw the texture at (screen_x, screen_y)
  if (texture) {
    SDL_FRect dst{static_cast<float>(screen_x), static_cast<float>(screen_y),
                  static_cast<float>(widget_w), static_cast<float>(widget_h)};
    SDL_RenderTexture(ctx->renderer, base_texture, nullptr, &dst);

    dst = {static_cast<float>(screen_x + content_offset),
           static_cast<float>(screen_y + content_offset),
           static_cast<float>(tex_w), static_cast<float>(tex_h)};
    SDL_RenderTexture(ctx->renderer, texture, nullptr, &dst);
  }
}

SDL_Texture* Stopwatch::create_base(SDL_Renderer* renderer) {
  int w, h;
  TTF_GetStringSize(font, "00:00:00.0", 0, &w, &h);
  std::cout << w << " " << h << std::endl;
  int padded_w = w + 2 * content_offset;
  int padded_h = h + 2 * content_offset;
  std::cout << padded_w << " " << padded_h << std::endl;

  SDL_Surface* surf =
      SDL_CreateSurface(padded_w, padded_h, SDL_PIXELFORMAT_XRGB8888);
  if (!surf) {
    SDL_Log("SDL_CreateRGBSurfaceWithFormat failed: %s", SDL_GetError());
    return nullptr;
  }
  widget_w = surf->w;
  widget_h = surf->h;

  const SDL_PixelFormatDetails* fmt_details =
      SDL_GetPixelFormatDetails(surf->format);
  Uint32 transparent = SDL_MapRGBA(fmt_details, NULL, 0, 0, 0, 0);
  Uint32 hlPix = SDL_MapRGBA(fmt_details, NULL, 177, 177, 177, 255);
  Uint32 shPix = SDL_MapRGBA(fmt_details, NULL, 77, 77, 77, 255);
  const SDL_Rect* r = new SDL_Rect{0, 0, padded_w, padded_h};
  SDL_FillSurfaceRect(surf, r, transparent);

  SDL_Rect topEdge = {0, 0, padded_w, edge_thickness};
  SDL_FillSurfaceRect(surf, &topEdge, hlPix);

  SDL_Rect leftEdge = {0, 0, edge_thickness, padded_h};
  SDL_FillSurfaceRect(surf, &leftEdge, hlPix);

  // 5) Draw the bottom & right edges with shadow
  SDL_Rect bottomEdge = {0, padded_h - edge_thickness, padded_w,
                         edge_thickness};
  SDL_FillSurfaceRect(surf, &bottomEdge, shPix);

  SDL_Rect rightEdge = {padded_w - edge_thickness, 0, edge_thickness, padded_h};
  SDL_FillSurfaceRect(surf, &rightEdge, shPix);

  SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
  if (!tex) {
    SDL_Log("SDL_CreateTextureFromSurface failed: %s", SDL_GetError());
    SDL_DestroySurface(surf);
    return nullptr;
  }

  std::cout << tex->w << " " << tex->h << std::endl;

  // 10) Clean up the surface; we only need the texture from now on
  SDL_DestroySurface(surf);

  return tex;
}

void ValueField::update_texture(const RenderContext* ctx) {
  free_texture();
  const RiderSnapshot* snap = ctx->get_snapshot(target_id);
  std::string val = "0.0";
  if (snap) {
    val = text_getter(*snap);
  }

  texture = render_text(ctx->renderer, val.c_str(), tex_w, tex_h);
}

void ValueField::render(const RenderContext* ctx) {
  if (base_texture == nullptr) {
    std::string placeholder(char_count, 'X'); // Placeholder to measure size
    base_texture = create_base(ctx->renderer);
  }

  if (self_update) {
    Uint32 now_ticks = SDL_GetTicks();
    if ((now_ticks - last_update_ticks) >=
        static_cast<Uint32>(update_interval_ms)) {
      update_texture(ctx);
      last_update_ticks = now_ticks;
    }
  }

  if (!texture)
    return;

  SDL_FRect dst{(float)screen_x, (float)screen_y, (float)widget_w,
                (float)widget_h};
  SDL_RenderTexture(ctx->renderer, base_texture, nullptr, &dst);

  int x;
  if (right_align) {
    x = screen_x + widget_w - tex_w - content_offset;
  } else {
    x = screen_x + content_offset;
  }

  dst = {(float)(x), (float)(screen_y + content_offset), (float)tex_w,
         (float)tex_h};
  SDL_RenderTexture(ctx->renderer, texture, nullptr, &dst);
}

SDL_Texture* ValueField::render_text(SDL_Renderer* renderer, const char* s,
                                     int& out_w, int& out_h) {
  SDL_Surface* surf = TTF_RenderText_Blended(font, s, 0, text_color);
  if (!surf) {
    SDL_Log("TTF_RenderText_LCD failed: %s", SDL_GetError());
    return nullptr;
  }
  out_w = surf->w;
  out_h = surf->h;
  SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
  return tex;
}

SDL_Texture* ValueField::create_base(SDL_Renderer* renderer) {
  int w, h;
  TTF_GetStringSize(font, std::string(char_count, 'X').c_str(), 0, &w, &h);
  int padded_w = w + 2 * content_offset;
  int padded_h = h + 2 * content_offset;

  SDL_Surface* surf =
      SDL_CreateSurface(padded_w, padded_h, SDL_PIXELFORMAT_XRGB8888);
  if (!surf) {
    SDL_Log("SDL_CreateSurface failed: %s", SDL_GetError());
    return nullptr;
  }
  widget_w = padded_w;
  widget_h = padded_h;

  const SDL_PixelFormatDetails* fmt_details =
      SDL_GetPixelFormatDetails(surf->format);
  Uint32 bg_color = SDL_MapRGBA(fmt_details, NULL, 0, 0, 0, 0);
  Uint32 edge_color = SDL_MapRGBA(fmt_details, NULL, 177, 177, 177, 255);

  SDL_Rect r = {0, 0, padded_w, padded_h};
  SDL_FillSurfaceRect(surf, &r, edge_color);
  r = SDL_Rect{edge_thickness, edge_thickness, padded_w - 2 * edge_thickness,
               padded_h - 2 * edge_thickness};
  SDL_FillSurfaceRect(surf, &r, bg_color);

  SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);
  return tex;
}

// ValueFieldPanel::ValueFieldPanel(int x, int y, TTF_Font* font_, Rider*
// rider_)
//     : screen_x(x), screen_y(y), font(font_), rider(rider_),
//     name(rider_->name) {
//   step_y = (int)TTF_GetFontSize(font) + spacing;
//   // add_field(0, rider_->get_id(), &RiderSnapshot::km_h);
//   // add_field(1, rider_->get_id(), &RiderSnapshot::pos);
//   // add_field(1, [this]() { return format_number(this->rider->km_h()); });
//   //
//   emplace_row("km", rider_->get_id(), &RiderSnapshot::km);
//   emplace_row("km/h", rider_->get_id(), &RiderSnapshot::km_h);
// }
//
// void ValueFieldPanel::update_texture(const RenderContext* ctx) {
//   for (auto& field : fields) {
//     field->update_texture(ctx);
//   }
//   for (auto& row : rows) {
//     row->update_texture(ctx);
//   }
// }
//
// template <typename T>
// void ValueFieldPanel::add_field(int row, const void* id,
//                                 T RiderSnapshot::* member_ptr) {
//   int widget_y = screen_y + (row + 1) * step_y + padding;
//   fields.emplace_back(std::move(std::make_unique<ValueField>(
//       screen_x + padding, widget_y, 5, font, id, member_ptr)));
// }
//
// void ValueFieldPanel::add_row(std::unique_ptr<ValueFieldRow> row) {
//   rows.emplace_back(std::move(row));
// }
//
// Emplace helper: build row + field together
// template <typename... FieldArgs>
// ValueFieldRow* ValueFieldPanel::emplace_row(std::string label,
//                                             FieldArgs&&... vf_args) {
//   int row_nr = rows.size() + 1;
//   // TODO - rename x, y - there are member variables named same!
//   int x = screen_x + padding;
//   int y = screen_y + (row_nr * step_y);
//   auto vf = std::make_unique<ValueField>(x, y, 5, font,
//                                          std::forward<FieldArgs>(vf_args)...);
//   auto row = std::make_unique<ValueFieldRow>(x, y, label, font,
//   std::move(vf)); ValueFieldRow* raw = row.get();
//   rows.emplace_back(std::move(row));
//   return raw;
// }
//
// SDL_Texture* ValueFieldPanel::create_base(SDL_Renderer* renderer) {
//   int padded_w = 120;
//   int padded_h = 120;
//   widget_w = padded_w;
//   widget_h = padded_h;
//
//   SDL_Surface* surf =
//       SDL_CreateSurface(padded_w, padded_h, SDL_PIXELFORMAT_XRGB8888);
//   if (!surf) {
//     SDL_Log("SDL_CreateSurface failed: %s", SDL_GetError());
//     return nullptr;
//   }
//   const SDL_PixelFormatDetails* fmt_details =
//       SDL_GetPixelFormatDetails(surf->format);
//   Uint32 bg_color = SDL_MapRGBA(fmt_details, NULL, 100, 100, 100, 255);
//   SDL_Rect r = {0, 0, padded_w, padded_h};
//   SDL_FillSurfaceRect(surf, &r, bg_color);
//
//   SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
//   SDL_DestroySurface(surf);
//   return tex;
// }
//
// void ValueFieldPanel::render(const RenderContext* ctx) {
//   if (base_texture == nullptr) {
//     base_texture = create_base(ctx->renderer);
//   }
//
//   if (!name_texture) {
//     SDL_Surface* surf =
//         TTF_RenderText_Blended(font, name.c_str(), 0, text_color);
//     name_texture = SDL_CreateTextureFromSurface(ctx->renderer, surf);
//     name_w = surf->w;
//     name_h = surf->h;
//     SDL_DestroySurface(surf);
//   }
//
//   Uint32 now_ticks = SDL_GetTicks();
//   if ((now_ticks - last_update_ticks) >=
//       static_cast<Uint32>(update_interval_ms)) {
//     const RiderSnapshot* snapshot = ctx->get_snapshot(rider->get_id());
//     update_texture(ctx);
//     last_update_ticks = now_ticks;
//   }
//
//   SDL_FRect dst{(float)screen_x, (float)screen_y, (float)widget_w,
//                 (float)widget_h};
//   SDL_RenderTexture(ctx->renderer, base_texture, nullptr, &dst);
//
//   dst = {(float)screen_x + padding, (float)screen_y + padding, (float)name_w,
//          (float)name_h};
//   SDL_RenderTexture(ctx->renderer, name_texture, nullptr, &dst);
//
//   for (auto& field : fields) {
//     field->render(ctx);
//   }
//   for (auto& row : rows) {
//     row->render(ctx);
//   }
// }
