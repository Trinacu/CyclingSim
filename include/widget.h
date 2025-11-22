// widget.h
#ifndef WIDGET_H
#define WIDGET_H

#include "display.h"
#include "rider.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

std::string format_number(double value, int precision);

// std::string format_number_fixed(double value, int max_digits,
//                                 int precision = 2) {
//   char buffer[32];
//   std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
//
//   int digit_count = 0;
//   int end = 0;
//   for (int i = 0; buffer[i] != '\0'; ++i) {
//     if (std::isdigit(buffer[i])) {
//       ++digit_count;
//     }
//     if (digit_count > max_digits)
//       break;
//     ++end;
//   }
//
//   // Trim to the end index
//   std::string result(buffer, end);
//
//   // Suppress trailing decimal point
//   if (!result.empty() && result.back() == '.')
//     result.pop_back();
//
//   return result;
// }

class Widget : public Drawable {
public:
  ~Widget() {
    free_texture();
    if (base_texture)
      SDL_DestroyTexture(base_texture);
  }
  RenderLayer layer() const override { return RenderLayer::UI; }
  virtual void
  render(const RenderContext* ctx) override = 0; // Still pure virtual
  std::pair<int, int> get_texture_size() const;

protected:
  int tex_w = 0, tex_h = 0;
  int widget_w = 0, widget_h = 0;
  const int padding = 4;
  const int edge_thickness = 2;
  const int content_offset = edge_thickness + padding;

  bool self_update = true;
  SDL_Texture* texture = nullptr;
  SDL_Texture* base_texture = nullptr;

  virtual void update_texture(const RenderContext* ctx) {};
  virtual void free_texture() {
    if (texture) {
      SDL_DestroyTexture(texture);
      texture = nullptr;
    }
  }
  // makes sure deletion happens at concrete class
  virtual SDL_Texture* create_base(SDL_Renderer* renderer) { return nullptr; };
};

class Stopwatch : public Widget {
private:
  Simulation* sim;
  TTF_Font* font;
  SDL_Color text_color = SDL_Color{80, 255, 40, 255};
  int screen_x, screen_y;
  int update_interval_ms;

  int padding = 6;
  int edge_thickness = 3;
  int content_offset = padding + edge_thickness;

  uint32_t last_update_ticks = 0;

  SDL_Texture* render_time(SDL_Renderer* renderer, const char* s, int& out_w,
                           int& out_h);
  SDL_Texture* create_base(SDL_Renderer* renderer) override;

public:
  Stopwatch(int x, int y, TTF_Font* font_, Simulation* sim_,
            int update_ms = 100)
      : sim(sim_), font(font_), screen_x(x), screen_y(y),
        update_interval_ms(update_ms) {
    last_update_ticks = 0;
  }

  // ~Stopwatch() { free_texture(); };

  void update_texture(const RenderContext* ctx) override;
  void render(const RenderContext* ctx) override;
};

class ValueField : public Widget {
private:
  bool is_numeric;
  int char_count;
  TTF_Font* font;
  SDL_Color text_color = SDL_Color{200, 200, 200, 255};
  int screen_x, screen_y;
  int update_interval_ms = 500;
  uint32_t last_update_ticks = 0;
  bool right_align;

  bool self_update = true;

  using StringGetter = std::function<std::string(const RiderSnapshot&)>;
  StringGetter text_getter;
  size_t target_id;

  std::function<std::string()> value_callback;
  // // takes snapshot, returns value to show
  // std::function<double(const RiderSnapshot)> numeric_getter;
  // void bind_object(const size_t id,
  //                  std::function<double(const RiderSnapshot)> getter = 0) {
  //   bound_id = id;
  //   if (getter) {
  //     numeric_getter = std::move(getter);
  //   }
  // }

  std::string formatter(double val) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    return std::string(buf);
  }

  SDL_Texture* render_text(SDL_Renderer* renderer, const char* s, int& out_w,
                           int& out_h);
  SDL_Texture* create_base(SDL_Renderer* renderer) override;

public:
  ValueField(int x_, int y_, int char_count_, TTF_Font* font_, size_t id,
             StringGetter getter)
      : screen_x(x_), screen_y(y_), char_count(char_count_), font(font_),
        target_id(id), text_getter(getter) {}

  void update_texture(const RenderContext* ctx) override;
  void render(const RenderContext* ctx) override;
};

class ValueFieldRow : public Widget {
private:
  std::string label_text;
  TTF_Font* font;
  SDL_Color label_color{200, 200, 200, 255};
  SDL_Texture* label_texture = nullptr;
  int label_w = 0, label_h = 0;

  std::unique_ptr<ValueField> value_field;

  int x, y;           // top-left anchor
  int spacing_px = 2; // gap between label and value

public:
  ValueFieldRow(int x_, int y_, const std::string& label, TTF_Font* font_,
                std::unique_ptr<ValueField> vf, int spacing = 10)
      : x(x_), y(y_), label_text(label), font(font_),
        value_field(std::move(vf)), spacing_px(spacing) {}

  ~ValueFieldRow() {
    if (label_texture) {
      SDL_DestroyTexture(label_texture);
    }
  }

  void update_texture(const RenderContext* ctx) override {
    if (!label_texture) {
      SDL_Surface* surf =
          TTF_RenderText_Blended(font, label_text.c_str(), 0, label_color);
      label_texture = SDL_CreateTextureFromSurface(ctx->renderer, surf);
      label_w = surf->w;
      label_h = surf->h;
      SDL_DestroySurface(surf);
    }
    value_field->update_texture(ctx);
  }

  void render(const RenderContext* ctx) override {
    std::pair<int, int> size = value_field->get_texture_size();
    float label_x = x + (float)size.first + spacing_px;
    float label_y = y + ((float)size.second - label_texture->h) / 2;
    SDL_FRect dst{label_x, label_y, (float)label_w, (float)label_h};
    SDL_RenderTexture(ctx->renderer, label_texture, nullptr, &dst);

    // Position value to the right
    // value_field->set_position(x + label_w + spacing_px, y);
    value_field->render(ctx);
  }

  void set_position(int x_, int y_) {
    x = x_;
    y = y_;
  }

  ValueField* get_value_field() { return value_field.get(); }
};

class ValueFieldPanel : public Widget {
  // TODO - add a set_position() or something
  // but make sure it also moves the children (ValueFieldRow)
private:
  TTF_Font* font;
  SDL_Color text_color = SDL_Color{200, 200, 200, 255};
  int screen_x, screen_y;
  Rider* rider = nullptr;
  std::string name;
  Simulation* sim;

  int name_w;
  int name_h;

  SDL_Texture* name_texture = nullptr;

  int spacing = 20;
  int step_y;

  std::vector<std::unique_ptr<ValueField>> fields;
  std::vector<std::unique_ptr<ValueFieldRow>> rows;

  uint32_t last_update_ticks = 0;
  int update_interval_ms = 100;

  SDL_Texture* create_base(SDL_Renderer* renderer) override;
  void update_texture(const RenderContext* ctx) override;

  template <typename T>
  void add_field(int row, const void* id, T RiderSnapshot::* member_ptr);

  void add_row(std::unique_ptr<ValueFieldRow> row);
  template <typename... FieldArgs>
  ValueFieldRow* emplace_row(std::string label, FieldArgs&&... vf_args);

public:
  ValueFieldPanel(int x, int y, TTF_Font* font_, Rider* rider_);
  void render(const RenderContext* ctx) override;
};

#endif
