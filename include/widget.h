// widget.h
#ifndef WIDGET_H
#define WIDGET_H

#include "display.h"
#include "sim.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

std::string format_number(double value, int precision = 1);

class Widget : public Drawable {
public:
  RenderLayer layer() const override { return RenderLayer::UI; }
  virtual void render(const RenderContext* ctx) override = 0;
  virtual ~Widget() = default;
};

class Stopwatch : public Widget {
private:
  Simulation* sim;
  TTF_Font* font;
  SDL_Color text_color = SDL_Color{80, 255, 40, 255};
  int screen_x, screen_y;
  int width = 0, height = 0; // Texture dimensions
  int bg_width = 0, bg_height = 0;
  int update_interval_ms;

  SDL_Texture* texture = nullptr;
  SDL_Texture* bg_texture = nullptr;

  int padding = 6;
  int edge_thickness = 3;
  int content_offset = padding + edge_thickness;

  uint32_t last_update_ticks = 0;

  SDL_Texture* render_time(SDL_Renderer* renderer, const char* s, int& out_w,
                           int& out_h);
  SDL_Texture* create_base(SDL_Renderer* renderer);

public:
  Stopwatch(int x, int y, TTF_Font* font_, Simulation* sim_,
            int update_ms = 100)
      : screen_x(x), screen_y(y), font(font_), sim(sim_),
        update_interval_ms(update_ms) {
    last_update_ticks = 0;
    SDL_Log("test");
  }

  ~Stopwatch() {
    if (texture)
      SDL_DestroyTexture(texture);
    if (bg_texture)
      SDL_DestroyTexture(bg_texture);
  }

  void update_texture(const RenderContext* ctx);
  void render(const RenderContext* ctx) override;
};

class ValueField {
public:
  using DataGetter = std::function<std::string(const RiderSnapshot&)>;

private:
  TTF_Font* font;
  SDL_Color text_color = {255, 255, 255, 255};
  int x, y, width, height;

  // Logic
  size_t target_rider_id;
  DataGetter getter;

  // Caching
  std::string current_text;
  SDL_Texture* texture = nullptr;
  SDL_Texture* bg_texture = nullptr;
  uint32_t last_update = 0;

  void update_texture(SDL_Renderer* renderer, const RiderSnapshot& snap);
  void create_bg(SDL_Renderer* renderer);

public:
  ValueField(int x, int y, int w, int h, TTF_Font* font, size_t id,
             DataGetter getter);
  ~ValueField();

  // void render(const RenderContext* ctx) override;
  void render_with_snapshot(const RenderContext* ctx,
                            const RiderSnapshot* snap);

  // Allow parents to move this widget
  void set_position(int new_x, int new_y) {
    x = new_x;
    y = new_y;
  }
  int get_width() const { return width; }
};

class MetricRow {
private:
  std::string label_txt;
  std::string unit_txt;
  TTF_Font* font;

  // Components
  std::unique_ptr<ValueField> field;
  SDL_Texture* label_tex = nullptr;
  SDL_Texture* unit_tex = nullptr;

  int x, y;
  int label_width = 80; // Fixed width so boxes align vertically
  int padding = 10;

public:
  MetricRow(int x, int y, TTF_Font* font, size_t id, std::string label,
            std::string unit, ValueField::DataGetter getter);

  ~MetricRow();

  // void render(const RenderContext* ctx) override;
  void render_for_rider(const RenderContext* ctx, const RiderSnapshot* snap);

  // Helper to calculate total height for the panel
  int get_height() const { return 30; } // simplified
  void set_position(int new_x, int new_y);
};

class RiderPanel : public Widget {
private:
  int x, y;
  int rider_uid = 0;
  TTF_Font* font;

  std::string title;
  SDL_Texture* title_tex = nullptr;

  // All the rows
  std::vector<std::unique_ptr<MetricRow>> rows;

public:
  RiderPanel(int x, int y, TTF_Font* font);
  ~RiderPanel();

  void set_rider_id(int uid);
  // Usage: panel->add_row("Speed", "km/h", [](auto s){ return ... });
  void add_row(std::string label, std::string unit,
               ValueField::DataGetter getter);

  void render(const RenderContext* ctx) override;
};

#endif
