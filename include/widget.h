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
  char text[16];

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

class Button : public Widget {
public:
  using Callback = std::function<void()>;

private:
  int x, y, w, h;
  std::string label;
  TTF_Font* font;
  Callback on_click;
  bool hovered = false;
  bool pressed = false;

public:
  Button(int x, int y, int w, int h, std::string label, TTF_Font* font,
         Callback cb);

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;
};

class TimeFactorButton : public Widget {
private:
  int x, y, w, h;
  double value;
  Simulation* sim;
  TTF_Font* font;

public:
  TimeFactorButton(int x_, int y_, int h_, double val, Simulation* sim_,
                   TTF_Font* f)
      : x(x_), y(y_), w(50), h(h_), value(val), sim(sim_), font(f) {}

  void render(const RenderContext* ctx) override;

  bool handle_event(const SDL_Event* e) override;
};

class TimeFactorSlider : public Widget {
private:
  int x, y, w, h;
  double neutral_point = 0.3; // where factor = 1.0
  int marker_width = 4;
  bool dragging = false;
  double slider_pos = 0.5; // slider position 0..1
  Simulation* sim;

public:
  TimeFactorSlider(int x_, int y_, int w_, int h_, Simulation* sim_)
      : x(x_), y(y_), w(w_), h(h_), sim(sim_) {}

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  double slider_to_factor(double t);
  double factor_to_slider(double f);
};

class ValueField : public Widget {
protected:
  int x, y, w, h;
  TTF_Font* font;
  SDL_Color text_color = {255, 255, 255, 255};

  // Caching
  std::string current_text;
  SDL_Texture* texture = nullptr;
  SDL_Texture* bg_texture = nullptr;

  void update_texture(SDL_Renderer* renderer);
  void create_bg(SDL_Renderer* renderer);

public:
  ValueField(int x, int y, int w, int h, TTF_Font* font);
  virtual ~ValueField();

  void set_text(const std::string& text);

  // Standard Widget interface
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override { return false; }

  void set_position(int new_x, int new_y) {
    x = new_x;
    y = new_y;
  }
  int get_width() const { return w; }
};

class RiderValueField : public ValueField {
public:
  using DataGetter = std::function<std::string(const RiderSnapshot&)>;

private:
  size_t target_rider_id;
  DataGetter getter;

public:
  RiderValueField(int x, int y, int w, int h, TTF_Font* font, size_t rider_id,
                  DataGetter getter);

  void render_with_snapshot(const RenderContext* ctx,
                            const RiderSnapshot* snap);
};

class TimeControlPanel : public Widget {
private:
  int x, y, h;
  int valfield_w = 50;
  int slider_w = 120;
  int button_w = 50;
  int gap_w = 5;
  int next_x = x + gap_w;
  std::vector<std::unique_ptr<Widget>> children;

  ValueField* time_factor_field = nullptr;

  // for reading current time_factor
  Simulation* sim = nullptr;

public:
  TimeControlPanel(int x_, int y_, int h_, TTF_Font* font, Simulation* sim);

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;
};

class MetricRow {
private:
  std::string label_txt;
  std::string unit_txt;
  TTF_Font* font;

  // Components
  std::unique_ptr<RiderValueField> field;
  SDL_Texture* label_tex = nullptr;
  SDL_Texture* unit_tex = nullptr;

  int x, y;
  int label_width = 80; // Fixed width so boxes align vertically
  int padding = 10;

public:
  MetricRow(int x, int y, TTF_Font* font, size_t id, std::string label,
            std::string unit, RiderValueField::DataGetter getter);

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
               RiderValueField::DataGetter getter);

  void render(const RenderContext* ctx) override;
};

#endif
