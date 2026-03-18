// widget.h
#ifndef WIDGET_H
#define WIDGET_H

#include "SDL3/SDL_pixels.h"
#include "SDL3/SDL_render.h"
#include "display.h"
#include "layout_types.h"
#include "sim.h"
#include "snapshot.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <functional>

std::string format_number(double value, int precision = 1);

class Widget : public Drawable {
public:
  RenderLayer layer() const override { return RenderLayer::UI; }
  virtual void render(const RenderContext* ctx) override = 0;
  virtual ~Widget() = default;
  virtual void render_imgui(const RenderContext* ctx) {}
  bool visible = true;
};

class ProgressBar : public Widget, public ILayoutWidget, public IRiderWidget {
public:
  using ValueFn = std::function<double()>;
  using RiderDataFn = std::function<double(const RiderRenderState&)>;
  using ColorFn = std::function<SDL_Color(double)>;

  // Standalone — pointer binding
  ProgressBar(int x, int y, int w, int h, const double* value_ptr,
              SDL_Color bg_color, SDL_Color fill_color, double min_val = 0.0,
              double max_val = 1.0);

  // Standalone — getter binding
  ProgressBar(int x, int y, int w, int h, ValueFn getter, SDL_Color bg_color,
              SDL_Color fill_color, double min_val = 0.0, double max_val = 1.0);

  // Rider-bound — looks up id in ctx->riders at render time
  ProgressBar(int x, int y, int w, int h, RiderId id, RiderDataFn getter,
              SDL_Color bg_color, SDL_Color fill_color, double min_val = 0.0,
              double max_val = 1.0);

  ~ProgressBar();

  // Configuration — safe to call before or after first render
  void set_label(std::string text, TTF_Font* font);
  void set_fill_color(SDL_Color c);
  void set_color_fn(ColorFn fn);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget
  void render(const RenderContext* ctx) override;

  // IRiderWidget
  void set_rider_id(RiderId id) override;

private:
  void build_label_tex(SDL_Renderer* r);
  void draw(SDL_Renderer* r, double t); // t is already normalized [0,1]

  int x, y, w, h;
  double min_val, max_val;

  ValueFn source; // set by standalone constructors

  struct RiderBinding {
    RiderId id;
    RiderDataFn getter;
  };
  std::optional<RiderBinding> rider_binding; // set by rider constructor

  std::string label_str;
  TTF_Font* label_font = nullptr;
  SDL_Texture* label_tex = nullptr;

  ColorFn color_fn;
  SDL_Color fill_color = {80, 200, 80, 255};
  SDL_Color bg_color = {40, 40, 40, 200};
};

class Stopwatch : public Widget, public ILayoutWidget {
private:
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
  Stopwatch(int x, int y, TTF_Font* font_, int update_ms = 100)
      : font(font_), screen_x(x), screen_y(y), update_interval_ms(update_ms) {
    last_update_ticks = 0;
  }

  ~Stopwatch() {
    if (texture)
      SDL_DestroyTexture(texture);
    if (bg_texture)
      SDL_DestroyTexture(bg_texture);
  }
  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void update_texture(const RenderContext* ctx);
  void render(const RenderContext* ctx) override;
};

class LateralOverview : public Widget, public ILayoutWidget {
public:
  LateralOverview(int w, int h, const Course* course);
  ~LateralOverview();

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;

private:
  const Course* course;
  SDL_Texture* tex = nullptr; // baked once
  int x_ = 0, y_ = 0;
  int w_, h_;

  static constexpr float kPad = 6.f;          // px inset from widget edge
  static constexpr float kRiderRadius = 4.5f; // px, circle radius for rider dot
  static constexpr int kCircleSegs = 16; // triangle-fan segments for circle

  // Map (lon_pos, lat_pos) → pixel position within the widget.
  // lon_min/lon_max  — visible longitudinal range from camera
  // road_half        — half the road width at camera centre
  SDL_FPoint to_widget(double lon_pos, double lat_pos, double lon_min,
                       double lon_max, double road_half) const;

  // Filled circle via SDL_RenderGeometry triangle fan.
  static void fill_circle(SDL_Renderer* r, SDL_FPoint centre, float radius,
                          SDL_FColor colour);

  void ensure_target(SDL_Renderer* r); // replaces draw_texture alloc logic
  void draw_into_target(const RenderContext* ctx); // renamed, no alloc inside

  // Colour assigned to each rider based on team_id.
  static SDL_FColor rider_colour(int team_id);
};

class MinimapWidget : public Widget, public ILayoutWidget {
  const Course* course;
  SDL_Texture* profile_tex = nullptr; // baked once

  int x, y, w, h; // screen rect
  float pad = 8.f;

  // computed once from visual_points
  double world_x_min, world_x_max;
  double world_y_min, world_y_max;

public:
  MinimapWidget(int x, int y, int w, int h, const Course* course);
  ~MinimapWidget() { SDL_DestroyTexture(profile_tex); }

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;

private:
  void bake_profile(SDL_Renderer* r);
  SDL_FPoint world_to_mini(double wx, double wy) const;
};

class Button : public Widget, public ILayoutWidget {
public:
  using Callback = std::function<void()>;
  Button(int x, int y, int w, int h, std::string label, TTF_Font* font,
         Callback cb);

  ~Button();

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  int x, y, w, h;
  std::string label;
  TTF_Font* font;
  Callback on_click;
  bool hovered = false;
  bool pressed = false;
  SDL_Texture* label_tex = nullptr;

  void build_label_text(SDL_Renderer* r);

protected:
  void set_label(std::string label);
};

class TimeFactorButton : public Button {
public:
  TimeFactorButton(int x, int y, int w, int h, double val, Simulation* sim,
                   TTF_Font* font)
      : Button(x, y, w, h, format_label(val), font,
               [sim, val]() { sim->set_time_factor(val); }) {}

private:
  static std::string format_label(double v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f ×", v);
    return std::string(buf);
  }
};

class PauseButton : public Button {
private:
  Simulation* sim = nullptr;

public:
  PauseButton(int x, int y, int w, int h, Simulation* sim, TTF_Font* font);
  void render(const RenderContext* ctx) override;
};

class TimeFactorSlider : public Widget, public ILayoutWidget {
public:
  TimeFactorSlider(int x_, int y_, int w_, int h_, Simulation* sim_)
      : x(x_), y(y_), w(w_), h(h_), sim(sim_) {}

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  int x, y, w, h;
  double neutral_point = 0.3; // where factor = 1.0
  int marker_width = 4;
  bool dragging = false;
  double slider_pos = 0.5; // slider position 0..1
  Simulation* sim;

  double slider_to_factor(double t);
  double factor_to_slider(double f);
};

class ValueField : public Widget, public ILayoutWidget {
protected:
  int x, y, w, h;
  TTF_Font* font;
  SDL_Color text_color = {255, 255, 255, 255};

  // Caching
  std::string current_text;
  SDL_Texture* texture = nullptr;
  SDL_Texture* bg_texture = nullptr;

  void create_bg(SDL_Renderer* renderer);
  void update_texture(SDL_Renderer* renderer);
  // so child class can overwrite with custom logic
  virtual std::string get_display_text() const { return current_text; }

public:
  ValueField(int x, int y, int w, int h, TTF_Font* font);
  virtual ~ValueField();

  void set_text(const std::string& text);
  const std::string& get_text() const { return current_text; }

  void set_position(int new_x, int new_y) {
    x = new_x;
    y = new_y;
  }
  int get_width() const { return w; }

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Standard Widget interface
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override { return false; }
};

class RiderValueField : public ValueField {
public:
  using DataGetter = std::function<std::string(const RiderRenderState&)>;

private:
  DataGetter getter;

public:
  RiderValueField(int x, int y, int w, int h, TTF_Font* font,
                  DataGetter getter);

  void render_with_snapshot(const RenderContext* ctx,
                            const RiderRenderState* snap);
};

class TimeControlPanel : public Widget, public ILayoutWidget {
public:
  TimeControlPanel(int x_, int y_, int h_, TTF_Font* font, Simulation* sim);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;

private:
  int x, y, w, h;
  int valfield_w = 50;
  int slider_w = 120;
  int button_w = 50;
  int gap_w = 5;
  int next_x = x + gap_w;
  std::vector<std::unique_ptr<Widget>> children;
  std::vector<std::pair<int, int>> child_rel_positions;

  ValueField* time_factor_field = nullptr;

  // for reading current time_factor
  Simulation* sim = nullptr;

  void do_layout(int base_x, int base_y);
};

// After:
class MetricRow : public Widget, public ILayoutWidget, public IRiderWidget {
public:
  MetricRow(int x, int y, TTF_Font* font, RiderId id, std::string label,
            std::string unit, RiderValueField::DataGetter getter);

  ~MetricRow();

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget
  void render(const RenderContext* ctx) override;

  // IRiderWidget
  void set_rider_id(RiderId id) override;

private:
  std::string label_txt;
  std::string unit_txt;
  TTF_Font* font;
  RiderId rider_id;

  // render_with_snapshot is called only here — not part of any public interface
  std::unique_ptr<RiderValueField> field;
  SDL_Texture* label_tex = nullptr;
  SDL_Texture* unit_tex = nullptr;

  int x, y;
  static constexpr int label_width = 80;
  static constexpr int row_h = 30;
  static constexpr int field_w = 80;
  static constexpr int field_h = 24;
};

class RiderPanel : public Widget, public ILayoutWidget {
public:
  RiderPanel(int x, int y, TTF_Font* font);
  ~RiderPanel();

  void set_rider_id(RiderId id_);
  // Usage: panel->add_row("Speed", "km/h", [](auto s){ return ... });
  void add_row(std::string label, std::string unit,
               RiderValueField::DataGetter getter);
  void add_bar(std::string label, ProgressBar::RiderDataFn getter,
               SDL_Color bg_color = {15, 150, 15, 255},
               SDL_Color fill_color = {20, 200, 20, 255}, double min = 0.0,
               double max = 1.0);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  void render(const RenderContext* ctx) override;
  void render_imgui(const RenderContext* ctx) override;

  void render_plot_overlay(const RenderContext* ctx);

private:
  int x, y;
  RiderId id = -1;
  TTF_Font* font;
  bool dirty = true;

  std::string title;
  SDL_Texture* title_tex = nullptr;
  bool show_plot = false;

  // All the rows
  std::vector<std::unique_ptr<Widget>> children;

  void do_layout();
  void build_fields();
};

/* EDITABLE FIELDS */

class BaseEditableField : public ValueField {
public:
  using CommitCallback = std::function<void(const std::string&)>;

protected:
  bool editing = false;
  std::string buffer;

  SDL_Window* window;
  CommitCallback on_commit;

  // blinking cursor
  Uint32 last_cursor_blink = 0;
  bool cursor_visible = true;
  const Uint32 BLINK_MS = 500;

  // hook: validation/acceptance of character input
  virtual bool accept_char(char c, const std::string& before) = 0;

  // helper: commit and fire callback
  void commit();

  // Chooses which text ValueField should render
  std::string current_display_text() const;

public:
  BaseEditableField(int x, int y, int w, int h, TTF_Font* font, SDL_Window* win,
                    CommitCallback cb)
      : ValueField(x, y, w, h, font), window(win), on_commit(std::move(cb)) {}

  bool handle_event(const SDL_Event* e) override;
  void render(const RenderContext* ctx) override;
};

class EditableNumberField : public BaseEditableField {
public:
  EditableNumberField(int x, int y, int w, int h, TTF_Font* font,
                      SDL_Window* win, std::function<void(double)> cb)
      : BaseEditableField(x, y, w, h, font, win,
                          [cb](const std::string& s) { cb(atof(s.c_str())); }) {
  }

protected:
  bool accept_char(char c, const std::string& before) override;
};

class EditableStringField : public BaseEditableField {
public:
  EditableStringField(int x, int y, int w, int h, TTF_Font* font,
                      SDL_Window* win,
                      std::function<void(const std::string&)> cb)
      : BaseEditableField(x, y, w, h, font, win, std::move(cb)) {}

protected:
  bool accept_char(char c, const std::string&) override { return true; }
};

#endif
