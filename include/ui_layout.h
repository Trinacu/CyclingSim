#ifndef UI_LAYOUT_H
#define UI_LAYOUT_H

#include "layout_types.h"
#include "widget.h"
#include <memory>
#include <vector>

class VStack : public Widget, public ILayoutWidget {
public:
  explicit VStack(int gap = 4, int padding = 0, HAlign halign = HAlign::Left);

  // Takes ownership of the child widget.
  void add(std::unique_ptr<Widget> child);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget / Drawable
  RenderLayer layer() const override { return RenderLayer::UI; }
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;
  void render_imgui(const RenderContext* ctx) override;

private:
  void do_layout();

  int origin_x = 0;
  int origin_y = 0;
  int gap;
  int padding;
  HAlign halign;
  bool dirty = true;

  std::vector<std::unique_ptr<Widget>> children;
};

class HStack : public Widget, public ILayoutWidget {
public:
  explicit HStack(int gap = 4, int padding = 0, VAlign valign = VAlign::Top);

  void add(std::unique_ptr<Widget> child);

  // ILayoutWidget
  LayoutSize get_preferred_size() const override;
  void set_bounds(LayoutRect r) override;

  // Widget / Drawable
  RenderLayer layer() const override { return RenderLayer::UI; }
  void render(const RenderContext* ctx) override;
  bool handle_event(const SDL_Event* e) override;
  void render_imgui(const RenderContext* ctx) override;

private:
  void do_layout();

  int origin_x = 0;
  int origin_y = 0;
  int gap;
  int padding;
  VAlign valign;
  bool dirty = true;

  std::vector<std::unique_ptr<Widget>> children;
};

enum class UIAnchor {
  TopLeft,
  TopCenter,
  TopRight,
  BottomLeft,
  BottomCenter,
  BottomRight,
};

class UIRoot {
public:
  UIRoot(int screen_w, int screen_h);

  // Add a widget at the given anchor with `margin` pixels from that edge.
  // Takes ownership of the widget.
  void add(UIAnchor anchor, int margin, std::unique_ptr<Widget> widget);

  // Compute and push absolute bounds into all managed widgets.
  // Widgets that don't implement ILayoutWidget are skipped (they
  // self-position).
  void resolve();

  // Per-frame hooks — call from the owning screen.
  void render(const RenderContext* ctx);
  bool handle_event(const SDL_Event* e);
  void render_imgui(const RenderContext* ctx);

private:
  LayoutRect rect_for(UIAnchor anchor, int margin, LayoutSize size) const;

  int screen_w;
  int screen_h;

  struct Entry {
    UIAnchor anchor;
    int margin;
    std::unique_ptr<Widget> widget;
  };

  std::vector<Entry> entries;
};

#endif
