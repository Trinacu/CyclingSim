// src/ui_layout.cpp
#include "ui_layout.h"
#include "layout_types.h"
#include <algorithm>

// ================================================================
//  VStack
// ================================================================

VStack::VStack(int gap_, int padding_, HAlign halign_)
    : gap(gap_), padding(padding_), halign(halign_) {}

void VStack::add(std::unique_ptr<Widget> child) {
  children.push_back(std::move(child));
  dirty = true;
}

LayoutSize VStack::get_preferred_size() const {
  int total_h = 2 * padding;
  int max_w = 0;

  for (const auto& child : children) {
    if (auto* lw = dynamic_cast<const ILayoutWidget*>(child.get())) {
      LayoutSize s = lw->get_preferred_size();
      total_h += s.h;
      max_w = std::max(max_w, s.w);
    }
    // Non-layout children don't contribute to preferred size —
    // the stack can't know their dimensions without the interface.
  }

  const int n = static_cast<int>(children.size());
  if (n > 1)
    total_h += gap * (n - 1);

  return {max_w + 2 * padding, total_h};
}

void VStack::set_bounds(LayoutRect r) {
  origin_x = r.x;
  origin_y = r.y;
  // Note: we intentionally ignore r.w / r.h — the stack sizes itself
  // from its children, not from the rect imposed from above.
  dirty = true;
}

void VStack::do_layout() {
  const int inner_w = get_preferred_size().w - 2 * padding;

  int cursor_y = origin_y + padding;

  for (auto& child : children) {
    auto* lw = dynamic_cast<ILayoutWidget*>(child.get());
    if (!lw)
      continue;

    LayoutSize s = lw->get_preferred_size();

    int x_offset = 0;
    switch (halign) {
    case HAlign::Left:
      x_offset = 0;
      break;
    case HAlign::Center:
      x_offset = (inner_w - s.w) / 2;
      break;
    case HAlign::Right:
      x_offset = inner_w - s.w;
      break;
    }
    lw->set_bounds({origin_x + padding + x_offset, cursor_y, s.w, s.h});
    cursor_y += s.h + gap;
  }
  dirty = false;
}

void VStack::render(const RenderContext* ctx) {
  if (!visible)
    return;
  if (dirty)
    do_layout();
  for (auto& child : children)
    child->render(ctx);
}

bool VStack::handle_event(const SDL_Event* e) {
  // Iterate in reverse so the topmost (last-added) child gets first pick,
  // matching typical z-order conventions.
  for (auto it = children.rbegin(); it != children.rend(); ++it)
    if ((*it)->handle_event(e))
      return true;
  return false;
}

void VStack::render_imgui(const RenderContext* ctx) {
  for (auto& child : children)
    child->render_imgui(ctx);
}

// ================================================================
//  HStack
// ================================================================

HStack::HStack(int gap_, int padding_, VAlign valign_)
    : gap(gap_), padding(padding_), valign(valign_) {}

void HStack::add(std::unique_ptr<Widget> child) {
  children.push_back(std::move(child));
  dirty = true;
}

LayoutSize HStack::get_preferred_size() const {
  int total_w = 2 * padding;
  int max_h = 0;

  for (const auto& child : children) {
    if (auto* lw = dynamic_cast<const ILayoutWidget*>(child.get())) {
      LayoutSize s = lw->get_preferred_size();
      total_w += s.w;
      max_h = std::max(max_h, s.h);
    }
  }

  const int n = static_cast<int>(children.size());
  if (n > 1)
    total_w += gap * (n - 1);

  return {total_w, max_h + 2 * padding};
}

void HStack::set_bounds(LayoutRect r) {
  origin_x = r.x;
  origin_y = r.y;
  dirty = true;
}

void HStack::do_layout() {
  const int inner_h = get_preferred_size().h - 2 * padding;

  int cursor_x = origin_x + padding;

  for (auto& child : children) {
    auto* lw = dynamic_cast<ILayoutWidget*>(child.get());
    if (!lw)
      continue;

    LayoutSize s = lw->get_preferred_size();

    // Compute Y offset from the stack's top inner edge based on alignment.
    int y_offset = 0;
    switch (valign) {
    case VAlign::Top:
      y_offset = 0;
      break;
    case VAlign::Center:
      y_offset = (inner_h - s.h) / 2;
      break;
    case VAlign::Bottom:
      y_offset = inner_h - s.h;
      break;
    }

    lw->set_bounds({cursor_x, origin_y + padding + y_offset, s.w, s.h});
    cursor_x += s.w + gap;
  }

  dirty = false;
}

void HStack::render(const RenderContext* ctx) {
  if (!visible)
    return;
  if (dirty)
    do_layout();
  for (auto& child : children)
    child->render(ctx);
}

bool HStack::handle_event(const SDL_Event* e) {
  for (auto it = children.rbegin(); it != children.rend(); ++it)
    if ((*it)->handle_event(e))
      return true;
  return false;
}

void HStack::render_imgui(const RenderContext* ctx) {
  for (auto& child : children)
    child->render_imgui(ctx);
}

// ================================================================
//  UIRoot
// ================================================================

UIRoot::UIRoot(int w, int h) : screen_w(w), screen_h(h) {}

void UIRoot::add(UIAnchor anchor, int margin, std::unique_ptr<Widget> widget) {
  entries.push_back({anchor, margin, std::move(widget)});
}

LayoutRect UIRoot::rect_for(UIAnchor anchor, int margin,
                            LayoutSize size) const {
  int x = 0, y = 0;

  switch (anchor) {
  case UIAnchor::TopLeft:
    x = margin;
    y = margin;
    break;
  case UIAnchor::TopCenter:
    x = (screen_w - size.w) / 2;
    y = margin;
    break;
  case UIAnchor::TopRight:
    x = screen_w - size.w - margin;
    y = margin;
    break;
  case UIAnchor::BottomLeft:
    x = margin;
    y = screen_h - size.h - margin;
    break;
  case UIAnchor::BottomCenter:
    x = (screen_w - size.w) / 2;
    y = screen_h - size.h - margin;
    break;
  case UIAnchor::BottomRight:
    x = screen_w - size.w - margin;
    y = screen_h - size.h - margin;
    break;
  }

  return {x, y, size.w, size.h};
}

void UIRoot::resolve() {
  for (auto& entry : entries) {
    if (auto* lw = dynamic_cast<ILayoutWidget*>(entry.widget.get())) {
      LayoutSize size = lw->get_preferred_size();
      LayoutRect rect = rect_for(entry.anchor, entry.margin, size);
      lw->set_bounds(rect);
    }
    // Widgets without ILayoutWidget are self-positioning; skip them.
  }
}

void UIRoot::render(const RenderContext* ctx) {
  for (auto& entry : entries)
    entry.widget->render(ctx);
}

bool UIRoot::handle_event(const SDL_Event* e) {
  // Last-added entry is visually on top; give it first pick on events.
  for (auto it = entries.rbegin(); it != entries.rend(); ++it)
    if (it->widget->handle_event(e))
      return true;
  return false;
}

void UIRoot::render_imgui(const RenderContext* ctx) {
  for (auto& entry : entries)
    entry.widget->render_imgui(ctx);
}
