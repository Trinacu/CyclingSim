// include/layout_types.h
#ifndef LAYOUT_TYPES_H
#define LAYOUT_TYPES_H

// ================================================================
//  Plain geometry types — no SDL, no simulation dependencies.
//  Included by both widget.h (for ILayoutWidget inheritance) and
//  ui_layout.h (for VStack / HStack / UIRoot).
// ================================================================

struct LayoutSize {
  int w, h;
};

struct LayoutRect {
  int x, y, w, h;
};

enum class HAlign { Left, Center, Right };
enum class VAlign { Top, Center, Bottom };

// ================================================================
//  ILayoutWidget
//
//  Mixin for any Widget that participates in automatic layout.
//  A widget that implements this interface:
//    - reports how much space it prefers via get_preferred_size()
//    - accepts an externally computed absolute rect via set_bounds()
//
//  Widgets that do NOT implement this are simply ignored by layout
//  containers; they render at whatever position they hold internally.
// ================================================================

class ILayoutWidget {
public:
  virtual LayoutSize get_preferred_size() const = 0;
  virtual void set_bounds(LayoutRect r) = 0;
  virtual ~ILayoutWidget() = default;
};

#endif
