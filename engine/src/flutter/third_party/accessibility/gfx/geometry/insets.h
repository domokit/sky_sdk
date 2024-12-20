// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_INSETS_H_
#define UI_GFX_GEOMETRY_INSETS_H_

#include <memory>
#include <string>

#include "gfx/gfx_export.h"
#include "insets_f.h"
#include "size.h"

namespace gfx {

class Vector2d;

// Represents the widths of the four borders or margins of an unspecified
// rectangle. An Insets stores the thickness of the top, left, bottom and right
// edges, without storing the actual size and position of the rectangle itself.
//
// This can be used to represent a space within a rectangle, by "shrinking" the
// rectangle by the inset amount on all four sides. Alternatively, it can
// represent a border that has a different thickness on each side.
class GFX_EXPORT Insets {
 public:
  constexpr Insets() : top_(0), left_(0), bottom_(0), right_(0) {}
  constexpr explicit Insets(int all)
      : top_(all),
        left_(all),
        bottom_(GetClampedValue(all, all)),
        right_(GetClampedValue(all, all)) {}
  constexpr explicit Insets(int vertical, int horizontal)
      : top_(vertical),
        left_(horizontal),
        bottom_(GetClampedValue(vertical, vertical)),
        right_(GetClampedValue(horizontal, horizontal)) {}
  constexpr Insets(int top, int left, int bottom, int right)
      : top_(top),
        left_(left),
        bottom_(GetClampedValue(top, bottom)),
        right_(GetClampedValue(left, right)) {}

  constexpr int top() const { return top_; }
  constexpr int left() const { return left_; }
  constexpr int bottom() const { return bottom_; }
  constexpr int right() const { return right_; }

  // Returns the total width taken up by the insets, which is the sum of the
  // left and right insets.
  constexpr int width() const { return left_ + right_; }

  // Returns the total height taken up by the insets, which is the sum of the
  // top and bottom insets.
  constexpr int height() const { return top_ + bottom_; }

  // Returns the sum of the left and right insets as the width, the sum of the
  // top and bottom insets as the height.
  constexpr Size size() const { return Size(width(), height()); }

  // Returns true if the insets are empty.
  bool IsEmpty() const { return width() == 0 && height() == 0; }

  void set_top(int top) {
    top_ = top;
    bottom_ = GetClampedValue(top_, bottom_);
  }
  void set_left(int left) {
    left_ = left;
    right_ = GetClampedValue(left_, right_);
  }
  void set_bottom(int bottom) { bottom_ = GetClampedValue(top_, bottom); }
  void set_right(int right) { right_ = GetClampedValue(left_, right); }

  void Set(int top, int left, int bottom, int right) {
    top_ = top;
    left_ = left;
    bottom_ = GetClampedValue(top_, bottom);
    right_ = GetClampedValue(left_, right);
  }

  bool operator==(const Insets& insets) const {
    return top_ == insets.top_ && left_ == insets.left_ &&
           bottom_ == insets.bottom_ && right_ == insets.right_;
  }

  bool operator!=(const Insets& insets) const { return !(*this == insets); }

  void operator+=(const Insets& insets) {
    top_ = base::ClampAdd(top_, insets.top_);
    left_ = base::ClampAdd(left_, insets.left_);
    bottom_ = GetClampedValue(top_, base::ClampAdd(bottom_, insets.bottom_));
    right_ = GetClampedValue(left_, base::ClampAdd(right_, insets.right_));
  }

  void operator-=(const Insets& insets) {
    top_ = base::ClampSub(top_, insets.top_);
    left_ = base::ClampSub(left_, insets.left_);
    bottom_ = GetClampedValue(top_, base::ClampSub(bottom_, insets.bottom_));
    right_ = GetClampedValue(left_, base::ClampSub(right_, insets.right_));
  }

  Insets operator-() const {
    return Insets(-base::MakeClampedNum(top_), -base::MakeClampedNum(left_),
                  -base::MakeClampedNum(bottom_),
                  -base::MakeClampedNum(right_));
  }

  Insets Scale(float scale) const { return Scale(scale, scale); }

  Insets Scale(float x_scale, float y_scale) const {
    return Insets(static_cast<int>(base::ClampMul(top(), y_scale)),
                  static_cast<int>(base::ClampMul(left(), x_scale)),
                  static_cast<int>(base::ClampMul(bottom(), y_scale)),
                  static_cast<int>(base::ClampMul(right(), x_scale)));
  }

  // Adjusts the vertical and horizontal dimensions by the values described in
  // |vector|. Offsetting insets before applying to a rectangle would be
  // equivalent to offseting the rectangle then applying the insets.
  Insets Offset(const gfx::Vector2d& vector) const;

  operator InsetsF() const {
    return InsetsF(static_cast<float>(top()), static_cast<float>(left()),
                   static_cast<float>(bottom()), static_cast<float>(right()));
  }

  // Returns a string representation of the insets.
  std::string ToString() const;

 private:
  int top_;
  int left_;
  int bottom_;
  int right_;

  // See rect.h
  // Returns true iff a+b would overflow max int.
  static constexpr bool AddWouldOverflow(int a, int b) {
    // In this function, GCC tries to make optimizations that would only work if
    // max - a wouldn't overflow but it isn't smart enough to notice that a > 0.
    // So cast everything to unsigned to avoid this.  As it is guaranteed that
    // max - a and b are both already positive, the cast is a noop.
    //
    // This is intended to be: a > 0 && max - a < b
    return a > 0 && b > 0 &&
           static_cast<unsigned>(std::numeric_limits<int>::max() - a) <
               static_cast<unsigned>(b);
  }

  // Returns true iff a+b would underflow min int.
  static constexpr bool AddWouldUnderflow(int a, int b) {
    return a < 0 && b < 0 && std::numeric_limits<int>::min() - a > b;
  }

  // Clamp the right/bottom to avoid integer over/underflow in width() and
  // height(). This returns the right/bottom given a top_or_left and a
  // bottom_or_right.
  // TODO(enne): this should probably use base::ClampAdd, but that
  // function is not a constexpr.
  static constexpr int GetClampedValue(int top_or_left, int bottom_or_right) {
    if (AddWouldOverflow(top_or_left, bottom_or_right)) {
      return std::numeric_limits<int>::max() - top_or_left;
    } else if (AddWouldUnderflow(top_or_left, bottom_or_right)) {
      // If |top_or_left| and |bottom_or_right| are both negative,
      // adds |top_or_left| to prevent underflow by subtracting it.
      return std::numeric_limits<int>::min() - top_or_left;
    } else {
      return bottom_or_right;
    }
  }
};

inline Insets operator+(Insets lhs, const Insets& rhs) {
  lhs += rhs;
  return lhs;
}

inline Insets operator-(Insets lhs, const Insets& rhs) {
  lhs -= rhs;
  return lhs;
}

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_INSETS_H_
