// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_INSETS_F_H_
#define UI_GFX_GEOMETRY_INSETS_F_H_

#include <string>

#include "gfx/gfx_export.h"

namespace gfx {

// A floating point version of gfx::Insets.
class GFX_EXPORT InsetsF {
 public:
  constexpr InsetsF() : top_(0.f), left_(0.f), bottom_(0.f), right_(0.f) {}
  constexpr explicit InsetsF(float all)
      : top_(all), left_(all), bottom_(all), right_(all) {}
  constexpr InsetsF(float vertical, float horizontal)
      : top_(vertical),
        left_(horizontal),
        bottom_(vertical),
        right_(horizontal) {}
  constexpr InsetsF(float top, float left, float bottom, float right)
      : top_(top), left_(left), bottom_(bottom), right_(right) {}

  constexpr float top() const { return top_; }
  constexpr float left() const { return left_; }
  constexpr float bottom() const { return bottom_; }
  constexpr float right() const { return right_; }

  // Returns the total width taken up by the insets, which is the sum of the
  // left and right insets.
  constexpr float width() const { return left_ + right_; }

  // Returns the total height taken up by the insets, which is the sum of the
  // top and bottom insets.
  constexpr float height() const { return top_ + bottom_; }

  // Returns true if the insets are empty.
  bool IsEmpty() const { return width() == 0.f && height() == 0.f; }

  void Set(float top, float left, float bottom, float right) {
    top_ = top;
    left_ = left;
    bottom_ = bottom;
    right_ = right;
  }

  bool operator==(const InsetsF& insets) const {
    return top_ == insets.top_ && left_ == insets.left_ &&
           bottom_ == insets.bottom_ && right_ == insets.right_;
  }

  bool operator!=(const InsetsF& insets) const { return !(*this == insets); }

  void operator+=(const InsetsF& insets) {
    top_ += insets.top_;
    left_ += insets.left_;
    bottom_ += insets.bottom_;
    right_ += insets.right_;
  }

  void operator-=(const InsetsF& insets) {
    top_ -= insets.top_;
    left_ -= insets.left_;
    bottom_ -= insets.bottom_;
    right_ -= insets.right_;
  }

  InsetsF operator-() const {
    return InsetsF(-top_, -left_, -bottom_, -right_);
  }

  InsetsF Scale(float scale) const {
    return InsetsF(scale * top(), scale * left(), scale * bottom(),
                   scale * right());
  }

  // Returns a string representation of the insets.
  std::string ToString() const;

 private:
  float top_;
  float left_;
  float bottom_;
  float right_;
};

inline InsetsF operator+(InsetsF lhs, const InsetsF& rhs) {
  lhs += rhs;
  return lhs;
}

inline InsetsF operator-(InsetsF lhs, const InsetsF& rhs) {
  lhs -= rhs;
  return lhs;
}

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_INSETS_F_H_
