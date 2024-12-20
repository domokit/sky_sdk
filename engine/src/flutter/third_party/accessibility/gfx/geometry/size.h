// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_SIZE_H_
#define UI_GFX_GEOMETRY_SIZE_H_

#include <algorithm>
#include <iosfwd>
#include <string>

#include "ax_build/build_config.h"
#include "base/compiler_specific.h"
#include "base/numerics/safe_math.h"
#include "gfx/gfx_export.h"

#if defined(OS_WIN)
typedef struct tagSIZE SIZE;
#elif defined(OS_APPLE)
typedef struct CGSize CGSize;
#endif

namespace gfx {

// A size has width and height values.
class GFX_EXPORT Size {
 public:
  constexpr Size() : width_(0), height_(0) {}
  constexpr Size(int width, int height)
      : width_(std::max(0, width)), height_(std::max(0, height)) {}
#if defined(OS_APPLE)
  explicit Size(const CGSize& s);
#endif

#if defined(OS_APPLE)
  Size& operator=(const CGSize& s);
#endif

  void operator+=(const Size& size);

  void operator-=(const Size& size);

#if defined(OS_WIN)
  SIZE ToSIZE() const;
#elif defined(OS_APPLE)
  CGSize ToCGSize() const;
#endif

  constexpr int width() const { return width_; }
  constexpr int height() const { return height_; }

  void set_width(int width) { width_ = std::max(0, width); }
  void set_height(int height) { height_ = std::max(0, height); }

  // This call will CHECK if the area of this size would overflow int.
  int GetArea() const;
  // Returns a checked numeric representation of the area.
  base::CheckedNumeric<int> GetCheckedArea() const;

  void SetSize(int width, int height) {
    set_width(width);
    set_height(height);
  }

  void Enlarge(int grow_width, int grow_height);

  void SetToMin(const Size& other);
  void SetToMax(const Size& other);

  bool IsEmpty() const { return !width() || !height(); }

  std::string ToString() const;

 private:
  int width_;
  int height_;
};

inline bool operator==(const Size& lhs, const Size& rhs) {
  return lhs.width() == rhs.width() && lhs.height() == rhs.height();
}

inline bool operator!=(const Size& lhs, const Size& rhs) {
  return !(lhs == rhs);
}

inline Size operator+(Size lhs, const Size& rhs) {
  lhs += rhs;
  return lhs;
}

inline Size operator-(Size lhs, const Size& rhs) {
  lhs -= rhs;
  return lhs;
}

// This is declared here for use in gtest-based unit tests but is defined in
// the //ui/gfx:test_support target. Depend on that to use this in your unit
// test. This should not be used in production code - call ToString() instead.
void PrintTo(const Size& size, ::std::ostream* os);

// Helper methods to scale a gfx::Size to a new gfx::Size.
GFX_EXPORT Size ScaleToCeiledSize(const Size& size, float x_scale, float y_scale);
GFX_EXPORT Size ScaleToCeiledSize(const Size& size, float scale);
GFX_EXPORT Size ScaleToFlooredSize(const Size& size, float x_scale, float y_scale);
GFX_EXPORT Size ScaleToFlooredSize(const Size& size, float scale);
GFX_EXPORT Size ScaleToRoundedSize(const Size& size, float x_scale, float y_scale);
GFX_EXPORT Size ScaleToRoundedSize(const Size& size, float scale);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_SIZE_H_
