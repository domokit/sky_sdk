// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_FONT_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_FONT_H_

#include <memory>

#include "fml/hash_combine.h"
#include "impeller/base/comparable.h"
#include "impeller/geometry/scalar.h"
#include "impeller/typographer/typeface.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Determines the axis along which there is subpixel positioning.
///
enum class AxisAlignment : uint8_t {
  // No subpixel positioning.
  kNone,
  // Subpixel positioning in the X axis only.
  kX,
  // Subpixel positioning in the Y axis only.
  kY,
  // No specific axis, subpixel positioning in each direction.
  kAll,
};

//------------------------------------------------------------------------------
/// @brief      Describes a typeface along with any modifications to its
///             intrinsic properties.
///
class Font : public Comparable<Font> {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Describes the modifications made to the intrinsic properties
  ///             of a typeface.
  ///
  ///             The coordinate system of a font has its origin at (0, 0) on
  ///             the baseline with an upper-left-origin coordinate system.
  ///
  struct Metrics {
    //--------------------------------------------------------------------------
    /// The point size of the font.
    ///
    Scalar point_size = 12.0f;
    bool embolden = false;
    Scalar skewX = 0.0f;
    Scalar scaleX = 1.0f;

    constexpr bool operator==(const Metrics& o) const {
      return point_size == o.point_size && embolden == o.embolden &&
             skewX == o.skewX && scaleX == o.scaleX;
    }
  };

  Font(std::shared_ptr<Typeface> typeface,
       Metrics metrics,
       AxisAlignment axis_alignment);

  ~Font();

  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      The typeface whose intrinsic properties this font modifies.
  ///
  /// @return     The typeface.
  ///
  const std::shared_ptr<Typeface>& GetTypeface() const;

  const Metrics& GetMetrics() const;

  // |Comparable<Font>|
  std::size_t GetHash() const override;

  // |Comparable<Font>|
  bool IsEqual(const Font& other) const override;

  AxisAlignment GetAxisAlignment() const;

 private:
  std::shared_ptr<Typeface> typeface_;
  Metrics metrics_ = {};
  AxisAlignment axis_alignment_;
  bool is_valid_ = false;
};

}  // namespace impeller

template <>
struct std::hash<impeller::Font::Metrics> {
  constexpr std::size_t operator()(const impeller::Font::Metrics& m) const {
    return fml::HashCombine(m.point_size, m.skewX, m.scaleX);
  }
};

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_FONT_H_
