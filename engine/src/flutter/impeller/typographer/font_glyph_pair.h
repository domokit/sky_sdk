// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_FONT_GLYPH_PAIR_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_FONT_GLYPH_PAIR_H_

#include "impeller/geometry/color.h"
#include "impeller/geometry/path.h"
#include "impeller/geometry/point.h"
#include "impeller/typographer/font.h"
#include "impeller/typographer/glyph.h"

namespace impeller {

struct GlyphProperties {
  Color color = Color::Black();
  Scalar stroke_width = 0.0;
  Cap stroke_cap = Cap::kButt;
  Join stroke_join = Join::kMiter;
  Scalar stroke_miter = 4.0;
  bool stroke = false;

  struct Equal {
    constexpr bool operator()(const impeller::GlyphProperties& lhs,
                              const impeller::GlyphProperties& rhs) const {
      return lhs.color.ToARGB() == rhs.color.ToARGB() &&
             lhs.stroke == rhs.stroke && lhs.stroke_cap == rhs.stroke_cap &&
             lhs.stroke_join == rhs.stroke_join &&
             lhs.stroke_miter == rhs.stroke_miter &&
             lhs.stroke_width == rhs.stroke_width;
    }
  };
};

//------------------------------------------------------------------------------
/// @brief      A font and a scale.  Used as a key that represents a typeface
///             within a glyph atlas.
///
struct ScaledFont {
  Font font;
  Scalar scale;

  template <typename H>
  friend H AbslHashValue(H h, const ScaledFont& sf) {
    return H::combine(std::move(h), sf.font.GetHash(), sf.scale);
  }

  struct Equal {
    constexpr bool operator()(const impeller::ScaledFont& lhs,
                              const impeller::ScaledFont& rhs) const {
      return lhs.font.IsEqual(rhs.font) && lhs.scale == rhs.scale;
    }
  };
};

//------------------------------------------------------------------------------
/// @brief      A glyph and its subpixel position.
///
struct SubpixelGlyph {
  Glyph glyph;
  Point subpixel_offset;
  std::optional<GlyphProperties> properties;

  SubpixelGlyph(Glyph p_glyph,
                Point p_subpixel_offset,
                std::optional<GlyphProperties> p_properties)
      : glyph(p_glyph),
        subpixel_offset(p_subpixel_offset),
        properties(p_properties) {}

  template <typename H>
  friend H AbslHashValue(H h, const SubpixelGlyph& sg) {
    if (!sg.properties.has_value()) {
      return H::combine(std::move(h), sg.glyph.index, sg.subpixel_offset.x,
                        sg.subpixel_offset.y);
    }
    return H::combine(std::move(h), sg.glyph.index, sg.subpixel_offset.x,
                      sg.subpixel_offset.y, sg.properties->color.ToARGB(),
                      sg.properties->stroke, sg.properties->stroke_cap,
                      sg.properties->stroke_join, sg.properties->stroke_miter,
                      sg.properties->stroke_width);
  }

  struct Equal {
    constexpr bool operator()(const impeller::SubpixelGlyph& lhs,
                              const impeller::SubpixelGlyph& rhs) const {
      // Check simple non-optionals first.
      if (lhs.glyph.index != rhs.glyph.index ||
          lhs.glyph.type != rhs.glyph.type ||
          lhs.subpixel_offset != rhs.subpixel_offset ||
          // Mixmatch properties.
          lhs.properties.has_value() != rhs.properties.has_value()) {
        return false;
      }
      if (lhs.properties.has_value()) {
        // Both have properties.
        return GlyphProperties::Equal{}(lhs.properties.value(),
                                        rhs.properties.value());
      }
      return true;
    }
  };
};

//------------------------------------------------------------------------------
/// @brief      A font along with a glyph in that font rendered at a particular
///             scale and subpixel position.
///
struct FontGlyphPair {
  FontGlyphPair(const ScaledFont& sf, const SubpixelGlyph& g)
      : scaled_font(sf), glyph(g) {}
  ScaledFont scaled_font;
  SubpixelGlyph glyph;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_FONT_GLYPH_PAIR_H_
