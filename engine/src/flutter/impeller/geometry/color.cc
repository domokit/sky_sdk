// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/geometry/color.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <type_traits>

#include "impeller/base/strings.h"
#include "impeller/geometry/constants.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/vector.h"

namespace impeller {

#define _IMPELLER_ASSERT_BLEND_MODE(blend_mode)                            \
  auto enum_##blend_mode = static_cast<std::underlying_type_t<BlendMode>>( \
      BlendMode::k##blend_mode);                                           \
  if (i != enum_##blend_mode) {                                            \
    return false;                                                          \
  }                                                                        \
  ++i;

static constexpr inline bool ValidateBlendModes() {
  std::underlying_type_t<BlendMode> i = 0;
  // Ensure the order of the blend modes match.
  IMPELLER_FOR_EACH_BLEND_MODE(_IMPELLER_ASSERT_BLEND_MODE)
  // Ensure the total number of blend modes match.
  if (i - 1 !=
      static_cast<std::underlying_type_t<BlendMode>>(BlendMode::kLast)) {
    return false;
  }
  return true;
}
static_assert(ValidateBlendModes(),
              "IMPELLER_FOR_EACH_BLEND_MODE must match impeller::BlendMode.");

#define _IMPELLER_BLEND_MODE_NAME_LIST(blend_mode) #blend_mode,

static constexpr const char* kBlendModeNames[] = {
    IMPELLER_FOR_EACH_BLEND_MODE(_IMPELLER_BLEND_MODE_NAME_LIST)};

const char* BlendModeToString(BlendMode blend_mode) {
  return kBlendModeNames[static_cast<std::underlying_type_t<BlendMode>>(
      blend_mode)];
}

Color::Color(const Vector4& value)
    : red(value.x), green(value.y), blue(value.z), alpha(value.w) {}

static constexpr inline Color Min(Color c, float threshold) {
  return Color(std::min(c.red, threshold), std::min(c.green, threshold),
               std::min(c.blue, threshold), std::min(c.alpha, threshold));
}

// The following HSV utilities correspond to the W3C blend definitions
// implemented in: impeller/compiler/shader_lib/impeller/blending.glsl

static constexpr inline Scalar Luminosity(Vector3 color) {
  return color.x * 0.3f + color.y * 0.59f + color.z * 0.11f;
}

static constexpr inline Vector3 ClipColor(Vector3 color) {
  Scalar lum = Luminosity(color);
  Scalar mn = std::min(std::min(color.x, color.y), color.z);
  Scalar mx = std::max(std::max(color.x, color.y), color.z);
  // `lum - mn` and `mx - lum` will always be >= 0 in the following conditions,
  // so adding a tiny value is enough to make these divisions safe.
  if (mn < 0.0f) {
    color = lum + (((color - lum) * lum) / (lum - mn + kEhCloseEnough));
  }
  if (mx > 1.0) {
    color =
        lum + (((color - lum) * (1.0f - lum)) / (mx - lum + kEhCloseEnough));
  }
  return color;
}

static constexpr inline Vector3 SetLuminosity(Vector3 color,
                                              Scalar luminosity) {
  Scalar relative_lum = luminosity - Luminosity(color);
  return ClipColor(color + relative_lum);
}

static constexpr inline Scalar Saturation(Vector3 color) {
  return std::max(std::max(color.x, color.y), color.z) -
         std::min(std::min(color.x, color.y), color.z);
}

static constexpr inline Vector3 SetSaturation(Vector3 color,
                                              Scalar saturation) {
  Scalar mn = std::min(std::min(color.x, color.y), color.z);
  Scalar mx = std::max(std::max(color.x, color.y), color.z);
  return (mn < mx) ? ((color - mn) * saturation) / (mx - mn) : Vector3();
}

static constexpr inline Vector3 ComponentChoose(Vector3 a,
                                                Vector3 b,
                                                Vector3 value,
                                                Scalar cutoff) {
  return Vector3(value.x > cutoff ? b.x : a.x,  //
                 value.y > cutoff ? b.y : a.y,  //
                 value.z > cutoff ? b.z : a.z   //
  );
}

static constexpr inline Vector3 ToRGB(Color color) {
  return {color.red, color.green, color.blue};
}

static constexpr inline Color FromRGB(Vector3 color, Scalar alpha) {
  return {color.x, color.y, color.z, alpha};
}

/// Composite a blended color onto the destination.
/// All three parameters are unpremultiplied. Returns a premultiplied result.
///
/// This routine is the same as `IPApplyBlendedColor` in the Impeller shader
/// library.
static constexpr inline Color ApplyBlendedColor(Color dst,
                                                Color src,
                                                Vector3 blend_result) {
  dst = dst.Premultiply();
  src =
      // Use the blended color for areas where the source and destination
      // colors overlap.
      FromRGB(blend_result, src.alpha * dst.alpha).Premultiply() +
      // Use the original source color for any remaining non-overlapping areas.
      src.Premultiply() * (1.0f - dst.alpha);

  // Source-over composite the blended source color atop the destination.
  return src + dst * (1.0f - src.alpha);
}

static constexpr inline Color DoColorBlend(
    Color dst,
    Color src,
    const std::function<Vector3(Vector3, Vector3)>& blend_rgb_func) {
  const Vector3 blend_result = blend_rgb_func(ToRGB(dst), ToRGB(src));
  return ApplyBlendedColor(dst, src, blend_result).Unpremultiply();
}

static constexpr inline Color DoColorBlendComponents(
    Color dst,
    Color src,
    const std::function<Scalar(Scalar, Scalar)>& blend_func) {
  Vector3 blend_result = Vector3(blend_func(dst.red, src.red),      //
                                 blend_func(dst.green, src.green),  //
                                 blend_func(dst.blue, src.blue));   //
  return ApplyBlendedColor(dst, src, blend_result).Unpremultiply();
}

Color Color::Blend(Color src, BlendMode blend_mode) const {
  Color dst = *this;

  switch (blend_mode) {
    case BlendMode::kClear:
      return Color::BlackTransparent();
    case BlendMode::kSource:
      return src;
    case BlendMode::kDestination:
      return dst;
    case BlendMode::kSourceOver:
      // r = s + (1-sa)*d
      return (src.Premultiply() + dst.Premultiply() * (1 - src.alpha))
          .Unpremultiply();
    case BlendMode::kDestinationOver:
      // r = d + (1-da)*s
      return (dst.Premultiply() + src.Premultiply() * (1 - dst.alpha))
          .Unpremultiply();
    case BlendMode::kSourceIn:
      // r = s * da
      return (src.Premultiply() * dst.alpha).Unpremultiply();
    case BlendMode::kDestinationIn:
      // r = d * sa
      return (dst.Premultiply() * src.alpha).Unpremultiply();
    case BlendMode::kSourceOut:
      // r = s * ( 1- da)
      return (src.Premultiply() * (1 - dst.alpha)).Unpremultiply();
    case BlendMode::kDestinationOut:
      // r = d * (1-sa)
      return (dst.Premultiply() * (1 - src.alpha)).Unpremultiply();
    case BlendMode::kSourceATop:
      // r = s*da + d*(1-sa)
      return (src.Premultiply() * dst.alpha +
              dst.Premultiply() * (1 - src.alpha))
          .Unpremultiply();
    case BlendMode::kDestinationATop:
      // r = d*sa + s*(1-da)
      return (dst.Premultiply() * src.alpha +
              src.Premultiply() * (1 - dst.alpha))
          .Unpremultiply();
    case BlendMode::kXor:
      // r = s*(1-da) + d*(1-sa)
      return (src.Premultiply() * (1 - dst.alpha) +
              dst.Premultiply() * (1 - src.alpha))
          .Unpremultiply();
    case BlendMode::kPlus:
      // r = min(s + d, 1)
      return (Min(src.Premultiply() + dst.Premultiply(), 1)).Unpremultiply();
    case BlendMode::kModulate:
      // r = s*d
      return (src.Premultiply() * dst.Premultiply()).Unpremultiply();
    case BlendMode::kScreen: {
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return s + d - s * d;
      });
    }
    case BlendMode::kOverlay:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        // The same as HardLight, but with the source and destination reversed.
        Vector3 screen_src = 2.0 * d - 1.0;
        Vector3 screen = screen_src + s - screen_src * s;
        return ComponentChoose(s * (2.0 * d),  //
                               screen,         //
                               d,              //
                               0.5);
      });
    case BlendMode::kDarken:
      return DoColorBlend(
          dst, src, [](Vector3 d, Vector3 s) -> Vector3 { return d.Min(s); });
    case BlendMode::kLighten:
      return DoColorBlend(
          dst, src, [](Vector3 d, Vector3 s) -> Vector3 { return d.Max(s); });
    case BlendMode::kColorDodge:
      return DoColorBlendComponents(dst, src, [](Scalar d, Scalar s) -> Scalar {
        if (d < kEhCloseEnough) {
          return 0.0f;
        }
        if (1.0 - s < kEhCloseEnough) {
          return 1.0f;
        }
        return std::min(1.0f, d / (1.0f - s));
      });
    case BlendMode::kColorBurn:
      return DoColorBlendComponents(dst, src, [](Scalar d, Scalar s) -> Scalar {
        if (1.0 - d < kEhCloseEnough) {
          return 1.0f;
        }
        if (s < kEhCloseEnough) {
          return 0.0f;
        }
        return 1.0f - std::min(1.0f, (1.0f - d) / s);
      });
    case BlendMode::kHardLight:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        Vector3 screen_src = 2.0 * s - 1.0;
        Vector3 screen = screen_src + d - screen_src * d;
        return ComponentChoose(d * (2.0 * s),  //
                               screen,         //
                               s,              //
                               0.5);
      });
    case BlendMode::kSoftLight:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        Vector3 D = ComponentChoose(((16.0 * d - 12.0) * d + 4.0) * d,  //
                                    Vector3(std::sqrt(d.x), std::sqrt(d.y),
                                            std::sqrt(d.z)),  //
                                    d,                        //
                                    0.25);
        return ComponentChoose(d - (1.0 - 2.0 * s) * d * (1.0 - d),  //
                               d + (2.0 * s - 1.0) * (D - d),        //
                               s,                                    //
                               0.5);
      });
    case BlendMode::kDifference:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return (d - s).Abs();
      });
    case BlendMode::kExclusion:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return d + s - 2.0f * d * s;
      });
    case BlendMode::kMultiply:
      return DoColorBlend(
          dst, src, [](Vector3 d, Vector3 s) -> Vector3 { return d * s; });
    case BlendMode::kHue: {
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return SetLuminosity(SetSaturation(s, Saturation(d)), Luminosity(d));
      });
    }
    case BlendMode::kSaturation:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return SetLuminosity(SetSaturation(d, Saturation(s)), Luminosity(d));
      });
    case BlendMode::kColor:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return SetLuminosity(s, Luminosity(d));
      });
    case BlendMode::kLuminosity:
      return DoColorBlend(dst, src, [](Vector3 d, Vector3 s) -> Vector3 {
        return SetLuminosity(d, Luminosity(s));
      });
  }
}

Color Color::ApplyColorMatrix(const ColorMatrix& color_matrix) const {
  auto* c = color_matrix.array;
  return Color(
             c[0] * red + c[1] * green + c[2] * blue + c[3] * alpha + c[4],
             c[5] * red + c[6] * green + c[7] * blue + c[8] * alpha + c[9],
             c[10] * red + c[11] * green + c[12] * blue + c[13] * alpha + c[14],
             c[15] * red + c[16] * green + c[17] * blue + c[18] * alpha + c[19])
      .Clamp01();
}

Color Color::LinearToSRGB() const {
  static auto conversion = [](Scalar component) {
    if (component <= 0.0031308) {
      return component * 12.92;
    }
    return 1.055 * pow(component, (1.0 / 2.4)) - 0.055;
  };

  return Color(conversion(red), conversion(green), conversion(blue), alpha);
}

Color Color::SRGBToLinear() const {
  static auto conversion = [](Scalar component) {
    if (component <= 0.04045) {
      return component / 12.92;
    }
    return pow((component + 0.055) / 1.055, 2.4);
  };

  return Color(conversion(red), conversion(green), conversion(blue), alpha);
}

std::string ColorToString(const Color& color) {
  return SPrintF("R=%.1f,G=%.1f,B=%.1f,A=%.1f",  //
                 color.red,                      //
                 color.green,                    //
                 color.blue,                     //
                 color.alpha                     //
  );
}

}  // namespace impeller
