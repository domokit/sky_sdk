// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BLENDING_GLSL_
#define BLENDING_GLSL_

#include <impeller/branching.glsl>
#include <impeller/color.glsl>
#include <impeller/constants.glsl>
#include <impeller/types.glsl>

/// Composite a blended color onto the destination.
/// All three parameters are unpremultiplied. Returns a premultiplied result.
///
/// This routine is the same as `ApplyBlendedColor` in
/// `impeller/geometry/color.cc`.
f16vec4 IPApplyBlendedColor(f16vec4 dst, f16vec4 src, f16vec3 blend_result) {
  dst = IPHalfPremultiply(dst);
  src =
      // Use the blended color for areas where the source and destination
      // colors overlap.
      IPHalfPremultiply(f16vec4(blend_result, src.a * dst.a)) +
      // Use the original source color for any remaining non-overlapping areas.
      IPHalfPremultiply(src) * (1.0hf - dst.a);

  // Source-over composite the blended source color atop the destination.
  return src + dst * (1.0hf - src.a);
}

//------------------------------------------------------------------------------
/// HSV utilities.
///

float16_t IPLuminosity(f16vec3 color) {
  return color.r * 0.3hf + color.g * 0.59hf + color.b * 0.11hf;
}

/// Scales the color's luma by the amount necessary to place the color
/// components in a 1-0 range.
f16vec3 IPClipColor(f16vec3 color) {
  float16_t lum = IPLuminosity(color);
  float16_t mn = min(min(color.r, color.g), color.b);
  float16_t mx = max(max(color.r, color.g), color.b);
  // `lum - mn` and `mx - lum` will always be >= 0 in the following conditions,
  // so adding a tiny value is enough to make these divisions safe.
  if (mn < 0.0hf) {
    color = lum + (((color - lum) * lum) / (lum - mn + kEhCloseEnoughHalf));
  }
  if (mx > 1.0hf) {
    color = lum +
            (((color - lum) * (1.0hf - lum)) / (mx - lum + kEhCloseEnoughHalf));
  }
  return color;
}

f16vec3 IPSetLuminosity(f16vec3 color, float16_t luminosity) {
  float16_t relative_lum = luminosity - IPLuminosity(color);
  return IPClipColor(color + relative_lum);
}

float16_t IPSaturation(f16vec3 color) {
  return max(max(color.r, color.g), color.b) -
         min(min(color.r, color.g), color.b);
}

f16vec3 IPSetSaturation(f16vec3 color, float16_t saturation) {
  float16_t mn = min(min(color.r, color.g), color.b);
  float16_t mx = max(max(color.r, color.g), color.b);
  return (mn < mx) ? ((color - mn) * saturation) / (mx - mn) : f16vec3(0.0hf);
}

//------------------------------------------------------------------------------
/// Color blend functions.
///
/// These routines take two unpremultiplied RGB colors and output a third color.
/// They can be combined with any alpha compositing operation. When these blend
/// functions are used for drawing Entities in Impeller, the output is always
/// applied to the destination using `SourceOver` alpha compositing.
///

f16vec3 IPBlendScreen(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingscreen
  return dst + src - (dst * src);
}

f16vec3 IPBlendHardLight(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendinghardlight
  return IPHalfVec3Choose(dst * (2.0hf * src),
                          IPBlendScreen(dst, 2.0hf * src - 1.0hf), src);
}

f16vec3 IPBlendOverlay(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingoverlay
  // HardLight, but with reversed parameters.
  return IPBlendHardLight(src, dst);
}

f16vec3 IPBlendDarken(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingdarken
  return min(dst, src);
}

f16vec3 IPBlendLighten(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendinglighten
  return max(dst, src);
}

f16vec3 IPBlendColorDodge(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingcolordodge

  f16vec3 color = min(f16vec3(1.0hf), dst / (1.0hf - src));

  if (dst.r < kEhCloseEnoughHalf) {
    color.r = 0.0hf;
  }
  if (dst.g < kEhCloseEnoughHalf) {
    color.g = 0.0hf;
  }
  if (dst.b < kEhCloseEnoughHalf) {
    color.b = 0.0hf;
  }

  if (1.0hf - src.r < kEhCloseEnoughHalf) {
    color.r = 1.0hf;
  }
  if (1.0hf - src.g < kEhCloseEnoughHalf) {
    color.g = 1.0hf;
  }
  if (1.0hf - src.b < kEhCloseEnoughHalf) {
    color.b = 1.0hf;
  }

  return color;
}

f16vec3 IPBlendColorBurn(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingcolorburn

  f16vec3 color = 1.0hf - min(f16vec3(1.0hf), (1.0hf - dst) / src);

  if (1.0hf - dst.r < kEhCloseEnoughHalf) {
    color.r = 1.0hf;
  }
  if (1.0hf - dst.g < kEhCloseEnoughHalf) {
    color.g = 1.0hf;
  }
  if (1.0hf - dst.b < kEhCloseEnoughHalf) {
    color.b = 1.0hf;
  }

  if (src.r < kEhCloseEnoughHalf) {
    color.r = 0.0hf;
  }
  if (src.g < kEhCloseEnoughHalf) {
    color.g = 0.0hf;
  }
  if (src.b < kEhCloseEnoughHalf) {
    color.b = 0.0hf;
  }

  return color;
}

f16vec3 IPBlendSoftLight(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingsoftlight

  f16vec3 D =
      IPHalfVec3ChooseCutoff(((16.0hf * dst - 12.0hf) * dst + 4.0hf) * dst,  //
                             sqrt(dst),                                      //
                             dst,                                            //
                             0.25hf);

  return IPHalfVec3Choose(dst - (1.0hf - 2.0hf * src) * dst * (1.0hf - dst),  //
                          dst + (2.0hf * src - 1.0hf) * (D - dst),            //
                          src);
}

f16vec3 IPBlendDifference(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingdifference
  return abs(dst - src);
}

f16vec3 IPBlendExclusion(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingexclusion
  return dst + src - 2.0hf * dst * src;
}

f16vec3 IPBlendMultiply(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingmultiply
  return dst * src;
}

f16vec3 IPBlendHue(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendinghue
  return IPSetLuminosity(IPSetSaturation(src, IPSaturation(dst)),
                         IPLuminosity(dst));
}

f16vec3 IPBlendSaturation(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingsaturation
  return IPSetLuminosity(IPSetSaturation(dst, IPSaturation(src)),
                         IPLuminosity(dst));
}

f16vec3 IPBlendColor(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingcolor
  return IPSetLuminosity(src, IPLuminosity(dst));
}

f16vec3 IPBlendLuminosity(f16vec3 dst, f16vec3 src) {
  // https://www.w3.org/TR/compositing-1/#blendingluminosity
  return IPSetLuminosity(dst, IPLuminosity(src));
}

#endif
