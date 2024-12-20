// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_BLEND_MODE_H_
#define FLUTTER_DISPLAY_LIST_DL_BLEND_MODE_H_

namespace flutter {

/// A enum define the blend mode.
/// Blends are operators that take in two colors (source, destination) and
/// return a new color. Many of these operate the same on all 4
/// components: red, green, blue, alpha. For these, we just document what
/// happens to one component, rather than naming each one separately. Different
/// color types might have different representations for color components:
///     8-bit: 0..255
///     6-bit: 0..63
///     5-bit: 0..31
///     4-bit: 0..15
///    floats: 0...1
/// The comments are expressed as if the component values are always 0..1
/// (floats). For brevity, the documentation uses the following abbreviations s
/// : source d  : destination sa : source alpha da : destination alpha Results
/// are abbreviated r  : if all 4 components are computed in the same manner ra
/// : result alpha component rc : result "color": red, green, blue components
enum class DlBlendMode {
  kClear,     //!< r = 0
  kSrc,       //!< r = s
  kDst,       //!< r = d
  kSrcOver,   //!< r = s + (1-sa)*d
  kDstOver,   //!< r = d + (1-da)*s
  kSrcIn,     //!< r = s * da
  kDstIn,     //!< r = d * sa
  kSrcOut,    //!< r = s * (1-da)
  kDstOut,    //!< r = d * (1-sa)
  kSrcATop,   //!< r = s*da + d*(1-sa)
  kDstATop,   //!< r = d*sa + s*(1-da)
  kXor,       //!< r = s*(1-da) + d*(1-sa)
  kPlus,      //!< r = min(s + d, 1)
  kModulate,  //!< r = s*d
  kScreen,    //!< r = s + d - s*d

  kOverlay,     //!< multiply or screen, depending on destination
  kDarken,      //!< rc = s + d - max(s*da, d*sa), ra = kSrcOver
  kLighten,     //!< rc = s + d - min(s*da, d*sa), ra = kSrcOver
  kColorDodge,  //!< brighten destination to reflect source
  kColorBurn,   //!< darken destination to reflect source
  kHardLight,   //!< multiply or screen, depending on source
  kSoftLight,   //!< lighten or darken, depending on source
  kDifference,  //!< rc = s + d - 2*(min(s*da, d*sa)), ra = kSrcOver
  kExclusion,   //!< rc = s + d - two(s*d), ra = kSrcOver
  kMultiply,    //!< r = s*(1-da) + d*(1-sa) + s*d

  kHue,         //!< hue of source with saturation and luminosity of destination
  kSaturation,  //!< saturation of source with hue and luminosity of destination
  kColor,       //!< hue and saturation of source with luminosity of destination
  kLuminosity,  //!< luminosity of source with hue and saturation of destination

  kLastCoeffMode = kScreen,  //!< last porter duff blend mode
  kLastSeparableMode =
      kMultiply,  //!< last blend mode operating separately on components
  kLastMode = kLuminosity,  //!< last valid value
  kDefaultMode = kSrcOver,
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_BLEND_MODE_H_
