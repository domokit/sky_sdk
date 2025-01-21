// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_COLOR_H_
#define FLUTTER_DISPLAY_LIST_DL_COLOR_H_

#include "flutter/display_list/geometry/dl_geometry_types.h"

namespace flutter {

// These should match the enumerations defined in //lib/ui/painting.dart.
enum class DlColorSpace { kSRGB = 0, kExtendedSRGB = 1, kDisplayP3 = 2 };

/// A representation of a color.
///
/// The color belongs to a DlColorSpace. Using deprecated integer data accessors
/// on colors not in the kSRGB colorspace can lead to data loss. Using the
/// floating point accessors and constructors that were added for wide-gamut
/// support are preferred.
struct DlColor {
 public:
  constexpr DlColor()
      : alpha_(1.f),
        red_(0.f),
        green_(0.f),
        blue_(0.f),
        color_space_(DlColorSpace::kSRGB) {}
  constexpr explicit DlColor(uint32_t argb)
      : alpha_(toF((argb >> 24) & 0xff)),
        red_(toF((argb >> 16) & 0xff)),
        green_(toF((argb >> 8) & 0xff)),
        blue_(toF((argb >> 0) & 0xff)),
        color_space_(DlColorSpace::kSRGB) {}
  constexpr DlColor(DlScalar alpha,
                    DlScalar red,
                    DlScalar green,
                    DlScalar blue,
                    DlColorSpace colorspace)
      : alpha_(alpha),
        red_(red),
        green_(green),
        blue_(blue),
        color_space_(colorspace) {}

  /// @brief Construct a 32 bit color from floating point R, G, B, and A color
  /// channels.
  static constexpr DlColor RGBA(DlScalar r,
                                DlScalar g,
                                DlScalar b,
                                DlScalar a) {
    return ARGB(a, r, g, b);
  }

  /// @brief Construct a 32 bit color from floating point A, R, G, and B color
  /// channels.
  static constexpr DlColor ARGB(DlScalar a,
                                DlScalar r,
                                DlScalar g,
                                DlScalar b) {
    return DlColor(a, r, g, b, DlColorSpace::kSRGB);
  }

  static constexpr uint8_t toAlpha(DlScalar opacity) { return toC(opacity); }
  static constexpr DlScalar toOpacity(uint8_t alpha) { return toF(alpha); }

  // clang-format off
  static constexpr DlColor kTransparent()        {return DlColor(0x00000000);};
  static constexpr DlColor kBlack()              {return DlColor(0xFF000000);};
  static constexpr DlColor kWhite()              {return DlColor(0xFFFFFFFF);};
  static constexpr DlColor kRed()                {return DlColor(0xFFFF0000);};
  static constexpr DlColor kGreen()              {return DlColor(0xFF00FF00);};
  static constexpr DlColor kBlue()               {return DlColor(0xFF0000FF);};
  static constexpr DlColor kCyan()               {return DlColor(0xFF00FFFF);};
  static constexpr DlColor kMagenta()            {return DlColor(0xFFFF00FF);};
  static constexpr DlColor kYellow()             {return DlColor(0xFFFFFF00);};
  static constexpr DlColor kDarkGrey()           {return DlColor(0xFF3F3F3F);};
  static constexpr DlColor kMidGrey()            {return DlColor(0xFF808080);};
  static constexpr DlColor kLightGrey()          {return DlColor(0xFFC0C0C0);};
  static constexpr DlColor kAliceBlue()          {return DlColor(0xFFF0F8FF);};
  static constexpr DlColor kFuchsia()            {return DlColor(0xFFFF00FF);};
  static constexpr DlColor kMaroon()             {return DlColor(0xFF800000);};
  static constexpr DlColor kSkyBlue()            {return DlColor(0xFF87CEEB);};
  static constexpr DlColor kCornflowerBlue()     {return DlColor(0xFF6495ED);};
  static constexpr DlColor kCrimson()            {return DlColor(0xFFFF5733);};
  static constexpr DlColor kAqua()               {return DlColor(0xFF00FFFF);};
  static constexpr DlColor kOrange()             {return DlColor(0xFFFFA500);};
  static constexpr DlColor kPurple()             {return DlColor(0xFF800080);};
  static constexpr DlColor kLimeGreen()          {return DlColor(0xFF32CD32);};
  static constexpr DlColor kGreenYellow()        {return DlColor(0xFFADFF2F);};
  static constexpr DlColor kDarkMagenta()        {return DlColor(0xFF8B008B);};
  static constexpr DlColor kOrangeRed()          {return DlColor(0xFFFF4500);};
  static constexpr DlColor kDarkGreen()          {return DlColor(0xFF006400);};
  static constexpr DlColor kChartreuse()         {return DlColor(0xFF7FFF00);};
  // clang-format on

  constexpr bool isOpaque() const { return alpha_ >= 1.f; }
  constexpr bool isTransparent() const { return alpha_ <= 0.f; }

  ///\deprecated Use floating point accessors to avoid data loss when using wide
  /// gamut colors.
  constexpr int getAlpha() const { return toC(alpha_); }
  ///\deprecated Use floating point accessors to avoid data loss when using wide
  /// gamut colors.
  constexpr int getRed() const { return toC(red_); }
  ///\deprecated Use floating point accessors to avoid data loss when using wide
  /// gamut colors.
  constexpr int getGreen() const { return toC(green_); }
  ///\deprecated Use floating point accessors to avoid data loss when using wide
  /// gamut colors.
  constexpr int getBlue() const { return toC(blue_); }

  constexpr DlScalar getAlphaF() const { return alpha_; }
  constexpr DlScalar getRedF() const { return red_; }
  constexpr DlScalar getGreenF() const { return green_; }
  constexpr DlScalar getBlueF() const { return blue_; }

  constexpr DlColorSpace getColorSpace() const { return color_space_; }

  constexpr DlColor withAlpha(uint8_t alpha) const {  //
    return DlColor((argb() & 0x00FFFFFF) | (alpha << 24));
  }
  constexpr DlColor withRed(uint8_t red) const {  //
    return DlColor((argb() & 0xFF00FFFF) | (red << 16));
  }
  constexpr DlColor withGreen(uint8_t green) const {  //
    return DlColor((argb() & 0xFFFF00FF) | (green << 8));
  }
  constexpr DlColor withBlue(uint8_t blue) const {  //
    return DlColor((argb() & 0xFFFFFF00) | (blue << 0));
  }
  constexpr DlColor withAlphaF(float alpha) const {  //
    return DlColor(alpha, red_, green_, blue_, color_space_);
  }
  constexpr DlColor withRedF(float red) const {  //
    return DlColor(alpha_, red, green_, blue_, color_space_);
  }
  constexpr DlColor withGreenF(float green) const {  //
    return DlColor(alpha_, red_, green, blue_, color_space_);
  }
  constexpr DlColor withBlueF(float blue) const {  //
    return DlColor(alpha_, red_, green_, blue, color_space_);
  }
  /// Performs a colorspace transformation.
  ///
  /// This isn't just a replacement of the color space field, the new color
  /// components are calculated.
  DlColor withColorSpace(DlColorSpace color_space) const;

  constexpr DlColor modulateOpacity(DlScalar opacity) const {
    return opacity <= 0   ? withAlpha(0)
           : opacity >= 1 ? *this
                          : withAlpha(round(getAlpha() * opacity));
  }

  ///\deprecated Use floating point accessors to avoid data loss when using wide
  /// gamut colors.
  constexpr uint32_t argb() const {
    return toC(alpha_) << 24 |  //
           toC(red_) << 16 |    //
           toC(green_) << 8 |   //
           toC(blue_) << 0;
  }

  /// Checks that no difference in color components exceeds the delta.
  ///
  /// This doesn't check against the actual distance between the colors in some
  /// space.
  bool isClose(DlColor const& other, DlScalar delta = 1.0f / 256.0f) {
    return color_space_ == other.color_space_ &&
           std::abs(alpha_ - other.alpha_) < delta &&
           std::abs(red_ - other.red_) < delta &&
           std::abs(green_ - other.green_) < delta &&
           std::abs(blue_ - other.blue_) < delta;
  }
  bool operator==(DlColor const& other) const {
    return alpha_ == other.alpha_ && red_ == other.red_ &&
           green_ == other.green_ && blue_ == other.blue_ &&
           color_space_ == other.color_space_;
  }
  bool operator!=(DlColor const& other) const {
    return !this->operator==(other);
  }

 private:
  DlScalar alpha_;
  DlScalar red_;
  DlScalar green_;
  DlScalar blue_;
  DlColorSpace color_space_;

  static constexpr DlScalar toF(uint8_t comp) { return comp * (1.0f / 255); }
  static constexpr uint8_t toC(DlScalar fComp) { return round(fComp * 255); }
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_COLOR_H_
