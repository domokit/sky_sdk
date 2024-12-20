// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_SCREENSHOT_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_SCREENSHOT_H_

#include <lib/zx/vmo.h>
#include <zircon/status.h>

#include <cmath>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>

namespace fuchsia_test_utils {
// Represents a Pixel in BGRA format.
// Uses the sRGB color space.
struct Pixel {
  uint8_t blue = 0;
  uint8_t green = 0;
  uint8_t red = 0;
  uint8_t alpha = 0;

  Pixel(uint8_t blue, uint8_t green, uint8_t red, uint8_t alpha)
      : blue(blue), green(green), red(red), alpha(alpha) {}

  bool operator==(const Pixel& rhs) const {
    return blue == rhs.blue && green == rhs.green && red == rhs.red &&
           alpha == rhs.alpha;
  }

  inline bool operator!=(const Pixel& rhs) const { return !(*this == rhs); }

  bool operator<(const Pixel& other) const {
    return std::tie(blue, green, red, alpha) <
           std::tie(other.blue, other.green, other.red, other.alpha);
  }
};

std::ostream& operator<<(std::ostream& stream, const Pixel& pixel);

// Helper class to get information about a screenshot returned by
// |fuchsia.ui.composition.Screenshot| protocol.
class Screenshot {
 public:
  // BGRA format.
  inline static const Pixel kBlack = Pixel(0, 0, 0, 255);
  inline static const Pixel kBlue = Pixel(255, 0, 0, 255);
  inline static const Pixel kRed = Pixel(0, 0, 255, 255);
  inline static const Pixel kMagenta = Pixel(255, 0, 255, 255);
  inline static const Pixel kGreen = Pixel(0, 255, 0, 255);

  // Params:-
  // |screenshot_vmo| - The VMO returned by
  //    fuchsia.ui.composition.Screenshot.Take representing the screenshot data.
  // |width|, |height| - Width and height of the physical display in pixels as
  //    returned by |fuchsia.ui.display.singleton.Info|.
  // |rotation| - The display rotation value in degrees. The width and the
  //    height of the screenshot are flipped if this value is 90 or 270 degrees,
  //    as the screenshot shows how content is seen by the user.
  Screenshot(const zx::vmo& screenshot_vmo,
             uint64_t width,
             uint64_t height,
             int rotation);

  // Returns the |Pixel| located at (x,y) coordinates. |x| and |y| should range
  // from [0,width_) and [0,height_) respectively.
  //
  //  (0,0)________________width_____________(w-1,0)
  //      |                       |         |
  //      |                       | y       |h
  //      |          x            |         |e
  //      |-----------------------X         |i
  //      |                                 |g
  //      |                                 |h
  //      |                                 |t
  //      |_________________________________|
  // (0,h-1)           screenshot             (w-1,h-1)
  //
  // Clients should only use this function to get the pixel data.
  Pixel GetPixelAt(uint64_t x, uint64_t y) const;

  // Counts the frequencies of each color in a screenshot.
  std::map<Pixel, uint32_t> Histogram() const;

  // Returns a 2D vector of size |height_ * width_|. Each value in the vector
  // corresponds to a pixel in the screenshot.
  std::vector<std::vector<Pixel>> screenshot() const { return screenshot_; }

  uint64_t width() const { return width_; }

  uint64_t height() const { return height_; }

 private:
  // Populates |screenshot_| by converting the linear array of bytes in
  // |screenshot_vmo| of size |kBytesPerPixel * width_ * height_| to a 2D vector
  //  of |Pixel|s of size |height_ * width_|.
  void ExtractScreenshotFromVMO(uint8_t* screenshot_vmo);

  // Returns the |Pixel|s in the |row_index| row of the screenshot.
  std::vector<Pixel> GetPixelsInRow(uint8_t* screenshot_vmo, size_t row_index);

  uint64_t width_ = 0;
  uint64_t height_ = 0;
  std::vector<std::vector<Pixel>> screenshot_;
};

}  // namespace fuchsia_test_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_SCREENSHOT_H_
