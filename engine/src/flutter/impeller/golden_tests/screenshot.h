// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GOLDEN_TESTS_SCREENSHOT_H_
#define FLUTTER_IMPELLER_GOLDEN_TESTS_SCREENSHOT_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace impeller {
namespace testing {

class Screenshot {
 public:
  virtual ~Screenshot() = default;

  /// Access raw data of the screenshot.
  virtual const uint8_t* GetBytes() const = 0;

  /// Returns the height of the image in pixels.
  virtual size_t GetHeight() const = 0;

  /// Returns the width of the image in pixels.
  virtual size_t GetWidth() const = 0;

  /// Returns number of bytes required to represent one row of the raw image.
  virtual size_t GetBytesPerRow() const = 0;

  /// Synchronously write the screenshot to disk as a PNG at `path`.  Returns
  /// `true` if it succeeded.
  virtual bool WriteToPNG(const std::string& path) const = 0;
};

}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GOLDEN_TESTS_SCREENSHOT_H_
