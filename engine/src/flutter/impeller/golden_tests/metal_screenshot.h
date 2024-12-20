// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GOLDEN_TESTS_METAL_SCREENSHOT_H_
#define FLUTTER_IMPELLER_GOLDEN_TESTS_METAL_SCREENSHOT_H_

#include "flutter/impeller/golden_tests/screenshot.h"

#include <CoreFoundation/CoreFoundation.h>
#include <CoreImage/CoreImage.h>
#include <string>

#include "flutter/fml/platform/darwin/cf_utils.h"

namespace fml {

/// fml::CFRef retain and release implementations for CGImageRef.
template <>
struct CFRefTraits<CGImageRef> {
  static constexpr CGImageRef kNullValue = nullptr;
  static void Retain(CGImageRef instance) { CGImageRetain(instance); }
  static void Release(CGImageRef instance) { CGImageRelease(instance); }
};

}  // namespace fml

namespace impeller {
namespace testing {

/// A screenshot that was produced from `MetalScreenshotter`.
class MetalScreenshot : public Screenshot {
 public:
  explicit MetalScreenshot(CGImageRef cgImage);

  ~MetalScreenshot();

  const uint8_t* GetBytes() const override;

  size_t GetHeight() const override;

  size_t GetWidth() const override;

  size_t GetBytesPerRow() const override;

  bool WriteToPNG(const std::string& path) const override;

 private:
  MetalScreenshot(const MetalScreenshot&) = delete;

  MetalScreenshot& operator=(const MetalScreenshot&) = delete;
  fml::CFRef<CGImageRef> cg_image_;
  fml::CFRef<CFDataRef> pixel_data_;
};
}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GOLDEN_TESTS_METAL_SCREENSHOT_H_
