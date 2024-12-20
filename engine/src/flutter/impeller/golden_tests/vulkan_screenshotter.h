// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GOLDEN_TESTS_VULKAN_SCREENSHOTTER_H_
#define FLUTTER_IMPELLER_GOLDEN_TESTS_VULKAN_SCREENSHOTTER_H_

#include "flutter/impeller/golden_tests/metal_screenshot.h"
#include "flutter/impeller/golden_tests/screenshotter.h"
#include "flutter/impeller/playground/playground_impl.h"

namespace impeller {
namespace testing {

/// Converts `Picture`s and `DisplayList`s to `MetalScreenshot`s with the
/// playground backend.
class VulkanScreenshotter : public Screenshotter {
 public:
  explicit VulkanScreenshotter(
      const std::unique_ptr<PlaygroundImpl>& playground);

  std::unique_ptr<Screenshot> MakeScreenshot(
      AiksContext& aiks_context,
      const std::shared_ptr<Texture> texture) override;

  PlaygroundImpl& GetPlayground() override { return *playground_; }

 private:
  const std::unique_ptr<PlaygroundImpl>& playground_;
};

}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GOLDEN_TESTS_VULKAN_SCREENSHOTTER_H_
