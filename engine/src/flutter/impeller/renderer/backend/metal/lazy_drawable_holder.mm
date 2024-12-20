// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/lazy_drawable_holder.h"

#include <QuartzCore/CAMetalLayer.h>
#include <future>
#include <memory>

#include "flutter/fml/trace_event.h"
#include "impeller/base/validation.h"

namespace impeller {

#pragma GCC diagnostic push
// Disable the diagnostic for iOS Simulators. Metal without emulation isn't
// available prior to iOS 13 and that's what the simulator headers say when
// support for CAMetalLayer begins. CAMetalLayer is available on iOS 8.0 and
// above which is well below Flutters support level.
#pragma GCC diagnostic ignored "-Wunguarded-availability-new"

std::shared_future<id<CAMetalDrawable>> GetDrawableDeferred(
    CAMetalLayer* layer) {
  auto future =
      std::async(std::launch::deferred, [layer]() -> id<CAMetalDrawable> {
        id<CAMetalDrawable> current_drawable = nil;
        {
          TRACE_EVENT0("impeller", "WaitForNextDrawable");
          current_drawable = [layer nextDrawable];
        }
        if (!current_drawable) {
          VALIDATION_LOG << "Could not acquire current drawable.";
          return nullptr;
        }
        return current_drawable;
      });
  return std::shared_future<id<CAMetalDrawable>>(std::move(future));
}

std::shared_ptr<TextureMTL> CreateTextureFromDrawableFuture(
    TextureDescriptor desc,
    const std::shared_future<id<CAMetalDrawable>>& drawble_future) {
  return std::make_shared<TextureMTL>(
      desc, [drawble_future]() { return drawble_future.get().texture; },
      /*wrapped=*/false, /*drawable=*/true);
}

#pragma GCC diagnostic pop

}  // namespace impeller
