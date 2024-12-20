// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_NOOP_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_NOOP_H_

#import "flutter/shell/platform/darwin/ios/ios_context.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

@class CALayer;

namespace flutter {

/// @brief A rendering surface that accepts rendering intent but does not render
///        anything.
///
/// This is useful for running on platforms that need an engine instance and
/// don't have the required drivers.
class IOSSurfaceNoop final : public IOSSurface {
 public:
  explicit IOSSurfaceNoop(std::shared_ptr<IOSContext> context);

  ~IOSSurfaceNoop() override;

  // |IOSSurface|
  bool IsValid() const override;

  // |IOSSurface|
  void UpdateStorageSizeIfNecessary() override;

  // |IOSSurface|
  std::unique_ptr<Surface> CreateGPUSurface(GrDirectContext* gr_context = nullptr) override;

 private:
  IOSSurfaceNoop(const IOSSurfaceNoop&) = delete;

  IOSSurfaceNoop& operator=(const IOSSurfaceNoop&) = delete;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_NOOP_H_
