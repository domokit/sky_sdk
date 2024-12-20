// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_SOFTWARE_H_

#include "flutter/flow/embedded_views.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_software.h"
#import "flutter/shell/platform/darwin/ios/ios_context.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

@class CALayer;

namespace flutter {

class IOSSurfaceSoftware final : public IOSSurface, public GPUSurfaceSoftwareDelegate {
 public:
  IOSSurfaceSoftware(CALayer* layer, std::shared_ptr<IOSContext> context);

  ~IOSSurfaceSoftware() override;

  // |IOSSurface|
  bool IsValid() const override;

  // |IOSSurface|
  void UpdateStorageSizeIfNecessary() override;

  // |IOSSurface|
  std::unique_ptr<Surface> CreateGPUSurface(GrDirectContext* gr_context = nullptr) override;

  // |GPUSurfaceSoftwareDelegate|
  sk_sp<SkSurface> AcquireBackingStore(const SkISize& size) override;

  // |GPUSurfaceSoftwareDelegate|
  bool PresentBackingStore(sk_sp<SkSurface> backing_store) override;

 private:
  CALayer* layer_;
  sk_sp<SkSurface> sk_surface_;

  FML_DISALLOW_COPY_AND_ASSIGN(IOSSurfaceSoftware);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_SOFTWARE_H_
