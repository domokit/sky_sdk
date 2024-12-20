// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_surface.h"

#include <memory>

#import "flutter/shell/platform/darwin/ios/ios_surface_metal_impeller.h"
#import "flutter/shell/platform/darwin/ios/ios_surface_metal_skia.h"
#import "flutter/shell/platform/darwin/ios/ios_surface_noop.h"
#import "flutter/shell/platform/darwin/ios/ios_surface_software.h"
#include "flutter/shell/platform/darwin/ios/rendering_api_selection.h"

FLUTTER_ASSERT_ARC

namespace flutter {

std::unique_ptr<IOSSurface> IOSSurface::Create(std::shared_ptr<IOSContext> context,
                                               CALayer* layer) {
  FML_DCHECK(layer);
  FML_DCHECK(context);

  if (@available(iOS METAL_IOS_VERSION_BASELINE, *)) {
    if ([layer isKindOfClass:[CAMetalLayer class]]) {
      switch (context->GetBackend()) {
        case IOSRenderingBackend::kSkia:
#if !SLIMPELLER
          return std::make_unique<IOSSurfaceMetalSkia>(
              static_cast<CAMetalLayer*>(layer),  // Metal layer
              std::move(context)                  // context
          );
#else   //  !SLIMPELLER
          FML_LOG(FATAL) << "Impeller opt-out unavailable.";
          return nullptr;
#endif  //  !SLIMPELLER
          break;
        case IOSRenderingBackend::kImpeller:
          return std::make_unique<IOSSurfaceMetalImpeller>(
              static_cast<CAMetalLayer*>(layer),  // Metal layer
              std::move(context)                  // context
          );
      }
    }
  }
  if (context->GetBackend() == IOSRenderingBackend::kImpeller) {
    return std::make_unique<IOSSurfaceNoop>(std::move(context));
  }

  return std::make_unique<IOSSurfaceSoftware>(layer,              // layer
                                              std::move(context)  // context
  );
}

IOSSurface::IOSSurface(std::shared_ptr<IOSContext> ios_context)
    : ios_context_(std::move(ios_context)) {
  FML_DCHECK(ios_context_);
}

IOSSurface::~IOSSurface() = default;

std::shared_ptr<IOSContext> IOSSurface::GetContext() const {
  return ios_context_;
}

}  // namespace flutter
