// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_surface_noop.h"
#include "shell/gpu/gpu_surface_noop.h"

#include <QuartzCore/CALayer.h>

#include <memory>

#include "flutter/fml/logging.h"
#include "flutter/fml/platform/darwin/cf_utils.h"
#include "flutter/fml/trace_event.h"

#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/utils/mac/SkCGUtils.h"

FLUTTER_ASSERT_ARC

namespace flutter {

IOSSurfaceNoop::IOSSurfaceNoop(std::shared_ptr<IOSContext> context)
    : IOSSurface(std::move(context)) {}

IOSSurfaceNoop::~IOSSurfaceNoop() = default;

bool IOSSurfaceNoop::IsValid() const {
  return true;
}

void IOSSurfaceNoop::UpdateStorageSizeIfNecessary() {}

std::unique_ptr<Surface> IOSSurfaceNoop::CreateGPUSurface(GrDirectContext* gr_context) {
  return std::make_unique<GPUSurfaceNoop>();
}

}  // namespace flutter
