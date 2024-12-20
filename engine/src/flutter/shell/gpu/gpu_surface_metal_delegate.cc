// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"

namespace flutter {

GPUSurfaceMetalDelegate::GPUSurfaceMetalDelegate(
    MTLRenderTargetType render_target_type)
    : render_target_type_(render_target_type) {}

GPUSurfaceMetalDelegate::~GPUSurfaceMetalDelegate() = default;

MTLRenderTargetType GPUSurfaceMetalDelegate::GetRenderTargetType() {
  return render_target_type_;
}

bool GPUSurfaceMetalDelegate::AllowsDrawingWhenGpuDisabled() const {
  return true;
}

}  // namespace flutter
