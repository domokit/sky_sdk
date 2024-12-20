// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/surface.h"

namespace flutter {

Surface::Surface() = default;

Surface::~Surface() = default;

std::unique_ptr<GLContextResult> Surface::MakeRenderContextCurrent() {
  return std::make_unique<GLContextDefaultResult>(true);
}

bool Surface::ClearRenderContext() {
  return false;
}

bool Surface::AllowsDrawingWhenGpuDisabled() const {
  return true;
}

bool Surface::EnableRasterCache() const {
  return true;
}

std::shared_ptr<impeller::AiksContext> Surface::GetAiksContext() const {
  return nullptr;
}

Surface::SurfaceData Surface::GetSurfaceData() const {
  return {};
}

}  // namespace flutter
