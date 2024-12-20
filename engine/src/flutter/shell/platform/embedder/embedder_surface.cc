// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_surface.h"

namespace flutter {

EmbedderSurface::EmbedderSurface() = default;

EmbedderSurface::~EmbedderSurface() = default;

std::shared_ptr<impeller::Context> EmbedderSurface::CreateImpellerContext()
    const {
  return nullptr;
}

sk_sp<GrDirectContext> EmbedderSurface::CreateResourceContext() const {
  return nullptr;
}

}  // namespace flutter
