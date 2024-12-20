// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_H_

#include <memory>
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/macros.h"

namespace flutter {

class EmbedderSurface {
 public:
  EmbedderSurface();

  virtual ~EmbedderSurface();

  virtual bool IsValid() const = 0;

  virtual std::unique_ptr<Surface> CreateGPUSurface() = 0;

  virtual std::shared_ptr<impeller::Context> CreateImpellerContext() const;

  virtual sk_sp<GrDirectContext> CreateResourceContext() const;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderSurface);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_H_
