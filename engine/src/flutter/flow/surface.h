// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_SURFACE_H_
#define FLUTTER_FLOW_SURFACE_H_

#include <memory>

#include "flutter/common/graphics/gl_context_switch.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface_frame.h"
#include "flutter/fml/macros.h"

class GrDirectContext;

namespace impeller {
class AiksContext;
}  // namespace impeller

namespace flutter {

/// Abstract Base Class that represents where we will be rendering content.
class Surface {
 public:
  /// A screenshot of the surface's raw data.
  struct SurfaceData {
    std::string pixel_format;
    sk_sp<SkData> data;
  };

  Surface();

  virtual ~Surface();

  virtual bool IsValid() = 0;

  virtual std::unique_ptr<SurfaceFrame> AcquireFrame(const SkISize& size) = 0;

  virtual SkMatrix GetRootTransformation() const = 0;

  virtual GrDirectContext* GetContext() = 0;

  virtual std::unique_ptr<GLContextResult> MakeRenderContextCurrent();

  virtual bool ClearRenderContext();

  virtual bool AllowsDrawingWhenGpuDisabled() const;

  virtual bool EnableRasterCache() const;

  virtual std::shared_ptr<impeller::AiksContext> GetAiksContext() const;

  /// Capture the `SurfaceData` currently present in the surface.
  ///
  /// Not guaranteed to work on all setups and not intended to be used in
  /// production. The data field will be null if it was unable to work.
  virtual SurfaceData GetSurfaceData() const;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(Surface);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_SURFACE_H_
