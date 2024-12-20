// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/shell/common/context_options.h"

#include "flutter/common/graphics/persistent_cache.h"

namespace flutter {

GrContextOptions MakeDefaultContextOptions(ContextType type,
                                           std::optional<GrBackendApi> api) {
  GrContextOptions options;

  if (PersistentCache::cache_sksl()) {
    options.fShaderCacheStrategy = GrContextOptions::ShaderCacheStrategy::kSkSL;
  }
  PersistentCache::MarkStrategySet();
  options.fPersistentCache = PersistentCache::GetCacheForProcess();

  if (api.has_value() && api.value() == GrBackendApi::kOpenGL) {
    // Using stencil buffers has caused memory and performance regressions.
    // See b/226484927 for internal customer regressions doc.
    // Before enabling, we need to show a motivating case for where it will
    // improve performance on OpenGL backend.
    options.fAvoidStencilBuffers = true;

    // To get video playback on the widest range of devices, we limit Skia to
    // ES2 shading language when the ES3 external image extension is missing.
    options.fPreferExternalImagesOverES3 = true;
  }

  // TODO(goderbauer): remove option when skbug.com/7523 is fixed.
  options.fDisableGpuYUVConversion = true;

  options.fReduceOpsTaskSplitting = GrContextOptions::Enable::kNo;

  options.fReducedShaderVariations = false;

  return options;
};

}  // namespace flutter

#endif  //  !SLIMPELLER
