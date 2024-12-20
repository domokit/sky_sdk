// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_GLES_H_

#include "impeller/base/backend_cast.h"
#include "impeller/core/sampler.h"

namespace impeller {

class TextureGLES;
class SamplerLibraryGLES;
class ProcTableGLES;

class SamplerGLES final : public Sampler,
                          public BackendCast<SamplerGLES, Sampler> {
 public:
  ~SamplerGLES();

  bool ConfigureBoundTexture(const TextureGLES& texture,
                             const ProcTableGLES& gl) const;

 private:
  friend class SamplerLibraryGLES;

  explicit SamplerGLES(const SamplerDescriptor&);

  SamplerGLES(const SamplerGLES&) = delete;

  SamplerGLES& operator=(const SamplerGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_GLES_H_
