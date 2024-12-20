// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_LIBRARY_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_LIBRARY_GLES_H_

#include "impeller/core/sampler.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/renderer/backend/gles/sampler_gles.h"
#include "impeller/renderer/sampler_library.h"

namespace impeller {

class SamplerLibraryGLES final : public SamplerLibrary {
 public:
  explicit SamplerLibraryGLES(bool supports_decal_sampler_address_mode);
  // |SamplerLibrary|
  ~SamplerLibraryGLES() override;

 private:
  friend class ContextGLES;

  std::vector<std::pair<uint64_t, std::shared_ptr<const Sampler>>> samplers_;

  SamplerLibraryGLES();

  // |SamplerLibrary|
  raw_ptr<const Sampler> GetSampler(
      const SamplerDescriptor& descriptor) override;

  bool supports_decal_sampler_address_mode_ = false;

  SamplerLibraryGLES(const SamplerLibraryGLES&) = delete;

  SamplerLibraryGLES& operator=(const SamplerLibraryGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SAMPLER_LIBRARY_GLES_H_
