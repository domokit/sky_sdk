// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_SAMPLER_LIBRARY_H_
#define FLUTTER_IMPELLER_RENDERER_SAMPLER_LIBRARY_H_

#include "impeller/core/raw_ptr.h"
#include "impeller/core/sampler.h"
#include "impeller/core/sampler_descriptor.h"

namespace impeller {

class SamplerLibrary {
 public:
  virtual ~SamplerLibrary();

  /// @brief Retrieve a backend specific sampler object for the given sampler
  ///        descriptor.
  ///
  ///        If the descriptor is invalid or there is a loss of rendering
  ///        context, this method may return a nullptr.
  ///
  ///        The sampler library implementations must cache this sampler object
  ///        and guarantee that the reference will continue to be valid
  ///        throughout the lifetime of the Impeller context.
  virtual raw_ptr<const Sampler> GetSampler(
      const SamplerDescriptor& descriptor) = 0;

 protected:
  SamplerLibrary();

 private:
  SamplerLibrary(const SamplerLibrary&) = delete;

  SamplerLibrary& operator=(const SamplerLibrary&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_SAMPLER_LIBRARY_H_
