// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_LIBRARY_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_LIBRARY_MTL_H_

#include <Metal/Metal.h>

#include "impeller/renderer/pipeline_library.h"

namespace impeller {

class ContextMTL;

class PipelineLibraryMTL final : public PipelineLibrary {
 public:
  PipelineLibraryMTL();

  // |PipelineLibrary|
  ~PipelineLibraryMTL() override;

 private:
  friend ContextMTL;

  id<MTLDevice> device_ = nullptr;
  PipelineMap pipelines_;
  ComputePipelineMap compute_pipelines_;

  explicit PipelineLibraryMTL(id<MTLDevice> device);

  // |PipelineLibrary|
  bool IsValid() const override;

  // |PipelineLibrary|
  PipelineFuture<PipelineDescriptor> GetPipeline(PipelineDescriptor descriptor,
                                                 bool async) override;

  // |PipelineLibrary|
  PipelineFuture<ComputePipelineDescriptor> GetPipeline(
      ComputePipelineDescriptor descriptor,
      bool async) override;

  // |PipelineLibrary|
  bool HasPipeline(const PipelineDescriptor& descriptor) override;

  // |PipelineLibrary|
  void RemovePipelinesWithEntryPoint(
      std::shared_ptr<const ShaderFunction> function) override;

  PipelineLibraryMTL(const PipelineLibraryMTL&) = delete;

  PipelineLibraryMTL& operator=(const PipelineLibraryMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_LIBRARY_MTL_H_
