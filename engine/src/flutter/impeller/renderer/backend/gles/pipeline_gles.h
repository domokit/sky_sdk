// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PIPELINE_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PIPELINE_GLES_H_

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/backend/gles/buffer_bindings_gles.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"
#include "impeller/renderer/backend/gles/unique_handle_gles.h"
#include "impeller/renderer/pipeline.h"

namespace impeller {

class PipelineLibraryGLES;

class PipelineGLES final
    : public Pipeline<PipelineDescriptor>,
      public BackendCast<PipelineGLES, Pipeline<PipelineDescriptor>> {
 public:
  // |Pipeline|
  ~PipelineGLES() override;

  const HandleGLES& GetProgramHandle() const;

  const std::shared_ptr<UniqueHandleGLES> GetSharedHandle() const;

  [[nodiscard]] bool BindProgram() const;

  [[nodiscard]] bool UnbindProgram() const;

  BufferBindingsGLES* GetBufferBindings() const;

  [[nodiscard]] bool BuildVertexDescriptor(const ProcTableGLES& gl,
                                           GLuint program);

 private:
  friend PipelineLibraryGLES;

  std::shared_ptr<ReactorGLES> reactor_;
  std::shared_ptr<UniqueHandleGLES> handle_;
  std::unique_ptr<BufferBindingsGLES> buffer_bindings_;
  bool is_valid_ = false;

  // |Pipeline|
  bool IsValid() const override;

  PipelineGLES(std::shared_ptr<ReactorGLES> reactor,
               std::weak_ptr<PipelineLibrary> library,
               const PipelineDescriptor& desc,
               std::shared_ptr<UniqueHandleGLES> handle);

  PipelineGLES(const PipelineGLES&) = delete;

  PipelineGLES& operator=(const PipelineGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PIPELINE_GLES_H_
