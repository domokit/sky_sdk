// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_surface_gl_impeller.h"

#include <utility>

#include "impeller/entity/gles/entity_shaders_gles.h"
#include "impeller/entity/gles/framebuffer_blend_shaders_gles.h"
#include "impeller/entity/gles/modern_shaders_gles.h"
#include "impeller/renderer/backend/gles/context_gles.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"

namespace flutter {

class ReactorWorker final : public impeller::ReactorGLES::Worker {
 public:
  ReactorWorker() = default;

  // |ReactorGLES::Worker|
  bool CanReactorReactOnCurrentThreadNow(
      const impeller::ReactorGLES& reactor) const override {
    impeller::ReaderLock lock(mutex_);
    auto found = reactions_allowed_.find(std::this_thread::get_id());
    if (found == reactions_allowed_.end()) {
      return false;
    }
    return found->second;
  }

  void SetReactionsAllowedOnCurrentThread(bool allowed) {
    impeller::WriterLock lock(mutex_);
    reactions_allowed_[std::this_thread::get_id()] = allowed;
  }

 private:
  mutable impeller::RWMutex mutex_;
  std::map<std::thread::id, bool> reactions_allowed_ IPLR_GUARDED_BY(mutex_);

  FML_DISALLOW_COPY_AND_ASSIGN(ReactorWorker);
};

EmbedderSurfaceGLImpeller::EmbedderSurfaceGLImpeller(
    EmbedderSurfaceGLSkia::GLDispatchTable gl_dispatch_table,
    bool fbo_reset_after_present,
    std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder)
    : gl_dispatch_table_(std::move(gl_dispatch_table)),
      fbo_reset_after_present_(fbo_reset_after_present),
      external_view_embedder_(std::move(external_view_embedder)),
      worker_(std::make_shared<ReactorWorker>()) {
  // Make sure all required members of the dispatch table are checked.
  if (!gl_dispatch_table_.gl_make_current_callback ||
      !gl_dispatch_table_.gl_clear_current_callback ||
      !gl_dispatch_table_.gl_present_callback ||
      !gl_dispatch_table_.gl_fbo_callback ||
      !gl_dispatch_table_.gl_populate_existing_damage ||
      !gl_dispatch_table_.gl_proc_resolver) {
    return;
  }
  // Certain GL backends need to made current before any GL
  // state can be accessed.
  gl_dispatch_table_.gl_make_current_callback();

  std::vector<std::shared_ptr<fml::Mapping>> shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_entity_shaders_gles_data,
          impeller_entity_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_modern_shaders_gles_data,
          impeller_modern_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_gles_data,
          impeller_framebuffer_blend_shaders_gles_length),
  };
  auto gl = std::make_unique<impeller::ProcTableGLES>(
      gl_dispatch_table_.gl_proc_resolver);
  if (!gl->IsValid()) {
    return;
  }

  impeller_context_ = impeller::ContextGLES::Create(
      std::move(gl), shader_mappings, /*enable_gpu_tracing=*/false);

  if (!impeller_context_) {
    FML_LOG(ERROR) << "Could not create Impeller context.";
    return;
  }

  auto worker_id = impeller_context_->AddReactorWorker(worker_);
  if (!worker_id.has_value()) {
    FML_LOG(ERROR) << "Could not add reactor worker.";
    return;
  }

  gl_dispatch_table_.gl_clear_current_callback();
  FML_LOG(IMPORTANT) << "Using the Impeller rendering backend (OpenGL).";
  valid_ = true;
}

EmbedderSurfaceGLImpeller::~EmbedderSurfaceGLImpeller() = default;

// |EmbedderSurface|
bool EmbedderSurfaceGLImpeller::IsValid() const {
  return valid_;
}

// |GPUSurfaceGLDelegate|
std::unique_ptr<GLContextResult>
EmbedderSurfaceGLImpeller::GLContextMakeCurrent() {
  worker_->SetReactionsAllowedOnCurrentThread(true);
  return std::make_unique<GLContextDefaultResult>(
      gl_dispatch_table_.gl_make_current_callback());
}

// |GPUSurfaceGLDelegate|
bool EmbedderSurfaceGLImpeller::GLContextClearCurrent() {
  worker_->SetReactionsAllowedOnCurrentThread(false);
  return gl_dispatch_table_.gl_clear_current_callback();
}

// |GPUSurfaceGLDelegate|
bool EmbedderSurfaceGLImpeller::GLContextPresent(
    const GLPresentInfo& present_info) {
  // Pass the present information to the embedder present callback.
  return gl_dispatch_table_.gl_present_callback(present_info);
}

// |GPUSurfaceGLDelegate|
GLFBOInfo EmbedderSurfaceGLImpeller::GLContextFBO(
    GLFrameInfo frame_info) const {
  // Get the FBO ID using the gl_fbo_callback and then get exiting damage by
  // passing that ID to the gl_populate_existing_damage.
  return gl_dispatch_table_.gl_populate_existing_damage(
      gl_dispatch_table_.gl_fbo_callback(frame_info));
}

// |GPUSurfaceGLDelegate|
bool EmbedderSurfaceGLImpeller::GLContextFBOResetAfterPresent() const {
  return fbo_reset_after_present_;
}

// |GPUSurfaceGLDelegate|
SkMatrix EmbedderSurfaceGLImpeller::GLContextSurfaceTransformation() const {
  auto callback = gl_dispatch_table_.gl_surface_transformation_callback;
  if (!callback) {
    SkMatrix matrix;
    matrix.setIdentity();
    return matrix;
  }
  return callback();
}

// |GPUSurfaceGLDelegate|
EmbedderSurfaceGLSkia::GLProcResolver
EmbedderSurfaceGLImpeller::GetGLProcResolver() const {
  return gl_dispatch_table_.gl_proc_resolver;
}

// |GPUSurfaceGLDelegate|
SurfaceFrame::FramebufferInfo
EmbedderSurfaceGLImpeller::GLContextFramebufferInfo() const {
  // Enable partial repaint by default on the embedders.
  auto info = SurfaceFrame::FramebufferInfo{};
  info.supports_readback = true;
  info.supports_partial_repaint =
      gl_dispatch_table_.gl_populate_existing_damage != nullptr;
  return info;
}

// |EmbedderSurface|
std::unique_ptr<Surface> EmbedderSurfaceGLImpeller::CreateGPUSurface() {
  // Ensure that the GL context is current before creating the GPU surface.
  // GPUSurfaceGLImpeller initialization will set up shader pipelines, and the
  // current thread needs to be able to execute reactor operations.
  GLContextMakeCurrent();

  return std::make_unique<GPUSurfaceGLImpeller>(
      this,                     // GPU surface GL delegate
      impeller_context_,        // Impeller context
      !external_view_embedder_  // render to surface
  );
}

// |EmbedderSurface|
std::shared_ptr<impeller::Context>
EmbedderSurfaceGLImpeller::CreateImpellerContext() const {
  return impeller_context_;
}

// |EmbedderSurface|
sk_sp<GrDirectContext> EmbedderSurfaceGLImpeller::CreateResourceContext()
    const {
  if (gl_dispatch_table_.gl_make_resource_current_callback()) {
    worker_->SetReactionsAllowedOnCurrentThread(true);
  } else {
    FML_DLOG(ERROR) << "Could not make the resource context current.";
    worker_->SetReactionsAllowedOnCurrentThread(false);
  }
  return nullptr;
}

}  // namespace flutter
