// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_context_gl_impeller.h"

#include "flutter/impeller/renderer/backend/gles/context_gles.h"
#include "flutter/impeller/renderer/backend/gles/proc_table_gles.h"
#include "flutter/impeller/renderer/backend/gles/reactor_gles.h"
#include "flutter/impeller/toolkit/egl/context.h"
#include "flutter/impeller/toolkit/egl/surface.h"
#include "impeller/entity/gles/entity_shaders_gles.h"
#include "impeller/entity/gles/framebuffer_blend_shaders_gles.h"
#include "impeller/entity/gles3/entity_shaders_gles.h"
#include "impeller/entity/gles3/framebuffer_blend_shaders_gles.h"

namespace flutter {

class AndroidContextGLImpeller::ReactorWorker final
    : public impeller::ReactorGLES::Worker {
 public:
  ReactorWorker() = default;

  // |impeller::ReactorGLES::Worker|
  ~ReactorWorker() override = default;

  // |impeller::ReactorGLES::Worker|
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

static std::shared_ptr<impeller::Context> CreateImpellerContext(
    const std::shared_ptr<impeller::ReactorGLES::Worker>& worker,
    bool enable_gpu_tracing) {
  auto proc_table = std::make_unique<impeller::ProcTableGLES>(
      impeller::egl::CreateProcAddressResolver());

  if (!proc_table->IsValid()) {
    FML_LOG(ERROR) << "Could not create OpenGL proc table.";
    return nullptr;
  }
  bool is_gles3 = proc_table->GetDescription()->GetGlVersion().IsAtLeast(
      impeller::Version{3, 0, 0});

  std::vector<std::shared_ptr<fml::Mapping>> gles2_shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_entity_shaders_gles_data,
          impeller_entity_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_gles_data,
          impeller_framebuffer_blend_shaders_gles_length),
  };

  std::vector<std::shared_ptr<fml::Mapping>> gles3_shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_entity_shaders_gles3_data,
          impeller_entity_shaders_gles3_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_gles3_data,
          impeller_framebuffer_blend_shaders_gles3_length),
  };

  auto context = impeller::ContextGLES::Create(
      std::move(proc_table),
      is_gles3 ? gles3_shader_mappings : gles2_shader_mappings,
      enable_gpu_tracing);
  if (!context) {
    FML_LOG(ERROR) << "Could not create OpenGLES Impeller Context.";
    return nullptr;
  }

  if (!context->AddReactorWorker(worker).has_value()) {
    FML_LOG(ERROR) << "Could not add reactor worker.";
    return nullptr;
  }
  FML_LOG(IMPORTANT) << "Using the Impeller rendering backend (OpenGLES).";
  return context;
}

AndroidContextGLImpeller::AndroidContextGLImpeller(
    std::unique_ptr<impeller::egl::Display> display,
    bool enable_gpu_tracing)
    : AndroidContext(AndroidRenderingAPI::kImpellerOpenGLES),
      reactor_worker_(std::shared_ptr<ReactorWorker>(new ReactorWorker())),
      display_(std::move(display)) {
  if (!display_ || !display_->IsValid()) {
    FML_DLOG(ERROR) << "Could not create context with invalid EGL display.";
    return;
  }

  impeller::egl::ConfigDescriptor desc;
  desc.api = impeller::egl::API::kOpenGLES3;
  desc.color_format = impeller::egl::ColorFormat::kRGBA8888;
  desc.depth_bits = impeller::egl::DepthBits::kTwentyFour;
  desc.stencil_bits = impeller::egl::StencilBits::kEight;
  desc.samples = impeller::egl::Samples::kFour;

  desc.surface_type = impeller::egl::SurfaceType::kWindow;
  std::unique_ptr<impeller::egl::Config> onscreen_config =
      display_->ChooseConfig(desc);

  if (!onscreen_config) {
    desc.api = impeller::egl::API::kOpenGLES2;
    onscreen_config = display_->ChooseConfig(desc);
  }

  if (!onscreen_config) {
    // Fallback for Android emulator.
    desc.samples = impeller::egl::Samples::kOne;
    onscreen_config = display_->ChooseConfig(desc);
    if (onscreen_config) {
      FML_LOG(INFO) << "Warning: This device doesn't support MSAA for onscreen "
                       "framebuffers. Falling back to a single sample.";
    } else {
      FML_DLOG(ERROR) << "Could not choose onscreen config.";
      return;
    }
  }

  desc.surface_type = impeller::egl::SurfaceType::kPBuffer;
  auto offscreen_config = display_->ChooseConfig(desc);
  if (!offscreen_config) {
    FML_DLOG(ERROR) << "Could not choose offscreen config.";
    return;
  }

  auto onscreen_context = display_->CreateContext(*onscreen_config, nullptr);
  if (!onscreen_context) {
    FML_DLOG(ERROR) << "Could not create onscreen context.";
    return;
  }

  auto offscreen_context =
      display_->CreateContext(*offscreen_config, onscreen_context.get());
  if (!offscreen_context) {
    FML_DLOG(ERROR) << "Could not create offscreen context.";
    return;
  }

  // Creating the impeller::Context requires a current context, which requires
  // some surface.
  auto offscreen_surface =
      display_->CreatePixelBufferSurface(*offscreen_config, 1u, 1u);
  if (!offscreen_context->MakeCurrent(*offscreen_surface)) {
    FML_DLOG(ERROR) << "Could not make offscreen context current.";
    return;
  }

  auto impeller_context =
      CreateImpellerContext(reactor_worker_, enable_gpu_tracing);

  if (!impeller_context) {
    FML_DLOG(ERROR) << "Could not create Impeller context.";
    return;
  }

  if (!offscreen_context->ClearCurrent()) {
    FML_DLOG(ERROR) << "Could not clear offscreen context.";
    return;
  }
  // Setup context listeners.
  impeller::egl::Context::LifecycleListener listener =
      [worker =
           reactor_worker_](impeller::egl ::Context::LifecycleEvent event) {
        switch (event) {
          case impeller::egl::Context::LifecycleEvent::kDidMakeCurrent:
            worker->SetReactionsAllowedOnCurrentThread(true);
            break;
          case impeller::egl::Context::LifecycleEvent::kWillClearCurrent:
            worker->SetReactionsAllowedOnCurrentThread(false);
            break;
        }
      };
  if (!onscreen_context->AddLifecycleListener(listener).has_value() ||
      !offscreen_context->AddLifecycleListener(listener).has_value()) {
    FML_DLOG(ERROR) << "Could not add lifecycle listeners";
  }

  onscreen_config_ = std::move(onscreen_config);
  offscreen_config_ = std::move(offscreen_config);
  onscreen_context_ = std::move(onscreen_context);
  offscreen_context_ = std::move(offscreen_context);
  SetImpellerContext(impeller_context);

  is_valid_ = true;
}

AndroidContextGLImpeller::~AndroidContextGLImpeller() = default;

bool AndroidContextGLImpeller::IsValid() const {
  return is_valid_;
}

bool AndroidContextGLImpeller::ResourceContextClearCurrent() {
  if (!offscreen_context_) {
    return false;
  }

  return offscreen_context_->ClearCurrent();
}

bool AndroidContextGLImpeller::ResourceContextMakeCurrent(
    impeller::egl::Surface* offscreen_surface) {
  if (!offscreen_context_ || !offscreen_surface) {
    return false;
  }

  return offscreen_context_->MakeCurrent(*offscreen_surface);
}

std::unique_ptr<impeller::egl::Surface>
AndroidContextGLImpeller::CreateOffscreenSurface() {
  return display_->CreatePixelBufferSurface(*offscreen_config_, 1u, 1u);
}

bool AndroidContextGLImpeller::OnscreenContextMakeCurrent(
    impeller::egl::Surface* onscreen_surface) {
  if (!onscreen_surface || !onscreen_context_) {
    return false;
  }

  return onscreen_context_->MakeCurrent(*onscreen_surface);
}

bool AndroidContextGLImpeller::OnscreenContextClearCurrent() {
  if (!onscreen_context_) {
    return false;
  }

  return onscreen_context_->ClearCurrent();
}

std::unique_ptr<impeller::egl::Surface>
AndroidContextGLImpeller::CreateOnscreenSurface(EGLNativeWindowType window) {
  return display_->CreateWindowSurface(*onscreen_config_, window);
}

}  // namespace flutter
