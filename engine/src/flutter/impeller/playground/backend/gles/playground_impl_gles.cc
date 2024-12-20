// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/backend/gles/playground_impl_gles.h"

#define IMPELLER_PLAYGROUND_SUPPORTS_ANGLE FML_OS_MACOSX

#if IMPELLER_PLAYGROUND_SUPPORTS_ANGLE
#include <dlfcn.h>
#endif

#define GLFW_INCLUDE_NONE
#include "third_party/glfw/include/GLFW/glfw3.h"

#include "flutter/fml/build_config.h"
#include "impeller/entity/gles/entity_shaders_gles.h"
#include "impeller/entity/gles/framebuffer_blend_shaders_gles.h"
#include "impeller/entity/gles/modern_shaders_gles.h"
#include "impeller/fixtures/gles/fixtures_shaders_gles.h"
#include "impeller/fixtures/gles/modern_fixtures_shaders_gles.h"
#include "impeller/playground/imgui/gles/imgui_shaders_gles.h"
#include "impeller/renderer/backend/gles/context_gles.h"
#include "impeller/renderer/backend/gles/surface_gles.h"

namespace impeller {

class PlaygroundImplGLES::ReactorWorker final : public ReactorGLES::Worker {
 public:
  ReactorWorker() = default;

  // |ReactorGLES::Worker|
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override {
    ReaderLock lock(mutex_);
    auto found = reactions_allowed_.find(std::this_thread::get_id());
    if (found == reactions_allowed_.end()) {
      return false;
    }
    return found->second;
  }

  void SetReactionsAllowedOnCurrentThread(bool allowed) {
    WriterLock lock(mutex_);
    reactions_allowed_[std::this_thread::get_id()] = allowed;
  }

 private:
  mutable RWMutex mutex_;
  std::map<std::thread::id, bool> reactions_allowed_ IPLR_GUARDED_BY(mutex_);

  ReactorWorker(const ReactorWorker&) = delete;

  ReactorWorker& operator=(const ReactorWorker&) = delete;
};

void PlaygroundImplGLES::DestroyWindowHandle(WindowHandle handle) {
  if (!handle) {
    return;
  }
  ::glfwDestroyWindow(reinterpret_cast<GLFWwindow*>(handle));
}

PlaygroundImplGLES::PlaygroundImplGLES(PlaygroundSwitches switches)
    : PlaygroundImpl(switches),
      handle_(nullptr, &DestroyWindowHandle),
      worker_(std::shared_ptr<ReactorWorker>(new ReactorWorker())),
      use_angle_(switches.use_angle) {
  if (use_angle_) {
#if IMPELLER_PLAYGROUND_SUPPORTS_ANGLE
    angle_glesv2_ = dlopen("libGLESv2.dylib", RTLD_LAZY);
#endif
    FML_CHECK(angle_glesv2_ != nullptr);
  }

  ::glfwDefaultWindowHints();

#if FML_OS_MACOSX
  FML_CHECK(use_angle_) << "Must use Angle on macOS for OpenGL ES.";
  ::glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
#endif  // FML_OS_MACOSX
  ::glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
  ::glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  ::glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  ::glfwWindowHint(GLFW_RED_BITS, 8);
  ::glfwWindowHint(GLFW_GREEN_BITS, 8);
  ::glfwWindowHint(GLFW_BLUE_BITS, 8);
  ::glfwWindowHint(GLFW_ALPHA_BITS, 8);
  ::glfwWindowHint(GLFW_DEPTH_BITS, 32);   // 32 bit depth buffer
  ::glfwWindowHint(GLFW_STENCIL_BITS, 8);  // 8 bit stencil buffer
  ::glfwWindowHint(GLFW_SAMPLES, 4);       // 4xMSAA

  ::glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

  auto window = ::glfwCreateWindow(1, 1, "Test", nullptr, nullptr);

  ::glfwMakeContextCurrent(window);
  worker_->SetReactionsAllowedOnCurrentThread(true);

  handle_.reset(window);
}

PlaygroundImplGLES::~PlaygroundImplGLES() = default;

static std::vector<std::shared_ptr<fml::Mapping>>
ShaderLibraryMappingsForPlayground() {
  return {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_entity_shaders_gles_data,
          impeller_entity_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_modern_shaders_gles_data,
          impeller_modern_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_gles_data,
          impeller_framebuffer_blend_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_fixtures_shaders_gles_data,
          impeller_fixtures_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_modern_fixtures_shaders_gles_data,
          impeller_modern_fixtures_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_imgui_shaders_gles_data, impeller_imgui_shaders_gles_length),
  };
}

// |PlaygroundImpl|
std::shared_ptr<Context> PlaygroundImplGLES::GetContext() const {
  auto gl = std::make_unique<ProcTableGLES>(CreateGLProcAddressResolver());
  if (!gl->IsValid()) {
    FML_LOG(ERROR) << "Proc table when creating a playground was invalid.";
    return nullptr;
  }

  auto context = ContextGLES::Create(
      std::move(gl), ShaderLibraryMappingsForPlayground(), true);
  if (!context) {
    FML_LOG(ERROR) << "Could not create context.";
    return nullptr;
  }

  auto worker_id = context->AddReactorWorker(worker_);
  if (!worker_id.has_value()) {
    FML_LOG(ERROR) << "Could not add reactor worker.";
    return nullptr;
  }
  return context;
}

// |PlaygroundImpl|
Playground::GLProcAddressResolver
PlaygroundImplGLES::CreateGLProcAddressResolver() const {
  return use_angle_ ? [](const char* name) -> void* {
    void* symbol = nullptr;
#if IMPELLER_PLAYGROUND_SUPPORTS_ANGLE
    void* angle_glesv2 = dlopen("libGLESv2.dylib", RTLD_LAZY);
    symbol = dlsym(angle_glesv2, name);
#endif
    FML_CHECK(symbol);
    return symbol;
  }
  : [](const char* name) -> void* {
      return reinterpret_cast<void*>(::glfwGetProcAddress(name));
    };
}

// |PlaygroundImpl|
PlaygroundImpl::WindowHandle PlaygroundImplGLES::GetWindowHandle() const {
  return handle_.get();
}

// |PlaygroundImpl|
std::unique_ptr<Surface> PlaygroundImplGLES::AcquireSurfaceFrame(
    std::shared_ptr<Context> context) {
  auto window = reinterpret_cast<GLFWwindow*>(GetWindowHandle());
  int width = 0;
  int height = 0;
  ::glfwGetFramebufferSize(window, &width, &height);
  if (width <= 0 || height <= 0) {
    return nullptr;
  }
  SurfaceGLES::SwapCallback swap_callback = [window]() -> bool {
    ::glfwSwapBuffers(window);
    return true;
  };
  return SurfaceGLES::WrapFBO(context,                         //
                              swap_callback,                   //
                              0u,                              //
                              PixelFormat::kR8G8B8A8UNormInt,  //
                              ISize::MakeWH(width, height)     //
  );
}

fml::Status PlaygroundImplGLES::SetCapabilities(
    const std::shared_ptr<Capabilities>& capabilities) {
  return fml::Status(
      fml::StatusCode::kUnimplemented,
      "PlaygroundImplGLES doesn't support setting the capabilities.");
}

}  // namespace impeller
