// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <array>
#include <memory>
#include <optional>
#include <sstream>

#include "fml/closure.h"
#include "fml/time/time_point.h"
#include "impeller/core/host_buffer.h"
#include "impeller/playground/image/backends/skia/compressed_image_skia.h"
#include "impeller/playground/image/decompressed_image.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_target.h"
#include "impeller/runtime_stage/runtime_stage.h"

#define GLFW_INCLUDE_NONE
#include "third_party/glfw/include/GLFW/glfw3.h"

#include "flutter/fml/paths.h"
#include "impeller/base/validation.h"
#include "impeller/core/allocator.h"
#include "impeller/core/formats.h"
#include "impeller/playground/backend/vulkan/swiftshader_utilities.h"
#include "impeller/playground/image/compressed_image.h"
#include "impeller/playground/imgui/imgui_impl_impeller.h"
#include "impeller/playground/playground.h"
#include "impeller/playground/playground_impl.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/render_pass.h"
#include "third_party/imgui/backends/imgui_impl_glfw.h"
#include "third_party/imgui/imgui.h"

#if FML_OS_MACOSX
#include "fml/platform/darwin/scoped_nsautorelease_pool.h"
#endif  // FML_OS_MACOSX

#if IMPELLER_ENABLE_VULKAN
#include "impeller/playground/backend/vulkan/playground_impl_vk.h"
#endif  // IMPELLER_ENABLE_VULKAN

namespace impeller {

std::string PlaygroundBackendToString(PlaygroundBackend backend) {
  switch (backend) {
    case PlaygroundBackend::kMetal:
      return "Metal";
    case PlaygroundBackend::kOpenGLES:
      return "OpenGLES";
    case PlaygroundBackend::kVulkan:
      return "Vulkan";
  }
  FML_UNREACHABLE();
}

static void InitializeGLFWOnce() {
  // This guard is a hack to work around a problem where glfwCreateWindow
  // hangs when opening a second window after GLFW has been reinitialized (for
  // example, when flipping through multiple playground tests).
  //
  // Explanation:
  //  * glfwCreateWindow calls [NSApp run], which begins running the event
  //    loop on the current thread.
  //  * GLFW then immediately stops the loop when
  //    applicationDidFinishLaunching is fired.
  //  * applicationDidFinishLaunching is only ever fired once during the
  //    application's lifetime, so subsequent calls to [NSApp run] will always
  //    hang with this setup.
  //  * glfwInit resets the flag that guards against [NSApp run] being
  //    called a second time, which causes the subsequent `glfwCreateWindow`
  //    to hang indefinitely in the event loop, because
  //    applicationDidFinishLaunching is never fired.
  static std::once_flag sOnceInitializer;
  std::call_once(sOnceInitializer, []() {
    ::glfwSetErrorCallback([](int code, const char* description) {
      FML_LOG(ERROR) << "GLFW Error '" << description << "'  (" << code << ").";
    });
    FML_CHECK(::glfwInit() == GLFW_TRUE);
  });
}

Playground::Playground(PlaygroundSwitches switches) : switches_(switches) {
  InitializeGLFWOnce();
  SetupSwiftshaderOnce(switches_.use_swiftshader);
}

Playground::~Playground() = default;

std::shared_ptr<Context> Playground::GetContext() const {
  return context_;
}

std::shared_ptr<Context> Playground::MakeContext() const {
  // Playgrounds are already making a context for each test, so we can just
  // return the `context_`.
  return context_;
}

bool Playground::SupportsBackend(PlaygroundBackend backend) {
  switch (backend) {
    case PlaygroundBackend::kMetal:
#if IMPELLER_ENABLE_METAL
      return true;
#else   // IMPELLER_ENABLE_METAL
      return false;
#endif  // IMPELLER_ENABLE_METAL
    case PlaygroundBackend::kOpenGLES:
#if IMPELLER_ENABLE_OPENGLES
      return true;
#else   // IMPELLER_ENABLE_OPENGLES
      return false;
#endif  // IMPELLER_ENABLE_OPENGLES
    case PlaygroundBackend::kVulkan:
#if IMPELLER_ENABLE_VULKAN
      return PlaygroundImplVK::IsVulkanDriverPresent();
#else   // IMPELLER_ENABLE_VULKAN
      return false;
#endif  // IMPELLER_ENABLE_VULKAN
  }
  FML_UNREACHABLE();
}

void Playground::SetupContext(PlaygroundBackend backend,
                              const PlaygroundSwitches& switches) {
  FML_CHECK(SupportsBackend(backend));

  impl_ = PlaygroundImpl::Create(backend, switches);
  if (!impl_) {
    FML_LOG(WARNING) << "PlaygroundImpl::Create failed.";
    return;
  }

  context_ = impl_->GetContext();
}

void Playground::SetupWindow() {
  if (!context_) {
    FML_LOG(WARNING) << "Asked to set up a window with no context (call "
                        "SetupContext first).";
    return;
  }
  start_time_ = fml::TimePoint::Now().ToEpochDelta();
}

bool Playground::IsPlaygroundEnabled() const {
  return switches_.enable_playground;
}

void Playground::TeardownWindow() {
  if (host_buffer_) {
    host_buffer_.reset();
  }
  if (context_) {
    context_->Shutdown();
  }
  context_.reset();
  impl_.reset();
}

static std::atomic_bool gShouldOpenNewPlaygrounds = true;

bool Playground::ShouldOpenNewPlaygrounds() {
  return gShouldOpenNewPlaygrounds;
}

static void PlaygroundKeyCallback(GLFWwindow* window,
                                  int key,
                                  int scancode,
                                  int action,
                                  int mods) {
  if ((key == GLFW_KEY_ESCAPE) && action == GLFW_RELEASE) {
    if (mods & (GLFW_MOD_CONTROL | GLFW_MOD_SUPER | GLFW_MOD_SHIFT)) {
      gShouldOpenNewPlaygrounds = false;
    }
    ::glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

Point Playground::GetCursorPosition() const {
  return cursor_position_;
}

ISize Playground::GetWindowSize() const {
  return window_size_;
}

Point Playground::GetContentScale() const {
  return impl_->GetContentScale();
}

Scalar Playground::GetSecondsElapsed() const {
  return (fml::TimePoint::Now().ToEpochDelta() - start_time_).ToSecondsF();
}

void Playground::SetCursorPosition(Point pos) {
  cursor_position_ = pos;
}

bool Playground::OpenPlaygroundHere(
    const Playground::RenderCallback& render_callback) {
  if (!switches_.enable_playground) {
    return true;
  }

  if (!render_callback) {
    return true;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  fml::ScopedCleanupClosure destroy_imgui_context(
      []() { ImGui::DestroyContext(); });
  ImGui::StyleColorsDark();

  auto& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigWindowsResizeFromEdges = true;

  auto window = reinterpret_cast<GLFWwindow*>(impl_->GetWindowHandle());
  if (!window) {
    return false;
  }
  ::glfwSetWindowTitle(window, GetWindowTitle().c_str());
  ::glfwSetWindowUserPointer(window, this);
  ::glfwSetWindowSizeCallback(
      window, [](GLFWwindow* window, int width, int height) -> void {
        auto playground =
            reinterpret_cast<Playground*>(::glfwGetWindowUserPointer(window));
        if (!playground) {
          return;
        }
        playground->SetWindowSize(ISize{width, height}.Max({}));
      });
  ::glfwSetKeyCallback(window, &PlaygroundKeyCallback);
  ::glfwSetCursorPosCallback(window, [](GLFWwindow* window, double x,
                                        double y) {
    reinterpret_cast<Playground*>(::glfwGetWindowUserPointer(window))
        ->SetCursorPosition({static_cast<Scalar>(x), static_cast<Scalar>(y)});
  });

  ImGui_ImplGlfw_InitForOther(window, true);
  fml::ScopedCleanupClosure shutdown_imgui([]() { ImGui_ImplGlfw_Shutdown(); });

  ImGui_ImplImpeller_Init(context_);
  fml::ScopedCleanupClosure shutdown_imgui_impeller(
      []() { ImGui_ImplImpeller_Shutdown(); });

  ImGui::SetNextWindowPos({10, 10});

  ::glfwSetWindowSize(window, GetWindowSize().width, GetWindowSize().height);
  ::glfwSetWindowPos(window, 200, 100);
  ::glfwShowWindow(window);

  while (true) {
#if FML_OS_MACOSX
    fml::ScopedNSAutoreleasePool pool;
#endif
    ::glfwPollEvents();

    if (::glfwWindowShouldClose(window)) {
      return true;
    }

    ImGui_ImplGlfw_NewFrame();

    auto surface = impl_->AcquireSurfaceFrame(context_);
    RenderTarget render_target = surface->GetRenderTarget();

    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(),
                                 ImGuiDockNodeFlags_PassthruCentralNode);
    bool result = render_callback(render_target);
    ImGui::Render();

    // Render ImGui overlay.
    {
      auto buffer = context_->CreateCommandBuffer();
      if (!buffer) {
        VALIDATION_LOG << "Could not create command buffer.";
        return false;
      }
      buffer->SetLabel("ImGui Command Buffer");

      auto color0 = render_target.GetColorAttachment(0);
      color0.load_action = LoadAction::kLoad;
      if (color0.resolve_texture) {
        color0.texture = color0.resolve_texture;
        color0.resolve_texture = nullptr;
        color0.store_action = StoreAction::kStore;
      }
      render_target.SetColorAttachment(color0, 0);
      render_target.SetStencilAttachment(std::nullopt);
      render_target.SetDepthAttachment(std::nullopt);

      auto pass = buffer->CreateRenderPass(render_target);
      if (!pass) {
        VALIDATION_LOG << "Could not create render pass.";
        return false;
      }
      pass->SetLabel("ImGui Render Pass");
      if (!host_buffer_) {
        host_buffer_ = HostBuffer::Create(context_->GetResourceAllocator(),
                                          context_->GetIdleWaiter());
      }

      ImGui_ImplImpeller_RenderDrawData(ImGui::GetDrawData(), *pass,
                                        *host_buffer_);

      pass->EncodeCommands();

      if (!context_->GetCommandQueue()->Submit({buffer}).ok()) {
        return false;
      }
    }

    if (!result || !surface->Present()) {
      return false;
    }

    if (!ShouldKeepRendering()) {
      break;
    }
  }

  ::glfwHideWindow(window);

  return true;
}

bool Playground::OpenPlaygroundHere(SinglePassCallback pass_callback) {
  return OpenPlaygroundHere(
      [context = GetContext(), &pass_callback](RenderTarget& render_target) {
        auto buffer = context->CreateCommandBuffer();
        if (!buffer) {
          return false;
        }
        buffer->SetLabel("Playground Command Buffer");

        auto pass = buffer->CreateRenderPass(render_target);
        if (!pass) {
          return false;
        }
        pass->SetLabel("Playground Render Pass");

        if (!pass_callback(*pass)) {
          return false;
        }

        pass->EncodeCommands();
        if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
          return false;
        }
        return true;
      });
}

std::shared_ptr<CompressedImage> Playground::LoadFixtureImageCompressed(
    std::shared_ptr<fml::Mapping> mapping) {
  auto compressed_image = CompressedImageSkia::Create(std::move(mapping));
  if (!compressed_image) {
    VALIDATION_LOG << "Could not create compressed image.";
    return nullptr;
  }

  return compressed_image;
}

std::optional<DecompressedImage> Playground::DecodeImageRGBA(
    const std::shared_ptr<CompressedImage>& compressed) {
  if (compressed == nullptr) {
    return std::nullopt;
  }
  // The decoded image is immediately converted into RGBA as that format is
  // known to be supported everywhere. For image sources that don't need 32
  // bit pixel strides, this is overkill. Since this is a test fixture we
  // aren't necessarily trying to eke out memory savings here and instead
  // favor simplicity.
  auto image = compressed->Decode().ConvertToRGBA();
  if (!image.IsValid()) {
    VALIDATION_LOG << "Could not decode image.";
    return std::nullopt;
  }

  return image;
}

static std::shared_ptr<Texture> CreateTextureForDecompressedImage(
    const std::shared_ptr<Context>& context,
    DecompressedImage& decompressed_image,
    bool enable_mipmapping) {
  TextureDescriptor texture_descriptor;
  texture_descriptor.storage_mode = StorageMode::kDevicePrivate;
  texture_descriptor.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_descriptor.size = decompressed_image.GetSize();
  texture_descriptor.mip_count =
      enable_mipmapping ? decompressed_image.GetSize().MipCount() : 1u;

  auto texture =
      context->GetResourceAllocator()->CreateTexture(texture_descriptor);
  if (!texture) {
    VALIDATION_LOG << "Could not allocate texture for fixture.";
    return nullptr;
  }

  auto command_buffer = context->CreateCommandBuffer();
  if (!command_buffer) {
    FML_DLOG(ERROR) << "Could not create command buffer for mipmap generation.";
    return nullptr;
  }
  command_buffer->SetLabel("Mipmap Command Buffer");

  auto blit_pass = command_buffer->CreateBlitPass();
  auto buffer_view = DeviceBuffer::AsBufferView(
      context->GetResourceAllocator()->CreateBufferWithCopy(
          *decompressed_image.GetAllocation()));
  blit_pass->AddCopy(buffer_view, texture);
  if (enable_mipmapping) {
    blit_pass->SetLabel("Mipmap Blit Pass");
    blit_pass->GenerateMipmap(texture);
  }
  blit_pass->EncodeCommands(context->GetResourceAllocator());
  if (!context->GetCommandQueue()->Submit({command_buffer}).ok()) {
    FML_DLOG(ERROR) << "Failed to submit blit pass command buffer.";
    return nullptr;
  }
  return texture;
}

std::shared_ptr<Texture> Playground::CreateTextureForMapping(
    const std::shared_ptr<Context>& context,
    std::shared_ptr<fml::Mapping> mapping,
    bool enable_mipmapping) {
  auto image = Playground::DecodeImageRGBA(
      Playground::LoadFixtureImageCompressed(std::move(mapping)));
  if (!image.has_value()) {
    return nullptr;
  }
  return CreateTextureForDecompressedImage(context, image.value(),
                                           enable_mipmapping);
}

std::shared_ptr<Texture> Playground::CreateTextureForFixture(
    const char* fixture_name,
    bool enable_mipmapping) const {
  auto texture = CreateTextureForMapping(
      context_, OpenAssetAsMapping(fixture_name), enable_mipmapping);
  if (texture == nullptr) {
    return nullptr;
  }
  texture->SetLabel(fixture_name);
  return texture;
}

std::shared_ptr<Texture> Playground::CreateTextureCubeForFixture(
    std::array<const char*, 6> fixture_names) const {
  std::array<DecompressedImage, 6> images;
  for (size_t i = 0; i < fixture_names.size(); i++) {
    auto image = DecodeImageRGBA(
        LoadFixtureImageCompressed(OpenAssetAsMapping(fixture_names[i])));
    if (!image.has_value()) {
      return nullptr;
    }
    images[i] = image.value();
  }

  TextureDescriptor texture_descriptor;
  texture_descriptor.storage_mode = StorageMode::kDevicePrivate;
  texture_descriptor.type = TextureType::kTextureCube;
  texture_descriptor.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_descriptor.size = images[0].GetSize();
  texture_descriptor.mip_count = 1u;

  auto texture =
      context_->GetResourceAllocator()->CreateTexture(texture_descriptor);
  if (!texture) {
    VALIDATION_LOG << "Could not allocate texture cube.";
    return nullptr;
  }
  texture->SetLabel("Texture cube");

  auto cmd_buffer = context_->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();
  for (size_t i = 0; i < fixture_names.size(); i++) {
    auto device_buffer = context_->GetResourceAllocator()->CreateBufferWithCopy(
        *images[i].GetAllocation());
    blit_pass->AddCopy(DeviceBuffer::AsBufferView(device_buffer), texture, {},
                       "", /*mip_level=*/0, /*slice=*/i);
  }

  if (!blit_pass->EncodeCommands(context_->GetResourceAllocator()) ||
      !context_->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok()) {
    VALIDATION_LOG << "Could not upload texture to device memory.";
    return nullptr;
  }

  return texture;
}

void Playground::SetWindowSize(ISize size) {
  window_size_ = size;
}

bool Playground::ShouldKeepRendering() const {
  return true;
}

fml::Status Playground::SetCapabilities(
    const std::shared_ptr<Capabilities>& capabilities) {
  return impl_->SetCapabilities(capabilities);
}

bool Playground::WillRenderSomething() const {
  return switches_.enable_playground;
}

Playground::GLProcAddressResolver Playground::CreateGLProcAddressResolver()
    const {
  return impl_->CreateGLProcAddressResolver();
}

}  // namespace impeller
