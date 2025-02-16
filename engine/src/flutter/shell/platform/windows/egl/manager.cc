// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/egl/manager.h"

#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/windows/egl/egl.h"

namespace flutter {
namespace egl {

int Manager::instance_count_ = 0;

std::unique_ptr<Manager> Manager::Create(GpuPreference gpu_preference) {
  std::unique_ptr<Manager> manager;
  manager.reset(new Manager(gpu_preference));
  if (!manager->IsValid()) {
    return nullptr;
  }
  return std::move(manager);
}

Manager::Manager(GpuPreference gpu_preference) {
  ++instance_count_;

  if (!InitializeDisplay(gpu_preference)) {
    return;
  }

  if (!InitializeConfig()) {
    return;
  }

  if (!InitializeContexts()) {
    return;
  }

  is_valid_ = true;
}

Manager::~Manager() {
  CleanUp();
  --instance_count_;
}

bool Manager::InitializeDisplay(GpuPreference gpu_preference) {
  // If the request for a low power GPU is provided,
  // we will attempt to select GPU explicitly, via ANGLE extension
  // that allows to specify the GPU to use via LUID.
  std::optional<LUID> luid = std::nullopt;
  if (gpu_preference == GpuPreference::LowPowerPreference) {
    luid = GetLowPowerGpuLuid();
  }

  // These are preferred display attributes and request ANGLE's D3D11
  // renderer (use only in case of valid LUID returned from above).
  const EGLint d3d11_display_attributes_with_luid[] = {
      EGL_PLATFORM_ANGLE_TYPE_ANGLE,
      EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE,

      // EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE is an option that will
      // enable ANGLE to automatically call the IDXGIDevice3::Trim method on
      // behalf of the application when it gets suspended.
      EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE,
      EGL_TRUE,

      // This extension allows angle to render directly on a D3D swapchain
      // in the correct orientation on D3D11.
      EGL_EXPERIMENTAL_PRESENT_PATH_ANGLE,
      EGL_EXPERIMENTAL_PRESENT_PATH_FAST_ANGLE,

      // Specify the LUID of the GPU to use.
      EGL_PLATFORM_ANGLE_D3D_LUID_HIGH_ANGLE,
      static_cast<EGLint>(luid.has_value() ? luid->HighPart : 0),
      EGL_PLATFORM_ANGLE_D3D_LUID_LOW_ANGLE,
      static_cast<EGLint>(luid.has_value() ? luid->LowPart : 0),
      EGL_NONE,
  };

  // These are preferred display attributes and request ANGLE's D3D11
  // renderer. eglInitialize will only succeed with these attributes if the
  // hardware supports D3D11 Feature Level 10_0+.
  const EGLint d3d11_display_attributes[] = {
      EGL_PLATFORM_ANGLE_TYPE_ANGLE,
      EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE,

      // EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE is an option that will
      // enable ANGLE to automatically call the IDXGIDevice3::Trim method on
      // behalf of the application when it gets suspended.
      EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE,
      EGL_TRUE,

      // This extension allows angle to render directly on a D3D swapchain
      // in the correct orientation on D3D11.
      EGL_EXPERIMENTAL_PRESENT_PATH_ANGLE,
      EGL_EXPERIMENTAL_PRESENT_PATH_FAST_ANGLE,

      EGL_NONE,
  };

  // These are used to request ANGLE's D3D11 renderer, with D3D11 Feature
  // Level 9_3.
  const EGLint d3d11_fl_9_3_display_attributes[] = {
      EGL_PLATFORM_ANGLE_TYPE_ANGLE,
      EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE,
      EGL_PLATFORM_ANGLE_MAX_VERSION_MAJOR_ANGLE,
      9,
      EGL_PLATFORM_ANGLE_MAX_VERSION_MINOR_ANGLE,
      3,
      EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE,
      EGL_TRUE,
      EGL_NONE,
  };

  // These attributes request D3D11 WARP (software rendering fallback) in case
  // hardware-backed D3D11 is unavailable.
  const EGLint d3d11_warp_display_attributes[] = {
      EGL_PLATFORM_ANGLE_TYPE_ANGLE,
      EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE,
      EGL_PLATFORM_ANGLE_ENABLE_AUTOMATIC_TRIM_ANGLE,
      EGL_TRUE,
      EGL_NONE,
  };

  std::vector<const EGLint*> display_attributes_configs;

  if (luid) {
    // If LUID value is present, obtain an adapter with that luid.
    display_attributes_configs.push_back(d3d11_display_attributes_with_luid);
  }
  display_attributes_configs.push_back(d3d11_display_attributes);
  display_attributes_configs.push_back(d3d11_fl_9_3_display_attributes);
  display_attributes_configs.push_back(d3d11_warp_display_attributes);

  PFNEGLGETPLATFORMDISPLAYEXTPROC egl_get_platform_display_EXT =
      reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
          ::eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (!egl_get_platform_display_EXT) {
    LogEGLError("eglGetPlatformDisplayEXT not available");
    return false;
  }

  // Attempt to initialize ANGLE's renderer in order of: D3D11, D3D11 Feature
  // Level 9_3 and finally D3D11 WARP.
  for (auto config : display_attributes_configs) {
    bool is_last = (config == display_attributes_configs.back());

    display_ = egl_get_platform_display_EXT(EGL_PLATFORM_ANGLE_ANGLE,
                                            EGL_DEFAULT_DISPLAY, config);

    if (display_ == EGL_NO_DISPLAY) {
      if (is_last) {
        LogEGLError("Failed to get a compatible EGLdisplay");
        return false;
      }

      // Try the next config.
      continue;
    }

    if (::eglInitialize(display_, nullptr, nullptr) == EGL_FALSE) {
      if (is_last) {
        LogEGLError("Failed to initialize EGL via ANGLE");
        return false;
      }

      // Try the next config.
      continue;
    }

    return true;
  }

  FML_UNREACHABLE();
}

bool Manager::InitializeConfig() {
  const EGLint config_attributes[] = {EGL_RED_SIZE,   8, EGL_GREEN_SIZE,   8,
                                      EGL_BLUE_SIZE,  8, EGL_ALPHA_SIZE,   8,
                                      EGL_DEPTH_SIZE, 8, EGL_STENCIL_SIZE, 8,
                                      EGL_NONE};

  EGLint num_config = 0;

  EGLBoolean result =
      ::eglChooseConfig(display_, config_attributes, &config_, 1, &num_config);

  if (result == EGL_TRUE && num_config > 0) {
    return true;
  }

  LogEGLError("Failed to choose EGL config");
  return false;
}

bool Manager::InitializeContexts() {
  const EGLint context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};

  auto const render_context =
      ::eglCreateContext(display_, config_, EGL_NO_CONTEXT, context_attributes);
  if (render_context == EGL_NO_CONTEXT) {
    LogEGLError("Failed to create EGL render context");
    return false;
  }

  auto const resource_context =
      ::eglCreateContext(display_, config_, render_context, context_attributes);
  if (resource_context == EGL_NO_CONTEXT) {
    LogEGLError("Failed to create EGL resource context");
    return false;
  }

  render_context_ = std::make_unique<Context>(display_, render_context);
  resource_context_ = std::make_unique<Context>(display_, resource_context);
  return true;
}

bool Manager::InitializeDevice() {
  const auto query_display_attrib_EXT =
      reinterpret_cast<PFNEGLQUERYDISPLAYATTRIBEXTPROC>(
          ::eglGetProcAddress("eglQueryDisplayAttribEXT"));
  const auto query_device_attrib_EXT =
      reinterpret_cast<PFNEGLQUERYDEVICEATTRIBEXTPROC>(
          ::eglGetProcAddress("eglQueryDeviceAttribEXT"));

  if (query_display_attrib_EXT == nullptr ||
      query_device_attrib_EXT == nullptr) {
    return false;
  }

  EGLAttrib egl_device = 0;
  EGLAttrib angle_device = 0;

  auto result = query_display_attrib_EXT(display_, EGL_DEVICE_EXT, &egl_device);
  if (result != EGL_TRUE) {
    return false;
  }

  result = query_device_attrib_EXT(reinterpret_cast<EGLDeviceEXT>(egl_device),
                                   EGL_D3D11_DEVICE_ANGLE, &angle_device);
  if (result != EGL_TRUE) {
    return false;
  }

  resolved_device_ = reinterpret_cast<ID3D11Device*>(angle_device);
  return true;
}

void Manager::CleanUp() {
  EGLBoolean result = EGL_FALSE;

  // Needs to be reset before destroying the contexts.
  resolved_device_.Reset();

  // Needs to be reset before destroying the EGLDisplay.
  render_context_.reset();
  resource_context_.reset();

  if (display_ != EGL_NO_DISPLAY) {
    // Display is reused between instances so only terminate display
    // if destroying last instance
    if (instance_count_ == 1) {
      ::eglTerminate(display_);
    }
    display_ = EGL_NO_DISPLAY;
  }
}

bool Manager::IsValid() const {
  return is_valid_;
}

std::unique_ptr<WindowSurface> Manager::CreateWindowSurface(HWND hwnd,
                                                            size_t width,
                                                            size_t height) {
  if (!hwnd || !is_valid_) {
    return nullptr;
  }

  // Disable ANGLE's automatic surface resizing and provide an explicit size.
  // The surface will need to be destroyed and re-created if the HWND is
  // resized.
  const EGLint surface_attributes[] = {EGL_FIXED_SIZE_ANGLE,
                                       EGL_TRUE,
                                       EGL_WIDTH,
                                       static_cast<EGLint>(width),
                                       EGL_HEIGHT,
                                       static_cast<EGLint>(height),
                                       EGL_NONE};

  auto const surface = ::eglCreateWindowSurface(
      display_, config_, static_cast<EGLNativeWindowType>(hwnd),
      surface_attributes);
  if (surface == EGL_NO_SURFACE) {
    LogEGLError("Surface creation failed.");
    return nullptr;
  }

  return std::make_unique<WindowSurface>(display_, render_context_->GetHandle(),
                                         surface, width, height);
}

bool Manager::HasContextCurrent() {
  return ::eglGetCurrentContext() != EGL_NO_CONTEXT;
}

EGLSurface Manager::CreateSurfaceFromHandle(EGLenum handle_type,
                                            EGLClientBuffer handle,
                                            const EGLint* attributes) const {
  return ::eglCreatePbufferFromClientBuffer(display_, handle_type, handle,
                                            config_, attributes);
}

bool Manager::GetDevice(ID3D11Device** device) {
  if (!resolved_device_) {
    if (!InitializeDevice()) {
      return false;
    }
  }

  resolved_device_.CopyTo(device);
  return (resolved_device_ != nullptr);
}

Context* Manager::render_context() const {
  return render_context_.get();
}

Context* Manager::resource_context() const {
  return resource_context_.get();
}

std::optional<LUID> Manager::GetLowPowerGpuLuid() {
  Microsoft::WRL::ComPtr<IDXGIFactory1> factory1 = nullptr;
  Microsoft::WRL::ComPtr<IDXGIFactory6> factory6 = nullptr;
  Microsoft::WRL::ComPtr<IDXGIAdapter1> adapter = nullptr;
  HRESULT hr = ::CreateDXGIFactory1(IID_PPV_ARGS(&factory1));
  if (FAILED(hr)) {
    return std::nullopt;
  }
  hr = factory1->QueryInterface(IID_PPV_ARGS(&factory6));
  if (FAILED(hr)) {
    // No support for IDXGIFactory6, so we will not use the selected GPU.
    // We will follow with the default ANGLE selection.
    return std::nullopt;
  }
  hr = factory6->EnumAdapterByGpuPreference(
      0, DXGI_GPU_PREFERENCE_MINIMUM_POWER, IID_PPV_ARGS(&adapter));
  if (FAILED(hr) || adapter == nullptr) {
    return std::nullopt;
  }
  // Get the LUID of the adapter.
  DXGI_ADAPTER_DESC desc;
  hr = adapter->GetDesc(&desc);
  if (FAILED(hr)) {
    return std::nullopt;
  }
  return std::make_optional(desc.AdapterLuid);
}

}  // namespace egl
}  // namespace flutter
