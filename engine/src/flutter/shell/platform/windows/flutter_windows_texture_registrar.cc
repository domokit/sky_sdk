// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_texture_registrar.h"

#include <mutex>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/embedder_struct_macros.h"
#include "flutter/shell/platform/windows/external_texture_d3d.h"
#include "flutter/shell/platform/windows/external_texture_pixelbuffer.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"

namespace {
static constexpr int64_t kInvalidTexture = -1;
}

namespace flutter {

FlutterWindowsTextureRegistrar::FlutterWindowsTextureRegistrar(
    FlutterWindowsEngine* engine,
    std::shared_ptr<egl::ProcTable> gl)
    : engine_(engine), gl_(std::move(gl)) {}

int64_t FlutterWindowsTextureRegistrar::RegisterTexture(
    const FlutterDesktopTextureInfo* texture_info) {
  if (!gl_) {
    return kInvalidTexture;
  }

  if (texture_info->type == kFlutterDesktopPixelBufferTexture) {
    if (!texture_info->pixel_buffer_config.callback) {
      FML_LOG(ERROR) << "Invalid pixel buffer texture callback.";
      return kInvalidTexture;
    }

    return EmplaceTexture(std::make_unique<flutter::ExternalTexturePixelBuffer>(
        texture_info->pixel_buffer_config.callback,
        texture_info->pixel_buffer_config.user_data, gl_));
  } else if (texture_info->type == kFlutterDesktopGpuSurfaceTexture) {
    const FlutterDesktopGpuSurfaceTextureConfig* gpu_surface_config =
        &texture_info->gpu_surface_config;
    auto surface_type = SAFE_ACCESS(gpu_surface_config, type,
                                    kFlutterDesktopGpuSurfaceTypeNone);
    if (surface_type == kFlutterDesktopGpuSurfaceTypeDxgiSharedHandle ||
        surface_type == kFlutterDesktopGpuSurfaceTypeD3d11Texture2D) {
      auto callback = SAFE_ACCESS(gpu_surface_config, callback, nullptr);
      if (!callback) {
        FML_LOG(ERROR) << "Invalid GPU surface descriptor callback.";
        return kInvalidTexture;
      }

      auto user_data = SAFE_ACCESS(gpu_surface_config, user_data, nullptr);
      return EmplaceTexture(std::make_unique<flutter::ExternalTextureD3d>(
          surface_type, callback, user_data, engine_->egl_manager(), gl_));
    }
  }

  FML_LOG(ERROR) << "Attempted to register texture of unsupport type.";
  return kInvalidTexture;
}

int64_t FlutterWindowsTextureRegistrar::EmplaceTexture(
    std::unique_ptr<ExternalTexture> texture) {
  int64_t texture_id = texture->texture_id();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    textures_[texture_id] = std::move(texture);
  }

  engine_->task_runner()->RunNowOrPostTask([engine = engine_, texture_id]() {
    engine->RegisterExternalTexture(texture_id);
  });

  return texture_id;
}

void FlutterWindowsTextureRegistrar::UnregisterTexture(int64_t texture_id,
                                                       fml::closure callback) {
  engine_->task_runner()->RunNowOrPostTask([engine = engine_, texture_id]() {
    engine->UnregisterExternalTexture(texture_id);
  });

  bool posted = engine_->PostRasterThreadTask([this, texture_id, callback]() {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      auto it = textures_.find(texture_id);
      if (it != textures_.end()) {
        textures_.erase(it);
      }
    }
    if (callback) {
      callback();
    }
  });

  if (!posted && callback) {
    callback();
  }
}

bool FlutterWindowsTextureRegistrar::MarkTextureFrameAvailable(
    int64_t texture_id) {
  engine_->task_runner()->RunNowOrPostTask([engine = engine_, texture_id]() {
    engine->MarkExternalTextureFrameAvailable(texture_id);
  });
  return true;
}

bool FlutterWindowsTextureRegistrar::PopulateTexture(
    int64_t texture_id,
    size_t width,
    size_t height,
    FlutterOpenGLTexture* opengl_texture) {
  flutter::ExternalTexture* texture;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = textures_.find(texture_id);
    if (it == textures_.end()) {
      return false;
    }
    texture = it->second.get();
  }
  return texture->PopulateTexture(width, height, opengl_texture);
}

};  // namespace flutter
