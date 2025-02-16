// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/external_view_embedder/surface_pool.h"

#include <utility>

namespace flutter {

OverlayLayer::OverlayLayer(int id,
                           std::unique_ptr<AndroidSurface> android_surface,
                           std::unique_ptr<Surface> surface)
    : id(id),
      android_surface(std::move(android_surface)),
      surface(std::move(surface)){};

OverlayLayer::~OverlayLayer() = default;

SurfacePool::SurfacePool(bool use_new_surface_methods)
    : use_new_surface_methods_(use_new_surface_methods) {}

SurfacePool::~SurfacePool() = default;

std::shared_ptr<OverlayLayer> SurfacePool::GetLayer(
    GrDirectContext* gr_context,
    const AndroidContext& android_context,
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade,
    const std::shared_ptr<AndroidSurfaceFactory>& surface_factory) {
  std::lock_guard lock(mutex_);
  // Destroy current layers in the pool if the frame size has changed.
  if (requested_frame_size_ != current_frame_size_) {
    DestroyLayersLocked(jni_facade);
  }
  intptr_t gr_context_key = reinterpret_cast<intptr_t>(gr_context);
  // Allocate a new surface if there isn't one available.
  if (available_layer_index_ >= layers_.size()) {
    std::unique_ptr<AndroidSurface> android_surface =
        surface_factory->CreateSurface();

    FML_CHECK(android_surface && android_surface->IsValid())
        << "Could not create an OpenGL, Vulkan or Software surface to set up "
           "rendering.";

    std::unique_ptr<PlatformViewAndroidJNI::OverlayMetadata> java_metadata =
        use_new_surface_methods_
            ? jni_facade->createOverlaySurface2()
            : jni_facade->FlutterViewCreateOverlaySurface();

    FML_CHECK(java_metadata->window);
    android_surface->SetNativeWindow(java_metadata->window, jni_facade);

    std::unique_ptr<Surface> surface =
        android_surface->CreateGPUSurface(gr_context);

    std::shared_ptr<OverlayLayer> layer =
        std::make_shared<OverlayLayer>(java_metadata->id,           //
                                       std::move(android_surface),  //
                                       std::move(surface)           //
        );
    layer->gr_context_key = gr_context_key;
    layers_.push_back(layer);
  }

  std::shared_ptr<OverlayLayer> layer = layers_[available_layer_index_];
  // Since the surfaces are recycled, it's possible that the GrContext is
  // different.
  if (gr_context_key != layer->gr_context_key) {
    layer->gr_context_key = gr_context_key;
    // The overlay already exists, but the GrContext was changed so we need to
    // recreate the rendering surface with the new GrContext.
    std::unique_ptr<Surface> surface =
        layer->android_surface->CreateGPUSurface(gr_context);
    layer->surface = std::move(surface);
  }
  available_layer_index_++;
  current_frame_size_ = requested_frame_size_;
  return layer;
}

void SurfacePool::RecycleLayers() {
  std::lock_guard lock(mutex_);
  available_layer_index_ = 0;
}

bool SurfacePool::HasLayers() {
  std::lock_guard lock(mutex_);
  return !layers_.empty();
}

void SurfacePool::DestroyLayers(
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade) {
  std::lock_guard lock(mutex_);
  DestroyLayersLocked(jni_facade);
}

void SurfacePool::DestroyLayersLocked(
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade) {
  if (layers_.empty()) {
    return;
  }
  if (use_new_surface_methods_) {
    jni_facade->destroyOverlaySurface2();
  } else {
    jni_facade->FlutterViewDestroyOverlaySurfaces();
  }
  layers_.clear();
  available_layer_index_ = 0;
}

std::vector<std::shared_ptr<OverlayLayer>> SurfacePool::GetUnusedLayers() {
  std::lock_guard lock(mutex_);
  std::vector<std::shared_ptr<OverlayLayer>> results;
  for (size_t i = available_layer_index_; i < layers_.size(); i++) {
    results.push_back(layers_[i]);
  }
  return results;
}

void SurfacePool::SetFrameSize(SkISize frame_size) {
  std::lock_guard lock(mutex_);
  requested_frame_size_ = frame_size;
}

void SurfacePool::ResetLayers() {
  available_layer_index_ = 0;
}

void SurfacePool::TrimLayers() {
  std::lock_guard lock(mutex_);
  layers_.erase(layers_.begin() + available_layer_index_, layers_.end());
  available_layer_index_ = 0;
}
}  // namespace flutter
