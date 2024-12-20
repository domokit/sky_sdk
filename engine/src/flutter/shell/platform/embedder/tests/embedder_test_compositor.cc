// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_compositor.h"

#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {
namespace testing {

EmbedderTestCompositor::EmbedderTestCompositor(SkISize surface_size,
                                               sk_sp<GrDirectContext> context)
    : surface_size_(surface_size), context_(std::move(context)) {
  FML_CHECK(!surface_size_.isEmpty()) << "Surface size must not be empty";
}

EmbedderTestCompositor::~EmbedderTestCompositor() = default;

static void InvokeAllCallbacks(const std::vector<fml::closure>& callbacks) {
  for (const auto& callback : callbacks) {
    if (callback) {
      callback();
    }
  }
}

bool EmbedderTestCompositor::CreateBackingStore(
    const FlutterBackingStoreConfig* config,
    FlutterBackingStore* backing_store_out) {
  bool success = backingstore_producer_->Create(config, backing_store_out);
  if (success) {
    backing_stores_created_++;
    InvokeAllCallbacks(on_create_render_target_callbacks_);
  }
  return success;
}

bool EmbedderTestCompositor::CollectBackingStore(
    const FlutterBackingStore* backing_store) {
  // We have already set the destruction callback for the various backing
  // stores. Our user_data is just the canvas from that backing store and does
  // not need to be explicitly collected. Embedders might have some other state
  // they want to collect though.
  backing_stores_collected_++;
  InvokeAllCallbacks(on_collect_render_target_callbacks_);
  return true;
}

sk_sp<SkSurface> EmbedderTestCompositor::GetSurface(
    const FlutterBackingStore* backing_store) const {
  return backingstore_producer_->GetSurface(backing_store);
}

sk_sp<SkImage> EmbedderTestCompositor::GetLastComposition() {
  return last_composition_;
}

bool EmbedderTestCompositor::Present(FlutterViewId view_id,
                                     const FlutterLayer** layers,
                                     size_t layers_count) {
  if (!UpdateOffscrenComposition(layers, layers_count)) {
    FML_LOG(ERROR)
        << "Could not update the off-screen composition in the test compositor";
    return false;
  }

  // If the test has asked to access the layers and renderers being presented.
  // Access the same and present it to the test for its test assertions.
  if (present_callback_) {
    auto callback = present_callback_;
    if (present_callback_is_one_shot_) {
      present_callback_ = nullptr;
    }
    callback(view_id, layers, layers_count);
  }

  InvokeAllCallbacks(on_present_callbacks_);
  return true;
}

void EmbedderTestCompositor::SetNextPresentCallback(
    const PresentCallback& next_present_callback) {
  SetPresentCallback(next_present_callback, true);
}

void EmbedderTestCompositor::SetPresentCallback(
    const PresentCallback& present_callback,
    bool one_shot) {
  FML_CHECK(!present_callback_);
  present_callback_ = present_callback;
  present_callback_is_one_shot_ = one_shot;
}

void EmbedderTestCompositor::SetNextSceneCallback(
    const NextSceneCallback& next_scene_callback) {
  FML_CHECK(!next_scene_callback_);
  next_scene_callback_ = next_scene_callback;
}

void EmbedderTestCompositor::SetPlatformViewRendererCallback(
    const PlatformViewRendererCallback& callback) {
  platform_view_renderer_callback_ = callback;
}

size_t EmbedderTestCompositor::GetPendingBackingStoresCount() const {
  FML_CHECK(backing_stores_created_ >= backing_stores_collected_);
  return backing_stores_created_ - backing_stores_collected_;
}

size_t EmbedderTestCompositor::GetBackingStoresCreatedCount() const {
  return backing_stores_created_;
}

size_t EmbedderTestCompositor::GetBackingStoresCollectedCount() const {
  return backing_stores_collected_;
}

void EmbedderTestCompositor::AddOnCreateRenderTargetCallback(
    const fml::closure& callback) {
  on_create_render_target_callbacks_.push_back(callback);
}

void EmbedderTestCompositor::AddOnCollectRenderTargetCallback(
    const fml::closure& callback) {
  on_collect_render_target_callbacks_.push_back(callback);
}

void EmbedderTestCompositor::AddOnPresentCallback(
    const fml::closure& callback) {
  on_present_callbacks_.push_back(callback);
}

sk_sp<GrDirectContext> EmbedderTestCompositor::GetGrContext() {
  return context_;
}

}  // namespace testing
}  // namespace flutter
