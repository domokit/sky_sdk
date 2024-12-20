// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_H_

#include <vector>

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {
namespace testing {

class EmbedderTestCompositor {
 public:
  using PlatformViewRendererCallback =
      std::function<sk_sp<SkImage>(const FlutterLayer& layer,
                                   GrDirectContext* context)>;
  using PresentCallback = std::function<void(FlutterViewId view_id,
                                             const FlutterLayer** layers,
                                             size_t layers_count)>;

  EmbedderTestCompositor(SkISize surface_size, sk_sp<GrDirectContext> context);

  virtual ~EmbedderTestCompositor();

  virtual void SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType type,
      FlutterSoftwarePixelFormat software_pixfmt) = 0;

  bool CreateBackingStore(const FlutterBackingStoreConfig* config,
                          FlutterBackingStore* backing_store_out);

  bool CollectBackingStore(const FlutterBackingStore* backing_store);

  bool Present(FlutterViewId view_id,
               const FlutterLayer** layers,
               size_t layers_count);

  void SetPlatformViewRendererCallback(
      const PlatformViewRendererCallback& callback);

  sk_sp<SkSurface> GetSurface(const FlutterBackingStore* backing_store) const;

  //----------------------------------------------------------------------------
  /// @brief      Allows tests to install a callback to notify them when the
  ///             entire render tree has been finalized so they can run their
  ///             assertions.
  ///
  /// @param[in]  next_present_callback  The next present callback
  ///
  void SetNextPresentCallback(const PresentCallback& next_present_callback);

  void SetPresentCallback(const PresentCallback& present_callback,
                          bool one_shot);

  using NextSceneCallback = std::function<void(sk_sp<SkImage> image)>;
  void SetNextSceneCallback(const NextSceneCallback& next_scene_callback);

  sk_sp<SkImage> GetLastComposition();

  size_t GetPendingBackingStoresCount() const;

  size_t GetBackingStoresCreatedCount() const;

  size_t GetBackingStoresCollectedCount() const;

  void AddOnCreateRenderTargetCallback(const fml::closure& callback);

  void AddOnCollectRenderTargetCallback(const fml::closure& callback);

  void AddOnPresentCallback(const fml::closure& callback);

  sk_sp<GrDirectContext> GetGrContext();

 protected:
  virtual bool UpdateOffscrenComposition(const FlutterLayer** layers,
                                         size_t layers_count) = 0;

  // TODO(gw280): encapsulate these properly for subclasses to use
  std::unique_ptr<EmbedderTestBackingStoreProducer> backingstore_producer_;
  const SkISize surface_size_;
  sk_sp<GrDirectContext> context_;

  PlatformViewRendererCallback platform_view_renderer_callback_;
  bool present_callback_is_one_shot_ = false;
  PresentCallback present_callback_;
  NextSceneCallback next_scene_callback_;
  sk_sp<SkImage> last_composition_;
  size_t backing_stores_created_ = 0;
  size_t backing_stores_collected_ = 0;
  std::vector<fml::closure> on_create_render_target_callbacks_;
  std::vector<fml::closure> on_collect_render_target_callbacks_;
  std::vector<fml::closure> on_present_callbacks_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestCompositor);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_H_
