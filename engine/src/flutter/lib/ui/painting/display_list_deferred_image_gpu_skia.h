// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_DEFERRED_IMAGE_GPU_SKIA_H_
#define FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_DEFERRED_IMAGE_GPU_SKIA_H_

#if !SLIMPELLER

#include <memory>
#include <mutex>

#include "flutter/common/graphics/texture.h"
#include "flutter/display_list/display_list.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/io_manager.h"
#include "flutter/lib/ui/snapshot_delegate.h"

#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"

namespace flutter {

class DlDeferredImageGPUSkia final : public DlImage {
 public:
  static sk_sp<DlDeferredImageGPUSkia> Make(
      const SkImageInfo& image_info,
      sk_sp<DisplayList> display_list,
      fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
      const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
      fml::RefPtr<SkiaUnrefQueue> unref_queue);

  static sk_sp<DlDeferredImageGPUSkia> MakeFromLayerTree(
      const SkImageInfo& image_info,
      std::unique_ptr<LayerTree> layer_tree,
      fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
      const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
      fml::RefPtr<SkiaUnrefQueue> unref_queue);

  // |DlImage|
  ~DlDeferredImageGPUSkia() override;

  // |DlImage|
  // This method is only safe to call from the raster thread.
  // Callers must not hold long term references to this image and
  // only use it for the immediate painting operation. It must be
  // collected on the raster task runner.
  sk_sp<SkImage> skia_image() const override;

  // |DlImage|
  std::shared_ptr<impeller::Texture> impeller_texture() const override;

  // |DlImage|
  bool isOpaque() const override;

  // |DlImage|
  bool isTextureBacked() const override;

  // |DlImage|
  bool isUIThreadSafe() const override;

  // |DlImage|
  SkISize dimensions() const override;

  // |DlImage|
  virtual size_t GetApproximateByteSize() const override;

  // |DlImage|
  // This method is safe to call from any thread.
  std::optional<std::string> get_error() const override;

  // |DlImage|
  OwningContext owning_context() const override {
    return OwningContext::kRaster;
  }

 private:
  class ImageWrapper final : public std::enable_shared_from_this<ImageWrapper>,
                             public ContextListener {
   public:
    static std::shared_ptr<ImageWrapper> Make(
        const SkImageInfo& image_info,
        sk_sp<DisplayList> display_list,
        fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
        fml::RefPtr<fml::TaskRunner> raster_task_runner,
        fml::RefPtr<SkiaUnrefQueue> unref_queue);

    static std::shared_ptr<ImageWrapper> MakeFromLayerTree(
        const SkImageInfo& image_info,
        std::unique_ptr<LayerTree> layer_tree,
        fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
        fml::RefPtr<fml::TaskRunner> raster_task_runner,
        fml::RefPtr<SkiaUnrefQueue> unref_queue);

    const SkImageInfo image_info() const { return image_info_; }
    const GrBackendTexture& texture() const { return texture_; }
    bool isTextureBacked() const;
    std::optional<std::string> get_error();
    sk_sp<SkImage> CreateSkiaImage() const;
    void Unregister();
    void DeleteTexture();

   private:
    const SkImageInfo image_info_;
    sk_sp<DisplayList> display_list_;
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate_;
    fml::RefPtr<fml::TaskRunner> raster_task_runner_;
    fml::RefPtr<SkiaUnrefQueue> unref_queue_;
    std::shared_ptr<TextureRegistry> texture_registry_;

    mutable std::mutex error_mutex_;
    std::optional<std::string> error_;

    GrBackendTexture texture_;
    sk_sp<GrDirectContext> context_;
    // May be used if this image is not texture backed.
    sk_sp<SkImage> image_;

    ImageWrapper(
        const SkImageInfo& image_info,
        sk_sp<DisplayList> display_list,
        fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
        fml::RefPtr<fml::TaskRunner> raster_task_runner,
        fml::RefPtr<SkiaUnrefQueue> unref_queue);

    // If a layer tree is provided, it will be flattened during the raster
    // thread task spawned by this method. After being flattened into a display
    // list, the image wrapper will be updated to hold this display list and the
    // layer tree can be dropped.
    void SnapshotDisplayList(std::unique_ptr<LayerTree> layer_tree = nullptr);

    // |ContextListener|
    void OnGrContextCreated() override;

    // |ContextListener|
    void OnGrContextDestroyed() override;
  };

  const std::shared_ptr<ImageWrapper> image_wrapper_;

  fml::RefPtr<fml::TaskRunner> raster_task_runner_;

  DlDeferredImageGPUSkia(std::shared_ptr<ImageWrapper> image_wrapper,
                         fml::RefPtr<fml::TaskRunner> raster_task_runner);

  FML_DISALLOW_COPY_AND_ASSIGN(DlDeferredImageGPUSkia);
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_DEFERRED_IMAGE_GPU_SKIA_H_
