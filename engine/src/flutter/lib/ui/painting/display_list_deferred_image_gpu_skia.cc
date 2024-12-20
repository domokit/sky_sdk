// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/lib/ui/painting/display_list_deferred_image_gpu_skia.h"

#include "flutter/fml/make_copyable.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/gpu/ganesh/SkImageGanesh.h"

namespace flutter {

sk_sp<DlDeferredImageGPUSkia> DlDeferredImageGPUSkia::Make(
    const SkImageInfo& image_info,
    sk_sp<DisplayList> display_list,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue) {
  return sk_sp<DlDeferredImageGPUSkia>(new DlDeferredImageGPUSkia(
      ImageWrapper::Make(image_info, std::move(display_list),
                         std::move(snapshot_delegate), raster_task_runner,
                         std::move(unref_queue)),
      raster_task_runner));
}

sk_sp<DlDeferredImageGPUSkia> DlDeferredImageGPUSkia::MakeFromLayerTree(
    const SkImageInfo& image_info,
    std::unique_ptr<LayerTree> layer_tree,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue) {
  return sk_sp<DlDeferredImageGPUSkia>(new DlDeferredImageGPUSkia(
      ImageWrapper::MakeFromLayerTree(
          image_info, std::move(layer_tree), std::move(snapshot_delegate),
          raster_task_runner, std::move(unref_queue)),
      raster_task_runner));
}

DlDeferredImageGPUSkia::DlDeferredImageGPUSkia(
    std::shared_ptr<ImageWrapper> image_wrapper,
    fml::RefPtr<fml::TaskRunner> raster_task_runner)
    : image_wrapper_(std::move(image_wrapper)),
      raster_task_runner_(std::move(raster_task_runner)) {}

// |DlImage|
DlDeferredImageGPUSkia::~DlDeferredImageGPUSkia() {
  fml::TaskRunner::RunNowOrPostTask(raster_task_runner_,
                                    [image_wrapper = image_wrapper_]() {
                                      if (!image_wrapper) {
                                        return;
                                      }
                                      image_wrapper->Unregister();
                                      image_wrapper->DeleteTexture();
                                    });
}

// |DlImage|
sk_sp<SkImage> DlDeferredImageGPUSkia::skia_image() const {
  return image_wrapper_ ? image_wrapper_->CreateSkiaImage() : nullptr;
};

// |DlImage|
std::shared_ptr<impeller::Texture> DlDeferredImageGPUSkia::impeller_texture()
    const {
  return nullptr;
}

// |DlImage|
bool DlDeferredImageGPUSkia::isOpaque() const {
  return image_wrapper_ ? image_wrapper_->image_info().isOpaque() : false;
}

// |DlImage|
bool DlDeferredImageGPUSkia::isTextureBacked() const {
  return image_wrapper_ ? image_wrapper_->isTextureBacked() : false;
}

// |DlImage|
bool DlDeferredImageGPUSkia::isUIThreadSafe() const {
  return true;
}

// |DlImage|
SkISize DlDeferredImageGPUSkia::dimensions() const {
  return image_wrapper_ ? image_wrapper_->image_info().dimensions()
                        : SkISize::MakeEmpty();
}

// |DlImage|
size_t DlDeferredImageGPUSkia::GetApproximateByteSize() const {
  return sizeof(*this) +
         (image_wrapper_ ? image_wrapper_->image_info().computeMinByteSize()
                         : 0);
}

std::optional<std::string> DlDeferredImageGPUSkia::get_error() const {
  return image_wrapper_ ? image_wrapper_->get_error() : std::nullopt;
}

std::shared_ptr<DlDeferredImageGPUSkia::ImageWrapper>
DlDeferredImageGPUSkia::ImageWrapper::Make(
    const SkImageInfo& image_info,
    sk_sp<DisplayList> display_list,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue) {
  auto wrapper = std::shared_ptr<ImageWrapper>(new ImageWrapper(
      image_info, std::move(display_list), std::move(snapshot_delegate),
      std::move(raster_task_runner), std::move(unref_queue)));
  wrapper->SnapshotDisplayList();
  return wrapper;
}

std::shared_ptr<DlDeferredImageGPUSkia::ImageWrapper>
DlDeferredImageGPUSkia::ImageWrapper::MakeFromLayerTree(
    const SkImageInfo& image_info,
    std::unique_ptr<LayerTree> layer_tree,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue) {
  auto wrapper = std::shared_ptr<ImageWrapper>(
      new ImageWrapper(image_info, nullptr, std::move(snapshot_delegate),
                       std::move(raster_task_runner), std::move(unref_queue)));
  wrapper->SnapshotDisplayList(std::move(layer_tree));
  return wrapper;
}

DlDeferredImageGPUSkia::ImageWrapper::ImageWrapper(
    const SkImageInfo& image_info,
    sk_sp<DisplayList> display_list,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue)
    : image_info_(image_info),
      display_list_(std::move(display_list)),
      snapshot_delegate_(std::move(snapshot_delegate)),
      raster_task_runner_(std::move(raster_task_runner)),
      unref_queue_(std::move(unref_queue)) {}

void DlDeferredImageGPUSkia::ImageWrapper::OnGrContextCreated() {
  FML_DCHECK(raster_task_runner_->RunsTasksOnCurrentThread());
  SnapshotDisplayList();
}

void DlDeferredImageGPUSkia::ImageWrapper::OnGrContextDestroyed() {
  FML_DCHECK(raster_task_runner_->RunsTasksOnCurrentThread());
  DeleteTexture();
}

sk_sp<SkImage> DlDeferredImageGPUSkia::ImageWrapper::CreateSkiaImage() const {
  FML_DCHECK(raster_task_runner_->RunsTasksOnCurrentThread());

  if (texture_.isValid() && context_) {
    return SkImages::BorrowTextureFrom(
        context_.get(), texture_, kTopLeft_GrSurfaceOrigin,
        image_info_.colorType(), image_info_.alphaType(),
        image_info_.refColorSpace());
  }
  return image_;
}

bool DlDeferredImageGPUSkia::ImageWrapper::isTextureBacked() const {
  return texture_.isValid();
}

void DlDeferredImageGPUSkia::ImageWrapper::SnapshotDisplayList(
    std::unique_ptr<LayerTree> layer_tree) {
  fml::TaskRunner::RunNowOrPostTask(
      raster_task_runner_,
      fml::MakeCopyable([weak_this = weak_from_this(),
                         layer_tree = std::move(layer_tree)]() mutable {
        auto wrapper = weak_this.lock();
        if (!wrapper) {
          return;
        }
        auto snapshot_delegate = wrapper->snapshot_delegate_;
        if (!snapshot_delegate) {
          return;
        }
        if (layer_tree) {
          auto display_list =
              layer_tree->Flatten(DlRect::MakeWH(wrapper->image_info_.width(),
                                                 wrapper->image_info_.height()),
                                  snapshot_delegate->GetTextureRegistry(),
                                  snapshot_delegate->GetGrContext());
          wrapper->display_list_ = std::move(display_list);
        }
        auto result = snapshot_delegate->MakeSkiaGpuImage(
            wrapper->display_list_, wrapper->image_info_);
        if (result->texture.isValid()) {
          wrapper->texture_ = result->texture;
          wrapper->context_ = std::move(result->context);
          wrapper->texture_registry_ =
              wrapper->snapshot_delegate_->GetTextureRegistry();
          wrapper->texture_registry_->RegisterContextListener(
              reinterpret_cast<uintptr_t>(wrapper.get()), weak_this);
        } else if (result->image) {
          wrapper->image_ = std::move(result->image);
        } else {
          std::scoped_lock lock(wrapper->error_mutex_);
          wrapper->error_ = result->error;
        }
      }));
}

std::optional<std::string> DlDeferredImageGPUSkia::ImageWrapper::get_error() {
  std::scoped_lock lock(error_mutex_);
  return error_;
}

void DlDeferredImageGPUSkia::ImageWrapper::Unregister() {
  if (texture_registry_) {
    texture_registry_->UnregisterContextListener(
        reinterpret_cast<uintptr_t>(this));
  }
}

void DlDeferredImageGPUSkia::ImageWrapper::DeleteTexture() {
  if (texture_.isValid()) {
    unref_queue_->DeleteTexture(texture_);
    texture_ = GrBackendTexture();
  }
  image_.reset();
  context_.reset();
}

}  // namespace flutter

#endif  //  !SLIMPELLER
