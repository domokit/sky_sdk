// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/display_list_deferred_image_gpu_impeller.h"

#include <utility>

#include "flutter/fml/make_copyable.h"

namespace flutter {

sk_sp<DlDeferredImageGPUImpeller> DlDeferredImageGPUImpeller::Make(
    std::unique_ptr<LayerTree> layer_tree,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner) {
  return sk_sp<DlDeferredImageGPUImpeller>(new DlDeferredImageGPUImpeller(
      DlDeferredImageGPUImpeller::ImageWrapper::Make(
          std::move(layer_tree), std::move(snapshot_delegate),
          std::move(raster_task_runner))));
}

sk_sp<DlDeferredImageGPUImpeller> DlDeferredImageGPUImpeller::Make(
    sk_sp<DisplayList> display_list,
    const SkISize& size,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner) {
  return sk_sp<DlDeferredImageGPUImpeller>(new DlDeferredImageGPUImpeller(
      DlDeferredImageGPUImpeller::ImageWrapper::Make(
          std::move(display_list), size, std::move(snapshot_delegate),
          std::move(raster_task_runner))));
}

DlDeferredImageGPUImpeller::DlDeferredImageGPUImpeller(
    std::shared_ptr<ImageWrapper> wrapper)
    : wrapper_(std::move(wrapper)) {}

// |DlImage|
DlDeferredImageGPUImpeller::~DlDeferredImageGPUImpeller() = default;

// |DlImage|
sk_sp<SkImage> DlDeferredImageGPUImpeller::skia_image() const {
  return nullptr;
};

// |DlImage|
std::shared_ptr<impeller::Texture>
DlDeferredImageGPUImpeller::impeller_texture() const {
  if (!wrapper_) {
    return nullptr;
  }
  return wrapper_->texture();
}

// |DlImage|
bool DlDeferredImageGPUImpeller::isOpaque() const {
  // Impeller doesn't currently implement opaque alpha types.
  return false;
}

// |DlImage|
bool DlDeferredImageGPUImpeller::isTextureBacked() const {
  return wrapper_ && wrapper_->isTextureBacked();
}

// |DlImage|
bool DlDeferredImageGPUImpeller::isUIThreadSafe() const {
  return true;
}

// |DlImage|
SkISize DlDeferredImageGPUImpeller::dimensions() const {
  if (!wrapper_) {
    return SkISize::MakeEmpty();
  }
  return wrapper_->size();
}

// |DlImage|
size_t DlDeferredImageGPUImpeller::GetApproximateByteSize() const {
  auto size = sizeof(DlDeferredImageGPUImpeller);
  if (wrapper_) {
    if (wrapper_->texture()) {
      size += wrapper_->texture()
                  ->GetTextureDescriptor()
                  .GetByteSizeOfBaseMipLevel();
    } else {
      size += wrapper_->size().width() * wrapper_->size().height() * 4;
    }
  }
  return size;
}

std::shared_ptr<DlDeferredImageGPUImpeller::ImageWrapper>
DlDeferredImageGPUImpeller::ImageWrapper::Make(
    sk_sp<DisplayList> display_list,
    const SkISize& size,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner) {
  auto wrapper = std::shared_ptr<ImageWrapper>(new ImageWrapper(
      std::move(display_list), size, std::move(snapshot_delegate),
      std::move(raster_task_runner)));
  wrapper->SnapshotDisplayList();
  return wrapper;
}

std::shared_ptr<DlDeferredImageGPUImpeller::ImageWrapper>
DlDeferredImageGPUImpeller::ImageWrapper::Make(
    std::unique_ptr<LayerTree> layer_tree,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner) {
  auto wrapper = std::shared_ptr<ImageWrapper>(new ImageWrapper(
      nullptr, ToSkISize(layer_tree->frame_size()),
      std::move(snapshot_delegate), std::move(raster_task_runner)));
  wrapper->SnapshotDisplayList(std::move(layer_tree));
  return wrapper;
}

DlDeferredImageGPUImpeller::ImageWrapper::ImageWrapper(
    sk_sp<DisplayList> display_list,
    const SkISize& size,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner)
    : size_(size),
      display_list_(std::move(display_list)),
      snapshot_delegate_(std::move(snapshot_delegate)),
      raster_task_runner_(std::move(raster_task_runner)) {}

DlDeferredImageGPUImpeller::ImageWrapper::~ImageWrapper() {
  fml::TaskRunner::RunNowOrPostTask(
      raster_task_runner_, [id = reinterpret_cast<uintptr_t>(this),
                            texture_registry = std::move(texture_registry_)]() {
        if (texture_registry) {
          texture_registry->UnregisterContextListener(id);
        }
      });
}

void DlDeferredImageGPUImpeller::ImageWrapper::OnGrContextCreated() {
  FML_DCHECK(raster_task_runner_->RunsTasksOnCurrentThread());
  SnapshotDisplayList();
}

void DlDeferredImageGPUImpeller::ImageWrapper::OnGrContextDestroyed() {
  // Impeller textures do not have threading requirements for deletion, and
  texture_.reset();
}

bool DlDeferredImageGPUImpeller::ImageWrapper::isTextureBacked() const {
  return texture_ && texture_->IsValid();
}

void DlDeferredImageGPUImpeller::ImageWrapper::SnapshotDisplayList(
    std::unique_ptr<LayerTree> layer_tree) {
  fml::TaskRunner::RunNowOrPostTask(
      raster_task_runner_,
      fml::MakeCopyable([weak_this = weak_from_this(),
                         layer_tree = std::move(layer_tree)]() {
        TRACE_EVENT0("flutter", "SnapshotDisplayList (impeller)");
        auto wrapper = weak_this.lock();
        if (!wrapper) {
          return;
        }
        auto snapshot_delegate = wrapper->snapshot_delegate_;
        if (!snapshot_delegate) {
          return;
        }

        wrapper->texture_registry_ = snapshot_delegate->GetTextureRegistry();
        wrapper->texture_registry_->RegisterContextListener(
            reinterpret_cast<uintptr_t>(wrapper.get()), weak_this);

        if (layer_tree) {
          wrapper->display_list_ = layer_tree->Flatten(
              DlRect::MakeWH(wrapper->size_.width(), wrapper->size_.height()),
              wrapper->texture_registry_);
        }
        auto snapshot = snapshot_delegate->MakeRasterSnapshotSync(
            wrapper->display_list_, wrapper->size_);
        if (!snapshot) {
          std::scoped_lock lock(wrapper->error_mutex_);
          wrapper->error_ = "Failed to create snapshot.";
          return;
        }
        wrapper->texture_ = snapshot->impeller_texture();
      }));
}

std::optional<std::string>
DlDeferredImageGPUImpeller::ImageWrapper::get_error() {
  std::scoped_lock lock(error_mutex_);
  return error_;
}

}  // namespace flutter
