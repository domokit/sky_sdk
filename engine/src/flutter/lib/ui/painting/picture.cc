// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/picture.h"

#include <memory>
#include <utility>

#include "flutter/fml/make_copyable.h"
#include "flutter/lib/ui/painting/canvas.h"
#include "flutter/lib/ui/painting/display_list_deferred_image_gpu_skia.h"
#include "flutter/lib/ui/ui_dart_state.h"
#if IMPELLER_SUPPORTS_RENDERING
#include "flutter/lib/ui/painting/display_list_deferred_image_gpu_impeller.h"
#endif  // IMPELLER_SUPPORTS_RENDERING
#include "flutter/lib/ui/painting/display_list_image_gpu.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/dart_persistent_value.h"
#include "third_party/tonic/logging/dart_invoke.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, Picture);

void Picture::CreateAndAssociateWithDartWrapper(
    Dart_Handle dart_handle,
    sk_sp<DisplayList> display_list) {
  FML_DCHECK(display_list->isUIThreadSafe());
  auto canvas_picture = fml::MakeRefCounted<Picture>(std::move(display_list));
  canvas_picture->AssociateWithDartWrapper(dart_handle);
}

Picture::Picture(sk_sp<DisplayList> display_list)
    : display_list_(std::move(display_list)) {}

Picture::~Picture() = default;

Dart_Handle Picture::toImage(uint32_t width,
                             uint32_t height,
                             Dart_Handle raw_image_callback) {
  if (!display_list_) {
    return tonic::ToDart("Picture is null");
  }
  return RasterizeToImage(display_list_, width, height, raw_image_callback);
}

void Picture::toImageSync(uint32_t width,
                          uint32_t height,
                          Dart_Handle raw_image_handle) {
  FML_DCHECK(display_list_);
  RasterizeToImageSync(display_list_, width, height, raw_image_handle);
}

static sk_sp<DlImage> CreateDeferredImage(
    bool impeller,
    sk_sp<DisplayList> display_list,
    uint32_t width,
    uint32_t height,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::RefPtr<fml::TaskRunner> raster_task_runner,
    fml::RefPtr<SkiaUnrefQueue> unref_queue) {
#if IMPELLER_SUPPORTS_RENDERING
  if (impeller) {
    return DlDeferredImageGPUImpeller::Make(
        std::move(display_list), SkISize::Make(width, height),
        std::move(snapshot_delegate), std::move(raster_task_runner));
  }
#endif  // IMPELLER_SUPPORTS_RENDERING

#if SLIMPELLER
  FML_LOG(FATAL) << "Impeller opt-out unavailable.";
  return nullptr;
#else   // SLIMPELLER
  const SkImageInfo image_info = SkImageInfo::Make(
      width, height, kRGBA_8888_SkColorType, kPremul_SkAlphaType);
  return DlDeferredImageGPUSkia::Make(
      image_info, std::move(display_list), std::move(snapshot_delegate),
      raster_task_runner, std::move(unref_queue));
#endif  //  !SLIMPELLER
}

// static
void Picture::RasterizeToImageSync(sk_sp<DisplayList> display_list,
                                   uint32_t width,
                                   uint32_t height,
                                   Dart_Handle raw_image_handle) {
  auto* dart_state = UIDartState::Current();
  if (!dart_state) {
    return;
  }
  auto unref_queue = dart_state->GetSkiaUnrefQueue();
  auto snapshot_delegate = dart_state->GetSnapshotDelegate();
  auto raster_task_runner = dart_state->GetTaskRunners().GetRasterTaskRunner();

  auto image = CanvasImage::Create();
  auto dl_image = CreateDeferredImage(
      dart_state->IsImpellerEnabled(), std::move(display_list), width, height,
      std::move(snapshot_delegate), std::move(raster_task_runner),
      std::move(unref_queue));
  image->set_image(dl_image);
  image->AssociateWithDartWrapper(raw_image_handle);
}

void Picture::dispose() {
  display_list_.reset();
  ClearDartWrapper();
}

size_t Picture::GetAllocationSize() const {
  if (display_list_) {
    return display_list_->bytes() + sizeof(Picture);
  } else {
    return sizeof(Picture);
  }
}

Dart_Handle Picture::RasterizeToImage(const sk_sp<DisplayList>& display_list,
                                      uint32_t width,
                                      uint32_t height,
                                      Dart_Handle raw_image_callback) {
  return DoRasterizeToImage(display_list, nullptr, width, height,
                            raw_image_callback);
}

Dart_Handle Picture::RasterizeLayerTreeToImage(
    std::unique_ptr<LayerTree> layer_tree,
    Dart_Handle raw_image_callback) {
  FML_DCHECK(layer_tree != nullptr);
  auto frame_size = layer_tree->frame_size();
  return DoRasterizeToImage(nullptr, std::move(layer_tree), frame_size.width,
                            frame_size.height, raw_image_callback);
}

Dart_Handle Picture::DoRasterizeToImage(const sk_sp<DisplayList>& display_list,
                                        std::unique_ptr<LayerTree> layer_tree,
                                        uint32_t width,
                                        uint32_t height,
                                        Dart_Handle raw_image_callback) {
  // Either display_list or layer_tree should be provided.
  FML_DCHECK((display_list == nullptr) != (layer_tree == nullptr));

  if (Dart_IsNull(raw_image_callback) || !Dart_IsClosure(raw_image_callback)) {
    return tonic::ToDart("Image callback was invalid");
  }

  if (width == 0 || height == 0) {
    return tonic::ToDart("Image dimensions for scene were invalid.");
  }

  auto* dart_state = UIDartState::Current();
  auto image_callback = std::make_unique<tonic::DartPersistentValue>(
      dart_state, raw_image_callback);
  auto unref_queue = dart_state->GetSkiaUnrefQueue();
  auto ui_task_runner = dart_state->GetTaskRunners().GetUITaskRunner();
  auto raster_task_runner = dart_state->GetTaskRunners().GetRasterTaskRunner();
  auto snapshot_delegate = dart_state->GetSnapshotDelegate();

  // We can't create an image on this task runner because we don't have a
  // graphics context. Even if we did, it would be slow anyway. Also, this
  // thread owns the sole reference to the layer tree. So we do it in the
  // raster thread.

  auto ui_task =
      // The static leak checker gets confused by the use of fml::MakeCopyable.
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
      fml::MakeCopyable([image_callback = std::move(image_callback),
                         unref_queue](sk_sp<DlImage> image) mutable {
        auto dart_state = image_callback->dart_state().lock();
        if (!dart_state) {
          // The root isolate could have died in the meantime.
          return;
        }
        tonic::DartState::Scope scope(dart_state);

        if (!image) {
          tonic::DartInvoke(image_callback->Get(), {Dart_Null()});
          return;
        }

        if (!image->isUIThreadSafe()) {
          // All images with impeller textures should already be safe.
          FML_DCHECK(image->impeller_texture() == nullptr);
          image =
              DlImageGPU::Make({image->skia_image(), std::move(unref_queue)});
        }

        auto dart_image = CanvasImage::Create();
        dart_image->set_image(image);
        auto* raw_dart_image = tonic::ToDart(dart_image);

        // All done!
        tonic::DartInvoke(image_callback->Get(), {raw_dart_image});

        // image_callback is associated with the Dart isolate and must be
        // deleted on the UI thread.
        image_callback.reset();
      });

  // Kick things off on the raster rask runner.
  fml::TaskRunner::RunNowOrPostTask(
      raster_task_runner,
      fml::MakeCopyable([ui_task_runner, snapshot_delegate, display_list, width,
                         height, ui_task,
                         layer_tree = std::move(layer_tree)]() mutable {
        auto picture_bounds = DlISize(width, height);
        sk_sp<DisplayList> snapshot_display_list = display_list;
        if (layer_tree) {
          FML_DCHECK(picture_bounds == layer_tree->frame_size());
          snapshot_display_list =
              layer_tree->Flatten(DlRect::MakeWH(width, height),
                                  snapshot_delegate->GetTextureRegistry(),
                                  snapshot_delegate->GetGrContext());
        }
        snapshot_delegate->MakeRasterSnapshot(
            snapshot_display_list, ToSkISize(picture_bounds),
            [ui_task_runner, ui_task](const sk_sp<DlImage>& image) {
              fml::TaskRunner::RunNowOrPostTask(
                  ui_task_runner, [ui_task, image]() { ui_task(image); });
            });
      }));

  return Dart_Null();
}

}  // namespace flutter
