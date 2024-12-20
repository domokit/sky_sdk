// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/lib/ui/painting/image_encoding.h"
#include "flutter/lib/ui/painting/image_encoding_impl.h"

#include "flutter/lib/ui/painting/image.h"

namespace flutter {

void ConvertImageToRasterSkia(
    const sk_sp<DlImage>& dl_image,
    std::function<void(sk_sp<SkImage>)> encode_task,
    const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
    const fml::RefPtr<fml::TaskRunner>& io_task_runner,
    const fml::WeakPtr<GrDirectContext>& resource_context,
    const fml::TaskRunnerAffineWeakPtr<SnapshotDelegate>& snapshot_delegate,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch) {
  // If the owning_context is kRaster, we can't access it on this task runner.
  if (dl_image->owning_context() != DlImage::OwningContext::kRaster) {
    auto image = dl_image->skia_image();

    // Check validity of the image.
    if (image == nullptr) {
      FML_LOG(ERROR) << "Image was null.";
      encode_task(nullptr);
      return;
    }

    auto dimensions = image->dimensions();

    if (dimensions.isEmpty()) {
      FML_LOG(ERROR) << "Image dimensions were empty.";
      encode_task(nullptr);
      return;
    }

    SkPixmap pixmap;
    if (image->peekPixels(&pixmap)) {
      // This is already a raster image.
      encode_task(image);
      return;
    }

    if (sk_sp<SkImage> raster_image = image->makeRasterImage()) {
      // The image can be converted to a raster image.
      encode_task(raster_image);
      return;
    }
  }

  if (!raster_task_runner) {
    FML_LOG(ERROR) << "Raster task runner was null.";
    encode_task(nullptr);
    return;
  }

  if (!io_task_runner) {
    FML_LOG(ERROR) << "IO task runner was null.";
    encode_task(nullptr);
    return;
  }

  // Cross-context images do not support makeRasterImage. Convert these images
  // by drawing them into a surface.  This must be done on the raster thread
  // to prevent concurrent usage of the image on both the IO and raster threads.
  raster_task_runner->PostTask([dl_image, encode_task = std::move(encode_task),
                                resource_context, snapshot_delegate,
                                io_task_runner, is_gpu_disabled_sync_switch,
                                raster_task_runner]() {
    auto image = dl_image->skia_image();
    if (!image || !snapshot_delegate) {
      io_task_runner->PostTask(
          [encode_task = encode_task]() mutable { encode_task(nullptr); });
      return;
    }

    sk_sp<SkImage> raster_image =
        snapshot_delegate->ConvertToRasterImage(image);

    io_task_runner->PostTask([image, encode_task = encode_task,
                              raster_image = std::move(raster_image),
                              resource_context, is_gpu_disabled_sync_switch,
                              owning_context = dl_image->owning_context(),
                              raster_task_runner]() mutable {
      if (!raster_image) {
        // The rasterizer was unable to render the cross-context image
        // (presumably because it does not have a GrContext).  In that case,
        // convert the image on the IO thread using the resource context.
        raster_image = ConvertToRasterUsingResourceContext(
            image, resource_context, is_gpu_disabled_sync_switch);
      }
      encode_task(raster_image);
      if (owning_context == DlImage::OwningContext::kRaster) {
        raster_task_runner->PostTask([image = std::move(image)]() {});
      }
    });
  });
}

}  // namespace flutter

#endif  //  !SLIMPELLER
