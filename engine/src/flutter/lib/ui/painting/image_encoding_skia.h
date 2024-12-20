// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_SKIA_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_SKIA_H_

#if !SLIMPELLER

#include "flutter/common/task_runners.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "flutter/lib/ui/snapshot_delegate.h"

namespace flutter {

void ConvertImageToRasterSkia(
    const sk_sp<DlImage>& dl_image,
    std::function<void(sk_sp<SkImage>)> encode_task,
    const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
    const fml::RefPtr<fml::TaskRunner>& io_task_runner,
    const fml::WeakPtr<GrDirectContext>& resource_context,
    const fml::TaskRunnerAffineWeakPtr<SnapshotDelegate>& snapshot_delegate,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch);

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_SKIA_H_
