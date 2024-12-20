// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPELLER_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPELLER_H_

#include "flutter/common/task_runners.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/fml/status_or.h"
#include "flutter/fml/synchronization/sync_switch.h"

namespace impeller {
class Context;
}  // namespace impeller

namespace flutter {

class ImageEncodingImpeller {
 public:
  static int GetColorSpace(const std::shared_ptr<impeller::Texture>& texture);

  /// Converts a DlImage to a SkImage.
  /// This should be called from the thread that corresponds to
  /// `dl_image->owning_context()` when gpu access is guaranteed.
  /// See also: `ConvertImageToRaster`.
  /// Visible for testing.
  static void ConvertDlImageToSkImage(
      const sk_sp<DlImage>& dl_image,
      std::function<void(fml::StatusOr<sk_sp<SkImage>>)> encode_task,
      const std::shared_ptr<impeller::Context>& impeller_context);

  /// Converts a DlImage to a SkImage.
  /// `encode_task` is executed with the resulting `SkImage`.
  static void ConvertImageToRaster(
      const sk_sp<DlImage>& dl_image,
      std::function<void(fml::StatusOr<sk_sp<SkImage>>)> encode_task,
      const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
      const fml::RefPtr<fml::TaskRunner>& io_task_runner,
      const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch,
      const std::shared_ptr<impeller::Context>& impeller_context);
};
}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPELLER_H_
