// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_H_

#include <memory>

#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/lib/ui/io_manager.h"
#include "flutter/lib/ui/painting/image_descriptor.h"

namespace flutter {

// An object that coordinates image decompression and texture upload across
// multiple threads/components in the shell. This object must be created,
// accessed and collected on the UI thread (typically the engine or its runtime
// controller). None of the expensive operations performed by this component
// occur in a frame pipeline.
class ImageDecoder {
 public:
  static std::unique_ptr<ImageDecoder> Make(
      const Settings& settings,
      const TaskRunners& runners,
      std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
      fml::WeakPtr<IOManager> io_manager,
      const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch);

  virtual ~ImageDecoder();

  using ImageResult = std::function<void(sk_sp<DlImage>, std::string)>;

  // Takes an image descriptor and returns a handle to a texture resident on the
  // GPU. All image decompression and resizes are done on a worker thread
  // concurrently. Texture upload is done on the IO thread and the result
  // returned back on the UI thread. On error, the texture is null but the
  // callback is guaranteed to return on the UI thread.
  virtual void Decode(fml::RefPtr<ImageDescriptor> descriptor,
                      uint32_t target_width,
                      uint32_t target_height,
                      const ImageResult& result) = 0;

  fml::WeakPtr<ImageDecoder> GetWeakPtr() const;

 protected:
  TaskRunners runners_;
  std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner_;
  fml::WeakPtr<IOManager> io_manager_;

  ImageDecoder(
      const TaskRunners& runners,
      std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
      fml::WeakPtr<IOManager> io_manager);

 private:
  fml::WeakPtrFactory<ImageDecoder> weak_factory_;

  FML_DISALLOW_COPY_AND_ASSIGN(ImageDecoder);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_H_
