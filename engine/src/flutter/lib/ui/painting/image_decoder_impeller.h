// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_IMPELLER_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_IMPELLER_H_

#include <future>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/painting/image_decoder.h"
#include "impeller/core/formats.h"
#include "impeller/geometry/size.h"
#include "impeller/renderer/capabilities.h"
#include "include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkBitmap.h"

namespace impeller {
class Context;
class Allocator;
class DeviceBuffer;
}  // namespace impeller

namespace flutter {

class ImpellerAllocator : public SkBitmap::Allocator {
 public:
  explicit ImpellerAllocator(std::shared_ptr<impeller::Allocator> allocator);

  ~ImpellerAllocator() = default;

  // |Allocator|
  bool allocPixelRef(SkBitmap* bitmap) override;

  std::shared_ptr<impeller::DeviceBuffer> GetDeviceBuffer() const;

 private:
  std::shared_ptr<impeller::Allocator> allocator_;
  std::shared_ptr<impeller::DeviceBuffer> buffer_;
};

struct DecompressResult {
  std::shared_ptr<impeller::DeviceBuffer> device_buffer;
  std::shared_ptr<SkBitmap> sk_bitmap;
  SkImageInfo image_info;
  std::optional<SkImageInfo> resize_info = std::nullopt;
  std::string decode_error;
};

class ImageDecoderImpeller final : public ImageDecoder {
 public:
  ImageDecoderImpeller(
      const TaskRunners& runners,
      std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
      const fml::WeakPtr<IOManager>& io_manager,
      bool supports_wide_gamut,
      const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch);

  ~ImageDecoderImpeller() override;

  // |ImageDecoder|
  void Decode(fml::RefPtr<ImageDescriptor> descriptor,
              uint32_t target_width,
              uint32_t target_height,
              const ImageResult& result) override;

  static DecompressResult DecompressTexture(
      ImageDescriptor* descriptor,
      SkISize target_size,
      impeller::ISize max_texture_size,
      bool supports_wide_gamut,
      const std::shared_ptr<const impeller::Capabilities>& capabilities,
      const std::shared_ptr<impeller::Allocator>& allocator);

  /// @brief Create a device private texture from the provided host buffer.
  ///
  /// @param result     The image result closure that accepts the DlImage and
  ///                   any encoding error messages.
  /// @param context    The Impeller graphics context.
  /// @param buffer     A host buffer containing the image to be uploaded.
  /// @param image_info Format information about the particular image.
  /// @param bitmap      A bitmap containg the image to be uploaded.
  /// @param gpu_disabled_switch Whether the GPU is available command encoding.
  static void UploadTextureToPrivate(
      ImageResult result,
      const std::shared_ptr<impeller::Context>& context,
      const std::shared_ptr<impeller::DeviceBuffer>& buffer,
      const SkImageInfo& image_info,
      const std::shared_ptr<SkBitmap>& bitmap,
      const std::optional<SkImageInfo>& resize_info,
      const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch);

  /// @brief Create a texture from the provided bitmap.
  /// @param context     The Impeller graphics context.
  /// @param bitmap      A bitmap containg the image to be uploaded.
  /// @return            A DlImage.
  static std::pair<sk_sp<DlImage>, std::string> UploadTextureToStorage(
      const std::shared_ptr<impeller::Context>& context,
      std::shared_ptr<SkBitmap> bitmap);

 private:
  using FutureContext = std::shared_future<std::shared_ptr<impeller::Context>>;
  FutureContext context_;
  const bool supports_wide_gamut_;
  std::shared_ptr<fml::SyncSwitch> gpu_disabled_switch_;

  /// Only call this method if the GPU is available.
  static std::pair<sk_sp<DlImage>, std::string> UnsafeUploadTextureToPrivate(
      const std::shared_ptr<impeller::Context>& context,
      const std::shared_ptr<impeller::DeviceBuffer>& buffer,
      const SkImageInfo& image_info,
      const std::optional<SkImageInfo>& resize_info);

  FML_DISALLOW_COPY_AND_ASSIGN(ImageDecoderImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_IMPELLER_H_
