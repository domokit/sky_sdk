// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/image_encoding_impeller.h"

#include "flutter/lib/ui/painting/image.h"
#include "fml/status.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/context.h"
#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkImage.h"

namespace flutter {
namespace {

std::optional<SkColorType> ToSkColorType(impeller::PixelFormat format) {
  switch (format) {
    case impeller::PixelFormat::kR8G8B8A8UNormInt:
      return SkColorType::kRGBA_8888_SkColorType;
    case impeller::PixelFormat::kR16G16B16A16Float:
      return SkColorType::kRGBA_F16_SkColorType;
    case impeller::PixelFormat::kB8G8R8A8UNormInt:
      return SkColorType::kBGRA_8888_SkColorType;
    case impeller::PixelFormat::kB10G10R10XR:
      return SkColorType::kBGR_101010x_XR_SkColorType;
    case impeller::PixelFormat::kB10G10R10A10XR:
      return SkColorType::kBGRA_10101010_XR_SkColorType;
    default:
      return std::nullopt;
  }
}

sk_sp<SkImage> ConvertBufferToSkImage(
    const std::shared_ptr<impeller::DeviceBuffer>& buffer,
    SkColorType color_type,
    SkISize dimensions) {
  SkImageInfo image_info = SkImageInfo::Make(dimensions, color_type,
                                             SkAlphaType::kPremul_SkAlphaType);
  SkBitmap bitmap;
  auto func = [](void* addr, void* context) {
    auto buffer =
        static_cast<std::shared_ptr<impeller::DeviceBuffer>*>(context);
    buffer->reset();
    delete buffer;
  };
  auto bytes_per_pixel = image_info.bytesPerPixel();
  bitmap.installPixels(image_info, buffer->OnGetContents(),
                       dimensions.width() * bytes_per_pixel, func,
                       new std::shared_ptr<impeller::DeviceBuffer>(buffer));
  bitmap.setImmutable();

  sk_sp<SkImage> raster_image = SkImages::RasterFromBitmap(bitmap);
  return raster_image;
}

[[nodiscard]] fml::Status DoConvertImageToRasterImpeller(
    const sk_sp<DlImage>& dl_image,
    const std::function<void(fml::StatusOr<sk_sp<SkImage>>)>& encode_task,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch,
    const std::shared_ptr<impeller::Context>& impeller_context) {
  fml::Status result;
  is_gpu_disabled_sync_switch->Execute(
      fml::SyncSwitch::Handlers()
          .SetIfTrue([&result] {
            result =
                fml::Status(fml::StatusCode::kUnavailable, "GPU unavailable.");
          })
          .SetIfFalse([&dl_image, &encode_task, &impeller_context] {
            ImageEncodingImpeller::ConvertDlImageToSkImage(
                dl_image, encode_task, impeller_context);
          }));
  return result;
}

/// Same as `DoConvertImageToRasterImpeller` but it will attempt to retry the
/// operation if `DoConvertImageToRasterImpeller` returns kUnavailable when the
/// GPU becomes available again.
void DoConvertImageToRasterImpellerWithRetry(
    const sk_sp<DlImage>& dl_image,
    std::function<void(fml::StatusOr<sk_sp<SkImage>>)>&& encode_task,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch,
    const std::shared_ptr<impeller::Context>& impeller_context,
    const fml::RefPtr<fml::TaskRunner>& retry_runner) {
  fml::Status status = DoConvertImageToRasterImpeller(
      dl_image, encode_task, is_gpu_disabled_sync_switch, impeller_context);
  if (!status.ok()) {
    // If the conversion failed because of the GPU is unavailable, store the
    // task on the Context so it can be executed when the GPU becomes available.
    if (status.code() == fml::StatusCode::kUnavailable) {
      impeller_context->StoreTaskForGPU(
          [dl_image, encode_task, is_gpu_disabled_sync_switch, impeller_context,
           retry_runner]() mutable {
            auto retry_task = [dl_image, encode_task = std::move(encode_task),
                               is_gpu_disabled_sync_switch, impeller_context] {
              fml::Status retry_status = DoConvertImageToRasterImpeller(
                  dl_image, encode_task, is_gpu_disabled_sync_switch,
                  impeller_context);
              if (!retry_status.ok()) {
                // The retry failed for some reason, maybe the GPU became
                // unavailable again. Don't retry again, just fail in this case.
                encode_task(retry_status);
              }
            };
            // If a `retry_runner` is specified, post the retry to it, otherwise
            // execute it directly.
            if (retry_runner) {
              retry_runner->PostTask(retry_task);
            } else {
              retry_task();
            }
          },
          [encode_task]() {
            encode_task(
                fml::Status(fml::StatusCode::kUnavailable, "GPU unavailable."));
          });
    } else {
      // Pass on errors that are not `kUnavailable`.
      encode_task(status);
    }
  }
}

}  // namespace

void ImageEncodingImpeller::ConvertDlImageToSkImage(
    const sk_sp<DlImage>& dl_image,
    std::function<void(fml::StatusOr<sk_sp<SkImage>>)> encode_task,
    const std::shared_ptr<impeller::Context>& impeller_context) {
  auto texture = dl_image->impeller_texture();

  if (impeller_context == nullptr) {
    encode_task(fml::Status(fml::StatusCode::kFailedPrecondition,
                            "Impeller context was null."));
    return;
  }

  if (texture == nullptr) {
    encode_task(
        fml::Status(fml::StatusCode::kFailedPrecondition, "Image was null."));
    return;
  }

  auto dimensions = dl_image->dimensions();
  auto color_type = ToSkColorType(texture->GetTextureDescriptor().format);

  if (dimensions.isEmpty()) {
    encode_task(fml::Status(fml::StatusCode::kFailedPrecondition,
                            "Image dimensions were empty."));
    return;
  }

  if (!color_type.has_value()) {
    encode_task(fml::Status(fml::StatusCode::kUnimplemented,
                            "Failed to get color type from pixel format."));
    return;
  }

  impeller::DeviceBufferDescriptor buffer_desc;
  buffer_desc.storage_mode = impeller::StorageMode::kHostVisible;
  buffer_desc.readback = true;  // set to false for testing.
  buffer_desc.size =
      texture->GetTextureDescriptor().GetByteSizeOfBaseMipLevel();
  auto buffer =
      impeller_context->GetResourceAllocator()->CreateBuffer(buffer_desc);
  if (!buffer) {
    encode_task(fml::Status(fml::StatusCode::kUnimplemented,
                            "Failed to allocate destination buffer."));
    return;
  }

  auto command_buffer = impeller_context->CreateCommandBuffer();
  command_buffer->SetLabel("BlitTextureToBuffer Command Buffer");
  auto pass = command_buffer->CreateBlitPass();
  pass->SetLabel("BlitTextureToBuffer Blit Pass");
  pass->AddCopy(texture, buffer);
  pass->EncodeCommands();
  auto completion = [buffer, color_type = color_type.value(), dimensions,
                     encode_task = std::move(encode_task)](
                        impeller::CommandBuffer::Status status) {
    if (status != impeller::CommandBuffer::Status::kCompleted) {
      encode_task(fml::Status(fml::StatusCode::kUnknown, ""));
      return;
    }
    buffer->Invalidate();
    auto sk_image = ConvertBufferToSkImage(buffer, color_type, dimensions);
    encode_task(sk_image);
  };

  if (!impeller_context->GetCommandQueue()
           ->Submit({command_buffer}, completion)
           .ok()) {
    FML_LOG(ERROR) << "Failed to submit commands.";
  }

  impeller_context->DisposeThreadLocalCachedResources();
}

void ImageEncodingImpeller::ConvertImageToRaster(
    const sk_sp<DlImage>& dl_image,
    std::function<void(fml::StatusOr<sk_sp<SkImage>>)> encode_task,
    const fml::RefPtr<fml::TaskRunner>& raster_task_runner,
    const fml::RefPtr<fml::TaskRunner>& io_task_runner,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch,
    const std::shared_ptr<impeller::Context>& impeller_context) {
  auto original_encode_task = std::move(encode_task);
  encode_task = [original_encode_task = std::move(original_encode_task),
                 io_task_runner](fml::StatusOr<sk_sp<SkImage>> image) mutable {
    fml::TaskRunner::RunNowOrPostTask(
        io_task_runner,
        [original_encode_task = std::move(original_encode_task),
         image = std::move(image)]() { original_encode_task(image); });
  };

  if (dl_image->owning_context() != DlImage::OwningContext::kRaster) {
    DoConvertImageToRasterImpellerWithRetry(dl_image, std::move(encode_task),
                                            is_gpu_disabled_sync_switch,
                                            impeller_context,
                                            /*retry_runner=*/nullptr);
    return;
  }

  raster_task_runner->PostTask([dl_image, encode_task = std::move(encode_task),
                                io_task_runner, is_gpu_disabled_sync_switch,
                                impeller_context,
                                raster_task_runner]() mutable {
    DoConvertImageToRasterImpellerWithRetry(
        dl_image, std::move(encode_task), is_gpu_disabled_sync_switch,
        impeller_context, raster_task_runner);
  });
}

int ImageEncodingImpeller::GetColorSpace(
    const std::shared_ptr<impeller::Texture>& texture) {
  const impeller::TextureDescriptor& desc = texture->GetTextureDescriptor();
  switch (desc.format) {
    case impeller::PixelFormat::kB10G10R10XR:  // intentional_fallthrough
    case impeller::PixelFormat::kR16G16B16A16Float:
      return ColorSpace::kExtendedSRGB;
    default:
      return ColorSpace::kSRGB;
  }
}

}  // namespace flutter
