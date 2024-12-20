// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/image_descriptor.h"

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "flutter/lib/ui/painting/multi_frame_codec.h"
#include "flutter/lib/ui/painting/single_frame_codec.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/logging/dart_invoke.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, ImageDescriptor);

const SkImageInfo ImageDescriptor::CreateImageInfo() const {
  FML_DCHECK(generator_);
  return generator_->GetInfo();
}

ImageDescriptor::ImageDescriptor(sk_sp<SkData> buffer,
                                 const SkImageInfo& image_info,
                                 std::optional<size_t> row_bytes)
    : buffer_(std::move(buffer)),
      generator_(nullptr),
      image_info_(image_info),
      row_bytes_(row_bytes) {}

ImageDescriptor::ImageDescriptor(sk_sp<SkData> buffer,
                                 std::shared_ptr<ImageGenerator> generator)
    : buffer_(std::move(buffer)),
      generator_(std::move(generator)),
      image_info_(CreateImageInfo()),
      row_bytes_(std::nullopt) {}

Dart_Handle ImageDescriptor::initEncoded(Dart_Handle descriptor_handle,
                                         ImmutableBuffer* immutable_buffer,
                                         Dart_Handle callback_handle) {
  if (!Dart_IsClosure(callback_handle)) {
    return tonic::ToDart("Callback must be a function");
  }

  if (!immutable_buffer) {
    return tonic::ToDart("Buffer parameter must not be null");
  }

  // This has to be valid because this method is called from Dart.
  auto dart_state = UIDartState::Current();
  auto registry = dart_state->GetImageGeneratorRegistry();

  if (!registry) {
    return tonic::ToDart(
        "Failed to access the internal image decoder "
        "registry on this isolate. Please file a bug on "
        "https://github.com/flutter/flutter/issues.");
  }

  auto generator =
      registry->CreateCompatibleGenerator(immutable_buffer->data());

  if (!generator) {
    // No compatible image decoder was found.
    return tonic::ToDart("Invalid image data");
  }

  auto descriptor = fml::MakeRefCounted<ImageDescriptor>(
      immutable_buffer->data(), std::move(generator));

  FML_DCHECK(descriptor);

  descriptor->AssociateWithDartWrapper(descriptor_handle);
  tonic::DartInvoke(callback_handle, {Dart_TypeVoid()});

  return Dart_Null();
}

void ImageDescriptor::initRaw(Dart_Handle descriptor_handle,
                              const fml::RefPtr<ImmutableBuffer>& data,
                              int width,
                              int height,
                              int row_bytes,
                              PixelFormat pixel_format) {
  SkColorType color_type = kUnknown_SkColorType;
  SkAlphaType alpha_type = kPremul_SkAlphaType;
  switch (pixel_format) {
    case PixelFormat::kRGBA8888:
      color_type = kRGBA_8888_SkColorType;
      break;
    case PixelFormat::kBGRA8888:
      color_type = kBGRA_8888_SkColorType;
      break;
    case PixelFormat::kRGBAFloat32:
      // `PixelFormat.rgbaFloat32` is documented to not use premultiplied alpha.
      color_type = kRGBA_F32_SkColorType;
      alpha_type = kUnpremul_SkAlphaType;
      break;
  }
  FML_DCHECK(color_type != kUnknown_SkColorType);
  auto image_info = SkImageInfo::Make(width, height, color_type, alpha_type);
  auto descriptor = fml::MakeRefCounted<ImageDescriptor>(
      data->data(), std::move(image_info),
      row_bytes == -1 ? std::nullopt : std::optional<size_t>(row_bytes));
  descriptor->AssociateWithDartWrapper(descriptor_handle);
}

void ImageDescriptor::instantiateCodec(Dart_Handle codec_handle,
                                       int target_width,
                                       int target_height) {
  fml::RefPtr<Codec> ui_codec;
  if (!generator_ || generator_->GetFrameCount() == 1) {
    ui_codec = fml::MakeRefCounted<SingleFrameCodec>(
        static_cast<fml::RefPtr<ImageDescriptor>>(this), target_width,
        target_height);
  } else {
    ui_codec = fml::MakeRefCounted<MultiFrameCodec>(generator_);
  }
  ui_codec->AssociateWithDartWrapper(codec_handle);
}

sk_sp<SkImage> ImageDescriptor::image() const {
  return generator_->GetImage();
}

bool ImageDescriptor::get_pixels(const SkPixmap& pixmap) const {
  FML_DCHECK(generator_);
  return generator_->GetPixels(pixmap.info(), pixmap.writable_addr(),
                               pixmap.rowBytes());
}

}  // namespace flutter
