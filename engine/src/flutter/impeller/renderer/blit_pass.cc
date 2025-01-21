// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/blit_pass.h"
#include <memory>
#include <utility>

#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"

namespace impeller {

BlitPass::BlitPass() {}

BlitPass::~BlitPass() = default;

void BlitPass::SetLabel(std::string_view label) {
  if (label.empty()) {
    return;
  }
  OnSetLabel(label);
}

bool BlitPass::AddCopy(std::shared_ptr<Texture> source,
                       std::shared_ptr<Texture> destination,
                       std::optional<IRect> source_region,
                       IPoint destination_origin,
                       std::string_view label) {
  if (!source) {
    VALIDATION_LOG << "Attempted to add a texture blit with no source.";
    return false;
  }
  if (!destination) {
    VALIDATION_LOG << "Attempted to add a texture blit with no destination.";
    return false;
  }

  if (source->GetTextureDescriptor().sample_count !=
      destination->GetTextureDescriptor().sample_count) {
    VALIDATION_LOG << SPrintF(
        "The source sample count (%d) must match the destination sample count "
        "(%d) for blits.",
        static_cast<int>(source->GetTextureDescriptor().sample_count),
        static_cast<int>(destination->GetTextureDescriptor().sample_count));
    return false;
  }
  if (source->GetTextureDescriptor().format !=
      destination->GetTextureDescriptor().format) {
    VALIDATION_LOG << SPrintF(
        "The source pixel format (%s) must match the destination pixel format "
        "(%s) "
        "for blits.",
        PixelFormatToString(source->GetTextureDescriptor().format),
        PixelFormatToString(destination->GetTextureDescriptor().format));
    return false;
  }

  if (!source_region.has_value()) {
    source_region = IRect::MakeSize(source->GetSize());
  }

  // Clip the source image.
  source_region =
      source_region->Intersection(IRect::MakeSize(source->GetSize()));
  if (!source_region.has_value()) {
    return true;  // Nothing to blit.
  }

  return OnCopyTextureToTextureCommand(
      std::move(source), std::move(destination), source_region.value(),
      destination_origin, label);
}

bool BlitPass::AddCopy(std::shared_ptr<Texture> source,
                       std::shared_ptr<DeviceBuffer> destination,
                       std::optional<IRect> source_region,
                       size_t destination_offset,
                       std::string_view label) {
  if (!source) {
    VALIDATION_LOG << "Attempted to add a texture blit with no source.";
    return false;
  }
  if (!destination) {
    VALIDATION_LOG << "Attempted to add a texture blit with no destination.";
    return false;
  }

  if (!source_region.has_value()) {
    source_region = IRect::MakeSize(source->GetSize());
  }

  auto bytes_per_pixel =
      BytesPerPixelForPixelFormat(source->GetTextureDescriptor().format);
  auto bytes_per_image = source_region->Area() * bytes_per_pixel;
  if (destination_offset + bytes_per_image >
      destination->GetDeviceBufferDescriptor().size) {
    VALIDATION_LOG
        << "Attempted to add a texture blit with out of bounds access.";
    return false;
  }

  // Clip the source image.
  source_region =
      source_region->Intersection(IRect::MakeSize(source->GetSize()));
  if (!source_region.has_value()) {
    return true;  // Nothing to blit.
  }

  return OnCopyTextureToBufferCommand(std::move(source), std::move(destination),
                                      source_region.value(), destination_offset,
                                      label);
}

bool BlitPass::AddCopy(BufferView source,
                       std::shared_ptr<Texture> destination,
                       std::optional<IRect> destination_region,
                       std::string_view label,
                       uint32_t mip_level,
                       uint32_t slice,
                       bool convert_to_read) {
  if (!destination) {
    VALIDATION_LOG << "Attempted to add a texture blit with no destination.";
    return false;
  }
  ISize destination_size = destination->GetSize();
  IRect destination_region_value =
      destination_region.value_or(IRect::MakeSize(destination_size));
  if (destination_region_value.GetX() < 0 ||
      destination_region_value.GetY() < 0 ||
      destination_region_value.GetRight() > destination_size.width ||
      destination_region_value.GetBottom() > destination_size.height) {
    VALIDATION_LOG << "Blit region cannot be larger than destination texture.";
    return false;
  }

  auto bytes_per_pixel =
      BytesPerPixelForPixelFormat(destination->GetTextureDescriptor().format);
  auto bytes_per_region = destination_region_value.Area() * bytes_per_pixel;

  if (source.GetRange().length != bytes_per_region) {
    VALIDATION_LOG
        << "Attempted to add a texture blit with out of bounds access.";
    return false;
  }
  if (mip_level >= destination->GetMipCount()) {
    VALIDATION_LOG << "Invalid value for mip_level: " << mip_level << ". "
                   << "The destination texture has "
                   << destination->GetMipCount() << " mip levels.";
    return false;
  }
  if (slice > 5) {
    VALIDATION_LOG << "Invalid value for slice: " << slice;
    return false;
  }

  return OnCopyBufferToTextureCommand(std::move(source), std::move(destination),
                                      destination_region_value, label,
                                      mip_level, slice, convert_to_read);
}

bool BlitPass::ConvertTextureToShaderRead(
    const std::shared_ptr<Texture>& texture) {
  return true;
}

bool BlitPass::GenerateMipmap(std::shared_ptr<Texture> texture,
                              std::string_view label) {
  if (!texture) {
    VALIDATION_LOG << "Attempted to add an invalid mipmap generation command "
                      "with no texture.";
    return false;
  }

  return OnGenerateMipmapCommand(std::move(texture), label);
}

}  // namespace impeller
