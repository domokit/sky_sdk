// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/gradient_generator.h"

#include "flutter/fml/logging.h"
#include "impeller/base/strings.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture.h"
#include "impeller/renderer/context.h"

namespace impeller {

std::shared_ptr<Texture> CreateGradientTexture(
    const GradientData& gradient_data,
    const std::shared_ptr<impeller::Context>& context) {
  if (gradient_data.texture_size == 0) {
    FML_DLOG(ERROR) << "Invalid gradient data.";
    return nullptr;
  }

  impeller::TextureDescriptor texture_descriptor;
  texture_descriptor.storage_mode = impeller::StorageMode::kHostVisible;
  texture_descriptor.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_descriptor.size = {gradient_data.texture_size, 1};
  auto texture =
      context->GetResourceAllocator()->CreateTexture(texture_descriptor);
  if (!texture) {
    FML_DLOG(ERROR) << "Could not create Impeller texture.";
    return nullptr;
  }

  auto data_mapping =
      std::make_shared<fml::DataMapping>(gradient_data.color_bytes);
  auto buffer =
      context->GetResourceAllocator()->CreateBufferWithCopy(*data_mapping);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();
  blit_pass->AddCopy(DeviceBuffer::AsBufferView(std::move(buffer)), texture);

  if (!blit_pass->EncodeCommands(context->GetResourceAllocator()) ||
      !context->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok()) {
    return nullptr;
  }

  texture->SetLabel(impeller::SPrintF("Gradient(%p)", texture.get()).c_str());
  return texture;
}

std::vector<StopData> CreateGradientColors(const std::vector<Color>& colors,
                                           const std::vector<Scalar>& stops) {
  FML_DCHECK(stops.size() == colors.size());

  std::vector<StopData> result;
  result.reserve(stops.size());
  Scalar last_stop = 0;
  for (auto i = 0u; i < stops.size(); i++) {
    Scalar delta = stops[i] - last_stop;
    Scalar inverse_delta = delta == 0.0f ? 0.0 : 1.0 / delta;
    result.emplace_back(StopData{
        .color = colors[i], .stop = stops[i], .inverse_delta = inverse_delta});
    last_stop = stops[i];
  }
  return result;
}

int PopulateUniformGradientColors(
    const std::vector<Color>& colors,
    const std::vector<Scalar>& stops,
    Vector4 frag_info_colors[kMaxUniformGradientStops],
    Vector4 frag_info_stop_pairs[kMaxUniformGradientStops / 2]) {
  FML_DCHECK(stops.size() == colors.size());

  Scalar last_stop = 0;
  int index = 0;
  for (auto i = 0u; i < stops.size() && i < kMaxUniformGradientStops; i++) {
    Scalar cur_stop = stops[i];
    Scalar delta = cur_stop - last_stop;
    Scalar inverse_delta = delta == 0.0f ? 0.0 : 1.0 / delta;
    frag_info_colors[index] = colors[i];
    if ((i & 1) == 0) {
      frag_info_stop_pairs[index / 2].x = cur_stop;
      frag_info_stop_pairs[index / 2].y = inverse_delta;
    } else {
      frag_info_stop_pairs[index / 2].z = cur_stop;
      frag_info_stop_pairs[index / 2].w = inverse_delta;
    }
    last_stop = cur_stop;
    index++;
  }
  return index;
}

}  // namespace impeller
