// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_BLIT_PASS_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_BLIT_PASS_VK_H_

#include "flutter/impeller/base/config.h"
#include "impeller/geometry/rect.h"
#include "impeller/renderer/blit_pass.h"

namespace impeller {

class CommandEncoderVK;
class CommandBufferVK;

class BlitPassVK final : public BlitPass {
 public:
  // |BlitPass|
  ~BlitPassVK() override;

 private:
  friend class CommandBufferVK;

  std::shared_ptr<CommandBufferVK> command_buffer_;

  explicit BlitPassVK(std::shared_ptr<CommandBufferVK> command_buffer);

  // |BlitPass|
  bool IsValid() const override;

  // |BlitPass|
  void OnSetLabel(std::string_view label) override;

  // |BlitPass|
  bool EncodeCommands(
      const std::shared_ptr<Allocator>& transients_allocator) const override;

  // |BlitPass|
  bool ResizeTexture(const std::shared_ptr<Texture>& source,
                     const std::shared_ptr<Texture>& destination) override;

  // |BlitPass|
  bool ConvertTextureToShaderRead(
      const std::shared_ptr<Texture>& texture) override;

  // |BlitPass|
  bool OnCopyTextureToTextureCommand(std::shared_ptr<Texture> source,
                                     std::shared_ptr<Texture> destination,
                                     IRect source_region,
                                     IPoint destination_origin,
                                     std::string_view label) override;

  // |BlitPass|
  bool OnCopyTextureToBufferCommand(std::shared_ptr<Texture> source,
                                    std::shared_ptr<DeviceBuffer> destination,
                                    IRect source_region,
                                    size_t destination_offset,
                                    std::string_view label) override;

  // |BlitPass|
  bool OnCopyBufferToTextureCommand(BufferView source,
                                    std::shared_ptr<Texture> destination,
                                    IRect destination_region,
                                    std::string_view label,
                                    uint32_t mip_level,
                                    uint32_t slice,
                                    bool convert_to_read) override;
  // |BlitPass|
  bool OnGenerateMipmapCommand(std::shared_ptr<Texture> texture,
                               std::string_view label) override;

  BlitPassVK(const BlitPassVK&) = delete;

  BlitPassVK& operator=(const BlitPassVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_BLIT_PASS_VK_H_
