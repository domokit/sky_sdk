// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BLIT_COMMAND_H_
#define FLUTTER_IMPELLER_RENDERER_BLIT_COMMAND_H_

#include <cstdint>
#include "impeller/core/device_buffer.h"
#include "impeller/core/texture.h"
#include "impeller/geometry/rect.h"

namespace impeller {

struct BlitCommand {
  std::string label;
};

struct BlitCopyTextureToTextureCommand : public BlitCommand {
  std::shared_ptr<Texture> source;
  std::shared_ptr<Texture> destination;
  IRect source_region;
  IPoint destination_origin;
};

struct BlitResizeTextureCommand : public BlitCommand {
  std::shared_ptr<Texture> source;
  std::shared_ptr<Texture> destination;
};

struct BlitCopyTextureToBufferCommand : public BlitCommand {
  std::shared_ptr<Texture> source;
  std::shared_ptr<DeviceBuffer> destination;
  IRect source_region;
  size_t destination_offset;
};

struct BlitCopyBufferToTextureCommand : public BlitCommand {
  BufferView source;
  std::shared_ptr<Texture> destination;
  IRect destination_region;
  uint32_t mip_level = 0;
  uint32_t slice = 0;
};

struct BlitGenerateMipmapCommand : public BlitCommand {
  std::shared_ptr<Texture> texture;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BLIT_COMMAND_H_
