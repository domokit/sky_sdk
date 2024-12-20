// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_TEXTURE_MIPMAP_H_
#define FLUTTER_IMPELLER_RENDERER_TEXTURE_MIPMAP_H_

#include "flutter/fml/status.h"
#include "impeller/core/texture.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/context.h"

namespace impeller {

/// Adds a blit command to the render pass.
[[nodiscard]] fml::Status AddMipmapGeneration(
    const std::shared_ptr<CommandBuffer>& command_buffer,
    const std::shared_ptr<Context>& context,
    const std::shared_ptr<Texture>& texture);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_TEXTURE_MIPMAP_H_
