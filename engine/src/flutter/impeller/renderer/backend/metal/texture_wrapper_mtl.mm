// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/texture_wrapper_mtl.h"

#include <Metal/Metal.h>

#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"

namespace impeller {

std::shared_ptr<Texture> WrapTextureMTL(TextureDescriptor desc,
                                        const void* mtl_texture,
                                        std::function<void()> deletion_proc) {
  auto texture = (__bridge id<MTLTexture>)mtl_texture;
  desc.format = FromMTLPixelFormat(texture.pixelFormat);
  return TextureMTL::Wrapper(desc, texture, std::move(deletion_proc));
}

}  // namespace impeller
