// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_TEXTURE_RESOLVER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_TEXTURE_RESOLVER_H_

#include <memory>

#include "flutter/common/graphics/texture.h"

#ifdef SHELL_ENABLE_GL
#include "flutter/shell/platform/embedder/embedder_external_texture_gl.h"
#endif

#ifdef SHELL_ENABLE_METAL
#include "flutter/shell/platform/embedder/embedder_external_texture_metal.h"
#endif

namespace flutter {
class EmbedderExternalTextureResolver {
 public:
  EmbedderExternalTextureResolver() = default;

  ~EmbedderExternalTextureResolver() = default;

#ifdef SHELL_ENABLE_GL
  explicit EmbedderExternalTextureResolver(
      EmbedderExternalTextureGL::ExternalTextureCallback gl_callback);
#endif

#ifdef SHELL_ENABLE_METAL
  explicit EmbedderExternalTextureResolver(
      EmbedderExternalTextureMetal::ExternalTextureCallback metal_callback);
#endif

  std::unique_ptr<Texture> ResolveExternalTexture(int64_t texture_id);

  bool SupportsExternalTextures();

 private:
#ifdef SHELL_ENABLE_GL
  EmbedderExternalTextureGL::ExternalTextureCallback gl_callback_;
#endif

#ifdef SHELL_ENABLE_METAL
  EmbedderExternalTextureMetal::ExternalTextureCallback metal_callback_;
#endif

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderExternalTextureResolver);
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_TEXTURE_RESOLVER_H_
