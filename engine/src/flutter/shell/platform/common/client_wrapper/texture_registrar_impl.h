// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TEXTURE_REGISTRAR_IMPL_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TEXTURE_REGISTRAR_IMPL_H_

#include "include/flutter/texture_registrar.h"

namespace flutter {

// Wrapper around a FlutterDesktopTextureRegistrarRef that implements the
// TextureRegistrar API.
class TextureRegistrarImpl : public TextureRegistrar {
 public:
  explicit TextureRegistrarImpl(
      FlutterDesktopTextureRegistrarRef texture_registrar_ref);
  virtual ~TextureRegistrarImpl();

  // Prevent copying.
  TextureRegistrarImpl(TextureRegistrarImpl const&) = delete;
  TextureRegistrarImpl& operator=(TextureRegistrarImpl const&) = delete;

  // |flutter::TextureRegistrar|
  int64_t RegisterTexture(TextureVariant* texture) override;

  // |flutter::TextureRegistrar|
  bool MarkTextureFrameAvailable(int64_t texture_id) override;

  // |flutter::TextureRegistrar|
  void UnregisterTexture(int64_t texture_id,
                         std::function<void()> callback) override;

  // |flutter::TextureRegistrar|
  bool UnregisterTexture(int64_t texture_id) override;

 private:
  // Handle for interacting with the C API.
  FlutterDesktopTextureRegistrarRef texture_registrar_ref_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TEXTURE_REGISTRAR_IMPL_H_
