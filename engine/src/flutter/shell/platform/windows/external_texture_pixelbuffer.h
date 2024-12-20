// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_PIXELBUFFER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_PIXELBUFFER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/public/flutter_texture_registrar.h"
#include "flutter/shell/platform/windows/egl/proc_table.h"
#include "flutter/shell/platform/windows/external_texture.h"

namespace flutter {

// An abstraction of an pixel-buffer based texture.
class ExternalTexturePixelBuffer : public ExternalTexture {
 public:
  ExternalTexturePixelBuffer(
      const FlutterDesktopPixelBufferTextureCallback texture_callback,
      void* user_data,
      std::shared_ptr<egl::ProcTable> gl);

  virtual ~ExternalTexturePixelBuffer();

  // |ExternalTexture|
  bool PopulateTexture(size_t width,
                       size_t height,
                       FlutterOpenGLTexture* opengl_texture) override;

 private:
  // Attempts to copy the pixel buffer returned by |texture_callback_| to
  // OpenGL.
  // The |width| and |height| will be set to the actual bounds of the copied
  // pixel buffer.
  // Returns true on success or false if the pixel buffer returned
  // by |texture_callback_| was invalid.
  bool CopyPixelBuffer(size_t& width, size_t& height);

  const FlutterDesktopPixelBufferTextureCallback texture_callback_ = nullptr;
  void* const user_data_ = nullptr;
  std::shared_ptr<egl::ProcTable> gl_;
  GLuint gl_texture_ = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(ExternalTexturePixelBuffer);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_PIXELBUFFER_H_
