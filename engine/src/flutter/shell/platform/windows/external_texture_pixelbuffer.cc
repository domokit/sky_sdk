// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/external_texture_pixelbuffer.h"

namespace flutter {

ExternalTexturePixelBuffer::ExternalTexturePixelBuffer(
    const FlutterDesktopPixelBufferTextureCallback texture_callback,
    void* user_data,
    std::shared_ptr<egl::ProcTable> gl)
    : texture_callback_(texture_callback),
      user_data_(user_data),
      gl_(std::move(gl)) {}

ExternalTexturePixelBuffer::~ExternalTexturePixelBuffer() {
  if (gl_texture_ != 0) {
    gl_->DeleteTextures(1, &gl_texture_);
  }
}

bool ExternalTexturePixelBuffer::PopulateTexture(
    size_t width,
    size_t height,
    FlutterOpenGLTexture* opengl_texture) {
  if (!CopyPixelBuffer(width, height)) {
    return false;
  }

  // Populate the texture object used by the engine.
  opengl_texture->target = GL_TEXTURE_2D;
  opengl_texture->name = gl_texture_;
  opengl_texture->format = GL_RGBA8_OES;
  opengl_texture->destruction_callback = nullptr;
  opengl_texture->user_data = nullptr;
  opengl_texture->width = width;
  opengl_texture->height = height;

  return true;
}

bool ExternalTexturePixelBuffer::CopyPixelBuffer(size_t& width,
                                                 size_t& height) {
  const FlutterDesktopPixelBuffer* pixel_buffer =
      texture_callback_(width, height, user_data_);
  if (!pixel_buffer || !pixel_buffer->buffer) {
    return false;
  }
  width = pixel_buffer->width;
  height = pixel_buffer->height;

  if (gl_texture_ == 0) {
    gl_->GenTextures(1, &gl_texture_);

    gl_->BindTexture(GL_TEXTURE_2D, gl_texture_);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  } else {
    gl_->BindTexture(GL_TEXTURE_2D, gl_texture_);
  }
  gl_->TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pixel_buffer->width,
                  pixel_buffer->height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                  pixel_buffer->buffer);
  if (pixel_buffer->release_callback) {
    pixel_buffer->release_callback(pixel_buffer->release_context);
  }
  return true;
}

}  // namespace flutter
