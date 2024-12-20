// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/egl/proc_table.h"

#include <EGL/egl.h>

namespace flutter {
namespace egl {

std::shared_ptr<ProcTable> ProcTable::Create() {
  auto gl = std::shared_ptr<ProcTable>(new ProcTable());

  gl->gen_textures_ =
      reinterpret_cast<GenTexturesProc>(::eglGetProcAddress("glGenTextures"));
  gl->delete_textures_ = reinterpret_cast<DeleteTexturesProc>(
      ::eglGetProcAddress("glDeleteTextures"));
  gl->bind_texture_ =
      reinterpret_cast<BindTextureProc>(::eglGetProcAddress("glBindTexture"));
  gl->tex_parameteri_ = reinterpret_cast<TexParameteriProc>(
      ::eglGetProcAddress("glTexParameteri"));
  gl->tex_image_2d_ =
      reinterpret_cast<TexImage2DProc>(::eglGetProcAddress("glTexImage2D"));

  if (!gl->gen_textures_ || !gl->delete_textures_ || !gl->bind_texture_ ||
      !gl->tex_parameteri_ || !gl->tex_image_2d_) {
    return nullptr;
  }

  return gl;
}

ProcTable::ProcTable() = default;

ProcTable::~ProcTable() = default;

void ProcTable::GenTextures(GLsizei n, GLuint* textures) const {
  gen_textures_(n, textures);
}

void ProcTable::DeleteTextures(GLsizei n, const GLuint* textures) const {
  delete_textures_(n, textures);
}

void ProcTable::BindTexture(GLenum target, GLuint texture) const {
  bind_texture_(target, texture);
}

void ProcTable::TexParameteri(GLenum target, GLenum pname, GLint param) const {
  tex_parameteri_(target, pname, param);
}

void ProcTable::TexImage2D(GLenum target,
                           GLint level,
                           GLint internalformat,
                           GLsizei width,
                           GLsizei height,
                           GLint border,
                           GLenum format,
                           GLenum type,
                           const void* data) const {
  tex_image_2d_(target, level, internalformat, width, height, border, format,
                type, data);
}

}  // namespace egl
}  // namespace flutter
