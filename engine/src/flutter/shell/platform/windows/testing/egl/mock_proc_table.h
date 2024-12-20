// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_PROC_TABLE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_PROC_TABLE_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/proc_table.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {
namespace egl {

/// Mock for the |ProcTable| base class.
class MockProcTable : public ::flutter::egl::ProcTable {
 public:
  MockProcTable() = default;
  virtual ~MockProcTable() = default;

  MOCK_METHOD(void,
              GenTextures,
              (GLsizei n, GLuint* textures),
              (const override));
  MOCK_METHOD(void,
              DeleteTextures,
              (GLsizei n, const GLuint* textures),
              (const override));
  MOCK_METHOD(void,
              BindTexture,
              (GLenum target, GLuint texture),
              (const override));
  MOCK_METHOD(void,
              TexParameteri,
              (GLenum target, GLenum pname, GLint param),
              (const override));
  MOCK_METHOD(void,
              TexImage2D,
              (GLenum target,
               GLint level,
               GLint internalformat,
               GLsizei width,
               GLsizei height,
               GLint border,
               GLenum format,
               GLenum type,
               const void* data),
              (const override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockProcTable);
};

}  // namespace egl
}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_PROC_TABLE_H_
