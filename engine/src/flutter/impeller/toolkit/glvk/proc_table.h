// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_GLVK_PROC_TABLE_H_
#define FLUTTER_IMPELLER_TOOLKIT_GLVK_PROC_TABLE_H_

#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/toolkit/gles/gles.h"

namespace impeller::glvk {

#define FOR_EACH_GLVK_PROC(PROC)   \
  PROC(ActiveTexture)              \
  PROC(AttachShader)               \
  PROC(BindAttribLocation)         \
  PROC(BindBuffer)                 \
  PROC(BindFramebuffer)            \
  PROC(BindTexture)                \
  PROC(BufferData)                 \
  PROC(CheckFramebufferStatus)     \
  PROC(Clear)                      \
  PROC(ClearColor)                 \
  PROC(ColorMask)                  \
  PROC(CompileShader)              \
  PROC(CreateProgram)              \
  PROC(CreateShader)               \
  PROC(DeleteBuffers)              \
  PROC(DeleteFramebuffers)         \
  PROC(DeleteProgram)              \
  PROC(DeleteShader)               \
  PROC(DeleteTextures)             \
  PROC(Disable)                    \
  PROC(DrawArrays)                 \
  PROC(EGLImageTargetTexture2DOES) \
  PROC(Enable)                     \
  PROC(EnableVertexAttribArray)    \
  PROC(Flush)                      \
  PROC(FramebufferTexture2D)       \
  PROC(GenBuffers)                 \
  PROC(GenFramebuffers)            \
  PROC(GenTextures)                \
  PROC(GetProgramiv)               \
  PROC(GetShaderiv)                \
  PROC(GetUniformLocation)         \
  PROC(LinkProgram)                \
  PROC(ShaderSource)               \
  PROC(TexParameteri)              \
  PROC(Uniform1i)                  \
  PROC(UniformMatrix4fv)           \
  PROC(UseProgram)                 \
  PROC(VertexAttribPointer)        \
  PROC(Viewport)

//------------------------------------------------------------------------------
/// @brief      A proc. table consisting of methods that are useful when
///             interoperating between OpenGL and Vulkan. This is different from
///             the OpenGL proc. table since it may contain more interop
///             extension related methods.
///
class ProcTable {
 public:
  using Resolver = std::function<void*(const char* function_name)>;

  //----------------------------------------------------------------------------
  /// @brief      Create a proc table using a resolver to resolve OpenGL
  ///             methods.
  ///
  /// @param[in]  resolver  The resolver
  ///
  explicit ProcTable(const Resolver& resolver);

  ~ProcTable();

  ProcTable(const ProcTable&) = delete;

  ProcTable& operator=(const ProcTable&) = delete;

  //----------------------------------------------------------------------------
  /// @brief      Determines if a proc. table is suitable for interop purposes.
  ///             The absence of optional extension methods that have fallbacks
  ///             don't result in an invalid proc. table. But an invalid proc
  ///             table must always be discarded as there can be no error
  ///             recovery.
  ///
  /// @return     True if valid, False otherwise.
  ///
  bool IsValid() const;

#define GLVK_PROC(name) GLProc<decltype(gl##name)> name = {"gl" #name, nullptr};

  FOR_EACH_GLVK_PROC(GLVK_PROC);

#undef GLVK_PROC

 private:
  bool is_valid_ = false;
};

}  // namespace impeller::glvk

#endif  // FLUTTER_IMPELLER_TOOLKIT_GLVK_PROC_TABLE_H_
