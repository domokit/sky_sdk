// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_GLVK_TRAMPOLINE_H_
#define FLUTTER_IMPELLER_TOOLKIT_GLVK_TRAMPOLINE_H_

#include "impeller/base/thread_safety.h"
#include "impeller/renderer/backend/vulkan/android/ahb_texture_source_vk.h"
#include "impeller/toolkit/egl/context.h"
#include "impeller/toolkit/egl/display.h"
#include "impeller/toolkit/egl/surface.h"
#include "impeller/toolkit/glvk/proc_table.h"

namespace impeller::glvk {

class AutoTrampolineContext;

//------------------------------------------------------------------------------
/// @brief      An object used to interoperate between OpenGL and Vulkan.
///
///             While these are not super expensive to create, they do manage an
///             internal EGL context as well as some OpenGL state. For this
///             reason, it is recommended that callers cache these for the
///             duration of the lifecycle of main rendering context.
///
class Trampoline {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Constructs a new trampoline. It is recommended that these
  ///             objects be cached and reused for all conversion operations.
  ///
  ///             EGL contexts on already bound to the callers thread may become
  ///             unbound after a call to this method.
  ///
  Trampoline();

  //----------------------------------------------------------------------------
  /// @brief      Destroys the trampoline. There are no threading restrictions.
  ///             EGL contexts on already bound to the callers thread may become
  ///             unbound after a call to this method.
  ///
  ~Trampoline();

  Trampoline(const Trampoline&) = delete;

  Trampoline& operator=(const Trampoline&) = delete;

  //----------------------------------------------------------------------------
  /// @brief      Determines if this is a valid trampoline. There is no error
  ///             recovery mechanism if a trampoline cannot be constructed and
  ///             an invalid trampoline must be immediately discarded.
  ///
  /// @return     True if valid, False otherwise.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Describes an OpenGL texture along with information on how to
  ///             sample from it.
  ///
  struct GLTextureInfo {
    //--------------------------------------------------------------------------
    /// The OpenGL texture handle.
    ///
    GLuint texture = 0;
    //--------------------------------------------------------------------------
    /// The OpenGL texture target enum. For instance, GL_TEXTURE_2D or
    /// GL_TEXTURE_EXTERNAL_OES.
    ///
    GLenum target = 0;
    //--------------------------------------------------------------------------
    /// A transformation applied to the texture coordinates in the form of (u,
    /// v, 0, 1) when sampling from the texture.
    ///
    Matrix uv_transformation;
  };

  //----------------------------------------------------------------------------
  /// @brief      Perform a blit operation from the source OpenGL texture to a
  ///             target Vulkan texture.
  ///
  ///             It is the callers responsibility to ensure that the EGL
  ///             context associated with the trampoline is already current
  ///             before making this call.
  ///
  ///             It is also the responsibility of the caller to ensure that the
  ///             destination texture is the color-attachment-optimal layout.
  ///             Failure to ensure this will lead to validation error.
  ///
  /// @see        `MakeCurrentContext`
  ///
  /// @param[in]  src_texture    The source OpenGL texture.
  /// @param[in]  dst_texture    The destination Vulkan texture.
  ///
  /// @return     True if the blit was successful, False otherwise.
  ///
  bool BlitTextureOpenGLToVulkan(const GLTextureInfo& src_texture,
                                 const AHBTextureSourceVK& dst_texture) const;

  //----------------------------------------------------------------------------
  /// @brief      Make the EGL context associated with this trampoline current
  ///             on the calling thread.
  ///
  /// @return     The automatic trampoline context. The collection of this
  ///             context clears the threads EGL binding.
  ///
  [[nodiscard]] AutoTrampolineContext MakeCurrentContext() const;

 private:
  friend class AutoTrampolineContext;

  std::unique_ptr<egl::Display> egl_display_;
  std::unique_ptr<egl::Context> egl_context_;
  std::unique_ptr<egl::Surface> egl_surface_;
  std::unique_ptr<ProcTable> gl_;
  GLuint program_ = GL_NONE;
  GLint texture_uniform_location_ = 0;
  GLint uv_transformation_location_ = 0;
  bool is_valid_ = false;
};

//------------------------------------------------------------------------------
/// @brief      An RAII object that makes the trampolines EGL context current
///             when constructed and clears the EGL binding on destruction.
///
class AutoTrampolineContext final {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Constructs a new instance and makes the trampolines EGL
  ///             context current on the calling thread.
  ///
  /// @param[in]  trampoline  The trampoline.
  ///
  explicit AutoTrampolineContext(const Trampoline& trampoline);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the object and clears the previous EGL binding.
  ///
  ~AutoTrampolineContext();

  AutoTrampolineContext(const AutoTrampolineContext&) = delete;

  AutoTrampolineContext& operator=(const AutoTrampolineContext&) = delete;

 private:
  const egl::Context* context_ = nullptr;
  const egl::Surface* surface_ = nullptr;
};

}  // namespace impeller::glvk

#endif  // FLUTTER_IMPELLER_TOOLKIT_GLVK_TRAMPOLINE_H_
