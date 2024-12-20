// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEXTURE_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEXTURE_GLES_H_

#include <bitset>

#include "fml/logging.h"
#include "impeller/base/backend_cast.h"
#include "impeller/core/texture.h"
#include "impeller/renderer/backend/gles/handle_gles.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"

namespace impeller {

class TextureGLES final : public Texture,
                          public BackendCast<TextureGLES, Texture> {
 public:
  enum class Type {
    kTexture,
    kTextureMultisampled,
    kRenderBuffer,
    kRenderBufferMultisampled,
  };

  //----------------------------------------------------------------------------
  /// @brief      Create a texture by wrapping an external framebuffer object
  ///             whose lifecycle is owned by the caller.
  ///
  ///             This is useful for creating a render target for the default
  ///             window managed framebuffer.
  ///
  /// @param[in]  reactor  The reactor
  /// @param[in]  desc     The description
  /// @param[in]  fbo      The fbo
  ///
  /// @return     If a texture representation of the framebuffer could be
  ///             created.
  ///
  static std::shared_ptr<TextureGLES> WrapFBO(
      std::shared_ptr<ReactorGLES> reactor,
      TextureDescriptor desc,
      GLuint fbo);

  //----------------------------------------------------------------------------
  /// @brief      Create a texture by wrapping an external OpenGL texture
  ///             handle. Ownership of the texture handle is assumed by the
  ///             reactor.
  ///
  /// @param[in]  reactor          The reactor
  /// @param[in]  desc             The description
  /// @param[in]  external_handle  The external handle
  ///
  /// @return     If a texture representation of the framebuffer could be
  ///             created.
  ///
  static std::shared_ptr<TextureGLES> WrapTexture(
      std::shared_ptr<ReactorGLES> reactor,
      TextureDescriptor desc,
      HandleGLES external_handle);

  //----------------------------------------------------------------------------
  /// @brief      Create a "texture" that is never expected to be bound/unbound
  ///             explicitly or initialized in any way. It only exists to setup
  ///             a render pass description.
  ///
  /// @param[in]  reactor  The reactor
  /// @param[in]  desc     The description
  ///
  /// @return     If a texture placeholder could be created.
  ///
  static std::shared_ptr<TextureGLES> CreatePlaceholder(
      std::shared_ptr<ReactorGLES> reactor,
      TextureDescriptor desc);

  TextureGLES(std::shared_ptr<ReactorGLES> reactor, TextureDescriptor desc);

  // |Texture|
  ~TextureGLES() override;

  // |Texture|
  bool IsValid() const override;

  std::optional<GLuint> GetGLHandle() const;

  [[nodiscard]] bool Bind() const;

  [[nodiscard]] bool GenerateMipmap();

  enum class AttachmentType {
    kColor0,
    kDepth,
    kStencil,
  };
  [[nodiscard]] bool SetAsFramebufferAttachment(
      GLenum target,
      AttachmentType attachment_type) const;

  Type GetType() const;

  bool IsWrapped() const;

  std::optional<GLuint> GetFBO() const;

  //----------------------------------------------------------------------------
  /// @brief      Indicates that all texture storage has already been allocated
  ///             and contents initialized.
  ///
  ///             This is similar to calling `MarkSliceInitialized` with all
  ///             slices.
  ///
  /// @see        MarkSliceInitialized.
  ///
  void MarkContentsInitialized();

  //----------------------------------------------------------------------------
  /// @brief      Indicates that a specific texture slice has been initialized.
  ///
  /// @param[in]  slice  The slice to mark as being initialized.
  ///
  void MarkSliceInitialized(size_t slice) const;

  bool IsSliceInitialized(size_t slice) const;

  //----------------------------------------------------------------------------
  /// @brief      Attach a sync fence to this texture that will be waited on
  ///             before encoding a rendering operation that references it.
  ///
  /// @param[in]  fence  A handle to a sync fence.
  ///
  void SetFence(HandleGLES fence);

  /// Store the FBO object for recycling in the 2D renderer.
  ///
  /// The color0 texture used by the 2D renderer will use this texture
  /// object to store the associated FBO the first time it is used.
  void SetCachedFBO(GLuint fbo);

  /// Retrieve the cached FBO object, or GL_NONE if there is no object.
  GLuint GetCachedFBO() const;

  // Visible for testing.
  std::optional<HandleGLES> GetSyncFence() const;

 private:
  std::shared_ptr<ReactorGLES> reactor_;
  const Type type_;
  HandleGLES handle_;
  mutable std::optional<HandleGLES> fence_ = std::nullopt;
  mutable std::bitset<6> slices_initialized_ = 0;
  const bool is_wrapped_;
  const std::optional<GLuint> wrapped_fbo_;
  GLuint cached_fbo_ = GL_NONE;
  bool is_valid_ = false;

  TextureGLES(std::shared_ptr<ReactorGLES> reactor,
              TextureDescriptor desc,
              std::optional<GLuint> fbo,
              std::optional<HandleGLES> external_handle);

  // |Texture|
  void SetLabel(std::string_view label) override;

  // |Texture|
  void SetLabel(std::string_view label, std::string_view trailing) override;

  // |Texture|
  bool OnSetContents(const uint8_t* contents,
                     size_t length,
                     size_t slice) override;

  // |Texture|
  bool OnSetContents(std::shared_ptr<const fml::Mapping> mapping,
                     size_t slice) override;

  // |Texture|
  ISize GetSize() const override;

  // |Texture|
  Scalar GetYCoordScale() const override;

  void InitializeContentsIfNecessary() const;

  TextureGLES(const TextureGLES&) = delete;

  TextureGLES& operator=(const TextureGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEXTURE_GLES_H_
