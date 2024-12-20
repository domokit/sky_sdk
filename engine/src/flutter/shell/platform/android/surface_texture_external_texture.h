// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_H_

#include <GLES/gl.h>

#include "flutter/common/graphics/texture.h"
#include "flutter/shell/platform/android/platform_view_android_jni_impl.h"
#include "flutter/third_party/skia/include/core/SkM44.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Instances of external textures peered to
///             `android.graphics.SurfaceTexture`.
///
///             SurfaceTextures are used on older versions of Android (API <
///             29). On newer versions, the Android Hardware Buffer backend
///             `flutter::ImageExternalTexture` instances are used instead.
///
///             Due to the way surface textures are designed, it is not possible
///             to have a Vulkan renderer interoperate with such textures.
///             Consequently, both Skia and Impeller only have OpenGL
///             implementations for these kinds of textures.
///
///             This is an abstract base class. Minimally, subclasses override
///             the pure virtual `ProcessFrame` method to bind the package
///             specific texture implementation to the surface texture.
///
class SurfaceTextureExternalTexture : public flutter::Texture {
 public:
  SurfaceTextureExternalTexture(
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);

  // |Texture|
  ~SurfaceTextureExternalTexture() override;

 protected:
  //----------------------------------------------------------------------------
  /// @brief      Subclasses override this method to bind the OpenGL texture
  ///             resource represented by this surface texture to the package
  ///             specific texture (SkImage, impeller::Texture, etc...).
  ///
  /// @important  The state of texture should be AttachmentState::kAttached
  ///             after a call to this method. That is the responsibility of the
  ///             subclass.
  ///
  /// @param      context  The context.
  /// @param[in]  bounds   The bounds of the texture.
  ///
  virtual void ProcessFrame(PaintContext& context, const SkRect& bounds) = 0;

  virtual void DrawFrame(PaintContext& context,
                         const SkRect& bounds,
                         const DlImageSampling sampling) const;

  //----------------------------------------------------------------------------
  /// @brief      Get the transformation that should be applied to the UV
  ///             texture coordinates when sampling from this texture.
  ///
  /// @return     The current uv transformation.
  ///
  const SkM44& GetCurrentUVTransformation() const;

  //----------------------------------------------------------------------------
  /// @brief      Provides an opportunity for the subclasses to sever the
  ///             connection between the OpenGL texture resource represented by
  ///             this surface texture and the underlying package handle
  ///             (SkImage, impeller::Texture, etc...).
  ///
  /// @important  It is the responsibility of the subclass to ensure that a
  ///             context is current when this call is made. Subclass can do
  ///             this by overriding this method, making the context current in
  ///             the implementation and calling the base class method.
  ///
  virtual void Detach();

  //----------------------------------------------------------------------------
  /// @brief      Attaches the given OpenGL texture handle to the surface
  ///             texture via a bind operation.
  ///
  /// @important  It is the responsibility of the subclass to ensure that a
  ///             context is current when this call is made. Subclass can do
  ///             this by overriding this method, making the context current in
  ///             the implementation and calling the base class method.
  ///
  /// @param[in]  gl_tex_id  The gl tex identifier
  ///
  void Attach(int gl_tex_id);

  bool ShouldUpdate();

  //----------------------------------------------------------------------------
  /// @brief      Update the surface texture contents and transformation matrix.
  ///
  void Update();

  //----------------------------------------------------------------------------
  /// @brief      Specifies how this instance is bound to the underlying surface
  ///             texture.
  ///
  enum class AttachmentState { kUninitialized, kAttached, kDetached };

  std::shared_ptr<PlatformViewAndroidJNI> jni_facade_;
  fml::jni::ScopedJavaGlobalRef<jobject> surface_texture_;
  AttachmentState state_ = AttachmentState::kUninitialized;
  sk_sp<flutter::DlImage> dl_image_;

 private:
  SkM44 transform_;

  // |Texture|
  void Paint(PaintContext& context,
             const SkRect& bounds,
             bool freeze,
             const DlImageSampling sampling) override;

  // |ContextListener|
  void OnGrContextCreated() override;

  // |ContextListener|
  void OnGrContextDestroyed() override;

  // |Texture|
  void MarkNewFrameAvailable() override;

  // |Texture|
  void OnTextureUnregistered() override;

  FML_DISALLOW_COPY_AND_ASSIGN(SurfaceTextureExternalTexture);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_H_
