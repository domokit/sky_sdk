// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_JNI_PLATFORM_VIEW_ANDROID_JNI_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_JNI_PLATFORM_VIEW_ANDROID_JNI_H_

#include <utility>

#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"

#include "flutter/flow/embedded_views.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/platform/android/surface/android_native_window.h"
#include "third_party/skia/include/core/SkMatrix.h"

#if FML_OS_ANDROID
#include "flutter/fml/platform/android/scoped_java_ref.h"
#endif

namespace flutter {

#if FML_OS_ANDROID
using JavaLocalRef = fml::jni::ScopedJavaLocalRef<jobject>;
#else
using JavaLocalRef = std::nullptr_t;
#endif

//------------------------------------------------------------------------------
/// Allows to call Java code running in the JVM from any thread. However, most
/// methods can only be called from the platform thread as that is where the
/// Java code runs.
///
/// This interface must not depend on the Android toolchain directly, so it can
/// be used in unit tests compiled with the host toolchain.
///
class PlatformViewAndroidJNI {
 public:
  virtual ~PlatformViewAndroidJNI();

  //----------------------------------------------------------------------------
  /// @brief      Sends a platform message. The message may be empty.
  ///
  virtual void FlutterViewHandlePlatformMessage(
      std::unique_ptr<flutter::PlatformMessage> message,
      int responseId) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Responds to a platform message. The data may be a `nullptr`.
  ///
  virtual void FlutterViewHandlePlatformMessageResponse(
      int responseId,
      std::unique_ptr<fml::Mapping> data) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Sends semantics tree updates.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewUpdateSemantics(
      std::vector<uint8_t> buffer,
      std::vector<std::string> strings,
      std::vector<std::vector<uint8_t>> string_attribute_args) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Sends new custom accessibility events.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewUpdateCustomAccessibilityActions(
      std::vector<uint8_t> actions_buffer,
      std::vector<std::string> strings) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Indicates that FlutterView should start painting pixels.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewOnFirstFrame() = 0;

  //----------------------------------------------------------------------------
  /// @brief      Indicates that a hot restart is about to happen.
  ///
  virtual void FlutterViewOnPreEngineRestart() = 0;

  //----------------------------------------------------------------------------
  /// @brief      Attach the SurfaceTexture to the OpenGL ES context that is
  ///             current on the calling thread.
  ///
  virtual void SurfaceTextureAttachToGLContext(JavaLocalRef surface_texture,
                                               int textureId) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Returns true if surface_texture should be updated.
  ///
  virtual bool SurfaceTextureShouldUpdate(JavaLocalRef surface_texture) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Updates the texture image to the most recent frame from the
  ///             image stream.
  ///
  virtual void SurfaceTextureUpdateTexImage(JavaLocalRef surface_texture) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Gets the transform matrix from the SurfaceTexture.
  ///             Then, it updates the `transform` matrix, so it fill the canvas
  ///             and preserve the aspect ratio.
  ///
  virtual SkM44 SurfaceTextureGetTransformMatrix(
      JavaLocalRef surface_texture) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Detaches a SurfaceTexture from the OpenGL ES context.
  ///
  virtual void SurfaceTextureDetachFromGLContext(
      JavaLocalRef surface_texture) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Acquire the latest image available.
  ///
  virtual JavaLocalRef ImageProducerTextureEntryAcquireLatestImage(
      JavaLocalRef image_texture_entry) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Grab the HardwareBuffer from image.
  ///
  virtual JavaLocalRef ImageGetHardwareBuffer(JavaLocalRef image) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Call close on image.
  ///
  virtual void ImageClose(JavaLocalRef image) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Call close on hardware_buffer.
  ///
  virtual void HardwareBufferClose(JavaLocalRef hardware_buffer) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Positions and sizes a platform view if using hybrid
  ///             composition.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewOnDisplayPlatformView(
      int view_id,
      int x,
      int y,
      int width,
      int height,
      int viewWidth,
      int viewHeight,
      MutatorsStack mutators_stack) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Positions and sizes an overlay surface in hybrid composition.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewDisplayOverlaySurface(int surface_id,
                                                int x,
                                                int y,
                                                int width,
                                                int height) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Initiates a frame if using hybrid composition.
  ///
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewBeginFrame() = 0;

  //----------------------------------------------------------------------------
  /// @brief      Indicates that the current frame ended.
  ///             It's used to clean up state.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewEndFrame() = 0;

  //------------------------------------------------------------------------------
  /// The metadata returned from Java which is converted into an |OverlayLayer|
  /// by |SurfacePool|.
  ///
  struct OverlayMetadata {
    OverlayMetadata(int id, fml::RefPtr<AndroidNativeWindow> window)
        : id(id), window(std::move(window)){};

    ~OverlayMetadata() = default;

    // A unique id to identify the overlay when it gets recycled.
    const int id;

    // Holds a reference to the native window. That is, an `ANativeWindow`,
    // which is the C counterpart of the `android.view.Surface` object in Java.
    const fml::RefPtr<AndroidNativeWindow> window;
  };

  //----------------------------------------------------------------------------
  /// @brief      Instantiates an overlay surface in hybrid composition and
  ///             provides the necessary metadata to operate the surface in C.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual std::unique_ptr<PlatformViewAndroidJNI::OverlayMetadata>
  FlutterViewCreateOverlaySurface() = 0;

  //----------------------------------------------------------------------------
  /// @brief      Destroys the overlay surfaces.
  ///
  /// @note       Must be called from the platform thread.
  ///
  virtual void FlutterViewDestroyOverlaySurfaces() = 0;

  //----------------------------------------------------------------------------
  /// @brief      Computes the locale Android would select.
  ///
  virtual std::unique_ptr<std::vector<std::string>>
  FlutterViewComputePlatformResolvedLocale(
      std::vector<std::string> supported_locales_data) = 0;

  virtual double GetDisplayRefreshRate() = 0;

  virtual double GetDisplayWidth() = 0;

  virtual double GetDisplayHeight() = 0;

  virtual double GetDisplayDensity() = 0;

  virtual bool RequestDartDeferredLibrary(int loading_unit_id) = 0;

  virtual double FlutterViewGetScaledFontSize(double unscaled_font_size,
                                              int configuration_id) const = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_JNI_PLATFORM_VIEW_ANDROID_JNI_H_
