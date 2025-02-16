// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_JNI_IMPL_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_JNI_IMPL_H_

#include "flutter/fml/platform/android/jni_weak_ref.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Concrete implementation of `PlatformViewAndroidJNI` that is
///             compiled with the Android toolchain.
///
class PlatformViewAndroidJNIImpl final : public PlatformViewAndroidJNI {
 public:
  explicit PlatformViewAndroidJNIImpl(
      const fml::jni::JavaObjectWeakGlobalRef& java_object);

  ~PlatformViewAndroidJNIImpl() override;

  void FlutterViewHandlePlatformMessage(
      std::unique_ptr<flutter::PlatformMessage> message,
      int responseId) override;

  void FlutterViewHandlePlatformMessageResponse(
      int responseId,
      std::unique_ptr<fml::Mapping> data) override;

  void FlutterViewUpdateSemantics(
      std::vector<uint8_t> buffer,
      std::vector<std::string> strings,
      std::vector<std::vector<uint8_t>> string_attribute_args) override;

  void FlutterViewUpdateCustomAccessibilityActions(
      std::vector<uint8_t> actions_buffer,
      std::vector<std::string> strings) override;

  void FlutterViewOnFirstFrame() override;

  void FlutterViewOnPreEngineRestart() override;

  void SurfaceTextureAttachToGLContext(JavaLocalRef surface_texture,
                                       int textureId) override;

  bool SurfaceTextureShouldUpdate(JavaLocalRef surface_texture) override;

  void SurfaceTextureUpdateTexImage(JavaLocalRef surface_texture) override;

  SkM44 SurfaceTextureGetTransformMatrix(JavaLocalRef surface_texture) override;

  void SurfaceTextureDetachFromGLContext(JavaLocalRef surface_texture) override;

  JavaLocalRef ImageProducerTextureEntryAcquireLatestImage(
      JavaLocalRef image_texture_entry) override;

  JavaLocalRef ImageGetHardwareBuffer(JavaLocalRef image) override;

  void ImageClose(JavaLocalRef image) override;

  void HardwareBufferClose(JavaLocalRef hardware_buffer) override;

  void FlutterViewOnDisplayPlatformView(int view_id,
                                        int x,
                                        int y,
                                        int width,
                                        int height,
                                        int viewWidth,
                                        int viewHeight,
                                        MutatorsStack mutators_stack) override;

  void FlutterViewDisplayOverlaySurface(int surface_id,
                                        int x,
                                        int y,
                                        int width,
                                        int height) override;

  void FlutterViewBeginFrame() override;

  void FlutterViewEndFrame() override;

  std::unique_ptr<PlatformViewAndroidJNI::OverlayMetadata>
  FlutterViewCreateOverlaySurface() override;

  void FlutterViewDestroyOverlaySurfaces() override;

  std::unique_ptr<std::vector<std::string>>
  FlutterViewComputePlatformResolvedLocale(
      std::vector<std::string> supported_locales_data) override;

  double GetDisplayRefreshRate() override;

  double GetDisplayWidth() override;

  double GetDisplayHeight() override;

  double GetDisplayDensity() override;

  bool RequestDartDeferredLibrary(int loading_unit_id) override;

  double FlutterViewGetScaledFontSize(double unscaled_font_size,
                                      int configuration_id) const override;

  // New Platform View Support.
  ASurfaceTransaction* createTransaction() override;

  void swapTransaction() override;

  void applyTransaction() override;

  std::unique_ptr<PlatformViewAndroidJNI::OverlayMetadata>
  createOverlaySurface2() override;

  void destroyOverlaySurface2() override;

  void onDisplayPlatformView2(int32_t view_id,
                              int32_t x,
                              int32_t y,
                              int32_t width,
                              int32_t height,
                              int32_t viewWidth,
                              int32_t viewHeight,
                              MutatorsStack mutators_stack) override;

  void showOverlaySurface2() override;

  void hideOverlaySurface2() override;

  void onEndFrame2() override;

 private:
  // Reference to FlutterJNI object.
  const fml::jni::JavaObjectWeakGlobalRef java_object_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformViewAndroidJNIImpl);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_JNI_IMPL_H_
