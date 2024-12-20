// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <android/hardware_buffer_jni.h>
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/snapshot_surface_producer.h"
#include "flutter/shell/platform/android/context/android_context.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"
#include "flutter/shell/platform/android/platform_message_handler_android.h"
#include "flutter/shell/platform/android/platform_view_android_delegate/platform_view_android_delegate.h"
#include "flutter/shell/platform/android/surface/android_native_window.h"
#include "flutter/shell/platform/android/surface/android_surface.h"

namespace flutter {

class AndroidSurfaceFactoryImpl : public AndroidSurfaceFactory {
 public:
  AndroidSurfaceFactoryImpl(const std::shared_ptr<AndroidContext>& context,
                            bool enable_impeller);

  ~AndroidSurfaceFactoryImpl() override;

  std::unique_ptr<AndroidSurface> CreateSurface() override;

 private:
  const std::shared_ptr<AndroidContext>& android_context_;
  const bool enable_impeller_;
};

class PlatformViewAndroid final : public PlatformView {
 public:
  static bool Register(JNIEnv* env);

  PlatformViewAndroid(PlatformView::Delegate& delegate,
                      const flutter::TaskRunners& task_runners,
                      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade,
                      bool use_software_rendering);

  //----------------------------------------------------------------------------
  /// @brief      Creates a new PlatformViewAndroid but using an existing
  ///             Android GPU context to create new surfaces. This maximizes
  ///             resource sharing between 2 PlatformViewAndroids of 2 Shells.
  ///
  PlatformViewAndroid(
      PlatformView::Delegate& delegate,
      const flutter::TaskRunners& task_runners,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade,
      const std::shared_ptr<flutter::AndroidContext>& android_context);

  ~PlatformViewAndroid() override;

  void NotifyCreated(fml::RefPtr<AndroidNativeWindow> native_window);

  void NotifySurfaceWindowChanged(
      fml::RefPtr<AndroidNativeWindow> native_window);

  void NotifyChanged(const SkISize& size);

  // |PlatformView|
  void NotifyDestroyed() override;

  void DispatchPlatformMessage(JNIEnv* env,
                               std::string name,
                               jobject message_data,
                               jint message_position,
                               jint response_id);

  void DispatchEmptyPlatformMessage(JNIEnv* env,
                                    std::string name,
                                    jint response_id);

  void DispatchSemanticsAction(JNIEnv* env,
                               jint id,
                               jint action,
                               jobject args,
                               jint args_position);

  void RegisterExternalTexture(
      int64_t texture_id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture);

  void RegisterImageTexture(
      int64_t texture_id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& image_texture_entry);

  // |PlatformView|
  void LoadDartDeferredLibrary(
      intptr_t loading_unit_id,
      std::unique_ptr<const fml::Mapping> snapshot_data,
      std::unique_ptr<const fml::Mapping> snapshot_instructions) override;

  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string error_message,
                                    bool transient) override;

  // |PlatformView|
  void UpdateAssetResolverByType(
      std::unique_ptr<AssetResolver> updated_asset_resolver,
      AssetResolver::AssetResolverType type) override;

  const std::shared_ptr<AndroidContext>& GetAndroidContext() {
    return android_context_;
  }

  std::shared_ptr<PlatformMessageHandler> GetPlatformMessageHandler()
      const override {
    return platform_message_handler_;
  }

 private:
  const std::shared_ptr<PlatformViewAndroidJNI> jni_facade_;
  std::shared_ptr<AndroidContext> android_context_;
  std::shared_ptr<AndroidSurfaceFactoryImpl> surface_factory_;

  PlatformViewAndroidDelegate platform_view_android_delegate_;

  std::unique_ptr<AndroidSurface> android_surface_;
  std::shared_ptr<PlatformMessageHandlerAndroid> platform_message_handler_;

  // |PlatformView|
  void UpdateSemantics(
      flutter::SemanticsNodeUpdates update,
      flutter::CustomAccessibilityActionUpdates actions) override;

  // |PlatformView|
  void HandlePlatformMessage(
      std::unique_ptr<flutter::PlatformMessage> message) override;

  // |PlatformView|
  void OnPreEngineRestart() const override;

  // |PlatformView|
  std::unique_ptr<VsyncWaiter> CreateVSyncWaiter() override;

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override;

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override;

  // |PlatformView|
  std::unique_ptr<SnapshotSurfaceProducer> CreateSnapshotSurfaceProducer()
      override;

  // |PlatformView|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  // |PlatformView|
  void ReleaseResourceContext() const override;

  // |PlatformView|
  std::shared_ptr<impeller::Context> GetImpellerContext() const override;

  // |PlatformView|
  std::unique_ptr<std::vector<std::string>> ComputePlatformResolvedLocales(
      const std::vector<std::string>& supported_locale_data) override;

  // |PlatformView|
  void RequestDartDeferredLibrary(intptr_t loading_unit_id) override;

  void InstallFirstFrameCallback();

  void FireFirstFrameCallback();

  double GetScaledFontSize(double unscaled_font_size,
                           int configuration_id) const override;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformViewAndroid);
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_H_
