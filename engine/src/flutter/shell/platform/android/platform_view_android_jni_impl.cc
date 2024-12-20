// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/platform_view_android_jni_impl.h"

#include <android/hardware_buffer_jni.h>
#include <android/native_window_jni.h>
#include <dlfcn.h>
#include <jni.h>
#include <memory>
#include <sstream>
#include <utility>

#include "impeller/toolkit/android/shadow_realm.h"
#include "include/android/SkImageAndroid.h"
#include "unicode/uchar.h"

#include "flutter/assets/directory_asset_bundle.h"
#include "flutter/common/constants.h"
#include "flutter/fml/file.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/native_library.h"
#include "flutter/fml/platform/android/jni_util.h"
#include "flutter/fml/platform/android/jni_weak_ref.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"
#include "flutter/lib/ui/plugins/callback_cache.h"
#include "flutter/runtime/dart_service_isolate.h"
#include "flutter/shell/common/run_configuration.h"
#include "flutter/shell/platform/android/android_shell_holder.h"
#include "flutter/shell/platform/android/apk_asset_provider.h"
#include "flutter/shell/platform/android/flutter_main.h"
#include "flutter/shell/platform/android/image_external_texture_gl.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"
#include "flutter/shell/platform/android/platform_view_android.h"

#define ANDROID_SHELL_HOLDER \
  (reinterpret_cast<AndroidShellHolder*>(shell_holder))

namespace flutter {

static fml::jni::ScopedJavaGlobalRef<jclass>* g_flutter_callback_info_class =
    nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_flutter_jni_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_java_weak_reference_class =
    nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_texture_wrapper_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>*
    g_image_consumer_texture_registry_interface = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_image_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_hardware_buffer_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_java_long_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_bitmap_class = nullptr;

static fml::jni::ScopedJavaGlobalRef<jclass>* g_bitmap_config_class = nullptr;

// Called By Native

static jmethodID g_flutter_callback_info_constructor = nullptr;
jobject CreateFlutterCallbackInformation(
    JNIEnv* env,
    const std::string& callbackName,
    const std::string& callbackClassName,
    const std::string& callbackLibraryPath) {
  return env->NewObject(g_flutter_callback_info_class->obj(),
                        g_flutter_callback_info_constructor,
                        env->NewStringUTF(callbackName.c_str()),
                        env->NewStringUTF(callbackClassName.c_str()),
                        env->NewStringUTF(callbackLibraryPath.c_str()));
}

static jfieldID g_jni_shell_holder_field = nullptr;

static jmethodID g_jni_constructor = nullptr;

static jmethodID g_long_constructor = nullptr;

static jmethodID g_handle_platform_message_method = nullptr;

static jmethodID g_handle_platform_message_response_method = nullptr;

static jmethodID g_update_semantics_method = nullptr;

static jmethodID g_update_custom_accessibility_actions_method = nullptr;

static jmethodID g_get_scaled_font_size_method = nullptr;

static jmethodID g_on_first_frame_method = nullptr;

static jmethodID g_on_engine_restart_method = nullptr;

static jmethodID g_create_overlay_surface_method = nullptr;

static jmethodID g_destroy_overlay_surfaces_method = nullptr;

static jmethodID g_on_begin_frame_method = nullptr;

static jmethodID g_on_end_frame_method = nullptr;

static jmethodID g_java_weak_reference_get_method = nullptr;

static jmethodID g_attach_to_gl_context_method = nullptr;

static jmethodID g_surface_texture_wrapper_should_update = nullptr;

static jmethodID g_update_tex_image_method = nullptr;

static jmethodID g_get_transform_matrix_method = nullptr;

static jmethodID g_detach_from_gl_context_method = nullptr;

static jmethodID g_acquire_latest_image_method = nullptr;

static jmethodID g_image_get_hardware_buffer_method = nullptr;

static jmethodID g_image_close_method = nullptr;

static jmethodID g_hardware_buffer_close_method = nullptr;

static jmethodID g_compute_platform_resolved_locale_method = nullptr;

static jmethodID g_request_dart_deferred_library_method = nullptr;

// Called By Java
static jmethodID g_on_display_platform_view_method = nullptr;

// static jmethodID g_on_composite_platform_view_method = nullptr;

static jmethodID g_on_display_overlay_surface_method = nullptr;

static jmethodID g_overlay_surface_id_method = nullptr;

static jmethodID g_overlay_surface_surface_method = nullptr;

static jmethodID g_bitmap_create_bitmap_method = nullptr;

static jmethodID g_bitmap_copy_pixels_from_buffer_method = nullptr;

static jmethodID g_bitmap_config_value_of = nullptr;

// Mutators
static fml::jni::ScopedJavaGlobalRef<jclass>* g_mutators_stack_class = nullptr;
static jmethodID g_mutators_stack_init_method = nullptr;
static jmethodID g_mutators_stack_push_transform_method = nullptr;
static jmethodID g_mutators_stack_push_cliprect_method = nullptr;
static jmethodID g_mutators_stack_push_cliprrect_method = nullptr;

// Called By Java
static jlong AttachJNI(JNIEnv* env, jclass clazz, jobject flutterJNI) {
  fml::jni::JavaObjectWeakGlobalRef java_object(env, flutterJNI);
  std::shared_ptr<PlatformViewAndroidJNI> jni_facade =
      std::make_shared<PlatformViewAndroidJNIImpl>(java_object);
  auto shell_holder = std::make_unique<AndroidShellHolder>(
      FlutterMain::Get().GetSettings(), jni_facade);
  if (shell_holder->IsValid()) {
    return reinterpret_cast<jlong>(shell_holder.release());
  } else {
    return 0;
  }
}

static void DestroyJNI(JNIEnv* env, jobject jcaller, jlong shell_holder) {
  delete ANDROID_SHELL_HOLDER;
}

// Signature is similar to RunBundleAndSnapshotFromLibrary but it can't change
// the bundle path or asset manager since we can only spawn with the same
// AOT.
//
// The shell_holder instance must be a pointer address to the current
// AndroidShellHolder whose Shell will be used to spawn a new Shell.
//
// This creates a Java Long that points to the newly created
// AndroidShellHolder's raw pointer, connects that Long to a newly created
// FlutterJNI instance, then returns the FlutterJNI instance.
static jobject SpawnJNI(JNIEnv* env,
                        jobject jcaller,
                        jlong shell_holder,
                        jstring jEntrypoint,
                        jstring jLibraryUrl,
                        jstring jInitialRoute,
                        jobject jEntrypointArgs) {
  jobject jni = env->NewObject(g_flutter_jni_class->obj(), g_jni_constructor);
  if (jni == nullptr) {
    FML_LOG(ERROR) << "Could not create a FlutterJNI instance";
    return nullptr;
  }

  fml::jni::JavaObjectWeakGlobalRef java_jni(env, jni);
  std::shared_ptr<PlatformViewAndroidJNI> jni_facade =
      std::make_shared<PlatformViewAndroidJNIImpl>(java_jni);

  auto entrypoint = fml::jni::JavaStringToString(env, jEntrypoint);
  auto libraryUrl = fml::jni::JavaStringToString(env, jLibraryUrl);
  auto initial_route = fml::jni::JavaStringToString(env, jInitialRoute);
  auto entrypoint_args = fml::jni::StringListToVector(env, jEntrypointArgs);

  auto spawned_shell_holder = ANDROID_SHELL_HOLDER->Spawn(
      jni_facade, entrypoint, libraryUrl, initial_route, entrypoint_args);

  if (spawned_shell_holder == nullptr || !spawned_shell_holder->IsValid()) {
    FML_LOG(ERROR) << "Could not spawn Shell";
    return nullptr;
  }

  jobject javaLong = env->CallStaticObjectMethod(
      g_java_long_class->obj(), g_long_constructor,
      reinterpret_cast<jlong>(spawned_shell_holder.release()));
  if (javaLong == nullptr) {
    FML_LOG(ERROR) << "Could not create a Long instance";
    return nullptr;
  }

  env->SetObjectField(jni, g_jni_shell_holder_field, javaLong);

  return jni;
}

static void SurfaceCreated(JNIEnv* env,
                           jobject jcaller,
                           jlong shell_holder,
                           jobject jsurface) {
  // Note: This frame ensures that any local references used by
  // ANativeWindow_fromSurface are released immediately. This is needed as a
  // workaround for https://code.google.com/p/android/issues/detail?id=68174
  fml::jni::ScopedJavaLocalFrame scoped_local_reference_frame(env);
  auto window = fml::MakeRefCounted<AndroidNativeWindow>(
      ANativeWindow_fromSurface(env, jsurface));
  ANDROID_SHELL_HOLDER->GetPlatformView()->NotifyCreated(std::move(window));
}

static void SurfaceWindowChanged(JNIEnv* env,
                                 jobject jcaller,
                                 jlong shell_holder,
                                 jobject jsurface) {
  // Note: This frame ensures that any local references used by
  // ANativeWindow_fromSurface are released immediately. This is needed as a
  // workaround for https://code.google.com/p/android/issues/detail?id=68174
  fml::jni::ScopedJavaLocalFrame scoped_local_reference_frame(env);
  auto window = fml::MakeRefCounted<AndroidNativeWindow>(
      ANativeWindow_fromSurface(env, jsurface));
  ANDROID_SHELL_HOLDER->GetPlatformView()->NotifySurfaceWindowChanged(
      std::move(window));
}

static void SurfaceChanged(JNIEnv* env,
                           jobject jcaller,
                           jlong shell_holder,
                           jint width,
                           jint height) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->NotifyChanged(
      SkISize::Make(width, height));
}

static void SurfaceDestroyed(JNIEnv* env, jobject jcaller, jlong shell_holder) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->NotifyDestroyed();
}

static void RunBundleAndSnapshotFromLibrary(JNIEnv* env,
                                            jobject jcaller,
                                            jlong shell_holder,
                                            jstring jBundlePath,
                                            jstring jEntrypoint,
                                            jstring jLibraryUrl,
                                            jobject jAssetManager,
                                            jobject jEntrypointArgs) {
  auto apk_asset_provider = std::make_unique<flutter::APKAssetProvider>(
      env,                                            // jni environment
      jAssetManager,                                  // asset manager
      fml::jni::JavaStringToString(env, jBundlePath)  // apk asset dir
  );
  auto entrypoint = fml::jni::JavaStringToString(env, jEntrypoint);
  auto libraryUrl = fml::jni::JavaStringToString(env, jLibraryUrl);
  auto entrypoint_args = fml::jni::StringListToVector(env, jEntrypointArgs);

  ANDROID_SHELL_HOLDER->Launch(std::move(apk_asset_provider), entrypoint,
                               libraryUrl, entrypoint_args);
}

static jobject LookupCallbackInformation(JNIEnv* env,
                                         /* unused */ jobject,
                                         jlong handle) {
  auto cbInfo = flutter::DartCallbackCache::GetCallbackInformation(handle);
  if (cbInfo == nullptr) {
    return nullptr;
  }
  return CreateFlutterCallbackInformation(env, cbInfo->name, cbInfo->class_name,
                                          cbInfo->library_path);
}

static void SetViewportMetrics(JNIEnv* env,
                               jobject jcaller,
                               jlong shell_holder,
                               jfloat devicePixelRatio,
                               jint physicalWidth,
                               jint physicalHeight,
                               jint physicalPaddingTop,
                               jint physicalPaddingRight,
                               jint physicalPaddingBottom,
                               jint physicalPaddingLeft,
                               jint physicalViewInsetTop,
                               jint physicalViewInsetRight,
                               jint physicalViewInsetBottom,
                               jint physicalViewInsetLeft,
                               jint systemGestureInsetTop,
                               jint systemGestureInsetRight,
                               jint systemGestureInsetBottom,
                               jint systemGestureInsetLeft,
                               jint physicalTouchSlop,
                               jintArray javaDisplayFeaturesBounds,
                               jintArray javaDisplayFeaturesType,
                               jintArray javaDisplayFeaturesState) {
  // Convert java->c++. javaDisplayFeaturesBounds, javaDisplayFeaturesType and
  // javaDisplayFeaturesState cannot be null
  jsize rectSize = env->GetArrayLength(javaDisplayFeaturesBounds);
  std::vector<int> boundsIntVector(rectSize);
  env->GetIntArrayRegion(javaDisplayFeaturesBounds, 0, rectSize,
                         &boundsIntVector[0]);
  std::vector<double> displayFeaturesBounds(boundsIntVector.begin(),
                                            boundsIntVector.end());
  jsize typeSize = env->GetArrayLength(javaDisplayFeaturesType);
  std::vector<int> displayFeaturesType(typeSize);
  env->GetIntArrayRegion(javaDisplayFeaturesType, 0, typeSize,
                         &displayFeaturesType[0]);

  jsize stateSize = env->GetArrayLength(javaDisplayFeaturesState);
  std::vector<int> displayFeaturesState(stateSize);
  env->GetIntArrayRegion(javaDisplayFeaturesState, 0, stateSize,
                         &displayFeaturesState[0]);

  const flutter::ViewportMetrics metrics{
      static_cast<double>(devicePixelRatio),
      static_cast<double>(physicalWidth),
      static_cast<double>(physicalHeight),
      static_cast<double>(physicalPaddingTop),
      static_cast<double>(physicalPaddingRight),
      static_cast<double>(physicalPaddingBottom),
      static_cast<double>(physicalPaddingLeft),
      static_cast<double>(physicalViewInsetTop),
      static_cast<double>(physicalViewInsetRight),
      static_cast<double>(physicalViewInsetBottom),
      static_cast<double>(physicalViewInsetLeft),
      static_cast<double>(systemGestureInsetTop),
      static_cast<double>(systemGestureInsetRight),
      static_cast<double>(systemGestureInsetBottom),
      static_cast<double>(systemGestureInsetLeft),
      static_cast<double>(physicalTouchSlop),
      displayFeaturesBounds,
      displayFeaturesType,
      displayFeaturesState,
      0,  // Display ID
  };

  ANDROID_SHELL_HOLDER->GetPlatformView()->SetViewportMetrics(
      kFlutterImplicitViewId, metrics);
}

static void UpdateDisplayMetrics(JNIEnv* env,
                                 jobject jcaller,
                                 jlong shell_holder) {
  ANDROID_SHELL_HOLDER->UpdateDisplayMetrics();
}

static jobject GetBitmap(JNIEnv* env, jobject jcaller, jlong shell_holder) {
  auto screenshot = ANDROID_SHELL_HOLDER->Screenshot(
      Rasterizer::ScreenshotType::UncompressedImage, false);
  if (screenshot.data == nullptr) {
    return nullptr;
  }

  jstring argb = env->NewStringUTF("ARGB_8888");
  if (argb == nullptr) {
    return nullptr;
  }

  jobject bitmap_config = env->CallStaticObjectMethod(
      g_bitmap_config_class->obj(), g_bitmap_config_value_of, argb);
  if (bitmap_config == nullptr) {
    return nullptr;
  }

  auto bitmap = env->CallStaticObjectMethod(
      g_bitmap_class->obj(), g_bitmap_create_bitmap_method,
      screenshot.frame_size.width(), screenshot.frame_size.height(),
      bitmap_config);

  fml::jni::ScopedJavaLocalRef<jobject> buffer(
      env,
      env->NewDirectByteBuffer(const_cast<uint8_t*>(screenshot.data->bytes()),
                               screenshot.data->size()));

  env->CallVoidMethod(bitmap, g_bitmap_copy_pixels_from_buffer_method,
                      buffer.obj());

  return bitmap;
}

static void DispatchPlatformMessage(JNIEnv* env,
                                    jobject jcaller,
                                    jlong shell_holder,
                                    jstring channel,
                                    jobject message,
                                    jint position,
                                    jint responseId) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->DispatchPlatformMessage(
      env,                                         //
      fml::jni::JavaStringToString(env, channel),  //
      message,                                     //
      position,                                    //
      responseId                                   //
  );
}

static void DispatchEmptyPlatformMessage(JNIEnv* env,
                                         jobject jcaller,
                                         jlong shell_holder,
                                         jstring channel,
                                         jint responseId) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->DispatchEmptyPlatformMessage(
      env,                                         //
      fml::jni::JavaStringToString(env, channel),  //
      responseId                                   //
  );
}

static void CleanupMessageData(JNIEnv* env,
                               jobject jcaller,
                               jlong message_data) {
  // Called from any thread.
  free(reinterpret_cast<void*>(message_data));
}

static void DispatchPointerDataPacket(JNIEnv* env,
                                      jobject jcaller,
                                      jlong shell_holder,
                                      jobject buffer,
                                      jint position) {
  uint8_t* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(buffer));
  auto packet = std::make_unique<flutter::PointerDataPacket>(data, position);
  ANDROID_SHELL_HOLDER->GetPlatformView()->DispatchPointerDataPacket(
      std::move(packet));
}

static void DispatchSemanticsAction(JNIEnv* env,
                                    jobject jcaller,
                                    jlong shell_holder,
                                    jint id,
                                    jint action,
                                    jobject args,
                                    jint args_position) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->DispatchSemanticsAction(
      env,           //
      id,            //
      action,        //
      args,          //
      args_position  //
  );
}

static void SetSemanticsEnabled(JNIEnv* env,
                                jobject jcaller,
                                jlong shell_holder,
                                jboolean enabled) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->SetSemanticsEnabled(enabled);
}

static void SetAccessibilityFeatures(JNIEnv* env,
                                     jobject jcaller,
                                     jlong shell_holder,
                                     jint flags) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->SetAccessibilityFeatures(flags);
}

static jboolean GetIsSoftwareRendering(JNIEnv* env, jobject jcaller) {
  return FlutterMain::Get().GetSettings().enable_software_rendering;
}

static void RegisterTexture(JNIEnv* env,
                            jobject jcaller,
                            jlong shell_holder,
                            jlong texture_id,
                            jobject surface_texture) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->RegisterExternalTexture(
      static_cast<int64_t>(texture_id),                             //
      fml::jni::ScopedJavaGlobalRef<jobject>(env, surface_texture)  //
  );
}

static void RegisterImageTexture(JNIEnv* env,
                                 jobject jcaller,
                                 jlong shell_holder,
                                 jlong texture_id,
                                 jobject image_texture_entry) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->RegisterImageTexture(
      static_cast<int64_t>(texture_id),                                 //
      fml::jni::ScopedJavaGlobalRef<jobject>(env, image_texture_entry)  //
  );
}

static void UnregisterTexture(JNIEnv* env,
                              jobject jcaller,
                              jlong shell_holder,
                              jlong texture_id) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->UnregisterTexture(
      static_cast<int64_t>(texture_id));
}

static void MarkTextureFrameAvailable(JNIEnv* env,
                                      jobject jcaller,
                                      jlong shell_holder,
                                      jlong texture_id) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->MarkTextureFrameAvailable(
      static_cast<int64_t>(texture_id));
}

static void ScheduleFrame(JNIEnv* env, jobject jcaller, jlong shell_holder) {
  ANDROID_SHELL_HOLDER->GetPlatformView()->ScheduleFrame();
}

static void InvokePlatformMessageResponseCallback(JNIEnv* env,
                                                  jobject jcaller,
                                                  jlong shell_holder,
                                                  jint responseId,
                                                  jobject message,
                                                  jint position) {
  uint8_t* response_data =
      static_cast<uint8_t*>(env->GetDirectBufferAddress(message));
  FML_DCHECK(response_data != nullptr);
  auto mapping = std::make_unique<fml::MallocMapping>(
      fml::MallocMapping::Copy(response_data, response_data + position));
  ANDROID_SHELL_HOLDER->GetPlatformMessageHandler()
      ->InvokePlatformMessageResponseCallback(responseId, std::move(mapping));
}

static void InvokePlatformMessageEmptyResponseCallback(JNIEnv* env,
                                                       jobject jcaller,
                                                       jlong shell_holder,
                                                       jint responseId) {
  ANDROID_SHELL_HOLDER->GetPlatformMessageHandler()
      ->InvokePlatformMessageEmptyResponseCallback(responseId);
}

static void NotifyLowMemoryWarning(JNIEnv* env,
                                   jobject obj,
                                   jlong shell_holder) {
  ANDROID_SHELL_HOLDER->NotifyLowMemoryWarning();
}

static jboolean FlutterTextUtilsIsEmoji(JNIEnv* env,
                                        jobject obj,
                                        jint codePoint) {
  return u_hasBinaryProperty(codePoint, UProperty::UCHAR_EMOJI);
}

static jboolean FlutterTextUtilsIsEmojiModifier(JNIEnv* env,
                                                jobject obj,
                                                jint codePoint) {
  return u_hasBinaryProperty(codePoint, UProperty::UCHAR_EMOJI_MODIFIER);
}

static jboolean FlutterTextUtilsIsEmojiModifierBase(JNIEnv* env,
                                                    jobject obj,
                                                    jint codePoint) {
  return u_hasBinaryProperty(codePoint, UProperty::UCHAR_EMOJI_MODIFIER_BASE);
}

static jboolean FlutterTextUtilsIsVariationSelector(JNIEnv* env,
                                                    jobject obj,
                                                    jint codePoint) {
  return u_hasBinaryProperty(codePoint, UProperty::UCHAR_VARIATION_SELECTOR);
}

static jboolean FlutterTextUtilsIsRegionalIndicator(JNIEnv* env,
                                                    jobject obj,
                                                    jint codePoint) {
  return u_hasBinaryProperty(codePoint, UProperty::UCHAR_REGIONAL_INDICATOR);
}

static void LoadLoadingUnitFailure(intptr_t loading_unit_id,
                                   const std::string& message,
                                   bool transient) {
  // TODO(garyq): Implement
}

static void DeferredComponentInstallFailure(JNIEnv* env,
                                            jobject obj,
                                            jint jLoadingUnitId,
                                            jstring jError,
                                            jboolean jTransient) {
  LoadLoadingUnitFailure(static_cast<intptr_t>(jLoadingUnitId),
                         fml::jni::JavaStringToString(env, jError),
                         static_cast<bool>(jTransient));
}

static void LoadDartDeferredLibrary(JNIEnv* env,
                                    jobject obj,
                                    jlong shell_holder,
                                    jint jLoadingUnitId,
                                    jobjectArray jSearchPaths) {
  // Convert java->c++
  intptr_t loading_unit_id = static_cast<intptr_t>(jLoadingUnitId);
  std::vector<std::string> search_paths =
      fml::jni::StringArrayToVector(env, jSearchPaths);

  // Use dlopen here to directly check if handle is nullptr before creating a
  // NativeLibrary.
  void* handle = nullptr;
  while (handle == nullptr && !search_paths.empty()) {
    std::string path = search_paths.back();
    handle = ::dlopen(path.c_str(), RTLD_NOW);
    search_paths.pop_back();
  }
  if (handle == nullptr) {
    LoadLoadingUnitFailure(loading_unit_id,
                           "No lib .so found for provided search paths.", true);
    return;
  }
  fml::RefPtr<fml::NativeLibrary> native_lib =
      fml::NativeLibrary::CreateWithHandle(handle, false);

  // Resolve symbols.
  std::unique_ptr<const fml::SymbolMapping> data_mapping =
      std::make_unique<const fml::SymbolMapping>(
          native_lib, DartSnapshot::kIsolateDataSymbol);
  std::unique_ptr<const fml::SymbolMapping> instructions_mapping =
      std::make_unique<const fml::SymbolMapping>(
          native_lib, DartSnapshot::kIsolateInstructionsSymbol);

  ANDROID_SHELL_HOLDER->GetPlatformView()->LoadDartDeferredLibrary(
      loading_unit_id, std::move(data_mapping),
      std::move(instructions_mapping));
}

static void UpdateJavaAssetManager(JNIEnv* env,
                                   jobject obj,
                                   jlong shell_holder,
                                   jobject jAssetManager,
                                   jstring jAssetBundlePath) {
  auto asset_resolver = std::make_unique<flutter::APKAssetProvider>(
      env,                                                   // jni environment
      jAssetManager,                                         // asset manager
      fml::jni::JavaStringToString(env, jAssetBundlePath));  // apk asset dir

  ANDROID_SHELL_HOLDER->GetPlatformView()->UpdateAssetResolverByType(
      std::move(asset_resolver),
      AssetResolver::AssetResolverType::kApkAssetProvider);
}

bool RegisterApi(JNIEnv* env) {
  static const JNINativeMethod flutter_jni_methods[] = {
      // Start of methods from FlutterJNI
      {
          .name = "nativeAttach",
          .signature = "(Lio/flutter/embedding/engine/FlutterJNI;)J",
          .fnPtr = reinterpret_cast<void*>(&AttachJNI),
      },
      {
          .name = "nativeDestroy",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&DestroyJNI),
      },
      {
          .name = "nativeSpawn",
          .signature = "(JLjava/lang/String;Ljava/lang/String;Ljava/lang/"
                       "String;Ljava/util/List;)Lio/flutter/"
                       "embedding/engine/FlutterJNI;",
          .fnPtr = reinterpret_cast<void*>(&SpawnJNI),
      },
      {
          .name = "nativeRunBundleAndSnapshotFromLibrary",
          .signature = "(JLjava/lang/String;Ljava/lang/String;"
                       "Ljava/lang/String;Landroid/content/res/"
                       "AssetManager;Ljava/util/List;)V",
          .fnPtr = reinterpret_cast<void*>(&RunBundleAndSnapshotFromLibrary),
      },
      {
          .name = "nativeDispatchEmptyPlatformMessage",
          .signature = "(JLjava/lang/String;I)V",
          .fnPtr = reinterpret_cast<void*>(&DispatchEmptyPlatformMessage),
      },
      {
          .name = "nativeCleanupMessageData",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&CleanupMessageData),
      },
      {
          .name = "nativeDispatchPlatformMessage",
          .signature = "(JLjava/lang/String;Ljava/nio/ByteBuffer;II)V",
          .fnPtr = reinterpret_cast<void*>(&DispatchPlatformMessage),
      },
      {
          .name = "nativeInvokePlatformMessageResponseCallback",
          .signature = "(JILjava/nio/ByteBuffer;I)V",
          .fnPtr =
              reinterpret_cast<void*>(&InvokePlatformMessageResponseCallback),
      },
      {
          .name = "nativeInvokePlatformMessageEmptyResponseCallback",
          .signature = "(JI)V",
          .fnPtr = reinterpret_cast<void*>(
              &InvokePlatformMessageEmptyResponseCallback),
      },
      {
          .name = "nativeNotifyLowMemoryWarning",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&NotifyLowMemoryWarning),
      },

      // Start of methods from FlutterView
      {
          .name = "nativeGetBitmap",
          .signature = "(J)Landroid/graphics/Bitmap;",
          .fnPtr = reinterpret_cast<void*>(&GetBitmap),
      },
      {
          .name = "nativeSurfaceCreated",
          .signature = "(JLandroid/view/Surface;)V",
          .fnPtr = reinterpret_cast<void*>(&SurfaceCreated),
      },
      {
          .name = "nativeSurfaceWindowChanged",
          .signature = "(JLandroid/view/Surface;)V",
          .fnPtr = reinterpret_cast<void*>(&SurfaceWindowChanged),
      },
      {
          .name = "nativeSurfaceChanged",
          .signature = "(JII)V",
          .fnPtr = reinterpret_cast<void*>(&SurfaceChanged),
      },
      {
          .name = "nativeSurfaceDestroyed",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&SurfaceDestroyed),
      },
      {
          .name = "nativeSetViewportMetrics",
          .signature = "(JFIIIIIIIIIIIIIII[I[I[I)V",
          .fnPtr = reinterpret_cast<void*>(&SetViewportMetrics),
      },
      {
          .name = "nativeDispatchPointerDataPacket",
          .signature = "(JLjava/nio/ByteBuffer;I)V",
          .fnPtr = reinterpret_cast<void*>(&DispatchPointerDataPacket),
      },
      {
          .name = "nativeDispatchSemanticsAction",
          .signature = "(JIILjava/nio/ByteBuffer;I)V",
          .fnPtr = reinterpret_cast<void*>(&DispatchSemanticsAction),
      },
      {
          .name = "nativeSetSemanticsEnabled",
          .signature = "(JZ)V",
          .fnPtr = reinterpret_cast<void*>(&SetSemanticsEnabled),
      },
      {
          .name = "nativeSetAccessibilityFeatures",
          .signature = "(JI)V",
          .fnPtr = reinterpret_cast<void*>(&SetAccessibilityFeatures),
      },
      {
          .name = "nativeGetIsSoftwareRenderingEnabled",
          .signature = "()Z",
          .fnPtr = reinterpret_cast<void*>(&GetIsSoftwareRendering),
      },
      {
          .name = "nativeRegisterTexture",
          .signature = "(JJLjava/lang/ref/"
                       "WeakReference;)V",
          .fnPtr = reinterpret_cast<void*>(&RegisterTexture),
      },
      {
          .name = "nativeRegisterImageTexture",
          .signature = "(JJLjava/lang/ref/"
                       "WeakReference;)V",
          .fnPtr = reinterpret_cast<void*>(&RegisterImageTexture),
      },
      {
          .name = "nativeMarkTextureFrameAvailable",
          .signature = "(JJ)V",
          .fnPtr = reinterpret_cast<void*>(&MarkTextureFrameAvailable),
      },
      {
          .name = "nativeScheduleFrame",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&ScheduleFrame),
      },
      {
          .name = "nativeUnregisterTexture",
          .signature = "(JJ)V",
          .fnPtr = reinterpret_cast<void*>(&UnregisterTexture),
      },
      // Methods for Dart callback functionality.
      {
          .name = "nativeLookupCallbackInformation",
          .signature = "(J)Lio/flutter/view/FlutterCallbackInformation;",
          .fnPtr = reinterpret_cast<void*>(&LookupCallbackInformation),
      },

      // Start of methods for FlutterTextUtils
      {
          .name = "nativeFlutterTextUtilsIsEmoji",
          .signature = "(I)Z",
          .fnPtr = reinterpret_cast<void*>(&FlutterTextUtilsIsEmoji),
      },
      {
          .name = "nativeFlutterTextUtilsIsEmojiModifier",
          .signature = "(I)Z",
          .fnPtr = reinterpret_cast<void*>(&FlutterTextUtilsIsEmojiModifier),
      },
      {
          .name = "nativeFlutterTextUtilsIsEmojiModifierBase",
          .signature = "(I)Z",
          .fnPtr =
              reinterpret_cast<void*>(&FlutterTextUtilsIsEmojiModifierBase),
      },
      {
          .name = "nativeFlutterTextUtilsIsVariationSelector",
          .signature = "(I)Z",
          .fnPtr =
              reinterpret_cast<void*>(&FlutterTextUtilsIsVariationSelector),
      },
      {
          .name = "nativeFlutterTextUtilsIsRegionalIndicator",
          .signature = "(I)Z",
          .fnPtr =
              reinterpret_cast<void*>(&FlutterTextUtilsIsRegionalIndicator),
      },
      {
          .name = "nativeLoadDartDeferredLibrary",
          .signature = "(JI[Ljava/lang/String;)V",
          .fnPtr = reinterpret_cast<void*>(&LoadDartDeferredLibrary),
      },
      {
          .name = "nativeUpdateJavaAssetManager",
          .signature =
              "(JLandroid/content/res/AssetManager;Ljava/lang/String;)V",
          .fnPtr = reinterpret_cast<void*>(&UpdateJavaAssetManager),
      },
      {
          .name = "nativeDeferredComponentInstallFailure",
          .signature = "(ILjava/lang/String;Z)V",
          .fnPtr = reinterpret_cast<void*>(&DeferredComponentInstallFailure),
      },
      {
          .name = "nativeUpdateDisplayMetrics",
          .signature = "(J)V",
          .fnPtr = reinterpret_cast<void*>(&UpdateDisplayMetrics),
      },
      {
          .name = "nativeShouldDisableAHB",
          .signature = "()Z",
          .fnPtr = reinterpret_cast<void*>(
              &impeller::android::ShadowRealm::ShouldDisableAHB),
      }};

  if (env->RegisterNatives(g_flutter_jni_class->obj(), flutter_jni_methods,
                           std::size(flutter_jni_methods)) != 0) {
    FML_LOG(ERROR) << "Failed to RegisterNatives with FlutterJNI";
    return false;
  }

  g_jni_shell_holder_field = env->GetFieldID(
      g_flutter_jni_class->obj(), "nativeShellHolderId", "Ljava/lang/Long;");

  if (g_jni_shell_holder_field == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterJNI's nativeShellHolderId field";
    return false;
  }

  g_jni_constructor =
      env->GetMethodID(g_flutter_jni_class->obj(), "<init>", "()V");

  if (g_jni_constructor == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterJNI's constructor";
    return false;
  }

  g_long_constructor = env->GetStaticMethodID(g_java_long_class->obj(),
                                              "valueOf", "(J)Ljava/lang/Long;");
  if (g_long_constructor == nullptr) {
    FML_LOG(ERROR) << "Could not locate Long's constructor";
    return false;
  }

  g_handle_platform_message_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "handlePlatformMessage",
                       "(Ljava/lang/String;Ljava/nio/ByteBuffer;IJ)V");

  if (g_handle_platform_message_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate handlePlatformMessage method";
    return false;
  }

  g_handle_platform_message_response_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "handlePlatformMessageResponse",
      "(ILjava/nio/ByteBuffer;)V");

  if (g_handle_platform_message_response_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate handlePlatformMessageResponse method";
    return false;
  }

  g_get_scaled_font_size_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "getScaledFontSize", "(FI)F");

  if (g_get_scaled_font_size_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterJNI#getScaledFontSize method";
    return false;
  }

  g_update_semantics_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "updateSemantics",
      "(Ljava/nio/ByteBuffer;[Ljava/lang/String;[Ljava/nio/ByteBuffer;)V");

  if (g_update_semantics_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate updateSemantics method";
    return false;
  }

  g_update_custom_accessibility_actions_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "updateCustomAccessibilityActions",
      "(Ljava/nio/ByteBuffer;[Ljava/lang/String;)V");

  if (g_update_custom_accessibility_actions_method == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate updateCustomAccessibilityActions method";
    return false;
  }

  g_on_first_frame_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "onFirstFrame", "()V");

  if (g_on_first_frame_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onFirstFrame method";
    return false;
  }

  g_on_engine_restart_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "onPreEngineRestart", "()V");

  if (g_on_engine_restart_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onEngineRestart method";
    return false;
  }

  g_create_overlay_surface_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "createOverlaySurface",
                       "()Lio/flutter/embedding/engine/FlutterOverlaySurface;");

  if (g_create_overlay_surface_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate createOverlaySurface method";
    return false;
  }

  g_destroy_overlay_surfaces_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "destroyOverlaySurfaces", "()V");

  if (g_destroy_overlay_surfaces_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate destroyOverlaySurfaces method";
    return false;
  }

  fml::jni::ScopedJavaLocalRef<jclass> overlay_surface_class(
      env, env->FindClass("io/flutter/embedding/engine/FlutterOverlaySurface"));
  if (overlay_surface_class.is_null()) {
    FML_LOG(ERROR) << "Could not locate FlutterOverlaySurface class";
    return false;
  }
  g_overlay_surface_id_method =
      env->GetMethodID(overlay_surface_class.obj(), "getId", "()I");
  if (g_overlay_surface_id_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterOverlaySurface#getId() method";
    return false;
  }
  g_overlay_surface_surface_method = env->GetMethodID(
      overlay_surface_class.obj(), "getSurface", "()Landroid/view/Surface;");
  if (g_overlay_surface_surface_method == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate FlutterOverlaySurface#getSurface() method";
    return false;
  }

  g_bitmap_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("android/graphics/Bitmap"));
  if (g_bitmap_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate Bitmap Class";
    return false;
  }

  g_bitmap_create_bitmap_method = env->GetStaticMethodID(
      g_bitmap_class->obj(), "createBitmap",
      "(IILandroid/graphics/Bitmap$Config;)Landroid/graphics/Bitmap;");
  if (g_bitmap_create_bitmap_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate Bitmap.createBitmap method";
    return false;
  }

  g_bitmap_copy_pixels_from_buffer_method = env->GetMethodID(
      g_bitmap_class->obj(), "copyPixelsFromBuffer", "(Ljava/nio/Buffer;)V");
  if (g_bitmap_copy_pixels_from_buffer_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate Bitmap.copyPixelsFromBuffer method";
    return false;
  }

  g_bitmap_config_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("android/graphics/Bitmap$Config"));
  if (g_bitmap_config_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate Bitmap.Config Class";
    return false;
  }

  g_bitmap_config_value_of = env->GetStaticMethodID(
      g_bitmap_config_class->obj(), "valueOf",
      "(Ljava/lang/String;)Landroid/graphics/Bitmap$Config;");
  if (g_bitmap_config_value_of == nullptr) {
    FML_LOG(ERROR) << "Could not locate Bitmap.Config.valueOf method";
    return false;
  }

  return true;
}

bool PlatformViewAndroid::Register(JNIEnv* env) {
  if (env == nullptr) {
    FML_LOG(ERROR) << "No JNIEnv provided";
    return false;
  }

  g_flutter_callback_info_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("io/flutter/view/FlutterCallbackInformation"));
  if (g_flutter_callback_info_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate FlutterCallbackInformation class";
    return false;
  }

  g_flutter_callback_info_constructor = env->GetMethodID(
      g_flutter_callback_info_class->obj(), "<init>",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
  if (g_flutter_callback_info_constructor == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterCallbackInformation constructor";
    return false;
  }

  g_flutter_jni_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("io/flutter/embedding/engine/FlutterJNI"));
  if (g_flutter_jni_class->is_null()) {
    FML_LOG(ERROR) << "Failed to find FlutterJNI Class.";
    return false;
  }

  g_mutators_stack_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env,
      env->FindClass(
          "io/flutter/embedding/engine/mutatorsstack/FlutterMutatorsStack"));
  if (g_mutators_stack_class == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterMutatorsStack";
    return false;
  }

  g_mutators_stack_init_method =
      env->GetMethodID(g_mutators_stack_class->obj(), "<init>", "()V");
  if (g_mutators_stack_init_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate FlutterMutatorsStack.init method";
    return false;
  }

  g_mutators_stack_push_transform_method =
      env->GetMethodID(g_mutators_stack_class->obj(), "pushTransform", "([F)V");
  if (g_mutators_stack_push_transform_method == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate FlutterMutatorsStack.pushTransform method";
    return false;
  }

  g_mutators_stack_push_cliprect_method = env->GetMethodID(
      g_mutators_stack_class->obj(), "pushClipRect", "(IIII)V");
  if (g_mutators_stack_push_cliprect_method == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate FlutterMutatorsStack.pushClipRect method";
    return false;
  }

  g_mutators_stack_push_cliprrect_method = env->GetMethodID(
      g_mutators_stack_class->obj(), "pushClipRRect", "(IIII[F)V");
  if (g_mutators_stack_push_cliprrect_method == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate FlutterMutatorsStack.pushClipRRect method";
    return false;
  }

  g_on_display_platform_view_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "onDisplayPlatformView",
                       "(IIIIIIILio/flutter/embedding/engine/mutatorsstack/"
                       "FlutterMutatorsStack;)V");

  if (g_on_display_platform_view_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onDisplayPlatformView method";
    return false;
  }

  g_on_begin_frame_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "onBeginFrame", "()V");

  if (g_on_begin_frame_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onBeginFrame method";
    return false;
  }

  g_on_end_frame_method =
      env->GetMethodID(g_flutter_jni_class->obj(), "onEndFrame", "()V");

  if (g_on_end_frame_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onEndFrame method";
    return false;
  }

  g_on_display_overlay_surface_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "onDisplayOverlaySurface", "(IIIII)V");

  if (g_on_display_overlay_surface_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate onDisplayOverlaySurface method";
    return false;
  }

  g_java_weak_reference_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("java/lang/ref/WeakReference"));
  if (g_java_weak_reference_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate WeakReference class";
    return false;
  }

  g_java_weak_reference_get_method = env->GetMethodID(
      g_java_weak_reference_class->obj(), "get", "()Ljava/lang/Object;");
  if (g_java_weak_reference_get_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate WeakReference.get method";
    return false;
  }

  g_texture_wrapper_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass(
               "io/flutter/embedding/engine/renderer/SurfaceTextureWrapper"));
  if (g_texture_wrapper_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate SurfaceTextureWrapper class";
    return false;
  }

  g_attach_to_gl_context_method = env->GetMethodID(
      g_texture_wrapper_class->obj(), "attachToGLContext", "(I)V");

  if (g_attach_to_gl_context_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate attachToGlContext method";
    return false;
  }

  g_surface_texture_wrapper_should_update =
      env->GetMethodID(g_texture_wrapper_class->obj(), "shouldUpdate", "()Z");

  if (g_surface_texture_wrapper_should_update == nullptr) {
    FML_LOG(ERROR)
        << "Could not locate SurfaceTextureWrapper.shouldUpdate method";
    return false;
  }

  g_update_tex_image_method =
      env->GetMethodID(g_texture_wrapper_class->obj(), "updateTexImage", "()V");

  if (g_update_tex_image_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate updateTexImage method";
    return false;
  }

  g_get_transform_matrix_method = env->GetMethodID(
      g_texture_wrapper_class->obj(), "getTransformMatrix", "([F)V");

  if (g_get_transform_matrix_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate getTransformMatrix method";
    return false;
  }

  g_detach_from_gl_context_method = env->GetMethodID(
      g_texture_wrapper_class->obj(), "detachFromGLContext", "()V");

  if (g_detach_from_gl_context_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate detachFromGlContext method";
    return false;
  }
  g_image_consumer_texture_registry_interface =
      new fml::jni::ScopedJavaGlobalRef<jclass>(
          env, env->FindClass("io/flutter/view/TextureRegistry$ImageConsumer"));
  if (g_image_consumer_texture_registry_interface->is_null()) {
    FML_LOG(ERROR) << "Could not locate TextureRegistry.ImageConsumer class";
    return false;
  }

  g_acquire_latest_image_method =
      env->GetMethodID(g_image_consumer_texture_registry_interface->obj(),
                       "acquireLatestImage", "()Landroid/media/Image;");
  if (g_acquire_latest_image_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate acquireLatestImage on "
                      "TextureRegistry.ImageConsumer class";
    return false;
  }

  g_image_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("android/media/Image"));
  if (g_image_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate Image class";
    return false;
  }

  // Ensure we don't have any pending exceptions.
  FML_CHECK(fml::jni::CheckException(env));

  g_image_get_hardware_buffer_method =
      env->GetMethodID(g_image_class->obj(), "getHardwareBuffer",
                       "()Landroid/hardware/HardwareBuffer;");

  if (g_image_get_hardware_buffer_method == nullptr) {
    // Continue on as this method may not exist at API <= 29.
    fml::jni::ClearException(env, true);
  }

  g_image_close_method = env->GetMethodID(g_image_class->obj(), "close", "()V");

  if (g_image_close_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate close on Image class";
    return false;
  }

  // Ensure we don't have any pending exceptions.
  FML_CHECK(fml::jni::CheckException(env));
  g_hardware_buffer_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("android/hardware/HardwareBuffer"));

  if (!g_hardware_buffer_class->is_null()) {
    g_hardware_buffer_close_method =
        env->GetMethodID(g_hardware_buffer_class->obj(), "close", "()V");
    if (g_hardware_buffer_close_method == nullptr) {
      // Continue on as this class may not exist at API <= 26.
      fml::jni::ClearException(env, true);
    }
  } else {
    // Continue on as this class may not exist at API <= 26.
    fml::jni::ClearException(env, true);
  }

  g_compute_platform_resolved_locale_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "computePlatformResolvedLocale",
      "([Ljava/lang/String;)[Ljava/lang/String;");

  if (g_compute_platform_resolved_locale_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate computePlatformResolvedLocale method";
    return false;
  }

  g_request_dart_deferred_library_method = env->GetMethodID(
      g_flutter_jni_class->obj(), "requestDartDeferredLibrary", "(I)V");

  if (g_request_dart_deferred_library_method == nullptr) {
    FML_LOG(ERROR) << "Could not locate requestDartDeferredLibrary method";
    return false;
  }

  g_java_long_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("java/lang/Long"));
  if (g_java_long_class->is_null()) {
    FML_LOG(ERROR) << "Could not locate java.lang.Long class";
    return false;
  }

  return RegisterApi(env);
}

PlatformViewAndroidJNIImpl::PlatformViewAndroidJNIImpl(
    const fml::jni::JavaObjectWeakGlobalRef& java_object)
    : java_object_(java_object) {}

PlatformViewAndroidJNIImpl::~PlatformViewAndroidJNIImpl() = default;

void PlatformViewAndroidJNIImpl::FlutterViewHandlePlatformMessage(
    std::unique_ptr<flutter::PlatformMessage> message,
    int responseId) {
  // Called from any thread.
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jstring> java_channel =
      fml::jni::StringToJavaString(env, message->channel());

  if (message->hasData()) {
    fml::jni::ScopedJavaLocalRef<jobject> message_array(
        env, env->NewDirectByteBuffer(
                 const_cast<uint8_t*>(message->data().GetMapping()),
                 message->data().GetSize()));
    // Message data is deleted in CleanupMessageData.
    fml::MallocMapping mapping = message->releaseData();
    env->CallVoidMethod(java_object.obj(), g_handle_platform_message_method,
                        java_channel.obj(), message_array.obj(), responseId,
                        reinterpret_cast<jlong>(mapping.Release()));
  } else {
    env->CallVoidMethod(java_object.obj(), g_handle_platform_message_method,
                        java_channel.obj(), nullptr, responseId, nullptr);
  }

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewHandlePlatformMessageResponse(
    int responseId,
    std::unique_ptr<fml::Mapping> data) {
  // We are on the platform thread. Attempt to get the strong reference to
  // the Java object.
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    // The Java object was collected before this message response got to
    // it. Drop the response on the floor.
    return;
  }
  if (data == nullptr) {  // Empty response.
    env->CallVoidMethod(java_object.obj(),
                        g_handle_platform_message_response_method, responseId,
                        nullptr);
  } else {
    // Convert the vector to a Java byte array.
    fml::jni::ScopedJavaLocalRef<jobject> data_array(
        env, env->NewDirectByteBuffer(const_cast<uint8_t*>(data->GetMapping()),
                                      data->GetSize()));

    env->CallVoidMethod(java_object.obj(),
                        g_handle_platform_message_response_method, responseId,
                        data_array.obj());
  }

  FML_CHECK(fml::jni::CheckException(env));
}

double PlatformViewAndroidJNIImpl::FlutterViewGetScaledFontSize(
    double font_size,
    int configuration_id) const {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return -3;
  }

  const jfloat scaledSize = env->CallFloatMethod(
      java_object.obj(), g_get_scaled_font_size_method,
      static_cast<jfloat>(font_size), static_cast<jint>(configuration_id));
  FML_CHECK(fml::jni::CheckException(env));
  return static_cast<double>(scaledSize);
}

void PlatformViewAndroidJNIImpl::FlutterViewUpdateSemantics(
    std::vector<uint8_t> buffer,
    std::vector<std::string> strings,
    std::vector<std::vector<uint8_t>> string_attribute_args) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jobject> direct_buffer(
      env, env->NewDirectByteBuffer(buffer.data(), buffer.size()));
  fml::jni::ScopedJavaLocalRef<jobjectArray> jstrings =
      fml::jni::VectorToStringArray(env, strings);
  fml::jni::ScopedJavaLocalRef<jobjectArray> jstring_attribute_args =
      fml::jni::VectorToBufferArray(env, string_attribute_args);

  env->CallVoidMethod(java_object.obj(), g_update_semantics_method,
                      direct_buffer.obj(), jstrings.obj(),
                      jstring_attribute_args.obj());

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewUpdateCustomAccessibilityActions(
    std::vector<uint8_t> actions_buffer,
    std::vector<std::string> strings) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jobject> direct_actions_buffer(
      env,
      env->NewDirectByteBuffer(actions_buffer.data(), actions_buffer.size()));

  fml::jni::ScopedJavaLocalRef<jobjectArray> jstrings =
      fml::jni::VectorToStringArray(env, strings);

  env->CallVoidMethod(java_object.obj(),
                      g_update_custom_accessibility_actions_method,
                      direct_actions_buffer.obj(), jstrings.obj());

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewOnFirstFrame() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_on_first_frame_method);

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewOnPreEngineRestart() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_on_engine_restart_method);

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::SurfaceTextureAttachToGLContext(
    JavaLocalRef surface_texture,
    int textureId) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (surface_texture.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jobject> surface_texture_local_ref(
      env, env->CallObjectMethod(surface_texture.obj(),
                                 g_java_weak_reference_get_method));

  if (surface_texture_local_ref.is_null()) {
    return;
  }

  env->CallVoidMethod(surface_texture_local_ref.obj(),
                      g_attach_to_gl_context_method, textureId);

  FML_CHECK(fml::jni::CheckException(env));
}

bool PlatformViewAndroidJNIImpl::SurfaceTextureShouldUpdate(
    JavaLocalRef surface_texture) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (surface_texture.is_null()) {
    return false;
  }

  fml::jni::ScopedJavaLocalRef<jobject> surface_texture_local_ref(
      env, env->CallObjectMethod(surface_texture.obj(),
                                 g_java_weak_reference_get_method));
  if (surface_texture_local_ref.is_null()) {
    return false;
  }

  jboolean shouldUpdate = env->CallBooleanMethod(
      surface_texture_local_ref.obj(), g_surface_texture_wrapper_should_update);

  FML_CHECK(fml::jni::CheckException(env));

  return shouldUpdate;
}

void PlatformViewAndroidJNIImpl::SurfaceTextureUpdateTexImage(
    JavaLocalRef surface_texture) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (surface_texture.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jobject> surface_texture_local_ref(
      env, env->CallObjectMethod(surface_texture.obj(),
                                 g_java_weak_reference_get_method));
  if (surface_texture_local_ref.is_null()) {
    return;
  }

  env->CallVoidMethod(surface_texture_local_ref.obj(),
                      g_update_tex_image_method);

  FML_CHECK(fml::jni::CheckException(env));
}

SkM44 PlatformViewAndroidJNIImpl::SurfaceTextureGetTransformMatrix(
    JavaLocalRef surface_texture) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (surface_texture.is_null()) {
    return {};
  }

  fml::jni::ScopedJavaLocalRef<jobject> surface_texture_local_ref(
      env, env->CallObjectMethod(surface_texture.obj(),
                                 g_java_weak_reference_get_method));
  if (surface_texture_local_ref.is_null()) {
    return {};
  }

  fml::jni::ScopedJavaLocalRef<jfloatArray> transformMatrix(
      env, env->NewFloatArray(16));

  env->CallVoidMethod(surface_texture_local_ref.obj(),
                      g_get_transform_matrix_method, transformMatrix.obj());
  FML_CHECK(fml::jni::CheckException(env));

  float* m = env->GetFloatArrayElements(transformMatrix.obj(), nullptr);

  static_assert(sizeof(SkScalar) == sizeof(float));
  const auto transform = SkM44::ColMajor(m);

  env->ReleaseFloatArrayElements(transformMatrix.obj(), m, JNI_ABORT);

  return transform;
}

void PlatformViewAndroidJNIImpl::SurfaceTextureDetachFromGLContext(
    JavaLocalRef surface_texture) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (surface_texture.is_null()) {
    return;
  }

  fml::jni::ScopedJavaLocalRef<jobject> surface_texture_local_ref(
      env, env->CallObjectMethod(surface_texture.obj(),
                                 g_java_weak_reference_get_method));
  if (surface_texture_local_ref.is_null()) {
    return;
  }

  env->CallVoidMethod(surface_texture_local_ref.obj(),
                      g_detach_from_gl_context_method);

  FML_CHECK(fml::jni::CheckException(env));
}

JavaLocalRef
PlatformViewAndroidJNIImpl::ImageProducerTextureEntryAcquireLatestImage(
    JavaLocalRef image_producer_texture_entry) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  if (image_producer_texture_entry.is_null()) {
    // Return null.
    return JavaLocalRef();
  }

  // Convert the weak reference to ImageTextureEntry into a strong local
  // reference.
  fml::jni::ScopedJavaLocalRef<jobject> image_producer_texture_entry_local_ref(
      env, env->CallObjectMethod(image_producer_texture_entry.obj(),
                                 g_java_weak_reference_get_method));

  if (image_producer_texture_entry_local_ref.is_null()) {
    // Return null.
    return JavaLocalRef();
  }

  JavaLocalRef r = JavaLocalRef(
      env, env->CallObjectMethod(image_producer_texture_entry_local_ref.obj(),
                                 g_acquire_latest_image_method));
  if (fml::jni::CheckException(env)) {
    return r;
  }
  // Return null.
  return JavaLocalRef();
}

JavaLocalRef PlatformViewAndroidJNIImpl::ImageGetHardwareBuffer(
    JavaLocalRef image) {
  FML_CHECK(g_image_get_hardware_buffer_method != nullptr);
  JNIEnv* env = fml::jni::AttachCurrentThread();
  if (image.is_null()) {
    // Return null.
    return JavaLocalRef();
  }
  JavaLocalRef r = JavaLocalRef(
      env,
      env->CallObjectMethod(image.obj(), g_image_get_hardware_buffer_method));
  FML_CHECK(fml::jni::CheckException(env));
  return r;
}

void PlatformViewAndroidJNIImpl::ImageClose(JavaLocalRef image) {
  JNIEnv* env = fml::jni::AttachCurrentThread();
  if (image.is_null()) {
    return;
  }
  env->CallVoidMethod(image.obj(), g_image_close_method);
  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::HardwareBufferClose(
    JavaLocalRef hardware_buffer) {
  FML_CHECK(g_hardware_buffer_close_method != nullptr);
  JNIEnv* env = fml::jni::AttachCurrentThread();
  if (hardware_buffer.is_null()) {
    return;
  }
  env->CallVoidMethod(hardware_buffer.obj(), g_hardware_buffer_close_method);
  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewOnDisplayPlatformView(
    int view_id,
    int x,
    int y,
    int width,
    int height,
    int viewWidth,
    int viewHeight,
    MutatorsStack mutators_stack) {
  JNIEnv* env = fml::jni::AttachCurrentThread();
  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  jobject mutatorsStack = env->NewObject(g_mutators_stack_class->obj(),
                                         g_mutators_stack_init_method);

  std::vector<std::shared_ptr<Mutator>>::const_iterator iter =
      mutators_stack.Begin();
  while (iter != mutators_stack.End()) {
    switch ((*iter)->GetType()) {
      case kTransform: {
        const SkMatrix& matrix = (*iter)->GetMatrix();
        SkScalar matrix_array[9];
        matrix.get9(matrix_array);
        fml::jni::ScopedJavaLocalRef<jfloatArray> transformMatrix(
            env, env->NewFloatArray(9));

        env->SetFloatArrayRegion(transformMatrix.obj(), 0, 9, matrix_array);
        env->CallVoidMethod(mutatorsStack,
                            g_mutators_stack_push_transform_method,
                            transformMatrix.obj());
        break;
      }
      case kClipRect: {
        const SkRect& rect = (*iter)->GetRect();
        env->CallVoidMethod(
            mutatorsStack, g_mutators_stack_push_cliprect_method,
            static_cast<int>(rect.left()), static_cast<int>(rect.top()),
            static_cast<int>(rect.right()), static_cast<int>(rect.bottom()));
        break;
      }
      case kClipRRect: {
        const SkRRect& rrect = (*iter)->GetRRect();
        const SkRect& rect = rrect.rect();
        const SkVector& upper_left = rrect.radii(SkRRect::kUpperLeft_Corner);
        const SkVector& upper_right = rrect.radii(SkRRect::kUpperRight_Corner);
        const SkVector& lower_right = rrect.radii(SkRRect::kLowerRight_Corner);
        const SkVector& lower_left = rrect.radii(SkRRect::kLowerLeft_Corner);
        SkScalar radiis[8] = {
            upper_left.x(),  upper_left.y(),  upper_right.x(), upper_right.y(),
            lower_right.x(), lower_right.y(), lower_left.x(),  lower_left.y(),
        };
        fml::jni::ScopedJavaLocalRef<jfloatArray> radiisArray(
            env, env->NewFloatArray(8));
        env->SetFloatArrayRegion(radiisArray.obj(), 0, 8, radiis);
        env->CallVoidMethod(
            mutatorsStack, g_mutators_stack_push_cliprrect_method,
            static_cast<int>(rect.left()), static_cast<int>(rect.top()),
            static_cast<int>(rect.right()), static_cast<int>(rect.bottom()),
            radiisArray.obj());
        break;
      }
      // TODO(cyanglaz): Implement other mutators.
      // https://github.com/flutter/flutter/issues/58426
      case kClipPath:
      case kOpacity:
      case kBackdropFilter:
        break;
    }
    ++iter;
  }

  env->CallVoidMethod(java_object.obj(), g_on_display_platform_view_method,
                      view_id, x, y, width, height, viewWidth, viewHeight,
                      mutatorsStack);

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewDisplayOverlaySurface(
    int surface_id,
    int x,
    int y,
    int width,
    int height) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_on_display_overlay_surface_method,
                      surface_id, x, y, width, height);

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewBeginFrame() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_on_begin_frame_method);

  FML_CHECK(fml::jni::CheckException(env));
}

void PlatformViewAndroidJNIImpl::FlutterViewEndFrame() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_on_end_frame_method);

  FML_CHECK(fml::jni::CheckException(env));
}

std::unique_ptr<PlatformViewAndroidJNI::OverlayMetadata>
PlatformViewAndroidJNIImpl::FlutterViewCreateOverlaySurface() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return nullptr;
  }

  fml::jni::ScopedJavaLocalRef<jobject> overlay(
      env, env->CallObjectMethod(java_object.obj(),
                                 g_create_overlay_surface_method));
  FML_CHECK(fml::jni::CheckException(env));

  if (overlay.is_null()) {
    return std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(0,
                                                                     nullptr);
  }

  jint overlay_id =
      env->CallIntMethod(overlay.obj(), g_overlay_surface_id_method);

  jobject overlay_surface =
      env->CallObjectMethod(overlay.obj(), g_overlay_surface_surface_method);

  auto overlay_window = fml::MakeRefCounted<AndroidNativeWindow>(
      ANativeWindow_fromSurface(env, overlay_surface));

  return std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
      overlay_id, std::move(overlay_window));
}

void PlatformViewAndroidJNIImpl::FlutterViewDestroyOverlaySurfaces() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return;
  }

  env->CallVoidMethod(java_object.obj(), g_destroy_overlay_surfaces_method);

  FML_CHECK(fml::jni::CheckException(env));
}

std::unique_ptr<std::vector<std::string>>
PlatformViewAndroidJNIImpl::FlutterViewComputePlatformResolvedLocale(
    std::vector<std::string> supported_locales_data) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  std::unique_ptr<std::vector<std::string>> out =
      std::make_unique<std::vector<std::string>>();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return out;
  }
  fml::jni::ScopedJavaLocalRef<jobjectArray> j_locales_data =
      fml::jni::VectorToStringArray(env, supported_locales_data);
  jobjectArray result = static_cast<jobjectArray>(env->CallObjectMethod(
      java_object.obj(), g_compute_platform_resolved_locale_method,
      j_locales_data.obj()));

  FML_CHECK(fml::jni::CheckException(env));

  int length = env->GetArrayLength(result);
  for (int i = 0; i < length; i++) {
    out->emplace_back(fml::jni::JavaStringToString(
        env, static_cast<jstring>(env->GetObjectArrayElement(result, i))));
  }
  return out;
}

double PlatformViewAndroidJNIImpl::GetDisplayRefreshRate() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return kUnknownDisplayRefreshRate;
  }

  fml::jni::ScopedJavaLocalRef<jclass> clazz(
      env, env->GetObjectClass(java_object.obj()));
  if (clazz.is_null()) {
    return kUnknownDisplayRefreshRate;
  }

  jfieldID fid = env->GetStaticFieldID(clazz.obj(), "refreshRateFPS", "F");
  return static_cast<double>(env->GetStaticFloatField(clazz.obj(), fid));
}

double PlatformViewAndroidJNIImpl::GetDisplayWidth() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return -1;
  }

  fml::jni::ScopedJavaLocalRef<jclass> clazz(
      env, env->GetObjectClass(java_object.obj()));
  if (clazz.is_null()) {
    return -1;
  }

  jfieldID fid = env->GetStaticFieldID(clazz.obj(), "displayWidth", "F");
  return static_cast<double>(env->GetStaticFloatField(clazz.obj(), fid));
}

double PlatformViewAndroidJNIImpl::GetDisplayHeight() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return -1;
  }

  fml::jni::ScopedJavaLocalRef<jclass> clazz(
      env, env->GetObjectClass(java_object.obj()));
  if (clazz.is_null()) {
    return -1;
  }

  jfieldID fid = env->GetStaticFieldID(clazz.obj(), "displayHeight", "F");
  return static_cast<double>(env->GetStaticFloatField(clazz.obj(), fid));
}

double PlatformViewAndroidJNIImpl::GetDisplayDensity() {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return -1;
  }

  fml::jni::ScopedJavaLocalRef<jclass> clazz(
      env, env->GetObjectClass(java_object.obj()));
  if (clazz.is_null()) {
    return -1;
  }

  jfieldID fid = env->GetStaticFieldID(clazz.obj(), "displayDensity", "F");
  return static_cast<double>(env->GetStaticFloatField(clazz.obj(), fid));
}

bool PlatformViewAndroidJNIImpl::RequestDartDeferredLibrary(
    int loading_unit_id) {
  JNIEnv* env = fml::jni::AttachCurrentThread();

  auto java_object = java_object_.get(env);
  if (java_object.is_null()) {
    return true;
  }

  env->CallVoidMethod(java_object.obj(), g_request_dart_deferred_library_method,
                      loading_unit_id);

  FML_CHECK(fml::jni::CheckException(env));
  return true;
}

}  // namespace flutter
