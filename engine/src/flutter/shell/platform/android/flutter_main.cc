// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <android/log.h>
#include <optional>
#include <vector>

#include "common/settings.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/file.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/native_library.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/platform/android/jni_util.h"
#include "flutter/fml/platform/android/paths_android.h"
#include "flutter/lib/ui/plugins/callback_cache.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/platform/android/android_context_vk_impeller.h"
#include "flutter/shell/platform/android/context/android_context.h"
#include "flutter/shell/platform/android/flutter_main.h"
#include "impeller/base/validation.h"
#include "impeller/toolkit/android/proc_table.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"
#include "txt/platform.h"

namespace flutter {

constexpr int kMinimumAndroidApiLevelForVulkan = 29;

extern "C" {
#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
// Used for debugging dart:* sources.
extern const uint8_t kPlatformStrongDill[];
extern const intptr_t kPlatformStrongDillSize;
#endif
}

namespace {

fml::jni::ScopedJavaGlobalRef<jclass>* g_flutter_jni_class = nullptr;

}  // anonymous namespace

FlutterMain::FlutterMain(const flutter::Settings& settings)
    : settings_(settings) {}

FlutterMain::~FlutterMain() = default;

static std::unique_ptr<FlutterMain> g_flutter_main;

FlutterMain& FlutterMain::Get() {
  FML_CHECK(g_flutter_main) << "ensureInitializationComplete must have already "
                               "been called.";
  return *g_flutter_main;
}

const flutter::Settings& FlutterMain::GetSettings() const {
  return settings_;
}

void FlutterMain::Init(JNIEnv* env,
                       jclass clazz,
                       jobject context,
                       jobjectArray jargs,
                       jstring kernelPath,
                       jstring appStoragePath,
                       jstring engineCachesPath,
                       jlong initTimeMillis) {
  std::vector<std::string> args;
  args.push_back("flutter");
  for (auto& arg : fml::jni::StringArrayToVector(env, jargs)) {
    args.push_back(std::move(arg));
  }
  auto command_line = fml::CommandLineFromIterators(args.begin(), args.end());

  auto settings = SettingsFromCommandLine(command_line);

  // Turn systracing on if ATrace_isEnabled is true and the user did not already
  // request systracing
  if (!settings.trace_systrace) {
    settings.trace_systrace =
        impeller::android::GetProcTable().TraceIsEnabled();
    if (settings.trace_systrace) {
      __android_log_print(
          ANDROID_LOG_INFO, "Flutter",
          "ATrace was enabled at startup. Flutter and Dart "
          "tracing will be forwarded to systrace and will not show up in "
          "Dart DevTools.");
    }
  }

  settings.android_rendering_api = SelectedRenderingAPI(settings);
  switch (settings.android_rendering_api) {
    case AndroidRenderingAPI::kSoftware:
    case AndroidRenderingAPI::kSkiaOpenGLES:
      settings.enable_impeller = false;
      break;
    case AndroidRenderingAPI::kImpellerOpenGLES:
    case AndroidRenderingAPI::kImpellerVulkan:
      settings.enable_impeller = true;
      break;
  }

#if FLUTTER_RELEASE
  // On most platforms the timeline is always disabled in release mode.
  // On Android, enable it in release mode only when using systrace.
  settings.enable_timeline_event_handler = settings.trace_systrace;
#endif  // FLUTTER_RELEASE

  // Restore the callback cache.
  // TODO(chinmaygarde): Route all cache file access through FML and remove this
  // setter.
  flutter::DartCallbackCache::SetCachePath(
      fml::jni::JavaStringToString(env, appStoragePath));

  fml::paths::InitializeAndroidCachesPath(
      fml::jni::JavaStringToString(env, engineCachesPath));

  flutter::DartCallbackCache::LoadCacheFromDisk();

  if (!flutter::DartVM::IsRunningPrecompiledCode() && kernelPath) {
    // Check to see if the appropriate kernel files are present and configure
    // settings accordingly.
    auto application_kernel_path =
        fml::jni::JavaStringToString(env, kernelPath);

    if (fml::IsFile(application_kernel_path)) {
      settings.application_kernel_asset = application_kernel_path;
    }
  }

  settings.task_observer_add = [](intptr_t key, const fml::closure& callback) {
    fml::MessageLoop::GetCurrent().AddTaskObserver(key, callback);
  };

  settings.task_observer_remove = [](intptr_t key) {
    fml::MessageLoop::GetCurrent().RemoveTaskObserver(key);
  };

  settings.log_message_callback = [](const std::string& tag,
                                     const std::string& message) {
    __android_log_print(ANDROID_LOG_INFO, tag.c_str(), "%.*s",
                        static_cast<int>(message.size()), message.c_str());
  };

  settings.enable_platform_isolates = true;

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
  // There are no ownership concerns here as all mappings are owned by the
  // embedder and not the engine.
  auto make_mapping_callback = [](const uint8_t* mapping, size_t size) {
    return [mapping, size]() {
      return std::make_unique<fml::NonOwnedMapping>(mapping, size);
    };
  };

  settings.dart_library_sources_kernel =
      make_mapping_callback(kPlatformStrongDill, kPlatformStrongDillSize);
#endif  // FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG

  // Not thread safe. Will be removed when FlutterMain is refactored to no
  // longer be a singleton.
  g_flutter_main.reset(new FlutterMain(settings));

  g_flutter_main->SetupDartVMServiceUriCallback(env);
}

void FlutterMain::SetupDartVMServiceUriCallback(JNIEnv* env) {
  g_flutter_jni_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("io/flutter/embedding/engine/FlutterJNI"));
  if (g_flutter_jni_class->is_null()) {
    return;
  }
  jfieldID uri_field = env->GetStaticFieldID(
      g_flutter_jni_class->obj(), "vmServiceUri", "Ljava/lang/String;");
  if (uri_field == nullptr) {
    return;
  }

  auto set_uri = [env, uri_field](const std::string& uri) {
    fml::jni::ScopedJavaLocalRef<jstring> java_uri =
        fml::jni::StringToJavaString(env, uri);
    env->SetStaticObjectField(g_flutter_jni_class->obj(), uri_field,
                              java_uri.obj());
  };

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  fml::RefPtr<fml::TaskRunner> platform_runner =
      fml::MessageLoop::GetCurrent().GetTaskRunner();

  vm_service_uri_callback_ = DartServiceIsolate::AddServerStatusCallback(
      [platform_runner, set_uri](const std::string& uri) {
        platform_runner->PostTask([uri, set_uri] { set_uri(uri); });
      });
}

static void PrefetchDefaultFontManager(JNIEnv* env, jclass jcaller) {
  // Initialize a singleton owned by Skia.
  txt::GetDefaultFontManager();
}

bool FlutterMain::Register(JNIEnv* env) {
  static const JNINativeMethod methods[] = {
      {
          .name = "nativeInit",
          .signature = "(Landroid/content/Context;[Ljava/lang/String;Ljava/"
                       "lang/String;Ljava/lang/String;Ljava/lang/String;J)V",
          .fnPtr = reinterpret_cast<void*>(&Init),
      },
      {
          .name = "nativePrefetchDefaultFontManager",
          .signature = "()V",
          .fnPtr = reinterpret_cast<void*>(&PrefetchDefaultFontManager),
      },
  };

  jclass clazz = env->FindClass("io/flutter/embedding/engine/FlutterJNI");

  if (clazz == nullptr) {
    return false;
  }

  return env->RegisterNatives(clazz, methods, std::size(methods)) == 0;
}

// static
AndroidRenderingAPI FlutterMain::SelectedRenderingAPI(
    const flutter::Settings& settings) {
  if (settings.enable_software_rendering) {
    FML_CHECK(!settings.enable_impeller)
        << "Impeller does not support software rendering. Either disable "
           "software rendering or disable impeller.";
    return AndroidRenderingAPI::kSoftware;
  }
  constexpr AndroidRenderingAPI kVulkanUnsupportedFallback =
      AndroidRenderingAPI::kImpellerOpenGLES;

  // Debug/Profile only functionality for testing a specific
  // backend configuration.
#ifndef FLUTTER_RELEASE
  if (settings.requested_rendering_backend == "opengles" &&
      settings.enable_impeller) {
    return AndroidRenderingAPI::kImpellerOpenGLES;
  }
  if (settings.requested_rendering_backend == "vulkan" &&
      settings.enable_impeller) {
    return AndroidRenderingAPI::kImpellerVulkan;
  }
#endif

  if (settings.enable_impeller) {
    // Vulkan must only be used on API level 29+, as older API levels do not
    // have requisite features to support platform views.
    //
    // Even if this check returns true, Impeller may determine it cannot use
    // Vulkan for some other reason, such as a missing required extension or
    // feature.
    int api_level = android_get_device_api_level();
    if (api_level < kMinimumAndroidApiLevelForVulkan) {
      return kVulkanUnsupportedFallback;
    }
    // Determine if Vulkan is supported by creating a Vulkan context and
    // checking if it is valid.
    impeller::ScopedValidationDisable disable_validation;
    auto vulkan_backend = std::make_unique<AndroidContextVKImpeller>(
        AndroidContext::ContextSettings{.enable_validation = false,
                                        .enable_gpu_tracing = false,
                                        .quiet = true});
    if (!vulkan_backend->IsValid()) {
      return kVulkanUnsupportedFallback;
    }
    return AndroidRenderingAPI::kImpellerVulkan;
  }

  return AndroidRenderingAPI::kSkiaOpenGLES;
}

}  // namespace flutter
