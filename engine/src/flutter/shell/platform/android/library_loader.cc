// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/android/jni_util.h"
#include "flutter/shell/platform/android/android_image_generator.h"
#include "flutter/shell/platform/android/flutter_main.h"
#include "flutter/shell/platform/android/platform_view_android.h"
#include "flutter/shell/platform/android/vsync_waiter_android.h"

// This is called by the VM when the shared library is first loaded.
JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
  // Initialize the Java VM.
  fml::jni::InitJavaVM(vm);

  JNIEnv* env = fml::jni::AttachCurrentThread();
  bool result = false;

  // Register FlutterMain.
  result = flutter::FlutterMain::Register(env);
  FML_CHECK(result);

  // Register PlatformView
  result = flutter::PlatformViewAndroid::Register(env);
  FML_CHECK(result);

  // Register VSyncWaiter.
  result = flutter::VsyncWaiterAndroid::Register(env);
  FML_CHECK(result);

  // Register AndroidImageDecoder.
  result = flutter::AndroidImageGenerator::Register(env);
  FML_CHECK(result);

  return JNI_VERSION_1_4;
}
