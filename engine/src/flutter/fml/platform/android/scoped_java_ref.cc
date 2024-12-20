// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/android/scoped_java_ref.h"

#include "flutter/fml/logging.h"
#include "flutter/fml/platform/android/jni_util.h"

namespace fml {
namespace jni {

static const int kDefaultLocalFrameCapacity = 16;

ScopedJavaLocalFrame::ScopedJavaLocalFrame(JNIEnv* env) : env_(env) {
  [[maybe_unused]] int failed =
      env_->PushLocalFrame(kDefaultLocalFrameCapacity);
  FML_DCHECK(!failed);
}

ScopedJavaLocalFrame::ScopedJavaLocalFrame(JNIEnv* env, int capacity)
    : env_(env) {
  [[maybe_unused]] int failed = env_->PushLocalFrame(capacity);
  FML_DCHECK(!failed);
}

ScopedJavaLocalFrame::~ScopedJavaLocalFrame() {
  env_->PopLocalFrame(NULL);
}

JavaRef<jobject>::JavaRef() : obj_(NULL) {}

JavaRef<jobject>::JavaRef(JNIEnv* env, jobject obj) : obj_(obj) {
  if (obj) {
    FML_DCHECK(env && env->GetObjectRefType(obj) == JNILocalRefType);
  }
}

JavaRef<jobject>::~JavaRef() = default;

JNIEnv* JavaRef<jobject>::SetNewLocalRef(JNIEnv* env, jobject obj) {
  if (!env) {
    env = AttachCurrentThread();
  } else {
    FML_DCHECK(env == AttachCurrentThread());  // Is |env| on correct thread.
  }
  if (obj) {
    obj = env->NewLocalRef(obj);
  }
  if (obj_) {
    env->DeleteLocalRef(obj_);
  }
  obj_ = obj;
  return env;
}

void JavaRef<jobject>::SetNewGlobalRef(JNIEnv* env, jobject obj) {
  if (!env) {
    env = AttachCurrentThread();
  } else {
    FML_DCHECK(env == AttachCurrentThread());  // Is |env| on correct thread.
  }
  if (obj) {
    obj = env->NewGlobalRef(obj);
  }
  if (obj_) {
    env->DeleteGlobalRef(obj_);
  }
  obj_ = obj;
}

void JavaRef<jobject>::ResetLocalRef(JNIEnv* env) {
  if (obj_) {
    FML_DCHECK(env == AttachCurrentThread());  // Is |env| on correct thread.
    env->DeleteLocalRef(obj_);
    obj_ = NULL;
  }
}

void JavaRef<jobject>::ResetGlobalRef() {
  if (obj_) {
    AttachCurrentThread()->DeleteGlobalRef(obj_);
    obj_ = NULL;
  }
}

jobject JavaRef<jobject>::ReleaseInternal() {
  jobject obj = obj_;
  obj_ = NULL;
  return obj;
}

}  // namespace jni
}  // namespace fml
