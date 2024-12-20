// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/android/jni_util.h"

#include <sys/prctl.h>

#include <memory>
#include <string>

#include "flutter/fml/logging.h"
#include "flutter/fml/string_conversion.h"

namespace fml {
namespace jni {

static JavaVM* g_jvm = nullptr;

#define ASSERT_NO_EXCEPTION() FML_CHECK(env->ExceptionCheck() == JNI_FALSE);

struct JNIDetach {
  ~JNIDetach() { DetachFromVM(); }
};

// Thread-local object that will detach from JNI during thread shutdown;
static thread_local std::unique_ptr<JNIDetach> tls_jni_detach;

void InitJavaVM(JavaVM* vm) {
  FML_DCHECK(g_jvm == nullptr);
  g_jvm = vm;
}

JNIEnv* AttachCurrentThread() {
  FML_DCHECK(g_jvm != nullptr)
      << "Trying to attach to current thread without calling InitJavaVM first.";

  JNIEnv* env = nullptr;
  if (g_jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_4) ==
      JNI_OK) {
    return env;
  }

  JavaVMAttachArgs args;
  args.version = JNI_VERSION_1_4;
  args.group = nullptr;
  // 16 is the maximum size for thread names on Android.
  char thread_name[16];
  int err = prctl(PR_GET_NAME, thread_name);
  if (err < 0) {
    args.name = nullptr;
  } else {
    args.name = thread_name;
  }
  [[maybe_unused]] jint ret = g_jvm->AttachCurrentThread(&env, &args);
  FML_DCHECK(JNI_OK == ret);

  FML_DCHECK(tls_jni_detach.get() == nullptr);
  tls_jni_detach.reset(new JNIDetach());

  return env;
}

void DetachFromVM() {
  if (g_jvm) {
    g_jvm->DetachCurrentThread();
  }
}

std::string JavaStringToString(JNIEnv* env, jstring str) {
  if (env == nullptr || str == nullptr) {
    return "";
  }
  const jchar* chars = env->GetStringChars(str, NULL);
  if (chars == nullptr) {
    return "";
  }
  std::u16string u16_string(reinterpret_cast<const char16_t*>(chars),
                            env->GetStringLength(str));
  std::string u8_string = Utf16ToUtf8(u16_string);
  env->ReleaseStringChars(str, chars);
  ASSERT_NO_EXCEPTION();
  return u8_string;
}

ScopedJavaLocalRef<jstring> StringToJavaString(JNIEnv* env,
                                               const std::string& u8_string) {
  std::u16string u16_string = Utf8ToUtf16(u8_string);
  auto result = ScopedJavaLocalRef<jstring>(
      env, env->NewString(reinterpret_cast<const jchar*>(u16_string.data()),
                          u16_string.length()));
  ASSERT_NO_EXCEPTION();
  return result;
}

std::vector<std::string> StringArrayToVector(JNIEnv* env, jobjectArray array) {
  std::vector<std::string> out;
  if (env == nullptr || array == nullptr) {
    return out;
  }

  jsize length = env->GetArrayLength(array);

  if (length == -1) {
    return out;
  }

  out.resize(length);
  for (jsize i = 0; i < length; ++i) {
    ScopedJavaLocalRef<jstring> java_string(
        env, static_cast<jstring>(env->GetObjectArrayElement(array, i)));
    out[i] = JavaStringToString(env, java_string.obj());
  }

  return out;
}

std::vector<std::string> StringListToVector(JNIEnv* env, jobject list) {
  std::vector<std::string> out;
  if (env == nullptr || list == nullptr) {
    return out;
  }

  ScopedJavaLocalRef<jclass> list_clazz(env, env->FindClass("java/util/List"));
  FML_DCHECK(!list_clazz.is_null());

  jmethodID list_get =
      env->GetMethodID(list_clazz.obj(), "get", "(I)Ljava/lang/Object;");
  jmethodID list_size = env->GetMethodID(list_clazz.obj(), "size", "()I");

  jint size = env->CallIntMethod(list, list_size);

  if (size == 0) {
    return out;
  }

  out.resize(size);
  for (jint i = 0; i < size; ++i) {
    ScopedJavaLocalRef<jstring> java_string(
        env, static_cast<jstring>(env->CallObjectMethod(list, list_get, i)));
    out[i] = JavaStringToString(env, java_string.obj());
  }

  return out;
}

ScopedJavaLocalRef<jobjectArray> VectorToStringArray(
    JNIEnv* env,
    const std::vector<std::string>& vector) {
  FML_DCHECK(env);
  ScopedJavaLocalRef<jclass> string_clazz(env,
                                          env->FindClass("java/lang/String"));
  FML_DCHECK(!string_clazz.is_null());
  jobjectArray java_array =
      env->NewObjectArray(vector.size(), string_clazz.obj(), NULL);
  ASSERT_NO_EXCEPTION();
  for (size_t i = 0; i < vector.size(); ++i) {
    ScopedJavaLocalRef<jstring> item = StringToJavaString(env, vector[i]);
    env->SetObjectArrayElement(java_array, i, item.obj());
  }
  return ScopedJavaLocalRef<jobjectArray>(env, java_array);
}

ScopedJavaLocalRef<jobjectArray> VectorToBufferArray(
    JNIEnv* env,
    const std::vector<std::vector<uint8_t>>& vector) {
  FML_DCHECK(env);
  ScopedJavaLocalRef<jclass> byte_buffer_clazz(
      env, env->FindClass("java/nio/ByteBuffer"));
  FML_DCHECK(!byte_buffer_clazz.is_null());
  jobjectArray java_array =
      env->NewObjectArray(vector.size(), byte_buffer_clazz.obj(), NULL);
  ASSERT_NO_EXCEPTION();
  for (size_t i = 0; i < vector.size(); ++i) {
    uint8_t* data = const_cast<uint8_t*>(vector[i].data());
    ScopedJavaLocalRef<jobject> item(
        env, env->NewDirectByteBuffer(reinterpret_cast<void*>(data),
                                      vector[i].size()));
    env->SetObjectArrayElement(java_array, i, item.obj());
  }
  return ScopedJavaLocalRef<jobjectArray>(env, java_array);
}

bool HasException(JNIEnv* env) {
  return env->ExceptionCheck() != JNI_FALSE;
}

bool ClearException(JNIEnv* env, bool silent) {
  if (!HasException(env)) {
    return false;
  }
  if (!silent) {
    env->ExceptionDescribe();
  }
  env->ExceptionClear();
  return true;
}

bool CheckException(JNIEnv* env) {
  if (!HasException(env)) {
    return true;
  }

  jthrowable exception = env->ExceptionOccurred();
  env->ExceptionClear();
  FML_LOG(ERROR) << fml::jni::GetJavaExceptionInfo(env, exception);
  env->DeleteLocalRef(exception);
  return false;
}

std::string GetJavaExceptionInfo(JNIEnv* env, jthrowable java_throwable) {
  ScopedJavaLocalRef<jclass> throwable_clazz(
      env, env->FindClass("java/lang/Throwable"));
  jmethodID throwable_printstacktrace = env->GetMethodID(
      throwable_clazz.obj(), "printStackTrace", "(Ljava/io/PrintStream;)V");

  // Create an instance of ByteArrayOutputStream.
  ScopedJavaLocalRef<jclass> bytearray_output_stream_clazz(
      env, env->FindClass("java/io/ByteArrayOutputStream"));
  jmethodID bytearray_output_stream_constructor =
      env->GetMethodID(bytearray_output_stream_clazz.obj(), "<init>", "()V");
  jmethodID bytearray_output_stream_tostring = env->GetMethodID(
      bytearray_output_stream_clazz.obj(), "toString", "()Ljava/lang/String;");
  ScopedJavaLocalRef<jobject> bytearray_output_stream(
      env, env->NewObject(bytearray_output_stream_clazz.obj(),
                          bytearray_output_stream_constructor));

  // Create an instance of PrintStream.
  ScopedJavaLocalRef<jclass> printstream_clazz(
      env, env->FindClass("java/io/PrintStream"));
  jmethodID printstream_constructor = env->GetMethodID(
      printstream_clazz.obj(), "<init>", "(Ljava/io/OutputStream;)V");
  ScopedJavaLocalRef<jobject> printstream(
      env, env->NewObject(printstream_clazz.obj(), printstream_constructor,
                          bytearray_output_stream.obj()));

  // Call Throwable.printStackTrace(PrintStream)
  env->CallVoidMethod(java_throwable, throwable_printstacktrace,
                      printstream.obj());

  // Call ByteArrayOutputStream.toString()
  ScopedJavaLocalRef<jstring> exception_string(
      env,
      static_cast<jstring>(env->CallObjectMethod(
          bytearray_output_stream.obj(), bytearray_output_stream_tostring)));
  if (ClearException(env)) {
    return "Java OOM'd in exception handling, check logcat";
  }

  return JavaStringToString(env, exception_string.obj());
}

}  // namespace jni
}  // namespace fml
