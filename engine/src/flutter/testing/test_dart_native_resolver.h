// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_DART_NATIVE_RESOLVER_H_
#define FLUTTER_TESTING_TEST_DART_NATIVE_RESOLVER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "flutter/fml/macros.h"
#include "third_party/dart/runtime/include/dart_api.h"

#define CREATE_NATIVE_ENTRY(native_entry)                                   \
  ([&]() {                                                                  \
    static ::flutter::testing::NativeEntry closure;                         \
    static Dart_NativeFunction entrypoint = [](Dart_NativeArguments args) { \
      closure(args);                                                        \
    };                                                                      \
    closure = (native_entry);                                               \
    return entrypoint;                                                      \
  })()

namespace flutter::testing {

using NativeEntry = std::function<void(Dart_NativeArguments)>;

class TestDartNativeResolver
    : public std::enable_shared_from_this<TestDartNativeResolver> {
 public:
  TestDartNativeResolver();

  ~TestDartNativeResolver();

  void AddNativeCallback(const std::string& name, Dart_NativeFunction callback);
  void AddFfiNativeCallback(const std::string& name, void* callback_ptr);

  void SetNativeResolverForIsolate();

 private:
  std::map<std::string, Dart_NativeFunction> native_callbacks_;
  std::map<std::string, void*> ffi_native_callbacks_;

  Dart_NativeFunction ResolveCallback(const std::string& name) const;
  void* ResolveFfiCallback(const std::string& name) const;

  static Dart_NativeFunction DartNativeEntryResolverCallback(
      Dart_Handle dart_name,
      int num_of_arguments,
      bool* auto_setup_scope);
  static void* FfiNativeResolver(const char* name, uintptr_t args_n);

  FML_DISALLOW_COPY_AND_ASSIGN(TestDartNativeResolver);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_DART_NATIVE_RESOLVER_H_
