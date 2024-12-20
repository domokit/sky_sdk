// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONTEXT_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONTEXT_H_

#include <string>
#include <string_view>
#include <vector>

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/testing/test_dart_native_resolver.h"

namespace flutter {
namespace testing {

// Context associated with the current Windows test fixture.
//
// Context data includes global Flutter and Dart runtime context such as the
// path of Flutter's asset directory, ICU path, and resolvers for any
// registered native functions.
class WindowsTestContext {
 public:
  explicit WindowsTestContext(std::string_view assets_path = "");
  virtual ~WindowsTestContext();

  // Returns the path to assets required by the Flutter runtime.
  const std::wstring& GetAssetsPath() const;

  // Returns the path to the ICU library data file.
  const std::wstring& GetIcuDataPath() const;

  // Returns the path to the application's AOT library file.
  const std::wstring& GetAotLibraryPath() const;

  // Registers a native function callable from Dart code in test fixtures. In
  // the Dart test fixture, the associated function can be declared as:
  //
  //   @pragma('vm:external-name', 'IdentifyingName')
  //   external ReturnType functionName();
  //
  // where `IdentifyingName` matches the |name| parameter to this method.
  void AddNativeFunction(std::string_view name, Dart_NativeFunction function);

  // Returns the root isolate create callback to register with the Flutter
  // runtime.
  fml::closure GetRootIsolateCallback();

 private:
  std::wstring assets_path_;
  std::wstring icu_data_path_ = L"icudtl.dat";
  std::wstring aot_library_path_ = L"aot.so";
  std::vector<fml::closure> isolate_create_callbacks_;
  std::shared_ptr<TestDartNativeResolver> native_resolver_;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowsTestContext);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONTEXT_H_
