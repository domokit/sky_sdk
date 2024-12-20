// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINETESTUTILS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINETESTUTILS_H_

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"

#import <OCMock/OCMock.h>

#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/test_dart_native_resolver.h"
#include "gtest/gtest.h"

namespace flutter::testing {

class FlutterEngineTest : public AutoreleasePoolTest {
 public:
  FlutterEngineTest();

  FlutterEngine* GetFlutterEngine() { return engine_; };

  void SetUp() override;
  void TearDown() override;

  void AddNativeCallback(const char* name, Dart_NativeFunction function);

  static void IsolateCreateCallback(void* user_data);

  void ShutDownEngine();

 private:
  inline static std::shared_ptr<TestDartNativeResolver> native_resolver_;

  FlutterDartProject* project_;
  FlutterEngine* engine_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterEngineTest);
};

// Returns a mock FlutterEngine that is able to work in environments
// without a real pasteboard.
//
// Callers MUST call [mockEngine shutDownEngine] when finished with the returned engine.
id CreateMockFlutterEngine(NSString* pasteboardString);

class MockFlutterEngineTest : public AutoreleasePoolTest {
 public:
  MockFlutterEngineTest();

  void SetUp() override;
  void TearDown() override;

  id GetMockEngine() { return engine_mock_; }

  void ShutDownEngine();

  ~MockFlutterEngineTest() {
    [engine_mock_ shutDownEngine];
    [engine_mock_ stopMocking];
  }

 private:
  id engine_mock_;

  FML_DISALLOW_COPY_AND_ASSIGN(MockFlutterEngineTest);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINETESTUTILS_H_
