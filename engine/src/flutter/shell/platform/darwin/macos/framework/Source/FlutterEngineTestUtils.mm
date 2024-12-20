
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngineTestUtils.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"

#include "flutter/testing/testing.h"

/**
 * Fake pasteboard implementation to allow tests to work in environments without a real
 * pasteboard.
 */
@interface FakePasteboard : FlutterPasteboard
@end

@implementation FakePasteboard {
  NSString* _result;
}

- (NSInteger)clearContents {
  size_t changeCount = (_result != nil) ? 1 : 0;
  _result = nil;
  return changeCount;
}

- (NSString*)stringForType:(NSPasteboardType)dataType {
  return _result;
}

- (BOOL)setString:(NSString*)string forType:(NSPasteboardType)dataType {
  _result = string;
  return YES;
}

@end

namespace flutter::testing {

FlutterEngineTest::FlutterEngineTest() = default;

void FlutterEngineTest::SetUp() {
  native_resolver_ = std::make_shared<TestDartNativeResolver>();
  NSString* fixtures = @(testing::GetFixturesPath());
  project_ = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  project_.rootIsolateCreateCallback = FlutterEngineTest::IsolateCreateCallback;
  engine_ = [[FlutterEngine alloc] initWithName:@"test"
                                        project:project_
                         allowHeadlessExecution:true];
}

void FlutterEngineTest::TearDown() {
  [engine_ shutDownEngine];
  engine_ = nil;
  native_resolver_.reset();
}

void FlutterEngineTest::ShutDownEngine() {
  [engine_ shutDownEngine];
  engine_ = nil;
}

void FlutterEngineTest::IsolateCreateCallback(void* user_data) {
  native_resolver_->SetNativeResolverForIsolate();
}

void FlutterEngineTest::AddNativeCallback(const char* name, Dart_NativeFunction function) {
  native_resolver_->AddNativeCallback({name}, function);
}

id CreateMockFlutterEngine(NSString* pasteboardString) {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:project
                                       allowHeadlessExecution:true];

  FakePasteboard* pasteboardMock = [[FakePasteboard alloc] init];
  [pasteboardMock setString:pasteboardString forType:NSPasteboardTypeString];
  engine.pasteboard = pasteboardMock;
  id engineMock = OCMPartialMock(engine);
  return engineMock;
}

MockFlutterEngineTest::MockFlutterEngineTest() = default;

void MockFlutterEngineTest::SetUp() {
  engine_mock_ = CreateMockFlutterEngine(@"");
}

void MockFlutterEngineTest::TearDown() {
  [engine_mock_ shutDownEngine];
  [engine_mock_ stopMocking];
  engine_mock_ = nil;
}

void MockFlutterEngineTest::ShutDownEngine() {
  [engine_mock_ shutDownEngine];
  engine_mock_ = nil;
}

}  // namespace flutter::testing
