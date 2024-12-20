// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import <objc/runtime.h>

#import "flutter/common/settings.h"
#include "flutter/fml/synchronization/sync_switch.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Test.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"

FLUTTER_ASSERT_ARC

@interface FlutterEngineSpy : FlutterEngine
@property(nonatomic) BOOL ensureSemanticsEnabledCalled;
@end

@implementation FlutterEngineSpy

- (void)ensureSemanticsEnabled {
  _ensureSemanticsEnabledCalled = YES;
}

@end

@interface FlutterEngine () <FlutterTextInputDelegate>

@end

/// FlutterBinaryMessengerRelay used for testing that setting FlutterEngine.binaryMessenger to
/// the current instance doesn't trigger a use-after-free bug.
///
/// See: testSetBinaryMessengerToSameBinaryMessenger
@interface FakeBinaryMessengerRelay : FlutterBinaryMessengerRelay
@property(nonatomic, assign) BOOL failOnDealloc;
@end

@implementation FakeBinaryMessengerRelay
- (void)dealloc {
  if (_failOnDealloc) {
    XCTFail("FakeBinaryMessageRelay should not be deallocated");
  }
}
@end

@interface FlutterEngineTest : XCTestCase
@end

@implementation FlutterEngineTest

- (void)setUp {
}

- (void)tearDown {
}

- (void)testCreate {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  XCTAssertNotNil(engine);
}

- (void)testShellGetters {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  XCTAssertNotNil(engine);

  // Ensure getters don't deref _shell when it's null, and instead return nullptr.
  XCTAssertEqual(engine.platformTaskRunner.get(), nullptr);
  XCTAssertEqual(engine.uiTaskRunner.get(), nullptr);
  XCTAssertEqual(engine.rasterTaskRunner.get(), nullptr);
}

- (void)testInfoPlist {
  // Check the embedded Flutter.framework Info.plist, not the linked dylib.
  NSURL* flutterFrameworkURL =
      [NSBundle.mainBundle.privateFrameworksURL URLByAppendingPathComponent:@"Flutter.framework"];
  NSBundle* flutterBundle = [NSBundle bundleWithURL:flutterFrameworkURL];
  XCTAssertEqualObjects(flutterBundle.bundleIdentifier, @"io.flutter.flutter");

  NSDictionary<NSString*, id>* infoDictionary = flutterBundle.infoDictionary;

  // OS version can have one, two, or three digits: "8", "8.0", "8.0.0"
  NSError* regexError = NULL;
  NSRegularExpression* osVersionRegex =
      [NSRegularExpression regularExpressionWithPattern:@"((0|[1-9]\\d*)\\.)*(0|[1-9]\\d*)"
                                                options:NSRegularExpressionCaseInsensitive
                                                  error:&regexError];
  XCTAssertNil(regexError);

  // Smoke test the test regex.
  NSString* testString = @"9";
  NSUInteger versionMatches =
      [osVersionRegex numberOfMatchesInString:testString
                                      options:NSMatchingAnchored
                                        range:NSMakeRange(0, testString.length)];
  XCTAssertEqual(versionMatches, 1UL);
  testString = @"9.1";
  versionMatches = [osVersionRegex numberOfMatchesInString:testString
                                                   options:NSMatchingAnchored
                                                     range:NSMakeRange(0, testString.length)];
  XCTAssertEqual(versionMatches, 1UL);
  testString = @"9.0.1";
  versionMatches = [osVersionRegex numberOfMatchesInString:testString
                                                   options:NSMatchingAnchored
                                                     range:NSMakeRange(0, testString.length)];
  XCTAssertEqual(versionMatches, 1UL);
  testString = @".0.1";
  versionMatches = [osVersionRegex numberOfMatchesInString:testString
                                                   options:NSMatchingAnchored
                                                     range:NSMakeRange(0, testString.length)];
  XCTAssertEqual(versionMatches, 0UL);

  // Test Info.plist values.
  NSString* minimumOSVersion = infoDictionary[@"MinimumOSVersion"];
  versionMatches = [osVersionRegex numberOfMatchesInString:minimumOSVersion
                                                   options:NSMatchingAnchored
                                                     range:NSMakeRange(0, minimumOSVersion.length)];
  XCTAssertEqual(versionMatches, 1UL);

  // SHA length is 40.
  XCTAssertEqual(((NSString*)infoDictionary[@"FlutterEngine"]).length, 40UL);

  // {clang_version} placeholder is 15 characters. The clang string version
  // is longer than that, so check if the placeholder has been replaced, without
  // actually checking a literal string, which could be different on various machines.
  XCTAssertTrue(((NSString*)infoDictionary[@"ClangVersion"]).length > 15UL);
}

- (void)testDeallocated {
  __weak FlutterEngine* weakEngine = nil;
  @autoreleasepool {
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar"];
    weakEngine = engine;
    [engine run];
    XCTAssertNotNil(weakEngine);
  }
  XCTAssertNil(weakEngine);
}

- (void)testSendMessageBeforeRun {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  XCTAssertNotNil(engine);
  XCTAssertThrows([engine.binaryMessenger
      sendOnChannel:@"foo"
            message:[@"bar" dataUsingEncoding:NSUTF8StringEncoding]
        binaryReply:nil]);
}

- (void)testSetMessageHandlerBeforeRun {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  XCTAssertNotNil(engine);
  XCTAssertThrows([engine.binaryMessenger
      setMessageHandlerOnChannel:@"foo"
            binaryMessageHandler:^(NSData* _Nullable message, FlutterBinaryReply _Nonnull reply){

            }]);
}

- (void)testNilSetMessageHandlerBeforeRun {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  XCTAssertNotNil(engine);
  XCTAssertNoThrow([engine.binaryMessenger setMessageHandlerOnChannel:@"foo"
                                                 binaryMessageHandler:nil]);
}

- (void)testNotifyPluginOfDealloc {
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  OCMStub([plugin detachFromEngineForRegistrar:[OCMArg any]]);
  {
    FlutterDartProject* project = [[FlutterDartProject alloc] init];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"engine" project:project];
    NSObject<FlutterPluginRegistrar>* registrar = [engine registrarForPlugin:@"plugin"];
    [registrar publish:plugin];
    engine = nil;
  }
  OCMVerify([plugin detachFromEngineForRegistrar:[OCMArg any]]);
}

- (void)testSetBinaryMessengerToSameBinaryMessenger {
  FakeBinaryMessengerRelay* fakeBinaryMessenger = [[FakeBinaryMessengerRelay alloc] init];

  FlutterEngine* engine = [[FlutterEngine alloc] init];
  [engine setBinaryMessenger:fakeBinaryMessenger];

  // Verify that the setter doesn't free the old messenger before setting the new messenger.
  fakeBinaryMessenger.failOnDealloc = YES;
  [engine setBinaryMessenger:fakeBinaryMessenger];

  // Don't fail when ARC releases the binary messenger.
  fakeBinaryMessenger.failOnDealloc = NO;
}

- (void)testRunningInitialRouteSendsNavigationMessage {
  id mockBinaryMessenger = OCMClassMock([FlutterBinaryMessengerRelay class]);

  FlutterEngine* engine = [[FlutterEngine alloc] init];
  [engine setBinaryMessenger:mockBinaryMessenger];

  // Run with an initial route.
  [engine runWithEntrypoint:FlutterDefaultDartEntrypoint initialRoute:@"test"];

  // Now check that an encoded method call has been made on the binary messenger to set the
  // initial route to "test".
  FlutterMethodCall* setInitialRouteMethodCall =
      [FlutterMethodCall methodCallWithMethodName:@"setInitialRoute" arguments:@"test"];
  NSData* encodedSetInitialRouteMethod =
      [[FlutterJSONMethodCodec sharedInstance] encodeMethodCall:setInitialRouteMethodCall];
  OCMVerify([mockBinaryMessenger sendOnChannel:@"flutter/navigation"
                                       message:encodedSetInitialRouteMethod]);
}

- (void)testInitialRouteSettingsSendsNavigationMessage {
  id mockBinaryMessenger = OCMClassMock([FlutterBinaryMessengerRelay class]);

  auto settings = FLTDefaultSettingsForBundle();
  settings.route = "test";
  FlutterDartProject* project = [[FlutterDartProject alloc] initWithSettings:settings];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  [engine setBinaryMessenger:mockBinaryMessenger];
  [engine run];

  // Now check that an encoded method call has been made on the binary messenger to set the
  // initial route to "test".
  FlutterMethodCall* setInitialRouteMethodCall =
      [FlutterMethodCall methodCallWithMethodName:@"setInitialRoute" arguments:@"test"];
  NSData* encodedSetInitialRouteMethod =
      [[FlutterJSONMethodCodec sharedInstance] encodeMethodCall:setInitialRouteMethodCall];
  OCMVerify([mockBinaryMessenger sendOnChannel:@"flutter/navigation"
                                       message:encodedSetInitialRouteMethod]);
}

- (void)testPlatformViewsControllerRenderingMetalBackend {
  FlutterEngine* engine = [[FlutterEngine alloc] init];
  [engine run];
  flutter::IOSRenderingAPI renderingApi = [engine platformViewsRenderingAPI];

  XCTAssertEqual(renderingApi, flutter::IOSRenderingAPI::kMetal);
}

- (void)testWaitForFirstFrameTimeout {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar"];
  [engine run];
  XCTestExpectation* timeoutFirstFrame = [self expectationWithDescription:@"timeoutFirstFrame"];
  [engine waitForFirstFrame:0.1
                   callback:^(BOOL didTimeout) {
                     if (timeoutFirstFrame) {
                       [timeoutFirstFrame fulfill];
                     }
                   }];
  [self waitForExpectations:@[ timeoutFirstFrame ]];
}

- (void)testSpawn {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar"];
  [engine run];
  FlutterEngine* spawn = [engine spawnWithEntrypoint:nil
                                          libraryURI:nil
                                        initialRoute:nil
                                      entrypointArgs:nil];
  XCTAssertNotNil(spawn);
}

- (void)testSetHandlerAfterRun {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar"];
  XCTestExpectation* gotMessage = [self expectationWithDescription:@"gotMessage"];
  dispatch_async(dispatch_get_main_queue(), ^{
    NSObject<FlutterPluginRegistrar>* registrar = [engine registrarForPlugin:@"foo"];
    fml::AutoResetWaitableEvent latch;
    [engine run];
    flutter::Shell& shell = engine.shell;
    fml::TaskRunner::RunNowOrPostTask(
        engine.shell.GetTaskRunners().GetUITaskRunner(), [&latch, &shell] {
          flutter::Engine::Delegate& delegate = shell;
          auto message = std::make_unique<flutter::PlatformMessage>("foo", nullptr);
          delegate.OnEngineHandlePlatformMessage(std::move(message));
          latch.Signal();
        });
    latch.Wait();
    [registrar.messenger setMessageHandlerOnChannel:@"foo"
                               binaryMessageHandler:^(NSData* message, FlutterBinaryReply reply) {
                                 [gotMessage fulfill];
                               }];
  });
  [self waitForExpectations:@[ gotMessage ]];
}

- (void)testThreadPrioritySetCorrectly {
  XCTestExpectation* prioritiesSet = [self expectationWithDescription:@"prioritiesSet"];
  prioritiesSet.expectedFulfillmentCount = 2;

  IMP mockSetThreadPriority =
      imp_implementationWithBlock(^(NSThread* thread, double threadPriority) {
        if ([thread.name hasSuffix:@".raster"]) {
          XCTAssertEqual(threadPriority, 1.0);
          [prioritiesSet fulfill];
        } else if ([thread.name hasSuffix:@".io"]) {
          XCTAssertEqual(threadPriority, 0.5);
          [prioritiesSet fulfill];
        }
      });
  Method method = class_getInstanceMethod([NSThread class], @selector(setThreadPriority:));
  IMP originalSetThreadPriority = method_getImplementation(method);
  method_setImplementation(method, mockSetThreadPriority);

  FlutterEngine* engine = [[FlutterEngine alloc] init];
  [engine run];
  [self waitForExpectations:@[ prioritiesSet ]];

  method_setImplementation(method, originalSetThreadPriority);
}

- (void)testCanEnableDisableEmbedderAPIThroughInfoPlist {
  {
    // Not enable embedder API by default
    auto settings = FLTDefaultSettingsForBundle();
    settings.enable_software_rendering = true;
    FlutterDartProject* project = [[FlutterDartProject alloc] initWithSettings:settings];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
    XCTAssertFalse(engine.enableEmbedderAPI);
  }
  {
    // Enable embedder api
    id mockMainBundle = OCMPartialMock([NSBundle mainBundle]);
    OCMStub([mockMainBundle objectForInfoDictionaryKey:@"FLTEnableIOSEmbedderAPI"])
        .andReturn(@"YES");
    auto settings = FLTDefaultSettingsForBundle();
    settings.enable_software_rendering = true;
    FlutterDartProject* project = [[FlutterDartProject alloc] initWithSettings:settings];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
    XCTAssertTrue(engine.enableEmbedderAPI);
  }
}

- (void)testFlutterTextInputViewDidResignFirstResponderWillCallTextInputClientConnectionClosed {
  id mockBinaryMessenger = OCMClassMock([FlutterBinaryMessengerRelay class]);
  FlutterEngine* engine = [[FlutterEngine alloc] init];
  [engine setBinaryMessenger:mockBinaryMessenger];
  [engine runWithEntrypoint:FlutterDefaultDartEntrypoint initialRoute:@"test"];
  [engine flutterTextInputView:nil didResignFirstResponderWithTextInputClient:1];
  FlutterMethodCall* methodCall =
      [FlutterMethodCall methodCallWithMethodName:@"TextInputClient.onConnectionClosed"
                                        arguments:@[ @(1) ]];
  NSData* encodedMethodCall = [[FlutterJSONMethodCodec sharedInstance] encodeMethodCall:methodCall];
  OCMVerify([mockBinaryMessenger sendOnChannel:@"flutter/textinput" message:encodedMethodCall]);
}

- (void)testFlutterEngineUpdatesDisplays {
  FlutterEngine* engine = [[FlutterEngine alloc] init];
  id mockEngine = OCMPartialMock(engine);

  [engine run];
  OCMVerify(times(1), [mockEngine updateDisplays]);
  engine.viewController = nil;
  OCMVerify(times(2), [mockEngine updateDisplays]);
}

- (void)testLifeCycleNotificationDidEnterBackground {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  [engine run];
  NSNotification* sceneNotification =
      [NSNotification notificationWithName:UISceneDidEnterBackgroundNotification
                                    object:nil
                                  userInfo:nil];
  NSNotification* applicationNotification =
      [NSNotification notificationWithName:UIApplicationDidEnterBackgroundNotification
                                    object:nil
                                  userInfo:nil];
  id mockEngine = OCMPartialMock(engine);
  [[NSNotificationCenter defaultCenter] postNotification:sceneNotification];
  [[NSNotificationCenter defaultCenter] postNotification:applicationNotification];
#if APPLICATION_EXTENSION_API_ONLY
  OCMVerify(times(1), [mockEngine sceneDidEnterBackground:[OCMArg any]]);
#else
  OCMVerify(times(1), [mockEngine applicationDidEnterBackground:[OCMArg any]]);
#endif
  XCTAssertTrue(engine.isGpuDisabled);
  bool switch_value = false;
  [engine shell].GetIsGpuDisabledSyncSwitch()->Execute(
      fml::SyncSwitch::Handlers().SetIfTrue([&] { switch_value = true; }).SetIfFalse([&] {
        switch_value = false;
      }));
  XCTAssertTrue(switch_value);
}

- (void)testLifeCycleNotificationWillEnterForeground {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  [engine run];
  NSNotification* sceneNotification =
      [NSNotification notificationWithName:UISceneWillEnterForegroundNotification
                                    object:nil
                                  userInfo:nil];
  NSNotification* applicationNotification =
      [NSNotification notificationWithName:UIApplicationWillEnterForegroundNotification
                                    object:nil
                                  userInfo:nil];
  id mockEngine = OCMPartialMock(engine);
  [[NSNotificationCenter defaultCenter] postNotification:sceneNotification];
  [[NSNotificationCenter defaultCenter] postNotification:applicationNotification];
#if APPLICATION_EXTENSION_API_ONLY
  OCMVerify(times(1), [mockEngine sceneWillEnterForeground:[OCMArg any]]);
#else
  OCMVerify(times(1), [mockEngine applicationWillEnterForeground:[OCMArg any]]);
#endif
  XCTAssertFalse(engine.isGpuDisabled);
  bool switch_value = true;
  [engine shell].GetIsGpuDisabledSyncSwitch()->Execute(
      fml::SyncSwitch::Handlers().SetIfTrue([&] { switch_value = true; }).SetIfFalse([&] {
        switch_value = false;
      }));
  XCTAssertFalse(switch_value);
}

- (void)testSpawnsShareGpuContext {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar"];
  [engine run];
  FlutterEngine* spawn = [engine spawnWithEntrypoint:nil
                                          libraryURI:nil
                                        initialRoute:nil
                                      entrypointArgs:nil];
  XCTAssertNotNil(spawn);
  XCTAssertTrue(engine.platformView != nullptr);
  XCTAssertTrue(spawn.platformView != nullptr);
  std::shared_ptr<flutter::IOSContext> engine_context = engine.platformView->GetIosContext();
  std::shared_ptr<flutter::IOSContext> spawn_context = spawn.platformView->GetIosContext();
  XCTAssertEqual(engine_context, spawn_context);
}

- (void)testEnableSemanticsWhenFlutterViewAccessibilityDidCall {
  FlutterEngineSpy* engine = [[FlutterEngineSpy alloc] initWithName:@"foobar"];
  engine.ensureSemanticsEnabledCalled = NO;
  [engine flutterViewAccessibilityDidCall];
  XCTAssertTrue(engine.ensureSemanticsEnabledCalled);
}

- (void)testCanMergePlatformAndUIThread {
#if defined(TARGET_IPHONE_SIMULATOR) && TARGET_IPHONE_SIMULATOR
  auto settings = FLTDefaultSettingsForBundle();
  FlutterDartProject* project = [[FlutterDartProject alloc] initWithSettings:settings];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  [engine run];

  XCTAssertEqual(engine.shell.GetTaskRunners().GetUITaskRunner(),
                 engine.shell.GetTaskRunners().GetPlatformTaskRunner());
#endif  // defined(TARGET_IPHONE_SIMULATOR) && TARGET_IPHONE_SIMULATOR
}

- (void)testCanUnMergePlatformAndUIThread {
#if defined(TARGET_IPHONE_SIMULATOR) && TARGET_IPHONE_SIMULATOR
  auto settings = FLTDefaultSettingsForBundle();
  settings.merged_platform_ui_thread = false;
  FlutterDartProject* project = [[FlutterDartProject alloc] initWithSettings:settings];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
  [engine run];

  XCTAssertNotEqual(engine.shell.GetTaskRunners().GetUITaskRunner(),
                    engine.shell.GetTaskRunners().GetPlatformTaskRunner());
#endif  // defined(TARGET_IPHONE_SIMULATOR) && TARGET_IPHONE_SIMULATOR
}

@end
