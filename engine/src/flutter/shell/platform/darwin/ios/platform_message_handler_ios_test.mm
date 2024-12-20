// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/ios/platform_message_handler_ios.h"

#import "flutter/common/task_runners.h"
#import "flutter/fml/message_loop.h"
#import "flutter/fml/thread.h"
#import "flutter/lib/ui/window/platform_message.h"
#import "flutter/lib/ui/window/platform_message_response.h"
#import "flutter/shell/common/thread_host.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

namespace {
using namespace flutter;
fml::RefPtr<fml::TaskRunner> CreateNewThread(const std::string& name) {
  auto thread = std::make_unique<fml::Thread>(name);
  auto runner = thread->GetTaskRunner();
  return runner;
}

fml::RefPtr<fml::TaskRunner> GetCurrentTaskRunner() {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  return fml::MessageLoop::GetCurrent().GetTaskRunner();
}

class MockPlatformMessageResponse : public PlatformMessageResponse {
 public:
  static fml::RefPtr<MockPlatformMessageResponse> Create() {
    return fml::AdoptRef(new MockPlatformMessageResponse());
  }
  void Complete(std::unique_ptr<fml::Mapping> data) override { is_complete_ = true; }
  void CompleteEmpty() override { is_complete_ = true; }
};
}  // namespace

@interface PlatformMessageHandlerIosTest : XCTestCase
@end

@implementation PlatformMessageHandlerIosTest
- (void)testCreate {
  TaskRunners task_runners("test", GetCurrentTaskRunner(), CreateNewThread("raster"),
                           CreateNewThread("ui"), CreateNewThread("io"));
  auto handler = std::make_unique<PlatformMessageHandlerIos>(task_runners.GetPlatformTaskRunner());
  XCTAssertTrue(handler);
}

- (void)testSetAndCallHandler {
  ThreadHost thread_host("io.flutter.test." + std::string(self.name.UTF8String),
                         ThreadHost::Type::kRaster | ThreadHost::Type::kIo | ThreadHost::Type::kUi);
  TaskRunners task_runners(
      "test", GetCurrentTaskRunner(), thread_host.raster_thread->GetTaskRunner(),
      thread_host.ui_thread->GetTaskRunner(), thread_host.io_thread->GetTaskRunner());

  auto handler = std::make_unique<PlatformMessageHandlerIos>(task_runners.GetPlatformTaskRunner());
  std::string channel = "foo";
  XCTestExpectation* didCallReply = [self expectationWithDescription:@"didCallReply"];
  handler->SetMessageHandler(
      channel,
      ^(NSData* _Nullable data, FlutterBinaryReply _Nonnull reply) {
        reply(nil);
        [didCallReply fulfill];
      },
      nil);
  auto response = MockPlatformMessageResponse::Create();
  task_runners.GetUITaskRunner()->PostTask([channel, response, &handler] {
    auto platform_message = std::make_unique<flutter::PlatformMessage>(channel, response);
    handler->HandlePlatformMessage(std::move(platform_message));
  });
  [self waitForExpectationsWithTimeout:1.0 handler:nil];
  XCTAssertTrue(response->is_complete());
}

- (void)testSetClearAndCallHandler {
  ThreadHost thread_host("io.flutter.test." + std::string(self.name.UTF8String),
                         ThreadHost::Type::kRaster | ThreadHost::Type::kIo | ThreadHost::Type::kUi);
  TaskRunners task_runners(
      "test", GetCurrentTaskRunner(), thread_host.raster_thread->GetTaskRunner(),
      thread_host.ui_thread->GetTaskRunner(), thread_host.io_thread->GetTaskRunner());

  auto handler = std::make_unique<PlatformMessageHandlerIos>(task_runners.GetPlatformTaskRunner());
  std::string channel = "foo";
  XCTestExpectation* didCallMessage = [self expectationWithDescription:@"didCallMessage"];
  handler->SetMessageHandler(
      channel,
      ^(NSData* _Nullable data, FlutterBinaryReply _Nonnull reply) {
        XCTFail(@"This shouldn't be called");
        reply(nil);
      },
      nil);
  handler->SetMessageHandler(channel, nil, nil);
  auto response = MockPlatformMessageResponse::Create();
  task_runners.GetUITaskRunner()->PostTask([channel, response, &handler, &didCallMessage] {
    auto platform_message = std::make_unique<flutter::PlatformMessage>(channel, response);
    handler->HandlePlatformMessage(std::move(platform_message));
    [didCallMessage fulfill];
  });
  [self waitForExpectationsWithTimeout:1.0 handler:nil];
  XCTAssertTrue(response->is_complete());
}

- (void)testSetAndCallHandlerTaskQueue {
  ThreadHost thread_host("io.flutter.test." + std::string(self.name.UTF8String),
                         ThreadHost::Type::kRaster | ThreadHost::Type::kIo | ThreadHost::Type::kUi);
  TaskRunners task_runners(
      "test", GetCurrentTaskRunner(), thread_host.raster_thread->GetTaskRunner(),
      thread_host.ui_thread->GetTaskRunner(), thread_host.io_thread->GetTaskRunner());

  auto handler = std::make_unique<PlatformMessageHandlerIos>(task_runners.GetPlatformTaskRunner());
  std::string channel = "foo";
  XCTestExpectation* didCallReply = [self expectationWithDescription:@"didCallReply"];
  NSObject<FlutterTaskQueue>* taskQueue = PlatformMessageHandlerIos::MakeBackgroundTaskQueue();
  handler->SetMessageHandler(
      channel,
      ^(NSData* _Nullable data, FlutterBinaryReply _Nonnull reply) {
        XCTAssertFalse([NSThread isMainThread]);
        reply(nil);
        [didCallReply fulfill];
      },
      taskQueue);
  auto response = MockPlatformMessageResponse::Create();
  task_runners.GetUITaskRunner()->PostTask([channel, response, &handler] {
    auto platform_message = std::make_unique<flutter::PlatformMessage>(channel, response);
    handler->HandlePlatformMessage(std::move(platform_message));
  });
  [self waitForExpectationsWithTimeout:1.0 handler:nil];
  XCTAssertTrue(response->is_complete());
}
@end
