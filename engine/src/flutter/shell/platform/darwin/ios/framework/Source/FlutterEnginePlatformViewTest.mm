// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#define FML_USED_ON_EMBEDDER

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#include "flutter/fml/message_loop.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Test.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"

FLUTTER_ASSERT_ARC

namespace flutter {
namespace {

class FakeDelegate : public PlatformView::Delegate {
 public:
  void OnPlatformViewCreated(std::unique_ptr<Surface> surface) override {}
  void OnPlatformViewDestroyed() override {}
  void OnPlatformViewScheduleFrame() override {}
  void OnPlatformViewAddView(int64_t view_id,
                             const ViewportMetrics& viewport_metrics,
                             AddViewCallback callback) override {}
  void OnPlatformViewRemoveView(int64_t view_id, RemoveViewCallback callback) override {}
  void OnPlatformViewSetNextFrameCallback(const fml::closure& closure) override {}
  void OnPlatformViewSetViewportMetrics(int64_t view_id, const ViewportMetrics& metrics) override {}
  const flutter::Settings& OnPlatformViewGetSettings() const override { return settings_; }
  void OnPlatformViewDispatchPlatformMessage(std::unique_ptr<PlatformMessage> message) override {}
  void OnPlatformViewDispatchPointerDataPacket(std::unique_ptr<PointerDataPacket> packet) override {
  }
  void OnPlatformViewDispatchSemanticsAction(int32_t id,
                                             SemanticsAction action,
                                             fml::MallocMapping args) override {}
  void OnPlatformViewSetSemanticsEnabled(bool enabled) override {}
  void OnPlatformViewSetAccessibilityFeatures(int32_t flags) override {}
  void OnPlatformViewRegisterTexture(std::shared_ptr<Texture> texture) override {}
  void OnPlatformViewUnregisterTexture(int64_t texture_id) override {}
  void OnPlatformViewMarkTextureFrameAvailable(int64_t texture_id) override {}

  void LoadDartDeferredLibrary(intptr_t loading_unit_id,
                               std::unique_ptr<const fml::Mapping> snapshot_data,
                               std::unique_ptr<const fml::Mapping> snapshot_instructions) override {
  }
  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string error_message,
                                    bool transient) override {}
  void UpdateAssetResolverByType(std::unique_ptr<AssetResolver> updated_asset_resolver,
                                 AssetResolver::AssetResolverType type) override {}

  flutter::Settings settings_;
};

}  // namespace
}  // namespace flutter

@interface FlutterEnginePlatformViewTest : XCTestCase
@end

@implementation FlutterEnginePlatformViewTest
std::unique_ptr<flutter::PlatformViewIOS> platform_view;
std::unique_ptr<fml::WeakPtrFactory<flutter::PlatformView>> weak_factory;
flutter::FakeDelegate fake_delegate;

- (void)setUp {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto thread_task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  auto sync_switch = std::make_shared<fml::SyncSwitch>();
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/fake_delegate,
      /*rendering_api=*/fake_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/sync_switch);
  weak_factory = std::make_unique<fml::WeakPtrFactory<flutter::PlatformView>>(platform_view.get());
}

- (void)tearDown {
  weak_factory.reset();
  platform_view.reset();
}

- (fml::WeakPtr<flutter::PlatformView>)platformViewReplacement {
  return weak_factory->GetWeakPtr();
}

- (void)testCallsNotifyLowMemory {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"tester"];
  XCTAssertNotNil(engine);
  id mockEngine = OCMPartialMock(engine);
  OCMStub([mockEngine notifyLowMemory]);
  OCMStub([mockEngine platformView]).andReturn(platform_view.get());

  [engine setViewController:nil];
  OCMVerify([mockEngine notifyLowMemory]);
  OCMReject([mockEngine notifyLowMemory]);

  XCTNSNotificationExpectation* memoryExpectation = [[XCTNSNotificationExpectation alloc]
      initWithName:UIApplicationDidReceiveMemoryWarningNotification];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationDidReceiveMemoryWarningNotification
                    object:nil];
  [self waitForExpectations:@[ memoryExpectation ] timeout:5.0];
  OCMVerify([mockEngine notifyLowMemory]);
  OCMReject([mockEngine notifyLowMemory]);

  XCTNSNotificationExpectation* backgroundExpectation = [[XCTNSNotificationExpectation alloc]
      initWithName:UIApplicationDidEnterBackgroundNotification];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationDidEnterBackgroundNotification
                    object:nil];
  [self waitForExpectations:@[ backgroundExpectation ] timeout:5.0];

  OCMVerify([mockEngine notifyLowMemory]);
}

@end
