// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/platform_view_embedder.h"

#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <cstring>

namespace flutter {
namespace testing {
namespace {
class MockDelegate : public PlatformView::Delegate {
  MOCK_METHOD(void,
              OnPlatformViewCreated,
              (std::unique_ptr<Surface>),
              (override));
  MOCK_METHOD(void, OnPlatformViewDestroyed, (), (override));
  MOCK_METHOD(void, OnPlatformViewScheduleFrame, (), (override));
  MOCK_METHOD(void,
              OnPlatformViewAddView,
              (int64_t view_id,
               const ViewportMetrics& viewport_metrics,
               AddViewCallback callback),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewRemoveView,
              (int64_t view_id, RemoveViewCallback callback),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewSetNextFrameCallback,
              (const fml::closure& closure),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewSetViewportMetrics,
              (int64_t view_id, const ViewportMetrics& metrics),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewDispatchPlatformMessage,
              (std::unique_ptr<PlatformMessage> message),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewDispatchPointerDataPacket,
              (std::unique_ptr<PointerDataPacket> packet),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewDispatchSemanticsAction,
              (int32_t id, SemanticsAction action, fml::MallocMapping args),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewSetSemanticsEnabled,
              (bool enabled),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewSetAccessibilityFeatures,
              (int32_t flags),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewRegisterTexture,
              (std::shared_ptr<Texture> texture),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewUnregisterTexture,
              (int64_t texture_id),
              (override));
  MOCK_METHOD(void,
              OnPlatformViewMarkTextureFrameAvailable,
              (int64_t texture_id),
              (override));
  MOCK_METHOD(void,
              LoadDartDeferredLibrary,
              (intptr_t loading_unit_id,
               std::unique_ptr<const fml::Mapping> snapshot_data,
               std::unique_ptr<const fml::Mapping> snapshot_instructions),
              (override));
  MOCK_METHOD(void,
              LoadDartDeferredLibraryError,
              (intptr_t loading_unit_id,
               const std::string error_message,
               bool transient),
              (override));
  MOCK_METHOD(void,
              UpdateAssetResolverByType,
              (std::unique_ptr<AssetResolver> updated_asset_resolver,
               AssetResolver::AssetResolverType type),
              (override));
  MOCK_METHOD(const Settings&,
              OnPlatformViewGetSettings,
              (),
              (const, override));
};

class MockResponse : public PlatformMessageResponse {
 public:
  MOCK_METHOD(void, Complete, (std::unique_ptr<fml::Mapping> data), (override));
  MOCK_METHOD(void, CompleteEmpty, (), (override));
};
}  // namespace

TEST(PlatformViewEmbedderTest, HasPlatformMessageHandler) {
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  flutter::TaskRunners task_runners = flutter::TaskRunners(
      "HasPlatformMessageHandler", thread_host.platform_thread->GetTaskRunner(),
      nullptr, nullptr, nullptr);
  fml::AutoResetWaitableEvent latch;
  task_runners.GetPlatformTaskRunner()->PostTask([&latch, task_runners] {
    MockDelegate delegate;
    EmbedderSurfaceSoftware::SoftwareDispatchTable software_dispatch_table;
    PlatformViewEmbedder::PlatformDispatchTable platform_dispatch_table;
    std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder;
    auto embedder = std::make_unique<PlatformViewEmbedder>(
        delegate, task_runners, software_dispatch_table,
        platform_dispatch_table, external_view_embedder);

    ASSERT_TRUE(embedder->GetPlatformMessageHandler());
    latch.Signal();
  });
  latch.Wait();
}

TEST(PlatformViewEmbedderTest, Dispatches) {
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  flutter::TaskRunners task_runners = flutter::TaskRunners(
      "HasPlatformMessageHandler", thread_host.platform_thread->GetTaskRunner(),
      nullptr, nullptr, nullptr);
  bool did_call = false;
  std::unique_ptr<PlatformViewEmbedder> embedder;
  {
    fml::AutoResetWaitableEvent latch;
    task_runners.GetPlatformTaskRunner()->PostTask([&latch, task_runners,
                                                    &did_call, &embedder] {
      MockDelegate delegate;
      EmbedderSurfaceSoftware::SoftwareDispatchTable software_dispatch_table;
      PlatformViewEmbedder::PlatformDispatchTable platform_dispatch_table;
      platform_dispatch_table.platform_message_response_callback =
          [&did_call](std::unique_ptr<PlatformMessage> message) {
            did_call = true;
          };
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder;
      embedder = std::make_unique<PlatformViewEmbedder>(
          delegate, task_runners, software_dispatch_table,
          platform_dispatch_table, external_view_embedder);
      auto platform_message_handler = embedder->GetPlatformMessageHandler();
      fml::RefPtr<PlatformMessageResponse> response =
          fml::MakeRefCounted<MockResponse>();
      std::unique_ptr<PlatformMessage> message =
          std::make_unique<PlatformMessage>("foo", response);
      platform_message_handler->HandlePlatformMessage(std::move(message));
      latch.Signal();
    });
    latch.Wait();
  }
  {
    fml::AutoResetWaitableEvent latch;
    thread_host.platform_thread->GetTaskRunner()->PostTask([&latch, &embedder] {
      embedder.reset();
      latch.Signal();
    });
    latch.Wait();
  }

  EXPECT_TRUE(did_call);
}

TEST(PlatformViewEmbedderTest, DeletionDisabledDispatch) {
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  flutter::TaskRunners task_runners = flutter::TaskRunners(
      "HasPlatformMessageHandler", thread_host.platform_thread->GetTaskRunner(),
      nullptr, nullptr, nullptr);
  bool did_call = false;
  {
    fml::AutoResetWaitableEvent latch;
    task_runners.GetPlatformTaskRunner()->PostTask([&latch, task_runners,
                                                    &did_call] {
      MockDelegate delegate;
      EmbedderSurfaceSoftware::SoftwareDispatchTable software_dispatch_table;
      PlatformViewEmbedder::PlatformDispatchTable platform_dispatch_table;
      platform_dispatch_table.platform_message_response_callback =
          [&did_call](std::unique_ptr<PlatformMessage> message) {
            did_call = true;
          };
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder;
      auto embedder = std::make_unique<PlatformViewEmbedder>(
          delegate, task_runners, software_dispatch_table,
          platform_dispatch_table, external_view_embedder);
      auto platform_message_handler = embedder->GetPlatformMessageHandler();
      fml::RefPtr<PlatformMessageResponse> response =
          fml::MakeRefCounted<MockResponse>();
      std::unique_ptr<PlatformMessage> message =
          std::make_unique<PlatformMessage>("foo", response);
      platform_message_handler->HandlePlatformMessage(std::move(message));
      embedder.reset();
      latch.Signal();
    });
    latch.Wait();
  }
  {
    fml::AutoResetWaitableEvent latch;
    thread_host.platform_thread->GetTaskRunner()->PostTask(
        [&latch] { latch.Signal(); });
    latch.Wait();
  }

  EXPECT_FALSE(did_call);
}

}  // namespace testing
}  // namespace flutter
