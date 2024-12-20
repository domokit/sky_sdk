// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <string>
#include <utility>
#include <vector>

#include "embedder.h"
#include "embedder_engine.h"
#include "flutter/common/constants.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/fml/file.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/thread.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_config_builder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer_software.h"
#include "flutter/shell/platform/embedder/tests/embedder_unittests_util.h"
#include "flutter/testing/assertions_skia.h"
#include "flutter/testing/testing.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/tonic/converter/dart_converter.h"

#if defined(FML_OS_MACOSX)
#include <pthread.h>
#endif

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace {

static uint64_t NanosFromEpoch(int millis_from_now) {
  const auto now = fml::TimePoint::Now();
  const auto delta = fml::TimeDelta::FromMilliseconds(millis_from_now);
  return (now + delta).ToEpochDelta().ToNanoseconds();
}

}  // namespace

namespace flutter {
namespace testing {

using EmbedderTest = testing::EmbedderTest;

TEST(EmbedderTestNoFixture, MustNotRunWithInvalidArgs) {
  EmbedderTestContextSoftware context;
  EmbedderConfigBuilder builder(
      context, EmbedderConfigBuilder::InitializationPreference::kNoInitialize);
  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, CanLaunchAndShutdownWithValidProjectArgs) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  fml::AutoResetWaitableEvent latch;
  context.AddIsolateCreateCallback([&latch]() { latch.Signal(); });
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  // Wait for the root isolate to launch.
  latch.Wait();
  engine.reset();
}

// TODO(41999): Disabled because flaky.
TEST_F(EmbedderTest, DISABLED_CanLaunchAndShutdownMultipleTimes) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  for (size_t i = 0; i < 3; ++i) {
    auto engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
    FML_LOG(INFO) << "Engine launch count: " << i + 1;
  }
}

TEST_F(EmbedderTest, CanInvokeCustomEntrypoint) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  static fml::AutoResetWaitableEvent latch;
  Dart_NativeFunction entrypoint = [](Dart_NativeArguments args) {
    latch.Signal();
  };
  context.AddNativeCallback("SayHiFromCustomEntrypoint", entrypoint);
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("customEntrypoint");
  auto engine = builder.LaunchEngine();
  latch.Wait();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, CanInvokeCustomEntrypointMacro) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  fml::AutoResetWaitableEvent latch1;
  fml::AutoResetWaitableEvent latch2;
  fml::AutoResetWaitableEvent latch3;

  // Can be defined separately.
  auto entry1 = [&latch1](Dart_NativeArguments args) {
    FML_LOG(INFO) << "In Callback 1";
    latch1.Signal();
  };
  auto native_entry1 = CREATE_NATIVE_ENTRY(entry1);
  context.AddNativeCallback("SayHiFromCustomEntrypoint1", native_entry1);

  // Can be wrapped in the args.
  auto entry2 = [&latch2](Dart_NativeArguments args) {
    FML_LOG(INFO) << "In Callback 2";
    latch2.Signal();
  };
  context.AddNativeCallback("SayHiFromCustomEntrypoint2",
                            CREATE_NATIVE_ENTRY(entry2));

  // Everything can be inline.
  context.AddNativeCallback(
      "SayHiFromCustomEntrypoint3",
      CREATE_NATIVE_ENTRY([&latch3](Dart_NativeArguments args) {
        FML_LOG(INFO) << "In Callback 3";
        latch3.Signal();
      }));

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("customEntrypoint1");
  auto engine = builder.LaunchEngine();
  latch1.Wait();
  latch2.Wait();
  latch3.Wait();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, CanTerminateCleanly) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("terminateExitCodeHandler");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, ExecutableNameNotNull) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  // Supply a callback to Dart for the test fixture to pass Platform.executable
  // back to us.
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "NotifyStringValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        const auto dart_string = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        EXPECT_EQ("/path/to/binary", dart_string);
        latch.Signal();
      }));

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("executableNameNotNull");
  builder.SetExecutableName("/path/to/binary");
  auto engine = builder.LaunchEngine();
  latch.Wait();
}

TEST_F(EmbedderTest, ImplicitViewNotNull) {
  // TODO(loicsharma): Update this test when embedders can opt-out
  // of the implicit view.
  // See: https://github.com/flutter/flutter/issues/120306
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  bool implicitViewNotNull = false;
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "NotifyBoolValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        implicitViewNotNull = tonic::DartConverter<bool>::FromDart(
            Dart_GetNativeArgument(args, 0));
        latch.Signal();
      }));

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("implicitViewNotNull");
  auto engine = builder.LaunchEngine();
  latch.Wait();

  EXPECT_TRUE(implicitViewNotNull);
}

std::atomic_size_t EmbedderTestTaskRunner::sEmbedderTaskRunnerIdentifiers = {};

TEST_F(EmbedderTest, CanSpecifyCustomPlatformTaskRunner) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  fml::AutoResetWaitableEvent latch;

  // Run the test on its own thread with a message loop so that it can safely
  // pump its event loop while we wait for all the conditions to be checked.
  auto platform_task_runner = CreateNewThread("test_platform_thread");
  static std::mutex engine_mutex;
  static bool signaled_once = false;
  static std::atomic<bool> destruction_callback_called = false;
  UniqueEngine engine;

  EmbedderTestTaskRunner test_task_runner(
      platform_task_runner, [&](FlutterTask task) {
        std::scoped_lock lock(engine_mutex);
        if (!engine.is_valid()) {
          return;
        }
        // There may be multiple tasks posted but we only need to check
        // assertions once.
        if (signaled_once) {
          FlutterEngineRunTask(engine.get(), &task);
          return;
        }

        signaled_once = true;
        ASSERT_TRUE(engine.is_valid());
        ASSERT_EQ(FlutterEngineRunTask(engine.get(), &task), kSuccess);
        latch.Signal();
      });
  test_task_runner.SetDestructionCallback(
      [](void* user_data) { destruction_callback_called = true; });

  platform_task_runner->PostTask([&]() {
    EmbedderConfigBuilder builder(context);
    const auto task_runner_description =
        test_task_runner.GetFlutterTaskRunnerDescription();
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetPlatformTaskRunner(&task_runner_description);
    builder.SetDartEntrypoint("invokePlatformTaskRunner");
    std::scoped_lock lock(engine_mutex);
    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
  });

  // Signaled when all the assertions are checked.
  latch.Wait();
  ASSERT_TRUE(engine.is_valid());

  // Since the engine was started on its own thread, it must be killed there as
  // well.
  fml::AutoResetWaitableEvent kill_latch;
  platform_task_runner->PostTask(fml::MakeCopyable([&]() mutable {
    std::scoped_lock lock(engine_mutex);
    engine.reset();

    // There may still be pending tasks on the platform thread that were queued
    // by the test_task_runner.  Signal the latch after these tasks have been
    // consumed.
    platform_task_runner->PostTask([&kill_latch] { kill_latch.Signal(); });
  }));
  kill_latch.Wait();

  ASSERT_TRUE(signaled_once);
  signaled_once = false;

  ASSERT_TRUE(destruction_callback_called);
  destruction_callback_called = false;
}

TEST(EmbedderTestNoFixture, CanGetCurrentTimeInNanoseconds) {
  auto point1 = fml::TimePoint::FromEpochDelta(
      fml::TimeDelta::FromNanoseconds(FlutterEngineGetCurrentTime()));
  auto point2 = fml::TimePoint::Now();

  ASSERT_LT((point2 - point1), fml::TimeDelta::FromMilliseconds(1));
}

TEST_F(EmbedderTest, CanReloadSystemFonts) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  auto result = FlutterEngineReloadSystemFonts(engine.get());
  ASSERT_EQ(result, kSuccess);
}

TEST_F(EmbedderTest, IsolateServiceIdSent) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  fml::AutoResetWaitableEvent latch;

  fml::Thread thread;
  UniqueEngine engine;
  std::string isolate_message;

  thread.GetTaskRunner()->PostTask([&]() {
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("main");
    builder.SetPlatformMessageCallback(
        [&](const FlutterPlatformMessage* message) {
          if (strcmp(message->channel, "flutter/isolate") == 0) {
            isolate_message = {reinterpret_cast<const char*>(message->message),
                               message->message_size};
            latch.Signal();
          }
        });
    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
  });

  // Wait for the isolate ID message and check its format.
  latch.Wait();
  ASSERT_EQ(isolate_message.find("isolates/"), 0ul);

  // Since the engine was started on its own thread, it must be killed there as
  // well.
  fml::AutoResetWaitableEvent kill_latch;
  thread.GetTaskRunner()->PostTask(
      fml::MakeCopyable([&engine, &kill_latch]() mutable {
        engine.reset();
        kill_latch.Signal();
      }));
  kill_latch.Wait();
}

//------------------------------------------------------------------------------
/// Creates a platform message response callbacks, does NOT send them, and
/// immediately collects the same.
///
TEST_F(EmbedderTest, CanCreateAndCollectCallbacks) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("platform_messages_response");
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([](Dart_NativeArguments args) {}));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterPlatformMessageResponseHandle* response_handle = nullptr;
  auto callback = [](const uint8_t* data, size_t size,
                     void* user_data) -> void {};
  auto result = FlutterPlatformMessageCreateResponseHandle(
      engine.get(), callback, nullptr, &response_handle);
  ASSERT_EQ(result, kSuccess);
  ASSERT_NE(response_handle, nullptr);

  result = FlutterPlatformMessageReleaseResponseHandle(engine.get(),
                                                       response_handle);
  ASSERT_EQ(result, kSuccess);
}

//------------------------------------------------------------------------------
/// Sends platform messages to Dart code than simply echoes the contents of the
/// message back to the embedder. The embedder registers a native callback to
/// intercept that message.
///
TEST_F(EmbedderTest, PlatformMessagesCanReceiveResponse) {
  struct Captures {
    fml::AutoResetWaitableEvent latch;
    std::thread::id thread_id;
  };
  Captures captures;

  CreateNewThread()->PostTask([&]() {
    captures.thread_id = std::this_thread::get_id();
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("platform_messages_response");

    fml::AutoResetWaitableEvent ready;
    context.AddNativeCallback(
        "SignalNativeTest",
        CREATE_NATIVE_ENTRY(
            [&ready](Dart_NativeArguments args) { ready.Signal(); }));

    auto engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());

    static std::string kMessageData = "Hello from embedder.";

    FlutterPlatformMessageResponseHandle* response_handle = nullptr;
    auto callback = [](const uint8_t* data, size_t size,
                       void* user_data) -> void {
      ASSERT_EQ(size, kMessageData.size());
      ASSERT_EQ(strncmp(reinterpret_cast<const char*>(kMessageData.data()),
                        reinterpret_cast<const char*>(data), size),
                0);
      auto captures = reinterpret_cast<Captures*>(user_data);
      ASSERT_EQ(captures->thread_id, std::this_thread::get_id());
      captures->latch.Signal();
    };
    auto result = FlutterPlatformMessageCreateResponseHandle(
        engine.get(), callback, &captures, &response_handle);
    ASSERT_EQ(result, kSuccess);

    FlutterPlatformMessage message = {};
    message.struct_size = sizeof(FlutterPlatformMessage);
    message.channel = "test_channel";
    message.message = reinterpret_cast<const uint8_t*>(kMessageData.data());
    message.message_size = kMessageData.size();
    message.response_handle = response_handle;

    ready.Wait();
    result = FlutterEngineSendPlatformMessage(engine.get(), &message);
    ASSERT_EQ(result, kSuccess);

    result = FlutterPlatformMessageReleaseResponseHandle(engine.get(),
                                                         response_handle);
    ASSERT_EQ(result, kSuccess);
  });

  captures.latch.Wait();
}

//------------------------------------------------------------------------------
/// Tests that a platform message can be sent with no response handle. Instead
/// of the platform message integrity checked via a response handle, a native
/// callback with the response is invoked to assert integrity.
///
TEST_F(EmbedderTest, PlatformMessagesCanBeSentWithoutResponseHandles) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("platform_messages_no_response");

  const std::string message_data = "Hello but don't call me back.";

  fml::AutoResetWaitableEvent ready, message;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready](Dart_NativeArguments args) { ready.Signal(); }));
  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY(
          ([&message, &message_data](Dart_NativeArguments args) {
            auto received_message = tonic::DartConverter<std::string>::FromDart(
                Dart_GetNativeArgument(args, 0));
            ASSERT_EQ(received_message, message_data);
            message.Signal();
          })));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());
  ready.Wait();

  FlutterPlatformMessage platform_message = {};
  platform_message.struct_size = sizeof(FlutterPlatformMessage);
  platform_message.channel = "test_channel";
  platform_message.message =
      reinterpret_cast<const uint8_t*>(message_data.data());
  platform_message.message_size = message_data.size();
  platform_message.response_handle = nullptr;  // No response needed.

  auto result =
      FlutterEngineSendPlatformMessage(engine.get(), &platform_message);
  ASSERT_EQ(result, kSuccess);
  message.Wait();
}

//------------------------------------------------------------------------------
/// Tests that a null platform message can be sent.
///
TEST_F(EmbedderTest, NullPlatformMessagesCanBeSent) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("null_platform_messages");

  fml::AutoResetWaitableEvent ready, message;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready](Dart_NativeArguments args) { ready.Signal(); }));
  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY(([&message](Dart_NativeArguments args) {
        auto received_message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        ASSERT_EQ("true", received_message);
        message.Signal();
      })));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());
  ready.Wait();

  FlutterPlatformMessage platform_message = {};
  platform_message.struct_size = sizeof(FlutterPlatformMessage);
  platform_message.channel = "test_channel";
  platform_message.message = nullptr;
  platform_message.message_size = 0;
  platform_message.response_handle = nullptr;  // No response needed.

  auto result =
      FlutterEngineSendPlatformMessage(engine.get(), &platform_message);
  ASSERT_EQ(result, kSuccess);
  message.Wait();
}

//------------------------------------------------------------------------------
/// Tests that a null platform message cannot be send if the message_size
/// isn't equals to 0.
///
TEST_F(EmbedderTest, InvalidPlatformMessages) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterPlatformMessage platform_message = {};
  platform_message.struct_size = sizeof(FlutterPlatformMessage);
  platform_message.channel = "test_channel";
  platform_message.message = nullptr;
  platform_message.message_size = 1;
  platform_message.response_handle = nullptr;  // No response needed.

  auto result =
      FlutterEngineSendPlatformMessage(engine.get(), &platform_message);
  ASSERT_EQ(result, kInvalidArguments);
}

//------------------------------------------------------------------------------
/// Tests that setting a custom log callback works as expected and defaults to
/// using tag "flutter".
TEST_F(EmbedderTest, CanSetCustomLogMessageCallback) {
  fml::AutoResetWaitableEvent callback_latch;
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("custom_logger");
  builder.SetSurface(SkISize::Make(1, 1));
  context.SetLogMessageCallback(
      [&callback_latch](const char* tag, const char* message) {
        EXPECT_EQ(std::string(tag), "flutter");
        EXPECT_EQ(std::string(message), "hello world");
        callback_latch.Signal();
      });
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  callback_latch.Wait();
}

//------------------------------------------------------------------------------
/// Tests that setting a custom log tag works.
TEST_F(EmbedderTest, CanSetCustomLogTag) {
  fml::AutoResetWaitableEvent callback_latch;
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("custom_logger");
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetLogTag("butterfly");
  context.SetLogMessageCallback(
      [&callback_latch](const char* tag, const char* message) {
        EXPECT_EQ(std::string(tag), "butterfly");
        EXPECT_EQ(std::string(message), "hello world");
        callback_latch.Signal();
      });
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  callback_latch.Wait();
}

//------------------------------------------------------------------------------
/// Asserts behavior of FlutterProjectArgs::shutdown_dart_vm_when_done (which is
/// set to true by default in these unit-tests).
///
TEST_F(EmbedderTest, VMShutsDownWhenNoEnginesInProcess) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  const auto launch_count = DartVM::GetVMLaunchCount();

  {
    auto engine = builder.LaunchEngine();
    ASSERT_EQ(launch_count + 1u, DartVM::GetVMLaunchCount());
  }

  {
    auto engine = builder.LaunchEngine();
    ASSERT_EQ(launch_count + 2u, DartVM::GetVMLaunchCount());
  }
}

//------------------------------------------------------------------------------
///
TEST_F(EmbedderTest, DartEntrypointArgs) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.AddDartEntrypointArgument("foo");
  builder.AddDartEntrypointArgument("bar");
  builder.SetDartEntrypoint("dart_entrypoint_args");
  fml::AutoResetWaitableEvent callback_latch;
  std::vector<std::string> callback_args;
  auto nativeArgumentsCallback = [&callback_args,
                                  &callback_latch](Dart_NativeArguments args) {
    Dart_Handle exception = nullptr;
    callback_args =
        tonic::DartConverter<std::vector<std::string>>::FromArguments(
            args, 0, exception);
    callback_latch.Signal();
  };
  context.AddNativeCallback("NativeArgumentsCallback",
                            CREATE_NATIVE_ENTRY(nativeArgumentsCallback));
  auto engine = builder.LaunchEngine();
  callback_latch.Wait();
  ASSERT_EQ(callback_args[0], "foo");
  ASSERT_EQ(callback_args[1], "bar");
}

//------------------------------------------------------------------------------
/// These snapshots may be materialized from symbols and the size field may not
/// be relevant. Since this information is redundant, engine launch should not
/// be gated on a non-zero buffer size.
///
TEST_F(EmbedderTest, VMAndIsolateSnapshotSizesAreRedundantInAOTMode) {
  if (!DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // The fixture sets this up correctly. Intentionally mess up the args.
  builder.GetProjectArgs().vm_snapshot_data_size = 0;
  builder.GetProjectArgs().vm_snapshot_instructions_size = 0;
  builder.GetProjectArgs().isolate_snapshot_data_size = 0;
  builder.GetProjectArgs().isolate_snapshot_instructions_size = 0;

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, CanRenderImplicitView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("render_implicit_view");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::AutoResetWaitableEvent latch;

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(view_id, kFlutterImplicitViewId);
        latch.Signal();
      });

  auto engine = builder.LaunchEngine();

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

TEST_F(EmbedderTest, CanRenderImplicitViewUsingPresentLayersCallback) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor(/* avoid_backing_store_cache = */ false,
                        /* use_present_layers_callback = */ true);
  builder.SetDartEntrypoint("render_implicit_view");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::AutoResetWaitableEvent latch;

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(view_id, kFlutterImplicitViewId);
        latch.Signal();
      });

  auto engine = builder.LaunchEngine();

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom software
/// compositor.
///
// TODO(143940): Convert this test to use SkiaGold.
#if FML_OS_MACOSX && FML_ARCH_CPU_ARM64
TEST_F(EmbedderTest,
       DISABLED_CompositorMustBeAbleToRenderKnownSceneWithSoftwareCompositor) {
#else
TEST_F(EmbedderTest,
       CompositorMustBeAbleToRenderKnownSceneWithSoftwareCompositor) {
#endif  // FML_OS_MACOSX && FML_ARCH_CPU_ARM64

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_known_scene");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::CountDownLatch latch(5);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 5u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          backing_store.software.height = 600;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 1;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(20.0, 20.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          backing_store.software.height = 600;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(30, 30, 80, 180),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[2], layer);
        }

        // Layer 3
        {
          FlutterPlatformView platform_view = *layers[3]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 2;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(40.0, 40.0);

          ASSERT_EQ(*layers[3], layer);
        }

        // Layer 4
        {
          FlutterBackingStore backing_store = *layers[4]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          backing_store.software.height = 600;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(50, 50, 100, 200),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[4], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer, GrDirectContext*
          /* don't use because software compositor */) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(
            layer, nullptr /* null because software compositor */);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          case 2: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorMAGENTA);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("compositor_software.png", scene_image));

  // There should no present calls on the root surface.
  ASSERT_EQ(context.GetSurfacePresentCount(), 0u);
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom software
/// compositor, with a transparent overlay
///
TEST_F(EmbedderTest, NoLayerCreatedForTransparentOverlayOnTopOfPlatformLayer) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_transparent_overlay");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::CountDownLatch latch(4);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          backing_store.software.height = 600;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 1;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(20.0, 20.0);

          ASSERT_EQ(*layers[1], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer, GrDirectContext*
          /* don't use because software compositor */) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(
            layer, nullptr /* null because software compositor */);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  // TODO(https://github.com/flutter/flutter/issues/53784): enable this on all
  // platforms.
#if !defined(FML_OS_LINUX)
  GTEST_SKIP() << "Skipping golden tests on non-Linux OSes";
#endif  // FML_OS_LINUX
  ASSERT_TRUE(ImageMatchesFixture(
      "compositor_platform_layer_with_no_overlay.png", scene_image));

  // There should no present calls on the root surface.
  ASSERT_EQ(context.GetSurfacePresentCount(), 0u);
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom software
/// compositor, with a no overlay
///
TEST_F(EmbedderTest, NoLayerCreatedForNoOverlayOnTopOfPlatformLayer) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_no_overlay");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::CountDownLatch latch(4);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          backing_store.software.height = 600;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 1;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(20.0, 20.0);

          ASSERT_EQ(*layers[1], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer, GrDirectContext*
          /* don't use because software compositor */) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(
            layer, nullptr /* null because software compositor */);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  // TODO(https://github.com/flutter/flutter/issues/53784): enable this on all
  // platforms.
#if !defined(FML_OS_LINUX)
  GTEST_SKIP() << "Skipping golden tests on non-Linux OSes";
#endif  // FML_OS_LINUX
  ASSERT_TRUE(ImageMatchesFixture(
      "compositor_platform_layer_with_no_overlay.png", scene_image));

  // There should no present calls on the root surface.
  ASSERT_EQ(context.GetSurfacePresentCount(), 0u);
}

//------------------------------------------------------------------------------
/// Test that an engine can be initialized but not run.
///
TEST_F(EmbedderTest, CanCreateInitializedEngine) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.InitializeEngine();
  ASSERT_TRUE(engine.is_valid());
  engine.reset();
}

//------------------------------------------------------------------------------
/// Test that an initialized engine can be run exactly once.
///
TEST_F(EmbedderTest, CanRunInitializedEngine) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.InitializeEngine();
  ASSERT_TRUE(engine.is_valid());
  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kSuccess);
  // Cannot re-run an already running engine.
  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kInvalidArguments);
  engine.reset();
}

//------------------------------------------------------------------------------
/// Test that an engine can be deinitialized.
///
TEST_F(EmbedderTest, CanDeinitializeAnEngine) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.InitializeEngine();
  ASSERT_TRUE(engine.is_valid());
  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kSuccess);
  // Cannot re-run an already running engine.
  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kInvalidArguments);
  ASSERT_EQ(FlutterEngineDeinitialize(engine.get()), kSuccess);
  // It is ok to deinitialize an engine multiple times.
  ASSERT_EQ(FlutterEngineDeinitialize(engine.get()), kSuccess);

  // Sending events to a deinitialized engine fails.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kInvalidArguments);
  engine.reset();
}

//------------------------------------------------------------------------------
/// Test that a view can be added to a running engine.
///
TEST_F(EmbedderTest, CanAddView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_all_view_ids");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  std::string message;
  context.AddNativeCallback("SignalNativeMessage",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              message =
                                  tonic::DartConverter<std::string>::FromDart(
                                      Dart_GetNativeArgument(args, 0));
                              message_latch.Signal();
                            }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 123;

  FlutterAddViewInfo info = {};
  info.struct_size = sizeof(FlutterAddViewInfo);
  info.view_id = 123;
  info.view_metrics = &metrics;
  info.add_view_callback = [](const FlutterAddViewResult* result) {
    EXPECT_TRUE(result->added);
  };
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ("View IDs: [0, 123]", message);
}

//------------------------------------------------------------------------------
/// Test that adding a view schedules a frame.
///
TEST_F(EmbedderTest, AddViewSchedulesFrame) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("add_view_schedules_frame");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  fml::AutoResetWaitableEvent check_latch;
  context.AddNativeCallback(
      "SignalNativeCount",
      CREATE_NATIVE_ENTRY(
          [&check_latch](Dart_NativeArguments args) { check_latch.Signal(); }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Wait for the application to attach the listener.
  latch.Wait();

  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 123;

  FlutterAddViewInfo info = {};
  info.struct_size = sizeof(FlutterAddViewInfo);
  info.view_id = 123;
  info.view_metrics = &metrics;
  info.add_view_callback = [](const FlutterAddViewResult* result) {
    EXPECT_TRUE(result->added);
  };
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &info), kSuccess);

  check_latch.Wait();
}

//------------------------------------------------------------------------------
/// Test that a view that was added can be removed.
///
TEST_F(EmbedderTest, CanRemoveView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_all_view_ids");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  std::string message;
  context.AddNativeCallback("SignalNativeMessage",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              message =
                                  tonic::DartConverter<std::string>::FromDart(
                                      Dart_GetNativeArgument(args, 0));
                              message_latch.Signal();
                            }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  // Add view 123.
  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 123;

  FlutterAddViewInfo add_info = {};
  add_info.struct_size = sizeof(FlutterAddViewInfo);
  add_info.view_id = 123;
  add_info.view_metrics = &metrics;
  add_info.add_view_callback = [](const FlutterAddViewResult* result) {
    ASSERT_TRUE(result->added);
  };
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0, 123]");

  // Remove view 123.
  FlutterRemoveViewInfo remove_info = {};
  remove_info.struct_size = sizeof(FlutterAddViewInfo);
  remove_info.view_id = 123;
  remove_info.remove_view_callback = [](const FlutterRemoveViewResult* result) {
    EXPECT_TRUE(result->removed);
  };
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &remove_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0]");
}

//------------------------------------------------------------------------------
/// The implicit view is a special view that the engine and framework assume
/// can *always* be rendered to. Test that this view cannot be removed.
///
TEST_F(EmbedderTest, CannotRemoveImplicitView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterRemoveViewInfo info = {};
  info.struct_size = sizeof(FlutterRemoveViewInfo);
  info.view_id = kFlutterImplicitViewId;
  info.remove_view_callback = [](const FlutterRemoveViewResult* result) {
    FAIL();
  };
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &info), kInvalidArguments);
}

//------------------------------------------------------------------------------
/// Test that a view cannot be added if its ID already exists.
///
TEST_F(EmbedderTest, CannotAddDuplicateViews) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_all_view_ids");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  std::string message;
  context.AddNativeCallback("SignalNativeMessage",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              message =
                                  tonic::DartConverter<std::string>::FromDart(
                                      Dart_GetNativeArgument(args, 0));
                              message_latch.Signal();
                            }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  // Add view 123.
  struct Captures {
    std::atomic<int> count = 0;
    fml::AutoResetWaitableEvent failure_latch;
  };
  Captures captures;

  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 123;

  FlutterAddViewInfo add_info = {};
  add_info.struct_size = sizeof(FlutterAddViewInfo);
  add_info.view_id = 123;
  add_info.view_metrics = &metrics;
  add_info.user_data = &captures;
  add_info.add_view_callback = [](const FlutterAddViewResult* result) {
    auto captures = reinterpret_cast<Captures*>(result->user_data);

    int count = captures->count.fetch_add(1);

    if (count == 0) {
      ASSERT_TRUE(result->added);
    } else {
      EXPECT_FALSE(result->added);
      captures->failure_latch.Signal();
    }
  };
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0, 123]");
  ASSERT_FALSE(captures.failure_latch.IsSignaledForTest());

  // Add view 123 a second time.
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_info), kSuccess);
  captures.failure_latch.Wait();
  ASSERT_EQ(captures.count, 2);
  ASSERT_FALSE(message_latch.IsSignaledForTest());
}

//------------------------------------------------------------------------------
/// Test that a removed view's ID can be reused to add a new view.
///
TEST_F(EmbedderTest, CanReuseViewIds) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_all_view_ids");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  std::string message;
  context.AddNativeCallback("SignalNativeMessage",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              message =
                                  tonic::DartConverter<std::string>::FromDart(
                                      Dart_GetNativeArgument(args, 0));
                              message_latch.Signal();
                            }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  // Add view 123.
  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 123;

  FlutterAddViewInfo add_info = {};
  add_info.struct_size = sizeof(FlutterAddViewInfo);
  add_info.view_id = 123;
  add_info.view_metrics = &metrics;
  add_info.add_view_callback = [](const FlutterAddViewResult* result) {
    ASSERT_TRUE(result->added);
  };
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0, 123]");

  // Remove view 123.
  FlutterRemoveViewInfo remove_info = {};
  remove_info.struct_size = sizeof(FlutterAddViewInfo);
  remove_info.view_id = 123;
  remove_info.remove_view_callback = [](const FlutterRemoveViewResult* result) {
    ASSERT_TRUE(result->removed);
  };
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &remove_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0]");

  // Re-add view 123.
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_info), kSuccess);
  message_latch.Wait();
  ASSERT_EQ(message, "View IDs: [0, 123]");
}

//------------------------------------------------------------------------------
/// Test that attempting to remove a view that does not exist fails as expected.
///
TEST_F(EmbedderTest, CannotRemoveUnknownView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;
  FlutterRemoveViewInfo info = {};
  info.struct_size = sizeof(FlutterRemoveViewInfo);
  info.view_id = 123;
  info.user_data = &latch;
  info.remove_view_callback = [](const FlutterRemoveViewResult* result) {
    EXPECT_FALSE(result->removed);
    reinterpret_cast<fml::AutoResetWaitableEvent*>(result->user_data)->Signal();
  };
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &info), kSuccess);
  latch.Wait();
}

//------------------------------------------------------------------------------
/// View operations - adding, removing, sending window metrics - must execute in
/// order even though they are asynchronous. This is necessary to ensure the
/// embedder's and engine's states remain synchronized.
///
TEST_F(EmbedderTest, ViewOperationsOrdered) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_all_view_ids");

  fml::AutoResetWaitableEvent ready_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  std::atomic<int> message_count = 0;
  context.AddNativeCallback("SignalNativeMessage",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              message_count.fetch_add(1);
                            }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  // Enqueue multiple view operations at once:
  //
  // 1. Add view 123 - This must succeed.
  // 2. Add duplicate view 123 - This must fail asynchronously.
  // 3. Add second view 456 - This must succeed.
  // 4. Remove second view 456 - This must succeed.
  //
  // The engine must execute view operations asynchronously in serial order.
  // If step 2 succeeds instead of step 1, this indicates the engine did not
  // execute the view operations in the correct order. If step 4 fails,
  // this indicates the engine did not wait until the add second view completed.
  FlutterWindowMetricsEvent metrics123 = {};
  metrics123.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics123.width = 800;
  metrics123.height = 600;
  metrics123.pixel_ratio = 1.0;
  metrics123.view_id = 123;

  FlutterWindowMetricsEvent metrics456 = {};
  metrics456.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics456.width = 800;
  metrics456.height = 600;
  metrics456.pixel_ratio = 1.0;
  metrics456.view_id = 456;

  struct Captures {
    fml::AutoResetWaitableEvent add_first_view;
    fml::AutoResetWaitableEvent add_duplicate_view;
    fml::AutoResetWaitableEvent add_second_view;
    fml::AutoResetWaitableEvent remove_second_view;
  };
  Captures captures;

  // Add view 123.
  FlutterAddViewInfo add_view_info = {};
  add_view_info.struct_size = sizeof(FlutterAddViewInfo);
  add_view_info.view_id = 123;
  add_view_info.view_metrics = &metrics123;
  add_view_info.user_data = &captures;
  add_view_info.add_view_callback = [](const FlutterAddViewResult* result) {
    auto captures = reinterpret_cast<Captures*>(result->user_data);

    ASSERT_TRUE(result->added);
    ASSERT_FALSE(captures->add_first_view.IsSignaledForTest());
    ASSERT_FALSE(captures->add_duplicate_view.IsSignaledForTest());
    ASSERT_FALSE(captures->add_second_view.IsSignaledForTest());
    ASSERT_FALSE(captures->remove_second_view.IsSignaledForTest());

    captures->add_first_view.Signal();
  };

  // Add duplicate view 123.
  FlutterAddViewInfo add_duplicate_view_info = {};
  add_duplicate_view_info.struct_size = sizeof(FlutterAddViewInfo);
  add_duplicate_view_info.view_id = 123;
  add_duplicate_view_info.view_metrics = &metrics123;
  add_duplicate_view_info.user_data = &captures;
  add_duplicate_view_info.add_view_callback =
      [](const FlutterAddViewResult* result) {
        auto captures = reinterpret_cast<Captures*>(result->user_data);

        ASSERT_FALSE(result->added);
        ASSERT_TRUE(captures->add_first_view.IsSignaledForTest());
        ASSERT_FALSE(captures->add_duplicate_view.IsSignaledForTest());
        ASSERT_FALSE(captures->add_second_view.IsSignaledForTest());
        ASSERT_FALSE(captures->remove_second_view.IsSignaledForTest());

        captures->add_duplicate_view.Signal();
      };

  // Add view 456.
  FlutterAddViewInfo add_second_view_info = {};
  add_second_view_info.struct_size = sizeof(FlutterAddViewInfo);
  add_second_view_info.view_id = 456;
  add_second_view_info.view_metrics = &metrics456;
  add_second_view_info.user_data = &captures;
  add_second_view_info.add_view_callback =
      [](const FlutterAddViewResult* result) {
        auto captures = reinterpret_cast<Captures*>(result->user_data);

        ASSERT_TRUE(result->added);
        ASSERT_TRUE(captures->add_first_view.IsSignaledForTest());
        ASSERT_TRUE(captures->add_duplicate_view.IsSignaledForTest());
        ASSERT_FALSE(captures->add_second_view.IsSignaledForTest());
        ASSERT_FALSE(captures->remove_second_view.IsSignaledForTest());

        captures->add_second_view.Signal();
      };

  // Remove view 456.
  FlutterRemoveViewInfo remove_second_view_info = {};
  remove_second_view_info.struct_size = sizeof(FlutterRemoveViewInfo);
  remove_second_view_info.view_id = 456;
  remove_second_view_info.user_data = &captures;
  remove_second_view_info.remove_view_callback =
      [](const FlutterRemoveViewResult* result) {
        auto captures = reinterpret_cast<Captures*>(result->user_data);

        ASSERT_TRUE(result->removed);
        ASSERT_TRUE(captures->add_first_view.IsSignaledForTest());
        ASSERT_TRUE(captures->add_duplicate_view.IsSignaledForTest());
        ASSERT_TRUE(captures->add_second_view.IsSignaledForTest());
        ASSERT_FALSE(captures->remove_second_view.IsSignaledForTest());

        captures->remove_second_view.Signal();
      };

  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_view_info), kSuccess);
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_duplicate_view_info),
            kSuccess);
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_second_view_info),
            kSuccess);
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &remove_second_view_info),
            kSuccess);
  captures.remove_second_view.Wait();
  captures.add_second_view.Wait();
  captures.add_duplicate_view.Wait();
  captures.add_first_view.Wait();
  ASSERT_EQ(message_count, 3);
}

//------------------------------------------------------------------------------
/// Test the engine can present to multiple views.
///
TEST_F(EmbedderTest, CanRenderMultipleViews) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetCompositor();
  builder.SetDartEntrypoint("render_all_views");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::AutoResetWaitableEvent latch0, latch123;
  context.GetCompositor().SetPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        switch (view_id) {
          case 0:
            latch0.Signal();
            break;
          case 123:
            latch123.Signal();
            break;
          default:
            FML_UNREACHABLE();
        }
      },
      /* one_shot= */ false);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Give the implicit view a non-zero size so that it renders something.
  FlutterWindowMetricsEvent metrics0 = {};
  metrics0.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics0.width = 800;
  metrics0.height = 600;
  metrics0.pixel_ratio = 1.0;
  metrics0.view_id = 0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &metrics0),
            kSuccess);

  // Add view 123.
  FlutterWindowMetricsEvent metrics123 = {};
  metrics123.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics123.width = 800;
  metrics123.height = 600;
  metrics123.pixel_ratio = 1.0;
  metrics123.view_id = 123;

  FlutterAddViewInfo add_view_info = {};
  add_view_info.struct_size = sizeof(FlutterAddViewInfo);
  add_view_info.view_id = 123;
  add_view_info.view_metrics = &metrics123;
  add_view_info.add_view_callback = [](const FlutterAddViewResult* result) {
    ASSERT_TRUE(result->added);
  };

  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_view_info), kSuccess);

  latch0.Wait();
  latch123.Wait();
}

//------------------------------------------------------------------------------
/// Test that the backing store is created with the correct view ID, is used
/// for the correct view, and is cached according to their views.
///
/// The test involves two frames:
/// 1. The first frame renders the implicit view and the second view.
/// 2. The second frame renders the implicit view and the third view.
///
/// The test verifies that:
/// - Each backing store is created with a valid view ID.
/// - Each backing store is presented for the view that it was created for.
/// - Both frames render the expected sets of views.
/// - By the end of frame 1, only 2 backing stores were created.
/// - By the end of frame 2, only 3 backing stores were created. This ensures
/// that the backing store for the 2nd view is not reused for the 3rd view.
TEST_F(EmbedderTest, BackingStoresCorrespondToTheirViews) {
  constexpr FlutterViewId kSecondViewId = 123;
  constexpr FlutterViewId kThirdViewId = 456;
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("render_all_views");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();

  EmbedderTestBackingStoreProducerSoftware producer(
      context.GetCompositor().GetGrContext(),
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  // The variables needed by the callbacks of the compositor.
  struct CompositorUserData {
    EmbedderTestBackingStoreProducer* producer;
    // Each latch is signaled when its corresponding view is presented.
    fml::AutoResetWaitableEvent latch_implicit;
    fml::AutoResetWaitableEvent latch_second;
    fml::AutoResetWaitableEvent latch_third;
    // Whether the respective view should be rendered in the frame.
    bool second_expected;
    bool third_expected;
    // The total number of backing stores created to verify caching.
    int backing_stores_created;
  };
  CompositorUserData compositor_user_data{
      .producer = &producer,
      .backing_stores_created = 0,
  };

  builder.GetCompositor() = FlutterCompositor{
      .struct_size = sizeof(FlutterCompositor),
      .user_data = reinterpret_cast<void*>(&compositor_user_data),
      .create_backing_store_callback =
          [](const FlutterBackingStoreConfig* config,
             FlutterBackingStore* backing_store_out, void* user_data) {
            // Verify that the backing store comes with the correct view ID.
            EXPECT_TRUE(config->view_id == 0 ||
                        config->view_id == kSecondViewId ||
                        config->view_id == kThirdViewId);
            auto compositor_user_data =
                reinterpret_cast<CompositorUserData*>(user_data);
            compositor_user_data->backing_stores_created += 1;
            bool result = compositor_user_data->producer->Create(
                config, backing_store_out);
            // The created backing store has a user_data that records the view
            // that the store is created for.
            backing_store_out->user_data =
                reinterpret_cast<void*>(config->view_id);
            return result;
          },
      .collect_backing_store_callback = [](const FlutterBackingStore* renderer,
                                           void* user_data) { return true; },
      .present_layers_callback = nullptr,
      .avoid_backing_store_cache = false,
      .present_view_callback =
          [](const FlutterPresentViewInfo* info) {
            EXPECT_EQ(info->layers_count, 1u);
            // Verify that the given layer's backing store has the same view ID
            // as the target view.
            int64_t store_view_id = reinterpret_cast<int64_t>(
                info->layers[0]->backing_store->user_data);
            EXPECT_EQ(store_view_id, info->view_id);
            auto compositor_user_data =
                reinterpret_cast<CompositorUserData*>(info->user_data);
            // Verify that the respective views are rendered.
            switch (info->view_id) {
              case 0:
                compositor_user_data->latch_implicit.Signal();
                break;
              case kSecondViewId:
                EXPECT_TRUE(compositor_user_data->second_expected);
                compositor_user_data->latch_second.Signal();
                break;
              case kThirdViewId:
                EXPECT_TRUE(compositor_user_data->third_expected);
                compositor_user_data->latch_third.Signal();
                break;
              default:
                FML_UNREACHABLE();
            }
            return true;
          },
  };

  compositor_user_data.second_expected = true;
  compositor_user_data.third_expected = false;

  /*=== First frame ===*/

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Give the implicit view a non-zero size so that it renders something.
  FlutterWindowMetricsEvent metrics_implicit = {
      .struct_size = sizeof(FlutterWindowMetricsEvent),
      .width = 800,
      .height = 600,
      .pixel_ratio = 1.0,
      .view_id = 0,
  };
  ASSERT_EQ(
      FlutterEngineSendWindowMetricsEvent(engine.get(), &metrics_implicit),
      kSuccess);

  // Add the second view.
  FlutterWindowMetricsEvent metrics_add = {
      .struct_size = sizeof(FlutterWindowMetricsEvent),
      .width = 800,
      .height = 600,
      .pixel_ratio = 1.0,
      .view_id = kSecondViewId,
  };

  FlutterAddViewInfo add_view_info = {};
  add_view_info.struct_size = sizeof(FlutterAddViewInfo);
  add_view_info.view_id = kSecondViewId;
  add_view_info.view_metrics = &metrics_add;
  add_view_info.add_view_callback = [](const FlutterAddViewResult* result) {
    ASSERT_TRUE(result->added);
  };

  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_view_info), kSuccess);

  compositor_user_data.latch_implicit.Wait();
  compositor_user_data.latch_second.Wait();

  /*=== Second frame ===*/

  compositor_user_data.second_expected = false;
  compositor_user_data.third_expected = true;
  EXPECT_EQ(compositor_user_data.backing_stores_created, 2);

  // Remove the second view
  FlutterRemoveViewInfo remove_view_info = {};
  remove_view_info.struct_size = sizeof(FlutterRemoveViewInfo);
  remove_view_info.view_id = kSecondViewId;
  remove_view_info.remove_view_callback =
      [](const FlutterRemoveViewResult* result) {
        ASSERT_TRUE(result->removed);
      };
  ASSERT_EQ(FlutterEngineRemoveView(engine.get(), &remove_view_info), kSuccess);

  // Add the third view.
  add_view_info.view_id = kThirdViewId;
  metrics_add.view_id = kThirdViewId;
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &add_view_info), kSuccess);
  // Adding the view should have scheduled a frame.

  compositor_user_data.latch_implicit.Wait();
  compositor_user_data.latch_third.Wait();
  EXPECT_EQ(compositor_user_data.backing_stores_created, 3);
}

TEST_F(EmbedderTest, CanUpdateLocales) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("can_receive_locale_updates");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  fml::AutoResetWaitableEvent check_latch;
  context.AddNativeCallback(
      "SignalNativeCount",
      CREATE_NATIVE_ENTRY([&check_latch](Dart_NativeArguments args) {
        ASSERT_EQ(tonic::DartConverter<int>::FromDart(
                      Dart_GetNativeArgument(args, 0)),
                  2);
        check_latch.Signal();
      }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Wait for the application to attach the listener.
  latch.Wait();

  FlutterLocale locale1 = {};
  locale1.struct_size = sizeof(locale1);
  locale1.language_code = "";  // invalid
  locale1.country_code = "US";
  locale1.script_code = "";
  locale1.variant_code = nullptr;

  FlutterLocale locale2 = {};
  locale2.struct_size = sizeof(locale2);
  locale2.language_code = "zh";
  locale2.country_code = "CN";
  locale2.script_code = "Hans";
  locale2.variant_code = nullptr;

  std::vector<const FlutterLocale*> locales;
  locales.push_back(&locale1);
  locales.push_back(&locale2);

  ASSERT_EQ(
      FlutterEngineUpdateLocales(engine.get(), locales.data(), locales.size()),
      kInvalidArguments);

  // Fix the invalid code.
  locale1.language_code = "en";

  ASSERT_EQ(
      FlutterEngineUpdateLocales(engine.get(), locales.data(), locales.size()),
      kSuccess);

  check_latch.Wait();
}

TEST_F(EmbedderTest, LocalizationCallbacksCalled) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  fml::AutoResetWaitableEvent latch;
  context.AddIsolateCreateCallback([&latch]() { latch.Signal(); });
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  // Wait for the root isolate to launch.
  latch.Wait();

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  std::vector<std::string> supported_locales;
  supported_locales.push_back("es");
  supported_locales.push_back("MX");
  supported_locales.push_back("");
  auto result = shell.GetPlatformView()->ComputePlatformResolvedLocales(
      supported_locales);

  ASSERT_EQ((*result).size(), supported_locales.size());  // 3
  ASSERT_EQ((*result)[0], supported_locales[0]);
  ASSERT_EQ((*result)[1], supported_locales[1]);
  ASSERT_EQ((*result)[2], supported_locales[2]);

  engine.reset();
}

TEST_F(EmbedderTest, CanQueryDartAOTMode) {
  ASSERT_EQ(FlutterEngineRunsAOTCompiledDartCode(),
            flutter::DartVM::IsRunningPrecompiledCode());
}

TEST_F(EmbedderTest, VerifyB143464703WithSoftwareBackend) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1024, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("verify_b143464703");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  // setup the screenshot promise.
  auto rendered_scene = context.GetNextSceneImage();

  fml::CountDownLatch latch(1);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 1024, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(1024.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(1024.0, 540.0);
          layer.offset = FlutterPointMake(135.0, 60.0);

          ASSERT_EQ(*layers[1], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [](const FlutterLayer& layer,
         GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(
            layer, nullptr /* null because software compositor */);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 42: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  // wait for scene to be rendered.
  latch.Wait();

  // TODO(https://github.com/flutter/flutter/issues/53784): enable this on all
  // platforms.
#if !defined(FML_OS_LINUX)
  GTEST_SKIP() << "Skipping golden tests on non-Linux OSes";
#endif  // FML_OS_LINUX
  ASSERT_TRUE(
      ImageMatchesFixture("verifyb143464703_soft_noxform.png", rendered_scene));
}

TEST_F(EmbedderTest, CanSendLowMemoryNotification) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  // TODO(chinmaygarde): The shell ought to have a mechanism for notification
  // dispatch that engine subsystems can register handlers to. This would allow
  // the raster cache and the secondary context caches to respond to
  // notifications. Once that is in place, this test can be updated to actually
  // ensure that the dispatched message is visible to engine subsystems.
  ASSERT_EQ(FlutterEngineNotifyLowMemoryWarning(engine.get()), kSuccess);
}

TEST_F(EmbedderTest, CanPostTaskToAllNativeThreads) {
  UniqueEngine engine;
  size_t worker_count = 0;
  fml::AutoResetWaitableEvent sync_latch;

  // One of the threads that the callback will be posted to is the platform
  // thread. So we cannot wait for assertions to complete on the platform
  // thread. Create a new thread to manage the engine instance and wait for
  // assertions on the test thread.
  auto platform_task_runner = CreateNewThread("platform_thread");

  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));

    engine = builder.LaunchEngine();

    ASSERT_TRUE(engine.is_valid());

    worker_count = ToEmbedderEngine(engine.get())
                       ->GetShell()
                       .GetDartVM()
                       ->GetConcurrentMessageLoop()
                       ->GetWorkerCount();

    sync_latch.Signal();
  });

  sync_latch.Wait();

  const auto engine_threads_count = worker_count + 4u;

  struct Captures {
    // Waits the adequate number of callbacks to fire.
    fml::CountDownLatch latch;

    // This class will be accessed from multiple threads concurrently to track
    // thread specific information that is later checked. All updates to fields
    // in this struct must be made with this mutex acquired.

    std::mutex captures_mutex;
    // Ensures that the expect number of distinct threads were serviced.
    std::set<std::thread::id> thread_ids;

    size_t platform_threads_count = 0;
    size_t render_threads_count = 0;
    size_t ui_threads_count = 0;
    size_t worker_threads_count = 0;

    explicit Captures(size_t count) : latch(count) {}
  };

  Captures captures(engine_threads_count);

  platform_task_runner->PostTask([&]() {
    ASSERT_EQ(FlutterEnginePostCallbackOnAllNativeThreads(
                  engine.get(),
                  [](FlutterNativeThreadType type, void* baton) {
                    auto captures = reinterpret_cast<Captures*>(baton);
                    {
                      std::scoped_lock lock(captures->captures_mutex);
                      switch (type) {
                        case kFlutterNativeThreadTypeRender:
                          captures->render_threads_count++;
                          break;
                        case kFlutterNativeThreadTypeWorker:
                          captures->worker_threads_count++;
                          break;
                        case kFlutterNativeThreadTypeUI:
                          captures->ui_threads_count++;
                          break;
                        case kFlutterNativeThreadTypePlatform:
                          captures->platform_threads_count++;
                          break;
                      }
                      captures->thread_ids.insert(std::this_thread::get_id());
                    }
                    captures->latch.CountDown();
                  },
                  &captures),
              kSuccess);
  });

  captures.latch.Wait();
  ASSERT_EQ(captures.thread_ids.size(), engine_threads_count);
  ASSERT_EQ(captures.platform_threads_count, 1u);
  ASSERT_EQ(captures.render_threads_count, 1u);
  ASSERT_EQ(captures.ui_threads_count, 1u);
  ASSERT_EQ(captures.worker_threads_count, worker_count + 1u /* for IO */);
  EXPECT_GE(captures.worker_threads_count - 1, 2u);
  EXPECT_LE(captures.worker_threads_count - 1, 4u);

  platform_task_runner->PostTask([&]() {
    engine.reset();
    sync_latch.Signal();
  });
  sync_latch.Wait();

  // The engine should have already been destroyed on the platform task runner.
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, InvalidAOTDataSourcesMustReturnError) {
  if (!DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }
  FlutterEngineAOTDataSource data_in = {};
  FlutterEngineAOTData data_out = nullptr;

  // Null source specified.
  ASSERT_EQ(FlutterEngineCreateAOTData(nullptr, &data_out), kInvalidArguments);
  ASSERT_EQ(data_out, nullptr);

  // Null data_out specified.
  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, nullptr), kInvalidArguments);

  // Invalid FlutterEngineAOTDataSourceType type specified.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  data_in.type = static_cast<FlutterEngineAOTDataSourceType>(-1);
  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, &data_out), kInvalidArguments);
  ASSERT_EQ(data_out, nullptr);

  // Invalid ELF path specified.
  data_in.type = kFlutterEngineAOTDataSourceTypeElfPath;
  data_in.elf_path = nullptr;
  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, &data_out), kInvalidArguments);
  ASSERT_EQ(data_in.type, kFlutterEngineAOTDataSourceTypeElfPath);
  ASSERT_EQ(data_in.elf_path, nullptr);
  ASSERT_EQ(data_out, nullptr);

  // Invalid ELF path specified.
  data_in.elf_path = "";
  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, &data_out), kInvalidArguments);
  ASSERT_EQ(data_in.type, kFlutterEngineAOTDataSourceTypeElfPath);
  ASSERT_EQ(data_in.elf_path, "");
  ASSERT_EQ(data_out, nullptr);

  // Could not find VM snapshot data.
  data_in.elf_path = "/bin/true";
  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, &data_out), kInvalidArguments);
  ASSERT_EQ(data_in.type, kFlutterEngineAOTDataSourceTypeElfPath);
  ASSERT_EQ(data_in.elf_path, "/bin/true");
  ASSERT_EQ(data_out, nullptr);
}

TEST_F(EmbedderTest, MustNotRunWithMultipleAOTSources) {
  if (!DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(
      context,
      EmbedderConfigBuilder::InitializationPreference::kMultiAOTInitialize);

  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, CanCreateAndCollectAValidElfSource) {
  if (!DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }
  FlutterEngineAOTDataSource data_in = {};
  FlutterEngineAOTData data_out = nullptr;

  // Collecting a null object should be allowed
  ASSERT_EQ(FlutterEngineCollectAOTData(data_out), kSuccess);

  const auto elf_path =
      fml::paths::JoinPaths({GetFixturesPath(), kDefaultAOTAppELFFileName});

  data_in.type = kFlutterEngineAOTDataSourceTypeElfPath;
  data_in.elf_path = elf_path.c_str();

  ASSERT_EQ(FlutterEngineCreateAOTData(&data_in, &data_out), kSuccess);
  ASSERT_EQ(data_in.type, kFlutterEngineAOTDataSourceTypeElfPath);
  ASSERT_EQ(data_in.elf_path, elf_path.c_str());
  ASSERT_NE(data_out, nullptr);

  ASSERT_EQ(FlutterEngineCollectAOTData(data_out), kSuccess);
}

TEST_F(EmbedderTest, CanLaunchAndShutdownWithAValidElfSource) {
  if (!DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  fml::AutoResetWaitableEvent latch;
  context.AddIsolateCreateCallback([&latch]() { latch.Signal(); });

  EmbedderConfigBuilder builder(
      context,
      EmbedderConfigBuilder::InitializationPreference::kAOTDataInitialize);

  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Wait for the root isolate to launch.
  latch.Wait();
  engine.reset();
}

#if defined(__clang_analyzer__)
#define TEST_VM_SNAPSHOT_DATA nullptr
#define TEST_VM_SNAPSHOT_INSTRUCTIONS nullptr
#define TEST_ISOLATE_SNAPSHOT_DATA nullptr
#define TEST_ISOLATE_SNAPSHOT_INSTRUCTIONS nullptr
#endif

//------------------------------------------------------------------------------
/// PopulateJITSnapshotMappingCallbacks should successfully change the callbacks
/// of the snapshots in the engine's settings when JIT snapshots are explicitly
/// defined.
///
TEST_F(EmbedderTest, CanSuccessfullyPopulateSpecificJITSnapshotCallbacks) {
// TODO(#107263): Inconsistent snapshot paths in the Linux Fuchsia FEMU test.
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Inconsistent paths in Fuchsia.";
#else

  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // Construct the location of valid JIT snapshots.
  const std::string src_path = GetSourcePath();
  const std::string vm_snapshot_data =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_DATA});
  const std::string vm_snapshot_instructions =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_INSTRUCTIONS});
  const std::string isolate_snapshot_data =
      fml::paths::JoinPaths({src_path, TEST_ISOLATE_SNAPSHOT_DATA});
  const std::string isolate_snapshot_instructions =
      fml::paths::JoinPaths({src_path, TEST_ISOLATE_SNAPSHOT_INSTRUCTIONS});

  // Explicitly define the locations of the JIT snapshots
  builder.GetProjectArgs().vm_snapshot_data =
      reinterpret_cast<const uint8_t*>(vm_snapshot_data.c_str());
  builder.GetProjectArgs().vm_snapshot_instructions =
      reinterpret_cast<const uint8_t*>(vm_snapshot_instructions.c_str());
  builder.GetProjectArgs().isolate_snapshot_data =
      reinterpret_cast<const uint8_t*>(isolate_snapshot_data.c_str());
  builder.GetProjectArgs().isolate_snapshot_instructions =
      reinterpret_cast<const uint8_t*>(isolate_snapshot_instructions.c_str());

  auto engine = builder.LaunchEngine();

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  const Settings settings = shell.GetSettings();

  ASSERT_NE(settings.vm_snapshot_data(), nullptr);
  ASSERT_NE(settings.vm_snapshot_instr(), nullptr);
  ASSERT_NE(settings.isolate_snapshot_data(), nullptr);
  ASSERT_NE(settings.isolate_snapshot_instr(), nullptr);
  ASSERT_NE(settings.dart_library_sources_kernel(), nullptr);
#endif  // OS_FUCHSIA
}

//------------------------------------------------------------------------------
/// PopulateJITSnapshotMappingCallbacks should still be able to successfully
/// change the callbacks of the snapshots in the engine's settings when JIT
/// snapshots are explicitly defined. However, if those snapshot locations are
/// invalid, the callbacks should return a nullptr.
///
TEST_F(EmbedderTest, JITSnapshotCallbacksFailWithInvalidLocation) {
// TODO(#107263): Inconsistent snapshot paths in the Linux Fuchsia FEMU test.
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Inconsistent paths in Fuchsia.";
#else

  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // Explicitly define the locations of the invalid JIT snapshots
  builder.GetProjectArgs().vm_snapshot_data =
      reinterpret_cast<const uint8_t*>("invalid_vm_data");
  builder.GetProjectArgs().vm_snapshot_instructions =
      reinterpret_cast<const uint8_t*>("invalid_vm_instructions");
  builder.GetProjectArgs().isolate_snapshot_data =
      reinterpret_cast<const uint8_t*>("invalid_snapshot_data");
  builder.GetProjectArgs().isolate_snapshot_instructions =
      reinterpret_cast<const uint8_t*>("invalid_snapshot_instructions");

  auto engine = builder.LaunchEngine();

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  const Settings settings = shell.GetSettings();

  ASSERT_EQ(settings.vm_snapshot_data(), nullptr);
  ASSERT_EQ(settings.vm_snapshot_instr(), nullptr);
  ASSERT_EQ(settings.isolate_snapshot_data(), nullptr);
  ASSERT_EQ(settings.isolate_snapshot_instr(), nullptr);
#endif  // OS_FUCHSIA
}

//------------------------------------------------------------------------------
/// The embedder must be able to run explicitly specified snapshots in JIT mode
/// (i.e. when those are present in known locations).
///
TEST_F(EmbedderTest, CanLaunchEngineWithSpecifiedJITSnapshots) {
  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // Construct the location of valid JIT snapshots.
  const std::string src_path = GetSourcePath();
  const std::string vm_snapshot_data =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_DATA});
  const std::string vm_snapshot_instructions =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_INSTRUCTIONS});
  const std::string isolate_snapshot_data =
      fml::paths::JoinPaths({src_path, TEST_ISOLATE_SNAPSHOT_DATA});
  const std::string isolate_snapshot_instructions =
      fml::paths::JoinPaths({src_path, TEST_ISOLATE_SNAPSHOT_INSTRUCTIONS});

  // Explicitly define the locations of the JIT snapshots
  builder.GetProjectArgs().vm_snapshot_data =
      reinterpret_cast<const uint8_t*>(vm_snapshot_data.c_str());
  builder.GetProjectArgs().vm_snapshot_instructions =
      reinterpret_cast<const uint8_t*>(vm_snapshot_instructions.c_str());
  builder.GetProjectArgs().isolate_snapshot_data =
      reinterpret_cast<const uint8_t*>(isolate_snapshot_data.c_str());
  builder.GetProjectArgs().isolate_snapshot_instructions =
      reinterpret_cast<const uint8_t*>(isolate_snapshot_instructions.c_str());

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

//------------------------------------------------------------------------------
/// The embedder must be able to run in JIT mode when only some snapshots are
/// specified.
///
TEST_F(EmbedderTest, CanLaunchEngineWithSomeSpecifiedJITSnapshots) {
  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // Construct the location of valid JIT snapshots.
  const std::string src_path = GetSourcePath();
  const std::string vm_snapshot_data =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_DATA});
  const std::string vm_snapshot_instructions =
      fml::paths::JoinPaths({src_path, TEST_VM_SNAPSHOT_INSTRUCTIONS});

  // Explicitly define the locations of the JIT snapshots
  builder.GetProjectArgs().vm_snapshot_data =
      reinterpret_cast<const uint8_t*>(vm_snapshot_data.c_str());
  builder.GetProjectArgs().vm_snapshot_instructions =
      reinterpret_cast<const uint8_t*>(vm_snapshot_instructions.c_str());

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

//------------------------------------------------------------------------------
/// The embedder must be able to run in JIT mode even when the specfied
/// snapshots are invalid. It should be able to resolve them as it would when
/// the snapshots are not specified.
///
TEST_F(EmbedderTest, CanLaunchEngineWithInvalidJITSnapshots) {
  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  // Explicitly define the locations of the JIT snapshots
  builder.GetProjectArgs().isolate_snapshot_data =
      reinterpret_cast<const uint8_t*>("invalid_snapshot_data");
  builder.GetProjectArgs().isolate_snapshot_instructions =
      reinterpret_cast<const uint8_t*>("invalid_snapshot_instructions");

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kInvalidArguments);
}

//------------------------------------------------------------------------------
/// The embedder must be able to launch even when the snapshots are not
/// explicitly defined in JIT mode. It must be able to resolve those snapshots.
///
TEST_F(EmbedderTest, CanLaunchEngineWithUnspecifiedJITSnapshots) {
  // This test is only relevant in JIT mode.
  if (DartVM::IsRunningPrecompiledCode()) {
    GTEST_SKIP();
    return;
  }

  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  ASSERT_EQ(builder.GetProjectArgs().vm_snapshot_data, nullptr);
  ASSERT_EQ(builder.GetProjectArgs().vm_snapshot_instructions, nullptr);
  ASSERT_EQ(builder.GetProjectArgs().isolate_snapshot_data, nullptr);
  ASSERT_EQ(builder.GetProjectArgs().isolate_snapshot_instructions, nullptr);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, InvalidFlutterWindowMetricsEvent) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 0.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;

  // Pixel ratio must be positive.
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kInvalidArguments);

  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = -1.0;
  event.physical_view_inset_right = -1.0;
  event.physical_view_inset_bottom = -1.0;
  event.physical_view_inset_left = -1.0;

  // Physical view insets must be non-negative.
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kInvalidArguments);

  event.physical_view_inset_top = 700;
  event.physical_view_inset_right = 900;
  event.physical_view_inset_bottom = 700;
  event.physical_view_inset_left = 900;

  // Top/bottom insets cannot be greater than height.
  // Left/right insets cannot be greater than width.
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kInvalidArguments);
}

static void expectSoftwareRenderingOutputMatches(
    EmbedderTest& test,
    std::string entrypoint,
    FlutterSoftwarePixelFormat pixfmt,
    const std::vector<uint8_t>& bytes) {
  auto& context = test.GetEmbedderContext<EmbedderTestContextSoftware>();

  EmbedderConfigBuilder builder(context);
  fml::AutoResetWaitableEvent latch;
  bool matches = false;

  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetCompositor();
  builder.SetDartEntrypoint(std::move(entrypoint));
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer2,
      pixfmt);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  context.GetCompositor().SetNextPresentCallback(
      [&context, &matches, &bytes, &latch](FlutterViewId view_id,
                                           const FlutterLayer** layers,
                                           size_t layers_count) {
        ASSERT_EQ(layers[0]->type, kFlutterLayerContentTypeBackingStore);
        ASSERT_EQ(layers[0]->backing_store->type,
                  kFlutterBackingStoreTypeSoftware2);
        sk_sp<SkSurface> surface =
            context.GetCompositor().GetSurface(layers[0]->backing_store);
        matches = SurfacePixelDataMatchesBytes(surface.get(), bytes);
        latch.Signal();
      });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1;
  event.height = 1;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
  ASSERT_TRUE(matches);

  engine.reset();
}

template <typename T>
static void expectSoftwareRenderingOutputMatches(
    EmbedderTest& test,
    std::string entrypoint,
    FlutterSoftwarePixelFormat pixfmt,
    T pixelvalue) {
  uint8_t* bytes = reinterpret_cast<uint8_t*>(&pixelvalue);
  return expectSoftwareRenderingOutputMatches(
      test, std::move(entrypoint), pixfmt,
      std::vector<uint8_t>(bytes, bytes + sizeof(T)));
}

#define SW_PIXFMT_TEST_F(test_name, dart_entrypoint, pixfmt, matcher)     \
  TEST_F(EmbedderTest, SoftwareRenderingPixelFormats##test_name) {        \
    expectSoftwareRenderingOutputMatches(*this, #dart_entrypoint, pixfmt, \
                                         matcher);                        \
  }

// Don't test the pixel formats that contain padding (so an X) and the
// kFlutterSoftwarePixelFormatNative32 pixel format here, so we don't add any
// flakiness.
SW_PIXFMT_TEST_F(RedRGBA565xF800,
                 draw_solid_red,
                 kFlutterSoftwarePixelFormatRGB565,
                 (uint16_t)0xF800);
SW_PIXFMT_TEST_F(RedRGBA4444xF00F,
                 draw_solid_red,
                 kFlutterSoftwarePixelFormatRGBA4444,
                 (uint16_t)0xF00F);
SW_PIXFMT_TEST_F(RedRGBA8888xFFx00x00xFF,
                 draw_solid_red,
                 kFlutterSoftwarePixelFormatRGBA8888,
                 (std::vector<uint8_t>{0xFF, 0x00, 0x00, 0xFF}));
SW_PIXFMT_TEST_F(RedBGRA8888x00x00xFFxFF,
                 draw_solid_red,
                 kFlutterSoftwarePixelFormatBGRA8888,
                 (std::vector<uint8_t>{0x00, 0x00, 0xFF, 0xFF}));
SW_PIXFMT_TEST_F(RedGray8x36,
                 draw_solid_red,
                 kFlutterSoftwarePixelFormatGray8,
                 (uint8_t)0x36);

SW_PIXFMT_TEST_F(GreenRGB565x07E0,
                 draw_solid_green,
                 kFlutterSoftwarePixelFormatRGB565,
                 (uint16_t)0x07E0);
SW_PIXFMT_TEST_F(GreenRGBA4444x0F0F,
                 draw_solid_green,
                 kFlutterSoftwarePixelFormatRGBA4444,
                 (uint16_t)0x0F0F);
SW_PIXFMT_TEST_F(GreenRGBA8888x00xFFx00xFF,
                 draw_solid_green,
                 kFlutterSoftwarePixelFormatRGBA8888,
                 (std::vector<uint8_t>{0x00, 0xFF, 0x00, 0xFF}));
SW_PIXFMT_TEST_F(GreenBGRA8888x00xFFx00xFF,
                 draw_solid_green,
                 kFlutterSoftwarePixelFormatBGRA8888,
                 (std::vector<uint8_t>{0x00, 0xFF, 0x00, 0xFF}));
SW_PIXFMT_TEST_F(GreenGray8xB6,
                 draw_solid_green,
                 kFlutterSoftwarePixelFormatGray8,
                 (uint8_t)0xB6);

SW_PIXFMT_TEST_F(BlueRGB565x001F,
                 draw_solid_blue,
                 kFlutterSoftwarePixelFormatRGB565,
                 (uint16_t)0x001F);
SW_PIXFMT_TEST_F(BlueRGBA4444x00FF,
                 draw_solid_blue,
                 kFlutterSoftwarePixelFormatRGBA4444,
                 (uint16_t)0x00FF);
SW_PIXFMT_TEST_F(BlueRGBA8888x00x00xFFxFF,
                 draw_solid_blue,
                 kFlutterSoftwarePixelFormatRGBA8888,
                 (std::vector<uint8_t>{0x00, 0x00, 0xFF, 0xFF}));
SW_PIXFMT_TEST_F(BlueBGRA8888xFFx00x00xFF,
                 draw_solid_blue,
                 kFlutterSoftwarePixelFormatBGRA8888,
                 (std::vector<uint8_t>{0xFF, 0x00, 0x00, 0xFF}));
SW_PIXFMT_TEST_F(BlueGray8x12,
                 draw_solid_blue,
                 kFlutterSoftwarePixelFormatGray8,
                 (uint8_t)0x12);

//------------------------------------------------------------------------------
// Key Data
//------------------------------------------------------------------------------

typedef struct {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch;
  bool returned;
} KeyEventUserData;

// Convert `kind` in integer form to its enum form.
//
// It performs a revesed mapping from `_serializeKeyEventType`
// in shell/platform/embedder/fixtures/main.dart.
FlutterKeyEventType UnserializeKeyEventType(uint64_t kind) {
  switch (kind) {
    case 1:
      return kFlutterKeyEventTypeUp;
    case 2:
      return kFlutterKeyEventTypeDown;
    case 3:
      return kFlutterKeyEventTypeRepeat;
    default:
      FML_UNREACHABLE();
      return kFlutterKeyEventTypeUp;
  }
}

// Convert `source` in integer form to its enum form.
//
// It performs a revesed mapping from `_serializeKeyEventDeviceType`
// in shell/platform/embedder/fixtures/main.dart.
FlutterKeyEventDeviceType UnserializeKeyEventDeviceType(uint64_t source) {
  switch (source) {
    case 1:
      return kFlutterKeyEventDeviceTypeKeyboard;
    case 2:
      return kFlutterKeyEventDeviceTypeDirectionalPad;
    case 3:
      return kFlutterKeyEventDeviceTypeGamepad;
    case 4:
      return kFlutterKeyEventDeviceTypeJoystick;
    case 5:
      return kFlutterKeyEventDeviceTypeHdmi;
    default:
      FML_UNREACHABLE();
      return kFlutterKeyEventDeviceTypeKeyboard;
  }
}

// Checks the equality of two `FlutterKeyEvent` by each of their members except
// for `character`. The `character` must be checked separately.
void ExpectKeyEventEq(const FlutterKeyEvent& subject,
                      const FlutterKeyEvent& baseline) {
  EXPECT_EQ(subject.timestamp, baseline.timestamp);
  EXPECT_EQ(subject.type, baseline.type);
  EXPECT_EQ(subject.physical, baseline.physical);
  EXPECT_EQ(subject.logical, baseline.logical);
  EXPECT_EQ(subject.synthesized, baseline.synthesized);
  EXPECT_EQ(subject.device_type, baseline.device_type);
}

TEST_F(EmbedderTest, KeyDataIsCorrectlySerialized) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  uint64_t echoed_char;
  FlutterKeyEvent echoed_event;
  echoed_event.struct_size = sizeof(FlutterKeyEvent);

  auto native_echo_event = [&](Dart_NativeArguments args) {
    echoed_event.type =
        UnserializeKeyEventType(tonic::DartConverter<uint64_t>::FromDart(
            Dart_GetNativeArgument(args, 0)));
    echoed_event.timestamp =
        static_cast<double>(tonic::DartConverter<uint64_t>::FromDart(
            Dart_GetNativeArgument(args, 1)));
    echoed_event.physical = tonic::DartConverter<uint64_t>::FromDart(
        Dart_GetNativeArgument(args, 2));
    echoed_event.logical = tonic::DartConverter<uint64_t>::FromDart(
        Dart_GetNativeArgument(args, 3));
    echoed_char = tonic::DartConverter<uint64_t>::FromDart(
        Dart_GetNativeArgument(args, 4));
    echoed_event.synthesized =
        tonic::DartConverter<bool>::FromDart(Dart_GetNativeArgument(args, 5));
    echoed_event.device_type =
        UnserializeKeyEventDeviceType(tonic::DartConverter<uint64_t>::FromDart(
            Dart_GetNativeArgument(args, 6)));

    message_latch->Signal();
  };

  auto platform_task_runner = CreateNewThread("platform_thread");

  UniqueEngine engine;
  fml::AutoResetWaitableEvent ready;
  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("key_data_echo");
    builder.SetPlatformMessageCallback(
        [&](const FlutterPlatformMessage* message) {
          FlutterEngineSendPlatformMessageResponse(
              engine.get(), message->response_handle, nullptr, 0);
        });
    context.AddNativeCallback(
        "SignalNativeTest",
        CREATE_NATIVE_ENTRY(
            [&ready](Dart_NativeArguments args) { ready.Signal(); }));

    context.AddNativeCallback("EchoKeyEvent",
                              CREATE_NATIVE_ENTRY(native_echo_event));

    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
  });

  ready.Wait();

  // A normal down event
  const FlutterKeyEvent down_event_upper_a{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 1,
      .type = kFlutterKeyEventTypeDown,
      .physical = 0x00070004,
      .logical = 0x00000000061,
      .character = "A",
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &down_event_upper_a, nullptr,
                              nullptr);
  });
  message_latch->Wait();

  ExpectKeyEventEq(echoed_event, down_event_upper_a);
  EXPECT_EQ(echoed_char, 0x41llu);

  // A repeat event with multi-byte character
  const FlutterKeyEvent repeat_event_wide_char{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 1000,
      .type = kFlutterKeyEventTypeRepeat,
      .physical = 0x00070005,
      .logical = 0x00000000062,
      .character = "∆",
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &repeat_event_wide_char, nullptr,
                              nullptr);
  });
  message_latch->Wait();

  ExpectKeyEventEq(echoed_event, repeat_event_wide_char);
  EXPECT_EQ(echoed_char, 0x2206llu);

  // An up event with no character, synthesized
  const FlutterKeyEvent up_event{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 1000000,
      .type = kFlutterKeyEventTypeUp,
      .physical = 0x00070006,
      .logical = 0x00000000063,
      .character = nullptr,
      .synthesized = true,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &up_event, nullptr, nullptr);
  });
  message_latch->Wait();

  ExpectKeyEventEq(echoed_event, up_event);
  EXPECT_EQ(echoed_char, 0llu);

  fml::AutoResetWaitableEvent shutdown_latch;
  platform_task_runner->PostTask([&]() {
    engine.reset();
    shutdown_latch.Signal();
  });
  shutdown_latch.Wait();
}

TEST_F(EmbedderTest, KeyDataAreBuffered) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  std::vector<FlutterKeyEvent> echoed_events;

  auto native_echo_event = [&](Dart_NativeArguments args) {
    echoed_events.push_back(FlutterKeyEvent{
        .timestamp =
            static_cast<double>(tonic::DartConverter<uint64_t>::FromDart(
                Dart_GetNativeArgument(args, 1))),
        .type =
            UnserializeKeyEventType(tonic::DartConverter<uint64_t>::FromDart(
                Dart_GetNativeArgument(args, 0))),
        .physical = tonic::DartConverter<uint64_t>::FromDart(
            Dart_GetNativeArgument(args, 2)),
        .logical = tonic::DartConverter<uint64_t>::FromDart(
            Dart_GetNativeArgument(args, 3)),
        .synthesized = tonic::DartConverter<bool>::FromDart(
            Dart_GetNativeArgument(args, 5)),
        .device_type = UnserializeKeyEventDeviceType(
            tonic::DartConverter<uint64_t>::FromDart(
                Dart_GetNativeArgument(args, 6))),
    });

    message_latch->Signal();
  };

  auto platform_task_runner = CreateNewThread("platform_thread");

  UniqueEngine engine;
  fml::AutoResetWaitableEvent ready;
  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("key_data_late_echo");
    builder.SetPlatformMessageCallback(
        [&](const FlutterPlatformMessage* message) {
          FlutterEngineSendPlatformMessageResponse(
              engine.get(), message->response_handle, nullptr, 0);
        });
    context.AddNativeCallback(
        "SignalNativeTest",
        CREATE_NATIVE_ENTRY(
            [&ready](Dart_NativeArguments args) { ready.Signal(); }));

    context.AddNativeCallback("EchoKeyEvent",
                              CREATE_NATIVE_ENTRY(native_echo_event));

    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
  });
  ready.Wait();

  FlutterKeyEvent sample_event{
      .struct_size = sizeof(FlutterKeyEvent),
      .type = kFlutterKeyEventTypeDown,
      .physical = 0x00070004,
      .logical = 0x00000000061,
      .character = "A",
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };

  // Send an event.
  sample_event.timestamp = 1.0l;
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &sample_event, nullptr, nullptr);
    message_latch->Signal();
  });
  message_latch->Wait();

  // Should not receive echos because the callback is not set yet.
  EXPECT_EQ(echoed_events.size(), 0u);

  // Send an empty message to 'test/starts_echo' to start echoing.
  FlutterPlatformMessageResponseHandle* response_handle = nullptr;
  FlutterPlatformMessageCreateResponseHandle(
      engine.get(), [](const uint8_t* data, size_t size, void* user_data) {},
      nullptr, &response_handle);

  FlutterPlatformMessage message{
      .struct_size = sizeof(FlutterPlatformMessage),
      .channel = "test/starts_echo",
      .message = nullptr,
      .message_size = 0,
      .response_handle = response_handle,
  };

  platform_task_runner->PostTask([&]() {
    FlutterEngineResult result =
        FlutterEngineSendPlatformMessage(engine.get(), &message);
    ASSERT_EQ(result, kSuccess);

    FlutterPlatformMessageReleaseResponseHandle(engine.get(), response_handle);
  });

  // message_latch->Wait();
  message_latch->Wait();
  // All previous events should be received now.
  EXPECT_EQ(echoed_events.size(), 1u);

  // Send a second event.
  sample_event.timestamp = 10.0l;
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &sample_event, nullptr, nullptr);
  });
  message_latch->Wait();

  // The event should be echoed, too.
  EXPECT_EQ(echoed_events.size(), 2u);

  fml::AutoResetWaitableEvent shutdown_latch;
  platform_task_runner->PostTask([&]() {
    engine.reset();
    shutdown_latch.Signal();
  });
  shutdown_latch.Wait();
}

TEST_F(EmbedderTest, KeyDataResponseIsCorrectlyInvoked) {
  UniqueEngine engine;
  fml::AutoResetWaitableEvent sync_latch;
  fml::AutoResetWaitableEvent ready;

  // One of the threads that the key data callback will be posted to is the
  // platform thread. So we cannot wait for assertions to complete on the
  // platform thread. Create a new thread to manage the engine instance and wait
  // for assertions on the test thread.
  auto platform_task_runner = CreateNewThread("platform_thread");

  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("key_data_echo");
    context.AddNativeCallback(
        "SignalNativeTest",
        CREATE_NATIVE_ENTRY(
            [&ready](Dart_NativeArguments args) { ready.Signal(); }));
    context.AddNativeCallback(
        "EchoKeyEvent", CREATE_NATIVE_ENTRY([](Dart_NativeArguments args) {}));

    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());

    sync_latch.Signal();
  });
  sync_latch.Wait();
  ready.Wait();

  // Dispatch a single event
  FlutterKeyEvent event{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 1000,
      .type = kFlutterKeyEventTypeDown,
      .physical = 0x00070005,
      .logical = 0x00000000062,
      .character = nullptr,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };

  KeyEventUserData user_data1{
      .latch = std::make_shared<fml::AutoResetWaitableEvent>(),
  };
  // Entrypoint `key_data_echo` returns `event.synthesized` as `handled`.
  event.synthesized = true;
  platform_task_runner->PostTask([&]() {
    // Test when the response callback is empty.
    // It should not cause a crash.
    FlutterEngineSendKeyEvent(engine.get(), &event, nullptr, nullptr);

    // Test when the response callback is non-empty.
    // It should be invoked (so that the latch can be unlocked.)
    FlutterEngineSendKeyEvent(
        engine.get(), &event,
        [](bool handled, void* untyped_user_data) {
          KeyEventUserData* user_data =
              reinterpret_cast<KeyEventUserData*>(untyped_user_data);
          EXPECT_EQ(handled, true);
          user_data->latch->Signal();
        },
        &user_data1);
  });
  user_data1.latch->Wait();
  fml::AutoResetWaitableEvent shutdown_latch;
  platform_task_runner->PostTask([&]() {
    engine.reset();
    shutdown_latch.Signal();
  });
  shutdown_latch.Wait();
}

TEST_F(EmbedderTest, BackToBackKeyEventResponsesCorrectlyInvoked) {
  UniqueEngine engine;
  fml::AutoResetWaitableEvent sync_latch;
  fml::AutoResetWaitableEvent ready;

  // One of the threads that the callback will be posted to is the platform
  // thread. So we cannot wait for assertions to complete on the platform
  // thread. Create a new thread to manage the engine instance and wait for
  // assertions on the test thread.
  auto platform_task_runner = CreateNewThread("platform_thread");

  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetDartEntrypoint("key_data_echo");
    context.AddNativeCallback(
        "SignalNativeTest",
        CREATE_NATIVE_ENTRY(
            [&ready](Dart_NativeArguments args) { ready.Signal(); }));

    context.AddNativeCallback(
        "EchoKeyEvent", CREATE_NATIVE_ENTRY([](Dart_NativeArguments args) {}));

    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());

    sync_latch.Signal();
  });
  sync_latch.Wait();
  ready.Wait();

  // Dispatch a single event
  FlutterKeyEvent event{
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 1000,
      .type = kFlutterKeyEventTypeDown,
      .physical = 0x00070005,
      .logical = 0x00000000062,
      .character = nullptr,
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };

  // Dispatch two events back to back, using the same callback on different
  // user_data
  KeyEventUserData user_data2{
      .latch = std::make_shared<fml::AutoResetWaitableEvent>(),
      .returned = false,
  };
  KeyEventUserData user_data3{
      .latch = std::make_shared<fml::AutoResetWaitableEvent>(),
      .returned = false,
  };
  auto callback23 = [](bool handled, void* untyped_user_data) {
    KeyEventUserData* user_data =
        reinterpret_cast<KeyEventUserData*>(untyped_user_data);
    EXPECT_EQ(handled, false);
    user_data->returned = true;
    user_data->latch->Signal();
  };
  platform_task_runner->PostTask([&]() {
    FlutterEngineSendKeyEvent(engine.get(), &event, callback23, &user_data2);
    FlutterEngineSendKeyEvent(engine.get(), &event, callback23, &user_data3);
  });
  user_data2.latch->Wait();
  user_data3.latch->Wait();

  EXPECT_TRUE(user_data2.returned);
  EXPECT_TRUE(user_data3.returned);

  fml::AutoResetWaitableEvent shutdown_latch;
  platform_task_runner->PostTask([&]() {
    engine.reset();
    shutdown_latch.Signal();
  });
  shutdown_latch.Wait();
}

//------------------------------------------------------------------------------
// Vsync waiter
//------------------------------------------------------------------------------

// This test schedules a frame for the future and asserts that vsync waiter
// posts the event at the right frame start time (which is in the future).
TEST_F(EmbedderTest, VsyncCallbackPostedIntoFuture) {
  UniqueEngine engine;
  fml::AutoResetWaitableEvent present_latch;
  fml::AutoResetWaitableEvent vsync_latch;

  // One of the threads that the callback (FlutterEngineOnVsync) will be posted
  // to is the platform thread. So we cannot wait for assertions to complete on
  // the platform thread. Create a new thread to manage the engine instance and
  // wait for assertions on the test thread.
  auto platform_task_runner = CreateNewThread("platform_thread");

  platform_task_runner->PostTask([&]() {
    auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
    context.SetVsyncCallback([&](intptr_t baton) {
      platform_task_runner->PostTask([baton = baton, &engine, &vsync_latch]() {
        FlutterEngineOnVsync(engine.get(), baton, NanosFromEpoch(16),
                             NanosFromEpoch(32));
        vsync_latch.Signal();
      });
    });
    context.AddNativeCallback(
        "SignalNativeTest", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
          present_latch.Signal();
        }));

    EmbedderConfigBuilder builder(context);
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetupVsyncCallback();
    builder.SetDartEntrypoint("empty_scene");
    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());

    // Send a window metrics events so frames may be scheduled.
    FlutterWindowMetricsEvent event = {};
    event.struct_size = sizeof(event);
    event.width = 800;
    event.height = 600;
    event.pixel_ratio = 1.0;

    ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
              kSuccess);
  });

  vsync_latch.Wait();
  present_latch.Wait();

  fml::AutoResetWaitableEvent shutdown_latch;
  platform_task_runner->PostTask([&]() {
    engine.reset();
    shutdown_latch.Signal();
  });
  shutdown_latch.Wait();
}

TEST_F(EmbedderTest, CanScheduleFrame) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("can_schedule_frame");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  fml::AutoResetWaitableEvent check_latch;
  context.AddNativeCallback(
      "SignalNativeCount",
      CREATE_NATIVE_ENTRY(
          [&check_latch](Dart_NativeArguments args) { check_latch.Signal(); }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Wait for the application to attach the listener.
  latch.Wait();

  ASSERT_EQ(FlutterEngineScheduleFrame(engine.get()), kSuccess);

  check_latch.Wait();
}

TEST_F(EmbedderTest, CanSetNextFrameCallback) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("draw_solid_red");

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Register the callback that is executed once the next frame is drawn.
  fml::AutoResetWaitableEvent callback_latch;
  VoidCallback callback = [](void* user_data) {
    fml::AutoResetWaitableEvent* callback_latch =
        static_cast<fml::AutoResetWaitableEvent*>(user_data);

    callback_latch->Signal();
  };

  auto result = FlutterEngineSetNextFrameCallback(engine.get(), callback,
                                                  &callback_latch);
  ASSERT_EQ(result, kSuccess);

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  event.physical_view_inset_top = 0.0;
  event.physical_view_inset_right = 0.0;
  event.physical_view_inset_bottom = 0.0;
  event.physical_view_inset_left = 0.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  callback_latch.Wait();
}

#if defined(FML_OS_MACOSX)

static void MockThreadConfigSetter(const fml::Thread::ThreadConfig& config) {
  pthread_t tid = pthread_self();
  struct sched_param param;
  int policy = SCHED_OTHER;
  switch (config.priority) {
    case fml::Thread::ThreadPriority::kDisplay:
      param.sched_priority = 10;
      break;
    default:
      param.sched_priority = 1;
  }
  pthread_setschedparam(tid, policy, &param);
}

TEST_F(EmbedderTest, EmbedderThreadHostUseCustomThreadConfig) {
  auto thread_host =
      flutter::EmbedderThreadHost::CreateEmbedderOrEngineManagedThreadHost(
          nullptr, MockThreadConfigSetter);

  fml::AutoResetWaitableEvent ui_latch;
  int ui_policy;
  struct sched_param ui_param;

  thread_host->GetTaskRunners().GetUITaskRunner()->PostTask([&] {
    pthread_t current_thread = pthread_self();
    pthread_getschedparam(current_thread, &ui_policy, &ui_param);
    ASSERT_EQ(ui_param.sched_priority, 10);
    ui_latch.Signal();
  });

  fml::AutoResetWaitableEvent io_latch;
  int io_policy;
  struct sched_param io_param;
  thread_host->GetTaskRunners().GetIOTaskRunner()->PostTask([&] {
    pthread_t current_thread = pthread_self();
    pthread_getschedparam(current_thread, &io_policy, &io_param);
    ASSERT_EQ(io_param.sched_priority, 1);
    io_latch.Signal();
  });

  ui_latch.Wait();
  io_latch.Wait();
}
#endif

/// Send a pointer event to Dart and wait until the Dart code signals
/// it received the event.
TEST_F(EmbedderTest, CanSendPointer) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("pointer_data_packet");

  fml::AutoResetWaitableEvent ready_latch, count_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));
  context.AddNativeCallback(
      "SignalNativeCount",
      CREATE_NATIVE_ENTRY([&count_latch](Dart_NativeArguments args) {
        int count = tonic::DartConverter<int>::FromDart(
            Dart_GetNativeArgument(args, 0));
        ASSERT_EQ(count, 1);
        count_latch.Signal();
      }));
  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY([&message_latch](Dart_NativeArguments args) {
        auto message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        ASSERT_EQ("PointerData(viewId: 0, x: 123.0, y: 456.0)", message);
        message_latch.Signal();
      }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  FlutterPointerEvent pointer_event = {};
  pointer_event.struct_size = sizeof(FlutterPointerEvent);
  pointer_event.phase = FlutterPointerPhase::kAdd;
  pointer_event.x = 123;
  pointer_event.y = 456;
  pointer_event.timestamp = static_cast<size_t>(1234567890);
  pointer_event.view_id = 0;

  FlutterEngineResult result =
      FlutterEngineSendPointerEvent(engine.get(), &pointer_event, 1);
  ASSERT_EQ(result, kSuccess);

  count_latch.Wait();
  message_latch.Wait();
}

/// Send a pointer event to Dart and wait until the Dart code echos with the
/// view ID.
TEST_F(EmbedderTest, CanSendPointerEventWithViewId) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("pointer_data_packet_view_id");

  fml::AutoResetWaitableEvent ready_latch, add_view_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));
  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY([&message_latch](Dart_NativeArguments args) {
        auto message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        ASSERT_EQ("ViewID: 2", message);
        message_latch.Signal();
      }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  ready_latch.Wait();

  // Add view 2
  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 800;
  metrics.height = 600;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = 2;

  FlutterAddViewInfo info = {};
  info.struct_size = sizeof(FlutterAddViewInfo);
  info.view_id = 2;
  info.view_metrics = &metrics;
  info.add_view_callback = [](const FlutterAddViewResult* result) {
    EXPECT_TRUE(result->added);
    fml::AutoResetWaitableEvent* add_view_latch =
        reinterpret_cast<fml::AutoResetWaitableEvent*>(result->user_data);
    add_view_latch->Signal();
  };
  info.user_data = &add_view_latch;
  ASSERT_EQ(FlutterEngineAddView(engine.get(), &info), kSuccess);
  add_view_latch.Wait();

  // Send a pointer event for view 2
  FlutterPointerEvent pointer_event = {};
  pointer_event.struct_size = sizeof(FlutterPointerEvent);
  pointer_event.phase = FlutterPointerPhase::kAdd;
  pointer_event.x = 123;
  pointer_event.y = 456;
  pointer_event.timestamp = static_cast<size_t>(1234567890);
  pointer_event.view_id = 2;

  FlutterEngineResult result =
      FlutterEngineSendPointerEvent(engine.get(), &pointer_event, 1);
  ASSERT_EQ(result, kSuccess);

  message_latch.Wait();
}

TEST_F(EmbedderTest, WindowMetricsEventDefaultsToImplicitView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_view_id");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));
  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY([&message_latch](Dart_NativeArguments args) {
        auto message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        ASSERT_EQ("Changed: [0]", message);
        message_latch.Signal();
      }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  FlutterWindowMetricsEvent event = {};
  // Simulate an event that comes from an old version of embedder.h that doesn't
  // have the view_id field.
  event.struct_size = offsetof(FlutterWindowMetricsEvent, view_id);
  event.width = 200;
  event.height = 300;
  event.pixel_ratio = 1.5;
  // Skip assigning event.view_id here to test the default behavior.

  FlutterEngineResult result =
      FlutterEngineSendWindowMetricsEvent(engine.get(), &event);
  ASSERT_EQ(result, kSuccess);

  message_latch.Wait();
}

TEST_F(EmbedderTest, IgnoresWindowMetricsEventForUnknownView) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("window_metrics_event_view_id");

  fml::AutoResetWaitableEvent ready_latch, message_latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&ready_latch](Dart_NativeArguments args) { ready_latch.Signal(); }));

  context.AddNativeCallback(
      "SignalNativeMessage",
      CREATE_NATIVE_ENTRY([&message_latch](Dart_NativeArguments args) {
        auto message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        // Message latch should only be signaled once as the bad
        // view metric should be dropped by the engine.
        ASSERT_FALSE(message_latch.IsSignaledForTest());
        ASSERT_EQ("Changed: [0]", message);
        message_latch.Signal();
      }));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ready_latch.Wait();

  // Send a window metric for a nonexistent view, which should be dropped by the
  // engine.
  FlutterWindowMetricsEvent bad_event = {};
  bad_event.struct_size = sizeof(FlutterWindowMetricsEvent);
  bad_event.width = 200;
  bad_event.height = 300;
  bad_event.pixel_ratio = 1.5;
  bad_event.view_id = 100;

  FlutterEngineResult result =
      FlutterEngineSendWindowMetricsEvent(engine.get(), &bad_event);
  ASSERT_EQ(result, kSuccess);

  // Send a window metric for a valid view. The engine notifies the Dart app.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(FlutterWindowMetricsEvent);
  event.width = 200;
  event.height = 300;
  event.pixel_ratio = 1.5;
  event.view_id = 0;

  result = FlutterEngineSendWindowMetricsEvent(engine.get(), &event);
  ASSERT_EQ(result, kSuccess);

  message_latch.Wait();
}

TEST_F(EmbedderTest, RegisterChannelListener) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();

  fml::AutoResetWaitableEvent latch;
  fml::AutoResetWaitableEvent latch2;
  bool listening = false;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments) { latch.Signal(); }));
  context.SetChannelUpdateCallback([&](const FlutterChannelUpdate* update) {
    EXPECT_STREQ(update->channel, "test/listen");
    EXPECT_TRUE(update->listening);
    listening = true;
    latch2.Signal();
  });

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetDartEntrypoint("channel_listener_response");

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
  // Drain tasks posted to platform thread task runner.
  fml::MessageLoop::GetCurrent().RunExpiredTasksNow();
  latch2.Wait();

  ASSERT_TRUE(listening);
}

TEST_F(EmbedderTest, PlatformThreadIsolatesWithCustomPlatformTaskRunner) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  static fml::AutoResetWaitableEvent latch;

  static std::thread::id ffi_call_thread_id;
  static void (*ffi_signal_native_test)() = []() -> void {
    ffi_call_thread_id = std::this_thread::get_id();
    latch.Signal();
  };

  Dart_FfiNativeResolver ffi_resolver = [](const char* name,
                                           uintptr_t args_n) -> void* {
    if (std::string_view(name) == "FFISignalNativeTest") {
      return reinterpret_cast<void*>(ffi_signal_native_test);
    }
    return nullptr;
  };

  // The test's Dart code will call this native function which overrides the
  // FFI resolver.  After that, the Dart code will invoke the FFI function
  // using runOnPlatformThread.
  context.AddNativeCallback(
      "SignalNativeTest", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        Dart_SetFfiNativeResolver(Dart_RootLibrary(), ffi_resolver);
      }));

  auto platform_task_runner = CreateNewThread("test_platform_thread");

  UniqueEngine engine;

  EmbedderTestTaskRunner test_task_runner(
      platform_task_runner, [&](FlutterTask task) {
        if (!engine.is_valid()) {
          return;
        }
        FlutterEngineRunTask(engine.get(), &task);
      });

  std::thread::id platform_thread_id;
  platform_task_runner->PostTask([&]() {
    platform_thread_id = std::this_thread::get_id();

    EmbedderConfigBuilder builder(context);
    const auto task_runner_description =
        test_task_runner.GetFlutterTaskRunnerDescription();
    builder.SetSurface(SkISize::Make(1, 1));
    builder.SetPlatformTaskRunner(&task_runner_description);
    builder.SetDartEntrypoint("invokePlatformThreadIsolate");
    builder.AddCommandLineArgument("--enable-platform-isolates");
    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());
  });

  latch.Wait();

  fml::AutoResetWaitableEvent kill_latch;
  platform_task_runner->PostTask(fml::MakeCopyable([&]() mutable {
    engine.reset();

    platform_task_runner->PostTask([&kill_latch] { kill_latch.Signal(); });
  }));
  kill_latch.Wait();

  // Check that the FFI call was executed on the platform thread.
  ASSERT_EQ(platform_thread_id, ffi_call_thread_id);
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
