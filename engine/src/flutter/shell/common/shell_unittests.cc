// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <algorithm>
#include <ctime>
#include <future>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#if SHELL_ENABLE_GL
#include <EGL/egl.h>
#endif  // SHELL_ENABLE_GL

#include "assets/asset_resolver.h"
#include "assets/directory_asset_bundle.h"
#include "common/graphics/persistent_cache.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/flow/layers/backdrop_filter_layer.h"
#include "flutter/flow/layers/clip_rect_layer.h"
#include "flutter/flow/layers/display_list_layer.h"
#include "flutter/flow/layers/layer_raster_cache_item.h"
#include "flutter/flow/layers/platform_view_layer.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/fml/backtrace.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/shell_test_external_view_embedder.h"
#include "flutter/shell/common/shell_test_platform_view.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/common/vsync_waiter_fallback.h"
#include "flutter/shell/common/vsync_waiters_test.h"
#include "flutter/shell/version/version.h"
#include "flutter/testing/testing.h"
#include "fml/mapping.h"
#include "gmock/gmock.h"
#include "impeller/core/runtime_types.h"
#include "lib/ui/semantics/semantics_node.h"
#include "third_party/rapidjson/include/rapidjson/writer.h"
#include "third_party/skia/include/codec/SkCodecAnimation.h"
#include "third_party/tonic/converter/dart_converter.h"

#ifdef SHELL_ENABLE_VULKAN
#include "flutter/vulkan/vulkan_application.h"  // nogncheck
#endif

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

constexpr int64_t kImplicitViewId = 0ll;

using ::testing::_;
using ::testing::Return;

namespace {

std::unique_ptr<PlatformMessage> MakePlatformMessage(
    const std::string& channel,
    const std::map<std::string, std::string>& values,
    const fml::RefPtr<PlatformMessageResponse>& response) {
  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  document.SetObject();

  for (const auto& pair : values) {
    rapidjson::Value key(pair.first.c_str(), strlen(pair.first.c_str()),
                         allocator);
    rapidjson::Value value(pair.second.c_str(), strlen(pair.second.c_str()),
                           allocator);
    document.AddMember(key, value, allocator);
  }

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.GetString());

  std::unique_ptr<PlatformMessage> message = std::make_unique<PlatformMessage>(
      channel, fml::MallocMapping::Copy(data, buffer.GetSize()), response);
  return message;
}

class MockPlatformViewDelegate : public PlatformView::Delegate {
  MOCK_METHOD(void,
              OnPlatformViewCreated,
              (std::unique_ptr<Surface> surface),
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

  MOCK_METHOD(const Settings&,
              OnPlatformViewGetSettings,
              (),
              (const, override));

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
};

class MockSurface : public Surface {
 public:
  MOCK_METHOD(bool, IsValid, (), (override));

  MOCK_METHOD(std::unique_ptr<SurfaceFrame>,
              AcquireFrame,
              (const SkISize& size),
              (override));

  MOCK_METHOD(SkMatrix, GetRootTransformation, (), (const, override));

  MOCK_METHOD(GrDirectContext*, GetContext, (), (override));

  MOCK_METHOD(std::unique_ptr<GLContextResult>,
              MakeRenderContextCurrent,
              (),
              (override));

  MOCK_METHOD(bool, ClearRenderContext, (), (override));
};

class MockPlatformView : public PlatformView {
 public:
  MockPlatformView(MockPlatformViewDelegate& delegate,
                   const TaskRunners& task_runners)
      : PlatformView(delegate, task_runners) {}
  MOCK_METHOD(std::unique_ptr<Surface>, CreateRenderingSurface, (), (override));
  MOCK_METHOD(std::shared_ptr<PlatformMessageHandler>,
              GetPlatformMessageHandler,
              (),
              (const, override));
};

class TestPlatformView : public PlatformView {
 public:
  TestPlatformView(Shell& shell, const TaskRunners& task_runners)
      : PlatformView(shell, task_runners) {}
  MOCK_METHOD(std::unique_ptr<Surface>, CreateRenderingSurface, (), (override));
};

class MockPlatformMessageHandler : public PlatformMessageHandler {
 public:
  MOCK_METHOD(void,
              HandlePlatformMessage,
              (std::unique_ptr<PlatformMessage> message),
              (override));
  MOCK_METHOD(bool,
              DoesHandlePlatformMessageOnPlatformThread,
              (),
              (const, override));
  MOCK_METHOD(void,
              InvokePlatformMessageResponseCallback,
              (int response_id, std::unique_ptr<fml::Mapping> mapping),
              (override));
  MOCK_METHOD(void,
              InvokePlatformMessageEmptyResponseCallback,
              (int response_id),
              (override));
};

class MockPlatformMessageResponse : public PlatformMessageResponse {
 public:
  static fml::RefPtr<MockPlatformMessageResponse> Create() {
    return fml::AdoptRef(new MockPlatformMessageResponse());
  }
  MOCK_METHOD(void, Complete, (std::unique_ptr<fml::Mapping> data), (override));
  MOCK_METHOD(void, CompleteEmpty, (), (override));
};
}  // namespace

class TestAssetResolver : public AssetResolver {
 public:
  TestAssetResolver(bool valid, AssetResolver::AssetResolverType type)
      : valid_(valid), type_(type) {}

  bool IsValid() const override { return true; }

  // This is used to identify if replacement was made or not.
  bool IsValidAfterAssetManagerChange() const override { return valid_; }

  AssetResolver::AssetResolverType GetType() const override { return type_; }

  std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const override {
    return nullptr;
  }

  std::vector<std::unique_ptr<fml::Mapping>> GetAsMappings(
      const std::string& asset_pattern,
      const std::optional<std::string>& subdir) const override {
    return {};
  };

  bool operator==(const AssetResolver& other) const override {
    return this == &other;
  }

 private:
  bool valid_;
  AssetResolver::AssetResolverType type_;
};

class ThreadCheckingAssetResolver : public AssetResolver {
 public:
  explicit ThreadCheckingAssetResolver(
      std::shared_ptr<fml::ConcurrentMessageLoop> concurrent_loop)
      : concurrent_loop_(std::move(concurrent_loop)) {}

  // |AssetResolver|
  bool IsValid() const override { return true; }

  // |AssetResolver|
  bool IsValidAfterAssetManagerChange() const override { return true; }

  // |AssetResolver|
  AssetResolverType GetType() const {
    return AssetResolverType::kApkAssetProvider;
  }

  // |AssetResolver|
  std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const override {
    if (asset_name == "FontManifest.json" ||
        asset_name == "NativeAssetsManifest.json") {
      // This file is loaded directly by the engine.
      return nullptr;
    }
    mapping_requests.push_back(asset_name);
    EXPECT_TRUE(concurrent_loop_->RunsTasksOnCurrentThread())
        << fml::BacktraceHere();
    return nullptr;
  }

  mutable std::vector<std::string> mapping_requests;

  bool operator==(const AssetResolver& other) const override {
    return this == &other;
  }

 private:
  std::shared_ptr<fml::ConcurrentMessageLoop> concurrent_loop_;
};

static bool ValidateShell(Shell* shell) {
  if (!shell) {
    return false;
  }

  if (!shell->IsSetup()) {
    return false;
  }

  ShellTest::PlatformViewNotifyCreated(shell);

  {
    fml::AutoResetWaitableEvent latch;
    fml::TaskRunner::RunNowOrPostTask(
        shell->GetTaskRunners().GetPlatformTaskRunner(), [shell, &latch]() {
          shell->GetPlatformView()->NotifyDestroyed();
          latch.Signal();
        });
    latch.Wait();
  }

  return true;
}

static bool RasterizerIsTornDown(Shell* shell) {
  fml::AutoResetWaitableEvent latch;
  bool is_torn_down = false;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(),
      [shell, &latch, &is_torn_down]() {
        is_torn_down = shell->GetRasterizer()->IsTornDown();
        latch.Signal();
      });
  latch.Wait();
  return is_torn_down;
}

static void ValidateDestroyPlatformView(Shell* shell) {
  ASSERT_TRUE(shell != nullptr);
  ASSERT_TRUE(shell->IsSetup());

  ASSERT_FALSE(RasterizerIsTornDown(shell));
  ShellTest::PlatformViewNotifyDestroyed(shell);
  ASSERT_TRUE(RasterizerIsTornDown(shell));
}

static std::string CreateFlagsString(std::vector<const char*>& flags) {
  if (flags.empty()) {
    return "";
  }
  std::string flags_string = flags[0];
  for (size_t i = 1; i < flags.size(); ++i) {
    flags_string += ",";
    flags_string += flags[i];
  }
  return flags_string;
}

static void TestDartVmFlags(std::vector<const char*>& flags) {
  std::string flags_string = CreateFlagsString(flags);
  const std::vector<fml::CommandLine::Option> options = {
      fml::CommandLine::Option("dart-flags", flags_string)};
  fml::CommandLine command_line("", options, std::vector<std::string>());
  flutter::Settings settings = flutter::SettingsFromCommandLine(command_line);
  EXPECT_EQ(settings.dart_flags.size(), flags.size());
  for (size_t i = 0; i < flags.size(); ++i) {
    EXPECT_EQ(settings.dart_flags[i], flags[i]);
  }
}

static void PostSync(const fml::RefPtr<fml::TaskRunner>& task_runner,
                     const fml::closure& task) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(task_runner, [&latch, &task] {
    task();
    latch.Signal();
  });
  latch.Wait();
}

static sk_sp<DisplayList> MakeSizedDisplayList(int width, int height) {
  DisplayListBuilder builder(DlRect::MakeXYWH(0, 0, width, height));
  builder.DrawRect(DlRect::MakeXYWH(0, 0, width, height),
                   DlPaint(DlColor::kRed()));
  return builder.Build();
}

TEST_F(ShellTest, InitializeWithInvalidThreads) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test", nullptr, nullptr, nullptr, nullptr);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_FALSE(shell);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, InitializeWithDifferentThreads) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  std::string name_prefix = "io.flutter.test." + GetCurrentTestName() + ".";
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      name_prefix, ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
                       ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  ASSERT_EQ(thread_host.name_prefix, name_prefix);

  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(ValidateShell(shell.get()));
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, InitializeWithSingleThread) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));
  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, InitializeWithSingleThreadWhichIsTheCallingThread) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(ValidateShell(shell.get()));
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest,
       InitializeWithMultipleThreadButCallingThreadAsPlatformThread) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kRaster | ThreadHost::Type::kIo |
                             ThreadHost::Type::kUi);
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  TaskRunners task_runners("test",
                           fml::MessageLoop::GetCurrent().GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  auto shell = Shell::Create(
      flutter::PlatformData(), task_runners, settings,
      [](Shell& shell) {
        // This is unused in the platform view as we are not using the simulated
        // vsync mechanism. We should have better DI in the tests.
        const auto vsync_clock = std::make_shared<ShellTestVsyncClock>();
        return ShellTestPlatformView::Create(
            ShellTestPlatformView::DefaultBackendType(), shell,
            shell.GetTaskRunners(), vsync_clock,
            [task_runners = shell.GetTaskRunners()]() {
              return static_cast<std::unique_ptr<VsyncWaiter>>(
                  std::make_unique<VsyncWaiterFallback>(task_runners));
            },
            nullptr, shell.GetIsGpuDisabledSyncSwitch());
      },
      [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
  ASSERT_TRUE(ValidateShell(shell.get()));
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, InitializeWithDisabledGpu) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
      .is_gpu_disabled = true,
  });
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  bool is_disabled = false;
  shell->GetIsGpuDisabledSyncSwitch()->Execute(
      fml::SyncSwitch::Handlers().SetIfTrue([&] { is_disabled = true; }));
  ASSERT_TRUE(is_disabled);

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, InitializeWithGPUAndPlatformThreadsTheSame) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                             ThreadHost::Type::kUi);
  TaskRunners task_runners(
      "test",
      thread_host.platform_thread->GetTaskRunner(),  // platform
      thread_host.platform_thread->GetTaskRunner(),  // raster
      thread_host.ui_thread->GetTaskRunner(),        // ui
      thread_host.io_thread->GetTaskRunner()         // io
  );
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));
  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, FixturesAreFunctional) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("fixturesAreFunctionalMain");

  fml::AutoResetWaitableEvent main_latch;
  AddNativeCallback(
      "SayHiFromFixturesAreFunctionalMain",
      CREATE_NATIVE_ENTRY([&main_latch](auto args) { main_latch.Signal(); }));

  RunEngine(shell.get(), std::move(configuration));
  main_latch.Wait();
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, SecondaryIsolateBindingsAreSetupViaShellSettings) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("testCanLaunchSecondaryIsolate");

  fml::CountDownLatch latch(2);
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&latch](auto args) {
                      latch.CountDown();
                    }));

  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();

  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, LastEntrypoint) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  std::string entry_point = "fixturesAreFunctionalMain";
  configuration.SetEntrypoint(entry_point);

  fml::AutoResetWaitableEvent main_latch;
  std::string last_entry_point;
  AddNativeCallback(
      "SayHiFromFixturesAreFunctionalMain", CREATE_NATIVE_ENTRY([&](auto args) {
        last_entry_point = shell->GetEngine()->GetLastEntrypoint();
        main_latch.Signal();
      }));

  RunEngine(shell.get(), std::move(configuration));
  main_latch.Wait();
  EXPECT_EQ(entry_point, last_entry_point);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, LastEntrypointArgs) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  std::string entry_point = "fixturesAreFunctionalMain";
  std::vector<std::string> entry_point_args = {"arg1"};
  configuration.SetEntrypoint(entry_point);
  configuration.SetEntrypointArgs(entry_point_args);

  fml::AutoResetWaitableEvent main_latch;
  std::vector<std::string> last_entry_point_args;
  AddNativeCallback(
      "SayHiFromFixturesAreFunctionalMain", CREATE_NATIVE_ENTRY([&](auto args) {
        last_entry_point_args = shell->GetEngine()->GetLastEntrypointArgs();
        main_latch.Signal();
      }));

  RunEngine(shell.get(), std::move(configuration));
  main_latch.Wait();
#if (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG)
  EXPECT_EQ(last_entry_point_args, entry_point_args);
#else
  ASSERT_TRUE(last_entry_point_args.empty());
#endif
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, DisallowedDartVMFlag) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "This test flakes on Fuchsia. https://fxbug.dev/110006 ";
#else

  // Run this test in a thread-safe manner, otherwise gtest will complain.
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  const std::vector<fml::CommandLine::Option> options = {
      fml::CommandLine::Option("dart-flags", "--verify_after_gc")};
  fml::CommandLine command_line("", options, std::vector<std::string>());

  // Upon encountering a disallowed Dart flag the process terminates.
  const char* expected =
      "Encountered disallowed Dart VM flag: --verify_after_gc";
  ASSERT_DEATH(flutter::SettingsFromCommandLine(command_line), expected);
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, AllowedDartVMFlag) {
  std::vector<const char*> flags = {
      "--enable-isolate-groups",
      "--no-enable-isolate-groups",
  };
#if !FLUTTER_RELEASE
  flags.push_back("--max_profile_depth 1");
  flags.push_back("--random_seed 42");
  flags.push_back("--max_subtype_cache_entries=22");
  if (!DartVM::IsRunningPrecompiledCode()) {
    flags.push_back("--enable_mirrors");
  }
#endif

  TestDartVmFlags(flags);
}

TEST_F(ShellTest, NoNeedToReportTimingsByDefault) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  ASSERT_FALSE(GetNeedsReportTimings(shell.get()));

  // This assertion may or may not be the direct result of needs_report_timings_
  // being false. The count could be 0 simply because we just cleared
  // unreported timings by reporting them. Hence this can't replace the
  // ASSERT_FALSE(GetNeedsReportTimings(shell.get())) check. We added
  // this assertion for an additional confidence that we're not pushing
  // back to unreported timings unnecessarily.
  //
  // Conversely, do not assert UnreportedTimingsCount(shell.get()) to be
  // positive in any tests. Otherwise those tests will be flaky as the clearing
  // of unreported timings is unpredictive.
  ASSERT_EQ(UnreportedTimingsCount(shell.get()), 0);
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, NeedsReportTimingsIsSetWithCallback) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("dummyReportTimingsMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  ASSERT_TRUE(GetNeedsReportTimings(shell.get()));
  DestroyShell(std::move(shell));
}

static void CheckFrameTimings(const std::vector<FrameTiming>& timings,
                              fml::TimePoint start,
                              fml::TimePoint finish) {
  fml::TimePoint last_frame_start;
  for (size_t i = 0; i < timings.size(); i += 1) {
    // Ensure that timings are sorted.
    ASSERT_TRUE(timings[i].Get(FrameTiming::kPhases[0]) >= last_frame_start);
    last_frame_start = timings[i].Get(FrameTiming::kPhases[0]);

    fml::TimePoint last_phase_time;
    for (auto phase : FrameTiming::kPhases) {
      // raster finish wall time doesn't use the same clock base
      // as rest of the frame timings.
      if (phase == FrameTiming::kRasterFinishWallTime) {
        continue;
      }

      ASSERT_TRUE(timings[i].Get(phase) >= start);
      ASSERT_TRUE(timings[i].Get(phase) <= finish);

      // phases should have weakly increasing time points
      ASSERT_TRUE(last_phase_time <= timings[i].Get(phase));
      last_phase_time = timings[i].Get(phase);
    }
  }
}

TEST_F(ShellTest, ReportTimingsIsCalled) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // We MUST put |start| after |CreateShell| because the clock source will be
  // reset through |TimePoint::SetClockSource()| in
  // |DartVMInitializer::Initialize()| within |CreateShell()|.
  fml::TimePoint start = fml::TimePoint::Now();

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("reportTimingsMain");
  fml::AutoResetWaitableEvent reportLatch;
  std::vector<int64_t> timestamps;
  auto nativeTimingCallback = [&reportLatch,
                               &timestamps](Dart_NativeArguments args) {
    Dart_Handle exception = nullptr;
    ASSERT_EQ(timestamps.size(), 0ul);
    timestamps = tonic::DartConverter<std::vector<int64_t>>::FromArguments(
        args, 0, exception);
    reportLatch.Signal();
  };
  AddNativeCallback("NativeReportTimingsCallback",
                    CREATE_NATIVE_ENTRY(nativeTimingCallback));
  RunEngine(shell.get(), std::move(configuration));

  // Pump many frames so we can trigger the report quickly instead of waiting
  // for the 1 second threshold.
  for (int i = 0; i < 200; i += 1) {
    PumpOneFrame(shell.get());
  }

  reportLatch.Wait();
  DestroyShell(std::move(shell));

  fml::TimePoint finish = fml::TimePoint::Now();
  ASSERT_TRUE(!timestamps.empty());
  ASSERT_TRUE(timestamps.size() % FrameTiming::kCount == 0);
  std::vector<FrameTiming> timings(timestamps.size() / FrameTiming::kCount);

  for (size_t i = 0; i * FrameTiming::kCount < timestamps.size(); i += 1) {
    for (auto phase : FrameTiming::kPhases) {
      timings[i].Set(
          phase,
          fml::TimePoint::FromEpochDelta(fml::TimeDelta::FromMicroseconds(
              timestamps[i * FrameTiming::kCount + phase])));
    }
  }
  CheckFrameTimings(timings, start, finish);
}

TEST_F(ShellTest, FrameRasterizedCallbackIsCalled) {
  auto settings = CreateSettingsForFixture();

  FrameTiming timing;
  fml::AutoResetWaitableEvent timingLatch;
  settings.frame_rasterized_callback = [&timing,
                                        &timingLatch](const FrameTiming& t) {
    timing = t;
    timingLatch.Signal();
  };

  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Wait to make |start| bigger than zero
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1ms);

  // We MUST put |start| after |CreateShell()| because the clock source will be
  // reset through |TimePoint::SetClockSource()| in
  // |DartVMInitializer::Initialize()| within |CreateShell()|.
  fml::TimePoint start = fml::TimePoint::Now();

  for (auto phase : FrameTiming::kPhases) {
    timing.Set(phase, fml::TimePoint());
    // Check that the time points are initially smaller than start, so
    // CheckFrameTimings will fail if they're not properly set later.
    ASSERT_TRUE(timing.Get(phase) < start);
  }

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("onBeginFrameMain");

  int64_t frame_target_time;
  auto nativeOnBeginFrame = [&frame_target_time](Dart_NativeArguments args) {
    Dart_Handle exception = nullptr;
    frame_target_time =
        tonic::DartConverter<int64_t>::FromArguments(args, 0, exception);
  };
  AddNativeCallback("NativeOnBeginFrame",
                    CREATE_NATIVE_ENTRY(nativeOnBeginFrame));

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  // Check that timing is properly set. This implies that
  // settings.frame_rasterized_callback is called.
  timingLatch.Wait();
  fml::TimePoint finish = fml::TimePoint::Now();
  std::vector<FrameTiming> timings = {timing};
  CheckFrameTimings(timings, start, finish);

  // Check that onBeginFrame, which is the frame_target_time, is after
  // FrameTiming's build start
  int64_t build_start =
      timing.Get(FrameTiming::kBuildStart).ToEpochDelta().ToMicroseconds();
  ASSERT_GT(frame_target_time, build_start);
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, ExternalEmbedderNoThreadMerger) {
  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  bool end_frame_called = false;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        ASSERT_TRUE(raster_thread_merger.get() == nullptr);
        ASSERT_FALSE(should_resubmit_frame);
        end_frame_called = true;
        end_frame_latch.Signal();
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kResubmitFrame, false);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();
  ASSERT_TRUE(end_frame_called);

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, PushBackdropFilterToVisitedPlatformViews) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();

  std::shared_ptr<ShellTestExternalViewEmbedder> external_view_embedder;

  fml::AutoResetWaitableEvent end_frame_latch;
  bool end_frame_called = false;
  std::vector<int64_t> visited_platform_views;
  MutatorsStack stack_50;
  MutatorsStack stack_75;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        if (end_frame_called) {
          return;
        }
        ASSERT_TRUE(raster_thread_merger.get() == nullptr);
        ASSERT_FALSE(should_resubmit_frame);
        end_frame_called = true;
        visited_platform_views =
            external_view_embedder->GetVisitedPlatformViews();
        stack_50 = external_view_embedder->GetStack(50);
        stack_75 = external_view_embedder->GetStack(75);
        end_frame_latch.Signal();
      };

  external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kResubmitFrame, false);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto platform_view_layer = std::make_shared<PlatformViewLayer>(
        DlPoint(10, 10), DlSize(10, 10), 50);
    root->Add(platform_view_layer);
    auto transform_layer = std::make_shared<TransformLayer>(
        DlMatrix::MakeTranslation({1.0f, 1.0f}));
    root->Add(transform_layer);
    auto clip_rect_layer = std::make_shared<ClipRectLayer>(
        DlRect::MakeLTRB(0, 0, 30, 30), Clip::kHardEdge);
    transform_layer->Add(clip_rect_layer);
    auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
    auto backdrop_filter_layer =
        std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver);
    clip_rect_layer->Add(backdrop_filter_layer);
    auto platform_view_layer2 = std::make_shared<PlatformViewLayer>(
        DlPoint(10, 10), DlSize(10, 10), 75);
    backdrop_filter_layer->Add(platform_view_layer2);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();
  ASSERT_EQ(visited_platform_views, (std::vector<int64_t>{50, 75}));
  ASSERT_TRUE(stack_75.is_empty());
  ASSERT_FALSE(stack_50.is_empty());

  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto mutator = *stack_50.Begin();
  ASSERT_EQ(mutator->GetType(), MutatorType::kBackdropFilter);
  ASSERT_EQ(mutator->GetFilterMutation().GetFilter(), *filter);
  // Make sure the filterRect is in global coordinates (contains the (1,1)
  // translation).
  ASSERT_EQ(mutator->GetFilterMutation().GetFilterRect(),
            SkRect::MakeLTRB(1, 1, 31, 31));

  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

// TODO(https://github.com/flutter/flutter/issues/59816): Enable on fuchsia.
TEST_F(ShellTest,
       ExternalEmbedderEndFrameIsCalledWhenPostPrerollResultIsResubmit) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  bool end_frame_called = false;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        ASSERT_TRUE(raster_thread_merger.get() != nullptr);
        ASSERT_TRUE(should_resubmit_frame);
        end_frame_called = true;
        end_frame_latch.Signal();
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kResubmitFrame, true);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();

  ASSERT_TRUE(end_frame_called);

  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, OnPlatformViewDestroyDisablesThreadMerger) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();
  fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          fml::RefPtr<fml::RasterThreadMerger> thread_merger) {
        raster_thread_merger = std::move(thread_merger);
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);

  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));

  auto result = shell->WaitForFirstFrame(fml::TimeDelta::Max());
  // Wait for the rasterizer to process the frame. WaitForFirstFrame only waits
  // for the Animator, but end_frame_callback is called by the Rasterizer.
  PostSync(shell->GetTaskRunners().GetRasterTaskRunner(), [] {});
  ASSERT_TRUE(result.ok()) << "Result: " << static_cast<int>(result.code())
                           << ": " << result.message();

  ASSERT_TRUE(raster_thread_merger->IsEnabled());

  ValidateDestroyPlatformView(shell.get());
  ASSERT_TRUE(raster_thread_merger->IsEnabled());

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());
  ASSERT_TRUE(raster_thread_merger->IsEnabled());
  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, OnPlatformViewDestroyAfterMergingThreads) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  const int ThreadMergingLease = 10;
  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  std::shared_ptr<ShellTestExternalViewEmbedder> external_view_embedder;

  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        if (should_resubmit_frame && !raster_thread_merger->IsMerged()) {
          raster_thread_merger->MergeWithLease(ThreadMergingLease);

          ASSERT_TRUE(raster_thread_merger->IsMerged());
          external_view_embedder->UpdatePostPrerollResult(
              PostPrerollResult::kSuccess);
        }
        end_frame_latch.Signal();
      };
  external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);
  // Set resubmit once to trigger thread merging.
  external_view_embedder->UpdatePostPrerollResult(
      PostPrerollResult::kResubmitFrame);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  // Pump one frame to trigger thread merging.
  end_frame_latch.Wait();
  // Pump another frame to ensure threads are merged and a regular layer tree is
  // submitted.
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  // Threads are merged here. PlatformViewNotifyDestroy should be executed
  // successfully.
  ASSERT_TRUE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));
  ValidateDestroyPlatformView(shell.get());

  // Ensure threads are unmerged after platform view destroy
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, OnPlatformViewDestroyWhenThreadsAreMerging) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  const int kThreadMergingLease = 10;
  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        if (should_resubmit_frame && !raster_thread_merger->IsMerged()) {
          raster_thread_merger->MergeWithLease(kThreadMergingLease);
        }
        end_frame_latch.Signal();
      };
  // Start with a regular layer tree with `PostPrerollResult::kSuccess` so we
  // can later check if the rasterizer is tore down using
  // |ValidateDestroyPlatformView|
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);

  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  // Pump one frame and threads aren't merged
  end_frame_latch.Wait();
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));

  // Pump a frame with `PostPrerollResult::kResubmitFrame` to start merging
  // threads
  external_view_embedder->UpdatePostPrerollResult(
      PostPrerollResult::kResubmitFrame);
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));

  // Now destroy the platform view immediately.
  // Two things can happen here:
  // 1. Threads haven't merged. 2. Threads has already merged.
  // |Shell:OnPlatformViewDestroy| should be able to handle both cases.
  ValidateDestroyPlatformView(shell.get());

  // Ensure threads are unmerged after platform view destroy
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest,
       OnPlatformViewDestroyWithThreadMergerWhileThreadsAreUnmerged) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        end_frame_latch.Signal();
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();

  // Threads should not be merged.
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));
  ValidateDestroyPlatformView(shell.get());

  // Ensure threads are unmerged after platform view destroy
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, OnPlatformViewDestroyWithoutRasterThreadMerger) {
  auto settings = CreateSettingsForFixture();

  auto shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));

  // Threads should not be merged.
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));
  ValidateDestroyPlatformView(shell.get());

  // Ensure threads are unmerged after platform view destroy
  ASSERT_FALSE(fml::TaskRunnerChecker::RunsOnTheSameThread(
      shell->GetTaskRunners().GetRasterTaskRunner()->GetTaskQueueId(),
      shell->GetTaskRunners().GetPlatformTaskRunner()->GetTaskQueueId()));

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell));
}

// TODO(https://github.com/flutter/flutter/issues/59816): Enable on fuchsia.
TEST_F(ShellTest, OnPlatformViewDestroyWithStaticThreadMerging) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        end_frame_latch.Signal();
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                             ThreadHost::Type::kUi);
  TaskRunners task_runners(
      "test",
      thread_host.platform_thread->GetTaskRunner(),  // platform
      thread_host.platform_thread->GetTaskRunner(),  // raster
      thread_host.ui_thread->GetTaskRunner(),        // ui
      thread_host.io_thread->GetTaskRunner()         // io
  );
  auto shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();

  ValidateDestroyPlatformView(shell.get());

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell), task_runners);
#endif  // OS_FUCHSIA
}

TEST_F(ShellTest, GetUsedThisFrameShouldBeSetBeforeEndFrame) {
  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  std::shared_ptr<ShellTestExternalViewEmbedder> external_view_embedder;
  bool used_this_frame = true;
  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        // We expect `used_this_frame` to be false.
        used_this_frame = external_view_embedder->GetUsedThisFrame();
        end_frame_latch.Signal();
      };
  external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSuccess, true);
  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };
  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  end_frame_latch.Wait();
  ASSERT_FALSE(used_this_frame);

  // Validate the platform view can be recreated and destroyed again
  ValidateShell(shell.get());

  DestroyShell(std::move(shell));
}

// TODO(https://github.com/flutter/flutter/issues/66056): Deflake on all other
// platforms
TEST_F(ShellTest, DISABLED_SkipAndSubmitFrame) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "RasterThreadMerger flakes on Fuchsia. "
                  "https://github.com/flutter/flutter/issues/59816 ";
#else

  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent end_frame_latch;
  std::shared_ptr<ShellTestExternalViewEmbedder> external_view_embedder;

  auto end_frame_callback =
      [&](bool should_resubmit_frame,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        if (should_resubmit_frame && !raster_thread_merger->IsMerged()) {
          raster_thread_merger->MergeWithLease(10);
          external_view_embedder->UpdatePostPrerollResult(
              PostPrerollResult::kSuccess);
        }
        end_frame_latch.Signal();
      };
  external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      end_frame_callback, PostPrerollResult::kSkipAndRetryFrame, true);

  auto shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));

  ASSERT_EQ(0, external_view_embedder->GetSubmittedFrameCount());

  PumpOneFrame(shell.get());

  // `EndFrame` changed the post preroll result to `kSuccess`.
  end_frame_latch.Wait();

  // Let the resubmitted frame to run and `GetSubmittedFrameCount` should be
  // called.
  end_frame_latch.Wait();
  // 2 frames are submitted because `kSkipAndRetryFrame`, but only the 2nd frame
  // should be submitted with `external_view_embedder`, hence the below check.
  ASSERT_EQ(1, external_view_embedder->GetSubmittedFrameCount());

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

TEST(SettingsTest, FrameTimingSetsAndGetsProperly) {
  // Ensure that all phases are in kPhases.
  ASSERT_EQ(sizeof(FrameTiming::kPhases),
            FrameTiming::kCount * sizeof(FrameTiming::Phase));

  int lastPhaseIndex = -1;
  FrameTiming timing;
  for (auto phase : FrameTiming::kPhases) {
    ASSERT_TRUE(phase > lastPhaseIndex);  // Ensure that kPhases are in order.
    lastPhaseIndex = phase;
    auto fake_time =
        fml::TimePoint::FromEpochDelta(fml::TimeDelta::FromMicroseconds(phase));
    timing.Set(phase, fake_time);
    ASSERT_TRUE(timing.Get(phase) == fake_time);
  }
}

TEST_F(ShellTest, ReportTimingsIsCalledImmediatelyAfterTheFirstFrame) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("reportTimingsMain");
  fml::AutoResetWaitableEvent reportLatch;
  std::vector<int64_t> timestamps;
  auto nativeTimingCallback = [&reportLatch,
                               &timestamps](Dart_NativeArguments args) {
    Dart_Handle exception = nullptr;
    ASSERT_EQ(timestamps.size(), 0ul);
    timestamps = tonic::DartConverter<std::vector<int64_t>>::FromArguments(
        args, 0, exception);
    reportLatch.Signal();
  };
  AddNativeCallback("NativeReportTimingsCallback",
                    CREATE_NATIVE_ENTRY(nativeTimingCallback));
  ASSERT_TRUE(configuration.IsValid());
  RunEngine(shell.get(), std::move(configuration));

  for (int i = 0; i < 10; i += 1) {
    PumpOneFrame(shell.get());
  }

  reportLatch.Wait();
  DestroyShell(std::move(shell));

  // Check for the immediate callback of the first frame that doesn't wait for
  // the other 9 frames to be rasterized.
  ASSERT_EQ(timestamps.size(), FrameTiming::kCount);
}

TEST_F(ShellTest, WaitForFirstFrame) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  fml::Status result = shell->WaitForFirstFrame(fml::TimeDelta::Max());
  ASSERT_TRUE(result.ok());

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, WaitForFirstFrameZeroSizeFrame) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get(), ViewContent::DummyView({1.0, 0.0, 0.0, 22, 0}));
  fml::Status result = shell->WaitForFirstFrame(fml::TimeDelta::Zero());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.message(), "timeout");
  EXPECT_EQ(result.code(), fml::StatusCode::kDeadlineExceeded);

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, WaitForFirstFrameTimeout) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  fml::Status result = shell->WaitForFirstFrame(fml::TimeDelta::Zero());
  ASSERT_FALSE(result.ok());
  ASSERT_EQ(result.code(), fml::StatusCode::kDeadlineExceeded);

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, WaitForFirstFrameMultiple) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  fml::Status result = shell->WaitForFirstFrame(fml::TimeDelta::Max());
  ASSERT_TRUE(result.ok());
  for (int i = 0; i < 100; ++i) {
    result = shell->WaitForFirstFrame(fml::TimeDelta::Zero());
    ASSERT_TRUE(result.ok());
  }

  DestroyShell(std::move(shell));
}

/// Makes sure that WaitForFirstFrame works if we rendered a frame with the
/// single-thread setup.
TEST_F(ShellTest, WaitForFirstFrameInlined) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  fml::AutoResetWaitableEvent event;
  task_runner->PostTask([&shell, &event] {
    fml::Status result = shell->WaitForFirstFrame(fml::TimeDelta::Max());
    ASSERT_FALSE(result.ok());
    ASSERT_EQ(result.code(), fml::StatusCode::kFailedPrecondition);
    event.Signal();
  });
  ASSERT_FALSE(event.WaitWithTimeout(fml::TimeDelta::Max()));

  DestroyShell(std::move(shell), task_runners);
}

static size_t GetRasterizerResourceCacheBytesSync(const Shell& shell) {
  size_t bytes = 0;
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell.GetTaskRunners().GetRasterTaskRunner(), [&]() {
        if (auto rasterizer = shell.GetRasterizer()) {
          bytes = rasterizer->GetResourceCacheMaxBytes().value_or(0U);
        }
        latch.Signal();
      });
  latch.Wait();
  return bytes;
}

TEST_F(ShellTest, MultipleFluttersSetResourceCacheBytes) {
  TaskRunners task_runners = GetTaskRunnersForFixture();
  auto settings = CreateSettingsForFixture();
  settings.resource_cache_max_bytes_threshold = 4000000U;
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  Shell::CreateCallback<PlatformView> platform_view_create_callback =
      [task_runners, main_context](flutter::Shell& shell) {
        auto result = std::make_unique<TestPlatformView>(shell, task_runners);
        ON_CALL(*result, CreateRenderingSurface())
            .WillByDefault(::testing::Invoke([main_context] {
              auto surface = std::make_unique<MockSurface>();
              ON_CALL(*surface, GetContext())
                  .WillByDefault(Return(main_context.get()));
              ON_CALL(*surface, IsValid()).WillByDefault(Return(true));
              ON_CALL(*surface, MakeRenderContextCurrent())
                  .WillByDefault(::testing::Invoke([] {
                    return std::make_unique<GLContextDefaultResult>(true);
                  }));
              return surface;
            }));
        return result;
      };

  auto shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
      .platform_view_create_callback = platform_view_create_callback,
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
    shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                 {1.0, 100, 100, 22, 0});
  });

  // first cache bytes
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(480000U));

  auto shell_spawn_callback = [&]() {
    std::unique_ptr<Shell> spawn;
    PostSync(
        shell->GetTaskRunners().GetPlatformTaskRunner(),
        [this, &spawn, &spawner = shell, platform_view_create_callback]() {
          auto configuration =
              RunConfiguration::InferFromSettings(CreateSettingsForFixture());
          configuration.SetEntrypoint("emptyMain");
          spawn = spawner->Spawn(
              std::move(configuration), "", platform_view_create_callback,
              [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
          ASSERT_NE(nullptr, spawn.get());
          ASSERT_TRUE(ValidateShell(spawn.get()));
        });
    return spawn;
  };

  std::unique_ptr<Shell> second_shell = shell_spawn_callback();
  PlatformViewNotifyCreated(second_shell.get());
  PostSync(second_shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&second_shell]() {
             second_shell->GetPlatformView()->SetViewportMetrics(
                 kImplicitViewId, {1.0, 100, 100, 22, 0});
           });
  // first cache bytes + second cache bytes
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(960000U));

  PostSync(second_shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&second_shell]() {
             second_shell->GetPlatformView()->SetViewportMetrics(
                 kImplicitViewId, {1.0, 100, 300, 22, 0});
           });
  // first cache bytes + second cache bytes
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(1920000U));

  std::unique_ptr<Shell> third_shell = shell_spawn_callback();
  PlatformViewNotifyCreated(third_shell.get());
  PostSync(third_shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&third_shell]() {
             third_shell->GetPlatformView()->SetViewportMetrics(
                 kImplicitViewId, {1.0, 400, 100, 22, 0});
           });
  // first cache bytes + second cache bytes + third cache bytes
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(3840000U));

  PostSync(third_shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&third_shell]() {
             third_shell->GetPlatformView()->SetViewportMetrics(
                 kImplicitViewId, {1.0, 800, 100, 22, 0});
           });
  // max bytes threshold
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(4000000U));
  DestroyShell(std::move(third_shell), task_runners);
  // max bytes threshold
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(4000000U));

  PostSync(second_shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&second_shell]() {
             second_shell->GetPlatformView()->SetViewportMetrics(
                 kImplicitViewId, {1.0, 100, 100, 22, 0});
           });
  // first cache bytes + second cache bytes
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(960000U));

  DestroyShell(std::move(second_shell), task_runners);
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, SetResourceCacheSize) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  // The Vulkan and GL backends set different default values for the resource
  // cache size. The default backend (specified by the default param of
  // `CreateShell` in this test) will only resolve to Vulkan (in
  // `ShellTestPlatformView::Create`) if GL is disabled. This situation arises
  // when targeting the Fuchsia Emulator.
#if defined(SHELL_ENABLE_VULKAN) && !defined(SHELL_ENABLE_GL)
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            vulkan::kGrCacheMaxByteSize);
#elif defined(SHELL_ENABLE_METAL)
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(256 * (1 << 20)));
#else
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(24 * (1 << 20)));
#endif

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
        shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                     {1.0, 400, 200, 22, 0});
      });
  PumpOneFrame(shell.get());

  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell), 3840000U);

  std::string request_json = R"json({
                                "method": "Skia.setResourceCacheMaxBytes",
                                "args": 10000
                              })json";
  auto data =
      fml::MallocMapping::Copy(request_json.c_str(), request_json.length());
  auto platform_message = std::make_unique<PlatformMessage>(
      "flutter/skia", std::move(data), nullptr);
  SendEnginePlatformMessage(shell.get(), std::move(platform_message));
  PumpOneFrame(shell.get());
  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell), 10000U);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
        shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                     {1.0, 800, 400, 22, 0});
      });
  PumpOneFrame(shell.get());

  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell), 10000U);
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, SetResourceCacheSizeEarly) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
        shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                     {1.0, 400, 200, 22, 0});
      });
  PumpOneFrame(shell.get());

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(3840000U));
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, SetResourceCacheSizeNotifiesDart) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
        shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                     {1.0, 400, 200, 22, 0});
      });
  PumpOneFrame(shell.get());

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testSkiaResourceCacheSendsResponse");

  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(3840000U));

  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&latch](auto args) {
                      latch.Signal();
                    }));

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  EXPECT_EQ(GetRasterizerResourceCacheBytesSync(*shell),
            static_cast<size_t>(10000U));
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, CanCreateImagefromDecompressedBytes) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();

  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("canCreateImageFromDecompressedData");

  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("NotifyWidthHeight",
                    CREATE_NATIVE_ENTRY([&latch](auto args) {
                      auto width = tonic::DartConverter<int>::FromDart(
                          Dart_GetNativeArgument(args, 0));
                      auto height = tonic::DartConverter<int>::FromDart(
                          Dart_GetNativeArgument(args, 1));
                      ASSERT_EQ(width, 10);
                      ASSERT_EQ(height, 10);
                      latch.Signal();
                    }));

  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

class MockTexture : public Texture {
 public:
  MockTexture(int64_t textureId,
              std::shared_ptr<fml::AutoResetWaitableEvent> latch)
      : Texture(textureId), latch_(std::move(latch)) {}

  ~MockTexture() override = default;

  // Called from raster thread.
  void Paint(PaintContext& context,
             const SkRect& bounds,
             bool freeze,
             const DlImageSampling) override {}

  void OnGrContextCreated() override {}

  void OnGrContextDestroyed() override {}

  void MarkNewFrameAvailable() override {
    frames_available_++;
    latch_->Signal();
  }

  void OnTextureUnregistered() override {
    unregistered_ = true;
    latch_->Signal();
  }

  bool unregistered() { return unregistered_; }
  int frames_available() { return frames_available_; }

 private:
  bool unregistered_ = false;
  int frames_available_ = 0;
  std::shared_ptr<fml::AutoResetWaitableEvent> latch_;
};

TEST_F(ShellTest, TextureFrameMarkedAvailableAndUnregister) {
  Settings settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(ValidateShell(shell.get()));
  PlatformViewNotifyCreated(shell.get());

  RunEngine(shell.get(), std::move(configuration));

  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();

  std::shared_ptr<MockTexture> mockTexture =
      std::make_shared<MockTexture>(0, latch);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(), [&]() {
        shell->GetPlatformView()->RegisterTexture(mockTexture);
        shell->GetPlatformView()->MarkTextureFrameAvailable(0);
      });
  latch->Wait();

  EXPECT_EQ(mockTexture->frames_available(), 1);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(),
      [&]() { shell->GetPlatformView()->UnregisterTexture(0); });
  latch->Wait();

  EXPECT_EQ(mockTexture->unregistered(), true);
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, IsolateCanAccessPersistentIsolateData) {
  const std::string message = "dummy isolate launch data.";

  Settings settings = CreateSettingsForFixture();
  settings.persistent_isolate_data =
      std::make_shared<fml::DataMapping>(message);
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  fml::AutoResetWaitableEvent message_latch;
  AddNativeCallback("NotifyMessage",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      const auto message_from_dart =
                          tonic::DartConverter<std::string>::FromDart(
                              Dart_GetNativeArgument(args, 0));
                      ASSERT_EQ(message, message_from_dart);
                      message_latch.Signal();
                    }));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("canAccessIsolateLaunchData");

  fml::AutoResetWaitableEvent event;
  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, CanScheduleFrameFromPlatform) {
  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners = GetTaskRunnersForFixture();
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback(
      "NotifyNative",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));
  fml::AutoResetWaitableEvent check_latch;
  AddNativeCallback("NativeOnBeginFrame",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      check_latch.Signal();
                    }));
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell->IsSetup());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("onBeginFrameWithNotifyNativeMain");
  RunEngine(shell.get(), std::move(configuration));

  // Wait for the application to attach the listener.
  latch.Wait();

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [&shell]() { shell->GetPlatformView()->ScheduleFrame(); });
  check_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, SecondaryVsyncCallbackShouldBeCalledAfterVsyncCallback) {
  bool is_on_begin_frame_called = false;
  bool is_secondary_callback_called = false;
  bool test_started = false;
  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners = GetTaskRunnersForFixture();
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback(
      "NotifyNative",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));
  fml::CountDownLatch count_down_latch(2);
  AddNativeCallback("NativeOnBeginFrame",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      if (!test_started) {
                        return;
                      }
                      EXPECT_FALSE(is_on_begin_frame_called);
                      EXPECT_FALSE(is_secondary_callback_called);
                      is_on_begin_frame_called = true;
                      count_down_latch.CountDown();
                    }));
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });
  ASSERT_TRUE(shell->IsSetup());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("onBeginFrameWithNotifyNativeMain");
  RunEngine(shell.get(), std::move(configuration));

  // Wait for the application to attach the listener.
  latch.Wait();

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetUITaskRunner(), [&]() {
        shell->GetEngine()->ScheduleSecondaryVsyncCallback(0, [&]() {
          if (!test_started) {
            return;
          }
          EXPECT_TRUE(is_on_begin_frame_called);
          EXPECT_FALSE(is_secondary_callback_called);
          is_secondary_callback_called = true;
          count_down_latch.CountDown();
        });
        shell->GetEngine()->ScheduleFrame();
        test_started = true;
      });
  count_down_latch.Wait();
  EXPECT_TRUE(is_on_begin_frame_called);
  EXPECT_TRUE(is_secondary_callback_called);
  DestroyShell(std::move(shell), task_runners);
}

static void LogSkData(const sk_sp<SkData>& data, const char* title) {
  FML_LOG(ERROR) << "---------- " << title;
  std::ostringstream ostr;
  for (size_t i = 0; i < data->size();) {
    ostr << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(data->bytes()[i]) << " ";
    i++;
    if (i % 16 == 0 || i == data->size()) {
      FML_LOG(ERROR) << ostr.str();
      ostr.str("");
      ostr.clear();
    }
  }
}

TEST_F(ShellTest, Screenshot) {
  auto settings = CreateSettingsForFixture();
  fml::AutoResetWaitableEvent firstFrameLatch;
  settings.frame_rasterized_callback =
      [&firstFrameLatch](const FrameTiming& t) { firstFrameLatch.Signal(); };

  std::unique_ptr<Shell> shell = CreateShell(settings);

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  LayerTreeBuilder builder = [&](const std::shared_ptr<ContainerLayer>& root) {
    auto display_list_layer = std::make_shared<DisplayListLayer>(
        DlPoint(10, 10), MakeSizedDisplayList(80, 80), false, false);
    root->Add(display_list_layer);
  };

  PumpOneFrame(shell.get(), ViewContent::ImplicitView(100, 100, builder));
  firstFrameLatch.Wait();

  std::promise<Rasterizer::Screenshot> screenshot_promise;
  auto screenshot_future = screenshot_promise.get_future();

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(),
      [&screenshot_promise, &shell]() {
        auto rasterizer = shell->GetRasterizer();
        screenshot_promise.set_value(rasterizer->ScreenshotLastLayerTree(
            Rasterizer::ScreenshotType::CompressedImage, false));
      });

  auto fixtures_dir =
      fml::OpenDirectory(GetFixturesPath(), false, fml::FilePermission::kRead);

  auto reference_png = fml::FileMapping::CreateReadOnly(
      fixtures_dir, "shelltest_screenshot.png");

  // Use MakeWithoutCopy instead of MakeWithCString because we don't want to
  // encode the null sentinel
  sk_sp<SkData> reference_data = SkData::MakeWithoutCopy(
      reference_png->GetMapping(), reference_png->GetSize());

  sk_sp<SkData> screenshot_data = screenshot_future.get().data;
  if (!reference_data->equals(screenshot_data.get())) {
    LogSkData(reference_data, "reference");
    LogSkData(screenshot_data, "screenshot");
    ASSERT_TRUE(false);
  }

  DestroyShell(std::move(shell));
}

// Compares local times as seen by the dart isolate and as seen by this test
// fixture, to a resolution of 1 hour.
//
// This verifies that (1) the isolate is able to get a timezone (doesn't lock
// up for example), and (2) that the host and the isolate agree on what the
// timezone is.
TEST_F(ShellTest, LocaltimesMatch) {
  fml::AutoResetWaitableEvent latch;
  std::string dart_isolate_time_str;

  // See fixtures/shell_test.dart, the callback NotifyLocalTime is declared
  // there.
  AddNativeCallback("NotifyLocalTime", CREATE_NATIVE_ENTRY([&](auto args) {
                      dart_isolate_time_str =
                          tonic::DartConverter<std::string>::FromDart(
                              Dart_GetNativeArgument(args, 0));
                      latch.Signal();
                    }));

  auto settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("localtimesMatch");
  std::unique_ptr<Shell> shell = CreateShell(settings);
  ASSERT_NE(shell.get(), nullptr);
  RunEngine(shell.get(), std::move(configuration));
  latch.Wait();

  char timestr[200];
  const time_t timestamp = time(nullptr);
  const struct tm* local_time = localtime(&timestamp);
  ASSERT_NE(local_time, nullptr)
      << "Could not get local time: errno=" << errno << ": " << strerror(errno);
  // Example: "2020-02-26 14" for 2pm on February 26, 2020.
  const size_t format_size =
      strftime(timestr, sizeof(timestr), "%Y-%m-%d %H", local_time);
  ASSERT_NE(format_size, 0UL)
      << "strftime failed: host time: " << std::string(timestr)
      << " dart isolate time: " << dart_isolate_time_str;

  const std::string host_local_time_str = timestr;

  ASSERT_EQ(dart_isolate_time_str, host_local_time_str)
      << "Local times in the dart isolate and the local time seen by the test "
      << "differ by more than 1 hour, but are expected to be about equal";

  DestroyShell(std::move(shell));
}

/// An image generator that always creates a 1x1 single-frame green image.
class SinglePixelImageGenerator : public ImageGenerator {
 public:
  SinglePixelImageGenerator()
      : info_(SkImageInfo::MakeN32(1, 1, SkAlphaType::kOpaque_SkAlphaType)){};
  ~SinglePixelImageGenerator() = default;
  const SkImageInfo& GetInfo() { return info_; }

  unsigned int GetFrameCount() const { return 1; }

  unsigned int GetPlayCount() const { return 1; }

  const ImageGenerator::FrameInfo GetFrameInfo(unsigned int frame_index) {
    return {std::nullopt, 0, SkCodecAnimation::DisposalMethod::kKeep};
  }

  SkISize GetScaledDimensions(float scale) {
    return SkISize::Make(info_.width(), info_.height());
  }

  bool GetPixels(const SkImageInfo& info,
                 void* pixels,
                 size_t row_bytes,
                 unsigned int frame_index,
                 std::optional<unsigned int> prior_frame) {
    assert(info.width() == 1);
    assert(info.height() == 1);
    assert(row_bytes == 4);

    reinterpret_cast<uint32_t*>(pixels)[0] = 0x00ff00ff;
    return true;
  };

 private:
  SkImageInfo info_;
};

TEST_F(ShellTest, CanRegisterImageDecoders) {
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("NotifyWidthHeight", CREATE_NATIVE_ENTRY([&](auto args) {
                      auto width = tonic::DartConverter<int>::FromDart(
                          Dart_GetNativeArgument(args, 0));
                      auto height = tonic::DartConverter<int>::FromDart(
                          Dart_GetNativeArgument(args, 1));
                      ASSERT_EQ(width, 1);
                      ASSERT_EQ(height, 1);
                      latch.Signal();
                    }));

  auto settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("canRegisterImageDecoders");
  std::unique_ptr<Shell> shell = CreateShell(settings);
  ASSERT_NE(shell.get(), nullptr);

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell]() {
        shell->RegisterImageDecoder(
            [](const sk_sp<SkData>& buffer) {
              return std::make_unique<SinglePixelImageGenerator>();
            },
            100);
      });

  RunEngine(shell.get(), std::move(configuration));
  latch.Wait();
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, OnServiceProtocolGetSkSLsWorks) {
  fml::ScopedTemporaryDirectory base_dir;
  ASSERT_TRUE(base_dir.fd().is_valid());
  PersistentCache::SetCacheDirectoryPath(base_dir.path());
  PersistentCache::ResetCacheForProcess();

  // Create 2 dummy SkSL cache file IE (base32 encoding of A), II (base32
  // encoding of B) with content x and y.
  std::vector<std::string> components = {
      "flutter_engine", GetFlutterEngineVersion(), "skia", GetSkiaVersion(),
      PersistentCache::kSkSLSubdirName};
  auto sksl_dir = fml::CreateDirectory(base_dir.fd(), components,
                                       fml::FilePermission::kReadWrite);
  const std::string x_key_str = "A";
  const std::string x_value_str = "x";
  sk_sp<SkData> x_key =
      SkData::MakeWithCopy(x_key_str.data(), x_key_str.size());
  sk_sp<SkData> x_value =
      SkData::MakeWithCopy(x_value_str.data(), x_value_str.size());
  auto x_data = PersistentCache::BuildCacheObject(*x_key, *x_value);

  const std::string y_key_str = "B";
  const std::string y_value_str = "y";
  sk_sp<SkData> y_key =
      SkData::MakeWithCopy(y_key_str.data(), y_key_str.size());
  sk_sp<SkData> y_value =
      SkData::MakeWithCopy(y_value_str.data(), y_value_str.size());
  auto y_data = PersistentCache::BuildCacheObject(*y_key, *y_value);

  ASSERT_TRUE(fml::WriteAtomically(sksl_dir, "x_cache", *x_data));
  ASSERT_TRUE(fml::WriteAtomically(sksl_dir, "y_cache", *y_data));

  Settings settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);
  ServiceProtocol::Handler::ServiceProtocolMap empty_params;
  rapidjson::Document document;
  OnServiceProtocol(shell.get(), ServiceProtocolEnum::kGetSkSLs,
                    shell->GetTaskRunners().GetIOTaskRunner(), empty_params,
                    &document);
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  DestroyShell(std::move(shell));

  const std::string expected_json1 =
      "{\"type\":\"GetSkSLs\",\"SkSLs\":{\"II\":\"eQ==\",\"IE\":\"eA==\"}}";
  const std::string expected_json2 =
      "{\"type\":\"GetSkSLs\",\"SkSLs\":{\"IE\":\"eA==\",\"II\":\"eQ==\"}}";
  bool json_is_expected = (expected_json1 == buffer.GetString()) ||
                          (expected_json2 == buffer.GetString());
  ASSERT_TRUE(json_is_expected) << buffer.GetString() << " is not equal to "
                                << expected_json1 << " or " << expected_json2;
}

TEST_F(ShellTest, RasterizerScreenshot) {
  Settings settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(ValidateShell(shell.get()));
  PlatformViewNotifyCreated(shell.get());

  RunEngine(shell.get(), std::move(configuration));

  auto latch = std::make_shared<fml::AutoResetWaitableEvent>();

  PumpOneFrame(shell.get());

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(), [&shell, &latch]() {
        Rasterizer::Screenshot screenshot =
            shell->GetRasterizer()->ScreenshotLastLayerTree(
                Rasterizer::ScreenshotType::CompressedImage, true);
        EXPECT_NE(screenshot.data, nullptr);

        latch->Signal();
      });
  latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, RasterizerMakeRasterSnapshot) {
  Settings settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(ValidateShell(shell.get()));
  PlatformViewNotifyCreated(shell.get());

  RunEngine(shell.get(), std::move(configuration));

  auto latch = std::make_shared<fml::AutoResetWaitableEvent>();

  PumpOneFrame(shell.get());

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetRasterTaskRunner(), [&shell, &latch]() {
        SnapshotDelegate* delegate =
            reinterpret_cast<Rasterizer*>(shell->GetRasterizer().get());
        sk_sp<DlImage> image = delegate->MakeRasterSnapshotSync(
            MakeSizedDisplayList(50, 50), SkISize::Make(50, 50));
        EXPECT_NE(image, nullptr);

        latch->Signal();
      });
  latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, OnServiceProtocolEstimateRasterCacheMemoryWorks) {
  Settings settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  // 1. Construct a picture and a picture layer to be raster cached.
  sk_sp<DisplayList> display_list = MakeSizedDisplayList(10, 10);
  auto display_list_layer = std::make_shared<DisplayListLayer>(
      DlPoint(0, 0), MakeSizedDisplayList(100, 100), false, false);
  display_list_layer->set_paint_bounds(DlRect::MakeWH(100, 100));

  // 2. Rasterize the picture and the picture layer in the raster cache.
  std::promise<bool> rasterized;

  shell->GetTaskRunners().GetRasterTaskRunner()->PostTask(
      [&shell, &rasterized, &display_list, &display_list_layer] {
        std::vector<RasterCacheItem*> raster_cache_items;
        auto* compositor_context = shell->GetRasterizer()->compositor_context();
        auto& raster_cache = compositor_context->raster_cache();

        LayerStateStack state_stack;
        FixedRefreshRateStopwatch raster_time;
        FixedRefreshRateStopwatch ui_time;
        PaintContext paint_context = {
            // clang-format off
            .state_stack                   = state_stack,
            .canvas                        = nullptr,
            .gr_context                    = nullptr,
            .dst_color_space               = nullptr,
            .view_embedder                 = nullptr,
            .raster_time                   = raster_time,
            .ui_time                       = ui_time,
            .texture_registry              = nullptr,
            .raster_cache                  = &raster_cache,
            // clang-format on
        };

        PrerollContext preroll_context = {
            // clang-format off
            .raster_cache                  = &raster_cache,
            .gr_context                    = nullptr,
            .view_embedder                 = nullptr,
            .state_stack                   = state_stack,
            .dst_color_space               = nullptr,
            .surface_needs_readback        = false,
            .raster_time                   = raster_time,
            .ui_time                       = ui_time,
            .texture_registry              = nullptr,
            .has_platform_view             = false,
            .has_texture_layer             = false,
            .raster_cached_entries         = &raster_cache_items,
            // clang-format on
        };

        // 2.1. Rasterize the picture. Call Draw multiple times to pass the
        // access threshold (default to 3) so a cache can be generated.
        DisplayListBuilder dummy_canvas;
        DlPaint paint;
        bool picture_cache_generated;
        DisplayListRasterCacheItem display_list_raster_cache_item(
            display_list, SkPoint(), true, false);
        for (int i = 0; i < 4; i += 1) {
          DlMatrix matrix;
          state_stack.set_preroll_delegate(matrix);
          display_list_raster_cache_item.PrerollSetup(&preroll_context, matrix);
          display_list_raster_cache_item.PrerollFinalize(&preroll_context,
                                                         matrix);
          picture_cache_generated =
              display_list_raster_cache_item.need_caching();
          state_stack.set_delegate(&dummy_canvas);
          display_list_raster_cache_item.TryToPrepareRasterCache(paint_context);
          display_list_raster_cache_item.Draw(paint_context, &dummy_canvas,
                                              &paint);
        }
        ASSERT_TRUE(picture_cache_generated);

        // 2.2. Rasterize the picture layer.
        LayerRasterCacheItem layer_raster_cache_item(display_list_layer.get());
        state_stack.set_preroll_delegate(DlMatrix());
        layer_raster_cache_item.PrerollSetup(&preroll_context, DlMatrix());
        layer_raster_cache_item.PrerollFinalize(&preroll_context, DlMatrix());
        state_stack.set_delegate(&dummy_canvas);
        layer_raster_cache_item.TryToPrepareRasterCache(paint_context);
        layer_raster_cache_item.Draw(paint_context, &dummy_canvas, &paint);
        rasterized.set_value(true);
      });
  rasterized.get_future().wait();

  // 3. Call the service protocol and check its output.
  ServiceProtocol::Handler::ServiceProtocolMap empty_params;
  rapidjson::Document document;
  OnServiceProtocol(
      shell.get(), ServiceProtocolEnum::kEstimateRasterCacheMemory,
      shell->GetTaskRunners().GetRasterTaskRunner(), empty_params, &document);
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::string expected_json =
      "{\"type\":\"EstimateRasterCacheMemory\",\"layerBytes\":40024,\"picture"
      "Bytes\":424}";
  std::string actual_json = buffer.GetString();
  ASSERT_EQ(actual_json, expected_json);

  DestroyShell(std::move(shell));
}

// TODO(https://github.com/flutter/flutter/issues/100273): Disabled due to
// flakiness.
// TODO(https://github.com/flutter/flutter/issues/100299): Fix it when
// re-enabling.
TEST_F(ShellTest, DISABLED_DiscardLayerTreeOnResize) {
  auto settings = CreateSettingsForFixture();

  SkISize wrong_size = SkISize::Make(400, 100);
  SkISize expected_size = SkISize::Make(400, 200);

  fml::AutoResetWaitableEvent end_frame_latch;
  auto end_frame_callback =
      [&](bool should_merge_thread,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        end_frame_latch.Signal();
      };
  auto external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      std::move(end_frame_callback), PostPrerollResult::kSuccess, false);
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [&shell, &expected_size]() {
        shell->GetPlatformView()->SetViewportMetrics(
            kImplicitViewId,
            {1.0, static_cast<double>(expected_size.width()),
             static_cast<double>(expected_size.height()), 22, 0});
      });

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  PumpOneFrame(shell.get(), ViewContent::DummyView(
                                static_cast<double>(wrong_size.width()),
                                static_cast<double>(wrong_size.height())));
  end_frame_latch.Wait();
  // Wrong size, no frames are submitted.
  ASSERT_EQ(0, external_view_embedder->GetSubmittedFrameCount());

  PumpOneFrame(shell.get(), ViewContent::DummyView(
                                static_cast<double>(expected_size.width()),
                                static_cast<double>(expected_size.height())));
  end_frame_latch.Wait();
  // Expected size, 1 frame submitted.
  ASSERT_EQ(1, external_view_embedder->GetSubmittedFrameCount());
  ASSERT_EQ(expected_size, external_view_embedder->GetLastSubmittedFrameSize());

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
}

// TODO(https://github.com/flutter/flutter/issues/100273): Disabled due to
// flakiness.
// TODO(https://github.com/flutter/flutter/issues/100299): Fix it when
// re-enabling.
TEST_F(ShellTest, DISABLED_DiscardResubmittedLayerTreeOnResize) {
  auto settings = CreateSettingsForFixture();

  SkISize origin_size = SkISize::Make(400, 100);
  SkISize new_size = SkISize::Make(400, 200);

  fml::AutoResetWaitableEvent end_frame_latch;

  fml::AutoResetWaitableEvent resize_latch;

  std::shared_ptr<ShellTestExternalViewEmbedder> external_view_embedder;
  fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger_ref;
  auto end_frame_callback =
      [&](bool should_merge_thread,
          const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
        if (!raster_thread_merger_ref) {
          raster_thread_merger_ref = raster_thread_merger;
        }
        if (should_merge_thread) {
          raster_thread_merger->MergeWithLease(10);
          external_view_embedder->UpdatePostPrerollResult(
              PostPrerollResult::kSuccess);
        }
        end_frame_latch.Signal();

        if (should_merge_thread) {
          resize_latch.Wait();
        }
      };

  external_view_embedder = std::make_shared<ShellTestExternalViewEmbedder>(
      std::move(end_frame_callback), PostPrerollResult::kResubmitFrame, true);

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .shell_test_external_view_embedder = external_view_embedder,
      }),
  });

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [&shell, &origin_size]() {
        shell->GetPlatformView()->SetViewportMetrics(
            kImplicitViewId,
            {1.0, static_cast<double>(origin_size.width()),
             static_cast<double>(origin_size.height()), 22, 0});
      });

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");

  RunEngine(shell.get(), std::move(configuration));

  PumpOneFrame(shell.get(), ViewContent::DummyView(
                                static_cast<double>(origin_size.width()),
                                static_cast<double>(origin_size.height())));

  end_frame_latch.Wait();
  ASSERT_EQ(0, external_view_embedder->GetSubmittedFrameCount());

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [&shell, &new_size, &resize_latch]() {
        shell->GetPlatformView()->SetViewportMetrics(
            kImplicitViewId, {1.0, static_cast<double>(new_size.width()),
                              static_cast<double>(new_size.height()), 22, 0});
        resize_latch.Signal();
      });

  end_frame_latch.Wait();

  // The frame resubmitted with origin size should be discarded after the
  // viewport metrics changed.
  ASSERT_EQ(0, external_view_embedder->GetSubmittedFrameCount());

  // Threads will be merged at the end of this frame.
  PumpOneFrame(shell.get(),
               ViewContent::DummyView(static_cast<double>(new_size.width()),
                                      static_cast<double>(new_size.height())));

  end_frame_latch.Wait();
  ASSERT_TRUE(raster_thread_merger_ref->IsMerged());
  ASSERT_EQ(1, external_view_embedder->GetSubmittedFrameCount());
  ASSERT_EQ(new_size, external_view_embedder->GetLastSubmittedFrameSize());

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, IgnoresInvalidMetrics) {
  fml::AutoResetWaitableEvent latch;
  double last_device_pixel_ratio;
  double last_width;
  double last_height;
  auto native_report_device_pixel_ratio = [&](Dart_NativeArguments args) {
    auto dpr_handle = Dart_GetNativeArgument(args, 0);
    ASSERT_TRUE(Dart_IsDouble(dpr_handle));
    Dart_DoubleValue(dpr_handle, &last_device_pixel_ratio);
    ASSERT_FALSE(last_device_pixel_ratio == 0.0);

    auto width_handle = Dart_GetNativeArgument(args, 1);
    ASSERT_TRUE(Dart_IsDouble(width_handle));
    Dart_DoubleValue(width_handle, &last_width);
    ASSERT_FALSE(last_width == 0.0);

    auto height_handle = Dart_GetNativeArgument(args, 2);
    ASSERT_TRUE(Dart_IsDouble(height_handle));
    Dart_DoubleValue(height_handle, &last_height);
    ASSERT_FALSE(last_height == 0.0);

    latch.Signal();
  };

  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  AddNativeCallback("ReportMetrics",
                    CREATE_NATIVE_ENTRY(native_report_device_pixel_ratio));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("reportMetrics");

  RunEngine(shell.get(), std::move(configuration));

  task_runner->PostTask([&]() {
    // This one is invalid for having 0 pixel ratio.
    shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                 {0.0, 400, 200, 22, 0});
    task_runner->PostTask([&]() {
      // This one is invalid for having 0 width.
      shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                   {0.8, 0.0, 200, 22, 0});
      task_runner->PostTask([&]() {
        // This one is invalid for having 0 height.
        shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                     {0.8, 400, 0.0, 22, 0});
        task_runner->PostTask([&]() {
          // This one makes it through.
          shell->GetPlatformView()->SetViewportMetrics(
              kImplicitViewId, {0.8, 400, 200.0, 22, 0});
        });
      });
    });
  });
  latch.Wait();
  ASSERT_EQ(last_device_pixel_ratio, 0.8);
  ASSERT_EQ(last_width, 400.0);
  ASSERT_EQ(last_height, 200.0);
  latch.Reset();

  task_runner->PostTask([&]() {
    shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                 {1.2, 600, 300, 22, 0});
  });
  latch.Wait();
  ASSERT_EQ(last_device_pixel_ratio, 1.2);
  ASSERT_EQ(last_width, 600.0);
  ASSERT_EQ(last_height, 300.0);

  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, IgnoresMetricsUpdateToInvalidView) {
  fml::AutoResetWaitableEvent latch;
  double last_device_pixel_ratio;
  // This callback will be called whenever any view's metrics change.
  auto native_report_device_pixel_ratio = [&](Dart_NativeArguments args) {
    // The correct call will have a DPR of 3.
    auto dpr_handle = Dart_GetNativeArgument(args, 0);
    ASSERT_TRUE(Dart_IsDouble(dpr_handle));
    Dart_DoubleValue(dpr_handle, &last_device_pixel_ratio);
    ASSERT_TRUE(last_device_pixel_ratio > 2.5);

    latch.Signal();
  };

  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  AddNativeCallback("ReportMetrics",
                    CREATE_NATIVE_ENTRY(native_report_device_pixel_ratio));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("reportMetrics");

  RunEngine(shell.get(), std::move(configuration));

  task_runner->PostTask([&]() {
    // This one is invalid for having an nonexistent view ID.
    // Also, it has a DPR of 2.0 for detection.
    shell->GetPlatformView()->SetViewportMetrics(2, {2.0, 400, 200, 22, 0});
    task_runner->PostTask([&]() {
      // This one is valid with DPR 3.0.
      shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId,
                                                   {3.0, 400, 200, 22, 0});
    });
  });
  latch.Wait();
  ASSERT_EQ(last_device_pixel_ratio, 3.0);
  latch.Reset();

  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, OnServiceProtocolSetAssetBundlePathWorks) {
  Settings settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);
  RunConfiguration configuration =
      RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("canAccessResourceFromAssetDir");

  // Verify isolate can load a known resource with the
  // default asset directory - kernel_blob.bin
  fml::AutoResetWaitableEvent latch;

  // Callback used to signal whether the resource was loaded successfully.
  bool can_access_resource = false;
  auto native_can_access_resource = [&can_access_resource,
                                     &latch](Dart_NativeArguments args) {
    Dart_Handle exception = nullptr;
    can_access_resource =
        tonic::DartConverter<bool>::FromArguments(args, 0, exception);
    latch.Signal();
  };
  AddNativeCallback("NotifyCanAccessResource",
                    CREATE_NATIVE_ENTRY(native_can_access_resource));

  // Callback used to delay the asset load until after the service
  // protocol method has finished.
  auto native_notify_set_asset_bundle_path =
      [&shell](Dart_NativeArguments args) {
        // Update the asset directory to a bonus path.
        ServiceProtocol::Handler::ServiceProtocolMap params;
        params["assetDirectory"] = "assetDirectory";
        rapidjson::Document document;
        OnServiceProtocol(shell.get(), ServiceProtocolEnum::kSetAssetBundlePath,
                          shell->GetTaskRunners().GetUITaskRunner(), params,
                          &document);
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);
      };
  AddNativeCallback("NotifySetAssetBundlePath",
                    CREATE_NATIVE_ENTRY(native_notify_set_asset_bundle_path));

  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();
  ASSERT_TRUE(can_access_resource);

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, EngineRootIsolateLaunchesDontTakeVMDataSettings) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  // Make sure the shell launch does not kick off the creation of the VM
  // instance by already creating one upfront.
  auto vm_settings = CreateSettingsForFixture();
  auto vm_ref = DartVMRef::Create(vm_settings);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());

  auto settings = vm_settings;
  fml::AutoResetWaitableEvent isolate_create_latch;
  settings.root_isolate_create_callback = [&](const auto& isolate) {
    isolate_create_latch.Signal();
  };
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));
  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  RunEngine(shell.get(), std::move(configuration));
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  DestroyShell(std::move(shell));
  isolate_create_latch.Wait();
}

TEST_F(ShellTest, AssetManagerSingle) {
  fml::ScopedTemporaryDirectory asset_dir;
  fml::UniqueFD asset_dir_fd = fml::OpenDirectory(
      asset_dir.path().c_str(), false, fml::FilePermission::kRead);

  std::string filename = "test_name";
  std::string content = "test_content";

  bool success = fml::WriteAtomically(asset_dir_fd, filename.c_str(),
                                      fml::DataMapping(content));
  ASSERT_TRUE(success);

  AssetManager asset_manager;
  asset_manager.PushBack(
      std::make_unique<DirectoryAssetBundle>(std::move(asset_dir_fd), false));

  auto mapping = asset_manager.GetAsMapping(filename);
  ASSERT_TRUE(mapping != nullptr);

  std::string result(reinterpret_cast<const char*>(mapping->GetMapping()),
                     mapping->GetSize());

  ASSERT_TRUE(result == content);
}

TEST_F(ShellTest, AssetManagerMulti) {
  fml::ScopedTemporaryDirectory asset_dir;
  fml::UniqueFD asset_dir_fd = fml::OpenDirectory(
      asset_dir.path().c_str(), false, fml::FilePermission::kRead);

  std::vector<std::string> filenames = {
      "good0",
      "bad0",
      "good1",
      "bad1",
  };

  for (const auto& filename : filenames) {
    bool success = fml::WriteAtomically(asset_dir_fd, filename.c_str(),
                                        fml::DataMapping(filename));
    ASSERT_TRUE(success);
  }

  AssetManager asset_manager;
  asset_manager.PushBack(
      std::make_unique<DirectoryAssetBundle>(std::move(asset_dir_fd), false));

  auto mappings = asset_manager.GetAsMappings("(.*)", std::nullopt);
  EXPECT_EQ(mappings.size(), 4u);

  std::vector<std::string> expected_results = {
      "good0",
      "good1",
  };

  mappings = asset_manager.GetAsMappings("(.*)good(.*)", std::nullopt);
  ASSERT_EQ(mappings.size(), expected_results.size());

  for (auto& mapping : mappings) {
    std::string result(reinterpret_cast<const char*>(mapping->GetMapping()),
                       mapping->GetSize());
    EXPECT_NE(
        std::find(expected_results.begin(), expected_results.end(), result),
        expected_results.end());
  }
}

#if defined(OS_FUCHSIA)
TEST_F(ShellTest, AssetManagerMultiSubdir) {
  std::string subdir_path = "subdir";

  fml::ScopedTemporaryDirectory asset_dir;
  fml::UniqueFD asset_dir_fd = fml::OpenDirectory(
      asset_dir.path().c_str(), false, fml::FilePermission::kRead);
  fml::UniqueFD subdir_fd =
      fml::OpenDirectory((asset_dir.path() + "/" + subdir_path).c_str(), true,
                         fml::FilePermission::kReadWrite);

  std::vector<std::string> filenames = {
      "bad0",
      "notgood",  // this is to make sure the pattern (.*)good(.*) only
                  // matches things in the subdirectory
  };

  std::vector<std::string> subdir_filenames = {
      "good0",
      "good1",
      "bad1",
  };

  for (auto filename : filenames) {
    bool success = fml::WriteAtomically(asset_dir_fd, filename.c_str(),
                                        fml::DataMapping(filename));
    ASSERT_TRUE(success);
  }

  for (auto filename : subdir_filenames) {
    bool success = fml::WriteAtomically(subdir_fd, filename.c_str(),
                                        fml::DataMapping(filename));
    ASSERT_TRUE(success);
  }

  AssetManager asset_manager;
  asset_manager.PushBack(
      std::make_unique<DirectoryAssetBundle>(std::move(asset_dir_fd), false));

  auto mappings = asset_manager.GetAsMappings("(.*)", std::nullopt);
  EXPECT_EQ(mappings.size(), 5u);

  mappings = asset_manager.GetAsMappings("(.*)", subdir_path);
  EXPECT_EQ(mappings.size(), 3u);

  std::vector<std::string> expected_results = {
      "good0",
      "good1",
  };

  mappings = asset_manager.GetAsMappings("(.*)good(.*)", subdir_path);
  ASSERT_EQ(mappings.size(), expected_results.size());

  for (auto& mapping : mappings) {
    std::string result(reinterpret_cast<const char*>(mapping->GetMapping()),
                       mapping->GetSize());
    ASSERT_NE(
        std::find(expected_results.begin(), expected_results.end(), result),
        expected_results.end());
  }
}
#endif  // OS_FUCHSIA

TEST_F(ShellTest, Spawn) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("fixturesAreFunctionalMain");

  auto second_configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(second_configuration.IsValid());
  second_configuration.SetEntrypoint("testCanLaunchSecondaryIsolate");

  const std::string initial_route("/foo");

  fml::AutoResetWaitableEvent main_latch;
  std::string last_entry_point;
  // Fulfill native function for the first Shell's entrypoint.
  AddNativeCallback(
      "SayHiFromFixturesAreFunctionalMain", CREATE_NATIVE_ENTRY([&](auto args) {
        last_entry_point = shell->GetEngine()->GetLastEntrypoint();
        main_latch.Signal();
      }));
  // Fulfill native function for the second Shell's entrypoint.
  fml::CountDownLatch second_latch(2);
  AddNativeCallback(
      // The Dart native function names aren't very consistent but this is
      // just the native function name of the second vm entrypoint in the
      // fixture.
      "NotifyNative",
      CREATE_NATIVE_ENTRY([&](auto args) { second_latch.CountDown(); }));

  RunEngine(shell.get(), std::move(configuration));
  main_latch.Wait();
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  // Check first Shell ran the first entrypoint.
  ASSERT_EQ("fixturesAreFunctionalMain", last_entry_point);

  PostSync(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [this, &spawner = shell, &second_configuration, &second_latch,
       initial_route]() {
        MockPlatformViewDelegate platform_view_delegate;
        auto spawn = spawner->Spawn(
            std::move(second_configuration), initial_route,
            [&platform_view_delegate](Shell& shell) {
              auto result = std::make_unique<MockPlatformView>(
                  platform_view_delegate, shell.GetTaskRunners());
              ON_CALL(*result, CreateRenderingSurface())
                  .WillByDefault(::testing::Invoke(
                      [] { return std::make_unique<MockSurface>(); }));
              return result;
            },
            [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
        ASSERT_NE(nullptr, spawn.get());
        ASSERT_TRUE(ValidateShell(spawn.get()));

        PostSync(spawner->GetTaskRunners().GetUITaskRunner(), [&spawn, &spawner,
                                                               initial_route] {
          // Check second shell ran the second entrypoint.
          ASSERT_EQ("testCanLaunchSecondaryIsolate",
                    spawn->GetEngine()->GetLastEntrypoint());
          ASSERT_EQ(initial_route, spawn->GetEngine()->InitialRoute());

          ASSERT_NE(spawner->GetEngine()
                        ->GetRuntimeController()
                        ->GetRootIsolateGroup(),
                    0u);
          ASSERT_EQ(spawner->GetEngine()
                        ->GetRuntimeController()
                        ->GetRootIsolateGroup(),
                    spawn->GetEngine()
                        ->GetRuntimeController()
                        ->GetRootIsolateGroup());
          auto spawner_snapshot_delegate = spawner->GetEngine()
                                               ->GetRuntimeController()
                                               ->GetSnapshotDelegate();
          auto spawn_snapshot_delegate =
              spawn->GetEngine()->GetRuntimeController()->GetSnapshotDelegate();
          PostSync(spawner->GetTaskRunners().GetRasterTaskRunner(),
                   [spawner_snapshot_delegate, spawn_snapshot_delegate] {
                     ASSERT_NE(spawner_snapshot_delegate.get(),
                               spawn_snapshot_delegate.get());
                   });
        });
        PostSync(
            spawner->GetTaskRunners().GetIOTaskRunner(), [&spawner, &spawn] {
              ASSERT_EQ(spawner->GetIOManager()->GetResourceContext().get(),
                        spawn->GetIOManager()->GetResourceContext().get());
            });

        // Before destroying the shell, wait for expectations of the spawned
        // isolate to be met.
        second_latch.Wait();

        DestroyShell(std::move(spawn));
      });

  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, SpawnWithDartEntrypointArgs) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("canReceiveArgumentsWhenEngineRun");
  const std::vector<std::string> entrypoint_args{"foo", "bar"};
  configuration.SetEntrypointArgs(entrypoint_args);

  auto second_configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(second_configuration.IsValid());
  second_configuration.SetEntrypoint("canReceiveArgumentsWhenEngineSpawn");
  const std::vector<std::string> second_entrypoint_args{"arg1", "arg2"};
  second_configuration.SetEntrypointArgs(second_entrypoint_args);

  const std::string initial_route("/foo");

  fml::AutoResetWaitableEvent main_latch;
  std::string last_entry_point;
  // Fulfill native function for the first Shell's entrypoint.
  AddNativeCallback("NotifyNativeWhenEngineRun",
                    CREATE_NATIVE_ENTRY(([&](Dart_NativeArguments args) {
                      ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(
                          Dart_GetNativeArgument(args, 0)));
                      last_entry_point =
                          shell->GetEngine()->GetLastEntrypoint();
                      main_latch.Signal();
                    })));

  fml::AutoResetWaitableEvent second_latch;
  // Fulfill native function for the second Shell's entrypoint.
  AddNativeCallback("NotifyNativeWhenEngineSpawn",
                    CREATE_NATIVE_ENTRY(([&](Dart_NativeArguments args) {
                      ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(
                          Dart_GetNativeArgument(args, 0)));
                      last_entry_point =
                          shell->GetEngine()->GetLastEntrypoint();
                      second_latch.Signal();
                    })));

  RunEngine(shell.get(), std::move(configuration));
  main_latch.Wait();
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  // Check first Shell ran the first entrypoint.
  ASSERT_EQ("canReceiveArgumentsWhenEngineRun", last_entry_point);

  PostSync(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [this, &spawner = shell, &second_configuration, &second_latch,
       initial_route]() {
        MockPlatformViewDelegate platform_view_delegate;
        auto spawn = spawner->Spawn(
            std::move(second_configuration), initial_route,
            [&platform_view_delegate](Shell& shell) {
              auto result = std::make_unique<MockPlatformView>(
                  platform_view_delegate, shell.GetTaskRunners());
              ON_CALL(*result, CreateRenderingSurface())
                  .WillByDefault(::testing::Invoke(
                      [] { return std::make_unique<MockSurface>(); }));
              return result;
            },
            [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
        ASSERT_NE(nullptr, spawn.get());
        ASSERT_TRUE(ValidateShell(spawn.get()));

        PostSync(spawner->GetTaskRunners().GetUITaskRunner(),
                 [&spawn, &spawner, initial_route] {
                   // Check second shell ran the second entrypoint.
                   ASSERT_EQ("canReceiveArgumentsWhenEngineSpawn",
                             spawn->GetEngine()->GetLastEntrypoint());
                   ASSERT_EQ(initial_route, spawn->GetEngine()->InitialRoute());

                   ASSERT_NE(spawner->GetEngine()
                                 ->GetRuntimeController()
                                 ->GetRootIsolateGroup(),
                             0u);
                   ASSERT_EQ(spawner->GetEngine()
                                 ->GetRuntimeController()
                                 ->GetRootIsolateGroup(),
                             spawn->GetEngine()
                                 ->GetRuntimeController()
                                 ->GetRootIsolateGroup());
                 });

        PostSync(
            spawner->GetTaskRunners().GetIOTaskRunner(), [&spawner, &spawn] {
              ASSERT_EQ(spawner->GetIOManager()->GetResourceContext().get(),
                        spawn->GetIOManager()->GetResourceContext().get());
            });

        // Before destroying the shell, wait for expectations of the spawned
        // isolate to be met.
        second_latch.Wait();

        DestroyShell(std::move(spawn));
      });

  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, IOManagerIsSharedBetweenParentAndSpawnedShell) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [this,
                                                             &spawner = shell,
                                                             &settings] {
    auto second_configuration = RunConfiguration::InferFromSettings(settings);
    ASSERT_TRUE(second_configuration.IsValid());
    second_configuration.SetEntrypoint("emptyMain");
    const std::string initial_route("/foo");
    MockPlatformViewDelegate platform_view_delegate;
    auto spawn = spawner->Spawn(
        std::move(second_configuration), initial_route,
        [&platform_view_delegate](Shell& shell) {
          auto result = std::make_unique<MockPlatformView>(
              platform_view_delegate, shell.GetTaskRunners());
          ON_CALL(*result, CreateRenderingSurface())
              .WillByDefault(::testing::Invoke(
                  [] { return std::make_unique<MockSurface>(); }));
          return result;
        },
        [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
    ASSERT_TRUE(ValidateShell(spawn.get()));

    PostSync(spawner->GetTaskRunners().GetIOTaskRunner(), [&spawner, &spawn] {
      ASSERT_NE(spawner->GetIOManager().get(), nullptr);
      ASSERT_EQ(spawner->GetIOManager().get(), spawn->GetIOManager().get());
    });

    // Destroy the child shell.
    DestroyShell(std::move(spawn));
  });
  // Destroy the parent shell.
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, IOManagerInSpawnedShellIsNotNullAfterParentShellDestroyed) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  PostSync(shell->GetTaskRunners().GetUITaskRunner(), [&shell] {
    // We must get engine on UI thread.
    auto runtime_controller = shell->GetEngine()->GetRuntimeController();
    PostSync(shell->GetTaskRunners().GetIOTaskRunner(),
             [&shell, &runtime_controller] {
               // We must get io_manager on IO thread.
               auto io_manager = runtime_controller->GetIOManager();
               // Check io_manager existence.
               ASSERT_NE(io_manager.get(), nullptr);
               ASSERT_NE(io_manager->GetSkiaUnrefQueue().get(), nullptr);
               // Get io_manager directly from shell and check its existence.
               ASSERT_NE(shell->GetIOManager().get(), nullptr);
             });
  });

  std::unique_ptr<Shell> spawn;

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell, &settings,
                                                             &spawn] {
    auto second_configuration = RunConfiguration::InferFromSettings(settings);
    ASSERT_TRUE(second_configuration.IsValid());
    second_configuration.SetEntrypoint("emptyMain");
    const std::string initial_route("/foo");
    MockPlatformViewDelegate platform_view_delegate;
    auto child = shell->Spawn(
        std::move(second_configuration), initial_route,
        [&platform_view_delegate](Shell& shell) {
          auto result = std::make_unique<MockPlatformView>(
              platform_view_delegate, shell.GetTaskRunners());
          ON_CALL(*result, CreateRenderingSurface())
              .WillByDefault(::testing::Invoke(
                  [] { return std::make_unique<MockSurface>(); }));
          return result;
        },
        [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
    spawn = std::move(child);
  });
  // Destroy the parent shell.
  DestroyShell(std::move(shell));

  PostSync(spawn->GetTaskRunners().GetUITaskRunner(), [&spawn] {
    // We must get engine on UI thread.
    auto runtime_controller = spawn->GetEngine()->GetRuntimeController();
    PostSync(spawn->GetTaskRunners().GetIOTaskRunner(),
             [&spawn, &runtime_controller] {
               // We must get io_manager on IO thread.
               auto io_manager = runtime_controller->GetIOManager();
               // Check io_manager existence here.
               ASSERT_NE(io_manager.get(), nullptr);
               ASSERT_NE(io_manager->GetSkiaUnrefQueue().get(), nullptr);
               // Get io_manager directly from shell and check its existence.
               ASSERT_NE(spawn->GetIOManager().get(), nullptr);
             });
  });
  // Destroy the child shell.
  DestroyShell(std::move(spawn));
}

TEST_F(ShellTest, ImageGeneratorRegistryNotNullAfterParentShellDestroyed) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  std::unique_ptr<Shell> spawn;

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell, &settings,
                                                             &spawn] {
    auto second_configuration = RunConfiguration::InferFromSettings(settings);
    ASSERT_TRUE(second_configuration.IsValid());
    second_configuration.SetEntrypoint("emptyMain");
    const std::string initial_route("/foo");
    MockPlatformViewDelegate platform_view_delegate;
    auto child = shell->Spawn(
        std::move(second_configuration), initial_route,
        [&platform_view_delegate](Shell& shell) {
          auto result = std::make_unique<MockPlatformView>(
              platform_view_delegate, shell.GetTaskRunners());
          ON_CALL(*result, CreateRenderingSurface())
              .WillByDefault(::testing::Invoke(
                  [] { return std::make_unique<MockSurface>(); }));
          return result;
        },
        [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
    spawn = std::move(child);
  });

  PostSync(spawn->GetTaskRunners().GetUITaskRunner(), [&spawn] {
    std::shared_ptr<const DartIsolate> isolate =
        spawn->GetEngine()->GetRuntimeController()->GetRootIsolate().lock();
    ASSERT_TRUE(isolate);
    ASSERT_TRUE(isolate->GetImageGeneratorRegistry());
  });

  // Destroy the parent shell.
  DestroyShell(std::move(shell));

  PostSync(spawn->GetTaskRunners().GetUITaskRunner(), [&spawn] {
    std::shared_ptr<const DartIsolate> isolate =
        spawn->GetEngine()->GetRuntimeController()->GetRootIsolate().lock();
    ASSERT_TRUE(isolate);
    ASSERT_TRUE(isolate->GetImageGeneratorRegistry());
  });
  // Destroy the child shell.
  DestroyShell(std::move(spawn));
}

TEST_F(ShellTest, UpdateAssetResolverByTypeReplaces) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  auto asset_manager = configuration.GetAssetManager();

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  auto platform_view =
      std::make_unique<PlatformView>(*shell.get(), task_runners);

  auto old_resolver = std::make_unique<TestAssetResolver>(
      true, AssetResolver::AssetResolverType::kApkAssetProvider);
  ASSERT_TRUE(old_resolver->IsValid());
  asset_manager->PushBack(std::move(old_resolver));

  auto updated_resolver = std::make_unique<TestAssetResolver>(
      false, AssetResolver::AssetResolverType::kApkAssetProvider);
  ASSERT_FALSE(updated_resolver->IsValidAfterAssetManagerChange());
  platform_view->UpdateAssetResolverByType(
      std::move(updated_resolver),
      AssetResolver::AssetResolverType::kApkAssetProvider);

  auto resolvers = asset_manager->TakeResolvers();
  ASSERT_EQ(resolvers.size(), 2ull);
  ASSERT_TRUE(resolvers[0]->IsValidAfterAssetManagerChange());

  ASSERT_FALSE(resolvers[1]->IsValidAfterAssetManagerChange());

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, UpdateAssetResolverByTypeAppends) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  auto asset_manager = configuration.GetAssetManager();

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  auto platform_view =
      std::make_unique<PlatformView>(*shell.get(), task_runners);

  auto updated_resolver = std::make_unique<TestAssetResolver>(
      false, AssetResolver::AssetResolverType::kApkAssetProvider);
  ASSERT_FALSE(updated_resolver->IsValidAfterAssetManagerChange());
  platform_view->UpdateAssetResolverByType(
      std::move(updated_resolver),
      AssetResolver::AssetResolverType::kApkAssetProvider);

  auto resolvers = asset_manager->TakeResolvers();
  ASSERT_EQ(resolvers.size(), 2ull);
  ASSERT_TRUE(resolvers[0]->IsValidAfterAssetManagerChange());

  ASSERT_FALSE(resolvers[1]->IsValidAfterAssetManagerChange());

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, UpdateAssetResolverByTypeNull) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform));
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  auto asset_manager = configuration.GetAssetManager();
  RunEngine(shell.get(), std::move(configuration));

  auto platform_view =
      std::make_unique<PlatformView>(*shell.get(), task_runners);

  auto old_resolver = std::make_unique<TestAssetResolver>(
      true, AssetResolver::AssetResolverType::kApkAssetProvider);
  ASSERT_TRUE(old_resolver->IsValid());
  asset_manager->PushBack(std::move(old_resolver));

  platform_view->UpdateAssetResolverByType(
      nullptr, AssetResolver::AssetResolverType::kApkAssetProvider);

  auto resolvers = asset_manager->TakeResolvers();
  ASSERT_EQ(resolvers.size(), 2ull);
  ASSERT_TRUE(resolvers[0]->IsValidAfterAssetManagerChange());
  ASSERT_TRUE(resolvers[1]->IsValidAfterAssetManagerChange());

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, UpdateAssetResolverByTypeDoesNotReplaceMismatchType) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  auto asset_manager = configuration.GetAssetManager();

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  auto platform_view =
      std::make_unique<PlatformView>(*shell.get(), task_runners);

  auto old_resolver = std::make_unique<TestAssetResolver>(
      true, AssetResolver::AssetResolverType::kAssetManager);
  ASSERT_TRUE(old_resolver->IsValid());
  asset_manager->PushBack(std::move(old_resolver));

  auto updated_resolver = std::make_unique<TestAssetResolver>(
      false, AssetResolver::AssetResolverType::kApkAssetProvider);
  ASSERT_FALSE(updated_resolver->IsValidAfterAssetManagerChange());
  platform_view->UpdateAssetResolverByType(
      std::move(updated_resolver),
      AssetResolver::AssetResolverType::kApkAssetProvider);

  auto resolvers = asset_manager->TakeResolvers();
  ASSERT_EQ(resolvers.size(), 3ull);
  ASSERT_TRUE(resolvers[0]->IsValidAfterAssetManagerChange());

  ASSERT_TRUE(resolvers[1]->IsValidAfterAssetManagerChange());

  ASSERT_FALSE(resolvers[2]->IsValidAfterAssetManagerChange());

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, CanCreateShellsWithGLBackend) {
#if !SHELL_ENABLE_GL
  // GL emulation does not exist on Fuchsia.
  GTEST_SKIP();
#else
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend = ShellTestPlatformView::BackendType::kGLBackend,
      }),
  });
  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // !SHELL_ENABLE_GL
}

TEST_F(ShellTest, CanCreateShellsWithVulkanBackend) {
#if !SHELL_ENABLE_VULKAN
  GTEST_SKIP();
#else
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend =
              ShellTestPlatformView::BackendType::kVulkanBackend,
      }),
  });
  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // !SHELL_ENABLE_VULKAN
}

TEST_F(ShellTest, CanCreateShellsWithMetalBackend) {
#if !SHELL_ENABLE_METAL
  GTEST_SKIP();
#else
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend =
              ShellTestPlatformView::BackendType::kMetalBackend,
      }),
  });
  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());
  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // !SHELL_ENABLE_METAL
}

TEST_F(ShellTest, UserTagSetOnStartup) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  // Make sure the shell launch does not kick off the creation of the VM
  // instance by already creating one upfront.
  auto vm_settings = CreateSettingsForFixture();
  auto vm_ref = DartVMRef::Create(vm_settings);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());

  auto settings = vm_settings;
  fml::AutoResetWaitableEvent isolate_create_latch;

  // ensure that "AppStartUpTag" is set during isolate creation.
  settings.root_isolate_create_callback = [&](const DartIsolate& isolate) {
    Dart_Handle current_tag = Dart_GetCurrentUserTag();
    Dart_Handle startup_tag = Dart_NewUserTag("AppStartUp");
    EXPECT_TRUE(Dart_IdentityEquals(current_tag, startup_tag));

    isolate_create_latch.Signal();
  };

  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());

  RunEngine(shell.get(), std::move(configuration));
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());

  DestroyShell(std::move(shell));
  isolate_create_latch.Wait();
}

TEST_F(ShellTest, PrefetchDefaultFontManager) {
  auto settings = CreateSettingsForFixture();
  settings.prefetched_default_font_manager = true;
  std::unique_ptr<Shell> shell;

  auto get_font_manager_count = [&] {
    fml::AutoResetWaitableEvent latch;
    size_t font_manager_count;
    fml::TaskRunner::RunNowOrPostTask(
        shell->GetTaskRunners().GetUITaskRunner(),
        [this, &shell, &latch, &font_manager_count]() {
          font_manager_count =
              GetFontCollection(shell.get())->GetFontManagersCount();
          latch.Signal();
        });
    latch.Wait();
    return font_manager_count;
  };
  size_t initial_font_manager_count = 0;
  settings.root_isolate_create_callback = [&](const auto& isolate) {
    ASSERT_GT(initial_font_manager_count, 0ul);
    // Should not have fetched the default font manager yet, since the root
    // isolate was only just created.
    ASSERT_EQ(get_font_manager_count(), initial_font_manager_count);
  };

  shell = CreateShell(settings);

  initial_font_manager_count = get_font_manager_count();

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));

  // If the prefetched_default_font_manager flag is set, then the default font
  // manager will not be added until the engine starts running.
  ASSERT_EQ(get_font_manager_count(), initial_font_manager_count + 1);

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, OnPlatformViewCreatedWhenUIThreadIsBusy) {
  // This test will deadlock if the threading logic in
  // Shell::OnCreatePlatformView is wrong.
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);

  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(shell->GetTaskRunners().GetUITaskRunner(),
                                    [&latch]() { latch.Wait(); });

  ShellTest::PlatformViewNotifyCreated(shell.get());
  latch.Signal();

  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, UIWorkAfterOnPlatformViewDestroyed) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("drawFrames");

  fml::AutoResetWaitableEvent latch;
  fml::AutoResetWaitableEvent notify_native_latch;
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&](auto args) {
                      notify_native_latch.Signal();
                      latch.Wait();
                    }));

  RunEngine(shell.get(), std::move(configuration));
  // Wait to make sure we get called back from Dart and thus have latched
  // the UI thread before we create/destroy the platform view.
  notify_native_latch.Wait();

  ShellTest::PlatformViewNotifyCreated(shell.get());

  fml::AutoResetWaitableEvent destroy_latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [&shell, &destroy_latch]() {
        shell->GetPlatformView()->NotifyDestroyed();
        destroy_latch.Signal();
      });

  destroy_latch.Wait();

  // Unlatch the UI thread and let it send us a scene to render.
  latch.Signal();

  // Flush the UI task runner to make sure we process the render/scheduleFrame
  // request.
  fml::AutoResetWaitableEvent ui_flush_latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetUITaskRunner(),
      [&ui_flush_latch]() { ui_flush_latch.Signal(); });
  ui_flush_latch.Wait();
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, UsesPlatformMessageHandler) {
  TaskRunners task_runners = GetTaskRunnersForFixture();
  auto settings = CreateSettingsForFixture();
  MockPlatformViewDelegate platform_view_delegate;
  auto platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  int message_id = 1;
  EXPECT_CALL(*platform_message_handler, HandlePlatformMessage(_));
  EXPECT_CALL(*platform_message_handler,
              InvokePlatformMessageEmptyResponseCallback(message_id));
  Shell::CreateCallback<PlatformView> platform_view_create_callback =
      [&platform_view_delegate, task_runners,
       platform_message_handler](flutter::Shell& shell) {
        auto result = std::make_unique<MockPlatformView>(platform_view_delegate,
                                                         task_runners);
        EXPECT_CALL(*result, GetPlatformMessageHandler())
            .WillOnce(Return(platform_message_handler));
        return result;
      };
  auto shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
      .platform_view_create_callback = platform_view_create_callback,
  });

  EXPECT_EQ(platform_message_handler, shell->GetPlatformMessageHandler());
  PostSync(task_runners.GetUITaskRunner(), [&shell]() {
    size_t data_size = 4;
    fml::MallocMapping bytes =
        fml::MallocMapping(static_cast<uint8_t*>(malloc(data_size)), data_size);
    fml::RefPtr<MockPlatformMessageResponse> response =
        MockPlatformMessageResponse::Create();
    auto message = std::make_unique<PlatformMessage>(
        /*channel=*/"foo", /*data=*/std::move(bytes), /*response=*/response);
    (static_cast<Engine::Delegate*>(shell.get()))
        ->OnEngineHandlePlatformMessage(std::move(message));
  });
  shell->GetPlatformMessageHandler()
      ->InvokePlatformMessageEmptyResponseCallback(message_id);
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, SpawnWorksWithOnError) {
  auto settings = CreateSettingsForFixture();
  auto shell = CreateShell(settings);
  ASSERT_TRUE(ValidateShell(shell.get()));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(configuration.IsValid());
  configuration.SetEntrypoint("onErrorA");

  auto second_configuration = RunConfiguration::InferFromSettings(settings);
  ASSERT_TRUE(second_configuration.IsValid());
  second_configuration.SetEntrypoint("onErrorB");

  fml::CountDownLatch latch(2);

  AddNativeCallback(
      "NotifyErrorA", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto string_handle = Dart_GetNativeArgument(args, 0);
        const char* c_str;
        Dart_StringToCString(string_handle, &c_str);
        EXPECT_STREQ(c_str, "Exception: I should be coming from A");
        latch.CountDown();
      }));

  AddNativeCallback(
      "NotifyErrorB", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto string_handle = Dart_GetNativeArgument(args, 0);
        const char* c_str;
        Dart_StringToCString(string_handle, &c_str);
        EXPECT_STREQ(c_str, "Exception: I should be coming from B");
        latch.CountDown();
      }));

  RunEngine(shell.get(), std::move(configuration));

  ASSERT_TRUE(DartVMRef::IsInstanceRunning());

  PostSync(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [this, &spawner = shell, &second_configuration, &latch]() {
        ::testing::NiceMock<MockPlatformViewDelegate> platform_view_delegate;
        auto spawn = spawner->Spawn(
            std::move(second_configuration), "",
            [&platform_view_delegate](Shell& shell) {
              auto result =
                  std::make_unique<::testing::NiceMock<MockPlatformView>>(
                      platform_view_delegate, shell.GetTaskRunners());
              ON_CALL(*result, CreateRenderingSurface())
                  .WillByDefault(::testing::Invoke([] {
                    return std::make_unique<::testing::NiceMock<MockSurface>>();
                  }));
              return result;
            },
            [](Shell& shell) { return std::make_unique<Rasterizer>(shell); });
        ASSERT_NE(nullptr, spawn.get());
        ASSERT_TRUE(ValidateShell(spawn.get()));

        // Before destroying the shell, wait for expectations of the spawned
        // isolate to be met.
        latch.Wait();

        DestroyShell(std::move(spawn));
      });

  DestroyShell(std::move(shell));
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, ImmutableBufferLoadsAssetOnBackgroundThread) {
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  fml::CountDownLatch latch(1);
  AddNativeCallback("NotifyNative",
                    CREATE_NATIVE_ENTRY([&](auto args) { latch.CountDown(); }));

  // Create the surface needed by rasterizer
  PlatformViewNotifyCreated(shell.get());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testThatAssetLoadingHappensOnWorkerThread");
  auto asset_manager = configuration.GetAssetManager();
  auto test_resolver = std::make_unique<ThreadCheckingAssetResolver>(
      shell->GetDartVM()->GetConcurrentMessageLoop());
  auto leaked_resolver = test_resolver.get();
  asset_manager->PushBack(std::move(test_resolver));

  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  EXPECT_EQ(leaked_resolver->mapping_requests[0], "DoesNotExist");

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, PictureToImageSync) {
#if !SHELL_ENABLE_GL
  // This test uses the GL backend.
  GTEST_SKIP();
#else
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend = ShellTestPlatformView::BackendType::kGLBackend,
      }),
  });

  AddNativeCallback("NativeOnBeforeToImageSync",
                    CREATE_NATIVE_ENTRY([&](auto args) {
                      // nop
                    }));

  fml::CountDownLatch latch(2);
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&](auto args) {
                      // Teardown and set up rasterizer again.
                      PlatformViewNotifyDestroyed(shell.get());
                      PlatformViewNotifyCreated(shell.get());
                      latch.CountDown();
                    }));

  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("toImageSync");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // !SHELL_ENABLE_GL
}

TEST_F(ShellTest, PictureToImageSyncImpellerNoSurface) {
#if !SHELL_ENABLE_METAL
  // This test uses the Metal backend.
  GTEST_SKIP();
#else
  auto settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend =
              ShellTestPlatformView::BackendType::kMetalBackend,
      }),
  });

  AddNativeCallback("NativeOnBeforeToImageSync",
                    CREATE_NATIVE_ENTRY([&](auto args) {
                      // nop
                    }));

  fml::CountDownLatch latch(2);
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&](auto args) {
                      // Teardown and set up rasterizer again.
                      PlatformViewNotifyDestroyed(shell.get());
                      PlatformViewNotifyCreated(shell.get());
                      latch.CountDown();
                    }));

  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);

  // Important: Do not create the platform view yet!
  // This test is making sure that the rasterizer can create the texture
  // as expected without a surface.

  configuration.SetEntrypoint("toImageSync");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
#endif  // !SHELL_ENABLE_METAL
}

#if SHELL_ENABLE_GL
// This test uses the GL backend and refers to symbols in egl.h
TEST_F(ShellTest, PictureToImageSyncWithTrampledContext) {
  // make it easier to trample the GL context by running on a single task
  // runner.
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
      .platform_view_create_callback = ShellTestPlatformViewBuilder({
          .rendering_backend = ShellTestPlatformView::BackendType::kGLBackend,
      }),
  });

  AddNativeCallback(
      "NativeOnBeforeToImageSync", CREATE_NATIVE_ENTRY([&](auto args) {
        // Trample the GL context. If the rasterizer fails
        // to make the right one current again, test will
        // fail.
        ::eglMakeCurrent(::eglGetCurrentDisplay(), NULL, NULL, NULL);
      }));

  fml::CountDownLatch latch(2);
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&](auto args) {
                      // Teardown and set up rasterizer again.
                      PlatformViewNotifyDestroyed(shell.get());
                      PlatformViewNotifyCreated(shell.get());
                      latch.CountDown();
                    }));

  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("toImageSync");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}
#endif  // SHELL_ENABLE_GL

TEST_F(ShellTest, PluginUtilitiesCallbackHandleErrorHandling) {
  auto settings = CreateSettingsForFixture();
  std::unique_ptr<Shell> shell = CreateShell(settings);

  fml::AutoResetWaitableEvent latch;
  bool test_passed;
  AddNativeCallback("NotifyNativeBool", CREATE_NATIVE_ENTRY([&](auto args) {
                      Dart_Handle exception = nullptr;
                      test_passed = tonic::DartConverter<bool>::FromArguments(
                          args, 0, exception);
                      latch.Signal();
                    }));

  ASSERT_NE(shell, nullptr);
  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  PlatformViewNotifyCreated(shell.get());
  configuration.SetEntrypoint("testPluginUtilitiesCallbackHandle");
  RunEngine(shell.get(), std::move(configuration));
  PumpOneFrame(shell.get());

  latch.Wait();

  ASSERT_TRUE(test_passed);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell));
}

TEST_F(ShellTest, NotifyIdleRejectsPastAndNearFuture) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform | ThreadHost::kUi |
                             ThreadHost::kIo | ThreadHost::kRaster);
  auto platform_task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  fml::AutoResetWaitableEvent latch;

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("emptyMain");
  RunEngine(shell.get(), std::move(configuration));

  fml::TaskRunner::RunNowOrPostTask(
      task_runners.GetUITaskRunner(), [&latch, &shell]() {
        auto runtime_controller = const_cast<RuntimeController*>(
            shell->GetEngine()->GetRuntimeController());

        auto now = fml::TimeDelta::FromMicroseconds(Dart_TimelineGetMicros());

        EXPECT_FALSE(runtime_controller->NotifyIdle(
            now - fml::TimeDelta::FromMilliseconds(10)));
        EXPECT_FALSE(runtime_controller->NotifyIdle(now));
        EXPECT_FALSE(runtime_controller->NotifyIdle(
            now + fml::TimeDelta::FromNanoseconds(100)));

        EXPECT_TRUE(runtime_controller->NotifyIdle(
            now + fml::TimeDelta::FromMilliseconds(100)));
        latch.Signal();
      });

  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, NotifyIdleNotCalledInLatencyMode) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform | ThreadHost::kUi |
                             ThreadHost::kIo | ThreadHost::kRaster);
  auto platform_task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  // we start off in balanced mode, where we expect idle notifications to
  // succeed. After the first `NotifyNativeBool` we expect to be in latency
  // mode, where we expect idle notifications to fail.
  fml::CountDownLatch latch(2);
  AddNativeCallback(
      "NotifyNativeBool", CREATE_NATIVE_ENTRY([&](auto args) {
        Dart_Handle exception = nullptr;
        bool is_in_latency_mode =
            tonic::DartConverter<bool>::FromArguments(args, 0, exception);
        auto runtime_controller = const_cast<RuntimeController*>(
            shell->GetEngine()->GetRuntimeController());
        bool success =
            runtime_controller->NotifyIdle(fml::TimeDelta::FromMicroseconds(
                Dart_TimelineGetMicros() + 100000));
        EXPECT_EQ(success, !is_in_latency_mode);
        latch.CountDown();
      }));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("performanceModeImpactsNotifyIdle");
  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, NotifyDestroyed) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform | ThreadHost::kUi |
                             ThreadHost::kIo | ThreadHost::kRaster);
  auto platform_task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  auto shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(DartVMRef::IsInstanceRunning());
  ASSERT_TRUE(ValidateShell(shell.get()));

  fml::CountDownLatch latch(1);
  AddNativeCallback("NotifyDestroyed", CREATE_NATIVE_ENTRY([&](auto args) {
                      auto runtime_controller = const_cast<RuntimeController*>(
                          shell->GetEngine()->GetRuntimeController());
                      bool success = runtime_controller->NotifyDestroyed();
                      EXPECT_TRUE(success);
                      latch.CountDown();
                    }));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("callNotifyDestroyed");
  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, PrintsErrorWhenPlatformMessageSentFromWrongThread) {
#if FLUTTER_RUNTIME_MODE != FLUTTER_RUNTIME_MODE_DEBUG || OS_FUCHSIA
  GTEST_SKIP() << "Test is for debug mode only on non-fuchsia targets.";
#else
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);

  {
    fml::testing::LogCapture log_capture;

    // The next call will result in a thread checker violation.
    fml::ThreadChecker::DisableNextThreadCheckFailure();
    SendPlatformMessage(shell.get(), std::make_unique<PlatformMessage>(
                                         "com.test.plugin", nullptr));

    EXPECT_THAT(
        log_capture.str(),
        ::testing::EndsWith(
            "The 'com.test.plugin' channel sent a message from native to "
            "Flutter on a non-platform thread. Platform channel messages "
            "must be sent on the platform thread. Failure to do so may "
            "result in data loss or crashes, and must be fixed in the "
            "plugin or application code creating that channel.\nSee "
            "https://docs.flutter.dev/platform-integration/"
            "platform-channels#channels-and-platform-threading for more "
            "information.\n"));
  }

  {
    fml::testing::LogCapture log_capture;

    // The next call will result in a thread checker violation.
    fml::ThreadChecker::DisableNextThreadCheckFailure();
    SendPlatformMessage(shell.get(), std::make_unique<PlatformMessage>(
                                         "com.test.plugin", nullptr));

    EXPECT_EQ(log_capture.str(), "");
  }

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
#endif
}

TEST_F(ShellTest, NavigationMessageDispachedImmediately) {
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  auto shell = CreateShell(settings, task_runners);

  auto latch = std::make_shared<fml::CountDownLatch>(1u);
  task_runner->PostTask([&]() {
    auto message = MakePlatformMessage(
        "flutter/navigation",
        {{"method", "setInitialRoute"}, {"args", "/testo"}}, nullptr);
    SendPlatformMessage(shell.get(), std::move(message));
    EXPECT_EQ(shell->GetEngine()->InitialRoute(), "/testo");

    latch->CountDown();
  });
  latch->Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

// Verifies a semantics Action will flush the dart event loop.
TEST_F(ShellTest, SemanticsActionsFlushMessageLoop) {
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  EXPECT_EQ(task_runners.GetPlatformTaskRunner(),
            task_runners.GetUITaskRunner());
  auto shell = CreateShell(settings, task_runners);
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testSemanticsActions");

  RunEngine(shell.get(), std::move(configuration));
  fml::CountDownLatch latch(1);
  AddNativeCallback(
      // The Dart native function names aren't very consistent but this is
      // just the native function name of the second vm entrypoint in the
      // fixture.
      "NotifyNative",
      CREATE_NATIVE_ENTRY([&](auto args) { latch.CountDown(); }));

  task_runners.GetPlatformTaskRunner()->PostTask([&] {
    SendSemanticsAction(shell.get(), 0, SemanticsAction::kTap,
                        fml::MallocMapping(nullptr, 0));
  });
  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

// Verifies a pointer event will flush the dart event loop.
TEST_F(ShellTest, PointerPacketFlushMessageLoop) {
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);

  EXPECT_EQ(task_runners.GetPlatformTaskRunner(),
            task_runners.GetUITaskRunner());
  auto shell = CreateShell(settings, task_runners);
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testPointerActions");

  RunEngine(shell.get(), std::move(configuration));
  fml::CountDownLatch latch(1);
  AddNativeCallback(
      // The Dart native function names aren't very consistent but this is
      // just the native function name of the second vm entrypoint in the
      // fixture.
      "NotifyNative",
      CREATE_NATIVE_ENTRY([&](auto args) { latch.CountDown(); }));

  DispatchFakePointerData(shell.get(), 23);
  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
}

TEST_F(ShellTest, DiesIfSoftwareRenderingAndImpellerAreEnabledDeathTest) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Fuchsia";
#else
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  settings.enable_software_rendering = true;
  ThreadHost thread_host("io.flutter.test." + GetCurrentTestName() + ".",
                         ThreadHost::Type::kPlatform);
  auto task_runner = thread_host.platform_thread->GetTaskRunner();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  EXPECT_DEATH_IF_SUPPORTED(
      CreateShell(settings, task_runners),
      "Software rendering is incompatible with Impeller.");
#endif  // OS_FUCHSIA
}

// Parse the arguments of NativeReportViewIdsCallback and
// store them in hasImplicitView and viewIds.
static void ParseViewIdsCallback(const Dart_NativeArguments& args,
                                 bool* hasImplicitView,
                                 std::vector<int64_t>* viewIds) {
  Dart_Handle exception = nullptr;
  viewIds->clear();
  *hasImplicitView =
      tonic::DartConverter<bool>::FromArguments(args, 0, exception);
  ASSERT_EQ(exception, nullptr);
  *viewIds = tonic::DartConverter<std::vector<int64_t>>::FromArguments(
      args, 1, exception);
  ASSERT_EQ(exception, nullptr);
}

TEST_F(ShellTest, ShellStartsWithImplicitView) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  auto task_runner = CreateNewThread();
  TaskRunners task_runners("test", task_runner, task_runner, task_runner,
                           task_runner);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  bool hasImplicitView;
  std::vector<int64_t> viewIds;
  fml::AutoResetWaitableEvent reportLatch;
  auto nativeViewIdsCallback = [&reportLatch, &hasImplicitView,
                                &viewIds](Dart_NativeArguments args) {
    ParseViewIdsCallback(args, &hasImplicitView, &viewIds);
    reportLatch.Signal();
  };
  AddNativeCallback("NativeReportViewIdsCallback",
                    CREATE_NATIVE_ENTRY(nativeViewIdsCallback));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewIds");
  RunEngine(shell.get(), std::move(configuration));
  reportLatch.Wait();

  ASSERT_TRUE(hasImplicitView);
  ASSERT_EQ(viewIds.size(), 1u);
  ASSERT_EQ(viewIds[0], 0ll);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// Tests that Shell::AddView and Shell::RemoveView works.
TEST_F(ShellTest, ShellCanAddViewOrRemoveView) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  bool hasImplicitView;
  std::vector<int64_t> viewIds;
  fml::AutoResetWaitableEvent reportLatch;
  auto nativeViewIdsCallback = [&reportLatch, &hasImplicitView,
                                &viewIds](Dart_NativeArguments args) {
    ParseViewIdsCallback(args, &hasImplicitView, &viewIds);
    reportLatch.Signal();
  };
  AddNativeCallback("NativeReportViewIdsCallback",
                    CREATE_NATIVE_ENTRY(nativeViewIdsCallback));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewIds");
  RunEngine(shell.get(), std::move(configuration));

  reportLatch.Wait();
  ASSERT_TRUE(hasImplicitView);
  ASSERT_EQ(viewIds.size(), 1u);
  ASSERT_EQ(viewIds[0], 0ll);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    shell->GetPlatformView()->AddView(2, ViewportMetrics{},
                                      [](bool added) { EXPECT_TRUE(added); });
  });
  reportLatch.Wait();
  ASSERT_TRUE(hasImplicitView);
  ASSERT_EQ(viewIds.size(), 2u);
  ASSERT_EQ(viewIds[1], 2ll);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    shell->GetPlatformView()->RemoveView(
        2, [](bool removed) { ASSERT_TRUE(removed); });
  });
  reportLatch.Wait();
  ASSERT_TRUE(hasImplicitView);
  ASSERT_EQ(viewIds.size(), 1u);
  ASSERT_EQ(viewIds[0], 0ll);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    shell->GetPlatformView()->AddView(4, ViewportMetrics{},
                                      [](bool added) { EXPECT_TRUE(added); });
  });
  reportLatch.Wait();
  ASSERT_TRUE(hasImplicitView);
  ASSERT_EQ(viewIds.size(), 2u);
  ASSERT_EQ(viewIds[1], 4ll);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// Test that add view fails if the view ID already exists.
TEST_F(ShellTest, ShellCannotAddDuplicateViewId) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  bool has_implicit_view;
  std::vector<int64_t> view_ids;
  fml::AutoResetWaitableEvent report_latch;
  AddNativeCallback("NativeReportViewIdsCallback",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      ParseViewIdsCallback(args, &has_implicit_view, &view_ids);
                      report_latch.Signal();
                    }));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewIds");
  RunEngine(shell.get(), std::move(configuration));

  report_latch.Wait();
  ASSERT_TRUE(has_implicit_view);
  ASSERT_EQ(view_ids.size(), 1u);
  ASSERT_EQ(view_ids[0], kImplicitViewId);

  // Add view 123.
  fml::AutoResetWaitableEvent add_latch;
  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell,
                                                             &add_latch] {
    shell->GetPlatformView()->AddView(123, ViewportMetrics{}, [&](bool added) {
      EXPECT_TRUE(added);
      add_latch.Signal();
    });
  });

  add_latch.Wait();

  report_latch.Wait();
  ASSERT_EQ(view_ids.size(), 2u);
  ASSERT_EQ(view_ids[0], kImplicitViewId);
  ASSERT_EQ(view_ids[1], 123);

  // Attempt to add duplicate view ID 123. This should fail.
  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell,
                                                             &add_latch] {
    shell->GetPlatformView()->AddView(123, ViewportMetrics{}, [&](bool added) {
      EXPECT_FALSE(added);
      add_latch.Signal();
    });
  });

  add_latch.Wait();

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// Test that remove view fails if the view ID does not exist.
TEST_F(ShellTest, ShellCannotRemoveNonexistentId) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  bool has_implicit_view;
  std::vector<int64_t> view_ids;
  fml::AutoResetWaitableEvent report_latch;
  AddNativeCallback("NativeReportViewIdsCallback",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      ParseViewIdsCallback(args, &has_implicit_view, &view_ids);
                      report_latch.Signal();
                    }));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewIds");
  RunEngine(shell.get(), std::move(configuration));

  report_latch.Wait();
  ASSERT_TRUE(has_implicit_view);
  ASSERT_EQ(view_ids.size(), 1u);
  ASSERT_EQ(view_ids[0], kImplicitViewId);

  // Remove view 123. This should fail as this view doesn't exist.
  fml::AutoResetWaitableEvent remove_latch;
  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(),
           [&shell, &remove_latch] {
             shell->GetPlatformView()->RemoveView(123, [&](bool removed) {
               EXPECT_FALSE(removed);
               remove_latch.Signal();
             });
           });

  remove_latch.Wait();

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// Parse the arguments of NativeReportViewWidthsCallback and
// store them in viewWidths.
static void ParseViewWidthsCallback(const Dart_NativeArguments& args,
                                    std::map<int64_t, int64_t>* viewWidths) {
  Dart_Handle exception = nullptr;
  viewWidths->clear();
  std::vector<int64_t> viewWidthPacket =
      tonic::DartConverter<std::vector<int64_t>>::FromArguments(args, 0,
                                                                exception);
  ASSERT_EQ(exception, nullptr);
  ASSERT_EQ(viewWidthPacket.size() % 2, 0ul);
  for (size_t packetIndex = 0; packetIndex < viewWidthPacket.size();
       packetIndex += 2) {
    (*viewWidths)[viewWidthPacket[packetIndex]] =
        viewWidthPacket[packetIndex + 1];
  }
}

// Ensure that PlatformView::SetViewportMetrics and Shell::AddView that were
// dispatched before the isolate is run have been flushed to the Dart VM when
// the main function starts.
TEST_F(ShellTest, ShellFlushesPlatformStatesByMain) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    auto platform_view = shell->GetPlatformView();
    // The construtor for ViewportMetrics{_, width, _, _, _} (only the 2nd
    // argument matters in this test).
    platform_view->SetViewportMetrics(0, ViewportMetrics{1, 10, 1, 0, 0});
    shell->GetPlatformView()->AddView(1, ViewportMetrics{1, 30, 1, 0, 0},
                                      [](bool added) { ASSERT_TRUE(added); });
    platform_view->SetViewportMetrics(0, ViewportMetrics{1, 20, 1, 0, 0});
  });

  bool first_report = true;
  std::map<int64_t, int64_t> viewWidths;
  fml::AutoResetWaitableEvent reportLatch;
  auto nativeViewWidthsCallback = [&reportLatch, &viewWidths,
                                   &first_report](Dart_NativeArguments args) {
    EXPECT_TRUE(first_report);
    first_report = false;
    ParseViewWidthsCallback(args, &viewWidths);
    reportLatch.Signal();
  };
  AddNativeCallback("NativeReportViewWidthsCallback",
                    CREATE_NATIVE_ENTRY(nativeViewWidthsCallback));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewWidths");
  RunEngine(shell.get(), std::move(configuration));

  reportLatch.Wait();
  EXPECT_EQ(viewWidths.size(), 2u);
  EXPECT_EQ(viewWidths[0], 20ll);
  EXPECT_EQ(viewWidths[1], 30ll);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// A view can be added and removed before the Dart isolate is launched.
TEST_F(ShellTest, CanRemoveViewBeforeLaunchingIsolate) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    auto platform_view = shell->GetPlatformView();

    // A view can be added and removed all before the isolate launches.
    // The pending add view operation is cancelled, the view is never
    // added to the Dart isolate.
    shell->GetPlatformView()->AddView(123, ViewportMetrics{1, 30, 1, 0, 0},
                                      [](bool added) { ASSERT_FALSE(added); });
    shell->GetPlatformView()->RemoveView(
        123, [](bool removed) { ASSERT_FALSE(removed); });
  });

  bool first_report = true;
  std::map<int64_t, int64_t> view_widths;
  fml::AutoResetWaitableEvent report_latch;
  AddNativeCallback("NativeReportViewWidthsCallback",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      EXPECT_TRUE(first_report);
                      first_report = false;
                      ParseViewWidthsCallback(args, &view_widths);
                      report_latch.Signal();
                    }));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewWidths");
  RunEngine(shell.get(), std::move(configuration));

  report_latch.Wait();
  EXPECT_EQ(view_widths.size(), 1u);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

// Ensure pending "add views" failures are properly flushed when the Dart
// isolate is launched.
TEST_F(ShellTest, IgnoresBadAddViewsBeforeLaunchingIsolate) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  PostSync(shell->GetTaskRunners().GetPlatformTaskRunner(), [&shell] {
    auto platform_view = shell->GetPlatformView();

    // Add the same view twice. The second time should fail.
    shell->GetPlatformView()->AddView(123, ViewportMetrics{1, 100, 1, 0, 0},
                                      [](bool added) { ASSERT_TRUE(added); });

    shell->GetPlatformView()->AddView(123, ViewportMetrics{1, 200, 1, 0, 0},
                                      [](bool added) { ASSERT_FALSE(added); });

    // Add another view. Previous failures should not affect this.
    shell->GetPlatformView()->AddView(456, ViewportMetrics{1, 300, 1, 0, 0},
                                      [](bool added) { ASSERT_TRUE(added); });
  });

  bool first_report = true;
  std::map<int64_t, int64_t> view_widths;
  fml::AutoResetWaitableEvent report_latch;
  AddNativeCallback("NativeReportViewWidthsCallback",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      EXPECT_TRUE(first_report);
                      first_report = false;
                      ParseViewWidthsCallback(args, &view_widths);
                      report_latch.Signal();
                    }));

  PlatformViewNotifyCreated(shell.get());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("testReportViewWidths");
  RunEngine(shell.get(), std::move(configuration));

  report_latch.Wait();
  EXPECT_EQ(view_widths.size(), 3u);
  EXPECT_EQ(view_widths[0], 0);
  EXPECT_EQ(view_widths[123], 100);
  EXPECT_EQ(view_widths[456], 300);

  PlatformViewNotifyDestroyed(shell.get());
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, RuntimeStageBackendDefaultsToSkSLWithoutImpeller) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = false;
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("NotifyNative", CREATE_NATIVE_ENTRY([&latch](auto args) {
                      auto backend =
                          UIDartState::Current()->GetRuntimeStageBackend();
                      EXPECT_EQ(backend, impeller::RuntimeStageBackend::kSkSL);
                      latch.Signal();
                    }));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("mainNotifyNative");
  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
}

#if IMPELLER_SUPPORTS_RENDERING
TEST_F(ShellTest, RuntimeStageBackendWithImpeller) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      "io.flutter.test." + GetCurrentTestName() + ".",
      ThreadHost::Type::kPlatform | ThreadHost::Type::kRaster |
          ThreadHost::Type::kIo | ThreadHost::Type::kUi));
  TaskRunners task_runners("test", thread_host.platform_thread->GetTaskRunner(),
                           thread_host.raster_thread->GetTaskRunner(),
                           thread_host.ui_thread->GetTaskRunner(),
                           thread_host.io_thread->GetTaskRunner());
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell);

  fml::AutoResetWaitableEvent latch;

  impeller::Context::BackendType impeller_backend;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [platform_view = shell->GetPlatformView(), &latch, &impeller_backend]() {
        auto impeller_context = platform_view->GetImpellerContext();
        EXPECT_TRUE(impeller_context);
        impeller_backend = impeller_context->GetBackendType();
        latch.Signal();
      });
  latch.Wait();

  AddNativeCallback(
      "NotifyNative", CREATE_NATIVE_ENTRY([&](auto args) {
        auto backend = UIDartState::Current()->GetRuntimeStageBackend();
        switch (impeller_backend) {
          case impeller::Context::BackendType::kMetal:
            EXPECT_EQ(backend, impeller::RuntimeStageBackend::kMetal);
            break;
          case impeller::Context::BackendType::kOpenGLES:
            EXPECT_EQ(backend, impeller::RuntimeStageBackend::kOpenGLES3);
            break;
          case impeller::Context::BackendType::kVulkan:
            EXPECT_EQ(backend, impeller::RuntimeStageBackend::kVulkan);
            break;
        }
        latch.Signal();
      }));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("mainNotifyNative");
  RunEngine(shell.get(), std::move(configuration));

  latch.Wait();

  DestroyShell(std::move(shell), task_runners);
}
#endif  // IMPELLER_SUPPORTS_RENDERING

TEST_F(ShellTest, WillLogWarningWhenImpellerIsOptedOut) {
#if !IMPELLER_SUPPORTS_RENDERING
  GTEST_SKIP() << "This platform doesn't support Impeller.";
#endif
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = false;
  settings.warn_on_impeller_opt_out = true;
  // Log captures are thread specific. Just put the shell in single threaded
  // configuration.
  const auto& runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners("test", runner, runner, runner, runner);
  std::ostringstream stream;
  fml::LogMessage::CaptureNextLog(&stream);
  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(stream.str().find(
                  "[Action Required] The application opted out of Impeller") !=
              std::string::npos);
  ASSERT_TRUE(shell);
  DestroyShell(std::move(shell), task_runners);
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
