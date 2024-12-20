// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <memory>
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/platform/android/android_context_gl_skia.h"
#include "flutter/shell/platform/android/android_egl_surface.h"
#include "flutter/shell/platform/android/android_environment_gl.h"
#include "flutter/shell/platform/android/android_surface_gl_skia.h"
#include "fml/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "impeller/core/runtime_types.h"
#include "shell/platform/android/context/android_context.h"

namespace flutter {
namespace testing {
namespace android {
namespace {

TaskRunners MakeTaskRunners(const std::string& thread_label,
                            const ThreadHost& thread_host) {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  fml::RefPtr<fml::TaskRunner> platform_runner =
      fml::MessageLoop::GetCurrent().GetTaskRunner();

  return TaskRunners(thread_label, platform_runner,
                     thread_host.raster_thread->GetTaskRunner(),
                     thread_host.ui_thread->GetTaskRunner(),
                     thread_host.io_thread->GetTaskRunner());
}
}  // namespace

class TestImpellerContext : public impeller::Context {
 public:
  TestImpellerContext() {}

  ~TestImpellerContext() {}

  impeller::Context::BackendType GetBackendType() const override {
    return impeller::Context::BackendType::kOpenGLES;
  }

  std::string DescribeGpuModel() const override { return ""; }

  bool IsValid() const override { return true; }

  const std::shared_ptr<const impeller::Capabilities>& GetCapabilities()
      const override {
    FML_UNREACHABLE();
  }

  bool UpdateOffscreenLayerPixelFormat(impeller::PixelFormat format) override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::Allocator> GetResourceAllocator() const override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::ShaderLibrary> GetShaderLibrary() const override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::SamplerLibrary> GetSamplerLibrary() const override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::PipelineLibrary> GetPipelineLibrary()
      const override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::CommandBuffer> CreateCommandBuffer()
      const override {
    FML_UNREACHABLE();
  }

  std::shared_ptr<impeller::CommandQueue> GetCommandQueue() const override {
    FML_UNREACHABLE();
  }

  void Shutdown() override { did_shutdown = true; }

  impeller::RuntimeStageBackend GetRuntimeStageBackend() const override {
    return impeller::RuntimeStageBackend::kVulkan;
  }

  bool did_shutdown = false;
};

class TestAndroidContext : public AndroidContext {
 public:
  TestAndroidContext(const std::shared_ptr<impeller::Context>& impeller_context,
                     AndroidRenderingAPI rendering_api)
      : AndroidContext(rendering_api) {
    SetImpellerContext(impeller_context);
  }
};

TEST(AndroidContextGl, Create) {
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  auto environment = fml::MakeRefCounted<AndroidEnvironmentGL>();
  std::string thread_label =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();

  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      thread_label, ThreadHost::Type::kUi | ThreadHost::Type::kRaster |
                        ThreadHost::Type::kIo));
  TaskRunners task_runners = MakeTaskRunners(thread_label, thread_host);
  auto context =
      std::make_unique<AndroidContextGLSkia>(environment, task_runners);
  context->SetMainSkiaContext(main_context);
  EXPECT_NE(context.get(), nullptr);
  context.reset();
  EXPECT_TRUE(main_context->abandoned());
}

TEST(AndroidContextGl, CreateImpeller) {
  auto impeller_context = std::make_shared<TestImpellerContext>();
  auto android_context = std::make_unique<TestAndroidContext>(
      impeller_context, AndroidRenderingAPI::kImpellerOpenGLES);
  EXPECT_FALSE(impeller_context->did_shutdown);

  android_context.reset();

  EXPECT_TRUE(impeller_context->did_shutdown);
}

TEST(AndroidContextGl, CreateSingleThread) {
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  auto environment = fml::MakeRefCounted<AndroidEnvironmentGL>();
  std::string thread_label =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  fml::RefPtr<fml::TaskRunner> platform_runner =
      fml::MessageLoop::GetCurrent().GetTaskRunner();
  TaskRunners task_runners =
      TaskRunners(thread_label, platform_runner, platform_runner,
                  platform_runner, platform_runner);
  auto context =
      std::make_unique<AndroidContextGLSkia>(environment, task_runners);
  context->SetMainSkiaContext(main_context);
  EXPECT_NE(context.get(), nullptr);
  context.reset();
  EXPECT_TRUE(main_context->abandoned());
}

TEST(AndroidSurfaceGL, CreateSnapshopSurfaceWhenOnscreenSurfaceIsNotNull) {
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  auto environment = fml::MakeRefCounted<AndroidEnvironmentGL>();
  std::string thread_label =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      thread_label, ThreadHost::Type::kUi | ThreadHost::Type::kRaster |
                        ThreadHost::Type::kIo));
  TaskRunners task_runners = MakeTaskRunners(thread_label, thread_host);
  auto android_context =
      std::make_shared<AndroidContextGLSkia>(environment, task_runners);
  auto android_surface =
      std::make_unique<AndroidSurfaceGLSkia>(android_context);
  auto window = fml::MakeRefCounted<AndroidNativeWindow>(
      nullptr, /*is_fake_window=*/true);
  android_surface->SetNativeWindow(window);
  auto onscreen_surface = android_surface->GetOnscreenSurface();
  EXPECT_NE(onscreen_surface, nullptr);
  android_surface->CreateSnapshotSurface();
  EXPECT_EQ(onscreen_surface, android_surface->GetOnscreenSurface());
}

TEST(AndroidSurfaceGL, CreateSnapshopSurfaceWhenOnscreenSurfaceIsNull) {
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  auto environment = fml::MakeRefCounted<AndroidEnvironmentGL>();
  std::string thread_label =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();

  auto mask =
      ThreadHost::Type::kUi | ThreadHost::Type::kRaster | ThreadHost::Type::kIo;
  flutter::ThreadHost::ThreadHostConfig host_config(mask);

  ThreadHost thread_host(host_config);
  TaskRunners task_runners = MakeTaskRunners(thread_label, thread_host);
  auto android_context =
      std::make_shared<AndroidContextGLSkia>(environment, task_runners);
  auto android_surface =
      std::make_unique<AndroidSurfaceGLSkia>(android_context);
  EXPECT_EQ(android_surface->GetOnscreenSurface(), nullptr);
  android_surface->CreateSnapshotSurface();
  EXPECT_NE(android_surface->GetOnscreenSurface(), nullptr);
}

TEST(AndroidContextGl, EnsureMakeCurrentChecksCurrentContextStatus) {
  GrMockOptions main_context_options;
  sk_sp<GrDirectContext> main_context =
      GrDirectContext::MakeMock(&main_context_options);
  auto environment = fml::MakeRefCounted<AndroidEnvironmentGL>();
  std::string thread_label =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();

  ThreadHost thread_host(ThreadHost::ThreadHostConfig(
      thread_label, ThreadHost::Type::kUi | ThreadHost::Type::kRaster |
                        ThreadHost::Type::kIo));
  TaskRunners task_runners = MakeTaskRunners(thread_label, thread_host);
  auto context =
      std::make_unique<AndroidContextGLSkia>(environment, task_runners);

  auto pbuffer_surface = context->CreatePbufferSurface();
  auto status = pbuffer_surface->MakeCurrent();
  EXPECT_EQ(AndroidEGLSurfaceMakeCurrentStatus::kSuccessMadeCurrent, status);

  // context already current, so status must reflect that.
  status = pbuffer_surface->MakeCurrent();
  EXPECT_EQ(AndroidEGLSurfaceMakeCurrentStatus::kSuccessAlreadyCurrent, status);
}
}  // namespace android
}  // namespace testing
}  // namespace flutter
