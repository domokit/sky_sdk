// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/shell_io_manager.h"

#include "flutter/common/task_runners.h"
#include "flutter/fml/mapping.h"
#include "flutter/lib/ui/painting/multi_frame_codec.h"
#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"
#include "flutter/testing/post_task_sync.h"
#include "flutter/testing/test_gl_surface.h"  // nogncheck
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

class ShellIOManagerTest : public FixtureTest {};

// Regression test for https://github.com/flutter/engine/pull/32106.
TEST_F(ShellIOManagerTest,
       ItDoesNotCrashThatSkiaUnrefQueueDrainAfterIOManagerReset) {
  auto settings = CreateSettingsForFixture();
  auto vm_ref = DartVMRef::Create(settings);
  auto vm_data = vm_ref.GetVMData();
  auto gif_mapping = flutter::testing::OpenFixtureAsSkData("hello_loop_2.gif");
  ASSERT_TRUE(gif_mapping);

  ImageGeneratorRegistry registry;
  std::shared_ptr<ImageGenerator> gif_generator =
      registry.CreateCompatibleGenerator(gif_mapping);
  ASSERT_TRUE(gif_generator);

  TaskRunners runners(GetCurrentTestName(),         // label
                      CreateNewThread("platform"),  // platform
                      CreateNewThread("raster"),    // raster
                      CreateNewThread("ui"),        // ui
                      CreateNewThread("io")         // io
  );

  std::unique_ptr<TestGLSurface> gl_surface;
  std::unique_ptr<ShellIOManager> io_manager;
  fml::RefPtr<MultiFrameCodec> codec;

  // Setup the IO manager.
  PostTaskSync(runners.GetIOTaskRunner(), [&]() {
    gl_surface = std::make_unique<TestGLSurface>(SkISize::Make(1, 1));
    io_manager = std::make_unique<ShellIOManager>(
        gl_surface->CreateGrContext(), std::make_shared<fml::SyncSwitch>(),
        runners.GetIOTaskRunner(), nullptr,
        fml::TimeDelta::FromMilliseconds(0));
  });

  auto isolate = RunDartCodeInIsolate(vm_ref, settings, runners, "emptyMain",
                                      {}, GetDefaultKernelFilePath(),
                                      io_manager->GetWeakIOManager());

  PostTaskSync(runners.GetUITaskRunner(), [&]() {
    fml::AutoResetWaitableEvent isolate_latch;

    EXPECT_TRUE(isolate->RunInIsolateScope([&]() -> bool {
      Dart_Handle library = Dart_RootLibrary();
      if (Dart_IsError(library)) {
        isolate_latch.Signal();
        return false;
      }
      Dart_Handle closure =
          Dart_GetField(library, Dart_NewStringFromCString("frameCallback"));
      if (Dart_IsError(closure) || !Dart_IsClosure(closure)) {
        isolate_latch.Signal();
        return false;
      }

      codec = fml::MakeRefCounted<MultiFrameCodec>(std::move(gif_generator));
      codec->getNextFrame(closure);
      isolate_latch.Signal();
      return true;
    }));
    isolate_latch.Wait();
  });

  // Destroy the IO manager
  PostTaskSync(runners.GetIOTaskRunner(), [&]() {
    // 'SkiaUnrefQueue.Drain' will be called after 'io_manager.reset()' in this
    // test, If the resource context has been destroyed at that time, it will
    // crash.
    //
    // 'Drain()' currently checks whether the weak pointer is still valid or not
    // before trying to call anything on it.
    //
    // However, calling 'unref' on the 'SkImage_Lazy' ends up freeing a
    // 'GrBackendTexture'. That object seems to assume that something else is
    // keeping the context alive. This seems like it might be a bad assumption
    // on Skia's part, but in Skia's defense we're doing something pretty weird
    // here by keeping GPU resident objects alive without keeping the
    // 'GrDirectContext' alive ourselves.
    //
    // See https://github.com/flutter/flutter/issues/87895
    io_manager.reset();
    gl_surface.reset();
  });
}

}  // namespace testing
}  // namespace flutter
