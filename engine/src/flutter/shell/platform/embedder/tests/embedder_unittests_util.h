// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_UNITTESTS_UTIL_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_UNITTESTS_UTIL_H_

#define FML_USED_ON_EMBEDDER

#include <future>
#include <utility>

#include "flutter/fml/mapping.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/paths.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_config_builder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {
namespace testing {

sk_sp<SkSurface> CreateRenderSurface(const FlutterLayer& layer,
                                     GrDirectContext* context);

bool RasterImagesAreSame(const sk_sp<SkImage>& a, const sk_sp<SkImage>& b);

/// @brief      Prepends a prefix to the name which is unique to the test
///             context type. This is useful for tests that use
///             EmbedderTestMultiBackend and require different fixtures per
///             backend. For OpenGL, the name remains unchanged.
/// @param[in]  backend  The test context type used to determine the prepended
///                      prefix (e.g. `vk_[name]` for Vulkan).
/// @param[in]  name     The name of the fixture without any special prefixes.
std::string FixtureNameForBackend(EmbedderTestContextType backend,
                                  const std::string& name);

/// @brief      Resolves a render target type for a given backend description.
///             This is useful for tests that use EmbedderTestMultiBackend.
/// @param[in]  backend             The test context type to resolve the render
///                                 target for.
/// @param[in]  opengl_framebuffer  Ignored for all non-OpenGL backends. Flutter
///                                 supports rendering to both OpenGL textures
///                                 and framebuffers. When false, the OpenGL
///                                 texture render target type is returned.
EmbedderTestBackingStoreProducer::RenderTargetType GetRenderTargetFromBackend(
    EmbedderTestContextType backend,
    bool opengl_framebuffer);

/// @brief      Configures per-backend properties for a given backing store.
/// @param[in]  backing_store       The backing store to configure.
/// @param[in]  backend             The test context type used to decide which
///                                 backend the backing store will be used with.
/// @param[in]  opengl_framebuffer  Ignored for all non-OpenGL backends. Flutter
///                                 supports rendering to both OpenGL textures
///                                 and framebuffers. When false, the backing
///                                 store is configured to be an OpenGL texture.
void ConfigureBackingStore(FlutterBackingStore& backing_store,
                           EmbedderTestContextType backend,
                           bool opengl_framebuffer);

bool WriteImageToDisk(const fml::UniqueFD& directory,
                      const std::string& name,
                      const sk_sp<SkImage>& image);

bool ImageMatchesFixture(const std::string& fixture_file_name,
                         const sk_sp<SkImage>& scene_image);

bool ImageMatchesFixture(const std::string& fixture_file_name,
                         std::future<sk_sp<SkImage>>& scene_image);

bool SurfacePixelDataMatchesBytes(SkSurface* surface,
                                  const std::vector<uint8_t>& bytes);

bool SurfacePixelDataMatchesBytes(std::future<SkSurface*>& surface_future,
                                  const std::vector<uint8_t>& bytes);

void FilterMutationsByType(
    const FlutterPlatformViewMutation** mutations,
    size_t count,
    FlutterPlatformViewMutationType type,
    const std::function<void(const FlutterPlatformViewMutation& mutation)>&
        handler);

void FilterMutationsByType(
    const FlutterPlatformView* view,
    FlutterPlatformViewMutationType type,
    const std::function<void(const FlutterPlatformViewMutation& mutation)>&
        handler);

SkMatrix GetTotalMutationTransformationMatrix(
    const FlutterPlatformViewMutation** mutations,
    size_t count);

SkMatrix GetTotalMutationTransformationMatrix(const FlutterPlatformView* view);

//------------------------------------------------------------------------------
/// @brief      A task runner that we expect the embedder to provide but whose
///             implementation is a real FML task runner.
///
class EmbedderTestTaskRunner {
 public:
  using TaskExpiryCallback = std::function<void(FlutterTask)>;
  EmbedderTestTaskRunner(fml::RefPtr<fml::TaskRunner> real_task_runner,
                         TaskExpiryCallback on_task_expired)
      : identifier_(++sEmbedderTaskRunnerIdentifiers),
        real_task_runner_(std::move(real_task_runner)),
        on_task_expired_(std::move(on_task_expired)) {
    FML_CHECK(real_task_runner_);
    FML_CHECK(on_task_expired_);

    task_runner_description_.struct_size = sizeof(FlutterTaskRunnerDescription);
    task_runner_description_.user_data = this;
    task_runner_description_.runs_task_on_current_thread_callback =
        [](void* user_data) -> bool {
      return reinterpret_cast<EmbedderTestTaskRunner*>(user_data)
          ->real_task_runner_->RunsTasksOnCurrentThread();
    };
    task_runner_description_.post_task_callback = [](FlutterTask task,
                                                     uint64_t target_time_nanos,
                                                     void* user_data) -> void {
      auto thiz = reinterpret_cast<EmbedderTestTaskRunner*>(user_data);

      auto target_time = fml::TimePoint::FromEpochDelta(
          fml::TimeDelta::FromNanoseconds(target_time_nanos));
      auto on_task_expired = thiz->on_task_expired_;
      auto invoke_task = [task, on_task_expired]() { on_task_expired(task); };
      auto real_task_runner = thiz->real_task_runner_;

      real_task_runner->PostTaskForTime(invoke_task, target_time);
    };
    task_runner_description_.destruction_callback = [](void* user_data) {};
    task_runner_description_.identifier = identifier_;
  }

  void SetDestructionCallback(VoidCallback callback) {
    task_runner_description_.destruction_callback = callback;
  }

  const FlutterTaskRunnerDescription& GetFlutterTaskRunnerDescription() {
    return task_runner_description_;
  }

 private:
  static std::atomic_size_t sEmbedderTaskRunnerIdentifiers;
  const size_t identifier_;
  fml::RefPtr<fml::TaskRunner> real_task_runner_;
  TaskExpiryCallback on_task_expired_;
  FlutterTaskRunnerDescription task_runner_description_ = {};

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestTaskRunner);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_UNITTESTS_UTIL_H_
