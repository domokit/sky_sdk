// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_TEST_H_
#define FLUTTER_SHELL_COMMON_SHELL_TEST_H_

#include "flutter/shell/common/shell.h"

#include <memory>

#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/common/settings.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/common/run_configuration.h"
#include "flutter/shell/common/shell_test_external_view_embedder.h"
#include "flutter/shell/common/shell_test_platform_view.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/common/vsync_waiters_test.h"
#include "flutter/testing/elf_loader.h"
#include "flutter/testing/fixture_test.h"
#include "flutter/testing/test_dart_native_resolver.h"

namespace flutter {
namespace testing {

// The signature of ViewContent::builder.
using LayerTreeBuilder =
    std::function<void(std::shared_ptr<ContainerLayer> root)>;
struct ViewContent;
// Defines the content to be rendered to all views of a frame in PumpOneFrame.
using FrameContent = std::map<int64_t, ViewContent>;
// Defines the content to be rendered to a view in PumpOneFrame.
struct ViewContent {
  flutter::ViewportMetrics viewport_metrics;
  // Given the root layer, this callback builds the layer tree to be rasterized
  // in PumpOneFrame.
  LayerTreeBuilder builder;

  // Build a frame with no views. This is useful when PumpOneFrame is used just
  // to schedule the frame while the frame content is defined by other means.
  static FrameContent NoViews();

  // Build a frame with a single implicit view with the specific size and no
  // content.
  static FrameContent DummyView(double width = 1, double height = 1);

  // Build a frame with a single implicit view with the specific viewport
  // metrics and no content.
  static FrameContent DummyView(flutter::ViewportMetrics viewport_metrics);

  // Build a frame with a single implicit view with the specific size and
  // content.
  static FrameContent ImplicitView(double width,
                                   double height,
                                   LayerTreeBuilder builder);
};

class ShellTest : public FixtureTest {
 public:
  struct Config {
    // Required.
    const Settings& settings;
    // Defaults to GetTaskRunnersForFixture().
    std::optional<TaskRunners> task_runners = {};
    bool is_gpu_disabled = false;
    // Defaults to calling ShellTestPlatformView::Create with the provided
    // arguments.
    Shell::CreateCallback<PlatformView> platform_view_create_callback;
  };

  ShellTest();

  Settings CreateSettingsForFixture() override;
  std::unique_ptr<Shell> CreateShell(
      const Settings& settings,
      std::optional<TaskRunners> task_runners = {});
  std::unique_ptr<Shell> CreateShell(const Config& config);
  void DestroyShell(std::unique_ptr<Shell> shell);
  void DestroyShell(std::unique_ptr<Shell> shell,
                    const TaskRunners& task_runners);
  TaskRunners GetTaskRunnersForFixture();

  fml::TimePoint GetLatestFrameTargetTime(Shell* shell) const;

  void SendPlatformMessage(Shell* shell,
                           std::unique_ptr<PlatformMessage> message);

  void SendSemanticsAction(Shell* shell,
                           int32_t node_id,
                           SemanticsAction action,
                           fml::MallocMapping args);

  void SendEnginePlatformMessage(Shell* shell,
                                 std::unique_ptr<PlatformMessage> message);

  static void PlatformViewNotifyCreated(
      Shell* shell);  // This creates the surface
  static void PlatformViewNotifyDestroyed(
      Shell* shell);  // This destroys the surface
  static void RunEngine(Shell* shell, RunConfiguration configuration);
  static void RestartEngine(Shell* shell, RunConfiguration configuration);

  /// Issue as many VSYNC as needed to flush the UI tasks so far, and reset
  /// the content of `will_draw_new_frame` to true if it's not nullptr.
  static void VSyncFlush(Shell* shell, bool* will_draw_new_frame = nullptr);

  static void SetViewportMetrics(Shell* shell, double width, double height);
  static void NotifyIdle(Shell* shell, fml::TimeDelta deadline);

  static void PumpOneFrame(Shell* shell);
  static void PumpOneFrame(Shell* shell, FrameContent frame_content);
  // Dispatch a PointerHoverEvent with the specified `x` as the pointer
  // position.
  static void DispatchFakePointerData(Shell* shell, double x);
  static void DispatchPointerData(Shell* shell,
                                  std::unique_ptr<PointerDataPacket> packet);
  // Declare |UnreportedTimingsCount|, |GetNeedsReportTimings| and
  // |SetNeedsReportTimings| inside |ShellTest| mainly for easier friend class
  // declarations as shell unit tests and Shell are in different name spaces.

  static bool GetNeedsReportTimings(Shell* shell);
  static void SetNeedsReportTimings(Shell* shell, bool value);

  // Declare |StorePersistentCache| inside |ShellTest| so |PersistentCache| can
  // friend |ShellTest| and allow us to call private |PersistentCache::store| in
  // unit tests.
  static void StorePersistentCache(PersistentCache* cache,
                                   const SkData& key,
                                   const SkData& value);

  static bool IsAnimatorRunning(Shell* shell);

  enum ServiceProtocolEnum {
    kGetSkSLs,
    kEstimateRasterCacheMemory,
    kSetAssetBundlePath,
    kRunInView,
  };

  // Helper method to test private method Shell::OnServiceProtocolGetSkSLs.
  // (ShellTest is a friend class of Shell.) We'll also make sure that it is
  // running on the correct task_runner.
  static void OnServiceProtocol(
      Shell* shell,
      ServiceProtocolEnum some_protocol,
      const fml::RefPtr<fml::TaskRunner>& task_runner,
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  std::shared_ptr<txt::FontCollection> GetFontCollection(Shell* shell);

  // Do not assert |UnreportedTimingsCount| to be positive in any tests.
  // Otherwise those tests will be flaky as the clearing of unreported timings
  // is unpredictive.
  static int UnreportedTimingsCount(Shell* shell);

  static void TurnOffGPU(Shell* shell, bool value);

 private:
  ThreadHost thread_host_;

  FML_DISALLOW_COPY_AND_ASSIGN(ShellTest);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SHELL_TEST_H_
