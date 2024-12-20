// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/shell/common/shell_test.h"

#include "flutter/flow/frame_timings.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/mapping.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test_platform_view.h"
#include "flutter/shell/common/vsync_waiter_fallback.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

constexpr int64_t kImplicitViewId = 0;

FrameContent ViewContent::NoViews() {
  return std::map<int64_t, ViewContent>();
}

FrameContent ViewContent::DummyView(double width, double height) {
  FrameContent result;
  result[kImplicitViewId] = ViewContent{
      .viewport_metrics = {1.0, width, height, 22, 0},
      .builder = {},
  };
  return result;
}

FrameContent ViewContent::DummyView(flutter::ViewportMetrics viewport_metrics) {
  FrameContent result;
  result[kImplicitViewId] = ViewContent{
      .viewport_metrics = std::move(viewport_metrics),
      .builder = {},
  };
  return result;
}

FrameContent ViewContent::ImplicitView(double width,
                                       double height,
                                       LayerTreeBuilder builder) {
  FrameContent result;
  result[kImplicitViewId] = ViewContent{
      .viewport_metrics = {1.0, width, height, 22, 0},
      .builder = std::move(builder),
  };
  return result;
}

ShellTest::ShellTest()
    : thread_host_("io.flutter.test." + GetCurrentTestName() + ".",
                   ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                       ThreadHost::Type::kUi | ThreadHost::Type::kRaster) {}

void ShellTest::SendPlatformMessage(Shell* shell,
                                    std::unique_ptr<PlatformMessage> message) {
  shell->OnPlatformViewDispatchPlatformMessage(std::move(message));
}

void ShellTest::SendSemanticsAction(Shell* shell,
                                    int32_t node_id,
                                    SemanticsAction action,
                                    fml::MallocMapping args) {
  shell->OnPlatformViewDispatchSemanticsAction(node_id, action,
                                               std::move(args));
}

void ShellTest::SendEnginePlatformMessage(
    Shell* shell,
    std::unique_ptr<PlatformMessage> message) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      fml::MakeCopyable(
          [shell, &latch, message = std::move(message)]() mutable {
            if (auto engine = shell->weak_engine_) {
              engine->HandlePlatformMessage(std::move(message));
            }
            latch.Signal();
          }));
  latch.Wait();
}

void ShellTest::PlatformViewNotifyCreated(Shell* shell) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [shell, &latch]() {
        shell->GetPlatformView()->NotifyCreated();
        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::PlatformViewNotifyDestroyed(Shell* shell) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), [shell, &latch]() {
        shell->GetPlatformView()->NotifyDestroyed();
        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::RunEngine(Shell* shell, RunConfiguration configuration) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [shell, &latch, &configuration]() {
        shell->RunEngine(std::move(configuration),
                         [&latch](Engine::RunStatus run_status) {
                           ASSERT_EQ(run_status, Engine::RunStatus::Success);
                           latch.Signal();
                         });
      });
  latch.Wait();
}

void ShellTest::RestartEngine(Shell* shell, RunConfiguration configuration) {
  std::promise<bool> restarted;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetUITaskRunner(),
      [shell, &restarted, &configuration]() {
        restarted.set_value(shell->engine_->Restart(std::move(configuration)));
      });
  ASSERT_TRUE(restarted.get_future().get());
}

void ShellTest::VSyncFlush(Shell* shell, bool* will_draw_new_frame) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(),
      [shell, will_draw_new_frame, &latch] {
        // The following UI task ensures that all previous UI tasks are flushed.
        fml::AutoResetWaitableEvent ui_latch;
        shell->GetTaskRunners().GetUITaskRunner()->PostTask(
            [&ui_latch, will_draw_new_frame]() {
              if (will_draw_new_frame != nullptr) {
                *will_draw_new_frame = true;
              }
              ui_latch.Signal();
            });

        ShellTestPlatformView* test_platform_view =
            static_cast<ShellTestPlatformView*>(shell->GetPlatformView().get());
        do {
          test_platform_view->SimulateVSync();
        } while (ui_latch.WaitWithTimeout(fml::TimeDelta::FromMilliseconds(1)));

        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::SetViewportMetrics(Shell* shell, double width, double height) {
  flutter::ViewportMetrics viewport_metrics = {
      1,                      // device pixel ratio
      width,                  // physical width
      height,                 // physical height
      0,                      // padding top
      0,                      // padding right
      0,                      // padding bottom
      0,                      // padding left
      0,                      // view inset top
      0,                      // view inset right
      0,                      // view inset bottom
      0,                      // view inset left
      0,                      // gesture inset top
      0,                      // gesture inset right
      0,                      // gesture inset bottom
      0,                      // gesture inset left
      22,                     // physical touch slop
      std::vector<double>(),  // display features bounds
      std::vector<int>(),     // display features type
      std::vector<int>(),     // display features state
      0                       // Display ID
  };
  // Set viewport to nonempty, and call Animator::BeginFrame to make the layer
  // tree pipeline nonempty. Without either of this, the layer tree below
  // won't be rasterized.
  fml::AutoResetWaitableEvent latch;
  shell->GetTaskRunners().GetUITaskRunner()->PostTask(
      [&latch, engine = shell->weak_engine_, viewport_metrics]() {
        if (engine) {
          engine->SetViewportMetrics(kImplicitViewId, viewport_metrics);
          const auto frame_begin_time = fml::TimePoint::Now();
          const auto frame_end_time =
              frame_begin_time + fml::TimeDelta::FromSecondsF(1.0 / 60.0);
          std::unique_ptr<FrameTimingsRecorder> recorder =
              std::make_unique<FrameTimingsRecorder>();
          recorder->RecordVsync(frame_begin_time, frame_end_time);
          engine->animator_->BeginFrame(std::move(recorder));
          engine->animator_->EndFrame();
        }
        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::NotifyIdle(Shell* shell, fml::TimeDelta deadline) {
  fml::AutoResetWaitableEvent latch;
  shell->GetTaskRunners().GetUITaskRunner()->PostTask(
      [&latch, engine = shell->weak_engine_, deadline]() {
        if (engine) {
          engine->NotifyIdle(deadline);
        }
        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::PumpOneFrame(Shell* shell) {
  PumpOneFrame(shell, ViewContent::DummyView());
}

void ShellTest::PumpOneFrame(Shell* shell, FrameContent frame_content) {
  // Set viewport to nonempty, and call Animator::BeginFrame to make the layer
  // tree pipeline nonempty. Without either of this, the layer tree below
  // won't be rasterized.
  fml::AutoResetWaitableEvent latch;
  fml::WeakPtr<RuntimeDelegate> runtime_delegate = shell->weak_engine_;
  shell->GetTaskRunners().GetUITaskRunner()->PostTask(
      [&latch, engine = shell->weak_engine_, &frame_content,
       runtime_delegate]() {
        for (auto& [view_id, view_content] : frame_content) {
          engine->SetViewportMetrics(view_id, view_content.viewport_metrics);
        }
        const auto frame_begin_time = fml::TimePoint::Now();
        const auto frame_end_time =
            frame_begin_time + fml::TimeDelta::FromSecondsF(1.0 / 60.0);
        std::unique_ptr<FrameTimingsRecorder> recorder =
            std::make_unique<FrameTimingsRecorder>();
        recorder->RecordVsync(frame_begin_time, frame_end_time);
        engine->animator_->BeginFrame(std::move(recorder));

        // The BeginFrame phase and the EndFrame phase must be performed in a
        // single task, otherwise a normal vsync might be inserted in between,
        // causing flaky assertion errors.

        for (auto& [view_id, view_content] : frame_content) {
          auto root_layer = std::make_shared<TransformLayer>(DlMatrix());
          auto layer_tree = std::make_unique<LayerTree>(
              root_layer,
              DlISize(view_content.viewport_metrics.physical_width,
                      view_content.viewport_metrics.physical_height));
          float device_pixel_ratio = static_cast<float>(
              view_content.viewport_metrics.device_pixel_ratio);
          if (view_content.builder) {
            view_content.builder(root_layer);
          }
          runtime_delegate->Render(view_id, std::move(layer_tree),
                                   device_pixel_ratio);
        }
        engine->animator_->EndFrame();
        latch.Signal();
      });
  latch.Wait();
}

void ShellTest::DispatchFakePointerData(Shell* shell, double x) {
  auto packet = std::make_unique<PointerDataPacket>(1);
  packet->SetPointerData(0, PointerData{
                                .change = PointerData::Change::kHover,
                                .physical_x = x,
                            });
  DispatchPointerData(shell, std::move(packet));
}

void ShellTest::DispatchPointerData(Shell* shell,
                                    std::unique_ptr<PointerDataPacket> packet) {
  fml::AutoResetWaitableEvent latch;
  shell->GetTaskRunners().GetPlatformTaskRunner()->PostTask(
      [&latch, shell, &packet]() {
        // Goes through PlatformView to ensure packet is corrected converted.
        shell->GetPlatformView()->DispatchPointerDataPacket(std::move(packet));
        latch.Signal();
      });
  latch.Wait();
}

int ShellTest::UnreportedTimingsCount(Shell* shell) {
  return shell->unreported_timings_.size();
}

void ShellTest::SetNeedsReportTimings(Shell* shell, bool value) {
  shell->SetNeedsReportTimings(value);
}

bool ShellTest::GetNeedsReportTimings(Shell* shell) {
  return shell->needs_report_timings_;
}

void ShellTest::StorePersistentCache(PersistentCache* cache,
                                     const SkData& key,
                                     const SkData& value) {
  cache->store(key, value);
}

void ShellTest::OnServiceProtocol(
    Shell* shell,
    ServiceProtocolEnum some_protocol,
    const fml::RefPtr<fml::TaskRunner>& task_runner,
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  std::promise<bool> finished;
  fml::TaskRunner::RunNowOrPostTask(
      task_runner, [shell, some_protocol, params, response, &finished]() {
        switch (some_protocol) {
          case ServiceProtocolEnum::kGetSkSLs:
            shell->OnServiceProtocolGetSkSLs(params, response);
            break;
          case ServiceProtocolEnum::kEstimateRasterCacheMemory:
            shell->OnServiceProtocolEstimateRasterCacheMemory(params, response);
            break;
          case ServiceProtocolEnum::kSetAssetBundlePath:
            shell->OnServiceProtocolSetAssetBundlePath(params, response);
            break;
          case ServiceProtocolEnum::kRunInView:
            shell->OnServiceProtocolRunInView(params, response);
            break;
        }
        finished.set_value(true);
      });
  finished.get_future().wait();
}

std::shared_ptr<txt::FontCollection> ShellTest::GetFontCollection(
    Shell* shell) {
  return shell->weak_engine_->GetFontCollection().GetFontCollection();
}

Settings ShellTest::CreateSettingsForFixture() {
  Settings settings;
  settings.leak_vm = false;
  settings.task_observer_add = [](intptr_t key, const fml::closure& handler) {
    fml::MessageLoop::GetCurrent().AddTaskObserver(key, handler);
  };
  settings.task_observer_remove = [](intptr_t key) {
    fml::MessageLoop::GetCurrent().RemoveTaskObserver(key);
  };
  settings.isolate_create_callback = [this]() {
    native_resolver_->SetNativeResolverForIsolate();
  };
#if OS_FUCHSIA
  settings.verbose_logging = true;
#endif
  SetSnapshotsAndAssets(settings);
  return settings;
}

TaskRunners ShellTest::GetTaskRunnersForFixture() {
  return {
      "test",
      thread_host_.platform_thread->GetTaskRunner(),  // platform
      thread_host_.raster_thread->GetTaskRunner(),    // raster
      thread_host_.ui_thread->GetTaskRunner(),        // ui
      thread_host_.io_thread->GetTaskRunner()         // io
  };
}

fml::TimePoint ShellTest::GetLatestFrameTargetTime(Shell* shell) const {
  return shell->GetLatestFrameTargetTime();
}

std::unique_ptr<Shell> ShellTest::CreateShell(
    const Settings& settings,
    std::optional<TaskRunners> task_runners) {
  return CreateShell({
      .settings = settings,
      .task_runners = std::move(task_runners),
  });
}

std::unique_ptr<Shell> ShellTest::CreateShell(const Config& config) {
  TaskRunners task_runners = config.task_runners.has_value()
                                 ? config.task_runners.value()
                                 : GetTaskRunnersForFixture();
  Shell::CreateCallback<PlatformView> platform_view_create_callback =
      config.platform_view_create_callback;
  if (!platform_view_create_callback) {
    platform_view_create_callback = ShellTestPlatformViewBuilder({});
  }

  Shell::CreateCallback<Rasterizer> rasterizer_create_callback =
      [](Shell& shell) { return std::make_unique<Rasterizer>(shell); };

  return Shell::Create(flutter::PlatformData(),        //
                       task_runners,                   //
                       config.settings,                //
                       platform_view_create_callback,  //
                       rasterizer_create_callback,     //
                       config.is_gpu_disabled          //
  );
}

void ShellTest::DestroyShell(std::unique_ptr<Shell> shell) {
  DestroyShell(std::move(shell), GetTaskRunnersForFixture());
}

void ShellTest::DestroyShell(std::unique_ptr<Shell> shell,
                             const TaskRunners& task_runners) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(task_runners.GetPlatformTaskRunner(),
                                    [&shell, &latch]() mutable {
                                      shell.reset();
                                      latch.Signal();
                                    });
  latch.Wait();
}

void ShellTest::TurnOffGPU(Shell* shell, bool value) {
  shell->is_gpu_disabled_sync_switch_->SetSwitch(value);
}

}  // namespace testing
}  // namespace flutter
