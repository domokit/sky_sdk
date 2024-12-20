// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/engine.h"

// #include <cstring>

#include "flutter/common/constants.h"
#include "flutter/lib/ui/compositing/scene_builder.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/testing/fixture_test.h"
#include "gmock/gmock.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {

namespace {

using ::testing::Invoke;
using ::testing::ReturnRef;

fml::AutoResetWaitableEvent native_latch;

void PostSync(const fml::RefPtr<fml::TaskRunner>& task_runner,
              const fml::closure& task) {
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(task_runner, [&latch, &task] {
    task();
    latch.Signal();
  });
  latch.Wait();
}

// Sort the argument list of `LayerTreeTask` into a new list that is sorted by
// their view IDs. `FrameItem::layer_tree_tasks` might not come sorted.
std::vector<const LayerTreeTask*> Sorted(
    const std::vector<std::unique_ptr<LayerTreeTask>>& layer_tree_tasks) {
  std::vector<const LayerTreeTask*> result;
  result.reserve(layer_tree_tasks.size());
  for (auto& task_ptr : layer_tree_tasks) {
    result.push_back(task_ptr.get());
  }
  std::sort(result.begin(), result.end(),
            [](const LayerTreeTask* a, const LayerTreeTask* b) {
              return a->view_id < b->view_id;
            });
  return result;
}

class MockDelegate : public Engine::Delegate {
 public:
  MOCK_METHOD(void,
              OnEngineUpdateSemantics,
              (SemanticsNodeUpdates, CustomAccessibilityActionUpdates),
              (override));
  MOCK_METHOD(void,
              OnEngineHandlePlatformMessage,
              (std::unique_ptr<PlatformMessage>),
              (override));
  MOCK_METHOD(void, OnPreEngineRestart, (), (override));
  MOCK_METHOD(void, OnRootIsolateCreated, (), (override));
  MOCK_METHOD(void,
              UpdateIsolateDescription,
              (const std::string, int64_t),
              (override));
  MOCK_METHOD(void, SetNeedsReportTimings, (bool), (override));
  MOCK_METHOD(std::unique_ptr<std::vector<std::string>>,
              ComputePlatformResolvedLocale,
              (const std::vector<std::string>&),
              (override));
  MOCK_METHOD(void, RequestDartDeferredLibrary, (intptr_t), (override));
  MOCK_METHOD(fml::TimePoint, GetCurrentTimePoint, (), (override));
  MOCK_METHOD(const std::shared_ptr<PlatformMessageHandler>&,
              GetPlatformMessageHandler,
              (),
              (const, override));
  MOCK_METHOD(void, OnEngineChannelUpdate, (std::string, bool), (override));
  MOCK_METHOD(double,
              GetScaledFontSize,
              (double font_size, int configuration_id),
              (const, override));
};

class MockAnimatorDelegate : public Animator::Delegate {
 public:
  /* Animator::Delegate */
  MOCK_METHOD(void,
              OnAnimatorBeginFrame,
              (fml::TimePoint frame_target_time, uint64_t frame_number),
              (override));
  MOCK_METHOD(void,
              OnAnimatorNotifyIdle,
              (fml::TimeDelta deadline),
              (override));
  MOCK_METHOD(void,
              OnAnimatorUpdateLatestFrameTargetTime,
              (fml::TimePoint frame_target_time),
              (override));
  MOCK_METHOD(void,
              OnAnimatorDraw,
              (std::shared_ptr<FramePipeline> pipeline),
              (override));
  MOCK_METHOD(void,
              OnAnimatorDrawLastLayerTrees,
              (std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder),
              (override));
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

class EngineAnimatorTest : public testing::FixtureTest {
 public:
  EngineAnimatorTest()
      : thread_host_("EngineAnimatorTest",
                     ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                         ThreadHost::Type::kUi | ThreadHost::Type::kRaster),
        task_runners_({
            "EngineAnimatorTest",
            thread_host_.platform_thread->GetTaskRunner(),  // platform
            thread_host_.raster_thread->GetTaskRunner(),    // raster
            thread_host_.ui_thread->GetTaskRunner(),        // ui
            thread_host_.io_thread->GetTaskRunner()         // io
        }) {}

  void PostUITaskSync(const std::function<void()>& function) {
    fml::AutoResetWaitableEvent latch;
    task_runners_.GetUITaskRunner()->PostTask([&] {
      function();
      latch.Signal();
    });
    latch.Wait();
  }

 protected:
  void SetUp() override {
    settings_ = CreateSettingsForFixture();
    dispatcher_maker_ = [](PointerDataDispatcher::Delegate&) {
      return nullptr;
    };
  }

  MockDelegate delegate_;
  PointerDataDispatcherMaker dispatcher_maker_;
  ThreadHost thread_host_;
  TaskRunners task_runners_;
  Settings settings_;
  std::unique_ptr<Animator> animator_;
  fml::WeakPtr<IOManager> io_manager_;
  std::unique_ptr<RuntimeController> runtime_controller_;
  std::shared_ptr<fml::ConcurrentTaskRunner> image_decoder_task_runner_;
  fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate_;
};

// A class that can launch an Engine with the specified Engine::Delegate.
//
// To use this class, contruct this class with Create, call Run, and use the
// engine with EngineTaskSync().
class EngineContext {
 public:
  using EngineCallback = std::function<void(Engine&)>;

  [[nodiscard]] static std::unique_ptr<EngineContext> Create(
      Engine::Delegate& delegate,       //
      Settings settings,                //
      const TaskRunners& task_runners,  //
      std::unique_ptr<Animator> animator) {
    auto [vm, isolate_snapshot] = Shell::InferVmInitDataFromSettings(settings);
    FML_CHECK(vm) << "Must be able to initialize the VM.";
    // Construct the class with `new` because `make_unique` has no access to the
    // private constructor.
    EngineContext* raw_pointer =
        new EngineContext(delegate, settings, task_runners, std::move(animator),
                          vm, isolate_snapshot);
    return std::unique_ptr<EngineContext>(raw_pointer);
  }

  void Run(RunConfiguration configuration) {
    PostSync(task_runners_.GetUITaskRunner(), [this, &configuration] {
      Engine::RunStatus run_status = engine_->Run(std::move(configuration));
      FML_CHECK(run_status == Engine::RunStatus::Success)
          << "Engine failed to run.";
      (void)run_status;  // Suppress unused-variable warning
    });
  }

  // Run a task that operates the Engine on the UI thread, and wait for the
  // task to end.
  //
  // If called on the UI thread, the task is executed synchronously.
  void EngineTaskSync(EngineCallback task) {
    ASSERT_TRUE(engine_);
    ASSERT_TRUE(task);
    auto runner = task_runners_.GetUITaskRunner();
    if (runner->RunsTasksOnCurrentThread()) {
      task(*engine_);
    } else {
      PostSync(task_runners_.GetUITaskRunner(), [&]() { task(*engine_); });
    }
  }

  ~EngineContext() {
    PostSync(task_runners_.GetUITaskRunner(), [this] { engine_.reset(); });
  }

 private:
  EngineContext(Engine::Delegate& delegate,          //
                Settings settings,                   //
                const TaskRunners& task_runners,     //
                std::unique_ptr<Animator> animator,  //
                const DartVMRef& vm,                 //
                fml::RefPtr<const DartSnapshot> isolate_snapshot)
      : task_runners_(task_runners), vm_(vm) {
    PostSync(task_runners.GetUITaskRunner(), [this, &settings, &animator,
                                              &delegate, &isolate_snapshot] {
      auto dispatcher_maker =
          [](DefaultPointerDataDispatcher::Delegate& delegate) {
            return std::make_unique<DefaultPointerDataDispatcher>(delegate);
          };
      engine_ = std::make_unique<Engine>(
          /*delegate=*/delegate,
          /*dispatcher_maker=*/dispatcher_maker,
          /*vm=*/*&vm_,
          /*isolate_snapshot=*/std::move(isolate_snapshot),
          /*task_runners=*/task_runners_,
          /*platform_data=*/PlatformData(),
          /*settings=*/settings,
          /*animator=*/std::move(animator),
          /*io_manager=*/io_manager_,
          /*unref_queue=*/nullptr,
          /*snapshot_delegate=*/snapshot_delegate_,
          /*gpu_disabled_switch=*/std::make_shared<fml::SyncSwitch>());
    });
  }

  TaskRunners task_runners_;
  DartVMRef vm_;
  std::unique_ptr<Engine> engine_;

  fml::WeakPtr<IOManager> io_manager_;
  fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate_;
};
}  // namespace

TEST_F(EngineAnimatorTest, AnimatorAcceptsMultipleRenders) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));
  fml::AutoResetWaitableEvent draw_latch;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(
          Invoke([&draw_latch](const std::shared_ptr<FramePipeline>& pipeline) {
            auto status =
                pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
                  auto tasks = Sorted(item->layer_tree_tasks);
                  EXPECT_EQ(tasks.size(), 2u);
                  EXPECT_EQ(tasks[0]->view_id, 1);
                  EXPECT_EQ(tasks[1]->view_id, 2);
                });
            EXPECT_EQ(status, PipelineConsumeResult::Done);
            draw_latch.Signal();
          }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillOnce(Invoke([&engine_context](fml::TimePoint frame_target_time,
                                         uint64_t frame_number) {
        engine_context->EngineTaskSync([&](Engine& engine) {
          engine.BeginFrame(frame_target_time, frame_number);
        });
      }));

  native_latch.Reset();
  AddNativeCallback("NotifyNative", [](auto args) { native_latch.Signal(); });

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));
  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("onDrawFrameRenderAllViews");
  engine_context->Run(std::move(configuration));

  engine_context->EngineTaskSync([](Engine& engine) {
    engine.AddView(1, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
    engine.AddView(2, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });

  native_latch.Wait();

  engine_context->EngineTaskSync(
      [](Engine& engine) { engine.ScheduleFrame(); });
  draw_latch.Wait();
}

TEST_F(EngineAnimatorTest, IgnoresOutOfFrameRenders) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));
  fml::AutoResetWaitableEvent draw_latch;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(
          Invoke([&draw_latch](const std::shared_ptr<FramePipeline>& pipeline) {
            auto status =
                pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
                  // View 1 is rendered before the frame, and is ignored.
                  // View 2 is rendered within the frame, and is accepted.
                  EXPECT_EQ(item->layer_tree_tasks.size(), 1u);
                  EXPECT_EQ(item->layer_tree_tasks[0]->view_id, 2);
                });
            EXPECT_EQ(status, PipelineConsumeResult::Done);
            draw_latch.Signal();
          }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillOnce(Invoke([&engine_context](fml::TimePoint frame_target_time,
                                         uint64_t frame_number) {
        engine_context->EngineTaskSync([&](Engine& engine) {
          engine.BeginFrame(frame_target_time, frame_number);
        });
      }));

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));

  engine_context->EngineTaskSync([](Engine& engine) {
    engine.AddView(1, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
    engine.AddView(2, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });

  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("renderViewsInFrameAndOutOfFrame");
  engine_context->Run(std::move(configuration));

  draw_latch.Wait();
}

TEST_F(EngineAnimatorTest, IgnoresDuplicateRenders) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::vector<std::shared_ptr<Layer>> benchmark_layers;
  auto capture_root_layer = [&benchmark_layers](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    SceneBuilder* scene_builder = reinterpret_cast<SceneBuilder*>(peer);
    ASSERT_TRUE(scene_builder);
    std::shared_ptr<ContainerLayer> root_layer =
        scene_builder->layer_stack()[0];
    ASSERT_TRUE(root_layer);
    benchmark_layers = root_layer->layers();
  };

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));
  fml::AutoResetWaitableEvent draw_latch;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(Invoke([&draw_latch, &benchmark_layers](
                           const std::shared_ptr<FramePipeline>& pipeline) {
        auto status = pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
          EXPECT_EQ(item->layer_tree_tasks.size(), 1u);
          EXPECT_EQ(item->layer_tree_tasks[0]->view_id, kFlutterImplicitViewId);
          ContainerLayer* root_layer = reinterpret_cast<ContainerLayer*>(
              item->layer_tree_tasks[0]->layer_tree->root_layer());
          std::vector<std::shared_ptr<Layer>> result_layers =
              root_layer->layers();
          EXPECT_EQ(result_layers.size(), benchmark_layers.size());
          EXPECT_EQ(result_layers[0], benchmark_layers[0]);
        });
        EXPECT_EQ(status, PipelineConsumeResult::Done);
        draw_latch.Signal();
      }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillOnce(Invoke([&engine_context](fml::TimePoint frame_target_time,
                                         uint64_t frame_number) {
        engine_context->EngineTaskSync([&](Engine& engine) {
          engine.BeginFrame(frame_target_time, frame_number);
        });
      }));

  AddNativeCallback("CaptureRootLayer",
                    CREATE_NATIVE_ENTRY(capture_root_layer));

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));

  engine_context->EngineTaskSync([](Engine& engine) {
    engine.AddView(kFlutterImplicitViewId, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });

  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("renderTwiceForOneView");
  engine_context->Run(std::move(configuration));

  draw_latch.Wait();
}

TEST_F(EngineAnimatorTest, AnimatorSubmitsImplicitViewBeforeDrawFrameEnds) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));

  bool rasterization_started = false;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(Invoke([&rasterization_started](
                           const std::shared_ptr<FramePipeline>& pipeline) {
        rasterization_started = true;
        auto status = pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
          EXPECT_EQ(item->layer_tree_tasks.size(), 1u);
          EXPECT_EQ(item->layer_tree_tasks[0]->view_id, kFlutterImplicitViewId);
        });
        EXPECT_EQ(status, PipelineConsumeResult::Done);
      }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillRepeatedly(Invoke([&engine_context](fml::TimePoint frame_target_time,
                                               uint64_t frame_number) {
        engine_context->EngineTaskSync([&](Engine& engine) {
          engine.BeginFrame(frame_target_time, frame_number);
        });
      }));

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  native_latch.Reset();
  // The native_latch is signaled at the end of handleDrawFrame.
  AddNativeCallback("NotifyNative",
                    CREATE_NATIVE_ENTRY([&rasterization_started](auto args) {
                      EXPECT_EQ(rasterization_started, true);
                      native_latch.Signal();
                    }));

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));

  engine_context->EngineTaskSync([](Engine& engine) {
    engine.AddView(kFlutterImplicitViewId, ViewportMetrics{1.0, 10, 10, 1, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });

  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("renderSingleViewAndCallAfterOnDrawFrame");
  engine_context->Run(std::move(configuration));

  native_latch.Wait();
}

// The animator should submit to the pipeline the implicit view rendered in a
// warm up frame if there's already a continuation (i.e. Animator::BeginFrame
// has been called)
TEST_F(EngineAnimatorTest, AnimatorSubmitWarmUpImplicitView) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));

  fml::AutoResetWaitableEvent continuation_ready_latch;
  fml::AutoResetWaitableEvent draw_latch;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(Invoke([&draw_latch](
                           const std::shared_ptr<FramePipeline>& pipeline) {
        auto status = pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
          EXPECT_EQ(item->layer_tree_tasks.size(), 1u);
          EXPECT_EQ(item->layer_tree_tasks[0]->view_id, kFlutterImplicitViewId);
        });
        EXPECT_EQ(status, PipelineConsumeResult::Done);
        draw_latch.Signal();
      }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillRepeatedly(
          Invoke([&engine_context, &continuation_ready_latch](
                     fml::TimePoint frame_target_time, uint64_t frame_number) {
            continuation_ready_latch.Signal();
            engine_context->EngineTaskSync([&](Engine& engine) {
              engine.BeginFrame(frame_target_time, frame_number);
            });
          }));

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));

  engine_context->EngineTaskSync([](Engine& engine) {
    // Schedule a frame to trigger Animator::BeginFrame to create a
    // continuation. The continuation needs to be available before `Engine::Run`
    // since the Dart program immediately schedules a warm up frame.
    engine.ScheduleFrame(true);
    // Add the implicit view so that the engine recognizes it and that its
    // metrics is not empty.
    engine.AddView(kFlutterImplicitViewId, ViewportMetrics{1.0, 10, 10, 1, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });
  continuation_ready_latch.Wait();

  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("renderWarmUpImplicitView");
  engine_context->Run(std::move(configuration));

  draw_latch.Wait();
}

// The warm up frame should work if only some of the registered views are
// included.
//
// This test also verifies that the warm up frame can render multiple views.
TEST_F(EngineAnimatorTest, AnimatorSubmitPartialViewsForWarmUp) {
  MockAnimatorDelegate animator_delegate;
  std::unique_ptr<EngineContext> engine_context;

  std::shared_ptr<PlatformMessageHandler> platform_message_handler =
      std::make_shared<MockPlatformMessageHandler>();
  EXPECT_CALL(delegate_, GetPlatformMessageHandler)
      .WillOnce(ReturnRef(platform_message_handler));

  fml::AutoResetWaitableEvent continuation_ready_latch;
  fml::AutoResetWaitableEvent draw_latch;
  EXPECT_CALL(animator_delegate, OnAnimatorDraw)
      .WillOnce(
          Invoke([&draw_latch](const std::shared_ptr<FramePipeline>& pipeline) {
            auto status =
                pipeline->Consume([&](std::unique_ptr<FrameItem> item) {
                  auto tasks = Sorted(item->layer_tree_tasks);
                  EXPECT_EQ(tasks.size(), 2u);
                  EXPECT_EQ(tasks[0]->view_id, 1);
                  EXPECT_EQ(tasks[1]->view_id, 2);
                });
            EXPECT_EQ(status, PipelineConsumeResult::Done);
            draw_latch.Signal();
          }));
  EXPECT_CALL(animator_delegate, OnAnimatorBeginFrame)
      .WillRepeatedly(
          Invoke([&engine_context, &continuation_ready_latch](
                     fml::TimePoint frame_target_time, uint64_t frame_number) {
            continuation_ready_latch.Signal();
            engine_context->EngineTaskSync([&](Engine& engine) {
              engine.BeginFrame(frame_target_time, frame_number);
            });
          }));

  std::unique_ptr<Animator> animator;
  PostSync(task_runners_.GetUITaskRunner(),
           [&animator, &animator_delegate, &task_runners = task_runners_] {
             animator = std::make_unique<Animator>(
                 animator_delegate, task_runners,
                 static_cast<std::unique_ptr<VsyncWaiter>>(
                     std::make_unique<testing::ConstantFiringVsyncWaiter>(
                         task_runners)));
           });

  engine_context = EngineContext::Create(delegate_, settings_, task_runners_,
                                         std::move(animator));

  engine_context->EngineTaskSync([](Engine& engine) {
    // Schedule a frame to make the animator create a continuation.
    engine.ScheduleFrame(true);
    // Add multiple views.
    engine.AddView(0, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
    engine.AddView(1, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
    engine.AddView(2, ViewportMetrics{1, 10, 10, 22, 0},
                   [](bool added) { ASSERT_TRUE(added); });
  });

  continuation_ready_latch.Wait();

  auto configuration = RunConfiguration::InferFromSettings(settings_);
  configuration.SetEntrypoint("renderWarmUpView1and2");
  engine_context->Run(std::move(configuration));

  draw_latch.Wait();
}

}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
