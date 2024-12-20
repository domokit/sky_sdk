// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fml/task_runner.h"
#define RAPIDJSON_HAS_STDSTRING 1
#include "flutter/shell/common/shell.h"

#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include "flutter/assets/directory_asset_bundle.h"
#include "flutter/common/constants.h"
#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/fml/base32.h"
#include "flutter/fml/file.h"
#include "flutter/fml/icu_util.h"
#include "flutter/fml/log_settings.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/trace_event.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/base64.h"
#include "flutter/shell/common/engine.h"
#include "flutter/shell/common/skia_event_tracer_impl.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/common/vsync_waiter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"
#include "third_party/skia/include/codec/SkBmpDecoder.h"
#include "third_party/skia/include/codec/SkCodec.h"
#include "third_party/skia/include/codec/SkGifDecoder.h"
#include "third_party/skia/include/codec/SkIcoDecoder.h"
#include "third_party/skia/include/codec/SkJpegDecoder.h"
#include "third_party/skia/include/codec/SkPngDecoder.h"
#include "third_party/skia/include/codec/SkWbmpDecoder.h"
#include "third_party/skia/include/codec/SkWebpDecoder.h"
#include "third_party/skia/include/core/SkGraphics.h"
#include "third_party/tonic/common/log.h"

namespace flutter {

constexpr char kSkiaChannel[] = "flutter/skia";
constexpr char kSystemChannel[] = "flutter/system";
constexpr char kTypeKey[] = "type";
constexpr char kFontChange[] = "fontsChange";

namespace {

std::unique_ptr<Engine> CreateEngine(
    Engine::Delegate& delegate,
    const PointerDataDispatcherMaker& dispatcher_maker,
    DartVM& vm,
    const fml::RefPtr<const DartSnapshot>& isolate_snapshot,
    const TaskRunners& task_runners,
    const PlatformData& platform_data,
    const Settings& settings,
    std::unique_ptr<Animator> animator,
    const fml::WeakPtr<IOManager>& io_manager,
    const fml::RefPtr<SkiaUnrefQueue>& unref_queue,
    const fml::TaskRunnerAffineWeakPtr<SnapshotDelegate>& snapshot_delegate,
    const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch,
    impeller::RuntimeStageBackend runtime_stage_backend) {
  return std::make_unique<Engine>(delegate,             //
                                  dispatcher_maker,     //
                                  vm,                   //
                                  isolate_snapshot,     //
                                  task_runners,         //
                                  platform_data,        //
                                  settings,             //
                                  std::move(animator),  //
                                  io_manager,           //
                                  unref_queue,          //
                                  snapshot_delegate,    //
                                  gpu_disabled_switch,  //
                                  runtime_stage_backend);
}

void RegisterCodecsWithSkia() {
  // These are in the order they will be attempted to be decoded from.
  // If we have data to back it up, we can order these by "frequency used in
  // the wild" for a very small performance bump, but for now we mirror the
  // order Skia had them in.
  SkCodecs::Register(SkPngDecoder::Decoder());
  SkCodecs::Register(SkJpegDecoder::Decoder());
  SkCodecs::Register(SkWebpDecoder::Decoder());
  SkCodecs::Register(SkGifDecoder::Decoder());
  SkCodecs::Register(SkBmpDecoder::Decoder());
  SkCodecs::Register(SkWbmpDecoder::Decoder());
  SkCodecs::Register(SkIcoDecoder::Decoder());
}

// Though there can be multiple shells, some settings apply to all components in
// the process. These have to be set up before the shell or any of its
// sub-components can be initialized. In a perfect world, this would be empty.
// TODO(chinmaygarde): The unfortunate side effect of this call is that settings
// that cause shell initialization failures will still lead to some of their
// settings being applied.
void PerformInitializationTasks(Settings& settings) {
  {
    fml::LogSettings log_settings;
    log_settings.min_log_level =
        settings.verbose_logging ? fml::kLogInfo : fml::kLogError;
    fml::SetLogSettings(log_settings);
  }

  static std::once_flag gShellSettingsInitialization = {};
  std::call_once(gShellSettingsInitialization, [&settings] {
    tonic::SetLogHandler(
        [](const char* message) { FML_LOG(ERROR) << message; });

    if (settings.trace_skia) {
      InitSkiaEventTracer(settings.trace_skia, settings.trace_skia_allowlist);
    }

    if (!settings.trace_allowlist.empty()) {
      fml::tracing::TraceSetAllowlist(settings.trace_allowlist);
    }

    if (!settings.skia_deterministic_rendering_on_cpu) {
      SkGraphics::Init();
    } else {
      FML_DLOG(INFO) << "Skia deterministic rendering is enabled.";
    }
    RegisterCodecsWithSkia();

    if (settings.icu_initialization_required) {
      if (!settings.icu_data_path.empty()) {
        fml::icu::InitializeICU(settings.icu_data_path);
      } else if (settings.icu_mapper) {
        fml::icu::InitializeICUFromMapping(settings.icu_mapper());
      } else {
        FML_DLOG(WARNING) << "Skipping ICU initialization in the shell.";
      }
    }
  });

#if !SLIMPELLER
  PersistentCache::SetCacheSkSL(settings.cache_sksl);
#endif  //  !SLIMPELLER
}

}  // namespace

std::pair<DartVMRef, fml::RefPtr<const DartSnapshot>>
Shell::InferVmInitDataFromSettings(Settings& settings) {
  // Always use the `vm_snapshot` and `isolate_snapshot` provided by the
  // settings to launch the VM.  If the VM is already running, the snapshot
  // arguments are ignored.
  auto vm_snapshot = DartSnapshot::VMSnapshotFromSettings(settings);
  auto isolate_snapshot = DartSnapshot::IsolateSnapshotFromSettings(settings);
  auto vm = DartVMRef::Create(settings, vm_snapshot, isolate_snapshot);

  // If the settings did not specify an `isolate_snapshot`, fall back to the
  // one the VM was launched with.
  if (!isolate_snapshot) {
    isolate_snapshot = vm->GetVMData()->GetIsolateSnapshot();
  }
  return {std::move(vm), isolate_snapshot};
}

std::unique_ptr<Shell> Shell::Create(
    const PlatformData& platform_data,
    const TaskRunners& task_runners,
    Settings settings,
    const Shell::CreateCallback<PlatformView>& on_create_platform_view,
    const Shell::CreateCallback<Rasterizer>& on_create_rasterizer,
    bool is_gpu_disabled) {
  // This must come first as it initializes tracing.
  PerformInitializationTasks(settings);

  TRACE_EVENT0("flutter", "Shell::Create");

  auto [vm, isolate_snapshot] = InferVmInitDataFromSettings(settings);
  auto resource_cache_limit_calculator =
      std::make_shared<ResourceCacheLimitCalculator>(
          settings.resource_cache_max_bytes_threshold);

  return CreateWithSnapshot(platform_data,                     //
                            task_runners,                      //
                            /*parent_thread_merger=*/nullptr,  //
                            /*parent_io_manager=*/nullptr,     //
                            resource_cache_limit_calculator,   //
                            settings,                          //
                            std::move(vm),                     //
                            std::move(isolate_snapshot),       //
                            on_create_platform_view,           //
                            on_create_rasterizer,              //
                            CreateEngine, is_gpu_disabled);
}

static impeller::RuntimeStageBackend DetermineRuntimeStageBackend(
    const std::shared_ptr<impeller::Context>& impeller_context) {
  if (!impeller_context) {
    return impeller::RuntimeStageBackend::kSkSL;
  }
  return impeller_context->GetRuntimeStageBackend();
}

std::unique_ptr<Shell> Shell::CreateShellOnPlatformThread(
    DartVMRef vm,
    fml::RefPtr<fml::RasterThreadMerger> parent_merger,
    std::shared_ptr<ShellIOManager> parent_io_manager,
    const std::shared_ptr<ResourceCacheLimitCalculator>&
        resource_cache_limit_calculator,
    const TaskRunners& task_runners,
    const PlatformData& platform_data,
    const Settings& settings,
    fml::RefPtr<const DartSnapshot> isolate_snapshot,
    const Shell::CreateCallback<PlatformView>& on_create_platform_view,
    const Shell::CreateCallback<Rasterizer>& on_create_rasterizer,
    const Shell::EngineCreateCallback& on_create_engine,
    bool is_gpu_disabled) {
  if (!task_runners.IsValid()) {
    FML_LOG(ERROR) << "Task runners to run the shell were invalid.";
    return nullptr;
  }

  auto shell = std::unique_ptr<Shell>(
      new Shell(std::move(vm), task_runners, std::move(parent_merger),
                resource_cache_limit_calculator, settings, is_gpu_disabled));

  // Create the platform view on the platform thread (this thread).
  auto platform_view = on_create_platform_view(*shell.get());
  if (!platform_view || !platform_view->GetWeakPtr()) {
    return nullptr;
  }

  // Create the rasterizer on the raster thread.
  std::promise<std::unique_ptr<Rasterizer>> rasterizer_promise;
  auto rasterizer_future = rasterizer_promise.get_future();
  std::promise<fml::TaskRunnerAffineWeakPtr<SnapshotDelegate>>
      snapshot_delegate_promise;
  auto snapshot_delegate_future = snapshot_delegate_promise.get_future();
  fml::TaskRunner::RunNowOrPostTask(
      task_runners.GetRasterTaskRunner(),
      [&rasterizer_promise,  //
       &snapshot_delegate_promise,
       on_create_rasterizer,                                   //
       shell = shell.get(),                                    //
       impeller_context = platform_view->GetImpellerContext()  //
  ]() {
        TRACE_EVENT0("flutter", "ShellSetupGPUSubsystem");
        std::unique_ptr<Rasterizer> rasterizer(on_create_rasterizer(*shell));
        rasterizer->SetImpellerContext(impeller_context);
        snapshot_delegate_promise.set_value(rasterizer->GetSnapshotDelegate());
        rasterizer_promise.set_value(std::move(rasterizer));
      });

  // Ask the platform view for the vsync waiter. This will be used by the engine
  // to create the animator.
  auto vsync_waiter = platform_view->CreateVSyncWaiter();
  if (!vsync_waiter) {
    return nullptr;
  }

  // Create the IO manager on the IO thread. The IO manager must be initialized
  // first because it has state that the other subsystems depend on. It must
  // first be booted and the necessary references obtained to initialize the
  // other subsystems.
  std::promise<std::shared_ptr<ShellIOManager>> io_manager_promise;
  auto io_manager_future = io_manager_promise.get_future();
  std::promise<fml::WeakPtr<ShellIOManager>> weak_io_manager_promise;
  auto weak_io_manager_future = weak_io_manager_promise.get_future();
  std::promise<fml::RefPtr<SkiaUnrefQueue>> unref_queue_promise;
  auto unref_queue_future = unref_queue_promise.get_future();
  auto io_task_runner = shell->GetTaskRunners().GetIOTaskRunner();

  // The platform_view will be stored into shell's platform_view_ in
  // shell->Setup(std::move(platform_view), ...) at the end.
  PlatformView* platform_view_ptr = platform_view.get();
  fml::TaskRunner::RunNowOrPostTask(
      io_task_runner,
      [&io_manager_promise,                                               //
       &weak_io_manager_promise,                                          //
       &parent_io_manager,                                                //
       &unref_queue_promise,                                              //
       platform_view_ptr,                                                 //
       io_task_runner,                                                    //
       is_backgrounded_sync_switch = shell->GetIsGpuDisabledSyncSwitch()  //
  ]() {
        TRACE_EVENT0("flutter", "ShellSetupIOSubsystem");
        std::shared_ptr<ShellIOManager> io_manager;
        if (parent_io_manager) {
          io_manager = parent_io_manager;
        } else {
          io_manager = std::make_shared<ShellIOManager>(
              platform_view_ptr->CreateResourceContext(),  // resource context
              is_backgrounded_sync_switch,                 // sync switch
              io_task_runner,  // unref queue task runner
              platform_view_ptr->GetImpellerContext()  // impeller context
          );
        }
        weak_io_manager_promise.set_value(io_manager->GetWeakPtr());
        unref_queue_promise.set_value(io_manager->GetSkiaUnrefQueue());
        io_manager_promise.set_value(io_manager);
      });

  // Send dispatcher_maker to the engine constructor because shell won't have
  // platform_view set until Shell::Setup is called later.
  auto dispatcher_maker = platform_view->GetDispatcherMaker();

  // Create the engine on the UI thread.
  std::promise<std::unique_ptr<Engine>> engine_promise;
  auto engine_future = engine_promise.get_future();
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetUITaskRunner(),
      fml::MakeCopyable([&engine_promise,                                 //
                         shell = shell.get(),                             //
                         &dispatcher_maker,                               //
                         &platform_data,                                  //
                         isolate_snapshot = std::move(isolate_snapshot),  //
                         vsync_waiter = std::move(vsync_waiter),          //
                         &weak_io_manager_future,                         //
                         &snapshot_delegate_future,                       //
                         &unref_queue_future,                             //
                         &on_create_engine,
                         runtime_stage_backend = DetermineRuntimeStageBackend(
                             platform_view->GetImpellerContext())]() mutable {
        TRACE_EVENT0("flutter", "ShellSetupUISubsystem");
        const auto& task_runners = shell->GetTaskRunners();

        // The animator is owned by the UI thread but it gets its vsync pulses
        // from the platform.
        auto animator = std::make_unique<Animator>(*shell, task_runners,
                                                   std::move(vsync_waiter));

        engine_promise.set_value(on_create_engine(
            *shell,                               //
            dispatcher_maker,                     //
            *shell->GetDartVM(),                  //
            std::move(isolate_snapshot),          //
            task_runners,                         //
            platform_data,                        //
            shell->GetSettings(),                 //
            std::move(animator),                  //
            weak_io_manager_future.get(),         //
            unref_queue_future.get(),             //
            snapshot_delegate_future.get(),       //
            shell->is_gpu_disabled_sync_switch_,  //
            runtime_stage_backend                 //
            ));
      }));

  if (!shell->Setup(std::move(platform_view),  //
                    engine_future.get(),       //
                    rasterizer_future.get(),   //
                    io_manager_future.get())   //
  ) {
    return nullptr;
  }

  return shell;
}

std::unique_ptr<Shell> Shell::CreateWithSnapshot(
    const PlatformData& platform_data,
    const TaskRunners& task_runners,
    const fml::RefPtr<fml::RasterThreadMerger>& parent_thread_merger,
    const std::shared_ptr<ShellIOManager>& parent_io_manager,
    const std::shared_ptr<ResourceCacheLimitCalculator>&
        resource_cache_limit_calculator,
    Settings settings,
    DartVMRef vm,
    fml::RefPtr<const DartSnapshot> isolate_snapshot,
    const Shell::CreateCallback<PlatformView>& on_create_platform_view,
    const Shell::CreateCallback<Rasterizer>& on_create_rasterizer,
    const Shell::EngineCreateCallback& on_create_engine,
    bool is_gpu_disabled) {
  // This must come first as it initializes tracing.
  PerformInitializationTasks(settings);

  TRACE_EVENT0("flutter", "Shell::CreateWithSnapshot");

  const bool callbacks_valid =
      on_create_platform_view && on_create_rasterizer && on_create_engine;
  if (!task_runners.IsValid() || !callbacks_valid) {
    return nullptr;
  }

  fml::AutoResetWaitableEvent latch;
  std::unique_ptr<Shell> shell;
  auto platform_task_runner = task_runners.GetPlatformTaskRunner();
  fml::TaskRunner::RunNowOrPostTask(
      platform_task_runner,
      fml::MakeCopyable([&latch,                                             //
                         &shell,                                             //
                         parent_thread_merger,                               //
                         parent_io_manager,                                  //
                         resource_cache_limit_calculator,                    //
                         task_runners = task_runners,                        //
                         platform_data = platform_data,                      //
                         settings = settings,                                //
                         vm = std::move(vm),                                 //
                         isolate_snapshot = std::move(isolate_snapshot),     //
                         on_create_platform_view = on_create_platform_view,  //
                         on_create_rasterizer = on_create_rasterizer,        //
                         on_create_engine = on_create_engine,
                         is_gpu_disabled]() mutable {
        shell = CreateShellOnPlatformThread(std::move(vm),                    //
                                            parent_thread_merger,             //
                                            parent_io_manager,                //
                                            resource_cache_limit_calculator,  //
                                            task_runners,                     //
                                            platform_data,                    //
                                            settings,                         //
                                            std::move(isolate_snapshot),      //
                                            on_create_platform_view,          //
                                            on_create_rasterizer,             //
                                            on_create_engine, is_gpu_disabled);
        latch.Signal();
      }));
  latch.Wait();
  return shell;
}

Shell::Shell(DartVMRef vm,
             const TaskRunners& task_runners,
             fml::RefPtr<fml::RasterThreadMerger> parent_merger,
             const std::shared_ptr<ResourceCacheLimitCalculator>&
                 resource_cache_limit_calculator,
             const Settings& settings,
             bool is_gpu_disabled)
    : task_runners_(task_runners),
      parent_raster_thread_merger_(std::move(parent_merger)),
      resource_cache_limit_calculator_(resource_cache_limit_calculator),
      settings_(settings),
      vm_(std::move(vm)),
      is_gpu_disabled_sync_switch_(new fml::SyncSwitch(is_gpu_disabled)),
      weak_factory_gpu_(nullptr),
      weak_factory_(this) {
  FML_CHECK(!settings.enable_software_rendering || !settings.enable_impeller)
      << "Software rendering is incompatible with Impeller.";
  if (!settings.enable_impeller && settings.warn_on_impeller_opt_out) {
    FML_LOG(IMPORTANT)
        << "[Action Required] The application opted out of Impeller by either "
           "using the --no-enable-impeller flag or FLTEnableImpeller=false "
           "plist flag. This option is going to go away in an upcoming Flutter "
           "release. Remove the explicit opt-out. If you need to opt-out, "
           "report a bug describing the issue.";
  }
  FML_CHECK(vm_) << "Must have access to VM to create a shell.";
  FML_DCHECK(task_runners_.IsValid());
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  display_manager_ = std::make_unique<DisplayManager>();
  resource_cache_limit_calculator->AddResourceCacheLimitItem(
      weak_factory_.GetWeakPtr());

  // Generate a WeakPtrFactory for use with the raster thread. This does not
  // need to wait on a latch because it can only ever be used from the raster
  // thread from this class, so we have ordering guarantees.
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetRasterTaskRunner(), fml::MakeCopyable([this]() mutable {
        this->weak_factory_gpu_ =
            std::make_unique<fml::TaskRunnerAffineWeakPtrFactory<Shell>>(this);
      }));

  // Install service protocol handlers.

  service_protocol_handlers_[ServiceProtocol::kScreenshotExtensionName] = {
      task_runners_.GetRasterTaskRunner(),
      std::bind(&Shell::OnServiceProtocolScreenshot, this,
                std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_[ServiceProtocol::kScreenshotSkpExtensionName] = {
      task_runners_.GetRasterTaskRunner(),
      std::bind(&Shell::OnServiceProtocolScreenshotSKP, this,
                std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_[ServiceProtocol::kRunInViewExtensionName] = {
      task_runners_.GetUITaskRunner(),
      std::bind(&Shell::OnServiceProtocolRunInView, this, std::placeholders::_1,
                std::placeholders::_2)};
  service_protocol_handlers_
      [ServiceProtocol::kFlushUIThreadTasksExtensionName] = {
          task_runners_.GetUITaskRunner(),
          std::bind(&Shell::OnServiceProtocolFlushUIThreadTasks, this,
                    std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_
      [ServiceProtocol::kSetAssetBundlePathExtensionName] = {
          task_runners_.GetUITaskRunner(),
          std::bind(&Shell::OnServiceProtocolSetAssetBundlePath, this,
                    std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_
      [ServiceProtocol::kGetDisplayRefreshRateExtensionName] = {
          task_runners_.GetUITaskRunner(),
          std::bind(&Shell::OnServiceProtocolGetDisplayRefreshRate, this,
                    std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_[ServiceProtocol::kGetSkSLsExtensionName] = {
      task_runners_.GetIOTaskRunner(),
      std::bind(&Shell::OnServiceProtocolGetSkSLs, this, std::placeholders::_1,
                std::placeholders::_2)};
  service_protocol_handlers_
      [ServiceProtocol::kEstimateRasterCacheMemoryExtensionName] = {
          task_runners_.GetRasterTaskRunner(),
          std::bind(&Shell::OnServiceProtocolEstimateRasterCacheMemory, this,
                    std::placeholders::_1, std::placeholders::_2)};
  service_protocol_handlers_[ServiceProtocol::kReloadAssetFonts] = {
      task_runners_.GetPlatformTaskRunner(),
      std::bind(&Shell::OnServiceProtocolReloadAssetFonts, this,
                std::placeholders::_1, std::placeholders::_2)};
}

Shell::~Shell() {
#if !SLIMPELLER
  PersistentCache::GetCacheForProcess()->RemoveWorkerTaskRunner(
      task_runners_.GetIOTaskRunner());
#endif  //  !SLIMPELLER

  vm_->GetServiceProtocol()->RemoveHandler(this);

  fml::AutoResetWaitableEvent platiso_latch, ui_latch, gpu_latch,
      platform_latch, io_latch;

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetPlatformTaskRunner(),
      fml::MakeCopyable([this, &platiso_latch]() mutable {
        engine_->ShutdownPlatformIsolates();
        platiso_latch.Signal();
      }));
  platiso_latch.Wait();

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable([this, &ui_latch]() mutable {
        engine_.reset();
        ui_latch.Signal();
      }));
  ui_latch.Wait();

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetRasterTaskRunner(),
      fml::MakeCopyable(
          [this, rasterizer = std::move(rasterizer_), &gpu_latch]() mutable {
            rasterizer.reset();
            this->weak_factory_gpu_.reset();
            gpu_latch.Signal();
          }));
  gpu_latch.Wait();

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetIOTaskRunner(),
      fml::MakeCopyable([io_manager = std::move(io_manager_),
                         platform_view = platform_view_.get(),
                         &io_latch]() mutable {
        io_manager.reset();
        if (platform_view) {
          platform_view->ReleaseResourceContext();
        }
        io_latch.Signal();
      }));

  io_latch.Wait();

  // The platform view must go last because it may be holding onto platform side
  // counterparts to resources owned by subsystems running on other threads. For
  // example, the NSOpenGLContext on the Mac.
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetPlatformTaskRunner(),
      fml::MakeCopyable([platform_view = std::move(platform_view_),
                         &platform_latch]() mutable {
        platform_view.reset();
        platform_latch.Signal();
      }));
  platform_latch.Wait();
}

std::unique_ptr<Shell> Shell::Spawn(
    RunConfiguration run_configuration,
    const std::string& initial_route,
    const CreateCallback<PlatformView>& on_create_platform_view,
    const CreateCallback<Rasterizer>& on_create_rasterizer) const {
  FML_DCHECK(task_runners_.IsValid());
  // It's safe to store this value since it is set on the platform thread.
  bool is_gpu_disabled = false;
  GetIsGpuDisabledSyncSwitch()->Execute(
      fml::SyncSwitch::Handlers()
          .SetIfFalse([&is_gpu_disabled] { is_gpu_disabled = false; })
          .SetIfTrue([&is_gpu_disabled] { is_gpu_disabled = true; }));
  std::unique_ptr<Shell> result = CreateWithSnapshot(
      PlatformData{}, task_runners_, rasterizer_->GetRasterThreadMerger(),
      io_manager_, resource_cache_limit_calculator_, GetSettings(), vm_,
      vm_->GetVMData()->GetIsolateSnapshot(), on_create_platform_view,
      on_create_rasterizer,
      [engine = this->engine_.get(), initial_route](
          Engine::Delegate& delegate,
          const PointerDataDispatcherMaker& dispatcher_maker, DartVM& vm,
          const fml::RefPtr<const DartSnapshot>& isolate_snapshot,
          const TaskRunners& task_runners, const PlatformData& platform_data,
          const Settings& settings, std::unique_ptr<Animator> animator,
          const fml::WeakPtr<IOManager>& io_manager,
          const fml::RefPtr<SkiaUnrefQueue>& unref_queue,
          fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
          const std::shared_ptr<fml::SyncSwitch>& is_gpu_disabled_sync_switch,
          impeller::RuntimeStageBackend runtime_stage_backend) {
        return engine->Spawn(
            /*delegate=*/delegate,
            /*dispatcher_maker=*/dispatcher_maker,
            /*settings=*/settings,
            /*animator=*/std::move(animator),
            /*initial_route=*/initial_route,
            /*io_manager=*/io_manager,
            /*snapshot_delegate=*/std::move(snapshot_delegate),
            /*gpu_disabled_switch=*/is_gpu_disabled_sync_switch);
      },
      is_gpu_disabled);
  result->RunEngine(std::move(run_configuration));
  return result;
}

void Shell::NotifyLowMemoryWarning() const {
  auto trace_id = fml::tracing::TraceNonce();
  TRACE_EVENT_ASYNC_BEGIN0("flutter", "Shell::NotifyLowMemoryWarning",
                           trace_id);
  // This does not require a current isolate but does require a running VM.
  // Since a valid shell will not be returned to the embedder without a valid
  // DartVMRef, we can be certain that this is a safe spot to assume a VM is
  // running.
  ::Dart_NotifyLowMemory();

  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), trace_id = trace_id]() {
        if (rasterizer) {
          rasterizer->NotifyLowMemoryWarning();
        }
        TRACE_EVENT_ASYNC_END0("flutter", "Shell::NotifyLowMemoryWarning",
                               trace_id);
      });
  // The IO Manager uses resource cache limits of 0, so it is not necessary
  // to purge them.
}

void Shell::RunEngine(RunConfiguration run_configuration) {
  RunEngine(std::move(run_configuration), nullptr);
}

void Shell::RunEngine(
    RunConfiguration run_configuration,
    const std::function<void(Engine::RunStatus)>& result_callback) {
  auto result = [platform_runner = task_runners_.GetPlatformTaskRunner(),
                 result_callback](Engine::RunStatus run_result) {
    if (!result_callback) {
      return;
    }
    platform_runner->PostTask(
        [result_callback, run_result]() { result_callback(run_result); });
  };
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable(
          [run_configuration = std::move(run_configuration),
           weak_engine = weak_engine_, result]() mutable {
            if (!weak_engine) {
              FML_LOG(ERROR)
                  << "Could not launch engine with configuration - no engine.";
              result(Engine::RunStatus::Failure);
              return;
            }
            auto run_result = weak_engine->Run(std::move(run_configuration));
            if (run_result == flutter::Engine::RunStatus::Failure) {
              FML_LOG(ERROR) << "Could not launch engine with configuration.";
            }

            result(run_result);
          }));
}

std::optional<DartErrorCode> Shell::GetUIIsolateLastError() const {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (!weak_engine_) {
    return std::nullopt;
  }
  switch (weak_engine_->GetUIIsolateLastError()) {
    case tonic::kCompilationErrorType:
      return DartErrorCode::CompilationError;
    case tonic::kApiErrorType:
      return DartErrorCode::ApiError;
    case tonic::kUnknownErrorType:
      return DartErrorCode::UnknownError;
    case tonic::kNoError:
      return DartErrorCode::NoError;
  }
  return DartErrorCode::UnknownError;
}

bool Shell::EngineHasLivePorts() const {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (!weak_engine_) {
    return false;
  }

  return weak_engine_->UIIsolateHasLivePorts();
}

bool Shell::EngineHasPendingMicrotasks() const {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (!weak_engine_) {
    return false;
  }

  return weak_engine_->UIIsolateHasPendingMicrotasks();
}

bool Shell::IsSetup() const {
  return is_set_up_;
}

bool Shell::Setup(std::unique_ptr<PlatformView> platform_view,
                  std::unique_ptr<Engine> engine,
                  std::unique_ptr<Rasterizer> rasterizer,
                  const std::shared_ptr<ShellIOManager>& io_manager) {
  if (is_set_up_) {
    return false;
  }

  if (!platform_view || !engine || !rasterizer || !io_manager) {
    return false;
  }

  platform_view_ = std::move(platform_view);
  platform_message_handler_ = platform_view_->GetPlatformMessageHandler();
  route_messages_through_platform_thread_.store(true);
  task_runners_.GetPlatformTaskRunner()->PostTask(
      [self = weak_factory_.GetWeakPtr()] {
        if (self) {
          self->route_messages_through_platform_thread_.store(false);
        }
      });
  engine_ = std::move(engine);
  rasterizer_ = std::move(rasterizer);
  io_manager_ = io_manager;

  // Set the external view embedder for the rasterizer.
  auto view_embedder = platform_view_->CreateExternalViewEmbedder();
  rasterizer_->SetExternalViewEmbedder(view_embedder);
  rasterizer_->SetSnapshotSurfaceProducer(
      platform_view_->CreateSnapshotSurfaceProducer());

  // The weak ptr must be generated in the platform thread which owns the unique
  // ptr.
  weak_engine_ = engine_->GetWeakPtr();
  weak_rasterizer_ = rasterizer_->GetWeakPtr();
  weak_platform_view_ = platform_view_->GetWeakPtr();

  // Add the implicit view with empty metrics.
  engine_->AddView(kFlutterImplicitViewId, ViewportMetrics{}, [](bool added) {
    FML_DCHECK(added) << "Failed to add the implicit view";
  });

  // Setup the time-consuming default font manager right after engine created.
  if (!settings_.prefetched_default_font_manager) {
    fml::TaskRunner::RunNowOrPostTask(task_runners_.GetUITaskRunner(),
                                      [engine = weak_engine_] {
                                        if (engine) {
                                          engine->SetupDefaultFontManager();
                                        }
                                      });
  }

  is_set_up_ = true;

#if !SLIMPELLER
  PersistentCache::GetCacheForProcess()->AddWorkerTaskRunner(
      task_runners_.GetIOTaskRunner());

  PersistentCache::GetCacheForProcess()->SetIsDumpingSkp(
      settings_.dump_skp_on_shader_compilation);

  if (settings_.purge_persistent_cache) {
    PersistentCache::GetCacheForProcess()->Purge();
  }
#endif  //  !SLIMPELLER

  return true;
}

const Settings& Shell::GetSettings() const {
  return settings_;
}

const TaskRunners& Shell::GetTaskRunners() const {
  return task_runners_;
}

const fml::RefPtr<fml::RasterThreadMerger> Shell::GetParentRasterThreadMerger()
    const {
  return parent_raster_thread_merger_;
}

fml::TaskRunnerAffineWeakPtr<Rasterizer> Shell::GetRasterizer() const {
  FML_DCHECK(is_set_up_);
  return weak_rasterizer_;
}

fml::WeakPtr<Engine> Shell::GetEngine() {
  FML_DCHECK(is_set_up_);
  return weak_engine_;
}

fml::WeakPtr<PlatformView> Shell::GetPlatformView() {
  FML_DCHECK(is_set_up_);
  return weak_platform_view_;
}

fml::WeakPtr<ShellIOManager> Shell::GetIOManager() {
  FML_DCHECK(is_set_up_);
  return io_manager_->GetWeakPtr();
}

DartVM* Shell::GetDartVM() {
  return &vm_;
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewCreated(std::unique_ptr<Surface> surface) {
  TRACE_EVENT0("flutter", "Shell::OnPlatformViewCreated");
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  // Prevent any request to change the thread configuration for raster and
  // platform queues while the platform view is being created.
  //
  // This prevents false positives such as this method starts assuming that the
  // raster and platform queues have a given thread configuration, but then the
  // configuration is changed by a task, and the assumption is no longer true.
  //
  // This incorrect assumption can lead to deadlock.
  // See `should_post_raster_task` for more.
  rasterizer_->DisableThreadMergerIfNeeded();

  // The normal flow executed by this method is that the platform thread is
  // starting the sequence and waiting on the latch. Later the UI thread posts
  // raster_task to the raster thread which signals the latch. If the raster and
  // the platform threads are the same this results in a deadlock as the
  // raster_task will never be posted to the platform/raster thread that is
  // blocked on a latch. To avoid the described deadlock, if the raster and the
  // platform threads are the same, should_post_raster_task will be false, and
  // then instead of posting a task to the raster thread, the ui thread just
  // signals the latch and the platform/raster thread follows with executing
  // raster_task.
  const bool should_post_raster_task =
      !task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread();

  auto raster_task = fml::MakeCopyable(
      [&waiting_for_first_frame = waiting_for_first_frame_,  //
       rasterizer = rasterizer_->GetWeakPtr(),               //
       surface = std::move(surface)                          //
  ]() mutable {
        if (rasterizer) {
          // Enables the thread merger which may be used by the external view
          // embedder.
          rasterizer->EnableThreadMergerIfNeeded();
          rasterizer->Setup(std::move(surface));
        }

        waiting_for_first_frame.store(true);
      });

  auto ui_task = [engine = engine_->GetWeakPtr()] {
    if (engine) {
      engine->ScheduleFrame();
    }
  };

  // Threading: Capture platform view by raw pointer and not the weak pointer.
  // We are going to use the pointer on the IO thread which is not safe with a
  // weak pointer. However, we are preventing the platform view from being
  // collected by using a latch.
  auto* platform_view = platform_view_.get();
  FML_DCHECK(platform_view);
  fml::AutoResetWaitableEvent latch;

  auto io_task = [io_manager = io_manager_->GetWeakPtr(), platform_view,
                  ui_task_runner = task_runners_.GetUITaskRunner(), ui_task,
                  raster_task_runner = task_runners_.GetRasterTaskRunner(),
                  raster_task, should_post_raster_task, &latch] {
    if (io_manager && !io_manager->GetResourceContext()) {
      sk_sp<GrDirectContext> resource_context =
          platform_view->CreateResourceContext();
      io_manager->NotifyResourceContextAvailable(resource_context);
    }
    // Step 1: Post a task on the UI thread to tell the engine that it has
    // an output surface.
    fml::TaskRunner::RunNowOrPostTask(ui_task_runner, ui_task);

    // Step 2: Tell the raster thread that it should create a surface for
    // its rasterizer.
    if (should_post_raster_task) {
      fml::TaskRunner::RunNowOrPostTask(raster_task_runner, raster_task);
    }
    latch.Signal();
  };

  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetIOTaskRunner(), io_task);

  latch.Wait();
  if (!should_post_raster_task) {
    // See comment on should_post_raster_task, in this case the raster_task
    // wasn't executed, and we just run it here as the platform thread
    // is the raster thread.
    raster_task();
  }
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewDestroyed() {
  TRACE_EVENT0("flutter", "Shell::OnPlatformViewDestroyed");
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  // Prevent any request to change the thread configuration for raster and
  // platform queues while the platform view is being destroyed.
  //
  // This prevents false positives such as this method starts assuming that the
  // raster and platform queues have a given thread configuration, but then the
  // configuration is changed by a task, and the assumption is no longer true.
  //
  // This incorrect assumption can lead to deadlock.
  rasterizer_->DisableThreadMergerIfNeeded();

  // Notify the Dart VM that the PlatformView has been destroyed and some
  // cleanup activity can be done (e.g: garbage collect the Dart heap).
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetUITaskRunner(),
                                    [engine = engine_->GetWeakPtr()]() {
                                      if (engine) {
                                        engine->NotifyDestroyed();
                                      }
                                    });

  // Note:
  // This is a synchronous operation because certain platforms depend on
  // setup/suspension of all activities that may be interacting with the GPU in
  // a synchronous fashion.
  // The UI thread does not need to be serialized here - there is sufficient
  // guardrailing in the rasterizer to allow the UI thread to post work to it
  // even after the surface has been torn down.

  fml::AutoResetWaitableEvent latch;

  auto io_task = [io_manager = io_manager_.get(), &latch]() {
    // Execute any pending Skia object deletions while GPU access is still
    // allowed.
    io_manager->GetIsGpuDisabledSyncSwitch()->Execute(
        fml::SyncSwitch::Handlers().SetIfFalse(
            [&] { io_manager->GetSkiaUnrefQueue()->Drain(); }));
    // Step 4: All done. Signal the latch that the platform thread is waiting
    // on.
    latch.Signal();
  };

  auto raster_task = [rasterizer = rasterizer_->GetWeakPtr(),
                      io_task_runner = task_runners_.GetIOTaskRunner(),
                      io_task]() {
    if (rasterizer) {
      // Enables the thread merger which is required prior tearing down the
      // rasterizer. If the raster and platform threads are merged, tearing down
      // the rasterizer unmerges the threads.
      rasterizer->EnableThreadMergerIfNeeded();
      rasterizer->Teardown();
    }
    // Step 2: Tell the IO thread to complete its remaining work.
    fml::TaskRunner::RunNowOrPostTask(io_task_runner, io_task);
  };

  // Step 1: Post a task to the Raster thread (possibly this thread) to tell the
  // rasterizer the output surface is going away.
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetRasterTaskRunner(),
                                    raster_task);
  latch.Wait();
  // On Android, the external view embedder may post a task to the platform
  // thread, and wait until it completes if overlay surfaces must be released.
  // However, the platform thread might be blocked when Dart is initializing.
  // In this situation, calling TeardownExternalViewEmbedder is safe because no
  // platform views have been created before Flutter renders the first frame.
  // Overall, the longer term plan is to remove this implementation once
  // https://github.com/flutter/flutter/issues/96679 is fixed.
  rasterizer_->TeardownExternalViewEmbedder();
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewScheduleFrame() {
  TRACE_EVENT0("flutter", "Shell::OnPlatformViewScheduleFrame");
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetUITaskRunner(),
                                    [engine = engine_->GetWeakPtr()]() {
                                      if (engine) {
                                        engine->ScheduleFrame();
                                      }
                                    });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewSetViewportMetrics(int64_t view_id,
                                             const ViewportMetrics& metrics) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  if (metrics.device_pixel_ratio <= 0 || metrics.physical_width <= 0 ||
      metrics.physical_height <= 0) {
    // Ignore invalid view-port metrics.
    return;
  }

  // This is the formula Android uses.
  // https://android.googlesource.com/platform/frameworks/base/+/39ae5bac216757bc201490f4c7b8c0f63006c6cd/libs/hwui/renderthread/CacheManager.cpp#45
  resource_cache_limit_ =
      metrics.physical_width * metrics.physical_height * 12 * 4;
  size_t resource_cache_max_bytes =
      resource_cache_limit_calculator_->GetResourceCacheMaxBytes();
  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), resource_cache_max_bytes] {
        if (rasterizer) {
          rasterizer->SetResourceCacheMaxBytes(resource_cache_max_bytes, false);
        }
      });

  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(),
      [engine = engine_->GetWeakPtr(), view_id, metrics]() {
        if (engine) {
          engine->SetViewportMetrics(view_id, metrics);
        }
      });

  {
    std::scoped_lock<std::mutex> lock(resize_mutex_);
    expected_frame_sizes_[view_id] =
        SkISize::Make(metrics.physical_width, metrics.physical_height);
    device_pixel_ratio_ = metrics.device_pixel_ratio;
  }
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewDispatchPlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  FML_DCHECK(is_set_up_);
#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
  if (!task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread()) {
    std::scoped_lock lock(misbehaving_message_channels_mutex_);
    auto inserted = misbehaving_message_channels_.insert(message->channel());
    if (inserted.second) {
      FML_LOG(ERROR)
          << "The '" << message->channel()
          << "' channel sent a message from native to Flutter on a "
             "non-platform thread. Platform channel messages must be sent on "
             "the platform thread. Failure to do so may result in data loss or "
             "crashes, and must be fixed in the plugin or application code "
             "creating that channel.\n"
             "See https://docs.flutter.dev/platform-integration/"
             "platform-channels#channels-and-platform-threading for more "
             "information.";
    }
  }
#endif  // FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG

  // The static leak checker gets confused by the use of fml::MakeCopyable.
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable([engine = engine_->GetWeakPtr(),
                         message = std::move(message)]() mutable {
        if (engine) {
          engine->DispatchPlatformMessage(std::move(message));
        }
      }));
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewDispatchPointerDataPacket(
    std::unique_ptr<PointerDataPacket> packet) {
  TRACE_EVENT0_WITH_FLOW_IDS(
      "flutter", "Shell::OnPlatformViewDispatchPointerDataPacket",
      /*flow_id_count=*/1, /*flow_ids=*/&next_pointer_flow_id_);
  TRACE_FLOW_BEGIN("flutter", "PointerEvent", next_pointer_flow_id_);
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable([engine = weak_engine_, packet = std::move(packet),
                         flow_id = next_pointer_flow_id_]() mutable {
        if (engine) {
          engine->DispatchPointerDataPacket(std::move(packet), flow_id);
        }
      }));
  next_pointer_flow_id_++;
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewDispatchSemanticsAction(int32_t node_id,
                                                  SemanticsAction action,
                                                  fml::MallocMapping args) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable([engine = engine_->GetWeakPtr(), node_id, action,
                         args = std::move(args)]() mutable {
        if (engine) {
          engine->DispatchSemanticsAction(node_id, action, std::move(args));
        }
      }));
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewSetSemanticsEnabled(bool enabled) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(),
      [engine = engine_->GetWeakPtr(), enabled] {
        if (engine) {
          engine->SetSemanticsEnabled(enabled);
        }
      });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewSetAccessibilityFeatures(int32_t flags) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  fml::TaskRunner::RunNowAndFlushMessages(
      task_runners_.GetUITaskRunner(), [engine = engine_->GetWeakPtr(), flags] {
        if (engine) {
          engine->SetAccessibilityFeatures(flags);
        }
      });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewRegisterTexture(
    std::shared_ptr<flutter::Texture> texture) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), texture] {
        if (rasterizer) {
          if (auto registry = rasterizer->GetTextureRegistry()) {
            registry->RegisterTexture(texture);
          }
        }
      });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewUnregisterTexture(int64_t texture_id) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), texture_id]() {
        if (rasterizer) {
          if (auto registry = rasterizer->GetTextureRegistry()) {
            registry->UnregisterTexture(texture_id);
          }
        }
      });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewMarkTextureFrameAvailable(int64_t texture_id) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  // Tell the rasterizer that one of its textures has a new frame available.
  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), texture_id]() {
        if (!rasterizer) {
          return;
        }
        auto registry = rasterizer->GetTextureRegistry();

        if (!registry) {
          return;
        }

        auto texture = registry->GetTexture(texture_id);

        if (!texture) {
          return;
        }

        texture->MarkNewFrameAvailable();
      });

  // Schedule a new frame without having to rebuild the layer tree.
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetUITaskRunner(),
                                    [engine = engine_->GetWeakPtr()]() {
                                      if (engine) {
                                        engine->ScheduleFrame(false);
                                      }
                                    });
}

// |PlatformView::Delegate|
void Shell::OnPlatformViewSetNextFrameCallback(const fml::closure& closure) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), closure = closure]() {
        if (rasterizer) {
          rasterizer->SetNextFrameCallback(closure);
        }
      });
}

// |PlatformView::Delegate|
const Settings& Shell::OnPlatformViewGetSettings() const {
  return settings_;
}

// |Animator::Delegate|
void Shell::OnAnimatorBeginFrame(fml::TimePoint frame_target_time,
                                 uint64_t frame_number) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  // record the target time for use by rasterizer.
  {
    std::scoped_lock time_recorder_lock(time_recorder_mutex_);
    latest_frame_target_time_.emplace(frame_target_time);
  }
  if (engine_) {
    engine_->BeginFrame(frame_target_time, frame_number);
  }
}

// |Animator::Delegate|
void Shell::OnAnimatorNotifyIdle(fml::TimeDelta deadline) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (engine_) {
    engine_->NotifyIdle(deadline);
  }
}

void Shell::OnAnimatorUpdateLatestFrameTargetTime(
    fml::TimePoint frame_target_time) {
  FML_DCHECK(is_set_up_);

  // record the target time for use by rasterizer.
  {
    std::scoped_lock time_recorder_lock(time_recorder_mutex_);
    if (!latest_frame_target_time_) {
      latest_frame_target_time_ = frame_target_time;
    } else if (latest_frame_target_time_ < frame_target_time) {
      latest_frame_target_time_ = frame_target_time;
    }
  }
}

// |Animator::Delegate|
void Shell::OnAnimatorDraw(std::shared_ptr<FramePipeline> pipeline) {
  FML_DCHECK(is_set_up_);

  task_runners_.GetRasterTaskRunner()->PostTask(fml::MakeCopyable(
      [&waiting_for_first_frame = waiting_for_first_frame_,
       &waiting_for_first_frame_condition = waiting_for_first_frame_condition_,
       rasterizer = rasterizer_->GetWeakPtr(),
       weak_pipeline = std::weak_ptr<FramePipeline>(pipeline)]() mutable {
        if (rasterizer) {
          std::shared_ptr<FramePipeline> pipeline = weak_pipeline.lock();
          if (pipeline) {
            rasterizer->Draw(pipeline);
          }

          if (waiting_for_first_frame.load()) {
            waiting_for_first_frame.store(false);
            waiting_for_first_frame_condition.notify_all();
          }
        }
      }));
}

// |Animator::Delegate|
void Shell::OnAnimatorDrawLastLayerTrees(
    std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) {
  FML_DCHECK(is_set_up_);

  auto task = fml::MakeCopyable(
      [rasterizer = rasterizer_->GetWeakPtr(),
       frame_timings_recorder = std::move(frame_timings_recorder)]() mutable {
        if (rasterizer) {
          rasterizer->DrawLastLayerTrees(std::move(frame_timings_recorder));
        }
      });

  task_runners_.GetRasterTaskRunner()->PostTask(task);
}

// |Engine::Delegate|
void Shell::OnEngineUpdateSemantics(SemanticsNodeUpdates update,
                                    CustomAccessibilityActionUpdates actions) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  task_runners_.GetPlatformTaskRunner()->RunNowOrPostTask(
      task_runners_.GetPlatformTaskRunner(),
      [view = platform_view_->GetWeakPtr(), update = std::move(update),
       actions = std::move(actions)] {
        if (view) {
          view->UpdateSemantics(update, actions);
        }
      });
}

// |Engine::Delegate|
void Shell::OnEngineHandlePlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (message->channel() == kSkiaChannel) {
    HandleEngineSkiaMessage(std::move(message));
    return;
  }

  if (platform_message_handler_) {
    if (route_messages_through_platform_thread_ &&
        !platform_message_handler_
             ->DoesHandlePlatformMessageOnPlatformThread()) {
#if _WIN32
      // On Windows capturing a TaskRunner with a TaskRunner will cause an
      // uncaught exception in process shutdown because of the deletion order of
      // global variables. See also
      // https://github.com/flutter/flutter/issues/111575.
      // This won't be an issue until Windows supports background platform
      // channels (https://github.com/flutter/flutter/issues/93945). Then this
      // can potentially be addressed by capturing a weak_ptr to an object that
      // retains the ui TaskRunner, instead of the TaskRunner directly.
      FML_DCHECK(false);
#endif
      // We route messages through the platform thread temporarily when the
      // shell is being initialized to be backwards compatible with setting
      // message handlers in the same event as starting the isolate, but after
      // it is started.
      auto ui_task_runner = task_runners_.GetUITaskRunner();
      task_runners_.GetPlatformTaskRunner()->PostTask(fml::MakeCopyable(
          [weak_platform_message_handler =
               std::weak_ptr<PlatformMessageHandler>(platform_message_handler_),
           message = std::move(message), ui_task_runner]() mutable {
            ui_task_runner->PostTask(
                fml::MakeCopyable([weak_platform_message_handler,
                                   message = std::move(message)]() mutable {
                  auto platform_message_handler =
                      weak_platform_message_handler.lock();
                  if (platform_message_handler) {
                    platform_message_handler->HandlePlatformMessage(
                        std::move(message));
                  }
                }));
          }));
    } else {
      platform_message_handler_->HandlePlatformMessage(std::move(message));
    }
  } else {
    task_runners_.GetPlatformTaskRunner()->PostTask(
        fml::MakeCopyable([view = platform_view_->GetWeakPtr(),
                           message = std::move(message)]() mutable {
          if (view) {
            view->HandlePlatformMessage(std::move(message));
          }
        }));
  }
}

void Shell::OnEngineChannelUpdate(std::string name, bool listening) {
  FML_DCHECK(is_set_up_);

  task_runners_.GetPlatformTaskRunner()->PostTask(
      [view = platform_view_->GetWeakPtr(), name = std::move(name), listening] {
        if (view) {
          view->SendChannelUpdate(name, listening);
        }
      });
}

void Shell::HandleEngineSkiaMessage(std::unique_ptr<PlatformMessage> message) {
  const auto& data = message->data();

  rapidjson::Document document;
  document.Parse(reinterpret_cast<const char*>(data.GetMapping()),
                 data.GetSize());
  if (document.HasParseError() || !document.IsObject()) {
    return;
  }
  auto root = document.GetObject();
  auto method = root.FindMember("method");
  if (method->value != "Skia.setResourceCacheMaxBytes") {
    return;
  }
  auto args = root.FindMember("args");
  if (args == root.MemberEnd() || !args->value.IsInt()) {
    return;
  }

  task_runners_.GetRasterTaskRunner()->PostTask(
      [rasterizer = rasterizer_->GetWeakPtr(), max_bytes = args->value.GetInt(),
       response = message->response()] {
        if (rasterizer) {
          rasterizer->SetResourceCacheMaxBytes(static_cast<size_t>(max_bytes),
                                               true);
        }
        if (response) {
          // The framework side expects this to be valid json encoded as a list.
          // Return `[true]` to signal success.
          std::vector<uint8_t> data = {'[', 't', 'r', 'u', 'e', ']'};
          response->Complete(
              std::make_unique<fml::DataMapping>(std::move(data)));
        }
      });
}

// |Engine::Delegate|
void Shell::OnPreEngineRestart() {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetPlatformTaskRunner(),
      [view = platform_view_->GetWeakPtr(), &latch]() {
        if (view) {
          view->OnPreEngineRestart();
        }
        latch.Signal();
      });
  // This is blocking as any embedded platform views has to be flushed before
  // we re-run the Dart code.
  latch.Wait();
}

// |Engine::Delegate|
void Shell::OnRootIsolateCreated() {
  if (is_added_to_service_protocol_) {
    return;
  }
  auto description = GetServiceProtocolDescription();
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetPlatformTaskRunner(),
      [self = weak_factory_.GetWeakPtr(),
       description = std::move(description)]() {
        if (self) {
          self->vm_->GetServiceProtocol()->AddHandler(self.get(), description);
        }
      });
  is_added_to_service_protocol_ = true;
}

// |Engine::Delegate|
void Shell::UpdateIsolateDescription(const std::string isolate_name,
                                     int64_t isolate_port) {
  Handler::Description description(isolate_port, isolate_name);
  vm_->GetServiceProtocol()->SetHandlerDescription(this, description);
}

void Shell::SetNeedsReportTimings(bool value) {
  needs_report_timings_ = value;
}

// |Engine::Delegate|
std::unique_ptr<std::vector<std::string>> Shell::ComputePlatformResolvedLocale(
    const std::vector<std::string>& supported_locale_data) {
  return platform_view_->ComputePlatformResolvedLocales(supported_locale_data);
}

void Shell::LoadDartDeferredLibrary(
    intptr_t loading_unit_id,
    std::unique_ptr<const fml::Mapping> snapshot_data,
    std::unique_ptr<const fml::Mapping> snapshot_instructions) {
  task_runners_.GetUITaskRunner()->PostTask(fml::MakeCopyable(
      [engine = engine_->GetWeakPtr(), loading_unit_id,
       data = std::move(snapshot_data),
       instructions = std::move(snapshot_instructions)]() mutable {
        if (engine) {
          engine->LoadDartDeferredLibrary(loading_unit_id, std::move(data),
                                          std::move(instructions));
        }
      }));
}

void Shell::LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                         const std::string error_message,
                                         bool transient) {
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      [engine = weak_engine_, loading_unit_id, error_message, transient] {
        if (engine) {
          engine->LoadDartDeferredLibraryError(loading_unit_id, error_message,
                                               transient);
        }
      });
}

void Shell::UpdateAssetResolverByType(
    std::unique_ptr<AssetResolver> updated_asset_resolver,
    AssetResolver::AssetResolverType type) {
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable(
          [engine = weak_engine_, type,
           asset_resolver = std::move(updated_asset_resolver)]() mutable {
            if (engine) {
              engine->GetAssetManager()->UpdateResolverByType(
                  std::move(asset_resolver), type);
            }
          }));
}

// |Engine::Delegate|
void Shell::RequestDartDeferredLibrary(intptr_t loading_unit_id) {
  task_runners_.GetPlatformTaskRunner()->PostTask(
      [view = platform_view_->GetWeakPtr(), loading_unit_id] {
        if (view) {
          view->RequestDartDeferredLibrary(loading_unit_id);
        }
      });
}

// |Engine::Delegate|
double Shell::GetScaledFontSize(double unscaled_font_size,
                                int configuration_id) const {
  return platform_view_->GetScaledFontSize(unscaled_font_size,
                                           configuration_id);
}

void Shell::ReportTimings() {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());

  auto timings = std::move(unreported_timings_);
  unreported_timings_ = {};
  task_runners_.GetUITaskRunner()->PostTask([timings, engine = weak_engine_] {
    if (engine) {
      engine->ReportTimings(timings);
    }
  });
}

size_t Shell::UnreportedFramesCount() const {
  // Check that this is running on the raster thread to avoid race conditions.
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());
  FML_DCHECK(unreported_timings_.size() % (FrameTiming::kStatisticsCount) == 0);
  return unreported_timings_.size() / (FrameTiming::kStatisticsCount);
}

void Shell::OnFrameRasterized(const FrameTiming& timing) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());

  // The C++ callback defined in settings.h and set by Flutter runner. This is
  // independent of the timings report to the Dart side.
  if (settings_.frame_rasterized_callback) {
    settings_.frame_rasterized_callback(timing);
  }

  if (!needs_report_timings_) {
    return;
  }

  size_t old_count = unreported_timings_.size();
  (void)old_count;
  for (auto phase : FrameTiming::kPhases) {
    unreported_timings_.push_back(
        timing.Get(phase).ToEpochDelta().ToMicroseconds());
  }
  unreported_timings_.push_back(timing.GetLayerCacheCount());
  unreported_timings_.push_back(timing.GetLayerCacheBytes());
  unreported_timings_.push_back(timing.GetPictureCacheCount());
  unreported_timings_.push_back(timing.GetPictureCacheBytes());
  unreported_timings_.push_back(timing.GetFrameNumber());
  FML_DCHECK(unreported_timings_.size() ==
             old_count + FrameTiming::kStatisticsCount);

  // In tests using iPhone 6S with profile mode, sending a batch of 1 frame or a
  // batch of 100 frames have roughly the same cost of less than 0.1ms. Sending
  // a batch of 500 frames costs about 0.2ms. The 1 second threshold usually
  // kicks in before we reaching the following 100 frames threshold. The 100
  // threshold here is mainly for unit tests (so we don't have to write a
  // 1-second unit test), and make sure that our vector won't grow too big with
  // future 120fps, 240fps, or 1000fps displays.
  //
  // In the profile/debug mode, the timings are used by development tools which
  // require a latency of no more than 100ms. Hence we lower that 1-second
  // threshold to 100ms because performance overhead isn't that critical in
  // those cases.
  if (!first_frame_rasterized_ || UnreportedFramesCount() >= 100) {
    first_frame_rasterized_ = true;
    ReportTimings();
  } else if (!frame_timings_report_scheduled_) {
#if FLUTTER_RELEASE
    constexpr int kBatchTimeInMilliseconds = 1000;
#else
    constexpr int kBatchTimeInMilliseconds = 100;
#endif

    // Also make sure that frame times get reported with a max latency of 1
    // second. Otherwise, the timings of last few frames of an animation may
    // never be reported until the next animation starts.
    frame_timings_report_scheduled_ = true;
    task_runners_.GetRasterTaskRunner()->PostDelayedTask(
        [self = weak_factory_gpu_->GetWeakPtr()]() {
          if (!self) {
            return;
          }
          self->frame_timings_report_scheduled_ = false;
          if (self->UnreportedFramesCount() > 0) {
            self->ReportTimings();
          }
        },
        fml::TimeDelta::FromMilliseconds(kBatchTimeInMilliseconds));
  }
}

fml::Milliseconds Shell::GetFrameBudget() {
  if (cached_display_refresh_rate_.has_value()) {
    return cached_display_refresh_rate_.value();
  }
  double display_refresh_rate = display_manager_->GetMainDisplayRefreshRate();
  if (display_refresh_rate > 0) {
    cached_display_refresh_rate_ =
        fml::RefreshRateToFrameBudget(display_refresh_rate);
  } else {
    cached_display_refresh_rate_ = fml::kDefaultFrameBudget;
  }
  return cached_display_refresh_rate_.value_or(fml::kDefaultFrameBudget);
}

fml::TimePoint Shell::GetLatestFrameTargetTime() const {
  std::scoped_lock time_recorder_lock(time_recorder_mutex_);
  FML_CHECK(latest_frame_target_time_.has_value())
      << "GetLatestFrameTargetTime called before OnAnimatorBeginFrame";
  // Covered by FML_CHECK().
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  return latest_frame_target_time_.value();
}

// |Rasterizer::Delegate|
bool Shell::ShouldDiscardLayerTree(int64_t view_id,
                                   const flutter::LayerTree& tree) {
  std::scoped_lock<std::mutex> lock(resize_mutex_);
  auto expected_frame_size = ExpectedFrameSize(view_id);
  return !expected_frame_size.isEmpty() &&
         ToSkISize(tree.frame_size()) != expected_frame_size;
}

// |ServiceProtocol::Handler|
fml::RefPtr<fml::TaskRunner> Shell::GetServiceProtocolHandlerTaskRunner(
    std::string_view method) const {
  FML_DCHECK(is_set_up_);
  auto found = service_protocol_handlers_.find(method);
  if (found != service_protocol_handlers_.end()) {
    return found->second.first;
  }
  return task_runners_.GetUITaskRunner();
}

// |ServiceProtocol::Handler|
bool Shell::HandleServiceProtocolMessage(
    std::string_view method,  // one if the extension names specified above.
    const ServiceProtocolMap& params,
    rapidjson::Document* response) {
  auto found = service_protocol_handlers_.find(method);
  if (found != service_protocol_handlers_.end()) {
    return found->second.second(params, response);
  }
  return false;
}

// |ServiceProtocol::Handler|
ServiceProtocol::Handler::Description Shell::GetServiceProtocolDescription()
    const {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (!weak_engine_) {
    return ServiceProtocol::Handler::Description();
  }

  return {
      weak_engine_->GetUIIsolateMainPort(),
      weak_engine_->GetUIIsolateName(),
  };
}

static void ServiceProtocolParameterError(rapidjson::Document* response,
                                          std::string error_details) {
  auto& allocator = response->GetAllocator();
  response->SetObject();
  const int64_t kInvalidParams = -32602;
  response->AddMember("code", kInvalidParams, allocator);
  response->AddMember("message", "Invalid params", allocator);
  {
    rapidjson::Value details(rapidjson::kObjectType);
    details.AddMember("details", std::move(error_details), allocator);
    response->AddMember("data", details, allocator);
  }
}

static void ServiceProtocolFailureError(rapidjson::Document* response,
                                        std::string message) {
  auto& allocator = response->GetAllocator();
  response->SetObject();
  const int64_t kJsonServerError = -32000;
  response->AddMember("code", kJsonServerError, allocator);
  response->AddMember("message", std::move(message), allocator);
}

// Service protocol handler
bool Shell::OnServiceProtocolScreenshot(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());
  auto screenshot = rasterizer_->ScreenshotLastLayerTree(
      Rasterizer::ScreenshotType::CompressedImage, true);
  if (screenshot.data) {
    response->SetObject();
    auto& allocator = response->GetAllocator();
    response->AddMember("type", "Screenshot", allocator);
    rapidjson::Value image;
    image.SetString(static_cast<const char*>(screenshot.data->data()),
                    screenshot.data->size(), allocator);
    response->AddMember("screenshot", image, allocator);
    return true;
  }
  ServiceProtocolFailureError(response, "Could not capture image screenshot.");
  return false;
}

// Service protocol handler
bool Shell::OnServiceProtocolScreenshotSKP(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());
  if (settings_.enable_impeller) {
    ServiceProtocolFailureError(
        response, "Cannot capture SKP screenshot with Impeller enabled.");
    return false;
  }
  auto screenshot = rasterizer_->ScreenshotLastLayerTree(
      Rasterizer::ScreenshotType::SkiaPicture, true);
  if (screenshot.data) {
    response->SetObject();
    auto& allocator = response->GetAllocator();
    response->AddMember("type", "ScreenshotSkp", allocator);
    rapidjson::Value skp;
    skp.SetString(static_cast<const char*>(screenshot.data->data()),
                  screenshot.data->size(), allocator);
    response->AddMember("skp", skp, allocator);
    return true;
  }
  ServiceProtocolFailureError(response, "Could not capture SKP screenshot.");
  return false;
}

// Service protocol handler
bool Shell::OnServiceProtocolRunInView(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (params.count("mainScript") == 0) {
    ServiceProtocolParameterError(response,
                                  "'mainScript' parameter is missing.");
    return false;
  }

  if (params.count("assetDirectory") == 0) {
    ServiceProtocolParameterError(response,
                                  "'assetDirectory' parameter is missing.");
    return false;
  }

  std::string main_script_path =
      fml::paths::FromURI(params.at("mainScript").data());
  std::string asset_directory_path =
      fml::paths::FromURI(params.at("assetDirectory").data());

  auto main_script_file_mapping =
      std::make_unique<fml::FileMapping>(fml::OpenFile(
          main_script_path.c_str(), false, fml::FilePermission::kRead));

  auto isolate_configuration = IsolateConfiguration::CreateForKernel(
      std::move(main_script_file_mapping));

  RunConfiguration configuration(std::move(isolate_configuration));

  configuration.SetEntrypointAndLibrary(engine_->GetLastEntrypoint(),
                                        engine_->GetLastEntrypointLibrary());
  configuration.SetEntrypointArgs(engine_->GetLastEntrypointArgs());

  configuration.AddAssetResolver(std::make_unique<DirectoryAssetBundle>(
      fml::OpenDirectory(asset_directory_path.c_str(), false,
                         fml::FilePermission::kRead),
      false));

  // Preserve any original asset resolvers to avoid syncing unchanged assets
  // over the DevFS connection.
  auto old_asset_manager = engine_->GetAssetManager();
  if (old_asset_manager != nullptr) {
    for (auto& old_resolver : old_asset_manager->TakeResolvers()) {
      if (old_resolver->IsValidAfterAssetManagerChange()) {
        configuration.AddAssetResolver(std::move(old_resolver));
      }
    }
  }

  auto& allocator = response->GetAllocator();
  response->SetObject();
  if (engine_->Restart(std::move(configuration))) {
    response->AddMember("type", "Success", allocator);
    auto new_description = GetServiceProtocolDescription();
    rapidjson::Value view(rapidjson::kObjectType);
    new_description.Write(this, view, allocator);
    response->AddMember("view", view, allocator);
    return true;
  } else {
    FML_DLOG(ERROR) << "Could not run configuration in engine.";
    ServiceProtocolFailureError(response,
                                "Could not run configuration in engine.");
    return false;
  }

  FML_DCHECK(false);
  return false;
}

// Service protocol handler
bool Shell::OnServiceProtocolFlushUIThreadTasks(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());
  // This API should not be invoked by production code.
  // It can potentially starve the service isolate if the main isolate pauses
  // at a breakpoint or is in an infinite loop.
  //
  // It should be invoked from the VM Service and blocks it until UI thread
  // tasks are processed.
  response->SetObject();
  response->AddMember("type", "Success", response->GetAllocator());
  return true;
}

bool Shell::OnServiceProtocolGetDisplayRefreshRate(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());
  response->SetObject();
  response->AddMember("type", "DisplayRefreshRate", response->GetAllocator());
  response->AddMember("fps", display_manager_->GetMainDisplayRefreshRate(),
                      response->GetAllocator());
  return true;
}

double Shell::GetMainDisplayRefreshRate() {
  return display_manager_->GetMainDisplayRefreshRate();
}

void Shell::RegisterImageDecoder(ImageGeneratorFactory factory,
                                 int32_t priority) {
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  FML_DCHECK(is_set_up_);

  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      [engine = engine_->GetWeakPtr(), factory = std::move(factory),
       priority]() {
        if (engine) {
          engine->GetImageGeneratorRegistry()->AddFactory(factory, priority);
        }
      });
}

bool Shell::OnServiceProtocolGetSkSLs(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetIOTaskRunner()->RunsTasksOnCurrentThread());
  response->SetObject();
  response->AddMember("type", "GetSkSLs", response->GetAllocator());

  rapidjson::Value shaders_json(rapidjson::kObjectType);
#if !SLIMPELLER
  PersistentCache* persistent_cache = PersistentCache::GetCacheForProcess();
  std::vector<PersistentCache::SkSLCache> sksls = persistent_cache->LoadSkSLs();
  for (const auto& sksl : sksls) {
    size_t b64_size = Base64::EncodedSize(sksl.value->size());
    sk_sp<SkData> b64_data = SkData::MakeUninitialized(b64_size + 1);
    char* b64_char = static_cast<char*>(b64_data->writable_data());
    Base64::Encode(sksl.value->data(), sksl.value->size(), b64_char);
    b64_char[b64_size] = 0;  // make it null terminated for printing
    rapidjson::Value shader_value(b64_char, response->GetAllocator());
    std::string_view key_view(reinterpret_cast<const char*>(sksl.key->data()),
                              sksl.key->size());
    auto encode_result = fml::Base32Encode(key_view);
    if (!encode_result.first) {
      continue;
    }
    rapidjson::Value shader_key(encode_result.second, response->GetAllocator());
    shaders_json.AddMember(shader_key, shader_value, response->GetAllocator());
  }
#endif  //  !SLIMPELLER
  response->AddMember("SkSLs", shaders_json, response->GetAllocator());
  return true;
}

bool Shell::OnServiceProtocolEstimateRasterCacheMemory(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());

  uint64_t layer_cache_byte_size = 0u;
  uint64_t picture_cache_byte_size = 0u;

#if !SLIMPELLER
  const auto& raster_cache = rasterizer_->compositor_context()->raster_cache();
  layer_cache_byte_size = raster_cache.EstimateLayerCacheByteSize();
  picture_cache_byte_size = raster_cache.EstimatePictureCacheByteSize();
#endif  //  !SLIMPELLER

  response->SetObject();
  response->AddMember("type", "EstimateRasterCacheMemory",
                      response->GetAllocator());
  response->AddMember<uint64_t>("layerBytes", layer_cache_byte_size,
                                response->GetAllocator());
  response->AddMember<uint64_t>("pictureBytes", picture_cache_byte_size,
                                response->GetAllocator());
  return true;
}

// Service protocol handler
bool Shell::OnServiceProtocolSetAssetBundlePath(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (params.count("assetDirectory") == 0) {
    ServiceProtocolParameterError(response,
                                  "'assetDirectory' parameter is missing.");
    return false;
  }

  auto& allocator = response->GetAllocator();
  response->SetObject();

  auto asset_manager = std::make_shared<AssetManager>();

  if (!asset_manager->PushFront(std::make_unique<DirectoryAssetBundle>(
          fml::OpenDirectory(params.at("assetDirectory").data(), false,
                             fml::FilePermission::kRead),
          false))) {
    // The new asset directory path was invalid.
    FML_DLOG(ERROR) << "Could not update asset directory.";
    ServiceProtocolFailureError(response, "Could not update asset directory.");
    return false;
  }

  // Preserve any original asset resolvers to avoid syncing unchanged assets
  // over the DevFS connection.
  auto old_asset_manager = engine_->GetAssetManager();
  if (old_asset_manager != nullptr) {
    for (auto& old_resolver : old_asset_manager->TakeResolvers()) {
      if (old_resolver->IsValidAfterAssetManagerChange()) {
        asset_manager->PushBack(std::move(old_resolver));
      }
    }
  }

  if (engine_->UpdateAssetManager(asset_manager)) {
    response->AddMember("type", "Success", allocator);
    auto new_description = GetServiceProtocolDescription();
    rapidjson::Value view(rapidjson::kObjectType);
    new_description.Write(this, view, allocator);
    response->AddMember("view", view, allocator);
    return true;
  } else {
    FML_DLOG(ERROR) << "Could not update asset directory.";
    ServiceProtocolFailureError(response, "Could not update asset directory.");
    return false;
  }

  FML_DCHECK(false);
  return false;
}

void Shell::SendFontChangeNotification() {
  // After system fonts are reloaded, we send a system channel message
  // to notify flutter framework.
  rapidjson::Document document;
  document.SetObject();
  auto& allocator = document.GetAllocator();
  rapidjson::Value message_value;
  message_value.SetString(kFontChange, allocator);
  document.AddMember(kTypeKey, message_value, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::string message = buffer.GetString();
  std::unique_ptr<PlatformMessage> fontsChangeMessage =
      std::make_unique<flutter::PlatformMessage>(
          kSystemChannel,
          fml::MallocMapping::Copy(message.c_str(), message.length()), nullptr);
  OnPlatformViewDispatchPlatformMessage(std::move(fontsChangeMessage));
}

bool Shell::OnServiceProtocolReloadAssetFonts(
    const ServiceProtocol::Handler::ServiceProtocolMap& params,
    rapidjson::Document* response) {
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  if (!engine_) {
    return false;
  }
  engine_->GetFontCollection().RegisterFonts(engine_->GetAssetManager());
  engine_->GetFontCollection().GetFontCollection()->ClearFontFamilyCache();
  SendFontChangeNotification();

  auto& allocator = response->GetAllocator();
  response->SetObject();
  response->AddMember("type", "Success", allocator);

  return true;
}

void Shell::OnPlatformViewAddView(int64_t view_id,
                                  const ViewportMetrics& viewport_metrics,
                                  AddViewCallback callback) {
  TRACE_EVENT0("flutter", "Shell::AddView");
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  FML_DCHECK(view_id != kFlutterImplicitViewId)
      << "Unexpected request to add the implicit view #"
      << kFlutterImplicitViewId << ". This view should never be added.";

  task_runners_.GetUITaskRunner()->RunNowOrPostTask(
      task_runners_.GetUITaskRunner(), [engine = engine_->GetWeakPtr(),  //
                                        viewport_metrics,                //
                                        view_id,                         //
                                        callback = std::move(callback)   //
  ] {
        if (engine) {
          engine->AddView(view_id, viewport_metrics, callback);
        }
      });
}

void Shell::OnPlatformViewRemoveView(int64_t view_id,
                                     RemoveViewCallback callback) {
  TRACE_EVENT0("flutter", "Shell::RemoveView");
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  FML_DCHECK(view_id != kFlutterImplicitViewId)
      << "Unexpected request to remove the implicit view #"
      << kFlutterImplicitViewId << ". This view should never be removed.";

  expected_frame_sizes_.erase(view_id);
  task_runners_.GetUITaskRunner()->RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      [&task_runners = task_runners_,           //
       engine = engine_->GetWeakPtr(),          //
       rasterizer = rasterizer_->GetWeakPtr(),  //
       view_id,                                 //
       callback = std::move(callback)           //
  ] {
        if (engine) {
          bool removed = engine->RemoveView(view_id);
          callback(removed);
        }
        // Don't wait for the raster task here, which only cleans up memory and
        // does not affect functionality. Make sure it is done after Dart
        // removes the view to avoid receiving another rasterization request
        // that adds back the view record.
        task_runners.GetRasterTaskRunner()->PostTask([rasterizer, view_id]() {
          if (rasterizer) {
            rasterizer->CollectView(view_id);
          }
        });
      });
}

Rasterizer::Screenshot Shell::Screenshot(
    Rasterizer::ScreenshotType screenshot_type,
    bool base64_encode) {
  if (settings_.enable_impeller) {
    switch (screenshot_type) {
      case Rasterizer::ScreenshotType::SkiaPicture:
        FML_LOG(ERROR)
            << "Impeller backend cannot produce ScreenshotType::SkiaPicture.";
        return {};
      case Rasterizer::ScreenshotType::UncompressedImage:
      case Rasterizer::ScreenshotType::CompressedImage:
      case Rasterizer::ScreenshotType::SurfaceData:
        break;
    }
  }
  TRACE_EVENT0("flutter", "Shell::Screenshot");
  fml::AutoResetWaitableEvent latch;
  Rasterizer::Screenshot screenshot;
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetRasterTaskRunner(), [&latch,                        //
                                            rasterizer = GetRasterizer(),  //
                                            &screenshot,                   //
                                            screenshot_type,               //
                                            base64_encode                  //
  ]() {
        if (rasterizer) {
          screenshot = rasterizer->ScreenshotLastLayerTree(screenshot_type,
                                                           base64_encode);
        }
        latch.Signal();
      });
  latch.Wait();
  return screenshot;
}

fml::Status Shell::WaitForFirstFrame(fml::TimeDelta timeout) {
  FML_DCHECK(is_set_up_);
  if (task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread() ||
      task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread()) {
    return fml::Status(fml::StatusCode::kFailedPrecondition,
                       "WaitForFirstFrame called from thread that can't wait "
                       "because it is responsible for generating the frame.");
  }

  // Check for overflow.
  auto now = std::chrono::steady_clock::now();
  auto max_duration = std::chrono::steady_clock::time_point::max() - now;
  auto desired_duration = std::chrono::milliseconds(timeout.ToMilliseconds());
  auto duration =
      now + (desired_duration > max_duration ? max_duration : desired_duration);

  std::unique_lock<std::mutex> lock(waiting_for_first_frame_mutex_);
  bool success = waiting_for_first_frame_condition_.wait_until(
      lock, duration, [&waiting_for_first_frame = waiting_for_first_frame_] {
        return !waiting_for_first_frame.load();
      });
  if (success) {
    return fml::Status();
  } else {
    return fml::Status(fml::StatusCode::kDeadlineExceeded, "timeout");
  }
}

bool Shell::ReloadSystemFonts() {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  if (!engine_) {
    return false;
  }
  engine_->SetupDefaultFontManager();
  engine_->GetFontCollection().GetFontCollection()->ClearFontFamilyCache();
  // After system fonts are reloaded, we send a system channel message
  // to notify flutter framework.
  SendFontChangeNotification();
  return true;
}

std::shared_ptr<const fml::SyncSwitch> Shell::GetIsGpuDisabledSyncSwitch()
    const {
  return is_gpu_disabled_sync_switch_;
}

void Shell::SetGpuAvailability(GpuAvailability availability) {
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  switch (availability) {
    case GpuAvailability::kAvailable:
      is_gpu_disabled_sync_switch_->SetSwitch(false);
      return;
    case GpuAvailability::kFlushAndMakeUnavailable: {
      fml::AutoResetWaitableEvent latch;
      fml::TaskRunner::RunNowOrPostTask(
          task_runners_.GetIOTaskRunner(),
          [io_manager = io_manager_.get(), &latch]() {
            io_manager->GetSkiaUnrefQueue()->Drain();
            latch.Signal();
          });
      latch.Wait();
    }
      // FALLTHROUGH
    case GpuAvailability::kUnavailable:
      is_gpu_disabled_sync_switch_->SetSwitch(true);
      return;
    default:
      FML_DCHECK(false);
  }
}

void Shell::OnDisplayUpdates(std::vector<std::unique_ptr<Display>> displays) {
  FML_DCHECK(is_set_up_);
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());

  std::vector<DisplayData> display_data;
  display_data.reserve(displays.size());
  for (const auto& display : displays) {
    display_data.push_back(display->GetDisplayData());
  }
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetUITaskRunner(),
                                    [engine = engine_->GetWeakPtr(),
                                     display_data = std::move(display_data)]() {
                                      if (engine) {
                                        engine->SetDisplays(display_data);
                                      }
                                    });

  display_manager_->HandleDisplayUpdates(std::move(displays));
}

fml::TimePoint Shell::GetCurrentTimePoint() {
  return fml::TimePoint::Now();
}

const std::shared_ptr<PlatformMessageHandler>&
Shell::GetPlatformMessageHandler() const {
  return platform_message_handler_;
}

const std::weak_ptr<VsyncWaiter> Shell::GetVsyncWaiter() const {
  if (!engine_) {
    return {};
  }
  return engine_->GetVsyncWaiter();
}

const std::shared_ptr<fml::ConcurrentTaskRunner>
Shell::GetConcurrentWorkerTaskRunner() const {
  FML_DCHECK(vm_);
  if (!vm_) {
    return nullptr;
  }
  return vm_->GetConcurrentWorkerTaskRunner();
}

SkISize Shell::ExpectedFrameSize(int64_t view_id) {
  auto found = expected_frame_sizes_.find(view_id);
  if (found == expected_frame_sizes_.end()) {
    return SkISize::MakeEmpty();
  }
  return found->second;
}

}  // namespace flutter
