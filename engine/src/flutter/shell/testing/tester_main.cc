// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <cstdlib>
#include <cstring>
#include <iostream>

#include "flutter/assets/asset_manager.h"
#include "flutter/assets/directory_asset_bundle.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/file.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/shell.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/gpu/gpu_surface_software.h"
#include "flutter/third_party/abseil-cpp/absl/base/no_destructor.h"

#include "third_party/dart/runtime/include/bin/dart_io_api.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/skia/include/core/SkSurface.h"

// Impeller should only be enabled if the Vulkan backend is enabled.
#define ALLOW_IMPELLER (IMPELLER_SUPPORTS_RENDERING && IMPELLER_ENABLE_VULKAN)

#if ALLOW_IMPELLER
#include <vulkan/vulkan.h>                                        // nogncheck
#include "impeller/entity/vk/entity_shaders_vk.h"                 // nogncheck
#include "impeller/entity/vk/framebuffer_blend_shaders_vk.h"      // nogncheck
#include "impeller/entity/vk/modern_shaders_vk.h"                 // nogncheck
#include "impeller/renderer/backend/vulkan/context_vk.h"          // nogncheck
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"  // nogncheck
#include "impeller/renderer/context.h"                            // nogncheck
#include "impeller/renderer/vk/compute_shaders_vk.h"              // nogncheck
#include "shell/gpu/gpu_surface_vulkan_impeller.h"                // nogncheck

static std::vector<std::shared_ptr<fml::Mapping>> ShaderLibraryMappings() {
  return {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_vk_data,
                                             impeller_entity_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_vk_data,
                                             impeller_modern_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_vk_data,
          impeller_framebuffer_blend_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_compute_shaders_vk_data, impeller_compute_shaders_vk_length),
  };
}

struct ImpellerVulkanContextHolder {
  ImpellerVulkanContextHolder() = default;
  ImpellerVulkanContextHolder(ImpellerVulkanContextHolder&&) = default;
  std::shared_ptr<impeller::ContextVK> context;
  std::shared_ptr<impeller::SurfaceContextVK> surface_context;

  bool Initialize(bool enable_validation);
};

bool ImpellerVulkanContextHolder::Initialize(bool enable_validation) {
  impeller::ContextVK::Settings context_settings;
  context_settings.proc_address_callback = &vkGetInstanceProcAddr;
  context_settings.shader_libraries_data = ShaderLibraryMappings();
  context_settings.cache_directory = fml::paths::GetCachesDirectory();
  context_settings.enable_validation = enable_validation;

  context = impeller::ContextVK::Create(std::move(context_settings));
  if (!context || !context->IsValid()) {
    VALIDATION_LOG << "Could not create Vulkan context.";
    return false;
  }

  impeller::vk::SurfaceKHR vk_surface;
  impeller::vk::HeadlessSurfaceCreateInfoEXT surface_create_info;
  auto res = context->GetInstance().createHeadlessSurfaceEXT(
      &surface_create_info,  // surface create info
      nullptr,               // allocator
      &vk_surface            // surface
  );
  if (res != impeller::vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create surface for tester "
                   << impeller::vk::to_string(res);
    return false;
  }

  impeller::vk::UniqueSurfaceKHR surface{vk_surface, context->GetInstance()};
  surface_context = context->CreateSurfaceContext();
  if (!surface_context->SetWindowSurface(std::move(surface),
                                         impeller::ISize{1, 1})) {
    VALIDATION_LOG << "Could not set up surface for context.";
    return false;
  }
  return true;
}

#else
struct ImpellerVulkanContextHolder {};
#endif  // IMPELLER_SUPPORTS_RENDERING

#if defined(FML_OS_WIN)
#include <combaseapi.h>
#endif  // defined(FML_OS_WIN)

#if defined(FML_OS_POSIX)
#include <signal.h>
#endif  // defined(FML_OS_POSIX)

namespace flutter {

static absl::NoDestructor<std::unique_ptr<Shell>> g_shell;

static constexpr int64_t kImplicitViewId = 0ll;

static void ConfigureShell(Shell* shell) {
  auto device_pixel_ratio = 3.0;
  auto physical_width = 2400.0;   // 800 at 3x resolution.
  auto physical_height = 1800.0;  // 600 at 3x resolution.

  std::vector<std::unique_ptr<Display>> displays;
  displays.push_back(std::make_unique<Display>(
      0, 60, physical_width, physical_height, device_pixel_ratio));
  shell->OnDisplayUpdates(std::move(displays));

  flutter::ViewportMetrics metrics{};
  metrics.device_pixel_ratio = device_pixel_ratio;
  metrics.physical_width = physical_width;
  metrics.physical_height = physical_height;
  metrics.display_id = 0;
  shell->GetPlatformView()->SetViewportMetrics(kImplicitViewId, metrics);
}

class TesterExternalViewEmbedder : public ExternalViewEmbedder {
  // |ExternalViewEmbedder|
  DlCanvas* GetRootCanvas() override { return nullptr; }

  // |ExternalViewEmbedder|
  void CancelFrame() override {}

  // |ExternalViewEmbedder|
  void BeginFrame(GrDirectContext* context,
                  const fml::RefPtr<fml::RasterThreadMerger>&
                      raster_thread_merger) override {}

  // |ExternalViewEmbedder|
  void PrepareFlutterView(SkISize frame_size,
                          double device_pixel_ratio) override {}

  // |ExternalViewEmbedder|
  void PrerollCompositeEmbeddedView(
      int64_t view_id,
      std::unique_ptr<EmbeddedViewParams> params) override {}

  // |ExternalViewEmbedder|
  DlCanvas* CompositeEmbeddedView(int64_t view_id) override {
    return &builder_;
  }

 private:
  DisplayListBuilder builder_;
};

class TesterGPUSurfaceSoftware : public GPUSurfaceSoftware {
 public:
  TesterGPUSurfaceSoftware(GPUSurfaceSoftwareDelegate* delegate,
                           bool render_to_surface)
      : GPUSurfaceSoftware(delegate, render_to_surface) {}

  bool EnableRasterCache() const override { return false; }
};

class TesterPlatformView : public PlatformView,
                           public GPUSurfaceSoftwareDelegate {
 public:
  TesterPlatformView(Delegate& delegate,
                     const TaskRunners& task_runners,
                     ImpellerVulkanContextHolder&& impeller_context_holder)
      : PlatformView(delegate, task_runners),
        impeller_context_holder_(std::move(impeller_context_holder)) {}

  ~TesterPlatformView() {
#if ALLOW_IMPELLER
    if (impeller_context_holder_.context) {
      impeller_context_holder_.context->Shutdown();
    }
#endif
  }

  // |PlatformView|
  std::shared_ptr<impeller::Context> GetImpellerContext() const override {
#if ALLOW_IMPELLER
    return std::static_pointer_cast<impeller::Context>(
        impeller_context_holder_.context);
#else
    return nullptr;
#endif  // ALLOW_IMPELLER
  }

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override {
#if ALLOW_IMPELLER
    if (delegate_.OnPlatformViewGetSettings().enable_impeller) {
      FML_DCHECK(impeller_context_holder_.context);
      auto surface = std::make_unique<GPUSurfaceVulkanImpeller>(
          nullptr, impeller_context_holder_.surface_context);
      FML_DCHECK(surface->IsValid());
      return surface;
    }
#endif  // ALLOW_IMPELLER
    auto surface = std::make_unique<TesterGPUSurfaceSoftware>(
        this, true /* render to surface */);
    FML_DCHECK(surface->IsValid());
    return surface;
  }

  // |GPUSurfaceSoftwareDelegate|
  sk_sp<SkSurface> AcquireBackingStore(const SkISize& size) override {
    if (sk_surface_ != nullptr &&
        SkISize::Make(sk_surface_->width(), sk_surface_->height()) == size) {
      // The old and new surface sizes are the same. Nothing to do here.
      return sk_surface_;
    }

    SkImageInfo info =
        SkImageInfo::MakeN32(size.fWidth, size.fHeight, kPremul_SkAlphaType,
                             SkColorSpace::MakeSRGB());
    sk_surface_ = SkSurfaces::Raster(info, nullptr);

    if (sk_surface_ == nullptr) {
      FML_LOG(ERROR)
          << "Could not create backing store for software rendering.";
      return nullptr;
    }

    return sk_surface_;
  }

  // |GPUSurfaceSoftwareDelegate|
  bool PresentBackingStore(sk_sp<SkSurface> backing_store) override {
    return true;
  }

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override {
    return external_view_embedder_;
  }

 private:
  sk_sp<SkSurface> sk_surface_ = nullptr;
  [[maybe_unused]] ImpellerVulkanContextHolder impeller_context_holder_;
  std::shared_ptr<TesterExternalViewEmbedder> external_view_embedder_ =
      std::make_shared<TesterExternalViewEmbedder>();
};

// Checks whether the engine's main Dart isolate has no pending work.  If so,
// then exit the given message loop.
class ScriptCompletionTaskObserver {
 public:
  ScriptCompletionTaskObserver(Shell& shell,
                               fml::RefPtr<fml::TaskRunner> main_task_runner,
                               fml::RefPtr<fml::TaskRunner> ui_task_runner,
                               bool run_forever)
      : shell_(shell),
        main_task_runner_(std::move(main_task_runner)),
        ui_task_runner_(std::move(ui_task_runner)),
        run_forever_(run_forever) {}

  int GetExitCodeForLastError() const {
    return static_cast<int>(last_error_.value_or(DartErrorCode::NoError));
  }

  void DidProcessTask() {
    last_error_ = shell_.GetUIIsolateLastError();
    if (shell_.EngineHasLivePorts()) {
      // The UI isolate still has live ports and is running. Nothing to do
      // just yet.
      return;
    }
    if (shell_.EngineHasPendingMicrotasks()) {
      // Post an empty task to force a run of the engine task observer that
      // drains the microtask queue.
      ui_task_runner_->PostTask([] {});
      return;
    }

    if (run_forever_) {
      // We need this script to run forever. We have already recorded the last
      // error. Keep going.
      return;
    }

    if (!has_terminated_) {
      // Only try to terminate the loop once.
      has_terminated_ = true;
      fml::TaskRunner::RunNowOrPostTask(main_task_runner_, []() {
        fml::MessageLoop::GetCurrent().Terminate();
      });
    }
  }

 private:
  Shell& shell_;
  fml::RefPtr<fml::TaskRunner> main_task_runner_;
  fml::RefPtr<fml::TaskRunner> ui_task_runner_;
  bool run_forever_ = false;
  std::optional<DartErrorCode> last_error_;
  bool has_terminated_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(ScriptCompletionTaskObserver);
};

// Processes spawned via dart:io inherit their signal handling from the parent
// process. As part of spawning, the spawner blocks signals temporarily, so we
// need to explicitly unblock the signals we care about in the new process. In
// particular, we need to unblock SIGPROF for CPU profiling to work on the
// mutator thread in the main isolate in this process (threads spawned by the VM
// know about this limitation and automatically have this signal unblocked).
static void UnblockSIGPROF() {
#if defined(FML_OS_POSIX)
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGPROF);
  pthread_sigmask(SIG_UNBLOCK, &set, NULL);
#endif  // defined(FML_OS_POSIX)
}

int RunTester(const flutter::Settings& settings,
              bool run_forever,
              bool multithreaded) {
  const auto thread_label = "io.flutter.test.";

  // Necessary if we want to use the CPU profiler on the main isolate's mutator
  // thread.
  //
  // OSX WARNING: avoid spawning additional threads before this call due to a
  // kernel bug that may enable SIGPROF on an unintended thread in the process.
  UnblockSIGPROF();

  fml::MessageLoop::EnsureInitializedForCurrentThread();

  auto current_task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();

  std::unique_ptr<ThreadHost> threadhost;
  fml::RefPtr<fml::TaskRunner> platform_task_runner;
  fml::RefPtr<fml::TaskRunner> raster_task_runner;
  fml::RefPtr<fml::TaskRunner> ui_task_runner;
  fml::RefPtr<fml::TaskRunner> io_task_runner;

  if (multithreaded) {
    threadhost = std::make_unique<ThreadHost>(
        thread_label, ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                          ThreadHost::Type::kUi | ThreadHost::Type::kRaster);
    platform_task_runner = current_task_runner;
    raster_task_runner = threadhost->raster_thread->GetTaskRunner();
    ui_task_runner = threadhost->ui_thread->GetTaskRunner();
    io_task_runner = threadhost->io_thread->GetTaskRunner();
  } else {
    platform_task_runner = raster_task_runner = ui_task_runner =
        io_task_runner = current_task_runner;
  }

  const flutter::TaskRunners task_runners(thread_label,  // dart thread label
                                          platform_task_runner,  // platform
                                          raster_task_runner,    // raster
                                          ui_task_runner,        // ui
                                          io_task_runner         // io
  );

  ImpellerVulkanContextHolder impeller_context_holder;

#if ALLOW_IMPELLER
  if (settings.enable_impeller) {
    if (!impeller_context_holder.Initialize(
            settings.enable_vulkan_validation)) {
      return EXIT_FAILURE;
    }
  }
#endif  // ALLOW_IMPELLER

  Shell::CreateCallback<PlatformView> on_create_platform_view =
      fml::MakeCopyable([impeller_context_holder = std::move(
                             impeller_context_holder)](Shell& shell) mutable {
        return std::make_unique<TesterPlatformView>(
            shell, shell.GetTaskRunners(), std::move(impeller_context_holder));
      });

  Shell::CreateCallback<Rasterizer> on_create_rasterizer = [](Shell& shell) {
    return std::make_unique<Rasterizer>(
        shell, Rasterizer::MakeGpuImageBehavior::kBitmap);
  };

  g_shell->reset(Shell::Create(flutter::PlatformData(),  //
                               task_runners,             //
                               settings,                 //
                               on_create_platform_view,  //
                               on_create_rasterizer      //
                               )
                     .release());
  auto shell = g_shell->get();

  if (!shell || !shell->IsSetup()) {
    FML_LOG(ERROR) << "Could not set up the shell.";
    return EXIT_FAILURE;
  }

  if (settings.application_kernel_asset.empty()) {
    FML_LOG(ERROR) << "Dart kernel file not specified.";
    return EXIT_FAILURE;
  }

  shell->GetPlatformView()->NotifyCreated();

  // Initialize default testing locales. There is no platform to
  // pass locales on the tester, so to retain expected locale behavior,
  // we emulate it in here by passing in 'en_US' and 'zh_CN' as test locales.
  const char* locale_json =
      "{\"method\":\"setLocale\",\"args\":[\"en\",\"US\",\"\",\"\",\"zh\","
      "\"CN\",\"\",\"\"]}";
  auto locale_bytes = fml::MallocMapping::Copy(
      locale_json, locale_json + std::strlen(locale_json));
  fml::RefPtr<flutter::PlatformMessageResponse> response;
  shell->GetPlatformView()->DispatchPlatformMessage(
      std::make_unique<flutter::PlatformMessage>(
          "flutter/localization", std::move(locale_bytes), response));

  std::initializer_list<fml::FileMapping::Protection> protection = {
      fml::FileMapping::Protection::kRead};
  auto main_dart_file_mapping = std::make_unique<fml::FileMapping>(
      fml::OpenFile(
          fml::paths::AbsolutePath(settings.application_kernel_asset).c_str(),
          false, fml::FilePermission::kRead),
      protection);

  auto isolate_configuration =
      IsolateConfiguration::CreateForKernel(std::move(main_dart_file_mapping));

  if (!isolate_configuration) {
    FML_LOG(ERROR) << "Could create isolate configuration.";
    return EXIT_FAILURE;
  }

  auto asset_manager = std::make_shared<flutter::AssetManager>();
  asset_manager->PushBack(std::make_unique<flutter::DirectoryAssetBundle>(
      fml::Duplicate(settings.assets_dir), true));
  asset_manager->PushBack(std::make_unique<flutter::DirectoryAssetBundle>(
      fml::OpenDirectory(settings.assets_path.c_str(), false,
                         fml::FilePermission::kRead),
      true));

  RunConfiguration run_configuration(std::move(isolate_configuration),
                                     std::move(asset_manager));

  // The script completion task observer that will be installed on the UI thread
  // that watched if the engine has any live ports.
  ScriptCompletionTaskObserver completion_observer(
      *shell,  // a valid shell
      fml::MessageLoop::GetCurrent()
          .GetTaskRunner(),  // the message loop to terminate
      ui_task_runner,        // runner for Dart microtasks
      run_forever            // should the exit be ignored
  );

  bool engine_did_run = false;

  fml::AutoResetWaitableEvent latch;
  auto task_observer_add = [&completion_observer]() {
    fml::MessageLoop::GetCurrent().AddTaskObserver(
        reinterpret_cast<intptr_t>(&completion_observer),
        [&completion_observer]() { completion_observer.DidProcessTask(); });
  };

  auto task_observer_remove = [&completion_observer, &latch]() {
    fml::MessageLoop::GetCurrent().RemoveTaskObserver(
        reinterpret_cast<intptr_t>(&completion_observer));
    latch.Signal();
  };

  shell->RunEngine(std::move(run_configuration),
                   [&engine_did_run, &ui_task_runner,
                    &task_observer_add](Engine::RunStatus run_status) mutable {
                     if (run_status != flutter::Engine::RunStatus::Failure) {
                       engine_did_run = true;
                       // Now that our engine is initialized we can install the
                       // ScriptCompletionTaskObserver
                       fml::TaskRunner::RunNowOrPostTask(ui_task_runner,
                                                         task_observer_add);
                     }
                   });

  ConfigureShell(shell);

  // Run the message loop and wait for the script to do its thing.
  fml::MessageLoop::GetCurrent().Run();

  // Cleanup the completion observer synchronously as it is living on the
  // stack.
  fml::TaskRunner::RunNowOrPostTask(ui_task_runner, task_observer_remove);
  latch.Wait();

  delete g_shell->release();

  if (!engine_did_run) {
    // If the engine itself didn't have a chance to run, there is no point in
    // asking it if there was an error. Signal a failure unconditionally.
    return EXIT_FAILURE;
  }

  return completion_observer.GetExitCodeForLastError();
}

#ifdef _WIN32
#define EXPORTED __declspec(dllexport)
#else
#define EXPORTED __attribute__((visibility("default")))
#endif

extern "C" {
EXPORTED Dart_Handle LoadLibraryFromKernel(const char* path) {
  std::shared_ptr<fml::FileMapping> mapping =
      fml::FileMapping::CreateReadOnly(path);
  if (!mapping) {
    return Dart_Null();
  }
  return DartIsolate::LoadLibraryFromKernel(mapping);
}

EXPORTED Dart_Handle LookupEntryPoint(const char* uri, const char* name) {
  if (!uri || !name) {
    return Dart_Null();
  }
  Dart_Handle lib = Dart_LookupLibrary(Dart_NewStringFromCString(uri));
  if (Dart_IsError(lib)) {
    return lib;
  }
  return Dart_GetField(lib, Dart_NewStringFromCString(name));
}

EXPORTED void Spawn(const char* entrypoint, const char* route) {
  auto shell = g_shell->get();
  auto isolate = Dart_CurrentIsolate();
  auto spawn_task = [shell, entrypoint = std::string(entrypoint),
                     route = std::string(route)]() {
    auto configuration = RunConfiguration::InferFromSettings(
        shell->GetSettings(), /*io_worker=*/nullptr,
        /*launch_type=*/IsolateLaunchType::kExistingGroup);
    configuration.SetEntrypoint(entrypoint);

    Shell::CreateCallback<PlatformView> on_create_platform_view =
        fml::MakeCopyable([](Shell& shell) mutable {
          ImpellerVulkanContextHolder impeller_context_holder;
          return std::make_unique<TesterPlatformView>(
              shell, shell.GetTaskRunners(),
              std::move(impeller_context_holder));
        });

    Shell::CreateCallback<Rasterizer> on_create_rasterizer = [](Shell& shell) {
      return std::make_unique<Rasterizer>(
          shell, Rasterizer::MakeGpuImageBehavior::kBitmap);
    };

    // Spawn a shell, and keep it running until it has no live ports, then
    // delete it on the platform thread.
    auto spawned_shell =
        shell
            ->Spawn(std::move(configuration), route, on_create_platform_view,
                    on_create_rasterizer)
            .release();

    ConfigureShell(spawned_shell);

    fml::TaskRunner::RunNowOrPostTask(
        spawned_shell->GetTaskRunners().GetUITaskRunner(), [spawned_shell]() {
          fml::MessageLoop::GetCurrent().AddTaskObserver(
              reinterpret_cast<intptr_t>(spawned_shell), [spawned_shell]() {
                if (spawned_shell->EngineHasLivePorts()) {
                  return;
                }

                fml::MessageLoop::GetCurrent().RemoveTaskObserver(
                    reinterpret_cast<intptr_t>(spawned_shell));
                // Shell must be deleted on the platform task runner.
                fml::TaskRunner::RunNowOrPostTask(
                    spawned_shell->GetTaskRunners().GetPlatformTaskRunner(),
                    [spawned_shell]() { delete spawned_shell; });
              });
        });
  };
  Dart_ExitIsolate();
  // The global shell pointer is never deleted, short of application exit.
  // This UI task runner cannot be latched because it will block spawning a new
  // shell in the case where flutter_tester is running multithreaded.
  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetPlatformTaskRunner(), spawn_task);

  Dart_EnterIsolate(isolate);
}

EXPORTED void ForceShutdownIsolate() {
  // Enable Isolate.exit().
  FML_DCHECK(Dart_CurrentIsolate() != nullptr);
  Dart_Handle isolate_lib = Dart_LookupLibrary(tonic::ToDart("dart:isolate"));
  FML_CHECK(!tonic::CheckAndHandleError(isolate_lib));
  Dart_Handle isolate_type = Dart_GetNonNullableType(
      isolate_lib, tonic::ToDart("Isolate"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(isolate_type));
  Dart_Handle result =
      Dart_SetField(isolate_type, tonic::ToDart("_mayExit"), Dart_True());
  FML_CHECK(!tonic::CheckAndHandleError(result));
}
}

}  // namespace flutter

int main(int argc, char* argv[]) {
  dart::bin::SetExecutableName(argv[0]);
  dart::bin::SetExecutableArguments(argc - 1, argv);

  auto command_line = fml::CommandLineFromPlatformOrArgcArgv(argc, argv);

  if (command_line.HasOption(flutter::FlagForSwitch(flutter::Switch::Help))) {
    flutter::PrintUsage("flutter_tester");
    return EXIT_SUCCESS;
  }

  auto settings = flutter::SettingsFromCommandLine(command_line);
  if (!command_line.positional_args().empty()) {
    // The tester may not use the switch for the main dart file path. Specifying
    // it as a positional argument instead.
    settings.application_kernel_asset = command_line.positional_args()[0];
  }

  if (settings.application_kernel_asset.empty()) {
    FML_LOG(ERROR) << "Dart kernel file not specified.";
    return EXIT_FAILURE;
  }

  settings.leak_vm = false;
  settings.enable_platform_isolates = true;

  if (settings.icu_data_path.empty()) {
    settings.icu_data_path = "icudtl.dat";
  }

  // The tools that read logs get confused if there is a log tag specified.
  settings.log_tag = "";

  settings.log_message_callback = [](const std::string& tag,
                                     const std::string& message) {
    if (!tag.empty()) {
      std::cout << tag << ": ";
    }
    std::cout << message << std::endl;
  };

  settings.task_observer_add = [](intptr_t key, const fml::closure& callback) {
    fml::MessageLoop::GetCurrent().AddTaskObserver(key, callback);
  };

  settings.task_observer_remove = [](intptr_t key) {
    fml::MessageLoop::GetCurrent().RemoveTaskObserver(key);
  };

  settings.unhandled_exception_callback = [](const std::string& error,
                                             const std::string& stack_trace) {
    FML_LOG(ERROR) << "Unhandled exception" << std::endl
                   << "Exception: " << error << std::endl
                   << "Stack trace: " << stack_trace;
    ::exit(1);
    return true;
  };

#if defined(FML_OS_WIN)
  CoInitializeEx(nullptr, COINIT_MULTITHREADED);
#endif  // defined(FML_OS_WIN)

  return flutter::RunTester(settings,
                            command_line.HasOption(flutter::FlagForSwitch(
                                flutter::Switch::RunForever)),
                            command_line.HasOption(flutter::FlagForSwitch(
                                flutter::Switch::ForceMultithreading)));
}
