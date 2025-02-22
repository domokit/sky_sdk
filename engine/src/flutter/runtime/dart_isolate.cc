// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_isolate.h"

#include <cstdlib>
#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/fml/posix_wrappers.h"
#include "flutter/fml/trace_event.h"
#include "flutter/lib/io/dart_io.h"
#include "flutter/lib/ui/dart_runtime_hooks.h"
#include "flutter/lib/ui/dart_ui.h"
#include "flutter/runtime/dart_isolate_group_data.h"
#include "flutter/runtime/dart_plugin_registrant.h"
#include "flutter/runtime/dart_service_isolate.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/runtime/isolate_configuration.h"
#include "flutter/runtime/platform_isolate_manager.h"
#include "fml/message_loop_task_queues.h"
#include "fml/task_source.h"
#include "fml/time/time_point.h"
#include "third_party/dart/runtime/include/bin/native_assets_api.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_class_library.h"
#include "third_party/tonic/dart_class_provider.h"
#include "third_party/tonic/dart_message_handler.h"
#include "third_party/tonic/dart_state.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/scopes/dart_api_scope.h"
#include "third_party/tonic/scopes/dart_isolate_scope.h"

namespace flutter {

namespace {

constexpr std::string_view kFileUriPrefix = "file://";

class DartErrorString {
 public:
  DartErrorString() {}
  ~DartErrorString() {
    if (str_) {
      ::free(str_);
    }
  }
  char** error() { return &str_; }
  const char* str() const { return str_; }
  explicit operator bool() const { return str_ != nullptr; }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(DartErrorString);
  char* str_ = nullptr;
};

}  // anonymous namespace

DartIsolate::Flags::Flags() : Flags(nullptr) {}

DartIsolate::Flags::Flags(const Dart_IsolateFlags* flags) {
  if (flags) {
    flags_ = *flags;
  } else {
    ::Dart_IsolateFlagsInitialize(&flags_);
  }
}

DartIsolate::Flags::~Flags() = default;

void DartIsolate::Flags::SetNullSafetyEnabled(bool enabled) {
  flags_.null_safety = enabled;
}

void DartIsolate::Flags::SetIsDontNeedSafe(bool value) {
  flags_.snapshot_is_dontneed_safe = value;
}

Dart_IsolateFlags DartIsolate::Flags::Get() const {
  return flags_;
}

std::weak_ptr<DartIsolate> DartIsolate::CreateRunningRootIsolate(
    const Settings& settings,
    const fml::RefPtr<const DartSnapshot>& isolate_snapshot,
    std::unique_ptr<PlatformConfiguration> platform_configuration,
    Flags isolate_flags,
    const fml::closure& root_isolate_create_callback,
    const fml::closure& isolate_create_callback,
    const fml::closure& isolate_shutdown_callback,
    std::optional<std::string> dart_entrypoint,
    std::optional<std::string> dart_entrypoint_library,
    const std::vector<std::string>& dart_entrypoint_args,
    std::unique_ptr<IsolateConfiguration> isolate_configuration,
    const UIDartState::Context& context,
    const DartIsolate* spawning_isolate,
    std::shared_ptr<NativeAssetsManager> native_assets_manager) {
  if (!isolate_snapshot) {
    FML_LOG(ERROR) << "Invalid isolate snapshot.";
    return {};
  }

  if (!isolate_configuration) {
    FML_LOG(ERROR) << "Invalid isolate configuration.";
    return {};
  }

  isolate_flags.SetNullSafetyEnabled(
      isolate_configuration->IsNullSafetyEnabled(*isolate_snapshot));
  isolate_flags.SetIsDontNeedSafe(isolate_snapshot->IsDontNeedSafe());

  auto isolate = CreateRootIsolate(settings,                           //
                                   isolate_snapshot,                   //
                                   std::move(platform_configuration),  //
                                   isolate_flags,                      //
                                   isolate_create_callback,            //
                                   isolate_shutdown_callback,          //
                                   context,                            //
                                   spawning_isolate,                   //
                                   std::move(native_assets_manager)    //
                                   )
                     .lock();

  if (!isolate) {
    FML_LOG(ERROR) << "Could not create root isolate.";
    return {};
  }

  fml::ScopedCleanupClosure shutdown_on_error([isolate]() {
    if (!isolate->Shutdown()) {
      FML_DLOG(ERROR) << "Could not shutdown transient isolate.";
    }
  });

  if (isolate->GetPhase() != DartIsolate::Phase::LibrariesSetup) {
    FML_LOG(ERROR) << "Root isolate was created in an incorrect phase: "
                   << static_cast<int>(isolate->GetPhase());
    return {};
  }

  if (!isolate_configuration->PrepareIsolate(*isolate.get())) {
    FML_LOG(ERROR) << "Could not prepare isolate.";
    return {};
  }

  if (isolate->GetPhase() != DartIsolate::Phase::Ready) {
    FML_LOG(ERROR) << "Root isolate not in the ready phase for Dart entrypoint "
                      "invocation.";
    return {};
  }

  {
    tonic::DartState::Scope scope(isolate.get());
    Dart_SetCurrentThreadOwnsIsolate();

    if (settings.root_isolate_create_callback) {
      // Isolate callbacks always occur in isolate scope and before user code
      // has had a chance to run.
      settings.root_isolate_create_callback(*isolate.get());
    }
  }

  if (root_isolate_create_callback) {
    root_isolate_create_callback();
  }

  if (!isolate->RunFromLibrary(std::move(dart_entrypoint_library),  //
                               std::move(dart_entrypoint),          //
                               dart_entrypoint_args)) {
    FML_LOG(ERROR) << "Could not run the run main Dart entrypoint.";
    return {};
  }

  if (settings.root_isolate_shutdown_callback) {
    isolate->AddIsolateShutdownCallback(
        settings.root_isolate_shutdown_callback);
  }

  shutdown_on_error.Release();

  return isolate;
}

void DartIsolate::SpawnIsolateShutdownCallback(
    std::shared_ptr<DartIsolateGroupData>* isolate_group_data,
    std::shared_ptr<DartIsolate>* isolate_data) {
  DartIsolate::DartIsolateShutdownCallback(isolate_group_data, isolate_data);
}

std::weak_ptr<DartIsolate> DartIsolate::CreateRootIsolate(
    const Settings& settings,
    fml::RefPtr<const DartSnapshot> isolate_snapshot,
    std::unique_ptr<PlatformConfiguration> platform_configuration,
    const Flags& flags,
    const fml::closure& isolate_create_callback,
    const fml::closure& isolate_shutdown_callback,
    const UIDartState::Context& context,
    const DartIsolate* spawning_isolate,
    std::shared_ptr<NativeAssetsManager> native_assets_manager) {
  TRACE_EVENT0("flutter", "DartIsolate::CreateRootIsolate");

  // Only needed if this is the main isolate for the group.
  std::unique_ptr<std::shared_ptr<DartIsolateGroupData>> isolate_group_data;

  auto isolate_data = std::make_unique<std::shared_ptr<DartIsolate>>(
      std::shared_ptr<DartIsolate>(new DartIsolate(
          /*settings=*/settings,
          /*is_root_isolate=*/true,
          /*context=*/context,
          /*is_spawning_in_group=*/!!spawning_isolate)));

  DartErrorString error;
  Dart_Isolate vm_isolate = nullptr;
  auto isolate_flags = flags.Get();

  IsolateMaker isolate_maker;
  if (spawning_isolate) {
    isolate_maker =
        [spawning_isolate](
            std::shared_ptr<DartIsolateGroupData>* isolate_group_data,
            std::shared_ptr<DartIsolate>* isolate_data,
            Dart_IsolateFlags* flags, char** error) {
          return Dart_CreateIsolateInGroup(
              /*group_member=*/spawning_isolate->isolate(),
              /*name=*/
              spawning_isolate->GetIsolateGroupData()
                  .GetAdvisoryScriptEntrypoint()
                  .c_str(),
              /*shutdown_callback=*/
              reinterpret_cast<Dart_IsolateShutdownCallback>(
                  DartIsolate::SpawnIsolateShutdownCallback),
              /*cleanup_callback=*/
              reinterpret_cast<Dart_IsolateCleanupCallback>(
                  DartIsolateCleanupCallback),
              /*child_isolate_data=*/isolate_data,
              /*error=*/error);
        };
  } else {
    // The child isolate preparer is null but will be set when the isolate is
    // being prepared to run.
    isolate_group_data =
        std::make_unique<std::shared_ptr<DartIsolateGroupData>>(
            std::shared_ptr<DartIsolateGroupData>(new DartIsolateGroupData(
                settings,                            // settings
                std::move(isolate_snapshot),         // isolate snapshot
                context.advisory_script_uri,         // advisory URI
                context.advisory_script_entrypoint,  // advisory entrypoint
                nullptr,                             // child isolate preparer
                isolate_create_callback,             // isolate create callback
                isolate_shutdown_callback,        // isolate shutdown callback
                std::move(native_assets_manager)  //
                )));
    isolate_maker = [](std::shared_ptr<DartIsolateGroupData>*
                           isolate_group_data,
                       std::shared_ptr<DartIsolate>* isolate_data,
                       Dart_IsolateFlags* flags, char** error) {
      return Dart_CreateIsolateGroup(
          (*isolate_group_data)->GetAdvisoryScriptURI().c_str(),
          (*isolate_group_data)->GetAdvisoryScriptEntrypoint().c_str(),
          (*isolate_group_data)->GetIsolateSnapshot()->GetDataMapping(),
          (*isolate_group_data)->GetIsolateSnapshot()->GetInstructionsMapping(),
          flags, isolate_group_data, isolate_data, error);
    };
  }

  vm_isolate = CreateDartIsolateGroup(std::move(isolate_group_data),
                                      std::move(isolate_data), &isolate_flags,
                                      error.error(), isolate_maker);

  if (error) {
    FML_LOG(ERROR) << "CreateRootIsolate failed: " << error.str();
  }

  if (vm_isolate == nullptr) {
    return {};
  }

  std::shared_ptr<DartIsolate>* root_isolate_data =
      static_cast<std::shared_ptr<DartIsolate>*>(Dart_IsolateData(vm_isolate));

  (*root_isolate_data)
      ->SetPlatformConfiguration(std::move(platform_configuration));

  return (*root_isolate_data)->GetWeakIsolatePtr();
}

Dart_Isolate DartIsolate::CreatePlatformIsolate(Dart_Handle entry_point,
                                                char** error) {
  *error = nullptr;
  PlatformConfiguration* platform_config = platform_configuration();
  FML_DCHECK(platform_config != nullptr);
  std::shared_ptr<PlatformIsolateManager> platform_isolate_manager =
      platform_config->client()->GetPlatformIsolateManager();
  std::weak_ptr<PlatformIsolateManager> weak_platform_isolate_manager =
      platform_isolate_manager;
  if (platform_isolate_manager->HasShutdownMaybeFalseNegative()) {
    // Don't set the error string. We want to silently ignore this error,
    // because the engine is shutting down.
    FML_LOG(INFO) << "CreatePlatformIsolate called after shutdown";
    return nullptr;
  }

  Dart_Isolate parent_isolate = isolate();
  Dart_ExitIsolate();  // Exit parent_isolate.

  const TaskRunners& task_runners = GetTaskRunners();
  fml::RefPtr<fml::TaskRunner> platform_task_runner =
      task_runners.GetPlatformTaskRunner();
  FML_DCHECK(platform_task_runner);

  auto isolate_group_data = std::shared_ptr<DartIsolateGroupData>(
      *static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_IsolateGroupData(parent_isolate)));

  Settings settings(isolate_group_data->GetSettings());

  // PlatformIsolate.spawn should behave like Isolate.spawn when unhandled
  // exceptions happen (log the exception, but don't terminate the app). But the
  // default unhandled_exception_callback may terminate the app, because it is
  // only called for the root isolate (child isolates are managed by the VM and
  // have a different error code path). So override it to simply log the error.
  settings.unhandled_exception_callback = [](const std::string& error,
                                             const std::string& stack_trace) {
    FML_LOG(ERROR) << "Unhandled exception:\n" << error << "\n" << stack_trace;
    return true;
  };

  // The platform isolate task observer must be added on the platform thread. So
  // schedule the add function on the platform task runner.
  TaskObserverAdd old_task_observer_add = settings.task_observer_add;
  settings.task_observer_add = [old_task_observer_add, platform_task_runner,
                                weak_platform_isolate_manager](
                                   intptr_t key, const fml::closure& callback) {
    platform_task_runner->PostTask([old_task_observer_add,
                                    weak_platform_isolate_manager, key,
                                    callback]() {
      std::shared_ptr<PlatformIsolateManager> platform_isolate_manager =
          weak_platform_isolate_manager.lock();
      if (platform_isolate_manager == nullptr ||
          platform_isolate_manager->HasShutdown()) {
        // Shutdown happened in between this task being posted, and it running.
        // platform_isolate has already been shut down. Do nothing.
        FML_LOG(INFO) << "Shutdown before platform isolate task observer added";
        return;
      }
      old_task_observer_add(key, callback);
    });
  };

  UIDartState::Context context(task_runners);
  context.advisory_script_uri = isolate_group_data->GetAdvisoryScriptURI();
  context.advisory_script_entrypoint =
      isolate_group_data->GetAdvisoryScriptEntrypoint();
  auto isolate_data = std::make_unique<std::shared_ptr<DartIsolate>>(
      std::shared_ptr<DartIsolate>(
          new DartIsolate(settings, context, platform_isolate_manager)));

  IsolateMaker isolate_maker =
      [parent_isolate](
          std::shared_ptr<DartIsolateGroupData>* unused_isolate_group_data,
          std::shared_ptr<DartIsolate>* isolate_data, Dart_IsolateFlags* flags,
          char** error) {
        return Dart_CreateIsolateInGroup(
            /*group_member=*/parent_isolate,
            /*name=*/"PlatformIsolate",
            /*shutdown_callback=*/
            reinterpret_cast<Dart_IsolateShutdownCallback>(
                DartIsolate::SpawnIsolateShutdownCallback),
            /*cleanup_callback=*/
            reinterpret_cast<Dart_IsolateCleanupCallback>(
                DartIsolateCleanupCallback),
            /*child_isolate_data=*/isolate_data,
            /*error=*/error);
      };
  Dart_Isolate platform_isolate = CreateDartIsolateGroup(
      nullptr, std::move(isolate_data), nullptr, error, isolate_maker);

  Dart_EnterIsolate(parent_isolate);

  if (*error) {
    return nullptr;
  }

  if (!platform_isolate_manager->RegisterPlatformIsolate(platform_isolate)) {
    // The PlatformIsolateManager was shutdown while we were creating the
    // isolate. This means that we're shutting down the engine. We need to
    // shutdown the platform isolate.
    FML_LOG(INFO) << "Shutdown during platform isolate creation";
    tonic::DartIsolateScope isolate_scope(platform_isolate);
    Dart_ShutdownIsolate();
    return nullptr;
  }

  tonic::DartApiScope api_scope;
  Dart_PersistentHandle entry_point_handle =
      Dart_NewPersistentHandle(entry_point);

  platform_task_runner->PostTask([entry_point_handle, platform_isolate,
                                  weak_platform_isolate_manager]() {
    std::shared_ptr<PlatformIsolateManager> platform_isolate_manager =
        weak_platform_isolate_manager.lock();
    if (platform_isolate_manager == nullptr ||
        platform_isolate_manager->HasShutdown()) {
      // Shutdown happened in between this task being posted, and it running.
      // platform_isolate has already been shut down. Do nothing.
      FML_LOG(INFO) << "Shutdown before platform isolate entry point";
      return;
    }

    tonic::DartIsolateScope isolate_scope(platform_isolate);
    tonic::DartApiScope api_scope;
    Dart_Handle entry_point = Dart_HandleFromPersistent(entry_point_handle);
    Dart_DeletePersistentHandle(entry_point_handle);

    // Disable Isolate.exit().
    Dart_Handle isolate_lib = Dart_LookupLibrary(tonic::ToDart("dart:isolate"));
    FML_CHECK(!tonic::CheckAndHandleError(isolate_lib));
    Dart_Handle isolate_type = Dart_GetNonNullableType(
        isolate_lib, tonic::ToDart("Isolate"), 0, nullptr);
    FML_CHECK(!tonic::CheckAndHandleError(isolate_type));
    Dart_Handle result =
        Dart_SetField(isolate_type, tonic::ToDart("_mayExit"), Dart_False());
    FML_CHECK(!tonic::CheckAndHandleError(result));

    tonic::DartInvokeVoid(entry_point);
  });

  return platform_isolate;
}

DartIsolate::DartIsolate(const Settings& settings,
                         bool is_root_isolate,
                         const UIDartState::Context& context,
                         bool is_spawning_in_group)
    : UIDartState(settings.task_observer_add,
                  settings.task_observer_remove,
                  settings.log_tag,
                  settings.unhandled_exception_callback,
                  settings.log_message_callback,
                  DartVMRef::GetIsolateNameServer(),
                  is_root_isolate,
                  context),
      may_insecurely_connect_to_all_domains_(
          settings.may_insecurely_connect_to_all_domains),
      is_platform_isolate_(false),
      is_spawning_in_group_(is_spawning_in_group),
      domain_network_policy_(settings.domain_network_policy) {
  phase_ = Phase::Uninitialized;
}

DartIsolate::DartIsolate(
    const Settings& settings,
    const UIDartState::Context& context,
    std::shared_ptr<PlatformIsolateManager> platform_isolate_manager)
    : UIDartState(settings.task_observer_add,
                  settings.task_observer_remove,
                  settings.log_tag,
                  settings.unhandled_exception_callback,
                  settings.log_message_callback,
                  DartVMRef::GetIsolateNameServer(),
                  false,  // is_root_isolate
                  context),
      may_insecurely_connect_to_all_domains_(
          settings.may_insecurely_connect_to_all_domains),
      is_platform_isolate_(true),
      is_spawning_in_group_(false),
      domain_network_policy_(settings.domain_network_policy),
      platform_isolate_manager_(std::move(platform_isolate_manager)) {
  phase_ = Phase::Uninitialized;
}

DartIsolate::~DartIsolate() {
  if (IsRootIsolate() && GetMessageHandlingTaskRunner()) {
    FML_DCHECK(GetMessageHandlingTaskRunner()->RunsTasksOnCurrentThread());
  }
}

DartIsolate::Phase DartIsolate::GetPhase() const {
  return phase_;
}

std::string DartIsolate::GetServiceId() {
  const char* service_id_buf = Dart_IsolateServiceId(isolate());
  std::string service_id(service_id_buf);
  free(const_cast<char*>(service_id_buf));
  return service_id;
}

bool DartIsolate::Initialize(Dart_Isolate dart_isolate) {
  TRACE_EVENT0("flutter", "DartIsolate::Initialize");
  if (phase_ != Phase::Uninitialized) {
    return false;
  }

  FML_DCHECK(dart_isolate != nullptr);
  FML_DCHECK(dart_isolate == Dart_CurrentIsolate());

  // After this point, isolate scopes can be safely used.
  SetIsolate(dart_isolate);

  // For the root isolate set the "AppStartUp" as soon as the root isolate
  // has been initialized. This is to ensure that all the timeline events
  // that have the set user-tag will be listed user AppStartUp.
  if (IsRootIsolate()) {
    tonic::DartApiScope api_scope;
    Dart_SetCurrentUserTag(Dart_NewUserTag("AppStartUp"));
  }

  if (is_platform_isolate_) {
    SetMessageHandlingTaskRunner(GetTaskRunners().GetPlatformTaskRunner(),
                                 true);
  } else {
    // When running with custom UI task runner post directly to runner (there is
    // no task queue).
    bool post_directly_to_runner =
        GetTaskRunners().GetUITaskRunner() &&
        !GetTaskRunners().GetUITaskRunner()->GetTaskQueueId().is_valid();
    SetMessageHandlingTaskRunner(GetTaskRunners().GetUITaskRunner(),
                                 post_directly_to_runner);
  }

  if (tonic::CheckAndHandleError(
          Dart_SetLibraryTagHandler(tonic::DartState::HandleLibraryTag))) {
    return false;
  }

  if (tonic::CheckAndHandleError(
          Dart_SetDeferredLoadHandler(OnDartLoadLibrary))) {
    return false;
  }

  if (!UpdateThreadPoolNames()) {
    return false;
  }

  phase_ = Phase::Initialized;
  return true;
}

fml::RefPtr<fml::TaskRunner> DartIsolate::GetMessageHandlingTaskRunner() const {
  return message_handling_task_runner_;
}

bool DartIsolate::LoadLoadingUnit(
    intptr_t loading_unit_id,
    std::unique_ptr<const fml::Mapping> snapshot_data,
    std::unique_ptr<const fml::Mapping> snapshot_instructions) {
  tonic::DartState::Scope scope(this);

  fml::RefPtr<DartSnapshot> dart_snapshot =
      DartSnapshot::IsolateSnapshotFromMappings(
          std::move(snapshot_data), std::move(snapshot_instructions));

  Dart_Handle result = Dart_DeferredLoadComplete(
      loading_unit_id, dart_snapshot->GetDataMapping(),
      dart_snapshot->GetInstructionsMapping());
  if (tonic::CheckAndHandleError(result)) {
    LoadLoadingUnitError(loading_unit_id, Dart_GetError(result),
                         /*transient*/ true);
    return false;
  }
  loading_unit_snapshots_.insert(dart_snapshot);
  return true;
}

void DartIsolate::LoadLoadingUnitError(intptr_t loading_unit_id,
                                       const std::string& error_message,
                                       bool transient) {
  tonic::DartState::Scope scope(this);
  Dart_Handle result = Dart_DeferredLoadCompleteError(
      loading_unit_id, error_message.c_str(), transient);
  tonic::CheckAndHandleError(result);
}

void DartIsolate::SetMessageHandlingTaskRunner(
    const fml::RefPtr<fml::TaskRunner>& runner,
    bool post_directly_to_runner) {
  if (!runner) {
    return;
  }

  message_handling_task_runner_ = runner;

  tonic::DartMessageHandler::TaskDispatcher dispatcher;

#ifdef OS_FUCHSIA
  post_directly_to_runner = true;
#endif

  if (post_directly_to_runner) {
    dispatcher = [runner](std::function<void()> task) {
      runner->PostTask([task = std::move(task)]() {
        TRACE_EVENT0("flutter", "DartIsolate::HandleMessage");
        task();
      });
    };
  } else {
    dispatcher = [runner](std::function<void()> task) {
      auto task_queues = fml::MessageLoopTaskQueues::GetInstance();
      task_queues->RegisterTask(
          runner->GetTaskQueueId(),
          [task = std::move(task)]() {
            TRACE_EVENT0("flutter", "DartIsolate::HandleMessage");
            task();
          },
          fml::TimePoint::Now(), fml::TaskSourceGrade::kDartEventLoop);
    };
  }

  message_handler().Initialize(dispatcher);
}

// Updating thread names here does not change the underlying OS thread names.
// Instead, this is just additional metadata for the Dart VM Service to show the
// thread name of the isolate.
bool DartIsolate::UpdateThreadPoolNames() const {
  // TODO(chinmaygarde): This implementation does not account for multiple
  // shells sharing the same (or subset of) threads.
  const auto& task_runners = GetTaskRunners();

  if (auto task_runner = task_runners.GetRasterTaskRunner()) {
    task_runner->PostTask(
        [label = task_runners.GetLabel() + std::string{".raster"}]() {
          Dart_SetThreadName(label.c_str());
        });
  }

  if (auto task_runner = task_runners.GetUITaskRunner()) {
    task_runner->PostTask(
        [label = task_runners.GetLabel() + std::string{".ui"}]() {
          Dart_SetThreadName(label.c_str());
        });
  }

  if (auto task_runner = task_runners.GetIOTaskRunner()) {
    task_runner->PostTask(
        [label = task_runners.GetLabel() + std::string{".io"}]() {
          Dart_SetThreadName(label.c_str());
        });
  }

  if (auto task_runner = task_runners.GetPlatformTaskRunner()) {
    bool is_merged_platform_ui_thread =
        task_runner == task_runners.GetUITaskRunner();
    std::string label;
    if (is_merged_platform_ui_thread) {
      label = task_runners.GetLabel() + std::string{".ui"};
    } else {
      label = task_runners.GetLabel() + std::string{".platform"};
    }
    task_runner->PostTask(
        [label = std::move(label)]() { Dart_SetThreadName(label.c_str()); });
  }

  return true;
}

bool DartIsolate::LoadLibraries() {
  TRACE_EVENT0("flutter", "DartIsolate::LoadLibraries");
  if (phase_ != Phase::Initialized) {
    return false;
  }

  tonic::DartState::Scope scope(this);

  DartIO::InitForIsolate(may_insecurely_connect_to_all_domains_,
                         domain_network_policy_);

  DartUI::InitForIsolate(GetIsolateGroupData().GetSettings());

  const bool is_service_isolate = Dart_IsServiceIsolate(isolate());

  DartRuntimeHooks::Install(IsRootIsolate() && !is_service_isolate,
                            GetAdvisoryScriptURI());

  if (!is_service_isolate) {
    class_library().add_provider(
        "ui", std::make_unique<tonic::DartClassProvider>(this, "dart:ui"));
  }

  phase_ = Phase::LibrariesSetup;
  return true;
}

bool DartIsolate::PrepareForRunningFromPrecompiledCode() {
  TRACE_EVENT0("flutter", "DartIsolate::PrepareForRunningFromPrecompiledCode");
  if (phase_ != Phase::LibrariesSetup) {
    return false;
  }

  tonic::DartState::Scope scope(this);

  if (Dart_IsNull(Dart_RootLibrary())) {
    return false;
  }

  if (!MarkIsolateRunnable()) {
    return false;
  }

  if (GetIsolateGroupData().GetChildIsolatePreparer() == nullptr) {
    GetIsolateGroupData().SetChildIsolatePreparer([](DartIsolate* isolate) {
      return isolate->PrepareForRunningFromPrecompiledCode();
    });
  }

  const fml::closure& isolate_create_callback =
      GetIsolateGroupData().GetIsolateCreateCallback();
  if (isolate_create_callback) {
    isolate_create_callback();
  }

  phase_ = Phase::Ready;
  return true;
}

bool DartIsolate::LoadKernel(const std::shared_ptr<const fml::Mapping>& mapping,
                             bool last_piece) {
  if (!Dart_IsKernel(mapping->GetMapping(), mapping->GetSize())) {
    return false;
  }

  // Mapping must be retained until isolate group shutdown.
  GetIsolateGroupData().AddKernelBuffer(mapping);

  Dart_Handle library =
      Dart_LoadLibraryFromKernel(mapping->GetMapping(), mapping->GetSize());
  if (tonic::CheckAndHandleError(library)) {
    return false;
  }

  if (!last_piece) {
    // More to come.
    return true;
  }

  Dart_SetRootLibrary(library);
  if (tonic::CheckAndHandleError(Dart_FinalizeLoading(false))) {
    return false;
  }
  return true;
}

[[nodiscard]] bool DartIsolate::PrepareForRunningFromKernel(
    const std::shared_ptr<const fml::Mapping>& mapping,
    bool child_isolate,
    bool last_piece) {
  TRACE_EVENT0("flutter", "DartIsolate::PrepareForRunningFromKernel");
  if (phase_ != Phase::LibrariesSetup) {
    return false;
  }

  if (DartVM::IsRunningPrecompiledCode()) {
    return false;
  }

  tonic::DartState::Scope scope(this);

  if (!child_isolate && !is_spawning_in_group_) {
    if (!mapping || mapping->GetSize() == 0) {
      return false;
    }

    // Use root library provided by kernel in favor of one provided by snapshot.
    Dart_SetRootLibrary(Dart_Null());

    if (!LoadKernel(mapping, last_piece)) {
      return false;
    }
  }

  if (!last_piece) {
    // More to come.
    return true;
  }

  if (Dart_IsNull(Dart_RootLibrary())) {
    return false;
  }

  if (!MarkIsolateRunnable()) {
    return false;
  }

  // Child isolate shares root isolate embedder_isolate (lines 691 and 693
  // below). Re-initializing child_isolate_preparer_ lambda while it is being
  // executed leads to crashes.
  if (GetIsolateGroupData().GetChildIsolatePreparer() == nullptr) {
    GetIsolateGroupData().SetChildIsolatePreparer(
        [buffers =
             GetIsolateGroupData().GetKernelBuffers()](DartIsolate* isolate) {
          for (uint64_t i = 0; i < buffers.size(); i++) {
            bool last_piece = i + 1 == buffers.size();
            const std::shared_ptr<const fml::Mapping>& buffer = buffers.at(i);
            if (!isolate->PrepareForRunningFromKernel(buffer,
                                                      /*child_isolate=*/true,
                                                      last_piece)) {
              return false;
            }
          }
          return true;
        });
  }

  const fml::closure& isolate_create_callback =
      GetIsolateGroupData().GetIsolateCreateCallback();
  if (isolate_create_callback) {
    isolate_create_callback();
  }

  phase_ = Phase::Ready;

  return true;
}

[[nodiscard]] bool DartIsolate::PrepareForRunningFromKernels(
    std::vector<std::shared_ptr<const fml::Mapping>> kernels) {
  const auto count = kernels.size();
  if (count == 0) {
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    bool last = (i == (count - 1));
    if (!PrepareForRunningFromKernel(kernels[i], /*child_isolate=*/false,
                                     last)) {
      return false;
    }
  }

  return true;
}

[[nodiscard]] bool DartIsolate::PrepareForRunningFromKernels(
    std::vector<std::unique_ptr<const fml::Mapping>> kernels) {
  std::vector<std::shared_ptr<const fml::Mapping>> shared_kernels;
  shared_kernels.reserve(kernels.size());
  for (auto& kernel : kernels) {
    shared_kernels.emplace_back(std::move(kernel));
  }
  return PrepareForRunningFromKernels(shared_kernels);
}

bool DartIsolate::MarkIsolateRunnable() {
  TRACE_EVENT0("flutter", "DartIsolate::MarkIsolateRunnable");
  if (phase_ != Phase::LibrariesSetup) {
    return false;
  }

  // This function may only be called from an active isolate scope.
  if (Dart_CurrentIsolate() != isolate()) {
    return false;
  }

  // There must be no current isolate to mark an isolate as being runnable.
  Dart_ExitIsolate();

  char* error = Dart_IsolateMakeRunnable(isolate());
  if (error) {
    FML_DLOG(ERROR) << error;
    ::free(error);
    // Failed. Restore the isolate.
    Dart_EnterIsolate(isolate());
    return false;
  }
  // Success. Restore the isolate.
  Dart_EnterIsolate(isolate());
  return true;
}

[[nodiscard]] static bool InvokeMainEntrypoint(
    Dart_Handle user_entrypoint_function,
    Dart_Handle args) {
  if (tonic::CheckAndHandleError(user_entrypoint_function)) {
    FML_LOG(ERROR) << "Could not resolve main entrypoint function.";
    return false;
  }

  Dart_Handle start_main_isolate_function =
      tonic::DartInvokeField(Dart_LookupLibrary(tonic::ToDart("dart:isolate")),
                             "_getStartMainIsolateFunction", {});

  if (tonic::CheckAndHandleError(start_main_isolate_function)) {
    FML_LOG(ERROR) << "Could not resolve main entrypoint trampoline.";
    return false;
  }

  if (tonic::CheckAndHandleError(tonic::DartInvokeField(
          Dart_LookupLibrary(tonic::ToDart("dart:ui")), "_runMain",
          {start_main_isolate_function, user_entrypoint_function, args}))) {
    FML_LOG(ERROR) << "Could not invoke the main entrypoint.";
    return false;
  }

  return true;
}

bool DartIsolate::RunFromLibrary(std::optional<std::string> library_name,
                                 std::optional<std::string> entrypoint,
                                 const std::vector<std::string>& args) {
  TRACE_EVENT0("flutter", "DartIsolate::RunFromLibrary");
  if (phase_ != Phase::Ready) {
    return false;
  }

  tonic::DartState::Scope scope(this);

  auto library_handle =
      library_name.has_value() && !library_name.value().empty()
          ? ::Dart_LookupLibrary(tonic::ToDart(library_name.value().c_str()))
          : ::Dart_RootLibrary();
  auto entrypoint_handle = entrypoint.has_value() && !entrypoint.value().empty()
                               ? tonic::ToDart(entrypoint.value().c_str())
                               : tonic::ToDart("main");

  if (!FindAndInvokeDartPluginRegistrant()) {
    // TODO(gaaclarke): Remove once the framework PR lands that uses `--source`
    // to compile the Dart Plugin Registrant
    // (https://github.com/flutter/flutter/pull/100572).
    InvokeDartPluginRegistrantIfAvailable(library_handle);
  }

  auto user_entrypoint_function =
      ::Dart_GetField(library_handle, entrypoint_handle);

  auto entrypoint_args = tonic::ToDart(args);

  if (!InvokeMainEntrypoint(user_entrypoint_function, entrypoint_args)) {
    return false;
  }

  phase_ = Phase::Running;

  return true;
}

bool DartIsolate::Shutdown() {
  TRACE_EVENT0("flutter", "DartIsolate::Shutdown");
  // This call may be re-entrant since Dart_ShutdownIsolate can invoke the
  // cleanup callback which deletes the embedder side object of the dart isolate
  // (a.k.a. this).
  if (phase_ == Phase::Shutdown) {
    return false;
  }
  phase_ = Phase::Shutdown;
  Dart_Isolate vm_isolate = isolate();
  // The isolate can be nullptr if this instance is the stub isolate data used
  // during root isolate creation.
  if (vm_isolate != nullptr) {
    // We need to enter the isolate because Dart_ShutdownIsolate does not take
    // the isolate to shutdown as a parameter.
    FML_DCHECK(Dart_CurrentIsolate() == nullptr);
    Dart_EnterIsolate(vm_isolate);
    Dart_ShutdownIsolate();
    FML_DCHECK(Dart_CurrentIsolate() == nullptr);
  }
  return true;
}

Dart_Isolate DartIsolate::DartCreateAndStartServiceIsolate(
    const char* package_root,
    const char* package_config,
    Dart_IsolateFlags* flags,
    char** error) {
  auto vm_data = DartVMRef::GetVMData();

  if (!vm_data) {
    *error = fml::strdup(
        "Could not access VM data to initialize isolates. This may be because "
        "the VM has initialized shutdown on another thread already.");
    return nullptr;
  }

  const auto& settings = vm_data->GetSettings();

  if (!settings.enable_vm_service) {
    return nullptr;
  }

  flags->load_vmservice_library = true;

#if (FLUTTER_RUNTIME_MODE != FLUTTER_RUNTIME_MODE_DEBUG)
  // TODO(68663): The service isolate in debug mode is always launched without
  // sound null safety. Fix after the isolate snapshot data is created with the
  // right flags.
  flags->null_safety = vm_data->GetServiceIsolateSnapshotNullSafety();
#endif

  UIDartState::Context context(
      TaskRunners("io.flutter." DART_VM_SERVICE_ISOLATE_NAME, nullptr, nullptr,
                  nullptr, nullptr));
  context.advisory_script_uri = DART_VM_SERVICE_ISOLATE_NAME;
  context.advisory_script_entrypoint = DART_VM_SERVICE_ISOLATE_NAME;
  std::weak_ptr<DartIsolate> weak_service_isolate =
      DartIsolate::CreateRootIsolate(vm_data->GetSettings(),                //
                                     vm_data->GetServiceIsolateSnapshot(),  //
                                     nullptr,                               //
                                     DartIsolate::Flags{flags},             //
                                     nullptr,                               //
                                     nullptr,                               //
                                     context);                              //

  std::shared_ptr<DartIsolate> service_isolate = weak_service_isolate.lock();
  if (!service_isolate) {
    *error = fml::strdup("Could not create the service isolate.");
    FML_DLOG(ERROR) << *error;
    return nullptr;
  }

  tonic::DartState::Scope scope(service_isolate);
  if (!DartServiceIsolate::Startup(
          settings.vm_service_host,            // server IP address
          settings.vm_service_port,            // server VM service port
          tonic::DartState::HandleLibraryTag,  // embedder library tag handler
          false,  //  disable websocket origin check
          settings.disable_service_auth_codes,  // disable VM service auth codes
          settings.enable_service_port_fallback,  // enable fallback to port 0
                                                  // when bind fails.
          error                                   // error (out)
          )) {
    // Error is populated by call to startup.
    FML_DLOG(ERROR) << *error;
    return nullptr;
  }

  if (auto callback = vm_data->GetSettings().service_isolate_create_callback) {
    callback();
  }

  if (auto service_protocol = DartVMRef::GetServiceProtocol()) {
    service_protocol->ToggleHooks(true);
  } else {
    FML_DLOG(ERROR)
        << "Could not acquire the service protocol handlers. This might be "
           "because the VM has already begun teardown on another thread.";
  }

  return service_isolate->isolate();
}

DartIsolateGroupData& DartIsolate::GetIsolateGroupData() {
  std::shared_ptr<DartIsolateGroupData>* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_IsolateGroupData(isolate()));
  return **isolate_group_data;
}

const DartIsolateGroupData& DartIsolate::GetIsolateGroupData() const {
  DartIsolate* non_const_this = const_cast<DartIsolate*>(this);
  return non_const_this->GetIsolateGroupData();
}

// |Dart_IsolateGroupCreateCallback|
Dart_Isolate DartIsolate::DartIsolateGroupCreateCallback(
    const char* advisory_script_uri,
    const char* advisory_script_entrypoint,
    const char* package_root,
    const char* package_config,
    Dart_IsolateFlags* flags,
    std::shared_ptr<DartIsolate>* parent_isolate_data,
    char** error) {
  TRACE_EVENT0("flutter", "DartIsolate::DartIsolateGroupCreateCallback");
  if (parent_isolate_data == nullptr &&
      strcmp(advisory_script_uri, DART_VM_SERVICE_ISOLATE_NAME) == 0) {
    // The VM attempts to start the VM service for us on |Dart_Initialize|. In
    // such a case, the callback data will be null and the script URI will be
    // DART_VM_SERVICE_ISOLATE_NAME. In such cases, we just create the service
    // isolate like normal but dont hold a reference to it at all. We also start
    // this isolate since we will never again reference it from the engine.
    return DartCreateAndStartServiceIsolate(package_root,    //
                                            package_config,  //
                                            flags,           //
                                            error            //
    );
  }

  if (!parent_isolate_data) {
    return nullptr;
  }

  DartIsolateGroupData& parent_group_data =
      (*parent_isolate_data)->GetIsolateGroupData();

  if (strncmp(advisory_script_uri, kFileUriPrefix.data(),
              kFileUriPrefix.size())) {
    std::string error_msg =
        std::string("Unsupported isolate URI: ") + advisory_script_uri;
    *error = fml::strdup(error_msg.c_str());
    return nullptr;
  }

  auto isolate_group_data =
      std::make_unique<std::shared_ptr<DartIsolateGroupData>>(
          std::shared_ptr<DartIsolateGroupData>(new DartIsolateGroupData(
              parent_group_data.GetSettings(),
              parent_group_data.GetIsolateSnapshot(), advisory_script_uri,
              advisory_script_entrypoint,
              parent_group_data.GetChildIsolatePreparer(),
              parent_group_data.GetIsolateCreateCallback(),
              parent_group_data.GetIsolateShutdownCallback())));

  TaskRunners null_task_runners(advisory_script_uri,
                                /* platform= */ nullptr,
                                /* raster= */ nullptr,
                                /* ui= */ nullptr,
                                /* io= */ nullptr);

  UIDartState::Context context(null_task_runners);
  context.advisory_script_uri = advisory_script_uri;
  context.advisory_script_entrypoint = advisory_script_entrypoint;
  auto isolate_data = std::make_unique<std::shared_ptr<DartIsolate>>(
      std::shared_ptr<DartIsolate>(
          new DartIsolate((*isolate_group_data)->GetSettings(),  // settings
                          false,       // is_root_isolate
                          context)));  // context

  Dart_Isolate vm_isolate = CreateDartIsolateGroup(
      std::move(isolate_group_data), std::move(isolate_data), flags, error,
      [](std::shared_ptr<DartIsolateGroupData>* isolate_group_data,
         std::shared_ptr<DartIsolate>* isolate_data, Dart_IsolateFlags* flags,
         char** error) {
        return Dart_CreateIsolateGroup(
            (*isolate_group_data)->GetAdvisoryScriptURI().c_str(),
            (*isolate_group_data)->GetAdvisoryScriptEntrypoint().c_str(),
            (*isolate_group_data)->GetIsolateSnapshot()->GetDataMapping(),
            (*isolate_group_data)
                ->GetIsolateSnapshot()
                ->GetInstructionsMapping(),
            flags, isolate_group_data, isolate_data, error);
      });

  if (*error) {
    FML_LOG(ERROR) << "CreateDartIsolateGroup failed: " << *error;
  }

  return vm_isolate;
}

// |Dart_IsolateInitializeCallback|
bool DartIsolate::DartIsolateInitializeCallback(void** child_callback_data,
                                                char** error) {
  TRACE_EVENT0("flutter", "DartIsolate::DartIsolateInitializeCallback");
  Dart_Isolate isolate = Dart_CurrentIsolate();
  if (isolate == nullptr) {
    *error = fml::strdup("Isolate should be available in initialize callback.");
    FML_DLOG(ERROR) << *error;
    return false;
  }

  auto* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_CurrentIsolateGroupData());

  TaskRunners null_task_runners((*isolate_group_data)->GetAdvisoryScriptURI(),
                                /* platform= */ nullptr,
                                /* raster= */ nullptr,
                                /* ui= */ nullptr,
                                /* io= */ nullptr);

  UIDartState::Context context(null_task_runners);
  context.advisory_script_uri = (*isolate_group_data)->GetAdvisoryScriptURI();
  context.advisory_script_entrypoint =
      (*isolate_group_data)->GetAdvisoryScriptEntrypoint();
  auto embedder_isolate = std::make_unique<std::shared_ptr<DartIsolate>>(
      std::shared_ptr<DartIsolate>(
          new DartIsolate((*isolate_group_data)->GetSettings(),  // settings
                          false,       // is_root_isolate
                          context)));  // context

  // root isolate should have been created via CreateRootIsolate
  if (!InitializeIsolate(*embedder_isolate, isolate, error)) {
    return false;
  }

  // The ownership of the embedder object is controlled by the Dart VM. So the
  // only reference returned to the caller is weak.
  *child_callback_data = embedder_isolate.release();

  return true;
}

static void* NativeAssetsDlopenRelative(const char* path, char** error) {
  auto* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_CurrentIsolateGroupData());
  const std::string& script_uri = (*isolate_group_data)->GetAdvisoryScriptURI();
  return dart::bin::NativeAssets::DlopenRelative(path, script_uri.data(),
                                                 error);
}

static void* NativeAssetsDlopen(const char* asset_id, char** error) {
  auto* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_CurrentIsolateGroupData());
  auto native_assets_manager = (*isolate_group_data)->GetNativeAssetsManager();
  if (native_assets_manager == nullptr) {
    return nullptr;
  }

  std::vector<std::string> asset_path =
      native_assets_manager->LookupNativeAsset(asset_id);
  if (asset_path.size() == 0) {
    // The asset id was not in the mapping.
    return nullptr;
  }

  auto& path_type = asset_path[0];
  std::string path;
  static constexpr const char* kAbsolute = "absolute";
  static constexpr const char* kExecutable = "executable";
  static constexpr const char* kProcess = "process";
  static constexpr const char* kRelative = "relative";
  static constexpr const char* kSystem = "system";
  if (path_type == kAbsolute || path_type == kRelative ||
      path_type == kSystem) {
    path = asset_path[1];
  }

  if (path_type == kAbsolute) {
    return dart::bin::NativeAssets::DlopenAbsolute(path.c_str(), error);
  } else if (path_type == kRelative) {
    return NativeAssetsDlopenRelative(path.c_str(), error);
  } else if (path_type == kSystem) {
    return dart::bin::NativeAssets::DlopenSystem(path.c_str(), error);
  } else if (path_type == kProcess) {
    return dart::bin::NativeAssets::DlopenProcess(error);
  } else if (path_type == kExecutable) {
    return dart::bin::NativeAssets::DlopenExecutable(error);
  }

  return nullptr;
}

static char* NativeAssetsAvailableAssets() {
  auto* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_CurrentIsolateGroupData());
  auto native_assets_manager = (*isolate_group_data)->GetNativeAssetsManager();
  FML_DCHECK(native_assets_manager != nullptr);
  auto available_assets = native_assets_manager->AvailableNativeAssets();
  auto* result = fml::strdup(available_assets.c_str());
  return result;
}

static void InitDartFFIForIsolateGroup() {
  NativeAssetsApi native_assets;
  memset(&native_assets, 0, sizeof(native_assets));
  // TODO(dacoharkes): Remove after flutter_tools stops kernel embedding.
  native_assets.dlopen_absolute = &dart::bin::NativeAssets::DlopenAbsolute;
  native_assets.dlopen_relative = &NativeAssetsDlopenRelative;
  native_assets.dlopen_system = &dart::bin::NativeAssets::DlopenSystem;
  native_assets.dlopen_executable = &dart::bin::NativeAssets::DlopenExecutable;
  native_assets.dlopen_process = &dart::bin::NativeAssets::DlopenProcess;
  // TODO(dacoharkes): End todo.
  native_assets.dlsym = &dart::bin::NativeAssets::Dlsym;
  native_assets.dlopen = &NativeAssetsDlopen;
  native_assets.available_assets = &NativeAssetsAvailableAssets;
  Dart_InitializeNativeAssetsResolver(&native_assets);
};

Dart_Isolate DartIsolate::CreateDartIsolateGroup(
    std::unique_ptr<std::shared_ptr<DartIsolateGroupData>> isolate_group_data,
    std::unique_ptr<std::shared_ptr<DartIsolate>> isolate_data,
    Dart_IsolateFlags* flags,
    char** error,
    const DartIsolate::IsolateMaker& make_isolate) {
  TRACE_EVENT0("flutter", "DartIsolate::CreateDartIsolateGroup");

  // Create the Dart VM isolate and give it the embedder object as the baton.
  Dart_Isolate isolate =
      make_isolate(isolate_group_data.get(), isolate_data.get(), flags, error);

  if (isolate == nullptr) {
    return nullptr;
  }

  bool success = false;
  {
    // Ownership of the isolate data objects has been transferred to the Dart
    // VM.
    // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks)
    std::shared_ptr<DartIsolate> embedder_isolate(*isolate_data);
    isolate_group_data.release();
    isolate_data.release();
    // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)

    InitDartFFIForIsolateGroup();

    success = InitializeIsolate(embedder_isolate, isolate, error);
  }
  if (!success) {
    Dart_ShutdownIsolate();
    return nullptr;
  }

  // Balances the implicit [Dart_EnterIsolate] by [make_isolate] above.
  Dart_ExitIsolate();
  return isolate;
}

bool DartIsolate::InitializeIsolate(
    const std::shared_ptr<DartIsolate>& embedder_isolate,
    Dart_Isolate isolate,
    char** error) {
  TRACE_EVENT0("flutter", "DartIsolate::InitializeIsolate");
  if (!embedder_isolate->Initialize(isolate)) {
    *error = fml::strdup("Embedder could not initialize the Dart isolate.");
    FML_DLOG(ERROR) << *error;
    return false;
  }

  if (!embedder_isolate->LoadLibraries()) {
    *error = fml::strdup(
        "Embedder could not load libraries in the new Dart isolate.");
    FML_DLOG(ERROR) << *error;
    return false;
  }

  // Root isolates will be set up by the engine and the service isolate
  // (which is also a root isolate) by the utility routines in the VM.
  // However, secondary isolates will be run by the VM if they are
  // marked as runnable.
  if (!embedder_isolate->IsRootIsolate()) {
    auto child_isolate_preparer =
        embedder_isolate->GetIsolateGroupData().GetChildIsolatePreparer();
    FML_DCHECK(child_isolate_preparer);
    if (!child_isolate_preparer(embedder_isolate.get())) {
      *error = fml::strdup("Could not prepare the child isolate to run.");
      FML_DLOG(ERROR) << *error;
      return false;
    }
  }

  return true;
}

// |Dart_IsolateShutdownCallback|
void DartIsolate::DartIsolateShutdownCallback(
    std::shared_ptr<DartIsolateGroupData>* isolate_group_data,
    std::shared_ptr<DartIsolate>* isolate_data) {
  TRACE_EVENT0("flutter", "DartIsolate::DartIsolateShutdownCallback");

  // If the isolate initialization failed there will be nothing to do.
  // This can happen e.g. during a [DartIsolateInitializeCallback] invocation
  // that fails to initialize the VM-created isolate.
  if (isolate_data == nullptr) {
    return;
  }

  isolate_data->get()->OnShutdownCallback();
}

// |Dart_IsolateGroupCleanupCallback|
void DartIsolate::DartIsolateGroupCleanupCallback(
    std::shared_ptr<DartIsolateGroupData>* isolate_data) {
  TRACE_EVENT0("flutter", "DartIsolate::DartIsolateGroupCleanupCallback");
  delete isolate_data;
}

// |Dart_IsolateCleanupCallback|
void DartIsolate::DartIsolateCleanupCallback(
    std::shared_ptr<DartIsolateGroupData>* isolate_group_data,
    std::shared_ptr<DartIsolate>* isolate_data) {
  TRACE_EVENT0("flutter", "DartIsolate::DartIsolateCleanupCallback");
  delete isolate_data;
}

std::weak_ptr<DartIsolate> DartIsolate::GetWeakIsolatePtr() {
  return std::static_pointer_cast<DartIsolate>(shared_from_this());
}

void DartIsolate::AddIsolateShutdownCallback(const fml::closure& closure) {
  shutdown_callbacks_.emplace_back(std::make_unique<AutoFireClosure>(closure));
}

void DartIsolate::OnShutdownCallback() {
  tonic::DartState* state = tonic::DartState::Current();
  if (state != nullptr) {
    state->SetIsShuttingDown();
  }

  {
    tonic::DartApiScope api_scope;
    Dart_Handle sticky_error = Dart_GetStickyError();
    if (!Dart_IsNull(sticky_error) && !Dart_IsFatalError(sticky_error)) {
      FML_LOG(ERROR) << Dart_GetError(sticky_error);
    }
  }

  if (is_platform_isolate_) {
    FML_DCHECK(platform_isolate_manager_ != nullptr);
    platform_isolate_manager_->RemovePlatformIsolate(isolate());
  }

  shutdown_callbacks_.clear();

  const fml::closure& isolate_shutdown_callback =
      GetIsolateGroupData().GetIsolateShutdownCallback();
  if (isolate_shutdown_callback) {
    isolate_shutdown_callback();
  }
}

Dart_Handle DartIsolate::OnDartLoadLibrary(intptr_t loading_unit_id) {
  if (Current()->platform_configuration()) {
    Current()->platform_configuration()->client()->RequestDartDeferredLibrary(
        loading_unit_id);
    return Dart_Null();
  }
  const std::string error_message =
      "Platform Configuration was null. Deferred library load request "
      "for loading unit id " +
      std::to_string(loading_unit_id) + " was not sent.";
  FML_LOG(ERROR) << error_message;
  return Dart_NewApiError(error_message.c_str());
}

Dart_Handle DartIsolate::LoadLibraryFromKernel(
    const std::shared_ptr<const fml::Mapping>& mapping) {
  if (DartVM::IsRunningPrecompiledCode()) {
    return Dart_Null();
  }

  auto* isolate_group_data =
      static_cast<std::shared_ptr<DartIsolateGroupData>*>(
          Dart_CurrentIsolateGroupData());
  // Mapping must be retained until isolate shutdown.
  (*isolate_group_data)->AddKernelBuffer(mapping);

  auto lib =
      Dart_LoadLibraryFromKernel(mapping->GetMapping(), mapping->GetSize());
  if (tonic::CheckAndHandleError(lib)) {
    return Dart_Null();
  }
  auto result = Dart_FinalizeLoading(false);
  if (Dart_IsError(result)) {
    return result;
  }
  return Dart_GetField(lib, Dart_NewStringFromCString("main"));
}

DartIsolate::AutoFireClosure::AutoFireClosure(const fml::closure& closure)
    : closure_(closure) {}

DartIsolate::AutoFireClosure::~AutoFireClosure() {
  if (closure_) {
    closure_();
  }
}

}  // namespace flutter
