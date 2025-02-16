// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dart_runner.h"

#include <lib/async-loop/loop.h>
#include <lib/async/default.h>
#include <lib/vfs/cpp/pseudo_dir.h>
#include <sys/stat.h>
#include <zircon/status.h>
#include <zircon/syscalls.h>

#include <cerrno>
#include <memory>
#include <thread>
#include <utility>

#include "dart_component_controller.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "runtime/dart/utils/inlines.h"
#include "runtime/dart/utils/vmservice_object.h"
#include "service_isolate.h"
#include "third_party/dart/runtime/include/bin/dart_io_api.h"
#include "third_party/tonic/dart_microtask_queue.h"
#include "third_party/tonic/dart_state.h"

#if defined(AOT_RUNTIME)
extern "C" uint8_t _kDartVmSnapshotData[];
extern "C" uint8_t _kDartVmSnapshotInstructions[];
#endif

namespace dart_runner {

namespace {

const char* kDartVMArgs[] = {
    // clang-format off

    "--timeline_recorder=systrace",
    "--timeline_streams=Compiler,Dart,Debugger,Embedder,GC,Isolate,VM",

#if defined(AOT_RUNTIME)
    "--precompilation",
#else
    "--enable_mirrors=false",
#endif

    // No asserts in debug or release product.
    // No asserts in release with flutter_profile=true (non-product)
    // Yes asserts in non-product debug.
#if !defined(DART_PRODUCT) && (!defined(FLUTTER_PROFILE) || !defined(NDEBUG))
    "--enable_asserts",
#endif
};

Dart_Isolate IsolateGroupCreateCallback(const char* uri,
                                        const char* name,
                                        const char* package_root,
                                        const char* package_config,
                                        Dart_IsolateFlags* flags,
                                        void* callback_data,
                                        char** error) {
  if (std::string(uri) == DART_VM_SERVICE_ISOLATE_NAME) {
#if defined(DART_PRODUCT)
    *error = strdup("The service isolate is not implemented in product mode");
    return NULL;
#else
    return CreateServiceIsolate(uri, flags, error);
#endif
  }

  *error = strdup("Isolate spawning is not implemented in dart_runner");
  return NULL;
}

void IsolateShutdownCallback(void* isolate_group_data, void* isolate_data) {
  // The service isolate (and maybe later the kernel isolate) doesn't have an
  // async loop.
  auto dispatcher = async_get_default_dispatcher();
  auto loop = async_loop_from_dispatcher(dispatcher);
  if (loop) {
    tonic::DartMicrotaskQueue* queue =
        tonic::DartMicrotaskQueue::GetForCurrentThread();
    if (queue) {
      queue->Destroy();
    }

    async_loop_quit(loop);
  }

  auto state =
      static_cast<std::shared_ptr<tonic::DartState>*>(isolate_group_data);
  state->get()->SetIsShuttingDown();
}

void IsolateGroupCleanupCallback(void* isolate_group_data) {
  delete static_cast<std::shared_ptr<tonic::DartState>*>(isolate_group_data);
}

// Runs the application for a Dart component.
void RunApplication(
    DartRunner* runner,
    fuchsia::component::runner::ComponentStartInfo start_info,
    std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        controller) {
  const int64_t start = Dart_TimelineGetMicros();

  DartComponentController app(std::move(start_info), runner_incoming_services,
                              std::move(controller));
  const bool success = app.SetUp();

  const int64_t end = Dart_TimelineGetMicros();
  Dart_RecordTimelineEvent(
      "DartComponentController::SetUp", start, end, 0, nullptr,
      Dart_Timeline_Event_Duration, 0, NULL, NULL);
  if (success) {
    app.Run();
  }

  if (Dart_CurrentIsolate()) {
    Dart_ShutdownIsolate();
  }
}

void RunTestApplication(
    DartRunner* runner,
    fuchsia::component::runner::ComponentStartInfo start_info,
    std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        controller,
    fit::function<void(std::shared_ptr<DartTestComponentController>)>
        component_created_callback,
    fit::function<void(DartTestComponentController*)> done_callback) {
  const int64_t start = Dart_TimelineGetMicros();

  auto test_component = std::make_shared<DartTestComponentController>(
      std::move(start_info), runner_incoming_services, std::move(controller),
      std::move(done_callback));

  component_created_callback(test_component);

  // Start up the dart isolate and serve the suite protocol.
  test_component->SetUp();

  const int64_t end = Dart_TimelineGetMicros();
  Dart_RecordTimelineEvent(
      "DartTestComponentController::SetUp", start, end, 0, nullptr,
      Dart_Timeline_Event_Duration, 0, NULL, NULL);
}

bool EntropySource(uint8_t* buffer, intptr_t count) {
  zx_cprng_draw(buffer, count);
  return true;
}

}  // namespace

// "args" are how the component specifies arguments to the runner.
constexpr char kArgsKey[] = "args";

/// Parses the |args| field from the "program" field to determine
/// if a test component is being executed.
bool IsTestProgram(const fuchsia::data::Dictionary& program_metadata) {
  for (const auto& entry : program_metadata.entries()) {
    if (entry.key.compare(kArgsKey) != 0 || entry.value == nullptr) {
      continue;
    }
    auto args = entry.value->str_vec();

    // fml::CommandLine expects the first argument to be the name of the
    // program, so we prepend a dummy argument so we can use fml::CommandLine to
    // parse the arguments for us.
    std::vector<std::string> command_line_args = {""};
    command_line_args.insert(command_line_args.end(), args.begin(), args.end());
    fml::CommandLine parsed_args = fml::CommandLineFromIterators(
        command_line_args.begin(), command_line_args.end());

    std::string is_test_str;
    return parsed_args.GetOptionValue("is_test", &is_test_str) &&
           is_test_str == "true";
  }
  return false;
}

DartRunner::DartRunner(sys::ComponentContext* context) : context_(context) {
  context_->outgoing()
      ->AddPublicService<fuchsia::component::runner::ComponentRunner>(
          [this](fidl::InterfaceRequest<
                 fuchsia::component::runner::ComponentRunner> request) {
            component_runner_bindings_.AddBinding(this, std::move(request));
          });

#if !defined(DART_PRODUCT)
  // The VM service isolate uses the process-wide namespace. It writes the
  // vm service protocol port under /tmp. The VMServiceObject exposes that
  // port number to The Hub.
  context_->outgoing()->debug_dir()->AddEntry(
      dart_utils::VMServiceObject::kPortDirName,
      std::make_unique<dart_utils::VMServiceObject>());

#endif  // !defined(DART_PRODUCT)

  dart::bin::BootstrapDartIo();

  char* error =
      Dart_SetVMFlags(dart_utils::ArraySize(kDartVMArgs), kDartVMArgs);
  if (error) {
    FML_LOG(FATAL) << "Dart_SetVMFlags failed: " << error;
  }

  Dart_InitializeParams params = {};
  params.version = DART_INITIALIZE_PARAMS_CURRENT_VERSION;
#if defined(AOT_RUNTIME)
  params.vm_snapshot_data = ::_kDartVmSnapshotData;
  params.vm_snapshot_instructions = ::_kDartVmSnapshotInstructions;
#else
  if (!dart_utils::MappedResource::LoadFromNamespace(
          nullptr, "/pkg/data/vm_snapshot_data.bin", vm_snapshot_data_)) {
    FML_LOG(FATAL) << "Failed to load vm snapshot data";
  }
  params.vm_snapshot_data = vm_snapshot_data_.address();
#endif
  params.create_group = IsolateGroupCreateCallback;
  params.shutdown_isolate = IsolateShutdownCallback;
  params.cleanup_group = IsolateGroupCleanupCallback;
  params.entropy_source = EntropySource;
#if !defined(DART_PRODUCT)
  params.get_service_assets = GetVMServiceAssetsArchiveCallback;
#endif
  error = Dart_Initialize(&params);
  if (error)
    FML_LOG(FATAL) << "Dart_Initialize failed: " << error;
}

DartRunner::~DartRunner() {
  char* error = Dart_Cleanup();
  if (error)
    FML_LOG(FATAL) << "Dart_Cleanup failed: " << error;
}

void DartRunner::Start(
    fuchsia::component::runner::ComponentStartInfo start_info,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        controller) {
  // Parse the program field of the component's cml and check if it is a test
  // component. If so, serve the |fuchsia.test.Suite| protocol from the
  // component's outgoing directory, via DartTestComponentController.
  if (IsTestProgram(start_info.program())) {
    std::string url_copy = start_info.resolved_url();
    TRACE_EVENT1("dart", "Start", "url", url_copy.c_str());
    std::thread thread(
        RunTestApplication, this, std::move(start_info), context_->svc(),
        std::move(controller),
        // component_created_callback
        [this](std::shared_ptr<DartTestComponentController> ptr) {
          test_components_.emplace(ptr.get(), std::move(ptr));
        },
        // done_callback
        [this](DartTestComponentController* ptr) {
          auto it = test_components_.find(ptr);
          if (it != test_components_.end()) {
            test_components_.erase(it);
          }
        });
    thread.detach();
  } else {
    std::string url_copy = start_info.resolved_url();
    TRACE_EVENT1("dart", "Start", "url", url_copy.c_str());
    std::thread thread(RunApplication, this, std::move(start_info),
                       context_->svc(), std::move(controller));
    thread.detach();
  }
}

void DartRunner::handle_unknown_method(uint64_t ordinal,
                                       bool method_has_response) {
  FML_LOG(ERROR) << "Unknown method called on DartRunner. Ordinal: "
                 << ordinal;
}

}  // namespace dart_runner
