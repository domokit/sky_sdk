// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "runner.h"

#include <fcntl.h>
#include <fuchsia/mem/cpp/fidl.h>
#include <lib/async/cpp/task.h>
#include <lib/async/default.h>
#include <lib/inspect/cpp/inspect.h>
#include <lib/sys/cpp/component_context.h>
#include <lib/trace-engine/instrumentation.h>
#include <lib/vfs/cpp/pseudo_dir.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <cstdint>
#include <sstream>
#include <utility>

#include "flutter/fml/make_copyable.h"
#include "flutter/lib/ui/text/font_collection.h"
#include "flutter/runtime/dart_vm.h"
#include "runtime/dart/utils/files.h"
#include "runtime/dart/utils/root_inspect_node.h"
#include "runtime/dart/utils/vmo.h"
#include "runtime/dart/utils/vmservice_object.h"
#include "third_party/icu/source/common/unicode/udata.h"
#include "third_party/skia/include/core/SkGraphics.h"

namespace flutter_runner {

namespace {

static constexpr char kIcuDataPath[] = "/pkg/data/icudtl.dat";

// Environment variable containing the path to the directory containing the
// timezone files.
static constexpr char kICUTZEnv[] = "ICU_TIMEZONE_FILES_DIR";

// The data directory containing ICU timezone data files.
static constexpr char kICUTZDataDir[] = "/config/data/tzdata/icu/44/le";

// Map the memory into the process and return a pointer to the memory.
uintptr_t GetICUData(const fuchsia::mem::Buffer& icu_data) {
  uint64_t data_size = icu_data.size;
  if (data_size > std::numeric_limits<size_t>::max())
    return 0u;

  uintptr_t data = 0u;
  zx_status_t status =
      zx::vmar::root_self()->map(ZX_VM_PERM_READ, 0, icu_data.vmo, 0,
                                 static_cast<size_t>(data_size), &data);
  if (status == ZX_OK) {
    return data;
  }

  return 0u;
}

// Initializes the timezone data if available.  Timezone data file in Fuchsia
// is at a fixed directory path.  Returns true on success.  As a side effect
// sets the value of the environment variable "ICU_TIMEZONE_FILES_DIR" to a
// fixed value which is fuchsia-specific.
bool InitializeTZData() {
  // We need the ability to change the env variable for testing, so not
  // overwriting if set.
  setenv(kICUTZEnv, kICUTZDataDir, 0 /* No overwrite */);

  const std::string tzdata_dir = getenv(kICUTZEnv);
  // Try opening the path to check if present.  No need to verify that it is a
  // directory since ICU loading will return an error if the TZ data path is
  // wrong.
  int fd = openat(AT_FDCWD, tzdata_dir.c_str(), O_RDONLY);
  if (fd < 0) {
    FML_LOG(INFO) << "Could not open: '" << tzdata_dir
                  << "', proceeding without loading the timezone database: "
                  << strerror(errno);
    return false;
  }
  if (close(fd)) {
    FML_LOG(WARNING) << "Could not close: " << tzdata_dir << ": "
                     << strerror(errno);
  }
  return true;
}

// Return value indicates if initialization was successful.
bool InitializeICU() {
  const char* data_path = kIcuDataPath;

  fuchsia::mem::Buffer icu_data;
  if (!dart_utils::VmoFromFilename(data_path, false, &icu_data)) {
    return false;
  }

  uintptr_t data = GetICUData(icu_data);
  if (!data) {
    return false;
  }

  // If the loading fails, soldier on.  The loading is optional as we don't
  // want to crash the engine in transition.
  InitializeTZData();

  // Pass the data to ICU.
  UErrorCode err = U_ZERO_ERROR;
  udata_setCommonData(reinterpret_cast<const char*>(data), &err);
  if (err != U_ZERO_ERROR) {
    FML_LOG(ERROR) << "error loading ICU data: " << err;
    return false;
  }
  return true;
}

}  // namespace

static void SetProcessName() {
  std::stringstream stream;
#if defined(DART_PRODUCT)
  stream << "io.flutter.product_runner.";
#else
  stream << "io.flutter.runner.";
#endif
  if (flutter::DartVM::IsRunningPrecompiledCode()) {
    stream << "aot";
  } else {
    stream << "jit";
  }
  const auto name = stream.str();
  zx::process::self()->set_property(ZX_PROP_NAME, name.c_str(), name.size());
}

static void SetThreadName(const std::string& thread_name) {
  zx::thread::self()->set_property(ZX_PROP_NAME, thread_name.c_str(),
                                   thread_name.size());
}

#if !defined(DART_PRODUCT)
// Register native symbol information for the Dart VM's profiler.
static void RegisterProfilerSymbols(const char* symbols_path,
                                    const char* dso_name) {
  std::string* symbols = new std::string();
  if (dart_utils::ReadFileToString(symbols_path, symbols)) {
    Dart_AddSymbols(dso_name, symbols->data(), symbols->size());
  } else {
    FML_LOG(ERROR) << "Failed to load " << symbols_path;
  }
}
#endif  // !defined(DART_PRODUCT)

Runner::Runner(fml::RefPtr<fml::TaskRunner> task_runner,
               sys::ComponentContext* context)
    : task_runner_(task_runner), context_(context) {
#if !defined(DART_PRODUCT)
  // The VM service isolate uses the process-wide namespace. It writes the
  // vm service protocol port under /tmp. The VMServiceObject exposes that
  // port number to The Hub.
  context_->outgoing()->debug_dir()->AddEntry(
      dart_utils::VMServiceObject::kPortDirName,
      std::make_unique<dart_utils::VMServiceObject>());

  inspect::Inspector* inspector = dart_utils::RootInspectNode::GetInspector();
  inspector->GetRoot().CreateLazyValues(
      "vmservice_port",
      [&]() {
        inspect::Inspector inspector;
        dart_utils::VMServiceObject::LazyEntryVector out;
        dart_utils::VMServiceObject().GetContents(&out);
        std::string name = "";
        if (!out.empty()) {
          name = out[0].name;
        }
        inspector.GetRoot().CreateString("vm_service_port", name, &inspector);
        return fpromise::make_ok_promise(inspector);
      },
      inspector);

  SetupTraceObserver();
#endif  // !defined(DART_PRODUCT)

  SkGraphics::Init();

  SetupICU();

  SetProcessName();

  SetThreadName("io.flutter.runner.main");

  context_->outgoing()
      ->AddPublicService<fuchsia::component::runner::ComponentRunner>(
          std::bind(&Runner::RegisterComponentV2, this, std::placeholders::_1));

#if !defined(DART_PRODUCT)
  if (Dart_IsPrecompiledRuntime()) {
    RegisterProfilerSymbols("pkg/data/flutter_aot_runner.dartprofilersymbols",
                            "");
  } else {
    RegisterProfilerSymbols("pkg/data/flutter_jit_runner.dartprofilersymbols",
                            "");
  }
#endif  // !defined(DART_PRODUCT)
}

Runner::~Runner() {
  context_->outgoing()
      ->RemovePublicService<fuchsia::component::runner::ComponentRunner>();

#if !defined(DART_PRODUCT)
  trace_observer_->Stop();
#endif  // !defined(DART_PRODUCT)
}

// CF v2 lifecycle methods.

void Runner::RegisterComponentV2(
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentRunner>
        request) {
  active_components_v2_bindings_.AddBinding(this, std::move(request));
}

void Runner::Start(
    fuchsia::component::runner::ComponentStartInfo start_info,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        controller) {
  // TRACE_DURATION currently requires that the string data does not change
  // in the traced scope. Since |package| gets moved in the ComponentV2::Create
  // call below, we cannot ensure that |package.resolved_url| does not move or
  // change, so we make a copy to pass to TRACE_DURATION.
  // TODO(PT-169): Remove this copy when TRACE_DURATION reads string arguments
  // eagerly.
  const std::string url_copy = start_info.resolved_url();
  TRACE_EVENT1("flutter", "Start", "url", url_copy.c_str());

  // Notes on component termination: Components typically terminate on the
  // thread on which they were created. This usually means the thread was
  // specifically created to host the component. But we want to ensure that
  // access to the active components collection is made on the same thread. So
  // we capture the runner in the termination callback. There is no risk of
  // there being multiple component runner instances in the process at the same
  // time. So it is safe to use the raw pointer.
  ComponentV2::TerminationCallback termination_callback =
      [component_runner = this](const ComponentV2* component) {
        component_runner->task_runner_->PostTask(
            [component_runner, component]() {
              component_runner->OnComponentV2Terminate(component);
            });
      };

  ActiveComponentV2 active_component = ComponentV2::Create(
      std::move(termination_callback), std::move(start_info),
      context_->svc() /* runner_incoming_services */, std::move(controller));

  auto key = active_component.component.get();
  active_components_v2_[key] = std::move(active_component);
}

void Runner::OnComponentV2Terminate(const ComponentV2* component) {
  auto active_component_it = active_components_v2_.find(component);
  if (active_component_it == active_components_v2_.end()) {
    FML_LOG(INFO)
        << "The remote end of the component runner tried to terminate an "
           "component that has already been terminated, possibly because we "
           "initiated the termination";
    return;
  }
  ActiveComponentV2& active_component = active_component_it->second;

  // Grab the items out of the entry because we will have to rethread the
  // destruction.
  std::unique_ptr<ComponentV2> component_to_destroy =
      std::move(active_component.component);
  std::unique_ptr<fml::Thread> component_thread =
      std::move(active_component.platform_thread);

  // Delete the entry.
  active_components_v2_.erase(component);

  // Post the task to destroy the component and quit its message loop.
  component_thread->GetTaskRunner()->PostTask(fml::MakeCopyable(
      [instance = std::move(component_to_destroy),
       thread = component_thread.get()]() mutable { instance.reset(); }));

  // Terminate and join the thread's message loop.
  component_thread->Join();
}

void Runner::SetupICU() {
  // Exposes the TZ data setup for testing.  Failing here is not fatal.
  Runner::SetupTZDataInternal();
  if (!Runner::SetupICUInternal()) {
    FML_LOG(ERROR) << "Could not initialize ICU data.";
  }
}

// static
bool Runner::SetupICUInternal() {
  return InitializeICU();
}

// static
bool Runner::SetupTZDataInternal() {
  return InitializeTZData();
}

#if !defined(DART_PRODUCT)
void Runner::SetupTraceObserver() {
  fml::AutoResetWaitableEvent latch;

  fml::TaskRunner::RunNowOrPostTask(task_runner_, [&]() {
    // Running this initialization code on task_runner_ ensures that the call to
    // `async_get_default_dispatcher()` will capture the correct dispatcher.
    trace_observer_ = std::make_unique<trace::TraceObserver>();
    trace_observer_->Start(async_get_default_dispatcher(), [runner = this]() {
      if (!trace_is_category_enabled("dart:profiler")) {
        return;
      }
      if (trace_state() == TRACE_STARTED) {
        runner->prolonged_context_ = trace_acquire_prolonged_context();
        Dart_StartProfiling();
      } else if (trace_state() == TRACE_STOPPING) {
        auto write_profile_trace_for_components = [](auto& components) {
          for (auto& it : components) {
            fml::AutoResetWaitableEvent latch;
            fml::TaskRunner::RunNowOrPostTask(
                it.second.platform_thread->GetTaskRunner(), [&]() {
                  it.second.component->WriteProfileToTrace();
                  latch.Signal();
                });
            latch.Wait();
          }
        };
        write_profile_trace_for_components(runner->active_components_v2_);

        Dart_StopProfiling();
        trace_release_prolonged_context(runner->prolonged_context_);
      }
    });
    latch.Signal();
  });
  latch.Wait();
}
#endif  // !defined(DART_PRODUCT)

void Runner::handle_unknown_method(uint64_t ordinal, bool method_has_response) {
  FML_LOG(ERROR) << "Unknown method called on Runner. Ordinal: " << ordinal;
}

}  // namespace flutter_runner
