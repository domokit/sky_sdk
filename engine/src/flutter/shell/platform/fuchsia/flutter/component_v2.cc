// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "component_v2.h"

#include <dlfcn.h>
#include <fuchsia/mem/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async/cpp/task.h>
#include <lib/async/default.h>
#include <lib/fdio/directory.h>
#include <lib/fdio/io.h>
#include <lib/fdio/namespace.h>
#include <lib/vfs/cpp/composed_service_dir.h>
#include <lib/vfs/cpp/remote_dir.h>
#include <lib/vfs/cpp/service.h>
#include <sys/stat.h>
#include <zircon/dlfcn.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <memory>
#include <regex>
#include <sstream>

#include "file_in_namespace_buffer.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/platform/fuchsia/task_observers.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/unique_fd.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/shell/common/switches.h"
#include "runtime/dart/utils/files.h"
#include "runtime/dart/utils/handle_exception.h"
#include "runtime/dart/utils/mapped_resource.h"
#include "runtime/dart/utils/tempfs.h"
#include "runtime/dart/utils/vmo.h"

namespace flutter_runner {
namespace {

// "data" and "assets" are arguments that are specific to the Flutter runner.
// They will likely go away if we migrate to the ELF runner.
constexpr char kDataKey[] = "data";
constexpr char kAssetsKey[] = "assets";

// "args" are how the component specifies arguments to the runner.
constexpr char kArgsKey[] = "args";
constexpr char kOldGenHeapSizeKey[] = "old_gen_heap_size";
constexpr char kExposeDirsKey[] = "expose_dirs";

constexpr char kTmpPath[] = "/tmp";
constexpr char kServiceRootPath[] = "/svc";
constexpr char kRunnerConfigPath[] = "/config/data/flutter_runner_config";

std::string DebugLabelForUrl(const std::string& url) {
  auto found = url.rfind("/");
  if (found == std::string::npos) {
    return url;
  } else {
    return {url, found + 1};
  }
}

/// Parses the |args| field from the "program" field into
/// |metadata|.
void ParseArgs(std::vector<std::string>& args, ProgramMetadata* metadata) {
  // fml::CommandLine expects the first argument to be the name of the program,
  // so we prepend a dummy argument so we can use fml::CommandLine to parse the
  // arguments for us.
  std::vector<std::string> command_line_args = {""};
  command_line_args.insert(command_line_args.end(), args.begin(), args.end());
  fml::CommandLine parsed_args = fml::CommandLineFromIterators(
      command_line_args.begin(), command_line_args.end());

  std::string old_gen_heap_size_option;
  if (parsed_args.GetOptionValue(kOldGenHeapSizeKey,
                                 &old_gen_heap_size_option)) {
    int64_t specified_old_gen_heap_size = strtol(
        old_gen_heap_size_option.c_str(), nullptr /* endptr */, 10 /* base */);
    if (specified_old_gen_heap_size != 0) {
      metadata->old_gen_heap_size = specified_old_gen_heap_size;
    } else {
      FML_LOG(ERROR) << "Invalid old_gen_heap_size: "
                     << old_gen_heap_size_option;
    }
  }

  std::string expose_dirs_option;
  if (parsed_args.GetOptionValue(kExposeDirsKey, &expose_dirs_option)) {
    // Parse the comma delimited string
    std::vector<std::string> expose_dirs;
    std::stringstream s(expose_dirs_option);
    while (s.good()) {
      std::string dir;
      getline(s, dir, ',');  // get first string delimited by comma
      metadata->expose_dirs.push_back(dir);
    }
  }
}

}  // namespace

ProgramMetadata ComponentV2::ParseProgramMetadata(
    const fuchsia::data::Dictionary& program_metadata) {
  ProgramMetadata result;

  for (const auto& entry : program_metadata.entries()) {
    if (entry.key.compare(kDataKey) == 0 && entry.value != nullptr) {
      result.data_path = "pkg/" + entry.value->str();
    } else if (entry.key.compare(kAssetsKey) == 0 && entry.value != nullptr) {
      result.assets_path = "pkg/" + entry.value->str();
    } else if (entry.key.compare(kArgsKey) == 0 && entry.value != nullptr) {
      ParseArgs(entry.value->str_vec(), &result);
    }
  }

  // assets_path defaults to the same as data_path if omitted.
  if (result.assets_path.empty()) {
    result.assets_path = result.data_path;
  }

  return result;
}

ActiveComponentV2 ComponentV2::Create(
    TerminationCallback termination_callback,
    fuchsia::component::runner::ComponentStartInfo start_info,
    std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        controller) {
  auto thread = std::make_unique<fml::Thread>();
  std::unique_ptr<ComponentV2> component;

  fml::AutoResetWaitableEvent latch;
  thread->GetTaskRunner()->PostTask([&]() mutable {
    component.reset(
        new ComponentV2(std::move(termination_callback), std::move(start_info),
                        runner_incoming_services, std::move(controller)));
    latch.Signal();
  });

  latch.Wait();
  return {.platform_thread = std::move(thread),
          .component = std::move(component)};
}

ComponentV2::ComponentV2(
    TerminationCallback termination_callback,
    fuchsia::component::runner::ComponentStartInfo start_info,
    std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
    fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
        component_controller_request)
    : termination_callback_(std::move(termination_callback)),
      debug_label_(DebugLabelForUrl(start_info.resolved_url())),
      component_controller_(this),
      outgoing_dir_(new vfs::PseudoDir()),
      runtime_dir_(new vfs::PseudoDir()),
      runner_incoming_services_(runner_incoming_services),
      weak_factory_(this) {
  component_controller_.set_error_handler([this](zx_status_t status) {
    FML_LOG(ERROR) << "ComponentController binding error for component("
                   << debug_label_ << "): " << zx_status_get_string(status);
    KillWithEpitaph(
        zx_status_t(fuchsia::component::Error::INSTANCE_CANNOT_START));
  });

  FML_DCHECK(fdio_ns_.is_valid());

  // TODO(fxb/88391): Dart launch arguments.
  FML_LOG(WARNING) << "program() arguments are currently ignored (fxb/88391).";

  ProgramMetadata metadata = ParseProgramMetadata(start_info.program());

  if (metadata.data_path.empty()) {
    FML_LOG(ERROR) << "Could not find a /pkg/data directory for "
                   << start_info.resolved_url();
    return;
  }

  dart_utils::BindTemp(fdio_ns_.get());

  // ComponentStartInfo::ns (optional)
  if (start_info.has_ns()) {
    for (auto& entry : *start_info.mutable_ns()) {
      // /tmp/ is mapped separately to to a process-local virtual filesystem,
      // so we ignore it here.
      const auto& path = entry.path();
      if (path == kTmpPath) {
        continue;
      }

      // We should never receive namespace entries without a directory, but we
      // check it anyways to avoid crashing if we do.
      if (!entry.has_directory()) {
        FML_LOG(ERROR) << "Namespace entry at path (" << path
                       << ") has no directory.";
        continue;
      }

      zx::channel dir;
      if (path == kServiceRootPath) {
        svc_ = std::make_unique<sys::ServiceDirectory>(
            std::move(*entry.mutable_directory()));
        dir = svc_->CloneChannel().TakeChannel();
      } else {
        dir = entry.mutable_directory()->TakeChannel();
      }

      zx_handle_t dir_handle = dir.release();
      if (fdio_ns_bind(fdio_ns_.get(), path.data(), dir_handle) != ZX_OK) {
        FML_LOG(ERROR) << "Could not bind path to namespace: " << path;
        zx_handle_close(dir_handle);
      }
    }
  }

  // Open the data and assets directories inside our namespace.
  {
    fml::UniqueFD ns_fd(fdio_ns_opendir(fdio_ns_.get()));
    FML_DCHECK(ns_fd.is_valid());

    constexpr mode_t mode = O_RDONLY | O_DIRECTORY;

    component_assets_directory_.reset(
        openat(ns_fd.get(), metadata.assets_path.c_str(), mode));
    FML_DCHECK(component_assets_directory_.is_valid());

    component_data_directory_.reset(
        openat(ns_fd.get(), metadata.data_path.c_str(), mode));
    FML_DCHECK(component_data_directory_.is_valid());
  }

  // ComponentStartInfo::runtime_dir (optional).
  if (start_info.has_runtime_dir()) {
    fidl::ServerEnd<fuchsia_io::Directory> server_end{
        start_info.mutable_runtime_dir()->TakeChannel()};
    runtime_dir_->Serve(
        fuchsia_io::wire::kPermReadable | fuchsia_io::wire::kPermWritable,
        std::move(server_end));
  }

  // ComponentStartInfo::outgoing_dir (optional).
  if (start_info.has_outgoing_dir()) {
    fidl::ServerEnd<fuchsia_io::Directory> server_end{
        start_info.mutable_outgoing_dir()->TakeChannel()};
    outgoing_dir_->Serve(
        fuchsia_io::wire::kPermReadable | fuchsia_io::wire::kPermWritable,
        std::move(server_end));
  }

  directory_request_ = directory_ptr_.NewRequest();

  fuchsia::io::DirectoryHandle flutter_public_dir;
  {
    auto request = flutter_public_dir.NewRequest().TakeChannel();
    const zx_status_t status =
        fdio_open3_at(directory_ptr_.channel().get(), "svc",
                      uint64_t{fuchsia::io::PERM_READABLE}, request.release());
    if (status != ZX_OK) {
      FML_LOG(ERROR) << "Failed to open /svc in outgoing directory: "
                     << zx_status_get_string(status);
      return;
    }
  }

  auto composed_service_dir = std::make_unique<vfs::ComposedServiceDir>();
  composed_service_dir->set_fallback(std::move(flutter_public_dir));

  // Request an event from the directory to ensure it is servicing requests.
  directory_ptr_->Open3(".",
                        fuchsia::io::Flags::PROTOCOL_NODE |
                            fuchsia::io::Flags::FLAG_SEND_REPRESENTATION,
                        {}, cloned_directory_ptr_.NewRequest().TakeChannel());

  // Collect our standard set of directories along with directories that are
  // included in the cml file to expose.
  std::vector<std::string> other_dirs = {"debug", "ctrl"};
  for (auto dir : metadata.expose_dirs) {
    other_dirs.push_back(dir);
  }

  cloned_directory_ptr_.events().OnRepresentation = [this,
                                                     other_dirs](auto unused) {
    cloned_directory_ptr_.Unbind();
    // add other directories as RemoteDirs.
    for (auto& dir_str : other_dirs) {
      fuchsia::io::DirectoryHandle dir;
      auto request = dir.NewRequest().TakeChannel();
      const zx_status_t status =
          fdio_open3_at(directory_ptr_.channel().get(), dir_str.c_str(),
                        uint64_t{fuchsia::io::Flags::PROTOCOL_DIRECTORY |
                                 fuchsia::io::PERM_READABLE},
                        request.release());
      if (status == ZX_OK) {
        outgoing_dir_->AddEntry(
            dir_str.c_str(),
            std::make_unique<vfs::RemoteDir>(dir.TakeChannel()));
      } else {
        FML_LOG(ERROR) << "could not add out directory entry(" << dir_str
                       << ") for flutter component(" << debug_label_
                       << "): " << zx_status_get_string(status);
      }
    }
  };

  cloned_directory_ptr_.set_error_handler(
      [this](zx_status_t status) { cloned_directory_ptr_.Unbind(); });

  // TODO(fxb/89162): Close handles from ComponentStartInfo::numbered_handles
  // since we're not using them. See documentation from ComponentController:
  // https://cs.opensource.google/fuchsia/fuchsia/+/main:sdk/fidl/fuchsia.component.runner/component_runner.fidl;l=97;drc=e3b39f2b57e720770773b857feca4f770ee0619e

  // TODO(fxb/89162): There's an OnPublishDiagnostics event we may want to
  // fire for diagnostics. See documentation from ComponentController:
  // https://cs.opensource.google/fuchsia/fuchsia/+/main:sdk/fidl/fuchsia.component.runner/component_runner.fidl;l=181;drc=e3b39f2b57e720770773b857feca4f770ee0619e

  // All launch arguments have been read. Perform service binding and
  // final settings configuration. The next call will be to create a view
  // for this component.
  composed_service_dir->AddService(
      fuchsia::ui::app::ViewProvider::Name_,
      std::make_unique<vfs::Service>(
          [this](zx::channel channel, async_dispatcher_t* dispatcher) {
            shells_bindings_.AddBinding(
                this, fidl::InterfaceRequest<fuchsia::ui::app::ViewProvider>(
                          std::move(channel)));
          }));
  outgoing_dir_->AddEntry("svc", std::move(composed_service_dir));

  // Setup the component controller binding.
  if (component_controller_request) {
    component_controller_.Bind(std::move(component_controller_request));
  }

  // Load and use runner-specific configuration, if it exists.
  std::string json_string;
  if (dart_utils::ReadFileToString(kRunnerConfigPath, &json_string)) {
    product_config_ = FlutterRunnerProductConfiguration(json_string);
    FML_LOG(INFO) << "Successfully loaded runner configuration: "
                  << json_string;
  } else {
    FML_LOG(WARNING) << "Failed to load runner configuration from "
                     << kRunnerConfigPath << "; using default config values.";
  }

  // Load VM and component bytecode.
  // For AOT, compare with flutter_aot_app in flutter_app.gni.
  // For JIT, compare flutter_jit_runner in BUILD.gn.
  if (flutter::DartVM::IsRunningPrecompiledCode()) {
    std::shared_ptr<dart_utils::ElfSnapshot> snapshot =
        std::make_shared<dart_utils::ElfSnapshot>();
    if (snapshot->Load(component_data_directory_.get(),
                       "app_aot_snapshot.so")) {
      const uint8_t* isolate_data = snapshot->IsolateData();
      const uint8_t* isolate_instructions = snapshot->IsolateInstrs();
      const uint8_t* vm_data = snapshot->VmData();
      const uint8_t* vm_instructions = snapshot->VmInstrs();
      if (isolate_data == nullptr || isolate_instructions == nullptr ||
          vm_data == nullptr || vm_instructions == nullptr) {
        FML_LOG(FATAL) << "ELF snapshot missing AOT symbols.";
        return;
      }
      auto hold_snapshot = [snapshot](const uint8_t* _, size_t __) {};
      settings_.vm_snapshot_data = [hold_snapshot, vm_data]() {
        return std::make_unique<fml::NonOwnedMapping>(vm_data, 0, hold_snapshot,
                                                      true /* dontneed_safe */);
      };
      settings_.vm_snapshot_instr = [hold_snapshot, vm_instructions]() {
        return std::make_unique<fml::NonOwnedMapping>(
            vm_instructions, 0, hold_snapshot, true /* dontneed_safe */);
      };
      settings_.isolate_snapshot_data = [hold_snapshot, isolate_data]() {
        return std::make_unique<fml::NonOwnedMapping>(
            isolate_data, 0, hold_snapshot, true /* dontneed_safe */);
      };
      settings_.isolate_snapshot_instr = [hold_snapshot,
                                          isolate_instructions]() {
        return std::make_unique<fml::NonOwnedMapping>(
            isolate_instructions, 0, hold_snapshot, true /* dontneed_safe */);
      };
      isolate_snapshot_ = fml::MakeRefCounted<flutter::DartSnapshot>(
          std::make_shared<fml::NonOwnedMapping>(isolate_data, 0, hold_snapshot,
                                                 true /* dontneed_safe */),
          std::make_shared<fml::NonOwnedMapping>(isolate_instructions, 0,
                                                 hold_snapshot,
                                                 true /* dontneed_safe */));
    } else {
      const int namespace_fd = component_data_directory_.get();
      settings_.vm_snapshot_data = [namespace_fd]() {
        return LoadFile(namespace_fd, "vm_snapshot_data.bin",
                        false /* executable */);
      };
      settings_.vm_snapshot_instr = [namespace_fd]() {
        return LoadFile(namespace_fd, "vm_snapshot_instructions.bin",
                        true /* executable */);
      };
      settings_.isolate_snapshot_data = [namespace_fd]() {
        return LoadFile(namespace_fd, "isolate_snapshot_data.bin",
                        false /* executable */);
      };
      settings_.isolate_snapshot_instr = [namespace_fd]() {
        return LoadFile(namespace_fd, "isolate_snapshot_instructions.bin",
                        true /* executable */);
      };
    }
  } else {
    settings_.vm_snapshot_data = []() {
      return MakeFileMapping("/pkg/data/vm_snapshot_data.bin",
                             false /* executable */);
    };
    settings_.isolate_snapshot_data = []() {
      return MakeFileMapping("/pkg/data/isolate_core_snapshot_data.bin",
                             false /* executable */);
    };
  }

#if defined(DART_PRODUCT)
  settings_.enable_vm_service = false;
#else
  settings_.enable_vm_service = true;

  // TODO(cbracken): pass this in as a param to allow 0.0.0.0, ::1, etc.
  settings_.vm_service_host = "127.0.0.1";
#endif

  // Controls whether category "skia" trace events are enabled.
  settings_.trace_skia = true;

  settings_.verbose_logging = true;

  settings_.advisory_script_uri = debug_label_;

  settings_.advisory_script_entrypoint = debug_label_;

  settings_.icu_data_path = "";

  settings_.assets_dir = component_assets_directory_.get();

  // Compare flutter_jit_app in flutter_app.gni.
  settings_.application_kernel_list_asset = "app.dilplist";

  settings_.log_tag = debug_label_ + std::string{"(flutter)"};

  if (metadata.old_gen_heap_size.has_value()) {
    settings_.old_gen_heap_size = *metadata.old_gen_heap_size;
  }

  // No asserts in debug or release product.
  // No asserts in release with flutter_profile=true (non-product)
  // Yes asserts in non-product debug.
#if !defined(DART_PRODUCT) && (!defined(FLUTTER_PROFILE) || !defined(NDEBUG))
  // Debug mode
  settings_.disable_dart_asserts = false;
#else
  // Release mode
  settings_.disable_dart_asserts = true;
#endif

  // Do not leak the VM; allow it to shut down normally when the last shell
  // terminates.
  settings_.leak_vm = false;

  settings_.task_observer_add =
      std::bind(&fml::CurrentMessageLoopAddAfterTaskObserver,
                std::placeholders::_1, std::placeholders::_2);

  settings_.task_observer_remove = std::bind(
      &fml::CurrentMessageLoopRemoveAfterTaskObserver, std::placeholders::_1);

  settings_.log_message_callback = [](const std::string& tag,
                                      const std::string& message) {
    if (!tag.empty()) {
      std::cout << tag << ": ";
    }
    std::cout << message << std::endl;
  };

  settings_.dart_flags = {};

  // Don't collect CPU samples from Dart VM C++ code.
  settings_.dart_flags.push_back("--no_profile_vm");

  // Scale back CPU profiler sampling period on ARM64 to avoid overloading
  // the tracing engine.
#if defined(__aarch64__)
  settings_.dart_flags.push_back("--profile_period=10000");
#endif  // defined(__aarch64__)

  auto platform_task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  const std::string component_url = start_info.resolved_url();
  settings_.unhandled_exception_callback = [weak = weak_factory_.GetWeakPtr(),
                                            platform_task_runner,
                                            runner_incoming_services,
                                            component_url](
                                               const std::string& error,
                                               const std::string& stack_trace) {
    if (weak) {
      // TODO(cbracken): unsafe. The above check and the PostTask below are
      // happening on the UI thread. If the Component dtor and thread
      // termination happen (on the platform thread) between the previous
      // line and the next line, a crash will occur since we'll be posting
      // to a dead thread. See Runner::OnComponentV2Terminate() in
      // runner.cc.
      platform_task_runner->PostTask([weak, runner_incoming_services,
                                      component_url, error, stack_trace]() {
        if (weak) {
          dart_utils::HandleException(runner_incoming_services, component_url,
                                      error, stack_trace);
        } else {
          FML_LOG(WARNING)
              << "Exception was thrown which was not caught in Flutter app: "
              << error;
        }
      });
    } else {
      FML_LOG(WARNING)
          << "Exception was thrown which was not caught in Flutter app: "
          << error;
    }
    // Ideally we would return whether HandleException returned ZX_OK, but
    // short of knowing if the exception was correctly handled, we return
    // false to have the error and stack trace printed in the logs.
    return false;
  };
}

ComponentV2::~ComponentV2() = default;

const std::string& ComponentV2::GetDebugLabel() const {
  return debug_label_;
}

void ComponentV2::Kill() {
  FML_VLOG(1) << "received Kill event";

  // From the documentation for ComponentController, ZX_OK should be sent when
  // the ComponentController receives a termination request. However, if the
  // component exited with a non-zero return code, we indicate this by sending
  // an INTERNAL epitaph instead.
  //
  // TODO(fxb/86666): Communicate return code from the ComponentController once
  // v2 has support.
  auto [got_return_code, return_code] = last_return_code_;
  if (got_return_code && return_code == 0) {
    KillWithEpitaph(ZX_OK);
  } else {
    if (got_return_code) {
      FML_LOG(ERROR) << "Component exited with non-zero return code: "
                     << return_code;
    } else {
      FML_LOG(ERROR) << "Failed to get return code for component";
    }

    KillWithEpitaph(zx_status_t(fuchsia::component::Error::INTERNAL));
  }

  // WARNING: Don't do anything past this point as this instance may have been
  // collected.
}

void ComponentV2::KillWithEpitaph(zx_status_t epitaph_status) {
  component_controller_.set_error_handler(nullptr);
  component_controller_.Close(epitaph_status);

  termination_callback_(this);
  // WARNING: Don't do anything past this point as this instance may have been
  // collected.
}

void ComponentV2::Stop() {
  FML_VLOG(1) << "received Stop event";

  // TODO(fxb/89162): Any other cleanup logic we should do that's appropriate
  // for Stop but not for Kill?
  KillWithEpitaph(ZX_OK);
}

void ComponentV2::OnEngineTerminate(const Engine* shell_holder) {
  auto found = std::find_if(shell_holders_.begin(), shell_holders_.end(),
                            [shell_holder](const auto& holder) {
                              return holder.get() == shell_holder;
                            });

  if (found == shell_holders_.end()) {
    // This indicates a deeper issue with memory management and should never
    // happen.
    FML_LOG(ERROR) << "Tried to terminate an unregistered shell holder.";
    FML_DCHECK(false);

    return;
  }

  // We may launch multiple shells in this component. However, we will
  // terminate when the last shell goes away. The error code returned to the
  // component controller will be the last isolate that had an error.
  auto return_code = shell_holder->GetEngineReturnCode();
  if (return_code.has_value()) {
    last_return_code_ = {true, return_code.value()};
  } else {
    FML_LOG(ERROR) << "Failed to get return code from terminated shell holder.";
  }

  shell_holders_.erase(found);

  if (shell_holders_.empty()) {
    FML_VLOG(1) << "Killing component because all shell holders have been "
                   "terminated.";
    Kill();
    // WARNING: Don't do anything past this point because the delegate may have
    // collected this instance via the termination callback.
  }
}

void ComponentV2::CreateView2(fuchsia::ui::app::CreateView2Args view_args) {
  if (!svc_) {
    FML_LOG(ERROR)
        << "Component incoming services was invalid when attempting to "
           "create a shell for a view provider request.";
    return;
  }

  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);
  auto view_ref_pair =
      std::make_pair(std::move(view_ref_control), std::move(view_ref));

  shell_holders_.emplace(std::make_unique<Engine>(
      *this,                      // delegate
      debug_label_,               // thread label
      svc_,                       // Component incoming services
      runner_incoming_services_,  // Runner incoming services
      settings_,                  // settings
      std::move(
          *view_args.mutable_view_creation_token()),  // view creation token
      std::move(view_ref_pair),                       // view ref pair
      std::move(fdio_ns_),                            // FDIO namespace
      std::move(directory_request_),                  // outgoing request
      product_config_,                                // product configuration
      std::vector<std::string>()                      // dart entrypoint args
      ));
}

#if !defined(DART_PRODUCT)
void ComponentV2::WriteProfileToTrace() const {
  for (const auto& engine : shell_holders_) {
    engine->WriteProfileToTrace();
  }
}
#endif  // !defined(DART_PRODUCT)

}  // namespace flutter_runner
