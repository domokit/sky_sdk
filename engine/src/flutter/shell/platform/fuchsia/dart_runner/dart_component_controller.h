// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_COMPONENT_CONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_COMPONENT_CONTROLLER_H_

#include <dart/test/cpp/fidl.h>
#include <fuchsia/component/runner/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async/cpp/wait.h>
#include <lib/fdio/namespace.h>
#include <lib/fidl/cpp/binding.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/fidl/cpp/string.h>
#include <lib/sys/cpp/component_context.h>
#include <lib/sys/cpp/service_directory.h>
#include <lib/vfs/cpp/pseudo_dir.h>
#include <lib/zx/timer.h>

#include <memory>

#include "runtime/dart/utils/mapped_resource.h"
#include "third_party/dart/runtime/include/dart_api.h"

namespace dart_runner {

/// Starts a Dart component written in CFv2.
class DartComponentController
    : public dart::test::Echo,
      public fuchsia::component::runner::ComponentController {
 public:
  DartComponentController(
      fuchsia::component::runner::ComponentStartInfo start_info,
      std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
          controller);
  ~DartComponentController() override;

  /// Sets up the controller.
  ///
  /// This should be called before |Run|.
  bool SetUp();

  /// Runs the Dart component in a task, sending the return code back to
  /// the Fuchsia component controller.
  ///
  /// This should be called after |SetUp|.
  void Run();

  void handle_unknown_method(uint64_t ordinal,
                             bool method_has_response) override;

 private:
  /// Helper for actually running the Dart main. Returns true if successful,
  /// false otherwise.
  bool RunDartMain();

  /// Creates and binds the namespace for this component. Returns true if
  /// successful, false otherwise.
  bool CreateAndBindNamespace();

  bool SetUpFromKernel();
  bool SetUpFromAppSnapshot();

  bool CreateIsolate(const uint8_t* isolate_snapshot_data,
                     const uint8_t* isolate_snapshot_instructions);

  // |Echo|
  void EchoString(fidl::StringPtr value, EchoStringCallback callback) override;

  // |ComponentController|
  void Kill() override;
  void Stop() override;

  // Idle notification.
  void MessageEpilogue(Dart_Handle result);
  void OnIdleTimer(async_dispatcher_t* dispatcher,
                   async::WaitBase* wait,
                   zx_status_t status,
                   const zx_packet_signal* signal);

  // The loop must be the first declared member so that it gets destroyed after
  // binding_ which expects the existence of a loop.
  std::unique_ptr<async::Loop> loop_;

  std::string label_;
  std::string url_;
  std::string data_path_;
  std::shared_ptr<sys::ServiceDirectory> runner_incoming_services_;
  std::unique_ptr<sys::ComponentContext> context_;
  std::unique_ptr<vfs::PseudoDir> dart_outgoing_dir_;
  fuchsia::io::DirectoryPtr dart_outgoing_dir_ptr_;
  fidl::InterfaceRequest<fuchsia::io::Directory> dart_outgoing_dir_request_;
  fuchsia::io::NodePtr dart_outgoing_dir_ptr_to_check_on_open_;

  fuchsia::component::runner::ComponentStartInfo start_info_;
  fidl::Binding<fuchsia::component::runner::ComponentController> binding_;
  fidl::BindingSet<dart::test::Echo> echo_binding_;

  fdio_ns_t* namespace_ = nullptr;
  int stdout_fd_ = -1;
  int stderr_fd_ = -1;

  dart_utils::ElfSnapshot elf_snapshot_;                      // AOT snapshot
  dart_utils::MappedResource isolate_snapshot_data_;          // JIT snapshot
  dart_utils::MappedResource isolate_snapshot_instructions_;  // JIT snapshot
  std::vector<dart_utils::MappedResource> kernel_peices_;

  Dart_Isolate isolate_;
  int32_t return_code_ = 0;

  zx::time idle_start_{0};
  zx::timer idle_timer_;
  async::WaitMethod<DartComponentController,
                    &DartComponentController::OnIdleTimer>
      idle_wait_{this};

  // Disallow copy and assignment.
  DartComponentController(const DartComponentController&) = delete;
  DartComponentController& operator=(const DartComponentController&) = delete;
};

}  // namespace dart_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_COMPONENT_CONTROLLER_H_
