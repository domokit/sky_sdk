// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_TEST_COMPONENT_CONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_TEST_COMPONENT_CONTROLLER_H_

#include <memory>

#include <fuchsia/component/runner/cpp/fidl.h>
#include <fuchsia/test/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async/cpp/executor.h>
#include <lib/async/cpp/wait.h>
#include <lib/fdio/namespace.h>
#include <lib/sys/cpp/component_context.h>
#include <lib/sys/cpp/service_directory.h>
#include <lib/zx/timer.h>

#include <lib/fidl/cpp/binding_set.h>
#include "lib/fidl/cpp/binding.h"
#include "runtime/dart/utils/mapped_resource.h"
#include "third_party/dart/runtime/include/dart_api.h"

namespace dart_runner {

/// Starts a Dart test component written in CFv2. It's different from
/// DartComponentController in that it must implement the
/// |fuchsia.test.Suite| protocol. It was forked to avoid a naming clash
/// between the two classes' methods as the Suite protocol requires a Run()
/// method for the test_manager to call on. This way, we avoid an extra layer
/// between the test_manager and actual test execution.
/// TODO(fxb/98369): Look into combining the two component classes once dart
/// testing is stable.
class DartTestComponentController
    : public fuchsia::component::runner::ComponentController,
      public fuchsia::test::Suite {
  using DoneCallback = fit::function<void(DartTestComponentController*)>;

 public:
  DartTestComponentController(
      fuchsia::component::runner::ComponentStartInfo start_info,
      std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
          controller,
      DoneCallback done_callback);

  ~DartTestComponentController() override;

  /// Sets up the controller.
  ///
  /// This should be called before |Run|.
  void SetUp();

  /// |Suite| protocol implementation.
  void GetTests(
      fidl::InterfaceRequest<fuchsia::test::CaseIterator> iterator) override;

  /// |Suite| protocol implementation.
  void Run(std::vector<fuchsia::test::Invocation> tests,
           fuchsia::test::RunOptions options,
           fidl::InterfaceHandle<fuchsia::test::RunListener> listener) override;

  fidl::InterfaceRequestHandler<fuchsia::test::Suite> GetHandler() {
    return suite_bindings_.GetHandler(this, loop_->dispatcher());
  }

  void handle_unknown_method(uint64_t ordinal,
                             bool method_has_response) override;

 private:
  /// Helper for actually running the Dart main. Returns a promise.
  fpromise::promise<> RunDartMain();

  /// Creates and binds the namespace for this component. Returns true if
  /// successful, false otherwise.
  bool CreateAndBindNamespace();

  bool SetUpFromKernel();
  bool SetUpFromAppSnapshot();

  bool CreateIsolate(const uint8_t* isolate_snapshot_data,
                     const uint8_t* isolate_snapshot_instructions);

  // |ComponentController|
  void Kill() override;
  void Stop() override;

  // Idle notification.
  void MessageEpilogue(Dart_Handle result);
  void OnIdleTimer(async_dispatcher_t* dispatcher,
                   async::WaitBase* wait,
                   zx_status_t status,
                   const zx_packet_signal* signal);

  // |CaseIterator|
  class CaseIterator final : public fuchsia::test::CaseIterator {
   public:
    CaseIterator(fidl::InterfaceRequest<fuchsia::test::CaseIterator> request,
                 async_dispatcher_t* dispatcher,
                 std::string test_component_name,
                 fit::function<void(CaseIterator*)> done_callback);

    void GetNext(GetNextCallback callback) override;

   private:
    bool first_case_ = true;
    fidl::Binding<fuchsia::test::CaseIterator> binding_;
    std::string test_component_name_;
    fit::function<void(CaseIterator*)> done_callback_;
  };

  std::unique_ptr<CaseIterator> RemoveCaseInterator(CaseIterator*);

  // We only need one case_listener currently as dart tests are run as one
  // large test file. In future iterations, case_listeners must be
  // created per test case.
  fidl::InterfacePtr<fuchsia::test::CaseListener> case_listener_;
  std::map<CaseIterator*, std::unique_ptr<CaseIterator>> case_iterators_;

  // |Suite|

  /// Exposes suite protocol on behalf of test component.
  std::string test_component_name_;
  std::unique_ptr<sys::ComponentContext> suite_context_;
  fidl::BindingSet<fuchsia::test::Suite> suite_bindings_;

  // The loop must be the first declared member so that it gets destroyed after
  // binding_ which expects the existence of a loop.
  std::unique_ptr<async::Loop> loop_;
  async::Executor executor_;

  std::string label_;
  std::string url_;
  std::shared_ptr<sys::ServiceDirectory> runner_incoming_services_;
  std::string data_path_;
  std::unique_ptr<sys::ComponentContext> context_;

  fuchsia::component::runner::ComponentStartInfo start_info_;
  fidl::Binding<fuchsia::component::runner::ComponentController> binding_;
  DoneCallback done_callback_;

  zx::socket out_, err_, out_client_, err_client_;
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
  async::WaitMethod<DartTestComponentController,
                    &DartTestComponentController::OnIdleTimer>
      idle_wait_{this};

  // Disallow copy and assignment.
  DartTestComponentController(const DartTestComponentController&) = delete;
  DartTestComponentController& operator=(const DartTestComponentController&) =
      delete;
};

}  // namespace dart_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_DART_TEST_COMPONENT_CONTROLLER_H_
