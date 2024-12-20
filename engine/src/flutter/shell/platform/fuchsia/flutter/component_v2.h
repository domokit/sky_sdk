// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_COMPONENT_V2_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_COMPONENT_V2_H_

#include <array>
#include <memory>
#include <set>

#include <fuchsia/component/runner/cpp/fidl.h>
#include <fuchsia/io/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async/default.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/fidl/cpp/interface_request.h>
#include <lib/fit/function.h>
#include <lib/sys/cpp/service_directory.h>
#include <lib/vfs/cpp/pseudo_dir.h>
#include <lib/zx/eventpair.h>

#include "flutter/common/settings.h"
#include "flutter/fml/macros.h"

#include "engine.h"
#include "flutter_runner_product_configuration.h"
#include "program_metadata.h"
#include "unique_fdio_ns.h"

namespace flutter_runner {

class ComponentV2;

struct ActiveComponentV2 {
  std::unique_ptr<fml::Thread> platform_thread;
  std::unique_ptr<ComponentV2> component;

  ActiveComponentV2& operator=(ActiveComponentV2&& other) noexcept {
    if (this != &other) {
      this->platform_thread.reset(other.platform_thread.release());
      this->component.reset(other.component.release());
    }
    return *this;
  }

  ~ActiveComponentV2() = default;
};

// Represents an instance of a CF v2 Flutter component that contains one or more
// Flutter engine instances.
//
// TODO(fxb/50694): Add unit tests once we've verified that the current behavior
// is working correctly.
class ComponentV2 final
    : public Engine::Delegate,
      public fuchsia::component::runner::ComponentController,
      public fuchsia::ui::app::ViewProvider {
 public:
  using TerminationCallback = fit::function<void(const ComponentV2*)>;

  // Creates a dedicated thread to run the component and creates the
  // component on it. The component can be accessed only on this thread.
  // This is a synchronous operation.
  static ActiveComponentV2 Create(
      TerminationCallback termination_callback,
      fuchsia::component::runner::ComponentStartInfo start_info,
      std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
          controller);

  // Must be called on the same thread returned from the create call. The thread
  // may be collected after.
  ~ComponentV2();

  /// Parses the program metadata that was provided for the component.
  ///
  /// |old_gen_heap_size| will be set to -1 if no value was specified.
  static ProgramMetadata ParseProgramMetadata(
      const fuchsia::data::Dictionary& program_metadata);

  const std::string& GetDebugLabel() const;

#if !defined(DART_PRODUCT)
  void WriteProfileToTrace() const;
#endif  // !defined(DART_PRODUCT)

 private:
  ComponentV2(
      TerminationCallback termination_callback,
      fuchsia::component::runner::ComponentStartInfo start_info,
      std::shared_ptr<sys::ServiceDirectory> runner_incoming_services,
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
          controller);

  // |fuchsia::component::runner::ComponentController|
  void Kill() override;

  /// Helper to actually |Kill| the component, closing the connection via an
  /// epitaph with the given |epitaph_status|. Call this instead of
  /// Kill() in the implementation of this class, as |Kill| is only intended for
  /// clients of the ComponentController protocol to call.
  ///
  /// To determine what |epitaph_status| is appropriate for your situation,
  /// see the documentation for |fuchsia.component.runner.ComponentController|.
  void KillWithEpitaph(zx_status_t epitaph_status);

  // |fuchsia::component::runner::ComponentController|
  void Stop() override;

  // |fuchsia::ui::app::ViewProvider|
  void CreateView2(fuchsia::ui::app::CreateView2Args view_args) override;
  void CreateViewWithViewRef(
      ::zx::eventpair token,
      ::fuchsia::ui::views::ViewRefControl view_ref_control,
      ::fuchsia::ui::views::ViewRef view_ref) override {}

  // |flutter::Engine::Delegate|
  void OnEngineTerminate(const Engine* holder) override;

  flutter::Settings settings_;
  FlutterRunnerProductConfiguration product_config_;
  TerminationCallback termination_callback_;
  const std::string debug_label_;
  UniqueFDIONS fdio_ns_ = UniqueFDIONSCreate();
  fml::UniqueFD component_data_directory_;
  fml::UniqueFD component_assets_directory_;

  fidl::Binding<fuchsia::component::runner::ComponentController>
      component_controller_;
  fuchsia::io::DirectoryPtr directory_ptr_;
  fuchsia::io::NodePtr cloned_directory_ptr_;
  fidl::InterfaceRequest<fuchsia::io::Directory> directory_request_;
  std::unique_ptr<vfs::PseudoDir> outgoing_dir_;
  std::unique_ptr<vfs::PseudoDir> runtime_dir_;
  std::shared_ptr<sys::ServiceDirectory> svc_;
  std::shared_ptr<sys::ServiceDirectory> runner_incoming_services_;
  fidl::BindingSet<fuchsia::ui::app::ViewProvider> shells_bindings_;

  fml::RefPtr<flutter::DartSnapshot> isolate_snapshot_;
  std::set<std::unique_ptr<Engine>> shell_holders_;
  std::pair<bool, uint32_t> last_return_code_;
  fml::WeakPtrFactory<ComponentV2> weak_factory_;  // Must be the last member.
  FML_DISALLOW_COPY_AND_ASSIGN(ComponentV2);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_COMPONENT_V2_H_
