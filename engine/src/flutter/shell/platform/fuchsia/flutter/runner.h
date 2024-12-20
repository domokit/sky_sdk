// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_RUNNER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_RUNNER_H_

#include <memory>
#include <unordered_map>

#include <fuchsia/component/runner/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/sys/cpp/component_context.h>
#include <lib/trace-engine/instrumentation.h>
#include <lib/trace/observer.h>

#include "component_v2.h"
#include "flutter/fml/macros.h"
#include "fml/memory/ref_ptr.h"
#include "fml/task_runner.h"
#include "lib/fidl/cpp/binding_set.h"
#include "runtime/dart/utils/vmservice_object.h"

namespace flutter_runner {

/// Publishes the CF v2 runner service.
///
/// Each component will be run on a separate thread dedicated to that component.
///
/// TODO(fxb/50694): Add unit tests for CF v2.
class Runner final
    : public fuchsia::component::runner::ComponentRunner /* CF v2 */ {
 public:
  // Does not take ownership of context.
  Runner(fml::RefPtr<fml::TaskRunner> task_runner,
         sys::ComponentContext* context);

  ~Runner();

 private:
  // CF v2 lifecycle methods.

  // |fuchsia::component::runner::ComponentRunner|
  void Start(
      fuchsia::component::runner::ComponentStartInfo start_info,
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentController>
          controller) override;

  /// Registers a new CF v2 component with this runner, binding the component
  /// to this runner.
  void RegisterComponentV2(
      fidl::InterfaceRequest<fuchsia::component::runner::ComponentRunner>
          request);

  /// Callback that should be fired when a registered CF v2 component is
  /// terminated.
  void OnComponentV2Terminate(const ComponentV2* component);

  void SetupICU();

#if !defined(DART_PRODUCT)
  void SetupTraceObserver();
#endif  // !defined(DART_PRODUCT)

  // Called from SetupICU, for testing only.  Returns false on error.
  static bool SetupICUInternal();
  // Called from SetupICU, for testing only.  Returns false on error.
  static bool SetupTZDataInternal();
#if defined(FRIEND_TEST)
  FRIEND_TEST(RunnerTZDataTest, LoadsWithTZDataPresent);
  FRIEND_TEST(RunnerTZDataTest, LoadsWithoutTZDataPresent);
#endif  // defined(FRIEND_TEST)

  fml::RefPtr<fml::TaskRunner> task_runner_;

  sys::ComponentContext* context_;

  // CF v2 component state.

  /// The components that are currently bound to this runner.
  fidl::BindingSet<fuchsia::component::runner::ComponentRunner>
      active_components_v2_bindings_;

  /// The components that are currently actively running on threads.
  std::unordered_map<const ComponentV2*, ActiveComponentV2>
      active_components_v2_;

#if !defined(DART_PRODUCT)
  // The connection between the Dart VM service and The Hub.
  std::unique_ptr<dart_utils::VMServiceObject> vmservice_object_;

  std::unique_ptr<trace::TraceObserver> trace_observer_;
  trace_prolonged_context_t* prolonged_context_;
#endif  // !defined(DART_PRODUCT)

  FML_DISALLOW_COPY_AND_ASSIGN(Runner);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_RUNNER_H_
