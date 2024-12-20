// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_DART_ISOLATE_RUNNER_H_
#define FLUTTER_TESTING_DART_ISOLATE_RUNNER_H_

#include "flutter/common/task_runners.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/thread.h"
#include "flutter/runtime/dart_isolate.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/dart_vm_lifecycle.h"

namespace flutter::testing {

class AutoIsolateShutdown {
 public:
  AutoIsolateShutdown() = default;

  AutoIsolateShutdown(std::shared_ptr<DartIsolate> isolate,
                      fml::RefPtr<fml::TaskRunner> runner);

  ~AutoIsolateShutdown();

  bool IsValid() const { return isolate_ != nullptr && runner_; }

  [[nodiscard]] bool RunInIsolateScope(
      const std::function<bool(void)>& closure);

  void Shutdown();

  DartIsolate* get() {
    FML_CHECK(isolate_);
    return isolate_.get();
  }

 private:
  std::shared_ptr<DartIsolate> isolate_;
  fml::RefPtr<fml::TaskRunner> runner_;

  FML_DISALLOW_COPY_AND_ASSIGN(AutoIsolateShutdown);
};

void RunDartCodeInIsolate(
    DartVMRef& vm_ref,
    std::unique_ptr<AutoIsolateShutdown>& result,
    const Settings& settings,
    const TaskRunners& task_runners,
    std::string entrypoint,
    const std::vector<std::string>& args,
    const std::string& fixtures_path,
    fml::WeakPtr<IOManager> io_manager = {},
    std::unique_ptr<PlatformConfiguration> platform_configuration = nullptr);

std::unique_ptr<AutoIsolateShutdown> RunDartCodeInIsolate(
    DartVMRef& vm_ref,
    const Settings& settings,
    const TaskRunners& task_runners,
    std::string entrypoint,
    const std::vector<std::string>& args,
    const std::string& fixtures_path,
    fml::WeakPtr<IOManager> io_manager = {},
    std::unique_ptr<PlatformConfiguration> platform_configuration = nullptr);

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_DART_ISOLATE_RUNNER_H_
