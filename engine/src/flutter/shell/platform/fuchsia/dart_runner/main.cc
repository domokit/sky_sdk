// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/inspect/component/cpp/component.h>
#include <lib/trace-provider/provider.h>
#include <lib/trace/event.h>

#include "dart_runner.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/platform/fuchsia/log_interest_listener.h"
#include "flutter/fml/platform/fuchsia/log_state.h"
#include "flutter/fml/trace_event.h"
#include "logging.h"
#include "platform/utils.h"
#include "runtime/dart/utils/build_info.h"
#include "runtime/dart/utils/files.h"
#include "runtime/dart/utils/root_inspect_node.h"
#include "runtime/dart/utils/tempfs.h"
#include "third_party/dart/runtime/include/dart_api.h"

#if !defined(DART_PRODUCT)
// Register native symbol information for the Dart VM's profiler.
static void RegisterProfilerSymbols(const char* symbols_path,
                                    const char* dso_name) {
  std::string* symbols = new std::string();
  if (dart_utils::ReadFileToString(symbols_path, symbols)) {
    Dart_AddSymbols(dso_name, symbols->data(), symbols->size());
  } else {
    FML_LOG(ERROR) << "Failed to load " << symbols_path;
    FML_CHECK(false);
  }
}
#endif  // !defined(DART_PRODUCT)

int main(int argc, const char** argv) {
  async::Loop loop(&kAsyncLoopConfigAttachToCurrentThread);

  // Setup logging.
  fml::LogState::Default().SetTags({LOG_TAG});
  fml::LogInterestListener listener(fml::LogState::Default().TakeClientEnd(),
                                    loop.dispatcher());
  listener.AsyncWaitForInterestChanged();

  // Create our component context which is served later.
  auto context = sys::ComponentContext::Create();
  dart_utils::RootInspectNode::Initialize(context.get());
  auto build_info = dart_utils::RootInspectNode::CreateRootChild("build_info");
  dart_utils::BuildInfo::Dump(build_info);

  dart::SetDartVmNode(std::make_unique<inspect::Node>(
      dart_utils::RootInspectNode::CreateRootChild("vm")));

  std::unique_ptr<trace::TraceProviderWithFdio> provider;
  {
    bool already_started;
    // Use CreateSynchronously to prevent loss of early events.
    trace::TraceProviderWithFdio::CreateSynchronously(
        loop.dispatcher(), "dart_runner", &provider, &already_started);
  }

#if !defined(DART_PRODUCT)
#if defined(AOT_RUNTIME)
  RegisterProfilerSymbols("pkg/data/dart_aot_runner.dartprofilersymbols", "");
#else
  RegisterProfilerSymbols("pkg/data/dart_jit_runner.dartprofilersymbols", "");
#endif  // defined(AOT_RUNTIME)
#endif  // !defined(DART_PRODUCT)

  dart_runner::DartRunner runner(context.get());

  // Wait to serve until we have finished all of our setup.
  context->outgoing()->ServeFromStartupInfo();

  loop.Run();
  return 0;
}
