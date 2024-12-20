// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <lib/async-loop/cpp/loop.h>
#include <lib/inspect/component/cpp/component.h>
#include <lib/trace-provider/provider.h>
#include <lib/trace/event.h>

#include <cstdlib>

#include "fml/message_loop.h"
#include "fml/platform/fuchsia/log_interest_listener.h"
#include "fml/platform/fuchsia/log_state.h"
#include "lib/async/default.h"
#include "logging.h"
#include "platform/utils.h"
#include "runner.h"
#include "runtime/dart/utils/build_info.h"
#include "runtime/dart/utils/root_inspect_node.h"
#include "runtime/dart/utils/tempfs.h"

int main(int argc, char const* argv[]) {
  fml::MessageLoop::EnsureInitializedForCurrentThread();

  // Setup logging.
  fml::LogState::Default().SetTags({LOG_TAG});
  fml::LogInterestListener listener(fml::LogState::Default().TakeClientEnd(),
                                    async_get_default_dispatcher());
  listener.AsyncWaitForInterestChanged();

  // Create our component context which is served later.
  auto context = sys::ComponentContext::Create();
  dart_utils::RootInspectNode::Initialize(context.get());
  auto build_info = dart_utils::RootInspectNode::CreateRootChild("build_info");
  dart_utils::BuildInfo::Dump(build_info);

  // We inject the 'vm' node into the dart vm so that it can add any inspect
  // data that it needs to the inspect tree.
  dart::SetDartVmNode(std::make_unique<inspect::Node>(
      dart_utils::RootInspectNode::CreateRootChild("vm")));

  std::unique_ptr<trace::TraceProviderWithFdio> provider;
  {
    bool already_started;
    // Use CreateSynchronously to prevent loss of early events.
    trace::TraceProviderWithFdio::CreateSynchronously(
        async_get_default_dispatcher(), "flutter_runner", &provider,
        &already_started);
  }

  fml::MessageLoop& loop = fml::MessageLoop::GetCurrent();
  flutter_runner::Runner runner(loop.GetTaskRunner(), context.get());

  // Wait to serve until we have finished all of our setup.
  context->outgoing()->ServeFromStartupInfo();

  loop.Run();

  return EXIT_SUCCESS;
}
