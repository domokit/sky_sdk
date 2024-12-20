// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fidl/fuchsia.diagnostics/cpp/fidl.h>
#include <fidl/fuchsia.logger/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/async-loop/testing/cpp/real_loop.h>
#include <lib/async/dispatcher.h>
#include <lib/component/incoming/cpp/protocol.h>
#include <lib/fidl/cpp/client.h>
#include <lib/fidl/cpp/wire/channel.h>
#include <lib/fidl/cpp/wire/connect_service.h>
#include <lib/fit/defer.h>
#include <lib/fit/result.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>
#include <zircon/errors.h>

#include "flutter/fml/log_settings.h"
#include "flutter/fml/platform/fuchsia/log_interest_listener.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

constexpr static char kLogSink[] = "log_sink";

class LogInterestListenerFuchsia : public ::loop_fixture::RealLoop,
                                   public ::testing::Test {};

TEST_F(LogInterestListenerFuchsia, SeverityChanges) {
  ScopedSetLogSettings backup({.min_log_level = kLogInfo});
  {
    ::fuchsia_diagnostics::Interest interest;
    interest.min_severity(::fuchsia_diagnostics::Severity::kTrace);
    LogInterestListener::HandleInterestChange(interest);
    EXPECT_EQ(GetMinLogLevel(), -1);  // VERBOSE
  }
  {
    ::fuchsia_diagnostics::Interest interest;
    interest.min_severity(::fuchsia_diagnostics::Severity::kInfo);
    LogInterestListener::HandleInterestChange(interest);
    EXPECT_EQ(GetMinLogLevel(), kLogInfo);
  }
  {
    ::fuchsia_diagnostics::Interest interest;
    interest.min_severity(::fuchsia_diagnostics::Severity::kError);
    LogInterestListener::HandleInterestChange(interest);
    EXPECT_EQ(GetMinLogLevel(), kLogError);
  }
}

// Class to mock the server end of a LogSink.
class MockLogSink : public component_testing::LocalComponentImpl,
                    public fidl::Server<fuchsia_logger::LogSink> {
 public:
  MockLogSink(fit::closure quitLoop, async_dispatcher_t* dispatcher)
      : quit_loop_(std::move(quitLoop)), dispatcher_(dispatcher) {}

  void WaitForInterestChange(
      WaitForInterestChangeCompleter::Sync& completer) override {
    if (first_call_) {
      // If it has not been called before, then return a result right away.
      fuchsia_logger::LogSinkWaitForInterestChangeResponse response = {
          {.data = {{.min_severity = fuchsia_diagnostics::Severity::kWarn}}}};
      completer.Reply(fit::ok(response));
      first_call_ = false;
    } else {
      // On the second call, don't return a result.
      completer_.emplace(completer.ToAsync());
      quit_loop_();
    }
  }

  void Connect(fuchsia_logger::LogSinkConnectRequest& request,
               ConnectCompleter::Sync& completer) override {}

  void ConnectStructured(
      fuchsia_logger::LogSinkConnectStructuredRequest& request,
      ConnectStructuredCompleter::Sync& completer) override {}

  void OnStart() override {
    ASSERT_EQ(outgoing()->AddProtocol<fuchsia_logger::LogSink>(
                  bindings_.CreateHandler(this, dispatcher_,
                                          fidl::kIgnoreBindingClosure)),
              ZX_OK);
  }

 private:
  bool first_call_ = true;
  fit::closure quit_loop_;
  async_dispatcher_t* dispatcher_;
  fidl::ServerBindingGroup<fuchsia_logger::LogSink> bindings_;
  std::optional<WaitForInterestChangeCompleter::Async> completer_;
};

TEST_F(LogInterestListenerFuchsia, AsyncWaitForInterestChange) {
  ScopedSetLogSettings backup({.min_log_level = kLogInfo});
  auto realm_builder = component_testing::RealmBuilder::Create();
  realm_builder.AddLocalChild(kLogSink, [&]() {
    return std::make_unique<MockLogSink>(QuitLoopClosure(), dispatcher());
  });
  realm_builder.AddRoute(component_testing::Route{
      .capabilities = {component_testing::Protocol{
          fidl::DiscoverableProtocolName<fuchsia_logger::LogSink>}},
      .source = component_testing::ChildRef{kLogSink},
      .targets = {component_testing::ParentRef()}});

  auto realm = realm_builder.Build(dispatcher());
  auto cleanup = fit::defer([&]() {
    bool complete = false;
    realm.Teardown([&](auto result) { complete = true; });
    RunLoopUntil([&]() { return complete; });
  });
  auto client_end = realm.component().Connect<fuchsia_logger::LogSink>();
  ASSERT_TRUE(client_end.is_ok());
  LogInterestListener listener(std::move(client_end.value()), dispatcher());
  listener.AsyncWaitForInterestChanged();
  RunLoop();

  EXPECT_EQ(GetMinLogLevel(), kLogWarning);
}

}  // namespace testing
}  // namespace fml
