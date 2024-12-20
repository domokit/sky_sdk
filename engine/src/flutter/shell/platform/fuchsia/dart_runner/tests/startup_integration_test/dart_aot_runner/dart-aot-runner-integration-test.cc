// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <dart/test/cpp/fidl.h>
#include <fuchsia/tracing/provider/cpp/fidl.h>
#include <lib/async-loop/testing/cpp/real_loop.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>

#include "flutter/fml/logging.h"
#include "gtest/gtest.h"

namespace dart_aot_runner_testing::testing {
namespace {

// Types imported for the realm_builder library
using component_testing::ChildOptions;
using component_testing::ChildRef;
using component_testing::Directory;
using component_testing::ParentRef;
using component_testing::Protocol;
using component_testing::RealmBuilder;
using component_testing::RealmRoot;
using component_testing::Route;

constexpr auto kDartRunnerEnvironment = "dart_runner_env";

constexpr auto kDartAotRunner = "dart_aot_runner";
constexpr auto kDartAotRunnerRef = ChildRef{kDartAotRunner};
constexpr auto kDartAotRunnerUrl =
    "fuchsia-pkg://fuchsia.com/oot_dart_aot_runner#meta/"
    "dart_aot_runner.cm";

constexpr auto kDartAotEchoServer = "dart_aot_echo_server";
constexpr auto kDartAotEchoServerRef = ChildRef{kDartAotEchoServer};
constexpr auto kDartAotEchoServerUrl =
    "fuchsia-pkg://fuchsia.com/dart_aot_echo_server#meta/"
    "dart_aot_echo_server.cm";

class RealmBuilderTest : public ::loop_fixture::RealLoop,
                         public ::testing::Test {
 public:
  RealmBuilderTest() = default;
};

TEST_F(RealmBuilderTest, DartRunnerStartsUp) {
  auto realm_builder = RealmBuilder::Create();
  // Add Dart AOT runner as a child of RealmBuilder
  realm_builder.AddChild(kDartAotRunner, kDartAotRunnerUrl);

  // Add environment providing the Dart AOT runner
  fuchsia::component::decl::Environment dart_runner_environment;
  dart_runner_environment.set_name(kDartRunnerEnvironment);
  dart_runner_environment.set_extends(
      fuchsia::component::decl::EnvironmentExtends::REALM);
  dart_runner_environment.set_runners({});
  auto environment_runners = dart_runner_environment.mutable_runners();

  fuchsia::component::decl::RunnerRegistration dart_aot_runner_reg;
  dart_aot_runner_reg.set_source(fuchsia::component::decl::Ref::WithChild(
      fuchsia::component::decl::ChildRef{.name = kDartAotRunner}));
  dart_aot_runner_reg.set_source_name(kDartAotRunner);
  dart_aot_runner_reg.set_target_name(kDartAotRunner);
  environment_runners->push_back(std::move(dart_aot_runner_reg));

  auto realm_decl = realm_builder.GetRealmDecl();
  if (!realm_decl.has_environments()) {
    realm_decl.set_environments({});
  }
  auto realm_environments = realm_decl.mutable_environments();
  realm_environments->push_back(std::move(dart_runner_environment));
  realm_builder.ReplaceRealmDecl(std::move(realm_decl));

  // Add Dart server component as a child of Realm Builder
  realm_builder.AddChild(kDartAotEchoServer, kDartAotEchoServerUrl,
                         ChildOptions{.environment = kDartRunnerEnvironment});

  // Route base capabilities to the Dart AOT runner
  realm_builder.AddRoute(
      Route{.capabilities = {Protocol{"fuchsia.logger.LogSink"},
                             Protocol{"fuchsia.tracing.provider.Registry"},
                             Protocol{"fuchsia.posix.socket.Provider"},
                             Protocol{"fuchsia.intl.PropertyProvider"},
                             Protocol{"fuchsia.vulkan.loader.Loader"},
                             Protocol{"fuchsia.inspect.InspectSink"},
                             Directory{"config-data"}},
            .source = ParentRef(),
            .targets = {kDartAotRunnerRef, kDartAotEchoServerRef}});

  // Route the Echo FIDL protocol, this allows the Dart echo server to
  // communicate with the Realm Builder
  realm_builder.AddRoute(Route{.capabilities = {Protocol{"dart.test.Echo"}},
                               .source = kDartAotEchoServerRef,
                               .targets = {ParentRef()}});

  // Build the Realm with the provided child and protocols
  auto realm = realm_builder.Build(dispatcher());
  FML_LOG(INFO) << "Realm built: " << realm.component().GetChildName();
  // Connect to the Dart echo server
  auto echo = realm.component().ConnectSync<dart::test::Echo>();
  fidl::StringPtr response;
  // Attempt to ping the Dart echo server for a response
  echo->EchoString("hello", &response);
  ASSERT_EQ(response, "hello");
}

}  // namespace
}  // namespace dart_aot_runner_testing::testing
