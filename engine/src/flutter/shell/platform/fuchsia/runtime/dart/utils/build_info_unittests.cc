// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "build_info.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/inspect/cpp/reader.h>

const std::string& inspect_node_name = "build_info_unittests";

void checkProperty(inspect::Hierarchy& root,
                   const std::string& name,
                   const std::string& expected_value) {
  const inspect::Hierarchy* build_info = root.GetByPath({inspect_node_name});
  EXPECT_TRUE(build_info != nullptr);
  auto* actual_value =
      build_info->node().get_property<inspect::StringPropertyValue>(name);
  EXPECT_TRUE(actual_value != nullptr);
  EXPECT_EQ(actual_value->value(), expected_value);
}

namespace dart_utils {

class BuildInfoTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    async::Loop loop(&kAsyncLoopConfigAttachToCurrentThread);
    auto context = sys::ComponentContext::Create();
    RootInspectNode::Initialize(context.get());
  }
};

TEST_F(BuildInfoTest, AllPropertiesAreDefined) {
  EXPECT_NE(BuildInfo::DartSdkGitRevision(), "{{DART_SDK_GIT_REVISION}}");
  EXPECT_NE(BuildInfo::DartSdkSemanticVersion(),
            "{{DART_SDK_SEMANTIC_VERSION}}");
  EXPECT_NE(BuildInfo::FlutterEngineGitRevision(),
            "{{FLUTTER_ENGINE_GIT_REVISION}}");
  EXPECT_NE(BuildInfo::FuchsiaSdkVersion(), "{{FUCHSIA_SDK_VERSION}}");
}

TEST_F(BuildInfoTest, AllPropertiesAreDumped) {
  inspect::Node node =
      dart_utils::RootInspectNode::CreateRootChild(inspect_node_name);
  BuildInfo::Dump(node);
  inspect::Hierarchy root =
      inspect::ReadFromVmo(
          std::move(
              dart_utils::RootInspectNode::GetInspector()->DuplicateVmo()))
          .take_value();
  checkProperty(root, "dart_sdk_git_revision", BuildInfo::DartSdkGitRevision());
  checkProperty(root, "dart_sdk_semantic_version",
                BuildInfo::DartSdkSemanticVersion());
  checkProperty(root, "flutter_engine_git_revision",
                BuildInfo::FlutterEngineGitRevision());
  checkProperty(root, "fuchsia_sdk_version", BuildInfo::FuchsiaSdkVersion());
}

}  // namespace dart_utils
