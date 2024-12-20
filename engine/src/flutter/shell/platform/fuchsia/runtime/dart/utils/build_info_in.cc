// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "runtime/dart/utils/build_info.h"

namespace dart_utils {

const char* BuildInfo::DartSdkGitRevision() {
  return "{{DART_SDK_GIT_REVISION}}";
}

const char* BuildInfo::DartSdkSemanticVersion() {
  return "{{DART_SDK_SEMANTIC_VERSION}}";
}

const char* BuildInfo::FlutterEngineGitRevision() {
  return "{{FLUTTER_ENGINE_GIT_REVISION}}";
}

const char* BuildInfo::FuchsiaSdkVersion() {
  return "{{FUCHSIA_SDK_VERSION}}";
}

void BuildInfo::Dump(inspect::Node& node) {
  node.CreateString("dart_sdk_git_revision", DartSdkGitRevision(),
                    RootInspectNode::GetInspector());
  node.CreateString("dart_sdk_semantic_version", DartSdkSemanticVersion(),
                    RootInspectNode::GetInspector());
  node.CreateString("flutter_engine_git_revision", FlutterEngineGitRevision(),
                    RootInspectNode::GetInspector());
  node.CreateString("fuchsia_sdk_version", FuchsiaSdkVersion(),
                    RootInspectNode::GetInspector());
}

}  // namespace dart_utils
