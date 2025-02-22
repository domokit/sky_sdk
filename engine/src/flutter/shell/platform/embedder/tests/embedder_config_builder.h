// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_CONFIG_BUILDER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_CONFIG_BUILDER_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/unique_object.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_context_software.h"

namespace flutter::testing {

struct UniqueEngineTraits {
  static FlutterEngine InvalidValue() { return nullptr; }

  static bool IsValid(const FlutterEngine& value) { return value != nullptr; }

  static void Free(FlutterEngine& engine) {
    auto result = FlutterEngineShutdown(engine);
    FML_CHECK(result == kSuccess);
  }
};

using UniqueEngine = fml::UniqueObject<FlutterEngine, UniqueEngineTraits>;

class EmbedderConfigBuilder {
 public:
  enum class InitializationPreference {
    kSnapshotsInitialize,
    kAOTDataInitialize,
    kMultiAOTInitialize,
    kNoInitialize,
  };

  explicit EmbedderConfigBuilder(
      EmbedderTestContext& context,
      InitializationPreference preference =
          InitializationPreference::kSnapshotsInitialize);

  ~EmbedderConfigBuilder();

  FlutterProjectArgs& GetProjectArgs();

  void SetAssetsPath();

  void SetSnapshots();

  void SetAOTDataElf();

  void SetIsolateCreateCallbackHook();

  void SetSemanticsCallbackHooks();

  // Used to set a custom log message handler.
  void SetLogMessageCallbackHook();

  void SetChannelUpdateCallbackHook();

  void SetViewFocusChangeRequestHook();

  // Used to set a custom log tag.
  void SetLogTag(std::string tag);

  void SetLocalizationCallbackHooks();

  void SetExecutableName(std::string executable_name);

  void SetDartEntrypoint(std::string entrypoint);

  void AddCommandLineArgument(std::string arg);

  void AddDartEntrypointArgument(std::string arg);

  void SetPlatformTaskRunner(const FlutterTaskRunnerDescription* runner);

  void SetUITaskRunner(const FlutterTaskRunnerDescription* runner);

  void SetRenderTaskRunner(const FlutterTaskRunnerDescription* runner);

  void SetPlatformMessageCallback(
      const std::function<void(const FlutterPlatformMessage*)>& callback);

  void SetViewFocusChangeRequestCallback(
      const std::function<void(const FlutterViewFocusChangeRequest*)>&
          callback);

  void SetCompositor(bool avoid_backing_store_cache = false,
                     bool use_present_layers_callback = false);

  FlutterCompositor& GetCompositor();

  void SetSurface(SkISize surface_size) { context_.SetSurface(surface_size); }

  void SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType type,
      FlutterSoftwarePixelFormat software_pixfmt =
          kFlutterSoftwarePixelFormatNative32);

  UniqueEngine LaunchEngine() const;

  UniqueEngine InitializeEngine() const;

  // Sets up the callback for vsync, the callbacks needs to be specified on the
  // text context vis `SetVsyncCallback`.
  void SetupVsyncCallback();

  void SetViewFocusChangeRequestCallback(
      const FlutterViewFocusChangeRequestCallback& callback);

 private:
  EmbedderTestContext& context_;
  FlutterProjectArgs project_args_ = {};
  std::string dart_entrypoint_;
  FlutterCustomTaskRunners custom_task_runners_ = {};
  FlutterCompositor compositor_ = {};
  std::vector<std::string> command_line_arguments_;
  std::vector<std::string> dart_entrypoint_arguments_;
  std::string log_tag_;

  UniqueEngine SetupEngine(bool run) const;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderConfigBuilder);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_CONFIG_BUILDER_H_
