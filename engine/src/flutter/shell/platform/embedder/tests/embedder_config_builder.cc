// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_config_builder.h"

#include "flutter/common/constants.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "tests/embedder_test_context.h"
#include "third_party/skia/include/core/SkImage.h"

namespace flutter::testing {

EmbedderConfigBuilder::EmbedderConfigBuilder(
    EmbedderTestContext& context,
    InitializationPreference preference)
    : context_(context) {
  project_args_.struct_size = sizeof(project_args_);
  project_args_.shutdown_dart_vm_when_done = true;
  project_args_.platform_message_callback =
      [](const FlutterPlatformMessage* message, void* context) {
        reinterpret_cast<EmbedderTestContext*>(context)
            ->PlatformMessageCallback(message);
      };

  custom_task_runners_.struct_size = sizeof(FlutterCustomTaskRunners);

  // The first argument is always the executable name. Don't make tests have to
  // do this manually.
  AddCommandLineArgument("embedder_unittest");

  if (preference != InitializationPreference::kNoInitialize) {
    SetAssetsPath();
    SetIsolateCreateCallbackHook();
    SetSemanticsCallbackHooks();
    SetLogMessageCallbackHook();
    SetLocalizationCallbackHooks();
    SetChannelUpdateCallbackHook();
    SetViewFocusChangeRequestHook();
    AddCommandLineArgument("--disable-vm-service");

    if (preference == InitializationPreference::kSnapshotsInitialize ||
        preference == InitializationPreference::kMultiAOTInitialize) {
      SetSnapshots();
    }
    if (preference == InitializationPreference::kAOTDataInitialize ||
        preference == InitializationPreference::kMultiAOTInitialize) {
      SetAOTDataElf();
    }
  }
}

EmbedderConfigBuilder::~EmbedderConfigBuilder() = default;

FlutterProjectArgs& EmbedderConfigBuilder::GetProjectArgs() {
  return project_args_;
}

void EmbedderConfigBuilder::SetAssetsPath() {
  project_args_.assets_path = context_.GetAssetsPath().c_str();
}

void EmbedderConfigBuilder::SetSnapshots() {
  if (auto mapping = context_.GetVMSnapshotData()) {
    project_args_.vm_snapshot_data = mapping->GetMapping();
    project_args_.vm_snapshot_data_size = mapping->GetSize();
  }

  if (auto mapping = context_.GetVMSnapshotInstructions()) {
    project_args_.vm_snapshot_instructions = mapping->GetMapping();
    project_args_.vm_snapshot_instructions_size = mapping->GetSize();
  }

  if (auto mapping = context_.GetIsolateSnapshotData()) {
    project_args_.isolate_snapshot_data = mapping->GetMapping();
    project_args_.isolate_snapshot_data_size = mapping->GetSize();
  }

  if (auto mapping = context_.GetIsolateSnapshotInstructions()) {
    project_args_.isolate_snapshot_instructions = mapping->GetMapping();
    project_args_.isolate_snapshot_instructions_size = mapping->GetSize();
  }
}

void EmbedderConfigBuilder::SetAOTDataElf() {
  project_args_.aot_data = context_.GetAOTData();
}

void EmbedderConfigBuilder::SetIsolateCreateCallbackHook() {
  project_args_.root_isolate_create_callback =
      EmbedderTestContext::GetIsolateCreateCallbackHook();
}

void EmbedderConfigBuilder::SetSemanticsCallbackHooks() {
  project_args_.update_semantics_callback2 =
      context_.GetUpdateSemanticsCallback2Hook();
  project_args_.update_semantics_callback =
      context_.GetUpdateSemanticsCallbackHook();
  project_args_.update_semantics_node_callback =
      context_.GetUpdateSemanticsNodeCallbackHook();
  project_args_.update_semantics_custom_action_callback =
      context_.GetUpdateSemanticsCustomActionCallbackHook();
}

void EmbedderConfigBuilder::SetLogMessageCallbackHook() {
  project_args_.log_message_callback =
      EmbedderTestContext::GetLogMessageCallbackHook();
}

void EmbedderConfigBuilder::SetChannelUpdateCallbackHook() {
  project_args_.channel_update_callback =
      context_.GetChannelUpdateCallbackHook();
}

void EmbedderConfigBuilder::SetViewFocusChangeRequestHook() {
  project_args_.view_focus_change_request_callback =
      context_.GetViewFocusChangeRequestCallbackHook();
}

void EmbedderConfigBuilder::SetLogTag(std::string tag) {
  log_tag_ = std::move(tag);
  project_args_.log_tag = log_tag_.c_str();
}

void EmbedderConfigBuilder::SetLocalizationCallbackHooks() {
  project_args_.compute_platform_resolved_locale_callback =
      EmbedderTestContext::GetComputePlatformResolvedLocaleCallbackHook();
}

void EmbedderConfigBuilder::SetExecutableName(std::string executable_name) {
  if (executable_name.empty()) {
    return;
  }
  command_line_arguments_[0] = std::move(executable_name);
}

void EmbedderConfigBuilder::SetDartEntrypoint(std::string entrypoint) {
  if (entrypoint.empty()) {
    return;
  }

  dart_entrypoint_ = std::move(entrypoint);
  project_args_.custom_dart_entrypoint = dart_entrypoint_.c_str();
}

void EmbedderConfigBuilder::AddCommandLineArgument(std::string arg) {
  if (arg.empty()) {
    return;
  }

  command_line_arguments_.emplace_back(std::move(arg));
}

void EmbedderConfigBuilder::AddDartEntrypointArgument(std::string arg) {
  if (arg.empty()) {
    return;
  }

  dart_entrypoint_arguments_.emplace_back(std::move(arg));
}

void EmbedderConfigBuilder::SetPlatformTaskRunner(
    const FlutterTaskRunnerDescription* runner) {
  if (runner == nullptr) {
    return;
  }
  custom_task_runners_.platform_task_runner = runner;
  project_args_.custom_task_runners = &custom_task_runners_;
}

void EmbedderConfigBuilder::SetUITaskRunner(
    const FlutterTaskRunnerDescription* runner) {
  if (runner == nullptr) {
    return;
  }
  custom_task_runners_.ui_task_runner = runner;
  project_args_.custom_task_runners = &custom_task_runners_;
}

void EmbedderConfigBuilder::SetupVsyncCallback() {
  project_args_.vsync_callback = [](void* user_data, intptr_t baton) {
    auto context = reinterpret_cast<EmbedderTestContext*>(user_data);
    context->RunVsyncCallback(baton);
  };
}

void EmbedderConfigBuilder::SetRenderTaskRunner(
    const FlutterTaskRunnerDescription* runner) {
  if (runner == nullptr) {
    return;
  }

  custom_task_runners_.render_task_runner = runner;
  project_args_.custom_task_runners = &custom_task_runners_;
}

void EmbedderConfigBuilder::SetPlatformMessageCallback(
    const std::function<void(const FlutterPlatformMessage*)>& callback) {
  context_.SetPlatformMessageCallback(callback);
}

void EmbedderConfigBuilder::SetViewFocusChangeRequestCallback(
    const std::function<void(const FlutterViewFocusChangeRequest*)>& callback) {
  context_.SetViewFocusChangeRequestCallback(callback);
}

void EmbedderConfigBuilder::SetCompositor(bool avoid_backing_store_cache,
                                          bool use_present_layers_callback) {
  context_.SetupCompositor();
  auto& compositor = context_.GetCompositor();
  compositor_.struct_size = sizeof(compositor_);
  compositor_.user_data = &compositor;
  compositor_.create_backing_store_callback =
      [](const FlutterBackingStoreConfig* config,  //
         FlutterBackingStore* backing_store_out,   //
         void* user_data                           //
      ) {
        return reinterpret_cast<EmbedderTestCompositor*>(user_data)
            ->CreateBackingStore(config, backing_store_out);
      };
  compositor_.collect_backing_store_callback =
      [](const FlutterBackingStore* backing_store,  //
         void* user_data                            //
      ) {
        return reinterpret_cast<EmbedderTestCompositor*>(user_data)
            ->CollectBackingStore(backing_store);
      };
  if (use_present_layers_callback) {
    compositor_.present_layers_callback = [](const FlutterLayer** layers,
                                             size_t layers_count,
                                             void* user_data) {
      auto compositor = reinterpret_cast<EmbedderTestCompositor*>(user_data);

      // The present layers callback is incompatible with multiple views;
      // it can only be used to render the implicit view.
      return compositor->Present(kFlutterImplicitViewId, layers, layers_count);
    };
  } else {
    compositor_.present_view_callback = [](const FlutterPresentViewInfo* info) {
      auto compositor =
          reinterpret_cast<EmbedderTestCompositor*>(info->user_data);

      return compositor->Present(info->view_id, info->layers,
                                 info->layers_count);
    };
  }
  compositor_.avoid_backing_store_cache = avoid_backing_store_cache;
  project_args_.compositor = &compositor_;
}

FlutterCompositor& EmbedderConfigBuilder::GetCompositor() {
  return compositor_;
}

void EmbedderConfigBuilder::SetRenderTargetType(
    EmbedderTestBackingStoreProducer::RenderTargetType type,
    FlutterSoftwarePixelFormat software_pixfmt) {
  context_.GetCompositor().SetRenderTargetType(type, software_pixfmt);
}

UniqueEngine EmbedderConfigBuilder::LaunchEngine() const {
  return SetupEngine(true);
}

UniqueEngine EmbedderConfigBuilder::InitializeEngine() const {
  return SetupEngine(false);
}

UniqueEngine EmbedderConfigBuilder::SetupEngine(bool run) const {
  FlutterEngine engine = nullptr;
  FlutterProjectArgs project_args = project_args_;

  std::vector<const char*> args;
  args.reserve(command_line_arguments_.size());

  for (const auto& arg : command_line_arguments_) {
    args.push_back(arg.c_str());
  }

  if (!args.empty()) {
    project_args.command_line_argv = args.data();
    project_args.command_line_argc = args.size();
  } else {
    // Clear it out in case this is not the first engine launch from the
    // embedder config builder.
    project_args.command_line_argv = nullptr;
    project_args.command_line_argc = 0;
  }

  std::vector<const char*> dart_args;
  dart_args.reserve(dart_entrypoint_arguments_.size());

  for (const auto& arg : dart_entrypoint_arguments_) {
    dart_args.push_back(arg.c_str());
  }

  if (!dart_args.empty()) {
    project_args.dart_entrypoint_argv = dart_args.data();
    project_args.dart_entrypoint_argc = dart_args.size();
  } else {
    // Clear it out in case this is not the first engine launch from the
    // embedder config builder.
    project_args.dart_entrypoint_argv = nullptr;
    project_args.dart_entrypoint_argc = 0;
  }

  auto result = run ? FlutterEngineRun(FLUTTER_ENGINE_VERSION,
                                       &context_.GetRendererConfig(),
                                       &project_args, &context_, &engine)
                    : FlutterEngineInitialize(
                          FLUTTER_ENGINE_VERSION, &context_.GetRendererConfig(),
                          &project_args, &context_, &engine);

  if (result != kSuccess) {
    return {};
  }

  return UniqueEngine{engine};
}

}  // namespace flutter::testing
