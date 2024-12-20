// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/runtime_controller.h"

#include <utility>

#include "flutter/common/settings.h"
#include "flutter/fml/trace_event.h"
#include "flutter/lib/ui/compositing/scene.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/runtime/dart_isolate_group_data.h"
#include "flutter/runtime/isolate_configuration.h"
#include "flutter/runtime/runtime_delegate.h"
#include "third_party/tonic/dart_message_handler.h"

namespace flutter {

RuntimeController::RuntimeController(RuntimeDelegate& p_client,
                                     const TaskRunners& task_runners)
    : client_(p_client),
      vm_(nullptr),
      context_(task_runners),
      pointer_data_packet_converter_(*this) {}

RuntimeController::RuntimeController(
    RuntimeDelegate& p_client,
    DartVM* p_vm,
    fml::RefPtr<const DartSnapshot> p_isolate_snapshot,
    const std::function<void(int64_t)>& p_idle_notification_callback,
    const PlatformData& p_platform_data,
    const fml::closure& p_isolate_create_callback,
    const fml::closure& p_isolate_shutdown_callback,
    std::shared_ptr<const fml::Mapping> p_persistent_isolate_data,
    const UIDartState::Context& p_context)
    : client_(p_client),
      vm_(p_vm),
      isolate_snapshot_(std::move(p_isolate_snapshot)),
      idle_notification_callback_(p_idle_notification_callback),
      platform_data_(p_platform_data),
      isolate_create_callback_(p_isolate_create_callback),
      isolate_shutdown_callback_(p_isolate_shutdown_callback),
      persistent_isolate_data_(std::move(p_persistent_isolate_data)),
      context_(p_context),
      pointer_data_packet_converter_(*this) {}

std::unique_ptr<RuntimeController> RuntimeController::Spawn(
    RuntimeDelegate& p_client,
    const std::string& advisory_script_uri,
    const std::string& advisory_script_entrypoint,
    const std::function<void(int64_t)>& p_idle_notification_callback,
    const fml::closure& p_isolate_create_callback,
    const fml::closure& p_isolate_shutdown_callback,
    const std::shared_ptr<const fml::Mapping>& p_persistent_isolate_data,
    fml::WeakPtr<IOManager> io_manager,
    fml::WeakPtr<ImageDecoder> image_decoder,
    fml::WeakPtr<ImageGeneratorRegistry> image_generator_registry,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate) const {
  UIDartState::Context spawned_context{context_.task_runners,
                                       std::move(snapshot_delegate),
                                       std::move(io_manager),
                                       context_.unref_queue,
                                       std::move(image_decoder),
                                       std::move(image_generator_registry),
                                       advisory_script_uri,
                                       advisory_script_entrypoint,
                                       context_.deterministic_rendering_enabled,
                                       context_.concurrent_task_runner,
                                       context_.enable_impeller,
                                       context_.runtime_stage_backend};
  auto result =
      std::make_unique<RuntimeController>(p_client,                      //
                                          vm_,                           //
                                          isolate_snapshot_,             //
                                          p_idle_notification_callback,  //
                                          platform_data_,                //
                                          p_isolate_create_callback,     //
                                          p_isolate_shutdown_callback,   //
                                          p_persistent_isolate_data,     //
                                          spawned_context);              //
  result->spawning_isolate_ = root_isolate_;
  return result;
}

RuntimeController::~RuntimeController() {
  FML_DCHECK(Dart_CurrentIsolate() == nullptr);
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (root_isolate) {
    root_isolate->SetReturnCodeCallback(nullptr);
    auto result = root_isolate->Shutdown();
    if (!result) {
      FML_DLOG(ERROR) << "Could not shutdown the root isolate.";
    }
    root_isolate_ = {};
  }
}

bool RuntimeController::IsRootIsolateRunning() const {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (root_isolate) {
    return root_isolate->GetPhase() == DartIsolate::Phase::Running;
  }
  return false;
}

std::unique_ptr<RuntimeController> RuntimeController::Clone() const {
  return std::make_unique<RuntimeController>(client_,                      //
                                             vm_,                          //
                                             isolate_snapshot_,            //
                                             idle_notification_callback_,  //
                                             platform_data_,               //
                                             isolate_create_callback_,     //
                                             isolate_shutdown_callback_,   //
                                             persistent_isolate_data_,     //
                                             context_                      //
  );
}

bool RuntimeController::FlushRuntimeStateToIsolate() {
  FML_DCHECK(!has_flushed_runtime_state_)
      << "FlushRuntimeStateToIsolate is called more than once somehow.";
  has_flushed_runtime_state_ = true;

  auto platform_configuration = GetPlatformConfigurationIfAvailable();
  if (!platform_configuration) {
    return false;
  }

  for (auto const& [view_id, viewport_metrics] :
       platform_data_.viewport_metrics_for_views) {
    bool added = platform_configuration->AddView(view_id, viewport_metrics);

    // Callbacks will have been already invoked if the engine was restarted.
    if (pending_add_view_callbacks_.find(view_id) !=
        pending_add_view_callbacks_.end()) {
      pending_add_view_callbacks_[view_id](added);
      pending_add_view_callbacks_.erase(view_id);
    }

    if (!added) {
      FML_LOG(ERROR) << "Failed to flush view #" << view_id
                     << ". The Dart isolate may be in an inconsistent state.";
    }
  }

  FML_DCHECK(pending_add_view_callbacks_.empty());
  return SetLocales(platform_data_.locale_data) &&
         SetSemanticsEnabled(platform_data_.semantics_enabled) &&
         SetAccessibilityFeatures(
             platform_data_.accessibility_feature_flags_) &&
         SetUserSettingsData(platform_data_.user_settings_data) &&
         SetInitialLifecycleState(platform_data_.lifecycle_state) &&
         SetDisplays(platform_data_.displays);
}

void RuntimeController::AddView(int64_t view_id,
                                const ViewportMetrics& view_metrics,
                                AddViewCallback callback) {
  // If the Dart isolate is not running, |FlushRuntimeStateToIsolate| will
  // add the view and invoke the callback when the isolate is started.
  auto* platform_configuration = GetPlatformConfigurationIfAvailable();
  if (!platform_configuration) {
    FML_DCHECK(has_flushed_runtime_state_ == false);

    if (pending_add_view_callbacks_.find(view_id) !=
        pending_add_view_callbacks_.end()) {
      FML_LOG(ERROR) << "View #" << view_id << " is already pending creation.";
      callback(false);
      return;
    }

    platform_data_.viewport_metrics_for_views[view_id] = view_metrics;
    pending_add_view_callbacks_[view_id] = std::move(callback);
    return;
  }

  FML_DCHECK(has_flushed_runtime_state_ || pending_add_view_callbacks_.empty());

  platform_data_.viewport_metrics_for_views[view_id] = view_metrics;
  bool added = platform_configuration->AddView(view_id, view_metrics);
  if (added) {
    ScheduleFrame();
  }

  callback(added);
}

bool RuntimeController::RemoveView(int64_t view_id) {
  platform_data_.viewport_metrics_for_views.erase(view_id);

  // If the Dart isolate has not been launched yet, the pending
  // add view operation's callback is stored by the runtime controller.
  // Notify this callback of the cancellation.
  auto* platform_configuration = GetPlatformConfigurationIfAvailable();
  if (!platform_configuration) {
    FML_DCHECK(has_flushed_runtime_state_ == false);
    if (pending_add_view_callbacks_.find(view_id) !=
        pending_add_view_callbacks_.end()) {
      pending_add_view_callbacks_[view_id](false);
      pending_add_view_callbacks_.erase(view_id);
    }

    return false;
  }

  return platform_configuration->RemoveView(view_id);
}

bool RuntimeController::ViewExists(int64_t view_id) const {
  return platform_data_.viewport_metrics_for_views.count(view_id) != 0;
}

bool RuntimeController::SetViewportMetrics(int64_t view_id,
                                           const ViewportMetrics& metrics) {
  TRACE_EVENT0("flutter", "SetViewportMetrics");

  platform_data_.viewport_metrics_for_views[view_id] = metrics;
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    return platform_configuration->UpdateViewMetrics(view_id, metrics);
  }

  return false;
}

bool RuntimeController::SetLocales(
    const std::vector<std::string>& locale_data) {
  platform_data_.locale_data = locale_data;

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateLocales(locale_data);
    return true;
  }

  return false;
}

bool RuntimeController::SetUserSettingsData(const std::string& data) {
  platform_data_.user_settings_data = data;

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateUserSettingsData(
        platform_data_.user_settings_data);
    return true;
  }

  return false;
}

bool RuntimeController::SetInitialLifecycleState(const std::string& data) {
  platform_data_.lifecycle_state = data;

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateInitialLifecycleState(
        platform_data_.lifecycle_state);
    return true;
  }

  return false;
}

bool RuntimeController::SetSemanticsEnabled(bool enabled) {
  platform_data_.semantics_enabled = enabled;

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateSemanticsEnabled(
        platform_data_.semantics_enabled);
    return true;
  }

  return false;
}

bool RuntimeController::SetAccessibilityFeatures(int32_t flags) {
  platform_data_.accessibility_feature_flags_ = flags;
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateAccessibilityFeatures(
        platform_data_.accessibility_feature_flags_);
    return true;
  }

  return false;
}

bool RuntimeController::BeginFrame(fml::TimePoint frame_time,
                                   uint64_t frame_number) {
  MarkAsFrameBorder();
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->BeginFrame(frame_time, frame_number);
    return true;
  }

  return false;
}

bool RuntimeController::ReportTimings(std::vector<int64_t> timings) {
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->ReportTimings(std::move(timings));
    return true;
  }

  return false;
}

bool RuntimeController::NotifyIdle(fml::TimeDelta deadline) {
  if (deadline - fml::TimeDelta::FromMicroseconds(Dart_TimelineGetMicros()) <
      fml::TimeDelta::FromMilliseconds(1)) {
    // There's less than 1ms left before the deadline. Upstream callers do not
    // check to see if the deadline is in the past, and work after this point
    // will be in vain.
    return false;
  }

  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (!root_isolate) {
    return false;
  }

  tonic::DartState::Scope scope(root_isolate);

  Dart_PerformanceMode performance_mode =
      PlatformConfigurationNativeApi::GetDartPerformanceMode();
  if (performance_mode == Dart_PerformanceMode::Dart_PerformanceMode_Latency) {
    return false;
  }

  Dart_NotifyIdle(deadline.ToMicroseconds());

  // Idle notifications being in isolate scope are part of the contract.
  if (idle_notification_callback_) {
    TRACE_EVENT0("flutter", "EmbedderIdleNotification");
    idle_notification_callback_(deadline.ToMicroseconds());
  }
  return true;
}

bool RuntimeController::NotifyDestroyed() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (!root_isolate) {
    return false;
  }

  tonic::DartState::Scope scope(root_isolate);

  Dart_NotifyDestroyed();

  return true;
}

bool RuntimeController::DispatchPlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    TRACE_EVENT0("flutter", "RuntimeController::DispatchPlatformMessage");
    platform_configuration->DispatchPlatformMessage(std::move(message));
    return true;
  }

  return false;
}

bool RuntimeController::DispatchPointerDataPacket(
    const PointerDataPacket& packet) {
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    TRACE_EVENT0("flutter", "RuntimeController::DispatchPointerDataPacket");
    std::unique_ptr<PointerDataPacket> converted_packet =
        pointer_data_packet_converter_.Convert(packet);
    if (converted_packet->GetLength() != 0) {
      platform_configuration->DispatchPointerDataPacket(*converted_packet);
    }
    return true;
  }

  return false;
}

bool RuntimeController::DispatchSemanticsAction(int32_t node_id,
                                                SemanticsAction action,
                                                fml::MallocMapping args) {
  TRACE_EVENT1("flutter", "RuntimeController::DispatchSemanticsAction", "mode",
               "basic");
  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->DispatchSemanticsAction(node_id, action,
                                                    std::move(args));
    return true;
  }

  return false;
}

PlatformConfiguration*
RuntimeController::GetPlatformConfigurationIfAvailable() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  return root_isolate ? root_isolate->platform_configuration() : nullptr;
}

// |PlatformConfigurationClient|
std::string RuntimeController::DefaultRouteName() {
  return client_.DefaultRouteName();
}

// |PlatformConfigurationClient|
void RuntimeController::ScheduleFrame() {
  client_.ScheduleFrame();
}

void RuntimeController::EndWarmUpFrame() {
  client_.OnAllViewsRendered();
}

// |PlatformConfigurationClient|
void RuntimeController::Render(int64_t view_id,
                               Scene* scene,
                               double width,
                               double height) {
  const ViewportMetrics* view_metrics =
      UIDartState::Current()->platform_configuration()->GetMetrics(view_id);
  if (view_metrics == nullptr) {
    return;
  }
  client_.Render(view_id, scene->takeLayerTree(width, height),
                 view_metrics->device_pixel_ratio);
  rendered_views_during_frame_.insert(view_id);
  CheckIfAllViewsRendered();
}

void RuntimeController::MarkAsFrameBorder() {
  rendered_views_during_frame_.clear();
}

void RuntimeController::CheckIfAllViewsRendered() {
  if (rendered_views_during_frame_.size() != 0 &&
      rendered_views_during_frame_.size() ==
          platform_data_.viewport_metrics_for_views.size()) {
    client_.OnAllViewsRendered();
    MarkAsFrameBorder();
  }
}

// |PlatformConfigurationClient|
void RuntimeController::UpdateSemantics(SemanticsUpdate* update) {
  if (platform_data_.semantics_enabled) {
    client_.UpdateSemantics(update->takeNodes(), update->takeActions());
  }
}

// |PlatformConfigurationClient|
void RuntimeController::HandlePlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  client_.HandlePlatformMessage(std::move(message));
}

// |PlatformConfigurationClient|
FontCollection& RuntimeController::GetFontCollection() {
  return client_.GetFontCollection();
}

// |PlatfromConfigurationClient|
std::shared_ptr<AssetManager> RuntimeController::GetAssetManager() {
  return client_.GetAssetManager();
}

// |PlatformConfigurationClient|
void RuntimeController::UpdateIsolateDescription(const std::string isolate_name,
                                                 int64_t isolate_port) {
  client_.UpdateIsolateDescription(isolate_name, isolate_port);
}

// |PlatformConfigurationClient|
void RuntimeController::SetNeedsReportTimings(bool value) {
  client_.SetNeedsReportTimings(value);
}

// |PlatformConfigurationClient|
std::shared_ptr<const fml::Mapping>
RuntimeController::GetPersistentIsolateData() {
  return persistent_isolate_data_;
}

// |PlatformConfigurationClient|
std::unique_ptr<std::vector<std::string>>
RuntimeController::ComputePlatformResolvedLocale(
    const std::vector<std::string>& supported_locale_data) {
  return client_.ComputePlatformResolvedLocale(supported_locale_data);
}

// |PlatformConfigurationClient|
void RuntimeController::SendChannelUpdate(std::string name, bool listening) {
  client_.SendChannelUpdate(std::move(name), listening);
}

Dart_Port RuntimeController::GetMainPort() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  return root_isolate ? root_isolate->main_port() : ILLEGAL_PORT;
}

std::string RuntimeController::GetIsolateName() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  return root_isolate ? root_isolate->debug_name() : "";
}

bool RuntimeController::HasLivePorts() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (!root_isolate) {
    return false;
  }
  tonic::DartState::Scope scope(root_isolate);
  return Dart_HasLivePorts();
}

bool RuntimeController::HasPendingMicrotasks() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  if (!root_isolate) {
    return false;
  }
  return root_isolate->HasPendingMicrotasks();
}

tonic::DartErrorHandleType RuntimeController::GetLastError() {
  std::shared_ptr<DartIsolate> root_isolate = root_isolate_.lock();
  return root_isolate ? root_isolate->GetLastError() : tonic::kNoError;
}

bool RuntimeController::LaunchRootIsolate(
    const Settings& settings,
    const fml::closure& root_isolate_create_callback,
    std::optional<std::string> dart_entrypoint,
    std::optional<std::string> dart_entrypoint_library,
    const std::vector<std::string>& dart_entrypoint_args,
    std::unique_ptr<IsolateConfiguration> isolate_configuration,
    std::shared_ptr<NativeAssetsManager> native_assets_manager) {
  if (root_isolate_.lock()) {
    FML_LOG(ERROR) << "Root isolate was already running.";
    return false;
  }

  auto strong_root_isolate =
      DartIsolate::CreateRunningRootIsolate(
          settings,                                       //
          isolate_snapshot_,                              //
          std::make_unique<PlatformConfiguration>(this),  //
          DartIsolate::Flags{},                           //
          root_isolate_create_callback,                   //
          isolate_create_callback_,                       //
          isolate_shutdown_callback_,                     //
          std::move(dart_entrypoint),                     //
          std::move(dart_entrypoint_library),             //
          dart_entrypoint_args,                           //
          std::move(isolate_configuration),               //
          context_,                                       //
          spawning_isolate_.lock().get(),
          std::move(native_assets_manager))  //
          .lock();

  if (!strong_root_isolate) {
    FML_LOG(ERROR) << "Could not create root isolate.";
    return false;
  }

  // Enable platform channels for background isolates.
  strong_root_isolate->GetIsolateGroupData().SetPlatformMessageHandler(
      strong_root_isolate->GetRootIsolateToken(),
      client_.GetPlatformMessageHandler());

  // The root isolate ivar is weak.
  root_isolate_ = strong_root_isolate;

  // Capture by `this` here is safe because the callback is made by the dart
  // state itself. The isolate (and its Dart state) is owned by this object and
  // it will be collected before this object.
  strong_root_isolate->SetReturnCodeCallback(
      [this](uint32_t code) { root_isolate_return_code_ = code; });

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    tonic::DartState::Scope scope(strong_root_isolate);
    platform_configuration->DidCreateIsolate();
    if (!FlushRuntimeStateToIsolate()) {
      FML_DLOG(ERROR) << "Could not set up initial isolate state.";
    }
  } else {
    FML_DCHECK(false) << "RuntimeController created without window binding.";
  }

  FML_DCHECK(Dart_CurrentIsolate() == nullptr);

  client_.OnRootIsolateCreated();

  return true;
}

std::optional<std::string> RuntimeController::GetRootIsolateServiceID() const {
  if (auto isolate = root_isolate_.lock()) {
    return isolate->GetServiceId();
  }
  return std::nullopt;
}

std::optional<uint32_t> RuntimeController::GetRootIsolateReturnCode() {
  return root_isolate_return_code_;
}

uint64_t RuntimeController::GetRootIsolateGroup() const {
  auto isolate = root_isolate_.lock();
  if (isolate) {
    auto isolate_scope = tonic::DartIsolateScope(isolate->isolate());
    Dart_IsolateGroup isolate_group = Dart_CurrentIsolateGroup();
    return reinterpret_cast<uint64_t>(isolate_group);
  } else {
    return 0;
  }
}

void RuntimeController::LoadDartDeferredLibrary(
    intptr_t loading_unit_id,
    std::unique_ptr<const fml::Mapping> snapshot_data,
    std::unique_ptr<const fml::Mapping> snapshot_instructions) {
  root_isolate_.lock()->LoadLoadingUnit(loading_unit_id,
                                        std::move(snapshot_data),
                                        std::move(snapshot_instructions));
}

void RuntimeController::LoadDartDeferredLibraryError(
    intptr_t loading_unit_id,
    const std::string
        error_message,  // NOLINT(performance-unnecessary-value-param)
    bool transient) {
  root_isolate_.lock()->LoadLoadingUnitError(loading_unit_id, error_message,
                                             transient);
}

void RuntimeController::RequestDartDeferredLibrary(intptr_t loading_unit_id) {
  return client_.RequestDartDeferredLibrary(loading_unit_id);
}

bool RuntimeController::SetDisplays(const std::vector<DisplayData>& displays) {
  TRACE_EVENT0("flutter", "SetDisplays");
  platform_data_.displays = displays;

  if (auto* platform_configuration = GetPlatformConfigurationIfAvailable()) {
    platform_configuration->UpdateDisplays(displays);
    return true;
  }
  return false;
}

double RuntimeController::GetScaledFontSize(double unscaled_font_size,
                                            int configuration_id) const {
  return client_.GetScaledFontSize(unscaled_font_size, configuration_id);
}

void RuntimeController::ShutdownPlatformIsolates() {
  platform_isolate_manager_->ShutdownPlatformIsolates();
}

RuntimeController::Locale::Locale(std::string language_code_,
                                  std::string country_code_,
                                  std::string script_code_,
                                  std::string variant_code_)
    : language_code(std::move(language_code_)),
      country_code(std::move(country_code_)),
      script_code(std::move(script_code_)),
      variant_code(std::move(variant_code_)) {}

RuntimeController::Locale::~Locale() = default;

}  // namespace flutter
