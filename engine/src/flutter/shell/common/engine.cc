// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/engine.h"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "flutter/assets/native_assets.h"
#include "flutter/common/settings.h"
#include "flutter/fml/trace_event.h"
#include "flutter/lib/ui/text/font_collection.h"
#include "flutter/shell/common/animator.h"
#include "rapidjson/document.h"

namespace flutter {

static constexpr char kAssetChannel[] = "flutter/assets";
static constexpr char kLifecycleChannel[] = "flutter/lifecycle";
static constexpr char kNavigationChannel[] = "flutter/navigation";
static constexpr char kLocalizationChannel[] = "flutter/localization";
static constexpr char kSettingsChannel[] = "flutter/settings";
static constexpr char kIsolateChannel[] = "flutter/isolate";

namespace {
fml::MallocMapping MakeMapping(const std::string& str) {
  return fml::MallocMapping::Copy(str.c_str(), str.length());
}
}  // namespace

Engine::Engine(
    Delegate& delegate,
    const PointerDataDispatcherMaker& dispatcher_maker,
    const std::shared_ptr<fml::ConcurrentTaskRunner>& image_decoder_task_runner,
    const TaskRunners& task_runners,
    const Settings& settings,
    std::unique_ptr<Animator> animator,
    const fml::WeakPtr<IOManager>& io_manager,
    const std::shared_ptr<FontCollection>& font_collection,
    std::unique_ptr<RuntimeController> runtime_controller,
    const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch)
    : delegate_(delegate),
      settings_(settings),
      animator_(std::move(animator)),
      runtime_controller_(std::move(runtime_controller)),
      font_collection_(font_collection),
      image_decoder_(ImageDecoder::Make(settings_,
                                        task_runners,
                                        image_decoder_task_runner,
                                        io_manager,
                                        gpu_disabled_switch)),
      task_runners_(task_runners),
      weak_factory_(this) {
  pointer_data_dispatcher_ = dispatcher_maker(*this);
}

Engine::Engine(Delegate& delegate,
               const PointerDataDispatcherMaker& dispatcher_maker,
               DartVM& vm,
               fml::RefPtr<const DartSnapshot> isolate_snapshot,
               const TaskRunners& task_runners,
               const PlatformData& platform_data,
               const Settings& settings,
               std::unique_ptr<Animator> animator,
               fml::WeakPtr<IOManager> io_manager,
               const fml::RefPtr<SkiaUnrefQueue>& unref_queue,
               fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
               const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch,
               impeller::RuntimeStageBackend runtime_stage_type)
    : Engine(delegate,
             dispatcher_maker,
             vm.GetConcurrentWorkerTaskRunner(),
             task_runners,
             settings,
             std::move(animator),
             io_manager,
             std::make_shared<FontCollection>(),
             nullptr,
             gpu_disabled_switch) {
  runtime_controller_ = std::make_unique<RuntimeController>(
      *this,                                 // runtime delegate
      &vm,                                   // VM
      std::move(isolate_snapshot),           // isolate snapshot
      settings_.idle_notification_callback,  // idle notification callback
      platform_data,                         // platform data
      settings_.isolate_create_callback,     // isolate create callback
      settings_.isolate_shutdown_callback,   // isolate shutdown callback
      settings_.persistent_isolate_data,     // persistent isolate data
      UIDartState::Context{
          task_runners_,                           // task runners
          std::move(snapshot_delegate),            // snapshot delegate
          std::move(io_manager),                   // io manager
          unref_queue,                             // Skia unref queue
          image_decoder_->GetWeakPtr(),            // image decoder
          image_generator_registry_.GetWeakPtr(),  // image generator registry
          settings_.advisory_script_uri,           // advisory script uri
          settings_.advisory_script_entrypoint,    // advisory script entrypoint
          settings_
              .skia_deterministic_rendering_on_cpu,  // deterministic rendering
          vm.GetConcurrentWorkerTaskRunner(),        // concurrent task runner
          settings_.enable_impeller,                 // enable impeller
          runtime_stage_type,                        // runtime stage type
      });
}

std::unique_ptr<Engine> Engine::Spawn(
    Delegate& delegate,
    const PointerDataDispatcherMaker& dispatcher_maker,
    const Settings& settings,
    std::unique_ptr<Animator> animator,
    const std::string& initial_route,
    const fml::WeakPtr<IOManager>& io_manager,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch) const {
  auto result = std::make_unique<Engine>(
      /*delegate=*/delegate,
      /*dispatcher_maker=*/dispatcher_maker,
      /*image_decoder_task_runner=*/
      runtime_controller_->GetDartVM()->GetConcurrentWorkerTaskRunner(),
      /*task_runners=*/task_runners_,
      /*settings=*/settings,
      /*animator=*/std::move(animator),
      /*io_manager=*/io_manager,
      /*font_collection=*/font_collection_,
      /*runtime_controller=*/nullptr,
      /*gpu_disabled_switch=*/gpu_disabled_switch);
  result->runtime_controller_ = runtime_controller_->Spawn(
      /*p_client=*/*result,
      /*advisory_script_uri=*/settings.advisory_script_uri,
      /*advisory_script_entrypoint=*/settings.advisory_script_entrypoint,
      /*idle_notification_callback=*/settings.idle_notification_callback,
      /*isolate_create_callback=*/settings.isolate_create_callback,
      /*isolate_shutdown_callback=*/settings.isolate_shutdown_callback,
      /*persistent_isolate_data=*/settings.persistent_isolate_data,
      /*io_manager=*/io_manager,
      /*image_decoder=*/result->GetImageDecoderWeakPtr(),
      /*image_generator_registry=*/result->GetImageGeneratorRegistry(),
      /*snapshot_delegate=*/std::move(snapshot_delegate));
  result->initial_route_ = initial_route;
  result->asset_manager_ = asset_manager_;
  return result;
}

Engine::~Engine() = default;

fml::WeakPtr<Engine> Engine::GetWeakPtr() const {
  return weak_factory_.GetWeakPtr();
}

void Engine::SetupDefaultFontManager() {
  TRACE_EVENT0("flutter", "Engine::SetupDefaultFontManager");
  font_collection_->SetupDefaultFontManager(settings_.font_initialization_data);
}

std::shared_ptr<AssetManager> Engine::GetAssetManager() {
  return asset_manager_;
}

fml::WeakPtr<ImageDecoder> Engine::GetImageDecoderWeakPtr() {
  return image_decoder_->GetWeakPtr();
}

fml::WeakPtr<ImageGeneratorRegistry> Engine::GetImageGeneratorRegistry() {
  return image_generator_registry_.GetWeakPtr();
}

bool Engine::UpdateAssetManager(
    const std::shared_ptr<AssetManager>& new_asset_manager) {
  if (asset_manager_ && new_asset_manager &&
      *asset_manager_ == *new_asset_manager) {
    return false;
  }

  asset_manager_ = new_asset_manager;

  if (!asset_manager_) {
    return false;
  }

  // Using libTXT as the text engine.
  if (settings_.use_asset_fonts) {
    font_collection_->RegisterFonts(asset_manager_);
  }

  if (settings_.use_test_fonts) {
    font_collection_->RegisterTestFonts();
  }

  if (native_assets_manager_ == nullptr) {
    native_assets_manager_ = std::make_shared<NativeAssetsManager>();
  }
  native_assets_manager_->RegisterNativeAssets(asset_manager_);

  return true;
}

bool Engine::Restart(RunConfiguration configuration) {
  TRACE_EVENT0("flutter", "Engine::Restart");
  if (!configuration.IsValid()) {
    FML_LOG(ERROR) << "Engine run configuration was invalid.";
    return false;
  }
  delegate_.OnPreEngineRestart();
  runtime_controller_ = runtime_controller_->Clone();
  UpdateAssetManager(nullptr);
  return Run(std::move(configuration)) == Engine::RunStatus::Success;
}

Engine::RunStatus Engine::Run(RunConfiguration configuration) {
  if (!configuration.IsValid()) {
    FML_LOG(ERROR) << "Engine run configuration was invalid.";
    return RunStatus::Failure;
  }

  last_entry_point_ = configuration.GetEntrypoint();
  last_entry_point_library_ = configuration.GetEntrypointLibrary();
#if (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG)
  // This is only used to support restart.
  last_entry_point_args_ = configuration.GetEntrypointArgs();
#endif

  UpdateAssetManager(configuration.GetAssetManager());

  if (runtime_controller_->IsRootIsolateRunning()) {
    return RunStatus::FailureAlreadyRunning;
  }

  // If the embedding prefetched the default font manager, then set up the
  // font manager later in the engine launch process.  This makes it less
  // likely that the setup will need to wait for the prefetch to complete.
  auto root_isolate_create_callback = [&]() {
    if (settings_.prefetched_default_font_manager) {
      SetupDefaultFontManager();
    }
  };

  if (!runtime_controller_->LaunchRootIsolate(
          settings_,                                 //
          root_isolate_create_callback,              //
          configuration.GetEntrypoint(),             //
          configuration.GetEntrypointLibrary(),      //
          configuration.GetEntrypointArgs(),         //
          configuration.TakeIsolateConfiguration(),  //
          native_assets_manager_)                    //
  ) {
    return RunStatus::Failure;
  }

  auto service_id = runtime_controller_->GetRootIsolateServiceID();
  if (service_id.has_value()) {
    std::unique_ptr<PlatformMessage> service_id_message =
        std::make_unique<flutter::PlatformMessage>(
            kIsolateChannel, MakeMapping(service_id.value()), nullptr);
    HandlePlatformMessage(std::move(service_id_message));
  }

  return Engine::RunStatus::Success;
}

void Engine::BeginFrame(fml::TimePoint frame_time, uint64_t frame_number) {
  runtime_controller_->BeginFrame(frame_time, frame_number);
}

void Engine::ReportTimings(std::vector<int64_t> timings) {
  runtime_controller_->ReportTimings(std::move(timings));
}

void Engine::NotifyIdle(fml::TimeDelta deadline) {
  runtime_controller_->NotifyIdle(deadline);
}

void Engine::NotifyDestroyed() {
  TRACE_EVENT0("flutter", "Engine::NotifyDestroyed");
  runtime_controller_->NotifyDestroyed();
}

std::optional<uint32_t> Engine::GetUIIsolateReturnCode() {
  return runtime_controller_->GetRootIsolateReturnCode();
}

Dart_Port Engine::GetUIIsolateMainPort() {
  return runtime_controller_->GetMainPort();
}

std::string Engine::GetUIIsolateName() {
  return runtime_controller_->GetIsolateName();
}

bool Engine::UIIsolateHasLivePorts() {
  return runtime_controller_->HasLivePorts();
}

bool Engine::UIIsolateHasPendingMicrotasks() {
  return runtime_controller_->HasPendingMicrotasks();
}

tonic::DartErrorHandleType Engine::GetUIIsolateLastError() {
  return runtime_controller_->GetLastError();
}

void Engine::AddView(int64_t view_id,
                     const ViewportMetrics& view_metrics,
                     std::function<void(bool added)> callback) {
  runtime_controller_->AddView(view_id, view_metrics, std::move(callback));
}

bool Engine::RemoveView(int64_t view_id) {
  return runtime_controller_->RemoveView(view_id);
}

void Engine::SetViewportMetrics(int64_t view_id,
                                const ViewportMetrics& metrics) {
  runtime_controller_->SetViewportMetrics(view_id, metrics);
  ScheduleFrame();
}

void Engine::DispatchPlatformMessage(std::unique_ptr<PlatformMessage> message) {
  std::string channel = message->channel();
  if (channel == kLifecycleChannel) {
    if (HandleLifecyclePlatformMessage(message.get())) {
      return;
    }
  } else if (channel == kLocalizationChannel) {
    if (HandleLocalizationPlatformMessage(message.get())) {
      return;
    }
  } else if (channel == kSettingsChannel) {
    HandleSettingsPlatformMessage(message.get());
    return;
  } else if (!runtime_controller_->IsRootIsolateRunning() &&
             channel == kNavigationChannel) {
    // If there's no runtime_, we may still need to set the initial route.
    HandleNavigationPlatformMessage(std::move(message));
    return;
  }

  if (runtime_controller_->IsRootIsolateRunning() &&
      runtime_controller_->DispatchPlatformMessage(std::move(message))) {
    return;
  }

  FML_DLOG(WARNING) << "Dropping platform message on channel: " << channel;
}

bool Engine::HandleLifecyclePlatformMessage(PlatformMessage* message) {
  const auto& data = message->data();
  std::string state(reinterpret_cast<const char*>(data.GetMapping()),
                    data.GetSize());

  // Always schedule a frame when the app does become active as per API
  // recommendation
  // https://developer.apple.com/documentation/uikit/uiapplicationdelegate/1622956-applicationdidbecomeactive?language=objc
  if (state == "AppLifecycleState.resumed" ||
      state == "AppLifecycleState.inactive") {
    ScheduleFrame();
  }
  runtime_controller_->SetInitialLifecycleState(state);
  // Always forward these messages to the framework by returning false.
  return false;
}

bool Engine::HandleNavigationPlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  const auto& data = message->data();

  rapidjson::Document document;
  document.Parse(reinterpret_cast<const char*>(data.GetMapping()),
                 data.GetSize());
  if (document.HasParseError() || !document.IsObject()) {
    return false;
  }
  auto root = document.GetObject();
  auto method = root.FindMember("method");
  if (method->value != "setInitialRoute") {
    return false;
  }
  auto route = root.FindMember("args");
  initial_route_ = route->value.GetString();
  return true;
}

bool Engine::HandleLocalizationPlatformMessage(PlatformMessage* message) {
  const auto& data = message->data();

  rapidjson::Document document;
  document.Parse(reinterpret_cast<const char*>(data.GetMapping()),
                 data.GetSize());
  if (document.HasParseError() || !document.IsObject()) {
    return false;
  }
  auto root = document.GetObject();
  auto method = root.FindMember("method");
  if (method == root.MemberEnd()) {
    return false;
  }
  const size_t strings_per_locale = 4;
  if (method->value == "setLocale") {
    // Decode and pass the list of locale data onwards to dart.
    auto args = root.FindMember("args");
    if (args == root.MemberEnd() || !args->value.IsArray()) {
      return false;
    }

    if (args->value.Size() % strings_per_locale != 0) {
      return false;
    }
    std::vector<std::string> locale_data;
    for (size_t locale_index = 0; locale_index < args->value.Size();
         locale_index += strings_per_locale) {
      if (!args->value[locale_index].IsString() ||
          !args->value[locale_index + 1].IsString()) {
        return false;
      }
      locale_data.push_back(args->value[locale_index].GetString());
      locale_data.push_back(args->value[locale_index + 1].GetString());
      locale_data.push_back(args->value[locale_index + 2].GetString());
      locale_data.push_back(args->value[locale_index + 3].GetString());
    }

    return runtime_controller_->SetLocales(locale_data);
  }
  return false;
}

void Engine::HandleSettingsPlatformMessage(PlatformMessage* message) {
  const auto& data = message->data();
  std::string jsonData(reinterpret_cast<const char*>(data.GetMapping()),
                       data.GetSize());
  if (runtime_controller_->SetUserSettingsData(jsonData)) {
    ScheduleFrame();
  }
}

void Engine::DispatchPointerDataPacket(
    std::unique_ptr<PointerDataPacket> packet,
    uint64_t trace_flow_id) {
  TRACE_EVENT0_WITH_FLOW_IDS("flutter", "Engine::DispatchPointerDataPacket",
                             /*flow_id_count=*/1,
                             /*flow_ids=*/&trace_flow_id);
  TRACE_FLOW_STEP("flutter", "PointerEvent", trace_flow_id);
  pointer_data_dispatcher_->DispatchPacket(std::move(packet), trace_flow_id);
}

void Engine::DispatchSemanticsAction(int node_id,
                                     SemanticsAction action,
                                     fml::MallocMapping args) {
  runtime_controller_->DispatchSemanticsAction(node_id, action,
                                               std::move(args));
}

void Engine::SetSemanticsEnabled(bool enabled) {
  runtime_controller_->SetSemanticsEnabled(enabled);
}

void Engine::SetAccessibilityFeatures(int32_t flags) {
  runtime_controller_->SetAccessibilityFeatures(flags);
}

std::string Engine::DefaultRouteName() {
  if (!initial_route_.empty()) {
    return initial_route_;
  }
  return "/";
}

void Engine::ScheduleFrame(bool regenerate_layer_trees) {
  animator_->RequestFrame(regenerate_layer_trees);
}

void Engine::OnAllViewsRendered() {
  animator_->OnAllViewsRendered();
}

void Engine::Render(int64_t view_id,
                    std::unique_ptr<flutter::LayerTree> layer_tree,
                    float device_pixel_ratio) {
  if (!layer_tree) {
    return;
  }

  // Ensure frame dimensions are sane.
  if (layer_tree->frame_size().IsEmpty() || device_pixel_ratio <= 0.0f) {
    return;
  }

  animator_->Render(view_id, std::move(layer_tree), device_pixel_ratio);
}

void Engine::UpdateSemantics(SemanticsNodeUpdates update,
                             CustomAccessibilityActionUpdates actions) {
  delegate_.OnEngineUpdateSemantics(std::move(update), std::move(actions));
}

void Engine::HandlePlatformMessage(std::unique_ptr<PlatformMessage> message) {
  if (message->channel() == kAssetChannel) {
    HandleAssetPlatformMessage(std::move(message));
  } else {
    delegate_.OnEngineHandlePlatformMessage(std::move(message));
  }
}

void Engine::OnRootIsolateCreated() {
  delegate_.OnRootIsolateCreated();
}

void Engine::UpdateIsolateDescription(const std::string isolate_name,
                                      int64_t isolate_port) {
  delegate_.UpdateIsolateDescription(isolate_name, isolate_port);
}

std::unique_ptr<std::vector<std::string>> Engine::ComputePlatformResolvedLocale(
    const std::vector<std::string>& supported_locale_data) {
  return delegate_.ComputePlatformResolvedLocale(supported_locale_data);
}

double Engine::GetScaledFontSize(double unscaled_font_size,
                                 int configuration_id) const {
  return delegate_.GetScaledFontSize(unscaled_font_size, configuration_id);
}

void Engine::SetNeedsReportTimings(bool needs_reporting) {
  delegate_.SetNeedsReportTimings(needs_reporting);
}

FontCollection& Engine::GetFontCollection() {
  return *font_collection_;
}

void Engine::DoDispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                              uint64_t trace_flow_id) {
  animator_->EnqueueTraceFlowId(trace_flow_id);
  if (runtime_controller_) {
    runtime_controller_->DispatchPointerDataPacket(*packet);
  }
}

void Engine::ScheduleSecondaryVsyncCallback(uintptr_t id,
                                            const fml::closure& callback) {
  animator_->ScheduleSecondaryVsyncCallback(id, callback);
}

void Engine::HandleAssetPlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  fml::RefPtr<PlatformMessageResponse> response = message->response();
  if (!response) {
    return;
  }
  const auto& data = message->data();
  std::string asset_name(reinterpret_cast<const char*>(data.GetMapping()),
                         data.GetSize());

  if (asset_manager_) {
    std::unique_ptr<fml::Mapping> asset_mapping =
        asset_manager_->GetAsMapping(asset_name);
    if (asset_mapping) {
      response->Complete(std::move(asset_mapping));
      return;
    }
  }

  response->CompleteEmpty();
}

const std::string& Engine::GetLastEntrypoint() const {
  return last_entry_point_;
}

const std::string& Engine::GetLastEntrypointLibrary() const {
  return last_entry_point_library_;
}

const std::vector<std::string>& Engine::GetLastEntrypointArgs() const {
  return last_entry_point_args_;
}

// |RuntimeDelegate|
void Engine::RequestDartDeferredLibrary(intptr_t loading_unit_id) {
  return delegate_.RequestDartDeferredLibrary(loading_unit_id);
}

std::weak_ptr<PlatformMessageHandler> Engine::GetPlatformMessageHandler()
    const {
  return delegate_.GetPlatformMessageHandler();
}

void Engine::SendChannelUpdate(std::string name, bool listening) {
  delegate_.OnEngineChannelUpdate(std::move(name), listening);
}

void Engine::LoadDartDeferredLibrary(
    intptr_t loading_unit_id,
    std::unique_ptr<const fml::Mapping> snapshot_data,
    std::unique_ptr<const fml::Mapping> snapshot_instructions) {
  if (runtime_controller_->IsRootIsolateRunning()) {
    runtime_controller_->LoadDartDeferredLibrary(
        loading_unit_id, std::move(snapshot_data),
        std::move(snapshot_instructions));
  } else {
    LoadDartDeferredLibraryError(loading_unit_id, "No running root isolate.",
                                 true);
  }
}

void Engine::LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                          const std::string& error_message,
                                          bool transient) {
  if (runtime_controller_->IsRootIsolateRunning()) {
    runtime_controller_->LoadDartDeferredLibraryError(loading_unit_id,
                                                      error_message, transient);
  }
}

const std::weak_ptr<VsyncWaiter> Engine::GetVsyncWaiter() const {
  return animator_->GetVsyncWaiter();
}

void Engine::SetDisplays(const std::vector<DisplayData>& displays) {
  runtime_controller_->SetDisplays(displays);
  ScheduleFrame();
}

void Engine::ShutdownPlatformIsolates() {
  runtime_controller_->ShutdownPlatformIsolates();
}

}  // namespace flutter
