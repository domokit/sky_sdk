// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_engine.h"

#include <dwmapi.h>

#include <filesystem>
#include <shared_mutex>
#include <sstream>

#include "flutter/fml/logging.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/platform/win/wstring_conversion.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/shell/platform/common/client_wrapper/binary_messenger_impl.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_message_codec.h"
#include "flutter/shell/platform/common/path_utils.h"
#include "flutter/shell/platform/embedder/embedder_struct_macros.h"
#include "flutter/shell/platform/windows/accessibility_bridge_windows.h"
#include "flutter/shell/platform/windows/compositor_opengl.h"
#include "flutter/shell/platform/windows/compositor_software.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/keyboard_key_channel_handler.h"
#include "flutter/shell/platform/windows/system_utils.h"
#include "flutter/shell/platform/windows/task_runner.h"
#include "flutter/third_party/accessibility/ax/ax_node.h"

// winbase.h defines GetCurrentTime as a macro.
#undef GetCurrentTime

static constexpr char kAccessibilityChannelName[] = "flutter/accessibility";

namespace flutter {

namespace {

// Lifted from vsync_waiter_fallback.cc
static std::chrono::nanoseconds SnapToNextTick(
    std::chrono::nanoseconds value,
    std::chrono::nanoseconds tick_phase,
    std::chrono::nanoseconds tick_interval) {
  std::chrono::nanoseconds offset = (tick_phase - value) % tick_interval;
  if (offset != std::chrono::nanoseconds::zero())
    offset = offset + tick_interval;
  return value + offset;
}

// Creates and returns a FlutterRendererConfig that renders to the view (if any)
// of a FlutterWindowsEngine, using OpenGL (via ANGLE).
// The user_data received by the render callbacks refers to the
// FlutterWindowsEngine.
FlutterRendererConfig GetOpenGLRendererConfig() {
  FlutterRendererConfig config = {};
  config.type = kOpenGL;
  config.open_gl.struct_size = sizeof(config.open_gl);
  config.open_gl.make_current = [](void* user_data) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (!host->egl_manager()) {
      return false;
    }
    return host->egl_manager()->render_context()->MakeCurrent();
  };
  config.open_gl.clear_current = [](void* user_data) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (!host->egl_manager()) {
      return false;
    }
    return host->egl_manager()->render_context()->ClearCurrent();
  };
  config.open_gl.present = [](void* user_data) -> bool { FML_UNREACHABLE(); };
  config.open_gl.fbo_reset_after_present = true;
  config.open_gl.fbo_with_frame_info_callback =
      [](void* user_data, const FlutterFrameInfo* info) -> uint32_t {
    FML_UNREACHABLE();
  };
  config.open_gl.gl_proc_resolver = [](void* user_data,
                                       const char* what) -> void* {
    return reinterpret_cast<void*>(eglGetProcAddress(what));
  };
  config.open_gl.make_resource_current = [](void* user_data) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (!host->egl_manager()) {
      return false;
    }
    return host->egl_manager()->resource_context()->MakeCurrent();
  };
  config.open_gl.gl_external_texture_frame_callback =
      [](void* user_data, int64_t texture_id, size_t width, size_t height,
         FlutterOpenGLTexture* texture) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (!host->texture_registrar()) {
      return false;
    }
    return host->texture_registrar()->PopulateTexture(texture_id, width, height,
                                                      texture);
  };
  return config;
}

// Creates and returns a FlutterRendererConfig that renders to the view (if any)
// of a FlutterWindowsEngine, using software rasterization.
// The user_data received by the render callbacks refers to the
// FlutterWindowsEngine.
FlutterRendererConfig GetSoftwareRendererConfig() {
  FlutterRendererConfig config = {};
  config.type = kSoftware;
  config.software.struct_size = sizeof(config.software);
  config.software.surface_present_callback =
      [](void* user_data, const void* allocation, size_t row_bytes,
         size_t height) {
        FML_UNREACHABLE();
        return false;
      };
  return config;
}

// Converts a FlutterPlatformMessage to an equivalent FlutterDesktopMessage.
static FlutterDesktopMessage ConvertToDesktopMessage(
    const FlutterPlatformMessage& engine_message) {
  FlutterDesktopMessage message = {};
  message.struct_size = sizeof(message);
  message.channel = engine_message.channel;
  message.message = engine_message.message;
  message.message_size = engine_message.message_size;
  message.response_handle = engine_message.response_handle;
  return message;
}

// Converts a LanguageInfo struct to a FlutterLocale struct. |info| must outlive
// the returned value, since the returned FlutterLocale has pointers into it.
FlutterLocale CovertToFlutterLocale(const LanguageInfo& info) {
  FlutterLocale locale = {};
  locale.struct_size = sizeof(FlutterLocale);
  locale.language_code = info.language.c_str();
  if (!info.region.empty()) {
    locale.country_code = info.region.c_str();
  }
  if (!info.script.empty()) {
    locale.script_code = info.script.c_str();
  }
  return locale;
}

}  // namespace

FlutterWindowsEngine::FlutterWindowsEngine(
    const FlutterProjectBundle& project,
    std::shared_ptr<WindowsProcTable> windows_proc_table)
    : project_(std::make_unique<FlutterProjectBundle>(project)),
      windows_proc_table_(std::move(windows_proc_table)),
      aot_data_(nullptr, nullptr),
      lifecycle_manager_(std::make_unique<WindowsLifecycleManager>(this)) {
  if (windows_proc_table_ == nullptr) {
    windows_proc_table_ = std::make_shared<WindowsProcTable>();
  }

  gl_ = egl::ProcTable::Create();

  embedder_api_.struct_size = sizeof(FlutterEngineProcTable);
  FlutterEngineGetProcAddresses(&embedder_api_);

  task_runner_ =
      std::make_unique<TaskRunner>(
          embedder_api_.GetCurrentTime, [this](const auto* task) {
            if (!engine_) {
              FML_LOG(ERROR)
                  << "Cannot post an engine task when engine is not running.";
              return;
            }
            if (embedder_api_.RunTask(engine_, task) != kSuccess) {
              FML_LOG(ERROR) << "Failed to post an engine task.";
            }
          });

  // Set up the legacy structs backing the API handles.
  messenger_ =
      fml::RefPtr<FlutterDesktopMessenger>(new FlutterDesktopMessenger());
  messenger_->SetEngine(this);
  plugin_registrar_ = std::make_unique<FlutterDesktopPluginRegistrar>();
  plugin_registrar_->engine = this;

  messenger_wrapper_ =
      std::make_unique<BinaryMessengerImpl>(messenger_->ToRef());
  message_dispatcher_ =
      std::make_unique<IncomingMessageDispatcher>(messenger_->ToRef());

  texture_registrar_ =
      std::make_unique<FlutterWindowsTextureRegistrar>(this, gl_);

  // Check for impeller support.
  auto& switches = project_->GetSwitches();
  enable_impeller_ = std::find(switches.begin(), switches.end(),
                               "--enable-impeller=true") != switches.end();

  egl_manager_ = egl::Manager::Create();
  window_proc_delegate_manager_ = std::make_unique<WindowProcDelegateManager>();
  window_proc_delegate_manager_->RegisterTopLevelWindowProcDelegate(
      [](HWND hwnd, UINT msg, WPARAM wpar, LPARAM lpar, void* user_data,
         LRESULT* result) {
        BASE_DCHECK(user_data);
        FlutterWindowsEngine* that =
            static_cast<FlutterWindowsEngine*>(user_data);
        BASE_DCHECK(that->lifecycle_manager_);
        return that->lifecycle_manager_->WindowProc(hwnd, msg, wpar, lpar,
                                                    result);
      },
      static_cast<void*>(this));

  // Set up internal channels.
  // TODO: Replace this with an embedder.h API. See
  // https://github.com/flutter/flutter/issues/71099
  internal_plugin_registrar_ =
      std::make_unique<PluginRegistrar>(plugin_registrar_.get());

  accessibility_plugin_ = std::make_unique<AccessibilityPlugin>(this);
  AccessibilityPlugin::SetUp(messenger_wrapper_.get(),
                             accessibility_plugin_.get());

  cursor_handler_ =
      std::make_unique<CursorHandler>(messenger_wrapper_.get(), this);
  platform_handler_ =
      std::make_unique<PlatformHandler>(messenger_wrapper_.get(), this);
  settings_plugin_ = std::make_unique<SettingsPlugin>(messenger_wrapper_.get(),
                                                      task_runner_.get());
}

FlutterWindowsEngine::~FlutterWindowsEngine() {
  messenger_->SetEngine(nullptr);
  Stop();
}

void FlutterWindowsEngine::SetSwitches(
    const std::vector<std::string>& switches) {
  project_->SetSwitches(switches);
}

bool FlutterWindowsEngine::Run() {
  return Run("");
}

bool FlutterWindowsEngine::Run(std::string_view entrypoint) {
  if (!project_->HasValidPaths()) {
    FML_LOG(ERROR) << "Missing or unresolvable paths to assets.";
    return false;
  }
  std::string assets_path_string = project_->assets_path().u8string();
  std::string icu_path_string = project_->icu_path().u8string();
  if (embedder_api_.RunsAOTCompiledDartCode()) {
    aot_data_ = project_->LoadAotData(embedder_api_);
    if (!aot_data_) {
      FML_LOG(ERROR) << "Unable to start engine without AOT data.";
      return false;
    }
  }

  // FlutterProjectArgs is expecting a full argv, so when processing it for
  // flags the first item is treated as the executable and ignored. Add a dummy
  // value so that all provided arguments are used.
  std::string executable_name = GetExecutableName();
  std::vector<const char*> argv = {executable_name.c_str()};
  std::vector<std::string> switches = project_->GetSwitches();
  std::transform(
      switches.begin(), switches.end(), std::back_inserter(argv),
      [](const std::string& arg) -> const char* { return arg.c_str(); });

  const std::vector<std::string>& entrypoint_args =
      project_->dart_entrypoint_arguments();
  std::vector<const char*> entrypoint_argv;
  std::transform(
      entrypoint_args.begin(), entrypoint_args.end(),
      std::back_inserter(entrypoint_argv),
      [](const std::string& arg) -> const char* { return arg.c_str(); });

  // Configure task runners.
  FlutterTaskRunnerDescription platform_task_runner = {};
  platform_task_runner.struct_size = sizeof(FlutterTaskRunnerDescription);
  platform_task_runner.user_data = task_runner_.get();
  platform_task_runner.runs_task_on_current_thread_callback =
      [](void* user_data) -> bool {
    return static_cast<TaskRunner*>(user_data)->RunsTasksOnCurrentThread();
  };
  platform_task_runner.post_task_callback = [](FlutterTask task,
                                               uint64_t target_time_nanos,
                                               void* user_data) -> void {
    static_cast<TaskRunner*>(user_data)->PostFlutterTask(task,
                                                         target_time_nanos);
  };
  FlutterCustomTaskRunners custom_task_runners = {};
  custom_task_runners.struct_size = sizeof(FlutterCustomTaskRunners);
  custom_task_runners.platform_task_runner = &platform_task_runner;
  custom_task_runners.thread_priority_setter =
      &WindowsPlatformThreadPrioritySetter;

  FlutterProjectArgs args = {};
  args.struct_size = sizeof(FlutterProjectArgs);
  args.shutdown_dart_vm_when_done = true;
  args.assets_path = assets_path_string.c_str();
  args.icu_data_path = icu_path_string.c_str();
  args.command_line_argc = static_cast<int>(argv.size());
  args.command_line_argv = argv.empty() ? nullptr : argv.data();

  // Fail if conflicting non-default entrypoints are specified in the method
  // argument and the project.
  //
  // TODO(cbracken): https://github.com/flutter/flutter/issues/109285
  // The entrypoint method parameter should eventually be removed from this
  // method and only the entrypoint specified in project_ should be used.
  if (!project_->dart_entrypoint().empty() && !entrypoint.empty() &&
      project_->dart_entrypoint() != entrypoint) {
    FML_LOG(ERROR) << "Conflicting entrypoints were specified in "
                      "FlutterDesktopEngineProperties.dart_entrypoint and "
                      "FlutterDesktopEngineRun(engine, entry_point). ";
    return false;
  }
  if (!entrypoint.empty()) {
    args.custom_dart_entrypoint = entrypoint.data();
  } else if (!project_->dart_entrypoint().empty()) {
    args.custom_dart_entrypoint = project_->dart_entrypoint().c_str();
  }
  args.dart_entrypoint_argc = static_cast<int>(entrypoint_argv.size());
  args.dart_entrypoint_argv =
      entrypoint_argv.empty() ? nullptr : entrypoint_argv.data();
  args.platform_message_callback =
      [](const FlutterPlatformMessage* engine_message,
         void* user_data) -> void {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    return host->HandlePlatformMessage(engine_message);
  };
  args.vsync_callback = [](void* user_data, intptr_t baton) -> void {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    host->OnVsync(baton);
  };
  args.on_pre_engine_restart_callback = [](void* user_data) {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    host->OnPreEngineRestart();
  };
  args.update_semantics_callback2 = [](const FlutterSemanticsUpdate2* update,
                                       void* user_data) {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);

    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    auto view = host->view(kImplicitViewId);
    if (!view) {
      return;
    }

    auto accessibility_bridge = view->accessibility_bridge().lock();
    if (!accessibility_bridge) {
      return;
    }

    for (size_t i = 0; i < update->node_count; i++) {
      const FlutterSemanticsNode2* node = update->nodes[i];
      accessibility_bridge->AddFlutterSemanticsNodeUpdate(*node);
    }

    for (size_t i = 0; i < update->custom_action_count; i++) {
      const FlutterSemanticsCustomAction2* action = update->custom_actions[i];
      accessibility_bridge->AddFlutterSemanticsCustomActionUpdate(*action);
    }

    accessibility_bridge->CommitUpdates();
  };
  args.root_isolate_create_callback = [](void* user_data) {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (host->root_isolate_create_callback_) {
      host->root_isolate_create_callback_();
    }
  };
  args.channel_update_callback = [](const FlutterChannelUpdate* update,
                                    void* user_data) {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);
    if (SAFE_ACCESS(update, channel, nullptr) != nullptr) {
      std::string channel_name(update->channel);
      host->OnChannelUpdate(std::move(channel_name),
                            SAFE_ACCESS(update, listening, false));
    }
  };

  args.custom_task_runners = &custom_task_runners;

  if (!platform_view_plugin_) {
    platform_view_plugin_ = std::make_unique<PlatformViewPlugin>(
        messenger_wrapper_.get(), task_runner_.get());
  }
  if (egl_manager_) {
    auto resolver = [](const char* name) -> void* {
      return reinterpret_cast<void*>(::eglGetProcAddress(name));
    };

    // TODO(schectman) Pass the platform view manager to the compositor
    // constructors: https://github.com/flutter/flutter/issues/143375
    compositor_ =
        std::make_unique<CompositorOpenGL>(this, resolver, enable_impeller_);
  } else {
    compositor_ = std::make_unique<CompositorSoftware>();
  }

  FlutterCompositor compositor = {};
  compositor.struct_size = sizeof(FlutterCompositor);
  compositor.user_data = this;
  compositor.create_backing_store_callback =
      [](const FlutterBackingStoreConfig* config,
         FlutterBackingStore* backing_store_out, void* user_data) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);

    return host->compositor_->CreateBackingStore(*config, backing_store_out);
  };

  compositor.collect_backing_store_callback =
      [](const FlutterBackingStore* backing_store, void* user_data) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(user_data);

    return host->compositor_->CollectBackingStore(backing_store);
  };

  compositor.present_view_callback =
      [](const FlutterPresentViewInfo* info) -> bool {
    auto host = static_cast<FlutterWindowsEngine*>(info->user_data);

    return host->Present(info);
  };
  args.compositor = &compositor;

  if (aot_data_) {
    args.aot_data = aot_data_.get();
  }

  // The platform thread creates OpenGL contexts. These
  // must be released to be used by the engine's threads.
  FML_DCHECK(!egl_manager_ || !egl_manager_->HasContextCurrent());

  FlutterRendererConfig renderer_config;

  if (enable_impeller_) {
    // Impeller does not support a Software backend. Avoid falling back and
    // confusing the engine on which renderer is selected.
    if (!egl_manager_) {
      FML_LOG(ERROR) << "Could not create surface manager. Impeller backend "
                        "does not support software rendering.";
      return false;
    }
    renderer_config = GetOpenGLRendererConfig();
  } else {
    renderer_config =
        egl_manager_ ? GetOpenGLRendererConfig() : GetSoftwareRendererConfig();
  }

  auto result = embedder_api_.Run(FLUTTER_ENGINE_VERSION, &renderer_config,
                                  &args, this, &engine_);
  if (result != kSuccess || engine_ == nullptr) {
    FML_LOG(ERROR) << "Failed to start Flutter engine: error " << result;
    return false;
  }

  // Configure device frame rate displayed via devtools.
  FlutterEngineDisplay display = {};
  display.struct_size = sizeof(FlutterEngineDisplay);
  display.display_id = 0;
  display.single_display = true;
  display.refresh_rate =
      1.0 / (static_cast<double>(FrameInterval().count()) / 1000000000.0);

  std::vector<FlutterEngineDisplay> displays = {display};
  embedder_api_.NotifyDisplayUpdate(engine_,
                                    kFlutterEngineDisplaysUpdateTypeStartup,
                                    displays.data(), displays.size());

  SendSystemLocales();
  SetLifecycleState(flutter::AppLifecycleState::kResumed);

  settings_plugin_->StartWatching();
  settings_plugin_->SendSettings();

  InitializeKeyboard();

  return true;
}

bool FlutterWindowsEngine::Stop() {
  if (engine_) {
    for (const auto& [callback, registrar] :
         plugin_registrar_destruction_callbacks_) {
      callback(registrar);
    }
    FlutterEngineResult result = embedder_api_.Shutdown(engine_);
    engine_ = nullptr;
    return (result == kSuccess);
  }
  return false;
}

std::unique_ptr<FlutterWindowsView> FlutterWindowsEngine::CreateView(
    std::unique_ptr<WindowBindingHandler> window) {
  auto view_id = next_view_id_;
  auto view = std::make_unique<FlutterWindowsView>(
      view_id, this, std::move(window), windows_proc_table_);

  view->CreateRenderSurface();

  next_view_id_++;

  {
    // Add the view to the embedder. This must happen before the engine
    // is notified the view exists and starts presenting to it.
    std::unique_lock write_lock(views_mutex_);
    FML_DCHECK(views_.find(view_id) == views_.end());
    views_[view_id] = view.get();
  }

  if (!view->IsImplicitView()) {
    FML_DCHECK(running());

    struct Captures {
      fml::AutoResetWaitableEvent latch;
      bool added;
    };
    Captures captures = {};

    FlutterWindowMetricsEvent metrics = view->CreateWindowMetricsEvent();

    FlutterAddViewInfo info = {};
    info.struct_size = sizeof(FlutterAddViewInfo);
    info.view_id = view_id;
    info.view_metrics = &metrics;
    info.user_data = &captures;
    info.add_view_callback = [](const FlutterAddViewResult* result) {
      Captures* captures = reinterpret_cast<Captures*>(result->user_data);
      captures->added = result->added;
      captures->latch.Signal();
    };

    FlutterEngineResult result = embedder_api_.AddView(engine_, &info);
    if (result != kSuccess) {
      FML_LOG(ERROR)
          << "Starting the add view operation failed. FlutterEngineAddView "
             "returned an unexpected result: "
          << result << ". This indicates a bug in the Windows embedder.";
      FML_DCHECK(false);
      return nullptr;
    }

    // Block the platform thread until the engine has added the view.
    // TODO(loicsharma): This blocks the platform thread eagerly and can
    // cause unnecessary delay in input processing. Instead, this should block
    // lazily only when the app does an operation which needs the view.
    // https://github.com/flutter/flutter/issues/146248
    captures.latch.Wait();

    if (!captures.added) {
      // Adding the view failed. Update the embedder's state to match the
      // engine's state. This is unexpected and indicates a bug in the Windows
      // embedder.
      FML_LOG(ERROR) << "FlutterEngineAddView failed to add view";
      std::unique_lock write_lock(views_mutex_);
      views_.erase(view_id);
      return nullptr;
    }
  }

  return std::move(view);
}

void FlutterWindowsEngine::RemoveView(FlutterViewId view_id) {
  FML_DCHECK(running());

  // Notify the engine to stop rendering to the view if it isn't the implicit
  // view. The engine and framework assume the implicit view always exists and
  // can continue presenting.
  if (view_id != kImplicitViewId) {
    struct Captures {
      fml::AutoResetWaitableEvent latch;
      bool removed;
    };
    Captures captures = {};

    FlutterRemoveViewInfo info = {};
    info.struct_size = sizeof(FlutterRemoveViewInfo);
    info.view_id = view_id;
    info.user_data = &captures;
    info.remove_view_callback = [](const FlutterRemoveViewResult* result) {
      // This is invoked on an engine thread. If
      // |FlutterRemoveViewResult.removed| is `true`, the engine guarantees the
      // view won't be presented.
      Captures* captures = reinterpret_cast<Captures*>(result->user_data);
      captures->removed = result->removed;
      captures->latch.Signal();
    };

    FlutterEngineResult result = embedder_api_.RemoveView(engine_, &info);
    if (result != kSuccess) {
      FML_LOG(ERROR) << "Starting the remove view operation failed. "
                        "FlutterEngineRemoveView "
                        "returned an unexpected result: "
                     << result
                     << ". This indicates a bug in the Windows embedder.";
      FML_DCHECK(false);
      return;
    }

    // Block the platform thread until the engine has removed the view.
    // TODO(loicsharma): This blocks the platform thread eagerly and can
    // cause unnecessary delay in input processing. Instead, this should block
    // lazily only when an operation needs the view.
    // https://github.com/flutter/flutter/issues/146248
    captures.latch.Wait();

    if (!captures.removed) {
      // Removing the view failed. This is unexpected and indicates a bug in the
      // Windows embedder.
      FML_LOG(ERROR) << "FlutterEngineRemoveView failed to remove view";
      return;
    }
  }

  {
    // The engine no longer presents to the view. Remove the view from the
    // embedder.
    std::unique_lock write_lock(views_mutex_);

    FML_DCHECK(views_.find(view_id) != views_.end());
    views_.erase(view_id);
  }
}

void FlutterWindowsEngine::OnVsync(intptr_t baton) {
  std::chrono::nanoseconds current_time =
      std::chrono::nanoseconds(embedder_api_.GetCurrentTime());
  std::chrono::nanoseconds frame_interval = FrameInterval();
  auto next = SnapToNextTick(current_time, start_time_, frame_interval);
  embedder_api_.OnVsync(engine_, baton, next.count(),
                        (next + frame_interval).count());
}

std::chrono::nanoseconds FlutterWindowsEngine::FrameInterval() {
  if (frame_interval_override_.has_value()) {
    return frame_interval_override_.value();
  }
  uint64_t interval = 16600000;

  DWM_TIMING_INFO timing_info = {};
  timing_info.cbSize = sizeof(timing_info);
  HRESULT result = DwmGetCompositionTimingInfo(NULL, &timing_info);
  if (result == S_OK && timing_info.rateRefresh.uiDenominator > 0 &&
      timing_info.rateRefresh.uiNumerator > 0) {
    interval = static_cast<double>(timing_info.rateRefresh.uiDenominator *
                                   1000000000.0) /
               static_cast<double>(timing_info.rateRefresh.uiNumerator);
  }

  return std::chrono::nanoseconds(interval);
}

FlutterWindowsView* FlutterWindowsEngine::view(FlutterViewId view_id) const {
  std::shared_lock read_lock(views_mutex_);

  auto iterator = views_.find(view_id);
  if (iterator == views_.end()) {
    return nullptr;
  }

  return iterator->second;
}

// Returns the currently configured Plugin Registrar.
FlutterDesktopPluginRegistrarRef FlutterWindowsEngine::GetRegistrar() {
  return plugin_registrar_.get();
}

void FlutterWindowsEngine::AddPluginRegistrarDestructionCallback(
    FlutterDesktopOnPluginRegistrarDestroyed callback,
    FlutterDesktopPluginRegistrarRef registrar) {
  plugin_registrar_destruction_callbacks_[callback] = registrar;
}

void FlutterWindowsEngine::SendWindowMetricsEvent(
    const FlutterWindowMetricsEvent& event) {
  if (engine_) {
    embedder_api_.SendWindowMetricsEvent(engine_, &event);
  }
}

void FlutterWindowsEngine::SendPointerEvent(const FlutterPointerEvent& event) {
  if (engine_) {
    embedder_api_.SendPointerEvent(engine_, &event, 1);
  }
}

void FlutterWindowsEngine::SendKeyEvent(const FlutterKeyEvent& event,
                                        FlutterKeyEventCallback callback,
                                        void* user_data) {
  if (engine_) {
    embedder_api_.SendKeyEvent(engine_, &event, callback, user_data);
  }
}

bool FlutterWindowsEngine::SendPlatformMessage(
    const char* channel,
    const uint8_t* message,
    const size_t message_size,
    const FlutterDesktopBinaryReply reply,
    void* user_data) {
  FlutterPlatformMessageResponseHandle* response_handle = nullptr;
  if (reply != nullptr && user_data != nullptr) {
    FlutterEngineResult result =
        embedder_api_.PlatformMessageCreateResponseHandle(
            engine_, reply, user_data, &response_handle);
    if (result != kSuccess) {
      FML_LOG(ERROR) << "Failed to create response handle";
      return false;
    }
  }

  FlutterPlatformMessage platform_message = {
      sizeof(FlutterPlatformMessage),
      channel,
      message,
      message_size,
      response_handle,
  };

  FlutterEngineResult message_result =
      embedder_api_.SendPlatformMessage(engine_, &platform_message);
  if (response_handle != nullptr) {
    embedder_api_.PlatformMessageReleaseResponseHandle(engine_,
                                                       response_handle);
  }
  return message_result == kSuccess;
}

void FlutterWindowsEngine::SendPlatformMessageResponse(
    const FlutterDesktopMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length) {
  embedder_api_.SendPlatformMessageResponse(engine_, handle, data, data_length);
}

void FlutterWindowsEngine::HandlePlatformMessage(
    const FlutterPlatformMessage* engine_message) {
  if (engine_message->struct_size != sizeof(FlutterPlatformMessage)) {
    FML_LOG(ERROR) << "Invalid message size received. Expected: "
                   << sizeof(FlutterPlatformMessage) << " but received "
                   << engine_message->struct_size;
    return;
  }

  auto message = ConvertToDesktopMessage(*engine_message);

  message_dispatcher_->HandleMessage(message, [this] {}, [this] {});
}

void FlutterWindowsEngine::ReloadSystemFonts() {
  embedder_api_.ReloadSystemFonts(engine_);
}

void FlutterWindowsEngine::ScheduleFrame() {
  embedder_api_.ScheduleFrame(engine_);
}

void FlutterWindowsEngine::SetNextFrameCallback(fml::closure callback) {
  next_frame_callback_ = std::move(callback);

  embedder_api_.SetNextFrameCallback(
      engine_,
      [](void* user_data) {
        // Embedder callback runs on raster thread. Switch back to platform
        // thread.
        FlutterWindowsEngine* self =
            static_cast<FlutterWindowsEngine*>(user_data);

        self->task_runner_->PostTask(std::move(self->next_frame_callback_));
      },
      this);
}

void FlutterWindowsEngine::SetLifecycleState(flutter::AppLifecycleState state) {
  if (lifecycle_manager_) {
    lifecycle_manager_->SetLifecycleState(state);
  }
}

void FlutterWindowsEngine::SendSystemLocales() {
  std::vector<LanguageInfo> languages =
      GetPreferredLanguageInfo(*windows_proc_table_);
  std::vector<FlutterLocale> flutter_locales;
  flutter_locales.reserve(languages.size());
  for (const auto& info : languages) {
    flutter_locales.push_back(CovertToFlutterLocale(info));
  }
  // Convert the locale list to the locale pointer list that must be provided.
  std::vector<const FlutterLocale*> flutter_locale_list;
  flutter_locale_list.reserve(flutter_locales.size());
  std::transform(flutter_locales.begin(), flutter_locales.end(),
                 std::back_inserter(flutter_locale_list),
                 [](const auto& arg) -> const auto* { return &arg; });
  embedder_api_.UpdateLocales(engine_, flutter_locale_list.data(),
                              flutter_locale_list.size());
}

void FlutterWindowsEngine::InitializeKeyboard() {
  auto internal_plugin_messenger = internal_plugin_registrar_->messenger();
  KeyboardKeyEmbedderHandler::GetKeyStateHandler get_key_state = GetKeyState;
  KeyboardKeyEmbedderHandler::MapVirtualKeyToScanCode map_vk_to_scan =
      [](UINT virtual_key, bool extended) {
        return MapVirtualKey(virtual_key,
                             extended ? MAPVK_VK_TO_VSC_EX : MAPVK_VK_TO_VSC);
      };
  keyboard_key_handler_ = std::move(CreateKeyboardKeyHandler(
      internal_plugin_messenger, get_key_state, map_vk_to_scan));
  text_input_plugin_ =
      std::move(CreateTextInputPlugin(internal_plugin_messenger));
}

std::unique_ptr<KeyboardHandlerBase>
FlutterWindowsEngine::CreateKeyboardKeyHandler(
    BinaryMessenger* messenger,
    KeyboardKeyEmbedderHandler::GetKeyStateHandler get_key_state,
    KeyboardKeyEmbedderHandler::MapVirtualKeyToScanCode map_vk_to_scan) {
  auto keyboard_key_handler = std::make_unique<KeyboardKeyHandler>(messenger);
  keyboard_key_handler->AddDelegate(
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [this](const FlutterKeyEvent& event, FlutterKeyEventCallback callback,
                 void* user_data) {
            return SendKeyEvent(event, callback, user_data);
          },
          get_key_state, map_vk_to_scan));
  keyboard_key_handler->AddDelegate(
      std::make_unique<KeyboardKeyChannelHandler>(messenger));
  keyboard_key_handler->InitKeyboardChannel();
  return keyboard_key_handler;
}

std::unique_ptr<TextInputPlugin> FlutterWindowsEngine::CreateTextInputPlugin(
    BinaryMessenger* messenger) {
  return std::make_unique<TextInputPlugin>(messenger, this);
}

bool FlutterWindowsEngine::RegisterExternalTexture(int64_t texture_id) {
  return (embedder_api_.RegisterExternalTexture(engine_, texture_id) ==
          kSuccess);
}

bool FlutterWindowsEngine::UnregisterExternalTexture(int64_t texture_id) {
  return (embedder_api_.UnregisterExternalTexture(engine_, texture_id) ==
          kSuccess);
}

bool FlutterWindowsEngine::MarkExternalTextureFrameAvailable(
    int64_t texture_id) {
  return (embedder_api_.MarkExternalTextureFrameAvailable(
              engine_, texture_id) == kSuccess);
}

bool FlutterWindowsEngine::PostRasterThreadTask(fml::closure callback) const {
  struct Captures {
    fml::closure callback;
  };
  auto captures = new Captures();
  captures->callback = std::move(callback);
  if (embedder_api_.PostRenderThreadTask(
          engine_,
          [](void* opaque) {
            auto captures = reinterpret_cast<Captures*>(opaque);
            captures->callback();
            delete captures;
          },
          captures) == kSuccess) {
    return true;
  }
  delete captures;
  return false;
}

bool FlutterWindowsEngine::DispatchSemanticsAction(
    uint64_t target,
    FlutterSemanticsAction action,
    fml::MallocMapping data) {
  return (embedder_api_.DispatchSemanticsAction(engine_, target, action,
                                                data.GetMapping(),
                                                data.GetSize()) == kSuccess);
}

void FlutterWindowsEngine::UpdateSemanticsEnabled(bool enabled) {
  if (engine_ && semantics_enabled_ != enabled) {
    std::shared_lock read_lock(views_mutex_);

    semantics_enabled_ = enabled;
    embedder_api_.UpdateSemanticsEnabled(engine_, enabled);
    for (auto iterator = views_.begin(); iterator != views_.end(); iterator++) {
      iterator->second->UpdateSemanticsEnabled(enabled);
    }
  }
}

void FlutterWindowsEngine::OnPreEngineRestart() {
  // Reset the keyboard's state on hot restart.
  InitializeKeyboard();
}

std::string FlutterWindowsEngine::GetExecutableName() const {
  std::pair<bool, std::string> result = fml::paths::GetExecutablePath();
  if (result.first) {
    const std::string& executable_path = result.second;
    size_t last_separator = executable_path.find_last_of("/\\");
    if (last_separator == std::string::npos ||
        last_separator == executable_path.size() - 1) {
      return executable_path;
    }
    return executable_path.substr(last_separator + 1);
  }
  return "Flutter";
}

void FlutterWindowsEngine::UpdateAccessibilityFeatures() {
  UpdateHighContrastMode();
}

void FlutterWindowsEngine::UpdateHighContrastMode() {
  high_contrast_enabled_ = windows_proc_table_->GetHighContrastEnabled();

  SendAccessibilityFeatures();
  settings_plugin_->UpdateHighContrastMode(high_contrast_enabled_);
}

void FlutterWindowsEngine::SendAccessibilityFeatures() {
  int flags = 0;

  if (high_contrast_enabled_) {
    flags |=
        FlutterAccessibilityFeature::kFlutterAccessibilityFeatureHighContrast;
  }

  embedder_api_.UpdateAccessibilityFeatures(
      engine_, static_cast<FlutterAccessibilityFeature>(flags));
}

void FlutterWindowsEngine::RequestApplicationQuit(HWND hwnd,
                                                  WPARAM wparam,
                                                  LPARAM lparam,
                                                  AppExitType exit_type) {
  platform_handler_->RequestAppExit(hwnd, wparam, lparam, exit_type, 0);
}

void FlutterWindowsEngine::OnQuit(std::optional<HWND> hwnd,
                                  std::optional<WPARAM> wparam,
                                  std::optional<LPARAM> lparam,
                                  UINT exit_code) {
  lifecycle_manager_->Quit(hwnd, wparam, lparam, exit_code);
}

void FlutterWindowsEngine::OnDwmCompositionChanged() {
  std::shared_lock read_lock(views_mutex_);

  for (auto iterator = views_.begin(); iterator != views_.end(); iterator++) {
    iterator->second->OnDwmCompositionChanged();
  }
}

void FlutterWindowsEngine::OnWindowStateEvent(HWND hwnd,
                                              WindowStateEvent event) {
  lifecycle_manager_->OnWindowStateEvent(hwnd, event);
}

std::optional<LRESULT> FlutterWindowsEngine::ProcessExternalWindowMessage(
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam) {
  if (lifecycle_manager_) {
    return lifecycle_manager_->ExternalWindowMessage(hwnd, message, wparam,
                                                     lparam);
  }
  return std::nullopt;
}

void FlutterWindowsEngine::OnChannelUpdate(std::string name, bool listening) {
  if (name == "flutter/platform" && listening) {
    lifecycle_manager_->BeginProcessingExit();
  } else if (name == "flutter/lifecycle" && listening) {
    lifecycle_manager_->BeginProcessingLifecycle();
  }
}

bool FlutterWindowsEngine::Present(const FlutterPresentViewInfo* info) {
  // This runs on the raster thread. Lock the views map for the entirety of the
  // present operation to block the platform thread from destroying the
  // view during the present.
  std::shared_lock read_lock(views_mutex_);

  auto iterator = views_.find(info->view_id);
  if (iterator == views_.end()) {
    return false;
  }

  FlutterWindowsView* view = iterator->second;

  return compositor_->Present(view, info->layers, info->layers_count);
}

}  // namespace flutter
