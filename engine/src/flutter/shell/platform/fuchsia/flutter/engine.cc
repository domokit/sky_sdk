// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "engine.h"

#include <fuchsia/accessibility/semantics/cpp/fidl.h>
#include <fuchsia/media/cpp/fidl.h>
#include <lib/async/cpp/task.h>
#include <lib/zx/eventpair.h>
#include <lib/zx/thread.h>
#include <zircon/rights.h>
#include <zircon/status.h>
#include <zircon/types.h>
#include <memory>

#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/common/task_runners.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/run_configuration.h"
#include "flutter/shell/common/serialization_callbacks.h"
#include "flutter/shell/common/thread_host.h"
#include "third_party/skia/include/core/SkPicture.h"
#include "third_party/skia/include/core/SkSerialProcs.h"
#include "third_party/skia/include/gpu/ganesh/GrTypes.h"
#include "third_party/skia/include/ports/SkFontMgr_fuchsia.h"

#include "../runtime/dart/utils/files.h"
#include "../runtime/dart/utils/root_inspect_node.h"
#include "focus_delegate.h"
#include "fuchsia_intl.h"
#include "platform_view.h"
#include "software_surface_producer.h"
#include "surface.h"
#include "vsync_waiter.h"
#include "vulkan_surface_producer.h"

namespace flutter_runner {
namespace {

zx_koid_t GetKoid(const fuchsia::ui::views::ViewRef& view_ref) {
  zx_handle_t handle = view_ref.reference.get();
  zx_info_handle_basic_t info;
  zx_status_t status = zx_object_get_info(handle, ZX_INFO_HANDLE_BASIC, &info,
                                          sizeof(info), nullptr, nullptr);
  return status == ZX_OK ? info.koid : ZX_KOID_INVALID;
}

std::unique_ptr<flutter::PlatformMessage> MakeLocalizationPlatformMessage(
    const fuchsia::intl::Profile& intl_profile) {
  return std::make_unique<flutter::PlatformMessage>(
      "flutter/localization", MakeLocalizationPlatformMessageData(intl_profile),
      nullptr);
}

//
// Fuchsia scheduler role naming scheme employed here:
//
// Roles based on thread function:
//   <prefix>.type.{platform,ui,raster,io,profiler}
//
// Roles based on fml::Thread::ThreadPriority:
//   <prefix>.thread.{background,display,raster,normal}
//

void SetThreadRole(
    const fuchsia::media::ProfileProviderSyncPtr& profile_provider,
    const std::string& role) {
  ZX_ASSERT(profile_provider);

  zx::thread dup;
  const zx_status_t dup_status =
      zx::thread::self()->duplicate(ZX_RIGHT_SAME_RIGHTS, &dup);
  if (dup_status != ZX_OK) {
    FML_LOG(WARNING)
        << "Failed to duplicate thread handle when setting thread config: "
        << zx_status_get_string(dup_status)
        << ". Thread will run at default priority.";
    return;
  }

  int64_t unused_period;
  int64_t unused_capacity;
  const zx_status_t status = profile_provider->RegisterHandlerWithCapacity(
      std::move(dup), role, 0, 0.f, &unused_period, &unused_capacity);
  if (status != ZX_OK) {
    FML_LOG(WARNING) << "Failed to set thread role to \"" << role
                     << "\": " << zx_status_get_string(status)
                     << ". Thread will run at default priority.";
    return;
  }
}

void SetThreadConfig(
    const std::string& name_prefix,
    const fuchsia::media::ProfileProviderSyncPtr& profile_provider,
    const fml::Thread::ThreadConfig& config) {
  ZX_ASSERT(profile_provider);

  fml::Thread::SetCurrentThreadName(config);

  // Derive the role name from the prefix and priority. See comment above about
  // the role naming scheme.
  std::string role;
  switch (config.priority) {
    case fml::Thread::ThreadPriority::kBackground:
      role = name_prefix + ".thread.background";
      break;
    case fml::Thread::ThreadPriority::kDisplay:
      role = name_prefix + ".thread.display";
      break;
    case fml::Thread::ThreadPriority::kRaster:
      role = name_prefix + ".thread.raster";
      break;
    case fml::Thread::ThreadPriority::kNormal:
      role = name_prefix + ".thread.normal";
      break;
    default:
      FML_LOG(WARNING) << "Unknown thread priority "
                       << static_cast<int>(config.priority)
                       << ". Thread will run at default priority.";
      return;
  }
  ZX_ASSERT(!role.empty());

  SetThreadRole(profile_provider, role);
}

}  // namespace

flutter::ThreadHost Engine::CreateThreadHost(
    const std::string& name_prefix,
    const std::shared_ptr<sys::ServiceDirectory>& services) {
  fml::Thread::SetCurrentThreadName(
      fml::Thread::ThreadConfig(name_prefix + ".platform"));

  // Default the config setter to setup the thread name only.
  flutter::ThreadConfigSetter config_setter = fml::Thread::SetCurrentThreadName;

  // Override the config setter if the media profile provider is available.
  if (services) {
    // Connect to the media profile provider to assign thread priorities using
    // Fuchsia's scheduler role API. Failure to connect will print a warning and
    // proceed with engine initialization, leaving threads created by the engine
    // at default priority.
    //
    // The use of std::shared_ptr here is to work around the unfortunate
    // requirement for flutter::ThreadConfigSetter (i.e. std::function<>) that
    // the target callable be copy-constructible. This awkwardly conflicts with
    // fuchsia::media::ProfileProviderSyncPtr being move-only. std::shared_ptr
    // provides copyable object that references the move-only SyncPtr.
    std::shared_ptr<fuchsia::media::ProfileProviderSyncPtr>
        media_profile_provider =
            std::make_shared<fuchsia::media::ProfileProviderSyncPtr>();

    const zx_status_t connect_status =
        services->Connect(media_profile_provider->NewRequest());
    if (connect_status != ZX_OK) {
      FML_LOG(WARNING)
          << "Failed to connect to " << fuchsia::media::ProfileProvider::Name_
          << ": " << zx_status_get_string(connect_status)
          << " This is not a fatal error, but threads created by the engine "
             "will run at default priority, regardless of the requested "
             "priority.";
    } else {
      // Set the role for (this) platform thread. See comment above about the
      // role naming scheme.
      SetThreadRole(*media_profile_provider, name_prefix + ".type.platform");

      // This lambda must be copyable or the assignment fails to compile,
      // necessitating the use of std::shared_ptr for the profile provider.
      config_setter = [name_prefix, media_profile_provider](
                          const fml::Thread::ThreadConfig& config) {
        SetThreadConfig(name_prefix, *media_profile_provider, config);
      };
    }
  }

  flutter::ThreadHost::ThreadHostConfig thread_host_config{config_setter};

  thread_host_config.SetRasterConfig(
      {flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
           flutter::ThreadHost::Type::kRaster, name_prefix),
       fml::Thread::ThreadPriority::kRaster});
  thread_host_config.SetUIConfig(
      {flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
           flutter::ThreadHost::Type::kUi, name_prefix),
       fml::Thread::ThreadPriority::kDisplay});
  thread_host_config.SetIOConfig(
      {flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
           flutter::ThreadHost::Type::kIo, name_prefix),
       fml::Thread::ThreadPriority::kNormal});

  return flutter::ThreadHost(thread_host_config);
}

Engine::Engine(Delegate& delegate,
               std::string thread_label,
               std::shared_ptr<sys::ServiceDirectory> svc,
               std::shared_ptr<sys::ServiceDirectory> runner_services,
               flutter::Settings settings,
               fuchsia::ui::views::ViewCreationToken view_creation_token,
               std::pair<fuchsia::ui::views::ViewRefControl,
                         fuchsia::ui::views::ViewRef> view_ref_pair,
               UniqueFDIONS fdio_ns,
               fidl::InterfaceRequest<fuchsia::io::Directory> directory_request,
               FlutterRunnerProductConfiguration product_config,
               const std::vector<std::string>& dart_entrypoint_args)
    : delegate_(delegate),
      thread_label_(std::move(thread_label)),
      thread_host_(CreateThreadHost(thread_label_, runner_services)),
      view_creation_token_(std::move(view_creation_token)),
      memory_pressure_watcher_binding_(this),
      latest_memory_pressure_level_(fuchsia::memorypressure::Level::NORMAL),
      intercept_all_input_(product_config.get_intercept_all_input()),
      weak_factory_(this) {
  Initialize(std::move(view_ref_pair), std::move(svc),
             std::move(runner_services), std::move(settings),
             std::move(fdio_ns), std::move(directory_request),
             std::move(product_config), dart_entrypoint_args);
}

void Engine::Initialize(
    std::pair<fuchsia::ui::views::ViewRefControl, fuchsia::ui::views::ViewRef>
        view_ref_pair,
    std::shared_ptr<sys::ServiceDirectory> svc,
    std::shared_ptr<sys::ServiceDirectory> runner_services,
    flutter::Settings settings,
    UniqueFDIONS fdio_ns,
    fidl::InterfaceRequest<fuchsia::io::Directory> directory_request,
    FlutterRunnerProductConfiguration product_config,
    const std::vector<std::string>& dart_entrypoint_args) {
  // Flatland uses |view_creation_token_| for linking.
  FML_CHECK(view_creation_token_.value.is_valid());

  // Get the task runners from the managed threads. The current thread will be
  // used as the "platform" thread.
  fml::RefPtr<fml::TaskRunner> platform_task_runner =
      fml::MessageLoop::GetCurrent().GetTaskRunner();

  const flutter::TaskRunners task_runners(
      thread_label_,                                // Dart thread labels
      platform_task_runner,                         // platform
      thread_host_.raster_thread->GetTaskRunner(),  // raster
      thread_host_.ui_thread->GetTaskRunner(),      // ui
      thread_host_.io_thread->GetTaskRunner()       // io
  );

  fuchsia::ui::views::FocuserHandle focuser;
  fuchsia::ui::views::ViewRefFocusedHandle view_ref_focused;
  fuchsia::ui::pointer::TouchSourceHandle touch_source;
  fuchsia::ui::pointer::MouseSourceHandle mouse_source;

  fuchsia::ui::composition::ViewBoundProtocols view_protocols;
  view_protocols.set_view_focuser(focuser.NewRequest());
  view_protocols.set_view_ref_focused(view_ref_focused.NewRequest());
  view_protocols.set_touch_source(touch_source.NewRequest());
  view_protocols.set_mouse_source(mouse_source.NewRequest());

  // Connect to Flatland.
  fuchsia::ui::composition::FlatlandHandle flatland;
  zx_status_t flatland_status =
      runner_services->Connect<fuchsia::ui::composition::Flatland>(
          flatland.NewRequest());
  if (flatland_status != ZX_OK) {
    FML_LOG(WARNING) << "fuchsia::ui::composition::Flatland connection failed: "
                     << zx_status_get_string(flatland_status);
  }

  // Connect to SemanticsManager service.
  fuchsia::accessibility::semantics::SemanticsManagerHandle semantics_manager;
  zx_status_t semantics_status =
      runner_services
          ->Connect<fuchsia::accessibility::semantics::SemanticsManager>(
              semantics_manager.NewRequest());
  if (semantics_status != ZX_OK) {
    FML_LOG(WARNING)
        << "fuchsia::accessibility::semantics::SemanticsManager connection "
           "failed: "
        << zx_status_get_string(semantics_status);
  }

  // Connect to ImeService service.
  fuchsia::ui::input::ImeServiceHandle ime_service;
  zx_status_t ime_status =
      runner_services->Connect<fuchsia::ui::input::ImeService>(
          ime_service.NewRequest());
  if (ime_status != ZX_OK) {
    FML_LOG(WARNING) << "fuchsia::ui::input::ImeService connection failed: "
                     << zx_status_get_string(ime_status);
  }

  // Connect to Keyboard service.
  fuchsia::ui::input3::KeyboardHandle keyboard;
  zx_status_t keyboard_status =
      runner_services->Connect<fuchsia::ui::input3::Keyboard>(
          keyboard.NewRequest());
  FML_DCHECK(keyboard_status == ZX_OK)
      << "fuchsia::ui::input3::Keyboard connection failed: "
      << zx_status_get_string(keyboard_status);

  // Connect to Pointerinjector service.
  fuchsia::ui::pointerinjector::RegistryHandle pointerinjector_registry;
  zx_status_t pointerinjector_registry_status =
      runner_services->Connect<fuchsia::ui::pointerinjector::Registry>(
          pointerinjector_registry.NewRequest());
  if (pointerinjector_registry_status != ZX_OK) {
    FML_LOG(WARNING)
        << "fuchsia::ui::pointerinjector::Registry connection failed: "
        << zx_status_get_string(pointerinjector_registry_status);
  }

  // Make clones of the `ViewRef` before sending it to various places.
  fuchsia::ui::views::ViewRef platform_view_ref;
  view_ref_pair.second.Clone(&platform_view_ref);
  fuchsia::ui::views::ViewRef accessibility_view_ref;
  view_ref_pair.second.Clone(&accessibility_view_ref);
  fuchsia::ui::views::ViewRef isolate_view_ref;
  view_ref_pair.second.Clone(&isolate_view_ref);

  // Session is terminated on the raster thread, but we must terminate ourselves
  // on the platform thread.
  //
  // This handles the fidl error callback when the Session connection is
  // broken. The SessionListener interface also has an OnError method, which is
  // invoked on the platform thread (in PlatformView).
  fml::closure session_error_callback = [task_runner = platform_task_runner,
                                         weak = weak_factory_.GetWeakPtr()]() {
    task_runner->PostTask([weak]() {
      if (weak) {
        FML_LOG(ERROR) << "Terminating from session_error_callback";
        weak->Terminate();
      }
    });
  };

  // Set up the session connection and other Scenic helpers on the raster
  // thread. We also need to wait for the external view embedder to be set up
  // before creating the shell.
  fuchsia::ui::composition::ParentViewportWatcherPtr parent_viewport_watcher;
  fml::AutoResetWaitableEvent view_embedder_latch;
  auto session_inspect_node =
      dart_utils::RootInspectNode::CreateRootChild("vsync_stats");
  task_runners.GetRasterTaskRunner()->PostTask(fml::MakeCopyable(
      [this, &view_embedder_latch,
       session_inspect_node = std::move(session_inspect_node),
       flatland = std::move(flatland),
       session_error_callback = std::move(session_error_callback),
       view_creation_token = std::move(view_creation_token_),
       view_protocols = std::move(view_protocols),
       request = parent_viewport_watcher.NewRequest(),
       view_ref_pair = std::move(view_ref_pair),
       software_rendering = product_config.software_rendering()]() mutable {
        if (software_rendering) {
          surface_producer_ = std::make_shared<SoftwareSurfaceProducer>();
        } else {
          surface_producer_ = std::make_shared<VulkanSurfaceProducer>();
        }

        flatland_connection_ = std::make_shared<FlatlandConnection>(
            thread_label_, std::move(flatland),
            std::move(session_error_callback), [](auto) {});

        fuchsia::ui::views::ViewIdentityOnCreation view_identity = {
            .view_ref = std::move(view_ref_pair.second),
            .view_ref_control = std::move(view_ref_pair.first)};
        view_embedder_ = std::make_shared<ExternalViewEmbedder>(
            std::move(view_creation_token), std::move(view_identity),
            std::move(view_protocols), std::move(request), flatland_connection_,
            surface_producer_, intercept_all_input_);

        view_embedder_latch.Signal();
      }));
  view_embedder_latch.Wait();

  AccessibilityBridge::SetSemanticsEnabledCallback
      set_semantics_enabled_callback = [this](bool enabled) {
        auto platform_view = shell_->GetPlatformView();

        if (platform_view) {
          platform_view->SetSemanticsEnabled(enabled);
        }
      };

  AccessibilityBridge::DispatchSemanticsActionCallback
      dispatch_semantics_action_callback =
          [this](int32_t node_id, flutter::SemanticsAction action) {
            auto platform_view = shell_->GetPlatformView();

            if (platform_view) {
              platform_view->DispatchSemanticsAction(node_id, action, {});
            }
          };

  const std::string accessibility_inspect_name =
      std::to_string(GetKoid(accessibility_view_ref));
  accessibility_bridge_ = std::make_unique<AccessibilityBridge>(
      std::move(set_semantics_enabled_callback),
      std::move(dispatch_semantics_action_callback),
      std::move(semantics_manager), std::move(accessibility_view_ref),
      dart_utils::RootInspectNode::CreateRootChild(
          std::move(accessibility_inspect_name)));

  OnEnableWireframeCallback on_enable_wireframe_callback = std::bind(
      &Engine::DebugWireframeSettingsChanged, this, std::placeholders::_1);

  OnCreateViewCallback on_create_view_callback = std::bind(
      &Engine::CreateView, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);

  OnUpdateViewCallback on_update_view_callback = std::bind(
      &Engine::UpdateView, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4);

  OnDestroyViewCallback on_destroy_view_callback = std::bind(
      &Engine::DestroyView, this, std::placeholders::_1, std::placeholders::_2);

  OnCreateSurfaceCallback on_create_surface_callback =
      std::bind(&Engine::CreateSurface, this);

  // SessionListener has a OnScenicError method; invoke this callback on the
  // platform thread when that happens. The Session itself should also be
  // disconnected when this happens, and it will also attempt to terminate.
  fit::closure on_session_listener_error_callback =
      [task_runner = platform_task_runner,
       weak = weak_factory_.GetWeakPtr()]() {
        task_runner->PostTask([weak]() {
          if (weak) {
            FML_LOG(ERROR) << "Terminating from "
                              "on_session_listener_error_callback";
            weak->Terminate();
          }
        });
      };

  // Launch the engine in the appropriate configuration.
  // Note: this initializes the Asset Manager on the global PersistantCache
  // so it must be called before WarmupSkps() is called below.
  auto run_configuration = flutter::RunConfiguration::InferFromSettings(
      settings, task_runners.GetIOTaskRunner());
  run_configuration.SetEntrypointArgs(std::move(dart_entrypoint_args));

  OnSemanticsNodeUpdateCallback on_semantics_node_update_callback =
      [this](flutter::SemanticsNodeUpdates updates, float pixel_ratio) {
        accessibility_bridge_->AddSemanticsNodeUpdate(updates, pixel_ratio);
      };

  OnRequestAnnounceCallback on_request_announce_callback =
      [this](const std::string& message) {
        accessibility_bridge_->RequestAnnounce(message);
      };

  // Setup the callback that will instantiate the platform view.
  flutter::Shell::CreateCallback<flutter::PlatformView>
      on_create_platform_view = fml::MakeCopyable(
          [this, view_ref = std::move(platform_view_ref),
           parent_viewport_watcher = std::move(parent_viewport_watcher),
           ime_service = std::move(ime_service), keyboard = std::move(keyboard),
           focuser = std::move(focuser),
           view_ref_focused = std::move(view_ref_focused),
           touch_source = std::move(touch_source),
           mouse_source = std::move(mouse_source),
           pointerinjector_registry = std::move(pointerinjector_registry),
           on_session_listener_error_callback =
               std::move(on_session_listener_error_callback),
           on_enable_wireframe_callback =
               std::move(on_enable_wireframe_callback),
           on_create_view_callback = std::move(on_create_view_callback),
           on_update_view_callback = std::move(on_update_view_callback),
           on_destroy_view_callback = std::move(on_destroy_view_callback),
           on_create_surface_callback = std::move(on_create_surface_callback),
           on_semantics_node_update_callback =
               std::move(on_semantics_node_update_callback),
           on_request_announce_callback =
               std::move(on_request_announce_callback),
           external_view_embedder = GetExternalViewEmbedder(),
           await_vsync_callback =
               [this](FireCallbackCallback cb) {
                 flatland_connection_->AwaitVsync(cb);
               },
           await_vsync_for_secondary_callback_callback =
               [this](FireCallbackCallback cb) {
                 flatland_connection_->AwaitVsyncForSecondaryCallback(cb);
               },
           product_config, svc](flutter::Shell& shell) mutable {
            OnShaderWarmupCallback on_shader_warmup_callback = nullptr;
            if (product_config.enable_shader_warmup()) {
              FML_DCHECK(surface_producer_);
              if (product_config.enable_shader_warmup_dart_hooks()) {
                on_shader_warmup_callback =
                    [this, &shell](
                        const std::vector<std::string>& skp_names,
                        std::function<void(uint32_t)> completion_callback,
                        uint64_t width, uint64_t height) {
                      WarmupSkps(
                          shell.GetDartVM()
                              ->GetConcurrentMessageLoop()
                              ->GetTaskRunner()
                              .get(),
                          shell.GetTaskRunners().GetRasterTaskRunner().get(),
                          surface_producer_, SkISize::Make(width, height),
                          flutter::PersistentCache::GetCacheForProcess()
                              ->asset_manager(),
                          skp_names, completion_callback);
                    };
              } else {
                WarmupSkps(shell.GetDartVM()
                               ->GetConcurrentMessageLoop()
                               ->GetTaskRunner()
                               .get(),
                           shell.GetTaskRunners().GetRasterTaskRunner().get(),
                           surface_producer_, SkISize::Make(1024, 600),
                           flutter::PersistentCache::GetCacheForProcess()
                               ->asset_manager(),
                           std::nullopt, std::nullopt);
              }
            }

            return std::make_unique<flutter_runner::PlatformView>(
                shell, shell.GetTaskRunners(), std::move(view_ref),
                std::move(external_view_embedder), std::move(ime_service),
                std::move(keyboard), std::move(touch_source),
                std::move(mouse_source), std::move(focuser),
                std::move(view_ref_focused), std::move(parent_viewport_watcher),
                std::move(pointerinjector_registry),
                std::move(on_enable_wireframe_callback),
                std::move(on_create_view_callback),
                std::move(on_update_view_callback),
                std::move(on_destroy_view_callback),
                std::move(on_create_surface_callback),
                std::move(on_semantics_node_update_callback),
                std::move(on_request_announce_callback),
                std::move(on_shader_warmup_callback),
                std::move(await_vsync_callback),
                std::move(await_vsync_for_secondary_callback_callback),
                std::move(svc));
          });

  // Setup the callback that will instantiate the rasterizer.
  flutter::Shell::CreateCallback<flutter::Rasterizer> on_create_rasterizer =
      [](flutter::Shell& shell) {
        return std::make_unique<flutter::Rasterizer>(shell);
      };

  settings.root_isolate_create_callback =
      std::bind(&Engine::OnMainIsolateStart, this);
  settings.root_isolate_shutdown_callback =
      std::bind([weak = weak_factory_.GetWeakPtr(),
                 runner = task_runners.GetPlatformTaskRunner()]() {
        runner->PostTask([weak = std::move(weak)] {
          if (weak) {
            weak->OnMainIsolateShutdown();
          }
        });
      });

  // Connect and set up the system font provider.
  fuchsia::fonts::ProviderSyncPtr sync_font_provider;
  runner_services->Connect(sync_font_provider.NewRequest());
  settings.font_initialization_data =
      sync_font_provider.Unbind().TakeChannel().release();

  {
    TRACE_EVENT0("flutter", "CreateShell");
    shell_ = flutter::Shell::Create(
        flutter::PlatformData(),             // default window data
        std::move(task_runners),             // host task runners
        std::move(settings),                 // shell launch settings
        std::move(on_create_platform_view),  // platform view create callback
        std::move(on_create_rasterizer)      // rasterizer create callback
    );
  }

  if (!shell_) {
    FML_LOG(ERROR) << "Could not launch the shell.";
    return;
  }

  // Shell has been created. Before we run the engine, set up the isolate
  // configurator.
  isolate_configurator_ = std::make_unique<IsolateConfigurator>(
      std::move(fdio_ns), directory_request.TakeChannel(),
      std::move(isolate_view_ref.reference));

  //  This platform does not get a separate surface platform view creation
  //  notification. Fire one eagerly.
  shell_->GetPlatformView()->NotifyCreated();

  // Connect to the memory pressure provider.  If the connection fails, the
  // initialization of the engine will simply proceed, printing a warning
  // message.  The engine will be fully functional, except that the Flutter
  // shell will not be notified when memory is low.
  {
    memory_pressure_provider_.set_error_handler([](zx_status_t status) {
      FML_LOG(WARNING)
          << "Failed to connect to " << fuchsia::memorypressure::Provider::Name_
          << ": " << zx_status_get_string(status)
          << " This is not a fatal error, but the heap will not be "
          << " compacted when memory is low.";
    });

    // Note that we're using the runner's services, not the component's.
    // The Flutter Shell should be notified when memory is low regardless of
    // whether the component has direct access to the
    // fuchsia.memorypressure.Provider service.
    ZX_ASSERT(runner_services->Connect(
                  memory_pressure_provider_.NewRequest()) == ZX_OK);

    FML_VLOG(1) << "Registering memorypressure watcher";

    // Register for changes, which will make the request for the initial
    // memory level.
    memory_pressure_provider_->RegisterWatcher(
        memory_pressure_watcher_binding_.NewBinding());
  }

  // Connect to the intl property provider.  If the connection fails, the
  // initialization of the engine will simply proceed, printing a warning
  // message.  The engine will be fully functional, except that the user's
  // locale preferences would not be communicated to flutter engine.
  {
    intl_property_provider_.set_error_handler([](zx_status_t status) {
      FML_LOG(WARNING) << "Failed to connect to "
                       << fuchsia::intl::PropertyProvider::Name_ << ": "
                       << zx_status_get_string(status)
                       << " This is not a fatal error, but the user locale "
                       << " preferences will not be forwarded to flutter apps";
    });

    // Note that we're using the runner's services, not the component's.
    // Flutter locales should be updated regardless of whether the component has
    // direct access to the fuchsia.intl.PropertyProvider service.
    ZX_ASSERT(runner_services->Connect(intl_property_provider_.NewRequest()) ==
              ZX_OK);

    auto get_profile_callback = [weak = weak_factory_.GetWeakPtr()](
                                    const fuchsia::intl::Profile& profile) {
      if (!weak) {
        return;
      }
      if (!profile.has_locales()) {
        FML_LOG(WARNING) << "Got intl Profile without locales";
      }
      auto message = MakeLocalizationPlatformMessage(profile);
      FML_VLOG(1) << "Sending LocalizationPlatformMessage";
      weak->shell_->GetPlatformView()->DispatchPlatformMessage(
          std::move(message));
    };

    FML_VLOG(1) << "Requesting intl Profile";

    // Make the initial request
    intl_property_provider_->GetProfile(get_profile_callback);

    // And register for changes
    intl_property_provider_.events().OnChange = [this, runner_services,
                                                 get_profile_callback]() {
      FML_VLOG(1) << fuchsia::intl::PropertyProvider::Name_ << ": OnChange";
      runner_services->Connect(intl_property_provider_.NewRequest());
      intl_property_provider_->GetProfile(get_profile_callback);
    };
  }

  auto on_run_failure = [weak = weak_factory_.GetWeakPtr()]() {
    // The engine could have been killed by the caller right after the
    // constructor was called but before it could run on the UI thread.
    if (weak) {
      FML_LOG(ERROR) << "Terminating from on_run_failure";
      weak->Terminate();
    }
  };

  shell_->GetTaskRunners().GetUITaskRunner()->PostTask(
      fml::MakeCopyable([engine = shell_->GetEngine(),                      //
                         run_configuration = std::move(run_configuration),  //
                         on_run_failure                                     //
  ]() mutable {
        if (!engine) {
          return;
        }

        if (engine->Run(std::move(run_configuration)) ==
            flutter::Engine::RunStatus::Failure) {
          on_run_failure();
        }
      }));
}

Engine::~Engine() {
  shell_.reset();

  // Destroy rendering objects on the raster thread.
  fml::AutoResetWaitableEvent view_embedder_latch;
  thread_host_.raster_thread->GetTaskRunner()->PostTask(
      fml::MakeCopyable([this, &view_embedder_latch]() mutable {
        view_embedder_.reset();
        flatland_connection_.reset();
        surface_producer_.reset();
        view_embedder_latch.Signal();
      }));
  view_embedder_latch.Wait();
}

std::optional<uint32_t> Engine::GetEngineReturnCode() const {
  if (!shell_) {
    return std::nullopt;
  }
  std::optional<uint32_t> code;
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(
      shell_->GetTaskRunners().GetUITaskRunner(),
      [&latch, &code, engine = shell_->GetEngine()]() {
        if (engine) {
          code = engine->GetUIIsolateReturnCode();
        }
        latch.Signal();
      });
  latch.Wait();
  return code;
}

void Engine::OnMainIsolateStart() {
  if (!isolate_configurator_ ||
      !isolate_configurator_->ConfigureCurrentIsolate()) {
    FML_LOG(ERROR) << "Could not configure some native embedder bindings for a "
                      "new root isolate.";
  }
}

void Engine::OnMainIsolateShutdown() {
  Terminate();
}

void Engine::Terminate() {
  delegate_.OnEngineTerminate(this);
  // Warning. Do not do anything after this point as the delegate may have
  // collected this object.
}

void Engine::DebugWireframeSettingsChanged(bool enabled) {
  FML_CHECK(shell_);

  shell_->GetTaskRunners().GetRasterTaskRunner()->PostTask([]() {
    // TODO(fxbug.dev/116000): Investigate if we can add flatland wireframe code
    // for debugging.
  });
}

void Engine::CreateView(int64_t view_id,
                        ViewCallback on_view_created,
                        ViewCreatedCallback on_view_bound,
                        bool hit_testable,
                        bool focusable) {
  FML_CHECK(shell_);
  shell_->GetTaskRunners().GetRasterTaskRunner()->PostTask(
      [this, view_id, hit_testable, focusable,
       on_view_created = std::move(on_view_created),
       on_view_bound = std::move(on_view_bound)]() {
        FML_CHECK(view_embedder_);
        view_embedder_->CreateView(view_id, std::move(on_view_created),
                                   std::move(on_view_bound));
        view_embedder_->SetViewProperties(view_id, SkRect::MakeEmpty(),
                                          hit_testable, focusable);
      });
}

void Engine::UpdateView(int64_t view_id,
                        SkRect occlusion_hint,
                        bool hit_testable,
                        bool focusable) {
  FML_CHECK(shell_);
  shell_->GetTaskRunners().GetRasterTaskRunner()->PostTask(
      [this, view_id, occlusion_hint, hit_testable, focusable]() {
        FML_CHECK(view_embedder_);
        view_embedder_->SetViewProperties(view_id, occlusion_hint, hit_testable,
                                          focusable);
      });
}

void Engine::DestroyView(int64_t view_id, ViewIdCallback on_view_unbound) {
  FML_CHECK(shell_);

  shell_->GetTaskRunners().GetRasterTaskRunner()->PostTask(
      [this, view_id, on_view_unbound = std::move(on_view_unbound)]() {
        FML_CHECK(view_embedder_);
        view_embedder_->DestroyView(view_id, std::move(on_view_unbound));
      });
}

std::unique_ptr<flutter::Surface> Engine::CreateSurface() {
  return std::make_unique<Surface>(thread_label_, GetExternalViewEmbedder(),
                                   surface_producer_->gr_context());
}

std::shared_ptr<flutter::ExternalViewEmbedder>
Engine::GetExternalViewEmbedder() {
  FML_CHECK(view_embedder_);
  return view_embedder_;
}

#if !defined(DART_PRODUCT)
void Engine::WriteProfileToTrace() const {
  Dart_Port main_port = shell_->GetEngine()->GetUIIsolateMainPort();
  char* error = NULL;
  bool success = Dart_WriteProfileToTimeline(main_port, &error);
  if (!success) {
    FML_LOG(ERROR) << "Failed to write Dart profile to trace: " << error;
    free(error);
  }
}
#endif  // !defined(DART_PRODUCT)

void Engine::WarmupSkps(
    fml::BasicTaskRunner* concurrent_task_runner,
    fml::BasicTaskRunner* raster_task_runner,
    std::shared_ptr<SurfaceProducer> surface_producer,
    SkISize size,
    std::shared_ptr<flutter::AssetManager> asset_manager,
    std::optional<const std::vector<std::string>> skp_names,
    std::optional<std::function<void(uint32_t)>> maybe_completion_callback,
    bool synchronous) {
  // Wrap the optional validity checks up in a lambda to simplify the various
  // callsites below
  auto completion_callback = [maybe_completion_callback](uint32_t skp_count) {
    if (maybe_completion_callback.has_value() &&
        maybe_completion_callback.value()) {
      maybe_completion_callback.value()(skp_count);
    }
  };

  // We use this bizzare raw pointer to a smart pointer thing here because we
  // want to keep the surface alive until all gpu work is done and the
  // callbacks skia takes for this are function pointers so we are unable to
  // use a lambda that captures the smart pointer. We need two levels of
  // indirection because it needs to be the same across all invocations of the
  // raster task lambda from a single invocation of WarmupSkps, but be
  // different across different invocations of WarmupSkps (so we cant
  // statically initialialize it in the lambda itself). Basically the result
  // of a mashup of wierd call dynamics, multithreading, and lifecycle
  // management with C style Skia callbacks.
  std::unique_ptr<SurfaceProducerSurface>* skp_warmup_surface =
      new std::unique_ptr<SurfaceProducerSurface>(nullptr);

  // tell concurrent task runner to deserialize all skps available from
  // the asset manager
  concurrent_task_runner->PostTask([raster_task_runner, size,
                                    skp_warmup_surface, surface_producer,
                                    asset_manager, skp_names,
                                    completion_callback, synchronous]() {
    TRACE_DURATION("flutter", "DeserializeSkps");
    std::vector<std::unique_ptr<fml::Mapping>> skp_mappings;
    if (skp_names) {
      for (auto& skp_name : skp_names.value()) {
        auto skp_mapping = asset_manager->GetAsMapping(skp_name);
        if (skp_mapping) {
          skp_mappings.push_back(std::move(skp_mapping));
        } else {
          FML_LOG(ERROR) << "Failed to get mapping for " << skp_name;
        }
      }
    } else {
      skp_mappings = asset_manager->GetAsMappings(".*\\.skp$", "shaders");
    }

    if (skp_mappings.empty()) {
      FML_LOG(WARNING)
          << "Engine::WarmupSkps got zero SKP mappings, returning early";
      completion_callback(0);
      return;
    }

    size_t total_size = 0;
    for (auto& mapping : skp_mappings) {
      total_size += mapping->GetSize();
    }

    FML_LOG(INFO) << "Shader warmup got " << skp_mappings.size()
                  << " skp's with a total size of " << total_size << " bytes";

    std::vector<sk_sp<SkPicture>> pictures;
    unsigned int i = 0;
    for (auto& mapping : skp_mappings) {
      std::unique_ptr<SkMemoryStream> stream =
          SkMemoryStream::MakeDirect(mapping->GetMapping(), mapping->GetSize());
      SkDeserialProcs procs = {0};
      procs.fImageProc = flutter::DeserializeImageWithoutData;
      procs.fTypefaceProc = flutter::DeserializeTypefaceWithoutData;
      sk_sp<SkPicture> picture =
          SkPicture::MakeFromStream(stream.get(), &procs);
      if (!picture) {
        FML_LOG(ERROR) << "Failed to deserialize picture " << i;
        continue;
      }

      // Tell raster task runner to warmup have the compositor
      // context warm up the newly deserialized picture
      raster_task_runner->PostTask([picture, skp_warmup_surface, size,
                                    surface_producer, completion_callback, i,
                                    count = skp_mappings.size(), synchronous] {
        TRACE_DURATION("flutter", "WarmupSkp");
        if (*skp_warmup_surface == nullptr) {
          skp_warmup_surface->reset(
              surface_producer->ProduceOffscreenSurface(size).release());

          if (*skp_warmup_surface == nullptr) {
            FML_LOG(ERROR) << "Failed to create offscreen warmup surface";
            // Tell client that zero shaders were warmed up because warmup
            // failed.
            completion_callback(0);
            return;
          }
        }

        // Do the actual warmup
        (*skp_warmup_surface)
            ->GetSkiaSurface()
            ->getCanvas()
            ->drawPicture(picture);

        if (i == count - 1) {
          // We call this here instead of inside fFinishedProc below because
          // we want to unblock the dart animation code as soon as the
          // raster thread is free to enque work, rather than waiting for
          // the GPU work itself to finish.
          completion_callback(count);
        }

        if (surface_producer->gr_context()) {
          if (i < count - 1) {
            // For all but the last skp we fire and forget
            surface_producer->gr_context()->flushAndSubmit();
          } else {
            // For the last skp we provide a callback that frees the warmup
            // surface and calls the completion callback
            struct GrFlushInfo flush_info;
            flush_info.fFinishedContext = skp_warmup_surface;
            flush_info.fFinishedProc = [](void* skp_warmup_surface) {
              delete static_cast<std::unique_ptr<SurfaceProducerSurface>*>(
                  skp_warmup_surface);
            };

            surface_producer->gr_context()->flush(flush_info);
            surface_producer->gr_context()->submit(
                synchronous ? GrSyncCpu::kYes : GrSyncCpu::kNo);
          }
        } else {
          if (i == count - 1) {
            delete skp_warmup_surface;
          }
        }
      });
      i++;
    }
  });
}

void Engine::OnLevelChanged(
    fuchsia::memorypressure::Level level,
    fuchsia::memorypressure::Watcher::OnLevelChangedCallback callback) {
  // The callback must be invoked immediately to acknowledge the message.
  // This is the "Throttle push using acknowledgements" pattern:
  // https://fuchsia.dev/fuchsia-src/concepts/api/fidl#throttle_push_using_acknowledgements
  callback();

  FML_LOG(WARNING) << "memorypressure watcher: OnLevelChanged from "
                   << static_cast<int>(latest_memory_pressure_level_) << " to "
                   << static_cast<int>(level);

  if (latest_memory_pressure_level_ == fuchsia::memorypressure::Level::NORMAL &&
      (level == fuchsia::memorypressure::Level::WARNING ||
       level == fuchsia::memorypressure::Level::CRITICAL)) {
    FML_LOG(WARNING)
        << "memorypressure watcher: notifying Flutter that memory is low";
    shell_->NotifyLowMemoryWarning();
  }
  latest_memory_pressure_level_ = level;
}

}  // namespace flutter_runner
