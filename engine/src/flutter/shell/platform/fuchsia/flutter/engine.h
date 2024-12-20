// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ENGINE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ENGINE_H_

#include <optional>

#include <fuchsia/intl/cpp/fidl.h>
#include <fuchsia/io/cpp/fidl.h>
#include <fuchsia/memorypressure/cpp/fidl.h>
#include <fuchsia/ui/composition/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/sys/cpp/service_directory.h>

#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/common/shell.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/platform/fuchsia/flutter/accessibility_bridge.h"

#include "external_view_embedder.h"
#include "flatland_connection.h"
#include "flutter_runner_product_configuration.h"
#include "isolate_configurator.h"
#include "surface_producer.h"

namespace flutter_runner {

namespace testing {
class EngineTest;
}

// Represents an instance of running Flutter engine along with the threads
// that host the same.
class Engine final : public fuchsia::memorypressure::Watcher {
 public:
  class Delegate {
   public:
    virtual void OnEngineTerminate(const Engine* holder) = 0;
  };

  static flutter::ThreadHost CreateThreadHost(
      const std::string& name_prefix,
      const std::shared_ptr<sys::ServiceDirectory>& runner_services = nullptr);

  Engine(Delegate& delegate,
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
         const std::vector<std::string>& dart_entrypoint_args);

  ~Engine();

  // Returns the Dart return code for the root isolate if one is present. This
  // call is thread safe and synchronous. This call must be made infrequently.
  std::optional<uint32_t> GetEngineReturnCode() const;

#if !defined(DART_PRODUCT)
  void WriteProfileToTrace() const;
#endif  // !defined(DART_PRODUCT)

 private:
  void Initialize(
      std::pair<fuchsia::ui::views::ViewRefControl, fuchsia::ui::views::ViewRef>
          view_ref_pair,
      std::shared_ptr<sys::ServiceDirectory> svc,
      std::shared_ptr<sys::ServiceDirectory> runner_services,
      flutter::Settings settings,
      UniqueFDIONS fdio_ns,
      fidl::InterfaceRequest<fuchsia::io::Directory> directory_request,
      FlutterRunnerProductConfiguration product_config,
      const std::vector<std::string>& dart_entrypoint_args);

  static void WarmupSkps(
      fml::BasicTaskRunner* concurrent_task_runner,
      fml::BasicTaskRunner* raster_task_runner,
      std::shared_ptr<SurfaceProducer> surface_producer,
      SkISize size,
      std::shared_ptr<flutter::AssetManager> asset_manager,
      std::optional<const std::vector<std::string>> skp_names,
      std::optional<std::function<void(uint32_t)>> completion_callback,
      bool synchronous = false);

  void OnMainIsolateStart();

  void OnMainIsolateShutdown();

  void Terminate();

  void DebugWireframeSettingsChanged(bool enabled);
  void CreateView(int64_t view_id,
                  ViewCallback on_view_created,
                  ViewCreatedCallback on_view_bound,
                  bool hit_testable,
                  bool focusable);
  void UpdateView(int64_t view_id,
                  SkRect occlusion_hint,
                  bool hit_testable,
                  bool focusable);
  void DestroyView(int64_t view_id, ViewIdCallback on_view_unbound);

  // |fuchsia::memorypressure::Watcher|
  void OnLevelChanged(fuchsia::memorypressure::Level level,
                      fuchsia::memorypressure::Watcher::OnLevelChangedCallback
                          callback) override;

  std::shared_ptr<flutter::ExternalViewEmbedder> GetExternalViewEmbedder();

  std::unique_ptr<flutter::Surface> CreateSurface();

  Delegate& delegate_;

  const std::string thread_label_;
  flutter::ThreadHost thread_host_;

  fuchsia::ui::views::ViewCreationToken view_creation_token_;
  std::shared_ptr<FlatlandConnection>
      flatland_connection_;  // Must come before surface_producer_
  std::shared_ptr<SurfaceProducer> surface_producer_;
  std::shared_ptr<ExternalViewEmbedder> view_embedder_;

  std::unique_ptr<IsolateConfigurator> isolate_configurator_;
  std::unique_ptr<flutter::Shell> shell_;
  std::unique_ptr<AccessibilityBridge> accessibility_bridge_;

  fuchsia::intl::PropertyProviderPtr intl_property_provider_;

  fuchsia::memorypressure::ProviderPtr memory_pressure_provider_;
  fidl::Binding<fuchsia::memorypressure::Watcher>
      memory_pressure_watcher_binding_;
  // We need to track the latest memory pressure level to determine
  // the direction of change when a new level is provided.
  fuchsia::memorypressure::Level latest_memory_pressure_level_;

  bool intercept_all_input_ = false;

  fml::WeakPtrFactory<Engine> weak_factory_;
  friend class testing::EngineTest;

  FML_DISALLOW_COPY_AND_ASSIGN(Engine);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ENGINE_H_
