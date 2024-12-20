// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_COMMON_SETTINGS_H_
#define FLUTTER_COMMON_SETTINGS_H_

#include <fcntl.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "flutter/fml/build_config.h"
#include "flutter/fml/closure.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/fml/unique_fd.h"

namespace flutter {

// The combination of targeted graphics API and Impeller support.
enum class AndroidRenderingAPI {
  kSoftware,
  kImpellerOpenGLES,
  kImpellerVulkan,
  kSkiaOpenGLES
};

class FrameTiming {
 public:
  enum Phase {
    kVsyncStart,
    kBuildStart,
    kBuildFinish,
    kRasterStart,
    kRasterFinish,
    kRasterFinishWallTime,
    kCount
  };

  static constexpr Phase kPhases[kCount] = {
      kVsyncStart,  kBuildStart,   kBuildFinish,
      kRasterStart, kRasterFinish, kRasterFinishWallTime};

  static constexpr int kStatisticsCount = kCount + 5;

  fml::TimePoint Get(Phase phase) const { return data_[phase]; }
  fml::TimePoint Set(Phase phase, fml::TimePoint value) {
    return data_[phase] = value;
  }

  uint64_t GetFrameNumber() const { return frame_number_; }
  void SetFrameNumber(uint64_t frame_number) { frame_number_ = frame_number; }
  uint64_t GetLayerCacheCount() const { return layer_cache_count_; }
  uint64_t GetLayerCacheBytes() const { return layer_cache_bytes_; }
  uint64_t GetPictureCacheCount() const { return picture_cache_count_; }
  uint64_t GetPictureCacheBytes() const { return picture_cache_bytes_; }
  void SetRasterCacheStatistics(size_t layer_cache_count,
                                size_t layer_cache_bytes,
                                size_t picture_cache_count,
                                size_t picture_cache_bytes) {
    layer_cache_count_ = layer_cache_count;
    layer_cache_bytes_ = layer_cache_bytes;
    picture_cache_count_ = picture_cache_count;
    picture_cache_bytes_ = picture_cache_bytes;
  }

 private:
  fml::TimePoint data_[kCount];
  uint64_t frame_number_;
  size_t layer_cache_count_;
  size_t layer_cache_bytes_;
  size_t picture_cache_count_;
  size_t picture_cache_bytes_;
};

using TaskObserverAdd =
    std::function<void(intptr_t /* key */, fml::closure /* callback */)>;
using TaskObserverRemove = std::function<void(intptr_t /* key */)>;
using UnhandledExceptionCallback =
    std::function<bool(const std::string& /* error */,
                       const std::string& /* stack trace */)>;
using LogMessageCallback =
    std::function<void(const std::string& /* tag */,
                       const std::string& /* message */)>;

// TODO(26783): Deprecate all the "path" struct members in favor of the
// callback that generates the mapping from these paths.
using MappingCallback = std::function<std::unique_ptr<fml::Mapping>(void)>;
using Mappings = std::vector<std::unique_ptr<const fml::Mapping>>;
using MappingsCallback = std::function<Mappings(void)>;

using FrameRasterizedCallback = std::function<void(const FrameTiming&)>;

class DartIsolate;

// TODO(https://github.com/flutter/flutter/issues/138750): Re-order fields to
// reduce padding.
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
struct Settings {
  Settings();

  Settings(const Settings& other);

  ~Settings();

  /// Determines if attempts at grabbing the Surface's SurfaceData can be
  /// attempted.
  static constexpr bool kSurfaceDataAccessible =
#ifdef _NDEBUG
      false;
#else
      true;
#endif

  // VM settings
  std::string vm_snapshot_data_path;  // deprecated
  MappingCallback vm_snapshot_data;
  std::string vm_snapshot_instr_path;  // deprecated
  MappingCallback vm_snapshot_instr;

  std::string isolate_snapshot_data_path;  // deprecated
  MappingCallback isolate_snapshot_data;
  std::string isolate_snapshot_instr_path;  // deprecated
  MappingCallback isolate_snapshot_instr;

  std::string route;

  // Returns the Mapping to a kernel buffer which contains sources for dart:*
  // libraries.
  MappingCallback dart_library_sources_kernel;

  // Path to a library containing the application's compiled Dart code.
  // This is a vector so that the embedder can provide fallback paths in
  // case the primary path to the library can not be loaded.
  std::vector<std::string> application_library_path;

  // Path to a library containing compiled Dart code usable for launching
  // the VM service isolate.
  std::vector<std::string> vmservice_snapshot_library_path;

  std::string application_kernel_asset;       // deprecated
  std::string application_kernel_list_asset;  // deprecated
  MappingsCallback application_kernels;

  std::string temp_directory_path;
  std::vector<std::string> dart_flags;
  // Isolate settings
  bool enable_checked_mode = false;
  bool start_paused = false;
  bool trace_skia = false;
  std::vector<std::string> trace_allowlist;
  std::optional<std::vector<std::string>> trace_skia_allowlist;
  bool trace_startup = false;
  bool trace_systrace = false;
  std::string trace_to_file;
  bool enable_timeline_event_handler = true;
  bool dump_skp_on_shader_compilation = false;
  bool cache_sksl = false;
  bool purge_persistent_cache = false;
  bool endless_trace_buffer = false;
  bool enable_dart_profiling = false;
  bool disable_dart_asserts = false;
  bool enable_serial_gc = false;

  // Whether embedder only allows secure connections.
  bool may_insecurely_connect_to_all_domains = true;
  // JSON-formatted domain network policy.
  std::string domain_network_policy;

  // Used as the script URI in debug messages. Does not affect how the Dart code
  // is executed.
  std::string advisory_script_uri = "main.dart";
  // Used as the script entrypoint in debug messages. Does not affect how the
  // Dart code is executed.
  std::string advisory_script_entrypoint = "main";

  // The executable path associated with this process. This is returned by
  // Platform.executable from dart:io. If unknown, defaults to "Flutter".
  std::string executable_name = "Flutter";

  // VM Service settings

  // Whether the Dart VM service should be enabled.
  bool enable_vm_service = false;

  // Whether to publish the VM Service URL over mDNS.
  // On iOS 14 this prompts a local network permission dialog,
  // which cannot be accepted or dismissed in a CI environment.
  bool enable_vm_service_publication = true;

  // The IP address to which the Dart VM service is bound.
  std::string vm_service_host;

  // The port to which the Dart VM service is bound. When set to `0`, a free
  // port will be automatically selected by the OS. A message is logged on the
  // target indicating the URL at which the VM service can be accessed.
  uint32_t vm_service_port = 0;

  // Determines whether an authentication code is required to communicate with
  // the VM service.
  bool disable_service_auth_codes = true;

  // Determine whether the vmservice should fallback to automatic port selection
  // after failing to bind to a specified port.
  bool enable_service_port_fallback = false;

  // Font settings
  bool use_test_fonts = false;

  bool use_asset_fonts = true;

  // Indicates whether the embedding started a prefetch of the default font
  // manager before creating the engine.
  bool prefetched_default_font_manager = false;

  // Enable the rendering of colors outside of the sRGB gamut.
  bool enable_wide_gamut = false;

  // Enable the Impeller renderer on supported platforms. Ignored if Impeller is
  // not supported on the platform.
#if FML_OS_ANDROID || FML_OS_IOS || FML_OS_IOS_SIMULATOR
  // On iOS devices, Impeller is the default with no opt-out and this field is
  // const.
#if FML_OS_IOS || FML_OS_IOS_SIMULATOR
  static constexpr const
#endif                              // FML_OS_IOS && !FML_OS_IOS_SIMULATOR
      bool enable_impeller = true;  // NOLINT(readability-identifier-naming)
#else
  bool enable_impeller = false;
#endif

  // Force disable the android surface control even where supported.
  bool disable_surface_control = false;

  // Log a warning during shell initialization if Impeller is not enabled.
  bool warn_on_impeller_opt_out = false;

  // The selected Android rendering API.
  AndroidRenderingAPI android_rendering_api =
      AndroidRenderingAPI::kSkiaOpenGLES;

  // Requests a specific rendering backend.
  std::optional<std::string> requested_rendering_backend;

  // Enable Vulkan validation on backends that support it. The validation layers
  // must be available to the application.
  bool enable_vulkan_validation = false;

  // Enable GPU tracing in GLES backends.
  // Some devices claim to support the required APIs but crash on their usage.
  bool enable_opengl_gpu_tracing = false;

  // Enable GPU tracing in Vulkan backends.
  bool enable_vulkan_gpu_tracing = false;

  // Data set by platform-specific embedders for use in font initialization.
  uint32_t font_initialization_data = 0;

  // All shells in the process share the same VM. The last shell to shutdown
  // should typically shut down the VM as well. However, applications depend on
  // the behavior of "warming-up" the VM by creating a shell that does not do
  // anything. This used to work earlier when the VM could not be shut down (and
  // hence never was). Shutting down the VM now breaks such assumptions in
  // existing embedders. To keep this behavior consistent and allow existing
  // embedders the chance to migrate, this flag defaults to true. Any shell
  // launched with this flag set to true will leak the VM in the process. There
  // is no way to shut down the VM once such a shell has been started. All
  // shells in the platform (via their embedding APIs) should cooperate to make
  // sure this flag is never set if they want the VM to shutdown and free all
  // associated resources.
  // It can be customized by application, more detail:
  // https://github.com/flutter/flutter/issues/95903
  bool leak_vm = true;

  // Engine settings
  TaskObserverAdd task_observer_add;
  TaskObserverRemove task_observer_remove;
  // The main isolate is current when this callback is made. This is a good spot
  // to perform native Dart bindings for libraries not built in.
  std::function<void(const DartIsolate&)> root_isolate_create_callback;
  // TODO(68738): Update isolate callbacks in settings to accept an additional
  // DartIsolate parameter.
  fml::closure isolate_create_callback;
  // The isolate is not current and may have already been destroyed when this
  // call is made.
  fml::closure root_isolate_shutdown_callback;
  fml::closure isolate_shutdown_callback;
  // A callback made in the isolate scope of the service isolate when it is
  // launched. Care must be taken to ensure that callers are assigning callbacks
  // to the settings object used to launch the VM. If an existing VM is used to
  // launch an isolate using these settings, the callback will be ignored as the
  // service isolate has already been launched. Also, this callback will only be
  // made in the modes in which the service isolate is eligible for launch
  // (debug and profile).
  fml::closure service_isolate_create_callback;
  // The callback made on the UI thread in an isolate scope when the engine
  // detects that the framework is idle. The VM also uses this time to perform
  // tasks suitable when idling. Due to this, embedders are still advised to be
  // as fast as possible in returning from this callback. Long running
  // operations in this callback do have the capability of introducing jank.
  std::function<void(int64_t)> idle_notification_callback;
  // A callback given to the embedder to react to unhandled exceptions in the
  // running Flutter application. This callback is made on an internal engine
  // managed thread and embedders must re-thread as necessary. Performing
  // blocking calls in this callback will cause applications to jank.
  UnhandledExceptionCallback unhandled_exception_callback;
  // A callback given to the embedder to log print messages from the running
  // Flutter application. This callback is made on an internal engine managed
  // thread and embedders must re-thread if necessary. Performing blocking
  // calls in this callback will cause applications to jank.
  LogMessageCallback log_message_callback;
  bool enable_software_rendering = false;
  bool skia_deterministic_rendering_on_cpu = false;
  bool verbose_logging = false;
  std::string log_tag = "flutter";

  // The icu_initialization_required setting does not have a corresponding
  // switch because it is intended to be decided during build time, not runtime.
  // Some companies apply source modification here because their build system
  // brings its own ICU data files.
  bool icu_initialization_required = true;
  std::string icu_data_path;
  MappingCallback icu_mapper;

  // Assets settings
  fml::UniqueFD::element_type assets_dir =
      fml::UniqueFD::traits_type::InvalidValue();
  std::string assets_path;

  // Callback to handle the timings of a rasterized frame. This is called as
  // soon as a frame is rasterized.
  FrameRasterizedCallback frame_rasterized_callback;

  // This data will be available to the isolate immediately on launch via the
  // PlatformDispatcher.getPersistentIsolateData callback. This is meant for
  // information that the isolate cannot request asynchronously (platform
  // messages can be used for that purpose). This data is held for the lifetime
  // of the shell and is available on isolate restarts in the shell instance.
  // Due to this, the buffer must be as small as possible.
  std::shared_ptr<const fml::Mapping> persistent_isolate_data;

  /// Max size of old gen heap size in MB, or 0 for unlimited, -1 for default
  /// value.
  ///
  /// See also:
  /// https://github.com/dart-lang/sdk/blob/ca64509108b3e7219c50d6c52877c85ab6a35ff2/runtime/vm/flag_list.h#L150
  int64_t old_gen_heap_size = -1;

  // Max bytes threshold of resource cache, or 0 for unlimited.
  size_t resource_cache_max_bytes_threshold = 0;

  /// Enable embedder api on the embedder.
  ///
  /// This is currently only used by iOS.
  bool enable_embedder_api = false;

  /// Enable support for isolates that run on the platform thread.
  ///
  /// This is used by the runOnPlatformThread API.
  bool enable_platform_isolates = false;

  // If true, the UI thread is the platform thread on supported
  // platforms.
  bool merged_platform_ui_thread = true;
};

}  // namespace flutter

#endif  // FLUTTER_COMMON_SETTINGS_H_
