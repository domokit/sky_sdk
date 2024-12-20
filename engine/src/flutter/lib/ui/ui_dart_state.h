// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_UI_DART_STATE_H_
#define FLUTTER_LIB_UI_UI_DART_STATE_H_

#include <memory>
#include <string>
#include <utility>

#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/lib/ui/io_manager.h"
#include "flutter/lib/ui/isolate_name_server/isolate_name_server.h"
#include "flutter/lib/ui/painting/image_decoder.h"
#include "flutter/lib/ui/snapshot_delegate.h"
#include "flutter/shell/common/platform_message_handler.h"
#include "impeller/runtime_stage/runtime_stage.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/tonic/dart_microtask_queue.h"
#include "third_party/tonic/dart_persistent_value.h"
#include "third_party/tonic/dart_state.h"

namespace flutter {
class FontSelector;
class ImageGeneratorRegistry;
class PlatformConfiguration;
class PlatformMessage;

class UIDartState : public tonic::DartState {
 public:
  static UIDartState* Current();

  /// @brief  The subset of state which is owned by the shell or engine
  ///         and passed through the RuntimeController into DartIsolates.
  ///         If a shell-owned resource needs to be exposed to the framework via
  ///         UIDartState, a pointer to the resource can be added to this
  ///         struct with appropriate default construction.
  struct Context {
    explicit Context(const TaskRunners& task_runners);

    Context(const TaskRunners& task_runners,
            fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
            fml::WeakPtr<IOManager> io_manager,
            fml::RefPtr<SkiaUnrefQueue> unref_queue,
            fml::WeakPtr<ImageDecoder> image_decoder,
            fml::WeakPtr<ImageGeneratorRegistry> image_generator_registry,
            std::string advisory_script_uri,
            std::string advisory_script_entrypoint,
            bool deterministic_rendering_enabled,
            std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
            bool enable_impeller,
            impeller::RuntimeStageBackend runtime_stage_backend);

    /// The task runners used by the shell hosting this runtime controller. This
    /// may be used by the isolate to scheduled asynchronous texture uploads or
    /// post tasks to the platform task runner.
    const TaskRunners task_runners;

    /// The snapshot delegate used by the
    /// isolate to gather raster snapshots
    /// of Flutter view hierarchies.
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate;

    /// The IO manager used by the isolate for asynchronous texture uploads.
    fml::WeakPtr<IOManager> io_manager;

    /// The unref queue used by the isolate to collect resources that may
    /// reference resources on the GPU.
    fml::RefPtr<SkiaUnrefQueue> unref_queue;

    /// The image decoder.
    fml::WeakPtr<ImageDecoder> image_decoder;

    /// Cascading registry of image generator builders. Given compressed image
    /// bytes as input, this is used to find and create image generators, which
    /// can then be used for image decoding.
    fml::WeakPtr<ImageGeneratorRegistry> image_generator_registry;

    /// The advisory script URI (only used for debugging). This does not affect
    /// the code being run in the isolate in any way.
    std::string advisory_script_uri;

    /// The advisory script entrypoint (only used for debugging). This does not
    /// affect the code being run in the isolate in any way. The isolate must be
    /// transitioned to the running state explicitly by the caller.
    std::string advisory_script_entrypoint;

    /// Whether deterministic rendering practices should be used.
    bool deterministic_rendering_enabled = false;

    /// The task runner whose tasks may be executed concurrently on a pool
    /// of shared worker threads.
    std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner;

    /// Whether Impeller is enabled or not.
    bool enable_impeller = false;

    /// The expected backend for runtime stage shaders.
    impeller::RuntimeStageBackend runtime_stage_backend;
  };

  Dart_Port main_port() const { return main_port_; }
  // Root isolate of the VM application
  bool IsRootIsolate() const { return is_root_isolate_; }
  static void ThrowIfUIOperationsProhibited();

  void SetDebugName(const std::string& name);

  const std::string& debug_name() const { return debug_name_; }

  const std::string& logger_prefix() const { return logger_prefix_; }

  PlatformConfiguration* platform_configuration() const {
    return platform_configuration_.get();
  }

  void SetPlatformMessageHandler(std::weak_ptr<PlatformMessageHandler> handler);

  Dart_Handle HandlePlatformMessage(std::unique_ptr<PlatformMessage> message);

  const TaskRunners& GetTaskRunners() const;

  void ScheduleMicrotask(Dart_Handle handle);

  void FlushMicrotasksNow();

  bool HasPendingMicrotasks();

  fml::WeakPtr<IOManager> GetIOManager() const;

  fml::RefPtr<flutter::SkiaUnrefQueue> GetSkiaUnrefQueue() const;

  std::shared_ptr<fml::ConcurrentTaskRunner> GetConcurrentTaskRunner() const;

  fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> GetSnapshotDelegate() const;

  fml::WeakPtr<ImageDecoder> GetImageDecoder() const;

  fml::WeakPtr<ImageGeneratorRegistry> GetImageGeneratorRegistry() const;

  std::shared_ptr<IsolateNameServer> GetIsolateNameServer() const;

  tonic::DartErrorHandleType GetLastError();

  // Logs `print` messages from the application via an embedder-specified
  // logging mechanism.
  //
  // @param[in]  tag      A component name or tag that identifies the logging
  //                      application.
  // @param[in]  message  The message to be logged.
  void LogMessage(const std::string& tag, const std::string& message) const;

  UnhandledExceptionCallback unhandled_exception_callback() const {
    return unhandled_exception_callback_;
  }

  /// Returns a enumeration that uniquely represents this root isolate.
  /// Returns `0` if called from a non-root isolate.
  int64_t GetRootIsolateToken() const;

  /// Whether deterministic rendering practices are enabled for this application
  bool IsDeterministicRenderingEnabled() const;

  /// Whether Impeller is enabled for this application.
  bool IsImpellerEnabled() const;

  /// The expected type for runtime stage shaders.
  impeller::RuntimeStageBackend GetRuntimeStageBackend() const;

  virtual Dart_Isolate CreatePlatformIsolate(Dart_Handle entry_point,
                                             char** error);

 protected:
  UIDartState(TaskObserverAdd add_callback,
              TaskObserverRemove remove_callback,
              std::string logger_prefix,
              UnhandledExceptionCallback unhandled_exception_callback,
              LogMessageCallback log_message_callback,
              std::shared_ptr<IsolateNameServer> isolate_name_server,
              bool is_root_isolate_,
              const UIDartState::Context& context);

  ~UIDartState() override;

  void SetPlatformConfiguration(
      std::unique_ptr<PlatformConfiguration> platform_configuration);

  const std::string& GetAdvisoryScriptURI() const;

 private:
  void DidSetIsolate() override;

  const TaskObserverAdd add_callback_;
  const TaskObserverRemove remove_callback_;
  const std::string logger_prefix_;
  Dart_Port main_port_ = ILLEGAL_PORT;
  const bool is_root_isolate_;
  std::string debug_name_;
  std::unique_ptr<PlatformConfiguration> platform_configuration_;
  std::weak_ptr<PlatformMessageHandler> platform_message_handler_;
  tonic::DartMicrotaskQueue microtask_queue_;
  UnhandledExceptionCallback unhandled_exception_callback_;
  LogMessageCallback log_message_callback_;
  const std::shared_ptr<IsolateNameServer> isolate_name_server_;
  UIDartState::Context context_;

  void AddOrRemoveTaskObserver(bool add);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_UI_DART_STATE_H_
