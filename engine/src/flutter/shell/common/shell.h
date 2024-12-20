// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_H_
#define FLUTTER_SHELL_COMMON_SHELL_H_

#include <functional>
#include <mutex>
#include <string_view>
#include <unordered_map>

#include "flutter/assets/directory_asset_bundle.h"
#include "flutter/common/graphics/texture.h"
#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/memory/thread_checker.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/status.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/thread.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/lib/ui/painting/image_generator_registry.h"
#include "flutter/lib/ui/semantics/custom_accessibility_action.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/runtime/platform_data.h"
#include "flutter/runtime/service_protocol.h"
#include "flutter/shell/common/animator.h"
#include "flutter/shell/common/display_manager.h"
#include "flutter/shell/common/engine.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/resource_cache_limit_calculator.h"
#include "flutter/shell/common/shell_io_manager.h"
#include "impeller/renderer/context.h"
#include "impeller/runtime_stage/runtime_stage.h"

namespace flutter {

/// Error exit codes for the Dart isolate.
enum class DartErrorCode {
  // NOLINTBEGIN(readability-identifier-naming)
  /// No error has occurred.
  NoError = 0,
  /// The Dart error code for an API error.
  ApiError = 253,
  /// The Dart error code for a compilation error.
  CompilationError = 254,
  /// The Dart error code for an unknown error.
  UnknownError = 255
  // NOLINTEND(readability-identifier-naming)
};

/// Values for |Shell::SetGpuAvailability|.
enum class GpuAvailability {
  /// Indicates that GPU operations should be permitted.
  kAvailable = 0,
  /// Indicates that the GPU is about to become unavailable, and to attempt to
  /// flush any GPU related resources now.
  kFlushAndMakeUnavailable = 1,
  /// Indicates that the GPU is unavailable, and that no attempt should be made
  /// to even flush GPU objects until it is available again.
  kUnavailable = 2
};

//------------------------------------------------------------------------------
/// Perhaps the single most important class in the Flutter engine repository.
/// When embedders create a Flutter application, they are referring to the
/// creation of an instance of a shell. Creation and destruction of the shell is
/// synchronous and the embedder only holds a unique pointer to the shell. The
/// shell does not create the threads its primary components run on. Instead, it
/// is the embedder's responsibility to create threads and give the shell task
/// runners for those threads. Due to deterministic destruction of the shell,
/// the embedder can terminate all threads immediately after collecting the
/// shell. The shell must be created and destroyed on the same thread, but,
/// different shells (i.e. a separate instances of a Flutter application) may be
/// run on different threads simultaneously. The task runners themselves do not
/// have to be unique. If all task runner references given to the shell during
/// shell creation point to the same task runner, the Flutter application is
/// effectively single threaded.
///
/// The shell is the central nervous system of the Flutter application. None of
/// the shell components are thread safe and must be created, accessed and
/// destroyed on the same thread. To interact with one another, the various
/// components delegate to the shell for communication. Instead of using back
/// pointers to the shell, a delegation pattern is used by all components that
/// want to communicate with one another. Because of this, the shell implements
/// the delegate interface for all these components.
///
/// All shell methods accessed by the embedder may only be called on the
/// platform task runner. In case the embedder wants to directly access a shell
/// subcomponent, it is the embedder's responsibility to acquire a weak pointer
/// to that component and post a task to the task runner used by the component
/// to access its methods. The shell must also be destroyed on the platform
/// task runner.
///
/// There is no explicit API to bootstrap and shutdown the Dart VM. The first
/// instance of the shell in the process bootstraps the Dart VM and the
/// destruction of the last shell instance destroys the same. Since different
/// shells may be created and destroyed on different threads. VM bootstrap may
/// happen on one thread but its collection on another. This behavior is thread
/// safe.
///
class Shell final : public PlatformView::Delegate,
                    public Animator::Delegate,
                    public Engine::Delegate,
                    public Rasterizer::Delegate,
                    public ServiceProtocol::Handler,
                    public ResourceCacheLimitItem {
 public:
  template <class T>
  using CreateCallback = std::function<std::unique_ptr<T>(Shell&)>;
  typedef std::function<std::unique_ptr<Engine>(
      Engine::Delegate& delegate,
      const PointerDataDispatcherMaker& dispatcher_maker,
      DartVM& vm,
      fml::RefPtr<const DartSnapshot> isolate_snapshot,
      TaskRunners task_runners,
      const PlatformData& platform_data,
      Settings settings,
      std::unique_ptr<Animator> animator,
      fml::WeakPtr<IOManager> io_manager,
      fml::RefPtr<SkiaUnrefQueue> unref_queue,
      fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
      const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch,
      impeller::RuntimeStageBackend runtime_stage_type)>
      EngineCreateCallback;

  //----------------------------------------------------------------------------
  /// @brief      Creates a shell instance using the provided settings. The
  ///             callbacks to create the various shell subcomponents will be
  ///             called on the appropriate threads before this method returns.
  ///             If this is the first instance of a shell in the process, this
  ///             call also bootstraps the Dart VM.
  /// @note       The root isolate which will run this Shell's Dart code takes
  ///             its instructions from the passed in settings.  This allows
  ///             embedders to host multiple Shells with different Dart code.
  ///
  /// @param[in]  task_runners             The task runners
  /// @param[in]  settings                 The settings
  /// @param[in]  on_create_platform_view  The callback that must return a
  ///                                      platform view. This will be called on
  ///                                      the platform task runner before this
  ///                                      method returns.
  /// @param[in]  on_create_rasterizer     That callback that must provide a
  ///                                      valid rasterizer. This will be called
  ///                                      on the render task runner before this
  ///                                      method returns.
  /// @param[in]  is_gpu_disabled          The default value for the switch that
  ///                                      turns off the GPU.
  ///
  /// @return     A full initialized shell if the settings and callbacks are
  ///             valid. The root isolate has been created but not yet launched.
  ///             It may be launched by obtaining the engine weak pointer and
  ///             posting a task onto the UI task runner with a valid run
  ///             configuration to run the isolate. The embedder must always
  ///             check the validity of the shell (using the IsSetup call)
  ///             immediately after getting a pointer to it.
  ///
  static std::unique_ptr<Shell> Create(
      const PlatformData& platform_data,
      const TaskRunners& task_runners,
      Settings settings,
      const CreateCallback<PlatformView>& on_create_platform_view,
      const CreateCallback<Rasterizer>& on_create_rasterizer,
      bool is_gpu_disabled = false);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the shell. This is a synchronous operation and
  ///             synchronous barrier blocks are introduced on the various
  ///             threads to ensure shutdown of all shell sub-components before
  ///             this method returns.
  ///
  ~Shell();

  //----------------------------------------------------------------------------
  /// @brief      Creates one Shell from another Shell where the created Shell
  ///             takes the opportunity to share any internal components it can.
  ///             This results is a Shell that has a smaller startup time cost
  ///             and a smaller memory footprint than an Shell created with the
  ///             Create function.
  ///
  ///             The new Shell is returned in a running state so RunEngine
  ///             shouldn't be called again on the Shell. Once running, the
  ///             second Shell is mostly independent from the original Shell
  ///             and the original Shell doesn't need to keep running for the
  ///             spawned Shell to keep functioning.
  /// @param[in]  run_configuration  A RunConfiguration used to run the Isolate
  ///             associated with this new Shell. It doesn't have to be the same
  ///             configuration as the current Shell but it needs to be in the
  ///             same snapshot or AOT.
  ///
  /// @see        http://flutter.dev/go/multiple-engines
  std::unique_ptr<Shell> Spawn(
      RunConfiguration run_configuration,
      const std::string& initial_route,
      const CreateCallback<PlatformView>& on_create_platform_view,
      const CreateCallback<Rasterizer>& on_create_rasterizer) const;

  //----------------------------------------------------------------------------
  /// @brief      Starts an isolate for the given RunConfiguration.
  ///
  void RunEngine(RunConfiguration run_configuration);

  //----------------------------------------------------------------------------
  /// @brief      Starts an isolate for the given RunConfiguration. The
  ///             result_callback will be called with the status of the
  ///             operation.
  ///
  void RunEngine(RunConfiguration run_configuration,
                 const std::function<void(Engine::RunStatus)>& result_callback);

  //------------------------------------------------------------------------------
  /// @return     The settings used to launch this shell.
  ///
  const Settings& GetSettings() const override;

  //------------------------------------------------------------------------------
  /// @brief      If callers wish to interact directly with any shell
  ///             subcomponents, they must (on the platform thread) obtain a
  ///             task runner that the component is designed to run on and a
  ///             weak pointer to that component. They may then post a task to
  ///             that task runner, do the validity check on that task runner
  ///             before performing any operation on that component. This
  ///             accessor allows callers to access the task runners for this
  ///             shell.
  ///
  /// @return     The task runners current in use by the shell.
  ///
  const TaskRunners& GetTaskRunners() const override;

  //------------------------------------------------------------------------------
  /// @brief      Getting the raster thread merger from parent shell, it can be
  ///             a null RefPtr when it's a root Shell or the
  ///             embedder_->SupportsDynamicThreadMerging() returns false.
  ///
  /// @return     The raster thread merger used by the parent shell.
  ///
  const fml::RefPtr<fml::RasterThreadMerger> GetParentRasterThreadMerger()
      const override;

  //----------------------------------------------------------------------------
  /// @brief      Rasterizers may only be accessed on the raster task runner.
  ///
  /// @return     A weak pointer to the rasterizer.
  ///
  fml::TaskRunnerAffineWeakPtr<Rasterizer> GetRasterizer() const;

  //------------------------------------------------------------------------------
  /// @brief      Engines may only be accessed on the UI thread. This method is
  ///             deprecated, and implementers should instead use other API
  ///             available on the Shell or the PlatformView.
  ///
  /// @return     A weak pointer to the engine.
  ///
  fml::WeakPtr<Engine> GetEngine();

  //----------------------------------------------------------------------------
  /// @brief      Platform views may only be accessed on the platform task
  ///             runner.
  ///
  /// @return     A weak pointer to the platform view.
  ///
  fml::WeakPtr<PlatformView> GetPlatformView();

  //----------------------------------------------------------------------------
  /// @brief      The IO Manager may only be accessed on the IO task runner.
  ///
  /// @return     A weak pointer to the IO manager.
  ///
  fml::WeakPtr<ShellIOManager> GetIOManager();

  // Embedders should call this under low memory conditions to free up
  // internal caches used.
  //
  // This method posts a task to the raster threads to signal the Rasterizer to
  // free resources.

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to notify that there is a low memory
  ///             warning. The shell will attempt to purge caches. Current, only
  ///             the rasterizer cache is purged.
  void NotifyLowMemoryWarning() const;

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to check if all shell subcomponents are
  ///             initialized. It is the embedder's responsibility to make this
  ///             call before accessing any other shell method. A shell that is
  ///             not set up must be discarded and another one created with
  ///             updated settings.
  ///
  /// @return     Returns if the shell has been set up. Once set up, this does
  ///             not change for the life-cycle of the shell.
  ///
  bool IsSetup() const;

  //----------------------------------------------------------------------------
  /// @brief      Captures a screenshot and optionally Base64 encodes the data
  ///             of the last layer tree rendered by the rasterizer in this
  ///             shell.
  ///
  /// @param[in]  type           The type of screenshot to capture.
  /// @param[in]  base64_encode  If the screenshot data should be base64
  ///                            encoded.
  ///
  /// @return     The screenshot result.
  ///
  Rasterizer::Screenshot Screenshot(Rasterizer::ScreenshotType type,
                                    bool base64_encode);

  //----------------------------------------------------------------------------
  /// @brief      Pauses the calling thread until the first frame is presented.
  ///
  /// @param[in]  timeout  The duration to wait before timing out. If this
  ///                      duration would cause an overflow when added to
  ///                      std::chrono::steady_clock::now(), this method will
  ///                      wait indefinitely for the first frame.
  ///
  /// @return     'kOk' when the first frame has been presented before the
  ///             timeout successfully, 'kFailedPrecondition' if called from the
  ///             GPU or UI thread, 'kDeadlineExceeded' if there is a timeout.
  ///
  fml::Status WaitForFirstFrame(fml::TimeDelta timeout);

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to reload the system fonts in
  ///             FontCollection.
  ///             It also clears the cached font families and send system
  ///             channel message to framework to rebuild affected widgets.
  ///
  /// @return     Returns if shell reloads system fonts successfully.
  ///
  bool ReloadSystemFonts();

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to get the last error from the Dart UI
  ///             Isolate, if one exists.
  ///
  /// @return     Returns the last error code from the UI Isolate.
  ///
  std::optional<DartErrorCode> GetUIIsolateLastError() const;

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to check if the Engine is running and has
  ///             any live ports remaining. For example, the Flutter tester uses
  ///             this method to check whether it should continue to wait for
  ///             a running test or not.
  ///
  /// @return     Returns if the shell has an engine and the engine has any live
  ///             Dart ports.
  ///
  bool EngineHasLivePorts() const;

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to check if the Engine is running and has
  ///             any microtasks that have been queued but have not yet run.
  ///             The Flutter tester uses this as a signal that a test is still
  ///             running.
  ///
  /// @return     Returns if the shell has an engine and the engine has pending
  ///             microtasks.
  ///
  bool EngineHasPendingMicrotasks() const;

  //----------------------------------------------------------------------------
  /// @brief     Accessor for the disable GPU SyncSwitch.
  // |Rasterizer::Delegate|
  std::shared_ptr<const fml::SyncSwitch> GetIsGpuDisabledSyncSwitch()
      const override;

  //----------------------------------------------------------------------------
  /// @brief     Marks the GPU as available or unavailable.
  void SetGpuAvailability(GpuAvailability availability);

  //----------------------------------------------------------------------------
  /// @brief      Get a pointer to the Dart VM used by this running shell
  ///             instance.
  ///
  /// @return     The Dart VM pointer.
  ///
  DartVM* GetDartVM();

  //----------------------------------------------------------------------------
  /// @brief      Notifies the display manager of the updates.
  ///
  void OnDisplayUpdates(std::vector<std::unique_ptr<Display>> displays);

  //----------------------------------------------------------------------------
  /// @brief Queries the `DisplayManager` for the main display refresh rate.
  ///
  double GetMainDisplayRefreshRate();

  //----------------------------------------------------------------------------
  /// @brief      Install a new factory that can match against and decode image
  ///             data.
  /// @param[in]  factory   Callback that produces `ImageGenerator`s for
  ///                       compatible input data.
  /// @param[in]  priority  The priority used to determine the order in which
  ///                       factories are tried. Higher values mean higher
  ///                       priority. The built-in Skia decoders are installed
  ///                       at priority 0, and so a priority > 0 takes precedent
  ///                       over the builtin decoders. When multiple decoders
  ///                       are added with the same priority, those which are
  ///                       added earlier take precedent.
  /// @see        `CreateCompatibleGenerator`
  void RegisterImageDecoder(ImageGeneratorFactory factory, int32_t priority);

  // |Engine::Delegate|
  const std::shared_ptr<PlatformMessageHandler>& GetPlatformMessageHandler()
      const override;

  const std::weak_ptr<VsyncWaiter> GetVsyncWaiter() const;

  const std::shared_ptr<fml::ConcurrentTaskRunner>
  GetConcurrentWorkerTaskRunner() const;

  // Infer the VM ref and the isolate snapshot based on the settings.
  //
  // If the VM is already running, the settings are ignored, but the returned
  // isolate snapshot always prioritize what is specified by the settings, and
  // falls back to the one VM was launched with.
  //
  // This function is what Shell::Create uses to infer snapshot settings.
  //
  // TODO(dkwingsmt): Extracting this method is part of a bigger change. If the
  // entire change is not eventually landed, we should merge this method back
  // to Create. https://github.com/flutter/flutter/issues/136826
  static std::pair<DartVMRef, fml::RefPtr<const DartSnapshot>>
  InferVmInitDataFromSettings(Settings& settings);

 private:
  using ServiceProtocolHandler =
      std::function<bool(const ServiceProtocol::Handler::ServiceProtocolMap&,
                         rapidjson::Document*)>;

  /// A collection of message channels (by name) that have sent at least one
  /// message from a non-platform thread. Used to prevent printing the error
  /// log more than once per channel, as a badly behaving plugin may send
  /// multiple messages per second indefinitely.
  std::mutex misbehaving_message_channels_mutex_;
  std::set<std::string> misbehaving_message_channels_;
  const TaskRunners task_runners_;
  const fml::RefPtr<fml::RasterThreadMerger> parent_raster_thread_merger_;
  std::shared_ptr<ResourceCacheLimitCalculator>
      resource_cache_limit_calculator_;
  size_t resource_cache_limit_;
  const Settings settings_;
  DartVMRef vm_;
  mutable std::mutex time_recorder_mutex_;
  std::optional<fml::TimePoint> latest_frame_target_time_;
  std::unique_ptr<PlatformView> platform_view_;  // on platform task runner
  std::unique_ptr<Engine> engine_;               // on UI task runner
  std::unique_ptr<Rasterizer> rasterizer_;       // on raster task runner
  std::shared_ptr<ShellIOManager> io_manager_;   // on IO task runner
  std::shared_ptr<fml::SyncSwitch> is_gpu_disabled_sync_switch_;
  std::shared_ptr<PlatformMessageHandler> platform_message_handler_;
  std::atomic<bool> route_messages_through_platform_thread_ = false;

  fml::WeakPtr<Engine> weak_engine_;  // to be shared across threads
  fml::TaskRunnerAffineWeakPtr<Rasterizer>
      weak_rasterizer_;  // to be shared across threads
  fml::WeakPtr<PlatformView>
      weak_platform_view_;  // to be shared across threads

  std::unordered_map<std::string_view,  // method
                     std::pair<fml::RefPtr<fml::TaskRunner>,
                               ServiceProtocolHandler>  // task-runner/function
                                                        // pair
                     >
      service_protocol_handlers_;
  bool is_set_up_ = false;
  bool is_added_to_service_protocol_ = false;
  uint64_t next_pointer_flow_id_ = 0;

  bool first_frame_rasterized_ = false;
  std::atomic<bool> waiting_for_first_frame_ = true;
  std::mutex waiting_for_first_frame_mutex_;
  std::condition_variable waiting_for_first_frame_condition_;

  // Written in the UI thread and read from the raster thread. Hence make it
  // atomic.
  std::atomic<bool> needs_report_timings_{false};

  // Whether there's a task scheduled to report the timings to Dart through
  // ui.PlatformDispatcher.onReportTimings.
  bool frame_timings_report_scheduled_ = false;

  // Vector of FrameTiming::kCount * n timestamps for n frames whose timings
  // have not been reported yet. Vector of ints instead of FrameTiming is
  // stored here for easier conversions to Dart objects.
  std::vector<int64_t> unreported_timings_;

  /// Manages the displays. This class is thread safe, can be accessed from
  /// any of the threads.
  std::unique_ptr<DisplayManager> display_manager_;

  // protects expected_frame_size_ which is set on platform thread and read on
  // raster thread
  std::mutex resize_mutex_;

  // used to discard wrong size layer tree produced during interactive
  // resizing
  std::unordered_map<int64_t, SkISize> expected_frame_sizes_;

  // Used to communicate the right frame bounds via service protocol.
  double device_pixel_ratio_ = 0.0;

  // Cached refresh rate used by the performance overlay.
  std::optional<fml::Milliseconds> cached_display_refresh_rate_;

  // How many frames have been timed since last report.
  size_t UnreportedFramesCount() const;

  Shell(DartVMRef vm,
        const TaskRunners& task_runners,
        fml::RefPtr<fml::RasterThreadMerger> parent_merger,
        const std::shared_ptr<ResourceCacheLimitCalculator>&
            resource_cache_limit_calculator,
        const Settings& settings,
        bool is_gpu_disabled);

  static std::unique_ptr<Shell> CreateShellOnPlatformThread(
      DartVMRef vm,
      fml::RefPtr<fml::RasterThreadMerger> parent_merger,
      std::shared_ptr<ShellIOManager> parent_io_manager,
      const std::shared_ptr<ResourceCacheLimitCalculator>&
          resource_cache_limit_calculator,
      const TaskRunners& task_runners,
      const PlatformData& platform_data,
      const Settings& settings,
      fml::RefPtr<const DartSnapshot> isolate_snapshot,
      const Shell::CreateCallback<PlatformView>& on_create_platform_view,
      const Shell::CreateCallback<Rasterizer>& on_create_rasterizer,
      const EngineCreateCallback& on_create_engine,
      bool is_gpu_disabled);

  static std::unique_ptr<Shell> CreateWithSnapshot(
      const PlatformData& platform_data,
      const TaskRunners& task_runners,
      const fml::RefPtr<fml::RasterThreadMerger>& parent_thread_merger,
      const std::shared_ptr<ShellIOManager>& parent_io_manager,
      const std::shared_ptr<ResourceCacheLimitCalculator>&
          resource_cache_limit_calculator,
      Settings settings,
      DartVMRef vm,
      fml::RefPtr<const DartSnapshot> isolate_snapshot,
      const CreateCallback<PlatformView>& on_create_platform_view,
      const CreateCallback<Rasterizer>& on_create_rasterizer,
      const EngineCreateCallback& on_create_engine,
      bool is_gpu_disabled);

  bool Setup(std::unique_ptr<PlatformView> platform_view,
             std::unique_ptr<Engine> engine,
             std::unique_ptr<Rasterizer> rasterizer,
             const std::shared_ptr<ShellIOManager>& io_manager);

  void ReportTimings();

  // |PlatformView::Delegate|
  void OnPlatformViewCreated(std::unique_ptr<Surface> surface) override;

  // |PlatformView::Delegate|
  void OnPlatformViewDestroyed() override;

  // |PlatformView::Delegate|
  void OnPlatformViewScheduleFrame() override;

  // |PlatformView::Delegate|
  void OnPlatformViewAddView(int64_t view_id,
                             const ViewportMetrics& viewport_metrics,
                             AddViewCallback callback) override;

  // |PlatformView::Delegate|
  void OnPlatformViewRemoveView(int64_t view_id,
                                RemoveViewCallback callback) override;

  // |PlatformView::Delegate|
  void OnPlatformViewSetViewportMetrics(
      int64_t view_id,
      const ViewportMetrics& metrics) override;

  // |PlatformView::Delegate|
  void OnPlatformViewDispatchPlatformMessage(
      std::unique_ptr<PlatformMessage> message) override;

  // |PlatformView::Delegate|
  void OnPlatformViewDispatchPointerDataPacket(
      std::unique_ptr<PointerDataPacket> packet) override;

  // |PlatformView::Delegate|
  void OnPlatformViewDispatchSemanticsAction(int32_t node_id,
                                             SemanticsAction action,
                                             fml::MallocMapping args) override;

  // |PlatformView::Delegate|
  void OnPlatformViewSetSemanticsEnabled(bool enabled) override;

  // |shell:PlatformView::Delegate|
  void OnPlatformViewSetAccessibilityFeatures(int32_t flags) override;

  // |PlatformView::Delegate|
  void OnPlatformViewRegisterTexture(
      std::shared_ptr<flutter::Texture> texture) override;

  // |PlatformView::Delegate|
  void OnPlatformViewUnregisterTexture(int64_t texture_id) override;

  // |PlatformView::Delegate|
  void OnPlatformViewMarkTextureFrameAvailable(int64_t texture_id) override;

  // |PlatformView::Delegate|
  void OnPlatformViewSetNextFrameCallback(const fml::closure& closure) override;

  // |PlatformView::Delegate|
  const Settings& OnPlatformViewGetSettings() const override;

  // |PlatformView::Delegate|
  void LoadDartDeferredLibrary(
      intptr_t loading_unit_id,
      std::unique_ptr<const fml::Mapping> snapshot_data,
      std::unique_ptr<const fml::Mapping> snapshot_instructions) override;

  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string error_message,
                                    bool transient) override;

  // |PlatformView::Delegate|
  void UpdateAssetResolverByType(
      std::unique_ptr<AssetResolver> updated_asset_resolver,
      AssetResolver::AssetResolverType type) override;

  // |Animator::Delegate|
  void OnAnimatorBeginFrame(fml::TimePoint frame_target_time,
                            uint64_t frame_number) override;

  // |Animator::Delegate|
  void OnAnimatorNotifyIdle(fml::TimeDelta deadline) override;

  // |Animator::Delegate|
  void OnAnimatorUpdateLatestFrameTargetTime(
      fml::TimePoint frame_target_time) override;

  // |Animator::Delegate|
  void OnAnimatorDraw(std::shared_ptr<FramePipeline> pipeline) override;

  // |Animator::Delegate|
  void OnAnimatorDrawLastLayerTrees(
      std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) override;

  // |Engine::Delegate|
  void OnEngineUpdateSemantics(
      SemanticsNodeUpdates update,
      CustomAccessibilityActionUpdates actions) override;

  // |Engine::Delegate|
  void OnEngineHandlePlatformMessage(
      std::unique_ptr<PlatformMessage> message) override;

  void HandleEngineSkiaMessage(std::unique_ptr<PlatformMessage> message);

  // |Engine::Delegate|
  void OnPreEngineRestart() override;

  // |Engine::Delegate|
  void OnRootIsolateCreated() override;

  // |Engine::Delegate|
  void UpdateIsolateDescription(const std::string isolate_name,
                                int64_t isolate_port) override;

  // |Engine::Delegate|
  void SetNeedsReportTimings(bool value) override;

  // |Engine::Delegate|
  std::unique_ptr<std::vector<std::string>> ComputePlatformResolvedLocale(
      const std::vector<std::string>& supported_locale_data) override;

  // |Engine::Delegate|
  void RequestDartDeferredLibrary(intptr_t loading_unit_id) override;

  // |Engine::Delegate|
  fml::TimePoint GetCurrentTimePoint() override;

  // |Engine::Delegate|
  void OnEngineChannelUpdate(std::string name, bool listening) override;

  // |Engine::Delegate|
  double GetScaledFontSize(double unscaled_font_size,
                           int configuration_id) const override;

  // |Rasterizer::Delegate|
  void OnFrameRasterized(const FrameTiming&) override;

  // |Rasterizer::Delegate|
  fml::Milliseconds GetFrameBudget() override;

  // |Rasterizer::Delegate|
  fml::TimePoint GetLatestFrameTargetTime() const override;

  // |Rasterizer::Delegate|
  bool ShouldDiscardLayerTree(int64_t view_id,
                              const flutter::LayerTree& tree) override;

  // |ServiceProtocol::Handler|
  fml::RefPtr<fml::TaskRunner> GetServiceProtocolHandlerTaskRunner(
      std::string_view method) const override;

  // |ServiceProtocol::Handler|
  bool HandleServiceProtocolMessage(
      std::string_view method,  // one if the extension names specified above.
      const ServiceProtocolMap& params,
      rapidjson::Document* response) override;

  // |ServiceProtocol::Handler|
  ServiceProtocol::Handler::Description GetServiceProtocolDescription()
      const override;

  // Service protocol handler
  bool OnServiceProtocolScreenshot(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolScreenshotSKP(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolRunInView(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolFlushUIThreadTasks(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolSetAssetBundlePath(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolGetDisplayRefreshRate(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  //
  // The returned SkSLs are base64 encoded. Decode before storing them to
  // files.
  bool OnServiceProtocolGetSkSLs(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  bool OnServiceProtocolEstimateRasterCacheMemory(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Service protocol handler
  //
  // Forces the FontCollection to reload the font manifest. Used to support
  // hot reload for fonts.
  bool OnServiceProtocolReloadAssetFonts(
      const ServiceProtocol::Handler::ServiceProtocolMap& params,
      rapidjson::Document* response);

  // Send a system font change notification.
  void SendFontChangeNotification();

  // |ResourceCacheLimitItem|
  size_t GetResourceCacheLimit() override { return resource_cache_limit_; };

  // Creates an asset bundle from the original settings asset path or
  // directory.
  std::unique_ptr<DirectoryAssetBundle> RestoreOriginalAssetResolver();

  SkISize ExpectedFrameSize(int64_t view_id);

  // For accessing the Shell via the raster thread, necessary for various
  // rasterizer callbacks.
  std::unique_ptr<fml::TaskRunnerAffineWeakPtrFactory<Shell>> weak_factory_gpu_;

  fml::WeakPtrFactory<Shell> weak_factory_;
  friend class testing::ShellTest;

  FML_DISALLOW_COPY_AND_ASSIGN(Shell);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SHELL_H_
