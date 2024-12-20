// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_ENGINE_H_
#define FLUTTER_SHELL_COMMON_ENGINE_H_

#include <memory>
#include <string>

#include "flutter/assets/asset_manager.h"
#include "flutter/common/task_runners.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/painting/image_decoder.h"
#include "flutter/lib/ui/painting/image_generator_registry.h"
#include "flutter/lib/ui/semantics/custom_accessibility_action.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#include "flutter/lib/ui/snapshot_delegate.h"
#include "flutter/lib/ui/text/font_collection.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/runtime_controller.h"
#include "flutter/runtime/runtime_delegate.h"
#include "flutter/shell/common/animator.h"
#include "flutter/shell/common/pointer_data_dispatcher.h"
#include "flutter/shell/common/run_configuration.h"

namespace flutter {

//------------------------------------------------------------------------------
/// The engine is a component owned by the shell that resides on the UI task
/// runner and is responsible for managing the needs of the root isolate and its
/// runtime. The engine can only be created, accessed and collected on the UI
/// task runner. Each shell owns exactly one instance of the engine.
///
/// The root isolate of Flutter application gets "window" bindings. Using these
/// bindings, the application can schedule frames, post layer-trees for
/// rendering, ask to decompress images and upload them to the GPU, etc..
/// Non-root isolates of the VM do not get any of these capabilities and are run
/// in a VM managed thread pool (so if they did have "window", the threading
/// guarantees needed for engine operation would be violated).
///
/// The engine is responsible for the entire life-cycle of the root isolate.
/// When the engine is collected, its owner assumes that the root isolate has
/// been shutdown and appropriate resources collected. While each engine
/// instance can only manage a single instance of a root isolate, it may restart
/// that isolate on request. This is how the cold-restart development scenario
/// is supported.
///
/// When the engine instance is initially created, the root isolate is created
/// but it is not in the |DartIsolate::Phase::Running| phase yet. It only moves
/// into that phase when a successful call to `Engine::Run` is made.
///
/// @see      `Shell`
///
/// @note     This name of this class is perhaps a bit unfortunate and has
///           sometimes been the cause of confusion. For a class named "Engine"
///           in the Flutter "Engine" repository, its responsibilities are
///           decidedly unremarkable. But, it does happen to be the primary
///           entry-point used by components higher up in the Flutter tech stack
///           (usually in Dart code) to peer into the lower level functionality.
///           Besides, the authors haven't been able to come up with a more apt
///           name and it does happen to be one of the older classes in the
///           repository.
///
class Engine final : public RuntimeDelegate, PointerDataDispatcher::Delegate {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Indicates the result of the call to `Engine::Run`.
  ///
  enum class RunStatus {
    // NOLINTBEGIN(readability-identifier-naming)
    //--------------------------------------------------------------------------
    /// The call to |Engine::Run| was successful and the root isolate is in the
    /// `DartIsolate::Phase::Running` phase with its entry-point invocation
    /// already pending in the task queue.
    ///
    Success,

    //--------------------------------------------------------------------------
    /// The engine can only manage a single instance of a root isolate. If a
    /// previous call to run the root isolate was successful, subsequent calls
    /// to run the isolate (even if the new run configuration is different) will
    /// be rejected.
    ///
    /// It is up to the caller to decide to re-purpose the running isolate,
    /// terminate it, or use another shell to host the new isolate. This is
    /// mostly used by embedders which have a fire-and-forget strategy to root
    /// isolate launch. For example, the application may try to "launch" an
    /// isolate when the embedders launches or resumes from a paused state. That
    /// the isolate is running is not necessarily a failure condition for them.
    /// But from the engine's perspective, the run configuration was rejected.
    ///
    FailureAlreadyRunning,

    //--------------------------------------------------------------------------
    /// Used to indicate to the embedder that a root isolate was not already
    /// running but the run configuration was not valid and root isolate could
    /// not be moved into the `DartIsolate::Phase::Running` phase.
    ///
    /// The caller must attempt the run call again with a valid configuration.
    /// The set of all failure modes is massive and can originate from a variety
    /// of sub-components. The engine will attempt to log the same when
    /// possible. With the aid of logs, the common causes of failure are:
    ///
    /// * AOT assets were given to JIT/DBC mode VM's and vice-versa.
    /// * The assets could not be found in the asset manager. Callers must make
    ///   sure their run configuration asset managers have been correctly set
    ///   up.
    /// * The assets themselves were corrupt or invalid. Callers must make sure
    ///   their asset delivery mechanisms are sound.
    /// * The application entry-point or the root library of the entry-point
    ///   specified in the run configuration was invalid. Callers must make sure
    ///   that the entry-point is present in the application. If the name of the
    ///   entrypoint is not "main" in the root library, callers must also ensure
    ///   that the snapshotting process has not tree-shaken away this
    ///   entrypoint. This requires the decoration of the entrypoint with the
    ///   `@pragma('vm:entry-point')` directive. This problem will manifest in
    ///   AOT mode operation of the Dart VM.
    ///
    Failure,
    // NOLINTEND(readability-identifier-naming)
  };

  //----------------------------------------------------------------------------
  /// @brief      While the engine operates entirely on the UI task runner, it
  ///             needs the capabilities of the other components to fulfill the
  ///             requirements of the root isolate. The shell is the only class
  ///             that implements this interface as no other component has
  ///             access to all components in a thread safe manner. The engine
  ///             delegates these tasks to the shell via this interface.
  ///
  class Delegate {
   public:
    //--------------------------------------------------------------------------
    /// @brief      When the accessibility tree has been updated by the Flutter
    ///             application, this new information needs to be conveyed to
    ///             the underlying platform. The engine delegates this task to
    ///             the shell via this call. The engine cannot access the
    ///             underlying platform directly because of threading
    ///             considerations. Most platform specific APIs to convey
    ///             accessibility information are only safe to access on the
    ///             platform task runner while the engine is running on the UI
    ///             task runner.
    ///
    /// @see        `SemanticsNode`, `SemanticsNodeUpdates`,
    ///             `CustomAccessibilityActionUpdates`,
    ///             `PlatformView::UpdateSemantics`
    ///
    /// @param[in]  updates  A map with the stable semantics node identifier as
    ///                      key and the node properties as the value.
    /// @param[in]  actions  A map with the stable semantics node identifier as
    ///                      key and the custom node action as the value.
    ///
    virtual void OnEngineUpdateSemantics(
        SemanticsNodeUpdates updates,
        CustomAccessibilityActionUpdates actions) = 0;

    //--------------------------------------------------------------------------
    /// @brief      When the Flutter application has a message to send to the
    ///             underlying platform, the message needs to be forwarded to
    ///             the platform on the appropriate thread (via the platform
    ///             task runner). The engine delegates this task to the shell
    ///             via this method.
    ///
    /// @see        `PlatformView::HandlePlatformMessage`
    ///
    /// @param[in]  message  The message from the Flutter application to send to
    ///                      the underlying platform.
    ///
    virtual void OnEngineHandlePlatformMessage(
        std::unique_ptr<PlatformMessage> message) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the root isolate of the
    ///             application is about to be discarded and a new isolate with
    ///             the same runtime started in its place. This should only
    ///             happen in the Flutter "debug" runtime mode in the
    ///             cold-restart scenario. The embedder may need to reset native
    ///             resource in response to the restart.
    ///
    /// @see        `PlatformView::OnPreEngineRestart`
    ///
    virtual void OnPreEngineRestart() = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the shell that the root isolate is created.
    ///             Currently, this information is to add to the service
    ///             protocol list of available root isolates running in the VM
    ///             and their names so that the appropriate isolate can be
    ///             selected in the tools for debugging and instrumentation.
    ///
    virtual void OnRootIsolateCreated() = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the shell of the name of the root isolate and its
    ///             port when that isolate is launched, restarted (in the
    ///             cold-restart scenario) or the application itself updates the
    ///             name of the root isolate (via
    ///             `PlatformDispatcher.setIsolateDebugName` in
    ///             `platform_dispatcher.dart`). The name of the isolate is
    ///             meaningless to the engine but is used in instrumentation and
    ///             tooling. Currently, this information is to update the
    ///             service protocol list of available root isolates running in
    ///             the VM and their names so that the appropriate isolate can
    ///             be selected in the tools for debugging and instrumentation.
    ///
    /// @param[in]  isolate_name  The isolate name
    /// @param[in]  isolate_port  The isolate port
    ///
    virtual void UpdateIsolateDescription(const std::string isolate_name,
                                          int64_t isolate_port) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the shell that the application has an opinion about
    ///             whether its frame timings need to be reported backed to it.
    ///             Due to the asynchronous nature of rendering in Flutter, it
    ///             is not possible for the application to determine the total
    ///             time it took to render a specific frame. While the
    ///             layer-tree is constructed on the UI thread, it needs to be
    ///             rendering on the raster thread. Dart code cannot execute on
    ///             this thread. So any instrumentation about the frame times
    ///             gathered on this thread needs to be aggregated and sent back
    ///             to the UI thread for processing in Dart.
    ///
    ///             When the application indicates that frame times need to be
    ///             reported, it collects this information till a specified
    ///             number of data points are gathered. Then this information is
    ///             sent back to Dart code via `Engine::ReportTimings`.
    ///
    ///             This option is engine counterpart of the
    ///             `Window._setNeedsReportTimings` in `window.dart`.
    ///
    /// @param[in]  needs_reporting  If reporting information should be
    ///                              collected and send back to Dart.
    ///
    virtual void SetNeedsReportTimings(bool needs_reporting) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Directly invokes platform-specific APIs to compute the
    ///             locale the platform would have natively resolved to.
    ///
    /// @param[in]  supported_locale_data  The vector of strings that represents
    ///                                    the locales supported by the app.
    ///                                    Each locale consists of three
    ///                                    strings: languageCode, countryCode,
    ///                                    and scriptCode in that order.
    ///
    /// @return     A vector of 3 strings languageCode, countryCode, and
    ///             scriptCode that represents the locale selected by the
    ///             platform. Empty strings mean the value was unassigned. Empty
    ///             vector represents a null locale.
    ///
    virtual std::unique_ptr<std::vector<std::string>>
    ComputePlatformResolvedLocale(
        const std::vector<std::string>& supported_locale_data) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Invoked when the Dart VM requests that a deferred library
    ///             be loaded. Notifies the engine that the deferred library
    ///             identified by the specified loading unit id should be
    ///             downloaded and loaded into the Dart VM via
    ///             `LoadDartDeferredLibrary`
    ///
    ///             Upon encountering errors or otherwise failing to load a
    ///             loading unit with the specified id, the failure should be
    ///             directly reported to dart by calling
    ///             `LoadDartDeferredLibraryFailure` to ensure the waiting dart
    ///             future completes with an error.
    ///
    /// @param[in]  loading_unit_id  The unique id of the deferred library's
    ///                              loading unit. This id is to be passed
    ///                              back into LoadDartDeferredLibrary
    ///                              in order to identify which deferred
    ///                              library to load.
    ///
    virtual void RequestDartDeferredLibrary(intptr_t loading_unit_id) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Returns the current fml::TimePoint.
    ///             This method is primarily provided to allow tests to control
    ///             Any methods that rely on advancing the clock.
    virtual fml::TimePoint GetCurrentTimePoint() = 0;

    //----------------------------------------------------------------------------
    /// @brief Returns the delegate object that handles PlatformMessage's from
    ///        Flutter to the host platform (and its responses).
    virtual const std::shared_ptr<PlatformMessageHandler>&
    GetPlatformMessageHandler() const = 0;

    //--------------------------------------------------------------------------
    /// @brief      Invoked when a listener is registered on a platform channel.
    ///
    /// @param[in]  name             The name of the platform channel to which a
    ///                              listener has been registered or cleared.
    ///
    /// @param[in]  listening        Whether the listener has been set (true) or
    ///                              cleared (false).
    ///
    virtual void OnEngineChannelUpdate(std::string name, bool listening) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Synchronously invokes platform-specific APIs to apply the
    ///             system text scaling on the given unscaled font size.
    ///
    ///             Platforms that support this feature (currently it's only
    ///             implemented for Android SDK level 34+) will send a valid
    ///             configuration_id to potential callers, before this method
    ///             can be called.
    ///
    /// @param[in]  unscaled_font_size  The unscaled font size specified by the
    ///                                 app developer. The value is in logical
    ///                                 pixels, and is guaranteed to be finite
    ///                                 and non-negative.
    /// @param[in]  configuration_id    The unique id of the configuration to
    ///                                 use for computing the scaled font size.
    ///
    /// @return     The scaled font size in logical pixels, or -1 when the given
    ///             configuration_id did not match a valid configuration.
    ///
    virtual double GetScaledFontSize(double unscaled_font_size,
                                     int configuration_id) const = 0;
  };

  //----------------------------------------------------------------------------
  /// @brief      Creates an instance of the engine with a supplied
  ///             `RuntimeController`.  Use the other constructor except for
  ///             tests.
  ///
  Engine(Delegate& delegate,
         const PointerDataDispatcherMaker& dispatcher_maker,
         const std::shared_ptr<fml::ConcurrentTaskRunner>&
             image_decoder_task_runner,
         const TaskRunners& task_runners,
         const Settings& settings,
         std::unique_ptr<Animator> animator,
         const fml::WeakPtr<IOManager>& io_manager,
         const std::shared_ptr<FontCollection>& font_collection,
         std::unique_ptr<RuntimeController> runtime_controller,
         const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch);

  //----------------------------------------------------------------------------
  /// @brief      Creates an instance of the engine. This is done by the Shell
  ///             on the UI task runner.
  ///
  /// @param      delegate           The object used by the engine to perform
  ///                                tasks that require access to components
  ///                                that cannot be safely accessed by the
  ///                                engine. This is the shell.
  /// @param      dispatcher_maker   The callback provided by `PlatformView` for
  ///                                engine to create the pointer data
  ///                                dispatcher. Similar to other engine
  ///                                resources, this dispatcher_maker and its
  ///                                returned dispatcher is only safe to be
  ///                                called from the UI thread.
  /// @param      vm                 An instance of the running Dart VM.
  /// @param[in]  isolate_snapshot   The snapshot used to create the root
  ///                                isolate. Even though the isolate is not
  ///                                `DartIsolate::Phase::Running` phase, it is
  ///                                created when the engine is created. This
  ///                                requires access to the isolate snapshot
  ///                                upfront.
  //  TODO(chinmaygarde): This is probably redundant now that the IO manager is
  //  it's own object.
  /// @param[in]  task_runners       The task runners used by the shell that
  ///                                hosts this engine.
  /// @param[in]  settings           The settings used to initialize the shell
  ///                                and the engine.
  /// @param[in]  animator           The animator used to schedule frames.
  //  TODO(chinmaygarde): Move this to `Engine::Delegate`
  /// @param[in]  snapshot_delegate  The delegate used to fulfill requests to
  ///                                snapshot a specified scene. The engine
  ///                                cannot snapshot a scene on the UI thread
  ///                                directly because the scene (described via
  ///                                a `DisplayList`) may reference resources on
  ///                                the GPU and there is no GPU context current
  ///                                on the UI thread. The delegate is a
  ///                                component that has access to all the
  ///                                requisite GPU resources.
  /// @param[in]  io_manager         The IO manager used by this root isolate to
  ///                                schedule tasks that manage resources on the
  ///                                GPU.
  ///
  Engine(Delegate& delegate,
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
         impeller::RuntimeStageBackend runtime_stage_type =
             impeller::RuntimeStageBackend::kSkSL);

  //----------------------------------------------------------------------------
  /// @brief      Create a Engine that shares as many resources as
  ///             possible with the calling Engine such that together
  ///             they occupy less memory and be created faster.
  /// @details    This should only be called on running Engines.
  /// @return     A new Engine with a running isolate.
  /// @see        Engine::Engine
  /// @see        DartIsolate::SpawnIsolate
  ///
  std::unique_ptr<Engine> Spawn(
      Delegate& delegate,
      const PointerDataDispatcherMaker& dispatcher_maker,
      const Settings& settings,
      std::unique_ptr<Animator> animator,
      const std::string& initial_route,
      const fml::WeakPtr<IOManager>& io_manager,
      fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
      const std::shared_ptr<fml::SyncSwitch>& gpu_disabled_switch) const;

  //----------------------------------------------------------------------------
  /// @brief      Destroys the engine engine. Called by the shell on the UI task
  ///             runner. The running root isolate is terminated and will no
  ///             longer access the task runner after this call returns. This
  ///             allows the embedder to tear down the thread immediately if
  ///             needed.
  ///
  ~Engine() override;

  //----------------------------------------------------------------------------
  /// @return     The pointer to this instance of the engine. The engine may
  ///             only be accessed safely on the UI task runner.
  ///
  fml::WeakPtr<Engine> GetWeakPtr() const;

  //----------------------------------------------------------------------------
  /// @brief      Moves the root isolate to the `DartIsolate::Phase::Running`
  ///             phase on a successful call to this method.
  ///
  ///             The isolate itself is created when the engine is created, but
  ///             it is not yet in the running phase. This is done to amortize
  ///             initial time taken to launch the root isolate. The isolate
  ///             snapshots used to run the isolate can be fetched on another
  ///             thread while the engine itself is launched on the UI task
  ///             runner.
  ///
  ///             Repeated calls to this method after a successful run will be
  ///             rejected even if the run configuration is valid (with the
  ///             appropriate error returned).
  ///
  /// @param[in]  configuration  The configuration used to run the root isolate.
  ///                            The configuration must be valid.
  ///
  /// @return     The result of the call to run the root isolate.
  ///
  [[nodiscard]] RunStatus Run(RunConfiguration configuration);

  //----------------------------------------------------------------------------
  /// @brief      Tears down an existing root isolate, reuses the components of
  ///             that isolate and attempts to launch a new isolate using the
  ///             given the run configuration. This is only used in the
  ///             "debug" Flutter runtime mode in the cold-restart scenario.
  ///
  /// @attention  This operation must be performed with care as even a
  ///             non-successful restart will still tear down any existing root
  ///             isolate. In such cases, the engine and its shell must be
  ///             discarded.
  ///
  /// @param[in]  configuration  The configuration used to launch the new
  ///                            isolate.
  ///
  /// @return     Whether the restart was successful. If not, the engine and its
  ///             shell must be discarded.
  ///
  [[nodiscard]] bool Restart(RunConfiguration configuration);

  //----------------------------------------------------------------------------
  /// @brief      Setup default font manager according to specific platform.
  ///
  void SetupDefaultFontManager();

  //----------------------------------------------------------------------------
  /// @brief      Updates the asset manager referenced by the root isolate of a
  ///             Flutter application. This happens implicitly in the call to
  ///             `Engine::Run` and `Engine::Restart` as the asset manager is
  ///             referenced from the run configuration provided to those calls.
  ///             In addition to the `Engine::Run` and `Engine::Restart`
  ///             calls, the tooling may need to update the assets available to
  ///             the application as the user adds them to their project. For
  ///             example, these assets may be referenced by code that is newly
  ///             patched in after a hot-reload. Neither the shell or the
  ///             isolate in relaunched in such cases. The tooling usually
  ///             patches in the new assets in a temporary location and updates
  ///             the asset manager to point to that location.
  ///
  /// @param[in]  asset_manager  The new asset manager to use for the running
  ///                            root isolate.
  ///
  /// @return     If the asset manager was successfully replaced. This may fail
  ///             if the new asset manager is invalid.
  ///
  bool UpdateAssetManager(const std::shared_ptr<AssetManager>& asset_manager);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that it is time to begin working on a new
  ///             frame previously scheduled via a call to
  ///             `Engine::ScheduleFrame`. This call originates in the animator.
  ///
  ///             The frame time given as the argument indicates the point at
  ///             which the current frame interval began. It is very slightly
  ///             (because of scheduling overhead) in the past. If a new layer
  ///             tree is not produced and given to the raster task runner
  ///             within one frame interval from this point, the Flutter
  ///             application will jank.
  ///
  ///             If a root isolate is running, this method calls the
  ///             `::_beginFrame` method in `hooks.dart`. If a root isolate is
  ///             not running, this call does nothing.
  ///
  ///             This method encapsulates the entire UI thread frame workload.
  ///             The following (mis)behavior in the functioning of the method
  ///             will cause the jank in the Flutter application:
  ///             * The time taken by this method to create a layer-tree exceeds
  ///               one frame interval (for example, 16.66 ms on a 60Hz
  ///               display).
  ///             * The time take by this method to generate a new layer-tree
  ///               causes the current layer-tree pipeline depth to change. To
  ///               illustrate this point, note that maximum pipeline depth used
  ///               by layer tree in the engine is 2. If both the UI and GPU
  ///               task runner tasks finish within one frame interval, the
  ///               pipeline depth is one. If the UI thread happens to be
  ///               working on a frame when the raster thread is still not done
  ///               with the previous frame, the pipeline depth is 2. When the
  ///               pipeline depth changes from 1 to 2, animations and UI
  ///               interactions that cause the generation of the new layer tree
  ///               appropriate for (frame_time + one frame interval) will
  ///               actually end up at (frame_time + two frame intervals). This
  ///               is not what code running on the UI thread expected would
  ///               happen. This causes perceptible jank.
  ///
  /// @param[in]  frame_time  The point at which the current frame interval
  ///                         began. May be used by animation interpolators,
  ///                         physics simulations, etc..
  ///
  /// @param[in]  frame_number The frame number recorded by the animator. Used
  ///                          by the framework to associate frame specific
  ///                          debug information with frame timings and timeline
  ///                          events.
  void BeginFrame(fml::TimePoint frame_time, uint64_t frame_number);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the UI task runner is not expected to
  ///             undertake a new frame workload till a specified timepoint. The
  ///             timepoint is measured in microseconds against the system's
  ///             monotonic clock. It is recommended that the clock be accessed
  ///             via `Dart_TimelineGetMicros` from `dart_api.h` for
  ///             consistency. In reality, the clocks used by Dart, FML and
  ///             std::steady_clock are all the same and the timepoints can be
  ///             converted from on clock type to another.
  ///
  ///             The Dart VM uses this notification to schedule book-keeping
  ///             tasks that may include a garbage collection. In this way, it
  ///             is less likely for the VM to perform such (potentially long
  ///             running) tasks in the middle of a frame workload.
  ///
  ///             This notification is advisory. That is, not providing this
  ///             notification does not mean garbage collection is postponed
  ///             till this call is made. If this notification is not provided,
  ///             garbage collection will happen based on the usual heuristics
  ///             used by the Dart VM.
  ///
  ///             Currently, this idle notification is delivered to the engine
  ///             at two points. Once, the deadline is calculated based on how
  ///             much time in the current frame interval is left on the UI task
  ///             runner. Since the next frame workload cannot begin till at
  ///             least the next callback from the vsync waiter, this period may
  ///             be used to used as a "small" idle notification. On the other
  ///             hand, if no more frames are scheduled, a large (but arbitrary)
  ///             idle notification deadline is chosen for a "big" idle
  ///             notification. Again, this notification does not guarantee
  ///             collection, just gives the Dart VM more hints about opportune
  ///             moments to perform collections.
  ///
  ///
  /// @param[in]  deadline  The deadline is used by the VM to determine if the
  ///                       corresponding sweep can be performed within the
  ///                       deadline.
  ///
  void NotifyIdle(fml::TimeDelta deadline);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the attached flutter view has been
  ///             destroyed.
  ///             This enables the engine to notify the Dart VM so it can do
  ///             some cleanp activities.
  void NotifyDestroyed();

  //----------------------------------------------------------------------------
  /// @brief      Dart code cannot fully measure the time it takes for a
  ///             specific frame to be rendered. This is because Dart code only
  ///             runs on the UI task runner. That is only a small part of the
  ///             overall frame workload. The raster task runner frame workload
  ///             is executed on a thread where Dart code cannot run (and hence
  ///             instrument). Besides, due to the pipelined nature of rendering
  ///             in Flutter, there may be multiple frame workloads being
  ///             processed at any given time. However, for non-Timeline based
  ///             profiling, it is useful for trace collection and processing to
  ///             happen in Dart. To do this, the raster task runner frame
  ///             workloads need to be instrumented separately. After a set
  ///             number of these profiles have been gathered, they need to be
  ///             reported back to Dart code. The shell reports this extra
  ///             instrumentation information back to Dart code running on the
  ///             engine by invoking this method at predefined intervals.
  ///
  /// @see        `FrameTiming`
  ///
  //  TODO(chinmaygarde): The use `int64_t` is added for ease of conversion to
  //  Dart but hurts readability. The phases and the units of the timepoints are
  //  not obvious without some sleuthing. The conversion can happen at the
  //  native interface boundary instead.
  ///
  /// @param[in]  timings  Collection of `FrameTiming::kCount` * `n` timestamps
  ///                      for `n` frames whose timings have not been reported
  ///                      yet. A collection of integers is reported here for
  ///                      easier conversions to Dart objects. The timestamps
  ///                      are measured against the system monotonic clock
  ///                      measured in microseconds.
  ///
  void ReportTimings(std::vector<int64_t> timings);

  //----------------------------------------------------------------------------
  /// @brief      Gets the main port of the root isolate. Since the isolate is
  ///             created immediately in the constructor of the engine, it is
  ///             possible to get its main port immediately (even before a call
  ///             to `Run` can be made). This is useful in registering the port
  ///             in a race free manner with a port nameserver.
  ///
  /// @return     The main port of the root isolate.
  ///
  Dart_Port GetUIIsolateMainPort();

  //----------------------------------------------------------------------------
  /// @brief      Gets the debug name of the root isolate. By default, the
  ///             debug name of the isolate is derived from its advisory script
  ///             URI, advisory main entrypoint and its main port name. For
  ///             example, "main.dart$main-1234" where the script URI is
  ///             "main.dart", the entrypoint is "main" and the port name
  ///             "1234". Once launched, the isolate may re-christen itself
  ///             using a name it selects via `setIsolateDebugName` in
  ///             `platform_dispatcher.dart`. This name is purely advisory and
  ///             only used by instrumentation and reporting purposes.
  ///
  /// @return     The debug name of the root isolate.
  ///
  std::string GetUIIsolateName();

  //----------------------------------------------------------------------------
  /// @brief      It is an unexpected challenge to determine when a Dart
  ///             application is "done". The application cannot simply terminate
  ///             the native process (and perhaps return an exit code) because
  ///             it does not have that power. After all, Flutter applications
  ///             reside within a host process that may have other
  ///             responsibilities besides just running Flutter applications.
  ///             Also, the `main` entry-points are run on an event loop and
  ///             returning from "main" (unlike in C/C++ applications) does not
  ///             mean termination of the process. Besides, the return value of
  ///             the main entrypoint is discarded.
  ///
  ///             One technique used by embedders to determine "liveness" is to
  ///             count the outstanding live ports dedicated to the application.
  ///             These ports may be live as a result of pending timers,
  ///             scheduled tasks, pending IO on sockets, channels open with
  ///             other isolates, etc.. At regular intervals (sometimes as often
  ///             as after the UI task runner processes any task), embedders may
  ///             check for the "liveness" of the application and perform
  ///             teardown of the embedder when no more ports are live.
  ///
  /// @return     Check if the root isolate has any live ports.
  ///
  bool UIIsolateHasLivePorts();

  /// @brief      Another signal of liveness is the presence of microtasks that
  ///             have been queued by the application but have not yet been
  ///             executed.  Embedders may want to check for pending microtasks
  ///             and ensure that the microtask queue has been drained before
  ///             the embedder terminates.
  ///
  /// @return     Check if the root isolate has any pending microtasks.
  bool UIIsolateHasPendingMicrotasks();

  //----------------------------------------------------------------------------
  /// @brief      Errors that are unhandled on the Dart message loop are kept
  ///             for further inspection till the next unhandled error comes
  ///             along. This accessor returns the last unhandled error
  ///             encountered by the root isolate.
  ///
  /// @return     The ui isolate last error.
  ///
  tonic::DartErrorHandleType GetUIIsolateLastError();

  //----------------------------------------------------------------------------
  /// @brief      As described in the discussion for `UIIsolateHasLivePorts`,
  ///             the "done-ness" of a Dart application is tricky to ascertain
  ///             and the return value from the main entrypoint is discarded
  ///             (because the Dart isolate is still running after the main
  ///             entrypoint returns). But, the concept of an exit code akin to
  ///             those returned by native applications is still useful. Short
  ///             lived Dart applications (usually tests), emulate this by
  ///             setting a per isolate "return value" and then indicating their
  ///             "done-ness" (usually via closing all live ports). This
  ///             accessor returns that "return value" is present.
  ///
  /// @see        `UIIsolateHasLivePorts`
  ///
  /// @return     The return code (if specified) by the isolate.
  ///
  std::optional<uint32_t> GetUIIsolateReturnCode();

  //----------------------------------------------------------------------------
  /// @brief      Notify the Flutter application that a new view is available.
  ///
  ///             A view must be added before other methods can refer to it,
  ///             including the implicit view. Adding a view that already exists
  ///             triggers an assertion.
  ///
  /// @param[in]  view_id           The ID of the new view.
  /// @param[in]  viewport_metrics  The initial viewport metrics for the view.
  /// @param[in]  callback          Callback that will be invoked once
  ///                               the engine attempts to add the view.
  ///
  void AddView(int64_t view_id,
               const ViewportMetrics& view_metrics,
               std::function<void(bool added)> callback);

  //----------------------------------------------------------------------------
  /// @brief      Notify the Flutter application that a view is no
  ///             longer available.
  ///
  ///             Removing a view that does not exist triggers an assertion.
  ///
  ///             The implicit view (kFlutterImplicitViewId) should never be
  ///             removed. Doing so triggers an assertion.
  ///
  /// @param[in]  view_id  The ID of the view.
  ///
  /// @return     Whether the view was removed.
  ///
  bool RemoveView(int64_t view_id);

  //----------------------------------------------------------------------------
  /// @brief      Updates the viewport metrics for a view. The viewport metrics
  ///             detail the size of the rendering viewport in texels as well as
  ///             edge insets if present.
  ///
  /// @see        `ViewportMetrics`
  ///
  /// @param[in]  view_id  The ID for the view that `metrics` describes.
  /// @param[in]  metrics  The metrics.
  ///
  void SetViewportMetrics(int64_t view_id, const ViewportMetrics& metrics);

  //----------------------------------------------------------------------------
  /// @brief      Updates the display metrics for the currently running Flutter
  ///             application.
  ///
  /// @param[in]  displays  A complete list of displays
  ///
  void SetDisplays(const std::vector<DisplayData>& displays);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the embedder has sent it a message.
  ///             This call originates in the platform view and has been
  ///             forwarded to the engine on the UI task runner here.
  ///
  /// @param[in]  message  The message sent from the embedder to the Dart
  ///                      application.
  ///
  void DispatchPlatformMessage(std::unique_ptr<PlatformMessage> message);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the embedder has sent it a pointer
  ///             data packet. A pointer data packet may contain multiple
  ///             input events. This call originates in the platform view and
  ///             the shell has forwarded the same to the engine on the UI task
  ///             runner here.
  ///
  /// @param[in]  packet         The pointer data packet containing multiple
  ///                            input events.
  /// @param[in]  trace_flow_id  The trace flow identifier associated with the
  ///                            pointer data packet. The engine uses this trace
  ///                            identifier to connect trace flows in the
  ///                            timeline from the input event to the
  ///                            frames generated due to those input events.
  ///                            These flows are tagged as "PointerEvent" in the
  ///                            timeline and allow grouping frames and input
  ///                            events into logical chunks.
  ///
  void DispatchPointerDataPacket(std::unique_ptr<PointerDataPacket> packet,
                                 uint64_t trace_flow_id);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the embedder encountered an
  ///             accessibility related action on the specified node. This call
  ///             originates on the platform view and has been forwarded to the
  ///             engine here on the UI task runner by the shell.
  ///
  /// @param[in]  node_id The identifier of the accessibility node.
  /// @param[in]  action  The accessibility related action performed on the
  ///                     node of the specified ID.
  /// @param[in]  args    Optional data that applies to the specified action.
  ///
  void DispatchSemanticsAction(int node_id,
                               SemanticsAction action,
                               fml::MallocMapping args);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the embedder has expressed an opinion
  ///             about whether the accessibility tree should be generated or
  ///             not. This call originates in the platform view and is
  ///             forwarded to the engine here on the UI task runner by the
  ///             shell.
  ///
  /// @param[in]  enabled  Whether the accessibility tree is enabled or
  ///                      disabled.
  ///
  void SetSemanticsEnabled(bool enabled);

  //----------------------------------------------------------------------------
  /// @brief      Notifies the engine that the embedder has expressed an opinion
  ///             about where the flags to set on the accessibility tree. This
  ///             flag originates in the platform view and is forwarded to the
  ///             engine here on the UI task runner by the shell.
  ///
  ///             The engine does not care about the accessibility feature flags
  ///             as all it does is forward this information from the embedder
  ///             to the framework. However, curious readers may refer to
  ///             `AccessibilityFeatures` in `window.dart` for currently
  ///             supported accessibility feature flags.
  ///
  /// @param[in]  flags  The features to enable in the accessibility tree.
  ///
  void SetAccessibilityFeatures(int32_t flags);

  // |RuntimeDelegate|
  void ScheduleFrame(bool regenerate_layer_trees) override;

  /// Schedule a frame with the default parameter of regenerating the layer
  /// tree.
  void ScheduleFrame() { ScheduleFrame(true); }

  // |RuntimeDelegate|
  void OnAllViewsRendered() override;

  // |RuntimeDelegate|
  FontCollection& GetFontCollection() override;

  // |RuntimeDelegate|
  std::shared_ptr<AssetManager> GetAssetManager() override;

  // Return the weak_ptr of ImageDecoder.
  fml::WeakPtr<ImageDecoder> GetImageDecoderWeakPtr();

  //----------------------------------------------------------------------------
  /// @brief      Get the `ImageGeneratorRegistry` associated with the current
  ///             engine.
  ///
  /// @return     The engine's `ImageGeneratorRegistry`.
  ///
  fml::WeakPtr<ImageGeneratorRegistry> GetImageGeneratorRegistry();

  // |PointerDataDispatcher::Delegate|
  void DoDispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                        uint64_t trace_flow_id) override;

  // |PointerDataDispatcher::Delegate|
  void ScheduleSecondaryVsyncCallback(uintptr_t id,
                                      const fml::closure& callback) override;

  //----------------------------------------------------------------------------
  /// @brief      Get the last Entrypoint that was used in the RunConfiguration
  ///             when |Engine::Run| was called.
  ///
  const std::string& GetLastEntrypoint() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the last Entrypoint Library that was used in the
  ///             RunConfiguration when |Engine::Run| was called.
  ///
  const std::string& GetLastEntrypointLibrary() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the last Entrypoint Arguments that was used in the
  ///             RunConfiguration when |Engine::Run| was called.This is only
  ///             valid in debug mode.
  ///
  const std::vector<std::string>& GetLastEntrypointArgs() const;

  //----------------------------------------------------------------------------
  /// @brief      Getter for the initial route.  This can be set with a platform
  ///             message.
  ///
  const std::string& InitialRoute() const { return initial_route_; }

  //--------------------------------------------------------------------------
  /// @brief      Loads the Dart shared library into the Dart VM. When the
  ///             Dart library is loaded successfully, the Dart future
  ///             returned by the originating loadLibrary() call completes.
  ///
  ///             The Dart compiler may generate separate shared libraries
  ///             files called 'loading units' when libraries are imported
  ///             as deferred. Each of these shared libraries are identified
  ///             by a unique loading unit id. Callers should open and resolve
  ///             a SymbolMapping from the shared library. The Mappings should
  ///             be moved into this method, as ownership will be assumed by the
  ///             dart root isolate after successful loading and released after
  ///             shutdown of the root isolate. The loading unit may not be
  ///             used after isolate shutdown. If loading fails, the mappings
  ///             will be released.
  ///
  ///             This method is paired with a RequestDartDeferredLibrary
  ///             invocation that provides the embedder with the loading unit id
  ///             of the deferred library to load.
  ///
  ///
  /// @param[in]  loading_unit_id  The unique id of the deferred library's
  ///                              loading unit, as passed in by
  ///                              RequestDartDeferredLibrary.
  ///
  /// @param[in]  snapshot_data    Dart snapshot data of the loading unit's
  ///                              shared library.
  ///
  /// @param[in]  snapshot_data    Dart snapshot instructions of the loading
  ///                              unit's shared library.
  ///
  void LoadDartDeferredLibrary(
      intptr_t loading_unit_id,
      std::unique_ptr<const fml::Mapping> snapshot_data,
      std::unique_ptr<const fml::Mapping> snapshot_instructions);

  //--------------------------------------------------------------------------
  /// @brief      Indicates to the dart VM that the request to load a deferred
  ///             library with the specified loading unit id has failed.
  ///
  ///             The dart future returned by the initiating loadLibrary() call
  ///             will complete with an error.
  ///
  /// @param[in]  loading_unit_id  The unique id of the deferred library's
  ///                              loading unit, as passed in by
  ///                              RequestDartDeferredLibrary.
  ///
  /// @param[in]  error_message    The error message that will appear in the
  ///                              dart Future.
  ///
  /// @param[in]  transient        A transient error is a failure due to
  ///                              temporary conditions such as no network.
  ///                              Transient errors allow the dart VM to
  ///                              re-request the same deferred library and
  ///                              loading_unit_id again. Non-transient
  ///                              errors are permanent and attempts to
  ///                              re-request the library will instantly
  ///                              complete with an error.
  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string& error_message,
                                    bool transient);

  //--------------------------------------------------------------------------
  /// @brief      Accessor for the RuntimeController.
  ///
  const RuntimeController* GetRuntimeController() const {
    return runtime_controller_.get();
  }

  const std::weak_ptr<VsyncWaiter> GetVsyncWaiter() const;

  //--------------------------------------------------------------------------
  /// @brief      Shuts down all registered platform isolates. Must be called
  ///             from the platform thread.
  ///
  void ShutdownPlatformIsolates();

 private:
  // |RuntimeDelegate|
  std::string DefaultRouteName() override;

  // |RuntimeDelegate|
  void Render(int64_t view_id,
              std::unique_ptr<flutter::LayerTree> layer_tree,
              float device_pixel_ratio) override;

  // |RuntimeDelegate|
  void UpdateSemantics(SemanticsNodeUpdates update,
                       CustomAccessibilityActionUpdates actions) override;

  // |RuntimeDelegate|
  void HandlePlatformMessage(std::unique_ptr<PlatformMessage> message) override;

  // |RuntimeDelegate|
  void OnRootIsolateCreated() override;

  // |RuntimeDelegate|
  void UpdateIsolateDescription(const std::string isolate_name,
                                int64_t isolate_port) override;

  // |RuntimeDelegate|
  std::unique_ptr<std::vector<std::string>> ComputePlatformResolvedLocale(
      const std::vector<std::string>& supported_locale_data) override;

  // |RuntimeDelegate|
  void RequestDartDeferredLibrary(intptr_t loading_unit_id) override;

  // |RuntimeDelegate|
  std::weak_ptr<PlatformMessageHandler> GetPlatformMessageHandler()
      const override;

  // |RuntimeDelegate|
  void SendChannelUpdate(std::string name, bool listening) override;

  // |RuntimeDelegate|
  double GetScaledFontSize(double unscaled_font_size,
                           int configuration_id) const override;

  void SetNeedsReportTimings(bool value) override;

  bool HandleLifecyclePlatformMessage(PlatformMessage* message);

  bool HandleNavigationPlatformMessage(
      std::unique_ptr<PlatformMessage> message);

  bool HandleLocalizationPlatformMessage(PlatformMessage* message);

  void HandleSettingsPlatformMessage(PlatformMessage* message);

  void HandleAssetPlatformMessage(std::unique_ptr<PlatformMessage> message);

  bool GetAssetAsBuffer(const std::string& name, std::vector<uint8_t>* data);

  friend class testing::ShellTest;

  Engine::Delegate& delegate_;
  const Settings settings_;
  std::unique_ptr<Animator> animator_;
  std::unique_ptr<RuntimeController> runtime_controller_;

  // The pointer_data_dispatcher_ depends on animator_ and runtime_controller_.
  // So it should be defined after them to ensure that pointer_data_dispatcher_
  // is destructed first.
  std::unique_ptr<PointerDataDispatcher> pointer_data_dispatcher_;

  std::string last_entry_point_;
  std::string last_entry_point_library_;
  std::vector<std::string> last_entry_point_args_;
  std::string initial_route_;
  std::shared_ptr<AssetManager> asset_manager_;
  std::shared_ptr<FontCollection> font_collection_;
  std::shared_ptr<NativeAssetsManager> native_assets_manager_;
  const std::unique_ptr<ImageDecoder> image_decoder_;
  ImageGeneratorRegistry image_generator_registry_;
  TaskRunners task_runners_;
  fml::WeakPtrFactory<Engine> weak_factory_;  // Must be the last member.
  FML_DISALLOW_COPY_AND_ASSIGN(Engine);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_ENGINE_H_
