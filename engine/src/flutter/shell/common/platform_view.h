// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_PLATFORM_VIEW_H_
#define FLUTTER_SHELL_COMMON_PLATFORM_VIEW_H_

#include <functional>
#include <memory>

#include "flutter/common/graphics/texture.h"
#include "flutter/common/task_runners.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/semantics/custom_accessibility_action.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#include "flutter/lib/ui/window/key_data_packet.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/lib/ui/window/pointer_data_packet.h"
#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/shell/common/platform_message_handler.h"
#include "flutter/shell/common/pointer_data_dispatcher.h"
#include "flutter/shell/common/vsync_waiter.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace impeller {

class Context;

}  // namespace impeller

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Platform views are created by the shell on the platform task
///             runner. Unless explicitly specified, all platform view methods
///             are called on the platform task runner as well. Platform views
///             are usually sub-classed on a per platform basis and the bulk of
///             the window system integration happens using that subclass. Since
///             most platform window toolkits are usually only safe to access on
///             a single "main" thread, any interaction that requires access to
///             the underlying platform's window toolkit is routed through the
///             platform view associated with that shell. This involves
///             operations like settings up and tearing down the render surface,
///             platform messages, interacting with accessibility features on
///             the platform, input events, etc.
///
class PlatformView {
 public:
  using AddViewCallback = std::function<void(bool added)>;
  using RemoveViewCallback = std::function<void(bool removed)>;
  //----------------------------------------------------------------------------
  /// @brief      Used to forward events from the platform view to interested
  ///             subsystems. This forwarding is done by the shell which sets
  ///             itself up as the delegate of the platform view.
  ///
  class Delegate {
   public:
    using AddViewCallback = PlatformView::AddViewCallback;
    using RemoveViewCallback = PlatformView::RemoveViewCallback;
    using KeyDataResponse = std::function<void(bool)>;
    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform view was created
    ///             with the given render surface. This surface is platform
    ///             (iOS, Android) and client-rendering API (OpenGL, Software,
    ///             Metal, Vulkan) specific. This is usually a sign to the
    ///             rasterizer to set up and begin rendering to that surface.
    ///
    /// @param[in]  surface           The surface
    ///
    virtual void OnPlatformViewCreated(std::unique_ptr<Surface> surface) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform view was destroyed.
    ///             This is usually a sign to the rasterizer to suspend
    ///             rendering a previously configured surface and collect any
    ///             intermediate resources.
    ///
    virtual void OnPlatformViewDestroyed() = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform needs to schedule a
    ///             frame to regenerate the layer tree and redraw the surface.
    ///
    virtual void OnPlatformViewScheduleFrame() = 0;

    /// @brief  Allocate resources for a new non-implicit view and inform
    ///         Dart about the view, and on success, schedules a new frame.
    ///
    ///         After the operation, |callback| should be invoked with whether
    ///         the operation is successful.
    ///
    ///         Adding |kFlutterImplicitViewId| or an existing view ID should
    ///         result in failure.
    ///
    /// @param[in]  view_id           The view ID of the new view.
    /// @param[in]  viewport_metrics  The initial viewport metrics for the view.
    /// @param[in]  callback          The callback that's invoked once the shell
    ///                               has attempted to add the view.
    ///
    virtual void OnPlatformViewAddView(int64_t view_id,
                                       const ViewportMetrics& viewport_metrics,
                                       AddViewCallback callback) = 0;

    /// @brief  Deallocate resources for a removed view and inform
    ///         Dart about the removal.
    ///
    ///         After the operation, |callback| should be invoked with whether
    ///         the operation is successful.
    ///
    ///         Removing |kFlutterImplicitViewId| or an non-existent view ID
    ///         should result in failure.
    ///
    /// @param[in]  view_id     The view ID of the view to be removed.
    /// @param[in]  callback    The callback that's invoked once the shell has
    ///                         attempted to remove the view.
    ///
    virtual void OnPlatformViewRemoveView(int64_t view_id,
                                          RemoveViewCallback callback) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the specified callback needs to
    ///             be invoked after the rasterizer is done rendering the next
    ///             frame. This callback will be called on the render thread and
    ///             it is caller responsibility to perform any re-threading as
    ///             necessary. Due to the asynchronous nature of rendering in
    ///             Flutter, embedders usually add a placeholder over the
    ///             contents in which Flutter is going to render when Flutter is
    ///             first initialized. This callback may be used as a signal to
    ///             remove that placeholder.
    ///
    /// @attention  The callback will be invoked on the render thread and not
    ///             the calling thread.
    ///
    /// @param[in]  closure  The callback to execute on the next frame.
    ///
    virtual void OnPlatformViewSetNextFrameCallback(
        const fml::closure& closure) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate the viewport metrics of a view have
    ///             been updated. The rasterizer will need to be reconfigured to
    ///             render the frame in the updated viewport metrics.
    ///
    /// @param[in]  view_id  The ID for the view that `metrics` describes.
    /// @param[in]  metrics  The updated viewport metrics.
    ///
    virtual void OnPlatformViewSetViewportMetrics(
        int64_t view_id,
        const ViewportMetrics& metrics) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform has dispatched a
    ///             platform message from the embedder to the Flutter
    ///             application. This message must be forwarded to the running
    ///             isolate hosted by the engine on the UI thread.
    ///
    /// @param[in]  message  The platform message to dispatch to the running
    ///                      root isolate.
    ///
    virtual void OnPlatformViewDispatchPlatformMessage(
        std::unique_ptr<PlatformMessage> message) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform view has encountered
    ///             a pointer event. This pointer event needs to be forwarded to
    ///             the running root isolate hosted by the engine on the UI
    ///             thread.
    ///
    /// @param[in]  packet  The pointer data packet containing multiple pointer
    ///                     events.
    ///
    virtual void OnPlatformViewDispatchPointerDataPacket(
        std::unique_ptr<PointerDataPacket> packet) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the platform view has encountered
    ///             an accessibility related action on the specified node. This
    ///             event must be forwarded to the running root isolate hosted
    ///             by the engine on the UI thread.
    ///
    /// @param[in]  node_id The identifier of the accessibility node.
    /// @param[in]  action  The accessibility related action performed on the
    ///                     node of the specified ID.
    /// @param[in]  args    An optional list of argument that apply to the
    ///                     specified action.
    ///
    virtual void OnPlatformViewDispatchSemanticsAction(
        int32_t node_id,
        SemanticsAction action,
        fml::MallocMapping args) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the embedder has expressed an
    ///             opinion about whether the accessibility tree needs to be
    ///             enabled or disabled. This information needs to be forwarded
    ///             to the root isolate running on the UI thread.
    ///
    /// @param[in]  enabled  Whether the accessibility tree is enabled or
    ///                      disabled.
    ///
    virtual void OnPlatformViewSetSemanticsEnabled(bool enabled) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the embedder has expressed an
    ///             opinion about the features to enable in the accessibility
    ///             tree.
    ///
    ///             The engine does not care about the accessibility feature
    ///             flags as all it does is forward this information from the
    ///             embedder to the framework. However, curious readers may
    ///             refer to `AccessibilityFeatures` in `window.dart` for
    ///             currently supported accessibility feature flags.
    ///
    /// @param[in]  flags  The features to enable in the accessibility tree.
    ///
    virtual void OnPlatformViewSetAccessibilityFeatures(int32_t flags) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the embedder has specified a
    ///             texture that it want the rasterizer to composite within the
    ///             Flutter layer tree. All textures must have a unique
    ///             identifier. When the rasterizer encounters an external
    ///             texture within its hierarchy, it gives the embedder a chance
    ///             to update that texture on the raster thread before it
    ///             composites the same on-screen.
    ///
    /// @param[in]  texture  The texture that is being updated by the embedder
    ///                      but composited by Flutter in its own hierarchy.
    ///
    virtual void OnPlatformViewRegisterTexture(
        std::shared_ptr<Texture> texture) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the embedder will no longer
    ///             attempt to composite the specified texture within the layer
    ///             tree. This allows the rasterizer to collect associated
    ///             resources.
    ///
    /// @param[in]  texture_id  The identifier of the texture to unregister. If
    ///                         the texture has not been previously registered,
    ///                         this call does nothing.
    ///
    virtual void OnPlatformViewUnregisterTexture(int64_t texture_id) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that the embedder has updated the
    ///             contents of the texture with the specified identifier.
    ///             Typically, Flutter will only render a frame if there is an
    ///             updated layer tree. However, in cases where the layer tree
    ///             is static but one of the externally composited textures has
    ///             been updated by the embedder, the embedder needs to notify
    ///             the rasterizer to render a new frame. In such cases, the
    ///             existing layer tree may be reused with the frame composited
    ///             with all updated external textures.
    ///
    /// @param[in]  texture_id  The identifier of the texture that has been
    ///                         updated.
    ///
    virtual void OnPlatformViewMarkTextureFrameAvailable(
        int64_t texture_id) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Loads the dart shared library into the dart VM. When the
    ///             dart library is loaded successfully, the dart future
    ///             returned by the originating loadLibrary() call completes.
    ///
    ///             The Dart compiler may generate separate shared libraries
    ///             files called 'loading units' when libraries are imported
    ///             as deferred. Each of these shared libraries are identified
    ///             by a unique loading unit id. Callers should open and resolve
    ///             a SymbolMapping from the shared library. The Mappings should
    ///             be moved into this method, as ownership will be assumed by
    ///             the dart root isolate after successful loading and released
    ///             after shutdown of the root isolate. The loading unit may not
    ///             be used after isolate shutdown. If loading fails, the
    ///             mappings will be released.
    ///
    ///             This method is paired with a RequestDartDeferredLibrary
    ///             invocation that provides the embedder with the loading unit
    ///             id of the deferred library to load.
    ///
    ///
    /// @param[in]  loading_unit_id  The unique id of the deferred library's
    ///                              loading unit.
    ///
    /// @param[in]  snapshot_data    Dart snapshot data of the loading unit's
    ///                              shared library.
    ///
    /// @param[in]  snapshot_data    Dart snapshot instructions of the loading
    ///                              unit's shared library.
    ///
    virtual void LoadDartDeferredLibrary(
        intptr_t loading_unit_id,
        std::unique_ptr<const fml::Mapping> snapshot_data,
        std::unique_ptr<const fml::Mapping> snapshot_instructions) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Indicates to the dart VM that the request to load a deferred
    ///             library with the specified loading unit id has failed.
    ///
    ///             The dart future returned by the initiating loadLibrary()
    ///             call will complete with an error.
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
    virtual void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                              const std::string error_message,
                                              bool transient) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Replaces the asset resolver handled by the engine's
    ///             AssetManager of the specified `type` with
    ///             `updated_asset_resolver`. The matching AssetResolver is
    ///             removed and replaced with `updated_asset_resolvers`.
    ///
    ///             AssetResolvers should be updated when the existing resolver
    ///             becomes obsolete and a newer one becomes available that
    ///             provides updated access to the same type of assets as the
    ///             existing one. This update process is meant to be performed
    ///             at runtime.
    ///
    ///             If a null resolver is provided, nothing will be done. If no
    ///             matching resolver is found, the provided resolver will be
    ///             added to the end of the AssetManager resolvers queue. The
    ///             replacement only occurs with the first matching resolver.
    ///             Any additional matching resolvers are untouched.
    ///
    /// @param[in]  updated_asset_resolver  The asset resolver to replace the
    ///             resolver of matching type with.
    ///
    /// @param[in]  type  The type of AssetResolver to update. Only resolvers of
    ///                   the specified type will be replaced by the updated
    ///                   resolver.
    ///
    virtual void UpdateAssetResolverByType(
        std::unique_ptr<AssetResolver> updated_asset_resolver,
        AssetResolver::AssetResolverType type) = 0;

    //--------------------------------------------------------------------------
    /// @brief      Called by the platform view on the platform thread to get
    ///             the settings object associated with the platform view
    ///             instance.
    ///
    /// @return     The settings.
    ///
    virtual const Settings& OnPlatformViewGetSettings() const = 0;
  };

  //----------------------------------------------------------------------------
  /// @brief      Creates a platform view with the specified delegate and task
  ///             runner. The base class by itself does not do much but is
  ///             suitable for use in test environments where full platform
  ///             integration may not be necessary. The platform view may only
  ///             be created, accessed and destroyed on the platform task
  ///             runner.
  ///
  /// @param      delegate      The delegate. This is typically the shell.
  /// @param[in]  task_runners  The task runners used by this platform view.
  ///
  explicit PlatformView(Delegate& delegate, const TaskRunners& task_runners);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the platform view. The platform view is owned by the
  ///             shell and will be destroyed by the same on the platform tasks
  ///             runner.
  ///
  virtual ~PlatformView();

  //----------------------------------------------------------------------------
  /// @brief      Invoked by the shell to obtain a platform specific vsync
  ///             waiter. It is optional for platforms to override this method
  ///             and provide a custom vsync waiter because a timer based
  ///             fall-back waiter is used by default. However, it is highly
  ///             recommended that platform provide their own Vsync waiter as
  ///             the timer based fall-back will not render frames aligned with
  ///             vsync boundaries.
  ///
  /// @attention  If a timer based fall-back is used, a warning is logged to the
  ///             console. In case this method is overridden in a subclass, it
  ///             must return a valid vsync waiter. Returning null will lead to
  ///             internal errors. If a valid vsync waiter cannot be returned,
  ///             subclasses should just call the based class method instead.
  ///
  /// @return     A vsync waiter. If is an internal error to return a null
  ///             waiter.
  ///
  virtual std::unique_ptr<VsyncWaiter> CreateVSyncWaiter();

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to dispatch a platform message to a
  ///             running root isolate hosted by the engine. If an isolate is
  ///             not running, the message is dropped. If there is no one on the
  ///             other side listening on the channel, the message is dropped.
  ///             When a platform message is dropped, any response handles
  ///             associated with that message will be dropped as well. All
  ///             users of platform messages must assume that message may not be
  ///             delivered and/or their response handles may not be invoked.
  ///             Platform messages are not buffered.
  ///
  ///             For embedders that wish to respond to platform message
  ///             directed from the framework to the embedder, the
  ///             `HandlePlatformMessage` method may be overridden.
  ///
  /// @see        HandlePlatformMessage()
  ///
  /// @param[in]  message  The platform message to deliver to the root isolate.
  ///
  void DispatchPlatformMessage(std::unique_ptr<PlatformMessage> message);

  //----------------------------------------------------------------------------
  /// @brief      Overridden by embedders to perform actions in response to
  ///             platform messages sent from the framework to the embedder.
  ///             Default implementation of this method simply returns an empty
  ///             response.
  ///
  ///             Embedders that wish to send platform messages to the framework
  ///             may use the `DispatchPlatformMessage` method. This method is
  ///             for messages that go the other way.
  ///
  /// @see        DispatchPlatformMessage()
  ///
  /// @param[in]  message  The message
  ///
  virtual void HandlePlatformMessage(std::unique_ptr<PlatformMessage> message);

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to dispatch an accessibility action to a
  ///             running isolate hosted by the engine.
  ///
  /// @param[in]  node_id The identifier of the accessibility node on which to
  ///                     perform the action.
  /// @param[in]  action  The action
  /// @param[in]  args    The arguments
  ///
  void DispatchSemanticsAction(int32_t node_id,
                               SemanticsAction action,
                               fml::MallocMapping args);

  //----------------------------------------------------------------------------
  /// @brief      Used by embedder to notify the running isolate hosted by the
  ///             engine on the UI thread that the accessibility tree needs to
  ///             be generated.
  ///
  /// @attention  Subclasses may choose to override this method to perform
  ///             platform specific functions. However, they must call the base
  ///             class method at some point in their implementation.
  ///
  /// @param[in]  enabled  Whether the accessibility tree needs to be generated.
  ///
  virtual void SetSemanticsEnabled(bool enabled);

  //----------------------------------------------------------------------------
  /// @brief      Used by the embedder to specify the features to enable in the
  ///             accessibility tree generated by the isolate. This information
  ///             is forwarded to the root isolate hosted by the engine on the
  ///             UI thread.
  ///
  ///             The engine does not care about the accessibility feature flags
  ///             as all it does is forward this information from the embedder
  ///             to the framework. However, curious readers may refer to
  ///             `AccessibilityFeatures` in `window.dart` for currently
  ///             supported accessibility feature flags.
  ///
  /// @attention  Subclasses may choose to override this method to perform
  ///             platform specific functions. However, they must call the base
  ///             class method at some point in their implementation.
  ///
  /// @param[in]  flags  The features to enable in the accessibility tree.
  ///
  virtual void SetAccessibilityFeatures(int32_t flags);

  //----------------------------------------------------------------------------
  /// @brief      Used by the framework to tell the embedder to apply the
  ///             specified semantics node updates. The default implementation
  ///             of this method does nothing.
  ///
  /// @see        SemanticsNode, SemticsNodeUpdates,
  ///             CustomAccessibilityActionUpdates
  ///
  /// @param[in]  updates  A map with the stable semantics node identifier as
  ///                      key and the node properties as the value.
  /// @param[in]  actions  A map with the stable semantics node identifier as
  ///                      key and the custom node action as the value.
  ///
  virtual void UpdateSemantics(SemanticsNodeUpdates updates,
                               CustomAccessibilityActionUpdates actions);

  //----------------------------------------------------------------------------
  /// @brief      Used by the framework to tell the embedder that it has
  ///             registered a listener on a given channel.
  ///
  /// @param[in]  name      The name of the channel on which the listener has
  ///                       set or cleared a listener.
  /// @param[in]  listening True if a listener has been set, false if it has
  ///                       been cleared.
  ///
  virtual void SendChannelUpdate(const std::string& name, bool listening);

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to specify the updated viewport metrics for
  ///             a view. In response to this call, on the raster thread, the
  ///             rasterizer may need to be reconfigured to the updated viewport
  ///             dimensions. On the UI thread, the framework may need to start
  ///             generating a new frame for the updated viewport metrics as
  ///             well.
  ///
  /// @param[in]  view_id  The ID for the view that `metrics` describes.
  /// @param[in]  metrics  The updated viewport metrics.
  ///
  void SetViewportMetrics(int64_t view_id, const ViewportMetrics& metrics);

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to notify the shell that a platform view
  ///             has been created. This notification is used to create a
  ///             rendering surface and pick the client rendering API to use to
  ///             render into this surface. No frames will be scheduled or
  ///             rendered before this call. The surface must remain valid till
  ///             the corresponding call to NotifyDestroyed.
  ///
  void NotifyCreated();

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to notify the shell that the platform view
  ///             has been destroyed. This notification used to collect the
  ///             rendering surface and all associated resources. Frame
  ///             scheduling is also suspended.
  ///
  /// @attention  Subclasses may choose to override this method to perform
  ///             platform specific functions. However, they must call the base
  ///             class method at some point in their implementation.
  ///
  virtual void NotifyDestroyed();

  //----------------------------------------------------------------------------
  /// @brief      Used by embedders to schedule a frame. In response to this
  ///             call, the framework may need to start generating a new frame.
  ///
  void ScheduleFrame();

  /// @brief  Used by embedders to notify the shell of a new non-implicit view.
  ///
  ///         This method notifies the shell to allocate resources and inform
  ///         Dart about the view, and on success, schedules a new frame.
  ///         Finally, it invokes |callback| with whether the operation is
  ///         successful.
  ///
  ///         This operation is asynchronous; avoid using the view until
  ///         |callback| returns true. Callers should prepare resources for the
  ///         view (if any) in advance but be ready to clean up on failure.
  ///
  ///         The callback is called on a different thread.
  ///
  ///         Do not use for implicit views, which are added internally during
  ///         shell initialization. Adding |kFlutterImplicitViewId| or an
  ///         existing view ID will fail, indicated by |callback| returning
  ///         false.
  ///
  /// @param[in]  view_id           The view ID of the new view.
  /// @param[in]  viewport_metrics  The initial viewport metrics for the view.
  /// @param[in]  callback          The callback that's invoked once the shell
  ///                               has attempted to add the view.
  ///
  void AddView(int64_t view_id,
               const ViewportMetrics& viewport_metrics,
               AddViewCallback callback);

  /// @brief  Used by embedders to notify the shell of a removed non-implicit
  ///         view.
  ///
  ///         This method notifies the shell to deallocate resources and inform
  ///         Dart about the removal. Finally, it invokes |callback| with
  ///         whether the operation is successful.
  ///
  ///         This operation is asynchronous. The embedder should not deallocate
  ///         resources until the |callback| is invoked.
  ///
  ///         The callback is called on a different thread.
  ///
  ///         Do not use for implicit views, which are never removed throughout
  ///         the lifetime of the app.
  ///         Removing |kFlutterImplicitViewId| or an
  ///         non-existent view ID will fail, indicated by |callback| returning
  ///         false.
  ///
  /// @param[in]  view_id     The view ID of the view to be removed.
  /// @param[in]  callback    The callback that's invoked once the shell has
  ///                         attempted to remove the view.
  ///
  void RemoveView(int64_t view_id, RemoveViewCallback callback);

  //----------------------------------------------------------------------------
  /// @brief      Used by the shell to obtain a Skia GPU context that is capable
  ///             of operating on the IO thread. The context must be in the same
  ///             share-group as the Skia GPU context used on the render thread.
  ///             This context will always be used on the IO thread. Because it
  ///             is in the same share-group as the separate render thread
  ///             context, any GPU resources uploaded in this context will be
  ///             visible to the render thread context (synchronization of GPU
  ///             resources is managed by Skia).
  ///
  ///             If such context cannot be created on the IO thread, callers
  ///             may return `nullptr`. This will mean that all texture uploads
  ///             will be queued onto the render thread which will cause
  ///             performance issues. When this context is `nullptr`, an error
  ///             is logged to the console. It is highly recommended that all
  ///             platforms provide a resource context.
  ///
  /// @attention  Unlike all other methods on the platform view, this will be
  ///             called on IO task runner.
  ///
  /// @return     The Skia GPU context that is in the same share-group as the
  ///             main render thread GPU context. May be `nullptr` in case such
  ///             a context cannot be created.
  ///
  virtual sk_sp<GrDirectContext> CreateResourceContext() const;

  virtual std::shared_ptr<impeller::Context> GetImpellerContext() const;

  //----------------------------------------------------------------------------
  /// @brief      Used by the shell to notify the embedder that the resource
  ///             context previously obtained via a call to
  ///             `CreateResourceContext()` is being collected. The embedder
  ///             is free to collect an platform specific resources
  ///             associated with this context.
  ///
  /// @attention  Unlike all other methods on the platform view, this will be
  ///             called on IO task runner.
  ///
  virtual void ReleaseResourceContext() const;

  //--------------------------------------------------------------------------
  /// @brief      Returns a platform-specific PointerDataDispatcherMaker so the
  ///             `Engine` can construct the PointerDataPacketDispatcher based
  ///             on platforms.
  virtual PointerDataDispatcherMaker GetDispatcherMaker();

  //----------------------------------------------------------------------------
  /// @brief      Returns a weak pointer to the platform view. Since the
  ///             platform view may only be created, accessed and destroyed
  ///             on the platform thread, any access to the platform view
  ///             from a non-platform task runner needs a weak pointer to
  ///             the platform view along with a reference to the platform
  ///             task runner. A task must be posted to the platform task
  ///             runner with the weak pointer captured in the same. The
  ///             platform view method may only be called in the posted task
  ///             once the weak pointer validity has been checked. This
  ///             method is used by callers to obtain that weak pointer.
  ///
  /// @return     The weak pointer to the platform view.
  ///
  fml::WeakPtr<PlatformView> GetWeakPtr() const;

  //----------------------------------------------------------------------------
  /// @brief      Gives embedders a chance to react to a "cold restart" of the
  ///             running isolate. The default implementation of this method
  ///             does nothing.
  ///
  ///             While a "hot restart" patches a running isolate, a "cold
  ///             restart" restarts the root isolate in a running shell.
  ///
  virtual void OnPreEngineRestart() const;

  //----------------------------------------------------------------------------
  /// @brief      Sets a callback that gets executed when the rasterizer renders
  ///             the next frame. Due to the asynchronous nature of
  ///             rendering in Flutter, embedders usually add a placeholder
  ///             over the contents in which Flutter is going to render when
  ///             Flutter is first initialized. This callback may be used as
  ///             a signal to remove that placeholder. The callback is
  ///             executed on the render task runner and not the platform
  ///             task runner. It is the embedder's responsibility to
  ///             re-thread as necessary.
  ///
  /// @attention  The callback is executed on the render task runner and not the
  ///             platform task runner. Embedders must re-thread as necessary.
  ///
  /// @param[in]  closure  The callback to execute on the render thread when the
  ///                      next frame gets rendered.
  ///
  void SetNextFrameCallback(const fml::closure& closure);

  //----------------------------------------------------------------------------
  /// @brief      Dispatches pointer events from the embedder to the
  ///             framework. Each pointer data packet may contain multiple
  ///             pointer input events. Each call to this method wakes up
  ///             the UI thread.
  ///
  /// @param[in]  packet  The pointer data packet to dispatch to the framework.
  ///
  void DispatchPointerDataPacket(std::unique_ptr<PointerDataPacket> packet);

  //--------------------------------------------------------------------------
  /// @brief      Used by the embedder to specify a texture that it wants the
  ///             rasterizer to composite within the Flutter layer tree. All
  ///             textures must have a unique identifier. When the
  ///             rasterizer encounters an external texture within its
  ///             hierarchy, it gives the embedder a chance to update that
  ///             texture on the raster thread before it composites the same
  ///             on-screen.
  ///
  /// @attention  This method must only be called once per texture. When the
  ///             texture is updated, calling `MarkTextureFrameAvailable`
  ///             with the specified texture identifier is sufficient to
  ///             make Flutter re-render the frame with the updated texture
  ///             composited in-line.
  ///
  /// @see        UnregisterTexture, MarkTextureFrameAvailable
  ///
  /// @param[in]  texture  The texture that is being updated by the embedder
  ///                      but composited by Flutter in its own hierarchy.
  ///
  void RegisterTexture(std::shared_ptr<flutter::Texture> texture);

  //--------------------------------------------------------------------------
  /// @brief      Used by the embedder to notify the rasterizer that it will
  ///             no longer attempt to composite the specified texture within
  ///             the layer tree. This allows the rasterizer to collect
  ///             associated resources.
  ///
  /// @attention  This call must only be called once per texture identifier.
  ///
  /// @see        RegisterTexture, MarkTextureFrameAvailable
  ///
  /// @param[in]  texture_id  The identifier of the texture to unregister. If
  ///                         the texture has not been previously registered,
  ///                         this call does nothing.
  ///
  void UnregisterTexture(int64_t texture_id);

  //--------------------------------------------------------------------------
  /// @brief      Used by the embedder to notify the rasterizer that the context
  ///             of the previously registered texture have been updated.
  ///             Typically, Flutter will only render a frame if there is an
  ///             updated layer tree. However, in cases where the layer tree
  ///             is static but one of the externally composited textures
  ///             has been updated by the embedder, the embedder needs to
  ///             notify the rasterizer to render a new frame. In such
  ///             cases, the existing layer tree may be reused with the
  ///             frame re-composited with all updated external textures.
  ///             Unlike the calls to register and unregister the texture,
  ///             this call must be made each time a new texture frame is
  ///             available.
  ///
  /// @see        RegisterTexture, UnregisterTexture
  ///
  /// @param[in]  texture_id  The identifier of the texture that has been
  ///                         updated.
  ///
  void MarkTextureFrameAvailable(int64_t texture_id);

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
  ComputePlatformResolvedLocales(
      const std::vector<std::string>& supported_locale_data);

  virtual std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder();

  //--------------------------------------------------------------------------
  /// @brief      Invoked when the dart VM requests that a deferred library
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
  virtual void RequestDartDeferredLibrary(intptr_t loading_unit_id);

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
  ///             dart isolate after successful loading and released after
  ///             shutdown of the dart isolate. If loading fails, the mappings
  ///             will naturally go out of scope.
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
  virtual void LoadDartDeferredLibrary(
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
  ///
  virtual void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                            const std::string error_message,
                                            bool transient);

  //--------------------------------------------------------------------------
  /// @brief      Replaces the asset resolver handled by the engine's
  ///             AssetManager of the specified `type` with
  ///             `updated_asset_resolver`. The matching AssetResolver is
  ///             removed and replaced with `updated_asset_resolvers`.
  ///
  ///             AssetResolvers should be updated when the existing resolver
  ///             becomes obsolete and a newer one becomes available that
  ///             provides updated access to the same type of assets as the
  ///             existing one. This update process is meant to be performed
  ///             at runtime.
  ///
  ///             If a null resolver is provided, nothing will be done. If no
  ///             matching resolver is found, the provided resolver will be
  ///             added to the end of the AssetManager resolvers queue. The
  ///             replacement only occurs with the first matching resolver.
  ///             Any additional matching resolvers are untouched.
  ///
  /// @param[in]  updated_asset_resolver  The asset resolver to replace the
  ///             resolver of matching type with.
  ///
  /// @param[in]  type  The type of AssetResolver to update. Only resolvers of
  ///                   the specified type will be replaced by the updated
  ///                   resolver.
  ///
  virtual void UpdateAssetResolverByType(
      std::unique_ptr<AssetResolver> updated_asset_resolver,
      AssetResolver::AssetResolverType type);

  //--------------------------------------------------------------------------
  /// @brief      Creates an object that produces surfaces suitable for raster
  ///             snapshotting. The rasterizer will request this surface if no
  ///             on screen surface is currently available when an application
  ///             requests a snapshot, e.g. if `Scene.toImage` or
  ///             `Picture.toImage` are called while the application is in the
  ///             background.
  ///
  ///             Not all backends support this kind of surface usage, and the
  ///             default implementation returns nullptr. Platforms should
  ///             override this if they can support GPU operations in the
  ///             background and support GPU resource context usage.
  ///
  virtual std::unique_ptr<SnapshotSurfaceProducer>
  CreateSnapshotSurfaceProducer();

  //--------------------------------------------------------------------------
  /// @brief Specifies a delegate that will receive PlatformMessages from
  /// Flutter to the host platform.
  ///
  /// @details If this returns `null` that means PlatformMessages should be sent
  /// to the PlatformView.  That is to protect legacy behavior, any embedder
  /// that wants to support executing Platform Channel handlers on background
  /// threads should be returning a thread-safe PlatformMessageHandler instead.
  virtual std::shared_ptr<PlatformMessageHandler> GetPlatformMessageHandler()
      const;

  //----------------------------------------------------------------------------
  /// @brief      Get the settings for this platform view instance.
  ///
  /// @return     The settings.
  ///
  const Settings& GetSettings() const;

  //--------------------------------------------------------------------------
  /// @brief      Synchronously invokes platform-specific APIs to apply the
  ///             system text scaling on the given unscaled font size.
  ///
  ///             Platforms that support this feature (currently it's only
  ///             implemented for Android SDK level 34+) will send a valid
  ///             configuration_id to potential callers, before this method can
  ///             be called.
  ///
  /// @param[in]  unscaled_font_size  The unscaled font size specified by the
  ///                                 app developer. The value is in logical
  ///                                 pixels, and is guaranteed to be finite and
  ///                                 non-negative.
  /// @param[in]  configuration_id    The unique id of the configuration to use
  ///                                 for computing the scaled font size.
  ///
  /// @return     The scaled font size in logical pixels, or -1 if the given
  ///             configuration_id did not match a valid configuration.
  ///
  virtual double GetScaledFontSize(double unscaled_font_size,
                                   int configuration_id) const;

 protected:
  // This is the only method called on the raster task runner.
  virtual std::unique_ptr<Surface> CreateRenderingSurface();

  PlatformView::Delegate& delegate_;
  const TaskRunners task_runners_;
  fml::WeakPtrFactory<PlatformView> weak_factory_;  // Must be the last member.

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(PlatformView);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_PLATFORM_VIEW_H_
