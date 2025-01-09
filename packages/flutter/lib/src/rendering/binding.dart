// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'layer.dart';
library;

import 'dart:ui' as ui show PictureRecorder, SceneBuilder, SemanticsUpdate;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';

import 'debug.dart';
import 'mouse_tracker.dart';
import 'object.dart';
import 'service_extensions.dart';
import 'view.dart';

export 'package:flutter/gestures.dart' show HitTestResult;

// Examples can assume:
// late BuildContext context;

/// The glue between the render trees and the Flutter engine.
///
/// The [RendererBinding] manages multiple independent render trees. Each render
/// tree is rooted in a [RenderView] that must be added to the binding via
/// [addRenderView] to be considered during frame production, hit testing, etc.
/// Furthermore, the render tree must be managed by a [PipelineOwner] that is
/// part of the pipeline owner tree rooted at [rootPipelineOwner].
///
/// Adding [PipelineOwner]s and [RenderView]s to this binding in the way
/// described above is left as a responsibility for a higher level abstraction.
/// The widgets library, for example, introduces the [View] widget, which
/// registers its [RenderView] and [PipelineOwner] with this binding.
mixin RendererBinding
    on
        BindingBase,
        ServicesBinding,
        SchedulerBinding,
        GestureBinding,
        SemanticsBinding,
        HitTestable {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    _rootPipelineOwner = createRootPipelineOwner();
    platformDispatcher
      ..onMetricsChanged = handleMetricsChanged
      ..onTextScaleFactorChanged = handleTextScaleFactorChanged
      ..onPlatformBrightnessChanged = handlePlatformBrightnessChanged;
    addPersistentFrameCallback(_handlePersistentFrameCallback);
    initMouseTracker();
    if (kIsWeb) {
      addPostFrameCallback(_handleWebFirstFrame, debugLabel: 'RendererBinding.webFirstFrame');
    }
    rootPipelineOwner.attach(_manifold);
  }

  /// The current [RendererBinding], if one has been created.
  ///
  /// Provides access to the features exposed by this mixin. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [runApp] or [WidgetsFlutterBinding.ensureInitialized].
  static RendererBinding get instance => BindingBase.checkInstance(_instance);
  static RendererBinding? _instance;

  @override
  void initServiceExtensions() {
    super.initServiceExtensions();

    assert(() {
      // these service extensions only work in debug mode
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.invertOversizedImages.name,
        getter: () async => debugInvertOversizedImages,
        setter: (bool value) async {
          if (debugInvertOversizedImages != value) {
            debugInvertOversizedImages = value;
            return _forceRepaint();
          }
          return Future<void>.value();
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.debugPaint.name,
        getter: () async => debugPaintSizeEnabled,
        setter: (bool value) {
          if (debugPaintSizeEnabled == value) {
            return Future<void>.value();
          }
          debugPaintSizeEnabled = value;
          return _forceRepaint();
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.debugPaintBaselinesEnabled.name,
        getter: () async => debugPaintBaselinesEnabled,
        setter: (bool value) {
          if (debugPaintBaselinesEnabled == value) {
            return Future<void>.value();
          }
          debugPaintBaselinesEnabled = value;
          return _forceRepaint();
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.repaintRainbow.name,
        getter: () async => debugRepaintRainbowEnabled,
        setter: (bool value) {
          final bool repaint = debugRepaintRainbowEnabled && !value;
          debugRepaintRainbowEnabled = value;
          if (repaint) {
            return _forceRepaint();
          }
          return Future<void>.value();
        },
      );
      registerServiceExtension(
        name: RenderingServiceExtensions.debugDumpLayerTree.name,
        callback: (Map<String, String> parameters) async {
          return <String, Object>{'data': _debugCollectLayerTrees()};
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.debugDisableClipLayers.name,
        getter: () async => debugDisableClipLayers,
        setter: (bool value) {
          if (debugDisableClipLayers == value) {
            return Future<void>.value();
          }
          debugDisableClipLayers = value;
          return _forceRepaint();
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.debugDisablePhysicalShapeLayers.name,
        getter: () async => debugDisablePhysicalShapeLayers,
        setter: (bool value) {
          if (debugDisablePhysicalShapeLayers == value) {
            return Future<void>.value();
          }
          debugDisablePhysicalShapeLayers = value;
          return _forceRepaint();
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.debugDisableOpacityLayers.name,
        getter: () async => debugDisableOpacityLayers,
        setter: (bool value) {
          if (debugDisableOpacityLayers == value) {
            return Future<void>.value();
          }
          debugDisableOpacityLayers = value;
          return _forceRepaint();
        },
      );
      return true;
    }());

    if (!kReleaseMode) {
      // these service extensions work in debug or profile mode
      registerServiceExtension(
        name: RenderingServiceExtensions.debugDumpRenderTree.name,
        callback: (Map<String, String> parameters) async {
          return <String, Object>{'data': _debugCollectRenderTrees()};
        },
      );
      registerServiceExtension(
        name: RenderingServiceExtensions.debugDumpSemanticsTreeInTraversalOrder.name,
        callback: (Map<String, String> parameters) async {
          return <String, Object>{
            'data': _debugCollectSemanticsTrees(DebugSemanticsDumpOrder.traversalOrder),
          };
        },
      );
      registerServiceExtension(
        name: RenderingServiceExtensions.debugDumpSemanticsTreeInInverseHitTestOrder.name,
        callback: (Map<String, String> parameters) async {
          return <String, Object>{
            'data': _debugCollectSemanticsTrees(DebugSemanticsDumpOrder.inverseHitTest),
          };
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.profileRenderObjectPaints.name,
        getter: () async => debugProfilePaintsEnabled,
        setter: (bool value) async {
          if (debugProfilePaintsEnabled != value) {
            debugProfilePaintsEnabled = value;
          }
        },
      );
      registerBoolServiceExtension(
        name: RenderingServiceExtensions.profileRenderObjectLayouts.name,
        getter: () async => debugProfileLayoutsEnabled,
        setter: (bool value) async {
          if (debugProfileLayoutsEnabled != value) {
            debugProfileLayoutsEnabled = value;
          }
        },
      );
    }
  }

  late final PipelineManifold _manifold = _BindingPipelineManifold(this);

  /// The object that manages state about currently connected mice, for hover
  /// notification.
  MouseTracker get mouseTracker => _mouseTracker!;
  MouseTracker? _mouseTracker;

  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// This is typically the owner of the render tree bootstrapped by [runApp]
  /// and rooted in [renderView]. It maintains dirty state for layout,
  /// composite, paint, and accessibility semantics for that tree.
  ///
  /// However, by default, the [pipelineOwner] does not participate in frame
  /// production because it is not automatically attached to the
  /// [rootPipelineOwner] or any of its descendants. It is also not
  /// automatically associated with the [renderView]. This is left as a
  /// responsibility for a higher level abstraction. The [WidgetsBinding], for
  /// example, wires this up in [WidgetsBinding.wrapWithDefaultView], which is
  /// called indirectly from [runApp].
  ///
  /// Apps, that don't use the [WidgetsBinding] or don't call [runApp] (or
  /// [WidgetsBinding.wrapWithDefaultView]) must manually add this pipeline owner
  /// to the pipeline owner tree rooted at [rootPipelineOwner] and assign a
  /// [RenderView] to it if the they want to use this deprecated property.
  ///
  /// Instead of accessing this deprecated property, consider interacting with
  /// the root of the [PipelineOwner] tree (exposed in [rootPipelineOwner]) or
  /// instead of accessing the [SemanticsOwner] of any [PipelineOwner] consider
  /// interacting with the [SemanticsBinding] (exposed via
  /// [SemanticsBinding.instance]) directly.
  @Deprecated(
    'Interact with the pipelineOwner tree rooted at RendererBinding.rootPipelineOwner instead. '
    'Or instead of accessing the SemanticsOwner of any PipelineOwner interact with the SemanticsBinding directly. '
    'This feature was deprecated after v3.10.0-12.0.pre.',
  )
  late final PipelineOwner pipelineOwner = PipelineOwner(
    onSemanticsOwnerCreated: () {
      (pipelineOwner.rootNode as RenderView?)?.scheduleInitialSemantics();
    },
    onSemanticsUpdate: (ui.SemanticsUpdate update) {
      (pipelineOwner.rootNode as RenderView?)?.updateSemantics(update);
    },
    onSemanticsOwnerDisposed: () {
      (pipelineOwner.rootNode as RenderView?)?.clearSemantics();
    },
  );

  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// This is typically the root of the render tree bootstrapped by [runApp].
  ///
  /// However, by default this render view is not associated with any
  /// [PipelineOwner] and therefore isn't considered during frame production.
  /// It is also not registered with this binding via [addRenderView].
  /// Wiring this up is left as a responsibility for a higher level. The
  /// [WidgetsBinding], for example, sets this up in
  /// [WidgetsBinding.wrapWithDefaultView], which is called indirectly from
  /// [runApp].
  ///
  /// Apps that don't use the [WidgetsBinding] or don't call [runApp] (or
  /// [WidgetsBinding.wrapWithDefaultView]) must manually assign a
  /// [PipelineOwner] to this [RenderView], make sure the pipeline owner is part
  /// of the pipeline owner tree rooted at [rootPipelineOwner], and call
  /// [addRenderView] if they want to use this deprecated property.
  ///
  /// Instead of interacting with this deprecated property, consider using
  /// [renderViews] instead, which contains all [RenderView]s managed by the
  /// binding.
  @Deprecated(
    'Consider using RendererBinding.renderViews instead as the binding may manage multiple RenderViews. '
    'This feature was deprecated after v3.10.0-12.0.pre.',
  )
  // TODO(goderbauer): When this deprecated property is removed also delete the _ReusableRenderView class.
  late final RenderView renderView = _ReusableRenderView(view: platformDispatcher.implicitView!);

  /// Creates the [PipelineOwner] that serves as the root of the pipeline owner
  /// tree ([rootPipelineOwner]).
  ///
  /// {@template flutter.rendering.createRootPipelineOwner}
  /// By default, the root pipeline owner is not setup to manage a render tree
  /// and its [PipelineOwner.rootNode] must not be assigned. If necessary,
  /// [createRootPipelineOwner] may be overridden to create a root pipeline
  /// owner configured to manage its own render tree.
  ///
  /// In typical use, child pipeline owners are added to the root pipeline owner
  /// (via [PipelineOwner.adoptChild]). Those children typically do each manage
  /// their own [RenderView] and produce distinct render trees which render
  /// their content into the [FlutterView] associated with that [RenderView].
  /// {@endtemplate}
  PipelineOwner createRootPipelineOwner() {
    return _DefaultRootPipelineOwner();
  }

  /// The [PipelineOwner] that is the root of the PipelineOwner tree.
  ///
  /// {@macro flutter.rendering.createRootPipelineOwner}
  PipelineOwner get rootPipelineOwner => _rootPipelineOwner;
  late PipelineOwner _rootPipelineOwner;

  /// The [RenderView]s managed by this binding.
  ///
  /// A [RenderView] is added by [addRenderView] and removed by [removeRenderView].
  Iterable<RenderView> get renderViews => _viewIdToRenderView.values;
  final Map<Object, RenderView> _viewIdToRenderView = <Object, RenderView>{};

  /// Adds a [RenderView] to this binding.
  ///
  /// The binding will interact with the [RenderView] in the following ways:
  ///
  ///  * setting and updating [RenderView.configuration],
  ///  * calling [RenderView.compositeFrame] when it is time to produce a new
  ///    frame, and
  ///  * forwarding relevant pointer events to the [RenderView] for hit testing.
  ///
  /// To remove a [RenderView] from the binding, call [removeRenderView].
  void addRenderView(RenderView view) {
    final Object viewId = view.flutterView.viewId;
    assert(!_viewIdToRenderView.containsValue(view));
    assert(!_viewIdToRenderView.containsKey(viewId));
    _viewIdToRenderView[viewId] = view;
    view.configuration = createViewConfigurationFor(view);
  }

  /// Removes a [RenderView] previously added with [addRenderView] from the
  /// binding.
  void removeRenderView(RenderView view) {
    final Object viewId = view.flutterView.viewId;
    assert(_viewIdToRenderView[viewId] == view);
    _viewIdToRenderView.remove(viewId);
  }

  /// Returns a [ViewConfiguration] configured for the provided [RenderView]
  /// based on the current environment.
  ///
  /// This is called during [addRenderView] and also in response to changes to
  /// the system metrics to update all [renderViews] added to the binding.
  ///
  /// Bindings can override this method to change what size or device pixel
  /// ratio the [RenderView] will use. For example, the testing framework uses
  /// this to force the display into 800x600 when a test is run on the device
  /// using `flutter run`.
  @protected
  ViewConfiguration createViewConfigurationFor(RenderView renderView) {
    return ViewConfiguration.fromView(renderView.flutterView);
  }

  /// Create a [SceneBuilder].
  ///
  /// This hook enables test bindings to instrument the rendering layer.
  ///
  /// This is used by the [RenderView] to create the [SceneBuilder] that is
  /// passed to the [Layer] system to render the scene.
  ui.SceneBuilder createSceneBuilder() => ui.SceneBuilder();

  /// Create a [PictureRecorder].
  ///
  /// This hook enables test bindings to instrument the rendering layer.
  ///
  /// This is used by the [PaintingContext] to create the [PictureRecorder]s
  /// used when painting [RenderObject]s into [Picture]s passed to
  /// [PictureLayer]s.
  ui.PictureRecorder createPictureRecorder() => ui.PictureRecorder();

  /// Create a [Canvas] from a [PictureRecorder].
  ///
  /// This hook enables test bindings to instrument the rendering layer.
  ///
  /// This is used by the [PaintingContext] after creating a [PictureRecorder]
  /// using [createPictureRecorder].
  Canvas createCanvas(ui.PictureRecorder recorder) => Canvas(recorder);

  /// Called when the system metrics change.
  ///
  /// See [dart:ui.PlatformDispatcher.onMetricsChanged].
  @protected
  @visibleForTesting
  void handleMetricsChanged() {
    bool forceFrame = false;
    for (final RenderView view in renderViews) {
      forceFrame = forceFrame || view.child != null;
      view.configuration = createViewConfigurationFor(view);
    }
    if (forceFrame) {
      scheduleForcedFrame();
    }
  }

  /// Called when the platform text scale factor changes.
  ///
  /// See [dart:ui.PlatformDispatcher.onTextScaleFactorChanged].
  @protected
  void handleTextScaleFactorChanged() {}

  /// Called when the platform brightness changes.
  ///
  /// The current platform brightness can be queried from a Flutter binding or
  /// from a [MediaQuery] widget. The latter is preferred from widgets because
  /// it causes the widget to be automatically rebuilt when the brightness
  /// changes.
  ///
  /// {@tool snippet}
  /// Querying [MediaQuery.platformBrightnessOf] directly. Preferred.
  ///
  /// ```dart
  /// final Brightness brightness = MediaQuery.platformBrightnessOf(context);
  /// ```
  /// {@end-tool}
  ///
  /// {@tool snippet}
  /// Querying [PlatformDispatcher.platformBrightness].
  ///
  /// ```dart
  /// final Brightness brightness = WidgetsBinding.instance.platformDispatcher.platformBrightness;
  /// ```
  /// {@end-tool}
  ///
  /// See [dart:ui.PlatformDispatcher.onPlatformBrightnessChanged].
  @protected
  void handlePlatformBrightnessChanged() {}

  /// Creates a [MouseTracker] which manages state about currently connected
  /// mice, for hover notification.
  ///
  /// Used by testing framework to reinitialize the mouse tracker between tests.
  @visibleForTesting
  void initMouseTracker([MouseTracker? tracker]) {
    _mouseTracker?.dispose();
    _mouseTracker =
        tracker ??
        MouseTracker((Offset position, int viewId) {
          final HitTestResult result = HitTestResult();
          hitTestInView(result, position, viewId);
          return result;
        });
  }

  @override // from GestureBinding
  void dispatchEvent(PointerEvent event, HitTestResult? hitTestResult) {
    _mouseTracker!.updateWithEvent(
      event,
      // When the button is pressed, normal hit test uses a cached
      // result, but MouseTracker requires that the hit test is re-executed to
      // update the hovering events.
      event is PointerMoveEvent ? null : hitTestResult,
    );
    super.dispatchEvent(event, hitTestResult);
  }

  @override
  void performSemanticsAction(SemanticsActionEvent action) {
    // Due to the asynchronicity in some screen readers (they may not have
    // processed the latest semantics update yet) this code is more forgiving
    // and actions for views/nodes that no longer exist are gracefully ignored.
    _viewIdToRenderView[action.viewId]?.owner?.semanticsOwner?.performAction(
      action.nodeId,
      action.type,
      action.arguments,
    );
  }

  void _handleWebFirstFrame(Duration _) {
    assert(kIsWeb);
    const MethodChannel methodChannel = MethodChannel('flutter/service_worker');
    methodChannel.invokeMethod<void>('first-frame');
  }

  void _handlePersistentFrameCallback(Duration timeStamp) {
    drawFrame();
    _scheduleMouseTrackerUpdate();
  }

  bool _debugMouseTrackerUpdateScheduled = false;
  void _scheduleMouseTrackerUpdate() {
    assert(!_debugMouseTrackerUpdateScheduled);
    assert(() {
      _debugMouseTrackerUpdateScheduled = true;
      return true;
    }());
    SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
      assert(_debugMouseTrackerUpdateScheduled);
      assert(() {
        _debugMouseTrackerUpdateScheduled = false;
        return true;
      }());
      _mouseTracker!.updateAllDevices();
    }, debugLabel: 'RendererBinding.mouseTrackerUpdate');
  }

  int _firstFrameDeferredCount = 0;
  bool _firstFrameSent = false;

  /// Whether frames produced by [drawFrame] are sent to the engine.
  ///
  /// If false the framework will do all the work to produce a frame,
  /// but the frame is never sent to the engine to actually appear on screen.
  ///
  /// See also:
  ///
  ///  * [deferFirstFrame], which defers when the first frame is sent to the
  ///    engine.
  bool get sendFramesToEngine => _firstFrameSent || _firstFrameDeferredCount == 0;

  /// Tell the framework to not send the first frames to the engine until there
  /// is a corresponding call to [allowFirstFrame].
  ///
  /// Call this to perform asynchronous initialization work before the first
  /// frame is rendered (which takes down the splash screen). The framework
  /// will still do all the work to produce frames, but those frames are never
  /// sent to the engine and will not appear on screen.
  ///
  /// Calling this has no effect after the first frame has been sent to the
  /// engine.
  void deferFirstFrame() {
    assert(_firstFrameDeferredCount >= 0);
    _firstFrameDeferredCount += 1;
  }

  /// Called after [deferFirstFrame] to tell the framework that it is ok to
  /// send the first frame to the engine now.
  ///
  /// For best performance, this method should only be called while the
  /// [schedulerPhase] is [SchedulerPhase.idle].
  ///
  /// This method may only be called once for each corresponding call
  /// to [deferFirstFrame].
  void allowFirstFrame() {
    assert(_firstFrameDeferredCount > 0);
    _firstFrameDeferredCount -= 1;
    // Always schedule a warm up frame even if the deferral count is not down to
    // zero yet since the removal of a deferral may uncover new deferrals that
    // are lower in the widget tree.
    if (!_firstFrameSent) {
      scheduleWarmUpFrame();
    }
  }

  /// Call this to pretend that no frames have been sent to the engine yet.
  ///
  /// This is useful for tests that want to call [deferFirstFrame] and
  /// [allowFirstFrame] since those methods only have an effect if no frames
  /// have been sent to the engine yet.
  void resetFirstFrameSent() {
    _firstFrameSent = false;
  }

  /// Pump the rendering pipeline to generate a frame.
  ///
  /// This method is called by [handleDrawFrame], which itself is called
  /// automatically by the engine when it is time to lay out and paint a frame.
  ///
  /// Each frame consists of the following phases:
  ///
  /// 1. The animation phase: The [handleBeginFrame] method, which is registered
  /// with [PlatformDispatcher.onBeginFrame], invokes all the transient frame
  /// callbacks registered with [scheduleFrameCallback], in registration order.
  /// This includes all the [Ticker] instances that are driving
  /// [AnimationController] objects, which means all of the active [Animation]
  /// objects tick at this point.
  ///
  /// 2. Microtasks: After [handleBeginFrame] returns, any microtasks that got
  /// scheduled by transient frame callbacks get to run. This typically includes
  /// callbacks for futures from [Ticker]s and [AnimationController]s that
  /// completed this frame.
  ///
  /// After [handleBeginFrame], [handleDrawFrame], which is registered with
  /// [dart:ui.PlatformDispatcher.onDrawFrame], is called, which invokes all the
  /// persistent frame callbacks, of which the most notable is this method,
  /// [drawFrame], which proceeds as follows:
  ///
  /// 3. The layout phase: All the dirty [RenderObject]s in the system are laid
  /// out (see [RenderObject.performLayout]). See [RenderObject.markNeedsLayout]
  /// for further details on marking an object dirty for layout.
  ///
  /// 4. The compositing bits phase: The compositing bits on any dirty
  /// [RenderObject] objects are updated. See
  /// [RenderObject.markNeedsCompositingBitsUpdate].
  ///
  /// 5. The paint phase: All the dirty [RenderObject]s in the system are
  /// repainted (see [RenderObject.paint]). This generates the [Layer] tree. See
  /// [RenderObject.markNeedsPaint] for further details on marking an object
  /// dirty for paint.
  ///
  /// 6. The compositing phase: The layer tree is turned into a [Scene] and
  /// sent to the GPU.
  ///
  /// 7. The semantics phase: All the dirty [RenderObject]s in the system have
  /// their semantics updated. This generates the [SemanticsNode] tree. See
  /// [RenderObject.markNeedsSemanticsUpdate] for further details on marking an
  /// object dirty for semantics.
  ///
  /// For more details on steps 3-7, see [PipelineOwner].
  ///
  /// 8. The finalization phase: After [drawFrame] returns, [handleDrawFrame]
  /// then invokes post-frame callbacks (registered with [addPostFrameCallback]).
  ///
  /// Some bindings (for example, the [WidgetsBinding]) add extra steps to this
  /// list (for example, see [WidgetsBinding.drawFrame]).
  //
  // When editing the above, also update widgets/binding.dart's copy.
  @protected
  void drawFrame() {
    rootPipelineOwner.flushLayout();
    rootPipelineOwner.flushCompositingBits();
    rootPipelineOwner.flushPaint();
    if (sendFramesToEngine) {
      for (final RenderView renderView in renderViews) {
        renderView.compositeFrame(); // this sends the bits to the GPU
      }
      rootPipelineOwner.flushSemantics(); // this sends the semantics to the OS.
      _firstFrameSent = true;
    }
  }

  @override
  Future<void> performReassemble() async {
    await super.performReassemble();
    if (!kReleaseMode) {
      FlutterTimeline.startSync('Preparing Hot Reload (layout)');
    }
    try {
      for (final RenderView renderView in renderViews) {
        renderView.reassemble();
      }
    } finally {
      if (!kReleaseMode) {
        FlutterTimeline.finishSync();
      }
    }
    scheduleWarmUpFrame();
    await endOfFrame;
  }

  @override
  void hitTestInView(HitTestResult result, Offset position, int viewId) {
    _viewIdToRenderView[viewId]?.hitTest(result, position: position);
    super.hitTestInView(result, position, viewId);
  }

  Future<void> _forceRepaint() {
    late RenderObjectVisitor visitor;
    visitor = (RenderObject child) {
      child.markNeedsPaint();
      child.visitChildren(visitor);
    };
    for (final RenderView renderView in renderViews) {
      renderView.visitChildren(visitor);
    }
    return endOfFrame;
  }
}

String _debugCollectRenderTrees() {
  if (RendererBinding.instance.renderViews.isEmpty) {
    return 'No render tree root was added to the binding.';
  }
  return <String>[
    for (final RenderView renderView in RendererBinding.instance.renderViews)
      renderView.toStringDeep(),
  ].join('\n\n');
}

/// Prints a textual representation of the render trees.
///
/// {@template flutter.rendering.debugDumpRenderTree}
/// It prints the trees associated with every [RenderView] in
/// [RendererBinding.renderViews], separated by two blank lines.
/// {@endtemplate}
void debugDumpRenderTree() {
  debugPrint(_debugCollectRenderTrees());
}

String _debugCollectLayerTrees() {
  if (RendererBinding.instance.renderViews.isEmpty) {
    return 'No render tree root was added to the binding.';
  }
  return <String>[
    for (final RenderView renderView in RendererBinding.instance.renderViews)
      renderView.debugLayer?.toStringDeep() ?? 'Layer tree unavailable for $renderView.',
  ].join('\n\n');
}

/// Prints a textual representation of the layer trees.
///
/// {@macro flutter.rendering.debugDumpRenderTree}
void debugDumpLayerTree() {
  debugPrint(_debugCollectLayerTrees());
}

String _debugCollectSemanticsTrees(DebugSemanticsDumpOrder childOrder) {
  if (RendererBinding.instance.renderViews.isEmpty) {
    return 'No render tree root was added to the binding.';
  }
  const String explanation =
      'For performance reasons, the framework only generates semantics when asked to do so by the platform.\n'
      'Usually, platforms only ask for semantics when assistive technologies (like screen readers) are running.\n'
      'To generate semantics, try turning on an assistive technology (like VoiceOver or TalkBack) on your device.';
  final List<String> trees = <String>[];
  bool printedExplanation = false;
  for (final RenderView renderView in RendererBinding.instance.renderViews) {
    final String? tree = renderView.debugSemantics?.toStringDeep(childOrder: childOrder);
    if (tree != null) {
      trees.add(tree);
    } else {
      String message = 'Semantics not generated for $renderView.';
      if (!printedExplanation) {
        printedExplanation = true;
        message = '$message\n$explanation';
      }
      trees.add(message);
    }
  }
  return trees.join('\n\n');
}

/// Prints a textual representation of the semantics trees.
///
/// {@macro flutter.rendering.debugDumpRenderTree}
///
/// Semantics trees are only constructed when semantics are enabled (see
/// [SemanticsBinding.semanticsEnabled]). If a semantics tree is not available,
/// a notice about the missing semantics tree is printed instead.
///
/// The order in which the children of a [SemanticsNode] will be printed is
/// controlled by the [childOrder] parameter.
void debugDumpSemanticsTree([
  DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.traversalOrder,
]) {
  debugPrint(_debugCollectSemanticsTrees(childOrder));
}

/// Prints a textual representation of the [PipelineOwner] tree rooted at
/// [RendererBinding.rootPipelineOwner].
void debugDumpPipelineOwnerTree() {
  debugPrint(RendererBinding.instance.rootPipelineOwner.toStringDeep());
}

/// A concrete binding for applications that use the Rendering framework
/// directly. This is the glue that binds the framework to the Flutter engine.
///
/// When using the rendering framework directly, this binding, or one that
/// implements the same interfaces, must be used. The following
/// mixins are used to implement this binding:
///
/// * [GestureBinding], which implements the basics of hit testing.
/// * [SchedulerBinding], which introduces the concepts of frames.
/// * [ServicesBinding], which provides access to the plugin subsystem.
/// * [SemanticsBinding], which supports accessibility.
/// * [PaintingBinding], which enables decoding images.
/// * [RendererBinding], which handles the render tree.
///
/// You would only use this binding if you are writing to the
/// rendering layer directly. If you are writing to a higher-level
/// library, such as the Flutter Widgets library, then you would use
/// that layer's binding (see [WidgetsFlutterBinding]).
///
/// The [RenderingFlutterBinding] can manage multiple render trees. Each render
/// tree is rooted in a [RenderView] that must be added to the binding via
/// [addRenderView] to be consider during frame production, hit testing, etc.
/// Furthermore, the render tree must be managed by a [PipelineOwner] that is
/// part of the pipeline owner tree rooted at [rootPipelineOwner].
///
/// Adding [PipelineOwner]s and [RenderView]s to this binding in the way
/// described above is left as a responsibility for a higher level abstraction.
/// The binding does not own any [RenderView]s directly.
class RenderingFlutterBinding extends BindingBase
    with
        GestureBinding,
        SchedulerBinding,
        ServicesBinding,
        SemanticsBinding,
        PaintingBinding,
        RendererBinding {
  /// Returns an instance of the binding that implements
  /// [RendererBinding]. If no binding has yet been initialized, the
  /// [RenderingFlutterBinding] class is used to create and initialize
  /// one.
  ///
  /// You need to call this method before using the rendering framework
  /// if you are using it directly. If you are using the widgets framework,
  /// see [WidgetsFlutterBinding.ensureInitialized].
  static RendererBinding ensureInitialized() {
    if (RendererBinding._instance == null) {
      RenderingFlutterBinding();
    }
    return RendererBinding.instance;
  }
}

/// A [PipelineManifold] implementation that is backed by the [RendererBinding].
class _BindingPipelineManifold extends ChangeNotifier implements PipelineManifold {
  _BindingPipelineManifold(this._binding) {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
    _binding.addSemanticsEnabledListener(notifyListeners);
  }

  final RendererBinding _binding;

  @override
  void requestVisualUpdate() {
    _binding.ensureVisualUpdate();
  }

  @override
  bool get semanticsEnabled => _binding.semanticsEnabled;

  @override
  void dispose() {
    _binding.removeSemanticsEnabledListener(notifyListeners);
    super.dispose();
  }
}

// A [PipelineOwner] that cannot have a root node.
class _DefaultRootPipelineOwner extends PipelineOwner {
  _DefaultRootPipelineOwner() : super(onSemanticsUpdate: _onSemanticsUpdate);

  @override
  set rootNode(RenderObject? _) {
    assert(() {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Cannot set a rootNode on the default root pipeline owner.'),
        ErrorDescription(
          'By default, the RendererBinding.rootPipelineOwner is not configured '
          'to manage a root node because this pipeline owner does not define a '
          'proper onSemanticsUpdate callback to handle semantics for that node.',
        ),
        ErrorHint(
          'Typically, the root pipeline owner does not manage a root node. '
          'Instead, properly configured child pipeline owners (which do manage '
          'root nodes) are added to it. Alternatively, if you do want to set a '
          'root node for the root pipeline owner, override '
          'RendererBinding.createRootPipelineOwner to create a '
          'pipeline owner that is configured to properly handle semantics for '
          'the provided root node.',
        ),
      ]);
    }());
  }

  static void _onSemanticsUpdate(ui.SemanticsUpdate _) {
    // Neve called because we don't have a root node.
    assert(false);
  }
}

// Prior to multi view support, the [RendererBinding] would own a long-lived
// [RenderView], that was never disposed (see [RendererBinding.renderView]).
// With multi view support, the [RendererBinding] no longer owns a [RenderView]
// and instead higher level abstractions (like the [View] widget) can add/remove
// multiple [RenderView]s to the binding as needed. When the [View] widget is no
// longer needed, it expects to dispose its [RenderView].
//
// This special version of a [RenderView] now exists as a bridge between those
// worlds to continue supporting the [RendererBinding.renderView] property
// through its deprecation period. Per the property's contract, it is supposed
// to be long-lived, but it is also managed by a [View] widget (introduced by
// [WidgetsBinding.wrapWithDefaultView]), that expects to dispose its render
// object at the end of the widget's life time. This special version now
// implements logic to reset the [RenderView] when it is "disposed" so it can be
// reused by another [View] widget.
//
// Once the deprecated [RendererBinding.renderView] property is removed, this
// class is no longer necessary.
class _ReusableRenderView extends RenderView {
  _ReusableRenderView({required super.view});

  bool _initialFramePrepared = false;

  @override
  void prepareInitialFrame() {
    if (_initialFramePrepared) {
      return;
    }
    super.prepareInitialFrame();
    _initialFramePrepared = true;
  }

  @override
  void scheduleInitialSemantics() {
    clearSemantics();
    super.scheduleInitialSemantics();
  }

  @override
  // ignore: must_call_super
  void dispose() {
    child = null;
  }
}
