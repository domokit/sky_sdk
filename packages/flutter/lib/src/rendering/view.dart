// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/semantics.dart';
/// @docImport 'package:flutter/widgets.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:io' show Platform;
import 'dart:ui' as ui show FlutterView, Scene, SceneBuilder, SemanticsUpdate;

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'binding.dart';
import 'box.dart';
import 'debug.dart';
import 'layer.dart';
import 'object.dart';

/// The layout constraints for the root render object.
@immutable
class ViewConfiguration {
  /// Creates a view configuration.
  ///
  /// By default, the view has [logicalConstraints] and [physicalConstraints]
  /// with all dimensions set to zero (i.e. the view is forced to [Size.zero])
  /// and a [devicePixelRatio] of 1.0.
  ///
  /// [ViewConfiguration.fromView] is a more convenient way for deriving a
  /// [ViewConfiguration] from a given [ui.FlutterView].
  const ViewConfiguration({
    this.physicalConstraints = const BoxConstraints(maxWidth: 0, maxHeight: 0),
    this.logicalConstraints = const BoxConstraints(maxWidth: 0, maxHeight: 0),
    this.devicePixelRatio = 1.0,
  });

  /// Creates a view configuration for the provided [ui.FlutterView].
  factory ViewConfiguration.fromView(ui.FlutterView view) {
    final BoxConstraints physicalConstraints = BoxConstraints.fromViewConstraints(
      view.physicalConstraints,
    );
    final double devicePixelRatio = view.devicePixelRatio;
    return ViewConfiguration(
      physicalConstraints: physicalConstraints,
      logicalConstraints: physicalConstraints / devicePixelRatio,
      devicePixelRatio: devicePixelRatio,
    );
  }

  /// The constraints of the output surface in logical pixel.
  ///
  /// The constraints are passed to the child of the root render object.
  final BoxConstraints logicalConstraints;

  /// The constraints of the output surface in physical pixel.
  ///
  /// These constraints are enforced in [toPhysicalSize] when translating
  /// the logical size of the root render object back to physical pixels for
  /// the [ui.FlutterView.render] method.
  final BoxConstraints physicalConstraints;

  /// The pixel density of the output surface.
  final double devicePixelRatio;

  /// Creates a transformation matrix that applies the [devicePixelRatio].
  ///
  /// The matrix translates points from the local coordinate system of the
  /// app (in logical pixels) to the global coordinate system of the
  /// [ui.FlutterView] (in physical pixels).
  Matrix4 toMatrix() {
    return Matrix4.diagonal3Values(devicePixelRatio, devicePixelRatio, 1.0);
  }

  /// Returns whether [toMatrix] would return a different value for this
  /// configuration than it would for the given `oldConfiguration`.
  bool shouldUpdateMatrix(ViewConfiguration oldConfiguration) {
    if (oldConfiguration.runtimeType != runtimeType) {
      // New configuration could have different logic, so we don't know
      // whether it will need a new transform. Return a conservative result.
      return true;
    }
    // For this class, the only input to toMatrix is the device pixel ratio,
    // so we return true if they differ and false otherwise.
    return oldConfiguration.devicePixelRatio != devicePixelRatio;
  }

  /// Transforms the provided [Size] in logical pixels to physical pixels.
  ///
  /// The [ui.FlutterView.render] method accepts only sizes in physical pixels, but
  /// the framework operates in logical pixels. This method is used to transform
  /// the logical size calculated for a [RenderView] back to a physical size
  /// suitable to be passed to [ui.FlutterView.render].
  ///
  /// By default, this method just multiplies the provided [Size] with the
  /// [devicePixelRatio] and constraints the results to the
  /// [physicalConstraints].
  Size toPhysicalSize(Size logicalSize) {
    return physicalConstraints.constrain(logicalSize * devicePixelRatio);
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ViewConfiguration &&
        other.logicalConstraints == logicalConstraints &&
        other.physicalConstraints == physicalConstraints &&
        other.devicePixelRatio == devicePixelRatio;
  }

  @override
  int get hashCode => Object.hash(logicalConstraints, physicalConstraints, devicePixelRatio);

  @override
  String toString() => '$logicalConstraints at ${debugFormatDouble(devicePixelRatio)}x';
}

/// The root of the render tree.
///
/// The view represents the total output surface of the render tree and handles
/// bootstrapping the rendering pipeline. The view has a unique child
/// [RenderBox], which is required to fill the entire output surface.
///
/// This object must be bootstrapped in a specific order:
///
///  1. First, set the [configuration] (either in the constructor or after
///     construction).
///  2. Second, [attach] the object to a [PipelineOwner].
///  3. Third, use [prepareInitialFrame] to bootstrap the layout and paint logic.
///
/// After the bootstrapping is complete, the [compositeFrame] method may be used
/// to obtain the rendered output.
class RenderView extends RenderObject with RenderObjectWithChildMixin<RenderBox> {
  /// Creates the root of the render tree.
  ///
  /// Typically created by the binding (e.g., [RendererBinding]).
  ///
  /// Providing a [configuration] is optional, but a configuration must be set
  /// before calling [prepareInitialFrame]. This decouples creating the
  /// [RenderView] object from configuring it. Typically, the object is created
  /// by the [View] widget and configured by the [RendererBinding] when the
  /// [RenderView] is registered with it by the [View] widget.
  RenderView({RenderBox? child, ViewConfiguration? configuration, required ui.FlutterView view})
    : _view = view {
    if (configuration != null) {
      this.configuration = configuration;
    }
    this.child = child;
  }

  /// The current layout size of the view.
  Size get size => _size;
  Size _size = Size.zero;

  /// The constraints used for the root layout.
  ///
  /// Typically, this configuration is set by the [RendererBinding], when the
  /// [RenderView] is registered with it. It will also update the configuration
  /// if necessary. Therefore, if used in conjunction with the [RendererBinding]
  /// this property must not be set manually as the [RendererBinding] will just
  /// override it.
  ///
  /// For tests that want to change the size of the view, set
  /// [TestFlutterView.physicalSize] on the appropriate [TestFlutterView]
  /// (typically [WidgetTester.view]) instead of setting a configuration
  /// directly on the [RenderView].
  ///
  /// A [configuration] must be set (either directly or by passing one to the
  /// constructor) before calling [prepareInitialFrame].
  ViewConfiguration get configuration => _configuration!;
  ViewConfiguration? _configuration;
  set configuration(ViewConfiguration value) {
    if (_configuration == value) {
      return;
    }
    final ViewConfiguration? oldConfiguration = _configuration;
    _configuration = value;
    if (_rootTransform == null) {
      // [prepareInitialFrame] has not been called yet, nothing more to do for now.
      return;
    }
    if (oldConfiguration == null || configuration.shouldUpdateMatrix(oldConfiguration)) {
      replaceRootLayer(_updateMatricesAndCreateNewRootLayer());
    }
    assert(_rootTransform != null);
    markNeedsLayout();
  }

  /// Whether a [configuration] has been set.
  ///
  /// This must be true before calling [prepareInitialFrame].
  bool get hasConfiguration => _configuration != null;

  @override
  BoxConstraints get constraints {
    if (!hasConfiguration) {
      throw StateError(
        'Constraints are not available because RenderView has not been given a configuration yet.',
      );
    }
    return configuration.logicalConstraints;
  }

  /// The [ui.FlutterView] into which this [RenderView] will render.
  ui.FlutterView get flutterView => _view;
  final ui.FlutterView _view;

  /// Whether Flutter should automatically compute the desired system UI.
  ///
  /// When this setting is enabled, Flutter will hit-test the layer tree at the
  /// top and bottom of the screen on each frame looking for an
  /// [AnnotatedRegionLayer] with an instance of a [SystemUiOverlayStyle]. The
  /// hit-test result from the top of the screen provides the status bar settings
  /// and the hit-test result from the bottom of the screen provides the system
  /// nav bar settings.
  ///
  /// If there is no [AnnotatedRegionLayer] on the bottom, the hit-test result
  /// from the top provides the system nav bar settings. If there is no
  /// [AnnotatedRegionLayer] on the top, the hit-test result from the bottom
  /// provides the system status bar settings.
  ///
  /// Setting this to false does not cause previous automatic adjustments to be
  /// reset, nor does setting it to true cause the app to update immediately.
  ///
  /// If you want to imperatively set the system ui style instead, it is
  /// recommended that [automaticSystemUiAdjustment] is set to false.
  ///
  /// See also:
  ///
  ///  * [AnnotatedRegion], for placing [SystemUiOverlayStyle] in the layer tree.
  ///  * [SystemChrome.setSystemUIOverlayStyle], for imperatively setting the system ui style.
  bool automaticSystemUiAdjustment = true;

  /// Bootstrap the rendering pipeline by preparing the first frame.
  ///
  /// This should only be called once. It is typically called immediately after
  /// setting the [configuration] the first time (whether by passing one to the
  /// constructor, or setting it directly). The [configuration] must have been
  /// set before calling this method, and the [RenderView] must have been
  /// attached to a [PipelineOwner] using [attach].
  ///
  /// This does not actually schedule the first frame. Call
  /// [PipelineOwner.requestVisualUpdate] on the [owner] to do that.
  ///
  /// This should be called before using any methods that rely on the [layer]
  /// being initialized, such as [compositeFrame].
  ///
  /// This method calls [scheduleInitialLayout] and [scheduleInitialPaint].
  void prepareInitialFrame() {
    assert(
      owner != null,
      'attach the RenderView to a PipelineOwner before calling prepareInitialFrame',
    );
    assert(
      _rootTransform == null,
      'prepareInitialFrame must only be called once',
    ); // set by _updateMatricesAndCreateNewRootLayer
    assert(hasConfiguration, 'set a configuration before calling prepareInitialFrame');
    scheduleInitialLayout();
    scheduleInitialPaint(_updateMatricesAndCreateNewRootLayer());
    assert(_rootTransform != null);
  }

  Matrix4? _rootTransform;

  TransformLayer _updateMatricesAndCreateNewRootLayer() {
    assert(hasConfiguration);
    _rootTransform = configuration.toMatrix();
    final TransformLayer rootLayer = TransformLayer(transform: _rootTransform);
    rootLayer.attach(this);
    assert(_rootTransform != null);
    return rootLayer;
  }

  // We never call layout() on this class, so this should never get
  // checked. (This class is laid out using scheduleInitialLayout().)
  @override
  void debugAssertDoesMeetConstraints() {
    assert(false);
  }

  @override
  void performResize() {
    assert(false);
  }

  @override
  void performLayout() {
    assert(_rootTransform != null);
    final bool sizedByChild = !constraints.isTight;
    child?.layout(constraints, parentUsesSize: sizedByChild);
    _size = sizedByChild && child != null ? child!.size : constraints.smallest;
    assert(size.isFinite);
    assert(constraints.isSatisfiedBy(size));
  }

  /// Determines the set of render objects located at the given position.
  ///
  /// Returns true if the given point is contained in this render object or one
  /// of its descendants. Adds any render objects that contain the point to the
  /// given hit test result.
  ///
  /// The [position] argument is in the coordinate system of the render view,
  /// which is to say, in logical pixels. This is not necessarily the same
  /// coordinate system as that expected by the root [Layer], which will
  /// normally be in physical (device) pixels.
  bool hitTest(HitTestResult result, {required Offset position}) {
    child?.hitTest(BoxHitTestResult.wrap(result), position: position);
    result.add(HitTestEntry(this));
    return true;
  }

  @override
  bool get isRepaintBoundary => true;

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      context.paintChild(child!, offset);
    }
    assert(() {
      final List<DebugPaintCallback> localCallbacks = _debugPaintCallbacks.toList();
      for (final DebugPaintCallback paintCallback in localCallbacks) {
        if (_debugPaintCallbacks.contains(paintCallback)) {
          paintCallback(context, offset, this);
        }
      }
      return true;
    }());
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    assert(_rootTransform != null);
    transform.multiply(_rootTransform!);
    super.applyPaintTransform(child, transform);
  }

  /// Uploads the composited layer tree to the engine.
  ///
  /// Actually causes the output of the rendering pipeline to appear on screen.
  ///
  /// Before calling this method, the [owner] must be set by calling [attach],
  /// the [configuration] must be set to a non-null value, and the
  /// [prepareInitialFrame] method must have been called.
  void compositeFrame() {
    if (!kReleaseMode) {
      FlutterTimeline.startSync('COMPOSITING');
    }
    try {
      assert(hasConfiguration, 'set the RenderView configuration before calling compositeFrame');
      assert(_rootTransform != null, 'call prepareInitialFrame before calling compositeFrame');
      assert(layer != null, 'call prepareInitialFrame before calling compositeFrame');
      final ui.SceneBuilder builder = RendererBinding.instance.createSceneBuilder();
      final ui.Scene scene = layer!.buildScene(builder);
      if (automaticSystemUiAdjustment) {
        _updateSystemChrome();
      }
      assert(configuration.logicalConstraints.isSatisfiedBy(size));
      _view.render(scene, size: configuration.toPhysicalSize(size));
      scene.dispose();
      assert(() {
        if (debugRepaintRainbowEnabled || debugRepaintTextRainbowEnabled) {
          debugCurrentRepaintColor = debugCurrentRepaintColor.withHue(
            (debugCurrentRepaintColor.hue + 2.0) % 360.0,
          );
        }
        return true;
      }());
    } finally {
      if (!kReleaseMode) {
        FlutterTimeline.finishSync();
      }
    }
  }

  /// Sends the provided [ui.SemanticsUpdate] to the [ui.FlutterView] associated with
  /// this [RenderView].
  ///
  /// A [ui.SemanticsUpdate] is produced by a [SemanticsOwner] during the
  /// [EnginePhase.flushSemantics] phase.
  void updateSemantics(ui.SemanticsUpdate update) {
    _view.updateSemantics(update);
  }

  void _updateSystemChrome() {
    // Take overlay style from the place where a system status bar and system
    // navigation bar are placed to update system style overlay.
    // The center of the system navigation bar and the center of the status bar
    // are used to get SystemUiOverlayStyle's to update system overlay appearance.
    //
    //         Horizontal center of the screen
    //                 V
    //    ++++++++++++++++++++++++++
    //    |                        |
    //    |    System status bar   |  <- Vertical center of the status bar
    //    |                        |
    //    ++++++++++++++++++++++++++
    //    |                        |
    //    |        Content         |
    //    ~                        ~
    //    |                        |
    //    ++++++++++++++++++++++++++
    //    |                        |
    //    |  System navigation bar | <- Vertical center of the navigation bar
    //    |                        |
    //    ++++++++++++++++++++++++++ <- bounds.bottom
    final Rect bounds = paintBounds;
    // Center of the status bar
    final Offset top = Offset(
      // Horizontal center of the screen
      bounds.center.dx,
      // The vertical center of the system status bar. The system status bar
      // height is kept as top window padding.
      _view.padding.top / 2.0,
    );
    // Center of the navigation bar
    final Offset bottom = Offset(
      // Horizontal center of the screen
      bounds.center.dx,
      // Vertical center of the system navigation bar. The system navigation bar
      // height is kept as bottom window padding. The "1" needs to be subtracted
      // from the bottom because available pixels are in (0..bottom) range.
      // I.e. for a device with 1920 height, bound.bottom is 1920, but the most
      // bottom drawn pixel is at 1919 position.
      bounds.bottom - 1.0 - _view.padding.bottom / 2.0,
    );
    final SystemUiOverlayStyle? upperOverlayStyle = layer!.find<SystemUiOverlayStyle>(top);
    // Only android has a customizable system navigation bar.
    SystemUiOverlayStyle? lowerOverlayStyle;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        lowerOverlayStyle = layer!.find<SystemUiOverlayStyle>(bottom);
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        break;
    }
    // If there are no overlay style in the UI don't bother updating.
    if (upperOverlayStyle == null && lowerOverlayStyle == null) {
      return;
    }

    // If both are not null, the upper provides the status bar properties and the lower provides
    // the system navigation bar properties. This is done for advanced use cases where a widget
    // on the top (for instance an app bar) will create an annotated region to set the status bar
    // style and another widget on the bottom will create an annotated region to set the system
    // navigation bar style.
    if (upperOverlayStyle != null && lowerOverlayStyle != null) {
      final SystemUiOverlayStyle overlayStyle = SystemUiOverlayStyle(
        statusBarBrightness: upperOverlayStyle.statusBarBrightness,
        statusBarIconBrightness: upperOverlayStyle.statusBarIconBrightness,
        statusBarColor: upperOverlayStyle.statusBarColor,
        systemStatusBarContrastEnforced: upperOverlayStyle.systemStatusBarContrastEnforced,
        systemNavigationBarColor: lowerOverlayStyle.systemNavigationBarColor,
        systemNavigationBarDividerColor: lowerOverlayStyle.systemNavigationBarDividerColor,
        systemNavigationBarIconBrightness: lowerOverlayStyle.systemNavigationBarIconBrightness,
        systemNavigationBarContrastEnforced: lowerOverlayStyle.systemNavigationBarContrastEnforced,
      );
      SystemChrome.setSystemUIOverlayStyle(overlayStyle);
      return;
    }
    // If only one of the upper or the lower overlay style is not null, it provides all properties.
    // This is done for developer convenience as it allows setting both status bar style and
    // navigation bar style using only one annotated region layer (for instance the one
    // automatically created by an [AppBar]).
    final bool isAndroid = defaultTargetPlatform == TargetPlatform.android;
    final SystemUiOverlayStyle definedOverlayStyle = (upperOverlayStyle ?? lowerOverlayStyle)!;
    final SystemUiOverlayStyle overlayStyle = SystemUiOverlayStyle(
      statusBarBrightness: definedOverlayStyle.statusBarBrightness,
      statusBarIconBrightness: definedOverlayStyle.statusBarIconBrightness,
      statusBarColor: definedOverlayStyle.statusBarColor,
      systemStatusBarContrastEnforced: definedOverlayStyle.systemStatusBarContrastEnforced,
      systemNavigationBarColor: isAndroid ? definedOverlayStyle.systemNavigationBarColor : null,
      systemNavigationBarDividerColor:
          isAndroid ? definedOverlayStyle.systemNavigationBarDividerColor : null,
      systemNavigationBarIconBrightness:
          isAndroid ? definedOverlayStyle.systemNavigationBarIconBrightness : null,
      systemNavigationBarContrastEnforced:
          isAndroid ? definedOverlayStyle.systemNavigationBarContrastEnforced : null,
    );
    SystemChrome.setSystemUIOverlayStyle(overlayStyle);
  }

  @override
  Rect get paintBounds => Offset.zero & (size * configuration.devicePixelRatio);

  @override
  Rect get semanticBounds {
    assert(_rootTransform != null);
    return MatrixUtils.transformRect(_rootTransform!, Offset.zero & size);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    // call to ${super.debugFillProperties(description)} is omitted because the
    // root superclasses don't include any interesting information for this
    // class
    assert(() {
      properties.add(
        DiagnosticsNode.message(
          'debug mode enabled - ${kIsWeb ? 'Web' : Platform.operatingSystem}',
        ),
      );
      return true;
    }());
    properties.add(
      DiagnosticsProperty<Size>('view size', _view.physicalSize, tooltip: 'in physical pixels'),
    );
    properties.add(
      DoubleProperty(
        'device pixel ratio',
        _view.devicePixelRatio,
        tooltip: 'physical pixels per logical pixel',
      ),
    );
    properties.add(
      DiagnosticsProperty<ViewConfiguration>(
        'configuration',
        configuration,
        tooltip: 'in logical pixels',
      ),
    );
    if (_view.platformDispatcher.semanticsEnabled) {
      properties.add(DiagnosticsNode.message('semantics enabled'));
    }
  }

  static final List<DebugPaintCallback> _debugPaintCallbacks = <DebugPaintCallback>[];

  /// Registers a [DebugPaintCallback] that is called every time a [RenderView]
  /// repaints in debug mode.
  ///
  /// The callback may paint a debug overlay on top of the content of the
  /// [RenderView] provided to the callback. Callbacks are invoked in the
  /// order they were registered in.
  ///
  /// Neither registering a callback nor the continued presence of a callback
  /// changes how often [RenderView]s are repainted. It is up to the owner of
  /// the callback to call [markNeedsPaint] on any [RenderView] for which it
  /// wants to update the painted overlay.
  ///
  /// Does nothing in release mode.
  static void debugAddPaintCallback(DebugPaintCallback callback) {
    assert(() {
      _debugPaintCallbacks.add(callback);
      return true;
    }());
  }

  /// Removes a callback registered with [debugAddPaintCallback].
  ///
  /// It does not schedule a frame to repaint the [RenderView]s without the
  /// overlay painted by the removed callback. It is up to the owner of the
  /// callback to call [markNeedsPaint] on the relevant [RenderView]s to
  /// repaint them without the overlay.
  ///
  /// Does nothing in release mode.
  static void debugRemovePaintCallback(DebugPaintCallback callback) {
    assert(() {
      _debugPaintCallbacks.remove(callback);
      return true;
    }());
  }
}

/// A callback for painting a debug overlay on top of the provided [RenderView].
///
/// Used by [RenderView.debugAddPaintCallback] and
/// [RenderView.debugRemovePaintCallback].
typedef DebugPaintCallback =
    void Function(PaintingContext context, Offset offset, RenderView renderView);
