// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
library;

import 'dart:async';
import 'dart:collection' show HashMap;
import 'dart:convert';
import 'dart:developer' as developer;
import 'dart:math' as math;
import 'dart:ui'
    as ui
    show
        ClipOp,
        FlutterView,
        Image,
        ImageByteFormat,
        Paragraph,
        Picture,
        PictureRecorder,
        PointMode,
        SceneBuilder,
        Vertices;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:meta/meta_meta.dart';

import 'basic.dart';
import 'binding.dart';
import 'debug.dart';
import 'framework.dart';
import 'gesture_detector.dart';
import 'service_extensions.dart';
import 'view.dart';

/// Signature for the builder callback used by
/// [WidgetInspector.exitWidgetSelectionButtonBuilder].
typedef ExitWidgetSelectionButtonBuilder =
    Widget Function(
      BuildContext context, {
      required VoidCallback onPressed,
      required GlobalKey key,
    });

/// Signature for the builder callback used by
/// [WidgetInspector.moveExitWidgetSelectionButtonBuilder].
typedef MoveExitWidgetSelectionButtonBuilder =
    Widget Function(BuildContext context, {required VoidCallback onPressed, bool isLeftAligned});

/// Signature for a method that registers the service extension `callback` with
/// the given `name`.
///
/// Used as argument to [WidgetInspectorService.initServiceExtensions]. The
/// [BindingBase.registerServiceExtension] implements this signature.
typedef RegisterServiceExtensionCallback =
    void Function({required String name, required ServiceExtensionCallback callback});

/// A layer that mimics the behavior of another layer.
///
/// A proxy layer is used for cases where a layer needs to be placed into
/// multiple trees of layers.
class _ProxyLayer extends Layer {
  _ProxyLayer(this._layer);

  final Layer _layer;

  @override
  void addToScene(ui.SceneBuilder builder) {
    _layer.addToScene(builder);
  }

  @override
  @protected
  bool findAnnotations<S extends Object>(
    AnnotationResult<S> result,
    Offset localPosition, {
    required bool onlyFirst,
  }) {
    return _layer.findAnnotations(result, localPosition, onlyFirst: onlyFirst);
  }
}

/// A [Canvas] that multicasts all method calls to a main canvas and a
/// secondary screenshot canvas so that a screenshot can be recorded at the same
/// time as performing a normal paint.
class _MulticastCanvas implements Canvas {
  _MulticastCanvas({required Canvas main, required Canvas screenshot})
    : _main = main,
      _screenshot = screenshot;

  final Canvas _main;
  final Canvas _screenshot;

  @override
  void clipPath(Path path, {bool doAntiAlias = true}) {
    _main.clipPath(path, doAntiAlias: doAntiAlias);
    _screenshot.clipPath(path, doAntiAlias: doAntiAlias);
  }

  @override
  void clipRRect(RRect rrect, {bool doAntiAlias = true}) {
    _main.clipRRect(rrect, doAntiAlias: doAntiAlias);
    _screenshot.clipRRect(rrect, doAntiAlias: doAntiAlias);
  }

  @override
  void clipRect(Rect rect, {ui.ClipOp clipOp = ui.ClipOp.intersect, bool doAntiAlias = true}) {
    _main.clipRect(rect, clipOp: clipOp, doAntiAlias: doAntiAlias);
    _screenshot.clipRect(rect, clipOp: clipOp, doAntiAlias: doAntiAlias);
  }

  @override
  void drawArc(Rect rect, double startAngle, double sweepAngle, bool useCenter, Paint paint) {
    _main.drawArc(rect, startAngle, sweepAngle, useCenter, paint);
    _screenshot.drawArc(rect, startAngle, sweepAngle, useCenter, paint);
  }

  @override
  void drawAtlas(
    ui.Image atlas,
    List<RSTransform> transforms,
    List<Rect> rects,
    List<Color>? colors,
    BlendMode? blendMode,
    Rect? cullRect,
    Paint paint,
  ) {
    _main.drawAtlas(atlas, transforms, rects, colors, blendMode, cullRect, paint);
    _screenshot.drawAtlas(atlas, transforms, rects, colors, blendMode, cullRect, paint);
  }

  @override
  void drawCircle(Offset c, double radius, Paint paint) {
    _main.drawCircle(c, radius, paint);
    _screenshot.drawCircle(c, radius, paint);
  }

  @override
  void drawColor(Color color, BlendMode blendMode) {
    _main.drawColor(color, blendMode);
    _screenshot.drawColor(color, blendMode);
  }

  @override
  void drawDRRect(RRect outer, RRect inner, Paint paint) {
    _main.drawDRRect(outer, inner, paint);
    _screenshot.drawDRRect(outer, inner, paint);
  }

  @override
  void drawImage(ui.Image image, Offset p, Paint paint) {
    _main.drawImage(image, p, paint);
    _screenshot.drawImage(image, p, paint);
  }

  @override
  void drawImageNine(ui.Image image, Rect center, Rect dst, Paint paint) {
    _main.drawImageNine(image, center, dst, paint);
    _screenshot.drawImageNine(image, center, dst, paint);
  }

  @override
  void drawImageRect(ui.Image image, Rect src, Rect dst, Paint paint) {
    _main.drawImageRect(image, src, dst, paint);
    _screenshot.drawImageRect(image, src, dst, paint);
  }

  @override
  void drawLine(Offset p1, Offset p2, Paint paint) {
    _main.drawLine(p1, p2, paint);
    _screenshot.drawLine(p1, p2, paint);
  }

  @override
  void drawOval(Rect rect, Paint paint) {
    _main.drawOval(rect, paint);
    _screenshot.drawOval(rect, paint);
  }

  @override
  void drawPaint(Paint paint) {
    _main.drawPaint(paint);
    _screenshot.drawPaint(paint);
  }

  @override
  void drawParagraph(ui.Paragraph paragraph, Offset offset) {
    _main.drawParagraph(paragraph, offset);
    _screenshot.drawParagraph(paragraph, offset);
  }

  @override
  void drawPath(Path path, Paint paint) {
    _main.drawPath(path, paint);
    _screenshot.drawPath(path, paint);
  }

  @override
  void drawPicture(ui.Picture picture) {
    _main.drawPicture(picture);
    _screenshot.drawPicture(picture);
  }

  @override
  void drawPoints(ui.PointMode pointMode, List<Offset> points, Paint paint) {
    _main.drawPoints(pointMode, points, paint);
    _screenshot.drawPoints(pointMode, points, paint);
  }

  @override
  void drawRRect(RRect rrect, Paint paint) {
    _main.drawRRect(rrect, paint);
    _screenshot.drawRRect(rrect, paint);
  }

  @override
  void drawRawAtlas(
    ui.Image atlas,
    Float32List rstTransforms,
    Float32List rects,
    Int32List? colors,
    BlendMode? blendMode,
    Rect? cullRect,
    Paint paint,
  ) {
    _main.drawRawAtlas(atlas, rstTransforms, rects, colors, blendMode, cullRect, paint);
    _screenshot.drawRawAtlas(atlas, rstTransforms, rects, colors, blendMode, cullRect, paint);
  }

  @override
  void drawRawPoints(ui.PointMode pointMode, Float32List points, Paint paint) {
    _main.drawRawPoints(pointMode, points, paint);
    _screenshot.drawRawPoints(pointMode, points, paint);
  }

  @override
  void drawRect(Rect rect, Paint paint) {
    _main.drawRect(rect, paint);
    _screenshot.drawRect(rect, paint);
  }

  @override
  void drawShadow(Path path, Color color, double elevation, bool transparentOccluder) {
    _main.drawShadow(path, color, elevation, transparentOccluder);
    _screenshot.drawShadow(path, color, elevation, transparentOccluder);
  }

  @override
  void drawVertices(ui.Vertices vertices, BlendMode blendMode, Paint paint) {
    _main.drawVertices(vertices, blendMode, paint);
    _screenshot.drawVertices(vertices, blendMode, paint);
  }

  @override
  int getSaveCount() {
    // The main canvas is used instead of the screenshot canvas as the main
    // canvas is guaranteed to be consistent with the canvas expected by the
    // normal paint pipeline so any logic depending on getSaveCount() will
    // behave the same as for the regular paint pipeline.
    return _main.getSaveCount();
  }

  @override
  void restore() {
    _main.restore();
    _screenshot.restore();
  }

  @override
  void rotate(double radians) {
    _main.rotate(radians);
    _screenshot.rotate(radians);
  }

  @override
  void save() {
    _main.save();
    _screenshot.save();
  }

  @override
  void saveLayer(Rect? bounds, Paint paint) {
    _main.saveLayer(bounds, paint);
    _screenshot.saveLayer(bounds, paint);
  }

  @override
  void scale(double sx, [double? sy]) {
    _main.scale(sx, sy);
    _screenshot.scale(sx, sy);
  }

  @override
  void skew(double sx, double sy) {
    _main.skew(sx, sy);
    _screenshot.skew(sx, sy);
  }

  @override
  void transform(Float64List matrix4) {
    _main.transform(matrix4);
    _screenshot.transform(matrix4);
  }

  @override
  void translate(double dx, double dy) {
    _main.translate(dx, dy);
    _screenshot.translate(dx, dy);
  }

  @override
  dynamic noSuchMethod(Invocation invocation) {
    super.noSuchMethod(invocation);
  }
}

Rect _calculateSubtreeBoundsHelper(RenderObject object, Matrix4 transform) {
  Rect bounds = MatrixUtils.transformRect(transform, object.semanticBounds);

  object.visitChildren((RenderObject child) {
    final Matrix4 childTransform = transform.clone();
    object.applyPaintTransform(child, childTransform);
    Rect childBounds = _calculateSubtreeBoundsHelper(child, childTransform);
    final Rect? paintClip = object.describeApproximatePaintClip(child);
    if (paintClip != null) {
      final Rect transformedPaintClip = MatrixUtils.transformRect(transform, paintClip);
      childBounds = childBounds.intersect(transformedPaintClip);
    }

    if (childBounds.isFinite && !childBounds.isEmpty) {
      bounds = bounds.isEmpty ? childBounds : bounds.expandToInclude(childBounds);
    }
  });

  return bounds;
}

/// Calculate bounds for a render object and all of its descendants.
Rect _calculateSubtreeBounds(RenderObject object) {
  return _calculateSubtreeBoundsHelper(object, Matrix4.identity());
}

/// A layer that omits its own offset when adding children to the scene so that
/// screenshots render to the scene in the local coordinate system of the layer.
class _ScreenshotContainerLayer extends OffsetLayer {
  @override
  void addToScene(ui.SceneBuilder builder) {
    addChildrenToScene(builder);
  }
}

/// Data shared between nested [_ScreenshotPaintingContext] objects recording
/// a screenshot.
class _ScreenshotData {
  _ScreenshotData({required this.target}) : containerLayer = _ScreenshotContainerLayer() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$_ScreenshotData',
        object: this,
      );
    }
  }

  /// Target to take a screenshot of.
  final RenderObject target;

  /// Root of the layer tree containing the screenshot.
  final OffsetLayer containerLayer;

  /// Whether the screenshot target has already been found in the render tree.
  bool foundTarget = false;

  /// Whether paint operations should record to the screenshot.
  ///
  /// At least one of [includeInScreenshot] and [includeInRegularContext] must
  /// be true.
  bool includeInScreenshot = false;

  /// Whether paint operations should record to the regular context.
  ///
  /// This should only be set to false before paint operations that should only
  /// apply to the screenshot such rendering debug information about the
  /// [target].
  ///
  /// At least one of [includeInScreenshot] and [includeInRegularContext] must
  /// be true.
  bool includeInRegularContext = true;

  /// Offset of the screenshot corresponding to the offset [target] was given as
  /// part of the regular paint.
  Offset get screenshotOffset {
    assert(foundTarget);
    return containerLayer.offset;
  }

  set screenshotOffset(Offset offset) {
    containerLayer.offset = offset;
  }

  /// Releases allocated resources.
  @mustCallSuper
  void dispose() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    containerLayer.dispose();
  }
}

/// A place to paint to build screenshots of [RenderObject]s.
///
/// Requires that the render objects have already painted successfully as part
/// of the regular rendering pipeline.
/// This painting context behaves the same as standard [PaintingContext] with
/// instrumentation added to compute a screenshot of a specified [RenderObject]
/// added. To correctly mimic the behavior of the regular rendering pipeline, the
/// full subtree of the first [RepaintBoundary] ancestor of the specified
/// [RenderObject] will also be rendered rather than just the subtree of the
/// render object.
class _ScreenshotPaintingContext extends PaintingContext {
  _ScreenshotPaintingContext({
    required ContainerLayer containerLayer,
    required Rect estimatedBounds,
    required _ScreenshotData screenshotData,
  }) : _data = screenshotData,
       super(containerLayer, estimatedBounds);

  final _ScreenshotData _data;

  // Recording state
  PictureLayer? _screenshotCurrentLayer;
  ui.PictureRecorder? _screenshotRecorder;
  Canvas? _screenshotCanvas;
  _MulticastCanvas? _multicastCanvas;

  @override
  Canvas get canvas {
    if (_data.includeInScreenshot) {
      if (_screenshotCanvas == null) {
        _startRecordingScreenshot();
      }
      assert(_screenshotCanvas != null);
      return _data.includeInRegularContext ? _multicastCanvas! : _screenshotCanvas!;
    } else {
      assert(_data.includeInRegularContext);
      return super.canvas;
    }
  }

  bool get _isScreenshotRecording {
    final bool hasScreenshotCanvas = _screenshotCanvas != null;
    assert(() {
      if (hasScreenshotCanvas) {
        assert(_screenshotCurrentLayer != null);
        assert(_screenshotRecorder != null);
        assert(_screenshotCanvas != null);
      } else {
        assert(_screenshotCurrentLayer == null);
        assert(_screenshotRecorder == null);
        assert(_screenshotCanvas == null);
      }
      return true;
    }());
    return hasScreenshotCanvas;
  }

  void _startRecordingScreenshot() {
    assert(_data.includeInScreenshot);
    assert(!_isScreenshotRecording);
    _screenshotCurrentLayer = PictureLayer(estimatedBounds);
    _screenshotRecorder = ui.PictureRecorder();
    _screenshotCanvas = Canvas(_screenshotRecorder!);
    _data.containerLayer.append(_screenshotCurrentLayer!);
    if (_data.includeInRegularContext) {
      _multicastCanvas = _MulticastCanvas(main: super.canvas, screenshot: _screenshotCanvas!);
    } else {
      _multicastCanvas = null;
    }
  }

  @override
  void stopRecordingIfNeeded() {
    super.stopRecordingIfNeeded();
    _stopRecordingScreenshotIfNeeded();
  }

  void _stopRecordingScreenshotIfNeeded() {
    if (!_isScreenshotRecording) {
      return;
    }
    // There is no need to ever draw repaint rainbows as part of the screenshot.
    _screenshotCurrentLayer!.picture = _screenshotRecorder!.endRecording();
    _screenshotCurrentLayer = null;
    _screenshotRecorder = null;
    _multicastCanvas = null;
    _screenshotCanvas = null;
  }

  @override
  void appendLayer(Layer layer) {
    if (_data.includeInRegularContext) {
      super.appendLayer(layer);
      if (_data.includeInScreenshot) {
        assert(!_isScreenshotRecording);
        // We must use a proxy layer here as the layer is already attached to
        // the regular layer tree.
        _data.containerLayer.append(_ProxyLayer(layer));
      }
    } else {
      // Only record to the screenshot.
      assert(!_isScreenshotRecording);
      assert(_data.includeInScreenshot);
      layer.remove();
      _data.containerLayer.append(layer);
      return;
    }
  }

  @override
  PaintingContext createChildContext(ContainerLayer childLayer, Rect bounds) {
    if (_data.foundTarget) {
      // We have already found the screenshotTarget in the layer tree
      // so we can optimize and use a standard PaintingContext.
      return super.createChildContext(childLayer, bounds);
    } else {
      return _ScreenshotPaintingContext(
        containerLayer: childLayer,
        estimatedBounds: bounds,
        screenshotData: _data,
      );
    }
  }

  @override
  void paintChild(RenderObject child, Offset offset) {
    final bool isScreenshotTarget = identical(child, _data.target);
    if (isScreenshotTarget) {
      assert(!_data.includeInScreenshot);
      assert(!_data.foundTarget);
      _data.foundTarget = true;
      _data.screenshotOffset = offset;
      _data.includeInScreenshot = true;
    }
    super.paintChild(child, offset);
    if (isScreenshotTarget) {
      _stopRecordingScreenshotIfNeeded();
      _data.includeInScreenshot = false;
    }
  }

  /// Captures an image of the current state of [renderObject] and its children.
  ///
  /// The returned [ui.Image] has uncompressed raw RGBA bytes, will be offset
  /// by the top-left corner of [renderBounds], and have dimensions equal to the
  /// size of [renderBounds] multiplied by [pixelRatio].
  ///
  /// To use [toImage], the render object must have gone through the paint phase
  /// (i.e. [RenderObject.debugNeedsPaint] must be false).
  ///
  /// The [pixelRatio] describes the scale between the logical pixels and the
  /// size of the output image. It is independent of the
  /// [FlutterView.devicePixelRatio] for the device, so specifying 1.0 (the default)
  /// will give you a 1:1 mapping between logical pixels and the output pixels
  /// in the image.
  ///
  /// The [debugPaint] argument specifies whether the image should include the
  /// output of [RenderObject.debugPaint] for [renderObject] with
  /// [debugPaintSizeEnabled] set to true. Debug paint information is not
  /// included for the children of [renderObject] so that it is clear precisely
  /// which object the debug paint information references.
  ///
  /// See also:
  ///
  ///  * [RenderRepaintBoundary.toImage] for a similar API for [RenderObject]s
  ///    that are repaint boundaries that can be used outside of the inspector.
  ///  * [OffsetLayer.toImage] for a similar API at the layer level.
  ///  * [dart:ui.Scene.toImage] for more information about the image returned.
  static Future<ui.Image> toImage(
    RenderObject renderObject,
    Rect renderBounds, {
    double pixelRatio = 1.0,
    bool debugPaint = false,
  }) async {
    RenderObject repaintBoundary = renderObject;
    while (!repaintBoundary.isRepaintBoundary) {
      repaintBoundary = repaintBoundary.parent!;
    }
    final _ScreenshotData data = _ScreenshotData(target: renderObject);
    final _ScreenshotPaintingContext context = _ScreenshotPaintingContext(
      containerLayer: repaintBoundary.debugLayer!,
      estimatedBounds: repaintBoundary.paintBounds,
      screenshotData: data,
    );

    if (identical(renderObject, repaintBoundary)) {
      // Painting the existing repaint boundary to the screenshot is sufficient.
      // We don't just take a direct screenshot of the repaint boundary as we
      // want to capture debugPaint information as well.
      data.containerLayer.append(_ProxyLayer(repaintBoundary.debugLayer!));
      data.foundTarget = true;
      final OffsetLayer offsetLayer = repaintBoundary.debugLayer! as OffsetLayer;
      data.screenshotOffset = offsetLayer.offset;
    } else {
      // Repaint everything under the repaint boundary.
      // We call debugInstrumentRepaintCompositedChild instead of paintChild as
      // we need to force everything under the repaint boundary to repaint.
      PaintingContext.debugInstrumentRepaintCompositedChild(
        repaintBoundary,
        customContext: context,
      );
    }

    // The check that debugPaintSizeEnabled is false exists to ensure we only
    // call debugPaint when it wasn't already called.
    if (debugPaint && !debugPaintSizeEnabled) {
      data.includeInRegularContext = false;
      // Existing recording may be to a canvas that draws to both the normal and
      // screenshot canvases.
      context.stopRecordingIfNeeded();
      assert(data.foundTarget);
      data.includeInScreenshot = true;

      debugPaintSizeEnabled = true;
      try {
        renderObject.debugPaint(context, data.screenshotOffset);
      } finally {
        debugPaintSizeEnabled = false;
        context.stopRecordingIfNeeded();
      }
    }

    // We must build the regular scene before we can build the screenshot
    // scene as building the screenshot scene assumes addToScene has already
    // been called successfully for all layers in the regular scene.
    repaintBoundary.debugLayer!.buildScene(ui.SceneBuilder());

    final ui.Image image;

    try {
      image = await data.containerLayer.toImage(renderBounds, pixelRatio: pixelRatio);
    } finally {
      data.dispose();
    }

    return image;
  }
}

/// A class describing a step along a path through a tree of [DiagnosticsNode]
/// objects.
///
/// This class is used to bundle all data required to display the tree with just
/// the nodes along a path expanded into a single JSON payload.
class _DiagnosticsPathNode {
  /// Creates a full description of a step in a path through a tree of
  /// [DiagnosticsNode] objects.
  _DiagnosticsPathNode({required this.node, required this.children, this.childIndex});

  /// Node at the point in the path this [_DiagnosticsPathNode] is describing.
  final DiagnosticsNode node;

  /// Children of the [node] being described.
  ///
  /// This value is cached instead of relying on `node.getChildren()` as that
  /// method call might create new [DiagnosticsNode] objects for each child
  /// and we would prefer to use the identical [DiagnosticsNode] for each time
  /// a node exists in the path.
  final List<DiagnosticsNode> children;

  /// Index of the child that the path continues on.
  ///
  /// Equal to null if the path does not continue.
  final int? childIndex;
}

List<_DiagnosticsPathNode>? _followDiagnosticableChain(
  List<Diagnosticable> chain, {
  String? name,
  DiagnosticsTreeStyle? style,
}) {
  final List<_DiagnosticsPathNode> path = <_DiagnosticsPathNode>[];
  if (chain.isEmpty) {
    return path;
  }
  DiagnosticsNode diagnostic = chain.first.toDiagnosticsNode(name: name, style: style);
  for (int i = 1; i < chain.length; i += 1) {
    final Diagnosticable target = chain[i];
    bool foundMatch = false;
    final List<DiagnosticsNode> children = diagnostic.getChildren();
    for (int j = 0; j < children.length; j += 1) {
      final DiagnosticsNode child = children[j];
      if (child.value == target) {
        foundMatch = true;
        path.add(_DiagnosticsPathNode(node: diagnostic, children: children, childIndex: j));
        diagnostic = child;
        break;
      }
    }
    assert(foundMatch);
  }
  path.add(_DiagnosticsPathNode(node: diagnostic, children: diagnostic.getChildren()));
  return path;
}

/// Signature for the selection change callback used by
/// [WidgetInspectorService.selectionChangedCallback].
typedef InspectorSelectionChangedCallback = void Function();

/// Structure to help reference count Dart objects referenced by a GUI tool
/// using [WidgetInspectorService].
///
/// Does not hold the object from garbage collection.
@visibleForTesting
class InspectorReferenceData {
  /// Creates an instance of [InspectorReferenceData].
  InspectorReferenceData(Object object, this.id) {
    // These types are not supported by [WeakReference].
    // See https://api.dart.dev/stable/3.0.2/dart-core/WeakReference-class.html
    if (object is String || object is num || object is bool) {
      _value = object;
      return;
    }

    _ref = WeakReference<Object>(object);
  }

  WeakReference<Object>? _ref;

  Object? _value;

  /// The id of the object in the widget inspector records.
  final String id;

  /// The number of times the object has been referenced.
  int count = 1;

  /// The value.
  Object? get value => _ref?.target ?? _value;
}

// Production implementation of [WidgetInspectorService].
class _WidgetInspectorService with WidgetInspectorService {
  _WidgetInspectorService() {
    selection.addListener(() => selectionChangedCallback?.call());
  }
}

/// Service used by GUI tools to interact with the [WidgetInspector].
///
/// Calls to this object are typically made from GUI tools such as the [Flutter
/// IntelliJ Plugin](https://github.com/flutter/flutter-intellij/blob/master/README.md)
/// using the [Dart VM Service](https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md).
/// This class uses its own object id and manages object lifecycles itself
/// instead of depending on the [object ids](https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md#getobject)
/// specified by the VM Service Protocol because the VM Service Protocol ids
/// expire unpredictably. Object references are tracked in groups so that tools
/// that clients can use dereference all objects in a group with a single
/// operation making it easier to avoid memory leaks.
///
/// All methods in this class are appropriate to invoke from debugging tools
/// using the VM service protocol to evaluate Dart expressions of the
/// form `WidgetInspectorService.instance.methodName(arg1, arg2, ...)`. If you
/// make changes to any instance method of this class you need to verify that
/// the [Flutter IntelliJ Plugin](https://github.com/flutter/flutter-intellij/blob/master/README.md)
/// widget inspector support still works with the changes.
///
/// All methods returning String values return JSON.
mixin WidgetInspectorService {
  /// Ring of cached JSON values to prevent JSON from being garbage
  /// collected before it can be requested over the VM service protocol.
  final List<String?> _serializeRing = List<String?>.filled(20, null);
  int _serializeRingIndex = 0;

  /// The current [WidgetInspectorService].
  static WidgetInspectorService get instance => _instance;
  static WidgetInspectorService _instance = _WidgetInspectorService();

  /// Enables select mode for the Inspector.
  ///
  /// In select mode, pointer interactions trigger widget selection instead of
  /// normal interactions. Otherwise the previously selected widget is
  /// highlighted but the application can be interacted with normally.
  @visibleForTesting
  set isSelectMode(bool enabled) {
    _changeWidgetSelectionMode(enabled);
  }

  @protected
  static set instance(WidgetInspectorService instance) {
    _instance = instance;
  }

  static bool _debugServiceExtensionsRegistered = false;

  /// Ground truth tracking what object(s) are currently selected used by both
  /// GUI tools such as the Flutter IntelliJ Plugin and the [WidgetInspector]
  /// displayed on the device.
  final InspectorSelection selection = InspectorSelection();

  /// Callback typically registered by the [WidgetInspector] to receive
  /// notifications when [selection] changes.
  ///
  /// The Flutter IntelliJ Plugin does not need to listen for this event as it
  /// instead listens for `dart:developer` `inspect` events which also trigger
  /// when the inspection target changes on device.
  InspectorSelectionChangedCallback? selectionChangedCallback;

  /// The VM service protocol does not keep alive object references so this
  /// class needs to manually manage groups of objects that should be kept
  /// alive.
  final Map<String, Set<InspectorReferenceData>> _groups = <String, Set<InspectorReferenceData>>{};
  final Map<String, InspectorReferenceData> _idToReferenceData = <String, InspectorReferenceData>{};
  final WeakMap<Object, String> _objectToId = WeakMap<Object, String>();
  int _nextId = 0;

  /// The pubRootDirectories that are currently configured for the widget inspector.
  List<String>? _pubRootDirectories;

  /// Memoization for [_isLocalCreationLocation].
  final HashMap<String, bool> _isLocalCreationCache = HashMap<String, bool>();

  bool _trackRebuildDirtyWidgets = false;
  bool _trackRepaintWidgets = false;

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name").
  ///
  /// The given callback is called when the extension method is called. The
  /// callback must return a value that can be converted to JSON using
  /// `json.encode()` (see [JsonEncoder]). The return value is stored as a
  /// property named `result` in the JSON. In case of failure, the failure is
  /// reported to the remote caller and is dumped to the logs.
  @protected
  void registerServiceExtension({
    required String name,
    required ServiceExtensionCallback callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerExtension(name: 'inspector.$name', callback: callback);
  }

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name"), which takes no arguments.
  void _registerSignalServiceExtension({
    required String name,
    required FutureOr<Object?> Function() callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerServiceExtension(
      name: name,
      callback: (Map<String, String> parameters) async {
        return <String, Object?>{'result': await callback()};
      },
      registerExtension: registerExtension,
    );
  }

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name"), which takes a single optional argument
  /// "objectGroup" specifying what group is used to manage lifetimes of
  /// object references in the returned JSON (see [disposeGroup]).
  /// If "objectGroup" is omitted, the returned JSON will not include any object
  /// references to avoid leaking memory.
  void _registerObjectGroupServiceExtension({
    required String name,
    required FutureOr<Object?> Function(String objectGroup) callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerServiceExtension(
      name: name,
      callback: (Map<String, String> parameters) async {
        return <String, Object?>{'result': await callback(parameters['objectGroup']!)};
      },
      registerExtension: registerExtension,
    );
  }

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name"), which takes a single argument
  /// "enabled" which can have the value "true" or the value "false"
  /// or can be omitted to read the current value. (Any value other
  /// than "true" is considered equivalent to "false". Other arguments
  /// are ignored.)
  ///
  /// Calls the `getter` callback to obtain the value when
  /// responding to the service extension method being called.
  ///
  /// Calls the `setter` callback with the new value when the
  /// service extension method is called with a new value.
  void _registerBoolServiceExtension({
    required String name,
    required AsyncValueGetter<bool> getter,
    required AsyncValueSetter<bool> setter,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerServiceExtension(
      name: name,
      callback: (Map<String, String> parameters) async {
        if (parameters.containsKey('enabled')) {
          final bool value = parameters['enabled'] == 'true';
          await setter(value);
          _postExtensionStateChangedEvent(name, value);
        }
        return <String, dynamic>{'enabled': await getter() ? 'true' : 'false'};
      },
      registerExtension: registerExtension,
    );
  }

  /// Sends an event when a service extension's state is changed.
  ///
  /// Clients should listen for this event to stay aware of the current service
  /// extension state. Any service extension that manages a state should call
  /// this method on state change.
  ///
  /// `value` reflects the newly updated service extension value.
  ///
  /// This will be called automatically for service extensions registered via
  /// [BindingBase.registerBoolServiceExtension].
  void _postExtensionStateChangedEvent(String name, Object? value) {
    postEvent('Flutter.ServiceExtensionStateChanged', <String, Object?>{
      'extension': 'ext.flutter.inspector.$name',
      'value': value,
    });
  }

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name") which takes an optional parameter named
  /// "arg" and a required parameter named "objectGroup" used to control the
  /// lifetimes of object references in the returned JSON (see [disposeGroup]).
  void _registerServiceExtensionWithArg({
    required String name,
    required FutureOr<Object?> Function(String? objectId, String objectGroup) callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerServiceExtension(
      name: name,
      callback: (Map<String, String> parameters) async {
        assert(parameters.containsKey('objectGroup'));
        return <String, Object?>{
          'result': await callback(parameters['arg'], parameters['objectGroup']!),
        };
      },
      registerExtension: registerExtension,
    );
  }

  /// Registers a service extension method with the given name (full
  /// name "ext.flutter.inspector.name"), that takes arguments
  /// "arg0", "arg1", "arg2", ..., "argn".
  void _registerServiceExtensionVarArgs({
    required String name,
    required FutureOr<Object?> Function(List<String> args) callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    registerServiceExtension(
      name: name,
      callback: (Map<String, String> parameters) async {
        int index;
        final List<String> args = <String>[
          for (index = 0; parameters['arg$index'] != null; index++) parameters['arg$index']!,
        ];
        // Verify that the only arguments other than perhaps 'isolateId' are
        // arguments we have already handled.
        assert(
          index == parameters.length ||
              (index == parameters.length - 1 && parameters.containsKey('isolateId')),
        );
        return <String, Object?>{'result': await callback(args)};
      },
      registerExtension: registerExtension,
    );
  }

  /// Cause the entire tree to be rebuilt. This is used by development tools
  /// when the application code has changed and is being hot-reloaded, to cause
  /// the widget tree to pick up any changed implementations.
  ///
  /// This is expensive and should not be called except during development.
  @protected
  Future<void> forceRebuild() {
    final WidgetsBinding binding = WidgetsBinding.instance;
    if (binding.rootElement != null) {
      binding.buildOwner!.reassemble(binding.rootElement!);
      return binding.endOfFrame;
    }
    return Future<void>.value();
  }

  static const String _consoleObjectGroup = 'console-group';

  int _errorsSinceReload = 0;

  void _reportStructuredError(FlutterErrorDetails details) {
    final Map<String, Object?> errorJson =
        _nodeToJson(
          details.toDiagnosticsNode(),
          InspectorSerializationDelegate(
            groupName: _consoleObjectGroup,
            subtreeDepth: 5,
            includeProperties: true,
            maxDescendantsTruncatableNode: 5,
            service: this,
          ),
        )!;

    errorJson['errorsSinceReload'] = _errorsSinceReload;
    if (_errorsSinceReload == 0) {
      errorJson['renderedErrorText'] =
          TextTreeRenderer(
            wrapWidthProperties: FlutterError.wrapWidth,
            maxDescendentsTruncatableNode: 5,
          ).render(details.toDiagnosticsNode(style: DiagnosticsTreeStyle.error)).trimRight();
    } else {
      errorJson['renderedErrorText'] = 'Another exception was thrown: ${details.summary}';
    }

    _errorsSinceReload += 1;
    postEvent('Flutter.Error', errorJson);
  }

  /// Resets the count of errors since the last hot reload.
  ///
  /// This data is sent to clients as part of the 'Flutter.Error' service
  /// protocol event. Clients may choose to display errors received after the
  /// first error differently.
  void _resetErrorCount() {
    _errorsSinceReload = 0;
  }

  /// Whether structured errors are enabled.
  ///
  /// Structured errors provide semantic information that can be used by IDEs
  /// to enhance the display of errors with rich formatting.
  bool isStructuredErrorsEnabled() {
    // This is a debug mode only feature and will default to false for
    // profile mode.
    bool enabled = false;
    assert(() {
      // TODO(kenz): add support for structured errors on the web.
      enabled = const bool.fromEnvironment(
        'flutter.inspector.structuredErrors',
        defaultValue: !kIsWeb,
      );
      return true;
    }());
    return enabled;
  }

  /// Called to register service extensions.
  ///
  /// See also:
  ///
  ///  * <https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md#rpcs-requests-and-responses>
  ///  * [BindingBase.initServiceExtensions], which explains when service
  ///    extensions can be used.
  void initServiceExtensions(RegisterServiceExtensionCallback registerExtension) {
    final FlutterExceptionHandler defaultExceptionHandler = FlutterError.presentError;

    if (isStructuredErrorsEnabled()) {
      FlutterError.presentError = _reportStructuredError;
    }
    assert(!_debugServiceExtensionsRegistered);
    assert(() {
      _debugServiceExtensionsRegistered = true;
      return true;
    }());

    SchedulerBinding.instance.addPersistentFrameCallback(_onFrameStart);

    _registerBoolServiceExtension(
      name: WidgetInspectorServiceExtensions.structuredErrors.name,
      getter: () async => FlutterError.presentError == _reportStructuredError,
      setter: (bool value) {
        FlutterError.presentError = value ? _reportStructuredError : defaultExceptionHandler;
        return Future<void>.value();
      },
      registerExtension: registerExtension,
    );

    _registerBoolServiceExtension(
      name: WidgetInspectorServiceExtensions.show.name,
      getter: () async => WidgetsBinding.instance.debugShowWidgetInspectorOverride,
      setter: (bool value) {
        if (WidgetsBinding.instance.debugShowWidgetInspectorOverride != value) {
          WidgetsBinding.instance.debugShowWidgetInspectorOverride = value;
        }
        return Future<void>.value();
      },
      registerExtension: registerExtension,
    );

    if (isWidgetCreationTracked()) {
      // Service extensions that are only supported if widget creation locations
      // are tracked.
      _registerBoolServiceExtension(
        name: WidgetInspectorServiceExtensions.trackRebuildDirtyWidgets.name,
        getter: () async => _trackRebuildDirtyWidgets,
        setter: (bool value) async {
          if (value == _trackRebuildDirtyWidgets) {
            return;
          }
          _rebuildStats.resetCounts();
          _trackRebuildDirtyWidgets = value;
          if (value) {
            assert(debugOnRebuildDirtyWidget == null);
            debugOnRebuildDirtyWidget = _onRebuildWidget;
            // Trigger a rebuild so there are baseline stats for rebuilds
            // performed by the app.
            await forceRebuild();
            return;
          } else {
            debugOnRebuildDirtyWidget = null;
            return;
          }
        },
        registerExtension: registerExtension,
      );

      _registerSignalServiceExtension(
        name: WidgetInspectorServiceExtensions.widgetLocationIdMap.name,
        callback: () {
          return _locationIdMapToJson();
        },
        registerExtension: registerExtension,
      );

      _registerBoolServiceExtension(
        name: WidgetInspectorServiceExtensions.trackRepaintWidgets.name,
        getter: () async => _trackRepaintWidgets,
        setter: (bool value) async {
          if (value == _trackRepaintWidgets) {
            return;
          }
          _repaintStats.resetCounts();
          _trackRepaintWidgets = value;
          if (value) {
            assert(debugOnProfilePaint == null);
            debugOnProfilePaint = _onPaint;
            // Trigger an immediate paint so the user has some baseline painting
            // stats to view.
            void markTreeNeedsPaint(RenderObject renderObject) {
              renderObject.markNeedsPaint();
              renderObject.visitChildren(markTreeNeedsPaint);
            }

            RendererBinding.instance.renderViews.forEach(markTreeNeedsPaint);
          } else {
            debugOnProfilePaint = null;
          }
        },
        registerExtension: registerExtension,
      );
    }

    _registerSignalServiceExtension(
      name: WidgetInspectorServiceExtensions.disposeAllGroups.name,
      callback: () async {
        disposeAllGroups();
        return null;
      },
      registerExtension: registerExtension,
    );
    _registerObjectGroupServiceExtension(
      name: WidgetInspectorServiceExtensions.disposeGroup.name,
      callback: (String name) async {
        disposeGroup(name);
        return null;
      },
      registerExtension: registerExtension,
    );
    _registerSignalServiceExtension(
      name: WidgetInspectorServiceExtensions.isWidgetTreeReady.name,
      callback: isWidgetTreeReady,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.disposeId.name,
      callback: (String? objectId, String objectGroup) async {
        disposeId(objectId, objectGroup);
        return null;
      },
      registerExtension: registerExtension,
    );
    _registerServiceExtensionVarArgs(
      name: WidgetInspectorServiceExtensions.setPubRootDirectories.name,
      callback: (List<String> args) async {
        setPubRootDirectories(args);
        return null;
      },
      registerExtension: registerExtension,
    );
    _registerServiceExtensionVarArgs(
      name: WidgetInspectorServiceExtensions.addPubRootDirectories.name,
      callback: (List<String> args) async {
        addPubRootDirectories(args);
        return null;
      },
      registerExtension: registerExtension,
    );
    _registerServiceExtensionVarArgs(
      name: WidgetInspectorServiceExtensions.removePubRootDirectories.name,
      callback: (List<String> args) async {
        removePubRootDirectories(args);
        return null;
      },
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.getPubRootDirectories.name,
      callback: pubRootDirectories,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.setSelectionById.name,
      callback: setSelectionById,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getParentChain.name,
      callback: _getParentChain,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getProperties.name,
      callback: _getProperties,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getChildren.name,
      callback: _getChildren,
      registerExtension: registerExtension,
    );

    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getChildrenSummaryTree.name,
      callback: _getChildrenSummaryTree,
      registerExtension: registerExtension,
    );

    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getChildrenDetailsSubtree.name,
      callback: _getChildrenDetailsSubtree,
      registerExtension: registerExtension,
    );

    _registerObjectGroupServiceExtension(
      name: WidgetInspectorServiceExtensions.getRootWidget.name,
      callback: _getRootWidget,
      registerExtension: registerExtension,
    );
    _registerObjectGroupServiceExtension(
      name: WidgetInspectorServiceExtensions.getRootWidgetSummaryTree.name,
      callback: _getRootWidgetSummaryTree,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.getRootWidgetSummaryTreeWithPreviews.name,
      callback: _getRootWidgetSummaryTreeWithPreviews,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.getRootWidgetTree.name,
      callback: _getRootWidgetTree,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.getDetailsSubtree.name,
      callback: (Map<String, String> parameters) async {
        assert(parameters.containsKey('objectGroup'));
        final String? subtreeDepth = parameters['subtreeDepth'];
        return <String, Object?>{
          'result': _getDetailsSubtree(
            parameters['arg'],
            parameters['objectGroup'],
            subtreeDepth != null ? int.parse(subtreeDepth) : 2,
          ),
        };
      },
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getSelectedWidget.name,
      callback: _getSelectedWidget,
      registerExtension: registerExtension,
    );
    _registerServiceExtensionWithArg(
      name: WidgetInspectorServiceExtensions.getSelectedSummaryWidget.name,
      callback: _getSelectedSummaryWidget,
      registerExtension: registerExtension,
    );

    _registerSignalServiceExtension(
      name: WidgetInspectorServiceExtensions.isWidgetCreationTracked.name,
      callback: isWidgetCreationTracked,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.screenshot.name,
      callback: (Map<String, String> parameters) async {
        assert(parameters.containsKey('id'));
        assert(parameters.containsKey('width'));
        assert(parameters.containsKey('height'));

        final ui.Image? image = await screenshot(
          toObject(parameters['id']),
          width: double.parse(parameters['width']!),
          height: double.parse(parameters['height']!),
          margin: parameters.containsKey('margin') ? double.parse(parameters['margin']!) : 0.0,
          maxPixelRatio:
              parameters.containsKey('maxPixelRatio')
                  ? double.parse(parameters['maxPixelRatio']!)
                  : 1.0,
          debugPaint: parameters['debugPaint'] == 'true',
        );
        if (image == null) {
          return <String, Object?>{'result': null};
        }
        final ByteData? byteData = await image.toByteData(format: ui.ImageByteFormat.png);
        image.dispose();

        return <String, Object>{'result': base64.encoder.convert(Uint8List.view(byteData!.buffer))};
      },
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.getLayoutExplorerNode.name,
      callback: _getLayoutExplorerNode,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.setFlexFit.name,
      callback: _setFlexFit,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.setFlexFactor.name,
      callback: _setFlexFactor,
      registerExtension: registerExtension,
    );
    registerServiceExtension(
      name: WidgetInspectorServiceExtensions.setFlexProperties.name,
      callback: _setFlexProperties,
      registerExtension: registerExtension,
    );
  }

  void _clearStats() {
    _rebuildStats.resetCounts();
    _repaintStats.resetCounts();
  }

  /// Clear all InspectorService object references.
  ///
  /// Use this method only for testing to ensure that object references from one
  /// test case do not impact other test cases.
  @visibleForTesting
  @protected
  void disposeAllGroups() {
    _groups.clear();
    _idToReferenceData.clear();
    _objectToId.clear();
    _nextId = 0;
  }

  /// Reset all InspectorService state.
  ///
  /// Use this method only for testing to write hermetic tests for
  /// WidgetInspectorService.
  @visibleForTesting
  @protected
  @mustCallSuper
  void resetAllState() {
    disposeAllGroups();
    selection.clear();
    resetPubRootDirectories();
  }

  /// Free all references to objects in a group.
  ///
  /// Objects and their associated ids in the group may be kept alive by
  /// references from a different group.
  @protected
  void disposeGroup(String name) {
    final Set<InspectorReferenceData>? references = _groups.remove(name);
    if (references == null) {
      return;
    }
    references.forEach(_decrementReferenceCount);
  }

  void _decrementReferenceCount(InspectorReferenceData reference) {
    reference.count -= 1;
    assert(reference.count >= 0);
    if (reference.count == 0) {
      final Object? value = reference.value;
      if (value != null) {
        _objectToId.remove(value);
      }
      _idToReferenceData.remove(reference.id);
    }
  }

  /// Returns a unique id for [object] that will remain live at least until
  /// [disposeGroup] is called on [groupName].
  @protected
  String? toId(Object? object, String groupName) {
    if (object == null) {
      return null;
    }

    final Set<InspectorReferenceData> group = _groups.putIfAbsent(
      groupName,
      () => Set<InspectorReferenceData>.identity(),
    );
    String? id = _objectToId[object];
    InspectorReferenceData referenceData;
    if (id == null) {
      // TODO(polina-c): comment here why we increase memory footprint by the prefix 'inspector-'.
      // https://github.com/flutter/devtools/issues/5995
      id = 'inspector-$_nextId';
      _nextId += 1;
      _objectToId[object] = id;
      referenceData = InspectorReferenceData(object, id);
      _idToReferenceData[id] = referenceData;
      group.add(referenceData);
    } else {
      referenceData = _idToReferenceData[id]!;
      if (group.add(referenceData)) {
        referenceData.count += 1;
      }
    }
    return id;
  }

  /// Returns whether the application has rendered its first frame and it is
  /// appropriate to display the Widget tree in the inspector.
  @protected
  bool isWidgetTreeReady([String? groupName]) {
    return WidgetsBinding.instance.debugDidSendFirstFrameEvent;
  }

  /// Returns the Dart object associated with a reference id.
  ///
  /// The `groupName` parameter is not required by is added to regularize the
  /// API surface of the methods in this class called from the Flutter IntelliJ
  /// Plugin.
  @protected
  Object? toObject(String? id, [String? groupName]) {
    if (id == null) {
      return null;
    }

    final InspectorReferenceData? data = _idToReferenceData[id];
    if (data == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[ErrorSummary('Id does not exist.')]);
    }
    return data.value;
  }

  /// Returns the object to introspect to determine the source location of an
  /// object's class.
  ///
  /// The Dart object for the id is returned for all cases but [Element] objects
  /// where the [Widget] configuring the [Element] is returned instead as the
  /// class of the [Widget] is more relevant than the class of the [Element].
  ///
  /// The `groupName` parameter is not required by is added to regularize the
  /// API surface of methods called from the Flutter IntelliJ Plugin.
  @protected
  Object? toObjectForSourceLocation(String id, [String? groupName]) {
    final Object? object = toObject(id);
    if (object is Element) {
      return object.widget;
    }
    return object;
  }

  /// Remove the object with the specified `id` from the specified object
  /// group.
  ///
  /// If the object exists in other groups it will remain alive and the object
  /// id will remain valid.
  @protected
  void disposeId(String? id, String groupName) {
    if (id == null) {
      return;
    }

    final InspectorReferenceData? referenceData = _idToReferenceData[id];
    if (referenceData == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[ErrorSummary('Id does not exist')]);
    }
    if (_groups[groupName]?.remove(referenceData) != true) {
      throw FlutterError.fromParts(<DiagnosticsNode>[ErrorSummary('Id is not in group')]);
    }
    _decrementReferenceCount(referenceData);
  }

  /// Set the list of directories that should be considered part of the local
  /// project.
  ///
  /// The local project directories are used to distinguish widgets created by
  /// the local project from widgets created from inside the framework
  /// or other packages.
  @protected
  @Deprecated(
    'Use addPubRootDirectories instead. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  void setPubRootDirectories(List<String> pubRootDirectories) {
    addPubRootDirectories(pubRootDirectories);
  }

  /// Resets the list of directories, that should be considered part of the
  /// local project, to the value passed in [pubRootDirectories].
  ///
  /// The local project directories are used to distinguish widgets created by
  /// the local project from widgets created from inside the framework
  /// or other packages.
  @visibleForTesting
  @protected
  void resetPubRootDirectories() {
    _pubRootDirectories = <String>[];
    _isLocalCreationCache.clear();
  }

  /// Add a list of directories that should be considered part of the local
  /// project.
  ///
  /// The local project directories are used to distinguish widgets created by
  /// the local project from widgets created from inside the framework
  /// or other packages.
  @protected
  void addPubRootDirectories(List<String> pubRootDirectories) {
    pubRootDirectories =
        pubRootDirectories.map<String>((String directory) => Uri.parse(directory).path).toList();

    final Set<String> directorySet = Set<String>.from(pubRootDirectories);
    if (_pubRootDirectories != null) {
      directorySet.addAll(_pubRootDirectories!);
    }

    _pubRootDirectories = directorySet.toList();
    _isLocalCreationCache.clear();
  }

  /// Remove a list of directories that should no longer be considered part
  /// of the local project.
  ///
  /// The local project directories are used to distinguish widgets created by
  /// the local project from widgets created from inside the framework
  /// or other packages.
  @protected
  void removePubRootDirectories(List<String> pubRootDirectories) {
    if (_pubRootDirectories == null) {
      return;
    }
    pubRootDirectories =
        pubRootDirectories.map<String>((String directory) => Uri.parse(directory).path).toList();

    final Set<String> directorySet = Set<String>.from(_pubRootDirectories!);
    directorySet.removeAll(pubRootDirectories);

    _pubRootDirectories = directorySet.toList();
    _isLocalCreationCache.clear();
  }

  /// Returns the list of directories that should be considered part of the
  /// local project.
  @protected
  @visibleForTesting
  Future<Map<String, dynamic>> pubRootDirectories(Map<String, String> parameters) {
    return Future<Map<String, Object>>.value(<String, Object>{
      'result': _pubRootDirectories ?? <String>[],
    });
  }

  /// Set the [WidgetInspector] selection to the object matching the specified
  /// id if the object is valid object to set as the inspector selection.
  ///
  /// Returns true if the selection was changed.
  ///
  /// The `groupName` parameter is not required by is added to regularize the
  /// API surface of methods called from the Flutter IntelliJ Plugin.
  @protected
  bool setSelectionById(String? id, [String? groupName]) {
    return setSelection(toObject(id), groupName);
  }

  /// Set the [WidgetInspector] selection to the specified `object` if it is
  /// a valid object to set as the inspector selection.
  ///
  /// Returns true if the selection was changed.
  ///
  /// The `groupName` parameter is not needed but is specified to regularize the
  /// API surface of methods called from the Flutter IntelliJ Plugin.
  @protected
  bool setSelection(Object? object, [String? groupName]) {
    switch (object) {
      case Element() when object != selection.currentElement:
        selection.currentElement = object;
        _sendInspectEvent(selection.currentElement);
        return true;
      case RenderObject() when object != selection.current:
        selection.current = object;
        _sendInspectEvent(selection.current);
        return true;
    }
    return false;
  }

  /// Notify attached tools to navigate to an object's source location.
  void _sendInspectEvent(Object? object) {
    inspect(object);

    final _Location? location = _getSelectedSummaryWidgetLocation(null);
    if (location != null) {
      postEvent('navigate', <String, Object>{
        'fileUri': location.file, // URI file path of the location.
        'line': location.line, // 1-based line number.
        'column': location.column, // 1-based column number.
        'source': 'flutter.inspector',
      }, stream: 'ToolEvent');
    }
  }

  /// Changes whether widget selection mode is [enabled].
  void _changeWidgetSelectionMode(bool enabled) {
    WidgetsBinding.instance.debugShowWidgetInspectorOverride = enabled;
    _postExtensionStateChangedEvent(WidgetInspectorServiceExtensions.show.name, enabled);
  }

  /// Returns a DevTools uri linking to a specific element on the inspector page.
  String? _devToolsInspectorUriForElement(Element element) {
    if (activeDevToolsServerAddress != null && connectedVmServiceUri != null) {
      final String? inspectorRef = toId(element, _consoleObjectGroup);
      if (inspectorRef != null) {
        return devToolsInspectorUri(inspectorRef);
      }
    }
    return null;
  }

  /// Returns the DevTools inspector uri for the given vm service connection and
  /// inspector reference.
  @visibleForTesting
  String devToolsInspectorUri(String inspectorRef) {
    assert(activeDevToolsServerAddress != null);
    assert(connectedVmServiceUri != null);

    final Uri uri = Uri.parse(activeDevToolsServerAddress!).replace(
      queryParameters: <String, dynamic>{
        'uri': connectedVmServiceUri,
        'inspectorRef': inspectorRef,
      },
    );

    // We cannot add the '/#/inspector' path by means of
    // [Uri.replace(path: '/#/inspector')] because the '#' character will be
    // encoded when we try to print the url as a string. DevTools will not
    // load properly if this character is encoded in the url.
    // Related: https://github.com/flutter/devtools/issues/2475.
    final String devToolsInspectorUri = uri.toString();
    final int startQueryParamIndex = devToolsInspectorUri.indexOf('?');
    // The query parameter character '?' should be present because we manually
    // added query parameters above.
    assert(startQueryParamIndex != -1);
    return '${devToolsInspectorUri.substring(0, startQueryParamIndex)}'
        '/#/inspector'
        '${devToolsInspectorUri.substring(startQueryParamIndex)}';
  }

  /// Returns JSON representing the chain of [DiagnosticsNode] instances from
  /// root of the tree to the [Element] or [RenderObject] matching `id`.
  ///
  /// The JSON contains all information required to display a tree view with
  /// all nodes other than nodes along the path collapsed.
  @protected
  String getParentChain(String id, String groupName) {
    return _safeJsonEncode(_getParentChain(id, groupName));
  }

  List<Object?> _getParentChain(String? id, String groupName) {
    final Object? value = toObject(id);
    final List<_DiagnosticsPathNode> path = switch (value) {
      RenderObject() => _getRenderObjectParentChain(value, groupName)!,
      Element() => _getElementParentChain(value, groupName),
      _ =>
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Cannot get parent chain for node of type ${value.runtimeType}'),
        ]),
    };

    InspectorSerializationDelegate createDelegate() =>
        InspectorSerializationDelegate(groupName: groupName, service: this);

    return <Object?>[
      for (final _DiagnosticsPathNode pathNode in path)
        if (createDelegate() case final InspectorSerializationDelegate delegate)
          <String, Object?>{
            'node': _nodeToJson(pathNode.node, delegate),
            'children': _nodesToJson(pathNode.children, delegate, parent: pathNode.node),
            'childIndex': pathNode.childIndex,
          },
    ];
  }

  List<Element> _getRawElementParentChain(Element element, {required int? numLocalParents}) {
    List<Element> elements = element.debugGetDiagnosticChain();
    if (numLocalParents != null) {
      for (int i = 0; i < elements.length; i += 1) {
        if (_isValueCreatedByLocalProject(elements[i])) {
          numLocalParents = numLocalParents! - 1;
          if (numLocalParents <= 0) {
            elements = elements.take(i + 1).toList();
            break;
          }
        }
      }
    }
    return elements.reversed.toList();
  }

  List<_DiagnosticsPathNode> _getElementParentChain(
    Element element,
    String groupName, {
    int? numLocalParents,
  }) {
    return _followDiagnosticableChain(
          _getRawElementParentChain(element, numLocalParents: numLocalParents),
        ) ??
        const <_DiagnosticsPathNode>[];
  }

  List<_DiagnosticsPathNode>? _getRenderObjectParentChain(
    RenderObject? renderObject,
    String groupName,
  ) {
    final List<RenderObject> chain = <RenderObject>[];
    while (renderObject != null) {
      chain.add(renderObject);
      renderObject = renderObject.parent;
    }
    return _followDiagnosticableChain(chain.reversed.toList());
  }

  Map<String, Object?>? _nodeToJson(
    DiagnosticsNode? node,
    InspectorSerializationDelegate delegate, {
    bool fullDetails = true,
  }) {
    if (fullDetails) {
      return node?.toJsonMap(delegate);
    } else {
      // If we don't need the full details fetched from all the subclasses, we
      // can iteratively build the JSON map. This prevents a stack overflow
      // exception for particularly large widget trees. For details, see:
      // https://github.com/flutter/devtools/issues/8553
      return node?.toJsonMapIterative(delegate);
    }
  }

  bool _isValueCreatedByLocalProject(Object? value) {
    final _Location? creationLocation = _getCreationLocation(value);
    if (creationLocation == null) {
      return false;
    }
    return _isLocalCreationLocation(creationLocation.file);
  }

  bool _isLocalCreationLocationImpl(String locationUri) {
    final String file = Uri.parse(locationUri).path;

    // By default check whether the creation location was within package:flutter.
    if (_pubRootDirectories == null) {
      // TODO(chunhtai): Make it more robust once
      // https://github.com/flutter/flutter/issues/32660 is fixed.
      return !file.contains('packages/flutter/');
    }
    for (final String directory in _pubRootDirectories!) {
      if (file.startsWith(directory)) {
        return true;
      }
    }
    return false;
  }

  /// Memoized version of [_isLocalCreationLocationImpl].
  bool _isLocalCreationLocation(String locationUri) {
    final bool? cachedValue = _isLocalCreationCache[locationUri];
    if (cachedValue != null) {
      return cachedValue;
    }
    final bool result = _isLocalCreationLocationImpl(locationUri);
    _isLocalCreationCache[locationUri] = result;
    return result;
  }

  /// Wrapper around `json.encode` that uses a ring of cached values to prevent
  /// the Dart garbage collector from collecting objects between when
  /// the value is returned over the VM service protocol and when the
  /// separate observatory protocol command has to be used to retrieve its full
  /// contents.
  //
  // TODO(jacobr): Replace this with a better solution once
  // https://github.com/dart-lang/sdk/issues/32919 is fixed.
  String _safeJsonEncode(Object? object) {
    final String jsonString = json.encode(object);
    _serializeRing[_serializeRingIndex] = jsonString;
    _serializeRingIndex = (_serializeRingIndex + 1) % _serializeRing.length;
    return jsonString;
  }

  List<DiagnosticsNode> _truncateNodes(
    Iterable<DiagnosticsNode> nodes,
    int maxDescendentsTruncatableNode,
  ) {
    if (nodes.every((DiagnosticsNode node) => node.value is Element) && isWidgetCreationTracked()) {
      final List<DiagnosticsNode> localNodes =
          nodes.where((DiagnosticsNode node) => _isValueCreatedByLocalProject(node.value)).toList();
      if (localNodes.isNotEmpty) {
        return localNodes;
      }
    }
    return nodes.take(maxDescendentsTruncatableNode).toList();
  }

  List<Map<String, Object?>> _nodesToJson(
    List<DiagnosticsNode> nodes,
    InspectorSerializationDelegate delegate, {
    required DiagnosticsNode? parent,
  }) {
    return DiagnosticsNode.toJsonList(nodes, parent, delegate);
  }

  /// Returns a JSON representation of the properties of the [DiagnosticsNode]
  /// object that `diagnosticsNodeId` references.
  @protected
  String getProperties(String diagnosticsNodeId, String groupName) {
    return _safeJsonEncode(_getProperties(diagnosticsNodeId, groupName));
  }

  List<Object> _getProperties(String? diagnosticableId, String groupName) {
    final DiagnosticsNode? node = _idToDiagnosticsNode(diagnosticableId);
    if (node == null) {
      return const <Object>[];
    }
    return _nodesToJson(
      node.getProperties(),
      InspectorSerializationDelegate(groupName: groupName, service: this),
      parent: node,
    );
  }

  /// Returns a JSON representation of the children of the [DiagnosticsNode]
  /// object that `diagnosticsNodeId` references.
  String getChildren(String diagnosticsNodeId, String groupName) {
    return _safeJsonEncode(_getChildren(diagnosticsNodeId, groupName));
  }

  List<Object> _getChildren(String? diagnosticsNodeId, String groupName) {
    final DiagnosticsNode? node = toObject(diagnosticsNodeId) as DiagnosticsNode?;
    final InspectorSerializationDelegate delegate = InspectorSerializationDelegate(
      groupName: groupName,
      service: this,
    );
    return _nodesToJson(
      node == null ? const <DiagnosticsNode>[] : _getChildrenFiltered(node, delegate),
      delegate,
      parent: node,
    );
  }

  /// Returns a JSON representation of the children of the [DiagnosticsNode]
  /// object that `diagnosticsNodeId` references only including children that
  /// were created directly by user code.
  ///
  /// {@template flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
  /// Requires [Widget] creation locations which are only available for debug
  /// mode builds when the `--track-widget-creation` flag is enabled on the call
  /// to the `flutter` tool. This flag is enabled by default in debug builds.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [isWidgetCreationTracked] which indicates whether this method can be
  ///    used.
  String getChildrenSummaryTree(String diagnosticsNodeId, String groupName) {
    return _safeJsonEncode(_getChildrenSummaryTree(diagnosticsNodeId, groupName));
  }

  DiagnosticsNode? _idToDiagnosticsNode(String? diagnosticableId) {
    final Object? object = toObject(diagnosticableId);
    return objectToDiagnosticsNode(object);
  }

  /// If possible, returns [DiagnosticsNode] for the object.
  @visibleForTesting
  static DiagnosticsNode? objectToDiagnosticsNode(Object? object) {
    if (object is Diagnosticable) {
      return object.toDiagnosticsNode();
    }
    return null;
  }

  List<Object> _getChildrenSummaryTree(String? diagnosticableId, String groupName) {
    final DiagnosticsNode? node = _idToDiagnosticsNode(diagnosticableId);
    if (node == null) {
      return <Object>[];
    }

    final InspectorSerializationDelegate delegate = InspectorSerializationDelegate(
      groupName: groupName,
      summaryTree: true,
      service: this,
    );
    return _nodesToJson(_getChildrenFiltered(node, delegate), delegate, parent: node);
  }

  /// Returns a JSON representation of the children of the [DiagnosticsNode]
  /// object that [diagnosticableId] references providing information needed
  /// for the details subtree view.
  ///
  /// The details subtree shows properties inline and includes all children
  /// rather than a filtered set of important children.
  String getChildrenDetailsSubtree(String diagnosticableId, String groupName) {
    return _safeJsonEncode(_getChildrenDetailsSubtree(diagnosticableId, groupName));
  }

  List<Object> _getChildrenDetailsSubtree(String? diagnosticableId, String groupName) {
    final DiagnosticsNode? node = _idToDiagnosticsNode(diagnosticableId);
    // With this value of minDepth we only expand one extra level of important nodes.
    final InspectorSerializationDelegate delegate = InspectorSerializationDelegate(
      groupName: groupName,
      includeProperties: true,
      service: this,
    );
    return _nodesToJson(
      node == null ? const <DiagnosticsNode>[] : _getChildrenFiltered(node, delegate),
      delegate,
      parent: node,
    );
  }

  bool _shouldShowInSummaryTree(DiagnosticsNode node) {
    if (node.level == DiagnosticLevel.error) {
      return true;
    }
    final Object? value = node.value;
    if (value is! Diagnosticable) {
      return true;
    }
    if (value is! Element || !isWidgetCreationTracked()) {
      // Creation locations are not available so include all nodes in the
      // summary tree.
      return true;
    }
    return _isValueCreatedByLocalProject(value);
  }

  List<DiagnosticsNode> _getChildrenFiltered(
    DiagnosticsNode node,
    InspectorSerializationDelegate delegate,
  ) {
    return _filterChildren(node.getChildren(), delegate);
  }

  List<DiagnosticsNode> _filterChildren(
    List<DiagnosticsNode> nodes,
    InspectorSerializationDelegate delegate,
  ) {
    final List<DiagnosticsNode> children = <DiagnosticsNode>[
      for (final DiagnosticsNode child in nodes)
        if (!delegate.summaryTree || _shouldShowInSummaryTree(child))
          child
        else
          ..._getChildrenFiltered(child, delegate),
    ];
    return children;
  }

  /// Returns a JSON representation of the [DiagnosticsNode] for the root
  /// [Element].
  String getRootWidget(String groupName) {
    return _safeJsonEncode(_getRootWidget(groupName));
  }

  Map<String, Object?>? _getRootWidget(String groupName) {
    return _nodeToJson(
      WidgetsBinding.instance.rootElement?.toDiagnosticsNode(),
      InspectorSerializationDelegate(groupName: groupName, service: this),
    );
  }

  /// Returns a JSON representation of the [DiagnosticsNode] for the root
  /// [Element] showing only nodes that should be included in a summary tree.
  String getRootWidgetSummaryTree(String groupName) {
    return _safeJsonEncode(_getRootWidgetSummaryTree(groupName));
  }

  Map<String, Object?>? _getRootWidgetSummaryTree(
    String groupName, {
    Map<String, Object>? Function(DiagnosticsNode, InspectorSerializationDelegate)?
    addAdditionalPropertiesCallback,
  }) {
    return _getRootWidgetTreeImpl(
      groupName: groupName,
      isSummaryTree: true,
      withPreviews: false,
      addAdditionalPropertiesCallback: addAdditionalPropertiesCallback,
    );
  }

  Future<Map<String, Object?>> _getRootWidgetSummaryTreeWithPreviews(
    Map<String, String> parameters,
  ) {
    final String groupName = parameters['groupName']!;
    final Map<String, Object?>? result = _getRootWidgetTreeImpl(
      groupName: groupName,
      isSummaryTree: true,
      withPreviews: true,
    );
    return Future<Map<String, dynamic>>.value(<String, dynamic>{'result': result});
  }

  Future<Map<String, Object?>> _getRootWidgetTree(Map<String, String> parameters) {
    final String groupName = parameters['groupName']!;
    final bool isSummaryTree = parameters['isSummaryTree'] == 'true';
    final bool withPreviews = parameters['withPreviews'] == 'true';
    // If the "fullDetails" parameter is not provided, default to true.
    final bool fullDetails = parameters['fullDetails'] != 'false';

    final Map<String, Object?>? result = _getRootWidgetTreeImpl(
      groupName: groupName,
      isSummaryTree: isSummaryTree,
      withPreviews: withPreviews,
      fullDetails: fullDetails,
    );

    return Future<Map<String, dynamic>>.value(<String, dynamic>{'result': result});
  }

  Map<String, Object?>? _getRootWidgetTreeImpl({
    required String groupName,
    required bool isSummaryTree,
    required bool withPreviews,
    bool fullDetails = true,
    Map<String, Object>? Function(DiagnosticsNode, InspectorSerializationDelegate)?
    addAdditionalPropertiesCallback,
  }) {
    final bool shouldAddAdditionalProperties =
        addAdditionalPropertiesCallback != null || withPreviews;

    // Combine the given addAdditionalPropertiesCallback with logic to add text
    // previews as well (if withPreviews is true):
    Map<String, Object>? combinedAddAdditionalPropertiesCallback(
      DiagnosticsNode node,
      InspectorSerializationDelegate delegate,
    ) {
      final Map<String, Object> additionalPropertiesJson =
          addAdditionalPropertiesCallback?.call(node, delegate) ?? <String, Object>{};
      if (!withPreviews) {
        return additionalPropertiesJson;
      }
      final Object? value = node.value;
      if (value is Element) {
        final RenderObject? renderObject = value.renderObject;
        if (renderObject is RenderParagraph) {
          additionalPropertiesJson['textPreview'] = renderObject.text.toPlainText();
        }
      }
      return additionalPropertiesJson;
    }

    return _nodeToJson(
      WidgetsBinding.instance.rootElement?.toDiagnosticsNode(),
      InspectorSerializationDelegate(
        groupName: groupName,
        subtreeDepth: 1000000,
        summaryTree: isSummaryTree,
        service: this,
        addAdditionalPropertiesCallback:
            shouldAddAdditionalProperties ? combinedAddAdditionalPropertiesCallback : null,
      ),
      fullDetails: fullDetails,
    );
  }

  /// Returns a JSON representation of the subtree rooted at the
  /// [DiagnosticsNode] object that `diagnosticsNodeId` references providing
  /// information needed for the details subtree view.
  ///
  /// The number of levels of the subtree that should be returned is specified
  /// by the [subtreeDepth] parameter. This value defaults to 2 for backwards
  /// compatibility.
  ///
  /// See also:
  ///
  ///  * [getChildrenDetailsSubtree], a method to get children of a node
  ///    in the details subtree.
  String getDetailsSubtree(String diagnosticableId, String groupName, {int subtreeDepth = 2}) {
    return _safeJsonEncode(_getDetailsSubtree(diagnosticableId, groupName, subtreeDepth));
  }

  Map<String, Object?>? _getDetailsSubtree(
    String? diagnosticableId,
    String? groupName,
    int subtreeDepth,
  ) {
    final DiagnosticsNode? root = _idToDiagnosticsNode(diagnosticableId);
    if (root == null) {
      return null;
    }
    return _nodeToJson(
      root,
      InspectorSerializationDelegate(
        groupName: groupName,
        subtreeDepth: subtreeDepth,
        includeProperties: true,
        service: this,
      ),
    );
  }

  /// Returns a [DiagnosticsNode] representing the currently selected [Element].
  @protected
  String getSelectedWidget(String? previousSelectionId, String groupName) {
    if (previousSelectionId != null) {
      debugPrint('previousSelectionId is deprecated in API');
    }
    return _safeJsonEncode(_getSelectedWidget(null, groupName));
  }

  /// Captures an image of the current state of an [object] that is a
  /// [RenderObject] or [Element].
  ///
  /// The returned [ui.Image] has uncompressed raw RGBA bytes and will be scaled
  /// to be at most [width] pixels wide and [height] pixels tall. The returned
  /// image will never have a scale between logical pixels and the
  /// size of the output image larger than maxPixelRatio.
  /// [margin] indicates the number of pixels relative to the un-scaled size of
  /// the [object] to include as a margin to include around the bounds of the
  /// [object] in the screenshot. Including a margin can be useful to capture
  /// areas that are slightly outside of the normal bounds of an object such as
  /// some debug paint information.
  @protected
  Future<ui.Image?> screenshot(
    Object? object, {
    required double width,
    required double height,
    double margin = 0.0,
    double maxPixelRatio = 1.0,
    bool debugPaint = false,
  }) async {
    if (object is! Element && object is! RenderObject) {
      return null;
    }
    final RenderObject? renderObject =
        object is Element ? object.renderObject : (object as RenderObject?);
    if (renderObject == null || !renderObject.attached) {
      return null;
    }

    if (renderObject.debugNeedsLayout) {
      final PipelineOwner owner = renderObject.owner!;
      assert(!owner.debugDoingLayout);
      owner
        ..flushLayout()
        ..flushCompositingBits()
        ..flushPaint();

      // If we still need layout, then that means that renderObject was skipped
      // in the layout phase and therefore can't be painted. It is clearer to
      // return null indicating that a screenshot is unavailable than to return
      // an empty image.
      if (renderObject.debugNeedsLayout) {
        return null;
      }
    }

    Rect renderBounds = _calculateSubtreeBounds(renderObject);
    if (margin != 0.0) {
      renderBounds = renderBounds.inflate(margin);
    }
    if (renderBounds.isEmpty) {
      return null;
    }

    final double pixelRatio = math.min(
      maxPixelRatio,
      math.min(width / renderBounds.width, height / renderBounds.height),
    );

    return _ScreenshotPaintingContext.toImage(
      renderObject,
      renderBounds,
      pixelRatio: pixelRatio,
      debugPaint: debugPaint,
    );
  }

  Future<Map<String, Object?>> _getLayoutExplorerNode(Map<String, String> parameters) {
    final String? diagnosticableId = parameters['id'];
    final int subtreeDepth = int.parse(parameters['subtreeDepth']!);
    final String? groupName = parameters['groupName'];
    Map<String, dynamic>? result = <String, dynamic>{};
    final DiagnosticsNode? root = _idToDiagnosticsNode(diagnosticableId);
    if (root == null) {
      return Future<Map<String, dynamic>>.value(<String, dynamic>{'result': result});
    }
    result = _nodeToJson(
      root,
      InspectorSerializationDelegate(
        groupName: groupName,
        summaryTree: true,
        subtreeDepth: subtreeDepth,
        service: this,
        addAdditionalPropertiesCallback: (
          DiagnosticsNode node,
          InspectorSerializationDelegate delegate,
        ) {
          final Object? value = node.value;
          final RenderObject? renderObject = value is Element ? value.renderObject : null;
          if (renderObject == null) {
            return const <String, Object>{};
          }

          final DiagnosticsSerializationDelegate renderObjectSerializationDelegate = delegate
              .copyWith(subtreeDepth: 0, includeProperties: true, expandPropertyValues: false);
          final Map<String, Object> additionalJson = <String, Object>{
            // Only include renderObject properties separately if this value is not already the renderObject.
            // Only include if we are expanding property values to mitigate the risk of infinite loops if
            // RenderObjects have properties that are Element objects.
            if (value is! RenderObject && delegate.expandPropertyValues)
              'renderObject': renderObject.toDiagnosticsNode().toJsonMap(
                renderObjectSerializationDelegate,
              ),
          };

          final RenderObject? renderParent = renderObject.parent;
          if (renderParent != null && delegate.subtreeDepth > 0 && delegate.expandPropertyValues) {
            final Object? parentCreator = renderParent.debugCreator;
            if (parentCreator is DebugCreator) {
              additionalJson['parentRenderElement'] = parentCreator.element
                  .toDiagnosticsNode()
                  .toJsonMap(delegate.copyWith(subtreeDepth: 0, includeProperties: true));
              // TODO(jacobr): also describe the path back up the tree to
              // the RenderParentElement from the current element. It
              // could be a surprising distance up the tree if a lot of
              // elements don't have their own RenderObjects.
            }
          }

          try {
            if (!renderObject.debugNeedsLayout) {
              // ignore: invalid_use_of_protected_member
              final Constraints constraints = renderObject.constraints;
              final Map<String, Object> constraintsProperty = <String, Object>{
                'type': constraints.runtimeType.toString(),
                'description': constraints.toString(),
              };
              if (constraints is BoxConstraints) {
                constraintsProperty.addAll(<String, Object>{
                  'minWidth': constraints.minWidth.toString(),
                  'minHeight': constraints.minHeight.toString(),
                  'maxWidth': constraints.maxWidth.toString(),
                  'maxHeight': constraints.maxHeight.toString(),
                });
              }
              additionalJson['constraints'] = constraintsProperty;
            }
          } catch (e) {
            // Constraints are sometimes unavailable even though
            // debugNeedsLayout is false.
          }

          try {
            if (renderObject is RenderBox) {
              additionalJson['isBox'] = true;
              additionalJson['size'] = <String, Object>{
                'width': renderObject.size.width.toString(),
                'height': renderObject.size.height.toString(),
              };

              final ParentData? parentData = renderObject.parentData;
              if (parentData is FlexParentData) {
                additionalJson['flexFactor'] = parentData.flex!;
                additionalJson['flexFit'] = (parentData.fit ?? FlexFit.tight).name;
              } else if (parentData is BoxParentData) {
                final Offset offset = parentData.offset;
                additionalJson['parentData'] = <String, Object>{
                  'offsetX': offset.dx.toString(),
                  'offsetY': offset.dy.toString(),
                };
              }
            } else if (renderObject is RenderView) {
              additionalJson['size'] = <String, Object>{
                'width': renderObject.size.width.toString(),
                'height': renderObject.size.height.toString(),
              };
            }
          } catch (e) {
            // Not laid out yet.
          }
          return additionalJson;
        },
      ),
    );
    return Future<Map<String, dynamic>>.value(<String, dynamic>{'result': result});
  }

  Future<Map<String, dynamic>> _setFlexFit(Map<String, String> parameters) {
    final String? id = parameters['id'];
    final String parameter = parameters['flexFit']!;
    final FlexFit flexFit = _toEnumEntry<FlexFit>(FlexFit.values, parameter);
    final Object? object = toObject(id);
    bool succeed = false;
    if (object != null && object is Element) {
      final RenderObject? render = object.renderObject;
      final ParentData? parentData = render?.parentData;
      if (parentData is FlexParentData) {
        parentData.fit = flexFit;
        render!.markNeedsLayout();
        succeed = true;
      }
    }
    return Future<Map<String, Object>>.value(<String, Object>{'result': succeed});
  }

  Future<Map<String, dynamic>> _setFlexFactor(Map<String, String> parameters) {
    final String? id = parameters['id'];
    final String flexFactor = parameters['flexFactor']!;
    final int? factor = flexFactor == 'null' ? null : int.parse(flexFactor);
    final dynamic object = toObject(id);
    bool succeed = false;
    if (object != null && object is Element) {
      final RenderObject? render = object.renderObject;
      final ParentData? parentData = render?.parentData;
      if (parentData is FlexParentData) {
        parentData.flex = factor;
        render!.markNeedsLayout();
        succeed = true;
      }
    }
    return Future<Map<String, Object>>.value(<String, Object>{'result': succeed});
  }

  Future<Map<String, dynamic>> _setFlexProperties(Map<String, String> parameters) {
    final String? id = parameters['id'];
    final MainAxisAlignment mainAxisAlignment = _toEnumEntry<MainAxisAlignment>(
      MainAxisAlignment.values,
      parameters['mainAxisAlignment']!,
    );
    final CrossAxisAlignment crossAxisAlignment = _toEnumEntry<CrossAxisAlignment>(
      CrossAxisAlignment.values,
      parameters['crossAxisAlignment']!,
    );
    final Object? object = toObject(id);
    bool succeed = false;
    if (object != null && object is Element) {
      final RenderObject? render = object.renderObject;
      if (render is RenderFlex) {
        render.mainAxisAlignment = mainAxisAlignment;
        render.crossAxisAlignment = crossAxisAlignment;
        render.markNeedsLayout();
        render.markNeedsPaint();
        succeed = true;
      }
    }
    return Future<Map<String, Object>>.value(<String, Object>{'result': succeed});
  }

  T _toEnumEntry<T>(List<T> enumEntries, String name) {
    for (final T entry in enumEntries) {
      if (entry.toString() == name) {
        return entry;
      }
    }
    throw Exception('Enum value $name not found');
  }

  Map<String, Object?>? _getSelectedWidget(String? previousSelectionId, String groupName) {
    return _nodeToJson(
      _getSelectedWidgetDiagnosticsNode(previousSelectionId),
      InspectorSerializationDelegate(groupName: groupName, service: this),
    );
  }

  DiagnosticsNode? _getSelectedWidgetDiagnosticsNode(String? previousSelectionId) {
    final DiagnosticsNode? previousSelection = toObject(previousSelectionId) as DiagnosticsNode?;
    final Element? current = selection.currentElement;
    return current == previousSelection?.value ? previousSelection : current?.toDiagnosticsNode();
  }

  /// Returns a [DiagnosticsNode] representing the currently selected [Element]
  /// if the selected [Element] should be shown in the summary tree otherwise
  /// returns the first ancestor of the selected [Element] shown in the summary
  /// tree.
  String getSelectedSummaryWidget(String? previousSelectionId, String groupName) {
    if (previousSelectionId != null) {
      debugPrint('previousSelectionId is deprecated in API');
    }
    return _safeJsonEncode(_getSelectedSummaryWidget(null, groupName));
  }

  _Location? _getSelectedSummaryWidgetLocation(String? previousSelectionId) {
    return _getCreationLocation(_getSelectedSummaryDiagnosticsNode(previousSelectionId)?.value);
  }

  DiagnosticsNode? _getSelectedSummaryDiagnosticsNode(String? previousSelectionId) {
    if (!isWidgetCreationTracked()) {
      return _getSelectedWidgetDiagnosticsNode(previousSelectionId);
    }
    final DiagnosticsNode? previousSelection = toObject(previousSelectionId) as DiagnosticsNode?;
    Element? current = selection.currentElement;
    if (current != null && !_isValueCreatedByLocalProject(current)) {
      Element? firstLocal;
      for (final Element candidate in current.debugGetDiagnosticChain()) {
        if (_isValueCreatedByLocalProject(candidate)) {
          firstLocal = candidate;
          break;
        }
      }
      current = firstLocal;
    }
    return current == previousSelection?.value ? previousSelection : current?.toDiagnosticsNode();
  }

  Map<String, Object?>? _getSelectedSummaryWidget(String? previousSelectionId, String groupName) {
    return _nodeToJson(
      _getSelectedSummaryDiagnosticsNode(previousSelectionId),
      InspectorSerializationDelegate(groupName: groupName, service: this),
    );
  }

  /// Returns whether [Widget] creation locations are available.
  ///
  /// {@macro flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
  bool isWidgetCreationTracked() {
    _widgetCreationTracked ??= const _WidgetForTypeTests() is _HasCreationLocation;
    return _widgetCreationTracked!;
  }

  bool? _widgetCreationTracked;

  late Duration _frameStart;
  late int _frameNumber;

  void _onFrameStart(Duration timeStamp) {
    _frameStart = timeStamp;
    _frameNumber = PlatformDispatcher.instance.frameData.frameNumber;
    SchedulerBinding.instance.addPostFrameCallback(
      _onFrameEnd,
      debugLabel: 'WidgetInspector.onFrameStart',
    );
  }

  void _onFrameEnd(Duration timeStamp) {
    if (_trackRebuildDirtyWidgets) {
      _postStatsEvent('Flutter.RebuiltWidgets', _rebuildStats);
    }
    if (_trackRepaintWidgets) {
      _postStatsEvent('Flutter.RepaintWidgets', _repaintStats);
    }
  }

  void _postStatsEvent(String eventName, _ElementLocationStatsTracker stats) {
    postEvent(eventName, stats.exportToJson(_frameStart, frameNumber: _frameNumber));
  }

  /// All events dispatched by a [WidgetInspectorService] use this method
  /// instead of calling [developer.postEvent] directly.
  ///
  /// This allows tests for [WidgetInspectorService] to track which events were
  /// dispatched by overriding this method.
  @protected
  void postEvent(String eventKind, Map<Object, Object?> eventData, {String stream = 'Extension'}) {
    developer.postEvent(eventKind, eventData, stream: stream);
  }

  /// All events dispatched by a [WidgetInspectorService] use this method
  /// instead of calling [developer.inspect].
  ///
  /// This allows tests for [WidgetInspectorService] to track which events were
  /// dispatched by overriding this method.
  @protected
  void inspect(Object? object) {
    developer.inspect(object);
  }

  final _ElementLocationStatsTracker _rebuildStats = _ElementLocationStatsTracker();
  final _ElementLocationStatsTracker _repaintStats = _ElementLocationStatsTracker();

  void _onRebuildWidget(Element element, bool builtOnce) {
    _rebuildStats.add(element);
  }

  void _onPaint(RenderObject renderObject) {
    try {
      final Element? element = (renderObject.debugCreator as DebugCreator?)?.element;
      if (element is! RenderObjectElement) {
        // This branch should not hit as long as all RenderObjects were created
        // by Widgets. It is possible there might be some render objects
        // created directly without using the Widget layer so we add this check
        // to improve robustness.
        return;
      }
      _repaintStats.add(element);

      // Give all ancestor elements credit for repainting as long as they do
      // not have their own associated RenderObject.
      element.visitAncestorElements((Element ancestor) {
        if (ancestor is RenderObjectElement) {
          // This ancestor has its own RenderObject so we can precisely track
          // when it repaints.
          return false;
        }
        _repaintStats.add(ancestor);
        return true;
      });
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widget inspector library',
          context: ErrorDescription('while tracking widget repaints'),
        ),
      );
    }
  }

  /// This method is called by [WidgetsBinding.performReassemble] to flush caches
  /// of obsolete values after a hot reload.
  ///
  /// Do not call this method directly. Instead, use
  /// [BindingBase.reassembleApplication].
  void performReassemble() {
    _clearStats();
    _resetErrorCount();
  }
}

/// Accumulator for a count associated with a specific source location.
///
/// The accumulator stores whether the source location is [local] and what its
/// [id] for efficiency encoding terse JSON payloads describing counts.
class _LocationCount {
  _LocationCount({required this.location, required this.id, required this.local});

  /// Location id.
  final int id;

  /// Whether the location is local to the current project.
  final bool local;

  final _Location location;

  int get count => _count;
  int _count = 0;

  /// Reset the count.
  void reset() {
    _count = 0;
  }

  /// Increment the count.
  void increment() {
    _count++;
  }
}

/// A stat tracker that aggregates a performance metric for [Element] objects at
/// the granularity of creation locations in source code.
///
/// This class is optimized to minimize the size of the JSON payloads describing
/// the aggregate statistics, for stable memory usage, and low CPU usage at the
/// expense of somewhat higher overall memory usage. Stable memory usage is more
/// important than peak memory usage to avoid the false impression that the
/// user's app is leaking memory each frame.
///
/// The number of unique widget creation locations tends to be at most in the
/// low thousands for regular flutter apps so the peak memory usage for this
/// class is not an issue.
class _ElementLocationStatsTracker {
  // All known creation location tracked.
  //
  // This could also be stored as a `Map<int, _LocationCount>` but this
  // representation is more efficient as all location ids from 0 to n are
  // typically present.
  //
  // All logic in this class assumes that if `_stats[i]` is not null
  // `_stats[i].id` equals `i`.
  final List<_LocationCount?> _stats = <_LocationCount?>[];

  /// Locations with a non-zero count.
  final List<_LocationCount> active = <_LocationCount>[];

  /// Locations that were added since stats were last exported.
  ///
  /// Only locations local to the current project are included as a performance
  /// optimization.
  final List<_LocationCount> newLocations = <_LocationCount>[];

  /// Increments the count associated with the creation location of [element] if
  /// the creation location is local to the current project.
  void add(Element element) {
    final Object widget = element.widget;
    if (widget is! _HasCreationLocation) {
      return;
    }
    final _HasCreationLocation creationLocationSource = widget;
    final _Location? location = creationLocationSource._location;
    if (location == null) {
      return;
    }
    final int id = _toLocationId(location);

    _LocationCount entry;
    if (id >= _stats.length || _stats[id] == null) {
      // After the first frame, almost all creation ids will already be in
      // _stats so this slow path will rarely be hit.
      while (id >= _stats.length) {
        _stats.add(null);
      }
      entry = _LocationCount(
        location: location,
        id: id,
        local: WidgetInspectorService.instance._isLocalCreationLocation(location.file),
      );
      if (entry.local) {
        newLocations.add(entry);
      }
      _stats[id] = entry;
    } else {
      entry = _stats[id]!;
    }

    // We could in the future add an option to track stats for all widgets but
    // that would significantly increase the size of the events posted using
    // [developer.postEvent] and current use cases for this feature focus on
    // helping users find problems with their widgets not the platform
    // widgets.
    if (entry.local) {
      if (entry.count == 0) {
        active.add(entry);
      }
      entry.increment();
    }
  }

  /// Clear all aggregated statistics.
  void resetCounts() {
    // We chose to only reset the active counts instead of clearing all data
    // to reduce the number memory allocations performed after the first frame.
    // Once an app has warmed up, location stats tracking should not
    // trigger significant additional memory allocations. Avoiding memory
    // allocations is important to minimize the impact this class has on cpu
    // and memory performance of the running app.
    for (final _LocationCount entry in active) {
      entry.reset();
    }
    active.clear();
  }

  /// Exports the current counts and then resets the stats to prepare to track
  /// the next frame of data.
  Map<String, dynamic> exportToJson(Duration startTime, {required int frameNumber}) {
    final List<int> events = List<int>.filled(active.length * 2, 0);
    int j = 0;
    for (final _LocationCount stat in active) {
      events[j++] = stat.id;
      events[j++] = stat.count;
    }

    final Map<String, dynamic> json = <String, dynamic>{
      'startTime': startTime.inMicroseconds,
      'frameNumber': frameNumber,
      'events': events,
    };

    // Encode the new locations using the older encoding.
    if (newLocations.isNotEmpty) {
      // Add all newly used location ids to the JSON.
      final Map<String, List<int>> locationsJson = <String, List<int>>{};
      for (final _LocationCount entry in newLocations) {
        final _Location location = entry.location;
        final List<int> jsonForFile = locationsJson.putIfAbsent(location.file, () => <int>[]);
        jsonForFile
          ..add(entry.id)
          ..add(location.line)
          ..add(location.column);
      }
      json['newLocations'] = locationsJson;
    }

    // Encode the new locations using the newer encoding (as of v2.4.0).
    if (newLocations.isNotEmpty) {
      final Map<String, Map<String, List<Object?>>> fileLocationsMap =
          <String, Map<String, List<Object?>>>{};
      for (final _LocationCount entry in newLocations) {
        final _Location location = entry.location;
        final Map<String, List<Object?>> locations = fileLocationsMap.putIfAbsent(
          location.file,
          () => <String, List<Object?>>{
            'ids': <int>[],
            'lines': <int>[],
            'columns': <int>[],
            'names': <String?>[],
          },
        );

        locations['ids']!.add(entry.id);
        locations['lines']!.add(location.line);
        locations['columns']!.add(location.column);
        locations['names']!.add(location.name);
      }
      json['locations'] = fileLocationsMap;
    }

    resetCounts();
    newLocations.clear();
    return json;
  }
}

class _WidgetForTypeTests extends Widget {
  const _WidgetForTypeTests();

  @override
  Element createElement() => throw UnimplementedError();
}

/// A widget that enables inspecting the child widget's structure.
///
/// Select a location on your device or emulator and view what widgets and
/// render object that best matches the location. An outline of the selected
/// widget and terse summary information is shown on device with detailed
/// information is shown in the observatory or in IntelliJ when using the
/// Flutter Plugin.
///
/// The inspector has a select mode and a view mode.
///
/// In the select mode, tapping the device selects the widget that best matches
/// the location of the touch and switches to view mode. Dragging a finger on
/// the device selects the widget under the drag location but does not switch
/// modes. Touching the very edge of the bounding box of a widget triggers
/// selecting the widget even if another widget that also overlaps that
/// location would otherwise have priority.
///
/// In the view mode, the previously selected widget is outlined, however,
/// touching the device has the same effect it would have if the inspector
/// wasn't present. This allows interacting with the application and viewing how
/// the selected widget changes position. Clicking on the select icon in the
/// bottom left corner of the application switches back to select mode.
class WidgetInspector extends StatefulWidget {
  /// Creates a widget that enables inspection for the child.
  const WidgetInspector({
    super.key,
    required this.child,
    required this.exitWidgetSelectionButtonBuilder,
    required this.moveExitWidgetSelectionButtonBuilder,
  });

  /// The widget that is being inspected.
  final Widget child;

  /// A builder that is called to create the exit select-mode button.
  ///
  /// The `onPressed` callback and key passed as arguments to the builder should
  /// be hooked up to the returned widget.
  final ExitWidgetSelectionButtonBuilder? exitWidgetSelectionButtonBuilder;

  /// A builder that is called to create the button that moves the exit select-
  /// mode button to the right or left.
  ///
  /// The `onPressed` callback passed as an argument to the builder should be
  /// hooked up to the returned widget.
  ///
  /// The button UI should respond to the `leftAligned` argument.
  final MoveExitWidgetSelectionButtonBuilder? moveExitWidgetSelectionButtonBuilder;

  @override
  State<WidgetInspector> createState() => _WidgetInspectorState();
}

class _WidgetInspectorState extends State<WidgetInspector> with WidgetsBindingObserver {
  _WidgetInspectorState();

  Offset? _lastPointerLocation;

  late InspectorSelection selection;

  late bool isSelectMode;

  final GlobalKey _ignorePointerKey = GlobalKey();

  /// Distance from the edge of the bounding box for an element to consider
  /// as selecting the edge of the bounding box.
  static const double _edgeHitMargin = 2.0;

  @override
  void initState() {
    super.initState();

    WidgetInspectorService.instance.selection.addListener(_selectionInformationChanged);
    WidgetsBinding.instance.debugShowWidgetInspectorOverrideNotifier.addListener(
      _selectionInformationChanged,
    );
    selection = WidgetInspectorService.instance.selection;
    isSelectMode = WidgetsBinding.instance.debugShowWidgetInspectorOverride;
  }

  @override
  void dispose() {
    WidgetInspectorService.instance.selection.removeListener(_selectionInformationChanged);
    WidgetsBinding.instance.debugShowWidgetInspectorOverrideNotifier.removeListener(
      _selectionInformationChanged,
    );
    super.dispose();
  }

  void _selectionInformationChanged() => setState(() {
    selection = WidgetInspectorService.instance.selection;
    isSelectMode = WidgetsBinding.instance.debugShowWidgetInspectorOverride;
  });

  bool _hitTestHelper(
    List<RenderObject> hits,
    List<RenderObject> edgeHits,
    Offset position,
    RenderObject object,
    Matrix4 transform,
  ) {
    bool hit = false;
    final Matrix4? inverse = Matrix4.tryInvert(transform);
    if (inverse == null) {
      // We cannot invert the transform. That means the object doesn't appear on
      // screen and cannot be hit.
      return false;
    }
    final Offset localPosition = MatrixUtils.transformPoint(inverse, position);

    final List<DiagnosticsNode> children = object.debugDescribeChildren();
    for (int i = children.length - 1; i >= 0; i -= 1) {
      final DiagnosticsNode diagnostics = children[i];
      if (diagnostics.style == DiagnosticsTreeStyle.offstage ||
          diagnostics.value is! RenderObject) {
        continue;
      }
      final RenderObject child = diagnostics.value! as RenderObject;
      final Rect? paintClip = object.describeApproximatePaintClip(child);
      if (paintClip != null && !paintClip.contains(localPosition)) {
        continue;
      }

      final Matrix4 childTransform = transform.clone();
      object.applyPaintTransform(child, childTransform);
      if (_hitTestHelper(hits, edgeHits, position, child, childTransform)) {
        hit = true;
      }
    }

    final Rect bounds = object.semanticBounds;
    if (bounds.contains(localPosition)) {
      hit = true;
      // Hits that occur on the edge of the bounding box of an object are
      // given priority to provide a way to select objects that would
      // otherwise be hard to select.
      if (!bounds.deflate(_edgeHitMargin).contains(localPosition)) {
        edgeHits.add(object);
      }
    }
    if (hit) {
      hits.add(object);
    }
    return hit;
  }

  /// Returns the list of render objects located at the given position ordered
  /// by priority.
  ///
  /// All render objects that are not offstage that match the location are
  /// included in the list of matches. Priority is given to matches that occur
  /// on the edge of a render object's bounding box and to matches found by
  /// [RenderBox.hitTest].
  List<RenderObject> hitTest(Offset position, RenderObject root) {
    final List<RenderObject> regularHits = <RenderObject>[];
    final List<RenderObject> edgeHits = <RenderObject>[];

    _hitTestHelper(regularHits, edgeHits, position, root, root.getTransformTo(null));
    // Order matches by the size of the hit area.
    double area(RenderObject object) {
      final Size size = object.semanticBounds.size;
      return size.width * size.height;
    }

    regularHits.sort((RenderObject a, RenderObject b) => area(a).compareTo(area(b)));
    final Set<RenderObject> hits = <RenderObject>{...edgeHits, ...regularHits};
    return hits.toList();
  }

  void _inspectAt(Offset position) {
    if (!isSelectMode) {
      return;
    }

    final RenderIgnorePointer ignorePointer =
        _ignorePointerKey.currentContext!.findRenderObject()! as RenderIgnorePointer;
    final RenderObject userRender = ignorePointer.child!;
    final List<RenderObject> selected = hitTest(position, userRender);

    selection.candidates = selected;
  }

  void _handlePanDown(DragDownDetails event) {
    _lastPointerLocation = event.globalPosition;
    _inspectAt(event.globalPosition);
  }

  void _handlePanUpdate(DragUpdateDetails event) {
    _lastPointerLocation = event.globalPosition;
    _inspectAt(event.globalPosition);
  }

  void _handlePanEnd(DragEndDetails details) {
    // If the pan ends on the edge of the window assume that it indicates the
    // pointer is being dragged off the edge of the display not a regular touch
    // on the edge of the display. If the pointer is being dragged off the edge
    // of the display we do not want to select anything. A user can still select
    // a widget that is only at the exact screen margin by tapping.
    final ui.FlutterView view = View.of(context);
    final Rect bounds = (Offset.zero & (view.physicalSize / view.devicePixelRatio)).deflate(
      _kOffScreenMargin,
    );
    if (!bounds.contains(_lastPointerLocation!)) {
      selection.clear();
    } else {
      // Otherwise notify DevTools of the current selection.
      WidgetInspectorService.instance._sendInspectEvent(selection.current);
    }
  }

  void _handleTap() {
    if (!isSelectMode) {
      return;
    }
    if (_lastPointerLocation != null) {
      _inspectAt(_lastPointerLocation!);
      WidgetInspectorService.instance._sendInspectEvent(selection.current);
    }
  }

  @override
  Widget build(BuildContext context) {
    // Be careful changing this build method. The _InspectorOverlayLayer
    // assumes the root RenderObject for the WidgetInspector will be
    // a RenderStack containing a _RenderInspectorOverlay as a child.
    return Stack(
      children: <Widget>[
        GestureDetector(
          onTap: _handleTap,
          onPanDown: _handlePanDown,
          onPanEnd: _handlePanEnd,
          onPanUpdate: _handlePanUpdate,
          behavior: HitTestBehavior.opaque,
          excludeFromSemantics: true,
          child: IgnorePointer(ignoring: isSelectMode, key: _ignorePointerKey, child: widget.child),
        ),
        _InspectorOverlay(selection: selection),
        if (isSelectMode && widget.exitWidgetSelectionButtonBuilder != null)
          _ExitWidgetSelectionButtonGroup(
            exitWidgetSelectionButtonBuilder: widget.exitWidgetSelectionButtonBuilder!,
            moveExitWidgetSelectionButtonBuilder: widget.moveExitWidgetSelectionButtonBuilder,
          ),
      ],
    );
  }
}

/// Mutable selection state of the inspector.
class InspectorSelection with ChangeNotifier {
  /// Creates an instance of [InspectorSelection].
  InspectorSelection() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// Render objects that are candidates to be selected.
  ///
  /// Tools may wish to iterate through the list of candidates.
  List<RenderObject> get candidates => _candidates;
  List<RenderObject> _candidates = <RenderObject>[];
  set candidates(List<RenderObject> value) {
    _candidates = value;
    _index = 0;
    _computeCurrent();
  }

  /// Index within the list of candidates that is currently selected.
  int get index => _index;
  int _index = 0;
  set index(int value) {
    _index = value;
    _computeCurrent();
  }

  /// Set the selection to empty.
  void clear() {
    _candidates = <RenderObject>[];
    _index = 0;
    _computeCurrent();
  }

  /// Selected render object typically from the [candidates] list.
  ///
  /// Setting [candidates] or calling [clear] resets the selection.
  ///
  /// Returns null if the selection is invalid.
  RenderObject? get current => active ? _current : null;

  RenderObject? _current;
  set current(RenderObject? value) {
    if (_current != value) {
      _current = value;
      _currentElement = (value?.debugCreator as DebugCreator?)?.element;
      notifyListeners();
    }
  }

  /// Selected [Element] consistent with the [current] selected [RenderObject].
  ///
  /// Setting [candidates] or calling [clear] resets the selection.
  ///
  /// Returns null if the selection is invalid.
  Element? get currentElement {
    return _currentElement?.debugIsDefunct ?? true ? null : _currentElement;
  }

  Element? _currentElement;
  set currentElement(Element? element) {
    if (element?.debugIsDefunct ?? false) {
      _currentElement = null;
      _current = null;
      notifyListeners();
      return;
    }
    if (currentElement != element) {
      _currentElement = element;
      _current = element!.findRenderObject();
      notifyListeners();
    }
  }

  void _computeCurrent() {
    if (_index < candidates.length) {
      _current = candidates[index];
      _currentElement = (_current?.debugCreator as DebugCreator?)?.element;
      notifyListeners();
    } else {
      _current = null;
      _currentElement = null;
      notifyListeners();
    }
  }

  /// Whether the selected render object is attached to the tree or has gone
  /// out of scope.
  bool get active => _current != null && _current!.attached;
}

class _InspectorOverlay extends LeafRenderObjectWidget {
  const _InspectorOverlay({required this.selection});

  final InspectorSelection selection;

  @override
  _RenderInspectorOverlay createRenderObject(BuildContext context) {
    return _RenderInspectorOverlay(selection: selection);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderInspectorOverlay renderObject) {
    renderObject.selection = selection;
  }
}

class _RenderInspectorOverlay extends RenderBox {
  _RenderInspectorOverlay({required InspectorSelection selection}) : _selection = selection;

  InspectorSelection get selection => _selection;
  InspectorSelection _selection;
  set selection(InspectorSelection value) {
    if (value != _selection) {
      _selection = value;
    }
    markNeedsPaint();
  }

  @override
  bool get sizedByParent => true;

  @override
  bool get alwaysNeedsCompositing => true;

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.constrain(Size.infinite);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    assert(needsCompositing);
    context.addLayer(
      _InspectorOverlayLayer(
        overlayRect: Rect.fromLTWH(offset.dx, offset.dy, size.width, size.height),
        selection: selection,
        rootRenderObject: parent is RenderObject ? parent! : null,
      ),
    );
  }
}

@immutable
class _TransformedRect {
  _TransformedRect(RenderObject object, RenderObject? ancestor)
    : rect = object.semanticBounds,
      transform = object.getTransformTo(ancestor);

  final Rect rect;
  final Matrix4 transform;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _TransformedRect && other.rect == rect && other.transform == transform;
  }

  @override
  int get hashCode => Object.hash(rect, transform);
}

/// State describing how the inspector overlay should be rendered.
///
/// The equality operator can be used to determine whether the overlay needs to
/// be rendered again.
@immutable
class _InspectorOverlayRenderState {
  const _InspectorOverlayRenderState({
    required this.overlayRect,
    required this.selected,
    required this.candidates,
    required this.tooltip,
    required this.textDirection,
  });

  final Rect overlayRect;
  final _TransformedRect selected;
  final List<_TransformedRect> candidates;
  final String tooltip;
  final TextDirection textDirection;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _InspectorOverlayRenderState &&
        other.overlayRect == overlayRect &&
        other.selected == selected &&
        listEquals<_TransformedRect>(other.candidates, candidates) &&
        other.tooltip == tooltip;
  }

  @override
  int get hashCode => Object.hash(overlayRect, selected, Object.hashAll(candidates), tooltip);
}

const int _kMaxTooltipLines = 5;
const Color _kTooltipBackgroundColor = Color.fromARGB(230, 60, 60, 60);
const Color _kHighlightedRenderObjectFillColor = Color.fromARGB(128, 128, 128, 255);
const Color _kHighlightedRenderObjectBorderColor = Color.fromARGB(128, 64, 64, 128);

/// A layer that outlines the selected [RenderObject] and candidate render
/// objects that also match the last pointer location.
///
/// This approach is horrific for performance and is only used here because this
/// is limited to debug mode. Do not duplicate the logic in production code.
class _InspectorOverlayLayer extends Layer {
  /// Creates a layer that displays the inspector overlay.
  _InspectorOverlayLayer({
    required this.overlayRect,
    required this.selection,
    required this.rootRenderObject,
  }) {
    bool inDebugMode = false;
    assert(() {
      inDebugMode = true;
      return true;
    }());
    if (!inDebugMode) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary(
          'The inspector should never be used in production mode due to the '
          'negative performance impact.',
        ),
      ]);
    }
  }

  InspectorSelection selection;

  /// The rectangle in this layer's coordinate system that the overlay should
  /// occupy.
  ///
  /// The scene must be explicitly recomposited after this property is changed
  /// (as described at [Layer]).
  final Rect overlayRect;

  /// Widget inspector root render object. The selection overlay will be painted
  /// with transforms relative to this render object.
  final RenderObject? rootRenderObject;

  _InspectorOverlayRenderState? _lastState;

  /// Picture generated from _lastState.
  ui.Picture? _picture;

  TextPainter? _textPainter;
  double? _textPainterMaxWidth;

  @override
  void dispose() {
    _textPainter?.dispose();
    _textPainter = null;
    _picture?.dispose();
    super.dispose();
  }

  @override
  void addToScene(ui.SceneBuilder builder) {
    if (!selection.active) {
      return;
    }

    final RenderObject selected = selection.current!;

    if (!_isInInspectorRenderObjectTree(selected)) {
      return;
    }

    final List<_TransformedRect> candidates = <_TransformedRect>[];
    for (final RenderObject candidate in selection.candidates) {
      if (candidate == selected ||
          !candidate.attached ||
          !_isInInspectorRenderObjectTree(candidate)) {
        continue;
      }
      candidates.add(_TransformedRect(candidate, rootRenderObject));
    }
    final _TransformedRect selectedRect = _TransformedRect(selected, rootRenderObject);
    final String widgetName = selection.currentElement!.toStringShort();
    final String width = selectedRect.rect.width.toStringAsFixed(1);
    final String height = selectedRect.rect.height.toStringAsFixed(1);

    final _InspectorOverlayRenderState state = _InspectorOverlayRenderState(
      overlayRect: overlayRect,
      selected: selectedRect,
      tooltip: '$widgetName ($width x $height)',
      textDirection: TextDirection.ltr,
      candidates: candidates,
    );

    if (state != _lastState) {
      _lastState = state;
      _picture?.dispose();
      _picture = _buildPicture(state);
    }
    builder.addPicture(Offset.zero, _picture!);
  }

  ui.Picture _buildPicture(_InspectorOverlayRenderState state) {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final Canvas canvas = Canvas(recorder, state.overlayRect);
    final Size size = state.overlayRect.size;
    // The overlay rect could have an offset if the widget inspector does
    // not take all the screen.
    canvas.translate(state.overlayRect.left, state.overlayRect.top);

    final Paint fillPaint =
        Paint()
          ..style = PaintingStyle.fill
          ..color = _kHighlightedRenderObjectFillColor;

    final Paint borderPaint =
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.0
          ..color = _kHighlightedRenderObjectBorderColor;

    // Highlight the selected renderObject.
    final Rect selectedPaintRect = state.selected.rect.deflate(0.5);
    canvas
      ..save()
      ..transform(state.selected.transform.storage)
      ..drawRect(selectedPaintRect, fillPaint)
      ..drawRect(selectedPaintRect, borderPaint)
      ..restore();

    // Show all other candidate possibly selected elements. This helps selecting
    // render objects by selecting the edge of the bounding box shows all
    // elements the user could toggle the selection between.
    for (final _TransformedRect transformedRect in state.candidates) {
      canvas
        ..save()
        ..transform(transformedRect.transform.storage)
        ..drawRect(transformedRect.rect.deflate(0.5), borderPaint)
        ..restore();
    }

    final Rect targetRect = MatrixUtils.transformRect(
      state.selected.transform,
      state.selected.rect,
    );
    if (!targetRect.hasNaN) {
      final Offset target = Offset(targetRect.left, targetRect.center.dy);
      const double offsetFromWidget = 9.0;
      final double verticalOffset = (targetRect.height) / 2 + offsetFromWidget;

      _paintDescription(
        canvas,
        state.tooltip,
        state.textDirection,
        target,
        verticalOffset,
        size,
        targetRect,
      );
    }
    // TODO(jacobr): provide an option to perform a debug paint of just the
    // selected widget.
    return recorder.endRecording();
  }

  void _paintDescription(
    Canvas canvas,
    String message,
    TextDirection textDirection,
    Offset target,
    double verticalOffset,
    Size size,
    Rect targetRect,
  ) {
    canvas.save();
    final double maxWidth = math.max(size.width - 2 * (_kScreenEdgeMargin + _kTooltipPadding), 0);
    final TextSpan? textSpan = _textPainter?.text as TextSpan?;
    if (_textPainter == null || textSpan!.text != message || _textPainterMaxWidth != maxWidth) {
      _textPainterMaxWidth = maxWidth;
      _textPainter?.dispose();
      _textPainter =
          TextPainter()
            ..maxLines = _kMaxTooltipLines
            ..ellipsis = '...'
            ..text = TextSpan(style: _messageStyle, text: message)
            ..textDirection = textDirection
            ..layout(maxWidth: maxWidth);
    }

    final Size tooltipSize =
        _textPainter!.size + const Offset(_kTooltipPadding * 2, _kTooltipPadding * 2);
    final Offset tipOffset = positionDependentBox(
      size: size,
      childSize: tooltipSize,
      target: target,
      verticalOffset: verticalOffset,
      preferBelow: false,
    );

    final Paint tooltipBackground =
        Paint()
          ..style = PaintingStyle.fill
          ..color = _kTooltipBackgroundColor;
    canvas.drawRect(
      Rect.fromPoints(tipOffset, tipOffset.translate(tooltipSize.width, tooltipSize.height)),
      tooltipBackground,
    );

    double wedgeY = tipOffset.dy;
    final bool tooltipBelow = tipOffset.dy > target.dy;
    if (!tooltipBelow) {
      wedgeY += tooltipSize.height;
    }

    const double wedgeSize = _kTooltipPadding * 2;
    double wedgeX = math.max(tipOffset.dx, target.dx) + wedgeSize * 2;
    wedgeX = math.min(wedgeX, tipOffset.dx + tooltipSize.width - wedgeSize * 2);
    final List<Offset> wedge = <Offset>[
      Offset(wedgeX - wedgeSize, wedgeY),
      Offset(wedgeX + wedgeSize, wedgeY),
      Offset(wedgeX, wedgeY + (tooltipBelow ? -wedgeSize : wedgeSize)),
    ];
    canvas.drawPath(Path()..addPolygon(wedge, true), tooltipBackground);
    _textPainter!.paint(canvas, tipOffset + const Offset(_kTooltipPadding, _kTooltipPadding));
    canvas.restore();
  }

  @override
  @protected
  bool findAnnotations<S extends Object>(
    AnnotationResult<S> result,
    Offset localPosition, {
    bool? onlyFirst,
  }) {
    return false;
  }

  /// Return whether or not a render object belongs to this inspector widget
  /// tree.
  /// The inspector selection is static, so if there are multiple inspector
  /// overlays in the same app (i.e. an storyboard), a selected or candidate
  /// render object may not belong to this tree.
  bool _isInInspectorRenderObjectTree(RenderObject child) {
    RenderObject? current = child.parent;
    while (current != null) {
      // We found the widget inspector render object.
      if (current is RenderStack &&
          current.getChildrenAsList().any((RenderBox child) => child is _RenderInspectorOverlay)) {
        return rootRenderObject == current;
      }
      current = current.parent;
    }
    return false;
  }
}

const double _kScreenEdgeMargin = 10.0;
const double _kTooltipPadding = 5.0;

/// Interpret pointer up events within with this margin as indicating the
/// pointer is moving off the device.
const double _kOffScreenMargin = 1.0;

const TextStyle _messageStyle = TextStyle(color: Color(0xFFFFFFFF), fontSize: 10.0, height: 1.2);

class _ExitWidgetSelectionButtonGroup extends StatefulWidget {
  const _ExitWidgetSelectionButtonGroup({
    required this.exitWidgetSelectionButtonBuilder,
    required this.moveExitWidgetSelectionButtonBuilder,
  });

  final ExitWidgetSelectionButtonBuilder exitWidgetSelectionButtonBuilder;
  final MoveExitWidgetSelectionButtonBuilder? moveExitWidgetSelectionButtonBuilder;

  @override
  State<_ExitWidgetSelectionButtonGroup> createState() => _ExitWidgetSelectionButtonGroupState();
}

class _ExitWidgetSelectionButtonGroupState extends State<_ExitWidgetSelectionButtonGroup> {
  static const double _kExitWidgetSelectionButtonPadding = 4.0;
  static const double _kExitWidgetSelectionButtonMargin = 10.0;

  final GlobalKey _exitWidgetSelectionButtonKey = GlobalKey(
    debugLabel: 'Exit Widget Selection button',
  );

  String? _tooltipMessage;

  bool _leftAligned = true;

  @override
  Widget build(BuildContext context) {
    final Widget? moveExitWidgetSelectionButton =
        widget.moveExitWidgetSelectionButtonBuilder != null
            ? Padding(
              padding: EdgeInsets.only(
                left: _leftAligned ? _kExitWidgetSelectionButtonPadding : 0.0,
                right: _leftAligned ? 0.0 : _kExitWidgetSelectionButtonPadding,
              ),
              child: _TooltipGestureDetector(
                button: widget.moveExitWidgetSelectionButtonBuilder!(
                  context,
                  onPressed: () {
                    _changeButtonGroupAlignment();
                    _onTooltipHidden();
                  },
                  isLeftAligned: _leftAligned,
                ),
                onTooltipVisible: () {
                  _changeTooltipMessage('Move to the ${_leftAligned ? 'right' : 'left'}');
                },
                onTooltipHidden: _onTooltipHidden,
              ),
            )
            : null;

    final Widget buttonGroup = Stack(
      alignment: AlignmentDirectional.topCenter,
      children: <Widget>[
        CustomPaint(
          painter: _ExitWidgetSelectionTooltipPainter(
            tooltipMessage: _tooltipMessage,
            buttonKey: _exitWidgetSelectionButtonKey,
            isLeftAligned: _leftAligned,
          ),
        ),
        Row(
          children: <Widget>[
            if (!_leftAligned && moveExitWidgetSelectionButton != null)
              moveExitWidgetSelectionButton,
            _TooltipGestureDetector(
              button: widget.exitWidgetSelectionButtonBuilder(
                context,
                onPressed: _exitWidgetSelectionMode,
                key: _exitWidgetSelectionButtonKey,
              ),
              onTooltipVisible: () {
                _changeTooltipMessage('Exit Select Widget mode');
              },
              onTooltipHidden: _onTooltipHidden,
            ),
            if (_leftAligned && moveExitWidgetSelectionButton != null)
              moveExitWidgetSelectionButton,
          ],
        ),
      ],
    );

    return Positioned(
      left: _leftAligned ? _kExitWidgetSelectionButtonMargin : null,
      right: _leftAligned ? null : _kExitWidgetSelectionButtonMargin,
      bottom: _kExitWidgetSelectionButtonMargin,
      child: buttonGroup,
    );
  }

  void _exitWidgetSelectionMode() {
    WidgetInspectorService.instance._changeWidgetSelectionMode(false);
  }

  void _changeButtonGroupAlignment() {
    if (mounted) {
      setState(() {
        _leftAligned = !_leftAligned;
      });
    }
  }

  void _onTooltipHidden() {
    _changeTooltipMessage(null);
  }

  void _changeTooltipMessage(String? message) {
    if (mounted) {
      setState(() {
        _tooltipMessage = message;
      });
    }
  }
}

class _TooltipGestureDetector extends StatefulWidget {
  const _TooltipGestureDetector({
    required this.button,
    required this.onTooltipVisible,
    required this.onTooltipHidden,
  });

  final Widget button;
  final void Function() onTooltipVisible;
  final void Function() onTooltipHidden;

  static const Duration _tooltipShownOnLongPressDuration = Duration(milliseconds: 1500);
  static const Duration _tooltipDelayDuration = Duration(milliseconds: 100);

  @override
  State<_TooltipGestureDetector> createState() => _TooltipGestureDetectorState();
}

class _TooltipGestureDetectorState extends State<_TooltipGestureDetector> {
  Timer? _tooltipVisibleTimer;
  Timer? _tooltipHiddenTimer;

  @override
  void dispose() {
    _tooltipVisibleTimer?.cancel();
    _tooltipVisibleTimer = null;
    _tooltipHiddenTimer?.cancel();
    _tooltipHiddenTimer = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      alignment: AlignmentDirectional.topCenter,
      children: <Widget>[
        GestureDetector(
          onLongPress: () {
            _tooltipVisibleAfter(_TooltipGestureDetector._tooltipDelayDuration);
            _tooltipHiddenAfter(
              _TooltipGestureDetector._tooltipShownOnLongPressDuration +
                  _TooltipGestureDetector._tooltipDelayDuration,
            );
          },
          child: MouseRegion(
            onEnter: (_) {
              _tooltipVisibleAfter(_TooltipGestureDetector._tooltipDelayDuration);
            },
            onExit: (_) {
              _tooltipHiddenAfter(_TooltipGestureDetector._tooltipDelayDuration);
            },
            child: widget.button,
          ),
        ),
      ],
    );
  }

  void _tooltipVisibleAfter(Duration duration) {
    _tooltipVisibilityChangedAfter(duration, isVisible: true);
  }

  void _tooltipHiddenAfter(Duration duration) {
    _tooltipVisibilityChangedAfter(duration, isVisible: false);
  }

  void _tooltipVisibilityChangedAfter(Duration duration, {required bool isVisible}) {
    final Timer? timer = isVisible ? _tooltipVisibleTimer : _tooltipHiddenTimer;
    if (timer?.isActive ?? false) {
      timer!.cancel();
    }

    if (isVisible) {
      _tooltipVisibleTimer = Timer(duration, () {
        widget.onTooltipVisible();
      });
    } else {
      _tooltipHiddenTimer = Timer(duration, () {
        widget.onTooltipHidden();
      });
    }
  }
}

class _ExitWidgetSelectionTooltipPainter extends CustomPainter {
  _ExitWidgetSelectionTooltipPainter({
    required this.tooltipMessage,
    required this.buttonKey,
    required this.isLeftAligned,
  });

  final String? tooltipMessage;
  final GlobalKey buttonKey;
  final bool isLeftAligned;

  @override
  void paint(Canvas canvas, Size size) {
    // Do not paint the tooltip if it is currently hidden.
    final bool isVisible = tooltipMessage != null;
    if (!isVisible) {
      return;
    }

    // Do not paint the tooltip if the exit select mode button is not rendered.
    final RenderObject? buttonRenderObject = buttonKey.currentContext?.findRenderObject();
    if (buttonRenderObject == null) {
      return;
    }

    // Define tooltip appearance.
    const double tooltipPadding = 4.0;
    const double tooltipSpacing = 6.0;

    final TextPainter tooltipTextPainter =
        TextPainter()
          ..maxLines = 1
          ..ellipsis = '...'
          ..text = TextSpan(text: tooltipMessage, style: _messageStyle)
          ..textDirection = TextDirection.ltr
          ..layout();

    final Paint tooltipPaint =
        Paint()
          ..style = PaintingStyle.fill
          ..color = _kTooltipBackgroundColor;

    // Determine tooltip position.
    final double buttonWidth = buttonRenderObject.paintBounds.width;
    final Size textSize = tooltipTextPainter.size;
    final double textWidth = textSize.width;
    final double textHeight = textSize.height;
    final double tooltipWidth = textWidth + (tooltipPadding * 2);
    final double tooltipHeight = textHeight + (tooltipPadding * 2);

    final double tooltipXOffset =
        isLeftAligned ? 0 - buttonWidth : 0 - (tooltipWidth - buttonWidth);
    final double tooltipYOffset = 0 - tooltipHeight - tooltipSpacing;

    // Draw tooltip background.
    canvas.drawRect(
      Rect.fromLTWH(tooltipXOffset, tooltipYOffset, tooltipWidth, tooltipHeight),
      tooltipPaint,
    );

    // Draw tooltip text.
    tooltipTextPainter.paint(
      canvas,
      Offset(tooltipXOffset + tooltipPadding, tooltipYOffset + tooltipPadding),
    );
  }

  @override
  bool shouldRepaint(covariant _ExitWidgetSelectionTooltipPainter oldDelegate) {
    return tooltipMessage != oldDelegate.tooltipMessage;
  }
}

/// Interface for classes that track the source code location the their
/// constructor was called from.
///
/// {@macro flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
// ignore: unused_element
abstract class _HasCreationLocation {
  _Location? get _location;
}

/// A tuple with file, line, and column number, for displaying human-readable
/// file locations.
class _Location {
  const _Location({
    required this.file,
    required this.line,
    required this.column,
    // TODO(srawlins): `unused_element_parameter` is being separated from
    // `unused_element`. Ignore both names until the separation is complete.
    // ignore: unused_element, unused_element_parameter
    this.name,
  });

  /// File path of the location.
  final String file;

  /// 1-based line number.
  final int line;

  /// 1-based column number.
  final int column;

  /// Optional name of the parameter or function at this location.
  final String? name;

  Map<String, Object?> toJsonMap() {
    return <String, Object?>{
      'file': file,
      'line': line,
      'column': column,
      if (name != null) 'name': name,
    };
  }

  @override
  String toString() => <String>[if (name != null) name!, file, '$line', '$column'].join(':');
}

bool _isDebugCreator(DiagnosticsNode node) => node is DiagnosticsDebugCreator;

/// Transformer to parse and gather information about [DiagnosticsDebugCreator].
///
/// This function will be registered to [FlutterErrorDetails.propertiesTransformers]
/// in [WidgetsBinding.initInstances].
///
/// This is meant to be called only in debug mode. In other modes, it yields an empty list.
Iterable<DiagnosticsNode> debugTransformDebugCreator(Iterable<DiagnosticsNode> properties) {
  if (!kDebugMode) {
    return <DiagnosticsNode>[];
  }
  final List<DiagnosticsNode> pending = <DiagnosticsNode>[];
  ErrorSummary? errorSummary;
  for (final DiagnosticsNode node in properties) {
    if (node is ErrorSummary) {
      errorSummary = node;
      break;
    }
  }
  bool foundStackTrace = false;
  final List<DiagnosticsNode> result = <DiagnosticsNode>[];
  for (final DiagnosticsNode node in properties) {
    if (!foundStackTrace && node is DiagnosticsStackTrace) {
      foundStackTrace = true;
    }
    if (_isDebugCreator(node)) {
      result.addAll(_parseDiagnosticsNode(node, errorSummary));
    } else {
      if (foundStackTrace) {
        pending.add(node);
      } else {
        result.add(node);
      }
    }
  }
  result.addAll(pending);
  return result;
}

/// Transform the input [DiagnosticsNode].
///
/// Return null if input [DiagnosticsNode] is not applicable.
Iterable<DiagnosticsNode> _parseDiagnosticsNode(DiagnosticsNode node, ErrorSummary? errorSummary) {
  assert(_isDebugCreator(node));
  try {
    final DebugCreator debugCreator = node.value! as DebugCreator;
    final Element element = debugCreator.element;
    return _describeRelevantUserCode(element, errorSummary);
  } catch (error, stack) {
    scheduleMicrotask(() {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: error,
          stack: stack,
          library: 'widget inspector',
          informationCollector:
              () => <DiagnosticsNode>[
                DiagnosticsNode.message(
                  'This exception was caught while trying to describe the user-relevant code of another error.',
                ),
              ],
        ),
      );
    });
    return <DiagnosticsNode>[];
  }
}

Iterable<DiagnosticsNode> _describeRelevantUserCode(Element element, ErrorSummary? errorSummary) {
  if (!WidgetInspectorService.instance.isWidgetCreationTracked()) {
    return <DiagnosticsNode>[
      ErrorDescription(
        'Widget creation tracking is currently disabled. Enabling '
        'it enables improved error messages. It can be enabled by passing '
        '`--track-widget-creation` to `flutter run` or `flutter test`.',
      ),
      ErrorSpacer(),
    ];
  }

  bool isOverflowError() {
    if (errorSummary != null && errorSummary.value.isNotEmpty) {
      final Object summary = errorSummary.value.first;
      if (summary is String && summary.startsWith('A RenderFlex overflowed by')) {
        return true;
      }
    }
    return false;
  }

  final List<DiagnosticsNode> nodes = <DiagnosticsNode>[];
  bool processElement(Element target) {
    // TODO(chunhtai): should print out all the widgets that are about to cross
    // package boundaries.
    if (debugIsLocalCreationLocation(target)) {
      DiagnosticsNode? devToolsDiagnostic;

      // TODO(kenz): once the inspector is better at dealing with broken trees,
      // we can enable deep links for more errors than just RenderFlex overflow
      // errors. See https://github.com/flutter/flutter/issues/74918.
      if (isOverflowError()) {
        final String? devToolsInspectorUri = WidgetInspectorService.instance
            ._devToolsInspectorUriForElement(target);
        if (devToolsInspectorUri != null) {
          devToolsDiagnostic = DevToolsDeepLinkProperty(
            'To inspect this widget in Flutter DevTools, visit: $devToolsInspectorUri',
            devToolsInspectorUri,
          );
        }
      }

      nodes.addAll(<DiagnosticsNode>[
        DiagnosticsBlock(
          name: 'The relevant error-causing widget was',
          children: <DiagnosticsNode>[
            ErrorDescription(
              '${target.widget.toStringShort()} ${_describeCreationLocation(target)}',
            ),
          ],
        ),
        ErrorSpacer(),
        if (devToolsDiagnostic != null) ...<DiagnosticsNode>[devToolsDiagnostic, ErrorSpacer()],
      ]);
      return false;
    }
    return true;
  }

  if (processElement(element)) {
    element.visitAncestorElements(processElement);
  }
  return nodes;
}

/// Debugging message for DevTools deep links.
///
/// The [value] for this property is a string representation of the Flutter
/// DevTools url.
class DevToolsDeepLinkProperty extends DiagnosticsProperty<String> {
  /// Creates a diagnostics property that displays a deep link to Flutter DevTools.
  ///
  /// The [value] of this property will return a map of data for the Flutter
  /// DevTools deep link, including the full `url`, the Flutter DevTools `screenId`,
  /// and the `objectId` in Flutter DevTools that this diagnostic references.
  DevToolsDeepLinkProperty(String description, String url)
    : super('', url, description: description, level: DiagnosticLevel.info);
}

/// Returns if an object is user created.
///
/// This always returns false if it is not called in debug mode.
///
/// {@macro flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
///
/// Currently is local creation locations are only available for
/// [Widget] and [Element].
bool debugIsLocalCreationLocation(Object object) {
  bool isLocal = false;
  assert(() {
    final _Location? location = _getCreationLocation(object);
    if (location != null) {
      isLocal = WidgetInspectorService.instance._isLocalCreationLocation(location.file);
    }
    return true;
  }());
  return isLocal;
}

/// Returns true if a [Widget] is user created.
///
/// This is a faster variant of `debugIsLocalCreationLocation` that is available
/// in debug and profile builds but only works for [Widget].
bool debugIsWidgetLocalCreation(Widget widget) {
  final _Location? location = _getObjectCreationLocation(widget);
  return location != null &&
      WidgetInspectorService.instance._isLocalCreationLocation(location.file);
}

/// Returns the creation location of an object in String format if one is available.
///
/// ex: "file:///path/to/main.dart:4:3"
///
/// {@macro flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
///
/// Currently creation locations are only available for [Widget] and [Element].
String? _describeCreationLocation(Object object) {
  final _Location? location = _getCreationLocation(object);
  return location?.toString();
}

_Location? _getObjectCreationLocation(Object object) {
  return object is _HasCreationLocation ? object._location : null;
}

/// Returns the creation location of an object if one is available.
///
/// {@macro flutter.widgets.WidgetInspectorService.getChildrenSummaryTree}
///
/// Currently creation locations are only available for [Widget] and [Element].
_Location? _getCreationLocation(Object? object) {
  final Object? candidate = object is Element && !object.debugIsDefunct ? object.widget : object;
  return candidate == null ? null : _getObjectCreationLocation(candidate);
}

// _Location objects are always const so we don't need to worry about the GC
// issues that are a concern for other object ids tracked by
// [WidgetInspectorService].
final Map<_Location, int> _locationToId = <_Location, int>{};
final List<_Location> _locations = <_Location>[];

int _toLocationId(_Location location) {
  int? id = _locationToId[location];
  if (id != null) {
    return id;
  }
  id = _locations.length;
  _locations.add(location);
  _locationToId[location] = id;
  return id;
}

Map<String, dynamic> _locationIdMapToJson() {
  const String idsKey = 'ids';
  const String linesKey = 'lines';
  const String columnsKey = 'columns';
  const String namesKey = 'names';

  final Map<String, Map<String, List<Object?>>> fileLocationsMap =
      <String, Map<String, List<Object?>>>{};
  for (final MapEntry<_Location, int> entry in _locationToId.entries) {
    final _Location location = entry.key;
    final Map<String, List<Object?>> locations = fileLocationsMap.putIfAbsent(
      location.file,
      () => <String, List<Object?>>{
        idsKey: <int>[],
        linesKey: <int>[],
        columnsKey: <int>[],
        namesKey: <String?>[],
      },
    );

    locations[idsKey]!.add(entry.value);
    locations[linesKey]!.add(location.line);
    locations[columnsKey]!.add(location.column);
    locations[namesKey]!.add(location.name);
  }
  return fileLocationsMap;
}

/// A delegate that configures how a hierarchy of [DiagnosticsNode]s are
/// serialized by the Flutter Inspector.
@visibleForTesting
class InspectorSerializationDelegate implements DiagnosticsSerializationDelegate {
  /// Creates an [InspectorSerializationDelegate] that serialize [DiagnosticsNode]
  /// for Flutter Inspector service.
  InspectorSerializationDelegate({
    this.groupName,
    this.summaryTree = false,
    this.maxDescendantsTruncatableNode = -1,
    this.expandPropertyValues = true,
    this.subtreeDepth = 1,
    this.includeProperties = false,
    required this.service,
    this.addAdditionalPropertiesCallback,
  });

  /// Service used by GUI tools to interact with the [WidgetInspector].
  final WidgetInspectorService service;

  /// Optional [groupName] parameter which indicates that the json should
  /// contain live object ids.
  ///
  /// Object ids returned as part of the json will remain live at least until
  /// [WidgetInspectorService.disposeGroup()] is called on [groupName].
  final String? groupName;

  /// Whether the tree should only include nodes created by the local project.
  final bool summaryTree;

  /// Maximum descendants of [DiagnosticsNode] before truncating.
  final int maxDescendantsTruncatableNode;

  @override
  final bool includeProperties;

  @override
  final int subtreeDepth;

  @override
  final bool expandPropertyValues;

  /// Callback to add additional experimental serialization properties.
  ///
  /// This callback can be used to customize the serialization of DiagnosticsNode
  /// objects for experimental features in widget inspector clients such as
  /// [Dart DevTools](https://github.com/flutter/devtools).
  final Map<String, Object>? Function(DiagnosticsNode, InspectorSerializationDelegate)?
  addAdditionalPropertiesCallback;

  final List<DiagnosticsNode> _nodesCreatedByLocalProject = <DiagnosticsNode>[];

  bool get _interactive => groupName != null;

  @override
  Map<String, Object?> additionalNodeProperties(DiagnosticsNode node, {bool fullDetails = true}) {
    final Map<String, Object?> result = <String, Object?>{};
    final Object? value = node.value;
    if (summaryTree && fullDetails) {
      result['summaryTree'] = true;
    }
    if (_interactive) {
      result['valueId'] = service.toId(value, groupName!);
    }
    final _Location? creationLocation = _getCreationLocation(value);
    if (creationLocation != null) {
      if (fullDetails) {
        result['locationId'] = _toLocationId(creationLocation);
        result['creationLocation'] = creationLocation.toJsonMap();
      }
      if (service._isLocalCreationLocation(creationLocation.file)) {
        _nodesCreatedByLocalProject.add(node);
        result['createdByLocalProject'] = true;
      }
    }
    if (addAdditionalPropertiesCallback != null) {
      result.addAll(addAdditionalPropertiesCallback!(node, this) ?? <String, Object>{});
    }
    return result;
  }

  @override
  DiagnosticsSerializationDelegate delegateForNode(DiagnosticsNode node) {
    // The tricky special case here is that when in the detailsTree,
    // we keep subtreeDepth from going down to zero until we reach nodes
    // that also exist in the summary tree. This ensures that every time
    // you expand a node in the details tree, you expand the entire subtree
    // up until you reach the next nodes shared with the summary tree.
    return summaryTree || subtreeDepth > 1 || service._shouldShowInSummaryTree(node)
        ? copyWith(subtreeDepth: subtreeDepth - 1)
        : this;
  }

  @override
  List<DiagnosticsNode> filterChildren(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    return service._filterChildren(nodes, this);
  }

  @override
  List<DiagnosticsNode> filterProperties(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    final bool createdByLocalProject = _nodesCreatedByLocalProject.contains(owner);
    return nodes.where((DiagnosticsNode node) {
      return !node.isFiltered(createdByLocalProject ? DiagnosticLevel.fine : DiagnosticLevel.info);
    }).toList();
  }

  @override
  List<DiagnosticsNode> truncateNodesList(List<DiagnosticsNode> nodes, DiagnosticsNode? owner) {
    if (maxDescendantsTruncatableNode >= 0 &&
        owner!.allowTruncate &&
        nodes.length > maxDescendantsTruncatableNode) {
      nodes = service._truncateNodes(nodes, maxDescendantsTruncatableNode);
    }
    return nodes;
  }

  @override
  DiagnosticsSerializationDelegate copyWith({
    int? subtreeDepth,
    bool? includeProperties,
    bool? expandPropertyValues,
  }) {
    return InspectorSerializationDelegate(
      groupName: groupName,
      summaryTree: summaryTree,
      maxDescendantsTruncatableNode: maxDescendantsTruncatableNode,
      expandPropertyValues: expandPropertyValues ?? this.expandPropertyValues,
      subtreeDepth: subtreeDepth ?? this.subtreeDepth,
      includeProperties: includeProperties ?? this.includeProperties,
      service: service,
      addAdditionalPropertiesCallback: addAdditionalPropertiesCallback,
    );
  }
}

@Target(<TargetKind>{TargetKind.method})
class _WidgetFactory {
  const _WidgetFactory();
}

/// Annotation which marks a function as a widget factory for the purpose of
/// widget creation tracking.
///
/// When widget creation tracking is enabled, the framework tracks the source
/// code location of the constructor call for each widget instance. This
/// information is used by the DevTools to provide an improved developer
/// experience. For example, it allows the Flutter inspector to present the
/// widget tree in a manner similar to how the UI was defined in your source
/// code.
///
/// [Widget] constructors are automatically instrumented to track the source
/// code location of constructor calls. However, there are cases where
/// a function acts as a sort of a constructor for a widget and a call to such
/// a function should be considered as the creation location for the returned
/// widget instance.
///
/// Annotating a function with this annotation marks the function as a widget
/// factory. The framework will then instrument that function in the same way
/// as it does for [Widget] constructors.
///
/// Tracking will not work correctly if the function has optional positional
/// parameters.
///
/// Currently this annotation is only supported on extension methods.
///
/// {@tool snippet}
///
/// This example shows how to use the [widgetFactory] annotation to mark an
/// extension method as a widget factory:
///
/// ```dart
/// extension PaddingModifier on Widget {
///   @widgetFactory
///   Widget padding(EdgeInsetsGeometry padding) {
///     return Padding(padding: padding, child: this);
///   }
/// }
/// ```
///
/// When using the above extension method, the framework will track the
/// creation location of the [Padding] widget instance as the source code
/// location where the `padding` extension method was called:
///
/// ```dart
/// // continuing from previous example...
/// const Text('Hello World!')
///     .padding(const EdgeInsets.all(8));
/// ```
///
/// {@end-tool}
///
/// See also:
///
/// * the documentation for [Track widget creation](https://flutter.dev/to/track-widget-creation).
// The below ignore is needed because the static type of the annotation is used
// by the CFE kernel transformer that implements the instrumentation to
// recognize the annotation.
// ignore: library_private_types_in_public_api
const _WidgetFactory widgetFactory = _WidgetFactory();

/// Does not hold keys from garbage collection.
@visibleForTesting
class WeakMap<K, V> {
  Expando<Object> _objects = Expando<Object>();

  /// Strings, numbers, booleans.
  final Map<K, V> _primitives = <K, V>{};

  bool _isPrimitive(Object? key) {
    return key == null || key is String || key is num || key is bool;
  }

  /// Returns the value for the given [key] or null if [key] is not in the map
  /// or garbage collected.
  ///
  /// Does not support records to act as keys.
  V? operator [](K key) {
    if (_isPrimitive(key)) {
      return _primitives[key];
    } else {
      return _objects[key!] as V?;
    }
  }

  /// Associates the [key] with the given [value].
  void operator []=(K key, V value) {
    if (_isPrimitive(key)) {
      _primitives[key] = value;
    } else {
      _objects[key!] = value;
    }
  }

  /// Removes the value for the given [key] from the map.
  V? remove(K key) {
    if (_isPrimitive(key)) {
      return _primitives.remove(key);
    } else {
      final V? result = _objects[key!] as V?;
      _objects[key] = null;
      return result;
    }
  }

  /// Removes all pairs from the map.
  void clear() {
    _objects = Expando<Object>();
    _primitives.clear();
  }
}
