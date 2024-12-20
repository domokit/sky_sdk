// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'box.dart';
/// @docImport 'proxy_box.dart';
/// @docImport 'view.dart';
/// @docImport 'viewport.dart';
library;

import 'dart:ui' as ui show PictureRecorder;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';

import 'binding.dart';
import 'debug.dart';
import 'layer.dart';

export 'package:flutter/foundation.dart'
    show
        DiagnosticPropertiesBuilder,
        DiagnosticsNode,
        DiagnosticsProperty,
        DoubleProperty,
        EnumProperty,
        ErrorDescription,
        ErrorHint,
        ErrorSummary,
        FlagProperty,
        FlutterError,
        InformationCollector,
        IntProperty,
        StringProperty;
export 'package:flutter/gestures.dart' show HitTestEntry, HitTestResult;
export 'package:flutter/painting.dart';

/// Base class for data associated with a [RenderObject] by its parent.
///
/// Some render objects wish to store data on their children, such as the
/// children's input parameters to the parent's layout algorithm or the
/// children's position relative to other children.
///
/// See also:
///
///  * [RenderObject.setupParentData], which [RenderObject] subclasses may
///    override to attach specific types of parent data to children.
class ParentData {
  /// Called when the RenderObject is removed from the tree.
  @protected
  @mustCallSuper
  void detach() {}

  @override
  String toString() => '<none>';
}

/// Signature for painting into a [PaintingContext].
///
/// The `offset` argument is the offset from the origin of the coordinate system
/// of the [PaintingContext.canvas] to the coordinate system of the callee.
///
/// Used by many of the methods of [PaintingContext].
typedef PaintingContextCallback = void Function(PaintingContext context, Offset offset);

/// A place to paint.
///
/// Rather than holding a canvas directly, [RenderObject]s paint using a painting
/// context. The painting context has a [Canvas], which receives the
/// individual draw operations, and also has functions for painting child
/// render objects.
///
/// When painting a child render object, the canvas held by the painting context
/// can change because the draw operations issued before and after painting the
/// child might be recorded in separate compositing layers. For this reason, do
/// not hold a reference to the canvas across operations that might paint
/// child render objects.
///
/// New [PaintingContext] objects are created automatically when using
/// [PaintingContext.repaintCompositedChild] and [pushLayer].
class PaintingContext extends ClipContext {
  /// Creates a painting context.
  ///
  /// Typically only called by [PaintingContext.repaintCompositedChild]
  /// and [pushLayer].
  @protected
  PaintingContext(this._containerLayer, this.estimatedBounds);

  final ContainerLayer _containerLayer;

  /// An estimate of the bounds within which the painting context's [canvas]
  /// will record painting commands. This can be useful for debugging.
  ///
  /// The canvas will allow painting outside these bounds.
  ///
  /// The [estimatedBounds] rectangle is in the [canvas] coordinate system.
  final Rect estimatedBounds;

  /// Repaint the given render object.
  ///
  /// The render object must be attached to a [PipelineOwner], must have a
  /// composited layer, and must be in need of painting. The render object's
  /// layer, if any, is re-used, along with any layers in the subtree that don't
  /// need to be repainted.
  ///
  /// See also:
  ///
  ///  * [RenderObject.isRepaintBoundary], which determines if a [RenderObject]
  ///    has a composited layer.
  static void repaintCompositedChild(RenderObject child, {bool debugAlsoPaintedParent = false}) {
    assert(child._needsPaint);
    _repaintCompositedChild(child, debugAlsoPaintedParent: debugAlsoPaintedParent);
  }

  static void _repaintCompositedChild(
    RenderObject child, {
    bool debugAlsoPaintedParent = false,
    PaintingContext? childContext,
  }) {
    assert(child.isRepaintBoundary);
    assert(() {
      // register the call for RepaintBoundary metrics
      child.debugRegisterRepaintBoundaryPaint(
        includedParent: debugAlsoPaintedParent,
        includedChild: true,
      );
      return true;
    }());
    OffsetLayer? childLayer = child._layerHandle.layer as OffsetLayer?;
    if (childLayer == null) {
      assert(debugAlsoPaintedParent);
      assert(child._layerHandle.layer == null);

      // Not using the `layer` setter because the setter asserts that we not
      // replace the layer for repaint boundaries. That assertion does not
      // apply here because this is exactly the place designed to create a
      // layer for repaint boundaries.
      final OffsetLayer layer = child.updateCompositedLayer(oldLayer: null);
      child._layerHandle.layer = childLayer = layer;
    } else {
      assert(debugAlsoPaintedParent || childLayer.attached);
      Offset? debugOldOffset;
      assert(() {
        debugOldOffset = childLayer!.offset;
        return true;
      }());
      childLayer.removeAllChildren();
      final OffsetLayer updatedLayer = child.updateCompositedLayer(oldLayer: childLayer);
      assert(
        identical(updatedLayer, childLayer),
        '$child created a new layer instance $updatedLayer instead of reusing the '
        'existing layer $childLayer. See the documentation of RenderObject.updateCompositedLayer '
        'for more information on how to correctly implement this method.',
      );
      assert(debugOldOffset == updatedLayer.offset);
    }
    child._needsCompositedLayerUpdate = false;

    assert(identical(childLayer, child._layerHandle.layer));
    assert(child._layerHandle.layer is OffsetLayer);
    assert(() {
      childLayer!.debugCreator = child.debugCreator ?? child.runtimeType;
      return true;
    }());

    childContext ??= PaintingContext(childLayer, child.paintBounds);
    child._paintWithContext(childContext, Offset.zero);

    // Double-check that the paint method did not replace the layer (the first
    // check is done in the [layer] setter itself).
    assert(identical(childLayer, child._layerHandle.layer));
    childContext.stopRecordingIfNeeded();
  }

  /// Update the composited layer of [child] without repainting its children.
  ///
  /// The render object must be attached to a [PipelineOwner], must have a
  /// composited layer, and must be in need of a composited layer update but
  /// not in need of painting. The render object's layer is re-used, and none
  /// of its children are repaint or their layers updated.
  ///
  /// See also:
  ///
  ///  * [RenderObject.isRepaintBoundary], which determines if a [RenderObject]
  ///    has a composited layer.
  static void updateLayerProperties(RenderObject child) {
    assert(child.isRepaintBoundary && child._wasRepaintBoundary);
    assert(!child._needsPaint);
    assert(child._layerHandle.layer != null);

    final OffsetLayer childLayer = child._layerHandle.layer! as OffsetLayer;
    Offset? debugOldOffset;
    assert(() {
      debugOldOffset = childLayer.offset;
      return true;
    }());
    final OffsetLayer updatedLayer = child.updateCompositedLayer(oldLayer: childLayer);
    assert(
      identical(updatedLayer, childLayer),
      '$child created a new layer instance $updatedLayer instead of reusing the '
      'existing layer $childLayer. See the documentation of RenderObject.updateCompositedLayer '
      'for more information on how to correctly implement this method.',
    );
    assert(debugOldOffset == updatedLayer.offset);
    child._needsCompositedLayerUpdate = false;
  }

  /// In debug mode, repaint the given render object using a custom painting
  /// context that can record the results of the painting operation in addition
  /// to performing the regular paint of the child.
  ///
  /// See also:
  ///
  ///  * [repaintCompositedChild], for repainting a composited child without
  ///    instrumentation.
  static void debugInstrumentRepaintCompositedChild(
    RenderObject child, {
    bool debugAlsoPaintedParent = false,
    required PaintingContext customContext,
  }) {
    assert(() {
      _repaintCompositedChild(
        child,
        debugAlsoPaintedParent: debugAlsoPaintedParent,
        childContext: customContext,
      );
      return true;
    }());
  }

  /// Paint a child [RenderObject].
  ///
  /// If the child has its own composited layer, the child will be composited
  /// into the layer subtree associated with this painting context. Otherwise,
  /// the child will be painted into the current PictureLayer for this context.
  void paintChild(RenderObject child, Offset offset) {
    assert(() {
      debugOnProfilePaint?.call(child);
      return true;
    }());

    if (child.isRepaintBoundary) {
      stopRecordingIfNeeded();
      _compositeChild(child, offset);
      // If a render object was a repaint boundary but no longer is one, this
      // is where the framework managed layer is automatically disposed.
    } else if (child._wasRepaintBoundary) {
      assert(child._layerHandle.layer is OffsetLayer);
      child._layerHandle.layer = null;
      child._paintWithContext(this, offset);
    } else {
      child._paintWithContext(this, offset);
    }
  }

  void _compositeChild(RenderObject child, Offset offset) {
    assert(!_isRecording);
    assert(child.isRepaintBoundary);
    assert(_canvas == null || _canvas!.getSaveCount() == 1);

    // Create a layer for our child, and paint the child into it.
    if (child._needsPaint || !child._wasRepaintBoundary) {
      repaintCompositedChild(child, debugAlsoPaintedParent: true);
    } else {
      if (child._needsCompositedLayerUpdate) {
        updateLayerProperties(child);
      }
      assert(() {
        // register the call for RepaintBoundary metrics
        child.debugRegisterRepaintBoundaryPaint();
        child._layerHandle.layer!.debugCreator = child.debugCreator ?? child;
        return true;
      }());
    }
    assert(child._layerHandle.layer is OffsetLayer);
    final OffsetLayer childOffsetLayer = child._layerHandle.layer! as OffsetLayer;
    childOffsetLayer.offset = offset;
    appendLayer(childOffsetLayer);
  }

  /// Adds a layer to the recording requiring that the recording is already
  /// stopped.
  ///
  /// Do not call this function directly: call [addLayer] or [pushLayer]
  /// instead. This function is called internally when all layers not
  /// generated from the [canvas] are added.
  ///
  /// Subclasses that need to customize how layers are added should override
  /// this method.
  @protected
  void appendLayer(Layer layer) {
    assert(!_isRecording);
    layer.remove();
    _containerLayer.append(layer);
  }

  bool get _isRecording {
    final bool hasCanvas = _canvas != null;
    assert(() {
      if (hasCanvas) {
        assert(_currentLayer != null);
        assert(_recorder != null);
        assert(_canvas != null);
      } else {
        assert(_currentLayer == null);
        assert(_recorder == null);
        assert(_canvas == null);
      }
      return true;
    }());
    return hasCanvas;
  }

  // Recording state
  PictureLayer? _currentLayer;
  ui.PictureRecorder? _recorder;
  Canvas? _canvas;

  /// The canvas on which to paint.
  ///
  /// The current canvas can change whenever you paint a child using this
  /// context, which means it's fragile to hold a reference to the canvas
  /// returned by this getter.
  @override
  Canvas get canvas {
    if (_canvas == null) {
      _startRecording();
    }
    assert(_currentLayer != null);
    return _canvas!;
  }

  void _startRecording() {
    assert(!_isRecording);
    _currentLayer = PictureLayer(estimatedBounds);
    _recorder = RendererBinding.instance.createPictureRecorder();
    _canvas = RendererBinding.instance.createCanvas(_recorder!);
    _containerLayer.append(_currentLayer!);
  }

  /// Adds a [CompositionCallback] for the current [ContainerLayer] used by this
  /// context.
  ///
  /// Composition callbacks are called whenever the layer tree containing the
  /// current layer of this painting context gets composited, or when it gets
  /// detached and will not be rendered again. This happens regardless of
  /// whether the layer is added via retained rendering or not.
  ///
  /// {@macro flutter.rendering.Layer.compositionCallbacks}
  ///
  /// See also:
  ///   *  [Layer.addCompositionCallback].
  VoidCallback addCompositionCallback(CompositionCallback callback) {
    return _containerLayer.addCompositionCallback(callback);
  }

  /// Stop recording to a canvas if recording has started.
  ///
  /// Do not call this function directly: functions in this class will call
  /// this method as needed. This function is called internally to ensure that
  /// recording is stopped before adding layers or finalizing the results of a
  /// paint.
  ///
  /// Subclasses that need to customize how recording to a canvas is performed
  /// should override this method to save the results of the custom canvas
  /// recordings.
  @protected
  @mustCallSuper
  void stopRecordingIfNeeded() {
    if (!_isRecording) {
      return;
    }
    assert(() {
      if (debugRepaintRainbowEnabled) {
        final Paint paint =
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 6.0
              ..color = debugCurrentRepaintColor.toColor();
        canvas.drawRect(estimatedBounds.deflate(3.0), paint);
      }
      if (debugPaintLayerBordersEnabled) {
        final Paint paint =
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 1.0
              ..color = const Color(0xFFFF9800);
        canvas.drawRect(estimatedBounds, paint);
      }
      return true;
    }());
    _currentLayer!.picture = _recorder!.endRecording();
    _currentLayer = null;
    _recorder = null;
    _canvas = null;
  }

  /// Hints that the painting in the current layer is complex and would benefit
  /// from caching.
  ///
  /// If this hint is not set, the compositor will apply its own heuristics to
  /// decide whether the current layer is complex enough to benefit from
  /// caching.
  ///
  /// Calling this ensures a [Canvas] is available. Only draw calls on the
  /// current canvas will be hinted; the hint is not propagated to new canvases
  /// created after a new layer is added to the painting context (e.g. with
  /// [addLayer] or [pushLayer]).
  void setIsComplexHint() {
    if (_currentLayer == null) {
      _startRecording();
    }
    _currentLayer!.isComplexHint = true;
  }

  /// Hints that the painting in the current layer is likely to change next frame.
  ///
  /// This hint tells the compositor not to cache the current layer because the
  /// cache will not be used in the future. If this hint is not set, the
  /// compositor will apply its own heuristics to decide whether the current
  /// layer is likely to be reused in the future.
  ///
  /// Calling this ensures a [Canvas] is available. Only draw calls on the
  /// current canvas will be hinted; the hint is not propagated to new canvases
  /// created after a new layer is added to the painting context (e.g. with
  /// [addLayer] or [pushLayer]).
  void setWillChangeHint() {
    if (_currentLayer == null) {
      _startRecording();
    }
    _currentLayer!.willChangeHint = true;
  }

  /// Adds a composited leaf layer to the recording.
  ///
  /// After calling this function, the [canvas] property will change to refer to
  /// a new [Canvas] that draws on top of the given layer.
  ///
  /// A [RenderObject] that uses this function is very likely to require its
  /// [RenderObject.alwaysNeedsCompositing] property to return true. That informs
  /// ancestor render objects that this render object will include a composited
  /// layer, which, for example, causes them to use composited clips.
  ///
  /// See also:
  ///
  ///  * [pushLayer], for adding a layer and painting further contents within
  ///    it.
  void addLayer(Layer layer) {
    stopRecordingIfNeeded();
    appendLayer(layer);
  }

  /// Appends the given layer to the recording, and calls the `painter` callback
  /// with that layer, providing the `childPaintBounds` as the estimated paint
  /// bounds of the child. The `childPaintBounds` can be used for debugging but
  /// have no effect on painting.
  ///
  /// The given layer must be an unattached orphan. (Providing a newly created
  /// object, rather than reusing an existing layer, satisfies that
  /// requirement.)
  ///
  /// {@template flutter.rendering.PaintingContext.pushLayer.offset}
  /// The `offset` is the offset to pass to the `painter`. In particular, it is
  /// not an offset applied to the layer itself. Layers conceptually by default
  /// have no position or size, though they can transform their contents. For
  /// example, an [OffsetLayer] applies an offset to its children.
  /// {@endtemplate}
  ///
  /// If the `childPaintBounds` are not specified then the current layer's paint
  /// bounds are used. This is appropriate if the child layer does not apply any
  /// transformation or clipping to its contents. The `childPaintBounds`, if
  /// specified, must be in the coordinate system of the new layer (i.e. as seen
  /// by its children after it applies whatever transform to its contents), and
  /// should not go outside the current layer's paint bounds.
  ///
  /// See also:
  ///
  ///  * [addLayer], for pushing a layer without painting further contents
  ///    within it.
  void pushLayer(
    ContainerLayer childLayer,
    PaintingContextCallback painter,
    Offset offset, {
    Rect? childPaintBounds,
  }) {
    // If a layer is being reused, it may already contain children. We remove
    // them so that `painter` can add children that are relevant for this frame.
    if (childLayer.hasChildren) {
      childLayer.removeAllChildren();
    }
    stopRecordingIfNeeded();
    appendLayer(childLayer);
    final PaintingContext childContext = createChildContext(
      childLayer,
      childPaintBounds ?? estimatedBounds,
    );

    painter(childContext, offset);
    childContext.stopRecordingIfNeeded();
  }

  /// Creates a painting context configured to paint into [childLayer].
  ///
  /// The `bounds` are estimated paint bounds for debugging purposes.
  @protected
  PaintingContext createChildContext(ContainerLayer childLayer, Rect bounds) {
    return PaintingContext(childLayer, bounds);
  }

  /// Clip further painting using a rectangle.
  ///
  /// {@template flutter.rendering.PaintingContext.pushClipRect.needsCompositing}
  /// The `needsCompositing` argument specifies whether the child needs
  /// compositing. Typically this matches the value of
  /// [RenderObject.needsCompositing] for the caller. If false, this method
  /// returns null, indicating that a layer is no longer necessary. If a render
  /// object calling this method stores the `oldLayer` in its
  /// [RenderObject.layer] field, it should set that field to null.
  ///
  /// When `needsCompositing` is false, this method will use a more efficient
  /// way to apply the layer effect than actually creating a layer.
  /// {@endtemplate}
  ///
  /// {@template flutter.rendering.PaintingContext.pushClipRect.offset}
  /// The `offset` argument is the offset from the origin of the canvas'
  /// coordinate system to the origin of the caller's coordinate system.
  /// {@endtemplate}
  ///
  /// The `clipRect` is the rectangle (in the caller's coordinate system) to use
  /// to clip the painting done by [painter]. It should not include the
  /// `offset`.
  ///
  /// The `painter` callback will be called while the `clipRect` is applied. It
  /// is called synchronously during the call to [pushClipRect].
  ///
  /// The `clipBehavior` argument controls how the rectangle is clipped.
  ///
  /// {@template flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  /// For the `oldLayer` argument, specify the layer created in the previous
  /// frame. This gives the engine more information for performance
  /// optimizations. Typically this is the value of [RenderObject.layer] that a
  /// render object creates once, then reuses for all subsequent frames until a
  /// layer is no longer needed (e.g. the render object no longer needs
  /// compositing) or until the render object changes the type of the layer
  /// (e.g. from opacity layer to a clip rect layer).
  /// {@endtemplate}
  ClipRectLayer? pushClipRect(
    bool needsCompositing,
    Offset offset,
    Rect clipRect,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.hardEdge,
    ClipRectLayer? oldLayer,
  }) {
    if (clipBehavior == Clip.none) {
      painter(this, offset);
      return null;
    }
    final Rect offsetClipRect = clipRect.shift(offset);
    if (needsCompositing) {
      final ClipRectLayer layer = oldLayer ?? ClipRectLayer();
      layer
        ..clipRect = offsetClipRect
        ..clipBehavior = clipBehavior;
      pushLayer(layer, painter, offset, childPaintBounds: offsetClipRect);
      return layer;
    } else {
      clipRectAndPaint(offsetClipRect, clipBehavior, offsetClipRect, () => painter(this, offset));
      return null;
    }
  }

  /// Clip further painting using a rounded rectangle.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.needsCompositing}
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.offset}
  ///
  /// The `bounds` argument is used to specify the region of the canvas (in the
  /// caller's coordinate system) into which `painter` will paint.
  ///
  /// The `clipRRect` argument specifies the rounded-rectangle (in the caller's
  /// coordinate system) to use to clip the painting done by `painter`. It
  /// should not include the `offset`.
  ///
  /// The `painter` callback will be called while the `clipRRect` is applied. It
  /// is called synchronously during the call to [pushClipRRect].
  ///
  /// The `clipBehavior` argument controls how the rounded rectangle is clipped.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  ClipRRectLayer? pushClipRRect(
    bool needsCompositing,
    Offset offset,
    Rect bounds,
    RRect clipRRect,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.antiAlias,
    ClipRRectLayer? oldLayer,
  }) {
    if (clipBehavior == Clip.none) {
      painter(this, offset);
      return null;
    }
    final Rect offsetBounds = bounds.shift(offset);
    final RRect offsetClipRRect = clipRRect.shift(offset);
    if (needsCompositing) {
      final ClipRRectLayer layer = oldLayer ?? ClipRRectLayer();
      layer
        ..clipRRect = offsetClipRRect
        ..clipBehavior = clipBehavior;
      pushLayer(layer, painter, offset, childPaintBounds: offsetBounds);
      return layer;
    } else {
      clipRRectAndPaint(offsetClipRRect, clipBehavior, offsetBounds, () => painter(this, offset));
      return null;
    }
  }

  /// Clip further painting using a path.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.needsCompositing}
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.offset}
  ///
  /// The `bounds` argument is used to specify the region of the canvas (in the
  /// caller's coordinate system) into which `painter` will paint.
  ///
  /// The `clipPath` argument specifies the [Path] (in the caller's coordinate
  /// system) to use to clip the painting done by `painter`. It should not
  /// include the `offset`.
  ///
  /// The `painter` callback will be called while the `clipPath` is applied. It
  /// is called synchronously during the call to [pushClipPath].
  ///
  /// The `clipBehavior` argument controls how the path is clipped.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  ClipPathLayer? pushClipPath(
    bool needsCompositing,
    Offset offset,
    Rect bounds,
    Path clipPath,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.antiAlias,
    ClipPathLayer? oldLayer,
  }) {
    if (clipBehavior == Clip.none) {
      painter(this, offset);
      return null;
    }
    final Rect offsetBounds = bounds.shift(offset);
    final Path offsetClipPath = clipPath.shift(offset);
    if (needsCompositing) {
      final ClipPathLayer layer = oldLayer ?? ClipPathLayer();
      layer
        ..clipPath = offsetClipPath
        ..clipBehavior = clipBehavior;
      pushLayer(layer, painter, offset, childPaintBounds: offsetBounds);
      return layer;
    } else {
      clipPathAndPaint(offsetClipPath, clipBehavior, offsetBounds, () => painter(this, offset));
      return null;
    }
  }

  /// Blend further painting with a color filter.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushLayer.offset}
  ///
  /// The `colorFilter` argument is the [ColorFilter] value to use when blending
  /// the painting done by `painter`.
  ///
  /// The `painter` callback will be called while the `colorFilter` is applied.
  /// It is called synchronously during the call to [pushColorFilter].
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  ///
  /// A [RenderObject] that uses this function is very likely to require its
  /// [RenderObject.alwaysNeedsCompositing] property to return true. That informs
  /// ancestor render objects that this render object will include a composited
  /// layer, which, for example, causes them to use composited clips.
  ColorFilterLayer pushColorFilter(
    Offset offset,
    ColorFilter colorFilter,
    PaintingContextCallback painter, {
    ColorFilterLayer? oldLayer,
  }) {
    final ColorFilterLayer layer = oldLayer ?? ColorFilterLayer();
    layer.colorFilter = colorFilter;
    pushLayer(layer, painter, offset);
    return layer;
  }

  /// Transform further painting using a matrix.
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.needsCompositing}
  ///
  /// The `offset` argument is the offset to pass to `painter` and the offset to
  /// the origin used by `transform`.
  ///
  /// The `transform` argument is the [Matrix4] with which to transform the
  /// coordinate system while calling `painter`. It should not include `offset`.
  /// It is applied effectively after applying `offset`.
  ///
  /// The `painter` callback will be called while the `transform` is applied. It
  /// is called synchronously during the call to [pushTransform].
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  TransformLayer? pushTransform(
    bool needsCompositing,
    Offset offset,
    Matrix4 transform,
    PaintingContextCallback painter, {
    TransformLayer? oldLayer,
  }) {
    final Matrix4 effectiveTransform =
        Matrix4.translationValues(offset.dx, offset.dy, 0.0)
          ..multiply(transform)
          ..translate(-offset.dx, -offset.dy);
    if (needsCompositing) {
      final TransformLayer layer = oldLayer ?? TransformLayer();
      layer.transform = effectiveTransform;
      pushLayer(
        layer,
        painter,
        offset,
        childPaintBounds: MatrixUtils.inverseTransformRect(effectiveTransform, estimatedBounds),
      );
      return layer;
    } else {
      canvas
        ..save()
        ..transform(effectiveTransform.storage);
      painter(this, offset);
      canvas.restore();
      return null;
    }
  }

  /// Blend further painting with an alpha value.
  ///
  /// The `offset` argument indicates an offset to apply to all the children
  /// (the rendering created by `painter`).
  ///
  /// The `alpha` argument is the alpha value to use when blending the painting
  /// done by `painter`. An alpha value of 0 means the painting is fully
  /// transparent and an alpha value of 255 means the painting is fully opaque.
  ///
  /// The `painter` callback will be called while the `alpha` is applied. It
  /// is called synchronously during the call to [pushOpacity].
  ///
  /// {@macro flutter.rendering.PaintingContext.pushClipRect.oldLayer}
  ///
  /// A [RenderObject] that uses this function is very likely to require its
  /// [RenderObject.alwaysNeedsCompositing] property to return true. That informs
  /// ancestor render objects that this render object will include a composited
  /// layer, which, for example, causes them to use composited clips.
  OpacityLayer pushOpacity(
    Offset offset,
    int alpha,
    PaintingContextCallback painter, {
    OpacityLayer? oldLayer,
  }) {
    final OpacityLayer layer = oldLayer ?? OpacityLayer();
    layer
      ..alpha = alpha
      ..offset = offset;
    pushLayer(layer, painter, Offset.zero);
    return layer;
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'PaintingContext')}#$hashCode(layer: $_containerLayer, canvas bounds: $estimatedBounds)';
}

/// An abstract set of layout constraints.
///
/// Concrete layout models (such as box) will create concrete subclasses to
/// communicate layout constraints between parents and children.
///
/// ## Writing a Constraints subclass
///
/// When creating a new [RenderObject] subclass with a new layout protocol, one
/// will usually need to create a new [Constraints] subclass to express the
/// input to the layout algorithms.
///
/// A [Constraints] subclass should be immutable (all fields final). There are
/// several members to implement, in addition to whatever fields, constructors,
/// and helper methods one may find useful for a particular layout protocol:
///
/// * The [isTight] getter, which should return true if the object represents a
///   case where the [RenderObject] class has no choice for how to lay itself
///   out. For example, [BoxConstraints] returns true for [isTight] when both
///   the minimum and maximum widths and the minimum and maximum heights are
///   equal.
///
/// * The [isNormalized] getter, which should return true if the object
///   represents its data in its canonical form. Sometimes, it is possible for
///   fields to be redundant with each other, such that several different
///   representations have the same implications. For example, a
///   [BoxConstraints] instance with its minimum width greater than its maximum
///   width is equivalent to one where the maximum width is set to that minimum
///   width (`2<w<1` is equivalent to `2<w<2`, since minimum constraints have
///   priority). This getter is used by the default implementation of
///   [debugAssertIsValid].
///
/// * The [debugAssertIsValid] method, which should assert if there's anything
///   wrong with the constraints object. (We use this approach rather than
///   asserting in constructors so that our constructors can be `const` and so
///   that it is possible to create invalid constraints temporarily while
///   building valid ones.) See the implementation of
///   [BoxConstraints.debugAssertIsValid] for an example of the detailed checks
///   that can be made.
///
/// * The [==] operator and the [hashCode] getter, so that constraints can be
///   compared for equality. If a render object is given constraints that are
///   equal, then the rendering library will avoid laying the object out again
///   if it is not dirty.
///
/// * The [toString] method, which should describe the constraints so that they
///   appear in a usefully readable form in the output of [debugDumpRenderTree].
@immutable
abstract class Constraints {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const Constraints();

  /// Whether there is exactly one size possible given these constraints.
  bool get isTight;

  /// Whether the constraint is expressed in a consistent manner.
  bool get isNormalized;

  /// Asserts that the constraints are valid.
  ///
  /// This might involve checks more detailed than [isNormalized].
  ///
  /// For example, the [BoxConstraints] subclass verifies that the constraints
  /// are not [double.nan].
  ///
  /// If the `isAppliedConstraint` argument is true, then even stricter rules
  /// are enforced. This argument is set to true when checking constraints that
  /// are about to be applied to a [RenderObject] during layout, as opposed to
  /// constraints that may be further affected by other constraints. For
  /// example, the asserts for verifying the validity of
  /// [RenderConstrainedBox.additionalConstraints] do not set this argument, but
  /// the asserts for verifying the argument passed to the [RenderObject.layout]
  /// method do.
  ///
  /// The `informationCollector` argument takes an optional callback which is
  /// called when an exception is to be thrown. The collected information is
  /// then included in the message after the error line.
  ///
  /// Returns the same as [isNormalized] if asserts are disabled.
  bool debugAssertIsValid({
    bool isAppliedConstraint = false,
    InformationCollector? informationCollector,
  }) {
    assert(isNormalized);
    return isNormalized;
  }
}

/// Signature for a function that is called for each [RenderObject].
///
/// Used by [RenderObject.visitChildren] and [RenderObject.visitChildrenForSemantics].
typedef RenderObjectVisitor = void Function(RenderObject child);

/// Signature for a function that is called during layout.
///
/// Used by [RenderObject.invokeLayoutCallback].
typedef LayoutCallback<T extends Constraints> = void Function(T constraints);

class _LocalSemanticsHandle implements SemanticsHandle {
  _LocalSemanticsHandle._(PipelineOwner owner, this.listener) : _owner = owner {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/rendering.dart',
        className: '$_LocalSemanticsHandle',
        object: this,
      );
    }

    if (listener != null) {
      _owner.semanticsOwner!.addListener(listener!);
    }
  }

  final PipelineOwner _owner;

  /// The callback that will be notified when the semantics tree updates.
  final VoidCallback? listener;

  @override
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }

    if (listener != null) {
      _owner.semanticsOwner!.removeListener(listener!);
    }
    _owner._didDisposeSemanticsHandle();
  }
}

/// The pipeline owner manages the rendering pipeline.
///
/// The pipeline owner provides an interface for driving the rendering pipeline
/// and stores the state about which render objects have requested to be visited
/// in each stage of the pipeline. To flush the pipeline, call the following
/// functions in order:
///
/// 1. [flushLayout] updates any render objects that need to compute their
///    layout. During this phase, the size and position of each render
///    object is calculated. Render objects might dirty their painting or
///    compositing state during this phase.
/// 2. [flushCompositingBits] updates any render objects that have dirty
///    compositing bits. During this phase, each render object learns whether
///    any of its children require compositing. This information is used during
///    the painting phase when selecting how to implement visual effects such as
///    clipping. If a render object has a composited child, it needs to use a
///    [Layer] to create the clip in order for the clip to apply to the
///    composited child (which will be painted into its own [Layer]).
/// 3. [flushPaint] visits any render objects that need to paint. During this
///    phase, render objects get a chance to record painting commands into
///    [PictureLayer]s and construct other composited [Layer]s.
/// 4. Finally, if semantics are enabled, [flushSemantics] will compile the
///    semantics for the render objects. This semantic information is used by
///    assistive technology to improve the accessibility of the render tree.
///
/// The [RendererBinding] holds the pipeline owner for the render objects that
/// are visible on screen. You can create other pipeline owners to manage
/// off-screen objects, which can flush their pipelines independently of the
/// on-screen render objects.
///
/// [PipelineOwner]s can be organized in a tree to manage multiple render trees,
/// where each [PipelineOwner] is responsible for one of the render trees. To
/// build or modify the tree, call [adoptChild] or [dropChild]. During each of
/// the different flush phases described above, a [PipelineOwner] will first
/// perform the phase on the nodes it manages in its own render tree before
/// calling the same flush method on its children. No assumption must be made
/// about the order in which child [PipelineOwner]s are flushed.
///
/// A [PipelineOwner] may also be [attach]ed to a [PipelineManifold], which
/// gives it access to platform functionality usually exposed by the bindings
/// without tying it to a specific binding implementation. All [PipelineOwner]s
/// in a given tree must be attached to the same [PipelineManifold]. This
/// happens automatically during [adoptChild].
class PipelineOwner with DiagnosticableTreeMixin {
  /// Creates a pipeline owner.
  ///
  /// Typically created by the binding (e.g., [RendererBinding]), but can be
  /// created separately from the binding to drive off-screen render objects
  /// through the rendering pipeline.
  PipelineOwner({
    this.onNeedVisualUpdate,
    this.onSemanticsOwnerCreated,
    this.onSemanticsUpdate,
    this.onSemanticsOwnerDisposed,
  }) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/rendering.dart',
        className: '$PipelineOwner',
        object: this,
      );
    }
  }

  /// Called when a render object associated with this pipeline owner wishes to
  /// update its visual appearance.
  ///
  /// Typical implementations of this function will schedule a task to flush the
  /// various stages of the pipeline. This function might be called multiple
  /// times in quick succession. Implementations should take care to discard
  /// duplicate calls quickly.
  ///
  /// When the [PipelineOwner] is attached to a [PipelineManifold] and
  /// [onNeedVisualUpdate] is provided, the [onNeedVisualUpdate] callback is
  /// invoked instead of calling [PipelineManifold.requestVisualUpdate].
  final VoidCallback? onNeedVisualUpdate;

  /// Called whenever this pipeline owner creates a semantics object.
  ///
  /// Typical implementations will schedule the creation of the initial
  /// semantics tree.
  final VoidCallback? onSemanticsOwnerCreated;

  /// Called whenever this pipeline owner's semantics owner emits a [SemanticsUpdate].
  ///
  /// Typical implementations will delegate the [SemanticsUpdate] to a [FlutterView]
  /// that can handle the [SemanticsUpdate].
  final SemanticsUpdateCallback? onSemanticsUpdate;

  /// Called whenever this pipeline owner disposes its semantics owner.
  ///
  /// Typical implementations will tear down the semantics tree.
  final VoidCallback? onSemanticsOwnerDisposed;

  /// Calls [onNeedVisualUpdate] if [onNeedVisualUpdate] is not null.
  ///
  /// Used to notify the pipeline owner that an associated render object wishes
  /// to update its visual appearance.
  void requestVisualUpdate() {
    if (onNeedVisualUpdate != null) {
      onNeedVisualUpdate!();
    } else {
      _manifold?.requestVisualUpdate();
    }
  }

  /// The unique object managed by this pipeline that has no parent.
  RenderObject? get rootNode => _rootNode;
  RenderObject? _rootNode;
  set rootNode(RenderObject? value) {
    if (_rootNode == value) {
      return;
    }
    _rootNode?.detach();
    _rootNode = value;
    _rootNode?.attach(this);
  }

  // Whether the current [flushLayout] call should pause to incorporate the
  // [RenderObject]s in `_nodesNeedingLayout` into the current dirty list,
  // before continuing to process dirty relayout boundaries.
  //
  // This flag is set to true when a [RenderObject.invokeLayoutCallback]
  // returns, to avoid laying out dirty relayout boundaries in an incorrect
  // order and causing them to be laid out more than once per frame. See
  // layout_builder_mutations_test.dart for an example.
  //
  // The new dirty nodes are not immediately merged after a
  // [RenderObject.invokeLayoutCallback] call because we may encounter multiple
  // such calls while processing a single relayout boundary in [flushLayout].
  // Batching new dirty nodes can reduce the number of merges [flushLayout]
  // has to perform.
  bool _shouldMergeDirtyNodes = false;
  List<RenderObject> _nodesNeedingLayout = <RenderObject>[];

  /// Whether this pipeline is currently in the layout phase.
  ///
  /// Specifically, whether [flushLayout] is currently running.
  ///
  /// Only valid when asserts are enabled; in release builds, this
  /// always returns false.
  bool get debugDoingLayout => _debugDoingLayout;
  bool _debugDoingLayout = false;
  bool _debugDoingChildLayout = false;

  /// Update the layout information for all dirty render objects.
  ///
  /// This function is one of the core stages of the rendering pipeline. Layout
  /// information is cleaned prior to painting so that render objects will
  /// appear on screen in their up-to-date locations.
  ///
  /// See [RendererBinding] for an example of how this function is used.
  void flushLayout() {
    if (!kReleaseMode) {
      Map<String, String>? debugTimelineArguments;
      assert(() {
        if (debugEnhanceLayoutTimelineArguments) {
          debugTimelineArguments = <String, String>{
            'dirty count': '${_nodesNeedingLayout.length}',
            'dirty list': '$_nodesNeedingLayout',
          };
        }
        return true;
      }());
      FlutterTimeline.startSync(
        'LAYOUT$_debugRootSuffixForTimelineEventNames',
        arguments: debugTimelineArguments,
      );
    }
    assert(() {
      _debugDoingLayout = true;
      return true;
    }());
    try {
      while (_nodesNeedingLayout.isNotEmpty) {
        assert(!_shouldMergeDirtyNodes);
        final List<RenderObject> dirtyNodes = _nodesNeedingLayout;
        _nodesNeedingLayout = <RenderObject>[];
        dirtyNodes.sort((RenderObject a, RenderObject b) => a.depth - b.depth);
        for (int i = 0; i < dirtyNodes.length; i++) {
          if (_shouldMergeDirtyNodes) {
            _shouldMergeDirtyNodes = false;
            if (_nodesNeedingLayout.isNotEmpty) {
              _nodesNeedingLayout.addAll(dirtyNodes.getRange(i, dirtyNodes.length));
              break;
            }
          }
          final RenderObject node = dirtyNodes[i];
          if (node._needsLayout && node.owner == this) {
            node._layoutWithoutResize();
          }
        }
        // No need to merge dirty nodes generated from processing the last
        // relayout boundary back.
        _shouldMergeDirtyNodes = false;
      }

      assert(() {
        _debugDoingChildLayout = true;
        return true;
      }());
      for (final PipelineOwner child in _children) {
        child.flushLayout();
      }
      assert(
        _nodesNeedingLayout.isEmpty,
        'Child PipelineOwners must not dirty nodes in their parent.',
      );
    } finally {
      _shouldMergeDirtyNodes = false;
      assert(() {
        _debugDoingLayout = false;
        _debugDoingChildLayout = false;
        return true;
      }());
      if (!kReleaseMode) {
        FlutterTimeline.finishSync();
      }
    }
  }

  // This flag is used to allow the kinds of mutations performed by GlobalKey
  // reparenting while a LayoutBuilder is being rebuilt and in so doing tries to
  // move a node from another LayoutBuilder subtree that hasn't been updated
  // yet. To set this, call [_enableMutationsToDirtySubtrees], which is called
  // by [RenderObject.invokeLayoutCallback].
  bool _debugAllowMutationsToDirtySubtrees = false;

  // See [RenderObject.invokeLayoutCallback].
  void _enableMutationsToDirtySubtrees(VoidCallback callback) {
    assert(_debugDoingLayout);
    bool? oldState;
    assert(() {
      oldState = _debugAllowMutationsToDirtySubtrees;
      _debugAllowMutationsToDirtySubtrees = true;
      return true;
    }());
    try {
      callback();
    } finally {
      _shouldMergeDirtyNodes = true;
      assert(() {
        _debugAllowMutationsToDirtySubtrees = oldState!;
        return true;
      }());
    }
  }

  final List<RenderObject> _nodesNeedingCompositingBitsUpdate = <RenderObject>[];

  /// Updates the [RenderObject.needsCompositing] bits.
  ///
  /// Called as part of the rendering pipeline after [flushLayout] and before
  /// [flushPaint].
  void flushCompositingBits() {
    if (!kReleaseMode) {
      FlutterTimeline.startSync('UPDATING COMPOSITING BITS$_debugRootSuffixForTimelineEventNames');
    }
    _nodesNeedingCompositingBitsUpdate.sort((RenderObject a, RenderObject b) => a.depth - b.depth);
    for (final RenderObject node in _nodesNeedingCompositingBitsUpdate) {
      if (node._needsCompositingBitsUpdate && node.owner == this) {
        node._updateCompositingBits();
      }
    }
    _nodesNeedingCompositingBitsUpdate.clear();
    for (final PipelineOwner child in _children) {
      child.flushCompositingBits();
    }
    assert(
      _nodesNeedingCompositingBitsUpdate.isEmpty,
      'Child PipelineOwners must not dirty nodes in their parent.',
    );
    if (!kReleaseMode) {
      FlutterTimeline.finishSync();
    }
  }

  List<RenderObject> _nodesNeedingPaint = <RenderObject>[];

  /// Whether this pipeline is currently in the paint phase.
  ///
  /// Specifically, whether [flushPaint] is currently running.
  ///
  /// Only valid when asserts are enabled. In release builds,
  /// this always returns false.
  bool get debugDoingPaint => _debugDoingPaint;
  bool _debugDoingPaint = false;

  /// Update the display lists for all render objects.
  ///
  /// This function is one of the core stages of the rendering pipeline.
  /// Painting occurs after layout and before the scene is recomposited so that
  /// scene is composited with up-to-date display lists for every render object.
  ///
  /// See [RendererBinding] for an example of how this function is used.
  void flushPaint() {
    if (!kReleaseMode) {
      Map<String, String>? debugTimelineArguments;
      assert(() {
        if (debugEnhancePaintTimelineArguments) {
          debugTimelineArguments = <String, String>{
            'dirty count': '${_nodesNeedingPaint.length}',
            'dirty list': '$_nodesNeedingPaint',
          };
        }
        return true;
      }());
      FlutterTimeline.startSync(
        'PAINT$_debugRootSuffixForTimelineEventNames',
        arguments: debugTimelineArguments,
      );
    }
    try {
      assert(() {
        _debugDoingPaint = true;
        return true;
      }());
      final List<RenderObject> dirtyNodes = _nodesNeedingPaint;
      _nodesNeedingPaint = <RenderObject>[];

      // Sort the dirty nodes in reverse order (deepest first).
      for (final RenderObject node
          in dirtyNodes..sort((RenderObject a, RenderObject b) => b.depth - a.depth)) {
        assert(node._layerHandle.layer != null);
        if ((node._needsPaint || node._needsCompositedLayerUpdate) && node.owner == this) {
          if (node._layerHandle.layer!.attached) {
            assert(node.isRepaintBoundary);
            if (node._needsPaint) {
              PaintingContext.repaintCompositedChild(node);
            } else {
              PaintingContext.updateLayerProperties(node);
            }
          } else {
            node._skippedPaintingOnLayer();
          }
        }
      }
      for (final PipelineOwner child in _children) {
        child.flushPaint();
      }
      assert(
        _nodesNeedingPaint.isEmpty,
        'Child PipelineOwners must not dirty nodes in their parent.',
      );
    } finally {
      assert(() {
        _debugDoingPaint = false;
        return true;
      }());
      if (!kReleaseMode) {
        FlutterTimeline.finishSync();
      }
    }
  }

  /// The object that is managing semantics for this pipeline owner, if any.
  ///
  /// An owner is created by [ensureSemantics] or when the [PipelineManifold] to
  /// which this owner is connected has [PipelineManifold.semanticsEnabled] set
  /// to true. The owner is valid for as long as
  /// [PipelineManifold.semanticsEnabled] remains true or while there are
  /// outstanding [SemanticsHandle]s from calls to [ensureSemantics]. The
  /// [semanticsOwner] field will revert to null once both conditions are no
  /// longer met.
  ///
  /// When [semanticsOwner] is null, the [PipelineOwner] skips all steps
  /// relating to semantics.
  SemanticsOwner? get semanticsOwner => _semanticsOwner;
  SemanticsOwner? _semanticsOwner;

  /// Deprecated.
  ///
  /// Use [SemanticsBinding.debugOutstandingSemanticsHandles] instead. This
  /// API is broken because an outstanding semantics handle on a given pipeline
  /// owner doesn't mean that semantics are actually produced.
  @Deprecated(
    'Use SemanticsBinding.debugOutstandingSemanticsHandles instead. '
    'This API is broken (see ensureSemantics). '
    'This feature was deprecated after v3.22.0-23.0.pre.',
  )
  int get debugOutstandingSemanticsHandles => _outstandingSemanticsHandles;
  int _outstandingSemanticsHandles = 0;

  /// Deprecated.
  ///
  /// Call [SemanticsBinding.ensureSemantics] instead and optionally add a
  /// listener to [PipelineOwner.semanticsOwner]. This API is broken as calling
  /// it does not guarantee that semantics are produced.
  @Deprecated(
    'Call SemanticsBinding.ensureSemantics instead and optionally add a listener to PipelineOwner.semanticsOwner. '
    'This API is broken; it does not guarantee that semantics are actually produced. '
    'This feature was deprecated after v3.22.0-23.0.pre.',
  )
  SemanticsHandle ensureSemantics({VoidCallback? listener}) {
    _outstandingSemanticsHandles += 1;
    _updateSemanticsOwner();
    return _LocalSemanticsHandle._(this, listener);
  }

  void _updateSemanticsOwner() {
    if ((_manifold?.semanticsEnabled ?? false) || _outstandingSemanticsHandles > 0) {
      if (_semanticsOwner == null) {
        assert(
          onSemanticsUpdate != null,
          'Attempted to enable semantics without configuring an onSemanticsUpdate callback.',
        );
        _semanticsOwner = SemanticsOwner(onSemanticsUpdate: onSemanticsUpdate!);
        onSemanticsOwnerCreated?.call();
      }
    } else if (_semanticsOwner != null) {
      _semanticsOwner?.dispose();
      _semanticsOwner = null;
      onSemanticsOwnerDisposed?.call();
    }
  }

  void _didDisposeSemanticsHandle() {
    assert(_semanticsOwner != null);
    _outstandingSemanticsHandles -= 1;
    _updateSemanticsOwner();
  }

  bool _debugDoingSemantics = false;
  final Set<RenderObject> _nodesNeedingSemantics = <RenderObject>{};

  /// Update the semantics for render objects marked as needing a semantics
  /// update.
  ///
  /// Initially, only the root node, as scheduled by
  /// [RenderObject.scheduleInitialSemantics], needs a semantics update.
  ///
  /// This function is one of the core stages of the rendering pipeline. The
  /// semantics are compiled after painting and only after
  /// [RenderObject.scheduleInitialSemantics] has been called.
  ///
  /// See [RendererBinding] for an example of how this function is used.
  void flushSemantics() {
    if (_semanticsOwner == null) {
      return;
    }
    if (!kReleaseMode) {
      FlutterTimeline.startSync('SEMANTICS$_debugRootSuffixForTimelineEventNames');
    }
    assert(_semanticsOwner != null);
    assert(() {
      _debugDoingSemantics = true;
      return true;
    }());
    try {
      final List<RenderObject> nodesToProcess =
          _nodesNeedingSemantics.toList()
            ..sort((RenderObject a, RenderObject b) => a.depth - b.depth);
      _nodesNeedingSemantics.clear();
      for (final RenderObject node in nodesToProcess) {
        if (node._needsSemanticsUpdate && node.owner == this) {
          node._updateSemantics();
        }
      }
      _semanticsOwner!.sendSemanticsUpdate();
      for (final PipelineOwner child in _children) {
        child.flushSemantics();
      }
      assert(
        _nodesNeedingSemantics.isEmpty,
        'Child PipelineOwners must not dirty nodes in their parent.',
      );
    } finally {
      assert(() {
        _debugDoingSemantics = false;
        return true;
      }());
      if (!kReleaseMode) {
        FlutterTimeline.finishSync();
      }
    }
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[
      for (final PipelineOwner child in _children) child.toDiagnosticsNode(),
    ];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<RenderObject>('rootNode', rootNode, defaultValue: null));
  }

  // TREE MANAGEMENT

  final Set<PipelineOwner> _children = <PipelineOwner>{};
  PipelineManifold? _manifold;

  PipelineOwner? _debugParent;
  bool _debugSetParent(PipelineOwner child, PipelineOwner? parent) {
    child._debugParent = parent;
    return true;
  }

  String get _debugRootSuffixForTimelineEventNames => _debugParent == null ? ' (root)' : '';

  /// Mark this [PipelineOwner] as attached to the given [PipelineManifold].
  ///
  /// Typically, this is only called directly on the root [PipelineOwner].
  /// Children are automatically attached to their parent's [PipelineManifold]
  /// when [adoptChild] is called.
  void attach(PipelineManifold manifold) {
    assert(_manifold == null);
    _manifold = manifold;
    _manifold!.addListener(_updateSemanticsOwner);
    _updateSemanticsOwner();

    for (final PipelineOwner child in _children) {
      child.attach(manifold);
    }
  }

  /// Mark this [PipelineOwner] as detached.
  ///
  /// Typically, this is only called directly on the root [PipelineOwner].
  /// Children are automatically detached from their parent's [PipelineManifold]
  /// when [dropChild] is called.
  void detach() {
    assert(_manifold != null);
    _manifold!.removeListener(_updateSemanticsOwner);
    _manifold = null;
    // Not updating the semantics owner here to not disrupt any of its clients
    // in case we get re-attached. If necessary, semantics owner will be updated
    // in "attach", or disposed in "dispose", if not reattached.

    for (final PipelineOwner child in _children) {
      child.detach();
    }
  }

  // In theory, child list modifications are also disallowed between
  // _debugDoingChildrenLayout and _debugDoingPaint as well as between
  // _debugDoingPaint and _debugDoingSemantics. However, since the associated
  // flush methods are usually called back to back, this gets us close enough.
  bool get _debugAllowChildListModifications =>
      !_debugDoingChildLayout && !_debugDoingPaint && !_debugDoingSemantics;

  /// Adds `child` to this [PipelineOwner].
  ///
  /// During the phases of frame production (see [RendererBinding.drawFrame]),
  /// the parent [PipelineOwner] will complete a phase for the nodes it owns
  /// directly before invoking the flush method corresponding to the current
  /// phase on its child [PipelineOwner]s. For example, during layout, the
  /// parent [PipelineOwner] will first lay out its own nodes before calling
  /// [flushLayout] on its children. During paint, it will first paint its own
  /// nodes before calling [flushPaint] on its children. This order also applies
  /// for all the other phases.
  ///
  /// No assumptions must be made about the order in which child
  /// [PipelineOwner]s are flushed.
  ///
  /// No new children may be added after the [PipelineOwner] has started calling
  /// [flushLayout] on any of its children until the end of the current frame.
  ///
  /// To remove a child, call [dropChild].
  void adoptChild(PipelineOwner child) {
    assert(child._debugParent == null);
    assert(!_children.contains(child));
    assert(_debugAllowChildListModifications, 'Cannot modify child list after layout.');
    _children.add(child);
    if (!kReleaseMode) {
      _debugSetParent(child, this);
    }
    if (_manifold != null) {
      child.attach(_manifold!);
    }
  }

  /// Removes a child [PipelineOwner] previously added via [adoptChild].
  ///
  /// This node will cease to call the flush methods on the `child` during frame
  /// production.
  ///
  /// No children may be removed after the [PipelineOwner] has started calling
  /// [flushLayout] on any of its children until the end of the current frame.
  void dropChild(PipelineOwner child) {
    assert(child._debugParent == this);
    assert(_children.contains(child));
    assert(_debugAllowChildListModifications, 'Cannot modify child list after layout.');
    _children.remove(child);
    if (!kReleaseMode) {
      _debugSetParent(child, null);
    }
    if (_manifold != null) {
      child.detach();
    }
  }

  /// Calls `visitor` for each immediate child of this [PipelineOwner].
  ///
  /// See also:
  ///
  ///  * [adoptChild] to add a child.
  ///  * [dropChild] to remove a child.
  void visitChildren(PipelineOwnerVisitor visitor) {
    _children.forEach(visitor);
  }

  /// Release any resources held by this pipeline owner.
  ///
  /// Prior to calling this method the pipeline owner must be removed from the
  /// pipeline owner tree, i.e. it must have neither a parent nor any children
  /// (see [dropChild]). It also must be [detach]ed from any [PipelineManifold].
  ///
  /// The object is no longer usable after calling dispose.
  void dispose() {
    assert(_children.isEmpty);
    assert(rootNode == null);
    assert(_manifold == null);
    assert(_debugParent == null);
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _semanticsOwner?.dispose();
    _semanticsOwner = null;
    _nodesNeedingLayout.clear();
    _nodesNeedingCompositingBitsUpdate.clear();
    _nodesNeedingPaint.clear();
    _nodesNeedingSemantics.clear();
  }
}

/// Signature for the callback to [PipelineOwner.visitChildren].
///
/// The argument is the child being visited.
typedef PipelineOwnerVisitor = void Function(PipelineOwner child);

/// Manages a tree of [PipelineOwner]s.
///
/// All [PipelineOwner]s within a tree are attached to the same
/// [PipelineManifold], which gives them access to shared functionality such
/// as requesting a visual update (by calling [requestVisualUpdate]). As such,
/// the [PipelineManifold] gives the [PipelineOwner]s access to functionality
/// usually provided by the bindings without tying the [PipelineOwner]s to a
/// particular binding implementation.
///
/// The root of the [PipelineOwner] tree is attached to a [PipelineManifold] by
/// passing the manifold to [PipelineOwner.attach]. Children are attached to the
/// same [PipelineManifold] as their parent when they are adopted via
/// [PipelineOwner.adoptChild].
///
/// [PipelineOwner]s can register listeners with the [PipelineManifold] to be
/// informed when certain values provided by the [PipelineManifold] change.
abstract class PipelineManifold implements Listenable {
  /// Whether [PipelineOwner]s connected to this [PipelineManifold] should
  /// collect semantics information and produce a semantics tree.
  ///
  /// The [PipelineManifold] notifies its listeners (managed with [addListener]
  /// and [removeListener]) when this property changes its value.
  ///
  /// See also:
  ///
  ///  * [SemanticsBinding.semanticsEnabled], which [PipelineManifold]
  ///    implementations typically use to back this property.
  bool get semanticsEnabled;

  /// Called by a [PipelineOwner] connected to this [PipelineManifold] when a
  /// [RenderObject] associated with that pipeline owner wishes to update its
  /// visual appearance.
  ///
  /// Typical implementations of this function will schedule a task to flush the
  /// various stages of the pipeline. This function might be called multiple
  /// times in quick succession. Implementations should take care to discard
  /// duplicate calls quickly.
  ///
  /// A [PipelineOwner] connected to this [PipelineManifold] will call
  /// [PipelineOwner.onNeedVisualUpdate] instead of this method if it has been
  /// configured with a non-null [PipelineOwner.onNeedVisualUpdate] callback.
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding.ensureVisualUpdate], which [PipelineManifold]
  ///    implementations typically call to implement this method.
  void requestVisualUpdate();
}

const String _flutterRenderingLibrary = 'package:flutter/rendering.dart';

/// An object in the render tree.
///
/// The [RenderObject] class hierarchy is the core of the rendering
/// library's reason for being.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=zmbmrw07qBc}
///
/// [RenderObject]s have a [parent], and have a slot called [parentData] in
/// which the parent [RenderObject] can store child-specific data, for example,
/// the child position. The [RenderObject] class also implements the basic
/// layout and paint protocols.
///
/// The [RenderObject] class, however, does not define a child model (e.g.
/// whether a node has zero, one, or more children). It also doesn't define a
/// coordinate system (e.g. whether children are positioned in Cartesian
/// coordinates, in polar coordinates, etc) or a specific layout protocol (e.g.
/// whether the layout is width-in-height-out, or constraint-in-size-out, or
/// whether the parent sets the size and position of the child before or after
/// the child lays out, etc; or indeed whether the children are allowed to read
/// their parent's [parentData] slot).
///
/// The [RenderBox] subclass introduces the opinion that the layout
/// system uses Cartesian coordinates.
///
/// ## Lifecycle
///
/// A [RenderObject] must [dispose] when it is no longer needed. The creator
/// of the object is responsible for disposing of it. Typically, the creator is
/// a [RenderObjectElement], and that element will dispose the object it creates
/// when it is unmounted.
///
/// [RenderObject]s are responsible for cleaning up any expensive resources
/// they hold when [dispose] is called, such as [Picture] or [Image] objects.
/// This includes any [Layer]s that the render object has directly created. The
/// base implementation of dispose will nullify the [layer] property. Subclasses
/// must also nullify any other layer(s) it directly creates.
///
/// ## Writing a RenderObject subclass
///
/// In most cases, subclassing [RenderObject] itself is overkill, and
/// [RenderBox] would be a better starting point. However, if a render object
/// doesn't want to use a Cartesian coordinate system, then it should indeed
/// inherit from [RenderObject] directly. This allows it to define its own
/// layout protocol by using a new subclass of [Constraints] rather than using
/// [BoxConstraints], and by potentially using an entirely new set of objects
/// and values to represent the result of the output rather than just a [Size].
/// This increased flexibility comes at the cost of not being able to rely on
/// the features of [RenderBox]. For example, [RenderBox] implements an
/// intrinsic sizing protocol that allows you to measure a child without fully
/// laying it out, in such a way that if that child changes size, the parent
/// will be laid out again (to take into account the new dimensions of the
/// child). This is a subtle and bug-prone feature to get right.
///
/// Most aspects of writing a [RenderBox] apply to writing a [RenderObject] as
/// well, and therefore the discussion at [RenderBox] is recommended background
/// reading. The main differences are around layout and hit testing, since those
/// are the aspects that [RenderBox] primarily specializes.
///
/// ### Layout
///
/// A layout protocol begins with a subclass of [Constraints]. See the
/// discussion at [Constraints] for more information on how to write a
/// [Constraints] subclass.
///
/// The [performLayout] method should take the [constraints], and apply them.
/// The output of the layout algorithm is fields set on the object that describe
/// the geometry of the object for the purposes of the parent's layout. For
/// example, with [RenderBox] the output is the [RenderBox.size] field. This
/// output should only be read by the parent if the parent specified
/// `parentUsesSize` as true when calling [layout] on the child.
///
/// Anytime anything changes on a render object that would affect the layout of
/// that object, it should call [markNeedsLayout].
///
/// ### Hit Testing
///
/// Hit testing is even more open-ended than layout. There is no method to
/// override, you are expected to provide one.
///
/// The general behavior of your hit-testing method should be similar to the
/// behavior described for [RenderBox]. The main difference is that the input
/// need not be an [Offset]. You are also allowed to use a different subclass of
/// [HitTestEntry] when adding entries to the [HitTestResult]. When the
/// [handleEvent] method is called, the same object that was added to the
/// [HitTestResult] will be passed in, so it can be used to track information
/// like the precise coordinate of the hit, in whatever coordinate system is
/// used by the new layout protocol.
///
/// ### Adapting from one protocol to another
///
/// In general, the root of a Flutter render object tree is a [RenderView]. This
/// object has a single child, which must be a [RenderBox]. Thus, if you want to
/// have a custom [RenderObject] subclass in the render tree, you have two
/// choices: you either need to replace the [RenderView] itself, or you need to
/// have a [RenderBox] that has your class as its child. (The latter is the much
/// more common case.)
///
/// This [RenderBox] subclass converts from the box protocol to the protocol of
/// your class.
///
/// In particular, this means that for hit testing it overrides
/// [RenderBox.hitTest], and calls whatever method you have in your class for
/// hit testing.
///
/// Similarly, it overrides [performLayout] to create a [Constraints] object
/// appropriate for your class and passes that to the child's [layout] method.
///
/// ### Layout interactions between render objects
///
/// In general, the layout of a render object should only depend on the output of
/// its child's layout, and then only if `parentUsesSize` is set to true in the
/// [layout] call. Furthermore, if it is set to true, the parent must call the
/// child's [layout] if the child is to be rendered, because otherwise the
/// parent will not be notified when the child changes its layout outputs.
///
/// It is possible to set up render object protocols that transfer additional
/// information. For example, in the [RenderBox] protocol you can query your
/// children's intrinsic dimensions and baseline geometry. However, if this is
/// done then it is imperative that the child call [markNeedsLayout] on the
/// parent any time that additional information changes, if the parent used it
/// in the last layout phase. For an example of how to implement this, see the
/// [RenderBox.markNeedsLayout] method. It overrides
/// [RenderObject.markNeedsLayout] so that if a parent has queried the intrinsic
/// or baseline information, it gets marked dirty whenever the child's geometry
/// changes.
abstract class RenderObject with DiagnosticableTreeMixin implements HitTestTarget {
  /// Initializes internal fields for subclasses.
  RenderObject() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterRenderingLibrary,
        className: '$RenderObject',
        object: this,
      );
    }
    _needsCompositing = isRepaintBoundary || alwaysNeedsCompositing;
    _wasRepaintBoundary = isRepaintBoundary;
  }

  /// Cause the entire subtree rooted at the given [RenderObject] to be marked
  /// dirty for layout, paint, etc, so that the effects of a hot reload can be
  /// seen, or so that the effect of changing a global debug flag (such as
  /// [debugPaintSizeEnabled]) can be applied.
  ///
  /// This is called by the [RendererBinding] in response to the
  /// `ext.flutter.reassemble` hook, which is used by development tools when the
  /// application code has changed, to cause the widget tree to pick up any
  /// changed implementations.
  ///
  /// This is expensive and should not be called except during development.
  ///
  /// See also:
  ///
  ///  * [BindingBase.reassembleApplication]
  void reassemble() {
    markNeedsLayout();
    markNeedsCompositingBitsUpdate();
    markNeedsPaint();
    markNeedsSemanticsUpdate();
    visitChildren((RenderObject child) {
      child.reassemble();
    });
  }

  /// Whether this has been disposed.
  ///
  /// If asserts are disabled, this property is always null.
  bool? get debugDisposed {
    bool? disposed;
    assert(() {
      disposed = _debugDisposed;
      return true;
    }());
    return disposed;
  }

  bool _debugDisposed = false;

  /// Release any resources held by this render object.
  ///
  /// The object that creates a RenderObject is in charge of disposing it.
  /// If this render object has created any children directly, it must dispose
  /// of those children in this method as well. It must not dispose of any
  /// children that were created by some other object, such as
  /// a [RenderObjectElement]. Those children will be disposed when that
  /// element unmounts, which may be delayed if the element is moved to another
  /// part of the tree.
  ///
  /// Implementations of this method must end with a call to the inherited
  /// method, as in `super.dispose()`.
  ///
  /// The object is no longer usable after calling dispose.
  @mustCallSuper
  void dispose() {
    assert(!_debugDisposed);
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _layerHandle.layer = null;
    assert(() {
      // TODO(dnfield): Enable this assert once clients have had a chance to
      // migrate.
      // visitChildren((RenderObject child) {
      //   assert(
      //     child.debugDisposed!,
      //     '${child.runtimeType} (child of $runtimeType) must be disposed before calling super.dispose().',
      //   );
      // });
      _debugDisposed = true;
      return true;
    }());
  }

  // LAYOUT

  /// Data for use by the parent render object.
  ///
  /// The parent data is used by the render object that lays out this object
  /// (typically this object's parent in the render tree) to store information
  /// relevant to itself and to any other nodes who happen to know exactly what
  /// the data means. The parent data is opaque to the child.
  ///
  ///  * The parent data field must not be directly set, except by calling
  ///    [setupParentData] on the parent node.
  ///  * The parent data can be set before the child is added to the parent, by
  ///    calling [setupParentData] on the future parent node.
  ///  * The conventions for using the parent data depend on the layout protocol
  ///    used between the parent and child. For example, in box layout, the
  ///    parent data is completely opaque but in sector layout the child is
  ///    permitted to read some fields of the parent data.
  ParentData? parentData;

  /// Override to setup parent data correctly for your children.
  ///
  /// You can call this function to set up the parent data for child before the
  /// child is added to the parent's child list.
  void setupParentData(covariant RenderObject child) {
    assert(_debugCanPerformMutations);
    if (child.parentData is! ParentData) {
      child.parentData = ParentData();
    }
  }

  /// The depth of this render object in the render tree.
  ///
  /// The depth of nodes in a tree monotonically increases as you traverse down
  /// the tree: a node always has a [depth] greater than its ancestors.
  /// There's no guarantee regarding depth between siblings.
  ///
  /// The [depth] of a child can be more than one greater than the [depth] of
  /// the parent, because the [depth] values are never decreased: all that
  /// matters is that it's greater than the parent. Consider a tree with a root
  /// node A, a child B, and a grandchild C. Initially, A will have [depth] 0,
  /// B [depth] 1, and C [depth] 2. If C is moved to be a child of A,
  /// sibling of B, then the numbers won't change. C's [depth] will still be 2.
  ///
  /// The depth of a node is used to ensure that nodes are processed in
  /// depth order.  The [depth] is automatically maintained by the [adoptChild]
  /// and [dropChild] methods.
  int get depth => _depth;
  int _depth = 0;

  /// Adjust the [depth] of the given [child] to be greater than this node's own
  /// [depth].
  ///
  /// Only call this method from overrides of [redepthChildren].
  @protected
  void redepthChild(RenderObject child) {
    assert(child.owner == owner);
    if (child._depth <= _depth) {
      child._depth = _depth + 1;
      child.redepthChildren();
    }
  }

  /// Adjust the [depth] of this node's children, if any.
  ///
  /// Override this method in subclasses with child nodes to call [redepthChild]
  /// for each child. Do not call this method directly.
  @protected
  void redepthChildren() {}

  /// The parent of this render object in the render tree.
  ///
  /// The [parent] of the root node in the render tree is null.
  RenderObject? get parent => _parent;
  RenderObject? _parent;

  /// The semantics parent of this render object in the semantics tree.
  ///
  /// This is typically the same as [parent].
  ///
  /// [OverlayPortal] overrides this field to change how it forms its
  /// semantics sub-tree.
  @visibleForOverriding
  RenderObject? get semanticsParent => _parent;

  /// Called by subclasses when they decide a render object is a child.
  ///
  /// Only for use by subclasses when changing their child lists. Calling this
  /// in other cases will lead to an inconsistent tree and probably cause crashes.
  @mustCallSuper
  @protected
  void adoptChild(RenderObject child) {
    assert(child._parent == null);
    assert(() {
      RenderObject node = this;
      while (node.parent != null) {
        node = node.parent!;
      }
      assert(node != child); // indicates we are about to create a cycle
      return true;
    }());

    setupParentData(child);
    markNeedsLayout();
    markNeedsCompositingBitsUpdate();
    markNeedsSemanticsUpdate();
    child._parent = this;
    if (attached) {
      child.attach(_owner!);
    }
    redepthChild(child);
  }

  /// Called by subclasses when they decide a render object is no longer a child.
  ///
  /// Only for use by subclasses when changing their child lists. Calling this
  /// in other cases will lead to an inconsistent tree and probably cause crashes.
  @mustCallSuper
  @protected
  void dropChild(RenderObject child) {
    assert(child._parent == this);
    assert(child.attached == attached);
    assert(child.parentData != null);
    _cleanChildRelayoutBoundary(child);
    child.parentData!.detach();
    child.parentData = null;
    child._parent = null;
    if (attached) {
      child.detach();
    }
    markNeedsLayout();
    markNeedsCompositingBitsUpdate();
    markNeedsSemanticsUpdate();
  }

  /// Calls visitor for each immediate child of this render object.
  ///
  /// Override in subclasses with children and call the visitor for each child.
  void visitChildren(RenderObjectVisitor visitor) {}

  /// The object responsible for creating this render object.
  ///
  /// Used in debug messages.
  ///
  /// See also:
  ///
  ///  * [DebugCreator], which the [widgets] library uses as values for this field.
  Object? debugCreator;

  void _reportException(String method, Object exception, StackTrace stack) {
    FlutterError.reportError(
      FlutterErrorDetails(
        exception: exception,
        stack: stack,
        library: 'rendering library',
        context: ErrorDescription('during $method()'),
        informationCollector:
            () => <DiagnosticsNode>[
              // debugCreator should always be null outside of debugMode, but we want
              // the tree shaker to notice this.
              if (kDebugMode && debugCreator != null) DiagnosticsDebugCreator(debugCreator!),
              describeForError(
                'The following RenderObject was being processed when the exception was fired',
              ),
              // TODO(jacobr): this error message has a code smell. Consider whether
              // displaying the truncated children is really useful for command line
              // users. Inspector users can see the full tree by clicking on the
              // render object so this may not be that useful.
              describeForError('RenderObject', style: DiagnosticsTreeStyle.truncateChildren),
            ],
      ),
    );
  }

  /// Whether [performResize] for this render object is currently running.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// false.
  bool get debugDoingThisResize => _debugDoingThisResize;
  bool _debugDoingThisResize = false;

  /// Whether [performLayout] for this render object is currently running.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// false.
  bool get debugDoingThisLayout => _debugDoingThisLayout;
  bool _debugDoingThisLayout = false;

  /// The render object that is actively computing layout.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// null.
  static RenderObject? get debugActiveLayout => _debugActiveLayout;
  static RenderObject? _debugActiveLayout;

  /// Set [debugActiveLayout] to null when [inner] callback is called.
  /// This is useful when you have to temporarily clear that variable to
  /// disable some false-positive checks, such as when computing toStringDeep
  /// or using custom trees.
  @pragma('dart2js:tryInline')
  @pragma('vm:prefer-inline')
  @pragma('wasm:prefer-inline')
  static T _withDebugActiveLayoutCleared<T>(T Function() inner) {
    RenderObject? debugPreviousActiveLayout;
    assert(() {
      debugPreviousActiveLayout = _debugActiveLayout;
      _debugActiveLayout = null;
      return true;
    }());
    final T result = inner();
    assert(() {
      _debugActiveLayout = debugPreviousActiveLayout;
      return true;
    }());
    return result;
  }

  /// Whether the parent render object is permitted to use this render object's
  /// size.
  ///
  /// Determined by the `parentUsesSize` parameter to [layout].
  ///
  /// Only valid when asserts are enabled. In release builds, throws.
  bool get debugCanParentUseSize => _debugCanParentUseSize!;
  bool? _debugCanParentUseSize;

  bool _debugMutationsLocked = false;

  /// Whether tree mutations are currently permitted.
  ///
  /// This is only useful during layout. One should also not mutate the tree at
  /// other times (e.g. during paint or while assembling the semantic tree) but
  /// this function does not currently enforce those conventions.
  ///
  /// Only valid when asserts are enabled. This will throw in release builds.
  bool get _debugCanPerformMutations {
    late bool result;
    assert(() {
      if (_debugDisposed) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('A disposed RenderObject was mutated.'),
          DiagnosticsProperty<RenderObject>(
            'The disposed RenderObject was',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }

      final PipelineOwner? owner = this.owner;
      // Detached nodes are allowed to mutate and the "can perform mutations"
      // check will be performed when they re-attach. This assert is only useful
      // during layout.
      if (owner == null || !owner.debugDoingLayout) {
        result = true;
        return true;
      }

      RenderObject? activeLayoutRoot = this;
      while (activeLayoutRoot != null) {
        final bool mutationsToDirtySubtreesAllowed =
            activeLayoutRoot.owner?._debugAllowMutationsToDirtySubtrees ?? false;
        final bool doingLayoutWithCallback = activeLayoutRoot._doingThisLayoutWithCallback;
        // Mutations on this subtree is allowed when:
        // - the "activeLayoutRoot" subtree is being mutated in a layout callback.
        // - a different part of the render tree is doing a layout callback,
        //   and this subtree is being reparented to that subtree, as a result
        //   of global key reparenting.
        if (doingLayoutWithCallback ||
            mutationsToDirtySubtreesAllowed && activeLayoutRoot._needsLayout) {
          result = true;
          return true;
        }

        if (!activeLayoutRoot._debugMutationsLocked) {
          activeLayoutRoot = activeLayoutRoot.debugLayoutParent;
        } else {
          // activeLayoutRoot found.
          break;
        }
      }

      final RenderObject debugActiveLayout = RenderObject.debugActiveLayout!;
      final String culpritMethodName =
          debugActiveLayout.debugDoingThisLayout ? 'performLayout' : 'performResize';
      final String culpritFullMethodName = '${debugActiveLayout.runtimeType}.$culpritMethodName';
      result = false;

      if (activeLayoutRoot == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('A $runtimeType was mutated in $culpritFullMethodName.'),
          ErrorDescription(
            'The RenderObject was mutated when none of its ancestors is actively performing layout.',
          ),
          DiagnosticsProperty<RenderObject>(
            'The RenderObject being mutated was',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          DiagnosticsProperty<RenderObject>(
            'The RenderObject that was mutating the said $runtimeType was',
            debugActiveLayout,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }

      if (activeLayoutRoot == this) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('A $runtimeType was mutated in its own $culpritMethodName implementation.'),
          ErrorDescription('A RenderObject must not re-dirty itself while still being laid out.'),
          DiagnosticsProperty<RenderObject>(
            'The RenderObject being mutated was',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          ErrorHint(
            'Consider using the LayoutBuilder widget to dynamically change a subtree during layout.',
          ),
        ]);
      }

      final ErrorSummary summary = ErrorSummary(
        'A $runtimeType was mutated in $culpritFullMethodName.',
      );
      final bool isMutatedByAncestor = activeLayoutRoot == debugActiveLayout;
      final String description =
          isMutatedByAncestor
              ? 'A RenderObject must not mutate its descendants in its $culpritMethodName method.'
              : 'A RenderObject must not mutate another RenderObject from a different render subtree '
                  'in its $culpritMethodName method.';

      throw FlutterError.fromParts(<DiagnosticsNode>[
        summary,
        ErrorDescription(description),
        DiagnosticsProperty<RenderObject>(
          'The RenderObject being mutated was',
          this,
          style: DiagnosticsTreeStyle.errorProperty,
        ),
        DiagnosticsProperty<RenderObject>(
          'The ${isMutatedByAncestor ? 'ancestor ' : ''}RenderObject that was mutating the said $runtimeType was',
          debugActiveLayout,
          style: DiagnosticsTreeStyle.errorProperty,
        ),
        if (!isMutatedByAncestor)
          DiagnosticsProperty<RenderObject>(
            'Their common ancestor was',
            activeLayoutRoot,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ErrorHint(
          'Mutating the layout of another RenderObject may cause some RenderObjects in its subtree to be laid out more than once. '
          'Consider using the LayoutBuilder widget to dynamically mutate a subtree during layout.',
        ),
      ]);
    }());
    return result;
  }

  /// The [RenderObject] that's expected to call [layout] on this [RenderObject]
  /// in its [performLayout] implementation.
  ///
  /// This method is used to implement an assert that ensures the render subtree
  /// actively performing layout can not get accidentally mutated. It's only
  /// implemented in debug mode and always returns null in release mode.
  ///
  /// The default implementation returns [parent] and overriding is rarely
  /// needed. A [RenderObject] subclass that expects its
  /// [RenderObject.performLayout] to be called from a different [RenderObject]
  /// that's not its [parent] should override this property to return the actual
  /// layout parent.
  @protected
  RenderObject? get debugLayoutParent {
    RenderObject? layoutParent;
    assert(() {
      layoutParent = parent;
      return true;
    }());
    return layoutParent;
  }

  /// The owner for this render object (null if unattached).
  ///
  /// The entire render tree that this render object belongs to
  /// will have the same owner.
  PipelineOwner? get owner => _owner;
  PipelineOwner? _owner;

  /// Whether the render tree this render object belongs to is attached to a [PipelineOwner].
  ///
  /// This becomes true during the call to [attach].
  ///
  /// This becomes false during the call to [detach].
  bool get attached => _owner != null;

  /// Mark this render object as attached to the given owner.
  ///
  /// Typically called only from the [parent]'s [attach] method, and by the
  /// [owner] to mark the root of a tree as attached.
  ///
  /// Subclasses with children should override this method to
  /// [attach] all their children to the same [owner]
  /// after calling the inherited method, as in `super.attach(owner)`.
  @mustCallSuper
  void attach(PipelineOwner owner) {
    assert(!_debugDisposed);
    assert(_owner == null);
    _owner = owner;
    // If the node was dirtied in some way while unattached, make sure to add
    // it to the appropriate dirty list now that an owner is available
    if (_needsLayout && _relayoutBoundary != null) {
      // Don't enter this block if we've never laid out at all;
      // scheduleInitialLayout() will handle it
      _needsLayout = false;
      markNeedsLayout();
    }
    if (_needsCompositingBitsUpdate) {
      _needsCompositingBitsUpdate = false;
      markNeedsCompositingBitsUpdate();
    }
    if (_needsPaint && _layerHandle.layer != null) {
      // Don't enter this block if we've never painted at all;
      // scheduleInitialPaint() will handle it
      _needsPaint = false;
      markNeedsPaint();
    }
    if (_needsSemanticsUpdate && _semanticsConfiguration.isSemanticBoundary) {
      // Don't enter this block if we've never updated semantics at all;
      // scheduleInitialSemantics() will handle it
      _needsSemanticsUpdate = false;
      markNeedsSemanticsUpdate();
    }
  }

  /// Mark this render object as detached from its [PipelineOwner].
  ///
  /// Typically called only from the [parent]'s [detach], and by the [owner] to
  /// mark the root of a tree as detached.
  ///
  /// Subclasses with children should override this method to
  /// [detach] all their children after calling the inherited method,
  /// as in `super.detach()`.
  @mustCallSuper
  void detach() {
    assert(_owner != null);
    _owner = null;
    assert(parent == null || attached == parent!.attached);
  }

  /// Whether this render object's layout information is dirty.
  ///
  /// This is only set in debug mode. In general, render objects should not need
  /// to condition their runtime behavior on whether they are dirty or not,
  /// since they should only be marked dirty immediately prior to being laid
  /// out and painted. In release builds, this throws.
  ///
  /// It is intended to be used by tests and asserts.
  bool get debugNeedsLayout {
    late bool result;
    assert(() {
      result = _needsLayout;
      return true;
    }());
    return result;
  }

  bool _needsLayout = true;

  /// The nearest relayout boundary enclosing this render object, if known.
  ///
  /// When a render object is marked as needing layout, its parent may
  /// as a result also need to be marked as needing layout.
  /// For details, see [markNeedsLayout].
  /// A render object where relayout does not require relayout of the parent
  /// (because its size cannot change on relayout, or because
  /// its parent does not use the child's size for its own layout)
  /// is a "relayout boundary".
  ///
  /// This property is set in [layout], and consulted by [markNeedsLayout] in
  /// deciding whether to recursively mark the parent as also needing layout.
  ///
  /// This property is initially null, and becomes null again if this
  /// render object is removed from the tree (with [dropChild]);
  /// it remains null until the first layout of this render object
  /// after it was most recently added to the tree.
  /// This property can also be null while an ancestor in the tree is
  /// currently doing layout, until this render object itself does layout.
  ///
  /// When not null, the relayout boundary is either this render object itself
  /// or one of its ancestors, and all the render objects in the ancestry chain
  /// up through that ancestor have the same [_relayoutBoundary].
  /// Equivalently: when not null, the relayout boundary is either this render
  /// object itself or the same as that of its parent.  (So [_relayoutBoundary]
  /// is one of `null`, `this`, or `parent!._relayoutBoundary!`.)
  RenderObject? _relayoutBoundary;

  /// Whether [invokeLayoutCallback] for this render object is currently running.
  bool get debugDoingThisLayoutWithCallback => _doingThisLayoutWithCallback;
  bool _doingThisLayoutWithCallback = false;

  /// The layout constraints most recently supplied by the parent.
  ///
  /// If layout has not yet happened, accessing this getter will
  /// throw a [StateError] exception.
  @protected
  Constraints get constraints {
    if (_constraints == null) {
      throw StateError('A RenderObject does not have any constraints before it has been laid out.');
    }
    return _constraints!;
  }

  Constraints? _constraints;

  /// Verify that the object's constraints are being met. Override this function
  /// in a subclass to verify that your state matches the constraints object.
  /// This function is only called when asserts are enabled (i.e. in debug mode)
  /// and only when needsLayout is false. If the constraints are not met, it
  /// should assert or throw an exception.
  @protected
  void debugAssertDoesMeetConstraints();

  /// When true, a debug method ([debugAssertDoesMeetConstraints], for instance)
  /// is currently executing asserts for verifying the consistent behavior of
  /// intrinsic dimensions methods.
  ///
  /// This is typically set by framework debug methods. It is read by tests to
  /// selectively ignore custom layout callbacks. It should not be set outside of
  /// intrinsic-checking debug methods, and should not be checked in release mode
  /// (where it will always be false).
  static bool debugCheckingIntrinsics = false;

  bool _debugRelayoutBoundaryAlreadyMarkedNeedsLayout() {
    if (_relayoutBoundary == null) {
      // We don't know where our relayout boundary is yet.
      return true;
    }
    RenderObject node = this;
    while (node != _relayoutBoundary) {
      assert(node._relayoutBoundary == _relayoutBoundary);
      assert(node.parent != null);
      node = node.parent!;
      if ((!node._needsLayout) && (!node._debugDoingThisLayout)) {
        return false;
      }
    }
    assert(node._relayoutBoundary == node);
    return true;
  }

  /// Mark this render object's layout information as dirty, and either register
  /// this object with its [PipelineOwner], or defer to the parent, depending on
  /// whether this object is a relayout boundary or not respectively.
  ///
  /// ## Background
  ///
  /// Rather than eagerly updating layout information in response to writes into
  /// a render object, we instead mark the layout information as dirty, which
  /// schedules a visual update. As part of the visual update, the rendering
  /// pipeline updates the render object's layout information.
  ///
  /// This mechanism batches the layout work so that multiple sequential writes
  /// are coalesced, removing redundant computation.
  ///
  /// If a render object's parent indicates that it uses the size of one of its
  /// render object children when computing its layout information, this
  /// function, when called for the child, will also mark the parent as needing
  /// layout. In that case, since both the parent and the child need to have
  /// their layout recomputed, the pipeline owner is only notified about the
  /// parent; when the parent is laid out, it will call the child's [layout]
  /// method and thus the child will be laid out as well.
  ///
  /// Once [markNeedsLayout] has been called on a render object,
  /// [debugNeedsLayout] returns true for that render object until just after
  /// the pipeline owner has called [layout] on the render object.
  ///
  /// ## Special cases
  ///
  /// Some subclasses of [RenderObject], notably [RenderBox], have other
  /// situations in which the parent needs to be notified if the child is
  /// dirtied (e.g., if the child's intrinsic dimensions or baseline changes).
  /// Such subclasses override markNeedsLayout and either call
  /// `super.markNeedsLayout()`, in the normal case, or call
  /// [markParentNeedsLayout], in the case where the parent needs to be laid out
  /// as well as the child.
  ///
  /// If [sizedByParent] has changed, calls
  /// [markNeedsLayoutForSizedByParentChange] instead of [markNeedsLayout].
  void markNeedsLayout() {
    assert(_debugCanPerformMutations);
    if (_needsLayout) {
      assert(_debugRelayoutBoundaryAlreadyMarkedNeedsLayout());
      return;
    }
    if (_relayoutBoundary == null) {
      _needsLayout = true;
      if (parent != null) {
        // _relayoutBoundary is cleaned by an ancestor in RenderObject.layout.
        // Conservatively mark everything dirty until it reaches the closest
        // known relayout boundary.
        markParentNeedsLayout();
      }
      return;
    }
    if (_relayoutBoundary != this) {
      markParentNeedsLayout();
    } else {
      _needsLayout = true;
      if (owner != null) {
        assert(() {
          if (debugPrintMarkNeedsLayoutStacks) {
            debugPrintStack(label: 'markNeedsLayout() called for $this');
          }
          return true;
        }());
        owner!._nodesNeedingLayout.add(this);
        owner!.requestVisualUpdate();
      }
    }
  }

  /// Mark this render object's layout information as dirty, and then defer to
  /// the parent.
  ///
  /// This function should only be called from [markNeedsLayout] or
  /// [markNeedsLayoutForSizedByParentChange] implementations of subclasses that
  /// introduce more reasons for deferring the handling of dirty layout to the
  /// parent. See [markNeedsLayout] for details.
  ///
  /// Only call this if [parent] is not null.
  @protected
  void markParentNeedsLayout() {
    assert(_debugCanPerformMutations);
    _needsLayout = true;
    assert(this.parent != null);
    final RenderObject parent = this.parent!;
    if (!_doingThisLayoutWithCallback) {
      parent.markNeedsLayout();
    } else {
      assert(parent._debugDoingThisLayout);
    }
    assert(parent == this.parent);
  }

  /// Mark this render object's layout information as dirty (like
  /// [markNeedsLayout]), and additionally also handle any necessary work to
  /// handle the case where [sizedByParent] has changed value.
  ///
  /// This should be called whenever [sizedByParent] might have changed.
  ///
  /// Only call this if [parent] is not null.
  void markNeedsLayoutForSizedByParentChange() {
    markNeedsLayout();
    markParentNeedsLayout();
  }

  /// Set [_relayoutBoundary] to null throughout this render object's subtree,
  /// stopping at relayout boundaries.
  // This is a static method to reduce closure allocation with visitChildren.
  static void _cleanChildRelayoutBoundary(RenderObject child) {
    if (child._relayoutBoundary != child) {
      child.visitChildren(_cleanChildRelayoutBoundary);
      child._relayoutBoundary = null;
    }
  }

  // This is a static method to reduce closure allocation with visitChildren.
  static void _propagateRelayoutBoundaryToChild(RenderObject child) {
    if (child._relayoutBoundary == child) {
      return;
    }
    final RenderObject? parentRelayoutBoundary = child.parent?._relayoutBoundary;
    assert(parentRelayoutBoundary != null);
    assert(parentRelayoutBoundary != child._relayoutBoundary);
    child._setRelayoutBoundary(parentRelayoutBoundary!);
  }

  /// Set [_relayoutBoundary] to [value] throughout this render object's
  /// subtree, including this render object but stopping at relayout boundaries
  /// thereafter.
  void _setRelayoutBoundary(RenderObject value) {
    assert(value != _relayoutBoundary);
    // This may temporarily break the _relayoutBoundary invariant at children;
    // the visitChildren restores the invariant.
    _relayoutBoundary = value;
    visitChildren(_propagateRelayoutBoundaryToChild);
  }

  /// Bootstrap the rendering pipeline by scheduling the very first layout.
  ///
  /// Requires this render object to be attached and that this render object
  /// is the root of the render tree.
  ///
  /// See [RenderView] for an example of how this function is used.
  void scheduleInitialLayout() {
    assert(!_debugDisposed);
    assert(attached);
    assert(parent is! RenderObject);
    assert(!owner!._debugDoingLayout);
    assert(_relayoutBoundary == null);
    _relayoutBoundary = this;
    assert(() {
      _debugCanParentUseSize = false;
      return true;
    }());
    owner!._nodesNeedingLayout.add(this);
  }

  @pragma('vm:notify-debugger-on-exception')
  void _layoutWithoutResize() {
    assert(_needsLayout);
    assert(_relayoutBoundary == this);
    RenderObject? debugPreviousActiveLayout;
    assert(!_debugMutationsLocked);
    assert(!_doingThisLayoutWithCallback);
    assert(_debugCanParentUseSize != null);
    assert(() {
      _debugMutationsLocked = true;
      _debugDoingThisLayout = true;
      debugPreviousActiveLayout = _debugActiveLayout;
      _debugActiveLayout = this;
      if (debugPrintLayouts) {
        debugPrint('Laying out (without resize) $this');
      }
      return true;
    }());
    try {
      performLayout();
      markNeedsSemanticsUpdate();
    } catch (e, stack) {
      _reportException('performLayout', e, stack);
    }
    assert(() {
      _debugActiveLayout = debugPreviousActiveLayout;
      _debugDoingThisLayout = false;
      _debugMutationsLocked = false;
      return true;
    }());
    _needsLayout = false;
    markNeedsPaint();
  }

  /// Compute the layout for this render object.
  ///
  /// This method is the main entry point for parents to ask their children to
  /// update their layout information. The parent passes a constraints object,
  /// which informs the child as to which layouts are permissible. The child is
  /// required to obey the given constraints.
  ///
  /// If the parent reads information computed during the child's layout, the
  /// parent must pass true for `parentUsesSize`. In that case, the parent will
  /// be marked as needing layout whenever the child is marked as needing layout
  /// because the parent's layout information depends on the child's layout
  /// information. If the parent uses the default value (false) for
  /// `parentUsesSize`, the child can change its layout information (subject to
  /// the given constraints) without informing the parent.
  ///
  /// Subclasses should not override [layout] directly. Instead, they should
  /// override [performResize] and/or [performLayout]. The [layout] method
  /// delegates the actual work to [performResize] and [performLayout].
  ///
  /// The parent's [performLayout] method should call the [layout] of all its
  /// children unconditionally. It is the [layout] method's responsibility (as
  /// implemented here) to return early if the child does not need to do any
  /// work to update its layout information.
  @pragma('vm:notify-debugger-on-exception')
  void layout(Constraints constraints, {bool parentUsesSize = false}) {
    assert(!_debugDisposed);
    if (!kReleaseMode && debugProfileLayoutsEnabled) {
      Map<String, String>? debugTimelineArguments;
      assert(() {
        if (debugEnhanceLayoutTimelineArguments) {
          debugTimelineArguments = toDiagnosticsNode().toTimelineArguments();
        }
        return true;
      }());
      FlutterTimeline.startSync('$runtimeType', arguments: debugTimelineArguments);
    }
    assert(
      constraints.debugAssertIsValid(
        isAppliedConstraint: true,
        informationCollector: () {
          final List<String> stack = StackTrace.current.toString().split('\n');
          int? targetFrame;
          final Pattern layoutFramePattern = RegExp(r'^#[0-9]+ +Render(?:Object|Box).layout \(');
          for (int i = 0; i < stack.length; i += 1) {
            if (layoutFramePattern.matchAsPrefix(stack[i]) != null) {
              targetFrame = i + 1;
            } else if (targetFrame != null) {
              break;
            }
          }
          if (targetFrame != null && targetFrame < stack.length) {
            final Pattern targetFramePattern = RegExp(r'^#[0-9]+ +(.+)$');
            final Match? targetFrameMatch = targetFramePattern.matchAsPrefix(stack[targetFrame]);
            final String? problemFunction =
                (targetFrameMatch != null && targetFrameMatch.groupCount > 0)
                    ? targetFrameMatch.group(1)
                    : stack[targetFrame].trim();
            return <DiagnosticsNode>[
              ErrorDescription(
                "These invalid constraints were provided to $runtimeType's layout() "
                'function by the following function, which probably computed the '
                'invalid constraints in question:\n'
                '  $problemFunction',
              ),
            ];
          }
          return <DiagnosticsNode>[];
        },
      ),
    );
    assert(!_debugDoingThisResize);
    assert(!_debugDoingThisLayout);
    final bool isRelayoutBoundary =
        !parentUsesSize || sizedByParent || constraints.isTight || parent is! RenderObject;
    final RenderObject relayoutBoundary = isRelayoutBoundary ? this : parent!._relayoutBoundary!;
    assert(() {
      _debugCanParentUseSize = parentUsesSize;
      return true;
    }());

    if (!_needsLayout && constraints == _constraints) {
      assert(() {
        // in case parentUsesSize changed since the last invocation, set size
        // to itself, so it has the right internal debug values.
        _debugDoingThisResize = sizedByParent;
        _debugDoingThisLayout = !sizedByParent;
        final RenderObject? debugPreviousActiveLayout = _debugActiveLayout;
        _debugActiveLayout = this;
        debugResetSize();
        _debugActiveLayout = debugPreviousActiveLayout;
        _debugDoingThisLayout = false;
        _debugDoingThisResize = false;
        return true;
      }());

      if (relayoutBoundary != _relayoutBoundary) {
        _setRelayoutBoundary(relayoutBoundary);
      }

      if (!kReleaseMode && debugProfileLayoutsEnabled) {
        FlutterTimeline.finishSync();
      }
      return;
    }
    _constraints = constraints;

    if (_relayoutBoundary != null && relayoutBoundary != _relayoutBoundary) {
      // The local relayout boundary has changed, must notify children in case
      // they also need updating. Otherwise, they will be confused about what
      // their actual relayout boundary is later.
      visitChildren(_cleanChildRelayoutBoundary);
    }
    _relayoutBoundary = relayoutBoundary;

    assert(!_debugMutationsLocked);
    assert(!_doingThisLayoutWithCallback);
    assert(() {
      _debugMutationsLocked = true;
      if (debugPrintLayouts) {
        debugPrint(
          'Laying out (${sizedByParent ? "with separate resize" : "with resize allowed"}) $this',
        );
      }
      return true;
    }());
    if (sizedByParent) {
      assert(() {
        _debugDoingThisResize = true;
        return true;
      }());
      try {
        performResize();
        assert(() {
          debugAssertDoesMeetConstraints();
          return true;
        }());
      } catch (e, stack) {
        _reportException('performResize', e, stack);
      }
      assert(() {
        _debugDoingThisResize = false;
        return true;
      }());
    }
    RenderObject? debugPreviousActiveLayout;
    assert(() {
      _debugDoingThisLayout = true;
      debugPreviousActiveLayout = _debugActiveLayout;
      _debugActiveLayout = this;
      return true;
    }());
    try {
      performLayout();
      markNeedsSemanticsUpdate();
      assert(() {
        debugAssertDoesMeetConstraints();
        return true;
      }());
    } catch (e, stack) {
      _reportException('performLayout', e, stack);
    }
    assert(() {
      _debugActiveLayout = debugPreviousActiveLayout;
      _debugDoingThisLayout = false;
      _debugMutationsLocked = false;
      return true;
    }());
    _needsLayout = false;
    markNeedsPaint();

    if (!kReleaseMode && debugProfileLayoutsEnabled) {
      FlutterTimeline.finishSync();
    }
  }

  /// If a subclass has a "size" (the state controlled by `parentUsesSize`,
  /// whatever it is in the subclass, e.g. the actual `size` property of
  /// [RenderBox]), and the subclass verifies that in debug mode this "size"
  /// property isn't used when [debugCanParentUseSize] isn't set, then that
  /// subclass should override [debugResetSize] to reapply the current values of
  /// [debugCanParentUseSize] to that state.
  @protected
  void debugResetSize() {}

  /// Whether the constraints are the only input to the sizing algorithm (in
  /// particular, child nodes have no impact).
  ///
  /// Returning false is always correct, but returning true can be more
  /// efficient when computing the size of this render object because we don't
  /// need to recompute the size if the constraints don't change.
  ///
  /// Typically, subclasses will always return the same value. If the value can
  /// change, then, when it does change, the subclass should make sure to call
  /// [markNeedsLayoutForSizedByParentChange].
  ///
  /// Subclasses that return true must not change the dimensions of this render
  /// object in [performLayout]. Instead, that work should be done by
  /// [performResize] or - for subclasses of [RenderBox] - in
  /// [RenderBox.computeDryLayout].
  @protected
  bool get sizedByParent => false;

  /// {@template flutter.rendering.RenderObject.performResize}
  /// Updates the render objects size using only the constraints.
  ///
  /// Do not call this function directly: call [layout] instead. This function
  /// is called by [layout] when there is actually work to be done by this
  /// render object during layout. The layout constraints provided by your
  /// parent are available via the [constraints] getter.
  ///
  /// This function is called only if [sizedByParent] is true.
  /// {@endtemplate}
  ///
  /// Subclasses that set [sizedByParent] to true should override this method to
  /// compute their size. Subclasses of [RenderBox] should consider overriding
  /// [RenderBox.computeDryLayout] instead.
  @protected
  void performResize();

  /// Do the work of computing the layout for this render object.
  ///
  /// Do not call this function directly: call [layout] instead. This function
  /// is called by [layout] when there is actually work to be done by this
  /// render object during layout. The layout constraints provided by your
  /// parent are available via the [constraints] getter.
  ///
  /// If [sizedByParent] is true, then this function should not actually change
  /// the dimensions of this render object. Instead, that work should be done by
  /// [performResize]. If [sizedByParent] is false, then this function should
  /// both change the dimensions of this render object and instruct its children
  /// to layout.
  ///
  /// In implementing this function, you must call [layout] on each of your
  /// children, passing true for parentUsesSize if your layout information is
  /// dependent on your child's layout information. Passing true for
  /// parentUsesSize ensures that this render object will undergo layout if the
  /// child undergoes layout. Otherwise, the child can change its layout
  /// information without informing this render object.
  @protected
  void performLayout();

  /// Allows mutations to be made to this object's child list (and any
  /// descendants) as well as to any other dirty nodes in the render tree owned
  /// by the same [PipelineOwner] as this object. The `callback` argument is
  /// invoked synchronously, and the mutations are allowed only during that
  /// callback's execution.
  ///
  /// This exists to allow child lists to be built on-demand during layout (e.g.
  /// based on the object's size), and to enable nodes to be moved around the
  /// tree as this happens (e.g. to handle [GlobalKey] reparenting), while still
  /// ensuring that any particular node is only laid out once per frame.
  ///
  /// Calling this function disables a number of assertions that are intended to
  /// catch likely bugs. As such, using this function is generally discouraged.
  ///
  /// This function can only be called during layout.
  @protected
  void invokeLayoutCallback<T extends Constraints>(LayoutCallback<T> callback) {
    assert(_debugMutationsLocked);
    assert(_debugDoingThisLayout);
    assert(!_doingThisLayoutWithCallback);
    _doingThisLayoutWithCallback = true;
    try {
      owner!._enableMutationsToDirtySubtrees(() {
        callback(constraints as T);
      });
    } finally {
      _doingThisLayoutWithCallback = false;
    }
  }

  // PAINTING

  /// Whether [paint] for this render object is currently running.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// false.
  bool get debugDoingThisPaint => _debugDoingThisPaint;
  bool _debugDoingThisPaint = false;

  /// The render object that is actively painting.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// null.
  static RenderObject? get debugActivePaint => _debugActivePaint;
  static RenderObject? _debugActivePaint;

  /// Whether this render object repaints separately from its parent.
  ///
  /// Override this in subclasses to indicate that instances of your class ought
  /// to repaint independently. For example, render objects that repaint
  /// frequently might want to repaint themselves without requiring their parent
  /// to repaint.
  ///
  /// If this getter returns true, the [paintBounds] are applied to this object
  /// and all descendants. The framework invokes [RenderObject.updateCompositedLayer]
  /// to create an [OffsetLayer] and assigns it to the [layer] field.
  /// Render objects that declare themselves as repaint boundaries must not replace
  /// the layer created by the framework.
  ///
  /// If the value of this getter changes, [markNeedsCompositingBitsUpdate] must
  /// be called.
  ///
  /// See [RepaintBoundary] for more information about how repaint boundaries function.
  bool get isRepaintBoundary => false;

  /// Called, in debug mode, if [isRepaintBoundary] is true, when either the
  /// this render object or its parent attempt to paint.
  ///
  /// This can be used to record metrics about whether the node should actually
  /// be a repaint boundary.
  void debugRegisterRepaintBoundaryPaint({
    bool includedParent = true,
    bool includedChild = false,
  }) {}

  /// Whether this render object always needs compositing.
  ///
  /// Override this in subclasses to indicate that your paint function always
  /// creates at least one composited layer. For example, videos should return
  /// true if they use hardware decoders.
  ///
  /// You must call [markNeedsCompositingBitsUpdate] if the value of this getter
  /// changes. (This is implied when [adoptChild] or [dropChild] are called.)
  @protected
  bool get alwaysNeedsCompositing => false;

  late bool _wasRepaintBoundary;

  /// Update the composited layer owned by this render object.
  ///
  /// This method is called by the framework when [isRepaintBoundary] is true.
  ///
  /// If [oldLayer] is `null`, this method must return a new [OffsetLayer]
  /// (or subtype thereof). If [oldLayer] is not `null`, then this method must
  /// reuse the layer instance that is provided - it is an error to create a new
  /// layer in this instance. The layer will be disposed by the framework when
  /// either the render object is disposed or if it is no longer a repaint
  /// boundary.
  ///
  /// The [OffsetLayer.offset] property will be managed by the framework and
  /// must not be updated by this method.
  ///
  /// If a property of the composited layer needs to be updated, the render object
  /// must call [markNeedsCompositedLayerUpdate] which will schedule this method
  /// to be called without repainting children. If this widget was marked as
  /// needing to paint and needing a composited layer update, this method is only
  /// called once.
  // TODO(jonahwilliams): https://github.com/flutter/flutter/issues/102102 revisit the
  // constraint that the instance/type of layer cannot be changed at runtime.
  OffsetLayer updateCompositedLayer({required covariant OffsetLayer? oldLayer}) {
    assert(isRepaintBoundary);
    return oldLayer ?? OffsetLayer();
  }

  /// The compositing layer that this render object uses to repaint.
  ///
  /// If this render object is not a repaint boundary, it is the responsibility
  /// of the [paint] method to populate this field. If [needsCompositing] is
  /// true, this field may be populated with the root-most layer used by the
  /// render object implementation. When repainting, instead of creating a new
  /// layer the render object may update the layer stored in this field for better
  /// performance. It is also OK to leave this field as null and create a new
  /// layer on every repaint, but without the performance benefit. If
  /// [needsCompositing] is false, this field must be set to null either by
  /// never populating this field, or by setting it to null when the value of
  /// [needsCompositing] changes from true to false.
  ///
  /// If a new layer is created and stored in some other field on the render
  /// object, the render object must use a [LayerHandle] to store it. A layer
  /// handle will prevent the layer from being disposed before the render
  /// object is finished with it, and it will also make sure that the layer
  /// gets appropriately disposed when the render object creates a replacement
  /// or nulls it out. The render object must null out the [LayerHandle.layer]
  /// in its [dispose] method.
  ///
  /// If this render object is a repaint boundary, the framework automatically
  /// creates an [OffsetLayer] and populates this field prior to calling the
  /// [paint] method. The [paint] method must not replace the value of this
  /// field.
  @protected
  ContainerLayer? get layer {
    assert(!isRepaintBoundary || _layerHandle.layer == null || _layerHandle.layer is OffsetLayer);
    return _layerHandle.layer;
  }

  @protected
  set layer(ContainerLayer? newLayer) {
    assert(
      !isRepaintBoundary,
      'Attempted to set a layer to a repaint boundary render object.\n'
      'The framework creates and assigns an OffsetLayer to a repaint '
      'boundary automatically.',
    );
    _layerHandle.layer = newLayer;
  }

  final LayerHandle<ContainerLayer> _layerHandle = LayerHandle<ContainerLayer>();

  /// In debug mode, the compositing layer that this render object uses to repaint.
  ///
  /// This getter is intended for debugging purposes only. In release builds, it
  /// always returns null. In debug builds, it returns the layer even if the layer
  /// is dirty.
  ///
  /// For production code, consider [layer].
  ContainerLayer? get debugLayer {
    ContainerLayer? result;
    assert(() {
      result = _layerHandle.layer;
      return true;
    }());
    return result;
  }

  bool _needsCompositingBitsUpdate = false; // set to true when a child is added
  /// Mark the compositing state for this render object as dirty.
  ///
  /// This is called to indicate that the value for [needsCompositing] needs to
  /// be recomputed during the next [PipelineOwner.flushCompositingBits] engine
  /// phase.
  ///
  /// When the subtree is mutated, we need to recompute our
  /// [needsCompositing] bit, and some of our ancestors need to do the
  /// same (in case ours changed in a way that will change theirs). To
  /// this end, [adoptChild] and [dropChild] call this method, and, as
  /// necessary, this method calls the parent's, etc, walking up the
  /// tree to mark all the nodes that need updating.
  ///
  /// This method does not schedule a rendering frame, because since
  /// it cannot be the case that _only_ the compositing bits changed,
  /// something else will have scheduled a frame for us.
  void markNeedsCompositingBitsUpdate() {
    assert(!_debugDisposed);
    if (_needsCompositingBitsUpdate) {
      return;
    }
    _needsCompositingBitsUpdate = true;
    if (parent is RenderObject) {
      final RenderObject parent = this.parent!;
      if (parent._needsCompositingBitsUpdate) {
        return;
      }

      if ((!_wasRepaintBoundary || !isRepaintBoundary) && !parent.isRepaintBoundary) {
        parent.markNeedsCompositingBitsUpdate();
        return;
      }
    }
    // parent is fine (or there isn't one), but we are dirty
    if (owner != null) {
      owner!._nodesNeedingCompositingBitsUpdate.add(this);
    }
  }

  late bool _needsCompositing; // initialized in the constructor
  /// Whether we or one of our descendants has a compositing layer.
  ///
  /// If this node needs compositing as indicated by this bit, then all ancestor
  /// nodes will also need compositing.
  ///
  /// Only legal to call after [PipelineOwner.flushLayout] and
  /// [PipelineOwner.flushCompositingBits] have been called.
  bool get needsCompositing {
    assert(!_needsCompositingBitsUpdate); // make sure we don't use this bit when it is dirty
    return _needsCompositing;
  }

  void _updateCompositingBits() {
    if (!_needsCompositingBitsUpdate) {
      return;
    }
    final bool oldNeedsCompositing = _needsCompositing;
    _needsCompositing = false;
    visitChildren((RenderObject child) {
      child._updateCompositingBits();
      if (child.needsCompositing) {
        _needsCompositing = true;
      }
    });
    if (isRepaintBoundary || alwaysNeedsCompositing) {
      _needsCompositing = true;
    }
    // If a node was previously a repaint boundary, but no longer is one, then
    // regardless of its compositing state we need to find a new parent to
    // paint from. To do this, we mark it clean again so that the traversal
    // in markNeedsPaint is not short-circuited. It is removed from _nodesNeedingPaint
    // so that we do not attempt to paint from it after locating a parent.
    if (!isRepaintBoundary && _wasRepaintBoundary) {
      _needsPaint = false;
      _needsCompositedLayerUpdate = false;
      owner?._nodesNeedingPaint.removeWhere((RenderObject t) => identical(t, this));
      _needsCompositingBitsUpdate = false;
      markNeedsPaint();
    } else if (oldNeedsCompositing != _needsCompositing) {
      _needsCompositingBitsUpdate = false;
      markNeedsPaint();
    } else {
      _needsCompositingBitsUpdate = false;
    }
  }

  /// Whether this render object's paint information is dirty.
  ///
  /// This is only set in debug mode. In general, render objects should not need
  /// to condition their runtime behavior on whether they are dirty or not,
  /// since they should only be marked dirty immediately prior to being laid
  /// out and painted. (In release builds, this throws.)
  ///
  /// It is intended to be used by tests and asserts.
  ///
  /// It is possible (and indeed, quite common) for [debugNeedsPaint] to be
  /// false and [debugNeedsLayout] to be true. The render object will still be
  /// repainted in the next frame when this is the case, because the
  /// [markNeedsPaint] method is implicitly called by the framework after a
  /// render object is laid out, prior to the paint phase.
  bool get debugNeedsPaint {
    late bool result;
    assert(() {
      result = _needsPaint;
      return true;
    }());
    return result;
  }

  bool _needsPaint = true;

  /// Whether this render object's layer information is dirty.
  ///
  /// This is only set in debug mode. In general, render objects should not need
  /// to condition their runtime behavior on whether they are dirty or not,
  /// since they should only be marked dirty immediately prior to being laid
  /// out and painted. (In release builds, this throws.)
  ///
  /// It is intended to be used by tests and asserts.
  bool get debugNeedsCompositedLayerUpdate {
    late bool result;
    assert(() {
      result = _needsCompositedLayerUpdate;
      return true;
    }());
    return result;
  }

  bool _needsCompositedLayerUpdate = false;

  /// Mark this render object as having changed its visual appearance.
  ///
  /// Rather than eagerly updating this render object's display list
  /// in response to writes, we instead mark the render object as needing to
  /// paint, which schedules a visual update. As part of the visual update, the
  /// rendering pipeline will give this render object an opportunity to update
  /// its display list.
  ///
  /// This mechanism batches the painting work so that multiple sequential
  /// writes are coalesced, removing redundant computation.
  ///
  /// Once [markNeedsPaint] has been called on a render object,
  /// [debugNeedsPaint] returns true for that render object until just after
  /// the pipeline owner has called [paint] on the render object.
  ///
  /// See also:
  ///
  ///  * [RepaintBoundary], to scope a subtree of render objects to their own
  ///    layer, thus limiting the number of nodes that [markNeedsPaint] must mark
  ///    dirty.
  void markNeedsPaint() {
    assert(!_debugDisposed);
    assert(owner == null || !owner!.debugDoingPaint);
    if (_needsPaint) {
      return;
    }
    _needsPaint = true;
    // If this was not previously a repaint boundary it will not have
    // a layer we can paint from.
    if (isRepaintBoundary && _wasRepaintBoundary) {
      assert(() {
        if (debugPrintMarkNeedsPaintStacks) {
          debugPrintStack(label: 'markNeedsPaint() called for $this');
        }
        return true;
      }());
      // If we always have our own layer, then we can just repaint
      // ourselves without involving any other nodes.
      assert(_layerHandle.layer is OffsetLayer);
      if (owner != null) {
        owner!._nodesNeedingPaint.add(this);
        owner!.requestVisualUpdate();
      }
    } else if (parent != null) {
      parent!.markNeedsPaint();
    } else {
      assert(() {
        if (debugPrintMarkNeedsPaintStacks) {
          debugPrintStack(label: 'markNeedsPaint() called for $this (root of render tree)');
        }
        return true;
      }());
      // If we are the root of the render tree and not a repaint boundary
      // then we have to paint ourselves, since nobody else can paint us.
      // We don't add ourselves to _nodesNeedingPaint in this case,
      // because the root is always told to paint regardless.
      //
      // Trees rooted at a RenderView do not go through this
      // code path because RenderViews are repaint boundaries.
      owner?.requestVisualUpdate();
    }
  }

  /// Mark this render object as having changed a property on its composited
  /// layer.
  ///
  /// Render objects that have a composited layer have [isRepaintBoundary] equal
  /// to true may update the properties of that composited layer without repainting
  /// their children. If this render object is a repaint boundary but does
  /// not yet have a composited layer created for it, this method will instead
  /// mark the nearest repaint boundary parent as needing to be painted.
  ///
  /// If this method is called on a render object that is not a repaint boundary
  /// or is a repaint boundary but hasn't been composited yet, it is equivalent
  /// to calling [markNeedsPaint].
  ///
  /// See also:
  ///
  ///  * [RenderOpacity], which uses this method when its opacity is updated to
  ///    update the layer opacity without repainting children.
  void markNeedsCompositedLayerUpdate() {
    assert(!_debugDisposed);
    assert(owner == null || !owner!.debugDoingPaint);
    if (_needsCompositedLayerUpdate || _needsPaint) {
      return;
    }
    _needsCompositedLayerUpdate = true;
    // If this was not previously a repaint boundary it will not have
    // a layer we can paint from.
    if (isRepaintBoundary && _wasRepaintBoundary) {
      // If we always have our own layer, then we can just repaint
      // ourselves without involving any other nodes.
      assert(_layerHandle.layer != null);
      if (owner != null) {
        owner!._nodesNeedingPaint.add(this);
        owner!.requestVisualUpdate();
      }
    } else {
      markNeedsPaint();
    }
  }

  // Called when flushPaint() tries to make us paint but our layer is detached.
  // To make sure that our subtree is repainted when it's finally reattached,
  // even in the case where some ancestor layer is itself never marked dirty, we
  // have to mark our entire detached subtree as dirty and needing to be
  // repainted. That way, we'll eventually be repainted.
  void _skippedPaintingOnLayer() {
    assert(attached);
    assert(isRepaintBoundary);
    assert(_needsPaint || _needsCompositedLayerUpdate);
    assert(_layerHandle.layer != null);
    assert(!_layerHandle.layer!.attached);
    RenderObject? node = parent;
    while (node is RenderObject) {
      if (node.isRepaintBoundary) {
        if (node._layerHandle.layer == null) {
          // Looks like the subtree here has never been painted. Let it handle itself.
          break;
        }
        if (node._layerHandle.layer!.attached) {
          // It's the one that detached us, so it's the one that will decide to repaint us.
          break;
        }
        node._needsPaint = true;
      }
      node = node.parent;
    }
  }

  /// Bootstrap the rendering pipeline by scheduling the very first paint.
  ///
  /// Requires that this render object is attached, is the root of the render
  /// tree, and has a composited layer.
  ///
  /// See [RenderView] for an example of how this function is used.
  void scheduleInitialPaint(ContainerLayer rootLayer) {
    assert(rootLayer.attached);
    assert(attached);
    assert(parent is! RenderObject);
    assert(!owner!._debugDoingPaint);
    assert(isRepaintBoundary);
    assert(_layerHandle.layer == null);
    _layerHandle.layer = rootLayer;
    assert(_needsPaint);
    owner!._nodesNeedingPaint.add(this);
  }

  /// Replace the layer. This is only valid for the root of a render
  /// object subtree (whatever object [scheduleInitialPaint] was
  /// called on).
  ///
  /// This might be called if, e.g., the device pixel ratio changed.
  void replaceRootLayer(OffsetLayer rootLayer) {
    assert(!_debugDisposed);
    assert(rootLayer.attached);
    assert(attached);
    assert(parent is! RenderObject);
    assert(!owner!._debugDoingPaint);
    assert(isRepaintBoundary);
    assert(_layerHandle.layer != null); // use scheduleInitialPaint the first time
    _layerHandle.layer!.detach();
    _layerHandle.layer = rootLayer;
    markNeedsPaint();
  }

  void _paintWithContext(PaintingContext context, Offset offset) {
    assert(!_debugDisposed);
    assert(() {
      if (_debugDoingThisPaint) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Tried to paint a RenderObject reentrantly.'),
          describeForError(
            'The following RenderObject was already being painted when it was '
            'painted again',
          ),
          ErrorDescription(
            'Since this typically indicates an infinite recursion, it is '
            'disallowed.',
          ),
        ]);
      }
      return true;
    }());
    // If we still need layout, then that means that we were skipped in the
    // layout phase and therefore don't need painting. We might not know that
    // yet (that is, our layer might not have been detached yet), because the
    // same node that skipped us in layout is above us in the tree (obviously)
    // and therefore may not have had a chance to paint yet (since the tree
    // paints in reverse order). In particular this will happen if they have
    // a different layer, because there's a repaint boundary between us.
    if (_needsLayout) {
      return;
    }
    if (!kReleaseMode && debugProfilePaintsEnabled) {
      Map<String, String>? debugTimelineArguments;
      assert(() {
        if (debugEnhancePaintTimelineArguments) {
          debugTimelineArguments = toDiagnosticsNode().toTimelineArguments();
        }
        return true;
      }());
      FlutterTimeline.startSync('$runtimeType', arguments: debugTimelineArguments);
    }
    assert(() {
      if (_needsCompositingBitsUpdate) {
        if (parent is RenderObject) {
          final RenderObject parent = this.parent!;
          bool visitedByParent = false;
          parent.visitChildren((RenderObject child) {
            if (child == this) {
              visitedByParent = true;
            }
          });
          if (!visitedByParent) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              ErrorSummary(
                "A RenderObject was not visited by the parent's visitChildren "
                'during paint.',
              ),
              parent.describeForError('The parent was'),
              describeForError('The child that was not visited was'),
              ErrorDescription(
                'A RenderObject with children must implement visitChildren and '
                'call the visitor exactly once for each child; it also should not '
                'paint children that were removed with dropChild.',
              ),
              ErrorHint('This usually indicates an error in the Flutter framework itself.'),
            ]);
          }
        }
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'Tried to paint a RenderObject before its compositing bits were '
            'updated.',
          ),
          describeForError(
            'The following RenderObject was marked as having dirty compositing '
            'bits at the time that it was painted',
          ),
          ErrorDescription(
            'A RenderObject that still has dirty compositing bits cannot be '
            'painted because this indicates that the tree has not yet been '
            'properly configured for creating the layer tree.',
          ),
          ErrorHint('This usually indicates an error in the Flutter framework itself.'),
        ]);
      }
      return true;
    }());
    RenderObject? debugLastActivePaint;
    assert(() {
      _debugDoingThisPaint = true;
      debugLastActivePaint = _debugActivePaint;
      _debugActivePaint = this;
      assert(!isRepaintBoundary || _layerHandle.layer != null);
      return true;
    }());
    _needsPaint = false;
    _needsCompositedLayerUpdate = false;
    _wasRepaintBoundary = isRepaintBoundary;
    try {
      paint(context, offset);
      assert(!_needsLayout); // check that the paint() method didn't mark us dirty again
      assert(!_needsPaint); // check that the paint() method didn't mark us dirty again
    } catch (e, stack) {
      _reportException('paint', e, stack);
    }
    assert(() {
      debugPaint(context, offset);
      _debugActivePaint = debugLastActivePaint;
      _debugDoingThisPaint = false;
      return true;
    }());
    if (!kReleaseMode && debugProfilePaintsEnabled) {
      FlutterTimeline.finishSync();
    }
  }

  /// An estimate of the bounds within which this render object will paint.
  /// Useful for debugging flags such as [debugPaintLayerBordersEnabled].
  ///
  /// These are also the bounds used by [showOnScreen] to make a [RenderObject]
  /// visible on screen.
  Rect get paintBounds;

  /// Override this method to paint debugging information.
  void debugPaint(PaintingContext context, Offset offset) {}

  /// Paint this render object into the given context at the given offset.
  ///
  /// Subclasses should override this method to provide a visual appearance
  /// for themselves. The render object's local coordinate system is
  /// axis-aligned with the coordinate system of the context's canvas and the
  /// render object's local origin (i.e, x=0 and y=0) is placed at the given
  /// offset in the context's canvas.
  ///
  /// Do not call this function directly. If you wish to paint yourself, call
  /// [markNeedsPaint] instead to schedule a call to this function. If you wish
  /// to paint one of your children, call [PaintingContext.paintChild] on the
  /// given `context`.
  ///
  /// When painting one of your children (via a paint child function on the
  /// given context), the current canvas held by the context might change
  /// because draw operations before and after painting children might need to
  /// be recorded on separate compositing layers.
  void paint(PaintingContext context, Offset offset) {}

  /// Applies the transform that would be applied when painting the given child
  /// to the given matrix.
  ///
  /// Used by coordinate conversion functions to translate coordinates local to
  /// one render object into coordinates local to another render object.
  ///
  /// Some RenderObjects will provide a zeroed out matrix in this method,
  /// indicating that the child should not paint anything or respond to hit
  /// tests currently. A parent may supply a non-zero matrix even though it
  /// does not paint its child currently, for example if the parent is a
  /// [RenderOffstage] with `offstage` set to true. In both of these cases,
  /// the parent must return `false` from [paintsChild].
  void applyPaintTransform(covariant RenderObject child, Matrix4 transform) {
    assert(child.parent == this);
  }

  /// Whether the given child would be painted if [paint] were called.
  ///
  /// Some RenderObjects skip painting their children if they are configured to
  /// not produce any visible effects. For example, a [RenderOffstage] with
  /// its `offstage` property set to true, or a [RenderOpacity] with its opacity
  /// value set to zero.
  ///
  /// In these cases, the parent may still supply a non-zero matrix in
  /// [applyPaintTransform] to inform callers about where it would paint the
  /// child if the child were painted at all. Alternatively, the parent may
  /// supply a zeroed out matrix if it would not otherwise be able to determine
  /// a valid matrix for the child and thus cannot meaningfully determine where
  /// the child would paint.
  bool paintsChild(covariant RenderObject child) {
    assert(child.parent == this);
    return true;
  }

  /// {@template flutter.rendering.RenderObject.getTransformTo}
  /// Applies the paint transform from this [RenderObject] to the `target`
  /// [RenderObject].
  ///
  /// Returns a matrix that maps the local paint coordinate system to the
  /// coordinate system of `target`, or a [Matrix4.zero] if the paint transform
  /// can not be computed.
  ///
  /// This method throws an exception when the `target` is not in the same render
  /// tree as this [RenderObject], as the behavior is undefined.
  ///
  /// This method ignores [RenderObject.paintsChild]. This means it will still
  /// try to compute the paint transform even if this [RenderObject] or
  /// `target` is currently not visible.
  ///
  /// If `target` is null, this method returns a matrix that maps from the
  /// local paint coordinate system to the coordinate system of the
  /// [PipelineOwner.rootNode].
  /// {@endtemplate}
  ///
  /// For the render tree owned by the [RendererBinding] (i.e. for the main
  /// render tree displayed on the device) this means that this method maps to
  /// the global coordinate system in logical pixels. To get physical pixels,
  /// use [applyPaintTransform] from the [RenderView] to further transform the
  /// coordinate.
  Matrix4 getTransformTo(RenderObject? target) {
    assert(attached);
    // The paths from to fromRenderObject and toRenderObject's common ancestor.
    // Each list's length is greater than 1 if not null.
    //
    // [this, ...., commonAncestorRenderObject], or null if `this` is the common
    // ancestor.
    List<RenderObject>? fromPath;
    // [target, ...., commonAncestorRenderObject], or null if `target` is the
    // common ancestor.
    List<RenderObject>? toPath;

    RenderObject from = this;
    RenderObject to = target ?? owner!.rootNode!;

    while (!identical(from, to)) {
      final int fromDepth = from.depth;
      final int toDepth = to.depth;

      if (fromDepth >= toDepth) {
        final RenderObject fromParent =
            from.parent ??
            (throw FlutterError('$target and $this are not in the same render tree.'));
        (fromPath ??= <RenderObject>[this]).add(fromParent);
        from = fromParent;
      }
      if (fromDepth <= toDepth) {
        final RenderObject toParent =
            to.parent ?? (throw FlutterError('$target and $this are not in the same render tree.'));
        assert(
          target != null,
          '$this has a depth that is less than or equal to ${owner?.rootNode}',
        );
        (toPath ??= <RenderObject>[target!]).add(toParent);
        to = toParent;
      }
    }

    Matrix4? fromTransform;
    if (fromPath != null) {
      assert(fromPath.length > 1);
      fromTransform = Matrix4.identity();
      final int lastIndex = target == null ? fromPath.length - 2 : fromPath.length - 1;
      for (int index = lastIndex; index > 0; index -= 1) {
        fromPath[index].applyPaintTransform(fromPath[index - 1], fromTransform);
      }
    }
    if (toPath == null) {
      return fromTransform ?? Matrix4.identity();
    }

    assert(toPath.length > 1);
    final Matrix4 toTransform = Matrix4.identity();
    for (int index = toPath.length - 1; index > 0; index -= 1) {
      toPath[index].applyPaintTransform(toPath[index - 1], toTransform);
    }
    if (toTransform.invert() == 0) {
      // If the matrix is singular then `invert()` doesn't do anything.
      return Matrix4.zero();
    }
    return (fromTransform?..multiply(toTransform)) ?? toTransform;
  }

  /// Returns a rect in this object's coordinate system that describes
  /// the approximate bounding box of the clip rect that would be
  /// applied to the given child during the paint phase, if any.
  ///
  /// Returns null if the child would not be clipped.
  ///
  /// This is used in the semantics phase to avoid including children
  /// that are not physically visible.
  ///
  /// RenderObjects that respect a [Clip] behavior when painting _must_ respect
  /// that same behavior when describing this value. For example, if passing
  /// [Clip.none] to [PaintingContext.pushClipRect] as the `clipBehavior`, then
  /// the implementation of this method must return null.
  Rect? describeApproximatePaintClip(covariant RenderObject child) => null;

  /// Returns a rect in this object's coordinate system that describes
  /// which [SemanticsNode]s produced by the `child` should be included in the
  /// semantics tree. [SemanticsNode]s from the `child` that are positioned
  /// outside of this rect will be dropped. Child [SemanticsNode]s that are
  /// positioned inside this rect, but outside of [describeApproximatePaintClip]
  /// will be included in the tree marked as hidden. Child [SemanticsNode]s
  /// that are inside of both rect will be included in the tree as regular
  /// nodes.
  ///
  /// This method only returns a non-null value if the semantics clip rect
  /// is different from the rect returned by [describeApproximatePaintClip].
  /// If the semantics clip rect and the paint clip rect are the same, this
  /// method returns null.
  ///
  /// A viewport would typically implement this method to include semantic nodes
  /// in the semantics tree that are currently hidden just before the leading
  /// or just after the trailing edge. These nodes have to be included in the
  /// semantics tree to implement implicit accessibility scrolling on iOS where
  /// the viewport scrolls implicitly when moving the accessibility focus from
  /// the last visible node in the viewport to the first hidden one.
  ///
  /// See also:
  ///
  /// * [RenderViewportBase.cacheExtent], used by viewports to extend their
  ///   semantics clip beyond their approximate paint clip.
  Rect? describeSemanticsClip(covariant RenderObject? child) => null;

  // SEMANTICS

  /// Bootstrap the semantics reporting mechanism by marking this node
  /// as needing a semantics update.
  ///
  /// Requires that this render object is attached, and is the root of
  /// the render tree.
  ///
  /// See [RendererBinding] for an example of how this function is used.
  void scheduleInitialSemantics() {
    assert(!_debugDisposed);
    assert(attached);
    assert(parent is! RenderObject);
    assert(!owner!._debugDoingSemantics);
    assert(_semantics == null);
    assert(_needsSemanticsUpdate);
    assert(owner!._semanticsOwner != null);
    owner!._nodesNeedingSemantics.add(this);
    owner!.requestVisualUpdate();
  }

  /// Report the semantics of this node, for example for accessibility purposes.
  ///
  /// This method should be overridden by subclasses that have interesting
  /// semantic information.
  ///
  /// The given [SemanticsConfiguration] object is mutable and should be
  /// annotated in a manner that describes the current state. No reference
  /// should be kept to that object; mutating it outside of the context of the
  /// [describeSemanticsConfiguration] call (for example as a result of
  /// asynchronous computation) will at best have no useful effect and at worse
  /// will cause crashes as the data will be in an inconsistent state.
  ///
  /// {@tool snippet}
  ///
  /// The following snippet will describe the node as a button that responds to
  /// tap actions.
  ///
  /// ```dart
  /// abstract class SemanticButtonRenderObject extends RenderObject {
  ///   @override
  ///   void describeSemanticsConfiguration(SemanticsConfiguration config) {
  ///     super.describeSemanticsConfiguration(config);
  ///     config
  ///       ..onTap = _handleTap
  ///       ..label = 'I am a button'
  ///       ..isButton = true;
  ///   }
  ///
  ///   void _handleTap() {
  ///     // Do something.
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  @protected
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    // Nothing to do by default.
  }

  /// Sends a [SemanticsEvent] associated with this render object's [SemanticsNode].
  ///
  /// If this render object has no semantics information, the first parent
  /// render object with a non-null semantic node is used.
  ///
  /// If semantics are disabled, no events are dispatched.
  ///
  /// See [SemanticsNode.sendEvent] for a full description of the behavior.
  void sendSemanticsEvent(SemanticsEvent semanticsEvent) {
    if (owner!.semanticsOwner == null) {
      return;
    }
    if (_semantics != null && !_semantics!.isMergedIntoParent) {
      _semantics!.sendEvent(semanticsEvent);
    } else if (parent != null) {
      parent!.sendSemanticsEvent(semanticsEvent);
    }
  }

  // Use [_semanticsConfiguration] to access.
  SemanticsConfiguration? _cachedSemanticsConfiguration;

  SemanticsConfiguration get _semanticsConfiguration {
    if (_cachedSemanticsConfiguration == null) {
      _cachedSemanticsConfiguration = SemanticsConfiguration();
      describeSemanticsConfiguration(_cachedSemanticsConfiguration!);
      assert(
        !_cachedSemanticsConfiguration!.explicitChildNodes ||
            _cachedSemanticsConfiguration!.childConfigurationsDelegate == null,
        'A SemanticsConfiguration with explicitChildNode set to true cannot have a non-null childConfigsDelegate.',
      );
    }
    return _cachedSemanticsConfiguration!;
  }

  /// The bounding box, in the local coordinate system, of this
  /// object, for accessibility purposes.
  Rect get semanticBounds;

  /// Whether the semantics of this render object is dirty and await the update.
  ///
  /// Always returns false in release mode.
  bool get debugNeedsSemanticsUpdate {
    if (kReleaseMode) {
      return false;
    }
    return _needsSemanticsUpdate;
  }

  bool _needsSemanticsUpdate = true;
  SemanticsNode? _semantics;

  /// The semantics of this render object.
  ///
  /// Exposed only for testing and debugging. To learn about the semantics of
  /// render objects in production, obtain a [SemanticsHandle] from
  /// [PipelineOwner.ensureSemantics].
  ///
  /// Only valid in debug and profile mode. In release builds, always returns
  /// null.
  SemanticsNode? get debugSemantics {
    if (!kReleaseMode) {
      return _semantics;
    }
    return null;
  }

  /// Removes all semantics from this render object and its descendants.
  ///
  /// Should only be called on objects whose [parent] is not a [RenderObject].
  ///
  /// Override this method if you instantiate new [SemanticsNode]s in an
  /// overridden [assembleSemanticsNode] method, to dispose of those nodes.
  @mustCallSuper
  void clearSemantics() {
    _needsSemanticsUpdate = true;
    _semantics = null;
    visitChildren((RenderObject child) {
      child.clearSemantics();
    });
  }

  /// Mark this node as needing an update to its semantics description.
  ///
  /// This must be called whenever the semantics configuration of this
  /// [RenderObject] as annotated by [describeSemanticsConfiguration] changes in
  /// any way to update the semantics tree.
  void markNeedsSemanticsUpdate() {
    assert(!_debugDisposed);
    assert(!attached || !owner!._debugDoingSemantics);
    if (!attached || owner!._semanticsOwner == null) {
      _cachedSemanticsConfiguration = null;
      return;
    }

    // Dirty the semantics tree starting at `this` until we have reached a
    // RenderObject that is a semantics boundary. All semantics past this
    // RenderObject are still up-to date. Therefore, we will later only rebuild
    // the semantics subtree starting at the identified semantics boundary.

    final bool wasSemanticsBoundary =
        _semantics != null && (_cachedSemanticsConfiguration?.isSemanticBoundary ?? false);

    bool mayProduceSiblingNodes =
        _cachedSemanticsConfiguration?.childConfigurationsDelegate != null ||
        _semanticsConfiguration.childConfigurationsDelegate != null;
    _cachedSemanticsConfiguration = null;

    bool isEffectiveSemanticsBoundary =
        _semanticsConfiguration.isSemanticBoundary && wasSemanticsBoundary;
    RenderObject node = this;

    // The sibling nodes will be attached to the parent of immediate semantics
    // node, thus marking this semantics boundary dirty is not enough, it needs
    // to find the first parent semantics boundary that does not have any
    // possible sibling node.
    while (node.semanticsParent != null &&
        (mayProduceSiblingNodes || !isEffectiveSemanticsBoundary)) {
      if (node != this && node._needsSemanticsUpdate) {
        break;
      }

      node._needsSemanticsUpdate = true;
      // Since this node is a semantics boundary, the produced sibling nodes will
      // be attached to the parent semantics boundary. Thus, these sibling nodes
      // will not be carried to the next loop.
      if (isEffectiveSemanticsBoundary) {
        mayProduceSiblingNodes = false;
      }

      node = node.semanticsParent!;
      isEffectiveSemanticsBoundary = node._semanticsConfiguration.isSemanticBoundary;
      if (isEffectiveSemanticsBoundary && node._semantics == null) {
        // We have reached a semantics boundary that doesn't own a semantics node.
        // That means the semantics of this branch are currently blocked and will
        // not appear in the semantics tree. We can abort the walk here.
        return;
      }
    }
    if (node != this && _semantics != null && _needsSemanticsUpdate) {
      // If `this` node has already been added to [owner._nodesNeedingSemantics]
      // remove it as it is no longer guaranteed that its semantics
      // node will continue to be in the tree. If it still is in the tree, the
      // ancestor `node` added to [owner._nodesNeedingSemantics] at the end of
      // this block will ensure that the semantics of `this` node actually gets
      // updated.
      // (See semantics_10_test.dart for an example why this is required).
      owner!._nodesNeedingSemantics.remove(this);
    }
    if (!node._needsSemanticsUpdate) {
      node._needsSemanticsUpdate = true;
      if (owner != null) {
        assert(node._semanticsConfiguration.isSemanticBoundary || node.semanticsParent == null);
        owner!._nodesNeedingSemantics.add(node);
        owner!.requestVisualUpdate();
      }
    }
  }

  /// Updates the semantic information of the render object.
  void _updateSemantics() {
    assert(_semanticsConfiguration.isSemanticBoundary || semanticsParent == null);
    if (_needsLayout) {
      // There's not enough information in this subtree to compute semantics.
      // The subtree is probably being kept alive by a viewport but not laid out.
      return;
    }
    if (!kReleaseMode) {
      FlutterTimeline.startSync('Semantics.GetFragment');
    }
    final _SemanticsFragment fragment = _getSemanticsForParent(
      mergeIntoParent: _semantics?.parent?.isPartOfNodeMerging ?? false,
      blockUserActions: _semantics?.areUserActionsBlocked ?? false,
    );
    if (!kReleaseMode) {
      FlutterTimeline.finishSync();
    }
    assert(fragment is _InterestingSemanticsFragment);
    final _InterestingSemanticsFragment interestingFragment =
        fragment as _InterestingSemanticsFragment;
    final List<SemanticsNode> result = <SemanticsNode>[];
    final List<SemanticsNode> siblingNodes = <SemanticsNode>[];

    if (!kReleaseMode) {
      FlutterTimeline.startSync('Semantics.compileChildren');
    }
    interestingFragment.compileChildren(
      parentSemanticsClipRect: _semantics?.parentSemanticsClipRect,
      parentPaintClipRect: _semantics?.parentPaintClipRect,
      elevationAdjustment: _semantics?.elevationAdjustment ?? 0.0,
      result: result,
      siblingNodes: siblingNodes,
    );
    if (!kReleaseMode) {
      FlutterTimeline.finishSync();
    }
    // Result may contain sibling nodes that are irrelevant for this update.
    assert(
      interestingFragment.config == null && result.any((SemanticsNode node) => node == _semantics),
    );
  }

  /// Returns the semantics that this node would like to add to its parent.
  _SemanticsFragment _getSemanticsForParent({
    required bool mergeIntoParent,
    required bool blockUserActions,
  }) {
    assert(!_needsLayout, 'Updated layout information required for $this to calculate semantics.');

    final SemanticsConfiguration config = _semanticsConfiguration;
    bool dropSemanticsOfPreviousSiblings = config.isBlockingSemanticsOfPreviouslyPaintedNodes;
    bool producesForkingFragment = !config.hasBeenAnnotated && !config.isSemanticBoundary;
    final bool blockChildInteractions = blockUserActions || config.isBlockingUserActions;
    final bool childrenMergeIntoParent = mergeIntoParent || config.isMergingSemanticsOfDescendants;
    final List<SemanticsConfiguration> childConfigurations = <SemanticsConfiguration>[];
    final bool explicitChildNode = config.explicitChildNodes || semanticsParent == null;
    final ChildSemanticsConfigurationsDelegate? childConfigurationsDelegate =
        config.childConfigurationsDelegate;
    final Map<SemanticsConfiguration, _InterestingSemanticsFragment> configToFragment =
        <SemanticsConfiguration, _InterestingSemanticsFragment>{};
    final List<_InterestingSemanticsFragment> mergeUpFragments = <_InterestingSemanticsFragment>[];
    final List<List<_InterestingSemanticsFragment>> siblingMergeFragmentGroups =
        <List<_InterestingSemanticsFragment>>[];
    final bool hasTags = config.tagsForChildren?.isNotEmpty ?? false;
    visitChildrenForSemantics((RenderObject renderChild) {
      assert(!_needsLayout);
      final _SemanticsFragment parentFragment = renderChild._getSemanticsForParent(
        mergeIntoParent: childrenMergeIntoParent,
        blockUserActions: blockChildInteractions,
      );
      if (parentFragment.dropsSemanticsOfPreviousSiblings) {
        childConfigurations.clear();
        mergeUpFragments.clear();
        siblingMergeFragmentGroups.clear();
        if (!config.isSemanticBoundary) {
          dropSemanticsOfPreviousSiblings = true;
        }
      }
      for (final _InterestingSemanticsFragment fragment in parentFragment.mergeUpFragments) {
        fragment.addAncestor(this);
        if (hasTags) {
          fragment.addTags(config.tagsForChildren!);
        }
        if (childConfigurationsDelegate != null && fragment.config != null) {
          // This fragment need to go through delegate to determine whether it
          // merge up or not.
          childConfigurations.add(fragment.config!);
          configToFragment[fragment.config!] = fragment;
        } else {
          mergeUpFragments.add(fragment);
        }
      }
      if (parentFragment is _ContainerSemanticsFragment) {
        // Container fragments needs to propagate sibling merge group to be
        // compiled by _SwitchableSemanticsFragment.
        for (final List<_InterestingSemanticsFragment> siblingMergeGroup
            in parentFragment.siblingMergeGroups) {
          for (final _InterestingSemanticsFragment siblingMergingFragment in siblingMergeGroup) {
            siblingMergingFragment.addAncestor(this);
            if (hasTags) {
              siblingMergingFragment.addTags(config.tagsForChildren!);
            }
          }
          siblingMergeFragmentGroups.add(siblingMergeGroup);
        }
      }
    });

    assert(childConfigurationsDelegate != null || configToFragment.isEmpty);

    if (explicitChildNode) {
      for (final _InterestingSemanticsFragment fragment in mergeUpFragments) {
        fragment.markAsExplicit();
      }
    } else if (childConfigurationsDelegate != null) {
      final ChildSemanticsConfigurationsResult result = childConfigurationsDelegate(
        childConfigurations,
      );
      mergeUpFragments.addAll(
        result.mergeUp.map<_InterestingSemanticsFragment>((SemanticsConfiguration config) {
          final _InterestingSemanticsFragment? fragment = configToFragment[config];
          if (fragment == null) {
            // Parent fragment of Incomplete fragments can't be a forking
            // fragment since they need to be merged.
            producesForkingFragment = false;
            return _IncompleteSemanticsFragment(config: config, owner: this);
          }
          return fragment;
        }),
      );
      for (final Iterable<SemanticsConfiguration> group in result.siblingMergeGroups) {
        siblingMergeFragmentGroups.add(
          group.map<_InterestingSemanticsFragment>((SemanticsConfiguration config) {
            return configToFragment[config] ??
                _IncompleteSemanticsFragment(config: config, owner: this);
          }).toList(),
        );
      }
    }

    _needsSemanticsUpdate = false;

    final _SemanticsFragment result;
    if (semanticsParent == null) {
      assert(!config.hasBeenAnnotated);
      assert(!mergeIntoParent);
      assert(siblingMergeFragmentGroups.isEmpty);
      _marksExplicitInMergeGroup(mergeUpFragments, isMergeUp: true);
      siblingMergeFragmentGroups.forEach(_marksExplicitInMergeGroup);
      result = _RootSemanticsFragment(
        owner: this,
        dropsSemanticsOfPreviousSiblings: dropSemanticsOfPreviousSiblings,
      );
    } else if (producesForkingFragment) {
      result = _ContainerSemanticsFragment(
        siblingMergeGroups: siblingMergeFragmentGroups,
        dropsSemanticsOfPreviousSiblings: dropSemanticsOfPreviousSiblings,
      );
    } else {
      _marksExplicitInMergeGroup(mergeUpFragments, isMergeUp: true);
      siblingMergeFragmentGroups.forEach(_marksExplicitInMergeGroup);
      result = _SwitchableSemanticsFragment(
        config: config,
        blockUserActions: blockUserActions,
        mergeIntoParent: mergeIntoParent,
        siblingMergeGroups: siblingMergeFragmentGroups,
        owner: this,
        dropsSemanticsOfPreviousSiblings: dropSemanticsOfPreviousSiblings,
      );
      if (config.isSemanticBoundary) {
        final _SwitchableSemanticsFragment fragment = result as _SwitchableSemanticsFragment;
        fragment.markAsExplicit();
      }
    }
    result.addAll(mergeUpFragments);
    return result;
  }

  void _marksExplicitInMergeGroup(
    List<_InterestingSemanticsFragment> mergeGroup, {
    bool isMergeUp = false,
  }) {
    final Set<_InterestingSemanticsFragment> toBeExplicit = <_InterestingSemanticsFragment>{};
    for (int i = 0; i < mergeGroup.length; i += 1) {
      final _InterestingSemanticsFragment fragment = mergeGroup[i];
      if (!fragment.hasConfigForParent) {
        continue;
      }
      if (isMergeUp && !_semanticsConfiguration.isCompatibleWith(fragment.config)) {
        toBeExplicit.add(fragment);
      }
      final int siblingLength = i;
      for (int j = 0; j < siblingLength; j += 1) {
        final _InterestingSemanticsFragment siblingFragment = mergeGroup[j];
        if (!fragment.config!.isCompatibleWith(siblingFragment.config)) {
          toBeExplicit.add(fragment);
          toBeExplicit.add(siblingFragment);
        }
      }
    }
    for (final _InterestingSemanticsFragment fragment in toBeExplicit) {
      fragment.markAsExplicit();
    }
  }

  /// Called when collecting the semantics of this node.
  ///
  /// The implementation has to return the children in paint order skipping all
  /// children that are not semantically relevant (e.g. because they are
  /// invisible).
  ///
  /// The default implementation mirrors the behavior of
  /// [visitChildren] (which is supposed to walk all the children).
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    visitChildren(visitor);
  }

  /// Assemble the [SemanticsNode] for this [RenderObject].
  ///
  /// If [describeSemanticsConfiguration] sets
  /// [SemanticsConfiguration.isSemanticBoundary] to true, this method is called
  /// with the `node` created for this [RenderObject], the `config` to be
  /// applied to that node and the `children` [SemanticsNode]s that descendants
  /// of this RenderObject have generated.
  ///
  /// By default, the method will annotate `node` with `config` and add the
  /// `children` to it.
  ///
  /// Subclasses can override this method to add additional [SemanticsNode]s
  /// to the tree. If new [SemanticsNode]s are instantiated in this method
  /// they must be disposed in [clearSemantics].
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    assert(node == _semantics);
    // TODO(a14n): remove the following cast by updating type of parameter in either updateWith or assembleSemanticsNode
    node.updateWith(config: config, childrenInInversePaintOrder: children as List<SemanticsNode>);
  }

  // EVENTS

  /// Override this method to handle pointer events that hit this render object.
  @override
  void handleEvent(PointerEvent event, covariant HitTestEntry entry) {}

  // HIT TESTING

  // RenderObject subclasses are expected to have a method like the following
  // (with the signature being whatever passes for coordinates for this
  // particular class):
  //
  // bool hitTest(HitTestResult result, { Offset position }) {
  //   // If the given position is not inside this node, then return false.
  //   // Otherwise:
  //   // For each child that intersects the position, in z-order starting from
  //   // the top, call hitTest() for that child, passing it /result/, and the
  //   // coordinates converted to the child's coordinate origin, and stop at
  //   // the first child that returns true.
  //   // Then, add yourself to /result/, and return true.
  // }
  //
  // If you add yourself to /result/ and still return false, then that means you
  // will see events but so will objects below you.

  /// Returns a human understandable name.
  @override
  String toStringShort() {
    String header = describeIdentity(this);
    if (!kReleaseMode) {
      if (_debugDisposed) {
        header += ' DISPOSED';
        return header;
      }
      if (_relayoutBoundary != null && _relayoutBoundary != this) {
        int count = 1;
        RenderObject? target = parent;
        while (target != null && target != _relayoutBoundary) {
          target = target.parent;
          count += 1;
        }
        header += ' relayoutBoundary=up$count';
      }
      if (_needsLayout) {
        header += ' NEEDS-LAYOUT';
      }
      if (_needsPaint) {
        header += ' NEEDS-PAINT';
      }
      if (_needsCompositingBitsUpdate) {
        header += ' NEEDS-COMPOSITING-BITS-UPDATE';
      }
      if (!attached) {
        header += ' DETACHED';
      }
    }
    return header;
  }

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) => toStringShort();

  /// Returns a description of the tree rooted at this node.
  /// If the prefix argument is provided, then every line in the output
  /// will be prefixed by that string.
  @override
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines = '',
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 65,
  }) {
    return _withDebugActiveLayoutCleared(
      () => super.toStringDeep(
        prefixLineOne: prefixLineOne,
        prefixOtherLines: prefixOtherLines,
        minLevel: minLevel,
        wrapWidth: wrapWidth,
      ),
    );
  }

  /// Returns a one-line detailed description of the render object.
  /// This description is often somewhat long.
  ///
  /// This includes the same information for this RenderObject as given by
  /// [toStringDeep], but does not recurse to any children.
  @override
  String toStringShallow({String joiner = ', ', DiagnosticLevel minLevel = DiagnosticLevel.debug}) {
    return _withDebugActiveLayoutCleared(
      () => super.toStringShallow(joiner: joiner, minLevel: minLevel),
    );
  }

  @protected
  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty('needsCompositing', value: _needsCompositing, ifTrue: 'needs compositing'),
    );
    properties.add(
      DiagnosticsProperty<Object?>(
        'creator',
        debugCreator,
        defaultValue: null,
        level: DiagnosticLevel.debug,
      ),
    );
    properties.add(
      DiagnosticsProperty<ParentData>(
        'parentData',
        parentData,
        tooltip: (_debugCanParentUseSize ?? false) ? 'can use size' : null,
        missingIfNull: true,
      ),
    );
    properties.add(
      DiagnosticsProperty<Constraints>('constraints', _constraints, missingIfNull: true),
    );
    // don't access it via the "layer" getter since that's only valid when we don't need paint
    properties.add(
      DiagnosticsProperty<ContainerLayer>('layer', _layerHandle.layer, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<SemanticsNode>('semantics node', _semantics, defaultValue: null),
    );
    properties.add(
      FlagProperty(
        'isBlockingSemanticsOfPreviouslyPaintedNodes',
        value: _semanticsConfiguration.isBlockingSemanticsOfPreviouslyPaintedNodes,
        ifTrue: 'blocks semantics of earlier render objects below the common boundary',
      ),
    );
    properties.add(
      FlagProperty(
        'isSemanticBoundary',
        value: _semanticsConfiguration.isSemanticBoundary,
        ifTrue: 'semantic boundary',
      ),
    );
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() => <DiagnosticsNode>[];

  /// Attempt to make (a portion of) this or a descendant [RenderObject] visible
  /// on screen.
  ///
  /// If `descendant` is provided, that [RenderObject] is made visible. If
  /// `descendant` is omitted, this [RenderObject] is made visible.
  ///
  /// The optional `rect` parameter describes which area of that [RenderObject]
  /// should be shown on screen. If `rect` is null, the entire
  /// [RenderObject] (as defined by its [paintBounds]) will be revealed. The
  /// `rect` parameter is interpreted relative to the coordinate system of
  /// `descendant` if that argument is provided and relative to this
  /// [RenderObject] otherwise.
  ///
  /// The `duration` parameter can be set to a non-zero value to bring the
  /// target object on screen in an animation defined by `curve`.
  ///
  /// See also:
  ///
  /// * [RenderViewportBase.showInViewport], which [RenderViewportBase] and
  ///   [SingleChildScrollView] delegate this method to.
  void showOnScreen({
    RenderObject? descendant,
    Rect? rect,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
  }) {
    if (parent is RenderObject) {
      parent!.showOnScreen(
        descendant: descendant ?? this,
        rect: rect,
        duration: duration,
        curve: curve,
      );
    }
  }

  /// Adds a debug representation of a [RenderObject] optimized for including in
  /// error messages.
  ///
  /// The default [style] of [DiagnosticsTreeStyle.shallow] ensures that all of
  /// the properties of the render object are included in the error output but
  /// none of the children of the object are.
  ///
  /// You should always include a RenderObject in an error message if it is the
  /// [RenderObject] causing the failure or contract violation of the error.
  DiagnosticsNode describeForError(
    String name, {
    DiagnosticsTreeStyle style = DiagnosticsTreeStyle.shallow,
  }) {
    return toDiagnosticsNode(name: name, style: style);
  }
}

/// Generic mixin for render objects with one child.
///
/// Provides a child model for a render object subclass that has
/// a unique child, which is accessible via the [child] getter.
///
/// This mixin is typically used to implement render objects created
/// in a [SingleChildRenderObjectWidget].
mixin RenderObjectWithChildMixin<ChildType extends RenderObject> on RenderObject {
  /// Checks whether the given render object has the correct [runtimeType] to be
  /// a child of this render object.
  ///
  /// Does nothing if assertions are disabled.
  ///
  /// Always returns true.
  bool debugValidateChild(RenderObject child) {
    assert(() {
      if (child is! ChildType) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'A $runtimeType expected a child of type $ChildType but received a '
            'child of type ${child.runtimeType}.',
          ),
          ErrorDescription(
            'RenderObjects expect specific types of children because they '
            'coordinate with their children during layout and paint. For '
            'example, a RenderSliver cannot be the child of a RenderBox because '
            'a RenderSliver does not understand the RenderBox layout protocol.',
          ),
          ErrorSpacer(),
          DiagnosticsProperty<Object?>(
            'The $runtimeType that expected a $ChildType child was created by',
            debugCreator,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          ErrorSpacer(),
          DiagnosticsProperty<Object?>(
            'The ${child.runtimeType} that did not match the expected child type '
            'was created by',
            child.debugCreator,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }
      return true;
    }());
    return true;
  }

  ChildType? _child;

  /// The render object's unique child.
  ChildType? get child => _child;
  set child(ChildType? value) {
    if (_child != null) {
      dropChild(_child!);
    }
    _child = value;
    if (_child != null) {
      adoptChild(_child!);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _child?.attach(owner);
  }

  @override
  void detach() {
    super.detach();
    _child?.detach();
  }

  @override
  void redepthChildren() {
    if (_child != null) {
      redepthChild(_child!);
    }
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    if (_child != null) {
      visitor(_child!);
    }
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return child != null
        ? <DiagnosticsNode>[child!.toDiagnosticsNode(name: 'child')]
        : <DiagnosticsNode>[];
  }
}

/// Parent data to support a doubly-linked list of children.
///
/// The children can be traversed using [nextSibling] or [previousSibling],
/// which can be called on the parent data of the render objects
/// obtained via [ContainerRenderObjectMixin.firstChild] or
/// [ContainerRenderObjectMixin.lastChild].
mixin ContainerParentDataMixin<ChildType extends RenderObject> on ParentData {
  /// The previous sibling in the parent's child list.
  ChildType? previousSibling;

  /// The next sibling in the parent's child list.
  ChildType? nextSibling;

  /// Clear the sibling pointers.
  @override
  void detach() {
    assert(
      previousSibling == null,
      'Pointers to siblings must be nulled before detaching ParentData.',
    );
    assert(nextSibling == null, 'Pointers to siblings must be nulled before detaching ParentData.');
    super.detach();
  }
}

/// Generic mixin for render objects with a list of children.
///
/// Provides a child model for a render object subclass that has a doubly-linked
/// list of children.
///
/// The [ChildType] specifies the type of the children (extending [RenderObject]),
/// e.g. [RenderBox].
///
/// [ParentDataType] stores parent container data on its child render objects.
/// It must extend [ContainerParentDataMixin], which provides the interface
/// for visiting children. This data is populated by
/// [RenderObject.setupParentData] implemented by the class using this mixin.
///
/// When using [RenderBox] as the child type, you will usually want to make use of
/// [RenderBoxContainerDefaultsMixin] and extend [ContainerBoxParentData] for the
/// parent data.
///
/// Moreover, this is a required mixin for render objects returned to [MultiChildRenderObjectWidget].
///
/// See also:
///
///  * [SlottedContainerRenderObjectMixin], which organizes its children
///    in different named slots.
mixin ContainerRenderObjectMixin<
  ChildType extends RenderObject,
  ParentDataType extends ContainerParentDataMixin<ChildType>
>
    on RenderObject {
  bool _debugUltimatePreviousSiblingOf(ChildType child, {ChildType? equals}) {
    ParentDataType childParentData = child.parentData! as ParentDataType;
    while (childParentData.previousSibling != null) {
      assert(childParentData.previousSibling != child);
      child = childParentData.previousSibling!;
      childParentData = child.parentData! as ParentDataType;
    }
    return child == equals;
  }

  bool _debugUltimateNextSiblingOf(ChildType child, {ChildType? equals}) {
    ParentDataType childParentData = child.parentData! as ParentDataType;
    while (childParentData.nextSibling != null) {
      assert(childParentData.nextSibling != child);
      child = childParentData.nextSibling!;
      childParentData = child.parentData! as ParentDataType;
    }
    return child == equals;
  }

  int _childCount = 0;

  /// The number of children.
  int get childCount => _childCount;

  /// Checks whether the given render object has the correct [runtimeType] to be
  /// a child of this render object.
  ///
  /// Does nothing if assertions are disabled.
  ///
  /// Always returns true.
  bool debugValidateChild(RenderObject child) {
    assert(() {
      if (child is! ChildType) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'A $runtimeType expected a child of type $ChildType but received a '
            'child of type ${child.runtimeType}.',
          ),
          ErrorDescription(
            'RenderObjects expect specific types of children because they '
            'coordinate with their children during layout and paint. For '
            'example, a RenderSliver cannot be the child of a RenderBox because '
            'a RenderSliver does not understand the RenderBox layout protocol.',
          ),
          ErrorSpacer(),
          DiagnosticsProperty<Object?>(
            'The $runtimeType that expected a $ChildType child was created by',
            debugCreator,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          ErrorSpacer(),
          DiagnosticsProperty<Object?>(
            'The ${child.runtimeType} that did not match the expected child type '
            'was created by',
            child.debugCreator,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }
      return true;
    }());
    return true;
  }

  ChildType? _firstChild;
  ChildType? _lastChild;
  void _insertIntoChildList(ChildType child, {ChildType? after}) {
    final ParentDataType childParentData = child.parentData! as ParentDataType;
    assert(childParentData.nextSibling == null);
    assert(childParentData.previousSibling == null);
    _childCount += 1;
    assert(_childCount > 0);
    if (after == null) {
      // insert at the start (_firstChild)
      childParentData.nextSibling = _firstChild;
      if (_firstChild != null) {
        final ParentDataType firstChildParentData = _firstChild!.parentData! as ParentDataType;
        firstChildParentData.previousSibling = child;
      }
      _firstChild = child;
      _lastChild ??= child;
    } else {
      assert(_firstChild != null);
      assert(_lastChild != null);
      assert(_debugUltimatePreviousSiblingOf(after, equals: _firstChild));
      assert(_debugUltimateNextSiblingOf(after, equals: _lastChild));
      final ParentDataType afterParentData = after.parentData! as ParentDataType;
      if (afterParentData.nextSibling == null) {
        // insert at the end (_lastChild); we'll end up with two or more children
        assert(after == _lastChild);
        childParentData.previousSibling = after;
        afterParentData.nextSibling = child;
        _lastChild = child;
      } else {
        // insert in the middle; we'll end up with three or more children
        // set up links from child to siblings
        childParentData.nextSibling = afterParentData.nextSibling;
        childParentData.previousSibling = after;
        // set up links from siblings to child
        final ParentDataType childPreviousSiblingParentData =
            childParentData.previousSibling!.parentData! as ParentDataType;
        final ParentDataType childNextSiblingParentData =
            childParentData.nextSibling!.parentData! as ParentDataType;
        childPreviousSiblingParentData.nextSibling = child;
        childNextSiblingParentData.previousSibling = child;
        assert(afterParentData.nextSibling == child);
      }
    }
  }

  /// Insert child into this render object's child list after the given child.
  ///
  /// If `after` is null, then this inserts the child at the start of the list,
  /// and the child becomes the new [firstChild].
  void insert(ChildType child, {ChildType? after}) {
    assert(child != this, 'A RenderObject cannot be inserted into itself.');
    assert(
      after != this,
      'A RenderObject cannot simultaneously be both the parent and the sibling of another RenderObject.',
    );
    assert(child != after, 'A RenderObject cannot be inserted after itself.');
    assert(child != _firstChild);
    assert(child != _lastChild);
    adoptChild(child);
    assert(
      child.parentData is ParentDataType,
      'A child of $runtimeType has parentData of type ${child.parentData.runtimeType}, '
      'which does not conform to $ParentDataType. Class using ContainerRenderObjectMixin '
      'should override setupParentData() to set parentData to type $ParentDataType.',
    );
    _insertIntoChildList(child, after: after);
  }

  /// Append child to the end of this render object's child list.
  void add(ChildType child) {
    insert(child, after: _lastChild);
  }

  /// Add all the children to the end of this render object's child list.
  void addAll(List<ChildType>? children) {
    children?.forEach(add);
  }

  void _removeFromChildList(ChildType child) {
    final ParentDataType childParentData = child.parentData! as ParentDataType;
    assert(_debugUltimatePreviousSiblingOf(child, equals: _firstChild));
    assert(_debugUltimateNextSiblingOf(child, equals: _lastChild));
    assert(_childCount >= 0);
    if (childParentData.previousSibling == null) {
      assert(_firstChild == child);
      _firstChild = childParentData.nextSibling;
    } else {
      final ParentDataType childPreviousSiblingParentData =
          childParentData.previousSibling!.parentData! as ParentDataType;
      childPreviousSiblingParentData.nextSibling = childParentData.nextSibling;
    }
    if (childParentData.nextSibling == null) {
      assert(_lastChild == child);
      _lastChild = childParentData.previousSibling;
    } else {
      final ParentDataType childNextSiblingParentData =
          childParentData.nextSibling!.parentData! as ParentDataType;
      childNextSiblingParentData.previousSibling = childParentData.previousSibling;
    }
    childParentData.previousSibling = null;
    childParentData.nextSibling = null;
    _childCount -= 1;
  }

  /// Remove this child from the child list.
  ///
  /// Requires the child to be present in the child list.
  void remove(ChildType child) {
    _removeFromChildList(child);
    dropChild(child);
  }

  /// Remove all their children from this render object's child list.
  ///
  /// More efficient than removing them individually.
  void removeAll() {
    ChildType? child = _firstChild;
    while (child != null) {
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      final ChildType? next = childParentData.nextSibling;
      childParentData.previousSibling = null;
      childParentData.nextSibling = null;
      dropChild(child);
      child = next;
    }
    _firstChild = null;
    _lastChild = null;
    _childCount = 0;
  }

  /// Move the given `child` in the child list to be after another child.
  ///
  /// More efficient than removing and re-adding the child. Requires the child
  /// to already be in the child list at some position. Pass null for `after` to
  /// move the child to the start of the child list.
  void move(ChildType child, {ChildType? after}) {
    assert(child != this);
    assert(after != this);
    assert(child != after);
    assert(child.parent == this);
    final ParentDataType childParentData = child.parentData! as ParentDataType;
    if (childParentData.previousSibling == after) {
      return;
    }
    _removeFromChildList(child);
    _insertIntoChildList(child, after: after);
    markNeedsLayout();
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    ChildType? child = _firstChild;
    while (child != null) {
      child.attach(owner);
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      child = childParentData.nextSibling;
    }
  }

  @override
  void detach() {
    super.detach();
    ChildType? child = _firstChild;
    while (child != null) {
      child.detach();
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      child = childParentData.nextSibling;
    }
  }

  @override
  void redepthChildren() {
    ChildType? child = _firstChild;
    while (child != null) {
      redepthChild(child);
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      child = childParentData.nextSibling;
    }
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    ChildType? child = _firstChild;
    while (child != null) {
      visitor(child);
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      child = childParentData.nextSibling;
    }
  }

  /// The first child in the child list.
  ChildType? get firstChild => _firstChild;

  /// The last child in the child list.
  ChildType? get lastChild => _lastChild;

  /// The previous child before the given child in the child list.
  ChildType? childBefore(ChildType child) {
    assert(child.parent == this);
    final ParentDataType childParentData = child.parentData! as ParentDataType;
    return childParentData.previousSibling;
  }

  /// The next child after the given child in the child list.
  ChildType? childAfter(ChildType child) {
    assert(child.parent == this);
    final ParentDataType childParentData = child.parentData! as ParentDataType;
    return childParentData.nextSibling;
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> children = <DiagnosticsNode>[];
    if (firstChild != null) {
      ChildType child = firstChild!;
      int count = 1;
      while (true) {
        children.add(child.toDiagnosticsNode(name: 'child $count'));
        if (child == lastChild) {
          break;
        }
        count += 1;
        final ParentDataType childParentData = child.parentData! as ParentDataType;
        child = childParentData.nextSibling!;
      }
    }
    return children;
  }
}

/// Mixin for [RenderObject] that will call [systemFontsDidChange] whenever the
/// system fonts change.
///
/// System fonts can change when the OS installs or removes a font. Use this
/// mixin if the [RenderObject] uses [TextPainter] or [Paragraph] to correctly
/// update the text when it happens.
mixin RelayoutWhenSystemFontsChangeMixin on RenderObject {
  /// A callback that is called when system fonts have changed.
  ///
  /// The framework defers the invocation of the callback to the
  /// [SchedulerPhase.transientCallbacks] phase to ensure that the
  /// [RenderObject]'s text layout is still valid when user interactions are in
  /// progress (which usually take place during the [SchedulerPhase.idle] phase).
  ///
  /// By default, [markNeedsLayout] is called on the [RenderObject]
  /// implementing this mixin.
  ///
  /// Subclass should override this method to clear any extra cache that depend
  /// on font-related metrics.
  @protected
  @mustCallSuper
  void systemFontsDidChange() {
    markNeedsLayout();
  }

  bool _hasPendingSystemFontsDidChangeCallBack = false;
  void _scheduleSystemFontsUpdate() {
    assert(
      SchedulerBinding.instance.schedulerPhase == SchedulerPhase.idle,
      '${objectRuntimeType(this, "RelayoutWhenSystemFontsChangeMixin")}._scheduleSystemFontsUpdate() '
      'called during ${SchedulerBinding.instance.schedulerPhase}.',
    );
    if (_hasPendingSystemFontsDidChangeCallBack) {
      return;
    }
    _hasPendingSystemFontsDidChangeCallBack = true;
    SchedulerBinding.instance.scheduleFrameCallback((Duration timeStamp) {
      assert(_hasPendingSystemFontsDidChangeCallBack);
      _hasPendingSystemFontsDidChangeCallBack = false;
      assert(
        attached || (debugDisposed ?? true),
        '$this is detached during ${SchedulerBinding.instance.schedulerPhase} but is not disposed.',
      );
      if (attached) {
        systemFontsDidChange();
      }
    });
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    // If there's a pending callback that would imply this node was detached
    // between the idle phase and the next transientCallbacks phase. The tree
    // can not be mutated between those two phases so that should never happen.
    assert(!_hasPendingSystemFontsDidChangeCallBack);
    PaintingBinding.instance.systemFonts.addListener(_scheduleSystemFontsUpdate);
  }

  @override
  void detach() {
    assert(!_hasPendingSystemFontsDidChangeCallBack);
    PaintingBinding.instance.systemFonts.removeListener(_scheduleSystemFontsUpdate);
    super.detach();
  }
}

/// Describes the semantics information a [RenderObject] wants to add to its
/// parent.
///
/// It has two notable subclasses:
///  * [_InterestingSemanticsFragment] describing actual semantic information to
///    be added to the parent.
///  * [_ContainerSemanticsFragment]: a container class to transport the semantic
///    information of multiple [_InterestingSemanticsFragment] to a parent.
abstract class _SemanticsFragment {
  _SemanticsFragment({required this.dropsSemanticsOfPreviousSiblings});

  /// Incorporate the fragments of children into this fragment.
  void addAll(Iterable<_InterestingSemanticsFragment> fragments);

  /// Whether this fragment wants to make the semantics information of
  /// previously painted [RenderObject]s unreachable for accessibility purposes.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.isBlockingSemanticsOfPreviouslyPaintedNodes]
  ///    describes what semantics are dropped in more detail.
  final bool dropsSemanticsOfPreviousSiblings;

  /// Returns [_InterestingSemanticsFragment] describing the actual semantic
  /// information that this fragment wants to add to the parent.
  List<_InterestingSemanticsFragment> get mergeUpFragments;
}

/// A container used when a [RenderObject] wants to add multiple independent
/// [_InterestingSemanticsFragment] to its parent.
///
/// The [_InterestingSemanticsFragment] to be added to the parent can be
/// obtained via [mergeUpFragments].
class _ContainerSemanticsFragment extends _SemanticsFragment {
  _ContainerSemanticsFragment({
    required super.dropsSemanticsOfPreviousSiblings,
    required this.siblingMergeGroups,
  });

  final List<List<_InterestingSemanticsFragment>> siblingMergeGroups;

  @override
  void addAll(Iterable<_InterestingSemanticsFragment> fragments) {
    mergeUpFragments.addAll(fragments);
  }

  @override
  final List<_InterestingSemanticsFragment> mergeUpFragments = <_InterestingSemanticsFragment>[];
}

/// A [_SemanticsFragment] that describes which concrete semantic information
/// a [RenderObject] wants to add to the [SemanticsNode] of its parent.
///
/// Specifically, it describes which children (as returned by [compileChildren])
/// should be added to the parent's [SemanticsNode] and which [config] should be
/// merged into the parent's [SemanticsNode].
abstract class _InterestingSemanticsFragment extends _SemanticsFragment {
  _InterestingSemanticsFragment({
    required RenderObject owner,
    required super.dropsSemanticsOfPreviousSiblings,
  }) : _ancestorChain = <RenderObject>[owner];

  /// The [RenderObject] that owns this fragment (and any new [SemanticsNode]
  /// introduced by it).
  RenderObject get owner => _ancestorChain.first;

  final List<RenderObject> _ancestorChain;

  /// The children to be added to the parent.
  ///
  /// See also:
  ///
  ///  * [SemanticsNode.parentSemanticsClipRect] for the source and definition
  ///    of the `parentSemanticsClipRect` argument.
  ///  * [SemanticsNode.parentPaintClipRect] for the source and definition
  ///    of the `parentPaintClipRect` argument.
  ///  * [SemanticsNode.elevationAdjustment] for the source and definition
  ///    of the `elevationAdjustment` argument.
  void compileChildren({
    required Rect? parentSemanticsClipRect,
    required Rect? parentPaintClipRect,
    required double elevationAdjustment,
    required List<SemanticsNode> result,
    required List<SemanticsNode> siblingNodes,
  });

  /// The [SemanticsConfiguration] the child wants to merge into the parent's
  /// [SemanticsNode] or null if it doesn't want to merge anything.
  SemanticsConfiguration? get config;

  /// Disallows this fragment to merge any configuration into its parent's
  /// [SemanticsNode].
  ///
  /// After calling this, the fragment will only produce children to be added
  /// to the parent and it will return null for [config].
  void markAsExplicit();

  /// Consume the fragments of children.
  ///
  /// For each provided fragment it will add that fragment's children to
  /// this fragment's children (as returned by [compileChildren]) and merge that
  /// fragment's [config] into this fragment's [config].
  ///
  /// If a provided fragment should not merge anything into [config] call
  /// [markAsExplicit] before passing the fragment to this method.
  @override
  void addAll(Iterable<_InterestingSemanticsFragment> fragments);

  /// Whether this fragment wants to add any semantic information to the parent
  /// [SemanticsNode].
  bool get hasConfigForParent => config != null;

  @override
  List<_InterestingSemanticsFragment> get mergeUpFragments => <_InterestingSemanticsFragment>[this];

  Set<SemanticsTag>? _tagsForChildren;

  /// Tag all children produced by [compileChildren] with `tags`.
  ///
  /// `tags` must not be empty.
  void addTags(Iterable<SemanticsTag> tags) {
    assert(tags.isNotEmpty);
    _tagsForChildren ??= <SemanticsTag>{};
    _tagsForChildren!.addAll(tags);
  }

  /// Adds the geometric information of `ancestor` to this object.
  ///
  /// Those information are required to properly compute the value for
  /// [SemanticsNode.transform], [SemanticsNode.parentSemanticsClipRect], and
  /// [SemanticsNode.rect].
  ///
  /// Ancestors have to be added in order from [owner] up until the next
  /// [RenderObject] that owns a [SemanticsNode] is reached.
  void addAncestor(RenderObject ancestor) {
    _ancestorChain.add(ancestor);
  }
}

/// An [_InterestingSemanticsFragment] that produces the root [SemanticsNode] of
/// the semantics tree.
///
/// The root node is available as the only element in the Iterable returned by
/// [_children].
class _RootSemanticsFragment extends _InterestingSemanticsFragment {
  _RootSemanticsFragment({required super.owner, required super.dropsSemanticsOfPreviousSiblings});

  @override
  void compileChildren({
    Rect? parentSemanticsClipRect,
    Rect? parentPaintClipRect,
    required double elevationAdjustment,
    required List<SemanticsNode> result,
    required List<SemanticsNode> siblingNodes,
  }) {
    assert(_tagsForChildren == null || _tagsForChildren!.isEmpty);
    assert(parentSemanticsClipRect == null);
    assert(parentPaintClipRect == null);
    assert(_ancestorChain.length == 1);
    assert(elevationAdjustment == 0.0);

    owner._semantics ??= SemanticsNode.root(
      showOnScreen: owner.showOnScreen,
      owner: owner.owner!.semanticsOwner!,
    );
    final SemanticsNode node = owner._semantics!;
    assert(MatrixUtils.matrixEquals(node.transform, Matrix4.identity()));
    assert(node.parentSemanticsClipRect == null);
    assert(node.parentPaintClipRect == null);

    node.rect = owner.semanticBounds;

    final List<SemanticsNode> children = <SemanticsNode>[];
    for (final _InterestingSemanticsFragment fragment in _children) {
      assert(fragment.config == null);
      fragment.compileChildren(
        parentSemanticsClipRect: parentSemanticsClipRect,
        parentPaintClipRect: parentPaintClipRect,
        elevationAdjustment: 0.0,
        result: children,
        siblingNodes: siblingNodes,
      );
    }
    // Root node does not have a parent and thus can't attach sibling nodes.
    assert(siblingNodes.isEmpty);
    node.updateWith(config: null, childrenInInversePaintOrder: children);

    // The root node is the only semantics node allowed to be invisible. This
    // can happen when the canvas the app is drawn on has a size of 0 by 0
    // pixel. If this happens, the root node must not have any children (because
    // these would be invisible as well and are therefore excluded from the
    // tree).
    assert(!node.isInvisible || children.isEmpty);
    result.add(node);
  }

  @override
  SemanticsConfiguration? get config => null;

  final List<_InterestingSemanticsFragment> _children = <_InterestingSemanticsFragment>[];

  @override
  void markAsExplicit() {
    // nothing to do, we are always explicit.
  }

  @override
  void addAll(Iterable<_InterestingSemanticsFragment> fragments) {
    _children.addAll(fragments);
  }
}

/// A fragment with partial information that must not form an explicit
/// semantics node without merging into another _SwitchableSemanticsFragment.
///
/// This fragment is generated from synthetic SemanticsConfiguration returned from
/// [SemanticsConfiguration.childConfigurationsDelegate].
class _IncompleteSemanticsFragment extends _InterestingSemanticsFragment {
  _IncompleteSemanticsFragment({required this.config, required super.owner})
    : super(dropsSemanticsOfPreviousSiblings: false);

  @override
  void addAll(Iterable<_InterestingSemanticsFragment> fragments) {
    assert(false, 'This fragment must be a leaf node');
  }

  @override
  void compileChildren({
    required Rect? parentSemanticsClipRect,
    required Rect? parentPaintClipRect,
    required double elevationAdjustment,
    required List<SemanticsNode> result,
    required List<SemanticsNode> siblingNodes,
  }) {
    // There is nothing to do because this fragment must be a leaf node and
    // must not be explicit.
  }

  @override
  final SemanticsConfiguration config;

  @override
  void markAsExplicit() {
    assert(
      false,
      'SemanticsConfiguration created in '
      'SemanticsConfiguration.childConfigurationsDelegate must not produce '
      'its own semantics node',
    );
  }
}

/// An [_InterestingSemanticsFragment] that can be told to only add explicit
/// [SemanticsNode]s to the parent.
///
/// If [markAsExplicit] was not called before this fragment is added to
/// another fragment it will merge [config] into the parent's [SemanticsNode]
/// and add its [_children] to it.
///
/// If [markAsExplicit] was called before adding this fragment to another
/// fragment it will create a new [SemanticsNode]. The newly created node will
/// be annotated with the [SemanticsConfiguration] that - without the call to
/// [markAsExplicit] - would have been merged into the parent's [SemanticsNode].
/// Similarly, the new node will also take over the children that otherwise
/// would have been added to the parent's [SemanticsNode].
///
/// After a call to [markAsExplicit] the only element returned by [_children]
/// is the newly created node and [config] will return null as the fragment
/// no longer wants to merge any semantic information into the parent's
/// [SemanticsNode].
class _SwitchableSemanticsFragment extends _InterestingSemanticsFragment {
  _SwitchableSemanticsFragment({
    required bool mergeIntoParent,
    required bool blockUserActions,
    required SemanticsConfiguration config,
    required List<List<_InterestingSemanticsFragment>> siblingMergeGroups,
    required super.owner,
    required super.dropsSemanticsOfPreviousSiblings,
  }) : _siblingMergeGroups = siblingMergeGroups,
       _mergeIntoParent = mergeIntoParent,
       _config = config {
    if (blockUserActions && !_config.isBlockingUserActions) {
      _ensureConfigIsWritable();
      _config.isBlockingUserActions = true;
    }
  }

  final bool _mergeIntoParent;
  SemanticsConfiguration _config;
  bool _isConfigWritable = false;
  bool _mergesToSibling = false;

  final List<List<_InterestingSemanticsFragment>> _siblingMergeGroups;

  void _mergeSiblingGroup(
    Rect? parentSemanticsClipRect,
    Rect? parentPaintClipRect,
    List<SemanticsNode> result,
    Set<int> usedSemanticsIds,
  ) {
    for (final List<_InterestingSemanticsFragment> group in _siblingMergeGroups) {
      Rect? rect;
      Rect? semanticsClipRect;
      Rect? paintClipRect;
      SemanticsConfiguration? configuration;
      // Use empty set because the _tagsForChildren may not contains all of the
      // tags if this fragment is not explicit. The _tagsForChildren are added
      // to sibling nodes at the end of compileChildren if this fragment is
      // explicit.
      final Set<SemanticsTag> tags = <SemanticsTag>{};
      SemanticsNode? node;
      for (final _InterestingSemanticsFragment fragment in group) {
        if (fragment.config != null) {
          final _SwitchableSemanticsFragment switchableFragment =
              fragment as _SwitchableSemanticsFragment;
          switchableFragment._mergesToSibling = true;
          node ??= fragment.owner._semantics;
          configuration ??= SemanticsConfiguration();
          configuration.absorb(switchableFragment.config!);
          // It is a child fragment of a _SwitchableFragment, it must have a
          // geometry.
          final _SemanticsGeometry geometry =
              switchableFragment._computeSemanticsGeometry(
                parentSemanticsClipRect: parentSemanticsClipRect,
                parentPaintClipRect: parentPaintClipRect,
              )!;
          final Rect fragmentRect = MatrixUtils.transformRect(geometry.transform, geometry.rect);
          rect = rect?.expandToInclude(fragmentRect) ?? fragmentRect;
          if (geometry.semanticsClipRect != null) {
            final Rect rect = MatrixUtils.transformRect(
              geometry.transform,
              geometry.semanticsClipRect!,
            );
            semanticsClipRect = semanticsClipRect?.intersect(rect) ?? rect;
          }
          if (geometry.paintClipRect != null) {
            final Rect rect = MatrixUtils.transformRect(
              geometry.transform,
              geometry.paintClipRect!,
            );
            paintClipRect = paintClipRect?.intersect(rect) ?? rect;
          }
          if (switchableFragment._tagsForChildren != null) {
            tags.addAll(switchableFragment._tagsForChildren!);
          }
        }
      }
      // Can be null if all fragments in group are marked as explicit.
      if (configuration != null && !rect!.isEmpty) {
        if (node == null || usedSemanticsIds.contains(node.id)) {
          node = SemanticsNode(showOnScreen: owner.showOnScreen);
        }
        usedSemanticsIds.add(node.id);
        node
          ..tags = tags
          ..rect = rect
          ..transform =
              null // Will be set when compiling immediate parent node.
          ..parentSemanticsClipRect = semanticsClipRect
          ..parentPaintClipRect = paintClipRect;
        for (final _InterestingSemanticsFragment fragment in group) {
          if (fragment.config != null) {
            fragment.owner._semantics = node;
          }
        }
        node.updateWith(config: configuration);
        result.add(node);
      }
    }
  }

  final List<_InterestingSemanticsFragment> _children = <_InterestingSemanticsFragment>[];

  @override
  void compileChildren({
    Rect? parentSemanticsClipRect,
    Rect? parentPaintClipRect,
    required double elevationAdjustment,
    required List<SemanticsNode> result,
    required List<SemanticsNode> siblingNodes,
  }) {
    final Set<int> usedSemanticsIds = <int>{};
    Iterable<_InterestingSemanticsFragment> compilingFragments = _children;
    for (final List<_InterestingSemanticsFragment> siblingGroup in _siblingMergeGroups) {
      compilingFragments = compilingFragments.followedBy(siblingGroup);
    }
    if (!_isExplicit) {
      if (!_mergesToSibling) {
        owner._semantics = null;
      }
      _mergeSiblingGroup(
        parentSemanticsClipRect,
        parentPaintClipRect,
        siblingNodes,
        usedSemanticsIds,
      );
      for (final _InterestingSemanticsFragment fragment in compilingFragments) {
        assert(_ancestorChain.first == fragment._ancestorChain.last);
        if (fragment is _SwitchableSemanticsFragment) {
          // Cached semantics node may be part of sibling merging group prior
          // to this update. In this case, the semantics node may continue to
          // be reused in that sibling merging group.
          if (fragment._isExplicit &&
              fragment.owner._semantics != null &&
              usedSemanticsIds.contains(fragment.owner._semantics!.id)) {
            fragment.owner._semantics = null;
          }
        }
        fragment._ancestorChain.addAll(_ancestorChain.skip(1));
        fragment.compileChildren(
          parentSemanticsClipRect: parentSemanticsClipRect,
          parentPaintClipRect: parentPaintClipRect,
          // The fragment is not explicit, its elevation has been absorbed by
          // the parent config (as thickness). We still need to make sure that
          // its children are placed at the elevation dictated by this config.
          elevationAdjustment: elevationAdjustment + _config.elevation,
          result: result,
          siblingNodes: siblingNodes,
        );
      }
      return;
    }

    final _SemanticsGeometry? geometry = _computeSemanticsGeometry(
      parentSemanticsClipRect: parentSemanticsClipRect,
      parentPaintClipRect: parentPaintClipRect,
    );

    if (!_mergeIntoParent && (geometry?.dropFromTree ?? false)) {
      return; // Drop the node, it's not going to be visible.
    }

    final SemanticsNode node =
        (owner._semantics ??= SemanticsNode(showOnScreen: owner.showOnScreen))
          ..tags = _tagsForChildren;

    node.elevationAdjustment = elevationAdjustment;
    if (elevationAdjustment != 0.0) {
      _ensureConfigIsWritable();
      _config.elevation += elevationAdjustment;
    }

    if (geometry != null) {
      assert(_needsGeometryUpdate);
      node
        ..rect = geometry.rect
        ..transform = geometry.transform
        ..parentSemanticsClipRect = geometry.semanticsClipRect
        ..parentPaintClipRect = geometry.paintClipRect;
      if (!_mergeIntoParent && geometry.markAsHidden) {
        _ensureConfigIsWritable();
        _config.isHidden = true;
      }
    }
    final List<SemanticsNode> children = <SemanticsNode>[];
    _mergeSiblingGroup(
      node.parentSemanticsClipRect,
      node.parentPaintClipRect,
      siblingNodes,
      usedSemanticsIds,
    );
    for (final _InterestingSemanticsFragment fragment in compilingFragments) {
      if (fragment is _SwitchableSemanticsFragment) {
        // Cached semantics node may be part of sibling merging group prior
        // to this update. In this case, the semantics node may continue to
        // be reused in that sibling merging group.
        if (fragment._isExplicit &&
            fragment.owner._semantics != null &&
            usedSemanticsIds.contains(fragment.owner._semantics!.id)) {
          fragment.owner._semantics = null;
        }
      }
      final List<SemanticsNode> childSiblingNodes = <SemanticsNode>[];
      fragment.compileChildren(
        parentSemanticsClipRect: node.parentSemanticsClipRect,
        parentPaintClipRect: node.parentPaintClipRect,
        elevationAdjustment: 0.0,
        result: children,
        siblingNodes: childSiblingNodes,
      );
      siblingNodes.addAll(childSiblingNodes);
    }

    if (_config.isSemanticBoundary) {
      owner.assembleSemanticsNode(node, _config, children);
    } else {
      node.updateWith(config: _config, childrenInInversePaintOrder: children);
    }
    result.add(node);
    // Sibling node needs to attach to the parent of an explicit node.
    for (final SemanticsNode siblingNode in siblingNodes) {
      // sibling nodes are in the same coordinate of the immediate explicit node.
      // They need to share the same transform if they are going to attach to the
      // parent of the immediate explicit node.
      assert(siblingNode.transform == null);
      siblingNode.transform = node.transform;
      if (_tagsForChildren != null) {
        siblingNode.tags ??= <SemanticsTag>{};
        siblingNode.tags!.addAll(_tagsForChildren!);
      }
    }
    result.addAll(siblingNodes);
    siblingNodes.clear();
  }

  _SemanticsGeometry? _computeSemanticsGeometry({
    required Rect? parentSemanticsClipRect,
    required Rect? parentPaintClipRect,
  }) {
    return _needsGeometryUpdate
        ? _SemanticsGeometry(
          parentSemanticsClipRect: parentSemanticsClipRect,
          parentPaintClipRect: parentPaintClipRect,
          ancestors: _ancestorChain,
        )
        : null;
  }

  @override
  SemanticsConfiguration? get config {
    return _isExplicit ? null : _config;
  }

  @override
  void addAll(Iterable<_InterestingSemanticsFragment> fragments) {
    for (final _InterestingSemanticsFragment fragment in fragments) {
      _children.add(fragment);
      if (fragment.config == null) {
        continue;
      }
      _ensureConfigIsWritable();
      _config.absorb(fragment.config!);
    }
  }

  @override
  void addTags(Iterable<SemanticsTag> tags) {
    super.addTags(tags);
    // _ContainerSemanticsFragments add their tags to child fragments through
    // this method. This fragment must make sure its _config is in sync.
    if (tags.isNotEmpty) {
      _ensureConfigIsWritable();
      tags.forEach(_config.addTagForChildren);
    }
  }

  void _ensureConfigIsWritable() {
    if (!_isConfigWritable) {
      _config = _config.copy();
      _isConfigWritable = true;
    }
  }

  bool _isExplicit = false;

  @override
  void markAsExplicit() {
    _isExplicit = true;
  }

  bool get _needsGeometryUpdate => _ancestorChain.length > 1;
}

/// Helper class that keeps track of the geometry of a [SemanticsNode].
///
/// It is used to annotate a [SemanticsNode] with the current information for
/// [SemanticsNode.rect] and [SemanticsNode.transform].
class _SemanticsGeometry {
  /// The `parentClippingRect` may be null if no clip is to be applied.
  ///
  /// The `ancestors` list has to include all [RenderObject] in order that are
  /// located between the [SemanticsNode] whose geometry is represented here
  /// (first [RenderObject] in the list) and its closest ancestor [RenderObject]
  /// that also owns its own [SemanticsNode] (last [RenderObject] in the list).
  _SemanticsGeometry({
    required Rect? parentSemanticsClipRect,
    required Rect? parentPaintClipRect,
    required List<RenderObject> ancestors,
  }) {
    _computeValues(parentSemanticsClipRect, parentPaintClipRect, ancestors);
  }

  Rect? _paintClipRect;
  Rect? _semanticsClipRect;
  late Matrix4 _transform;
  late Rect _rect;

  /// Value for [SemanticsNode.transform].
  Matrix4 get transform => _transform;

  /// Value for [SemanticsNode.parentSemanticsClipRect].
  Rect? get semanticsClipRect => _semanticsClipRect;

  /// Value for [SemanticsNode.parentPaintClipRect].
  Rect? get paintClipRect => _paintClipRect;

  /// Value for [SemanticsNode.rect].
  Rect get rect => _rect;

  /// Computes values, ensuring `rect` is properly bounded by ancestor clipping rects.
  ///
  /// See also:
  ///
  /// * [RenderObject.describeSemanticsClip], typically used to determine `parentSemanticsClipRect`.
  /// * [RenderObject.describeApproximatePaintClip], typically used to determine `parentPaintClipRect`.
  void _computeValues(
    Rect? parentSemanticsClipRect,
    Rect? parentPaintClipRect,
    List<RenderObject> ancestors,
  ) {
    assert(ancestors.length > 1);

    _transform = Matrix4.identity();
    _semanticsClipRect = parentSemanticsClipRect;
    _paintClipRect = parentPaintClipRect;

    for (int index = ancestors.length - 1; index > 0; index -= 1) {
      final RenderObject semanticsParent = ancestors[index];
      final RenderObject semanticsChild = ancestors[index - 1];
      _applyIntermediatePaintTransforms(semanticsParent, semanticsChild, _transform);

      if (identical(semanticsParent, semanticsChild.parent)) {
        // The easier and more common case: semanticsParent is directly
        // responsible for painting (and potentially clipping) the semanticsChild
        // RenderObject.
        _computeClipRect(semanticsParent, semanticsChild, _semanticsClipRect, _paintClipRect);
      } else {
        // Otherwise we have to find the closest ancestor RenderObject that
        // has up-to-date semantics geometry and compute the clip rects from there.
        //
        // Currently it can only happen when the subtree contains an OverlayPortal.
        final List<RenderObject> clipPath = <RenderObject>[semanticsChild];

        RenderObject? ancestor = semanticsChild.parent;
        while (ancestor != null && ancestor._semantics == null) {
          clipPath.add(ancestor);
          ancestor = ancestor.parent;
        }
        _paintClipRect = ancestor?._semantics?.parentPaintClipRect;
        _semanticsClipRect = ancestor?._semantics?.parentSemanticsClipRect;
        if (ancestor != null) {
          assert(ancestor._semantics != null);
          assert(!ancestor._needsSemanticsUpdate);
          RenderObject parent = ancestor;
          for (int i = clipPath.length - 1; i >= 0; i -= 1) {
            _computeClipRect(parent, clipPath[i], _semanticsClipRect, _paintClipRect);
            parent = clipPath[i];
          }
        }
      }
    }

    final RenderObject owner = ancestors.first;
    _rect = _semanticsClipRect?.intersect(owner.semanticBounds) ?? owner.semanticBounds;
    if (_paintClipRect != null) {
      final Rect paintRect = _paintClipRect!.intersect(_rect);
      _markAsHidden = paintRect.isEmpty && !_rect.isEmpty;
      if (!_markAsHidden) {
        _rect = paintRect;
      }
    }
  }

  /// From parent to child coordinate system.
  static Rect? _transformRect(Rect? rect, Matrix4 transform) {
    if (rect == null) {
      return null;
    }
    if (rect.isEmpty || transform.isZero()) {
      return Rect.zero;
    }
    return MatrixUtils.inverseTransformRect(transform, rect);
  }

  // Computes the paint transform from `childFragmentOwner` to
  // `parentFragmentOwner` and applies the paint transform to `transform` in
  // place.
  //
  // The `parentFragmentOwner` and `childFragmentOwner` [RenderObject]s must be
  // in the same render tree (so they have a common ancestor).
  static void _applyIntermediatePaintTransforms(
    RenderObject parentFragmentOwner,
    RenderObject childFragmentOwner,
    Matrix4 transform,
  ) {
    Matrix4? parentToCommonAncestorTransform;
    RenderObject from = childFragmentOwner;
    RenderObject to = parentFragmentOwner;

    while (!identical(from, to)) {
      final int fromDepth = from.depth;
      final int toDepth = to.depth;

      if (fromDepth >= toDepth) {
        assert(
          from.parent != null,
          '$parentFragmentOwner and $childFragmentOwner are not in the same render tree.',
        );
        final RenderObject fromParent = from.parent!;
        fromParent.applyPaintTransform(from, transform);
        from = fromParent;
      }
      if (fromDepth <= toDepth) {
        assert(
          to.parent != null,
          '$parentFragmentOwner and $childFragmentOwner are not in the same render tree.',
        );
        final RenderObject toParent = to.parent!;
        toParent.applyPaintTransform(to, parentToCommonAncestorTransform ??= Matrix4.identity());
        to = toParent;
      }
    }

    if (parentToCommonAncestorTransform != null) {
      if (parentToCommonAncestorTransform.invert() != 0) {
        transform.multiply(parentToCommonAncestorTransform);
      } else {
        transform.setZero();
      }
    }
  }

  // A matrix used to store transient transform data.
  //
  // Reusing this matrix avoids allocating a new matrix every time a temporary
  // matrix is needed.
  //
  // This instance should never be returned to the caller. Otherwise, the data
  // stored in it will be overwritten unpredictably by subsequent reuses.
  static final Matrix4 _temporaryTransformHolder = Matrix4.zero();

  // Computes the semantics and painting clip rects for the given child and
  // assigns the rects to _semanticsClipRect and _paintClipRect respectively.
  //
  // The caller must guarantee that child.parent == parent. The resulting rects
  // are in `child`'s coordinate system.
  void _computeClipRect(
    RenderObject parent,
    RenderObject child,
    Rect? parentSemanticsClipRect,
    Rect? parentPaintClipRect,
  ) {
    assert(identical(child.parent, parent));
    // Computes the paint transform from child to parent. The _transformRect
    // method will compute the inverse.
    _temporaryTransformHolder.setIdentity(); // clears data from previous call(s)
    parent.applyPaintTransform(child, _temporaryTransformHolder);

    final Rect? additionalPaintClip = parent.describeApproximatePaintClip(child);
    _paintClipRect = _transformRect(
      _intersectRects(additionalPaintClip, parentPaintClipRect),
      _temporaryTransformHolder,
    );

    if (_paintClipRect == null) {
      _semanticsClipRect = null;
    } else {
      final Rect? semanticsClip =
          parent.describeSemanticsClip(child) ??
          _intersectRects(parentSemanticsClipRect, additionalPaintClip);
      _semanticsClipRect = _transformRect(semanticsClip, _temporaryTransformHolder);
    }
  }

  static Rect? _intersectRects(Rect? a, Rect? b) {
    if (b == null) {
      return a;
    }
    return a?.intersect(b) ?? b;
  }

  /// Whether the [SemanticsNode] annotated with the geometric information tracked
  /// by this object can be dropped from the semantics tree without losing
  /// semantics information.
  bool get dropFromTree {
    return _rect.isEmpty || _transform.isZero();
  }

  /// Whether the [SemanticsNode] annotated with the geometric information
  /// tracked by this object should be marked as hidden because it is not
  /// visible on screen.
  ///
  /// Hidden elements should still be included in the tree to work around
  /// platform limitations (e.g. accessibility scrolling on iOS).
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.isHidden] for the purpose of marking a node as hidden.
  bool get markAsHidden => _markAsHidden;
  bool _markAsHidden = false;
}

/// A class that creates [DiagnosticsNode] by wrapping [RenderObject.debugCreator].
///
/// Attach a [DiagnosticsDebugCreator] into [FlutterErrorDetails.informationCollector]
/// when a [RenderObject.debugCreator] is available. This will lead to improved
/// error message.
class DiagnosticsDebugCreator extends DiagnosticsProperty<Object> {
  /// Create a [DiagnosticsProperty] with its [value] initialized to input
  /// [RenderObject.debugCreator].
  DiagnosticsDebugCreator(Object value)
    : super('debugCreator', value, level: DiagnosticLevel.hidden);
}
