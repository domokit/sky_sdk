// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:collection';

import 'package:flutter/foundation.dart';
import 'package:flutter/semantics.dart';

import 'box.dart';
import 'object.dart';
import 'proxy_box.dart';

/// Signature of the function returned by [CustomPainter.semanticsBuilder].
///
/// Builds semantics information describing the picture drawn by a
/// [CustomPainter]. Each [CustomPainterSemantics] in the returned list is
/// converted into a [SemanticsNode] by copying its properties.
///
/// The returned list must not be mutated after this function completes. To
/// change the semantic information, the function must return a new list
/// instead.
typedef SemanticsBuilderCallback = List<CustomPainterSemantics> Function(Size size);

/// The interface used by [CustomPaint] (in the widgets library) and
/// [RenderCustomPaint] (in the rendering library).
///
/// To implement a custom painter, either subclass or implement this interface
/// to define your custom paint delegate. [CustomPainter] subclasses must
/// implement the [paint] and [shouldRepaint] methods, and may optionally also
/// implement the [hitTest] and [shouldRebuildSemantics] methods, and the
/// [semanticsBuilder] getter.
///
/// The [paint] method is called whenever the custom object needs to be repainted.
///
/// The [shouldRepaint] method is called when a new instance of the class
/// is provided, to check if the new instance actually represents different
/// information.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=vvI_NUXK00s}
///
/// The most efficient way to trigger a repaint is to either:
///
/// * Extend this class and supply a `repaint` argument to the constructor of
///   the [CustomPainter], where that object notifies its listeners when it is
///   time to repaint.
/// * Extend [Listenable] (e.g. via [ChangeNotifier]) and implement
///   [CustomPainter], so that the object itself provides the notifications
///   directly.
///
/// In either case, the [CustomPaint] widget or [RenderCustomPaint]
/// render object will listen to the [Listenable] and repaint whenever the
/// animation ticks, avoiding both the build and layout phases of the pipeline.
///
/// The [hitTest] method is called when the user interacts with the underlying
/// render object, to determine if the user hit the object or missed it.
///
/// The [semanticsBuilder] is called whenever the custom object needs to rebuild
/// its semantics information.
///
/// The [shouldRebuildSemantics] method is called when a new instance of the
/// class is provided, to check if the new instance contains different
/// information that affects the semantics tree.
///
/// {@tool snippet}
///
/// This sample extends the same code shown for [RadialGradient] to create a
/// custom painter that paints a sky.
///
/// ```dart
/// class Sky extends CustomPainter {
///   @override
///   void paint(Canvas canvas, Size size) {
///     final Rect rect = Offset.zero & size;
///     const RadialGradient gradient = RadialGradient(
///       center: Alignment(0.7, -0.6),
///       radius: 0.2,
///       colors: <Color>[Color(0xFFFFFF00), Color(0xFF0099FF)],
///       stops: <double>[0.4, 1.0],
///     );
///     canvas.drawRect(
///       rect,
///       Paint()..shader = gradient.createShader(rect),
///     );
///   }
///
///   @override
///   SemanticsBuilderCallback get semanticsBuilder {
///     return (Size size) {
///       // Annotate a rectangle containing the picture of the sun
///       // with the label "Sun". When text to speech feature is enabled on the
///       // device, a user will be able to locate the sun on this picture by
///       // touch.
///       Rect rect = Offset.zero & size;
///       final double width = size.shortestSide * 0.4;
///       rect = const Alignment(0.8, -0.9).inscribe(Size(width, width), rect);
///       return <CustomPainterSemantics>[
///         CustomPainterSemantics(
///           rect: rect,
///           properties: const SemanticsProperties(
///             label: 'Sun',
///             textDirection: TextDirection.ltr,
///           ),
///         ),
///       ];
///     };
///   }
///
///   // Since this Sky painter has no fields, it always paints
///   // the same thing and semantics information is the same.
///   // Therefore we return false here. If we had fields (set
///   // from the constructor) then we would return true if any
///   // of them differed from the same fields on the oldDelegate.
///   @override
///   bool shouldRepaint(Sky oldDelegate) => false;
///   @override
///   bool shouldRebuildSemantics(Sky oldDelegate) => false;
/// }
/// ```
/// {@end-tool}
///
/// ## Composition and the sharing of canvases
///
/// Widgets (or rather, render objects) are composited together using a minimum
/// number of [Canvas]es, for performance reasons. As a result, a
/// [CustomPainter]'s [Canvas] may be the same as that used by other widgets
/// (including other [CustomPaint] widgets).
///
/// This is mostly unnoticeable, except when using unusual [BlendMode]s. For
/// example, trying to use [BlendMode.dstOut] to "punch a hole" through a
/// previously-drawn image may erase more than was intended, because previous
/// widgets will have been painted onto the same canvas.
///
/// To avoid this issue, consider using [Canvas.saveLayer] and
/// [Canvas.restore] when using such blend modes. Creating new layers is
/// relatively expensive, however, and should be done sparingly to avoid
/// introducing jank.
///
/// See also:
///
///  * [Canvas], the class that a custom painter uses to paint.
///  * [CustomPaint], the widget that uses [CustomPainter], and whose sample
///    code shows how to use the above `Sky` class.
///  * [RadialGradient], whose sample code section shows a different take
///    on the sample code above.
abstract class CustomPainter extends Listenable {
  /// Creates a custom painter.
  ///
  /// The painter will repaint whenever `repaint` notifies its listeners.
  const CustomPainter({Listenable? repaint}) : _repaint = repaint;

  final Listenable? _repaint;

  /// Register a closure to be notified when it is time to repaint.
  ///
  /// The [CustomPainter] implementation merely forwards to the same method on
  /// the [Listenable] provided to the constructor in the `repaint` argument, if
  /// it was not null.
  @override
  void addListener(VoidCallback listener) => _repaint?.addListener(listener);

  /// Remove a previously registered closure from the list of closures that the
  /// object notifies when it is time to repaint.
  ///
  /// The [CustomPainter] implementation merely forwards to the same method on
  /// the [Listenable] provided to the constructor in the `repaint` argument, if
  /// it was not null.
  @override
  void removeListener(VoidCallback listener) => _repaint?.removeListener(listener);

  /// Called whenever the object needs to paint. The given [Canvas] has its
  /// coordinate space configured such that the origin is at the top left of the
  /// box. The area of the box is the size of the [size] argument.
  ///
  /// Paint operations should remain inside the given area. Graphical
  /// operations outside the bounds may be silently ignored, clipped, or not
  /// clipped. It may sometimes be difficult to guarantee that a certain
  /// operation is inside the bounds (e.g., drawing a rectangle whose size is
  /// determined by user inputs). In that case, consider calling
  /// [Canvas.clipRect] at the beginning of [paint] so everything that follows
  /// will be guaranteed to only draw within the clipped area.
  ///
  /// Implementations should be wary of correctly pairing any calls to
  /// [Canvas.save]/[Canvas.saveLayer] and [Canvas.restore], otherwise all
  /// subsequent painting on this canvas may be affected, with potentially
  /// hilarious but confusing results.
  ///
  /// To paint text on a [Canvas], use a [TextPainter].
  ///
  /// To paint an image on a [Canvas]:
  ///
  /// 1. Obtain an [ImageStream], for example by calling [ImageProvider.resolve]
  ///    on an [AssetImage] or [NetworkImage] object.
  ///
  /// 2. Whenever the [ImageStream]'s underlying [ImageInfo] object changes
  ///    (see [ImageStream.addListener]), create a new instance of your custom
  ///    paint delegate, giving it the new [ImageInfo] object.
  ///
  /// 3. In your delegate's [paint] method, call the [Canvas.drawImage],
  ///    [Canvas.drawImageRect], or [Canvas.drawImageNine] methods to paint the
  ///    [ImageInfo.image] object, applying the [ImageInfo.scale] value to
  ///    obtain the correct rendering size.
  void paint(Canvas canvas, Size size);

  /// Returns a function that builds semantic information for the picture drawn
  /// by this painter.
  ///
  /// If the returned function is null, this painter will not contribute new
  /// [SemanticsNode]s to the semantics tree and the [CustomPaint] corresponding
  /// to this painter will not create a semantics boundary. However, if the
  /// child of a [CustomPaint] is not null, the child may contribute
  /// [SemanticsNode]s to the tree.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.isSemanticBoundary], which causes new
  ///    [SemanticsNode]s to be added to the semantics tree.
  ///  * [RenderCustomPaint], which uses this getter to build semantics.
  SemanticsBuilderCallback? get semanticsBuilder => null;

  /// Called whenever a new instance of the custom painter delegate class is
  /// provided to the [RenderCustomPaint] object, or any time that a new
  /// [CustomPaint] object is created with a new instance of the custom painter
  /// delegate class (which amounts to the same thing, because the latter is
  /// implemented in terms of the former).
  ///
  /// If the new instance would cause [semanticsBuilder] to create different
  /// semantics information, then this method should return true, otherwise it
  /// should return false.
  ///
  /// If the method returns false, then the [semanticsBuilder] call might be
  /// optimized away.
  ///
  /// It's possible that the [semanticsBuilder] will get called even if
  /// [shouldRebuildSemantics] would return false. For example, it is called
  /// when the [CustomPaint] is rendered for the very first time, or when the
  /// box changes its size.
  ///
  /// By default this method delegates to [shouldRepaint] under the assumption
  /// that in most cases semantics change when something new is drawn.
  bool shouldRebuildSemantics(covariant CustomPainter oldDelegate) => shouldRepaint(oldDelegate);

  /// Called whenever a new instance of the custom painter delegate class is
  /// provided to the [RenderCustomPaint] object, or any time that a new
  /// [CustomPaint] object is created with a new instance of the custom painter
  /// delegate class (which amounts to the same thing, because the latter is
  /// implemented in terms of the former).
  ///
  /// If the new instance represents different information than the old
  /// instance, then the method should return true, otherwise it should return
  /// false.
  ///
  /// If the method returns false, then the [paint] call might be optimized
  /// away.
  ///
  /// It's possible that the [paint] method will get called even if
  /// [shouldRepaint] returns false (e.g. if an ancestor or descendant needed to
  /// be repainted). It's also possible that the [paint] method will get called
  /// without [shouldRepaint] being called at all (e.g. if the box changes
  /// size).
  ///
  /// If a custom delegate has a particularly expensive paint function such that
  /// repaints should be avoided as much as possible, a [RepaintBoundary] or
  /// [RenderRepaintBoundary] (or other render object with
  /// [RenderObject.isRepaintBoundary] set to true) might be helpful.
  ///
  /// The `oldDelegate` argument will never be null.
  bool shouldRepaint(covariant CustomPainter oldDelegate);

  /// Called whenever a hit test is being performed on an object that is using
  /// this custom paint delegate.
  ///
  /// The given point is relative to the same coordinate space as the last
  /// [paint] call.
  ///
  /// The default behavior is to consider all points to be hits for
  /// background painters, and no points to be hits for foreground painters.
  ///
  /// Return true if the given position corresponds to a point on the drawn
  /// image that should be considered a "hit", false if it corresponds to a
  /// point that should be considered outside the painted image, and null to use
  /// the default behavior.
  bool? hitTest(Offset position) => null;

  @override
  String toString() => '${describeIdentity(this)}(${_repaint?.toString() ?? ""})';
}

/// Contains properties describing information drawn in a rectangle contained by
/// the [Canvas] used by a [CustomPaint].
///
/// This information is used, for example, by assistive technologies to improve
/// the accessibility of applications.
///
/// Implement [CustomPainter.semanticsBuilder] to build the semantic
/// description of the whole picture drawn by a [CustomPaint], rather than one
/// particular rectangle.
///
/// See also:
///
///  * [SemanticsNode], which is created using the properties of this class.
///  * [CustomPainter], which creates instances of this class.
@immutable
class CustomPainterSemantics {
  /// Creates semantics information describing a rectangle on a canvas.
  const CustomPainterSemantics({
    this.key,
    required this.rect,
    required this.properties,
    this.transform,
    this.tags,
  });

  /// Identifies this object in a list of siblings.
  ///
  /// [SemanticsNode] inherits this key, so that when the list of nodes is
  /// updated, its nodes are updated from [CustomPainterSemantics] with matching
  /// keys.
  ///
  /// If this is null, the update algorithm does not guarantee which
  /// [SemanticsNode] will be updated using this instance.
  ///
  /// This value is assigned to [SemanticsNode.key] during update.
  final Key? key;

  /// The location and size of the box on the canvas where this piece of semantic
  /// information applies.
  ///
  /// This value is assigned to [SemanticsNode.rect] during update.
  final Rect rect;

  /// The transform from the canvas' coordinate system to its parent's
  /// coordinate system.
  ///
  /// This value is assigned to [SemanticsNode.transform] during update.
  final Matrix4? transform;

  /// Contains properties that are assigned to the [SemanticsNode] created or
  /// updated from this object.
  ///
  /// See also:
  ///
  ///  * [Semantics], which is a widget that also uses [SemanticsProperties] to
  ///    annotate.
  final SemanticsProperties properties;

  /// Tags used by the parent [SemanticsNode] to determine the layout of the
  /// semantics tree.
  ///
  /// This value is assigned to [SemanticsNode.tags] during update.
  final Set<SemanticsTag>? tags;
}

/// Provides a canvas on which to draw during the paint phase.
///
/// When asked to paint, [RenderCustomPaint] first asks its [painter] to paint
/// on the current canvas, then it paints its child, and then, after painting
/// its child, it asks its [foregroundPainter] to paint. The coordinate system of
/// the canvas matches the coordinate system of the [CustomPaint] object. The
/// painters are expected to paint within a rectangle starting at the origin and
/// encompassing a region of the given size. (If the painters paint outside
/// those bounds, there might be insufficient memory allocated to rasterize the
/// painting commands and the resulting behavior is undefined.)
///
/// Painters are implemented by subclassing or implementing [CustomPainter].
///
/// Because custom paint calls its painters during paint, you cannot mark the
/// tree as needing a new layout during the callback (the layout for this frame
/// has already happened).
///
/// Custom painters normally size themselves to their child. If they do not have
/// a child, they attempt to size themselves to the [preferredSize], which
/// defaults to [Size.zero].
///
/// See also:
///
///  * [CustomPainter], the class that custom painter delegates should extend.
///  * [Canvas], the API provided to custom painter delegates.
class RenderCustomPaint extends RenderProxyBox {
  /// Creates a render object that delegates its painting.
  RenderCustomPaint({
    CustomPainter? painter,
    CustomPainter? foregroundPainter,
    Size preferredSize = Size.zero,
    this.isComplex = false,
    this.willChange = false,
    RenderBox? child,
  }) : _painter = painter,
       _foregroundPainter = foregroundPainter,
       _preferredSize = preferredSize,
       super(child);

  /// The background custom paint delegate.
  ///
  /// This painter, if non-null, is called to paint behind the children.
  CustomPainter? get painter => _painter;
  CustomPainter? _painter;

  /// Set a new background custom paint delegate.
  ///
  /// If the new delegate is the same as the previous one, this does nothing.
  ///
  /// If the new delegate is the same class as the previous one, then the new
  /// delegate has its [CustomPainter.shouldRepaint] called; if the result is
  /// true, then the delegate will be called.
  ///
  /// If the new delegate is a different class than the previous one, then the
  /// delegate will be called.
  ///
  /// If the new value is null, then there is no background custom painter.
  set painter(CustomPainter? value) {
    if (_painter == value) {
      return;
    }
    final CustomPainter? oldPainter = _painter;
    _painter = value;
    _didUpdatePainter(_painter, oldPainter);
  }

  /// The foreground custom paint delegate.
  ///
  /// This painter, if non-null, is called to paint in front of the children.
  CustomPainter? get foregroundPainter => _foregroundPainter;
  CustomPainter? _foregroundPainter;

  /// Set a new foreground custom paint delegate.
  ///
  /// If the new delegate is the same as the previous one, this does nothing.
  ///
  /// If the new delegate is the same class as the previous one, then the new
  /// delegate has its [CustomPainter.shouldRepaint] called; if the result is
  /// true, then the delegate will be called.
  ///
  /// If the new delegate is a different class than the previous one, then the
  /// delegate will be called.
  ///
  /// If the new value is null, then there is no foreground custom painter.
  set foregroundPainter(CustomPainter? value) {
    if (_foregroundPainter == value) {
      return;
    }
    final CustomPainter? oldPainter = _foregroundPainter;
    _foregroundPainter = value;
    _didUpdatePainter(_foregroundPainter, oldPainter);
  }

  void _didUpdatePainter(CustomPainter? newPainter, CustomPainter? oldPainter) {
    // Check if we need to repaint.
    if (newPainter == null) {
      assert(oldPainter != null); // We should be called only for changes.
      markNeedsPaint();
    } else if (oldPainter == null ||
        newPainter.runtimeType != oldPainter.runtimeType ||
        newPainter.shouldRepaint(oldPainter)) {
      markNeedsPaint();
    }
    if (attached) {
      oldPainter?.removeListener(markNeedsPaint);
      newPainter?.addListener(markNeedsPaint);
    }

    // Check if we need to rebuild semantics.
    if (newPainter == null) {
      assert(oldPainter != null); // We should be called only for changes.
      if (attached) {
        markNeedsSemanticsUpdate();
      }
    } else if (oldPainter == null ||
        newPainter.runtimeType != oldPainter.runtimeType ||
        newPainter.shouldRebuildSemantics(oldPainter)) {
      markNeedsSemanticsUpdate();
    }
  }

  /// The size that this [RenderCustomPaint] should aim for, given the layout
  /// constraints, if there is no child.
  ///
  /// Defaults to [Size.zero].
  ///
  /// If there's a child, this is ignored, and the size of the child is used
  /// instead.
  Size get preferredSize => _preferredSize;
  Size _preferredSize;
  set preferredSize(Size value) {
    if (preferredSize == value) {
      return;
    }
    _preferredSize = value;
    markNeedsLayout();
  }

  /// Whether to hint that this layer's painting should be cached.
  ///
  /// The compositor contains a raster cache that holds bitmaps of layers in
  /// order to avoid the cost of repeatedly rendering those layers on each
  /// frame. If this flag is not set, then the compositor will apply its own
  /// heuristics to decide whether the layer containing this render object is
  /// complex enough to benefit from caching.
  bool isComplex;

  /// Whether the raster cache should be told that this painting is likely
  /// to change in the next frame.
  ///
  /// This hint tells the compositor not to cache the layer containing this
  /// render object because the cache will not be used in the future. If this
  /// hint is not set, the compositor will apply its own heuristics to decide
  /// whether this layer is likely to be reused in the future.
  bool willChange;

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child == null) {
      return preferredSize.width.isFinite ? preferredSize.width : 0;
    }
    return super.computeMinIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child == null) {
      return preferredSize.width.isFinite ? preferredSize.width : 0;
    }
    return super.computeMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child == null) {
      return preferredSize.height.isFinite ? preferredSize.height : 0;
    }
    return super.computeMinIntrinsicHeight(width);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child == null) {
      return preferredSize.height.isFinite ? preferredSize.height : 0;
    }
    return super.computeMaxIntrinsicHeight(width);
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _painter?.addListener(markNeedsPaint);
    _foregroundPainter?.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    _painter?.removeListener(markNeedsPaint);
    _foregroundPainter?.removeListener(markNeedsPaint);
    super.detach();
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    if (_foregroundPainter != null && (_foregroundPainter!.hitTest(position) ?? false)) {
      return true;
    }
    return super.hitTestChildren(result, position: position);
  }

  @override
  bool hitTestSelf(Offset position) {
    return _painter != null && (_painter!.hitTest(position) ?? true);
  }

  @override
  void performLayout() {
    super.performLayout();
    markNeedsSemanticsUpdate();
  }

  @override
  Size computeSizeForNoChild(BoxConstraints constraints) {
    return constraints.constrain(preferredSize);
  }

  void _paintWithPainter(Canvas canvas, Offset offset, CustomPainter painter) {
    late int debugPreviousCanvasSaveCount;
    canvas.save();
    assert(() {
      debugPreviousCanvasSaveCount = canvas.getSaveCount();
      return true;
    }());
    if (offset != Offset.zero) {
      canvas.translate(offset.dx, offset.dy);
    }
    painter.paint(canvas, size);
    assert(() {
      // This isn't perfect. For example, we can't catch the case of
      // someone first restoring, then setting a transform or whatnot,
      // then saving.
      // If this becomes a real problem, we could add logic to the
      // Canvas class to lock the canvas at a particular save count
      // such that restore() fails if it would take the lock count
      // below that number.
      final int debugNewCanvasSaveCount = canvas.getSaveCount();
      if (debugNewCanvasSaveCount > debugPreviousCanvasSaveCount) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The $painter custom painter called canvas.save() or canvas.saveLayer() at least '
            '${debugNewCanvasSaveCount - debugPreviousCanvasSaveCount} more '
            'time${debugNewCanvasSaveCount - debugPreviousCanvasSaveCount == 1 ? '' : 's'} '
            'than it called canvas.restore().',
          ),
          ErrorDescription(
            'This leaves the canvas in an inconsistent state and will probably result in a broken display.',
          ),
          ErrorHint(
            'You must pair each call to save()/saveLayer() with a later matching call to restore().',
          ),
        ]);
      }
      if (debugNewCanvasSaveCount < debugPreviousCanvasSaveCount) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The $painter custom painter called canvas.restore() '
            '${debugPreviousCanvasSaveCount - debugNewCanvasSaveCount} more '
            'time${debugPreviousCanvasSaveCount - debugNewCanvasSaveCount == 1 ? '' : 's'} '
            'than it called canvas.save() or canvas.saveLayer().',
          ),
          ErrorDescription(
            'This leaves the canvas in an inconsistent state and will result in a broken display.',
          ),
          ErrorHint('You should only call restore() if you first called save() or saveLayer().'),
        ]);
      }
      return debugNewCanvasSaveCount == debugPreviousCanvasSaveCount;
    }());
    canvas.restore();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (_painter != null) {
      _paintWithPainter(context.canvas, offset, _painter!);
      _setRasterCacheHints(context);
    }
    super.paint(context, offset);
    if (_foregroundPainter != null) {
      _paintWithPainter(context.canvas, offset, _foregroundPainter!);
      _setRasterCacheHints(context);
    }
  }

  void _setRasterCacheHints(PaintingContext context) {
    if (isComplex) {
      context.setIsComplexHint();
    }
    if (willChange) {
      context.setWillChangeHint();
    }
  }

  /// Builds semantics for the picture drawn by [painter].
  SemanticsBuilderCallback? _backgroundSemanticsBuilder;

  /// Builds semantics for the picture drawn by [foregroundPainter].
  SemanticsBuilderCallback? _foregroundSemanticsBuilder;

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    _backgroundSemanticsBuilder = painter?.semanticsBuilder;
    _foregroundSemanticsBuilder = foregroundPainter?.semanticsBuilder;
    config.isSemanticBoundary =
        _backgroundSemanticsBuilder != null || _foregroundSemanticsBuilder != null;
  }

  /// Describe the semantics of the picture painted by the [painter].
  List<SemanticsNode>? _backgroundSemanticsNodes;

  /// Describe the semantics of the picture painted by the [foregroundPainter].
  List<SemanticsNode>? _foregroundSemanticsNodes;

  @override
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    assert(() {
      if (child == null && children.isNotEmpty) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            '$runtimeType does not have a child widget but received a non-empty list of child SemanticsNode:\n'
            '${children.join('\n')}',
          ),
        ]);
      }
      return true;
    }());

    final List<CustomPainterSemantics> backgroundSemantics =
        _backgroundSemanticsBuilder != null
            ? _backgroundSemanticsBuilder!(size)
            : const <CustomPainterSemantics>[];
    _backgroundSemanticsNodes = _updateSemanticsChildren(
      _backgroundSemanticsNodes,
      backgroundSemantics,
    );

    final List<CustomPainterSemantics> foregroundSemantics =
        _foregroundSemanticsBuilder != null
            ? _foregroundSemanticsBuilder!(size)
            : const <CustomPainterSemantics>[];
    _foregroundSemanticsNodes = _updateSemanticsChildren(
      _foregroundSemanticsNodes,
      foregroundSemantics,
    );

    final bool hasBackgroundSemantics =
        _backgroundSemanticsNodes != null && _backgroundSemanticsNodes!.isNotEmpty;
    final bool hasForegroundSemantics =
        _foregroundSemanticsNodes != null && _foregroundSemanticsNodes!.isNotEmpty;
    final List<SemanticsNode> finalChildren = <SemanticsNode>[
      if (hasBackgroundSemantics) ..._backgroundSemanticsNodes!,
      ...children,
      if (hasForegroundSemantics) ..._foregroundSemanticsNodes!,
    ];
    super.assembleSemanticsNode(node, config, finalChildren);
  }

  @override
  void clearSemantics() {
    super.clearSemantics();
    _backgroundSemanticsNodes = null;
    _foregroundSemanticsNodes = null;
  }

  /// Updates the nodes of `oldSemantics` using data in `newChildSemantics`, and
  /// returns a new list containing child nodes sorted according to the order
  /// specified by `newChildSemantics`.
  ///
  /// [SemanticsNode]s that match [CustomPainterSemantics] by [Key]s preserve
  /// their [SemanticsNode.key] field. If a node with the same key appears in
  /// a different position in the list, it is moved to the new position, but the
  /// same object is reused.
  ///
  /// [SemanticsNode]s whose `key` is null may be updated from
  /// [CustomPainterSemantics] whose `key` is also null. However, the algorithm
  /// does not guarantee it. If your semantics require that specific nodes are
  /// updated from specific [CustomPainterSemantics], it is recommended to match
  /// them by specifying non-null keys.
  ///
  /// The algorithm tries to be as close to [RenderObjectElement.updateChildren]
  /// as possible, deviating only where the concepts diverge between widgets and
  /// semantics. For example, a [SemanticsNode] can be updated from a
  /// [CustomPainterSemantics] based on `Key` alone; their types are not
  /// considered because there is only one type of [SemanticsNode]. There is no
  /// concept of a "forgotten" node in semantics, deactivated nodes, or global
  /// keys.
  static List<SemanticsNode> _updateSemanticsChildren(
    List<SemanticsNode>? oldSemantics,
    List<CustomPainterSemantics>? newChildSemantics,
  ) {
    oldSemantics = oldSemantics ?? const <SemanticsNode>[];
    newChildSemantics = newChildSemantics ?? const <CustomPainterSemantics>[];

    assert(() {
      final Map<Key, int> keys = HashMap<Key, int>();
      final List<DiagnosticsNode> information = <DiagnosticsNode>[];
      for (int i = 0; i < newChildSemantics!.length; i += 1) {
        final CustomPainterSemantics child = newChildSemantics[i];
        if (child.key != null) {
          if (keys.containsKey(child.key)) {
            information.add(ErrorDescription('- duplicate key ${child.key} found at position $i'));
          }
          keys[child.key!] = i;
        }
      }

      if (information.isNotEmpty) {
        information.insert(0, ErrorSummary('Failed to update the list of CustomPainterSemantics:'));
        throw FlutterError.fromParts(information);
      }

      return true;
    }());

    int newChildrenTop = 0;
    int oldChildrenTop = 0;
    int newChildrenBottom = newChildSemantics.length - 1;
    int oldChildrenBottom = oldSemantics.length - 1;

    final List<SemanticsNode?> newChildren = List<SemanticsNode?>.filled(
      newChildSemantics.length,
      null,
    );

    // Update the top of the list.
    while ((oldChildrenTop <= oldChildrenBottom) && (newChildrenTop <= newChildrenBottom)) {
      final SemanticsNode oldChild = oldSemantics[oldChildrenTop];
      final CustomPainterSemantics newSemantics = newChildSemantics[newChildrenTop];
      if (!_canUpdateSemanticsChild(oldChild, newSemantics)) {
        break;
      }
      final SemanticsNode newChild = _updateSemanticsChild(oldChild, newSemantics);
      newChildren[newChildrenTop] = newChild;
      newChildrenTop += 1;
      oldChildrenTop += 1;
    }

    // Scan the bottom of the list.
    while ((oldChildrenTop <= oldChildrenBottom) && (newChildrenTop <= newChildrenBottom)) {
      final SemanticsNode oldChild = oldSemantics[oldChildrenBottom];
      final CustomPainterSemantics newChild = newChildSemantics[newChildrenBottom];
      if (!_canUpdateSemanticsChild(oldChild, newChild)) {
        break;
      }
      oldChildrenBottom -= 1;
      newChildrenBottom -= 1;
    }

    // Scan the old children in the middle of the list.
    final bool haveOldChildren = oldChildrenTop <= oldChildrenBottom;
    late final Map<Key, SemanticsNode> oldKeyedChildren;
    if (haveOldChildren) {
      oldKeyedChildren = <Key, SemanticsNode>{};
      while (oldChildrenTop <= oldChildrenBottom) {
        final SemanticsNode oldChild = oldSemantics[oldChildrenTop];
        if (oldChild.key != null) {
          oldKeyedChildren[oldChild.key!] = oldChild;
        }
        oldChildrenTop += 1;
      }
    }

    // Update the middle of the list.
    while (newChildrenTop <= newChildrenBottom) {
      SemanticsNode? oldChild;
      final CustomPainterSemantics newSemantics = newChildSemantics[newChildrenTop];
      if (haveOldChildren) {
        final Key? key = newSemantics.key;
        if (key != null) {
          oldChild = oldKeyedChildren[key];
          if (oldChild != null) {
            if (_canUpdateSemanticsChild(oldChild, newSemantics)) {
              // we found a match!
              // remove it from oldKeyedChildren so we don't unsync it later
              oldKeyedChildren.remove(key);
            } else {
              // Not a match, let's pretend we didn't see it for now.
              oldChild = null;
            }
          }
        }
      }
      assert(oldChild == null || _canUpdateSemanticsChild(oldChild, newSemantics));
      final SemanticsNode newChild = _updateSemanticsChild(oldChild, newSemantics);
      assert(oldChild == newChild || oldChild == null);
      newChildren[newChildrenTop] = newChild;
      newChildrenTop += 1;
    }

    // We've scanned the whole list.
    assert(oldChildrenTop == oldChildrenBottom + 1);
    assert(newChildrenTop == newChildrenBottom + 1);
    assert(newChildSemantics.length - newChildrenTop == oldSemantics.length - oldChildrenTop);
    newChildrenBottom = newChildSemantics.length - 1;
    oldChildrenBottom = oldSemantics.length - 1;

    // Update the bottom of the list.
    while ((oldChildrenTop <= oldChildrenBottom) && (newChildrenTop <= newChildrenBottom)) {
      final SemanticsNode oldChild = oldSemantics[oldChildrenTop];
      final CustomPainterSemantics newSemantics = newChildSemantics[newChildrenTop];
      assert(_canUpdateSemanticsChild(oldChild, newSemantics));
      final SemanticsNode newChild = _updateSemanticsChild(oldChild, newSemantics);
      assert(oldChild == newChild);
      newChildren[newChildrenTop] = newChild;
      newChildrenTop += 1;
      oldChildrenTop += 1;
    }

    assert(() {
      for (final SemanticsNode? node in newChildren) {
        assert(node != null);
      }
      return true;
    }());

    return newChildren.cast<SemanticsNode>();
  }

  /// Whether `oldChild` can be updated with properties from `newSemantics`.
  ///
  /// If `oldChild` can be updated, it is updated using [_updateSemanticsChild].
  /// Otherwise, the node is replaced by a new instance of [SemanticsNode].
  static bool _canUpdateSemanticsChild(
    SemanticsNode oldChild,
    CustomPainterSemantics newSemantics,
  ) {
    return oldChild.key == newSemantics.key;
  }

  /// Updates `oldChild` using the properties of `newSemantics`.
  ///
  /// This method requires that `_canUpdateSemanticsChild(oldChild, newSemantics)`
  /// is true prior to calling it.
  static SemanticsNode _updateSemanticsChild(
    SemanticsNode? oldChild,
    CustomPainterSemantics newSemantics,
  ) {
    assert(oldChild == null || _canUpdateSemanticsChild(oldChild, newSemantics));

    final SemanticsNode newChild = oldChild ?? SemanticsNode(key: newSemantics.key);

    final SemanticsProperties properties = newSemantics.properties;
    final SemanticsConfiguration config = SemanticsConfiguration();
    if (properties.sortKey != null) {
      config.sortKey = properties.sortKey;
    }
    if (properties.checked != null) {
      config.isChecked = properties.checked;
    }
    if (properties.mixed != null) {
      config.isCheckStateMixed = properties.mixed;
    }
    if (properties.selected != null) {
      config.isSelected = properties.selected!;
    }
    if (properties.button != null) {
      config.isButton = properties.button!;
    }
    if (properties.expanded != null) {
      config.isExpanded = properties.expanded;
    }
    if (properties.link != null) {
      config.isLink = properties.link!;
    }
    if (properties.linkUrl != null) {
      config.linkUrl = properties.linkUrl;
    }
    if (properties.textField != null) {
      config.isTextField = properties.textField!;
    }
    if (properties.slider != null) {
      config.isSlider = properties.slider!;
    }
    if (properties.keyboardKey != null) {
      config.isKeyboardKey = properties.keyboardKey!;
    }
    if (properties.readOnly != null) {
      config.isReadOnly = properties.readOnly!;
    }
    if (properties.focusable != null) {
      config.isFocusable = properties.focusable!;
    }
    if (properties.focused != null) {
      config.isFocused = properties.focused!;
    }
    if (properties.enabled != null) {
      config.isEnabled = properties.enabled;
    }
    if (properties.inMutuallyExclusiveGroup != null) {
      config.isInMutuallyExclusiveGroup = properties.inMutuallyExclusiveGroup!;
    }
    if (properties.obscured != null) {
      config.isObscured = properties.obscured!;
    }
    if (properties.multiline != null) {
      config.isMultiline = properties.multiline!;
    }
    if (properties.hidden != null) {
      config.isHidden = properties.hidden!;
    }
    if (properties.header != null) {
      config.isHeader = properties.header!;
    }
    if (properties.headingLevel != null) {
      config.headingLevel = properties.headingLevel!;
    }
    if (properties.scopesRoute != null) {
      config.scopesRoute = properties.scopesRoute!;
    }
    if (properties.namesRoute != null) {
      config.namesRoute = properties.namesRoute!;
    }
    if (properties.liveRegion != null) {
      config.liveRegion = properties.liveRegion!;
    }
    if (properties.maxValueLength != null) {
      config.maxValueLength = properties.maxValueLength;
    }
    if (properties.currentValueLength != null) {
      config.currentValueLength = properties.currentValueLength;
    }
    if (properties.toggled != null) {
      config.isToggled = properties.toggled;
    }
    if (properties.image != null) {
      config.isImage = properties.image!;
    }
    if (properties.label != null) {
      config.label = properties.label!;
    }
    if (properties.value != null) {
      config.value = properties.value!;
    }
    if (properties.increasedValue != null) {
      config.increasedValue = properties.increasedValue!;
    }
    if (properties.decreasedValue != null) {
      config.decreasedValue = properties.decreasedValue!;
    }
    if (properties.hint != null) {
      config.hint = properties.hint!;
    }
    if (properties.textDirection != null) {
      config.textDirection = properties.textDirection;
    }
    if (properties.onTap != null) {
      config.onTap = properties.onTap;
    }
    if (properties.onLongPress != null) {
      config.onLongPress = properties.onLongPress;
    }
    if (properties.onScrollLeft != null) {
      config.onScrollLeft = properties.onScrollLeft;
    }
    if (properties.onScrollRight != null) {
      config.onScrollRight = properties.onScrollRight;
    }
    if (properties.onScrollUp != null) {
      config.onScrollUp = properties.onScrollUp;
    }
    if (properties.onScrollDown != null) {
      config.onScrollDown = properties.onScrollDown;
    }
    if (properties.onIncrease != null) {
      config.onIncrease = properties.onIncrease;
    }
    if (properties.onDecrease != null) {
      config.onDecrease = properties.onDecrease;
    }
    if (properties.onCopy != null) {
      config.onCopy = properties.onCopy;
    }
    if (properties.onCut != null) {
      config.onCut = properties.onCut;
    }
    if (properties.onPaste != null) {
      config.onPaste = properties.onPaste;
    }
    if (properties.onMoveCursorForwardByCharacter != null) {
      config.onMoveCursorForwardByCharacter = properties.onMoveCursorForwardByCharacter;
    }
    if (properties.onMoveCursorBackwardByCharacter != null) {
      config.onMoveCursorBackwardByCharacter = properties.onMoveCursorBackwardByCharacter;
    }
    if (properties.onMoveCursorForwardByWord != null) {
      config.onMoveCursorForwardByWord = properties.onMoveCursorForwardByWord;
    }
    if (properties.onMoveCursorBackwardByWord != null) {
      config.onMoveCursorBackwardByWord = properties.onMoveCursorBackwardByWord;
    }
    if (properties.onSetSelection != null) {
      config.onSetSelection = properties.onSetSelection;
    }
    if (properties.onSetText != null) {
      config.onSetText = properties.onSetText;
    }
    if (properties.onDidGainAccessibilityFocus != null) {
      config.onDidGainAccessibilityFocus = properties.onDidGainAccessibilityFocus;
    }
    if (properties.onDidLoseAccessibilityFocus != null) {
      config.onDidLoseAccessibilityFocus = properties.onDidLoseAccessibilityFocus;
    }
    if (properties.onFocus != null) {
      config.onFocus = properties.onFocus;
    }
    if (properties.onDismiss != null) {
      config.onDismiss = properties.onDismiss;
    }

    newChild.updateWith(
      config: config,
      // As of now CustomPainter does not support multiple tree levels.
      childrenInInversePaintOrder: const <SemanticsNode>[],
    );

    newChild
      ..rect = newSemantics.rect
      ..transform = newSemantics.transform
      ..tags = newSemantics.tags;

    return newChild;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(MessageProperty('painter', '$painter'));
    properties.add(
      MessageProperty(
        'foregroundPainter',
        '$foregroundPainter',
        level: foregroundPainter != null ? DiagnosticLevel.info : DiagnosticLevel.fine,
      ),
    );
    properties.add(
      DiagnosticsProperty<Size>('preferredSize', preferredSize, defaultValue: Size.zero),
    );
    properties.add(DiagnosticsProperty<bool>('isComplex', isComplex, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('willChange', willChange, defaultValue: false));
  }
}
