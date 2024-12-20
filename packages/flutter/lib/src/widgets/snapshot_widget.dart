// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';
import 'media_query.dart';

/// Controls how the [SnapshotWidget] paints its child.
enum SnapshotMode {
  /// The child is snapshotted, but only if all descendants can be snapshotted.
  ///
  /// If there is a platform view in the children of a snapshot widget, the
  /// snapshot will not be used and the child will be rendered using
  /// [SnapshotPainter.paint]. This uses an un-snapshotted child and by default
  /// paints it with no additional modification.
  permissive,

  /// An error is thrown if the child cannot be snapshotted.
  ///
  /// This setting is the default state of the [SnapshotWidget].
  normal,

  /// The child is snapshotted and any child platform views are ignored.
  ///
  /// This mode can be useful if there is a platform view descendant that does
  /// not need to be included in the snapshot.
  forced,
}

/// A controller for the [SnapshotWidget] that controls when the child image is displayed
/// and when to regenerated the child image.
///
/// When the value of [allowSnapshotting] is true, the [SnapshotWidget] will paint the child
/// widgets based on the [SnapshotMode] of the snapshot widget.
///
/// The controller notifies its listeners when the value of [allowSnapshotting] changes
/// or when [clear] is called.
///
/// To force [SnapshotWidget] to recreate the child image, call [clear].
class SnapshotController extends ChangeNotifier {
  /// Create a new [SnapshotController].
  ///
  /// By default, [allowSnapshotting] is `false` and cannot be `null`.
  SnapshotController({bool allowSnapshotting = false}) : _allowSnapshotting = allowSnapshotting;

  /// Reset the snapshot held by any listening [SnapshotWidget].
  ///
  /// This has no effect if [allowSnapshotting] is `false`.
  void clear() {
    notifyListeners();
  }

  /// Whether a snapshot of this child widget is painted in its place.
  bool get allowSnapshotting => _allowSnapshotting;
  bool _allowSnapshotting;
  set allowSnapshotting(bool value) {
    if (value == allowSnapshotting) {
      return;
    }
    _allowSnapshotting = value;
    notifyListeners();
  }
}

/// A widget that can replace its child with a snapshotted version of the child.
///
/// A snapshot is a frozen texture-backed representation of all child pictures
/// and layers stored as a [ui.Image].
///
/// This widget is useful for performing short animations that would otherwise
/// be expensive or that cannot rely on raster caching. For example, scale and
/// skew animations are often expensive to perform on complex children, as are
/// blurs. For a short animation, a widget that contains these expensive effects
/// can be replaced with a snapshot of itself and manipulated instead.
///
/// For example, the Android Q [ZoomPageTransitionsBuilder] uses a snapshot widget
/// for the forward and entering route to avoid the expensive scale animation.
/// This also has the effect of briefly pausing any animations on the page.
///
/// Generally, this widget should not be used in places where users expect the
/// child widget to continue animating or to be responsive, such as an unbounded
/// animation.
///
/// Caveats:
///
/// * The contents of platform views cannot be captured by a snapshot
///   widget. If a platform view is encountered, then the snapshot widget will
///   determine how to render its children based on the [SnapshotMode]. This
///   defaults to [SnapshotMode.normal] which will throw an exception if a
///   platform view is encountered.
///
/// * The snapshotting functionality of this widget is not supported on the HTML
///   backend of Flutter for the Web. Setting [SnapshotController.allowSnapshotting] to true
///   may cause an error to be thrown. On the CanvasKit backend of Flutter, the
///   performance of using this widget may regress performance due to the fact
///   that both the UI and engine share a single thread.
class SnapshotWidget extends SingleChildRenderObjectWidget {
  /// Create a new [SnapshotWidget].
  ///
  /// The [controller] and [child] arguments are required.
  const SnapshotWidget({
    super.key,
    this.mode = SnapshotMode.normal,
    this.painter = const _DefaultSnapshotPainter(),
    this.autoresize = false,
    required this.controller,
    required super.child,
  });

  /// The controller that determines when to display the children as a snapshot.
  final SnapshotController controller;

  /// Configuration that controls how the snapshot widget decides to paint its children.
  ///
  /// Defaults to [SnapshotMode.normal], which throws an error when a platform view
  /// or texture view is encountered.
  ///
  /// See [SnapshotMode] for more information.
  final SnapshotMode mode;

  /// Whether or not changes in render object size should automatically re-create
  /// the snapshot.
  ///
  /// Defaults to false.
  final bool autoresize;

  /// The painter used to paint the child snapshot or child widgets.
  final SnapshotPainter painter;

  @override
  RenderObject createRenderObject(BuildContext context) {
    debugCheckHasMediaQuery(context);
    return _RenderSnapshotWidget(
      controller: controller,
      mode: mode,
      devicePixelRatio: MediaQuery.devicePixelRatioOf(context),
      painter: painter,
      autoresize: autoresize,
    );
  }

  @override
  void updateRenderObject(BuildContext context, covariant RenderObject renderObject) {
    debugCheckHasMediaQuery(context);
    (renderObject as _RenderSnapshotWidget)
      ..controller = controller
      ..mode = mode
      ..devicePixelRatio = MediaQuery.devicePixelRatioOf(context)
      ..painter = painter
      ..autoresize = autoresize;
  }
}

// A render object that conditionally converts its child into a [ui.Image]
// and then paints it in place of the child.
class _RenderSnapshotWidget extends RenderProxyBox {
  // Create a new [_RenderSnapshotWidget].
  _RenderSnapshotWidget({
    required double devicePixelRatio,
    required SnapshotController controller,
    required SnapshotMode mode,
    required SnapshotPainter painter,
    required bool autoresize,
  }) : _devicePixelRatio = devicePixelRatio,
       _controller = controller,
       _mode = mode,
       _painter = painter,
       _autoresize = autoresize;

  /// The device pixel ratio used to create the child image.
  double get devicePixelRatio => _devicePixelRatio;
  double _devicePixelRatio;
  set devicePixelRatio(double value) {
    if (value == devicePixelRatio) {
      return;
    }
    _devicePixelRatio = value;
    if (_childRaster == null) {
      return;
    } else {
      _childRaster?.dispose();
      _childRaster = null;
      markNeedsPaint();
    }
  }

  /// The painter used to paint the child snapshot or child widgets.
  SnapshotPainter get painter => _painter;
  SnapshotPainter _painter;
  set painter(SnapshotPainter value) {
    if (value == painter) {
      return;
    }
    final SnapshotPainter oldPainter = painter;
    oldPainter.removeListener(markNeedsPaint);
    _painter = value;
    if (oldPainter.runtimeType != painter.runtimeType || painter.shouldRepaint(oldPainter)) {
      markNeedsPaint();
    }
    if (attached) {
      painter.addListener(markNeedsPaint);
    }
  }

  /// A controller that determines whether to paint the child normally or to
  /// paint a snapshotted version of that child.
  SnapshotController get controller => _controller;
  SnapshotController _controller;
  set controller(SnapshotController value) {
    if (value == controller) {
      return;
    }
    controller.removeListener(_onRasterValueChanged);
    final bool oldValue = controller.allowSnapshotting;
    _controller = value;
    if (attached) {
      controller.addListener(_onRasterValueChanged);
      if (oldValue != controller.allowSnapshotting) {
        _onRasterValueChanged();
      }
    }
  }

  /// How the snapshot widget will handle platform views in child layers.
  SnapshotMode get mode => _mode;
  SnapshotMode _mode;
  set mode(SnapshotMode value) {
    if (value == _mode) {
      return;
    }
    _mode = value;
    markNeedsPaint();
  }

  /// Whether or not changes in render object size should automatically re-rasterize.
  bool get autoresize => _autoresize;
  bool _autoresize;
  set autoresize(bool value) {
    if (value == autoresize) {
      return;
    }
    _autoresize = value;
    markNeedsPaint();
  }

  ui.Image? _childRaster;
  Size? _childRasterSize;
  // Set to true if the snapshot mode was not forced and a platform view
  // was encountered while attempting to snapshot the child.
  bool _disableSnapshotAttempt = false;

  @override
  void attach(covariant PipelineOwner owner) {
    controller.addListener(_onRasterValueChanged);
    painter.addListener(markNeedsPaint);
    super.attach(owner);
  }

  @override
  void detach() {
    _disableSnapshotAttempt = false;
    controller.removeListener(_onRasterValueChanged);
    painter.removeListener(markNeedsPaint);
    _childRaster?.dispose();
    _childRaster = null;
    _childRasterSize = null;
    super.detach();
  }

  @override
  void dispose() {
    controller.removeListener(_onRasterValueChanged);
    painter.removeListener(markNeedsPaint);
    _childRaster?.dispose();
    _childRaster = null;
    _childRasterSize = null;
    super.dispose();
  }

  void _onRasterValueChanged() {
    _disableSnapshotAttempt = false;
    _childRaster?.dispose();
    _childRaster = null;
    _childRasterSize = null;
    markNeedsPaint();
  }

  // Paint [child] with this painting context, then convert to a raster and detach all
  // children from this layer.
  ui.Image? _paintAndDetachToImage() {
    final OffsetLayer offsetLayer = OffsetLayer();
    final PaintingContext context = PaintingContext(offsetLayer, Offset.zero & size);
    super.paint(context, Offset.zero);
    // This ignore is here because this method is protected by the `PaintingContext`. Adding a new
    // method that performs the work of `_paintAndDetachToImage` would avoid the need for this, but
    // that would conflict with our goals of minimizing painting context.
    // ignore: invalid_use_of_protected_member
    context.stopRecordingIfNeeded();
    if (mode != SnapshotMode.forced && !offsetLayer.supportsRasterization()) {
      offsetLayer.dispose();
      if (mode == SnapshotMode.normal) {
        throw FlutterError('SnapshotWidget used with a child that contains a PlatformView.');
      }
      _disableSnapshotAttempt = true;
      return null;
    }
    final ui.Image image = offsetLayer.toImageSync(
      Offset.zero & size,
      pixelRatio: devicePixelRatio,
    );
    offsetLayer.dispose();
    _lastCachedSize = size;
    return image;
  }

  Size? _lastCachedSize;

  @override
  void paint(PaintingContext context, Offset offset) {
    if (size.isEmpty) {
      _childRaster?.dispose();
      _childRaster = null;
      _childRasterSize = null;
      return;
    }
    if (!controller.allowSnapshotting || _disableSnapshotAttempt) {
      _childRaster?.dispose();
      _childRaster = null;
      _childRasterSize = null;
      painter.paint(context, offset, size, super.paint);
      return;
    }

    if (autoresize && size != _lastCachedSize && _lastCachedSize != null) {
      _childRaster?.dispose();
      _childRaster = null;
    }

    if (_childRaster == null) {
      _childRaster = _paintAndDetachToImage();
      _childRasterSize = size * devicePixelRatio;
    }
    if (_childRaster == null) {
      painter.paint(context, offset, size, super.paint);
    } else {
      painter.paintSnapshot(
        context,
        offset,
        size,
        _childRaster!,
        _childRasterSize!,
        devicePixelRatio,
      );
    }
  }
}

/// A painter used to paint either a snapshot or the child widgets that
/// would be a snapshot.
///
/// The painter can call [notifyListeners] to have the [SnapshotWidget]
/// re-paint (re-using the same raster). This allows animations to be performed
/// without re-snapshotting of children. For certain scale or perspective changing
/// transforms, such as a rotation, this can be significantly faster than performing
/// the same animation at the widget level.
///
/// By default, the [SnapshotWidget] includes a delegate that draws the child raster
/// exactly as the child widgets would have been drawn. Nevertheless, this can
/// also be used to efficiently transform the child raster and apply complex paint
/// effects.
///
/// {@tool snippet}
///
/// The following method shows how to efficiently rotate the child raster.
///
/// ```dart
/// void paint(PaintingContext context, Offset offset, Size size, ui.Image image, double pixelRatio) {
///   const double radians = 0.5; // Could be driven by an animation.
///   final Matrix4 transform = Matrix4.rotationZ(radians);
///   context.canvas.transform(transform.storage);
///   final Rect src = Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble());
///   final Rect dst = Rect.fromLTWH(offset.dx, offset.dy, size.width, size.height);
///   final Paint paint = Paint()
///     ..filterQuality = FilterQuality.medium;
///   context.canvas.drawImageRect(image, src, dst, paint);
/// }
/// ```
/// {@end-tool}
abstract class SnapshotPainter extends ChangeNotifier {
  /// Creates an instance of [SnapshotPainter].
  SnapshotPainter() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// Called whenever the [image] that represents a [SnapshotWidget]s child should be painted.
  ///
  /// The image is rasterized at the physical pixel resolution and should be scaled down by
  /// [pixelRatio] to account for device independent pixels.
  ///
  /// {@tool snippet}
  ///
  /// The following method shows how the default implementation of the delegate used by the
  /// [SnapshotPainter] paints the snapshot. This must account for the fact that the image
  /// width and height will be given in physical pixels, while the image must be painted with
  /// device independent pixels. That is, the width and height of the image is the widget and
  /// height of the provided `size`, multiplied by the `pixelRatio`. In addition, the actual
  /// size of the scene captured by the `image` is not `image.width` or `image.height`, but
  /// indeed `sourceSize`, because the former is a rounded inaccurate integer:
  ///
  /// ```dart
  /// void paint(PaintingContext context, Offset offset, Size size, ui.Image image, Size sourceSize, double pixelRatio) {
  ///   final Rect src = Rect.fromLTWH(0, 0, sourceSize.width, sourceSize.height);
  ///   final Rect dst = Rect.fromLTWH(offset.dx, offset.dy, size.width, size.height);
  ///   final Paint paint = Paint()
  ///     ..filterQuality = FilterQuality.medium;
  ///   context.canvas.drawImageRect(image, src, dst, paint);
  /// }
  /// ```
  /// {@end-tool}
  void paintSnapshot(
    PaintingContext context,
    Offset offset,
    Size size,
    ui.Image image,
    Size sourceSize,
    double pixelRatio,
  );

  /// Paint the child via [painter], applying any effects that would have been painted
  /// in [SnapshotPainter.paintSnapshot].
  ///
  /// This method is called when snapshotting is disabled, or when [SnapshotMode.permissive]
  /// is used and a child platform view prevents snapshotting.
  ///
  /// The [offset] and [size] are the location and dimensions of the render object.
  void paint(PaintingContext context, Offset offset, Size size, PaintingContextCallback painter);

  /// Called whenever a new instance of the snapshot widget delegate class is
  /// provided to the [SnapshotWidget] object, or any time that a new
  /// [SnapshotPainter] object is created with a new instance of the
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
  /// Changing the delegate will not cause the child image retained by the
  /// [SnapshotWidget] to be updated. Instead, [SnapshotController.clear] can
  /// be used to force the generation of a new image.
  ///
  /// The `oldPainter` argument will never be null.
  bool shouldRepaint(covariant SnapshotPainter oldPainter);
}

class _DefaultSnapshotPainter implements SnapshotPainter {
  const _DefaultSnapshotPainter();

  @override
  void addListener(ui.VoidCallback listener) {}

  @override
  void dispose() {}

  @override
  bool get hasListeners => false;

  @override
  void notifyListeners() {}

  @override
  void paint(
    PaintingContext context,
    ui.Offset offset,
    ui.Size size,
    PaintingContextCallback painter,
  ) {
    painter(context, offset);
  }

  @override
  void paintSnapshot(
    PaintingContext context,
    ui.Offset offset,
    ui.Size size,
    ui.Image image,
    Size sourceSize,
    double pixelRatio,
  ) {
    final Rect src = Rect.fromLTWH(0, 0, sourceSize.width, sourceSize.height);
    final Rect dst = Rect.fromLTWH(offset.dx, offset.dy, size.width, size.height);
    final Paint paint = Paint()..filterQuality = FilterQuality.medium;
    context.canvas.drawImageRect(image, src, dst, paint);
  }

  @override
  void removeListener(ui.VoidCallback listener) {}

  @override
  bool shouldRepaint(covariant _DefaultSnapshotPainter oldPainter) => false;
}
