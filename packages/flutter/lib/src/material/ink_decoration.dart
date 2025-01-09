// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'ink_well.dart';
library;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'debug.dart';
import 'material.dart';

/// A convenience widget for drawing images and other decorations on [Material]
/// widgets, so that [InkWell] and [InkResponse] splashes will render over them.
///
/// Ink splashes and highlights, as rendered by [InkWell] and [InkResponse],
/// draw on the actual underlying [Material], under whatever widgets are drawn
/// over the material (such as [Text] and [Icon]s). If an opaque image is drawn
/// over the [Material] (maybe using a [Container] or [DecoratedBox]), these ink
/// effects will not be visible, as they will be entirely obscured by the opaque
/// graphics drawn above the [Material].
///
/// This widget draws the given [Decoration] directly on the [Material], in the
/// same way that [InkWell] and [InkResponse] draw there. This allows the
/// splashes to be drawn above the otherwise opaque graphics.
///
/// An alternative solution is to use a [MaterialType.transparency] material
/// above the opaque graphics, so that the ink responses from [InkWell]s and
/// [InkResponse]s will be drawn on the transparent material on top of the
/// opaque graphics, rather than under the opaque graphics on the underlying
/// [Material].
///
/// ## Limitations
///
/// This widget is subject to the same limitations as other ink effects, as
/// described in the documentation for [Material]. Most notably, the position of
/// an [Ink] widget must not change during the lifetime of the [Material] object
/// unless a [LayoutChangedNotification] is dispatched each frame that the
/// position changes. This is done automatically for [ListView] and other
/// scrolling widgets, but is not done for animated transitions such as
/// [SlideTransition].
///
/// Additionally, if multiple [Ink] widgets paint on the same [Material] in the
/// same location, their relative order is not guaranteed. The decorations will
/// be painted in the order that they were added to the material, which
/// generally speaking will match the order they are given in the widget tree,
/// but this order may appear to be somewhat random in more dynamic situations.
///
/// {@tool snippet}
///
/// This example shows how a [Material] widget can have a yellow rectangle drawn
/// on it using [Ink], while still having ink effects over the yellow rectangle:
///
/// ```dart
/// Material(
///   color: Colors.teal[900],
///   child: Center(
///     child: Ink(
///       color: Colors.yellow,
///       width: 200.0,
///       height: 100.0,
///       child: InkWell(
///         onTap: () { /* ... */ },
///         child: const Center(
///           child: Text('YELLOW'),
///         )
///       ),
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// The following example shows how an image can be printed on a [Material]
/// widget with an [InkWell] above it:
///
/// ```dart
/// Material(
///   color: Colors.grey[800],
///   child: Center(
///     child: Ink.image(
///       image: const AssetImage('cat.jpeg'),
///       fit: BoxFit.cover,
///       width: 300.0,
///       height: 200.0,
///       child: InkWell(
///         onTap: () { /* ... */ },
///         child: const Align(
///           alignment: Alignment.topLeft,
///           child: Padding(
///             padding: EdgeInsets.all(10.0),
///             child: Text(
///               'KITTEN',
///               style: TextStyle(
///                 fontWeight: FontWeight.w900,
///                 color: Colors.white,
///               ),
///             ),
///           ),
///         )
///       ),
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// What to do if you want to clip this [Ink.image]?
///
/// {@tool dartpad}
/// Wrapping the [Ink] in a clipping widget directly will not work since the
/// [Material] it will be printed on is responsible for clipping.
///
/// In this example the image is not being clipped as expected. This is because
/// it is being rendered onto the Scaffold body Material, which isn't wrapped in
/// the [ClipRRect].
///
/// ** See code in examples/api/lib/material/ink/ink.image_clip.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// One solution would be to deliberately wrap the [Ink.image] in a [Material].
/// This makes sure the Material that the image is painted on is also responsible
/// for clipping said content.
///
/// ** See code in examples/api/lib/material/ink/ink.image_clip.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Container], a more generic form of this widget which paints itself,
///    rather that deferring to the nearest [Material] widget.
///  * [InkDecoration], the [InkFeature] subclass used by this widget to paint
///    on [Material] widgets.
///  * [InkWell] and [InkResponse], which also draw on [Material] widgets.
class Ink extends StatefulWidget {
  /// Paints a decoration (which can be a simple color) on a [Material].
  ///
  /// The [height] and [width] values include the [padding].
  ///
  /// The `color` argument is a shorthand for
  /// `decoration: BoxDecoration(color: color)`, which means you cannot supply
  /// both a `color` and a `decoration` argument. If you want to have both a
  /// `color` and a `decoration`, you can pass the color as the `color`
  /// argument to the `BoxDecoration`.
  ///
  /// If there is no intention to render anything on this decoration, consider
  /// using a [Container] with a [BoxDecoration] instead.
  Ink({
    super.key,
    this.padding,
    Color? color,
    Decoration? decoration,
    this.width,
    this.height,
    this.child,
  }) : assert(padding == null || padding.isNonNegative),
       assert(decoration == null || decoration.debugAssertIsValid()),
       assert(
         color == null || decoration == null,
         'Cannot provide both a color and a decoration\n'
         'The color argument is just a shorthand for "decoration: BoxDecoration(color: color)".',
       ),
       decoration = decoration ?? (color != null ? BoxDecoration(color: color) : null);

  /// Creates a widget that shows an image (obtained from an [ImageProvider]) on
  /// a [Material].
  ///
  /// This argument is a shorthand for passing a [BoxDecoration] that has only
  /// its [BoxDecoration.image] property set to the [Ink] constructor. The
  /// properties of the [DecorationImage] of that [BoxDecoration] are set
  /// according to the arguments passed to this method.
  ///
  /// If there is no intention to render anything on this image, consider using
  /// a [Container] with a [BoxDecoration.image] instead. The `onImageError`
  /// argument may be provided to listen for errors when resolving the image.
  ///
  /// The `alignment`, `repeat`, and `matchTextDirection` arguments must not
  /// be null either, but they have default values.
  ///
  /// See [paintImage] for a description of the meaning of these arguments.
  Ink.image({
    super.key,
    this.padding,
    required ImageProvider image,
    ImageErrorListener? onImageError,
    ColorFilter? colorFilter,
    BoxFit? fit,
    AlignmentGeometry alignment = Alignment.center,
    Rect? centerSlice,
    ImageRepeat repeat = ImageRepeat.noRepeat,
    bool matchTextDirection = false,
    this.width,
    this.height,
    this.child,
  }) : assert(padding == null || padding.isNonNegative),
       decoration = BoxDecoration(
         image: DecorationImage(
           image: image,
           onError: onImageError,
           colorFilter: colorFilter,
           fit: fit,
           alignment: alignment,
           centerSlice: centerSlice,
           repeat: repeat,
           matchTextDirection: matchTextDirection,
         ),
       );

  /// The [child] contained by the container.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Empty space to inscribe inside the [decoration]. The [child], if any, is
  /// placed inside this padding.
  ///
  /// This padding is in addition to any padding inherent in the [decoration];
  /// see [Decoration.padding].
  final EdgeInsetsGeometry? padding;

  /// The decoration to paint on the nearest ancestor [Material] widget.
  ///
  /// A shorthand for specifying just a solid color is available in the
  /// constructor: set the `color` argument instead of the [decoration]
  /// argument.
  ///
  /// A shorthand for specifying just an image is also available using the
  /// [Ink.image] constructor.
  final Decoration? decoration;

  /// A width to apply to the [decoration] and the [child]. The width includes
  /// any [padding].
  final double? width;

  /// A height to apply to the [decoration] and the [child]. The height includes
  /// any [padding].
  final double? height;

  EdgeInsetsGeometry get _paddingIncludingDecoration {
    return switch ((padding, decoration?.padding)) {
      (null, null) => EdgeInsets.zero,
      (null, final EdgeInsetsGeometry padding) => padding,
      (final EdgeInsetsGeometry padding, null) => padding,
      _ => padding!.add(decoration!.padding),
    };
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(DiagnosticsProperty<Decoration>('bg', decoration, defaultValue: null));
  }

  @override
  State<Ink> createState() => _InkState();
}

class _InkState extends State<Ink> {
  final GlobalKey _boxKey = GlobalKey();
  InkDecoration? _ink;

  void _handleRemoved() {
    _ink = null;
  }

  @override
  void deactivate() {
    _ink?.dispose();
    assert(_ink == null);
    super.deactivate();
  }

  Widget _build(BuildContext context) {
    // By creating the InkDecoration from within a Builder widget, we can
    // use the RenderBox of the Padding widget.
    if (_ink == null) {
      _ink = InkDecoration(
        decoration: widget.decoration,
        isVisible: Visibility.of(context),
        configuration: createLocalImageConfiguration(context),
        controller: Material.of(context),
        referenceBox: _boxKey.currentContext!.findRenderObject()! as RenderBox,
        onRemoved: _handleRemoved,
      );
    } else {
      _ink!.decoration = widget.decoration;
      _ink!.isVisible = Visibility.of(context);
      _ink!.configuration = createLocalImageConfiguration(context);
    }
    return widget.child ?? ConstrainedBox(constraints: const BoxConstraints.expand());
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    Widget result = Padding(
      key: _boxKey,
      padding: widget._paddingIncludingDecoration,
      child: Builder(builder: _build),
    );
    if (widget.width != null || widget.height != null) {
      result = SizedBox(width: widget.width, height: widget.height, child: result);
    }
    return result;
  }
}

/// A decoration on a part of a [Material].
///
/// This object is rarely created directly. Instead of creating an ink
/// decoration directly, consider using an [Ink] widget, which uses this class
/// in combination with [Padding] and [ConstrainedBox] to draw a decoration on a
/// [Material].
///
/// See also:
///
///  * [Ink], the corresponding widget.
///  * [InkResponse], which uses gestures to trigger ink highlights and ink
///    splashes in the parent [Material].
///  * [InkWell], which is a rectangular [InkResponse] (the most common type of
///    ink response).
///  * [Material], which is the widget on which the ink is painted.
class InkDecoration extends InkFeature {
  /// Draws a decoration on a [Material].
  InkDecoration({
    required Decoration? decoration,
    bool isVisible = true,
    required ImageConfiguration configuration,
    required super.controller,
    required super.referenceBox,
    super.onRemoved,
  }) : _configuration = configuration {
    this.decoration = decoration;
    this.isVisible = isVisible;
    controller.addInkFeature(this);
  }

  BoxPainter? _painter;

  /// What to paint on the [Material].
  ///
  /// The decoration is painted at the position and size of the [referenceBox],
  /// on the [Material] that owns the [controller].
  Decoration? get decoration => _decoration;
  Decoration? _decoration;
  set decoration(Decoration? value) {
    if (value == _decoration) {
      return;
    }
    _decoration = value;
    _painter?.dispose();
    _painter = _decoration?.createBoxPainter(_handleChanged);
    controller.markNeedsPaint();
  }

  /// Whether the decoration should be painted.
  ///
  /// Defaults to true.
  bool get isVisible => _isVisible;
  bool _isVisible = true;
  set isVisible(bool value) {
    if (value == _isVisible) {
      return;
    }
    _isVisible = value;
    controller.markNeedsPaint();
  }

  /// The configuration to pass to the [BoxPainter] obtained from the
  /// [decoration], when painting.
  ///
  /// The [ImageConfiguration.size] field is ignored (and replaced by the size
  /// of the [referenceBox], at paint time).
  ImageConfiguration get configuration => _configuration;
  ImageConfiguration _configuration;
  set configuration(ImageConfiguration value) {
    if (value == _configuration) {
      return;
    }
    _configuration = value;
    controller.markNeedsPaint();
  }

  void _handleChanged() {
    controller.markNeedsPaint();
  }

  @override
  void dispose() {
    _painter?.dispose();
    super.dispose();
  }

  @override
  void paintFeature(Canvas canvas, Matrix4 transform) {
    if (_painter == null || !isVisible) {
      return;
    }
    final Offset? originOffset = MatrixUtils.getAsTranslation(transform);
    final ImageConfiguration sizedConfiguration = configuration.copyWith(size: referenceBox.size);
    if (originOffset == null) {
      canvas.save();
      canvas.transform(transform.storage);
      _painter!.paint(canvas, Offset.zero, sizedConfiguration);
      canvas.restore();
    } else {
      _painter!.paint(canvas, originOffset, sizedConfiguration);
    }
  }
}
