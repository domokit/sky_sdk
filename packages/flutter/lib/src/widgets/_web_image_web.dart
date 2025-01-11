// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
library;

import 'dart:ui_web' as ui_web;

import 'package:flutter/foundation.dart';

import '../painting/_web_image_info_web.dart';
import '../rendering/box.dart';
import '../rendering/shifted_box.dart';
import '../web.dart' as web;
import 'basic.dart';
import 'framework.dart';
import 'platform_view.dart';

/// Displays an `<img>` element with `src` set to [src].
class ImgElementPlatformView extends StatelessWidget {
  /// Creates a platform view backed with an `<img>` element.
  ImgElementPlatformView(this.src, {super.key}) {
    if (!_registered) {
      _register();
    }
  }

  static const String _viewType = 'Flutter__ImgElementImage__';
  static bool _registered = false;

  static void _register() {
    assert(!_registered);
    _registered = true;
    ui_web.platformViewRegistry.registerViewFactory(_viewType, (int viewId, {Object? params}) {
      final Map<Object?, Object?> paramsMap = params! as Map<Object?, Object?>;
      // Create a new <img> element. The browser is able to display the image
      // without fetching it over the network again.
      final web.HTMLImageElement img = web.document.createElement('img') as web.HTMLImageElement;
      img.src = paramsMap['src']! as String;
      return img;
    });
  }

  /// The `src` URL for the `<img>` tag.
  final String? src;

  @override
  Widget build(BuildContext context) {
    if (src == null) {
      return const SizedBox.expand();
    }
    return HtmlElementView(viewType: _viewType, creationParams: <String, String?>{'src': src});
  }
}

/// A widget which displays and lays out an underlying HTML element in a
/// platform view.
class RawWebImage extends SingleChildRenderObjectWidget {
  /// Creates a [RawWebImage].
  RawWebImage({
    super.key,
    required this.image,
    this.debugImageLabel,
    this.width,
    this.height,
    this.fit,
    this.alignment = Alignment.center,
    this.matchTextDirection = false,
  }) : super(child: ImgElementPlatformView(image.htmlImage.src));

  /// The underlying HTML element to be displayed.
  final WebImageInfo image;

  /// A debug label explaining the image.
  final String? debugImageLabel;

  /// The requested width for this widget.
  final double? width;

  /// The requested height for this widget.
  final double? height;

  /// How the HTML element should be inscribed in the box constraining it.
  final BoxFit? fit;

  /// How the image should be aligned in the box constraining it.
  final AlignmentGeometry alignment;

  /// Whether or not the alignment of the image should match the text direction.
  final bool matchTextDirection;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderWebImage(
      image: image.htmlImage,
      width: width,
      height: height,
      fit: fit,
      alignment: alignment,
      matchTextDirection: matchTextDirection,
      textDirection:
          matchTextDirection || alignment is! Alignment ? Directionality.of(context) : null,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderWebImage renderObject) {
    renderObject
      ..image = image.htmlImage
      ..width = width
      ..height = height
      ..fit = fit
      ..alignment = alignment
      ..matchTextDirection = matchTextDirection
      ..textDirection =
          matchTextDirection || alignment is! Alignment ? Directionality.of(context) : null;
  }
}

/// Lays out and positions the child HTML element similarly to [RenderImage].
class RenderWebImage extends RenderShiftedBox {
  /// Creates a new [RenderWebImage].
  RenderWebImage({
    RenderBox? child,
    required web.HTMLImageElement image,
    double? width,
    double? height,
    BoxFit? fit,
    AlignmentGeometry alignment = Alignment.center,
    bool matchTextDirection = false,
    TextDirection? textDirection,
  }) : _image = image,
       _width = width,
       _height = height,
       _fit = fit,
       _alignment = alignment,
       _matchTextDirection = matchTextDirection,
       _textDirection = textDirection,
       super(child);

  Alignment? _resolvedAlignment;
  bool? _flipHorizontally;

  void _resolve() {
    if (_resolvedAlignment != null) {
      return;
    }
    _resolvedAlignment = alignment.resolve(textDirection);
    _flipHorizontally = matchTextDirection && textDirection == TextDirection.rtl;
  }

  void _markNeedResolution() {
    _resolvedAlignment = null;
    _flipHorizontally = null;
    markNeedsPaint();
  }

  /// Whether to paint the image in the direction of the [TextDirection].
  ///
  /// If this is true, then in [TextDirection.ltr] contexts, the image will be
  /// drawn with its origin in the top left (the "normal" painting direction for
  /// images); and in [TextDirection.rtl] contexts, the image will be drawn with
  /// a scaling factor of -1 in the horizontal direction so that the origin is
  /// in the top right.
  ///
  /// This is occasionally used with images in right-to-left environments, for
  /// images that were designed for left-to-right locales. Be careful, when
  /// using this, to not flip images with integral shadows, text, or other
  /// effects that will look incorrect when flipped.
  ///
  /// If this is set to true, [textDirection] must not be null.
  bool get matchTextDirection => _matchTextDirection;
  bool _matchTextDirection;
  set matchTextDirection(bool value) {
    if (value == _matchTextDirection) {
      return;
    }
    _matchTextDirection = value;
    _markNeedResolution();
  }

  /// The text direction with which to resolve [alignment].
  ///
  /// This may be changed to null, but only after the [alignment] and
  /// [matchTextDirection] properties have been changed to values that do not
  /// depend on the direction.
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    _markNeedResolution();
  }

  /// The image to display.
  web.HTMLImageElement get image => _image;
  web.HTMLImageElement _image;
  set image(web.HTMLImageElement value) {
    if (value == _image) {
      return;
    }
    // If we get a clone of our image, it's the same underlying native data -
    // return early.
    if (value.src == _image.src) {
      return;
    }
    final bool sizeChanged =
        _image.naturalWidth != value.naturalWidth || _image.naturalHeight != value.naturalHeight;
    _image = value;
    markNeedsPaint();
    if (sizeChanged && (_width == null || _height == null)) {
      markNeedsLayout();
    }
  }

  /// If non-null, requires the image to have this width.
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio.
  double? get width => _width;
  double? _width;
  set width(double? value) {
    if (value == _width) {
      return;
    }
    _width = value;
    markNeedsLayout();
  }

  /// If non-null, require the image to have this height.
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio.
  double? get height => _height;
  double? _height;
  set height(double? value) {
    if (value == _height) {
      return;
    }
    _height = value;
    markNeedsLayout();
  }

  /// How to inscribe the image into the space allocated during layout.
  ///
  /// The default varies based on the other fields. See the discussion at
  /// [paintImage].
  BoxFit? get fit => _fit;
  BoxFit? _fit;
  set fit(BoxFit? value) {
    if (value == _fit) {
      return;
    }
    _fit = value;
    markNeedsPaint();
  }

  /// How to align the image within its bounds.
  ///
  /// If this is set to a text-direction-dependent value, [textDirection] must
  /// not be null.
  AlignmentGeometry get alignment => _alignment;
  AlignmentGeometry _alignment;
  set alignment(AlignmentGeometry value) {
    if (value == _alignment) {
      return;
    }
    _alignment = value;
    _markNeedResolution();
  }

  /// Find a size for the render image within the given constraints.
  ///
  ///  - The dimensions of the RenderImage must fit within the constraints.
  ///  - The aspect ratio of the RenderImage matches the intrinsic aspect
  ///    ratio of the image.
  ///  - The RenderImage's dimension are maximal subject to being smaller than
  ///    the intrinsic size of the image.
  Size _sizeForConstraints(BoxConstraints constraints) {
    // Folds the given |width| and |height| into |constraints| so they can all
    // be treated uniformly.
    constraints = BoxConstraints.tightFor(width: _width, height: _height).enforce(constraints);

    return constraints.constrainSizeAndAttemptToPreserveAspectRatio(
      Size(_image.naturalWidth.toDouble(), _image.naturalHeight.toDouble()),
    );
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    assert(height >= 0.0);
    if (_width == null && _height == null) {
      return 0.0;
    }
    return _sizeForConstraints(BoxConstraints.tightForFinite(height: height)).width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    assert(height >= 0.0);
    return _sizeForConstraints(BoxConstraints.tightForFinite(height: height)).width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    assert(width >= 0.0);
    if (_width == null && _height == null) {
      return 0.0;
    }
    return _sizeForConstraints(BoxConstraints.tightForFinite(width: width)).height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    assert(width >= 0.0);
    return _sizeForConstraints(BoxConstraints.tightForFinite(width: width)).height;
  }

  @override
  bool hitTestSelf(Offset position) => true;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _sizeForConstraints(constraints);
  }

  @override
  void performLayout() {
    _resolve();
    assert(_resolvedAlignment != null);
    assert(_flipHorizontally != null);
    size = _sizeForConstraints(constraints);

    if (child == null) {
      return;
    }

    final Size inputSize = Size(image.naturalWidth.toDouble(), image.naturalHeight.toDouble());
    fit ??= BoxFit.scaleDown;
    final FittedSizes fittedSizes = applyBoxFit(fit!, inputSize, size);
    final Size childSize = fittedSizes.destination;
    child!.layout(BoxConstraints.tight(childSize));
    final double halfWidthDelta = (size.width - childSize.width) / 2.0;
    final double halfHeightDelta = (size.height - childSize.height) / 2.0;
    final double dx =
        halfWidthDelta +
        (_flipHorizontally! ? -_resolvedAlignment!.x : _resolvedAlignment!.x) * halfWidthDelta;
    final double dy = halfHeightDelta + _resolvedAlignment!.y * halfHeightDelta;
    final BoxParentData childParentData = child!.parentData! as BoxParentData;
    childParentData.offset = Offset(dx, dy);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<web.HTMLImageElement>('image', image));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(DoubleProperty('height', height, defaultValue: null));
    properties.add(EnumProperty<BoxFit>('fit', fit, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
  }
}
