// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../engine_canvas.dart';
import '../svg.dart';
import '../text/canvas_paragraph.dart';
import '../util.dart';
import '../vector_math.dart';
import 'bitmap_canvas.dart';
import 'painting.dart';
import 'path/path.dart';
import 'path/path_to_svg.dart';
import 'shaders/image_shader.dart';
import 'shaders/shader.dart';

/// A canvas that renders to DOM elements and CSS properties.
class DomCanvas extends EngineCanvas with SaveElementStackTracking {
  DomCanvas(this.rootElement);

  @override
  final DomElement rootElement;

  /// Prepare to reuse this canvas by clearing it's current contents.
  @override
  void clear() {
    super.clear();
    removeAllChildren(rootElement);
  }

  @override
  void clipRect(ui.Rect rect, ui.ClipOp clipOp) {
    throw UnimplementedError();
  }

  @override
  void clipRRect(ui.RRect rrect) {
    throw UnimplementedError();
  }

  @override
  void clipPath(ui.Path path) {
    throw UnimplementedError();
  }

  @override
  void drawColor(ui.Color color, ui.BlendMode blendMode) {
    // TODO(yjbanov): implement blendMode
    final DomElement box = createDomElement('draw-color');
    box.style
      ..position = 'absolute'
      ..top = '0'
      ..right = '0'
      ..bottom = '0'
      ..left = '0'
      ..backgroundColor = color.toCssString();
    currentElement.append(box);
  }

  @override
  void drawLine(ui.Offset p1, ui.Offset p2, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawPaint(SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawRect(ui.Rect rect, SurfacePaintData paint) {
    rect = adjustRectForDom(rect, paint);
    currentElement.append(buildDrawRectElement(rect, paint, 'draw-rect', currentTransform));
  }

  @override
  void drawRRect(ui.RRect rrect, SurfacePaintData paint) {
    final ui.Rect outerRect = adjustRectForDom(rrect.outerRect, paint);
    final DomElement element = buildDrawRectElement(
      outerRect,
      paint,
      'draw-rrect',
      currentTransform,
    );
    applyRRectBorderRadius(element.style, rrect);
    currentElement.append(element);
  }

  @override
  void drawDRRect(ui.RRect outer, ui.RRect inner, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawOval(ui.Rect rect, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawCircle(ui.Offset c, double radius, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawPath(ui.Path path, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {
    throw UnimplementedError();
  }

  @override
  void drawImage(ui.Image image, ui.Offset p, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawImageRect(ui.Image image, ui.Rect src, ui.Rect dst, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawParagraph(ui.Paragraph paragraph, ui.Offset offset) {
    final DomElement paragraphElement = drawParagraphElement(
      paragraph as CanvasParagraph,
      offset,
      transform: currentTransform,
    );
    currentElement.append(paragraphElement);
  }

  @override
  void drawVertices(ui.Vertices vertices, ui.BlendMode blendMode, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void drawPoints(ui.PointMode pointMode, Float32List points, SurfacePaintData paint) {
    throw UnimplementedError();
  }

  @override
  void endOfPaint() {
    // No reuse of elements yet to handle here. Noop.
  }
}

/// Converts a shadow color specified by the framework to the color that should
/// actually be applied when rendering the element.
///
/// Returns a color for box-shadow based on blur filter at sigma.
ui.Color blurColor(ui.Color color, double sigma) {
  final double strength = math.min(math.sqrt(sigma) / (math.pi * 2.0), 1.0);
  final int reducedAlpha = ((1.0 - strength) * color.alpha).round();
  return ui.Color((reducedAlpha & 0xff) << 24 | (color.value & 0x00ffffff));
}

/// When drawing a shape (rect, rrect, circle, etc) in DOM/CSS, the [rect] given
/// by Flutter needs to be adjusted to what DOM/CSS expect.
///
/// This method takes Flutter's [rect] and produces a new rect that can be used
/// to generate the correct CSS properties to match Flutter's expectations.
///
///
/// Here's what Flutter's given [rect] and [paint.strokeWidth] represent:
///
///     top-left    ↓
///              ┌──↓──────────────────────┐
///             →→→→x                   x  │←←
///              │    ┌───────────────┐    │ |
///              │    │               │    │ |
///              │    │               │    │ | height
///              │    │               │    │ |
///              │    └───────────────┘    │ |
///              │  x                   x  │←←
///              └─────────────────────────┘
/// stroke-width ↑----↑                 ↑
///                 ↑-------------------↑ width
///
///
///
/// In the DOM/CSS, here's how the coordinates should look like:
///
///   top-left   ↓
///            →→x─────────────────────────┐
///              │                         │
///              │    x───────────────x    │←←
///              │    │               │    │ |
///              │    │               │    │ | height
///              │    │               │    │ |
///              │    x───────────────x    │←←
///              │                         │
///              └─────────────────────────┘
/// border-width ↑----↑               ↑
///                   ↑---------------↑ width
///
/// As shown in the drawing above, the width/height don't start at the top-left
/// coordinates. Instead, they start from the inner top-left (inside the border).
ui.Rect adjustRectForDom(ui.Rect rect, SurfacePaintData paint) {
  double left = math.min(rect.left, rect.right);
  double top = math.min(rect.top, rect.bottom);
  double width = rect.width.abs();
  double height = rect.height.abs();

  final bool isStroke = paint.style == ui.PaintingStyle.stroke;
  final double strokeWidth = paint.strokeWidth ?? 0.0;
  if (isStroke && strokeWidth > 0.0) {
    left -= strokeWidth / 2.0;
    top -= strokeWidth / 2.0;

    // width and height shouldn't go below zero.
    width = math.max(0, width - strokeWidth);
    height = math.max(0, height - strokeWidth);
  }

  if (left != rect.left || top != rect.top || width != rect.width || height != rect.height) {
    return ui.Rect.fromLTWH(left, top, width, height);
  }
  return rect;
}

DomHTMLElement buildDrawRectElement(
  ui.Rect rect,
  SurfacePaintData paint,
  String tagName,
  Matrix4 transform,
) {
  assert(rect.left <= rect.right);
  assert(rect.top <= rect.bottom);
  final DomHTMLElement rectangle = domDocument.createElement(tagName) as DomHTMLElement;
  assert(() {
    rectangle.setAttribute('flt-rect', '$rect');
    rectangle.setAttribute('flt-paint', '$paint');
    return true;
  }());
  String effectiveTransform;
  final bool isStroke = paint.style == ui.PaintingStyle.stroke;
  final double strokeWidth = paint.strokeWidth ?? 0.0;
  if (transform.isIdentity()) {
    effectiveTransform = 'translate(${rect.left}px, ${rect.top}px)';
  } else {
    // Clone to avoid mutating `transform`.
    final Matrix4 translated = transform.clone()..translate(rect.left, rect.top);
    effectiveTransform = matrix4ToCssTransform(translated);
  }
  final DomCSSStyleDeclaration style = rectangle.style;
  style
    ..position = 'absolute'
    ..transformOrigin = '0 0 0'
    ..transform = effectiveTransform;

  String cssColor = colorValueToCssString(paint.color);

  if (paint.maskFilter != null) {
    final double sigma = paint.maskFilter!.webOnlySigma;
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit && !isStroke) {
      // A bug in webkit leaves artifacts when this element is animated
      // with filter: blur, we use boxShadow instead.
      style.boxShadow = '0px 0px ${sigma * 2.0}px $cssColor';
      cssColor = blurColor(ui.Color(paint.color), sigma).toCssString();
    } else {
      style.filter = 'blur(${sigma}px)';
    }
  }

  style
    ..width = '${rect.width}px'
    ..height = '${rect.height}px';

  if (isStroke) {
    style.border = '${_borderStrokeToCssUnit(strokeWidth)} solid $cssColor';
  } else {
    style
      ..backgroundColor = cssColor
      ..backgroundImage = _getBackgroundImageCssValue(paint.shader, rect);
  }
  return rectangle;
}

String _getBackgroundImageCssValue(ui.Shader? shader, ui.Rect bounds) {
  final String url = _getBackgroundImageUrl(shader, bounds);
  return (url != '') ? "url('$url'" : '';
}

String _getBackgroundImageUrl(ui.Shader? shader, ui.Rect bounds) {
  if (shader != null) {
    if (shader is EngineImageShader) {
      return shader.image.imgElement.src ?? '';
    }

    if (shader is EngineGradient) {
      return shader.createImageBitmap(bounds, 1, true) as String;
    }
  }
  return '';
}

void applyRRectBorderRadius(DomCSSStyleDeclaration style, ui.RRect rrect) {
  if (rrect.tlRadiusX == rrect.trRadiusX &&
      rrect.tlRadiusX == rrect.blRadiusX &&
      rrect.tlRadiusX == rrect.brRadiusX &&
      rrect.tlRadiusX == rrect.tlRadiusY &&
      rrect.trRadiusX == rrect.trRadiusY &&
      rrect.blRadiusX == rrect.blRadiusY &&
      rrect.brRadiusX == rrect.brRadiusY) {
    style.borderRadius = _borderStrokeToCssUnit(rrect.blRadiusX);
    return;
  }
  // Non-uniform. Apply each corner radius.
  style.borderTopLeftRadius =
      '${_borderStrokeToCssUnit(rrect.tlRadiusX)} '
      '${_borderStrokeToCssUnit(rrect.tlRadiusY)}';
  style.borderTopRightRadius =
      '${_borderStrokeToCssUnit(rrect.trRadiusX)} '
      '${_borderStrokeToCssUnit(rrect.trRadiusY)}';
  style.borderBottomLeftRadius =
      '${_borderStrokeToCssUnit(rrect.blRadiusX)} '
      '${_borderStrokeToCssUnit(rrect.blRadiusY)}';
  style.borderBottomRightRadius =
      '${_borderStrokeToCssUnit(rrect.brRadiusX)} '
      '${_borderStrokeToCssUnit(rrect.brRadiusY)}';
}

String _borderStrokeToCssUnit(double value) {
  if (value == 0) {
    // TODO(ferhat): hairline nees to take into account both dpi and density.
    value = 1.0;
  }
  return '${value.toStringAsFixed(3)}px';
}

SVGSVGElement pathToSvgElement(SurfacePath path, SurfacePaintData paint) {
  // In Firefox some SVG typed attributes are returned as null without a
  // setter. So we use strings here.
  final SVGSVGElement root = createSVGSVGElement()..setAttribute('overflow', 'visible');

  final SVGPathElement svgPath = createSVGPathElement();
  root.append(svgPath);
  if (paint.style == ui.PaintingStyle.stroke ||
      (paint.style != ui.PaintingStyle.fill &&
          paint.strokeWidth != 0 &&
          paint.strokeWidth != null)) {
    svgPath.setAttribute('stroke', colorValueToCssString(paint.color));
    svgPath.setAttribute('stroke-width', '${paint.strokeWidth ?? 1.0}');
    if (paint.strokeCap != null) {
      svgPath.setAttribute('stroke-linecap', '${stringForStrokeCap(paint.strokeCap)}');
    }
    svgPath.setAttribute('fill', 'none');
  } else {
    svgPath.setAttribute('fill', colorValueToCssString(paint.color));
  }
  if (path.fillType == ui.PathFillType.evenOdd) {
    svgPath.setAttribute('fill-rule', 'evenodd');
  }
  svgPath.setAttribute('d', pathToSvg(path.pathRef));
  return root;
}
