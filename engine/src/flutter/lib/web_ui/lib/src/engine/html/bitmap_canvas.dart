// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../canvas_pool.dart';
import '../display.dart';
import '../dom.dart';
import '../engine_canvas.dart';
import '../frame_reference.dart';
import '../text/canvas_paragraph.dart';
import '../util.dart';
import '../vector_math.dart';
import 'clip.dart';
import 'color_filter.dart';
import 'dom_canvas.dart';
import 'image.dart';
import 'painting.dart';
import 'path/path.dart';
import 'recording_canvas.dart';
import 'render_vertices.dart';
import 'shaders/image_shader.dart';
import 'shaders/shader.dart';

/// A raw HTML canvas that is directly written to.
class BitmapCanvas extends EngineCanvas {
  /// Allocates a canvas with enough memory to paint a picture within the given
  /// [bounds].
  ///
  /// This canvas can be reused by pictures with different paint bounds as long
  /// as the [Rect.size] of the bounds fully fit within the size used to
  /// initialize this canvas.
  BitmapCanvas(this._bounds, RenderStrategy renderStrategy, {double density = 1.0})
    : _density = density,
      _renderStrategy = renderStrategy,
      widthInBitmapPixels = widthToPhysical(_bounds.width),
      heightInBitmapPixels = heightToPhysical(_bounds.height),
      _canvasPool = CanvasPool(
        widthToPhysical(_bounds.width),
        heightToPhysical(_bounds.height),
        density,
      ) {
    rootElement.style.position = 'absolute';
    // Adds one extra pixel to the requested size. This is to compensate for
    // _initializeViewport() snapping canvas position to 1 pixel, causing
    // painting to overflow by at most 1 pixel.
    _canvasPositionX = _bounds.left.floor() - kPaddingPixels;
    _canvasPositionY = _bounds.top.floor() - kPaddingPixels;
    _updateRootElementTransform();
    _canvasPool.mount(rootElement as DomHTMLElement);
    _setupInitialTransform();
  }

  /// Constructs bitmap canvas to capture image data.
  factory BitmapCanvas.imageData(ui.Rect bounds) {
    final BitmapCanvas bitmapCanvas = BitmapCanvas(bounds, RenderStrategy());
    bitmapCanvas._preserveImageData = true;
    return bitmapCanvas;
  }

  /// The rectangle positioned relative to the parent layer's coordinate
  /// system's origin, within which this canvas paints.
  ///
  /// Painting outside these bounds will result in cropping.
  ui.Rect get bounds => _bounds;
  set bounds(ui.Rect newValue) {
    _bounds = newValue;
    final int newCanvasPositionX = _bounds.left.floor() - kPaddingPixels;
    final int newCanvasPositionY = _bounds.top.floor() - kPaddingPixels;
    if (_canvasPositionX != newCanvasPositionX || _canvasPositionY != newCanvasPositionY) {
      _canvasPositionX = newCanvasPositionX;
      _canvasPositionY = newCanvasPositionY;
      _updateRootElementTransform();
    }
  }

  ui.Rect _bounds;
  CrossFrameCache<DomHTMLElement>? _elementCache;

  /// The amount of padding to add around the edges of this canvas to
  /// ensure that anti-aliased arcs are not clipped.
  static const int kPaddingPixels = 1;

  @override
  final DomElement rootElement = createDomElement('flt-canvas');

  final CanvasPool _canvasPool;

  /// The size of the paint [bounds].
  ui.Size get size => _bounds.size;

  /// The last CSS font string is cached to optimize the case where the font
  /// styles hasn't changed.
  String? _cachedLastCssFont;

  /// List of extra sibling elements created for paragraphs and clipping.
  final List<DomElement> _children = <DomElement>[];

  /// The number of pixels along the width of the bitmap that the canvas element
  /// renders into.
  ///
  /// These pixels are different from the logical CSS pixels. Here a pixel
  /// literally means 1 point with a RGBA color.
  final int widthInBitmapPixels;

  /// The number of pixels along the width of the bitmap that the canvas element
  /// renders into.
  ///
  /// These pixels are different from the logical CSS pixels. Here a pixel
  /// literally means 1 point with a RGBA color.
  final int heightInBitmapPixels;

  /// The number of pixels in the bitmap that the canvas element renders into.
  ///
  /// These pixels are different from the logical CSS pixels. Here a pixel
  /// literally means 1 point with a RGBA color.
  int get bitmapPixelCount => widthInBitmapPixels * heightInBitmapPixels;

  int _saveCount = 0;

  /// Keeps track of what device pixel ratio was used when this [BitmapCanvas]
  /// was created.
  final double _devicePixelRatio = EngineFlutterDisplay.instance.browserDevicePixelRatio;

  // Compensation for [_initializeViewport] snapping canvas position to 1 pixel.
  int? _canvasPositionX, _canvasPositionY;

  // Indicates the instructions following drawImage or drawParagraph that
  // a child element was created to paint.
  // TODO(yjbanov): When childElements are created by
  // drawImage/drawParagraph commands, compositing order is not correctly
  // handled when we interleave these with other paint commands.
  // To solve this, recording canvas will have to check the paint queue
  // and send a hint to EngineCanvas that additional canvas layers need
  // to be used to composite correctly. In practice this is very rare
  // with Widgets but CustomPainter(s) can hit this code path.
  bool _childOverdraw = false;

  /// Forces text to be drawn using HTML rather than bitmap.
  ///
  /// Use this for tests only.
  set debugChildOverdraw(bool value) {
    _childOverdraw = value;
  }

  /// Indicates bitmap canvas contains a 3d transform.
  /// WebKit fails to preserve paint order when this happens and therefore
  /// requires insertion of <div style="transform: translate3d(0,0,0);"> to be
  /// used for each child to force correct rendering order.
  bool _contains3dTransform = false;

  /// Indicates that contents should be rendered into canvas so a dataUrl
  /// can be constructed from contents.
  bool _preserveImageData = false;

  /// Canvas pixel to screen pixel ratio. Similar to dpi but
  /// uses global transform of canvas to compute ratio.
  double get density => _density;
  final double _density;

  final RenderStrategy _renderStrategy;

  /// Setup cache for reusing DOM elements across frames.
  void setElementCache(CrossFrameCache<DomHTMLElement>? cache) {
    _elementCache = cache;
  }

  void _updateRootElementTransform() {
    // Flutter emits paint operations positioned relative to the parent layer's
    // coordinate system. However, canvas' coordinate system's origin is always
    // in the top-left corner of the canvas. We therefore need to inject an
    // initial translation so the paint operations are positioned as expected.
    //
    // The flooring of the value is to ensure that canvas' top-left corner
    // lands on the physical pixel.
    // TODO(yjbanov): !This is not accurate if there are
    // transforms higher up in the stack.
    rootElement.style.transform = 'translate(${_canvasPositionX}px, ${_canvasPositionY}px)';
  }

  void _setupInitialTransform() {
    final double canvasPositionCorrectionX =
        _bounds.left - BitmapCanvas.kPaddingPixels - _canvasPositionX!.toDouble();
    final double canvasPositionCorrectionY =
        _bounds.top - BitmapCanvas.kPaddingPixels - _canvasPositionY!.toDouble();
    // This compensates for the translate on the `rootElement`.
    _canvasPool.initialTransform = ui.Offset(
      -_bounds.left + canvasPositionCorrectionX + BitmapCanvas.kPaddingPixels,
      -_bounds.top + canvasPositionCorrectionY + BitmapCanvas.kPaddingPixels,
    );
  }

  static int widthToPhysical(double width) {
    final double boundsWidth = width + 1;
    return (boundsWidth * EngineFlutterDisplay.instance.browserDevicePixelRatio).ceil() +
        2 * kPaddingPixels;
  }

  static int heightToPhysical(double height) {
    final double boundsHeight = height + 1;
    return (boundsHeight * EngineFlutterDisplay.instance.browserDevicePixelRatio).ceil() +
        2 * kPaddingPixels;
  }

  // Used by picture to assess if canvas is large enough to reuse as is.
  bool doesFitBounds(ui.Rect newBounds, double newDensity) {
    return widthInBitmapPixels >= widthToPhysical(newBounds.width) &&
        heightInBitmapPixels >= heightToPhysical(newBounds.height) &&
        _density == newDensity;
  }

  @override
  void dispose() {
    _canvasPool.dispose();
  }

  /// Prepare to reuse this canvas by clearing it's current contents.
  @override
  void clear() {
    _contains3dTransform = false;
    _canvasPool.clear();
    final int len = _children.length;
    for (int i = 0; i < len; i++) {
      final DomElement child = _children[i];
      // Don't remove children that have been reused by CrossFrameCache.
      if (child.parentNode == rootElement) {
        child.remove();
      }
    }
    _children.clear();
    _childOverdraw = false;
    _cachedLastCssFont = null;
    _setupInitialTransform();
  }

  /// Checks whether this [BitmapCanvas] can still be recycled and reused.
  ///
  /// See also:
  ///
  /// * [PersistedPicture._applyBitmapPaint] which uses this method to
  ///   decide whether to reuse this canvas or not.
  /// * [PersistedPicture._recycleCanvas] which also uses this method
  ///   for the same reason.
  bool isReusable() {
    return _devicePixelRatio == EngineFlutterDisplay.instance.browserDevicePixelRatio;
  }

  /// Returns a "data://" URI containing a representation of the image in this
  /// canvas in PNG format.
  String toDataUrl() {
    return _canvasPool.toDataUrl();
  }

  /// Sets the global paint styles to correspond to [paint].
  void setUpPaint(SurfacePaintData paint, ui.Rect? shaderBounds) {
    _canvasPool.contextHandle.setUpPaint(paint, shaderBounds);
  }

  void tearDownPaint() {
    _canvasPool.contextHandle.tearDownPaint();
  }

  @override
  int save() {
    _canvasPool.save();
    return _saveCount++;
  }

  void saveLayer(ui.Rect bounds, ui.Paint paint) {
    save();
  }

  @override
  void restore() {
    _canvasPool.restore();
    _saveCount--;
    _cachedLastCssFont = null;
  }

  // TODO(yjbanov): not sure what this is attempting to do, but it is probably
  //                wrong because some clips and transforms are expressed using
  //                HTML DOM elements.
  void restoreToCount(int count) {
    assert(_saveCount >= count);
    final int restores = _saveCount - count;
    for (int i = 0; i < restores; i++) {
      _canvasPool.restore();
    }
    _saveCount = count;
  }

  @override
  void translate(double dx, double dy) {
    _canvasPool.translate(dx, dy);
  }

  @override
  void scale(double sx, double sy) {
    _canvasPool.scale(sx, sy);
  }

  @override
  void rotate(double radians) {
    _canvasPool.rotate(radians);
  }

  @override
  void skew(double sx, double sy) {
    _canvasPool.skew(sx, sy);
  }

  @override
  void transform(Float32List matrix4) {
    final TransformKind transformKind = transformKindOf(matrix4);
    if (transformKind == TransformKind.complex) {
      _contains3dTransform = true;
    }
    _canvasPool.transform(matrix4);
  }

  @override
  void clipRect(ui.Rect rect, ui.ClipOp clipOp) {
    if (clipOp == ui.ClipOp.difference) {
      // Create 2 rectangles inside each other that represents
      // clip area difference using even-odd fill rule.
      final SurfacePath path = SurfacePath();
      path.fillType = ui.PathFillType.evenOdd;
      path.addRect(ui.Rect.fromLTWH(0, 0, _bounds.width, _bounds.height));
      path.addRect(rect);
      _canvasPool.clipPath(path);
    } else {
      _canvasPool.clipRect(rect);
    }
  }

  @override
  void clipRRect(ui.RRect rrect) {
    _canvasPool.clipRRect(rrect);
  }

  @override
  void clipPath(ui.Path path) {
    _canvasPool.clipPath(path);
  }

  /// Whether drawing operation should use DOM node instead of Canvas.
  ///
  /// - Perspective transforms are not supported by canvas and require
  ///   DOM to render correctly.
  /// - Pictures typically have large rect/rounded rectangles as background
  ///   prefer DOM if canvas has not been allocated yet.
  ///
  bool _useDomForRenderingFill(SurfacePaintData paint) {
    if (_preserveImageData) {
      return false;
    }
    return _renderStrategy.isInsideSvgFilterTree ||
        _contains3dTransform ||
        (_childOverdraw &&
            !_canvasPool.hasCanvas &&
            paint.maskFilter == null &&
            paint.shader == null &&
            paint.style != ui.PaintingStyle.stroke);
  }

  /// Same as [_useDomForRenderingFill] but allows stroke as well.
  ///
  /// DOM canvas is generated for simple strokes using borders.
  bool _useDomForRenderingFillAndStroke(SurfacePaintData paint) {
    if (_preserveImageData) {
      return false;
    }
    return _renderStrategy.isInsideSvgFilterTree ||
        _contains3dTransform ||
        ((_childOverdraw || _renderStrategy.hasImageElements || _renderStrategy.hasParagraphs) &&
            !_canvasPool.hasCanvas &&
            paint.maskFilter == null &&
            paint.shader == null);
  }

  @override
  void drawColor(ui.Color color, ui.BlendMode blendMode) {
    final SurfacePaintData paintData =
        SurfacePaintData()
          ..color = color.value
          ..blendMode = blendMode;
    if (_useDomForRenderingFill(paintData)) {
      drawRect(_computeScreenBounds(_canvasPool.currentTransform), paintData);
    } else {
      _canvasPool.drawColor(color, blendMode);
    }
  }

  @override
  void drawLine(ui.Offset p1, ui.Offset p2, SurfacePaintData paint) {
    if (_useDomForRenderingFill(paint)) {
      final SurfacePath path =
          SurfacePath()
            ..moveTo(p1.dx, p1.dy)
            ..lineTo(p2.dx, p2.dy);
      drawPath(path, paint);
    } else {
      final ui.Rect? shaderBounds = (paint.shader != null) ? ui.Rect.fromPoints(p1, p2) : null;
      setUpPaint(paint, shaderBounds);
      _canvasPool.strokeLine(p1, p2);
      tearDownPaint();
    }
  }

  @override
  void drawPaint(SurfacePaintData paint) {
    if (_useDomForRenderingFill(paint)) {
      drawRect(_computeScreenBounds(_canvasPool.currentTransform), paint);
    } else {
      final ui.Rect? shaderBounds = (paint.shader != null) ? _computePictureBounds() : null;
      setUpPaint(paint, shaderBounds);
      _canvasPool.fill();
      tearDownPaint();
    }
  }

  @override
  void drawRect(ui.Rect rect, SurfacePaintData paint) {
    if (_useDomForRenderingFillAndStroke(paint)) {
      rect = adjustRectForDom(rect, paint);
      final DomHTMLElement element = buildDrawRectElement(
        rect,
        paint,
        'draw-rect',
        _canvasPool.currentTransform,
      );
      _drawElement(element, rect.topLeft, paint);
    } else {
      setUpPaint(paint, rect);
      _canvasPool.drawRect(rect, paint.style);
      tearDownPaint();
    }
  }

  /// Inserts a dom element at [offset] creating stack of divs for clipping
  /// if required.
  void _drawElement(DomElement element, ui.Offset offset, SurfacePaintData paint) {
    if (_canvasPool.isClipped) {
      final List<DomElement> clipElements = _clipContent(
        _canvasPool.clipStack!,
        element,
        ui.Offset.zero,
        transformWithOffset(_canvasPool.currentTransform, offset),
      );
      for (final DomElement clipElement in clipElements) {
        rootElement.append(clipElement);
        _children.add(clipElement);
      }
    } else {
      rootElement.append(element);
      _children.add(element);
    }
    final ui.BlendMode? blendMode = paint.blendMode;
    if (blendMode != null) {
      element.style.mixBlendMode = blendModeToCssMixBlendMode(blendMode) ?? '';
    }
    // Switch to preferring DOM from now on, and close the current canvas.
    _closeCanvas();
  }

  @override
  void drawRRect(ui.RRect rrect, SurfacePaintData paint) {
    if (_useDomForRenderingFillAndStroke(paint)) {
      final ui.Rect rect = adjustRectForDom(rrect.outerRect, paint);
      final DomHTMLElement element = buildDrawRectElement(
        rect,
        paint,
        'draw-rrect',
        _canvasPool.currentTransform,
      );
      applyRRectBorderRadius(element.style, rrect);
      _drawElement(element, rect.topLeft, paint);
    } else {
      setUpPaint(paint, rrect.outerRect);
      _canvasPool.drawRRect(rrect, paint.style);
      tearDownPaint();
    }
  }

  @override
  void drawDRRect(ui.RRect outer, ui.RRect inner, SurfacePaintData paint) {
    setUpPaint(paint, outer.outerRect);
    _canvasPool.drawDRRect(outer, inner, paint.style);
    tearDownPaint();
  }

  @override
  void drawOval(ui.Rect rect, SurfacePaintData paint) {
    if (_useDomForRenderingFill(paint)) {
      rect = adjustRectForDom(rect, paint);
      final DomHTMLElement element = buildDrawRectElement(
        rect,
        paint,
        'draw-oval',
        _canvasPool.currentTransform,
      );
      _drawElement(element, rect.topLeft, paint);
      element.style.borderRadius = '${rect.width / 2.0}px / ${rect.height / 2.0}px';
    } else {
      setUpPaint(paint, rect);
      _canvasPool.drawOval(rect, paint.style);
      tearDownPaint();
    }
  }

  @override
  void drawCircle(ui.Offset c, double radius, SurfacePaintData paint) {
    if (_useDomForRenderingFillAndStroke(paint)) {
      final ui.Rect rect = adjustRectForDom(ui.Rect.fromCircle(center: c, radius: radius), paint);
      final DomHTMLElement element = buildDrawRectElement(
        rect,
        paint,
        'draw-circle',
        _canvasPool.currentTransform,
      );
      _drawElement(element, rect.topLeft, paint);
      element.style.borderRadius = '50%';
    } else {
      setUpPaint(
        paint,
        paint.shader != null ? ui.Rect.fromCircle(center: c, radius: radius) : null,
      );
      _canvasPool.drawCircle(c, radius, paint.style);
      tearDownPaint();
    }
  }

  @override
  void drawPath(ui.Path path, SurfacePaintData paint) {
    if (_useDomForRenderingFill(paint)) {
      final Matrix4 transform = _canvasPool.currentTransform;
      final SurfacePath surfacePath = path as SurfacePath;

      final ui.Rect? pathAsRect = surfacePath.toRect();
      if (pathAsRect != null) {
        drawRect(pathAsRect, paint);
        return;
      }
      final ui.RRect? pathAsRRect = surfacePath.toRoundedRect();
      if (pathAsRRect != null) {
        drawRRect(pathAsRRect, paint);
        return;
      }
      final DomElement svgElm = pathToSvgElement(surfacePath, paint);
      if (!_canvasPool.isClipped) {
        final DomCSSStyleDeclaration style = svgElm.style;
        style.position = 'absolute';
        if (!transform.isIdentity()) {
          style
            ..transform = matrix4ToCssTransform(transform)
            ..transformOrigin = '0 0 0';
        }
      }
      _applyFilter(svgElm, paint);
      _drawElement(svgElm, ui.Offset.zero, paint);
    } else {
      setUpPaint(paint, paint.shader != null ? path.getBounds() : null);
      if (paint.style == null && paint.strokeWidth != null) {
        _canvasPool.drawPath(path, ui.PaintingStyle.stroke);
      } else {
        _canvasPool.drawPath(path, paint.style);
      }
      tearDownPaint();
    }
  }

  void _applyFilter(DomElement element, SurfacePaintData paint) {
    if (paint.maskFilter != null) {
      final bool isStroke = paint.style == ui.PaintingStyle.stroke;
      final String cssColor = colorValueToCssString(paint.color);
      final double sigma = paint.maskFilter!.webOnlySigma;
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit && !isStroke) {
        // A bug in webkit leaves artifacts when this element is animated
        // with filter: blur, we use boxShadow instead.
        element.style.boxShadow = '0px 0px ${sigma * 2.0}px $cssColor';
      } else {
        element.style.filter = 'blur(${sigma}px)';
      }
    }
  }

  @override
  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {
    _canvasPool.drawShadow(path, color, elevation, transparentOccluder);
  }

  @override
  void drawImage(ui.Image image, ui.Offset p, SurfacePaintData paint) {
    final DomHTMLElement imageElement = _drawImage(image, p, paint);
    if (paint.colorFilter != null) {
      _applyTargetSize(imageElement, image.width.toDouble(), image.height.toDouble());
    }
    if (!_preserveImageData) {
      _closeCanvas();
    }
  }

  DomHTMLImageElement _reuseOrCreateImage(HtmlImage htmlImage) {
    final String cacheKey = htmlImage.imgElement.src!;
    if (_elementCache != null) {
      final DomHTMLImageElement? imageElement =
          _elementCache!.reuse(cacheKey) as DomHTMLImageElement?;
      if (imageElement != null) {
        return imageElement;
      }
    }
    // Can't reuse, create new instance.
    final DomHTMLImageElement newImageElement = htmlImage.cloneImageElement();
    if (_elementCache != null) {
      _elementCache!.cache(cacheKey, newImageElement, _onEvictElement);
    }
    return newImageElement;
  }

  static void _onEvictElement(DomHTMLElement element) {
    element.remove();
  }

  DomHTMLElement _drawImage(ui.Image image, ui.Offset p, SurfacePaintData paint) {
    final HtmlImage htmlImage = image as HtmlImage;
    final ui.BlendMode? blendMode = paint.blendMode;
    final EngineHtmlColorFilter? colorFilter = createHtmlColorFilter(paint.colorFilter);
    DomHTMLElement imgElement;
    if (colorFilter is ModeHtmlColorFilter) {
      imgElement = _createImageElementWithBlend(
        image,
        colorFilter.color,
        colorFilter.blendMode,
        paint,
      );
    } else if (colorFilter is MatrixHtmlColorFilter) {
      imgElement = _createImageElementWithSvgColorMatrixFilter(image, colorFilter.matrix, paint);
    } else {
      // No Blending, create an image by cloning original loaded image.
      imgElement = _reuseOrCreateImage(htmlImage);
    }
    imgElement.style.mixBlendMode = blendModeToCssMixBlendMode(blendMode) ?? '';
    if (_preserveImageData && imgElement is DomHTMLImageElement) {
      // If we're preserving image data, we have to actually draw the image
      // element onto the canvas.
      // TODO(jacksongardner): Make this actually work with color filters.
      setUpPaint(paint, null);
      _canvasPool.drawImage(imgElement, p);
      tearDownPaint();
    } else {
      if (_canvasPool.isClipped) {
        // Reset width/height since they may have been previously set.
        imgElement.style
          ..removeProperty('width')
          ..removeProperty('height');
        final List<DomElement> clipElements = _clipContent(
          _canvasPool.clipStack!,
          imgElement,
          p,
          _canvasPool.currentTransform,
        );
        for (final DomElement clipElement in clipElements) {
          rootElement.append(clipElement);
          _children.add(clipElement);
        }
      } else {
        final String cssTransform = float64ListToCssTransform(
          transformWithOffset(_canvasPool.currentTransform, p).storage,
        );
        imgElement.style
          ..transformOrigin = '0 0 0'
          ..transform = cssTransform
          // Reset width/height since they may have been previously set.
          ..removeProperty('width')
          ..removeProperty('height');
        rootElement.append(imgElement);
        _children.add(imgElement);
      }
    }
    return imgElement;
  }

  DomHTMLElement _createImageElementWithBlend(
    HtmlImage image,
    ui.Color color,
    ui.BlendMode blendMode,
    SurfacePaintData paint,
  ) {
    switch (blendMode) {
      case ui.BlendMode.colorBurn:
      case ui.BlendMode.colorDodge:
      case ui.BlendMode.hue:
      case ui.BlendMode.modulate:
      case ui.BlendMode.overlay:
      case ui.BlendMode.plus:
      case ui.BlendMode.srcIn:
      case ui.BlendMode.srcATop:
      case ui.BlendMode.srcOut:
      case ui.BlendMode.saturation:
      case ui.BlendMode.color:
      case ui.BlendMode.luminosity:
      case ui.BlendMode.xor:
      case ui.BlendMode.dstATop:
        return _createImageElementWithSvgBlendFilter(image, color, blendMode, paint);
      default:
        return _createBackgroundImageWithBlend(image, color, blendMode, paint);
    }
  }

  @override
  void drawImageRect(ui.Image image, ui.Rect src, ui.Rect dst, SurfacePaintData paint) {
    final bool requiresClipping =
        src.left != 0 || src.top != 0 || src.width != image.width || src.height != image.height;
    // If source and destination sizes are identical, we can skip the longer
    // code path that sets the size of the element and clips.
    //
    // If there is a color filter set however, we maybe using background-image
    // to render therefore we have to explicitly set width/height of the
    // element for blending to work with background-color.
    if (dst.width == image.width &&
        dst.height == image.height &&
        !requiresClipping &&
        paint.colorFilter == null) {
      _drawImage(image, dst.topLeft, paint);
    } else {
      if (requiresClipping) {
        save();
        clipRect(dst, ui.ClipOp.intersect);
      }
      double targetLeft = dst.left;
      double targetTop = dst.top;
      if (requiresClipping) {
        if (src.width != image.width) {
          final double leftMargin = -src.left * (dst.width / src.width);
          targetLeft += leftMargin;
        }
        if (src.height != image.height) {
          final double topMargin = -src.top * (dst.height / src.height);
          targetTop += topMargin;
        }
      }

      final DomElement imgElement = _drawImage(image, ui.Offset(targetLeft, targetTop), paint);
      // To scale set width / height on destination image.
      // For clipping we need to scale according to
      // clipped-width/full image width and shift it according to left/top of
      // source rectangle.
      double targetWidth = dst.width;
      double targetHeight = dst.height;
      if (requiresClipping) {
        targetWidth *= image.width / src.width;
        targetHeight *= image.height / src.height;
      }
      _applyTargetSize(imgElement as DomHTMLElement, targetWidth, targetHeight);
      if (requiresClipping) {
        restore();
      }
    }
    _closeCanvas();
  }

  void _applyTargetSize(DomHTMLElement imageElement, double targetWidth, double targetHeight) {
    final DomCSSStyleDeclaration imageStyle = imageElement.style;
    final String widthPx = '${targetWidth.toStringAsFixed(2)}px';
    final String heightPx = '${targetHeight.toStringAsFixed(2)}px';
    imageStyle
      // left,top are set to 0 (although position is absolute) because
      // Chrome will glitch if you leave them out, reproducible with
      // canvas_image_blend_test on row 6,  MacOS / Chrome 81.04.
      ..left = '0px'
      ..top = '0px'
      ..width = widthPx
      ..height = heightPx;
    if (!domInstanceOfString(imageElement, 'HTMLImageElement')) {
      imageElement.style.backgroundSize = '$widthPx $heightPx';
    }
  }

  // Creates a Div element to render an image using background-image css
  // attribute to be able to use background blend mode(s) when possible.
  //
  // Example: <div style="
  //               position:absolute;
  //               background-image:url(....);
  //               background-blend-mode:"darken"
  //               background-color: #RRGGBB">
  //
  // Special cases:
  // For clear,dstOut it generates a blank element.
  // For src,srcOver it only sets background-color attribute.
  // For dst,dstIn , it only sets source not background color.
  DomHTMLElement _createBackgroundImageWithBlend(
    HtmlImage image,
    ui.Color? filterColor,
    ui.BlendMode colorFilterBlendMode,
    SurfacePaintData paint,
  ) {
    // When blending with color we can't use an image element.
    // Instead use a div element with background image, color and
    // background blend mode.
    final DomHTMLElement imgElement = createDomHTMLDivElement();
    final DomCSSStyleDeclaration style = imgElement.style;
    switch (colorFilterBlendMode) {
      case ui.BlendMode.clear:
      case ui.BlendMode.dstOut:
        style.position = 'absolute';
      case ui.BlendMode.src:
      case ui.BlendMode.srcOver:
        style
          ..position = 'absolute'
          ..backgroundColor = filterColor!.toCssString();
      case ui.BlendMode.dst:
      case ui.BlendMode.dstIn:
        style
          ..position = 'absolute'
          ..backgroundImage = "url('${image.imgElement.src}')";
      default:
        style
          ..position = 'absolute'
          ..backgroundImage = "url('${image.imgElement.src}')"
          ..backgroundBlendMode = blendModeToCssMixBlendMode(colorFilterBlendMode) ?? ''
          ..backgroundColor = filterColor!.toCssString();
    }
    return imgElement;
  }

  // Creates an image element and an svg filter to apply on the element.
  DomHTMLElement _createImageElementWithSvgBlendFilter(
    HtmlImage image,
    ui.Color? filterColor,
    ui.BlendMode colorFilterBlendMode,
    SurfacePaintData paint,
  ) {
    // For srcIn blendMode, we use an svg filter to apply to image element.
    final SvgFilter svgFilter = svgFilterFromBlendMode(filterColor, colorFilterBlendMode);
    rootElement.append(svgFilter.element);
    _children.add(svgFilter.element);
    final DomHTMLElement imgElement = _reuseOrCreateImage(image);
    imgElement.style.filter = 'url(#${svgFilter.id})';
    if (colorFilterBlendMode == ui.BlendMode.saturation) {
      imgElement.style.backgroundColor = filterColor!.toCssString();
    }
    return imgElement;
  }

  // Creates an image element and an svg color matrix filter to apply on the element.
  DomHTMLElement _createImageElementWithSvgColorMatrixFilter(
    HtmlImage image,
    List<double> matrix,
    SurfacePaintData paint,
  ) {
    // For srcIn blendMode, we use an svg filter to apply to image element.
    final SvgFilter svgFilter = svgFilterFromColorMatrix(matrix);
    rootElement.append(svgFilter.element);
    _children.add(svgFilter.element);
    final DomHTMLElement imgElement = _reuseOrCreateImage(image);
    imgElement.style.filter = 'url(#${svgFilter.id})';
    return imgElement;
  }

  // Should be called when we add new html elements into rootElement so that
  // paint order is preserved.
  //
  // For example if we draw a path and then a paragraph and image:
  //   - rootElement
  //   |--- <canvas>
  //   |--- <p>
  //   |--- <img>
  // Any drawing operations after these tags should allocate a new canvas,
  // instead of drawing into earlier canvas.
  void _closeCanvas() {
    _canvasPool.closeCanvas();
    _childOverdraw = true;
    _cachedLastCssFont = null;
  }

  void setCssFont(String cssFont, ui.TextDirection textDirection) {
    final DomCanvasRenderingContext2D ctx = _canvasPool.context;
    ctx.direction = textDirection == ui.TextDirection.ltr ? 'ltr' : 'rtl';

    if (cssFont != _cachedLastCssFont) {
      ctx.font = cssFont;
      _cachedLastCssFont = cssFont;
    }
  }

  /// Measures the given [text] and returns a [DomTextMetrics] object that
  /// contains information about the measurement.
  ///
  /// The text is measured using the font set by the most recent call to
  /// [setCssFont].
  DomTextMetrics measureText(String text) {
    return _canvasPool.context.measureText(text);
  }

  /// Draws text to the canvas starting at coordinate ([x], [y]).
  ///
  /// The text is drawn starting at coordinates ([x], [y]). It uses the current
  /// font set by the most recent call to [setCssFont].
  void drawText(
    String text,
    double x,
    double y, {
    ui.PaintingStyle? style,
    List<ui.Shadow>? shadows,
  }) {
    final DomCanvasRenderingContext2D ctx = _canvasPool.context;
    if (shadows != null) {
      ctx.save();
      for (final ui.Shadow shadow in shadows) {
        ctx.shadowColor = shadow.color.toCssString();
        ctx.shadowBlur = shadow.blurRadius;
        ctx.shadowOffsetX = shadow.offset.dx;
        ctx.shadowOffsetY = shadow.offset.dy;

        if (style == ui.PaintingStyle.stroke) {
          ctx.strokeText(text, x, y);
        } else {
          ctx.fillText(text, x, y);
        }
      }
      ctx.restore();
    }

    if (style == ui.PaintingStyle.stroke) {
      ctx.strokeText(text, x, y);
    } else {
      ctx.fillText(text, x, y);
    }
  }

  @override
  void drawParagraph(CanvasParagraph paragraph, ui.Offset offset) {
    assert(paragraph.isLaidOut);

    // Normally, text is composited as a plain HTML <p> tag. However, if a
    // bitmap canvas was used for a preceding drawing command, then it's more
    // efficient to continue compositing into the existing canvas, if possible.
    // Whether it's possible to composite a paragraph into a 2D canvas depends
    // on the following:
    final bool canCompositeIntoBitmapCanvas =
        // Cannot composite if the paragraph cannot be drawn into bitmap canvas
        // in the first place.
        paragraph.canDrawOnCanvas &&
        // Cannot composite if there's no bitmap canvas to composite into.
            // Creating a new bitmap canvas just to draw text doesn't make sense.
            _canvasPool
            .hasCanvas &&
        !_childOverdraw &&
        // Bitmap canvas introduces correctness issues in the presence of SVG
        // filters, so prefer plain HTML in this case.
        !_renderStrategy.isInsideSvgFilterTree;

    if (canCompositeIntoBitmapCanvas) {
      paragraph.paint(this, offset);
      return;
    }

    final DomElement paragraphElement = drawParagraphElement(paragraph, offset);
    if (_canvasPool.isClipped) {
      final List<DomElement> clipElements = _clipContent(
        _canvasPool.clipStack!,
        paragraphElement,
        offset,
        _canvasPool.currentTransform,
      );
      for (final DomElement clipElement in clipElements) {
        rootElement.append(clipElement);
        _children.add(clipElement);
      }
    } else {
      setElementTransform(
        paragraphElement,
        transformWithOffset(_canvasPool.currentTransform, offset).storage,
      );
      rootElement.append(paragraphElement);
    }
    _children.add(paragraphElement);
    // If there is a prior sibling such as img prevent left/top shift.
    paragraphElement.style
      ..left = '0px'
      ..top = '0px';
    _closeCanvas();
  }

  /// Draws vertices on a gl context.
  ///
  /// If both colors and textures is specified in paint data,
  /// for [BlendMode.source] we skip colors and use textures,
  /// for [BlendMode.dst] we only use colors and ignore textures.
  /// We also skip paint shader when no texture is specified.
  ///
  /// If no colors or textures are specified, stroke hairlines with
  /// [Paint.color].
  ///
  /// If colors is specified, convert colors to premultiplied (alpha) colors
  /// and use a SkTriColorShader to render.
  @override
  void drawVertices(SurfaceVertices vertices, ui.BlendMode blendMode, SurfacePaintData paint) {
    // TODO(ferhat): Implement shaders for [Paint.shader] and
    // blendMode. https://github.com/flutter/flutter/issues/40096
    // Move rendering to OffscreenCanvas so that transform is preserved
    // as well.
    assert(
      paint.shader == null || paint.shader is EngineImageShader,
      'Linear/Radial/SweepGradient not supported yet',
    );
    final Int32List? colors = vertices.colors;
    final ui.VertexMode mode = vertices.mode;
    final DomCanvasRenderingContext2D ctx = _canvasPool.context;
    if (colors == null && paint.style != ui.PaintingStyle.fill && paint.shader == null) {
      final Float32List positions =
          mode == ui.VertexMode.triangles
              ? vertices.positions
              : convertVertexPositions(mode, vertices.positions);
      // Draw hairline for vertices if no vertex colors are specified.
      save();
      final ui.Color color = ui.Color(paint.color);
      _canvasPool.contextHandle
        ..fillStyle = null
        ..strokeStyle = color.toCssString();
      glRenderer!.drawHairline(ctx, positions);
      restore();
      return;
    }
    glRenderer!.drawVertices(
      ctx,
      widthInBitmapPixels,
      heightInBitmapPixels,
      _canvasPool.currentTransform,
      vertices,
      blendMode,
      paint,
    );
  }

  /// Stores paint data used by [drawPoints]. We cannot use the original paint
  /// data object because painting style is determined by [ui.PointMode] and
  /// not by [SurfacePointData.style].
  static final SurfacePaintData _drawPointsPaint =
      SurfacePaintData()
        ..strokeCap = ui.StrokeCap.round
        ..strokeJoin = ui.StrokeJoin.round
        ..blendMode = ui.BlendMode.srcOver;

  @override
  void drawPoints(ui.PointMode pointMode, Float32List points, SurfacePaintData paint) {
    if (pointMode == ui.PointMode.points) {
      _drawPointsPaint.style = ui.PaintingStyle.stroke;
    } else {
      _drawPointsPaint.style = ui.PaintingStyle.fill;
    }
    _drawPointsPaint.color = paint.color;
    _drawPointsPaint.maskFilter = paint.maskFilter;

    final double dpr = EngineFlutterDisplay.instance.devicePixelRatio;
    // Use hairline (device pixel when strokeWidth is not specified).
    final double strokeWidth = paint.strokeWidth == null ? 1.0 / dpr : paint.strokeWidth!;
    _drawPointsPaint.strokeWidth = strokeWidth;
    setUpPaint(_drawPointsPaint, null);
    // Draw point using circle with half radius.
    _canvasPool.drawPoints(pointMode, points, strokeWidth / 2.0);
    tearDownPaint();
  }

  @override
  void endOfPaint() {
    _canvasPool.endOfPaint();
    _elementCache?.commitFrame();
    if (_contains3dTransform && ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
      // Copy the children list to avoid concurrent modification.
      final List<DomElement> children = rootElement.children.toList();
      for (final DomElement element in children) {
        final DomHTMLDivElement paintOrderElement =
            createDomHTMLDivElement()..style.transform = 'translate3d(0,0,0)';
        paintOrderElement.append(element);
        rootElement.append(paintOrderElement);
        _children.add(paintOrderElement);
      }
    }
    final DomNode? firstChild = rootElement.firstChild;
    if (firstChild != null) {
      if (domInstanceOfString(firstChild, 'HTMLElement')) {
        final DomHTMLElement maybeCanvas = firstChild as DomHTMLElement;
        if (maybeCanvas.tagName.toLowerCase() == 'canvas') {
          maybeCanvas.style.zIndex = '-1';
        }
      }
    }
  }

  /// Computes paint bounds given [targetTransform] to completely cover window
  /// viewport.
  ui.Rect _computeScreenBounds(Matrix4 targetTransform) {
    final Matrix4 inverted = targetTransform.clone()..invert();
    final double dpr = EngineFlutterDisplay.instance.devicePixelRatio;
    final double width = ui.window.physicalSize.width * dpr;
    final double height = ui.window.physicalSize.height * dpr;
    final Vector3 topLeft = inverted.perspectiveTransform(x: 0, y: 0, z: 0);
    final Vector3 topRight = inverted.perspectiveTransform(x: width, y: 0, z: 0);
    final Vector3 bottomRight = inverted.perspectiveTransform(x: width, y: height, z: 0);
    final Vector3 bottomLeft = inverted.perspectiveTransform(x: 0, y: height, z: 0);
    return ui.Rect.fromLTRB(
      math.min(topLeft.x, math.min(topRight.x, math.min(bottomRight.x, bottomLeft.x))),
      math.min(topLeft.y, math.min(topRight.y, math.min(bottomRight.y, bottomLeft.y))),
      math.max(topLeft.x, math.max(topRight.x, math.max(bottomRight.x, bottomLeft.x))),
      math.max(topLeft.y, math.max(topRight.y, math.max(bottomRight.y, bottomLeft.y))),
    );
  }

  /// Computes paint bounds to completely cover picture.
  ui.Rect _computePictureBounds() {
    return ui.Rect.fromLTRB(0, 0, _bounds.width, _bounds.height);
  }
}

/// The CSS value for the `mix-blend-mode` CSS property.
///
/// This list includes values supposrted by SVG, but it's not the same.
///
/// See also:
///
///  * https://developer.mozilla.org/en-US/docs/Web/CSS/mix-blend-mode
///  * [blendModeToSvgEnum], which specializes on SVG blend modes
String? blendModeToCssMixBlendMode(ui.BlendMode? blendMode) {
  if (blendMode == null) {
    return null;
  }
  switch (blendMode) {
    case ui.BlendMode.srcOver:
      return 'source-over';
    case ui.BlendMode.srcIn:
      return 'source-in';
    case ui.BlendMode.srcOut:
      return 'source-out';
    case ui.BlendMode.srcATop:
      return 'source-atop';
    case ui.BlendMode.dstOver:
      return 'destination-over';
    case ui.BlendMode.dstIn:
      return 'destination-in';
    case ui.BlendMode.dstOut:
      return 'destination-out';
    case ui.BlendMode.dstATop:
      return 'destination-atop';
    case ui.BlendMode.plus:
      return 'lighten';
    case ui.BlendMode.src:
      return 'copy';
    case ui.BlendMode.xor:
      return 'xor';
    case ui.BlendMode.multiply:
    // Falling back to multiply, ignoring alpha channel.
    // TODO(ferhat): only used for debug, find better fallback for web.
    case ui.BlendMode.modulate:
      return 'multiply';
    case ui.BlendMode.screen:
      return 'screen';
    case ui.BlendMode.overlay:
      return 'overlay';
    case ui.BlendMode.darken:
      return 'darken';
    case ui.BlendMode.lighten:
      return 'lighten';
    case ui.BlendMode.colorDodge:
      return 'color-dodge';
    case ui.BlendMode.colorBurn:
      return 'color-burn';
    case ui.BlendMode.hardLight:
      return 'hard-light';
    case ui.BlendMode.softLight:
      return 'soft-light';
    case ui.BlendMode.difference:
      return 'difference';
    case ui.BlendMode.exclusion:
      return 'exclusion';
    case ui.BlendMode.hue:
      return 'hue';
    case ui.BlendMode.saturation:
      return 'saturation';
    case ui.BlendMode.color:
      return 'color';
    case ui.BlendMode.luminosity:
      return 'luminosity';
    default:
      throw UnimplementedError('Flutter Web does not support the blend mode: $blendMode');
  }
}

// Source: https://www.w3.org/TR/SVG11/filters.html#InterfaceSVGFEBlendElement
// These constant names deviate from Dart's camelCase convention on purpose to
// make it easier to search for them in W3 specs and in Chromium sources.
const int SVG_FEBLEND_MODE_UNKNOWN = 0;
const int SVG_FEBLEND_MODE_NORMAL = 1;
const int SVG_FEBLEND_MODE_MULTIPLY = 2;
const int SVG_FEBLEND_MODE_SCREEN = 3;
const int SVG_FEBLEND_MODE_DARKEN = 4;
const int SVG_FEBLEND_MODE_LIGHTEN = 5;
const int SVG_FEBLEND_MODE_OVERLAY = 6;
const int SVG_FEBLEND_MODE_COLOR_DODGE = 7;
const int SVG_FEBLEND_MODE_COLOR_BURN = 8;
const int SVG_FEBLEND_MODE_HARD_LIGHT = 9;
const int SVG_FEBLEND_MODE_SOFT_LIGHT = 10;
const int SVG_FEBLEND_MODE_DIFFERENCE = 11;
const int SVG_FEBLEND_MODE_EXCLUSION = 12;
const int SVG_FEBLEND_MODE_HUE = 13;
const int SVG_FEBLEND_MODE_SATURATION = 14;
const int SVG_FEBLEND_MODE_COLOR = 15;
const int SVG_FEBLEND_MODE_LUMINOSITY = 16;

// Source: https://github.com/chromium/chromium/blob/e1e495b29e1178a451f65980a6c4ae017c34dc94/third_party/blink/renderer/platform/graphics/graphics_types.cc#L55
const String kCompositeClear = 'clear';
const String kCompositeCopy = 'copy';
const String kCompositeSourceOver = 'source-over';
const String kCompositeSourceIn = 'source-in';
const String kCompositeSourceOut = 'source-out';
const String kCompositeSourceAtop = 'source-atop';
const String kCompositeDestinationOver = 'destination-over';
const String kCompositeDestinationIn = 'destination-in';
const String kCompositeDestinationOut = 'destination-out';
const String kCompositeDestinationAtop = 'destination-atop';
const String kCompositeXor = 'xor';
const String kCompositeLighter = 'lighter';

/// Compositing and blending operation in SVG.
///
/// Flutter's [BlendMode] flattens what SVG expresses as two orthogonal
/// properties, a composite operator and blend mode. Instances of this class
/// are returned from [blendModeToSvgEnum] by mapping Flutter's [BlendMode]
/// enum onto the SVG equivalent.
///
/// See also:
///
///  * https://www.w3.org/TR/compositing-1
///  * https://github.com/chromium/chromium/blob/e1e495b29e1178a451f65980a6c4ae017c34dc94/third_party/blink/renderer/platform/graphics/graphics_types.cc#L55
///  * https://github.com/chromium/chromium/blob/e1e495b29e1178a451f65980a6c4ae017c34dc94/third_party/blink/renderer/modules/canvas/canvas2d/base_rendering_context_2d.cc#L725
class SvgBlendMode {
  const SvgBlendMode(this.compositeOperator, this.blendMode);

  /// The name of the SVG composite operator.
  ///
  /// If this mode represents a blend mode, this is set to [kCompositeSourceOver].
  final String compositeOperator;

  /// The identifier of the SVG blend mode.
  ///
  /// This is mode represents a compositing operation, this is set to [SVG_FEBLEND_MODE_UNKNOWN].
  final int blendMode;
}

/// Converts Flutter's [ui.BlendMode] to SVG's <compositing operation, blend mode> pair.
SvgBlendMode? blendModeToSvgEnum(ui.BlendMode? blendMode) {
  if (blendMode == null) {
    return null;
  }
  switch (blendMode) {
    case ui.BlendMode.clear:
      return const SvgBlendMode(kCompositeClear, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.srcOver:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.srcIn:
      return const SvgBlendMode(kCompositeSourceIn, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.srcOut:
      return const SvgBlendMode(kCompositeSourceOut, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.srcATop:
      return const SvgBlendMode(kCompositeSourceAtop, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.dstOver:
      return const SvgBlendMode(kCompositeDestinationOver, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.dstIn:
      return const SvgBlendMode(kCompositeDestinationIn, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.dstOut:
      return const SvgBlendMode(kCompositeDestinationOut, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.dstATop:
      return const SvgBlendMode(kCompositeDestinationAtop, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.plus:
      return const SvgBlendMode(kCompositeLighter, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.src:
      return const SvgBlendMode(kCompositeCopy, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.xor:
      return const SvgBlendMode(kCompositeXor, SVG_FEBLEND_MODE_UNKNOWN);
    case ui.BlendMode.multiply:
    // Falling back to multiply, ignoring alpha channel.
    // TODO(ferhat): only used for debug, find better fallback for web.
    case ui.BlendMode.modulate:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_MULTIPLY);
    case ui.BlendMode.screen:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_SCREEN);
    case ui.BlendMode.overlay:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_OVERLAY);
    case ui.BlendMode.darken:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_DARKEN);
    case ui.BlendMode.lighten:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_LIGHTEN);
    case ui.BlendMode.colorDodge:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_COLOR_DODGE);
    case ui.BlendMode.colorBurn:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_COLOR_BURN);
    case ui.BlendMode.hardLight:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_HARD_LIGHT);
    case ui.BlendMode.softLight:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_SOFT_LIGHT);
    case ui.BlendMode.difference:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_DIFFERENCE);
    case ui.BlendMode.exclusion:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_EXCLUSION);
    case ui.BlendMode.hue:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_HUE);
    case ui.BlendMode.saturation:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_SATURATION);
    case ui.BlendMode.color:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_COLOR);
    case ui.BlendMode.luminosity:
      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_LUMINOSITY);
    default:
      assert(false, 'Flutter Web does not support the blend mode: $blendMode');

      return const SvgBlendMode(kCompositeSourceOver, SVG_FEBLEND_MODE_NORMAL);
  }
}

String? stringForStrokeCap(ui.StrokeCap? strokeCap) {
  if (strokeCap == null) {
    return null;
  }
  switch (strokeCap) {
    case ui.StrokeCap.butt:
      return 'butt';
    case ui.StrokeCap.round:
      return 'round';
    case ui.StrokeCap.square:
      return 'square';
  }
}

String stringForStrokeJoin(ui.StrokeJoin strokeJoin) {
  switch (strokeJoin) {
    case ui.StrokeJoin.round:
      return 'round';
    case ui.StrokeJoin.bevel:
      return 'bevel';
    case ui.StrokeJoin.miter:
      return 'miter';
  }
}

/// Clips the content element against a stack of clip operations and returns
/// root of a tree that contains content node.
///
/// The stack of clipping rectangles generate an element that either uses
/// overflow:hidden with bounds to clip child or sets a clip-path to clip
/// it's contents. The clipping rectangles are nested and returned together
/// with a list of svg elements that provide clip-paths.
List<DomElement> _clipContent(
  List<SaveClipEntry> clipStack,
  DomElement content,
  ui.Offset offset,
  Matrix4 currentTransform,
) {
  DomElement? root, curElement;
  final List<DomElement> clipDefs = <DomElement>[];
  final int len = clipStack.length;
  for (int clipIndex = 0; clipIndex < len; clipIndex++) {
    final SaveClipEntry entry = clipStack[clipIndex];
    final DomHTMLElement newElement = createDomHTMLDivElement();
    newElement.style.position = 'absolute';
    applyWebkitClipFix(newElement);
    if (root == null) {
      root = newElement;
    } else {
      curElement!.append(newElement);
    }
    curElement = newElement;
    final ui.Rect? rect = entry.rect;
    Matrix4 newClipTransform = entry.currentTransform;
    final TransformKind transformKind = transformKindOf(newClipTransform.storage);
    final bool requiresTransformStyle = transformKind == TransformKind.complex;
    if (rect != null) {
      final double clipOffsetX = rect.left;
      final double clipOffsetY = rect.top;
      newClipTransform = newClipTransform.clone()..translate(clipOffsetX, clipOffsetY);
      curElement.style
        ..overflow = 'hidden'
        ..width = '${rect.right - clipOffsetX}px'
        ..height = '${rect.bottom - clipOffsetY}px';
      setElementTransform(curElement, newClipTransform.storage);
    } else if (entry.rrect != null) {
      final ui.RRect roundRect = entry.rrect!;
      final String borderRadius =
          '${roundRect.tlRadiusX}px ${roundRect.trRadiusX}px '
          '${roundRect.brRadiusX}px ${roundRect.blRadiusX}px';
      final double clipOffsetX = roundRect.left;
      final double clipOffsetY = roundRect.top;
      newClipTransform = newClipTransform.clone()..translate(clipOffsetX, clipOffsetY);
      curElement.style
        ..borderRadius = borderRadius
        ..overflow = 'hidden'
        ..width = '${roundRect.right - clipOffsetX}px'
        ..height = '${roundRect.bottom - clipOffsetY}px';
      setElementTransform(curElement, newClipTransform.storage);
    } else if (entry.path != null) {
      // Clipping optimization when we know that the path is an oval.
      // We use a div with border-radius set to 50% with a size that is
      // set to path bounds and set overflow to hidden.
      final SurfacePath surfacePath = entry.path! as SurfacePath;
      if (surfacePath.pathRef.isOval != -1) {
        final ui.Rect ovalBounds = surfacePath.getBounds();
        final double clipOffsetX = ovalBounds.left;
        final double clipOffsetY = ovalBounds.top;
        newClipTransform = newClipTransform.clone()..translate(clipOffsetX, clipOffsetY);
        curElement.style
          ..overflow = 'hidden'
          ..width = '${ovalBounds.width}px'
          ..height = '${ovalBounds.height}px'
          ..borderRadius = '50%';
        setElementTransform(curElement, newClipTransform.storage);
      } else {
        // Abitrary path clipping.
        curElement.style
          ..transform = matrix4ToCssTransform(newClipTransform)
          ..transformOrigin = '0 0 0';
        final DomElement clipElement = createSvgClipDef(curElement, entry.path!);
        clipDefs.add(clipElement);
      }
    }
    // Reverse the transform of the clipping element so children can use
    // effective transform to render.
    // TODO(ferhat): When we have more than a single clip element,
    // reduce number of div nodes by merging (multiplying transforms).
    final DomElement reverseTransformDiv = createDomHTMLDivElement();
    reverseTransformDiv.style.position = 'absolute';
    setElementTransform(reverseTransformDiv, (newClipTransform.clone()..invert()).storage);
    if (requiresTransformStyle) {
      // Instead of flattening matrix3d, preserve so it can be reversed.
      curElement.style.transformStyle = 'preserve-3d';
      reverseTransformDiv.style.transformStyle = 'preserve-3d';
    }
    curElement.append(reverseTransformDiv);
    curElement = reverseTransformDiv;
  }

  root!.style.position = 'absolute';
  curElement!.append(content);
  setElementTransform(content, transformWithOffset(currentTransform, offset).storage);
  return <DomElement>[root, ...clipDefs];
}

/// Converts a [maskFilter] to the value to be used on a `<canvas>`.
///
/// Only supported in non-WebKit browsers.
String maskFilterToCanvasFilter(ui.MaskFilter? maskFilter) {
  assert(
    ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit,
    'WebKit (Safari) does not support `filter` canvas property.',
  );
  if (maskFilter != null) {
    // Multiply by device-pixel ratio because the canvas' pixel width and height
    // are larger than its CSS width and height by device-pixel ratio.
    return 'blur(${maskFilter.webOnlySigma * EngineFlutterDisplay.instance.devicePixelRatio}px)';
  } else {
    return 'none';
  }
}
