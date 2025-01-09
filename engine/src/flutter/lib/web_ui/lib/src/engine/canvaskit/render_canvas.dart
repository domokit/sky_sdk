// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import '../display.dart';
import '../dom.dart';
import '../util.dart';
import 'rasterizer.dart';

/// A visible (on-screen) canvas that can display bitmaps produced by CanvasKit
/// in the (off-screen) SkSurface which is backed by an OffscreenCanvas.
///
/// In a typical frame, the content will be rendered via CanvasKit in an
/// OffscreenCanvas, and then the contents will be transferred to the
/// RenderCanvas via `transferFromImageBitmap()`.
///
/// If we need more RenderCanvases, for example in the case where there are
/// platform views and we need overlays to render the frame correctly, then
/// we will create multiple RenderCanvases, but crucially still only have
/// one OffscreenCanvas which transfers bitmaps to all of the RenderCanvases.
///
/// To render into the OffscreenCanvas with CanvasKit we need to create a
/// WebGL context, which is not only expensive, but the browser has a limit
/// on the maximum amount of WebGL contexts which can be live at once. Using
/// a single OffscreenCanvas and multiple RenderCanvases allows us to only
/// create a single WebGL context.
class RenderCanvas extends DisplayCanvas {
  RenderCanvas() {
    canvasElement.setAttribute('aria-hidden', 'true');
    canvasElement.style.position = 'absolute';
    _updateLogicalHtmlCanvasSize();
    hostElement.append(canvasElement);
  }

  /// The root HTML element for this canvas.
  ///
  /// This element contains the canvas used to draw the UI. Unlike the canvas,
  /// this element is permanent. It is never replaced or deleted, until this
  /// canvas is disposed of via [dispose].
  ///
  /// Conversely, the canvas that lives inside this element can be swapped, for
  /// example, when the screen size changes, or when the WebGL context is lost
  /// due to the browser tab becoming dormant.
  @override
  final DomElement hostElement = createDomElement('flt-canvas-container');

  /// The underlying `<canvas>` element used to display the pixels.
  final DomCanvasElement canvasElement = createDomCanvasElement();
  int _pixelWidth = 0;
  int _pixelHeight = 0;

  late final DomCanvasRenderingContextBitmapRenderer renderContext =
      canvasElement.contextBitmapRenderer;

  late final DomCanvasRenderingContext2D renderContext2d = canvasElement.context2D;

  double _currentDevicePixelRatio = -1;

  /// Sets the CSS size of the canvas so that canvas pixels are 1:1 with device
  /// pixels.
  void _updateLogicalHtmlCanvasSize() {
    final double devicePixelRatio = EngineFlutterDisplay.instance.devicePixelRatio;
    final double logicalWidth = _pixelWidth / devicePixelRatio;
    final double logicalHeight = _pixelHeight / devicePixelRatio;
    final DomCSSStyleDeclaration style = canvasElement.style;
    style.width = '${logicalWidth}px';
    style.height = '${logicalHeight}px';
    _currentDevicePixelRatio = devicePixelRatio;
  }

  /// Render the given [bitmap] with this [RenderCanvas].
  ///
  /// The canvas will be resized to accomodate the bitmap immediately before
  /// rendering it.
  void render(DomImageBitmap bitmap) {
    _ensureSize(BitmapSize(bitmap.width.toDartInt, bitmap.height.toDartInt));
    renderContext.transferFromImageBitmap(bitmap);
  }

  void renderWithNoBitmapSupport(
    DomCanvasImageSource imageSource,
    int sourceHeight,
    BitmapSize size,
  ) {
    _ensureSize(size);
    renderContext2d.drawImage(
      imageSource,
      0,
      sourceHeight - size.height,
      size.width,
      size.height,
      0,
      0,
      size.width,
      size.height,
    );
  }

  /// Ensures that this canvas can draw a frame of the given [size].
  void _ensureSize(BitmapSize size) {
    // Check if the frame is the same size as before, and if so, we don't need
    // to resize the canvas.
    if (size.width == _pixelWidth && size.height == _pixelHeight) {
      // The existing canvas doesn't need to be resized (unless the device pixel
      // ratio changed).
      if (EngineFlutterDisplay.instance.devicePixelRatio != _currentDevicePixelRatio) {
        _updateLogicalHtmlCanvasSize();
      }
      return;
    }

    // If the canvas is too large or too small, resize it to the exact size of
    // the frame. We cannot allow the canvas to be larger than the screen
    // because then when we call `transferFromImageBitmap()` the bitmap will
    // be scaled to cover the entire canvas.
    _pixelWidth = size.width;
    _pixelHeight = size.height;
    canvasElement.width = _pixelWidth.toDouble();
    canvasElement.height = _pixelHeight.toDouble();
    _updateLogicalHtmlCanvasSize();
  }

  @override
  bool get isConnected => canvasElement.isConnected!;

  @override
  void initialize() {
    // No extra initialization needed.
  }

  @override
  void dispose() {
    hostElement.remove();
  }
}
