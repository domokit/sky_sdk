// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawCanvas extends Opaque {}

typedef CanvasHandle = Pointer<RawCanvas>;

@Native<Void Function(CanvasHandle)>(symbol: 'canvas_save', isLeaf: true)
external void canvasSave(CanvasHandle canvas);

@Native<Void Function(CanvasHandle, RawRect, PaintHandle, ImageFilterHandle, Int)>(
  symbol: 'canvas_saveLayer',
  isLeaf: true,
)
external void canvasSaveLayer(
  CanvasHandle canvas,
  RawRect rect,
  PaintHandle paint,
  ImageFilterHandle handle,
  int backdropTileMode,
);

@Native<Void Function(CanvasHandle)>(symbol: 'canvas_restore', isLeaf: true)
external void canvasRestore(CanvasHandle canvas);

@Native<Void Function(CanvasHandle, Int)>(symbol: 'canvas_restoreToCount', isLeaf: true)
external void canvasRestoreToCount(CanvasHandle canvas, int count);

@Native<Int Function(CanvasHandle)>(symbol: 'canvas_getSaveCount', isLeaf: true)
external int canvasGetSaveCount(CanvasHandle canvas);

@Native<Void Function(CanvasHandle, Float, Float)>(symbol: 'canvas_translate', isLeaf: true)
external void canvasTranslate(CanvasHandle canvas, double dx, double dy);

@Native<Void Function(CanvasHandle, Float, Float)>(symbol: 'canvas_scale', isLeaf: true)
external void canvasScale(CanvasHandle canvas, double sx, double sy);

@Native<Void Function(CanvasHandle, Float)>(symbol: 'canvas_rotate', isLeaf: true)
external void canvasRotate(CanvasHandle canvas, double degrees);

@Native<Void Function(CanvasHandle, Float, Float)>(symbol: 'canvas_skew', isLeaf: true)
external void canvasSkew(CanvasHandle canvas, double sx, double sy);

@Native<Void Function(CanvasHandle, RawMatrix44)>(symbol: 'canvas_transform', isLeaf: true)
external void canvasTransform(CanvasHandle canvas, RawMatrix44 matrix);

@Native<Void Function(CanvasHandle, RawRect, Int, Bool)>(symbol: 'canvas_clipRect', isLeaf: true)
external void canvasClipRect(CanvasHandle canvas, RawRect rect, int op, bool antialias);

@Native<Void Function(CanvasHandle, RawRRect, Bool)>(symbol: 'canvas_clipRRect', isLeaf: true)
external void canvasClipRRect(CanvasHandle canvas, RawRRect rrect, bool antialias);

@Native<Void Function(CanvasHandle, PathHandle, Bool)>(symbol: 'canvas_clipPath', isLeaf: true)
external void canvasClipPath(CanvasHandle canvas, PathHandle path, bool antialias);

@Native<Void Function(CanvasHandle, Int32, Int)>(symbol: 'canvas_drawColor', isLeaf: true)
external void canvasDrawColor(CanvasHandle canvas, int color, int blendMode);

@Native<Void Function(CanvasHandle, Float, Float, Float, Float, PaintHandle)>(
  symbol: 'canvas_drawLine',
  isLeaf: true,
)
external void canvasDrawLine(
  CanvasHandle canvas,
  double x1,
  double y1,
  double x2,
  double y2,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, PaintHandle)>(symbol: 'canvas_drawPaint', isLeaf: true)
external void canvasDrawPaint(CanvasHandle canvas, PaintHandle paint);

@Native<Void Function(CanvasHandle, RawRect, PaintHandle)>(symbol: 'canvas_drawRect', isLeaf: true)
external void canvasDrawRect(CanvasHandle canvas, RawRect rect, PaintHandle paint);

@Native<Void Function(CanvasHandle, RawRRect, PaintHandle)>(
  symbol: 'canvas_drawRRect',
  isLeaf: true,
)
external void canvasDrawRRect(CanvasHandle canvas, RawRRect rrect, PaintHandle paint);

@Native<Void Function(CanvasHandle, RawRRect, RawRRect, PaintHandle)>(
  symbol: 'canvas_drawDRRect',
  isLeaf: true,
)
external void canvasDrawDRRect(
  CanvasHandle canvas,
  RawRRect outer,
  RawRRect inner,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, RawRect, PaintHandle)>(symbol: 'canvas_drawOval', isLeaf: true)
external void canvasDrawOval(CanvasHandle canvas, RawRect oval, PaintHandle paint);

@Native<Void Function(CanvasHandle, Float, Float, Float, PaintHandle)>(
  symbol: 'canvas_drawCircle',
  isLeaf: true,
)
external void canvasDrawCircle(
  CanvasHandle canvas,
  double x,
  double y,
  double radius,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, RawRect, Float, Float, Bool, PaintHandle)>(
  symbol: 'canvas_drawArc',
  isLeaf: true,
)
external void canvasDrawArc(
  CanvasHandle canvas,
  RawRect rect,
  double startAngleDegrees,
  double sweepAngleDegrees,
  bool useCenter,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, PathHandle, PaintHandle)>(
  symbol: 'canvas_drawPath',
  isLeaf: true,
)
external void canvasDrawPath(CanvasHandle canvas, PathHandle path, PaintHandle paint);

@Native<Void Function(CanvasHandle, PictureHandle)>(symbol: 'canvas_drawPicture', isLeaf: true)
external void canvasDrawPicture(CanvasHandle canvas, PictureHandle picture);

@Native<Void Function(CanvasHandle, ImageHandle, Float, Float, PaintHandle, Int)>(
  symbol: 'canvas_drawImage',
  isLeaf: true,
)
external void canvasDrawImage(
  CanvasHandle handle,
  ImageHandle image,
  double offsetX,
  double offsetY,
  PaintHandle paint,
  int filterQuality,
);

@Native<Void Function(CanvasHandle, ImageHandle, Pointer<Float>, Pointer<Float>, PaintHandle, Int)>(
  symbol: 'canvas_drawImageRect',
  isLeaf: true,
)
external void canvasDrawImageRect(
  CanvasHandle handle,
  ImageHandle image,
  Pointer<Float> sourceRect,
  Pointer<Float> destRect,
  PaintHandle paint,
  int filterQuality,
);

@Native<Void Function(CanvasHandle, ImageHandle, Pointer<Int32>, Pointer<Float>, PaintHandle, Int)>(
  symbol: 'canvas_drawImageNine',
  isLeaf: true,
)
external void canvasDrawImageNine(
  CanvasHandle handle,
  ImageHandle image,
  Pointer<Int32> centerRect,
  Pointer<Float> destRect,
  PaintHandle paint,
  int filterQuality,
);

@Native<Void Function(CanvasHandle, PathHandle, Float, Float, Int32, Bool)>(
  symbol: 'canvas_drawShadow',
  isLeaf: true,
)
external void canvasDrawShadow(
  CanvasHandle canvas,
  PathHandle path,
  double elevation,
  double devicePixelRatio,
  int color,
  bool transparentOccluder,
);

@Native<Void Function(CanvasHandle, ParagraphHandle, Float, Float)>(
  symbol: 'canvas_drawParagraph',
  isLeaf: true,
)
external void canvasDrawParagraph(
  CanvasHandle handle,
  ParagraphHandle paragraphHandle,
  double x,
  double y,
);

@Native<Void Function(CanvasHandle, VerticesHandle, Int, PaintHandle)>(
  symbol: 'canvas_drawVertices',
  isLeaf: true,
)
external void canvasDrawVertices(
  CanvasHandle handle,
  VerticesHandle vertices,
  int blendMode,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, Int, RawPointArray, Int, PaintHandle)>(
  symbol: 'canvas_drawPoints',
  isLeaf: true,
)
external void canvasDrawPoints(
  CanvasHandle handle,
  int pointMode,
  RawPointArray points,
  int pointCount,
  PaintHandle paint,
);

@Native<
  Void Function(
    CanvasHandle,
    ImageHandle,
    RawRSTransformArray,
    RawRect,
    RawColorArray,
    Int,
    Int,
    RawRect,
    PaintHandle,
  )
>(symbol: 'canvas_drawAtlas', isLeaf: true)
external void canvasDrawAtlas(
  CanvasHandle handle,
  ImageHandle atlas,
  RawRSTransformArray transforms,
  RawRect rects,
  RawColorArray colors,
  int spriteCount,
  int blendMode,
  RawRect cullRect,
  PaintHandle paint,
);

@Native<Void Function(CanvasHandle, RawMatrix44)>(symbol: 'canvas_getTransform', isLeaf: true)
external void canvasGetTransform(CanvasHandle canvas, RawMatrix44 outMatrix);

@Native<Void Function(CanvasHandle, RawRect)>(symbol: 'canvas_getLocalClipBounds', isLeaf: true)
external void canvasGetLocalClipBounds(CanvasHandle canvas, RawRect outRect);

@Native<Void Function(CanvasHandle, RawIRect)>(symbol: 'canvas_getDeviceClipBounds', isLeaf: true)
external void canvasGetDeviceClipBounds(CanvasHandle canvas, RawIRect outRect);
