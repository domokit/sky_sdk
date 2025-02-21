// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
library;

import 'dart:ui' show Canvas, Clip, Paint, Path, RRect, RSuperellipse, Rect, VoidCallback;

/// Clip utilities used by [PaintingContext].
abstract class ClipContext {
  /// The canvas on which to paint.
  Canvas get canvas;

  void _clipAndPaint(
    void Function(bool doAntiAlias) canvasClipCall,
    Clip clipBehavior,
    Rect bounds,
    VoidCallback painter,
  ) {
    canvas.save();
    switch (clipBehavior) {
      case Clip.none:
        break;
      case Clip.hardEdge:
        canvasClipCall(false);
      case Clip.antiAlias:
        canvasClipCall(true);
      case Clip.antiAliasWithSaveLayer:
        canvasClipCall(true);
        canvas.saveLayer(bounds, Paint());
    }
    painter();
    if (clipBehavior == Clip.antiAliasWithSaveLayer) {
      canvas.restore();
    }
    canvas.restore();
  }

  /// Clip [canvas] with [Path] according to [Clip] and then paint. [canvas] is
  /// restored to the pre-clip status afterwards.
  ///
  /// `bounds` is the saveLayer bounds used for [Clip.antiAliasWithSaveLayer].
  void clipPathAndPaint(Path path, Clip clipBehavior, Rect bounds, VoidCallback painter) {
    _clipAndPaint(
      (bool doAntiAlias) => canvas.clipPath(path, doAntiAlias: doAntiAlias),
      clipBehavior,
      bounds,
      painter,
    );
  }

  /// Clip [canvas] with [Path] according to `rrect` and then paint. [canvas] is
  /// restored to the pre-clip status afterwards.
  ///
  /// `bounds` is the saveLayer bounds used for [Clip.antiAliasWithSaveLayer].
  void clipRRectAndPaint(RRect rrect, Clip clipBehavior, Rect bounds, VoidCallback painter) {
    _clipAndPaint(
      (bool doAntiAlias) => canvas.clipRRect(rrect, doAntiAlias: doAntiAlias),
      clipBehavior,
      bounds,
      painter,
    );
  }

  /// Clip [canvas] with [Path] according to the given rounded superellipse and
  /// then paint. [canvas] is restored to the pre-clip status afterwards.
  ///
  /// The `bounds` is the saveLayer bounds used for
  /// [Clip.antiAliasWithSaveLayer].
  void clipRSuperellipseAndPaint(
    RSuperellipse rse,
    Clip clipBehavior,
    Rect bounds,
    VoidCallback painter,
  ) {
    _clipAndPaint(
      (bool doAntiAlias) => canvas.clipRSuperellipse(rse, doAntiAlias: doAntiAlias),
      clipBehavior,
      bounds,
      painter,
    );
  }

  /// Clip [canvas] with [Path] according to `rect` and then paint. [canvas] is
  /// restored to the pre-clip status afterwards.
  ///
  /// `bounds` is the saveLayer bounds used for [Clip.antiAliasWithSaveLayer].
  void clipRectAndPaint(Rect rect, Clip clipBehavior, Rect bounds, VoidCallback painter) {
    _clipAndPaint(
      (bool doAntiAlias) => canvas.clipRect(rect, doAntiAlias: doAntiAlias),
      clipBehavior,
      bounds,
      painter,
    );
  }
}
