// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'canvas.dart';
import 'painting.dart';
import 'path.dart';

/// A virtual canvas that applies operations to multiple canvases at once.
class CkNWayCanvas {
  final List<CkCanvas> _canvases = <CkCanvas>[];

  void addCanvas(CkCanvas canvas) {
    _canvases.add(canvas);
  }

  /// Calls [save] on all canvases.
  int save() {
    int saveCount = 0;
    for (int i = 0; i < _canvases.length; i++) {
      saveCount = _canvases[i].save();
    }
    return saveCount;
  }

  /// Calls [saveLayer] on all canvases.
  void saveLayer(ui.Rect bounds, CkPaint? paint) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].saveLayer(bounds, paint);
    }
  }

  /// Calls [saveLayerWithFilter] on all canvases.
  void saveLayerWithFilter(ui.Rect bounds, ui.ImageFilter filter, [CkPaint? paint]) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].saveLayerWithFilter(bounds, filter, paint);
    }
  }

  /// Calls [restore] on all canvases.
  void restore() {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].restore();
    }
  }

  /// Calls [restoreToCount] on all canvases.
  void restoreToCount(int count) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].restoreToCount(count);
    }
  }

  /// Calls [translate] on all canvases.
  void translate(double dx, double dy) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].translate(dx, dy);
    }
  }

  /// Calls [transform] on all canvases.
  void transform(Float32List matrix) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].transform(matrix);
    }
  }

  /// Calls [clear] on all canvases.
  void clear(ui.Color color) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].clear(color);
    }
  }

  /// Calls [clipPath] on all canvases.
  void clipPath(CkPath path, bool doAntiAlias) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].clipPath(path, doAntiAlias);
    }
  }

  /// Calls [clipRect] on all canvases.
  void clipRect(ui.Rect rect, ui.ClipOp clipOp, bool doAntiAlias) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].clipRect(rect, clipOp, doAntiAlias);
    }
  }

  /// Calls [clipRRect] on all canvases.
  void clipRRect(ui.RRect rrect, bool doAntiAlias) {
    for (int i = 0; i < _canvases.length; i++) {
      _canvases[i].clipRRect(rrect, doAntiAlias);
    }
  }
}
