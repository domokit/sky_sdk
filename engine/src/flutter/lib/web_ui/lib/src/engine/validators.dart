// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

bool rectIsValid(ui.Rect rect) {
  assert(
    !(rect.left.isNaN || rect.right.isNaN || rect.top.isNaN || rect.bottom.isNaN),
    'Rect argument contained a NaN value.',
  );
  return true;
}

bool rrectIsValid(ui.RRect rrect) {
  assert(
    !(rrect.left.isNaN || rrect.right.isNaN || rrect.top.isNaN || rrect.bottom.isNaN),
    'RRect argument contained a NaN value.',
  );
  return true;
}

bool offsetIsValid(ui.Offset offset) {
  assert(!offset.dx.isNaN && !offset.dy.isNaN, 'Offset argument contained a NaN value.');
  return true;
}

bool matrix4IsValid(Float32List matrix4) {
  assert(matrix4.length == 16, 'Matrix4 must have 16 entries.');
  return true;
}

bool radiusIsValid(ui.Radius radius) {
  assert(!radius.x.isNaN && !radius.y.isNaN, 'Radius argument contained a NaN value.');
  return true;
}

/// Validates color and color stops used for a gradient.
void validateColorStops(List<ui.Color> colors, List<double>? colorStops) {
  if (colorStops == null) {
    if (colors.length != 2) {
      throw ArgumentError('"colors" must have length 2 if "colorStops" is omitted.');
    }
  } else {
    if (colors.length != colorStops.length) {
      throw ArgumentError('"colors" and "colorStops" arguments must have equal length.');
    }
  }
}
