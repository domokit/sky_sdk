// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import 'canvaskit_api.dart';

/// Creates and returns a [SkMaskFilter] that applies a blur effect.
///
/// It is the responsibility of the caller to delete the returned Skia object.
SkMaskFilter createBlurSkMaskFilter(ui.BlurStyle blurStyle, double sigma) {
  return canvasKit.MaskFilter.MakeBlur(toSkBlurStyle(blurStyle), sigma, true)!;
}
