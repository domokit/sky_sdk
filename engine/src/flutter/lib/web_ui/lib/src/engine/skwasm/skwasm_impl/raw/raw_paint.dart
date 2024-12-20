// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawPaint extends Opaque {}

typedef PaintHandle = Pointer<RawPaint>;

typedef _PaintCreateInitSignature =
    PaintHandle Function(Bool, Int, Int, Int, Float, Int, Int, Float);

@Native<_PaintCreateInitSignature>(symbol: 'paint_create', isLeaf: true)
external PaintHandle paintCreate(
  bool isAntiAlias,
  int blendMode,
  int color,
  int style,
  double strokeWidth,
  int strokeCap,
  int strokeJoin,
  double strokeMiterLimit,
);

@Native<Void Function(PaintHandle)>(symbol: 'paint_dispose', isLeaf: true)
external void paintDispose(PaintHandle paint);

@Native<Void Function(PaintHandle, ShaderHandle)>(symbol: 'paint_setShader', isLeaf: true)
external void paintSetShader(PaintHandle handle, ShaderHandle shader);

@Native<Void Function(PaintHandle, ImageFilterHandle)>(symbol: 'paint_setImageFilter', isLeaf: true)
external void paintSetImageFilter(PaintHandle handle, ImageFilterHandle filter);

@Native<Void Function(PaintHandle, ColorFilterHandle)>(symbol: 'paint_setColorFilter', isLeaf: true)
external void paintSetColorFilter(PaintHandle handle, ColorFilterHandle filter);

@Native<Void Function(PaintHandle, MaskFilterHandle)>(symbol: 'paint_setMaskFilter', isLeaf: true)
external void paintSetMaskFilter(PaintHandle handle, MaskFilterHandle filter);
