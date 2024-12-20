// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawImageFilter extends Opaque {}

typedef ImageFilterHandle = Pointer<RawImageFilter>;

final class RawColorFilter extends Opaque {}

typedef ColorFilterHandle = Pointer<RawColorFilter>;

final class RawMaskFilter extends Opaque {}

typedef MaskFilterHandle = Pointer<RawMaskFilter>;

@Native<ImageFilterHandle Function(Float, Float, Int)>(
  symbol: 'imageFilter_createBlur',
  isLeaf: true,
)
external ImageFilterHandle imageFilterCreateBlur(double sigmaX, double sigmaY, int tileMode);

@Native<ImageFilterHandle Function(Float, Float)>(symbol: 'imageFilter_createDilate', isLeaf: true)
external ImageFilterHandle imageFilterCreateDilate(double radiusX, double radiusY);

@Native<ImageFilterHandle Function(Float, Float)>(symbol: 'imageFilter_createErode', isLeaf: true)
external ImageFilterHandle imageFilterCreateErode(double radiusX, double radiusY);

@Native<ImageFilterHandle Function(Pointer<Float>, Int)>(
  symbol: 'imageFilter_createMatrix',
  isLeaf: true,
)
external ImageFilterHandle imageFilterCreateMatrix(Pointer<Float> matrix33, int quality);

@Native<ImageFilterHandle Function(ColorFilterHandle)>(
  symbol: 'imageFilter_createFromColorFilter',
  isLeaf: true,
)
external ImageFilterHandle imageFilterCreateFromColorFilter(ColorFilterHandle colorFilte);

@Native<ImageFilterHandle Function(ImageFilterHandle, ImageFilterHandle)>(
  symbol: 'imageFilter_compose',
  isLeaf: true,
)
external ImageFilterHandle imageFilterCompose(ImageFilterHandle outer, ImageFilterHandle inner);

@Native<Void Function(ImageFilterHandle)>(symbol: 'imageFilter_dispose', isLeaf: true)
external void imageFilterDispose(ImageFilterHandle handle);

@Native<Void Function(ImageFilterHandle, RawIRect)>(
  symbol: 'imageFilter_getFilterBounds',
  isLeaf: true,
)
external void imageFilterGetFilterBounds(ImageFilterHandle handle, RawIRect inOutRect);

@Native<ColorFilterHandle Function(Int, Int)>(symbol: 'colorFilter_createMode', isLeaf: true)
external ColorFilterHandle colorFilterCreateMode(int color, int mode);

@Native<ColorFilterHandle Function(Pointer<Float>)>(
  symbol: 'colorFilter_createMatrix',
  isLeaf: true,
)
external ColorFilterHandle colorFilterCreateMatrix(Pointer<Float> matrix);

@Native<ColorFilterHandle Function()>(symbol: 'colorFilter_createSRGBToLinearGamma', isLeaf: true)
external ColorFilterHandle colorFilterCreateSRGBToLinearGamma();

@Native<ColorFilterHandle Function()>(symbol: 'colorFilter_createLinearToSRGBGamma', isLeaf: true)
external ColorFilterHandle colorFilterCreateLinearToSRGBGamma();

@Native<ColorFilterHandle Function(ColorFilterHandle, ColorFilterHandle)>(
  symbol: 'colorFilter_compose',
  isLeaf: true,
)
external ColorFilterHandle colorFilterCompose(ColorFilterHandle outer, ColorFilterHandle inner);

@Native<Void Function(ColorFilterHandle)>(symbol: 'colorFilter_dispose', isLeaf: true)
external void colorFilterDispose(ColorFilterHandle handle);

@Native<MaskFilterHandle Function(Int, Float)>(symbol: 'maskFilter_createBlur', isLeaf: true)
external MaskFilterHandle maskFilterCreateBlur(int blurStyle, double sigma);

@Native<Void Function(MaskFilterHandle)>(symbol: 'maskFilter_dispose', isLeaf: true)
external void maskFilterDispose(MaskFilterHandle handle);
