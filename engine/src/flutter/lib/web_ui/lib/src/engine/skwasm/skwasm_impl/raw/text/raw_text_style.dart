// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawTextStyle extends Opaque {}

typedef TextStyleHandle = Pointer<RawTextStyle>;

@Native<TextStyleHandle Function()>(symbol: 'textStyle_create', isLeaf: true)
external TextStyleHandle textStyleCreate();

@Native<TextStyleHandle Function(TextStyleHandle)>(symbol: 'textStyle_copy', isLeaf: true)
external TextStyleHandle textStyleCopy(TextStyleHandle handle);

@Native<Void Function(TextStyleHandle)>(symbol: 'textStyle_dispose', isLeaf: true)
external void textStyleDispose(TextStyleHandle handle);

@Native<Void Function(TextStyleHandle, Int32)>(symbol: 'textStyle_setColor', isLeaf: true)
external void textStyleSetColor(TextStyleHandle handle, int color);

@Native<Void Function(TextStyleHandle, Int)>(symbol: 'textStyle_setDecoration', isLeaf: true)
external void textStyleSetDecoration(TextStyleHandle handle, int decoration);

@Native<Void Function(TextStyleHandle, Int32)>(symbol: 'textStyle_setDecorationColor', isLeaf: true)
external void textStyleSetDecorationColor(TextStyleHandle handle, int decorationColor);

@Native<Void Function(TextStyleHandle, Int)>(symbol: 'textStyle_setDecorationStyle', isLeaf: true)
external void textStyleSetDecorationStyle(TextStyleHandle handle, int decorationStyle);

@Native<Void Function(TextStyleHandle, Float)>(
  symbol: 'textStyle_setDecorationThickness',
  isLeaf: true,
)
external void textStyleSetDecorationThickness(TextStyleHandle handle, double decorationThickness);

@Native<Void Function(TextStyleHandle, Int, Int)>(symbol: 'textStyle_setFontStyle', isLeaf: true)
external void textStyleSetFontStyle(TextStyleHandle handle, int weight, int slant);

@Native<Void Function(TextStyleHandle, Int)>(symbol: 'textStyle_setTextBaseline', isLeaf: true)
external void textStyleSetTextBaseline(TextStyleHandle handle, int baseline);

@Native<Void Function(TextStyleHandle)>(symbol: 'textStyle_clearFontFamilies', isLeaf: true)
external void textStyleClearFontFamilies(TextStyleHandle handle);

@Native<Void Function(TextStyleHandle, Pointer<SkStringHandle>, Int count)>(
  symbol: 'textStyle_addFontFamilies',
  isLeaf: true,
)
external void textStyleAddFontFamilies(
  TextStyleHandle handle,
  Pointer<SkStringHandle> families,
  int count,
);

@Native<Void Function(TextStyleHandle, Float)>(symbol: 'textStyle_setFontSize', isLeaf: true)
external void textStyleSetFontSize(TextStyleHandle handle, double size);

@Native<Void Function(TextStyleHandle, Float)>(symbol: 'textStyle_setLetterSpacing', isLeaf: true)
external void textStyleSetLetterSpacing(TextStyleHandle handle, double spacing);

@Native<Void Function(TextStyleHandle, Float)>(symbol: 'textStyle_setWordSpacing', isLeaf: true)
external void textStyleSetWordSpacing(TextStyleHandle handle, double spacing);

@Native<Void Function(TextStyleHandle, Float)>(symbol: 'textStyle_setHeight', isLeaf: true)
external void textStyleSetHeight(TextStyleHandle handle, double height);

@Native<Void Function(TextStyleHandle, Bool)>(symbol: 'textStyle_setHalfLeading', isLeaf: true)
external void textStyleSetHalfLeading(TextStyleHandle handle, bool halfLeading);

@Native<Void Function(TextStyleHandle, SkStringHandle)>(symbol: 'textStyle_setLocale', isLeaf: true)
external void textStyleSetLocale(TextStyleHandle handle, SkStringHandle locale);

@Native<Void Function(TextStyleHandle, PaintHandle)>(
  symbol: 'textStyle_setBackground',
  isLeaf: true,
)
external void textStyleSetBackground(TextStyleHandle handle, PaintHandle paint);

@Native<Void Function(TextStyleHandle, PaintHandle)>(
  symbol: 'textStyle_setForeground',
  isLeaf: true,
)
external void textStyleSetForeground(TextStyleHandle handle, PaintHandle paint);

@Native<Void Function(TextStyleHandle, Int32, Float, Float, Float)>(
  symbol: 'textStyle_addShadow',
  isLeaf: true,
)
external void textStyleAddShadow(
  TextStyleHandle handle,
  int color,
  double offsetX,
  double offsetY,
  double blurSigma,
);

@Native<Void Function(TextStyleHandle, SkStringHandle, Int)>(
  symbol: 'textStyle_addFontFeature',
  isLeaf: true,
)
external void textStyleAddFontFeature(
  TextStyleHandle handle,
  SkStringHandle featureName,
  int value,
);

@Native<Void Function(TextStyleHandle, Pointer<Uint32>, Pointer<Float>, Int)>(
  symbol: 'textStyle_setFontVariations',
  isLeaf: true,
)
external void textStyleSetFontVariations(
  TextStyleHandle handle,
  Pointer<Uint32> axes,
  Pointer<Float> values,
  int count,
);
