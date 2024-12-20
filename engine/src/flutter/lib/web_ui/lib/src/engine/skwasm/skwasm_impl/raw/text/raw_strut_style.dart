// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawStrutStyle extends Opaque {}

typedef StrutStyleHandle = Pointer<RawStrutStyle>;

@Native<StrutStyleHandle Function()>(symbol: 'strutStyle_create', isLeaf: true)
external StrutStyleHandle strutStyleCreate();

@Native<Void Function(StrutStyleHandle)>(symbol: 'strutStyle_dispose', isLeaf: true)
external void strutStyleDispose(StrutStyleHandle handle);

@Native<Void Function(StrutStyleHandle, Pointer<SkStringHandle> families, Int count)>(
  symbol: 'strutStyle_setFontFamilies',
  isLeaf: true,
)
external void strutStyleSetFontFamilies(
  StrutStyleHandle handle,
  Pointer<SkStringHandle> families,
  int count,
);

@Native<Void Function(StrutStyleHandle, Float)>(symbol: 'strutStyle_setFontSize', isLeaf: true)
external void strutStyleSetFontSize(StrutStyleHandle handle, double fontSize);

@Native<Void Function(StrutStyleHandle, Float)>(symbol: 'strutStyle_setHeight', isLeaf: true)
external void strutStyleSetHeight(StrutStyleHandle handle, double height);

@Native<Void Function(StrutStyleHandle, Bool)>(symbol: 'strutStyle_setHalfLeading', isLeaf: true)
external void strutStyleSetHalfLeading(StrutStyleHandle handle, bool height);

@Native<Void Function(StrutStyleHandle, Float)>(symbol: 'strutStyle_setLeading', isLeaf: true)
external void strutStyleSetLeading(StrutStyleHandle handle, double leading);

@Native<Void Function(StrutStyleHandle, Int, Int)>(symbol: 'strutStyle_setFontStyle', isLeaf: true)
external void strutStyleSetFontStyle(StrutStyleHandle handle, int weight, int slant);

@Native<Void Function(StrutStyleHandle, Bool)>(
  symbol: 'strutStyle_setForceStrutHeight',
  isLeaf: true,
)
external void strutStyleSetForceStrutHeight(StrutStyleHandle handle, bool forceStrutHeight);
