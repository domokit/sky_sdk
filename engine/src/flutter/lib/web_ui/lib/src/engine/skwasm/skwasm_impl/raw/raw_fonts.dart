// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawFontCollection extends Opaque {}

typedef FontCollectionHandle = Pointer<RawFontCollection>;

final class RawTypeface extends Opaque {}

typedef TypefaceHandle = Pointer<RawTypeface>;

@Native<FontCollectionHandle Function()>(symbol: 'fontCollection_create', isLeaf: true)
external FontCollectionHandle fontCollectionCreate();

@Native<Void Function(FontCollectionHandle)>(symbol: 'fontCollection_dispose', isLeaf: true)
external void fontCollectionDispose(FontCollectionHandle handle);

@Native<TypefaceHandle Function(SkDataHandle)>(symbol: 'typeface_create', isLeaf: true)
external TypefaceHandle typefaceCreate(SkDataHandle fontData);

@Native<Void Function(TypefaceHandle)>(symbol: 'typeface_dispose', isLeaf: true)
external void typefaceDispose(TypefaceHandle handle);

@Native<Int Function(Pointer<TypefaceHandle>, Int, Pointer<Int32>, Int)>(
  symbol: 'typefaces_filterCoveredCodePoints',
  isLeaf: true,
)
external int typefacesFilterCoveredCodePoints(
  Pointer<TypefaceHandle> typefaces,
  int typefaceCount,
  Pointer<Int32> codepoints,
  int codePointCount,
);

@Native<Void Function(FontCollectionHandle, TypefaceHandle, SkStringHandle)>(
  symbol: 'fontCollection_registerTypeface',
  isLeaf: true,
)
external void fontCollectionRegisterTypeface(
  FontCollectionHandle handle,
  TypefaceHandle typeface,
  SkStringHandle fontName,
);

@Native<Void Function(FontCollectionHandle)>(symbol: 'fontCollection_clearCaches', isLeaf: true)
external void fontCollectionClearCaches(FontCollectionHandle handle);
