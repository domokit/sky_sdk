// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

final class RawSkData extends Opaque {}

typedef SkDataHandle = Pointer<RawSkData>;

@Native<SkDataHandle Function(Size)>(symbol: 'skData_create', isLeaf: true)
external SkDataHandle skDataCreate(int size);

@Native<Pointer<Void> Function(SkDataHandle)>(symbol: 'skData_getPointer', isLeaf: true)
external Pointer<Void> skDataGetPointer(SkDataHandle handle);

@Native<Pointer<Void> Function(SkDataHandle)>(symbol: 'skData_getConstPointer', isLeaf: true)
external Pointer<Void> skDataGetConstPointer(SkDataHandle handle);

@Native<Size Function(SkDataHandle)>(symbol: 'skData_getSize', isLeaf: true)
external int skDataGetSize(SkDataHandle handle);

@Native<Void Function(SkDataHandle)>(symbol: 'skData_dispose', isLeaf: true)
external void skDataDispose(SkDataHandle handle);
