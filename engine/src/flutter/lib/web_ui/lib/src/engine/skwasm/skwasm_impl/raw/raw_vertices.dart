// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawVertices extends Opaque {}

typedef VerticesHandle = Pointer<RawVertices>;

@Native<
  VerticesHandle Function(
    Int vertexMode,
    Int vertexCount,
    RawPointArray positions,
    RawPointArray textureCoordinates,
    RawColorArray colors,
    Int indexCount,
    Pointer<Uint16> indices,
  )
>(symbol: 'vertices_create', isLeaf: true)
external VerticesHandle verticesCreate(
  int vertexMode,
  int vertexCount,
  RawPointArray positions,
  RawPointArray textureCoordinates,
  RawColorArray colors,
  int indexCount,
  Pointer<Uint16> indices,
);

@Native<Void Function(VerticesHandle)>(symbol: 'vertices_dispose', isLeaf: true)
external void verticesDispose(VerticesHandle handle);
