// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'canvaskit_api.dart';
import 'native_memory.dart';

class CkVertices implements ui.Vertices {
  factory CkVertices(
    ui.VertexMode mode,
    List<ui.Offset> positions, {
    List<ui.Offset>? textureCoordinates,
    List<ui.Color>? colors,
    List<int>? indices,
  }) {
    if (textureCoordinates != null && textureCoordinates.length != positions.length) {
      throw ArgumentError('"positions" and "textureCoordinates" lengths must match.');
    }
    if (colors != null && colors.length != positions.length) {
      throw ArgumentError('"positions" and "colors" lengths must match.');
    }
    if (indices != null && indices.any((int i) => i < 0 || i >= positions.length)) {
      throw ArgumentError('"indices" values must be valid indices in the positions list.');
    }

    return CkVertices._(
      toSkVertexMode(mode),
      toFlatSkPoints(positions),
      textureCoordinates != null ? toFlatSkPoints(textureCoordinates) : null,
      colors != null ? toFlatColors(colors) : null,
      indices != null ? toUint16List(indices) : null,
    );
  }

  factory CkVertices.raw(
    ui.VertexMode mode,
    Float32List positions, {
    Float32List? textureCoordinates,
    Int32List? colors,
    Uint16List? indices,
  }) {
    if (textureCoordinates != null && textureCoordinates.length != positions.length) {
      throw ArgumentError('"positions" and "textureCoordinates" lengths must match.');
    }
    if (colors != null && colors.length * 2 != positions.length) {
      throw ArgumentError('"positions" and "colors" lengths must match.');
    }
    if (indices != null && indices.any((int i) => i < 0 || i >= positions.length)) {
      throw ArgumentError('"indices" values must be valid indices in the positions list.');
    }

    Uint32List? unsignedColors;
    if (colors != null) {
      unsignedColors = colors.buffer.asUint32List(colors.offsetInBytes, colors.length);
    }

    return CkVertices._(
      toSkVertexMode(mode),
      positions,
      textureCoordinates,
      unsignedColors,
      indices,
    );
  }

  CkVertices._(this._mode, this._positions, this._textureCoordinates, this._colors, this._indices) {
    final SkVertices skVertices = canvasKit.MakeVertices(
      _mode,
      _positions,
      _textureCoordinates,
      _colors,
      _indices,
    );
    _ref = UniqueRef<SkVertices>(this, skVertices, 'Vertices');
  }

  final SkVertexMode _mode;
  final Float32List _positions;
  final Float32List? _textureCoordinates;
  final Uint32List? _colors;
  final Uint16List? _indices;
  late final UniqueRef<SkVertices> _ref;

  SkVertices get skiaObject => _ref.nativeObject;

  @override
  void dispose() {
    _ref.dispose();
  }

  @override
  bool get debugDisposed => _ref.isDisposed;
}
