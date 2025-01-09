// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../util.dart';
import '../vector_math.dart';
import 'surface.dart';

/// A surface that transforms its children using CSS transform.
class PersistedTransform extends PersistedContainerSurface implements ui.TransformEngineLayer {
  PersistedTransform(PersistedTransform? super.oldLayer, this._matrixStorage);

  /// The storage representing the transform of this surface.
  final Float32List _matrixStorage;

  /// The matrix representing the transform of this surface.
  Matrix4 get matrix4 => _matrix4 ??= Matrix4.fromFloat32List(_matrixStorage);
  Matrix4? _matrix4;

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform!.multiplied(matrix4);
    projectedClip = null;
  }

  /// Cached inverse of transform on this node. Unlike [transform], this
  /// Matrix only contains local transform (not chain multiplied since root).
  Matrix4? _localTransformInverse;

  @override
  Matrix4? get localTransformInverse {
    _localTransformInverse ??= Matrix4.tryInvert(matrix4);
    return _localTransformInverse;
  }

  @override
  DomElement createElement() {
    final DomElement element = domDocument.createElement('flt-transform');
    setElementStyle(element, 'position', 'absolute');
    setElementStyle(element, 'transform-origin', '0 0 0');
    return element;
  }

  @override
  void apply() {
    rootElement!.style.transform = float64ListToCssTransform(_matrixStorage);
  }

  @override
  void update(PersistedTransform oldSurface) {
    super.update(oldSurface);

    if (identical(oldSurface._matrixStorage, _matrixStorage)) {
      // The matrix storage is identical, so we can copy the matrices from the
      // old surface to avoid recomputing them.
      _matrix4 = oldSurface._matrix4;
      _localTransformInverse = oldSurface._localTransformInverse;
      return;
    }

    bool matrixChanged = false;
    for (int i = 0; i < _matrixStorage.length; i++) {
      if (_matrixStorage[i] != oldSurface._matrixStorage[i]) {
        matrixChanged = true;
        break;
      }
    }

    if (matrixChanged) {
      apply();
    } else {
      // The matrix storage hasn't changed, so we can copy the matrices from the
      // old surface to avoid recomputing them.
      _matrix4 = oldSurface._matrix4;
      _localTransformInverse = oldSurface._localTransformInverse;
    }
  }
}
