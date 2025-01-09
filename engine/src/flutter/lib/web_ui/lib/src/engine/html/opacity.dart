// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../util.dart';
import '../vector_math.dart';
import 'surface.dart';

/// A surface that makes its children transparent.
class PersistedOpacity extends PersistedContainerSurface implements ui.OpacityEngineLayer {
  PersistedOpacity(PersistedOpacity? super.oldLayer, this.alpha, this.offset);

  final int alpha;
  final ui.Offset offset;

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform;

    final double dx = offset.dx;
    final double dy = offset.dy;

    if (dx != 0.0 || dy != 0.0) {
      transform = transform!.clone();
      transform!.translate(dx, dy);
    }
    projectedClip = null;
  }

  /// Cached inverse of transform on this node. Unlike transform, this
  /// Matrix only contains local transform (not chain multiplied since root).
  Matrix4? _localTransformInverse;

  @override
  Matrix4 get localTransformInverse =>
      _localTransformInverse ??= Matrix4.translationValues(-offset.dx, -offset.dy, 0);

  @override
  DomElement createElement() {
    final DomElement element = domDocument.createElement('flt-opacity');
    setElementStyle(element, 'position', 'absolute');
    setElementStyle(element, 'transform-origin', '0 0 0');
    return element;
  }

  @override
  void apply() {
    final DomElement element = rootElement!;
    setElementStyle(element, 'opacity', '${alpha / 255}');
    element.style.transform = 'translate(${offset.dx}px, ${offset.dy}px)';
  }

  @override
  void update(PersistedOpacity oldSurface) {
    super.update(oldSurface);
    if (alpha != oldSurface.alpha || offset != oldSurface.offset) {
      apply();
    }
  }
}
