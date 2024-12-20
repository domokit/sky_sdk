// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../util.dart';
import '../vector_math.dart';
import 'surface.dart';

/// A surface that translates its children using CSS transform and translate.
class PersistedOffset extends PersistedContainerSurface implements ui.OffsetEngineLayer {
  PersistedOffset(PersistedOffset? super.oldLayer, this.dx, this.dy);

  /// Horizontal displacement.
  final double dx;

  /// Vertical displacement.
  final double dy;

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform;
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
      _localTransformInverse ??= Matrix4.translationValues(-dx, -dy, 0);

  @override
  DomElement createElement() {
    final DomElement element = domDocument.createElement('flt-offset');
    setElementStyle(element, 'position', 'absolute');
    setElementStyle(element, 'transform-origin', '0 0 0');
    return element;
  }

  @override
  void apply() {
    rootElement!.style.transform = 'translate(${dx}px, ${dy}px)';
  }

  @override
  void update(PersistedOffset oldSurface) {
    super.update(oldSurface);

    if (oldSurface.dx != dx || oldSurface.dy != dy) {
      apply();
    }
  }
}
