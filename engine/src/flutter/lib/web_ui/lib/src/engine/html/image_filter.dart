// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../color_filter.dart';
import '../dom.dart';
import '../util.dart';
import '../vector_math.dart';
import 'resource_manager.dart';
import 'shaders/shader.dart';
import 'surface.dart';
import 'surface_stats.dart';

/// A surface that applies an [imageFilter] to its children.
class PersistedImageFilter extends PersistedContainerSurface implements ui.ImageFilterEngineLayer {
  PersistedImageFilter(PersistedImageFilter? super.oldLayer, this.filter, this.offset);

  final ui.ImageFilter filter;
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

  DomElement? _svgFilter;
  @override
  DomElement? get childContainer => _childContainer;
  DomElement? _childContainer;

  @override
  void adoptElements(PersistedImageFilter oldSurface) {
    super.adoptElements(oldSurface);
    _svgFilter = oldSurface._svgFilter;
    _childContainer = oldSurface._childContainer;
    oldSurface._svgFilter = null;
    oldSurface._childContainer = null;
  }

  @override
  void discard() {
    super.discard();
    ResourceManager.instance.removeResource(_svgFilter);
    _svgFilter = null;
    _childContainer = null;
  }

  @override
  DomElement createElement() {
    final DomElement element = defaultCreateElement('flt-image-filter');
    final DomElement container = defaultCreateElement('flt-image-filter-interior');
    if (debugExplainSurfaceStats) {
      // This creates an additional interior element. Count it too.
      surfaceStatsFor(this).allocatedDomNodeCount++;
    }

    setElementStyle(container, 'position', 'absolute');
    setElementStyle(container, 'transform-origin', '0 0 0');
    setElementStyle(element, 'position', 'absolute');
    setElementStyle(element, 'transform-origin', '0 0 0');

    _childContainer = container;
    element.appendChild(container);
    return element;
  }

  @override
  void apply() {
    EngineImageFilter backendFilter;
    if (filter is ui.ColorFilter) {
      backendFilter = createHtmlColorFilter(filter as EngineColorFilter)!;
    } else {
      backendFilter = filter as EngineImageFilter;
    }
    ResourceManager.instance.removeResource(_svgFilter);
    _svgFilter = null;
    if (backendFilter is ModeHtmlColorFilter) {
      _svgFilter = backendFilter.makeSvgFilter(rootElement);

      /// Some blendModes do not make an svgFilter. See [EngineHtmlColorFilter.makeSvgFilter()]
      if (_svgFilter == null) {
        return;
      }
    } else if (backendFilter is MatrixHtmlColorFilter) {
      _svgFilter = backendFilter.makeSvgFilter(rootElement);
    }

    _childContainer!.style.filter = backendFilter.filterAttribute;
    _childContainer!.style.transform = backendFilter.transformAttribute;
    rootElement!.style
      ..left = '${offset.dx}px'
      ..top = '${offset.dy}px';
  }

  @override
  void update(PersistedImageFilter oldSurface) {
    super.update(oldSurface);

    if (oldSurface.filter != filter || oldSurface.offset != offset) {
      apply();
    }
  }
}
