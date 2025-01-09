// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../svg.dart';
import '../util.dart';
import 'path_to_svg_clip.dart';
import 'surface.dart';
import 'surface_stats.dart';

/// Mixin used by surfaces that clip their contents using an overflowing DOM
/// element.
mixin _DomClip on PersistedContainerSurface {
  /// The dedicated child container element that's separate from the
  /// [rootElement] is used to compensate for the coordinate system shift
  /// introduced by the [rootElement] translation.
  @override
  DomElement? get childContainer => _childContainer;
  DomElement? _childContainer;

  @override
  void adoptElements(_DomClip oldSurface) {
    super.adoptElements(oldSurface);
    _childContainer = oldSurface._childContainer;
    oldSurface._childContainer = null;
  }

  @override
  DomElement createElement() {
    final DomElement element = defaultCreateElement('flt-clip');
    _childContainer = createDomElement('flt-clip-interior');
    if (debugExplainSurfaceStats) {
      // This creates an additional interior element. Count it too.
      surfaceStatsFor(this).allocatedDomNodeCount++;
    }
    _childContainer!.style.position = 'absolute';

    element.append(_childContainer!);
    return element;
  }

  @override
  void discard() {
    super.discard();

    // Do not detach the child container from the root. It is permanently
    // attached. The elements are reused together and are detached from the DOM
    // together.
    _childContainer = null;
  }

  void applyOverflow(DomElement element, ui.Clip? clipBehaviour) {
    if (!debugShowClipLayers) {
      // Hide overflow in production mode. When debugging we want to see the
      // clipped picture in full.
      if (clipBehaviour != ui.Clip.none) {
        element.style
          ..overflow = 'hidden'
          ..zIndex = '0';
      }
    } else {
      // Display the outline of the clipping region. When debugShowClipLayers is
      // `true` we don't hide clip overflow (see above). This outline helps
      // visualizing clip areas.
      element.style.boxShadow = 'inset 0 0 10px green';
    }
  }
}

/// A surface that creates a rectangular clip.
class PersistedClipRect extends PersistedContainerSurface
    with _DomClip
    implements ui.ClipRectEngineLayer {
  PersistedClipRect(PersistedClipRect? super.oldLayer, this.rect, this.clipBehavior);
  final ui.Clip? clipBehavior;
  final ui.Rect rect;

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform;
    if (clipBehavior != ui.Clip.none) {
      localClipBounds = rect;
    } else {
      localClipBounds = null;
    }
    projectedClip = null;
  }

  @override
  DomElement createElement() {
    return super.createElement()..setAttribute('clip-type', 'rect');
  }

  @override
  void apply() {
    rootElement!.style
      ..left = '${rect.left}px'
      ..top = '${rect.top}px'
      ..width = '${rect.right - rect.left}px'
      ..height = '${rect.bottom - rect.top}px';
    applyOverflow(rootElement!, clipBehavior);

    // Translate the child container in the opposite direction to compensate for
    // the shift in the coordinate system introduced by the translation of the
    // rootElement. Clipping in Flutter has no effect on the coordinate system.
    childContainer!.style
      ..left = '${-rect.left}px'
      ..top = '${-rect.top}px';
  }

  @override
  void update(PersistedClipRect oldSurface) {
    super.update(oldSurface);
    if (rect != oldSurface.rect || clipBehavior != oldSurface.clipBehavior) {
      localClipBounds = null;
      apply();
    }
  }

  @override
  bool get isClipping => true;
}

/// A surface that creates a rounded rectangular clip.
class PersistedClipRRect extends PersistedContainerSurface
    with _DomClip
    implements ui.ClipRRectEngineLayer {
  PersistedClipRRect(ui.EngineLayer? oldLayer, this.rrect, this.clipBehavior)
    : super(oldLayer as PersistedSurface?);

  final ui.RRect rrect;
  // TODO(yjbanov): can this be controlled in the browser?
  final ui.Clip? clipBehavior;

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform;
    if (clipBehavior != ui.Clip.none) {
      localClipBounds = rrect.outerRect;
    } else {
      localClipBounds = null;
    }
    projectedClip = null;
  }

  @override
  DomElement createElement() {
    return super.createElement()..setAttribute('clip-type', 'rrect');
  }

  @override
  void apply() {
    final DomCSSStyleDeclaration style = rootElement!.style;
    style
      ..left = '${rrect.left}px'
      ..top = '${rrect.top}px'
      ..width = '${rrect.width}px'
      ..height = '${rrect.height}px'
      ..borderTopLeftRadius = '${rrect.tlRadiusX}px'
      ..borderTopRightRadius = '${rrect.trRadiusX}px'
      ..borderBottomRightRadius = '${rrect.brRadiusX}px'
      ..borderBottomLeftRadius = '${rrect.blRadiusX}px';
    applyOverflow(rootElement!, clipBehavior);

    // Translate the child container in the opposite direction to compensate for
    // the shift in the coordinate system introduced by the translation of the
    // rootElement. Clipping in Flutter has no effect on the coordinate system.
    childContainer!.style
      ..left = '${-rrect.left}px'
      ..top = '${-rrect.top}px';
  }

  @override
  void update(PersistedClipRRect oldSurface) {
    super.update(oldSurface);
    if (rrect != oldSurface.rrect || clipBehavior != oldSurface.clipBehavior) {
      localClipBounds = null;
      apply();
    }
  }

  @override
  bool get isClipping => true;
}

/// A surface that clips it's children.
class PersistedClipPath extends PersistedContainerSurface implements ui.ClipPathEngineLayer {
  PersistedClipPath(PersistedClipPath? super.oldLayer, this.clipPath, this.clipBehavior);

  final ui.Path clipPath;
  final ui.Clip clipBehavior;
  DomElement? _clipElement;

  @override
  DomElement createElement() {
    return defaultCreateElement('flt-clippath');
  }

  @override
  void recomputeTransformAndClip() {
    super.recomputeTransformAndClip();
    if (clipBehavior != ui.Clip.none) {
      localClipBounds ??= clipPath.getBounds();
    } else {
      localClipBounds = null;
    }
  }

  @override
  void apply() {
    _clipElement?.remove();
    _clipElement = createSvgClipDef(childContainer!, clipPath);
    childContainer!.append(_clipElement!);
  }

  @override
  void update(PersistedClipPath oldSurface) {
    super.update(oldSurface);
    if (oldSurface.clipPath != clipPath) {
      localClipBounds = null;
      oldSurface._clipElement?.remove();
      apply();
    } else {
      _clipElement = oldSurface._clipElement;
    }
    oldSurface._clipElement = null;
  }

  @override
  void discard() {
    _clipElement?.remove();
    _clipElement = null;
    super.discard();
  }

  @override
  bool get isClipping => true;
}

/// Creates an svg clipPath and applies it to [element].
SVGSVGElement createSvgClipDef(DomElement element, ui.Path clipPath) {
  final ui.Rect pathBounds = clipPath.getBounds();
  final SVGSVGElement svgClipPath = pathToSvgClipPath(
    clipPath,
    scaleX: 1.0 / pathBounds.right,
    scaleY: 1.0 / pathBounds.bottom,
  );
  setClipPath(element, createSvgClipUrl());
  // We need to set width and height for the clipElement to cover the
  // bounds of the path since browsers such as Safari and Edge
  // seem to incorrectly intersect the element bounding rect with
  // the clip path. Chrome and Firefox don't perform intersect instead they
  // use the path itself as source of truth.
  element.style
    ..width = '${pathBounds.right}px'
    ..height = '${pathBounds.bottom}px';
  return svgClipPath;
}
