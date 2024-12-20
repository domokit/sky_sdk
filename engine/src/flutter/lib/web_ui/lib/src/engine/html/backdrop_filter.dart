// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../color_filter.dart';
import '../dom.dart';
import '../util.dart';
import '../vector_math.dart';
import 'resource_manager.dart';
import 'shaders/shader.dart';
import 'surface.dart';
import 'surface_stats.dart';

/// A surface that applies an image filter to background.
class PersistedBackdropFilter extends PersistedContainerSurface
    implements ui.BackdropFilterEngineLayer {
  PersistedBackdropFilter(PersistedBackdropFilter? super.oldLayer, this.filter);

  final ui.ImageFilter filter;

  /// The dedicated child container element that's separate from the
  /// [rootElement] is used to host child in front of [filterElement] that
  /// is transformed to cover background.
  @override
  DomElement? get childContainer => _childContainer;
  DomElement? _childContainer;
  DomElement? _filterElement;
  DomElement? _svgFilter;
  ui.Rect? _activeClipBounds;
  // Cached inverted transform for [transform].
  late Matrix4 _invertedTransform;
  // Reference to transform last used to cache [_invertedTransform].
  Matrix4? _previousTransform;

  @override
  void adoptElements(PersistedBackdropFilter oldSurface) {
    super.adoptElements(oldSurface);
    _childContainer = oldSurface._childContainer;
    _filterElement = oldSurface._filterElement;
    _svgFilter = oldSurface._svgFilter;
    oldSurface._childContainer = null;
  }

  @override
  DomElement createElement() {
    final DomElement element = defaultCreateElement('flt-backdrop');
    element.style.transformOrigin = '0 0 0';
    _childContainer = createDomElement('flt-backdrop-interior');
    _childContainer!.style.position = 'absolute';
    if (debugExplainSurfaceStats) {
      // This creates an additional interior element. Count it too.
      surfaceStatsFor(this).allocatedDomNodeCount++;
    }
    _filterElement = defaultCreateElement('flt-backdrop-filter');
    _filterElement!.style.transformOrigin = '0 0 0';
    element
      ..append(_filterElement!)
      ..append(_childContainer!);
    return element;
  }

  @override
  void discard() {
    super.discard();
    // Do not detach the child container from the root. It is permanently
    // attached. The elements are reused together and are detached from the DOM
    // together.
    ResourceManager.instance.removeResource(_svgFilter);
    _svgFilter = null;
    _childContainer = null;
    _filterElement = null;
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
    if (_previousTransform != transform) {
      _invertedTransform = Matrix4.inverted(transform!);
      _previousTransform = transform;
    }
    // https://api.flutter.dev/flutter/widgets/BackdropFilter-class.html
    // Defines the effective area as the parent/ancestor clip or if not
    // available, the whole screen.
    //
    // The CSS backdrop-filter will use isolation boundary defined in
    // https://drafts.fxtf.org/filter-effects-2/#BackdropFilterProperty
    // Therefore we need to use parent clip element bounds for
    // backdrop boundary.
    final double dpr = ui.window.devicePixelRatio;
    final ui.Rect rect = _invertedTransform.transformRect(
      ui.Rect.fromLTRB(
        0,
        0,
        ui.window.physicalSize.width * dpr,
        ui.window.physicalSize.height * dpr,
      ),
    );
    double left = rect.left;
    double top = rect.top;
    double width = rect.width;
    double height = rect.height;
    PersistedContainerSurface? parentSurface = parent;
    while (parentSurface != null) {
      if (parentSurface.isClipping) {
        final ui.Rect activeClipBounds = (_activeClipBounds = parentSurface.localClipBounds)!;
        left = activeClipBounds.left;
        top = activeClipBounds.top;
        width = activeClipBounds.width;
        height = activeClipBounds.height;
        break;
      }
      parentSurface = parentSurface.parent;
    }
    final DomCSSStyleDeclaration filterElementStyle = _filterElement!.style;
    filterElementStyle
      ..position = 'absolute'
      ..left = '${left}px'
      ..top = '${top}px'
      ..width = '${width}px'
      ..height = '${height}px';
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
      // For FireFox for now render transparent black background.
      // TODO(ferhat): Switch code to use filter when
      // See https://caniuse.com/#feat=css-backdrop-filter.
      filterElementStyle
        ..backgroundColor = '#000'
        ..opacity = '0.2';
    } else {
      if (backendFilter is ModeHtmlColorFilter) {
        _svgFilter = backendFilter.makeSvgFilter(_filterElement);

        /// Some blendModes do not make an svgFilter. See [EngineHtmlColorFilter.makeSvgFilter()]
        if (_svgFilter == null) {
          return;
        }
      } else if (backendFilter is MatrixHtmlColorFilter) {
        _svgFilter = backendFilter.makeSvgFilter(_filterElement);
      }

      // CSS uses pixel radius for blur. Flutter & SVG use sigma parameters. For
      // Gaussian blur with standard deviation (normal distribution),
      // the blur will fall within 2 * sigma pixels.
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
        setElementStyle(_filterElement!, '-webkit-backdrop-filter', backendFilter.filterAttribute);
      }
      setElementStyle(_filterElement!, 'backdrop-filter', backendFilter.filterAttribute);
    }
  }

  @override
  void update(PersistedBackdropFilter oldSurface) {
    super.update(oldSurface);
    if (filter != oldSurface.filter) {
      apply();
    } else {
      _checkForUpdatedAncestorClipElement();
    }
  }

  void _checkForUpdatedAncestorClipElement() {
    // If parent clip element has moved, adjust bounds.
    PersistedContainerSurface? parentSurface = parent;
    while (parentSurface != null) {
      if (parentSurface.isClipping) {
        if (parentSurface.localClipBounds != _activeClipBounds) {
          apply();
        }
        break;
      }
      parentSurface = parentSurface.parent;
    }
  }

  @override
  void retain() {
    super.retain();
    _checkForUpdatedAncestorClipElement();
  }
}
