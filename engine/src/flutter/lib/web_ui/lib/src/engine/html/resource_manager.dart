// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../view_embedder/dom_manager.dart';
import '../window.dart';

/// Manages global resources for the singleton Flutter Window in the HTML
/// renderer.
///
/// It's used for resources that are referenced by CSS, such as svg filters.
class ResourceManager {
  ResourceManager(this._domManager);

  final DomManager _domManager;

  static const String resourcesHostTagName = 'flt-svg-filters';

  static ResourceManager? _instance;
  static ResourceManager get instance {
    assert(_instance != null, 'ResourceManager has not been initialized.');
    return _instance!;
  }

  /// A child element of body outside the shadowroot that hosts
  /// global resources such svg filters and clip paths when using webkit.
  DomElement? _resourcesHost;

  /// Add an element as a global resource to be referenced by CSS.
  ///
  /// Creates a global resource host element on demand and either places it as
  /// first element of body(webkit), or as a child of the implicit view element
  /// for other browsers to make sure url resolution works correctly when
  /// content is inside a shadow root.
  void addResource(DomElement element) {
    _getOrCreateResourcesHost().append(element);
  }

  DomElement _getOrCreateResourcesHost() {
    if (_resourcesHost != null) {
      return _resourcesHost!;
    }

    final DomElement resourcesHost = domDocument.createElement(resourcesHostTagName);
    resourcesHost.style.visibility = 'hidden';
    _resourcesHost = resourcesHost;

    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
      final DomElement rootElement = _domManager.rootElement;
      // The resourcesHost *must* be a sibling of the rootElement.
      rootElement.parent!.prepend(resourcesHost);
    } else {
      final DomShadowRoot renderingHost = _domManager.renderingHost;
      renderingHost.prepend(resourcesHost);
    }
    return resourcesHost;
  }

  /// Removes a global resource element.
  void removeResource(DomElement? element) {
    if (element == null) {
      return;
    }
    assert(element.parentNode == _resourcesHost);
    element.remove();
  }
}

/// Initializes the [ResourceManager.instance] singleton if it hasn't been
/// initialized yet.
void ensureResourceManagerInitialized(EngineFlutterWindow implicitView) {
  if (ResourceManager._instance != null) {
    return;
  }
  ResourceManager._instance = ResourceManager(implicitView.dom);
}
