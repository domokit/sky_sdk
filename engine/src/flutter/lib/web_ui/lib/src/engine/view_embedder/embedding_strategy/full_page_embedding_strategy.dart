// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/util.dart' show setElementStyle;

import '../hot_restart_cache_handler.dart' show registerElementForCleanup;
import 'embedding_strategy.dart';

/// An [EmbeddingStrategy] that takes over the whole web page.
///
/// This strategy takes over the <body> element, modifies the viewport meta-tag,
/// and ensures that the root Flutter view covers the whole screen.
class FullPageEmbeddingStrategy implements EmbeddingStrategy {
  FullPageEmbeddingStrategy() {
    hostElement.setAttribute('flt-embedding', 'full-page');
    _applyViewportMeta();
    _setHostStyles();
  }

  @override
  final DomElement hostElement = domDocument.body!;

  @override
  DomEventTarget get globalEventTarget => domWindow;

  @override
  void attachViewRoot(DomElement rootElement) {
    /// Tweaks style so the rootElement works well with the hostElement.
    rootElement.style
      ..position = 'absolute'
      ..top = '0'
      ..right = '0'
      ..bottom = '0'
      ..left = '0';

    hostElement.append(rootElement);

    registerElementForCleanup(rootElement);
  }

  // Sets the global styles for a flutter app.
  void _setHostStyles() {
    setElementStyle(hostElement, 'position', 'fixed');
    setElementStyle(hostElement, 'top', '0');
    setElementStyle(hostElement, 'right', '0');
    setElementStyle(hostElement, 'bottom', '0');
    setElementStyle(hostElement, 'left', '0');
    setElementStyle(hostElement, 'overflow', 'hidden');
    setElementStyle(hostElement, 'padding', '0');
    setElementStyle(hostElement, 'margin', '0');

    setElementStyle(hostElement, 'user-select', 'none');
    setElementStyle(hostElement, '-webkit-user-select', 'none');

    // This is required to prevent the browser from doing any native touch
    // handling. If this is not done, the browser doesn't report 'pointermove'
    // events properly.
    setElementStyle(hostElement, 'touch-action', 'none');
  }

  // Sets a meta viewport tag appropriate for Flutter Web in full screen.
  void _applyViewportMeta() {
    for (final DomElement viewportMeta in domDocument.head!.querySelectorAll(
      'meta[name="viewport"]',
    )) {
      assert(() {
        // Filter out the meta tag that the engine placed on the page. This is
        // to avoid UI flicker during hot restart. Hot restart will clean up the
        // old meta tag synchronously with the first post-restart frame.
        if (!viewportMeta.hasAttribute('flt-viewport')) {
          print(
            'WARNING: found an existing <meta name="viewport"> tag. Flutter '
            'Web uses its own viewport configuration for better compatibility '
            'with Flutter. This tag will be replaced.',
          );
        }
        return true;
      }());
      viewportMeta.remove();
    }

    // The meta viewport is always removed by the for method above, so we don't
    // need to do anything else here, other than create it again.
    final DomHTMLMetaElement viewportMeta =
        createDomHTMLMetaElement()
          ..setAttribute('flt-viewport', '')
          ..name = 'viewport'
          ..content =
              'width=device-width, initial-scale=1.0, '
              'maximum-scale=1.0, user-scalable=no';

    domDocument.head!.append(viewportMeta);

    registerElementForCleanup(viewportMeta);
  }
}
