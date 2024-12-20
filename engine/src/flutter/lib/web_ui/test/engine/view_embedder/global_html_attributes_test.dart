// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  group('GlobalHtmlAttributes', () {
    test('applies global attributes to the root and host elements', () {
      final DomElement hostElement = createDomElement('host-element');
      final DomElement rootElement = createDomElement('root-element');
      final GlobalHtmlAttributes globalHtmlAttributes = GlobalHtmlAttributes(
        rootElement: rootElement,
        hostElement: hostElement,
      );

      globalHtmlAttributes.applyAttributes(
        viewId: 123,
        autoDetectRenderer: true,
        rendererTag: 'canvaskit',
        buildMode: 'release',
      );

      expect(rootElement.getAttribute('flt-view-id'), '123');
      expect(hostElement.getAttribute('flt-renderer'), 'canvaskit (auto-selected)');
      expect(hostElement.getAttribute('flt-build-mode'), 'release');
      expect(hostElement.getAttribute('spellcheck'), 'false');

      globalHtmlAttributes.applyAttributes(
        viewId: 456,
        autoDetectRenderer: false,
        rendererTag: 'html',
        buildMode: 'debug',
      );

      expect(rootElement.getAttribute('flt-view-id'), '456');
      expect(hostElement.getAttribute('flt-renderer'), 'html (requested explicitly)');
      expect(hostElement.getAttribute('flt-build-mode'), 'debug');
      expect(hostElement.getAttribute('spellcheck'), 'false');
    });
  });
}
