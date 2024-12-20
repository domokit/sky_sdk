// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser')
library;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/view_embedder/embedding_strategy/full_page_embedding_strategy.dart';

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  group('initialize', () {
    test('Prepares target environment', () {
      final DomElement target = domDocument.body!;
      final DomHTMLMetaElement meta = createDomHTMLMetaElement();
      meta
        ..id = 'my_viewport_meta_for_testing'
        ..name = 'viewport'
        ..content =
            'width=device-width, initial-scale=1.0, '
            'maximum-scale=1.0, user-scalable=no';
      domDocument.head!.append(meta);

      DomElement? userMeta = domDocument.querySelector('#my_viewport_meta_for_testing');

      expect(userMeta, isNotNull);

      // ignore: unused_local_variable
      final FullPageEmbeddingStrategy strategy = FullPageEmbeddingStrategy();

      expect(
        target.getAttribute('flt-embedding'),
        'full-page',
        reason: 'Should identify itself as a specific key=value into the target element.',
      );

      // Locate the viewport metas again...
      userMeta = domDocument.querySelector('#my_viewport_meta_for_testing');

      final DomElement? flutterMeta = domDocument.querySelector('meta[name="viewport"]');

      expect(userMeta, isNull, reason: 'Should delete previously existing viewport meta tags.');
      expect(flutterMeta, isNotNull);
      expect(
        flutterMeta!.hasAttribute('flt-viewport'),
        isTrue,
        reason: 'Should install flutter viewport meta tag.',
      );
    });
  });

  group('attachViewRoot', () {
    test('Should attach glasspane into embedder target (body)', () async {
      final FullPageEmbeddingStrategy strategy = FullPageEmbeddingStrategy();

      final DomElement glassPane = createDomElement('some-tag-for-tests');
      final DomCSSStyleDeclaration style = glassPane.style;

      expect(glassPane.isConnected, isFalse);
      expect(style.position, '', reason: 'Should not have any specific position.');
      expect(style.top, '', reason: 'Should not have any top/right/bottom/left positioning/inset.');

      strategy.attachViewRoot(glassPane);

      // Assert injection into <body>
      expect(glassPane.isConnected, isTrue, reason: 'Should inject glassPane into the document.');
      expect(glassPane.parent, domDocument.body, reason: 'Should inject glassPane into the <body>');

      final DomCSSStyleDeclaration styleAfter = glassPane.style;

      // Assert required styling to cover the viewport
      expect(styleAfter.position, 'absolute', reason: 'Should be absolutely positioned.');
      expect(styleAfter.top, '0px', reason: 'Should cover the whole viewport.');
      expect(styleAfter.right, '0px', reason: 'Should cover the whole viewport.');
      expect(styleAfter.bottom, '0px', reason: 'Should cover the whole viewport.');
      expect(styleAfter.left, '0px', reason: 'Should cover the whole viewport.');
    });
  });
}
