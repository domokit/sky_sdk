// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser')
library;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/view_embedder/embedding_strategy/custom_element_embedding_strategy.dart';
import 'package:ui/src/engine/view_embedder/embedding_strategy/embedding_strategy.dart';
import 'package:ui/src/engine/view_embedder/embedding_strategy/full_page_embedding_strategy.dart';

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  group('Factory', () {
    test('Creates a FullPage instance when hostElement is null', () async {
      final EmbeddingStrategy strategy = EmbeddingStrategy.create();

      expect(strategy, isA<FullPageEmbeddingStrategy>());
    });

    test('Creates a CustomElement instance when hostElement is not null', () async {
      final DomElement element = createDomElement('some-random-element');
      final EmbeddingStrategy strategy = EmbeddingStrategy.create(hostElement: element);

      expect(strategy, isA<CustomElementEmbeddingStrategy>());
    });
  });
}
