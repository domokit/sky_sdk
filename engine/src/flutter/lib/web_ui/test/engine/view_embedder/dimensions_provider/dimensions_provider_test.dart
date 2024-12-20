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
  group('Factory', () {
    test('Creates a FullPage instance when hostElement is null', () async {
      final DimensionsProvider provider = DimensionsProvider.create();

      expect(provider, isA<FullPageDimensionsProvider>());
    });

    test('Creates a CustomElement instance when hostElement is not null', () async {
      final DomElement element = createDomElement('some-random-element');
      final DimensionsProvider provider = DimensionsProvider.create(hostElement: element);

      expect(provider, isA<CustomElementDimensionsProvider>());
    });
  });
}
