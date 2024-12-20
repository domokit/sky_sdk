// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser')
library;

import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/view_embedder/hot_restart_cache_handler.dart';

@JS('window.__flutterState')
external JSArray<JSAny?>? get _jsHotRestartStore;

@JS('window.__flutterState')
external set _jsHotRestartStore(JSArray<JSAny?>? nodes);

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  tearDown(() {
    _jsHotRestartStore = null;
  });

  group('registerElementForCleanup', () {
    test('stores elements in a global cache', () async {
      final DomElement toBeCached = createDomElement('some-element-to-cache');
      final DomElement other = createDomElement('other-element-to-cache');
      final DomElement another = createDomElement('another-element-to-cache');

      registerElementForCleanup(toBeCached);
      registerElementForCleanup(other);
      registerElementForCleanup(another);

      expect(_jsHotRestartStore!.toDart, <DomElement>[toBeCached, other, another]);
    });
  });

  group('HotRestartCacheHandler Constructor', () {
    test('Creates a cache in the JS environment', () async {
      HotRestartCacheHandler();

      expect(_jsHotRestartStore, isNotNull);
      expect(_jsHotRestartStore!.length, 0);
    });
  });

  group('HotRestartCacheHandler.registerElement', () {
    late HotRestartCacheHandler cache;

    setUp(() {
      cache = HotRestartCacheHandler();
    });

    test('Registers an element in the DOM cache', () async {
      final DomElement element = createDomElement('for-test');
      cache.registerElement(element);

      expect(_jsHotRestartStore!.toDart, <DomElement>[element]);
    });

    test('Registers elements in the DOM cache', () async {
      final DomElement element = createDomElement('for-test');
      domDocument.body!.append(element);

      cache.registerElement(element);

      expect(_jsHotRestartStore!.toDart, <DomElement>[element]);
    });

    test('Clears registered elements from the DOM and the cache upon restart', () async {
      final DomElement element = createDomElement('for-test');
      final DomElement element2 = createDomElement('for-test-two');
      domDocument.body!.append(element);
      domDocument.body!.append(element2);

      cache.registerElement(element);

      expect(element.isConnected, isTrue);
      expect(element2.isConnected, isTrue);

      // Simulate a hot restart...
      cache = HotRestartCacheHandler();

      // For dart2wasm, we have to check the length this way.
      expect(_jsHotRestartStore!.length, 0);
      expect(element.isConnected, isFalse); // Removed
      expect(element2.isConnected, isTrue);
    });
  });
}
