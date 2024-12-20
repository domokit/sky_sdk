// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/util.dart';

typedef TestCacheEntry = ({String key, int value});

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('$LruCache starts out empty', () {
    final LruCache<String, int> cache = LruCache<String, int>(10);
    expect(cache.length, 0);
  });

  test('$LruCache adds up to a maximum number of items in most recently used first order', () {
    final LruCache<String, int> cache = LruCache<String, int>(3);
    cache.cache('a', 1);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[(key: 'a', value: 1)]);
    expect(cache['a'], 1);
    expect(cache['b'], isNull);

    cache.cache('b', 2);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'b', value: 2),
      (key: 'a', value: 1),
    ]);
    expect(cache['a'], 1);
    expect(cache['b'], 2);

    cache.cache('c', 3);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'c', value: 3),
      (key: 'b', value: 2),
      (key: 'a', value: 1),
    ]);

    cache.cache('d', 4);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'd', value: 4),
      (key: 'c', value: 3),
      (key: 'b', value: 2),
    ]);

    cache.cache('e', 5);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'e', value: 5),
      (key: 'd', value: 4),
      (key: 'c', value: 3),
    ]);
  });

  test('$LruCache promotes entry to most recently used position', () {
    final LruCache<String, int> cache = LruCache<String, int>(3);
    cache.cache('a', 1);
    cache.cache('b', 2);
    cache.cache('c', 3);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'c', value: 3),
      (key: 'b', value: 2),
      (key: 'a', value: 1),
    ]);

    cache.cache('b', 2);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'b', value: 2),
      (key: 'c', value: 3),
      (key: 'a', value: 1),
    ]);
  });

  test('$LruCache updates and promotes entry to most recently used position', () {
    final LruCache<String, int> cache = LruCache<String, int>(3);
    cache.cache('a', 1);
    cache.cache('b', 2);
    cache.cache('c', 3);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'c', value: 3),
      (key: 'b', value: 2),
      (key: 'a', value: 1),
    ]);
    expect(cache['b'], 2);

    cache.cache('b', 42);
    expect(cache.debugItemQueue.toList(), <TestCacheEntry>[
      (key: 'b', value: 42),
      (key: 'c', value: 3),
      (key: 'a', value: 1),
    ]);
    expect(cache['b'], 42);
  });
}
