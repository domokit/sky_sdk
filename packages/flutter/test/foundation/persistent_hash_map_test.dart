// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

@immutable
class _MockKey {
  const _MockKey({required this.hashCode, required this.payload});

  @override
  final int hashCode;
  final String payload;

  @override
  bool operator ==(Object other) {
    return other is _MockKey && other.payload == payload;
  }
}

void main() {
  test('PersistentHashMap - Simple Test', () {
    final List<PersistentHashMap<String, int>> maps = <PersistentHashMap<String, int>>[];
    maps.add(const PersistentHashMap<String, int>.empty());
    for (int i = 0; i < 50; i++) {
      maps.add(maps.last.put('key:$i', i));
    }
    for (int i = 1; i < maps.length; i++) {
      final PersistentHashMap<String, int> m = maps[i];
      for (int j = 0; j < i; j++) {
        expect(m['key:$j'], equals(j));
      }
    }
  });

  test('PersistentHashMap - hash collisions', () {
    const _MockKey key1 = _MockKey(hashCode: 1, payload: 'key:1');
    const _MockKey key2 = _MockKey(hashCode: 0 | (1 << 5), payload: 'key:2');
    const _MockKey key3 = _MockKey(hashCode: 1, payload: 'key:3');
    const _MockKey key4 = _MockKey(hashCode: 1 | (1 << 5), payload: 'key:4');

    final PersistentHashMap<_MockKey, String> map =
        const PersistentHashMap<_MockKey, String>.empty()
            .put(key1, 'a')
            .put(key2, 'b')
            .put(key3, 'c');

    expect(map[key1], equals('a'));
    expect(map[key2], equals('b'));
    expect(map[key3], equals('c'));

    final PersistentHashMap<_MockKey, String> map2 = map.put(key4, 'd');
    expect(map2[key4], equals('d'));

    final PersistentHashMap<_MockKey, String> map3 = map2
        .put(key1, 'updated(a)')
        .put(key2, 'updated(b)')
        .put(key3, 'updated(c)')
        .put(key4, 'updated(d)');
    expect(map3[key1], equals('updated(a)'));
    expect(map3[key2], equals('updated(b)'));
    expect(map3[key3], equals('updated(c)'));
    expect(map3[key4], equals('updated(d)'));
  });

  test('PersistentHashMap - inflation of nodes', () {
    final List<PersistentHashMap<_MockKey, int>> maps = <PersistentHashMap<_MockKey, int>>[];
    maps.add(const PersistentHashMap<_MockKey, int>.empty());
    for (int i = 0; i < 32 * 32; i++) {
      maps.add(maps.last.put(_MockKey(hashCode: i, payload: '$i'), i));
    }
    for (int i = 1; i < maps.length; i++) {
      final PersistentHashMap<_MockKey, int> m = maps[i];
      for (int j = 0; j < i; j++) {
        expect(m[_MockKey(hashCode: j, payload: '$j')], equals(j));
      }
    }
  });
}
