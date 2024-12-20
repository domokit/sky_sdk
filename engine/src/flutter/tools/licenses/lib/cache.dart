// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

Map<Key, dynamic> _cache = <Key, dynamic>{};
const int _maxSize = 10;

T cache<T>(Key key, T Function() getter) {
  T result;
  if (_cache[key] != null) {
    result = _cache[key] as T;
    _cache.remove(key);
  } else {
    if (_cache.length == _maxSize) {
      _cache.remove(_cache.keys.first);
    }
    result = getter();
    assert(result is! Function);
  }
  _cache[key] = result;
  return result;
}

abstract class Key {
  Key(this._value);

  final dynamic _value;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is Key && other._value == _value;
  }

  @override
  int get hashCode => runtimeType.hashCode ^ _value.hashCode;

  @override
  String toString() => '$runtimeType($_value)';
}
