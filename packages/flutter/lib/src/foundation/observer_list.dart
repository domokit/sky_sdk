// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';

/// A list optimized for the observer pattern when there are small numbers of
/// observers.
///
/// Consider using an [ObserverList] instead of a [List] when the number of
/// [contains] calls dominates the number of [add] and [remove] calls.
///
/// This class will include in the [iterator] each added item in the order it
/// was added, as many times as it was added.
///
/// If there will be a large number of observers, consider using
/// [HashedObserverList] instead. It has slightly different iteration semantics,
/// but serves a similar purpose, while being more efficient for large numbers
/// of observers.
///
/// See also:
///
///  * [HashedObserverList] for a list that is optimized for larger numbers of
///    observers.
// TODO(ianh): Use DelegatingIterable, possibly moving it from the collection
// package to foundation, or to dart:collection.
class ObserverList<T> extends Iterable<T> {
  final List<T> _list = <T>[];
  bool _isDirty = false;
  late final HashSet<T> _set = HashSet<T>();

  /// Adds an item to the end of this list.
  ///
  /// This operation has constant time complexity.
  void add(T item) {
    _isDirty = true;
    _list.add(item);
  }

  /// Removes an item from the list.
  ///
  /// This is O(N) in the number of items in the list.
  ///
  /// Returns whether the item was present in the list.
  bool remove(T item) {
    final bool removed = _list.remove(item);
    if (removed) {
      _isDirty = true;
      _set.clear(); // Clear the set so that we don't leak items.
    }
    return removed;
  }

  /// Removes all items from the [ObserverList].
  void clear() {
    _isDirty = false;
    _list.clear();
    _set.clear();
  }

  @override
  bool contains(Object? element) {
    if (_list.length < 3) {
      return _list.contains(element);
    }

    if (_isDirty) {
      _set.addAll(_list);
      _isDirty = false;
    }

    return _set.contains(element);
  }

  @override
  Iterator<T> get iterator => _list.iterator;

  @override
  bool get isEmpty => _list.isEmpty;

  @override
  bool get isNotEmpty => _list.isNotEmpty;

  /// Creates a List containing the elements of the [ObserverList].
  ///
  /// Overrides the default implementation of the [Iterable] to reduce number
  /// of allocations.
  @override
  List<T> toList({bool growable = true}) {
    return _list.toList(growable: growable);
  }
}

/// A list optimized for the observer pattern, but for larger numbers of observers.
///
/// For small numbers of observers (e.g. less than 10), use [ObserverList] instead.
///
/// The iteration semantics of the this class are slightly different from
/// [ObserverList]. This class will only return an item once in the [iterator],
/// no matter how many times it was added, although it does require that an item
/// be removed as many times as it was added for it to stop appearing in the
/// [iterator]. It will return them in the order the first instance of an item
/// was originally added.
///
/// See also:
///
///  * [ObserverList] for a list that is fast for small numbers of observers.
class HashedObserverList<T> extends Iterable<T> {
  final LinkedHashMap<T, int> _map = LinkedHashMap<T, int>();

  /// Adds an item to the end of this list.
  ///
  /// This has constant time complexity.
  void add(T item) {
    _map[item] = (_map[item] ?? 0) + 1;
  }

  /// Removes an item from the list.
  ///
  /// This operation has constant time complexity.
  ///
  /// Returns whether the item was present in the list.
  bool remove(T item) {
    final int? value = _map[item];
    if (value == null) {
      return false;
    }
    if (value == 1) {
      _map.remove(item);
    } else {
      _map[item] = value - 1;
    }
    return true;
  }

  /// Removes all items from the [HashedObserverList].
  void clear() => _map.clear();

  @override
  bool contains(Object? element) => _map.containsKey(element);

  @override
  Iterator<T> get iterator => _map.keys.iterator;

  @override
  bool get isEmpty => _map.isEmpty;

  @override
  bool get isNotEmpty => _map.isNotEmpty;

  /// Creates a List containing the elements of the [HashedObserverList].
  ///
  /// Overrides the default implementation of [Iterable] to reduce number of
  /// allocations.
  @override
  List<T> toList({bool growable = true}) {
    final Iterator<T> iterator = _map.keys.iterator;
    return List<T>.generate(_map.length, (_) => (iterator..moveNext()).current, growable: growable);
  }
}
