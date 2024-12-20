// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Indexes a list of `enum` values by simple name.
///
/// In Dart enum names are prefixed with enum class name. For example, for
/// `enum Vote { yea, nay }`, `Vote.yea.toString()` produces `"Vote.yea"`
/// rather than just `"yea"` - the simple name. This class provides methods for
/// getting and looking up by simple names.
///
/// Example:
///
///     enum Vote { yea, nay }
///     final index = EnumIndex(Vote.values);
///     index.lookupBySimpleName('yea'); // returns Vote.yea
///     index.toSimpleName(Vote.nay); // returns 'nay'
class EnumIndex<E> {
  /// Creates an index of [enumValues].
  EnumIndex(List<E> enumValues)
    : _nameToValue = Map<String, E>.fromIterable(enumValues, key: _getSimpleName),
      _valueToName = Map<E, String>.fromIterable(enumValues, value: _getSimpleName);

  final Map<String, E> _nameToValue;
  final Map<E, String> _valueToName;

  /// Given a [simpleName] finds the corresponding enum value.
  E lookupBySimpleName(String simpleName) => _nameToValue[simpleName]!;

  /// Returns the simple name for [enumValue].
  String toSimpleName(E enumValue) => _valueToName[enumValue]!;
}

String _getSimpleName(dynamic enumValue) {
  return enumValue.toString().split('.').last;
}
