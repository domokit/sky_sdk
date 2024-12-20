// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart' show immutable;

/// A subtree of a JSON object as well as its path.
@immutable
class JsonContext<T> {
  /// Create a [JsonContext] that represents a subtree.
  const JsonContext(this.current, this.path);

  /// The content of the subtree.
  final T current;

  /// The path from the root.
  final List<String> path;

  /// Create a [JsonContext] that represents the root of a tree.
  static JsonContext<Map<String, dynamic>> root(Map<String, dynamic> root) {
    return JsonContext<Map<String, dynamic>>(root, const <String>[]);
  }
}

/// A JSON object.
typedef JsonObject = Map<String, dynamic>;

/// A JSON array.
typedef JsonArray = List<dynamic>;

String _jsonTypeErrorMessage(
  List<String> currentPath,
  String nextKey,
  Type expectedType,
  Type actualType,
) {
  return 'Unexpected value at path ${currentPath.join('.')}.$nextKey: '
      'Expects $expectedType but got $actualType.';
}

/// Returns a JSON object's specified key.
///
/// If the result is not of type `T`, throws an `ArgumentError`.
JsonContext<T> jsonGetKey<T>(JsonContext<JsonObject> context, String key) {
  final dynamic result = context.current[key];
  if (result is! T) {
    throw ArgumentError(_jsonTypeErrorMessage(context.path, key, T, result.runtimeType));
  }
  return JsonContext<T>(result, <String>[...context.path, key]);
}

/// Returns a JSON array's specified index.
///
/// If the subtree is not of type `T`, throws an `ArgumentError`.
JsonContext<T> jsonGetIndex<T>(JsonContext<JsonArray> context, int index) {
  final dynamic result = context.current[index];
  if (result is! T) {
    throw ArgumentError(_jsonTypeErrorMessage(context.path, '$index', T, result.runtimeType));
  }
  return JsonContext<T>(result, <String>[...context.path, '$index']);
}

List<dynamic> _jsonPathSplit(String path) {
  return path.split('.').map((String key) {
    final int? index = int.tryParse(key);
    if (index != null) {
      return index;
    } else {
      return key;
    }
  }).toList();
}

/// Returns the value at `path` of a JSON tree.
///
/// The path is split using `.`. Integral elements are considered as array
/// indexes, while others are considered as map indexes.
///
/// If the final result is not of type `T`, throws an `ArgumentError`.
JsonContext<T> jsonGetPath<T>(JsonContext<dynamic> context, String path) {
  JsonContext<dynamic> current = context;
  void jsonGetKeyOrIndex<M>(dynamic key, int depth) {
    assert(key is String || key is int, 'Key at $depth is a ${key.runtimeType}.');
    if (key is String) {
      current = jsonGetKey<M>(current as JsonContext<JsonObject>, key);
    } else if (key is int) {
      current = jsonGetIndex<M>(current as JsonContext<JsonArray>, key);
    } else {
      assert(false);
    }
  }

  void jsonGetKeyOrIndexForNext(dynamic key, dynamic nextKey, int depth) {
    assert(nextKey is String || nextKey is int, 'Key at ${depth + 1} is a ${key.runtimeType}.');
    if (nextKey is String) {
      jsonGetKeyOrIndex<JsonObject>(key, depth);
    } else if (nextKey is int) {
      jsonGetKeyOrIndex<JsonArray>(key, depth);
    } else {
      assert(false);
    }
  }

  final List<dynamic> pathSegments = _jsonPathSplit(path);
  for (int depth = 0; depth < pathSegments.length; depth += 1) {
    if (depth != pathSegments.length - 1) {
      jsonGetKeyOrIndexForNext(pathSegments[depth], pathSegments[depth + 1], depth);
    } else {
      jsonGetKeyOrIndex<T>(pathSegments[depth], depth);
    }
  }
  return current as JsonContext<T>;
}
