// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show JsonEncoder, jsonDecode;

import 'package:meta/meta.dart';

/// A JSON object that contains key-value pairs of other JSON values.
///
/// This extension type provides type-safe access to the values in the
/// underlying JSON object without the need for manual type casting and with
/// helpful APIs for common operations.
///
/// ## Examples
///
/// ### Basic Usage
///
/// ```dart
/// const json = JsonObject({
///  'name': 'John Doe',
///  'age': 30,
///  'isEmployed': true,
///  'pets': ['Fido', 'Whiskers'],
/// });
///
/// final name = json.string('name');
/// final age = json.integer('age');
/// final isEmployed = json.boolean('isEmployed');
/// final pets = json.stringList('pets');
/// ```
///
/// ### Null Safety
///
/// In the above example, we assumed that all keys exist in the JSON object. To
/// gracefully handle cases where a key may not exist, you can use the `OrNull`
/// variants of the methods:
///
/// ```dart
/// final name = json.stringOrNull('name');
/// ```
///
/// This works nicely with null-aware operators:
///
/// ```dart
/// final name = json.stringOrNull('name') ?? 'Unknown';
/// ```
///
/// ### Bulk Operations
///
/// By default, a [JsonReadException] is thrown if a key is not found or if the
/// value cannot be cast to the expected type, which can be handled using a
/// `try-catch` block:
///
/// ```dart
/// try {
///   final name = json.string('name');
/// } on FormatException catch (e) {
///   print('Error: $e');
/// }
/// ```
///
/// However, it is cumbersome to wrap every access in a `try-catch` block, and
/// sometimes you will want to collect all errors at once. For this, you can use
/// [map] to read multiple values at once and convert them to a custom object:
///
/// ```dart
/// try {
///   final person = json.map(
///     (json) => Person(
///       name: json.string('name'),
///       age: json.integer('age'),
///       isEmployed: json.boolean('isEmployed'),
///       pets: json.stringList('pets'),
///     ),
///   );
/// } on JsonMapException catch (e) {
///   print('Errors: ${e.exceptions}');
/// }
/// ```
///
/// Or, provide an `onError` handler to handle the exception (either to log and
/// provide a default value, or to rethrow a different exception):
///
/// ```dart
/// final person = json.map(
///   (json) => Person(name: json.string('name')),
///   onError: (json, e) => throw FormatException('Failed to read person: $e'),
/// );
/// ```
extension type const JsonObject(Map<String, Object?> _object) {
  /// Parses a JSON string into a [JsonObject].
  static JsonObject parse(String jsonString) {
    return JsonObject(jsonDecode(jsonString) as Map<String, Object?>);
  }

  // If non-null, errors caught during [map] are stored here.
  static List<JsonReadException>? _mapErrors;

  // If in the context of a [map] operation, collects exceptions.
  // Otherwise, throws the exception.
  static void _error(JsonReadException e) {
    if (_mapErrors case final List<JsonReadException> list) {
      list.add(e);
    } else {
      throw e;
    }
  }

  T _get<T extends Object>(String key, T defaultTo) {
    final T? result = _getOrNull<T>(key);
    if (result == null) {
      if (!_object.containsKey(key)) {
        _error(MissingKeyJsonReadException(this, key));
      }
      return defaultTo;
    }
    return result;
  }

  /// Returns the value at the given [key] as a [String].
  ///
  /// Throws a [JsonReadException] if the value is not found or cannot be cast.
  String string(String key) => _get<String>(key, '');

  /// Returns the value at the given [key] as an [int].
  ///
  /// Throws a [JsonReadException] if the value is not found or cannot be cast.
  int integer(String key) => _get<int>(key, 0);

  /// Returns the value at the given [key] as a [bool].
  ///
  /// Throws a [JsonReadException] if the value is not found or cannot be cast.
  bool boolean(String key) => _get<bool>(key, false);

  List<T> _getList<T extends Object>(String key) {
    final List<T>? result = _getListOrNull<T>(key);
    if (result == null) {
      _error(MissingKeyJsonReadException(this, key));
      return <T>[];
    }
    return result;
  }

  /// Returns the value at the given [key] as a [List] of [String]s.
  ///
  /// Throws a [JsonReadException] if the value is not found or cannot be cast.
  List<String> stringList(String key) => _getList<String>(key);

  T? _getOrNull<T extends Object>(String key) {
    final Object? value = _object[key];
    if (value == null && !_object.containsKey(key)) {
      return null;
    } else if (value is! T) {
      _error(InvalidTypeJsonReadException(this, key, expected: T, actual: value.runtimeType));
      return null;
    } else {
      return value;
    }
  }

  /// Returns the value at the given [key] as a [String].
  ///
  /// If the value is not found, returns `null`.
  ///
  /// Throws a [JsonReadException] if the value cannot be cast.
  String? stringOrNull(String key) => _getOrNull<String>(key);

  /// Returns the value at the given [key] as an [int].
  ///
  /// If the value is not found, returns `null`.
  ///
  /// Throws a [JsonReadException] if the value cannot be cast.
  int? integerOrNull(String key) => _getOrNull<int>(key);

  /// Returns the value at the given [key] as a [bool].
  ///
  /// If the value is not found, returns `null`.
  ///
  /// Throws a [JsonReadException] if the value cannot be cast.
  bool? booleanOrNull(String key) => _getOrNull<bool>(key);

  List<T>? _getListOrNull<T extends Object>(String key) {
    final Object? value = _object[key];
    if (value == null && !_object.containsKey(key)) {
      return null;
    } else if (value is! List<Object?>) {
      _error(InvalidTypeJsonReadException(this, key, expected: List<T>, actual: value.runtimeType));
      return <T>[];
    } else {
      final List<T> result = <T>[];
      for (final Object? element in value) {
        if (element is! T) {
          _error(InvalidTypeJsonReadException(this, key, expected: T, actual: element.runtimeType));
          return <T>[];
        }
        result.add(element);
      }
      return result;
    }
  }

  /// Returns the value at the given [key] as a [List] of [String]s.
  ///
  /// If the value is not found, returns `null`.
  ///
  /// Throws a [JsonReadException] if the value cannot be cast.
  List<String>? stringListOrNull(String key) => _getListOrNull<String>(key);

  static Never _onErrorThrow(JsonObject object, JsonMapException e) {
    throw e;
  }

  JsonObject? _getObjectOrNull(String key) {
    final Object? value = _object[key];
    if (value == null && !_object.containsKey(key)) {
      return null;
    } else if (value is! Map<String, Object?>) {
      _error(
        InvalidTypeJsonReadException(
          this,
          key,
          expected: Map<String, Object?>,
          actual: value.runtimeType,
        ),
      );
      return const JsonObject({});
    } else {
      return JsonObject(value);
    }
  }

  JsonObject _getObject(String key) {
    final JsonObject? result = _getObjectOrNull(key);
    if (result == null) {
      _error(MissingKeyJsonReadException(this, key));
      return const JsonObject({});
    }
    return result;
  }

  /// Returns the value at the given [key] as a [JsonObject].
  ///
  /// Throws a [JsonReadException] if the value is not found or cannot be cast.
  JsonObject object(String key) => _getObject(key);

  /// Returns the value at the given [key] as a [JsonObject].
  ///
  /// If the value is not found, returns `null`.
  ///
  /// Throws a [JsonReadException] if the value cannot be cast.
  JsonObject? objectOrNull(String key) => _getObjectOrNull(key);

  /// Returns the result of applying the given [mapper] to this JSON object.
  ///
  /// Any exception that otherwise would be thrown by the mapper is caught
  /// internally and rethrown as a [JsonMapException] with all of the original
  /// exceptions added to its [exceptions] property.
  ///
  /// If [onError] is provided, it is called with this object and the caught
  /// [JsonMapException] to handle the error. If not provided, the exception is
  /// rethrown.
  ///
  /// **Note:** The [mapper] function _must_ be synchronous.
  ///
  /// ## Example
  ///
  /// ```dart
  /// final person = json.map(
  ///   (json) => Person(
  ///     name: json.string('name'),
  ///     age: json.integer('age'),
  ///     isEmployed: json.boolean('isEmployed'),
  ///   ), onError: (json, e) {
  ///     throw FormatException('Failed to read person: $e');
  ///   },
  /// );
  /// ```
  T map<T extends Object>(
    T Function(JsonObject) mapper, {
    T Function(JsonObject, JsonMapException) onError = _onErrorThrow,
  }) {
    // Refuse asynchronous mappers to avoid reentrancy issues.
    //
    // Could be replaced with a static check in the future:
    // https://github.com/dart-lang/sdk/issues/35024
    if (mapper is Future<void> Function(JsonObject)) {
      throw ArgumentError.value(mapper, 'mapper', 'must be synchronous');
    }

    // Store the previous errors, if any.
    final List<JsonReadException>? previousErrors = _mapErrors;

    // Start collecting errors for this map operation.
    final List<JsonReadException> currentErrors = <JsonReadException>[];
    _mapErrors = currentErrors;

    try {
      final T result = mapper(this);
      if (currentErrors.isEmpty) {
        return result;
      } else {
        return onError(this, JsonMapException(this, currentErrors));
      }
    } finally {
      // Restore the previous errors.
      _mapErrors = previousErrors;
    }
  }

  /// Returns the underlying JSON object as a [Map].
  ///
  /// This method is useful for passing the JSON object to APIs that expect a
  /// [Map] type, but it is generally recommended to use the provided methods
  /// for type-safe access to the JSON values.
  Map<String, Object?> asMap() => _object;

  /// Returns a "pretty" JSON representation of this object.
  String toPrettyString() {
    return const JsonEncoder.withIndent('  ').convert(_object);
  }
}

/// An exception thrown when a JSON value cannot be read as the expected type.
sealed class JsonReadException implements Exception {
  const JsonReadException(this.object);

  /// The JSON object that was being read.
  final JsonObject object;

  @override
  @mustBeOverridden
  String toString();
}

/// An exception thrown when read exceptions occur during [JsonObject.map].
final class JsonMapException extends JsonReadException {
  /// Creates an exception for multiple JSON read exceptions.
  ///
  /// If [exceptions] is empty, a [StateError] is thrown.
  JsonMapException(super.object, Iterable<JsonReadException> exceptions)
    : exceptions = List<JsonReadException>.unmodifiable(exceptions) {
    if (exceptions.isEmpty) {
      throw StateError('Must provide at least one exception.');
    }
  }

  /// The list of exceptions that occurred while mapping the JSON object.
  final List<JsonReadException> exceptions;

  @override
  String toString() {
    return 'Failed to read object:\n\n${exceptions.join('\n')}';
  }
}

/// An exception thrown when a JSON [key] is not found in the JSON object.
final class MissingKeyJsonReadException extends JsonReadException {
  /// Creates an exception for a missing key in a JSON object.
  const MissingKeyJsonReadException(super.object, this.key);

  /// The key that was being read.
  final String key;

  @override
  String toString() => 'Key "$key" not found.';
}

/// An exception thrown when a JSON [key] is found but the value cannot be cast.
final class InvalidTypeJsonReadException extends JsonReadException {
  /// Creates an exception for an invalid type in a JSON object.
  const InvalidTypeJsonReadException(
    super.object,
    this.key, {
    required this.expected,
    required this.actual,
  });

  /// The key that was being read.
  final String key;

  /// The expected type of the value.
  final Type expected;

  /// The actual type of the value.
  final Type actual;

  @override
  String toString() {
    return 'Key "$key" expected type: $expected, actual type: $actual.';
  }
}
