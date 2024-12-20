// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

import 'constants.dart';

/// The location of the Flutter root directory, based on the known location of
/// this script.
final Directory flutterRoot =
    Directory(path.dirname(Platform.script.toFilePath())).parent.parent.parent.parent;
String get dataRoot => testDataRoot ?? _dataRoot;
String _dataRoot = path.join(flutterRoot.path, 'dev', 'tools', 'gen_keycodes', 'data');

/// Allows overriding of the [dataRoot] for testing purposes.
@visibleForTesting
String? testDataRoot;

/// Converts `FOO_BAR` to `FooBar`.
String shoutingToUpperCamel(String shouting) {
  final RegExp initialLetter = RegExp(r'(?:_|^)([^_])([^_]*)');
  final String snake = shouting.toLowerCase();
  final String result = snake.replaceAllMapped(initialLetter, (Match match) {
    return match.group(1)!.toUpperCase() + match.group(2)!.toLowerCase();
  });
  return result;
}

/// Converts 'FooBar' to 'fooBar'.
///
/// 'TVFoo' should be convert to 'tvFoo'.
/// 'KeyX' should be convert to 'keyX'.
String upperCamelToLowerCamel(String upperCamel) {
  final RegExp initialGroup = RegExp(r'^([A-Z]([A-Z]*|[^A-Z]*))([A-Z]([^A-Z]|$)|$)');
  return upperCamel.replaceFirstMapped(initialGroup, (Match match) {
    return match.group(1)!.toLowerCase() + (match.group(3) ?? '');
  });
}

/// Converts 'fooBar' to 'FooBar'.
String lowerCamelToUpperCamel(String lowerCamel) {
  return lowerCamel.substring(0, 1).toUpperCase() + lowerCamel.substring(1);
}

/// A list of Dart reserved words.
///
/// Since these are Dart reserved words, we can't use them as-is for enum names.
const List<String> kDartReservedWords = <String>[
  'abstract',
  'as',
  'assert',
  'async',
  'await',
  'break',
  'case',
  'catch',
  'class',
  'const',
  'continue',
  'covariant',
  'default',
  'deferred',
  'do',
  'dynamic',
  'else',
  'enum',
  'export',
  'extends',
  'external',
  'factory',
  'false',
  'final',
  'finally',
  'for',
  'Function',
  'get',
  'hide',
  'if',
  'implements',
  'import',
  'in',
  'interface',
  'is',
  'library',
  'mixin',
  'new',
  'null',
  'on',
  'operator',
  'part',
  'rethrow',
  'return',
  'set',
  'show',
  'static',
  'super',
  'switch',
  'sync',
  'this',
  'throw',
  'true',
  'try',
  'typedef',
  'var',
  'void',
  'while',
  'with',
  'yield',
];

/// Converts an integer into a hex string with the given number of digits.
String toHex(int? value, {int digits = 8}) {
  if (value == null) {
    return 'null';
  }
  return '0x${value.toRadixString(16).padLeft(digits, '0')}';
}

/// Parses an integer from a hex string.
int getHex(String input) {
  return int.parse(input, radix: 16);
}

/// Given an [input] string, wraps the text at 80 characters and prepends each
/// line with the [prefix] string. Use for generated comments.
String wrapString(String input, {required String prefix}) {
  final int wrapWidth = 80 - prefix.length;
  final StringBuffer result = StringBuffer();
  final List<String> words = input.split(RegExp(r'\s+'));
  String currentLine = words.removeAt(0);
  for (final String word in words) {
    if ((currentLine.length + word.length) < wrapWidth) {
      currentLine += ' $word';
    } else {
      result.writeln('$prefix$currentLine');
      currentLine = word;
    }
  }
  if (currentLine.isNotEmpty) {
    result.writeln('$prefix$currentLine');
  }
  return result.toString();
}

/// Run `fn` with each corresponding element from list1 and list2.
///
/// If `list1` has a different length from `list2`, the execution is aborted
/// after printing an error.
///
/// An null list is considered a list with length 0.
void zipStrict<T1, T2>(Iterable<T1> list1, Iterable<T2> list2, void Function(T1, T2) fn) {
  assert(list1.length == list2.length);
  final Iterator<T1> it1 = list1.iterator;
  final Iterator<T2> it2 = list2.iterator;
  while (it1.moveNext()) {
    it2.moveNext();
    fn(it1.current, it2.current);
  }
}

/// Read a Map<String, String> out of its string representation in JSON.
Map<String, String> parseMapOfString(String jsonString) {
  return (json.decode(jsonString) as Map<String, dynamic>).cast<String, String>();
}

/// Read a Map<String, List<String>> out of its string representation in JSON.
Map<String, List<String>> parseMapOfListOfString(String jsonString) {
  final Map<String, List<dynamic>> dynamicMap =
      (json.decode(jsonString) as Map<String, dynamic>).cast<String, List<dynamic>>();
  return dynamicMap.map<String, List<String>>((String key, List<dynamic> value) {
    return MapEntry<String, List<String>>(key, value.cast<String>());
  });
}

Map<String, List<String?>> parseMapOfListOfNullableString(String jsonString) {
  final Map<String, List<dynamic>> dynamicMap =
      (json.decode(jsonString) as Map<String, dynamic>).cast<String, List<dynamic>>();
  return dynamicMap.map<String, List<String?>>((String key, List<dynamic> value) {
    return MapEntry<String, List<String?>>(key, value.cast<String?>());
  });
}

Map<String, bool> parseMapOfBool(String jsonString) {
  return (json.decode(jsonString) as Map<String, dynamic>).cast<String, bool>();
}

/// Reverse the map of { fromValue -> list of toValue } to { toValue -> fromValue } and return.
Map<String, String> reverseMapOfListOfString(
  Map<String, List<String>> inMap,
  void Function(String fromValue, String newToValue) onDuplicate,
) {
  final Map<String, String> result = <String, String>{};
  inMap.forEach((String fromValue, List<String> toValues) {
    for (final String toValue in toValues) {
      if (result.containsKey(toValue)) {
        onDuplicate(fromValue, toValue);
        continue;
      }
      result[toValue] = fromValue;
    }
  });
  return result;
}

/// Remove entries whose value `isEmpty` or is null, and return the map.
///
/// Will modify the input map.
Map<String, dynamic> removeEmptyValues(Map<String, dynamic> map) {
  return map..removeWhere(
    (String key, dynamic value) => switch (value) {
      null => true,
      Map<String, dynamic>() => removeEmptyValues(value).isEmpty,
      Iterable<dynamic>() => value.isEmpty,
      _ => false,
    },
  );
}

void addNameValue(List<String> names, List<int> values, String name, int value) {
  final int foundIndex = values.indexOf(value);
  if (foundIndex == -1) {
    names.add(name);
    values.add(value);
  } else {
    if (!RegExp(r'(^|, )abc1($|, )').hasMatch(name)) {
      names[foundIndex] = '${names[foundIndex]}, $name';
    }
  }
}

enum DeduplicateBehavior {
  // Warn the duplicate entry.
  kWarn,

  // Skip the latter duplicate entry.
  kSkip,

  // Keep all duplicate entries.
  kKeep,
}

/// The information for a line used by [OutputLines].
class OutputLine<T extends Comparable<Object>> {
  OutputLine(this.key, String value) : values = <String>[value];

  final T key;
  final List<String> values;
}

/// A utility class to build join a number of lines in a sorted order.
///
/// Use [add] to add a line and associate it with an index. Use [sortedJoin] to
/// get the joined string of these lines joined sorting them in the order of the
/// index.
class OutputLines<T extends Comparable<Object>> {
  OutputLines(this.mapName, {this.behavior = DeduplicateBehavior.kWarn});

  /// What to do if there are entries with the same key.
  final DeduplicateBehavior behavior;

  /// The name for this map.
  ///
  /// Used in warning messages.
  final String mapName;

  final Map<T, OutputLine<T>> lines = <T, OutputLine<T>>{};

  void add(T key, String line) {
    final OutputLine<T>? existing = lines[key];
    if (existing != null) {
      switch (behavior) {
        case DeduplicateBehavior.kWarn:
          print(
            'Warn: Request to add $key to map "$mapName" as:\n    $line\n  but it already exists as:\n    ${existing.values[0]}',
          );
          return;
        case DeduplicateBehavior.kSkip:
          return;
        case DeduplicateBehavior.kKeep:
          existing.values.add(line);
          return;
      }
    }
    lines[key] = OutputLine<T>(key, line);
  }

  String join() {
    return lines.values.map((OutputLine<T> line) => line.values.join('\n')).join('\n');
  }

  String sortedJoin() {
    return (lines.values.toList()
          ..sort((OutputLine<T> a, OutputLine<T> b) => a.key.compareTo(b.key)))
        .map((OutputLine<T> line) => line.values.join('\n'))
        .join('\n');
  }
}

int toPlane(int value, int plane) {
  return (value & kValueMask.value) + (plane & kPlaneMask.value);
}

int getPlane(int value) {
  return value & kPlaneMask.value;
}
