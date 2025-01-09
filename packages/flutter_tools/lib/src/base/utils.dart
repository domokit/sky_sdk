// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:file/file.dart';
import 'package:intl/intl.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path; // flutter_ignore: package_path_import

import '../convert.dart';
import 'platform.dart';

/// A path jointer for URL paths.
final path.Context urlContext = path.url;

/// Convert `foo_bar` to `fooBar`.
String camelCase(String str) {
  int index = str.indexOf('_');
  while (index != -1 && index < str.length - 2) {
    str =
        str.substring(0, index) +
        str.substring(index + 1, index + 2).toUpperCase() +
        str.substring(index + 2);
    index = str.indexOf('_');
  }
  return str;
}

/// Convert `fooBar` to `foo-bar`.
String kebabCase(String str) {
  return _reCase(str, '-');
}

final RegExp _upperRegex = RegExp(r'[A-Z]');

/// Convert `fooBar` to `foo_bar`.
String snakeCase(String str) {
  return _reCase(str, '_');
}

/// Convert `fooBar` to `foo[sep]bar`.
String _reCase(String str, String sep) {
  return str.replaceAllMapped(
    _upperRegex,
    (Match m) => '${m.start == 0 ? '' : sep}${m[0]!.toLowerCase()}',
  );
}

abstract interface class CliEnum implements Enum {
  String get cliName;
  String get helpText;

  static Map<String, String> allowedHelp<T extends CliEnum>(List<T> values) =>
      Map<String, String>.fromEntries(
        values.map((T e) => MapEntry<String, String>(e.cliName, e.helpText)),
      );
}

/// Converts `fooBar` to `FooBar`.
///
/// This uses [toBeginningOfSentenceCase](https://pub.dev/documentation/intl/latest/intl/toBeginningOfSentenceCase.html),
/// with the input and return value of non-nullable.
String sentenceCase(String str, [String? locale]) {
  if (str.isEmpty) {
    return str;
  }
  // TODO(christopherfujino): Remove this check after the next release of intl
  return ArgumentError.checkNotNull(toBeginningOfSentenceCase(str, locale));
}

/// Converts `foo_bar` to `Foo Bar`.
String snakeCaseToTitleCase(String snakeCaseString) {
  return snakeCaseString.split('_').map(camelCase).map(sentenceCase).join(' ');
}

/// Return the plural of the given word (`cat(s)`).
String pluralize(String word, int count) => count == 1 ? word : '${word}s';

String toPrettyJson(Object jsonable) {
  final String value = const JsonEncoder.withIndent('  ').convert(jsonable);
  return '$value\n';
}

final NumberFormat kSecondsFormat = NumberFormat('0.0');
final NumberFormat kMillisecondsFormat = NumberFormat.decimalPattern();

String getElapsedAsSeconds(Duration duration) {
  final double seconds = duration.inMilliseconds / Duration.millisecondsPerSecond;
  return '${kSecondsFormat.format(seconds)}s';
}

String getElapsedAsMilliseconds(Duration duration) {
  return '${kMillisecondsFormat.format(duration.inMilliseconds)}ms';
}

/// Return a platform-appropriate [String] representing the size of the given number of bytes.
String getSizeAsPlatformMB(
  int bytesLength, {
  @visibleForTesting Platform platform = const LocalPlatform(),
}) {
  // Because Windows displays 'MB' but actually reports MiB, we calculate MiB
  // accordingly on Windows.
  final int bytesInPlatformMB = platform.isWindows ? 1024 * 1024 : 1000 * 1000;
  return '${(bytesLength / bytesInPlatformMB).toStringAsFixed(1)}MB';
}

/// A class to maintain a list of items, fire events when items are added or
/// removed, and calculate a diff of changes when a new list of items is
/// available.
class ItemListNotifier<T> {
  ItemListNotifier() : _items = <T>{}, _isPopulated = false;

  ItemListNotifier.from(List<T> items) : _items = Set<T>.of(items), _isPopulated = true;

  Set<T> _items;

  final StreamController<T> _addedController = StreamController<T>.broadcast();
  final StreamController<T> _removedController = StreamController<T>.broadcast();

  Stream<T> get onAdded => _addedController.stream;
  Stream<T> get onRemoved => _removedController.stream;

  List<T> get items => _items.toList();

  bool _isPopulated;

  /// Returns whether the list has been populated.
  bool get isPopulated => _isPopulated;

  void updateWithNewList(List<T> updatedList) {
    final Set<T> updatedSet = Set<T>.of(updatedList);

    final Set<T> addedItems = updatedSet.difference(_items);
    final Set<T> removedItems = _items.difference(updatedSet);

    _items = updatedSet;
    _isPopulated = true;

    removedItems.forEach(_removedController.add);
    addedItems.forEach(_addedController.add);
  }

  void removeItem(T item) {
    if (_items.remove(item)) {
      _removedController.add(item);
    }
  }

  /// Close the streams.
  void dispose() {
    _addedController.close();
    _removedController.close();
  }
}

class SettingsFile {
  SettingsFile();

  SettingsFile.parse(String contents) {
    for (String line in contents.split('\n')) {
      line = line.trim();
      if (line.startsWith('#') || line.isEmpty) {
        continue;
      }
      final int index = line.indexOf('=');
      if (index != -1) {
        values[line.substring(0, index)] = line.substring(index + 1);
      }
    }
  }

  SettingsFile.parseFromFile(File file) : this.parse(file.readAsStringSync());

  final Map<String, String> values = <String, String>{};

  void writeContents(File file) {
    file.parent.createSync(recursive: true);
    file.writeAsStringSync(
      values.keys
          .map<String>((String key) {
            return '$key=${values[key]}';
          })
          .join('\n'),
    );
  }
}

/// Given a data structure which is a Map of String to dynamic values, return
/// the same structure (`Map<String, dynamic>`) with the correct runtime types.
Map<String, Object?>? castStringKeyedMap(Object? untyped) {
  final Map<dynamic, dynamic>? map = untyped as Map<dynamic, dynamic>?;
  return map?.cast<String, Object?>();
}

/// Smallest column that will be used for text wrapping. If the requested column
/// width is smaller than this, then this is what will be used.
const int kMinColumnWidth = 10;

/// Wraps a block of text into lines no longer than [columnWidth].
///
/// Tries to split at whitespace, but if that's not good enough to keep it under
/// the limit, then it splits in the middle of a word. If [columnWidth] (minus
/// any indent) is smaller than [kMinColumnWidth], the text is wrapped at that
/// [kMinColumnWidth] instead.
///
/// Preserves indentation (leading whitespace) for each line (delimited by '\n')
/// in the input, and will indent wrapped lines that same amount, adding
/// [indent] spaces in addition to any existing indent.
///
/// If [hangingIndent] is supplied, then that many additional spaces will be
/// added to each line, except for the first line. The [hangingIndent] is added
/// to the specified [indent], if any. This is useful for wrapping
/// text with a heading prefix (e.g. "Usage: "):
///
/// ```dart
/// String prefix = "Usage: ";
/// print(prefix + wrapText(invocation, indent: 2, hangingIndent: prefix.length, columnWidth: 40));
/// ```
///
/// yields:
/// ```none
///   Usage: app main_command <subcommand>
///          [arguments]
/// ```
///
/// If [outputPreferences.wrapText] is false, then the text will be returned
/// unchanged. If [shouldWrap] is specified, then it overrides the
/// [outputPreferences.wrapText] setting.
///
/// If the amount of indentation (from the text, [indent], and [hangingIndent])
/// is such that less than [kMinColumnWidth] characters can fit in the
/// [columnWidth], then the indent is truncated to allow the text to fit.
String wrapText(
  String text, {
  required int columnWidth,
  required bool shouldWrap,
  int? hangingIndent,
  int? indent,
}) {
  assert(columnWidth >= 0);
  if (text.isEmpty) {
    return '';
  }
  indent ??= 0;
  hangingIndent ??= 0;
  final List<String> splitText = text.split('\n');
  final List<String> result = <String>[];
  for (final String line in splitText) {
    String trimmedText = line.trimLeft();
    final String leadingWhitespace = line.substring(0, line.length - trimmedText.length);
    List<String> notIndented;
    if (hangingIndent != 0) {
      // When we have a hanging indent, we want to wrap the first line at one
      // width, and the rest at another (offset by hangingIndent), so we wrap
      // them twice and recombine.
      final List<String> firstLineWrap = _wrapTextAsLines(
        trimmedText,
        columnWidth: columnWidth - leadingWhitespace.length - indent,
        shouldWrap: shouldWrap,
      );
      notIndented = <String>[firstLineWrap.removeAt(0)];
      trimmedText = trimmedText.substring(notIndented[0].length).trimLeft();
      if (trimmedText.isNotEmpty) {
        notIndented.addAll(
          _wrapTextAsLines(
            trimmedText,
            columnWidth: columnWidth - leadingWhitespace.length - indent - hangingIndent,
            shouldWrap: shouldWrap,
          ),
        );
      }
    } else {
      notIndented = _wrapTextAsLines(
        trimmedText,
        columnWidth: columnWidth - leadingWhitespace.length - indent,
        shouldWrap: shouldWrap,
      );
    }
    String? hangingIndentString;
    final String indentString = ' ' * indent;
    result.addAll(
      notIndented.map<String>((String line) {
        // Don't return any lines with just whitespace on them.
        if (line.isEmpty) {
          return '';
        }
        String truncatedIndent = '$indentString${hangingIndentString ?? ''}$leadingWhitespace';
        if (truncatedIndent.length > columnWidth - kMinColumnWidth) {
          truncatedIndent = truncatedIndent.substring(
            0,
            math.max(columnWidth - kMinColumnWidth, 0),
          );
        }
        final String result = '$truncatedIndent$line';
        hangingIndentString ??= ' ' * hangingIndent!;
        return result;
      }),
    );
  }
  return result.join('\n');
}

// Used to represent a run of ANSI control sequences next to a visible
// character.
class _AnsiRun {
  _AnsiRun(this.original, this.character);

  String original;
  String character;
}

/// Wraps a block of text into lines no longer than [columnWidth], starting at the
/// [start] column, and returning the result as a list of strings.
///
/// Tries to split at whitespace, but if that's not good enough to keep it
/// under the limit, then splits in the middle of a word. Preserves embedded
/// newlines, but not indentation (it trims whitespace from each line).
///
/// If [columnWidth] is not specified, then the column width will be the width of the
/// terminal window by default. If the stdout is not a terminal window, then the
/// default will be [outputPreferences.wrapColumn].
///
/// The [columnWidth] is clamped to [kMinColumnWidth] at minimum (so passing negative
/// widths is fine, for instance).
///
/// If [outputPreferences.wrapText] is false, then the text will be returned
/// split at the newlines, but not wrapped. If [shouldWrap] is specified,
/// then it overrides the [outputPreferences.wrapText] setting.
List<String> _wrapTextAsLines(
  String text, {
  int start = 0,
  required int columnWidth,
  required bool shouldWrap,
}) {
  if (text.isEmpty) {
    return <String>[''];
  }
  assert(start >= 0);

  // Splits a string so that the resulting list has the same number of elements
  // as there are visible characters in the string, but elements may include one
  // or more adjacent ANSI sequences. Joining the list elements again will
  // reconstitute the original string. This is useful for manipulating "visible"
  // characters in the presence of ANSI control codes.
  List<_AnsiRun> splitWithCodes(String input) {
    final RegExp characterOrCode = RegExp('(\u001b\\[[0-9;]*m|.)', multiLine: true);
    List<_AnsiRun> result = <_AnsiRun>[];
    final StringBuffer current = StringBuffer();
    for (final Match match in characterOrCode.allMatches(input)) {
      current.write(match[0]);
      if (match[0]!.length < 4) {
        // This is a regular character, write it out.
        result.add(_AnsiRun(current.toString(), match[0]!));
        current.clear();
      }
    }
    // If there's something accumulated, then it must be an ANSI sequence, so
    // add it to the end of the last entry so that we don't lose it.
    if (current.isNotEmpty) {
      if (result.isNotEmpty) {
        result.last.original += current.toString();
      } else {
        // If there is nothing in the string besides control codes, then just
        // return them as the only entry.
        result = <_AnsiRun>[_AnsiRun(current.toString(), '')];
      }
    }
    return result;
  }

  String joinRun(List<_AnsiRun> list, int start, [int? end]) {
    return list.sublist(start, end).map<String>((_AnsiRun run) => run.original).join().trim();
  }

  final List<String> result = <String>[];
  final int effectiveLength = math.max(columnWidth - start, kMinColumnWidth);
  for (final String line in text.split('\n')) {
    // If the line is short enough, even with ANSI codes, then we can just add
    // it and move on.
    if (line.length <= effectiveLength || !shouldWrap) {
      result.add(line);
      continue;
    }
    final List<_AnsiRun> splitLine = splitWithCodes(line);
    if (splitLine.length <= effectiveLength) {
      result.add(line);
      continue;
    }

    int currentLineStart = 0;
    int? lastWhitespace;
    // Find the start of the current line.
    for (int index = 0; index < splitLine.length; ++index) {
      if (splitLine[index].character.isNotEmpty && _isWhitespace(splitLine[index])) {
        lastWhitespace = index;
      }

      if (index - currentLineStart >= effectiveLength) {
        // Back up to the last whitespace, unless there wasn't any, in which
        // case we just split where we are.
        if (lastWhitespace != null) {
          index = lastWhitespace;
        }

        result.add(joinRun(splitLine, currentLineStart, index));

        // Skip any intervening whitespace.
        while (index < splitLine.length && _isWhitespace(splitLine[index])) {
          index++;
        }

        currentLineStart = index;
        lastWhitespace = null;
      }
    }
    result.add(joinRun(splitLine, currentLineStart));
  }
  return result;
}

/// Returns true if the code unit at [index] in [text] is a whitespace
/// character.
///
/// Based on: https://en.wikipedia.org/wiki/Whitespace_character#Unicode
bool _isWhitespace(_AnsiRun run) {
  final int rune = run.character.isNotEmpty ? run.character.codeUnitAt(0) : 0x0;
  return rune >= 0x0009 && rune <= 0x000D ||
      rune == 0x0020 ||
      rune == 0x0085 ||
      rune == 0x1680 ||
      rune == 0x180E ||
      rune >= 0x2000 && rune <= 0x200A ||
      rune == 0x2028 ||
      rune == 0x2029 ||
      rune == 0x202F ||
      rune == 0x205F ||
      rune == 0x3000 ||
      rune == 0xFEFF;
}

final RegExp _interpolationRegex = RegExp(r'\$\{([^}]*)\}');

/// Given a string that possibly contains string interpolation sequences
/// (so for example, something like `ping -n 1 ${host}`), replace all those
/// interpolation sequences with the matching value given in [replacementValues].
///
/// If the value could not be found inside [replacementValues], an empty
/// string will be substituted instead.
///
/// However, if the dollar sign inside the string is preceded with a backslash,
/// the sequences won't be substituted at all.
///
/// Example:
/// ```dart
/// final interpolated = _interpolateString(r'ping -n 1 ${host}', {'host': 'raspberrypi'});
/// print(interpolated);  // will print 'ping -n 1 raspberrypi'
///
/// final interpolated2 = _interpolateString(r'ping -n 1 ${_host}', {'host': 'raspberrypi'});
/// print(interpolated2); // will print 'ping -n 1 '
/// ```
String interpolateString(String toInterpolate, Map<String, String> replacementValues) {
  return toInterpolate.replaceAllMapped(_interpolationRegex, (Match match) {
    /// The name of the variable to be inserted into the string.
    /// Example: If the source string is 'ping -n 1 ${host}',
    ///   `name` would be 'host'
    final String name = match.group(1)!;
    return replacementValues.containsKey(name) ? replacementValues[name]! : '';
  });
}

/// Given a list of strings possibly containing string interpolation sequences
/// (so for example, something like `['ping', '-n', '1', '${host}']`), replace
/// all those interpolation sequences with the matching value given in [replacementValues].
///
/// If the value could not be found inside [replacementValues], an empty
/// string will be substituted instead.
///
/// However, if the dollar sign inside the string is preceded with a backslash,
/// the sequences won't be substituted at all.
///
/// Example:
/// ```dart
/// final interpolated = _interpolateString(['ping', '-n', '1', r'${host}'], {'host': 'raspberrypi'});
/// print(interpolated);  // will print '[ping, -n, 1, raspberrypi]'
///
/// final interpolated2 = _interpolateString(['ping', '-n', '1', r'${_host}'], {'host': 'raspberrypi'});
/// print(interpolated2); // will print '[ping, -n, 1, ]'
/// ```
List<String> interpolateStringList(
  List<String> toInterpolate,
  Map<String, String> replacementValues,
) {
  return toInterpolate.map((String s) => interpolateString(s, replacementValues)).toList();
}

/// Returns the first line-based match for [regExp] in [file].
///
/// Assumes UTF8 encoding.
Match? firstMatchInFile(File file, RegExp regExp) {
  if (!file.existsSync()) {
    return null;
  }
  for (final String line in file.readAsLinesSync()) {
    final Match? match = regExp.firstMatch(line);
    if (match != null) {
      return match;
    }
  }
  return null;
}

/// Tests for shallow equality on two sets.
bool setEquals<T>(Set<T>? a, Set<T>? b) {
  if (a == null) {
    return b == null;
  }
  if (b == null || a.length != b.length) {
    return false;
  }
  if (identical(a, b)) {
    return true;
  }
  for (final T value in a) {
    if (!b.contains(value)) {
      return false;
    }
  }
  return true;
}

/// Tests for shallow equality on two lists.
bool listEquals<T>(List<T> a, List<T> b) {
  if (identical(a, b)) {
    return true;
  }
  if (a.length != b.length) {
    return false;
  }
  for (int index = 0; index < a.length; index++) {
    if (a[index] != b[index]) {
      return false;
    }
  }
  return true;
}
