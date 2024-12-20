// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:io';

import 'package:args/args.dart';
import 'package:path/path.dart' as path;

const int _kChar_A = 65;
const int _kChar_a = 97;

final ArgParser argParser = ArgParser()
  ..addFlag(
    'check',
    help: 'Check mode does not write anything to disk. '
        'It just checks if the generated files are still in sync or not.',
  );

/// A map of properties that could safely be normalized into other properties.
///
/// For example, a NL behaves exactly the same as BK so it gets normalized to BK
/// in the generated code.
const Map<String, String> normalizationTable = <String, String>{
  // NL behaves exactly the same as BK.
  // See: https://www.unicode.org/reports/tr14/tr14-45.html#NL
  'NL': 'BK',
  // In the absence of extra data (ICU data and language dictionaries), the
  // following properties will be treated as AL (alphabetic): AI, SA, SG and XX.
  // See LB1: https://www.unicode.org/reports/tr14/tr14-45.html#LB1
  'AI': 'AL',
  'SA': 'AL',
  'SG': 'AL',
  'XX': 'AL',
  // https://unicode.org/reports/tr14/tr14-45.html#CJ
  'CJ': 'NS',
};

/// A tuple that holds a [start] and [end] of a unicode range and a [property].
class UnicodeRange {
  const UnicodeRange(this.start, this.end, this.property);

  final int start;
  final int end;
  final EnumValue property;

  /// Checks if there's an overlap between this range and the [other] range.
  bool isOverlapping(UnicodeRange other) {
    return start <= other.end && end >= other.start;
  }

  /// Checks if the [other] range is adjacent to this range.
  ///
  /// Two ranges are considered adjacent if:
  /// - The new range immediately follows this range, and
  /// - The new range has the same property as this range.
  bool isAdjacent(UnicodeRange other) {
    return other.start == end + 1 && property == other.property;
  }

  /// Merges the ranges of the 2 [UnicodeRange]s if they are adjacent.
  UnicodeRange extendRange(UnicodeRange extension) {
    assert(isAdjacent(extension));
    return UnicodeRange(start, extension.end, property);
  }
}

final String webUnicodeRoot = path.dirname(path.dirname(Platform.script.toFilePath()));

final String propertiesDir = path.join(webUnicodeRoot, 'properties');
final String wordProperties = path.join(propertiesDir, 'WordBreakProperty.txt');
final String lineProperties = path.join(propertiesDir, 'LineBreak.txt');

final String codegenDir = path.join(webUnicodeRoot, 'lib', 'codegen');
final String wordBreakCodegen = path.join(codegenDir, 'word_break_properties.dart');
final String lineBreakCodegen = path.join(codegenDir, 'line_break_properties.dart');

/// This script parses the unicode word/line break properties(1) and generates Dart
/// code(2) that can perform lookups in the unicode ranges to find what property
/// a letter has.
///
/// (1) The word break properties file can be downloaded from:
///     https://www.unicode.org/Public/13.0.0/ucd/auxiliary/WordBreakProperty.txt
///
///     The line break properties file can be downloaded from:
///     https://www.unicode.org/Public/13.0.0/ucd/LineBreak.txt
///
///     Both files need to be located at third_party/web_unicode/properties.
///
/// (2) The codegen'd Dart files are located at:
///     third_party/web_unicode/lib/codegen/word_break_properties.dart
///     third_party/web_unicode/lib/codegen/line_break_properties.dart
Future<void> main(List<String> arguments) async {
  final ArgResults result = argParser.parse(arguments);
  final bool isCheck = result['check'] as bool;
  final List<PropertiesSyncer> syncers = <PropertiesSyncer>[
    WordBreakPropertiesSyncer(isCheck: isCheck),
    LineBreakPropertiesSyncer(isCheck: isCheck),
  ];

  for (final PropertiesSyncer syncer in syncers) {
    await syncer.perform();
  }
}

/// Base class that provides common logic for syncing all kinds of unicode
/// properties (e.g. word break properties, line break properties, etc).
///
/// Subclasses implement the [template] method which receives as argument the
/// list of data parsed by [processLines].
abstract class PropertiesSyncer {
  PropertiesSyncer(this._src, this._dest, {required this.isCheck});

  final String _src;
  final String _dest;
  final bool isCheck;

  String get prefix;
  String get enumDocLink;

  /// The default property to be used when a certain code point doesn't belong
  /// to any known range.
  String get defaultProperty;

  Future<void> perform() async {
    final List<String> lines = await File(_src).readAsLines();
    final PropertyCollection data =
        PropertyCollection.fromLines(lines, defaultProperty);

    final String output = template(data);

    if (isCheck) {
      // Read from destination and compare to the generated output.
      final String existing = await File(_dest).readAsString();
      if (existing != output) {
        final String relativeDest = path.relative(_dest, from: webUnicodeRoot);
        print('ERROR: $relativeDest is out of sync.');
        print('Please run "dart tool/unicode_sync_script.dart" to update it.');
        exit(1);
      }
    } else {
      final IOSink sink = File(_dest).openWrite();
      sink.write(output);
    }
  }

  String template(PropertyCollection data) {
    return '''
// Copyright 2022 Google LLC
//
// For terms of use, see https://www.unicode.org/copyright.html

// AUTO-GENERATED FILE.
// Generated by: tool/unicode_sync_script.dart

// ignore_for_file: public_member_api_docs

/// For an explanation of these enum values, see:
///
/// * $enumDocLink
enum ${prefix}CharProperty {
  ${_getEnumValues(data.enumCollection).join('\n  ')}
}

const String packed${prefix}BreakProperties =
  '${_packProperties(data)}';

const int single${prefix}BreakRangesCount = ${_getSingleRangesCount(data)};

const ${prefix}CharProperty default${prefix}CharProperty = ${prefix}CharProperty.$defaultProperty;
''';
  }

  Iterable<String> _getEnumValues(EnumCollection enumCollection) {
    return enumCollection.values.expand(
      (EnumValue value) => <String>[
        if (value.normalizedFrom.isNotEmpty)
          '// Normalized from: ${value.normalizedFrom.join(', ')}',
        '${value.enumName}, // serialized as "${value.serialized}"',
      ],
    );
  }

  int _getSingleRangesCount(PropertyCollection data) {
    int count = 0;
    for (final UnicodeRange range in data.ranges) {
      if (range.start == range.end) {
        count++;
      }
    }
    return count;
  }

  String _packProperties(PropertyCollection data) {
    final StringBuffer buffer = StringBuffer();
    for (final UnicodeRange range in data.ranges) {
      buffer.write(range.start.toRadixString(36).padLeft(4, '0'));
      if (range.start == range.end) {
        buffer.write('!');
      } else {
        buffer.write(range.end.toRadixString(36).padLeft(4, '0'));
      }
      buffer.write(range.property.serialized);
    }
    return buffer.toString();
  }
}

/// Syncs Unicode's word break properties.
class WordBreakPropertiesSyncer extends PropertiesSyncer {
  WordBreakPropertiesSyncer({required bool isCheck})
      : super(wordProperties, wordBreakCodegen, isCheck: isCheck);

  @override
  final String prefix = 'Word';

  @override
  final String enumDocLink =
      'http://unicode.org/reports/tr29/#Table_Word_Break_Property_Values';

  @override
  final String defaultProperty = 'Unknown';
}

/// Syncs Unicode's line break properties.
class LineBreakPropertiesSyncer extends PropertiesSyncer {
  LineBreakPropertiesSyncer({required bool isCheck})
      : super(lineProperties, lineBreakCodegen, isCheck: isCheck);

  @override
  final String prefix = 'Line';

  @override
  final String enumDocLink =
      'https://www.unicode.org/reports/tr14/tr14-45.html#DescriptionOfProperties';

  @override
  final String defaultProperty = 'AL';
}

/// Holds the collection of properties parsed from the unicode spec file.
class PropertyCollection {
  PropertyCollection.fromLines(List<String> lines, String defaultProperty) {
    final List<UnicodeRange> unprocessedRanges = lines
        .map(removeCommentFromLine)
        .where((String line) => line.isNotEmpty)
        .map(parseLineIntoUnicodeRange)
        .toList();
    // Insert the default property if it doesn't exist.
    final EnumValue? found = enumCollection.values.cast<EnumValue?>().firstWhere(
      (EnumValue? property) => property!.name == defaultProperty,
      orElse: () => null,
    );
    if (found == null) {
      enumCollection.add(defaultProperty);
    }
    ranges = processRanges(unprocessedRanges, defaultProperty).toList();
  }

  late List<UnicodeRange> ranges;

  final EnumCollection enumCollection = EnumCollection();

  /// Examples:
  ///
  /// 00C0..00D6    ; ALetter
  /// 037F          ; ALetter
  ///
  /// Would be parsed into:
  ///
  /// ```dart
  /// UnicodeRange(192, 214, EnumValue('ALetter'));
  /// UnicodeRange(895, 895, EnumValue('ALetter'));
  /// ```
  UnicodeRange parseLineIntoUnicodeRange(String line) {
    final List<String> split = line.split(';');
    final String rangeStr = split[0].trim();
    final String propertyStr = split[1].trim();

    final EnumValue property = normalizationTable.containsKey(propertyStr)
        ? enumCollection.add(normalizationTable[propertyStr]!, propertyStr)
        : enumCollection.add(propertyStr);

    return UnicodeRange(
      getRangeStart(rangeStr),
      getRangeEnd(rangeStr),
      property,
    );
  }
}

/// Represents the collection of values of an enum.
class EnumCollection {
  final List<EnumValue> values = <EnumValue>[];

  EnumValue add(String name, [String? normalizedFrom]) {
    final int index =
        values.indexWhere((EnumValue value) => value.name == name);
    EnumValue value;
    if (index == -1) {
      value = EnumValue(values.length, name);
      values.add(value);
    } else {
      value = values[index];
    }

    if (normalizedFrom != null) {
      value.normalizedFrom.add(normalizedFrom);
    }
    return value;
  }
}

/// Represents a single value in an [EnumCollection].
class EnumValue {
  EnumValue(this.index, this.name);

  final int index;
  final String name;

  /// The properties that were normalized to this value.
  final Set<String> normalizedFrom = <String>{};

  /// Returns a serialized, compact format of the enum value.
  ///
  /// Enum values are serialized based on their index. We start serializing them
  /// to "A", "B", "C", etc until we reach "Z". Then we continue with "a", "b",
  /// "c", etc.
  String get serialized {
    // We assign uppercase letters to the first 26 enum values.
    if (index < 26) {
      return String.fromCharCode(_kChar_A + index);
    }
    // Enum values above 26 will be assigned a lowercase letter.
    return String.fromCharCode(_kChar_a + index - 26);
  }

  /// Returns the enum name that'll be used in the Dart code.
  ///
  /// ```dart
  /// enum CharProperty {
  ///   ALetter, // <-- this is the name returned by this method ("ALetter").
  ///   Numeric,
  ///   // etc...
  /// }
  /// ```
  String get enumName {
    return name.replaceAll('_', '');
  }
}

/// Sorts ranges and combines adjacent ranges that have the same property and
/// can be merged.
Iterable<UnicodeRange> processRanges(
  List<UnicodeRange> data,
  String defaultProperty,
) {
  data.sort(
    // Ranges don't overlap so it's safe to sort based on the start of each
    // range.
    (UnicodeRange range1, UnicodeRange range2) =>
        range1.start.compareTo(range2.start),
  );
  verifyNoOverlappingRanges(data);
  return combineAdjacentRanges(data, defaultProperty);
}

/// Example:
///
/// ```none
/// 0x01C4..0x0293; ALetter
/// 0x0294..0x0294; ALetter
/// 0x0295..0x02AF; ALetter
/// ```
///
/// will get combined into:
///
/// ```none
/// 0x01C4..0x02AF; ALetter
/// ```
List<UnicodeRange> combineAdjacentRanges(
  List<UnicodeRange> data,
  String defaultProperty,
) {
  final List<UnicodeRange> result = <UnicodeRange>[data.first];
  for (int i = 1; i < data.length; i++) {
    final UnicodeRange prev = result.last;
    final UnicodeRange next = data[i];
    if (prev.isAdjacent(next)) {
      result.last = prev.extendRange(next);
    } else if (prev.property == next.property &&
        prev.property.name == defaultProperty) {
      // When there's a gap between two ranges, but they both have the default
      // property, it's safe to combine them.
      result.last = prev.extendRange(next);
    } else {
      // Check if there's a gap between the previous range and this range.
      result.add(next);
    }
  }
  return result;
}

int getRangeStart(String range) {
  return int.parse(range.split('..')[0], radix: 16);
}

int getRangeEnd(String range) {
  if (range.contains('..')) {
    return int.parse(range.split('..')[1], radix: 16);
  }
  return int.parse(range, radix: 16);
}

void verifyNoOverlappingRanges(List<UnicodeRange> data) {
  for (int i = 1; i < data.length; i++) {
    if (data[i].isOverlapping(data[i - 1])) {
      throw Exception('Data contains overlapping ranges.');
    }
  }
}

String removeCommentFromLine(String line) {
  final int poundIdx = line.indexOf('#');
  return (poundIdx == -1) ? line : line.substring(0, poundIdx);
}
