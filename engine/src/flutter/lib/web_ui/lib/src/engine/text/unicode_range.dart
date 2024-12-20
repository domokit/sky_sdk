// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

const int kChar_0 = 48;
const int kChar_9 = kChar_0 + 9;
const int kChar_A = 65;
const int kChar_Z = 90;
const int kChar_a = 97;
const int kChar_z = 122;
const int kCharBang = 33;
const int kMashriqi_0 = 0x660;
const int kMashriqi_9 = kMashriqi_0 + 9;

enum _ComparisonResult { inside, higher, lower }

/// Each instance of [UnicodeRange] represents a range of unicode characters
/// that are assigned a [CharProperty]. For example, the following snippet:
///
/// ```dart
/// UnicodeRange(0x0041, 0x005A, CharProperty.ALetter);
/// ```
///
/// is saying that all characters between 0x0041 ("A") and 0x005A ("Z") are
/// assigned the property [CharProperty.ALetter].
///
/// Note that the Unicode spec uses inclusive ranges and we are doing the
/// same here.
class UnicodeRange<P> {
  const UnicodeRange(this.start, this.end, this.property);

  final int start;

  final int end;

  final P property;

  /// Compare a [value] to this range.
  ///
  /// The return value is either:
  /// - lower: The value is lower than the range.
  /// - higher: The value is higher than the range
  /// - inside: The value is within the range.
  _ComparisonResult compare(int value) {
    if (value < start) {
      return _ComparisonResult.lower;
    }
    if (value > end) {
      return _ComparisonResult.higher;
    }
    return _ComparisonResult.inside;
  }
}

/// Checks whether the given char code is a UTF-16 surrogate.
///
/// See:
/// - http://www.unicode.org/faq//utf_bom.html#utf16-2
bool isUtf16Surrogate(int char) {
  return char & 0xF800 == 0xD800;
}

/// Combines a pair of UTF-16 surrogate into a single character code point.
///
/// The surrogate pair is expected to start at [index] in the [text].
///
/// See:
/// - http://www.unicode.org/faq//utf_bom.html#utf16-3
int combineSurrogatePair(String text, int index) {
  final int hi = text.codeUnitAt(index);
  final int lo = text.codeUnitAt(index + 1);

  final int x = (hi & ((1 << 6) - 1)) << 10 | lo & ((1 << 10) - 1);
  final int w = (hi >> 6) & ((1 << 5) - 1);
  final int u = w + 1;
  return u << 16 | x;
}

/// Returns the code point from [text] at [index] and handles surrogate pairs
/// for cases that involve two UTF-16 codes.
int? getCodePoint(String text, int index) {
  if (index < 0 || index >= text.length) {
    return null;
  }

  final int char = text.codeUnitAt(index);
  if (isUtf16Surrogate(char) && index < text.length - 1) {
    return combineSurrogatePair(text, index);
  }
  return char;
}

/// Given a list of [UnicodeRange]s, this class performs efficient lookup
/// to find which range a value falls into.
///
/// The lookup algorithm expects the ranges to have the following constraints:
/// - Be sorted.
/// - No overlap between the ranges.
/// - Gaps between ranges are ok.
///
/// This is used in the context of unicode to find out what property a letter
/// has. The properties are then used to decide word boundaries, line break
/// opportunities, etc.
class UnicodePropertyLookup<P> {
  UnicodePropertyLookup(this.ranges, this.defaultProperty);

  /// Creates a [UnicodePropertyLookup] from packed line break data.
  factory UnicodePropertyLookup.fromPackedData(
    String packedData,
    int singleRangesCount,
    List<P> propertyEnumValues,
    P defaultProperty,
  ) {
    return UnicodePropertyLookup<P>(
      _unpackProperties<P>(packedData, singleRangesCount, propertyEnumValues),
      defaultProperty,
    );
  }

  /// The list of unicode ranges and their associated properties.
  final List<UnicodeRange<P>> ranges;

  /// The default property to use when a character doesn't belong in any
  /// known range.
  final P defaultProperty;

  /// Cache for lookup results.
  final Map<int, P> _cache = <int, P>{};

  /// Take a [text] and an [index], and returns the property of the character
  /// located at that [index].
  ///
  /// If the [index] is out of range, null will be returned.
  P find(String text, int index) {
    final int? codePoint = getCodePoint(text, index);
    return codePoint == null ? defaultProperty : findForChar(codePoint);
  }

  /// Takes one character as an integer code unit and returns its property.
  ///
  /// If a property can't be found for the given character, then the default
  /// property will be returned.
  P findForChar(int? char) {
    if (char == null) {
      return defaultProperty;
    }

    final P? cacheHit = _cache[char];
    if (cacheHit != null) {
      return cacheHit;
    }

    final int rangeIndex = _binarySearch(char);
    final P result = rangeIndex == -1 ? defaultProperty : ranges[rangeIndex].property;
    // Cache the result.
    _cache[char] = result;
    return result;
  }

  int _binarySearch(int value) {
    int min = 0;
    int max = ranges.length;
    while (min < max) {
      final int mid = min + ((max - min) >> 1);
      final UnicodeRange<P> range = ranges[mid];
      switch (range.compare(value)) {
        case _ComparisonResult.higher:
          min = mid + 1;
        case _ComparisonResult.lower:
          max = mid;
        case _ComparisonResult.inside:
          return mid;
      }
    }
    return -1;
  }
}

List<UnicodeRange<P>> _unpackProperties<P>(
  String packedData,
  int singleRangesCount,
  List<P> propertyEnumValues,
) {
  // Packed data is mostly structured in chunks of 9 characters each:
  //
  // * [0..3]: Range start, encoded as a base36 integer.
  // * [4..7]: Range end, encoded as a base36 integer.
  // * [8]: Index of the property enum value, encoded as a single letter.
  //
  // When the range is a single number (i.e. range start == range end), it gets
  // packed more efficiently in a chunk of 6 characters:
  //
  // * [0..3]: Range start (and range end), encoded as a base 36 integer.
  // * [4]: "!" to indicate that there's no range end.
  // * [5]: Index of the property enum value, encoded as a single letter.

  // `packedData.length + singleRangesCount * 3` would have been the size of the
  // packed data if the efficient packing of single-range items wasn't applied.
  assert((packedData.length + singleRangesCount * 3) % 9 == 0);

  final List<UnicodeRange<P>> ranges = <UnicodeRange<P>>[];
  final int dataLength = packedData.length;
  int i = 0;
  while (i < dataLength) {
    final int rangeStart = _consumeInt(packedData, i);
    i += 4;

    int rangeEnd;
    if (packedData.codeUnitAt(i) == kCharBang) {
      rangeEnd = rangeStart;
      i++;
    } else {
      rangeEnd = _consumeInt(packedData, i);
      i += 4;
    }
    final int charCode = packedData.codeUnitAt(i);
    final P property = propertyEnumValues[_getEnumIndexFromPackedValue(charCode)];
    i++;

    ranges.add(UnicodeRange<P>(rangeStart, rangeEnd, property));
  }
  return ranges;
}

int _getEnumIndexFromPackedValue(int charCode) {
  // This has to stay in sync with [EnumValue.serialized] in
  // `tool/unicode_sync_script.dart`.

  assert(
    (charCode >= kChar_A && charCode <= kChar_Z) || (charCode >= kChar_a && charCode <= kChar_z),
  );

  // Uppercase letters were assigned to the first 26 enum values.
  if (charCode <= kChar_Z) {
    return charCode - kChar_A;
  }
  // Lowercase letters were assigned to enum values above 26.
  return 26 + charCode - kChar_a;
}

int _consumeInt(String packedData, int index) {
  // The implementation is equivalent to:
  //
  // ```dart
  // return int.tryParse(packedData.substring(index, index + 4), radix: 36);
  // ```
  //
  // But using substring is slow when called too many times. This custom
  // implementation makes the unpacking 25%-45% faster than using substring.
  final int digit0 = getIntFromCharCode(packedData.codeUnitAt(index + 3));
  final int digit1 = getIntFromCharCode(packedData.codeUnitAt(index + 2));
  final int digit2 = getIntFromCharCode(packedData.codeUnitAt(index + 1));
  final int digit3 = getIntFromCharCode(packedData.codeUnitAt(index));
  return digit0 + (digit1 * 36) + (digit2 * 36 * 36) + (digit3 * 36 * 36 * 36);
}

/// Does the same thing as [int.parse(str, 36)] but takes only a single
/// character as a [charCode] integer.
int getIntFromCharCode(int charCode) {
  assert(
    (charCode >= kChar_0 && charCode <= kChar_9) || (charCode >= kChar_a && charCode <= kChar_z),
  );

  if (charCode <= kChar_9) {
    return charCode - kChar_0;
  }
  // "a" starts from 10 and remaining letters go up from there.
  return charCode - kChar_a + 10;
}
