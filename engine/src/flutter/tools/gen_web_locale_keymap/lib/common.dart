// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// The following segment is not only used in the generating script, but also
// copied to the generated package.
/*@@@ SHARED SEGMENT START @@@*/

/// Used in the final mapping indicating the logical key should be derived from
/// KeyboardEvent.keyCode.
///
/// This value is chosen because it's a printable character within EASCII that
/// will never be mapped to (checked in the marshalling algorithm).
const int kUseKeyCode = 0xFF;

/// Used in the final mapping indicating the event key is 'Dead', the dead key.
final String _kUseDead = String.fromCharCode(0xFE);

/// The KeyboardEvent.key for a dead key.
const String _kEventKeyDead = 'Dead';

/// A map of all goals from the scan codes to their mapped value in US layout.
const Map<String, String> kLayoutGoals = <String, String>{
  'KeyA': 'a',
  'KeyB': 'b',
  'KeyC': 'c',
  'KeyD': 'd',
  'KeyE': 'e',
  'KeyF': 'f',
  'KeyG': 'g',
  'KeyH': 'h',
  'KeyI': 'i',
  'KeyJ': 'j',
  'KeyK': 'k',
  'KeyL': 'l',
  'KeyM': 'm',
  'KeyN': 'n',
  'KeyO': 'o',
  'KeyP': 'p',
  'KeyQ': 'q',
  'KeyR': 'r',
  'KeyS': 's',
  'KeyT': 't',
  'KeyU': 'u',
  'KeyV': 'v',
  'KeyW': 'w',
  'KeyX': 'x',
  'KeyY': 'y',
  'KeyZ': 'z',
  'Digit1': '1',
  'Digit2': '2',
  'Digit3': '3',
  'Digit4': '4',
  'Digit5': '5',
  'Digit6': '6',
  'Digit7': '7',
  'Digit8': '8',
  'Digit9': '9',
  'Digit0': '0',
  'Minus': '-',
  'Equal': '=',
  'BracketLeft': '[',
  'BracketRight': ']',
  'Backslash': r'\',
  'Semicolon': ';',
  'Quote': "'",
  'Backquote': '`',
  'Comma': ',',
  'Period': '.',
  'Slash': '/',
};

final int _kLowerA = 'a'.codeUnitAt(0);
final int _kUpperA = 'A'.codeUnitAt(0);
final int _kLowerZ = 'z'.codeUnitAt(0);
final int _kUpperZ = 'Z'.codeUnitAt(0);

bool _isAscii(int charCode) {
  // 0x20 is the first printable character in ASCII.
  return charCode >= 0x20 && charCode <= 0x7F;
}

/// Returns whether the `char` is a single character of a letter or a digit.
bool isLetter(int charCode) {
  return (charCode >= _kLowerA && charCode <= _kLowerZ) ||
      (charCode >= _kUpperA && charCode <= _kUpperZ);
}

/// A set of rules that can derive a large number of logical keys simply from
/// the event's code and key.
///
/// This greatly reduces the entries needed in the final mapping.
int? heuristicMapper(String code, String key) {
  // Digit code: return the digit by event code.
  if (code.startsWith('Digit')) {
    assert(code.length == 6);
    return code.codeUnitAt(5); // The character immediately after 'Digit'
  }
  final int charCode = key.codeUnitAt(0);
  // Non-ascii: return the goal (i.e. US mapping by event code).
  if (key.length > 1 || !_isAscii(charCode)) {
    return kLayoutGoals[code]?.codeUnitAt(0);
  }
  // Letter key: return the event key letter.
  if (isLetter(charCode)) {
    return key.toLowerCase().codeUnitAt(0);
  }
  return null;
}

// Maps an integer to a printable EASCII character by adding it to this value.
//
// We could've chosen 0x20, the first printable character, for a slightly bigger
// range, but it's prettier this way and sufficient.
final int _kMarshallIntBase = '0'.codeUnitAt(0);

class _StringStream {
  _StringStream(this._data) : _offset = 0;

  final String _data;
  final Map<int, String> _goalToEventCode = Map<int, String>.fromEntries(
    kLayoutGoals.entries.map(
      (MapEntry<String, String> beforeEntry) =>
          MapEntry<int, String>(beforeEntry.value.codeUnitAt(0), beforeEntry.key),
    ),
  );

  int get offest => _offset;
  int _offset;

  int readIntAsVerbatim() {
    final int result = _data.codeUnitAt(_offset);
    _offset += 1;
    assert(result >= _kMarshallIntBase);
    return result - _kMarshallIntBase;
  }

  int readIntAsChar() {
    final int result = _data.codeUnitAt(_offset);
    _offset += 1;
    return result;
  }

  String readEventKey() {
    final String char = String.fromCharCode(readIntAsChar());
    if (char == _kUseDead) {
      return _kEventKeyDead;
    } else {
      return char;
    }
  }

  String readEventCode() {
    final int charCode = _data.codeUnitAt(_offset);
    _offset += 1;
    return _goalToEventCode[charCode]!;
  }
}

Map<String, int> _unmarshallCodeMap(_StringStream stream) {
  final int entryNum = stream.readIntAsVerbatim();
  return <String, int>{
    for (int i = 0; i < entryNum; i++) stream.readEventKey(): stream.readIntAsChar(),
  };
}

/// Decode a key mapping data out of the string.
Map<String, Map<String, int>> unmarshallMappingData(String compressed) {
  final _StringStream stream = _StringStream(compressed);
  final int eventCodeNum = stream.readIntAsVerbatim();
  return <String, Map<String, int>>{
    for (int i = 0; i < eventCodeNum; i++) stream.readEventCode(): _unmarshallCodeMap(stream),
  };
}

/*@@@ SHARED SEGMENT END @@@*/

/// Whether the given charCode is a ASCII letter.
bool isLetterChar(int charCode) {
  return (charCode >= _kLowerA && charCode <= _kLowerZ) ||
      (charCode >= _kUpperA && charCode <= _kUpperZ);
}

bool _isPrintableEascii(int charCode) {
  return charCode >= 0x20 && charCode <= 0xFF;
}

typedef _ForEachAction<V> = void Function(String key, V value);
void _sortedForEach<V>(Map<String, V> map, _ForEachAction<V> action) {
  map.entries.toList()
    ..sort((MapEntry<String, V> a, MapEntry<String, V> b) => a.key.compareTo(b.key))
    ..forEach((MapEntry<String, V> entry) {
      action(entry.key, entry.value);
    });
}

// Encode a small integer as a character by its value.
//
// For example, 0x48 is encoded as '0'. This means that values within 0x0 - 0x19
// or greater than 0xFF are forbidden.
void _marshallIntAsChar(StringBuffer builder, int value) {
  assert(_isPrintableEascii(value), '$value');
  builder.writeCharCode(value);
}

const int _kMarshallIntEnd = 0xFF; // The last printable EASCII.
// Encode a small integer as a character based on a certain printable codepoint.
//
// For example, 0x0 is encoded as '0', and 0x1 is encoded as '1'. This function
// allows smaller values than _marshallIntAsChar.
void _marshallIntAsVerbatim(StringBuffer builder, int value) {
  final int result = value + _kMarshallIntBase;
  assert(result <= _kMarshallIntEnd);
  builder.writeCharCode(result);
}

void _marshallEventCode(StringBuffer builder, String value) {
  // Instead of recording the entire eventCode, since the eventCode is mapped
  // 1-to-1 to a character in kLayoutGoals, we record the goal instead.
  final String char = kLayoutGoals[value]!;
  builder.write(char);
}

void _marshallEventKey(StringBuffer builder, String value) {
  if (value == _kEventKeyDead) {
    builder.write(_kUseDead);
  } else {
    assert(value.length == 1, value);
    assert(value != _kUseDead);
    builder.write(value);
  }
}

/// Encode a key mapping data into a list of strings.
///
/// The list of strings should be used concatenated, but is returned this way
/// for aesthetic purposes (one entry per line).
///
/// The algorithm aims at encoding the map directly into a printable string
/// (instead of a binary stream converted by base64). Some characters in the
/// string can be multi-byte, which means the decoder should parse the string
/// using substr instead of as a binary stream.
List<String> marshallMappingData(Map<String, Map<String, int>> mappingData) {
  final StringBuffer builder = StringBuffer();
  _marshallIntAsVerbatim(builder, mappingData.length);
  _sortedForEach(mappingData, (String eventCode, Map<String, int> codeMap) {
    builder.write('\n');
    _marshallEventCode(builder, eventCode);
    _marshallIntAsVerbatim(builder, codeMap.length);
    _sortedForEach(codeMap, (String eventKey, int logicalKey) {
      _marshallEventKey(builder, eventKey);
      _marshallIntAsChar(builder, logicalKey);
    });
  });
  return builder.toString().split('\n');
}
