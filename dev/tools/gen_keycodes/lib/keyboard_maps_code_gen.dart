// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'data.dart';
import 'logical_key_data.dart';
import 'physical_key_data.dart';
import 'utils.dart';

bool _isAsciiLetter(String? char) {
  if (char == null) {
    return false;
  }
  const int charUpperA = 0x41;
  const int charUpperZ = 0x5A;
  const int charLowerA = 0x61;
  const int charLowerZ = 0x7A;
  assert(char.length == 1);
  final int charCode = char.codeUnitAt(0);
  return (charCode >= charUpperA && charCode <= charUpperZ) ||
      (charCode >= charLowerA && charCode <= charLowerZ);
}

bool _isDigit(String? char) {
  if (char == null) {
    return false;
  }
  final int charDigit0 = '0'.codeUnitAt(0);
  final int charDigit9 = '9'.codeUnitAt(0);
  assert(char.length == 1);
  final int charCode = char.codeUnitAt(0);
  return charCode >= charDigit0 && charCode <= charDigit9;
}

/// Generates the keyboard_maps.g.dart files, based on the information in the key
/// data structure given to it.
class KeyboardMapsCodeGenerator extends BaseCodeGenerator {
  KeyboardMapsCodeGenerator(super.keyData, super.logicalData);

  List<PhysicalKeyEntry> get _numpadKeyData {
    return keyData.entries.where((PhysicalKeyEntry entry) {
      return entry.constantName.startsWith('numpad') &&
          LogicalKeyData.printable.containsKey(entry.name);
    }).toList();
  }

  List<PhysicalKeyEntry> get _functionKeyData {
    final RegExp functionKeyRe = RegExp(r'^f[0-9]+$');
    return keyData.entries.where((PhysicalKeyEntry entry) {
      return functionKeyRe.hasMatch(entry.constantName);
    }).toList();
  }

  List<LogicalKeyEntry> get _numpadLogicalKeyData {
    return logicalData.entries.where((LogicalKeyEntry entry) {
      return entry.constantName.startsWith('numpad') &&
          LogicalKeyData.printable.containsKey(entry.name);
    }).toList();
  }

  /// This generates the map of GLFW number pad key codes to logical keys.
  String get _glfwNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('GLFW numpad map');
    for (final PhysicalKeyEntry entry in _numpadKeyData) {
      final LogicalKeyEntry logicalKey = logicalData.entryByName(entry.name);
      for (final int code in logicalKey.glfwValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of GLFW key codes to logical keys.
  String get _glfwKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('GLFW key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int value in entry.glfwValues) {
        lines.add(value, '  $value: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of GTK number pad key codes to logical keys.
  String get _gtkNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('GTK numpad map');
    for (final LogicalKeyEntry entry in _numpadLogicalKeyData) {
      for (final int code in entry.gtkValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of GTK key codes to logical keys.
  String get _gtkKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('GTK key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int code in entry.gtkValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of XKB USB HID codes to physical keys.
  String get _xkbScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('GTK scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.xKbScanCode != null) {
        lines.add(
          entry.xKbScanCode!,
          '  ${toHex(entry.xKbScanCode)}: PhysicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Android key codes to logical keys.
  String get _androidKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Android key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int code in entry.androidValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Android number pad key codes to logical keys.
  String get _androidNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('Android numpad map');
    for (final LogicalKeyEntry entry in _numpadLogicalKeyData) {
      for (final int code in entry.androidValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Android scan codes to physical keys.
  String get _androidScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Android scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      for (final int code in entry.androidScanCodes) {
        lines.add(code, '  $code: PhysicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Windows scan codes to physical keys.
  String get _windowsScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Windows scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.windowsScanCode != null) {
        lines.add(
          entry.windowsScanCode!,
          '  ${entry.windowsScanCode}: PhysicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Windows number pad key codes to logical keys.
  String get _windowsNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('Windows numpad map');
    for (final LogicalKeyEntry entry in _numpadLogicalKeyData) {
      for (final int code in entry.windowsValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Windows key codes to logical keys.
  String get _windowsKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Windows key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      // Letter keys on Windows are not recorded in logical_key_data.g.json,
      // because they are not used by the embedding. Add them manually.
      final List<int>? keyCodes =
          entry.windowsValues.isNotEmpty
              ? entry.windowsValues
              : (_isAsciiLetter(entry.keyLabel)
                  ? <int>[entry.keyLabel!.toUpperCase().codeUnitAt(0)]
                  : _isDigit(entry.keyLabel)
                  ? <int>[entry.keyLabel!.toUpperCase().codeUnitAt(0)]
                  : null);
      if (keyCodes != null) {
        for (final int code in keyCodes) {
          lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
        }
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of macOS key codes to physical keys.
  String get _macOSScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('macOS scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.macOSScanCode != null) {
        lines.add(
          entry.macOSScanCode!,
          '  ${toHex(entry.macOSScanCode)}: PhysicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of macOS number pad key codes to logical keys.
  String get _macOSNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('macOS numpad map');
    for (final PhysicalKeyEntry entry in _numpadKeyData) {
      if (entry.macOSScanCode != null) {
        lines.add(
          entry.macOSScanCode!,
          '  ${toHex(entry.macOSScanCode)}: LogicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  String get _macOSFunctionKeyMap {
    final OutputLines<int> lines = OutputLines<int>('macOS function key map');
    for (final PhysicalKeyEntry entry in _functionKeyData) {
      if (entry.macOSScanCode != null) {
        lines.add(
          entry.macOSScanCode!,
          '  ${toHex(entry.macOSScanCode)}: LogicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of macOS key codes to physical keys.
  String get _macOSKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('MacOS key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int code in entry.macOSKeyCodeValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of iOS key codes to physical keys.
  String get _iOSScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('iOS scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.iOSScanCode != null) {
        lines.add(
          entry.iOSScanCode!,
          '  ${toHex(entry.iOSScanCode)}: PhysicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of iOS key label to logical keys for special keys.
  String get _iOSSpecialMap {
    final OutputLines<int> lines = OutputLines<int>('iOS special key mapping');
    kIosSpecialKeyMapping.forEach((String key, String logicalName) {
      final LogicalKeyEntry entry = logicalData.entryByName(logicalName);
      lines.add(entry.value, "  '$key': LogicalKeyboardKey.${entry.constantName},");
    });
    return lines.join().trimRight();
  }

  /// This generates the map of iOS number pad key codes to logical keys.
  String get _iOSNumpadMap {
    final OutputLines<int> lines = OutputLines<int>('iOS numpad map');
    for (final PhysicalKeyEntry entry in _numpadKeyData) {
      if (entry.iOSScanCode != null) {
        lines.add(
          entry.iOSScanCode!,
          '  ${toHex(entry.iOSScanCode)}: LogicalKeyboardKey.${entry.constantName},',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of macOS key codes to physical keys.
  String get _iOSKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('iOS key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int code in entry.iOSKeyCodeValues) {
        lines.add(code, '  $code: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Fuchsia key codes to logical keys.
  String get _fuchsiaKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Fuchsia key code map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final int value in entry.fuchsiaValues) {
        lines.add(value, '  ${toHex(value)}: LogicalKeyboardKey.${entry.constantName},');
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Fuchsia USB HID codes to physical keys.
  String get _fuchsiaHidCodeMap {
    final StringBuffer fuchsiaScanCodeMap = StringBuffer();
    for (final PhysicalKeyEntry entry in keyData.entries) {
      fuchsiaScanCodeMap.writeln(
        '  ${toHex(entry.usbHidCode)}: PhysicalKeyboardKey.${entry.constantName},',
      );
    }
    return fuchsiaScanCodeMap.toString().trimRight();
  }

  /// This generates the map of Web KeyboardEvent codes to logical keys.
  String get _webLogicalKeyMap {
    final OutputLines<String> lines = OutputLines<String>('Web logical key map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      for (final String name in entry.webNames) {
        lines.add(name, "  '$name': LogicalKeyboardKey.${entry.constantName},");
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Web KeyboardEvent codes to physical keys.
  String get _webPhysicalKeyMap {
    final OutputLines<String> lines = OutputLines<String>(
      'Web physical key map',
      behavior: DeduplicateBehavior.kKeep,
    );
    for (final PhysicalKeyEntry entry in keyData.entries) {
      for (final String webCodes in entry.webCodes()) {
        lines.add(entry.name, "  '$webCodes': PhysicalKeyboardKey.${entry.constantName},");
      }
    }
    return lines.sortedJoin().trimRight();
  }

  String get _webNumpadMap {
    final OutputLines<String> lines = OutputLines<String>('Web numpad map');
    for (final LogicalKeyEntry entry in _numpadLogicalKeyData) {
      lines.add(entry.name, "  '${entry.name}': LogicalKeyboardKey.${entry.constantName},");
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Web number pad codes to logical keys.
  String get _webLocationMap {
    final String jsonRaw =
        File(path.join(dataRoot, 'web_logical_location_mapping.json')).readAsStringSync();
    final Map<String, List<String?>> locationMap = parseMapOfListOfNullableString(jsonRaw);
    final OutputLines<String> lines = OutputLines<String>('Web location map');
    locationMap.forEach((String key, List<String?> keyNames) {
      final String keyStrings = keyNames
          .map((String? keyName) {
            final String? constantName =
                keyName == null ? null : logicalData.entryByName(keyName).constantName;
            return constantName != null ? 'LogicalKeyboardKey.$constantName' : 'null';
          })
          .join(', ');
      lines.add(key, "  '$key': <LogicalKeyboardKey?>[$keyStrings],");
    });
    return lines.sortedJoin().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'keyboard_maps.tmpl');

  @override
  Map<String, String> mappings() {
    return <String, String>{
      'ANDROID_SCAN_CODE_MAP': _androidScanCodeMap,
      'ANDROID_KEY_CODE_MAP': _androidKeyCodeMap,
      'ANDROID_NUMPAD_MAP': _androidNumpadMap,
      'FUCHSIA_SCAN_CODE_MAP': _fuchsiaHidCodeMap,
      'FUCHSIA_KEY_CODE_MAP': _fuchsiaKeyCodeMap,
      'MACOS_SCAN_CODE_MAP': _macOSScanCodeMap,
      'MACOS_NUMPAD_MAP': _macOSNumpadMap,
      'MACOS_FUNCTION_KEY_MAP': _macOSFunctionKeyMap,
      'MACOS_KEY_CODE_MAP': _macOSKeyCodeMap,
      'IOS_SCAN_CODE_MAP': _iOSScanCodeMap,
      'IOS_SPECIAL_MAP': _iOSSpecialMap,
      'IOS_NUMPAD_MAP': _iOSNumpadMap,
      'IOS_KEY_CODE_MAP': _iOSKeyCodeMap,
      'GLFW_KEY_CODE_MAP': _glfwKeyCodeMap,
      'GLFW_NUMPAD_MAP': _glfwNumpadMap,
      'GTK_KEY_CODE_MAP': _gtkKeyCodeMap,
      'GTK_NUMPAD_MAP': _gtkNumpadMap,
      'XKB_SCAN_CODE_MAP': _xkbScanCodeMap,
      'WEB_LOGICAL_KEY_MAP': _webLogicalKeyMap,
      'WEB_PHYSICAL_KEY_MAP': _webPhysicalKeyMap,
      'WEB_NUMPAD_MAP': _webNumpadMap,
      'WEB_LOCATION_MAP': _webLocationMap,
      'WINDOWS_LOGICAL_KEY_MAP': _windowsKeyCodeMap,
      'WINDOWS_PHYSICAL_KEY_MAP': _windowsScanCodeMap,
      'WINDOWS_NUMPAD_MAP': _windowsNumpadMap,
    };
  }
}
