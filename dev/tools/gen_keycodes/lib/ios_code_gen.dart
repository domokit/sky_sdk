// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'constants.dart';
import 'data.dart';
import 'logical_key_data.dart';
import 'physical_key_data.dart';
import 'utils.dart';

const List<String> kModifiersOfInterest = <String>[
  'ShiftLeft',
  'ShiftRight',
  'ControlLeft',
  'ControlRight',
  'AltLeft',
  'AltRight',
  'MetaLeft',
  'MetaRight',
];

// The name of keys that require special attention.
const List<String> kSpecialPhysicalKeys = <String>['CapsLock'];
const List<String> kSpecialLogicalKeys = <String>['CapsLock'];

/// Generates the key mapping for iOS, based on the information in the key
/// data structure given to it.
class IOSCodeGenerator extends PlatformCodeGenerator {
  IOSCodeGenerator(super.keyData, super.logicalData);

  /// This generates the map of iOS key codes to physical keys.
  String get _scanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('iOS scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.iOSScanCode != null) {
        lines.add(
          entry.iOSScanCode!,
          '    {${toHex(entry.iOSScanCode)}, ${toHex(entry.usbHidCode)}},  // ${entry.constantName}',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  Iterable<PhysicalKeyEntry> get _functionKeyData {
    final RegExp functionKeyRe = RegExp(r'^f[0-9]+$');
    return keyData.entries.where((PhysicalKeyEntry entry) {
      return functionKeyRe.hasMatch(entry.constantName);
    });
  }

  String get _functionKeys {
    final StringBuffer result = StringBuffer();
    for (final PhysicalKeyEntry entry in _functionKeyData) {
      result.writeln('    ${toHex(entry.iOSScanCode)},  // ${entry.constantName}');
    }
    return result.toString().trimRight();
  }

  String get _keyCodeToLogicalMap {
    final OutputLines<int> lines = OutputLines<int>('iOS keycode map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      zipStrict(entry.iOSKeyCodeValues, entry.iOSKeyCodeNames, (int iOSValue, String iOSName) {
        lines.add(
          iOSValue,
          '    {${toHex(iOSValue)}, ${toHex(entry.value, digits: 11)}},  // $iOSName',
        );
      });
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the mask values for the part of a key code that defines its plane.
  String get _maskConstants {
    final StringBuffer buffer = StringBuffer();
    const List<MaskConstant> maskConstants = <MaskConstant>[kValueMask, kUnicodePlane, kIosPlane];
    for (final MaskConstant constant in maskConstants) {
      buffer.writeln('/**');
      buffer.write(wrapString(constant.description, prefix: ' * '));
      buffer.writeln(' */');
      buffer.writeln(
        'const uint64_t k${constant.upperCamelName} = ${toHex(constant.value, digits: 11)};',
      );
      buffer.writeln();
    }
    return buffer.toString().trimRight();
  }

  /// This generates a map from the key code to a modifier flag.
  String get _keyToModifierFlagMap {
    final StringBuffer modifierKeyMap = StringBuffer();
    for (final String name in kModifiersOfInterest) {
      final String line =
          '{${toHex(logicalData.entryByName(name).iOSKeyCodeValues[0])}, kModifierFlag${lowerCamelToUpperCamel(name)}},';
      modifierKeyMap.writeln('    ${line.padRight(42)}// $name');
    }
    return modifierKeyMap.toString().trimRight();
  }

  /// This generates a map from the modifier flag to the key code.
  String get _modifierFlagToKeyMap {
    final StringBuffer modifierKeyMap = StringBuffer();
    for (final String name in kModifiersOfInterest) {
      final String line =
          '{kModifierFlag${lowerCamelToUpperCamel(name)}, ${toHex(logicalData.entryByName(name).iOSKeyCodeValues[0])}},';
      modifierKeyMap.writeln('    ${line.padRight(42)}// $name');
    }
    return modifierKeyMap.toString().trimRight();
  }

  String get _specialKeyMapping {
    final OutputLines<int> lines = OutputLines<int>('iOS special key mapping');
    kIosSpecialKeyMapping.forEach((String key, String logicalName) {
      final int value = logicalData.entryByName(logicalName).value;
      lines.add(value, '  @"$key" : @(${toHex(value)}),');
    });
    return lines.join().trimRight();
  }

  /// This generates some keys that needs special attention.
  String get _specialKeyConstants {
    final StringBuffer specialKeyConstants = StringBuffer();
    for (final String keyName in kSpecialPhysicalKeys) {
      specialKeyConstants.writeln(
        'const uint64_t k${keyName}PhysicalKey = ${toHex(keyData.entryByName(keyName).usbHidCode)};',
      );
    }
    for (final String keyName in kSpecialLogicalKeys) {
      specialKeyConstants.writeln(
        'const uint64_t k${lowerCamelToUpperCamel(keyName)}LogicalKey = ${toHex(logicalData.entryByName(keyName).value)};',
      );
    }
    return specialKeyConstants.toString().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'ios_key_code_map_mm.tmpl');

  @override
  String outputPath(String platform) => path.join(
    PlatformCodeGenerator.engineRoot,
    'shell',
    'platform',
    'darwin',
    'ios',
    'framework',
    'Source',
    'KeyCodeMap.g.mm',
  );

  @override
  Map<String, String> mappings() {
    // There is no iOS keycode map since iOS uses keycode to represent a physical key.
    // The LogicalKeyboardKey is generated by raw_keyboard_ios.dart from the unmodified characters
    // from NSEvent.
    return <String, String>{
      'MASK_CONSTANTS': _maskConstants,
      'IOS_SCAN_CODE_MAP': _scanCodeMap,
      'IOS_KEYCODE_LOGICAL_MAP': _keyCodeToLogicalMap,
      'IOS_FUNCTION_KEY_SET': _functionKeys,
      'KEYCODE_TO_MODIFIER_FLAG_MAP': _keyToModifierFlagMap,
      'MODIFIER_FLAG_TO_KEYCODE_MAP': _modifierFlagToKeyMap,
      'SPECIAL_KEY_CONSTANTS': _specialKeyConstants,
      'SPECIAL_KEY_MAPPING': _specialKeyMapping,
    };
  }
}
