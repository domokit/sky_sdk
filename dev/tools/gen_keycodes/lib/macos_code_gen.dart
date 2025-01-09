// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'constants.dart';
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

/// Generates the key mapping for macOS, based on the information in the key
/// data structure given to it.
class MacOSCodeGenerator extends PlatformCodeGenerator {
  MacOSCodeGenerator(super.keyData, super.logicalData, this._layoutGoals);

  /// This generates the map of macOS key codes to physical keys.
  String get _scanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('macOS scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.macOSScanCode != null) {
        lines.add(
          entry.macOSScanCode!,
          '  @${toHex(entry.macOSScanCode)} : @${toHex(entry.usbHidCode)},  // ${entry.constantName}',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  String get _keyCodeToLogicalMap {
    final OutputLines<int> lines = OutputLines<int>('macOS keycode map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      zipStrict(entry.macOSKeyCodeValues, entry.macOSKeyCodeNames, (
        int macOSValue,
        String macOSName,
      ) {
        lines.add(
          macOSValue,
          '  @${toHex(macOSValue)} : @${toHex(entry.value, digits: 11)},  // $macOSName -> ${entry.constantName}',
        );
      });
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the mask values for the part of a key code that defines its plane.
  String get _maskConstants {
    final StringBuffer buffer = StringBuffer();
    const List<MaskConstant> maskConstants = <MaskConstant>[kValueMask, kUnicodePlane, kMacosPlane];
    for (final MaskConstant constant in maskConstants) {
      buffer.writeln(
        'const uint64_t k${constant.upperCamelName} = ${toHex(constant.value, digits: 11)};',
      );
    }
    return buffer.toString().trimRight();
  }

  /// This generates a map from the key code to a modifier flag.
  String get _keyToModifierFlagMap {
    final StringBuffer modifierKeyMap = StringBuffer();
    for (final String name in kModifiersOfInterest) {
      modifierKeyMap.writeln(
        '  @${toHex(logicalData.entryByName(name).macOSKeyCodeValues[0])} : @(kModifierFlag${lowerCamelToUpperCamel(name)}),',
      );
    }
    return modifierKeyMap.toString().trimRight();
  }

  /// This generates a map from the modifier flag to the key code.
  String get _modifierFlagToKeyMap {
    final StringBuffer modifierKeyMap = StringBuffer();
    for (final String name in kModifiersOfInterest) {
      modifierKeyMap.writeln(
        '  @(kModifierFlag${lowerCamelToUpperCamel(name)}) : @${toHex(logicalData.entryByName(name).macOSKeyCodeValues[0])},',
      );
    }
    return modifierKeyMap.toString().trimRight();
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

  final Map<String, bool> _layoutGoals;
  String get _layoutGoalsString {
    final OutputLines<int> lines = OutputLines<int>('macOS layout goals');
    _layoutGoals.forEach((String name, bool mandatory) {
      final PhysicalKeyEntry physicalEntry = keyData.entryByName(name);
      final LogicalKeyEntry logicalEntry = logicalData.entryByName(name);
      final String line =
          'LayoutGoal{'
          '${toHex(physicalEntry.macOSScanCode, digits: 2)}, '
          '${toHex(logicalEntry.value, digits: 2)}, '
          '${mandatory ? 'true' : 'false'}'
          '},';
      lines.add(
        logicalEntry.value,
        '    ${line.padRight(32)}'
        '// ${logicalEntry.name}',
      );
    });
    return lines.sortedJoin().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'macos_key_code_map_cc.tmpl');

  @override
  String outputPath(String platform) => path.join(
    PlatformCodeGenerator.engineRoot,
    'shell',
    'platform',
    'darwin',
    'macos',
    'framework',
    'Source',
    'KeyCodeMap.g.mm',
  );

  @override
  Map<String, String> mappings() {
    // There is no macOS keycode map since macOS uses keycode to represent a physical key.
    // The LogicalKeyboardKey is generated by raw_keyboard_macos.dart from the unmodified characters
    // from NSEvent.
    return <String, String>{
      'MACOS_SCAN_CODE_MAP': _scanCodeMap,
      'MACOS_KEYCODE_LOGICAL_MAP': _keyCodeToLogicalMap,
      'MASK_CONSTANTS': _maskConstants,
      'KEYCODE_TO_MODIFIER_FLAG_MAP': _keyToModifierFlagMap,
      'MODIFIER_FLAG_TO_KEYCODE_MAP': _modifierFlagToKeyMap,
      'SPECIAL_KEY_CONSTANTS': _specialKeyConstants,
      'LAYOUT_GOALS': _layoutGoalsString,
    };
  }
}
