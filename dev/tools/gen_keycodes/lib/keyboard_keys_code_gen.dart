// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'constants.dart';
import 'logical_key_data.dart';
import 'physical_key_data.dart';
import 'utils.dart';

/// Given an [input] string, wraps the text at 80 characters and prepends each
/// line with the [prefix] string. Use for generated comments.
String _wrapString(String input) {
  return wrapString(input, prefix: '  /// ');
}

final List<MaskConstant> _maskConstants = <MaskConstant>[
  kValueMask,
  kPlaneMask,
  kUnicodePlane,
  kUnprintablePlane,
  kFlutterPlane,
  kStartOfPlatformPlanes,
  kAndroidPlane,
  kFuchsiaPlane,
  kIosPlane,
  kMacosPlane,
  kGtkPlane,
  kWindowsPlane,
  kWebPlane,
  kGlfwPlane,
];

class SynonymKeyInfo {
  SynonymKeyInfo(this.keys, this.name);

  final List<LogicalKeyEntry> keys;
  final String name;

  // Use the first item in the synonyms as a template for the ID to use.
  // It won't end up being the same value because it'll be in the pseudo-key
  // plane.
  LogicalKeyEntry get primaryKey => keys[0];
  String get constantName => upperCamelToLowerCamel(name);
}

/// Generates the keyboard_key.g.dart based on the information in the key data
/// structure given to it.
class KeyboardKeysCodeGenerator extends BaseCodeGenerator {
  KeyboardKeysCodeGenerator(super.keyData, super.logicalData);

  /// Gets the generated definitions of PhysicalKeyboardKeys.
  String get _physicalDefinitions {
    final OutputLines<int> lines = OutputLines<int>('Physical Key Definition');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      final String firstComment = _wrapString(
        'Represents the location of the '
        '"${entry.commentName}" key on a generalized keyboard.',
      );
      final String otherComments = _wrapString(
        'See the function '
        '[RawKeyEvent.physicalKey] for more information.',
      );
      lines.add(entry.usbHidCode, '''
$firstComment  ///
$otherComments  static const PhysicalKeyboardKey ${entry.constantName} = PhysicalKeyboardKey(${toHex(entry.usbHidCode)});
''');
    }
    return lines.sortedJoin().trimRight();
  }

  String get _physicalDebugNames {
    final OutputLines<int> lines = OutputLines<int>('Physical debug names');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      lines.add(entry.usbHidCode, '''
      ${toHex(entry.usbHidCode)}: '${entry.commentName}',''');
    }
    return lines.sortedJoin().trimRight();
  }

  /// Gets the generated definitions of LogicalKeyboardKeys.
  String get _logicalDefinitions {
    final OutputLines<int> lines = OutputLines<int>(
      'Logical debug names',
      behavior: DeduplicateBehavior.kSkip,
    );
    void printKey(int flutterId, String constantName, String commentName, {String? otherComments}) {
      final String firstComment = _wrapString(
        'Represents the logical "$commentName" key on the keyboard.',
      );
      otherComments ??= _wrapString(
        'See the function [RawKeyEvent.logicalKey] for more information.',
      );
      lines.add(flutterId, '''
$firstComment  ///
$otherComments  static const LogicalKeyboardKey $constantName = LogicalKeyboardKey(${toHex(flutterId, digits: 11)});
''');
    }

    for (final LogicalKeyEntry entry in logicalData.entries) {
      printKey(
        entry.value,
        entry.constantName,
        entry.commentName,
        otherComments: _otherComments(entry.name),
      );
    }
    return lines.sortedJoin().trimRight();
  }

  String? _otherComments(String name) {
    if (synonyms.containsKey(name)) {
      final Set<String> unionNames =
          synonyms[name]!.keys.map((LogicalKeyEntry entry) => entry.constantName).toSet();
      return _wrapString(
        'This key represents the union of the keys '
        '$unionNames when comparing keys. This key will never be generated '
        'directly, its main use is in defining key maps.',
      );
    }
    return null;
  }

  String get _logicalSynonyms {
    final StringBuffer result = StringBuffer();
    for (final SynonymKeyInfo synonymInfo in synonyms.values) {
      for (final LogicalKeyEntry key in synonymInfo.keys) {
        final LogicalKeyEntry synonym = logicalData.entryByName(synonymInfo.name);
        result.writeln('    ${key.constantName}: <LogicalKeyboardKey>{${synonym.constantName}},');
      }
    }
    return result.toString();
  }

  String get _logicalReverseSynonyms {
    final StringBuffer result = StringBuffer();
    for (final SynonymKeyInfo synonymInfo in synonyms.values) {
      final LogicalKeyEntry synonym = logicalData.entryByName(synonymInfo.name);
      final List<String> entries =
          synonymInfo.keys.map<String>((LogicalKeyEntry entry) => entry.constantName).toList();
      result.writeln('    ${synonym.constantName}: <LogicalKeyboardKey>{${entries.join(', ')}},');
    }
    return result.toString();
  }

  String get _logicalKeyLabels {
    final OutputLines<int> lines = OutputLines<int>(
      'Logical key labels',
      behavior: DeduplicateBehavior.kSkip,
    );
    for (final LogicalKeyEntry entry in logicalData.entries) {
      lines.add(entry.value, '''
    ${toHex(entry.value, digits: 11)}: '${entry.commentName}',''');
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of USB HID codes to physical keys.
  String get _predefinedHidCodeMap {
    final OutputLines<int> lines = OutputLines<int>('Physical key map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      lines.add(entry.usbHidCode, '    ${toHex(entry.usbHidCode)}: ${entry.constantName},');
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of Flutter key codes to logical keys.
  String get _predefinedKeyCodeMap {
    final OutputLines<int> lines = OutputLines<int>(
      'Logical key map',
      behavior: DeduplicateBehavior.kSkip,
    );
    for (final LogicalKeyEntry entry in logicalData.entries) {
      lines.add(entry.value, '    ${toHex(entry.value, digits: 11)}: ${entry.constantName},');
    }
    return lines.sortedJoin().trimRight();
  }

  String get _maskConstantVariables {
    final OutputLines<int> lines = OutputLines<int>(
      'Mask constants',
      behavior: DeduplicateBehavior.kKeep,
    );
    for (final MaskConstant constant in _maskConstants) {
      lines.add(constant.value, '''
${_wrapString(constant.description)}  ///
  /// This is used by platform-specific code to generate Flutter key codes.
  static const int ${constant.lowerCamelName} = ${toHex(constant.value, digits: 11)};
''');
    }
    return lines.join().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'keyboard_key.tmpl');

  @override
  Map<String, String> mappings() {
    return <String, String>{
      'LOGICAL_KEY_MAP': _predefinedKeyCodeMap,
      'LOGICAL_KEY_DEFINITIONS': _logicalDefinitions,
      'LOGICAL_KEY_SYNONYMS': _logicalSynonyms,
      'LOGICAL_KEY_REVERSE_SYNONYMS': _logicalReverseSynonyms,
      'LOGICAL_KEY_KEY_LABELS': _logicalKeyLabels,
      'PHYSICAL_KEY_MAP': _predefinedHidCodeMap,
      'PHYSICAL_KEY_DEFINITIONS': _physicalDefinitions,
      'PHYSICAL_KEY_DEBUG_NAMES': _physicalDebugNames,
      'MASK_CONSTANTS': _maskConstantVariables,
    };
  }

  late final Map<String, SynonymKeyInfo> synonyms = Map<String, SynonymKeyInfo>.fromEntries(
    LogicalKeyData.synonyms.entries.map((MapEntry<String, List<String>> synonymDefinition) {
      final List<LogicalKeyEntry> entries =
          synonymDefinition.value.map((String name) => logicalData.entryByName(name)).toList();
      return MapEntry<String, SynonymKeyInfo>(
        synonymDefinition.key,
        SynonymKeyInfo(entries, synonymDefinition.key),
      );
    }),
  );
}
