// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'constants.dart';
import 'logical_key_data.dart';
import 'physical_key_data.dart';
import 'utils.dart';

/// Generates the key mapping for GTK, based on the information in the key
/// data structure given to it.
class GtkCodeGenerator extends PlatformCodeGenerator {
  GtkCodeGenerator(
    super.keyData,
    super.logicalData,
    String modifierBitMapping,
    String lockBitMapping,
    this._layoutGoals,
  ) : _modifierBitMapping = parseMapOfListOfString(modifierBitMapping),
      _lockBitMapping = parseMapOfListOfString(lockBitMapping);

  /// This generates the map of XKB scan codes to Flutter physical keys.
  String get _xkbScanCodeMap {
    final OutputLines<int> lines = OutputLines<int>('GTK scancode map');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      if (entry.xKbScanCode != null) {
        lines.add(
          entry.xKbScanCode!,
          '    {${toHex(entry.xKbScanCode)}, ${toHex(entry.usbHidCode)}},  // ${entry.constantName}',
        );
      }
    }
    return lines.sortedJoin().trimRight();
  }

  /// This generates the map of GTK keyval codes to Flutter logical keys.
  String get _gtkKeyvalCodeMap {
    final OutputLines<int> lines = OutputLines<int>('GTK keyval map');
    for (final LogicalKeyEntry entry in logicalData.entries) {
      zipStrict(entry.gtkValues, entry.gtkNames, (int value, String name) {
        lines.add(value, '    {${toHex(value)}, ${toHex(entry.value, digits: 11)}},  // $name');
      });
    }
    return lines.sortedJoin().trimRight();
  }

  static String constructMapFromModToKeys(
    Map<String, List<String>> source,
    PhysicalKeyData physicalData,
    LogicalKeyData logicalData,
    String debugFunctionName,
  ) {
    final StringBuffer result = StringBuffer();
    source.forEach((String modifierBitName, List<String> keyNames) {
      assert(keyNames.length == 2 || keyNames.length == 3);
      final String primaryPhysicalName = keyNames[0];
      final String primaryLogicalName = keyNames[1];
      final String? secondaryLogicalName = keyNames.length == 3 ? keyNames[2] : null;
      final PhysicalKeyEntry primaryPhysical = physicalData.entryByName(primaryPhysicalName);
      final LogicalKeyEntry primaryLogical = logicalData.entryByName(primaryLogicalName);
      final LogicalKeyEntry? secondaryLogical =
          secondaryLogicalName == null ? null : logicalData.entryByName(secondaryLogicalName);
      if (secondaryLogical == null && secondaryLogicalName != null) {
        print(
          'Unrecognized secondary logical key $secondaryLogicalName specified for $debugFunctionName.',
        );
        return;
      }
      final String pad = secondaryLogical == null ? '' : '  ';
      result.writeln(
        '''

  data = g_new(FlKeyEmbedderCheckedKey, 1);
  g_hash_table_insert(table, GUINT_TO_POINTER(GDK_${modifierBitName}_MASK), data);
  data->is_caps_lock = ${primaryPhysicalName == 'CapsLock' ? 'true' : 'false'};
  data->primary_physical_key = ${toHex(primaryPhysical.usbHidCode, digits: 9)};$pad   // ${primaryPhysical.constantName}
  data->primary_logical_key = ${toHex(primaryLogical.value, digits: 11)};$pad  // ${primaryLogical.constantName}''',
      );
      if (secondaryLogical != null) {
        result.writeln(
          '''
  data->secondary_logical_key = ${toHex(secondaryLogical.value, digits: 11)};  // ${secondaryLogical.constantName}''',
        );
      }
    });
    return result.toString().trimRight();
  }

  String get _gtkModifierBitMap {
    return constructMapFromModToKeys(
      _modifierBitMapping,
      keyData,
      logicalData,
      'gtkModifierBitMap',
    );
  }

  final Map<String, List<String>> _modifierBitMapping;

  String get _gtkModeBitMap {
    return constructMapFromModToKeys(_lockBitMapping, keyData, logicalData, 'gtkModeBitMap');
  }

  final Map<String, List<String>> _lockBitMapping;

  final Map<String, bool> _layoutGoals;
  String get _layoutGoalsString {
    final OutputLines<int> lines = OutputLines<int>('GTK layout goals');
    _layoutGoals.forEach((String name, bool mandatory) {
      final PhysicalKeyEntry physicalEntry = keyData.entryByName(name);
      final LogicalKeyEntry logicalEntry = logicalData.entryByName(name);
      final String line =
          'LayoutGoal{'
          '${toHex(physicalEntry.xKbScanCode, digits: 2)}, '
          '${toHex(logicalEntry.value, digits: 2)}, '
          '${mandatory ? 'true' : 'false'}'
          '},';
      lines.add(
        logicalEntry.value,
        '    ${line.padRight(39)}'
        '// ${logicalEntry.name}',
      );
    });
    return lines.sortedJoin().trimRight();
  }

  /// This generates the mask values for the part of a key code that defines its plane.
  String get _maskConstants {
    final StringBuffer buffer = StringBuffer();
    const List<MaskConstant> maskConstants = <MaskConstant>[kValueMask, kUnicodePlane, kGtkPlane];
    for (final MaskConstant constant in maskConstants) {
      buffer.writeln(
        'const uint64_t k${constant.upperCamelName} = ${toHex(constant.value, digits: 11)};',
      );
    }
    return buffer.toString().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'gtk_key_mapping_cc.tmpl');

  @override
  String outputPath(String platform) =>
      path.join(PlatformCodeGenerator.engineRoot, 'shell', 'platform', 'linux', 'key_mapping.g.cc');

  @override
  Map<String, String> mappings() {
    return <String, String>{
      'XKB_SCAN_CODE_MAP': _xkbScanCodeMap,
      'GTK_KEYVAL_CODE_MAP': _gtkKeyvalCodeMap,
      'GTK_MODIFIER_BIT_MAP': _gtkModifierBitMap,
      'GTK_MODE_BIT_MAP': _gtkModeBitMap,
      'MASK_CONSTANTS': _maskConstants,
      'LAYOUT_GOALS': _layoutGoalsString,
    };
  }
}
