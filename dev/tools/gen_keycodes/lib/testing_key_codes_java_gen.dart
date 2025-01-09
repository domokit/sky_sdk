// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import 'base_code_gen.dart';
import 'logical_key_data.dart';
import 'physical_key_data.dart';
import 'utils.dart';

String _toUpperSnake(String lowerCamel) {
  // Converts 'myTVFoo' to 'myTvFoo'.
  final String trueUpperCamel = lowerCamel.replaceAllMapped(RegExp(r'([A-Z]{3,})'), (Match match) {
    final String matched = match.group(1)!;
    return matched.substring(0, 1) +
        matched.substring(1, matched.length - 2).toLowerCase() +
        matched.substring(matched.length - 2, matched.length - 1);
  });
  // Converts 'myTvFoo' to 'MY_TV_FOO'.
  return trueUpperCamel
      .replaceAllMapped(RegExp(r'([A-Z])'), (Match match) => '_${match.group(1)!}')
      .toUpperCase();
}

/// Generates the common/testing/key_codes.h based on the information in the key
/// data structure given to it.
class KeyCodesJavaGenerator extends BaseCodeGenerator {
  KeyCodesJavaGenerator(super.keyData, super.logicalData);

  /// Gets the generated definitions of PhysicalKeyboardKeys.
  String get _physicalDefinitions {
    final OutputLines<int> lines = OutputLines<int>('Physical Key list');
    for (final PhysicalKeyEntry entry in keyData.entries) {
      lines.add(
        entry.usbHidCode,
        '''
  public static final long PHYSICAL_${_toUpperSnake(entry.constantName)} = ${toHex(entry.usbHidCode)}L;''',
      );
    }
    return lines.sortedJoin().trimRight();
  }

  /// Gets the generated definitions of PhysicalKeyboardKeys.
  String get _logicalDefinitions {
    final OutputLines<int> lines = OutputLines<int>(
      'Logical Key list',
      behavior: DeduplicateBehavior.kSkip,
    );
    for (final LogicalKeyEntry entry in logicalData.entries) {
      lines.add(
        entry.value,
        '''
  public static final long LOGICAL_${_toUpperSnake(entry.constantName)} = ${toHex(entry.value, digits: 11)}L;''',
      );
    }
    return lines.sortedJoin().trimRight();
  }

  @override
  String get templatePath => path.join(dataRoot, 'key_codes_java.tmpl');

  @override
  Map<String, String> mappings() {
    return <String, String>{
      'LOGICAL_KEY_DEFINITIONS': _logicalDefinitions,
      'PHYSICAL_KEY_DEFINITIONS': _physicalDefinitions,
    };
  }
}
