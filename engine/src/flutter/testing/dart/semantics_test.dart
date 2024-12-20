// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

// The body of this file is the same as ../../lib/web_ui/test/engine/semantics/semantics_api_test.dart
// Please keep them in sync.

void main() {
  // This must match the number of flags in lib/ui/semantics.dart
  const int numSemanticsFlags = 29;
  test('SemanticsFlag.values refers to all flags.', () async {
    expect(SemanticsFlag.values.length, equals(numSemanticsFlags));
    for (int index = 0; index < numSemanticsFlags; ++index) {
      final int flag = 1 << index;
      expect(SemanticsFlag.fromIndex(flag), isNotNull);
      expect(SemanticsFlag.fromIndex(flag).toString(), startsWith('SemanticsFlag.'));
    }
  });

  // This must match the number of actions in lib/ui/semantics.dart
  const int numSemanticsActions = 24;
  test('SemanticsAction.values refers to all actions.', () async {
    expect(SemanticsAction.values.length, equals(numSemanticsActions));
    for (int index = 0; index < numSemanticsActions; ++index) {
      final int flag = 1 << index;
      expect(SemanticsAction.fromIndex(flag), isNotNull);
      expect(SemanticsAction.fromIndex(flag).toString(), startsWith('SemanticsAction.'));
    }
  });

  test('SpellOutStringAttribute.toString', () async {
    expect(
      SpellOutStringAttribute(range: const TextRange(start: 2, end: 5)).toString(),
      'SpellOutStringAttribute(TextRange(start: 2, end: 5))',
    );
  });

  test('LocaleStringAttribute.toString', () async {
    expect(
      LocaleStringAttribute(
        range: const TextRange(start: 2, end: 5),
        locale: const Locale('test'),
      ).toString(),
      'LocaleStringAttribute(TextRange(start: 2, end: 5), test)',
    );
  });
}
