// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome || safari || firefox')
library;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

// The body of this file is the same as ../../../../../testing/dart/semantics_test.dart
// Please keep them in sync.

void testMain() {
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
      final int action = 1 << index;
      expect(SemanticsAction.fromIndex(action), isNotNull);
      expect(SemanticsAction.fromIndex(action).toString(), startsWith('SemanticsAction.'));
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
