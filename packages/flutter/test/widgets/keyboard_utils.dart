// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> sendKeyCombination(WidgetTester tester, SingleActivator activator) async {
  final List<LogicalKeyboardKey> modifiers = <LogicalKeyboardKey>[
    if (activator.control) LogicalKeyboardKey.control,
    if (activator.shift) LogicalKeyboardKey.shift,
    if (activator.alt) LogicalKeyboardKey.alt,
    if (activator.meta) LogicalKeyboardKey.meta,
  ];
  for (final LogicalKeyboardKey modifier in modifiers) {
    await tester.sendKeyDownEvent(modifier);
  }
  await tester.sendKeyDownEvent(activator.trigger);
  await tester.sendKeyUpEvent(activator.trigger);
  await tester.pump();
  for (final LogicalKeyboardKey modifier in modifiers.reversed) {
    await tester.sendKeyUpEvent(modifier);
  }
}
