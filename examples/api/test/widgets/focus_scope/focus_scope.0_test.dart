// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_scope/focus_scope.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  bool hasFocus(WidgetTester tester, IconData icon) =>
      tester
          .widget<IconButton>(
            find.ancestor(of: find.byIcon(icon), matching: find.byType(IconButton)),
          )
          .focusNode!
          .hasFocus;

  testWidgets('The focus is restricted to the foreground', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusScopeExampleApp());

    expect(find.text('FOREGROUND'), findsOne);
    expect(hasFocus(tester, Icons.menu), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(hasFocus(tester, Icons.menu), true);
  });

  testWidgets('The background can be focused', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusScopeExampleApp());

    expect(find.text('FOREGROUND'), findsOne);
    expect(hasFocus(tester, Icons.menu), true);

    await tester.tap(find.byIcon(Icons.menu));
    await tester.pumpAndSettle();

    expect(hasFocus(tester, Icons.close), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(hasFocus(tester, Icons.menu), false);
    expect(hasFocus(tester, Icons.close), false);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(hasFocus(tester, Icons.close), true);
  });
}
