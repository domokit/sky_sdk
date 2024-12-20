// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tab_controller/tab_controller.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify first tab is selected by default', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TabControllerExampleApp());

    final Tab firstTab = example.TabControllerExampleApp.tabs.first;

    expect(
      find.descendant(of: find.byType(TabBarView), matching: find.text('${firstTab.text} Tab')),
      findsOneWidget,
    );
  });

  testWidgets('Verify tabs can be changed', (WidgetTester tester) async {
    final List<String?> log = <String?>[];

    final DebugPrintCallback originalDebugPrint = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add(message);
    };

    await tester.pumpWidget(const example.TabControllerExampleApp());

    const List<Tab> tabs = example.TabControllerExampleApp.tabs;
    final List<Tab> tabsTraversalOrder = <Tab>[];

    // The traverse order is from the second tab from the start to the last,
    // and then from the second tab from the end to the first.
    tabsTraversalOrder.addAll(tabs.skip(1));
    tabsTraversalOrder.addAll(tabs.reversed.skip(1));

    for (final Tab tab in tabsTraversalOrder) {
      // Tap on the TabBar's tab to select it.
      await tester.tap(find.descendant(of: find.byType(TabBar), matching: find.text(tab.text!)));
      await tester.pumpAndSettle();

      expect(
        find.descendant(of: find.byType(TabBarView), matching: find.text('${tab.text} Tab')),
        findsOneWidget,
      );

      expect(log.length, equals(1));
      expect(log.last, equals('tab changed: ${tabs.indexOf(tab)}'));

      log.clear();
    }

    debugPrint = originalDebugPrint;
  });

  testWidgets('DefaultTabControllerListener throws when no DefaultTabController above', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      example.DefaultTabControllerListener(onTabChanged: (_) {}, child: const SizedBox.shrink()),
    );

    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);

    final FlutterError error = exception as FlutterError;
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   No DefaultTabController for DefaultTabControllerListener.\n'
        '   When creating a DefaultTabControllerListener, you must ensure\n'
        '   that there is a DefaultTabController above the\n'
        '   DefaultTabControllerListener.\n',
      ),
    );
  });
}
