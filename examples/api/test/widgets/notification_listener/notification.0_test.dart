// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/notification_listener/notification.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    final DebugPrintCallback originalDebugPrint = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {};
    await tester.pumpWidget(const example.NotificationExampleApp());

    expect(find.byType(NestedScrollView), findsOne);
    expect(find.widgetWithText(SliverAppBar, 'Notification Sample'), findsOne);
    expect(find.byType(TabBarView), findsOne);
    expect(find.widgetWithText(Tab, 'Months'), findsOne);
    expect(find.widgetWithText(Tab, 'Days'), findsOne);
    expect(find.widgetWithText(ListTile, 'January'), findsOne);
    expect(find.widgetWithText(ListTile, 'February'), findsOne);
    expect(find.widgetWithText(ListTile, 'March'), findsOne);

    await tester.tap(find.text('Days'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(ListTile, 'Sunday'), findsOne);
    expect(find.widgetWithText(ListTile, 'Monday'), findsOne);
    expect(find.widgetWithText(ListTile, 'Tuesday'), findsOne);

    debugPrint = originalDebugPrint;
  });

  testWidgets('Scrolling the scroll view triggers the notification', (WidgetTester tester) async {
    final DebugPrintCallback originalDebugPrint = debugPrint;
    final List<String> logs = <String>[];
    debugPrint = (String? message, {int? wrapWidth}) {
      logs.add(message!);
    };
    await tester.pumpWidget(const example.NotificationExampleApp());

    final TestPointer testPointer = TestPointer(1, PointerDeviceKind.mouse);
    testPointer.hover(tester.getCenter(find.byType(NestedScrollView)));
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 10)));
    expect(
      logs,
      orderedEquals(const <String>[
        'Scrolling has started',
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has ended',
      ]),
    );
    debugPrint = originalDebugPrint;
  });

  testWidgets('Changing tabs triggers the notification', (WidgetTester tester) async {
    final DebugPrintCallback originalDebugPrint = debugPrint;
    final List<String> logs = <String>[];
    debugPrint = (String? message, {int? wrapWidth}) {
      logs.add(message!);
    };
    await tester.pumpWidget(const example.NotificationExampleApp());

    final TestPointer testPointer = TestPointer(1, PointerDeviceKind.mouse);
    testPointer.hover(tester.getCenter(find.byType(NestedScrollView)));
    await tester.sendEventToBinding(testPointer.scroll(const Offset(500, 0)));
    expect(
      logs,
      orderedEquals(const <String>[
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
      ]),
    );
    await tester.pumpAndSettle();
    expect(
      logs,
      orderedEquals(const <String>[
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
        'Scrolling has ended',
      ]),
    );

    await tester.tap(find.text('Months'));
    await tester.pump();

    expect(
      logs,
      orderedEquals(const <String>[
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
      ]),
    );

    await tester.pumpAndSettle();

    expect(
      logs,
      orderedEquals(const <String>[
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
        'Scrolling has ended',
        'Scrolling has started',
        'Scrolling has ended',
      ]),
    );

    debugPrint = originalDebugPrint;
  });
}
