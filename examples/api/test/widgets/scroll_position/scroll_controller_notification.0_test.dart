// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_position/scroll_controller_notification.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can toggle between scroll notification types', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollNotificationDemo());

    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(find.text('Last notification: Null'), findsNothing);

    // Toggle to use NotificationListener
    await tester.tap(
      find.byWidgetPredicate((Widget widget) {
        return widget is Radio<bool> && !widget.value;
      }),
    );
    await tester.pumpAndSettle();

    expect(find.text('Last notification: Null'), findsOneWidget);
    await tester.drag(find.byType(CustomScrollView), const Offset(20.0, 20.0));
    await tester.pumpAndSettle();
    expect(find.text('Last notification: UserScrollNotification'), findsOneWidget);
  });
}
