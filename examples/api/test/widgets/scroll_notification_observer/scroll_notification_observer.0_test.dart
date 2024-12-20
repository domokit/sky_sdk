// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_notification_observer/scroll_notification_observer.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Scroll to top buttons appears when scrolling down', (WidgetTester tester) async {
    const String buttonText = 'Scroll to top';

    await tester.pumpWidget(const example.ScrollNotificationObserverApp());

    expect(find.byType(ScrollNotificationObserver), findsOneWidget);
    expect(find.text(buttonText), findsNothing);

    // Scroll down.
    await tester.drag(find.byType(ListView), const Offset(0.0, -300.0));
    await tester.pumpAndSettle();

    expect(find.text(buttonText), findsOneWidget);

    await tester.tap(find.text(buttonText));
    await tester.pumpAndSettle();

    expect(find.text(buttonText), findsNothing);
  });
}
