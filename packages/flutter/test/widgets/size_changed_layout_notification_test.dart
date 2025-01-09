// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SizeChangedLayoutNotification test', (WidgetTester tester) async {
    bool notified = false;

    await tester.pumpWidget(
      Center(
        child: NotificationListener<LayoutChangedNotification>(
          onNotification: (LayoutChangedNotification notification) {
            throw Exception('Should not reach this point.');
          },
          child: const SizeChangedLayoutNotifier(child: SizedBox(width: 100.0, height: 100.0)),
        ),
      ),
    );

    await tester.pumpWidget(
      Center(
        child: NotificationListener<LayoutChangedNotification>(
          onNotification: (LayoutChangedNotification notification) {
            expect(notification, isA<SizeChangedLayoutNotification>());
            notified = true;
            return true;
          },
          child: const SizeChangedLayoutNotifier(child: SizedBox(width: 200.0, height: 100.0)),
        ),
      ),
    );

    expect(notified, isTrue);
  });
}
