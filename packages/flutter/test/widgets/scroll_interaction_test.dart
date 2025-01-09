// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Scroll flings twice in a row does not crash', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(children: <Widget>[Container(height: 100000.0)]),
      ),
    );

    final ScrollableState scrollable = tester.state<ScrollableState>(find.byType(Scrollable));

    expect(scrollable.position.pixels, equals(0.0));

    await tester.flingFrom(const Offset(200.0, 300.0), const Offset(0.0, -200.0), 500.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));

    expect(scrollable.position.pixels, greaterThan(0.0));

    final double oldOffset = scrollable.position.pixels;

    await tester.flingFrom(const Offset(200.0, 300.0), const Offset(0.0, -200.0), 500.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));

    expect(scrollable.position.pixels, greaterThan(oldOffset));
  });
}
