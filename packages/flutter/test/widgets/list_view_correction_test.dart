// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListView can handle shrinking top elements', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          cacheExtent: 0.0,
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 400.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
            SizedBox(height: 400.0, child: Text('4')),
            SizedBox(height: 400.0, child: Text('5')),
            SizedBox(height: 400.0, child: Text('6')),
          ],
        ),
      ),
    );

    controller.jumpTo(1000.0);
    await tester.pump();

    expect(tester.getTopLeft(find.text('4')).dy, equals(200.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          cacheExtent: 0.0,
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 200.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
            SizedBox(height: 400.0, child: Text('4')),
            SizedBox(height: 400.0, child: Text('5')),
            SizedBox(height: 400.0, child: Text('6')),
          ],
        ),
      ),
    );

    expect(controller.offset, equals(1000.0));
    expect(tester.getTopLeft(find.text('4')).dy, equals(200.0));

    controller.jumpTo(300.0);
    await tester.pump();

    expect(controller.offset, equals(300.0));
    expect(tester.getTopLeft(find.text('2')).dy, equals(100.0));

    controller.jumpTo(50.0);
    await tester.pump();

    expect(controller.offset, equals(0.0));
    expect(tester.getTopLeft(find.text('2')).dy, equals(200.0));
  });

  testWidgets('ListView can handle shrinking top elements with cache extent', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 400.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
            SizedBox(height: 400.0, child: Text('4')),
            SizedBox(height: 400.0, child: Text('5')),
            SizedBox(height: 400.0, child: Text('6')),
          ],
        ),
      ),
    );

    controller.jumpTo(1000.0);
    await tester.pump();

    expect(tester.getTopLeft(find.text('4')).dy, equals(200.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 200.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
            SizedBox(height: 400.0, child: Text('4')),
            SizedBox(height: 400.0, child: Text('5')),
            SizedBox(height: 400.0, child: Text('6')),
          ],
        ),
      ),
    );

    expect(controller.offset, equals(1000.0));
    expect(tester.getTopLeft(find.text('4')).dy, equals(200.0));

    controller.jumpTo(300.0);
    await tester.pump();

    expect(controller.offset, equals(250.0));
    expect(tester.getTopLeft(find.text('2')).dy, equals(-50.0));

    controller.jumpTo(50.0);
    await tester.pump();

    expect(controller.offset, equals(50.0));
    expect(tester.getTopLeft(find.text('2')).dy, equals(150.0));
  });

  testWidgets('ListView can handle inserts at 0', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 400.0, child: Text('0')),
            SizedBox(height: 400.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
          ],
        ),
      ),
    );
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);

    final Finder findItemA = find.descendant(of: find.byType(SizedBox), matching: find.text('A'));
    final Finder findItemB = find.descendant(of: find.byType(SizedBox), matching: find.text('B'));
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 10.0, child: Text('A')),
            SizedBox(height: 10.0, child: Text('B')),
            SizedBox(height: 400.0, child: Text('0')),
            SizedBox(height: 400.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
          ],
        ),
      ),
    );
    expect(find.text('A'), findsOneWidget);
    expect(find.text('B'), findsOneWidget);
    expect(tester.getTopLeft(findItemA).dy, 0.0);
    expect(tester.getBottomRight(findItemA).dy, 10.0);
    expect(tester.getTopLeft(findItemB).dy, 10.0);
    expect(tester.getBottomRight(findItemB).dy, 20.0);

    controller.jumpTo(1200.0);
    await tester.pump();
    expect(find.text('A'), findsNothing);
    expect(find.text('B'), findsNothing);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[
            SizedBox(height: 200.0, child: Text('A')),
            SizedBox(height: 200.0, child: Text('B')),
            SizedBox(height: 400.0, child: Text('0')),
            SizedBox(height: 400.0, child: Text('1')),
            SizedBox(height: 400.0, child: Text('2')),
            SizedBox(height: 400.0, child: Text('3')),
          ],
        ),
      ),
    );
    expect(find.text('A'), findsNothing);
    expect(find.text('B'), findsNothing);

    // Scrolling to 0 causes items A and B to underflow (extend below
    // scrollOffset 0) because their heights have grown from 10 - 200.
    // RenderSliver list corrects the scroll offset in this case. Only item
    // B will become visible and item B's bottom edge will still appear
    // where it was when its height was 10.0.
    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.text('B'), findsOneWidget);
    expect(controller.offset, greaterThan(0.0)); // RenderSliverList corrected the offset.
    expect(tester.getTopLeft(findItemB).dy, -180.0);
    expect(tester.getBottomRight(findItemB).dy, 20.0);

    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.text('A'), findsOneWidget);
    expect(find.text('B'), findsOneWidget);
    expect(tester.getTopLeft(findItemA).dy, 0.0);
    expect(tester.getBottomRight(findItemA).dy, 200.0);
    expect(tester.getTopLeft(findItemB).dy, 200.0);
    expect(tester.getBottomRight(findItemB).dy, 400.0);
  });
}
