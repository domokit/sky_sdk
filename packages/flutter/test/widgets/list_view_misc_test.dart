// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const Key blockKey = Key('test');

void main() {
  testWidgets('Cannot scroll a non-overflowing block', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: blockKey,
          children: const <Widget>[
            SizedBox(
              height: 200.0, // less than 600, the height of the test area
              child: Text('Hello'),
            ),
          ],
        ),
      ),
    );

    final Offset middleOfContainer = tester.getCenter(find.text('Hello'));
    final Offset target = tester.getCenter(find.byKey(blockKey));
    final TestGesture gesture = await tester.startGesture(target);
    await gesture.moveBy(const Offset(0.0, -10.0));

    await tester.pump(const Duration(milliseconds: 1));

    expect(tester.getCenter(find.text('Hello')) == middleOfContainer, isTrue);

    await gesture.up();
  });

  testWidgets('Can scroll an overflowing block', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: blockKey,
          children: const <Widget>[
            SizedBox(
              height: 2000.0, // more than 600, the height of the test area
              child: Text('Hello'),
            ),
          ],
        ),
      ),
    );

    final Offset middleOfContainer = tester.getCenter(find.text('Hello'));
    expect(middleOfContainer.dx, equals(400.0));
    expect(middleOfContainer.dy, equals(1000.0));

    final Offset target = tester.getCenter(find.byKey(blockKey));
    final TestGesture gesture = await tester.startGesture(target);
    await gesture.moveBy(const Offset(0.0, -10.0));

    await tester.pump(); // redo layout

    expect(tester.getCenter(find.text('Hello')), isNot(equals(middleOfContainer)));

    await gesture.up();
  });

  testWidgets('ListView reverse', (WidgetTester tester) async {
    int first = 0;
    int second = 0;

    Widget buildBlock({bool reverse = false}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: UniqueKey(),
          reverse: reverse,
          children: <Widget>[
            GestureDetector(
              onTap: () {
                first += 1;
              },
              child: Container(
                height: 350.0, // more than half the height of the test area
                color: const Color(0xFF00FF00),
              ),
            ),
            GestureDetector(
              onTap: () {
                second += 1;
              },
              child: Container(
                height: 350.0, // more than half the height of the test area
                color: const Color(0xFF0000FF),
              ),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(buildBlock(reverse: true));

    const Offset target = Offset(200.0, 200.0);
    await tester.tapAt(target);
    expect(first, equals(0));
    expect(second, equals(1));

    await tester.pumpWidget(buildBlock());

    await tester.tapAt(target);
    expect(first, equals(1));
    expect(second, equals(1));
  });

  testWidgets('ListView controller', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    Widget buildBlock() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: const <Widget>[Text('A'), Text('B'), Text('C')],
        ),
      );
    }

    await tester.pumpWidget(buildBlock());
    expect(controller.offset, equals(0.0));
  });

  testWidgets('SliverBlockChildListDelegate.estimateMaxScrollOffset hits end', (
    WidgetTester tester,
  ) async {
    final SliverChildListDelegate delegate = SliverChildListDelegate(<Widget>[
      Container(),
      Container(),
      Container(),
      Container(),
      Container(),
    ]);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(slivers: <Widget>[SliverList(delegate: delegate)]),
      ),
    );

    final SliverMultiBoxAdaptorElement element = tester.element(
      find.byType(SliverList, skipOffstage: false),
    );

    final double maxScrollOffset = element.estimateMaxScrollOffset(
      null,
      firstIndex: 3,
      lastIndex: 4,
      leadingScrollOffset: 25.0,
      trailingScrollOffset: 26.0,
    );
    expect(maxScrollOffset, equals(26.0));
  });

  testWidgets('Resizing a ListView child restores scroll offset', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/9221
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(milliseconds: 200),
    );
    addTearDown(controller.dispose);

    // The overall height of the frame is (as ever) 600
    Widget buildFrame() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Column(
          children: <Widget>[
            Flexible(
              // The overall height of the ListView's contents is 500
              child: ListView(
                children: const <Widget>[
                  SizedBox(height: 150.0, child: Center(child: Text('top'))),
                  SizedBox(height: 200.0, child: Center(child: Text('middle'))),
                  SizedBox(height: 150.0, child: Center(child: Text('bottom'))),
                ],
              ),
            ),
            // If this widget's height is > 100 the ListView can scroll.
            SizeTransition(
              sizeFactor: controller.view,
              child: const SizedBox(height: 300.0, child: Text('keyboard')),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(find.text('top'), findsOneWidget);

    final ScrollPosition position = Scrollable.of(tester.element(find.text('middle'))).position;
    expect(position.viewportDimension, 600.0);
    expect(position.pixels, 0.0);

    // Animate the 'keyboard' height from 0 to 300
    controller.forward();
    await tester.pumpAndSettle();
    expect(position.viewportDimension, 300.0);

    // Scroll the ListView upwards
    position.jumpTo(200.0);
    await tester.pumpAndSettle();
    expect(position.pixels, 200.0);
    expect(find.text('top'), findsNothing);

    // Animate the 'keyboard' height back to 0. This causes the scroll
    // offset to return to 0.0
    controller.reverse();
    await tester.pumpAndSettle();
    expect(position.viewportDimension, 600.0);
    expect(position.pixels, 0.0);
    expect(find.text('top'), findsOneWidget);
  });
}
