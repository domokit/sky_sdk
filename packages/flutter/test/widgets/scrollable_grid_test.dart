// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('GridView default control', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: GridView.count(crossAxisCount: 1)),
      ),
    );
  });

  // Tests https://github.com/flutter/flutter/issues/5522
  testWidgets('GridView displays correct children with nonzero padding', (
    WidgetTester tester,
  ) async {
    const EdgeInsets padding = EdgeInsets.fromLTRB(0.0, 100.0, 0.0, 0.0);

    final Widget testWidget = Directionality(
      textDirection: TextDirection.ltr,
      child: Align(
        child: SizedBox(
          height: 800.0,
          width: 300.0, // forces the grid children to be 300..300
          child: GridView.count(
            crossAxisCount: 1,
            padding: padding,
            children:
                List<Widget>.generate(10, (int index) {
                  return Text('$index', key: ValueKey<int>(index));
                }).toList(),
          ),
        ),
      ),
    );

    await tester.pumpWidget(testWidget);

    // screen is 600px high, and has the following items:
    //   100..400 = 0
    //   400..700 = 1
    await tester.pump();
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);

    await tester.drag(find.text('1'), const Offset(0.0, -500.0));
    await tester.pump();
    //  -100..300 = 1
    //   300..600 = 2
    //   600..600 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    await tester.drag(find.text('1'), const Offset(0.0, 150.0));
    await tester.pump();
    // Child '0' is now back onscreen, but by less than `padding.top`.
    //  -250..050 = 0
    //   050..450 = 1
    //   450..750 = 2
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);
  });

  testWidgets('GridView.count() fixed itemExtent, scroll to end, append, scroll', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/9506
    Widget buildFrame(int itemCount) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          crossAxisCount: itemCount,
          children: List<Widget>.generate(itemCount, (int index) {
            return SizedBox(height: 200.0, child: Text('item $index'));
          }),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(3));
    expect(find.text('item 0'), findsOneWidget);
    expect(find.text('item 1'), findsOneWidget);
    expect(find.text('item 2'), findsOneWidget);

    await tester.pumpWidget(buildFrame(4));
    final TestGesture gesture = await tester.startGesture(const Offset(0.0, 300.0));
    await gesture.moveBy(const Offset(0.0, -200.0));
    await tester.pumpAndSettle();
    expect(find.text('item 3'), findsOneWidget);
  });
}
