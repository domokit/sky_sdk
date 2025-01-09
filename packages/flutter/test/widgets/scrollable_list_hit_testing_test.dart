// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const List<int> items = <int>[0, 1, 2, 3, 4, 5];

void main() {
  testWidgets('Tap item after scroll - horizontal', (WidgetTester tester) async {
    final List<int> tapped = <int>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 50.0,
            child: ListView(
              dragStartBehavior: DragStartBehavior.down,
              itemExtent: 290.0,
              scrollDirection: Axis.horizontal,
              children:
                  items.map<Widget>((int item) {
                    return GestureDetector(
                      onTap: () {
                        tapped.add(item);
                      },
                      dragStartBehavior: DragStartBehavior.down,
                      child: Text('$item'),
                    );
                  }).toList(),
            ),
          ),
        ),
      ),
    );
    await tester.drag(find.text('2'), const Offset(-280.0, 0.0));
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //  -280..10  = 0
    //    10..300 = 1
    //   300..590 = 2
    //   590..880 = 3
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);
    expect(tapped, equals(<int>[]));
    await tester.tap(find.text('2'));
    expect(tapped, equals(<int>[2]));
  });

  testWidgets('Tap item after scroll - vertical', (WidgetTester tester) async {
    final List<int> tapped = <int>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 50.0,
            child: ListView(
              dragStartBehavior: DragStartBehavior.down,
              itemExtent: 290.0,
              children:
                  items.map<Widget>((int item) {
                    return GestureDetector(
                      onTap: () {
                        tapped.add(item);
                      },
                      dragStartBehavior: DragStartBehavior.down,
                      child: Text('$item'),
                    );
                  }).toList(),
            ),
          ),
        ),
      ),
    );
    await tester.drag(find.text('1'), const Offset(0.0, -280.0));
    await tester.pump(const Duration(seconds: 1));
    // screen is 600px tall, and has the following items:
    //  -280..10  = 0
    //    10..300 = 1
    //   300..590 = 2
    //   590..880 = 3
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);
    expect(tapped, equals(<int>[]));
    await tester.tap(find.text('1'));
    expect(tapped, equals(<int>[1]));
    await tester.tap(find.text('3'), warnIfMissed: false);
    expect(
      tapped,
      equals(<int>[1]),
    ); // the center of the third item is off-screen so it shouldn't get hit
  });

  testWidgets('Padding scroll anchor start', (WidgetTester tester) async {
    final List<int> tapped = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          itemExtent: 290.0,
          padding: const EdgeInsets.fromLTRB(5.0, 20.0, 15.0, 10.0),
          children:
              items.map<Widget>((int item) {
                return GestureDetector(
                  onTap: () {
                    tapped.add(item);
                  },
                  child: Text('$item'),
                );
              }).toList(),
        ),
      ),
    );
    await tester.tapAt(const Offset(200.0, 19.0));
    expect(tapped, equals(<int>[]));
    await tester.tapAt(const Offset(200.0, 21.0));
    expect(tapped, equals(<int>[0]));
    await tester.tapAt(const Offset(4.0, 400.0));
    expect(tapped, equals(<int>[0]));
    await tester.tapAt(const Offset(6.0, 400.0));
    expect(tapped, equals(<int>[0, 1]));
    await tester.tapAt(const Offset(800.0 - 14.0, 400.0));
    expect(tapped, equals(<int>[0, 1]));
    await tester.tapAt(const Offset(800.0 - 16.0, 400.0));
    expect(tapped, equals(<int>[0, 1, 1]));
  });

  testWidgets('Padding scroll anchor end', (WidgetTester tester) async {
    final List<int> tapped = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          itemExtent: 290.0,
          reverse: true,
          padding: const EdgeInsets.fromLTRB(5.0, 20.0, 15.0, 10.0),
          children:
              items.map<Widget>((int item) {
                return GestureDetector(
                  onTap: () {
                    tapped.add(item);
                  },
                  child: Text('$item'),
                );
              }).toList(),
        ),
      ),
    );
    await tester.tapAt(const Offset(200.0, 600.0 - 9.0));
    expect(tapped, equals(<int>[]));
    await tester.tapAt(const Offset(200.0, 600.0 - 11.0));
    expect(tapped, equals(<int>[0]));
    await tester.tapAt(const Offset(4.0, 200.0));
    expect(tapped, equals(<int>[0]));
    await tester.tapAt(const Offset(6.0, 200.0));
    expect(tapped, equals(<int>[0, 1]));
    await tester.tapAt(const Offset(800.0 - 14.0, 200.0));
    expect(tapped, equals(<int>[0, 1]));
    await tester.tapAt(const Offset(800.0 - 16.0, 200.0));
    expect(tapped, equals(<int>[0, 1, 1]));
  });

  testWidgets('Tap immediately following clamped overscroll', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/5709
    final List<int> tapped = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          itemExtent: 200.0,
          children:
              items.map<Widget>((int item) {
                return GestureDetector(
                  onTap: () {
                    tapped.add(item);
                  },
                  child: Text('$item'),
                );
              }).toList(),
        ),
      ),
    );

    await tester.fling(find.text('0'), const Offset(0.0, 400.0), 1000.0);
    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    expect(scrollable.position.pixels, equals(0.0));

    await tester.tapAt(const Offset(200.0, 100.0));
    expect(tapped, equals(<int>[0]));
  });
}
