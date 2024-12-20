// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const List<int> items = <int>[0, 1, 2, 3, 4, 5];

Widget buildFrame({bool reverse = false, required TextDirection textDirection}) {
  return Directionality(
    textDirection: textDirection,
    child: Center(
      child: SizedBox(
        height: 50.0,
        child: ListView(
          itemExtent: 290.0,
          scrollDirection: Axis.horizontal,
          reverse: reverse,
          physics: const BouncingScrollPhysics(),
          children:
              items.map<Widget>((int item) {
                return Text('$item');
              }).toList(),
        ),
      ),
    ),
  );
}

void main() {
  testWidgets('Drag horizontally with scroll anchor at start (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(buildFrame(textDirection: TextDirection.ltr));

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('1'), const Offset(-300.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 1
    //   280..570 = 2
    //   570..860 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    // the center of item 3 is visible, so this works;
    // if item 3 was a bit wider, such that its center was past the 800px mark, this would fail,
    // because it wouldn't be hit tested when scrolling from its center, as drag() does.
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 2
    //   280..570 = 3
    //   570..860 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(0.0, -290.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // unchanged
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 3
    //   280..570 = 4
    //   570..860 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1));
    // at this point we can drag 60 pixels further before we hit the friction zone
    // then, every pixel we drag is equivalent to half a pixel of movement
    // to move item 3 entirely off screen therefore takes:
    //  60 + (290-60)*2 = 520 pixels
    // plus a couple more to be sure
    await tester.drag(find.text('3'), const Offset(-522.0, 0.0), touchSlopX: 0.0);
    await tester.pump(); // just after release
    // screen is 800px wide, and has the following items:
    //   -11..279 = 4
    //   279..569 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
    await tester.pump(const Duration(seconds: 1)); // a second after release
    // screen is 800px wide, and has the following items:
    //   -70..220 = 3
    //   220..510 = 4
    //   510..800 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildFrame(textDirection: TextDirection.ltr),
      duration: const Duration(seconds: 1),
    );
    await tester.drag(find.text('2'), const Offset(-280.0, 0.0), touchSlopX: 0.0);
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
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //  -280..10  = 1
    //    10..300 = 2
    //   300..590 = 3
    //   590..880 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);
  });

  testWidgets('Drag horizontally with scroll anchor at end (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(buildFrame(reverse: true, textDirection: TextDirection.ltr));

    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -70..220 = 2
    //   220..510 = 1
    //   510..800 = 0
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    await tester.drag(find.text('0'), const Offset(300.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -80..210 = 3
    //   230..520 = 2
    //   520..810 = 1
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    // the center of item 3 is visible, so this works;
    // if item 3 was a bit wider, such that its center was past the 800px mark, this would fail,
    // because it wouldn't be hit tested when scrolling from its center, as drag() does.
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 4
    //   280..570 = 3
    //   570..860 = 2
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(0.0, 290.0), touchSlopY: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // unchanged
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 5
    //   280..570 = 4
    //   570..860 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1));
    // at this point we can drag 60 pixels further before we hit the friction zone
    // then, every pixel we drag is equivalent to half a pixel of movement
    // to move item 3 entirely off screen therefore takes:
    //  60 + (290-60)*2 = 520 pixels
    // plus a couple more to be sure
    await tester.drag(find.text('4'), const Offset(522.0, 0.0), touchSlopX: 0.0);
    await tester.pump(); // just after release
    // screen is 800px wide, and has the following items:
    //   280..570 = 5
    //   570..860 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
    await tester.pump(const Duration(seconds: 1)); // a second after release
    // screen is 800px wide, and has the following items:
    //     0..290 = 5
    //   290..580 = 4
    //   580..870 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
  });

  testWidgets('Drag horizontally with scroll anchor at start (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(buildFrame(textDirection: TextDirection.rtl));

    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -70..220 = 2
    //   220..510 = 1
    //   510..800 = 0
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    await tester.drag(find.text('0'), const Offset(300.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -80..210 = 3
    //   230..520 = 2
    //   520..810 = 1
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    // the center of item 3 is visible, so this works;
    // if item 3 was a bit wider, such that its center was past the 800px mark, this would fail,
    // because it wouldn't be hit tested when scrolling from its center, as drag() does.
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 4
    //   280..570 = 3
    //   570..860 = 2
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(0.0, 290.0), touchSlopY: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // unchanged
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 5
    //   280..570 = 4
    //   570..860 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1));
    // at this point we can drag 60 pixels further before we hit the friction zone
    // then, every pixel we drag is equivalent to half a pixel of movement
    // to move item 3 entirely off screen therefore takes:
    //  60 + (290-60)*2 = 520 pixels
    // plus a couple more to be sure
    await tester.drag(find.text('4'), const Offset(522.0, 0.0), touchSlopX: 0.0);
    await tester.pump(); // just after release
    // screen is 800px wide, and has the following items:
    //   280..570 = 5
    //   570..860 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
    await tester.pump(const Duration(seconds: 1)); // a second after release
    // screen is 800px wide, and has the following items:
    //     0..290 = 5
    //   290..580 = 4
    //   580..870 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
  });

  testWidgets('Drag horizontally with scroll anchor at end (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(buildFrame(reverse: true, textDirection: TextDirection.rtl));

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('1'), const Offset(-300.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 1
    //   280..570 = 2
    //   570..860 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    // the center of item 3 is visible, so this works;
    // if item 3 was a bit wider, such that its center was past the 800px mark, this would fail,
    // because it wouldn't be hit tested when scrolling from its center, as drag() does.
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 2
    //   280..570 = 3
    //   570..860 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(0.0, -290.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // unchanged
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('3'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //   -10..280 = 3
    //   280..570 = 4
    //   570..860 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1));
    // at this point we can drag 60 pixels further before we hit the friction zone
    // then, every pixel we drag is equivalent to half a pixel of movement
    // to move item 3 entirely off screen therefore takes:
    //  60 + (290-60)*2 = 520 pixels
    // plus a couple more to be sure
    await tester.drag(find.text('3'), const Offset(-522.0, 0.0), touchSlopX: 0.0);
    await tester.pump(); // just after release
    // screen is 800px wide, and has the following items:
    //   -11..279 = 4
    //   279..569 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
    await tester.pump(const Duration(seconds: 1)); // a second after release
    // screen is 800px wide, and has the following items:
    //   -70..220 = 3
    //   220..510 = 4
    //   510..800 = 5
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildFrame(reverse: true, textDirection: TextDirection.rtl),
      duration: const Duration(seconds: 1),
    );
    await tester.drag(find.text('2'), const Offset(-280.0, 0.0), touchSlopX: 0.0);
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
    await tester.pump(const Duration(seconds: 1));
    await tester.drag(find.text('2'), const Offset(-290.0, 0.0), touchSlopX: 0.0);
    await tester.pump(const Duration(seconds: 1));
    // screen is 800px wide, and has the following items:
    //  -280..10  = 1
    //    10..300 = 2
    //   300..590 = 3
    //   590..880 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);
  });
}
