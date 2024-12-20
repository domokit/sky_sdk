// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const List<int> items = <int>[0, 1, 2, 3, 4, 5];

Widget buildFrame() {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: ListView(
      itemExtent: 290.0,
      children:
          items.map<Widget>((int item) {
            return Text('$item');
          }).toList(),
    ),
  );
}

void main() {
  testWidgets('Drag vertically', (WidgetTester tester) async {
    await tester.pumpWidget(buildFrame());

    await tester.pump();
    await tester.drag(find.text('1'), const Offset(0.0, -300.0));
    await tester.pump();
    // screen is 600px high, and has the following items:
    //   -10..280 = 1
    //   280..570 = 2
    //   570..860 = 3
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    await tester.pump();
    await tester.drag(find.text('2'), const Offset(0.0, -290.0));
    await tester.pump();
    // screen is 600px high, and has the following items:
    //   -10..280 = 2
    //   280..570 = 3
    //   570..860 = 4
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);

    await tester.pump();
    await tester.drag(find.text('3'), const Offset(-300.0, 0.0));
    await tester.pump();
    // nothing should have changed
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsNothing);
  });

  testWidgets('Drag vertically', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          itemExtent: 290.0,
          padding: const EdgeInsets.only(top: 250.0),
          children:
              items.map<Widget>((int item) {
                return Text('$item');
              }).toList(),
        ),
      ),
    );

    await tester.pump();
    // screen is 600px high, and has the following items:
    //   250..540 = 0
    //   540..830 = 1
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);

    await tester.drag(find.text('0'), const Offset(0.0, -300.0));
    await tester.pump();
    // screen is 600px high, and has the following items:
    //   -50..240 = 0
    //   240..530 = 1
    //   530..820 = 2
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);
    expect(find.text('5'), findsNothing);
  });
}
