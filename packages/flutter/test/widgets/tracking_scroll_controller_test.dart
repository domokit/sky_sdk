// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('TrackingScrollController saves offset', (WidgetTester tester) async {
    final TrackingScrollController controller = TrackingScrollController();
    addTearDown(controller.dispose);
    const double listItemHeight = 100.0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          itemBuilder: (BuildContext context, int index) {
            return ListView(
              controller: controller,
              children:
                  List<Widget>.generate(
                    10,
                    (int i) => SizedBox(height: listItemHeight, child: Text('Page$index-Item$i')),
                  ).toList(),
            );
          },
        ),
      ),
    );

    expect(find.text('Page0-Item1'), findsOneWidget);
    expect(find.text('Page1-Item1'), findsNothing);
    expect(find.text('Page2-Item0'), findsNothing);
    expect(find.text('Page2-Item1'), findsNothing);

    controller.jumpTo(listItemHeight + 10);
    await tester.pumpAndSettle();

    await tester.fling(find.text('Page0-Item1'), const Offset(-100.0, 0.0), 10000.0);
    await tester.pumpAndSettle();

    expect(find.text('Page0-Item1'), findsNothing);
    expect(find.text('Page1-Item1'), findsOneWidget);
    expect(find.text('Page2-Item0'), findsNothing);
    expect(find.text('Page2-Item1'), findsNothing);

    await tester.fling(find.text('Page1-Item1'), const Offset(-100.0, 0.0), 10000.0);
    await tester.pumpAndSettle();

    expect(find.text('Page0-Item1'), findsNothing);
    expect(find.text('Page1-Item1'), findsNothing);
    expect(find.text('Page2-Item0'), findsNothing);
    expect(find.text('Page2-Item1'), findsOneWidget);

    await tester.pumpWidget(const Text('Another page', textDirection: TextDirection.ltr));

    expect(controller.initialScrollOffset, 0.0);
  });

  testWidgets('TrackingScrollController saves offset', (WidgetTester tester) async {
    int attach = 0;
    int detach = 0;
    final TrackingScrollController controller = TrackingScrollController(
      onAttach: (_) {
        attach++;
      },
      onDetach: (_) {
        detach++;
      },
    );
    addTearDown(controller.dispose);
    const double listItemHeight = 100.0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          itemBuilder: (BuildContext context, int index) {
            return ListView(
              controller: controller,
              children:
                  List<Widget>.generate(
                    10,
                    (int i) => SizedBox(height: listItemHeight, child: Text('Page$index-Item$i')),
                  ).toList(),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(attach, 1);
    expect(detach, 0);

    await tester.pumpWidget(Container());
    await tester.pumpAndSettle();

    expect(attach, 1);
    expect(detach, 1);
  });
}
