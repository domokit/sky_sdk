// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_widgets.dart';

void main() {
  testWidgets('simultaneously dispose a widget and end the scroll animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FlipWidget(
          left: ListView(children: List<Widget>.generate(250, (int i) => Text('$i'))),
          right: Container(),
        ),
      ),
    );

    await tester.fling(find.byType(ListView), const Offset(0.0, -200.0), 1000.0);
    await tester.pump();

    tester.state<FlipWidgetState>(find.byType(FlipWidget)).flip();
    await tester.pump(const Duration(hours: 5));
  });

  testWidgets(
    'Disposing a (nested) Scrollable while holding in overscroll does not crash',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/27707.

      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final Key outerContainer = GlobalKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Container(
              key: outerContainer,
              color: Colors.purple,
              width: 400.0,
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: SizedBox(
                  width: 500.0,
                  child: ListView.builder(
                    controller: controller,
                    itemBuilder: (BuildContext context, int index) {
                      return Container(
                        color: index.isEven ? Colors.red : Colors.green,
                        height: 200.0,
                        child: Text('Hello $index'),
                      );
                    },
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      // Go into overscroll.
      double lastScrollOffset;
      await tester.fling(find.text('Hello 0'), const Offset(0.0, 1000.0), 1000.0);
      await tester.pump(const Duration(milliseconds: 100));
      expect(lastScrollOffset = controller.offset, lessThan(0.0));

      // Reduce the overscroll a little, but don't let it go back to 0.0.
      await tester.pump(const Duration(milliseconds: 100));
      expect(controller.offset, greaterThan(lastScrollOffset));
      expect(controller.offset, lessThan(0.0));
      final double currentOffset = controller.offset;

      // Start a hold activity by putting one pointer down.
      final TestGesture gesture = await tester.startGesture(
        tester.getTopLeft(find.byKey(outerContainer)) + const Offset(50.0, 50.0),
      );
      await tester
          .pumpAndSettle(); // This shouldn't change the scroll offset because of the down event above.
      expect(controller.offset, currentOffset);

      // Dispose the scrollables while the finger is still down, this should not crash.
      await tester.pumpWidget(MaterialApp(home: Container()));
      await tester.pumpAndSettle();
      expect(controller.hasClients, isFalse);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );
}
