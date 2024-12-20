// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Scrollable scaled up', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Transform.scale(
          scale: 2.0,
          child: Center(
            child: SizedBox(
              width: 200,
              child: ListView.builder(
                controller: controller,
                cacheExtent: 0.0,
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    height: 100.0,
                    color: index.isEven ? Colors.blue : Colors.red,
                    child: Text('Tile $index'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    expect(controller.offset, 0.0);

    await tester.drag(find.byType(ListView), const Offset(0.0, -100.0));
    await tester.pump();
    expect(controller.offset, 50.0); // 100.0 / 2.0

    await tester.drag(find.byType(ListView), const Offset(80.0, -70.0));
    await tester.pump();
    expect(controller.offset, 85.0); // 50.0 + (70.0 / 2)

    await tester.drag(find.byType(ListView), const Offset(100.0, 0.0));
    await tester.pump();
    expect(controller.offset, 85.0);

    await tester.drag(find.byType(ListView), const Offset(0.0, 85.0));
    await tester.pump();
    expect(controller.offset, 42.5); // 85.0 - (85.0 / 2)
  });

  testWidgets('Scrollable scaled down', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Transform.scale(
          scale: 0.5,
          child: Center(
            child: SizedBox(
              width: 200,
              child: ListView.builder(
                controller: controller,
                cacheExtent: 0.0,
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    height: 100.0,
                    color: index.isEven ? Colors.blue : Colors.red,
                    child: Text('Tile $index'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    expect(controller.offset, 0.0);

    await tester.drag(find.byType(ListView), const Offset(0.0, -100.0));
    await tester.pump();
    expect(controller.offset, 200.0); // 100.0 * 2.0

    await tester.drag(find.byType(ListView), const Offset(80.0, -70.0));
    await tester.pump();
    expect(controller.offset, 340.0); // 200.0 + (70.0 * 2)

    await tester.drag(find.byType(ListView), const Offset(100.0, 0.0));
    await tester.pump();
    expect(controller.offset, 340.0);

    await tester.drag(find.byType(ListView), const Offset(0.0, 170.0));
    await tester.pump();
    expect(controller.offset, 0.0); // 340.0 - (170.0 * 2)
  });

  testWidgets('Scrollable rotated 90 degrees', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Transform.rotate(
          angle: math.pi / 2,
          child: Center(
            child: SizedBox(
              width: 200,
              child: ListView.builder(
                controller: controller,
                cacheExtent: 0.0,
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    height: 100.0,
                    color: index.isEven ? Colors.blue : Colors.red,
                    child: Text('Tile $index'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    expect(controller.offset, 0.0);

    await tester.drag(find.byType(ListView), const Offset(100.0, 0.0));
    await tester.pump();
    expect(controller.offset, 100.0);

    await tester.drag(find.byType(ListView), const Offset(0.0, -100.0));
    await tester.pump();
    expect(controller.offset, 100.0);

    await tester.drag(find.byType(ListView), const Offset(-70.0, -50.0));
    await tester.pump();
    expect(controller.offset, 30.0); // 100.0 - 70.0
  });

  testWidgets('Perspective transform on scrollable', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Transform(
          transform:
              Matrix4.identity()
                ..setEntry(3, 2, 0.001)
                ..rotateX(math.pi / 4),
          child: Center(
            child: SizedBox(
              width: 200,
              child: ListView.builder(
                controller: controller,
                cacheExtent: 0.0,
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    height: 100.0,
                    color: index.isEven ? Colors.blue : Colors.red,
                    child: Text('Tile $index'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    expect(controller.offset, 0.0);

    // We want to test that the point in the ListView that the finger touches
    // on the screen stays under the finger as the finger scrolls the ListView
    // in vertical direction. For this, we pick a point in the ListView (here
    // the center of Tile 5) and calculate its position in the coordinate space
    // of the screen. We then place our finger on that point and drag that
    // point up in vertical direction. After the scroll activity is done,
    // we verify that - in the coordinate space of the screen (!) - the point
    // has moved the same distance as the finger. Due to the perspective
    // transform the point will have moved more distance in the *local*
    // coordinate system of the ListView.

    // Calculate where the center of Tile 5 is located in the coordinate
    // space of the screen. We cannot use `tester.getCenter` because it
    // does not properly remove the perspective component from the transform
    // to give us the place on the screen at which we need to touch the screen
    // to have the center of Tile 5 directly under our finger.
    final RenderBox tile5 = tester.renderObject(find.text('Tile 5'));
    final Offset pointOnScreenStart = MatrixUtils.transformPoint(
      PointerEvent.removePerspectiveTransform(tile5.getTransformTo(null)),
      tile5.size.center(Offset.zero),
    );

    // Place the finger on the tracked point and move the finger upwards for
    // 50 pixels to scroll the ListView (the ListView's scroll offset will
    // move more then 50 pixels due to the perspective transform).
    await tester.dragFrom(pointOnScreenStart, const Offset(0.0, -50.0));
    await tester.pump();

    // Get the new position of the tracked point in the screen's coordinate
    // system.
    final Offset pointOnScreenEnd = MatrixUtils.transformPoint(
      PointerEvent.removePerspectiveTransform(tile5.getTransformTo(null)),
      tile5.size.center(Offset.zero),
    );

    // The tracked point (in the coordinate space of the screen) and the finger
    // should have moved the same vertical distance over the screen.
    expect(pointOnScreenStart.dy - pointOnScreenEnd.dy, within(distance: 0.00001, from: 50.0));

    // While the point traveled the same distance as the finger in the
    // coordinate space of the screen, the scroll view actually moved far more
    // pixels in its local coordinate system due to the perspective transform.
    expect(controller.offset, greaterThan(100));
  });
}
