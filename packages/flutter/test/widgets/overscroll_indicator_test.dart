// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

final Matcher doesNotOverscroll = isNot(paints..circle());

Future<void> slowDrag(WidgetTester tester, Offset start, Offset offset) async {
  final TestGesture gesture = await tester.startGesture(start);
  for (int index = 0; index < 10; index += 1) {
    await gesture.moveBy(offset);
    await tester.pump(const Duration(milliseconds: 20));
  }
  await gesture.up();
}

void main() {
  testWidgets('Overscroll indicator color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));

    expect(painter, doesNotOverscroll);

    // the scroll gesture from tester.scroll happens in zero time, so nothing should appear:
    await tester.drag(find.byType(Scrollable), const Offset(0.0, 100.0));
    expect(painter, doesNotOverscroll);
    await tester.pump(); // allow the ticker to register itself
    expect(painter, doesNotOverscroll);
    await tester.pump(const Duration(milliseconds: 100)); // animate
    expect(painter, doesNotOverscroll);

    final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
    await tester.pump(const Duration(milliseconds: 100)); // animate
    expect(painter, doesNotOverscroll);
    await gesture.up();
    expect(painter, doesNotOverscroll);

    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
    expect(painter, paints..circle(color: const Color(0x0DFFFFFF)));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  testWidgets('Nested scrollable', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GlowingOverscrollIndicator(
          axisDirection: AxisDirection.down,
          color: const Color(0x0DFFFFFF),
          notificationPredicate: (ScrollNotification notification) => notification.depth == 1,
          child: const SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: SizedBox(
              width: 600.0,
              child: CustomScrollView(
                slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
              ),
            ),
          ),
        ),
      ),
    );

    final RenderObject outerPainter = tester.renderObject(find.byType(CustomPaint).first);
    final RenderObject innerPainter = tester.renderObject(find.byType(CustomPaint).last);

    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
    expect(outerPainter, paints..circle());
    expect(innerPainter, paints..circle());
  });

  testWidgets('Overscroll indicator changes side when you drag on the other side', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));

    await slowDrag(tester, const Offset(400.0, 200.0), const Offset(0.0, 10.0));
    expect(painter, paints..circle(x: 400.0));
    await slowDrag(tester, const Offset(100.0, 200.0), const Offset(0.0, 10.0));
    expect(
      painter,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawCircle) {
          return false;
        }
        final Offset center = arguments[0] as Offset;
        if (center.dx < 400.0) {
          return true;
        }
        throw 'Dragging on left hand side did not overscroll on left hand side.';
      }),
    );
    await slowDrag(tester, const Offset(700.0, 200.0), const Offset(0.0, 10.0));
    expect(
      painter,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawCircle) {
          return false;
        }
        final Offset center = arguments[0] as Offset;
        if (center.dx > 400.0) {
          return true;
        }
        throw 'Dragging on right hand side did not overscroll on right hand side.';
      }),
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  testWidgets('Overscroll indicator changes side when you shift sides', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    final TestGesture gesture = await tester.startGesture(const Offset(300.0, 200.0));
    await gesture.moveBy(const Offset(0.0, 10.0));
    await tester.pump(const Duration(milliseconds: 20));
    double oldX = 0.0;
    for (int index = 0; index < 10; index += 1) {
      await gesture.moveBy(const Offset(50.0, 50.0));
      await tester.pump(const Duration(milliseconds: 20));
      expect(
        painter,
        paints..something((Symbol method, List<dynamic> arguments) {
          if (method != #drawCircle) {
            return false;
          }
          final Offset center = arguments[0] as Offset;
          if (center.dx <= oldX) {
            throw 'Sliding to the right did not make the center of the radius slide to the right.';
          }
          oldX = center.dx;
          return true;
        }),
      );
    }
    await gesture.up();

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  group("Flipping direction of scrollable doesn't change overscroll behavior", () {
    testWidgets('down', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            physics: AlwaysScrollableScrollPhysics(),
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
          ),
        ),
      );
      final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
      await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
      expect(
        painter,
        paints
          ..save()
          ..circle()
          ..restore()
          ..save()
          ..scale(y: -1.0)
          ..restore()
          ..restore(),
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(painter, doesNotOverscroll);
    });

    testWidgets('up', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            reverse: true,
            physics: AlwaysScrollableScrollPhysics(),
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
          ),
        ),
      );
      final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
      await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
      expect(
        painter,
        paints
          ..save()
          ..scale(y: -1.0)
          ..restore()
          ..save()
          ..circle()
          ..restore()
          ..restore(),
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(painter, doesNotOverscroll);
    });
  });

  testWidgets('Overscroll in both directions', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          physics: AlwaysScrollableScrollPhysics(),
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
    expect(painter, paints..circle());
    expect(
      painter,
      isNot(
        paints
          ..circle()
          ..circle(),
      ),
    );
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, -5.0));
    expect(
      painter,
      paints
        ..circle()
        ..circle(),
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  testWidgets('Overscroll ignored from alternate axis', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: ScrollConfiguration(
          behavior: TestScrollBehaviorNoGlow(),
          child: GlowingOverscrollIndicator(
            axisDirection: AxisDirection.right,
            color: Color(0xFF0000FF),
            child: CustomScrollView(
              physics: AlwaysScrollableScrollPhysics(),
              slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
            ),
          ),
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
    expect(painter, doesNotOverscroll);
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, -5.0));
    expect(painter, doesNotOverscroll);

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  testWidgets('Overscroll horizontally', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          scrollDirection: Axis.horizontal,
          physics: AlwaysScrollableScrollPhysics(),
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(5.0, 0.0));
    expect(
      painter,
      paints
        ..rotate(angle: math.pi / 2.0)
        ..circle()
        ..saveRestore(),
    );
    expect(
      painter,
      isNot(
        paints
          ..circle()
          ..circle(),
      ),
    );
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(-5.0, 0.0));
    expect(
      painter,
      paints
        ..rotate(angle: math.pi / 2.0)
        ..circle()
        ..rotate(angle: math.pi / 2.0)
        ..circle(),
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(painter, doesNotOverscroll);
  });

  testWidgets('Nested overscrolls do not throw exceptions', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          children: <Widget>[
            ListView(
              children: <Widget>[
                Container(width: 2000.0, height: 2000.0, color: const Color(0xFF00FF00)),
              ],
            ),
          ],
        ),
      ),
    );

    await tester.dragFrom(const Offset(100.0, 100.0), const Offset(0.0, 2000.0));
    await tester.pumpAndSettle();
  });

  testWidgets('Changing settings', (WidgetTester tester) async {
    RenderObject painter;

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: ScrollConfiguration(
          behavior: TestScrollBehavior1(),
          child: CustomScrollView(
            scrollDirection: Axis.horizontal,
            physics: AlwaysScrollableScrollPhysics(),
            reverse: true,
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
          ),
        ),
      ),
    );
    painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(5.0, 0.0));
    expect(
      painter,
      paints
        ..rotate(angle: math.pi / 2.0)
        ..circle(color: const Color(0x0A00FF00)),
    );
    expect(
      painter,
      isNot(
        paints
          ..circle()
          ..circle(),
      ),
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: ScrollConfiguration(
          behavior: TestScrollBehavior2(),
          child: CustomScrollView(
            scrollDirection: Axis.horizontal,
            physics: AlwaysScrollableScrollPhysics(),
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 20.0))],
          ),
        ),
      ),
    );
    painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(5.0, 0.0));
    expect(
      painter,
      paints
        ..rotate(angle: math.pi / 2.0)
        ..circle(color: const Color(0x0A0000FF))
        ..saveRestore(),
    );
    expect(
      painter,
      isNot(
        paints
          ..circle()
          ..circle(),
      ),
    );
  });

  testWidgets('CustomScrollView overscroll indicator works if there is sliver before center', (
    WidgetTester tester,
  ) async {
    final Key centerKey = UniqueKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ScrollConfiguration(
          behavior: const TestScrollBehavior2(),
          child: CustomScrollView(
            center: centerKey,
            physics: const AlwaysScrollableScrollPhysics(),
            slivers: <Widget>[
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('First sliver $index'),
                  childCount: 2,
                ),
              ),
              SliverList(
                key: centerKey,
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Second sliver $index'),
                  childCount: 5,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text('First sliver 1'), findsNothing);

    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 300.0));
    expect(find.text('First sliver 1'), findsOneWidget);
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    // The scroll offset and paint extend should cancel out each other.
    expect(
      painter,
      paints
        ..save()
        ..translate(y: 0.0)
        ..scale()
        ..circle(),
    );
  });

  testWidgets(
    'CustomScrollView overscroll indicator works well with [CustomScrollView.center] and [OverscrollIndicatorNotification.paintOffset]',
    (WidgetTester tester) async {
      final Key centerKey = UniqueKey();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ScrollConfiguration(
            behavior: const TestScrollBehavior2(),
            child: NotificationListener<OverscrollIndicatorNotification>(
              onNotification: (OverscrollIndicatorNotification notification) {
                if (notification.leading) {
                  notification.paintOffset = 50.0;
                }
                return false;
              },
              child: CustomScrollView(
                center: centerKey,
                physics: const AlwaysScrollableScrollPhysics(),
                slivers: <Widget>[
                  SliverList(
                    delegate: SliverChildBuilderDelegate(
                      (BuildContext context, int index) => Text('First sliver $index'),
                      childCount: 2,
                    ),
                  ),
                  SliverList(
                    key: centerKey,
                    delegate: SliverChildBuilderDelegate(
                      (BuildContext context, int index) => Text('Second sliver $index'),
                      childCount: 5,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      expect(find.text('First sliver 1'), findsNothing);

      await slowDrag(
        tester,
        const Offset(200.0, 200.0),
        const Offset(0.0, 5.0),
      ); // offset will be magnified ten times
      expect(find.text('First sliver 1'), findsOneWidget);
      final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
      // The OverscrollIndicator should respect the [OverscrollIndicatorNotification.paintOffset] setting.
      expect(
        painter,
        paints
          ..save()
          ..translate(y: 50.0)
          ..scale()
          ..circle(),
      );
    },
  );

  testWidgets('The OverscrollIndicator should not overflow the scrollable view edge', (
    WidgetTester tester,
  ) async {
    // Regressing test for https://github.com/flutter/flutter/issues/64149
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: NotificationListener<OverscrollIndicatorNotification>(
          onNotification: (OverscrollIndicatorNotification notification) {
            notification.paintOffset =
                50.0; // both the leading and trailing indicator have a 50.0 pixels offset.
            return false;
          },
          child: const CustomScrollView(
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
          ),
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
    expect(
      painter,
      paints
        ..save()
        ..translate(y: 50.0)
        ..scale()
        ..circle(),
    );
    // Reverse scroll (30 pixels), and the offset < notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, -30.0));
    await tester.pump();
    // The OverscrollIndicator should move with the CustomScrollView.
    expect(
      painter,
      paints
        ..save()
        ..translate(y: 50.0 - 30.0)
        ..scale()
        ..circle(),
    );

    // Reverse scroll (30+20 pixels) and offset == notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, -20.0));
    await tester.pump();
    expect(
      painter,
      paints
        ..save()
        ..translate(y: 50.0 - 50.0)
        ..scale()
        ..circle(),
    );

    // Reverse scroll (30+20+10 pixels) and offset > notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, -10.0));
    await tester.pump();
    // The OverscrollIndicator should not overflow the CustomScrollView's edge.
    expect(
      painter,
      paints
        ..save()
        ..translate(y: 50.0 - 50.0)
        ..scale()
        ..circle(),
    );

    await tester.pumpAndSettle(); // Finish the leading indicator.

    // trigger the trailing indicator
    await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, -200.0));
    expect(
      painter,
      paints
        ..scale(y: -1.0)
        ..save()
        ..translate(y: 50.0)
        ..scale()
        ..circle(),
    );

    // Reverse scroll (30 pixels), and the offset < notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, 30.0));
    await tester.pump();
    // The OverscrollIndicator should move with the CustomScrollView.
    expect(
      painter,
      paints
        ..scale(y: -1.0)
        ..save()
        ..translate(y: 50.0 - 30.0)
        ..scale()
        ..circle(),
    );

    // Reverse scroll (30+20 pixels) and offset == notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, 20.0));
    await tester.pump();
    expect(
      painter,
      paints
        ..scale(y: -1.0)
        ..save()
        ..translate(y: 50.0 - 50.0)
        ..scale()
        ..circle(),
    );

    // Reverse scroll (30+20+10 pixels) and offset > notification.paintOffset.
    await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, 10.0));
    await tester.pump();
    // The OverscrollIndicator should not overflow the CustomScrollView's edge.
    expect(
      painter,
      paints
        ..scale(y: -1.0)
        ..save()
        ..translate(y: 50.0 - 50.0)
        ..scale()
        ..circle(),
    );
  });

  group('[OverscrollIndicatorNotification.paintOffset] test', () {
    testWidgets('Leading', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: NotificationListener<OverscrollIndicatorNotification>(
            onNotification: (OverscrollIndicatorNotification notification) {
              if (notification.leading) {
                notification.paintOffset = 50.0;
              }
              return false;
            },
            child: const CustomScrollView(
              slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
            ),
          ),
        ),
      );
      final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
      await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, 5.0));
      // The OverscrollIndicator should respect the [OverscrollIndicatorNotification.paintOffset] setting.
      expect(
        painter,
        paints
          ..save()
          ..translate(y: 50.0)
          ..scale()
          ..circle(),
      );
      // Reverse scroll direction.
      await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, -30.0));
      await tester.pump();
      // The OverscrollIndicator should move with the CustomScrollView.
      expect(
        painter,
        paints
          ..save()
          ..translate(y: 50.0 - 30.0)
          ..scale()
          ..circle(),
      );
    });

    testWidgets('Trailing', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: NotificationListener<OverscrollIndicatorNotification>(
            onNotification: (OverscrollIndicatorNotification notification) {
              if (!notification.leading) {
                notification.paintOffset = 50.0;
              }
              return false;
            },
            child: const CustomScrollView(
              slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
            ),
          ),
        ),
      );
      final RenderObject painter = tester.renderObject(find.byType(CustomPaint));
      await tester.dragFrom(const Offset(200.0, 200.0), const Offset(200.0, -10000.0));
      await tester.pump();
      await slowDrag(tester, const Offset(200.0, 200.0), const Offset(0.0, -5.0));
      // The OverscrollIndicator should respect the [OverscrollIndicatorNotification.paintOffset] setting.
      expect(
        painter,
        paints
          ..scale(y: -1.0)
          ..save()
          ..translate(y: 50.0)
          ..scale()
          ..circle(),
      );
      // Reverse scroll direction.
      await tester.dragFrom(const Offset(200.0, 200.0), const Offset(0.0, 30.0));
      await tester.pump();
      // The OverscrollIndicator should move with the CustomScrollView.
      expect(
        painter,
        paints
          ..scale(y: -1.0)
          ..save()
          ..translate(y: 50.0 - 30.0)
          ..scale()
          ..circle(),
      );
    });
  });
}

class TestScrollBehavior1 extends ScrollBehavior {
  const TestScrollBehavior1();

  @override
  Widget buildOverscrollIndicator(BuildContext context, Widget child, ScrollableDetails details) {
    return GlowingOverscrollIndicator(
      axisDirection: details.direction,
      color: const Color(0xFF00FF00),
      child: child,
    );
  }
}

class TestScrollBehavior2 extends ScrollBehavior {
  const TestScrollBehavior2();

  @override
  Widget buildOverscrollIndicator(BuildContext context, Widget child, ScrollableDetails details) {
    return GlowingOverscrollIndicator(
      axisDirection: details.direction,
      color: const Color(0xFF0000FF),
      child: child,
    );
  }
}

class TestScrollBehaviorNoGlow extends ScrollBehavior {
  const TestScrollBehaviorNoGlow();

  @override
  Widget buildOverscrollIndicator(BuildContext context, Widget child, ScrollableDetails details) {
    return child;
  }
}
