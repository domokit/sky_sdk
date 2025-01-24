// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart' show TestClipPaintingContext;
import 'states.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/100451
  testWidgets('GridView.builder respects findChildIndexCallback', (WidgetTester tester) async {
    bool finderCalled = false;
    int itemCount = 7;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return GridView.builder(
              itemCount: itemCount,
              itemBuilder:
                  (BuildContext _, int index) => Container(key: Key('$index'), height: 2000.0),
              findChildIndexCallback: (Key key) {
                finderCalled = true;
                return null;
              },
              gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 4),
            );
          },
        ),
      ),
    );
    expect(finderCalled, false);

    // Trigger update.
    stateSetter(() => itemCount = 77);
    await tester.pump();

    expect(finderCalled, true);
  });

  testWidgets('Empty GridView', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(dragStartBehavior: DragStartBehavior.down, crossAxisCount: 4),
      ),
    );
  });

  testWidgets('GridView.count control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          dragStartBehavior: DragStartBehavior.down,
          crossAxisCount: 4,
          children:
              kStates.map<Widget>((String state) {
                return GestureDetector(
                  dragStartBehavior: DragStartBehavior.down,
                  onTap: () {
                    log.add(state);
                  },
                  child: ColoredBox(color: const Color(0xFF0000FF), child: Text(state)),
                );
              }).toList(),
        ),
      ),
    );

    expect(tester.getSize(find.text('Arkansas')), equals(const Size(200.0, 200.0)));

    for (int i = 0; i < 8; ++i) {
      await tester.tap(find.text(kStates[i]));
      expect(log, equals(<String>[kStates[i]]));
      log.clear();
    }

    expect(find.text(kStates[12]), findsNothing);
    expect(find.text('Nevada'), findsNothing);

    await tester.drag(find.text('Arkansas'), const Offset(0.0, -200.0));
    await tester.pump();

    for (int i = 0; i < 4; ++i) {
      expect(find.text(kStates[i]), findsNothing);
    }

    for (int i = 4; i < 12; ++i) {
      await tester.tap(find.text(kStates[i]));
      expect(log, equals(<String>[kStates[i]]));
      log.clear();
    }

    await tester.drag(find.text('Delaware'), const Offset(0.0, -4000.0));
    await tester.pump();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Pennsylvania'), findsNothing);

    expect(tester.getCenter(find.text('Tennessee')), equals(const Offset(300.0, 100.0)));

    await tester.tap(find.text('Tennessee'));
    expect(log, equals(<String>['Tennessee']));
    log.clear();

    await tester.drag(find.text('Tennessee'), const Offset(0.0, 200.0));
    await tester.pump();

    await tester.tap(find.text('Tennessee'));
    expect(log, equals(<String>['Tennessee']));
    log.clear();

    await tester.tap(find.text('Pennsylvania'));
    expect(log, equals(<String>['Pennsylvania']));
    log.clear();
  });

  testWidgets('GridView.extent control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.extent(
          dragStartBehavior: DragStartBehavior.down,
          maxCrossAxisExtent: 200.0,
          children:
              kStates.map<Widget>((String state) {
                return GestureDetector(
                  dragStartBehavior: DragStartBehavior.down,
                  onTap: () {
                    log.add(state);
                  },
                  child: ColoredBox(color: const Color(0xFF0000FF), child: Text(state)),
                );
              }).toList(),
        ),
      ),
    );

    expect(tester.getSize(find.text('Arkansas')), equals(const Size(200.0, 200.0)));

    for (int i = 0; i < 8; ++i) {
      await tester.tap(find.text(kStates[i]));
      expect(log, equals(<String>[kStates[i]]));
      log.clear();
    }

    expect(find.text('Nevada'), findsNothing);

    await tester.drag(find.text('Arkansas'), const Offset(0.0, -4000.0));
    await tester.pump();

    expect(find.text('Alabama'), findsNothing);

    expect(tester.getCenter(find.text('Tennessee')), equals(const Offset(300.0, 100.0)));

    await tester.tap(find.text('Tennessee'));
    expect(log, equals(<String>['Tennessee']));
    log.clear();
  });

  testWidgets('GridView large scroll jump', (WidgetTester tester) async {
    final List<int> log = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.extent(
          scrollDirection: Axis.horizontal,
          maxCrossAxisExtent: 200.0,
          childAspectRatio: 0.75,
          children: List<Widget>.generate(80, (int i) {
            return Builder(
              builder: (BuildContext context) {
                log.add(i);
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(tester.getSize(find.text('4')), equals(const Size(200.0 / 0.75, 200.0)));

    expect(
      log,
      equals(<int>[
        0, 1, 2, // col 0
        3, 4, 5, // col 1
        6, 7, 8, // col 2
        9, 10, 11, // col 3 (in cached area)
      ]),
    );
    log.clear();

    for (int i = 0; i < 9; i++) {
      expect(find.text('$i'), findsOneWidget);
    }
    for (int i = 9; i < 80; i++) {
      expect(find.text('$i'), findsNothing);
    }

    final ScrollableState state = tester.state(find.byType(Scrollable));
    final ScrollPosition position = state.position;
    position.jumpTo(3025.0);

    expect(log, isEmpty);
    await tester.pump();

    expect(
      log,
      equals(<int>[
        30, 31, 32, // col 10 (in cached area)
        33, 34, 35, // col 11
        36, 37, 38, // col 12
        39, 40, 41, // col 13
        42, 43, 44, // col 14
        45, 46, 47, // col 15 (in cached area)
      ]),
    );
    log.clear();

    for (int i = 0; i < 33; i++) {
      expect(find.text('$i'), findsNothing);
    }
    for (int i = 33; i < 45; i++) {
      expect(find.text('$i'), findsOneWidget);
    }
    for (int i = 45; i < 80; i++) {
      expect(find.text('$i'), findsNothing);
    }

    position.jumpTo(975.0);

    expect(log, isEmpty);
    await tester.pump();

    expect(
      log,
      equals(<int>[
        6, 7, 8, // col2 (in cached area)
        9, 10, 11, // col 3
        12, 13, 14, // col 4
        15, 16, 17, // col 5
        18, 19, 20, // col 6
        21, 22, 23, // col 7 (in cached area)
      ]),
    );
    log.clear();

    for (int i = 0; i < 9; i++) {
      expect(find.text('$i'), findsNothing);
    }
    for (int i = 9; i < 21; i++) {
      expect(find.text('$i'), findsOneWidget);
    }
    for (int i = 21; i < 80; i++) {
      expect(find.text('$i'), findsNothing);
    }
  });

  testWidgets('GridView - change crossAxisCount', (WidgetTester tester) async {
    final List<int> log = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 4),
          children: List<Widget>.generate(40, (int i) {
            return Builder(
              builder: (BuildContext context) {
                log.add(i);
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(tester.getSize(find.text('4')), equals(const Size(200.0, 200.0)));

    expect(
      log,
      equals(<int>[
        0, 1, 2, 3, // row 0
        4, 5, 6, 7, // row 1
        8, 9, 10, 11, // row 2
        12, 13, 14, 15, // row 3 (in cached area)
        16, 17, 18, 19, // row 4 (in cached area)
      ]),
    );
    for (int i = 0; i < 12; i++) {
      expect(find.text('$i'), findsOneWidget);
    }
    for (int i = 12; i < 40; i++) {
      expect(find.text('$i'), findsNothing);
    }
    log.clear();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          children: List<Widget>.generate(40, (int i) {
            return Builder(
              builder: (BuildContext context) {
                log.add(i);
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(
      log,
      equals(<int>[
        0, 1, 2, 3, // row 0
        4, 5, 6, 7, // row 1
        8, 9, 10, 11, // row 2
        12, 13, 14, 15, // row 3 (in cached area)
        16, 17, 18, 19, // row 4 (in cached area)
      ]),
    );
    log.clear();

    expect(tester.getSize(find.text('3')), equals(const Size(400.0, 400.0)));
    expect(find.text('4'), findsNothing);
  });

  testWidgets('SliverGridRegularTileLayout - can handle close to zero mainAxisStride', (
    WidgetTester tester,
  ) async {
    const SliverGridDelegateWithMaxCrossAxisExtent delegate =
        SliverGridDelegateWithMaxCrossAxisExtent(
          childAspectRatio: 1e300,
          maxCrossAxisExtent: 500.0,
        );
    final SliverGridLayout layout = delegate.getLayout(
      const SliverConstraints(
        axisDirection: AxisDirection.down,
        growthDirection: GrowthDirection.forward,
        userScrollDirection: ScrollDirection.forward,
        scrollOffset: 100.0,
        precedingScrollExtent: 0.0,
        overlap: 0.0,
        remainingPaintExtent: 0.0,
        crossAxisExtent: 500,
        crossAxisDirection: AxisDirection.right,
        viewportMainAxisExtent: 100.0,
        remainingCacheExtent: 0.0,
        cacheOrigin: 0.0,
      ),
    );
    expect(layout.getMinChildIndexForScrollOffset(1000.0), 0.0);
  });

  testWidgets('GridView - change maxChildCrossAxisExtent', (WidgetTester tester) async {
    final List<int> log = <int>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(maxCrossAxisExtent: 200.0),
          children: List<Widget>.generate(40, (int i) {
            return Builder(
              builder: (BuildContext context) {
                log.add(i);
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(tester.getSize(find.text('4')), equals(const Size(200.0, 200.0)));

    expect(
      log,
      equals(<int>[
        0, 1, 2, 3, // row 0
        4, 5, 6, 7, // row 1
        8, 9, 10, 11, // row 2
        12, 13, 14, 15, // row 3 (in cached area)
        16, 17, 18, 19, // row 4 (in cached area)
      ]),
    );
    for (int i = 0; i < 12; i++) {
      expect(find.text('$i'), findsOneWidget);
    }
    for (int i = 12; i < 40; i++) {
      expect(find.text('$i'), findsNothing);
    }
    log.clear();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(maxCrossAxisExtent: 400.0),
          children: List<Widget>.generate(40, (int i) {
            return Builder(
              builder: (BuildContext context) {
                log.add(i);
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(
      log,
      equals(<int>[
        0, 1, 2, 3, // row 0
        4, 5, 6, 7, // row 1
        8, 9, 10, 11, // row 2
        12, 13, 14, 15, // row 3 (in cached area)
        16, 17, 18, 19, // row 4 (in cached area)
      ]),
    );
    log.clear();

    expect(tester.getSize(find.text('3')), equals(const Size(400.0, 400.0)));
    expect(find.text('4'), findsNothing);
  });

  testWidgets('One-line GridView paints', (WidgetTester tester) async {
    const Color green = Color(0xFF00FF00);

    final Container container = Container(decoration: const BoxDecoration(color: green));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            child: GridView.count(
              cacheExtent: 0.0,
              crossAxisCount: 2,
              children: <Widget>[container, container, container, container],
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(GridView),
      paints
        ..rect(color: green)
        ..rect(color: green),
    );
    expect(
      find.byType(GridView),
      isNot(
        paints
          ..rect(color: green)
          ..rect(color: green)
          ..rect(color: green),
      ),
    );
  });

  testWidgets('GridView in zero context', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox.shrink(
            child: GridView.count(
              crossAxisCount: 4,
              children: List<Widget>.generate(20, (int i) {
                return Text('$i');
              }),
            ),
          ),
        ),
      ),
    );

    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsNothing);
  });

  testWidgets('GridView in unbounded context', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SingleChildScrollView(
          child: GridView.count(
            crossAxisCount: 4,
            shrinkWrap: true,
            children: List<Widget>.generate(20, (int i) {
              return Text('$i');
            }),
          ),
        ),
      ),
    );

    expect(find.text('0'), findsOneWidget);
    expect(find.text('19'), findsOneWidget);
  });

  testWidgets('GridView.builder control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 4),
          shrinkWrap: true,
          itemCount: 20,
          itemBuilder: (BuildContext context, int index) {
            return Text('$index');
          },
        ),
      ),
    );
    expect(find.text('0'), findsOneWidget);
    expect(find.text('11'), findsOneWidget);
    expect(find.text('12'), findsNothing);
  });

  testWidgets('GridView.builder with undefined itemCount', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 4),
          shrinkWrap: true,
          itemBuilder: (BuildContext context, int index) {
            return Text('$index');
          },
        ),
      ),
    );
    expect(find.text('0'), findsOneWidget);
    expect(find.text('11'), findsOneWidget);
    await tester.drag(find.byType(GridView), const Offset(0.0, -300.0));
    await tester.pump(const Duration(milliseconds: 200));
    expect(find.text('13'), findsOneWidget);
  });

  testWidgets('GridView cross axis layout', (WidgetTester tester) async {
    final Key target = UniqueKey();

    Widget build(TextDirection textDirection) {
      return Directionality(
        textDirection: textDirection,
        child: GridView.count(crossAxisCount: 4, children: <Widget>[Container(key: target)]),
      );
    }

    await tester.pumpWidget(build(TextDirection.ltr));

    expect(tester.getTopLeft(find.byKey(target)), Offset.zero);
    expect(tester.getBottomRight(find.byKey(target)), const Offset(200.0, 200.0));

    await tester.pumpWidget(build(TextDirection.rtl));

    expect(tester.getTopLeft(find.byKey(target)), const Offset(600.0, 0.0));
    expect(tester.getBottomRight(find.byKey(target)), const Offset(800.0, 200.0));
  });

  testWidgets('GridView crossAxisSpacing', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/27151.
    final Key target = UniqueKey();

    Widget build(TextDirection textDirection) {
      return Directionality(
        textDirection: textDirection,
        child: GridView.count(
          crossAxisCount: 4,
          crossAxisSpacing: 8.0,
          children: <Widget>[Container(key: target)],
        ),
      );
    }

    await tester.pumpWidget(build(TextDirection.ltr));

    expect(tester.getTopLeft(find.byKey(target)), Offset.zero);
    expect(tester.getBottomRight(find.byKey(target)), const Offset(194.0, 194.0));

    await tester.pumpWidget(build(TextDirection.rtl));

    expect(tester.getTopLeft(find.byKey(target)), const Offset(606.0, 0.0));
    expect(tester.getBottomRight(find.byKey(target)), const Offset(800.0, 194.0));
  });

  testWidgets('GridView does not cache itemBuilder calls', (WidgetTester tester) async {
    final Map<int, int> counters = <int, int>{};

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          itemCount: 1000,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          itemBuilder: (BuildContext context, int index) {
            counters[index] = (counters[index] ?? 0) + 1;
            return SizedBox(key: ValueKey<int>(index), width: 200, height: 200);
          },
        ),
      ),
    );

    expect(find.byKey(const ValueKey<int>(4)), findsOneWidget);
    expect(counters[4], 1);

    await tester.fling(find.byType(GridView), const Offset(0, -300), 5000);
    await tester.pumpAndSettle();

    expect(find.byKey(const ValueKey<int>(4)), findsNothing);
    expect(counters[4], 1);

    await tester.fling(find.byType(GridView), const Offset(0, 300), 5000);
    await tester.pumpAndSettle();

    expect(find.byKey(const ValueKey<int>(4)), findsOneWidget);
    expect(counters[4], 2);
  });

  testWidgets('GridView does not report visual overflow unnecessarily', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          children: <Widget>[Container(height: 200.0)],
        ),
      ),
    );

    // 1st, check that the render object has received the default clip behavior.
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    // The context will get Clip.none because there is no actual visual overflow.
    final TestClipPaintingContext context = TestClipPaintingContext();
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.none));
    context.dispose();
  });

  testWidgets('GridView respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          children: <Widget>[
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
          ],
        ),
      ),
    );

    // 1st, check that the render object has received the default clip behavior.
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    // 2nd, check that the painting context has received the default clip behavior.
    final TestClipPaintingContext context = TestClipPaintingContext();
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.hardEdge));

    // 3rd, pump a new widget to check that the render object can update its clip behavior.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          clipBehavior: Clip.antiAlias,
          children: <Widget>[
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
            Container(height: 2000.0),
          ],
        ),
      ),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));

    // 4th, check that a non-default clip behavior can be sent to the painting context.
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.antiAlias));
    context.dispose();
  });

  testWidgets('GridView.builder respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          itemCount: 10,
          itemBuilder: (BuildContext _, int _) => Container(height: 2000.0),
          clipBehavior: Clip.antiAlias,
        ),
      ),
    );
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('GridView.custom respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.custom(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          childrenDelegate: SliverChildBuilderDelegate(
            (BuildContext context, int index) => Container(height: 2000.0),
            childCount: 1,
          ),
          clipBehavior: Clip.antiAlias,
        ),
      ),
    );
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('GridView.count respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          crossAxisCount: 3,
          clipBehavior: Clip.antiAlias,
          children: <Widget>[Container(height: 2000.0)],
        ),
      ),
    );
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('GridView.extent respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.extent(
          maxCrossAxisExtent: 1000,
          clipBehavior: Clip.antiAlias,
          children: <Widget>[Container(height: 2000.0)],
        ),
      ),
    );
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('SliverGridDelegateWithFixedCrossAxisCount mainAxisExtent works as expected', (
    WidgetTester tester,
  ) async {
    const int crossAxisCount = 4;
    const double mainAxisExtent = 100.0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: crossAxisCount,
            mainAxisExtent: mainAxisExtent,
          ),
          children: List<Widget>.generate(20, (int i) {
            return Builder(
              builder: (BuildContext context) {
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(tester.getSize(find.text('4')), equals(const Size(200.0, mainAxisExtent)));
  });

  testWidgets('SliverGridDelegateWithMaxCrossAxisExtent mainAxisExtent works as expected', (
    WidgetTester tester,
  ) async {
    const double maxCrossAxisExtent = 200.0;
    const double mainAxisExtent = 100.0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView(
          gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
            maxCrossAxisExtent: maxCrossAxisExtent,
            mainAxisExtent: mainAxisExtent,
          ),
          children: List<Widget>.generate(20, (int i) {
            return Builder(
              builder: (BuildContext context) {
                return Text('$i');
              },
            );
          }),
        ),
      ),
    );

    expect(tester.getSize(find.text('4')), equals(const Size(200.0, mainAxisExtent)));
  });

  testWidgets(
    'SliverGridDelegateWithMaxCrossAxisExtent throws assertion error when maxCrossAxisExtent is 0',
    (WidgetTester tester) async {
      const double maxCrossAxisExtent = 0;

      expect(
        () => Directionality(
          textDirection: TextDirection.ltr,
          child: GridView.extent(maxCrossAxisExtent: maxCrossAxisExtent),
        ),
        throwsAssertionError,
      );
    },
  );

  testWidgets('SliverGrid sets correct extent for null returning builder delegate', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/130685
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          controller: controller,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: 3,
            crossAxisSpacing: 16,
            mainAxisSpacing: 16,
          ),
          itemBuilder: (BuildContext context, int index) {
            if (index == 12) {
              return null;
            }
            return Container(
              height: 100,
              width: 100,
              color: const Color(0xFFFF8A80),
              alignment: Alignment.center,
              child: Text('item ${index + 1}'),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(controller.position.maxScrollExtent, double.infinity);
    expect(controller.position.pixels, 0.0);
    await tester.fling(find.byType(GridView), const Offset(0.0, -1300.0), 100.0);
    await tester.pumpAndSettle();
    // The actual extent of the children is 472.0. This should be reflected when
    // the builder returns null (meaning we have reached the end).
    expect(controller.position.maxScrollExtent, 472.0);
    expect(controller.position.pixels, 472.0);
  });

  testWidgets('SliverGridDelegate mainAxisExtent add assert', (WidgetTester tester) async {
    Widget buildGridView(SliverGridDelegate delegate) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          gridDelegate: delegate,
          itemBuilder: (BuildContext context, int index) {
            return Container(height: 50, alignment: Alignment.center, child: Text('$index'));
          },
          itemCount: 50,
        ),
      );
    }

    await expectLater(
      () => tester.pumpWidget(
        buildGridView(
          SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: 3,
            mainAxisSpacing: 8,
            crossAxisSpacing: 8,
            mainAxisExtent: -100,
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.toString(),
          '.toString()',
          contains('mainAxisExtent == null || mainAxisExtent >= 0'),
        ),
      ),
    );

    await expectLater(
      () => tester.pumpWidget(
        buildGridView(
          SliverGridDelegateWithMaxCrossAxisExtent(
            maxCrossAxisExtent: 100,
            mainAxisSpacing: 8,
            crossAxisSpacing: 8,
            mainAxisExtent: -100,
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.toString(),
          '.toString()',
          contains('mainAxisExtent == null || mainAxisExtent >= 0'),
        ),
      ),
    );
  });
}
