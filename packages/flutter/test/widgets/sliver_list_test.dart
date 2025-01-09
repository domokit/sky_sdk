// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverList reverse children (with keys)', (WidgetTester tester) async {
    final List<int> items = List<int>.generate(20, (int i) => i);
    const double itemHeight = 300.0;
    const double viewportHeight = 500.0;

    const double scrollPosition = 18 * itemHeight;
    final ScrollController controller = ScrollController(initialScrollOffset: scrollPosition);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      _buildSliverList(
        items: items,
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    await tester.pumpAndSettle();

    expect(controller.offset, scrollPosition);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsOneWidget);

    await tester.pumpWidget(
      _buildSliverList(
        items: items.reversed.toList(),
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    final int frames = await tester.pumpAndSettle();
    expect(frames, 1); // ensures that there is no (animated) bouncing of the scrollable

    expect(controller.offset, scrollPosition);
    expect(find.text('Tile 19'), findsNothing);
    expect(find.text('Tile 18'), findsNothing);
    expect(find.text('Tile 1'), findsOneWidget);
    expect(find.text('Tile 0'), findsOneWidget);

    controller.jumpTo(0.0);
    await tester.pumpAndSettle();

    expect(controller.offset, 0.0);
    expect(find.text('Tile 19'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 0'), findsNothing);
  });

  testWidgets('SliverList replace children (with keys)', (WidgetTester tester) async {
    final List<int> items = List<int>.generate(20, (int i) => i);
    const double itemHeight = 300.0;
    const double viewportHeight = 500.0;

    const double scrollPosition = 18 * itemHeight;
    final ScrollController controller = ScrollController(initialScrollOffset: scrollPosition);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      _buildSliverList(
        items: items,
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    await tester.pumpAndSettle();

    expect(controller.offset, scrollPosition);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsOneWidget);

    await tester.pumpWidget(
      _buildSliverList(
        items: items.map<int>((int i) => i + 100).toList(),
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    final int frames = await tester.pumpAndSettle();
    expect(frames, 1); // ensures that there is no (animated) bouncing of the scrollable

    expect(controller.offset, scrollPosition);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 18'), findsNothing);
    expect(find.text('Tile 19'), findsNothing);

    expect(find.text('Tile 100'), findsNothing);
    expect(find.text('Tile 101'), findsNothing);
    expect(find.text('Tile 118'), findsOneWidget);
    expect(find.text('Tile 119'), findsOneWidget);

    controller.jumpTo(0.0);
    await tester.pumpAndSettle();

    expect(controller.offset, 0.0);
    expect(find.text('Tile 100'), findsOneWidget);
    expect(find.text('Tile 101'), findsOneWidget);
    expect(find.text('Tile 118'), findsNothing);
    expect(find.text('Tile 119'), findsNothing);
  });

  testWidgets('SliverList replace with shorter children list (with keys)', (
    WidgetTester tester,
  ) async {
    final List<int> items = List<int>.generate(20, (int i) => i);
    const double itemHeight = 300.0;
    const double viewportHeight = 500.0;

    final double scrollPosition = items.length * itemHeight - viewportHeight;
    final ScrollController controller = ScrollController(initialScrollOffset: scrollPosition);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      _buildSliverList(
        items: items,
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    await tester.pumpAndSettle();

    expect(controller.offset, scrollPosition);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 17'), findsNothing);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsOneWidget);

    await tester.pumpWidget(
      _buildSliverList(
        items: items.sublist(0, items.length - 1),
        controller: controller,
        itemHeight: itemHeight,
        viewportHeight: viewportHeight,
      ),
    );
    final int frames = await tester.pumpAndSettle();
    expect(frames, 1); // No animation when content shrinks suddenly.

    expect(controller.offset, scrollPosition - itemHeight);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 17'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsNothing);
  });

  testWidgets('SliverList should layout first child in case of child reordering', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/35904.
    List<String> items = <String>['1', '2'];
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    await tester.pumpWidget(_buildSliverListRenderWidgetChild(items, controller1));
    await tester.pumpAndSettle();

    expect(find.text('Tile 1'), findsOneWidget);
    expect(find.text('Tile 2'), findsOneWidget);

    items = items.reversed.toList();
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(_buildSliverListRenderWidgetChild(items, controller2));
    await tester.pumpAndSettle();

    expect(find.text('Tile 1'), findsOneWidget);
    expect(find.text('Tile 2'), findsOneWidget);
  });

  testWidgets('SliverList should recalculate inaccurate layout offset case 1', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/42142.
    final List<int> items = List<int>.generate(20, (int i) => i);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      _buildSliverList(
        items: List<int>.from(items),
        controller: controller,
        itemHeight: 50,
        viewportHeight: 200,
      ),
    );
    await tester.pumpAndSettle();

    await tester.drag(find.text('Tile 2'), const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();

    // Viewport should be scrolled to the end of list.
    expect(controller.offset, 800.0);
    expect(find.text('Tile 15'), findsNothing);
    expect(find.text('Tile 16'), findsOneWidget);
    expect(find.text('Tile 17'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsOneWidget);

    // Prepends item to the list.
    items.insert(0, -1);
    await tester.pumpWidget(
      _buildSliverList(
        items: List<int>.from(items),
        controller: controller,
        itemHeight: 50,
        viewportHeight: 200,
      ),
    );
    await tester.pump();
    // We need second pump to ensure the scheduled animation gets run.
    await tester.pumpAndSettle();
    // Scroll offset should stay the same, and the items in viewport should be
    // shifted by one.
    expect(controller.offset, 800.0);
    expect(find.text('Tile 14'), findsNothing);
    expect(find.text('Tile 15'), findsOneWidget);
    expect(find.text('Tile 16'), findsOneWidget);
    expect(find.text('Tile 17'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsNothing);

    // Drags back to beginning and newly added item is visible.
    await tester.drag(find.text('Tile 16'), const Offset(0.0, 1000.0));
    await tester.pumpAndSettle();
    expect(controller.offset, 0.0);
    expect(find.text('Tile -1'), findsOneWidget);
    expect(find.text('Tile 0'), findsOneWidget);
    expect(find.text('Tile 1'), findsOneWidget);
    expect(find.text('Tile 2'), findsOneWidget);
    expect(find.text('Tile 3'), findsNothing);
  });

  testWidgets('SliverList should recalculate inaccurate layout offset case 2', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/42142.
    final List<int> items = List<int>.generate(20, (int i) => i);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      _buildSliverList(
        items: List<int>.from(items),
        controller: controller,
        itemHeight: 50,
        viewportHeight: 200,
      ),
    );
    await tester.pumpAndSettle();

    await tester.drag(find.text('Tile 2'), const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();

    // Viewport should be scrolled to the end of list.
    expect(controller.offset, 800.0);
    expect(find.text('Tile 15'), findsNothing);
    expect(find.text('Tile 16'), findsOneWidget);
    expect(find.text('Tile 17'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 19'), findsOneWidget);

    // Reorders item to the front. This should make item 19 to be first child
    // with layout offset = null.
    final int swap = items[19];
    items[19] = items[3];
    items[3] = swap;

    await tester.pumpWidget(
      _buildSliverList(
        items: List<int>.from(items),
        controller: controller,
        itemHeight: 50,
        viewportHeight: 200,
      ),
    );
    await tester.pump();
    // We need second pump to ensure the scheduled animation gets run.
    await tester.pumpAndSettle();
    // Scroll offset should stay the same
    expect(controller.offset, 800.0);
    expect(find.text('Tile 14'), findsNothing);
    expect(find.text('Tile 15'), findsNothing);
    expect(find.text('Tile 16'), findsOneWidget);
    expect(find.text('Tile 17'), findsOneWidget);
    expect(find.text('Tile 18'), findsOneWidget);
    expect(find.text('Tile 3'), findsOneWidget);
  });

  testWidgets(
    'SliverList should start to perform layout from the initial child when there is no valid offset',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/66198.
      bool isShow = true;
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      Widget buildSliverList(ScrollController controller) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              height: 200,
              child: ListView(
                controller: controller,
                children: <Widget>[
                  if (isShow)
                    for (int i = 0; i < 20; i++) SizedBox(height: 50, child: Text('Tile $i')),
                  const SizedBox(), // Use this widget to occupy the position where the offset is 0 when rebuild
                  const SizedBox(key: Key('key0'), height: 50.0),
                  const SizedBox(key: Key('key1'), height: 50.0),
                ],
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSliverList(controller));
      await tester.pumpAndSettle();

      // Scrolling to the bottom.
      await tester.drag(find.text('Tile 2'), const Offset(0.0, -1000.0));
      await tester.pumpAndSettle();

      // Viewport should be scrolled to the end of list.
      expect(controller.offset, 900.0);
      expect(find.text('Tile 17'), findsNothing);
      expect(find.text('Tile 18'), findsOneWidget);
      expect(find.text('Tile 19'), findsOneWidget);
      expect(find.byKey(const Key('key0')), findsOneWidget);
      expect(find.byKey(const Key('key1')), findsOneWidget);

      // Trigger rebuild.
      isShow = false;
      await tester.pumpWidget(buildSliverList(controller));

      // After rebuild, [ContainerRenderObjectMixin] has two children, and
      // neither of them has a valid layout offset.
      // SliverList can layout normally without any assert or dead loop.
      // Only the 'SizeBox' show in the viewport.
      expect(controller.offset, 0.0);
      expect(find.text('Tile 0'), findsNothing);
      expect(find.text('Tile 19'), findsNothing);
      expect(find.byKey(const Key('key0')), findsOneWidget);
      expect(find.byKey(const Key('key1')), findsOneWidget);
    },
  );
}

Widget _buildSliverListRenderWidgetChild(List<String> items, ScrollController controller) {
  return MaterialApp(
    home: Directionality(
      textDirection: TextDirection.ltr,
      child: Material(
        child: SizedBox(
          height: 500,
          child: CustomScrollView(
            controller: controller,
            slivers: <Widget>[
              SliverList(
                delegate: SliverChildListDelegate(
                  items.map<Widget>((String item) {
                    return Chip(key: Key(item), label: Text('Tile $item'));
                  }).toList(),
                ),
              ),
            ],
          ),
        ),
      ),
    ),
  );
}

Widget _buildSliverList({
  List<int> items = const <int>[],
  ScrollController? controller,
  double itemHeight = 500.0,
  double viewportHeight = 300.0,
}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Center(
      child: SizedBox(
        height: viewportHeight,
        child: CustomScrollView(
          controller: controller,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildBuilderDelegate(
                (BuildContext context, int i) {
                  return SizedBox(
                    key: ValueKey<int>(items[i]),
                    height: itemHeight,
                    child: Text('Tile ${items[i]}'),
                  );
                },
                findChildIndexCallback: (Key key) {
                  final ValueKey<int> valueKey = key as ValueKey<int>;
                  final int index = items.indexOf(valueKey.value);
                  return index == -1 ? null : index;
                },
                childCount: items.length,
              ),
            ),
          ],
        ),
      ),
    ),
  );
}
