// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CustomScrollView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: CustomScrollView(
          restorationId: 'list',
          cacheExtent: 0,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(
                List<Widget>.generate(
                  50,
                  (int index) => SizedBox(height: 50, child: Text('Tile $index')),
                ),
              ),
            ),
          ],
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('ListView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListView(
          restorationId: 'list',
          cacheExtent: 0,
          children: List<Widget>.generate(
            50,
            (int index) => SizedBox(height: 50, child: Text('Tile $index')),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('ListView.builder restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListView.builder(
          restorationId: 'list',
          cacheExtent: 0,
          itemBuilder:
              (BuildContext context, int index) => SizedBox(height: 50, child: Text('Tile $index')),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('ListView.separated restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListView.separated(
          restorationId: 'list',
          cacheExtent: 0,
          itemCount: 50,
          separatorBuilder: (BuildContext context, int index) => const SizedBox.shrink(),
          itemBuilder:
              (BuildContext context, int index) => SizedBox(height: 50, child: Text('Tile $index')),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('ListView.custom restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListView.custom(
          restorationId: 'list',
          cacheExtent: 0,
          childrenDelegate: SliverChildListDelegate(
            List<Widget>.generate(
              50,
              (int index) => SizedBox(height: 50, child: Text('Tile $index')),
            ),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('GridView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: GridView(
          restorationId: 'grid',
          cacheExtent: 0,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 1),
          children: List<Widget>.generate(
            50,
            (int index) => SizedBox(height: 50, child: Text('Tile $index')),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('GridView.builder restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: GridView.builder(
          restorationId: 'grid',
          cacheExtent: 0,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 1),
          itemBuilder:
              (BuildContext context, int index) => SizedBox(height: 50, child: Text('Tile $index')),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('GridView.custom restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: GridView.custom(
          restorationId: 'grid',
          cacheExtent: 0,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 1),
          childrenDelegate: SliverChildListDelegate(
            List<Widget>.generate(
              50,
              (int index) => SizedBox(height: 50, child: Text('Tile $index')),
            ),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('GridView.count restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: GridView.count(
          restorationId: 'grid',
          cacheExtent: 0,
          crossAxisCount: 1,
          children: List<Widget>.generate(
            50,
            (int index) => SizedBox(height: 50, child: Text('Tile $index')),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('GridView.extent restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: GridView.extent(
          restorationId: 'grid',
          cacheExtent: 0,
          maxCrossAxisExtent: 50,
          children: List<Widget>.generate(
            50,
            (int index) => SizedBox(height: 50, child: Text('Tile $index')),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester);
  });

  testWidgets('SingleChildScrollView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: SingleChildScrollView(
          restorationId: 'single',
          child: Column(
            children: List<Widget>.generate(
              50,
              (int index) => SizedBox(height: 50, child: Text('Tile $index')),
            ),
          ),
        ),
      ),
    );

    expect(tester.getTopLeft(find.text('Tile 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('Tile 1')), const Offset(0, 50));

    tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(525);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Tile 0')), const Offset(0, -525));
    expect(tester.getTopLeft(find.text('Tile 1')), const Offset(0, -475));

    await tester.restartAndRestore();

    expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 525);
    expect(tester.getTopLeft(find.text('Tile 0')), const Offset(0, -525));
    expect(tester.getTopLeft(find.text('Tile 1')), const Offset(0, -475));

    final TestRestorationData data = await tester.getRestorationData();
    tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(0);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Tile 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('Tile 1')), const Offset(0, 50));

    await tester.restoreFrom(data);

    expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 525);
    expect(tester.getTopLeft(find.text('Tile 0')), const Offset(0, -525));
    expect(tester.getTopLeft(find.text('Tile 1')), const Offset(0, -475));
  });

  testWidgets('PageView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: PageView(
          restorationId: 'pager',
          children: List<Widget>.generate(50, (int index) => Text('Tile $index')),
        ),
      ),
    );

    await pageViewScrollAndRestore(tester);
  });

  testWidgets('PageView.builder restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: PageView.builder(
          restorationId: 'pager',
          itemBuilder:
              (BuildContext context, int index) => SizedBox(height: 50, child: Text('Tile $index')),
        ),
      ),
    );

    await pageViewScrollAndRestore(tester);
  });

  testWidgets('PageView.custom restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: PageView.custom(
          restorationId: 'pager',
          childrenDelegate: SliverChildListDelegate(
            List<Widget>.generate(
              50,
              (int index) => SizedBox(height: 50, child: Text('Tile $index')),
            ),
          ),
        ),
      ),
    );

    await pageViewScrollAndRestore(tester);
  });

  testWidgets('ListWheelScrollView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListWheelScrollView(
          restorationId: 'wheel',
          itemExtent: 50,
          children: List<Widget>.generate(50, (int index) => Text('Tile $index')),
        ),
      ),
    );

    await restoreScrollAndVerify(tester, secondOffset: 542);
  });

  testWidgets('ListWheelScrollView.useDelegate restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListWheelScrollView.useDelegate(
          restorationId: 'wheel',
          itemExtent: 50,
          childDelegate: ListWheelChildListDelegate(
            children: List<Widget>.generate(
              50,
              (int index) => SizedBox(height: 50, child: Text('Tile $index')),
            ),
          ),
        ),
      ),
    );

    await restoreScrollAndVerify(tester, secondOffset: 542);
  });

  testWidgets('NestedScrollView restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: TestHarness(
          height: 200,
          child: NestedScrollView(
            restorationId: 'outer',
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverOverlapAbsorber(
                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                  sliver: SliverAppBar(
                    title: const Text('Books'),
                    pinned: true,
                    expandedHeight: 150.0,
                    forceElevated: innerBoxIsScrolled,
                  ),
                ),
              ];
            },
            body: ListView(
              restorationId: 'inner',
              cacheExtent: 0,
              children: List<Widget>.generate(
                50,
                (int index) => SizedBox(height: 50, child: Text('Tile $index')),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.renderObject<RenderSliver>(find.byType(SliverAppBar)).geometry!.paintExtent, 150);
    expect(find.text('Tile 0'), findsOneWidget);
    expect(find.text('Tile 10'), findsNothing);

    await tester.drag(find.byType(NestedScrollView), const Offset(0, -500));
    await tester.pump();

    expect(tester.renderObject<RenderSliver>(find.byType(SliverAppBar)).geometry!.paintExtent, 56);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 10'), findsOneWidget);

    await tester.restartAndRestore();

    expect(tester.renderObject<RenderSliver>(find.byType(SliverAppBar)).geometry!.paintExtent, 56);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 10'), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();
    await tester.drag(find.byType(NestedScrollView), const Offset(0, 600));
    await tester.pump();

    expect(tester.renderObject<RenderSliver>(find.byType(SliverAppBar)).geometry!.paintExtent, 150);
    expect(find.text('Tile 0'), findsOneWidget);
    expect(find.text('Tile 10'), findsNothing);

    await tester.restoreFrom(data);

    expect(tester.renderObject<RenderSliver>(find.byType(SliverAppBar)).geometry!.paintExtent, 56);
    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 10'), findsOneWidget);
  });

  testWidgets('RestorationData is flushed even if no frame is scheduled', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      TestHarness(
        child: ListView(
          restorationId: 'list',
          cacheExtent: 0,
          children: List<Widget>.generate(
            50,
            (int index) => SizedBox(height: 50, child: Text('Tile $index')),
          ),
        ),
      ),
    );

    expect(find.text('Tile 0'), findsOneWidget);
    expect(find.text('Tile 1'), findsOneWidget);
    expect(find.text('Tile 10'), findsNothing);
    expect(find.text('Tile 11'), findsNothing);
    expect(find.text('Tile 12'), findsNothing);

    final TestRestorationData initialData = await tester.getRestorationData();
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
    await gesture.moveBy(const Offset(0, -525));
    await tester.pump();

    expect(find.text('Tile 0'), findsNothing);
    expect(find.text('Tile 1'), findsNothing);
    expect(find.text('Tile 10'), findsOneWidget);
    expect(find.text('Tile 11'), findsOneWidget);
    expect(find.text('Tile 12'), findsOneWidget);

    // Restoration data hasn't changed.
    expect(await tester.getRestorationData(), initialData);

    // Restoration data changes with up event.
    await gesture.up();
    await tester.pump();
    expect(await tester.getRestorationData(), isNot(initialData));
  });
}

Future<void> pageViewScrollAndRestore(WidgetTester tester) async {
  expect(find.text('Tile 0'), findsOneWidget);
  expect(find.text('Tile 10'), findsNothing);

  tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(50.0 * 10);
  await tester.pumpAndSettle();

  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);

  await tester.restartAndRestore();

  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 50.0 * 10);
  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);

  final TestRestorationData data = await tester.getRestorationData();
  tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(0);
  await tester.pump();

  expect(find.text('Tile 0'), findsOneWidget);
  expect(find.text('Tile 10'), findsNothing);

  await tester.restoreFrom(data);

  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 50.0 * 10);
  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);
}

Future<void> restoreScrollAndVerify(WidgetTester tester, {double secondOffset = 525}) async {
  final Finder findScrollable = find.byElementPredicate((Element e) => e.widget is Scrollable);

  expect(find.text('Tile 0'), findsOneWidget);
  expect(find.text('Tile 1'), findsOneWidget);
  expect(find.text('Tile 10'), findsNothing);
  expect(find.text('Tile 11'), findsNothing);
  expect(find.text('Tile 12'), findsNothing);

  tester.state<ScrollableState>(findScrollable).position.jumpTo(secondOffset);
  await tester.pump();

  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 1'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);
  expect(find.text('Tile 11'), findsOneWidget);
  expect(find.text('Tile 12'), findsOneWidget);

  await tester.restartAndRestore();

  expect(tester.state<ScrollableState>(findScrollable).position.pixels, secondOffset);
  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 1'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);
  expect(find.text('Tile 11'), findsOneWidget);
  expect(find.text('Tile 12'), findsOneWidget);

  final TestRestorationData data = await tester.getRestorationData();
  tester.state<ScrollableState>(findScrollable).position.jumpTo(0);
  await tester.pump();

  expect(find.text('Tile 0'), findsOneWidget);
  expect(find.text('Tile 1'), findsOneWidget);
  expect(find.text('Tile 10'), findsNothing);
  expect(find.text('Tile 11'), findsNothing);
  expect(find.text('Tile 12'), findsNothing);

  await tester.restoreFrom(data);

  expect(tester.state<ScrollableState>(findScrollable).position.pixels, secondOffset);
  expect(find.text('Tile 0'), findsNothing);
  expect(find.text('Tile 1'), findsNothing);
  expect(find.text('Tile 10'), findsOneWidget);
  expect(find.text('Tile 11'), findsOneWidget);
  expect(find.text('Tile 12'), findsOneWidget);
}

class TestHarness extends StatelessWidget {
  const TestHarness({super.key, required this.child, this.height = 100});

  final Widget child;
  final double height;

  @override
  Widget build(BuildContext context) {
    return RootRestorationScope(
      restorationId: 'root',
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(height: height, width: 50, child: child),
        ),
      ),
    );
  }
}
