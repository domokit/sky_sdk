// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DecoratedSliver creates, paints, and disposes BoxPainter', (
    WidgetTester tester,
  ) async {
    final TestDecoration decoration = TestDecoration();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              DecoratedSliver(
                decoration: decoration,
                sliver: const SliverToBoxAdapter(child: SizedBox(width: 100, height: 100)),
              ),
            ],
          ),
        ),
      ),
    );

    expect(decoration.painters, hasLength(1));
    expect(decoration.painters.last.lastConfiguration!.size, const Size(800, 100));
    expect(decoration.painters.last.lastOffset, Offset.zero);
    expect(decoration.painters.last.disposed, false);

    await tester.pumpWidget(const SizedBox());

    expect(decoration.painters, hasLength(1));
    expect(decoration.painters.last.disposed, true);
  });

  testWidgets('DecoratedSliver can update box painter', (WidgetTester tester) async {
    final TestDecoration decorationA = TestDecoration();
    final TestDecoration decorationB = TestDecoration();

    Decoration activateDecoration = decorationA;
    late StateSetter localSetState;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              localSetState = setState;
              return CustomScrollView(
                slivers: <Widget>[
                  DecoratedSliver(
                    decoration: activateDecoration,
                    sliver: const SliverToBoxAdapter(child: SizedBox(width: 100, height: 100)),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    expect(decorationA.painters, hasLength(1));
    expect(decorationA.painters.last.paintCount, 1);
    expect(decorationB.painters, hasLength(0));

    localSetState(() {
      activateDecoration = decorationB;
    });
    await tester.pump();

    expect(decorationA.painters, hasLength(1));
    expect(decorationB.painters, hasLength(1));
    expect(decorationB.painters.last.paintCount, 1);
  });

  testWidgets('DecoratedSliver can update DecorationPosition', (WidgetTester tester) async {
    final TestDecoration decoration = TestDecoration();

    DecorationPosition activePosition = DecorationPosition.foreground;
    late StateSetter localSetState;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              localSetState = setState;
              return CustomScrollView(
                slivers: <Widget>[
                  DecoratedSliver(
                    decoration: decoration,
                    position: activePosition,
                    sliver: const SliverToBoxAdapter(child: SizedBox(width: 100, height: 100)),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    expect(decoration.painters, hasLength(1));
    expect(decoration.painters.last.paintCount, 1);

    localSetState(() {
      activePosition = DecorationPosition.background;
    });
    await tester.pump();

    expect(decoration.painters, hasLength(1));
    expect(decoration.painters.last.paintCount, 2);
  });

  testWidgets('DecoratedSliver golden test', (WidgetTester tester) async {
    const BoxDecoration decoration = BoxDecoration(color: Colors.blue);

    final Key backgroundKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RepaintBoundary(
            key: backgroundKey,
            child: CustomScrollView(
              slivers: <Widget>[
                DecoratedSliver(
                  decoration: decoration,
                  sliver: SliverPadding(
                    padding: const EdgeInsets.all(16),
                    sliver: SliverList(
                      delegate: SliverChildListDelegate.fixed(<Widget>[
                        Container(height: 100, color: Colors.red),
                        Container(height: 100, color: Colors.yellow),
                        Container(height: 100, color: Colors.red),
                      ]),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byKey(backgroundKey),
      matchesGoldenFile('decorated_sliver.moon.background.png'),
    );

    final Key foregroundKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RepaintBoundary(
            key: foregroundKey,
            child: CustomScrollView(
              slivers: <Widget>[
                DecoratedSliver(
                  decoration: decoration,
                  position: DecorationPosition.foreground,
                  sliver: SliverPadding(
                    padding: const EdgeInsets.all(16),
                    sliver: SliverList(
                      delegate: SliverChildListDelegate.fixed(<Widget>[
                        Container(height: 100, color: Colors.red),
                        Container(height: 100, color: Colors.yellow),
                        Container(height: 100, color: Colors.red),
                      ]),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byKey(foregroundKey),
      matchesGoldenFile('decorated_sliver.moon.foreground.png'),
    );
  });

  testWidgets('DecoratedSliver paints its border correctly vertically', (
    WidgetTester tester,
  ) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300,
            width: 100,
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverToBoxAdapter(child: SizedBox(width: 100, height: 500)),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    controller.jumpTo(200);
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(0.5, -199.5) & const Size(99, 499),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets('DecoratedSliver paints its border correctly vertically reverse', (
    WidgetTester tester,
  ) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300,
            width: 100,
            child: CustomScrollView(
              controller: controller,
              reverse: true,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverToBoxAdapter(child: SizedBox(width: 100, height: 500)),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    controller.jumpTo(200);
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(0.5, -199.5) & const Size(99, 499),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets('DecoratedSliver paints its border correctly horizontally', (
    WidgetTester tester,
  ) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 100,
            width: 300,
            child: CustomScrollView(
              scrollDirection: Axis.horizontal,
              controller: controller,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverToBoxAdapter(child: SizedBox(width: 500, height: 100)),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    controller.jumpTo(200);
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(-199.5, 0.5) & const Size(499, 99),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets('DecoratedSliver paints its border correctly horizontally reverse', (
    WidgetTester tester,
  ) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 100,
            width: 300,
            child: CustomScrollView(
              scrollDirection: Axis.horizontal,
              reverse: true,
              controller: controller,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverToBoxAdapter(child: SizedBox(width: 500, height: 100)),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    controller.jumpTo(200);
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(-199.5, 0.5) & const Size(499, 99),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets('DecoratedSliver works with SliverMainAxisGroup', (WidgetTester tester) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 100,
            width: 300,
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverMainAxisGroup(
                    slivers: <Widget>[
                      SliverToBoxAdapter(child: SizedBox(height: 100)),
                      SliverToBoxAdapter(child: SizedBox(height: 100)),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(0.5, 0.5) & const Size(299, 199),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets('DecoratedSliver works with SliverCrossAxisGroup', (WidgetTester tester) async {
    const Key key = Key('DecoratedSliver with border');
    const Color black = Color(0xFF000000);
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 100,
            width: 300,
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                DecoratedSliver(
                  key: key,
                  decoration: BoxDecoration(border: Border.all()),
                  sliver: const SliverCrossAxisGroup(
                    slivers: <Widget>[
                      SliverToBoxAdapter(child: SizedBox(height: 100)),
                      SliverToBoxAdapter(child: SizedBox(height: 100)),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(
      find.byKey(key),
      paints..rect(
        rect: const Offset(0.5, 0.5) & const Size(299, 99),
        color: black,
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
      ),
    );
  });

  testWidgets(
    'DecoratedSliver draws only up to the bottom cache when sliver has infinite scroll extent',
    (WidgetTester tester) async {
      const Key key = Key('DecoratedSliver with border');
      const Color black = Color(0xFF000000);
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              height: 100,
              width: 300,
              child: CustomScrollView(
                controller: controller,
                slivers: <Widget>[
                  DecoratedSliver(
                    key: key,
                    decoration: BoxDecoration(border: Border.all()),
                    sliver: SliverList.builder(
                      itemBuilder: (BuildContext context, int index) => const SizedBox(height: 100),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
      expect(
        find.byKey(key),
        paints..rect(
          rect: const Offset(0.5, 0.5) & const Size(299, 349),
          color: black,
          style: PaintingStyle.stroke,
          strokeWidth: 1.0,
        ),
      );
      controller.jumpTo(200);
      await tester.pumpAndSettle();
      // Note that the bottom edge is of the rect is the same as above.
      expect(
        find.byKey(key),
        paints..rect(
          rect: const Offset(0.5, -199.5) & const Size(299, 549),
          color: black,
          style: PaintingStyle.stroke,
          strokeWidth: 1.0,
        ),
      );
    },
  );
}

class TestDecoration extends Decoration {
  final List<TestBoxPainter> painters = <TestBoxPainter>[];

  @override
  BoxPainter createBoxPainter([VoidCallback? onChanged]) {
    final TestBoxPainter painter = TestBoxPainter();
    painters.add(painter);
    return painter;
  }
}

class TestBoxPainter extends BoxPainter {
  Offset? lastOffset;
  ImageConfiguration? lastConfiguration;
  bool disposed = false;
  int paintCount = 0;

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    lastOffset = offset;
    lastConfiguration = configuration;
    paintCount += 1;
  }

  @override
  void dispose() {
    assert(!disposed);
    disposed = true;
    super.dispose();
  }
}
