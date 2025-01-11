// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CarouselView defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    final ColorScheme colorScheme = theme.colorScheme;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: CarouselView(
            itemExtent: 200,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Finder carouselViewMaterial =
        find.descendant(of: find.byType(CarouselView), matching: find.byType(Material)).first;

    final Material material = tester.widget<Material>(carouselViewMaterial);
    expect(material.clipBehavior, Clip.antiAlias);
    expect(material.color, colorScheme.surface);
    expect(material.elevation, 0.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0))),
    );
  });

  testWidgets('CarouselView items customization', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: CarouselView(
            padding: const EdgeInsets.all(20.0),
            backgroundColor: Colors.amber,
            elevation: 10.0,
            shape: const StadiumBorder(),
            overlayColor: WidgetStateProperty.resolveWith((Set<WidgetState> states) {
              if (states.contains(WidgetState.pressed)) {
                return Colors.yellow;
              }
              if (states.contains(WidgetState.hovered)) {
                return Colors.red;
              }
              if (states.contains(WidgetState.focused)) {
                return Colors.purple;
              }
              return null;
            }),
            itemExtent: 200,
            children: List<Widget>.generate(10, (int index) {
              if (index == 0) {
                return Center(key: key, child: Center(child: Text('Item $index')));
              }
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Finder carouselViewMaterial =
        find.descendant(of: find.byType(CarouselView), matching: find.byType(Material)).first;

    expect(
      tester.getSize(carouselViewMaterial).width,
      200 - 20 - 20,
    ); // Padding is 20 on both side.
    final Material material = tester.widget<Material>(carouselViewMaterial);
    expect(material.color, Colors.amber);
    expect(material.elevation, 10.0);
    expect(material.shape, const StadiumBorder());

    SplashController splashController() {
      final BuildContext context = tester.element(
        find.descendant(of: carouselViewMaterial, matching: find.byType(InkWell)),
      );
      return Material.of(context);
    }

    // On hovered.
    final TestGesture gesture = await hoverPointerOverCarouselItem(tester, key);
    await tester.pumpAndSettle();
    expect(splashController(), paints..rect(color: Colors.red.withOpacity(1.0)));

    // On pressed.
    await tester.pumpAndSettle();
    await gesture.down(tester.getCenter(find.byKey(key)));
    await tester.pumpAndSettle();
    expect(
      splashController(),
      paints
        ..rect()
        ..rect(color: Colors.yellow.withOpacity(1.0)),
    );

    await tester.pumpAndSettle();
    await gesture.up();
    await gesture.removePointer();

    // On focused.
    final Element inkWellElement = tester.element(
      find.descendant(of: carouselViewMaterial, matching: find.byType(InkWell)),
    );
    expect(inkWellElement.widget, isA<InkWell>());
    final InkWell inkWell = inkWellElement.widget as InkWell;

    const MaterialState state = MaterialState.focused;

    // Check overlay color in focused state.
    expect(inkWell.overlayColor?.resolve(<WidgetState>{state}), Colors.purple);
  });

  testWidgets('CarouselView respects onTap', (WidgetTester tester) async {
    final List<Key> keys = List<Key>.generate(10, (_) => UniqueKey());
    final ThemeData theme = ThemeData();
    int tapIndex = 0;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: CarouselView(
            itemExtent: 50,
            onTap: (int index) {
              tapIndex = index;
            },
            children: List<Widget>.generate(10, (int index) {
              return Center(key: keys[index], child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Finder item1 = find.byKey(keys[1]);
    await tester.tap(find.ancestor(of: item1, matching: find.byType(Stack)));
    await tester.pump();
    expect(tapIndex, 1);

    final Finder item2 = find.byKey(keys[2]);
    await tester.tap(find.ancestor(of: item2, matching: find.byType(Stack)));
    await tester.pump();
    expect(tapIndex, 2);
  });

  testWidgets('CarouselView layout (Uncontained layout)', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 250,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Size viewportSize = MediaQuery.sizeOf(tester.element(find.byType(CarouselView)));
    expect(viewportSize, const Size(800, 600));

    expect(find.text('Item 0'), findsOneWidget);
    final Rect rect0 = tester.getRect(getItem(0));
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 250.0, 600.0));

    expect(find.text('Item 1'), findsOneWidget);
    final Rect rect1 = tester.getRect(getItem(1));
    expect(rect1, const Rect.fromLTRB(250.0, 0.0, 500.0, 600.0));

    expect(find.text('Item 2'), findsOneWidget);
    final Rect rect2 = tester.getRect(getItem(2));
    expect(rect2, const Rect.fromLTRB(500.0, 0.0, 750.0, 600.0));

    expect(find.text('Item 3'), findsOneWidget);
    final Rect rect3 = tester.getRect(getItem(3));
    expect(rect3, const Rect.fromLTRB(750.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 4'), findsNothing);
  });

  testWidgets('CarouselView.weighted layout', (WidgetTester tester) async {
    Widget buildCarouselView({required List<int> weights}) {
      return MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            flexWeights: weights,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCarouselView(weights: <int>[4, 3, 2, 1]));

    final Size viewportSize = MediaQuery.of(tester.element(find.byType(CarouselView))).size;
    expect(viewportSize, const Size(800, 600));

    expect(find.text('Item 0'), findsOneWidget);
    Rect rect0 = tester.getRect(getItem(0));
    // Item width is 4/10 of the viewport.
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 320.0, 600.0));

    expect(find.text('Item 1'), findsOneWidget);
    Rect rect1 = tester.getRect(getItem(1));
    // Item width is 3/10 of the viewport.
    expect(rect1, const Rect.fromLTRB(320.0, 0.0, 560.0, 600.0));

    expect(find.text('Item 2'), findsOneWidget);
    final Rect rect2 = tester.getRect(getItem(2));
    // Item width is 2/10 of the viewport.
    expect(rect2, const Rect.fromLTRB(560.0, 0.0, 720.0, 600.0));

    expect(find.text('Item 3'), findsOneWidget);
    final Rect rect3 = tester.getRect(getItem(3));
    // Item width is 1/10 of the viewport.
    expect(rect3, const Rect.fromLTRB(720.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 4'), findsNothing);

    // Test shorter weight list.
    await tester.pumpWidget(buildCarouselView(weights: <int>[7, 1]));
    await tester.pumpAndSettle();
    expect(viewportSize, const Size(800, 600));

    expect(find.text('Item 0'), findsOneWidget);
    rect0 = tester.getRect(getItem(0));
    // Item width is 7/8 of the viewport.
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 700.0, 600.0));

    expect(find.text('Item 1'), findsOneWidget);
    rect1 = tester.getRect(getItem(1));
    // Item width is 1/8 of the viewport.
    expect(rect1, const Rect.fromLTRB(700.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 2'), findsNothing);
  });

  testWidgets('CarouselController initialItem', (WidgetTester tester) async {
    final CarouselController controller = CarouselController(initialItem: 5);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            controller: controller,
            itemExtent: 400,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Size viewportSize = MediaQuery.sizeOf(tester.element(find.byType(CarouselView)));
    expect(viewportSize, const Size(800, 600));

    expect(find.text('Item 5'), findsOneWidget);
    final Rect rect5 = tester.getRect(getItem(5));
    // Item width is 400.
    expect(rect5, const Rect.fromLTRB(0.0, 0.0, 400.0, 600.0));

    expect(find.text('Item 6'), findsOneWidget);
    final Rect rect6 = tester.getRect(getItem(6));
    // Item width is 400.
    expect(rect6, const Rect.fromLTRB(400.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 4'), findsNothing);
    expect(find.text('Item 7'), findsNothing);
  });

  testWidgets('CarouselView.weighted respects CarouselController.initialItem', (
    WidgetTester tester,
  ) async {
    final CarouselController controller = CarouselController(initialItem: 5);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            controller: controller,
            flexWeights: const <int>[7, 1],
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Size viewportSize = MediaQuery.of(tester.element(find.byType(CarouselView))).size;
    expect(viewportSize, const Size(800, 600));

    expect(find.text('Item 5'), findsOneWidget);
    final Rect rect5 = tester.getRect(getItem(5));
    // Item width is 7/8 of the viewport.
    expect(rect5, const Rect.fromLTRB(0.0, 0.0, 700.0, 600.0));

    expect(find.text('Item 6'), findsOneWidget);
    final Rect rect6 = tester.getRect(getItem(6));
    // Item width is 1/8 of the viewport.
    expect(rect6, const Rect.fromLTRB(700.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 4'), findsNothing);
    expect(find.text('Item 7'), findsNothing);
  });

  testWidgets('The initialItem should be the first item with expanded size(max extent)', (
    WidgetTester tester,
  ) async {
    final CarouselController controller = CarouselController(initialItem: 5);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            controller: controller,
            flexWeights: const <int>[1, 8, 1],
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    final Size viewportSize = MediaQuery.of(tester.element(find.byType(CarouselView))).size;
    expect(viewportSize, const Size(800, 600));

    // Item 5 should have be the expanded item.
    expect(find.text('Item 5'), findsOneWidget);
    final Rect rect5 = tester.getRect(getItem(5));
    // Item width is 8/10 of the viewport.
    expect(rect5, const Rect.fromLTRB(80.0, 0.0, 720.0, 600.0));

    expect(find.text('Item 6'), findsOneWidget);
    final Rect rect6 = tester.getRect(getItem(6));
    // Item width is 1/10 of the viewport.
    expect(rect6, const Rect.fromLTRB(720.0, 0.0, 800.0, 600.0));

    expect(find.text('Item 4'), findsOneWidget);
    final Rect rect4 = tester.getRect(getItem(4));
    // Item width is 1/10 of the viewport.
    expect(rect4, const Rect.fromLTRB(0.0, 0.0, 80.0, 600.0));

    expect(find.text('Item 7'), findsNothing);
  });

  testWidgets('CarouselView respects itemSnapping', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemSnapping: true,
            itemExtent: 300,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    void checkOriginalExpectations() {
      expect(getItem(0), findsOneWidget);
      expect(getItem(1), findsOneWidget);
      expect(getItem(2), findsOneWidget);
      expect(getItem(3), findsNothing);
    }

    checkOriginalExpectations();

    // Snap back to the original item.
    await tester.drag(getItem(0), const Offset(-150, 0));
    await tester.pumpAndSettle();

    checkOriginalExpectations();

    // Snap back to the original item.
    await tester.drag(getItem(0), const Offset(100, 0));
    await tester.pumpAndSettle();

    checkOriginalExpectations();

    // Snap to the next item.
    await tester.drag(getItem(0), const Offset(-200, 0));
    await tester.pumpAndSettle();

    expect(getItem(0), findsNothing);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
  });

  testWidgets('CarouselView.weighted respects itemSnapping', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            itemSnapping: true,
            consumeMaxWeight: false,
            flexWeights: const <int>[1, 7],
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    void checkOriginalExpectations() {
      expect(getItem(0), findsOneWidget);
      expect(getItem(1), findsOneWidget);
      expect(getItem(2), findsNothing);
    }

    checkOriginalExpectations();

    // Snap back to the original item.
    await tester.drag(getItem(0), const Offset(-20, 0));
    await tester.pumpAndSettle();

    checkOriginalExpectations();

    // Snap back to the original item.
    await tester.drag(getItem(0), const Offset(50, 0));
    await tester.pumpAndSettle();

    checkOriginalExpectations();

    // Snap to the next item.
    await tester.drag(getItem(0), const Offset(-70, 0));
    await tester.pumpAndSettle();

    expect(getItem(0), findsNothing);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsNothing);
  });

  testWidgets('CarouselView respect itemSnapping when fling', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemSnapping: true,
            itemExtent: 300,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    // Show item 0, 1, and 2.
    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsNothing);

    // Snap to the next item. Show item 1, 2 and 3.
    await tester.fling(getItem(0), const Offset(-100, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(0), findsNothing);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsNothing);

    // Snap to the next item. Show item 2, 3 and 4.
    await tester.fling(getItem(1), const Offset(-100, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(0), findsNothing);
    expect(getItem(1), findsNothing);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsOneWidget);
    expect(getItem(5), findsNothing);

    // Fling back to the previous item. Show item 1, 2 and 3.
    await tester.fling(getItem(2), const Offset(100, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsNothing);
  });

  testWidgets('CarouselView.weighted respect itemSnapping when fling', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            itemSnapping: true,
            consumeMaxWeight: false,
            flexWeights: const <int>[1, 8, 1],
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('$index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    Finder getItem(int index) => find.descendant(
      of: find.byType(CarouselView),
      matching: find.ancestor(of: find.text('$index'), matching: find.byType(Padding)),
    );

    // Show item 0, 1, and 2.
    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsNothing);

    // Should snap to item 2 because of a long drag(-100). Show item 2, 3 and 4.
    await tester.fling(getItem(0), const Offset(-100, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(0), findsNothing);
    expect(getItem(1), findsNothing);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsOneWidget);

    // Fling to the next item (item 3). Show item 3, 4 and 5.
    await tester.fling(getItem(2), const Offset(-50, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(2), findsNothing);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsOneWidget);
    expect(getItem(5), findsOneWidget);

    // Fling back to the previous item. Show item 2, 3 and 4.
    await tester.fling(getItem(3), const Offset(50, 0), 800);
    await tester.pumpAndSettle();

    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsOneWidget);
    expect(getItem(5), findsNothing);
  });

  testWidgets('CarouselView respects scrollingDirection: Axis.vertical', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 200,
            padding: EdgeInsets.zero,
            scrollDirection: Axis.vertical,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsNothing);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item width is 200 of the viewport.
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0));

    // Simulate a scroll up
    await tester.drag(
      find.byType(CarouselView),
      const Offset(0, -200),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pumpAndSettle();
    expect(getItem(0), findsNothing);
    expect(getItem(3), findsOneWidget);
  });

  testWidgets('CarouselView.weighted respects scrollingDirection: Axis.vertical', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            flexWeights: const <int>[3, 2, 1],
            padding: EdgeInsets.zero,
            scrollDirection: Axis.vertical,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsNothing);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item width is 3/6 of the viewport.
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 800.0, 300.0));

    // Simulate a scroll up
    await tester.drag(
      find.byType(CarouselView),
      const Offset(0, -300),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pumpAndSettle();
    expect(getItem(0), findsNothing);
    expect(getItem(3), findsOneWidget);
  });

  testWidgets(
    'CarouselView.weighted respects scrollingDirection: Axis.vertical + itemSnapping: true',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CarouselView.weighted(
              itemSnapping: true,
              flexWeights: const <int>[3, 2, 1],
              scrollDirection: Axis.vertical,
              children: List<Widget>.generate(10, (int index) {
                return Center(child: Text('Item $index'));
              }),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(getItem(0), findsOneWidget);
      expect(getItem(1), findsOneWidget);
      expect(getItem(2), findsOneWidget);
      expect(getItem(3), findsNothing);
      final Rect rect0 = tester.getRect(getItem(0));
      // Item width is 3/6 of the viewport.
      expect(rect0, const Rect.fromLTRB(0.0, 0.0, 800.0, 300.0));

      // Simulate a scroll up but less than half of the leading item, the leading
      // item should go back to the original position because itemSnapping is set
      // to true.
      await tester.drag(
        find.byType(CarouselView),
        const Offset(0, -149),
        kind: PointerDeviceKind.trackpad,
      );
      await tester.pumpAndSettle();
      expect(getItem(0), findsOneWidget);
      expect(getItem(1), findsOneWidget);
      expect(getItem(2), findsOneWidget);
      expect(getItem(3), findsNothing);

      // Simulate a scroll up more than half of the leading item, the leading
      // item continue to scrolling and will disappear when animation ends because
      // itemSnapping is set to true.
      await tester.drag(
        find.byType(CarouselView),
        const Offset(0, -151),
        kind: PointerDeviceKind.trackpad,
      );
      await tester.pumpAndSettle();
      expect(getItem(0), findsNothing);
      expect(getItem(3), findsOneWidget);
    },
  );

  testWidgets('CarouselView respects reverse', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 200,
            reverse: true,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item 0 should be placed on the end of the screen.
    expect(rect0, const Rect.fromLTRB(600.0, 0.0, 800.0, 600.0));

    expect(getItem(1), findsOneWidget);
    final Rect rect1 = tester.getRect(getItem(1));
    // Item 1 should be placed before item 0.
    expect(rect1, const Rect.fromLTRB(400.0, 0.0, 600.0, 600.0));

    expect(getItem(2), findsOneWidget);
    final Rect rect2 = tester.getRect(getItem(2));
    // Item 2 should be placed before item 1.
    expect(rect2, const Rect.fromLTRB(200.0, 0.0, 400.0, 600.0));

    expect(getItem(3), findsOneWidget);
    final Rect rect3 = tester.getRect(getItem(3));
    // Item 3 should be placed before item 2.
    expect(rect3, const Rect.fromLTRB(0.0, 0.0, 200.0, 600.0));
  });

  testWidgets('CarouselView.weighted respects reverse', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            flexWeights: const <int>[4, 3, 2, 1],
            reverse: true,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item 0 should be placed on the end of the screen.
    const int item0Width = 80 * 4;
    expect(rect0, const Rect.fromLTRB(800.0 - item0Width, 0.0, 800.0, 600.0));

    expect(getItem(1), findsOneWidget);
    final Rect rect1 = tester.getRect(getItem(1));
    // Item 1 should be placed before item 0.
    const int item1Width = 80 * 3;
    expect(
      rect1,
      const Rect.fromLTRB(800.0 - item0Width - item1Width, 0.0, 800.0 - item0Width, 600.0),
    );

    expect(getItem(2), findsOneWidget);
    final Rect rect2 = tester.getRect(getItem(2));
    // Item 2 should be placed before item 1.
    const int item2Width = 80 * 2;
    expect(
      rect2,
      const Rect.fromLTRB(
        800.0 - item0Width - item1Width - item2Width,
        0.0,
        800.0 - item0Width - item1Width,
        600.0,
      ),
    );

    expect(getItem(3), findsOneWidget);
    final Rect rect3 = tester.getRect(getItem(3));
    // Item 3 should be placed before item 2.
    expect(
      rect3,
      const Rect.fromLTRB(0.0, 0.0, 800.0 - item0Width - item1Width - item2Width, 600.0),
    );
  });

  testWidgets('CarouselView.weighted respects reverse + vertical scroll direction', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            reverse: true,
            flexWeights: const <int>[4, 3, 2, 1],
            scrollDirection: Axis.vertical,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item 0 should be placed on the end of the screen.
    const int item0Height = 60 * 4;
    expect(rect0, const Rect.fromLTRB(0.0, 600.0 - item0Height, 800.0, 600.0));

    expect(getItem(1), findsOneWidget);
    final Rect rect1 = tester.getRect(getItem(1));
    // Item 1 should be placed before item 0.
    const int item1Height = 60 * 3;
    expect(
      rect1,
      const Rect.fromLTRB(0.0, 600.0 - item0Height - item1Height, 800.0, 600.0 - item0Height),
    );

    expect(getItem(2), findsOneWidget);
    final Rect rect2 = tester.getRect(getItem(2));
    // Item 2 should be placed before item 1.
    const int item2Height = 60 * 2;
    expect(
      rect2,
      const Rect.fromLTRB(
        0.0,
        600.0 - item0Height - item1Height - item2Height,
        800.0,
        600.0 - item0Height - item1Height,
      ),
    );

    expect(getItem(3), findsOneWidget);
    final Rect rect3 = tester.getRect(getItem(3));
    // Item 3 should be placed before item 2.
    expect(
      rect3,
      const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0 - item0Height - item1Height - item2Height),
    );
  });

  testWidgets('CarouselView.weighted respects reverse + vertical scroll direction + itemSnapping', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            reverse: true,
            flexWeights: const <int>[4, 3, 2, 1],
            scrollDirection: Axis.vertical,
            itemSnapping: true,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsNothing);
    final Rect rect0 = tester.getRect(getItem(0));
    // Item height is 4/10 of the viewport.
    expect(rect0, const Rect.fromLTRB(0.0, 360.0, 800.0, 600.0));

    // Simulate a scroll down but less than half of the leading item, the leading
    // item should go back to the original position because itemSnapping is set
    // to true.
    await tester.drag(
      find.byType(CarouselView),
      const Offset(0, 240 / 2 - 1),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pumpAndSettle();
    expect(getItem(0), findsOneWidget);
    expect(getItem(1), findsOneWidget);
    expect(getItem(2), findsOneWidget);
    expect(getItem(3), findsOneWidget);
    expect(getItem(4), findsNothing);

    // Simulate a scroll down more than half of the leading item, the leading
    // item continue to scrolling and will disappear when animation ends because
    // itemSnapping is set to true.
    await tester.drag(
      find.byType(CarouselView),
      const Offset(0, 240 / 2 + 1),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pumpAndSettle();
    expect(getItem(0), findsNothing);
    expect(getItem(4), findsOneWidget);
  });

  testWidgets('CarouselView respects shrinkExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 350,
            shrinkExtent: 300,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    final Rect rect0 = tester.getRect(getItem(0));
    expect(rect0, const Rect.fromLTRB(0.0, 0.0, 350.0, 600.0));

    final Rect rect1 = tester.getRect(getItem(1));
    expect(rect1, const Rect.fromLTRB(350.0, 0.0, 700.0, 600.0));

    final Rect rect2 = tester.getRect(getItem(2));
    // The extent of item 2 is 300, and only 100 is on screen.
    expect(rect2, const Rect.fromLTRB(700.0, 0.0, 1000.0, 600.0));

    await tester.drag(
      find.byType(CarouselView),
      const Offset(-50, 0),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pump();
    // The item 0 should be pinned and has a size change from 350 to 50.
    expect(tester.getRect(getItem(0)), const Rect.fromLTRB(0.0, 0.0, 300.0, 600.0));
    // Keep dragging to left, extent of item 0 won't change (still 300) and part of item 0 will
    // be off screen.
    await tester.drag(
      find.byType(CarouselView),
      const Offset(-50, 0),
      kind: PointerDeviceKind.trackpad,
    );
    await tester.pump();
    expect(tester.getRect(getItem(0)), const Rect.fromLTRB(-50, 0.0, 250, 600));
  });

  testWidgets('CarouselView.weighted respects consumeMaxWeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            flexWeights: const <int>[1, 2, 4, 2, 1],
            itemSnapping: true,
            children: List<Widget>.generate(10, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    // The initial item is item 0. To make sure the layout stays the same, the
    // first item should be placed at the middle of the screen and there are some
    // white space as if there are two more shinked items before the first item.
    final Rect rect0 = tester.getRect(getItem(0));
    expect(rect0, const Rect.fromLTRB(240.0, 0.0, 560.0, 600.0));

    for (int i = 0; i < 7; i++) {
      await tester.drag(find.byType(CarouselView), const Offset(-80.0, 0.0));
      await tester.pumpAndSettle();
    }

    // After scrolling the carousel 7 times, the last item(item 9) should be on
    // the end of the screen.
    expect(getItem(9), findsOneWidget);
    expect(tester.getRect(getItem(9)), const Rect.fromLTRB(720.0, 0.0, 800.0, 600.0));

    // Keep snapping twice. Item 9 should be fully expanded to the max size.
    for (int i = 0; i < 2; i++) {
      await tester.drag(find.byType(CarouselView), const Offset(-80.0, 0.0));
      await tester.pumpAndSettle();
    }
    expect(getItem(9), findsOneWidget);
    expect(tester.getRect(getItem(9)), const Rect.fromLTRB(240.0, 0.0, 560.0, 600.0));
  });

  testWidgets('The initialItem stays when the flexWeights is updated', (WidgetTester tester) async {
    final CarouselController controller = CarouselController(initialItem: 3);
    addTearDown(controller.dispose);

    Widget buildCarousel(List<int> flexWeights) {
      return MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            controller: controller,
            flexWeights: flexWeights,
            itemSnapping: true,
            children: List<Widget>.generate(20, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCarousel(<int>[1, 1, 6, 1, 1]));
    await tester.pumpAndSettle();

    expect(find.text('Item 0'), findsNothing);
    for (int i = 1; i <= 5; i++) {
      expect(find.text('Item $i'), findsOneWidget);
    }
    Rect rect3 = tester.getRect(getItem(3));
    expect(rect3.center.dx, 400.0);
    expect(rect3.center.dy, 300.0);

    expect(find.text('Item 6'), findsNothing);

    await tester.pumpWidget(buildCarousel(<int>[7, 1]));
    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsNothing);
    expect(find.text('Item 3'), findsOneWidget);
    expect(find.text('Item 4'), findsOneWidget);
    expect(find.text('Item 5'), findsNothing);

    rect3 = tester.getRect(getItem(3));
    expect(rect3, const Rect.fromLTRB(0.0, 0.0, 700.0, 600.0));
    final Rect rect4 = tester.getRect(getItem(4));
    expect(rect4, const Rect.fromLTRB(700.0, 0.0, 800.0, 600.0));
  });

  testWidgets('The item that currently occupies max weight stays when the flexWeights is updated', (
    WidgetTester tester,
  ) async {
    final CarouselController controller = CarouselController(initialItem: 3);
    addTearDown(controller.dispose);

    Widget buildCarousel(List<int> flexWeights) {
      return MaterialApp(
        home: Scaffold(
          body: CarouselView.weighted(
            controller: controller,
            flexWeights: flexWeights,
            itemSnapping: true,
            children: List<Widget>.generate(20, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCarousel(<int>[1, 1, 6, 1, 1]));
    await tester.pumpAndSettle();
    // Item 3 is centered.
    final Rect rect3 = tester.getRect(getItem(3));
    expect(rect3.center.dx, 400.0);
    expect(rect3.center.dy, 300.0);

    // Simulate scroll to right and show item 4 to be the centered max item.
    await tester.drag(find.byType(CarouselView), const Offset(-80.0, 0.0));
    await tester.pumpAndSettle();

    expect(find.text('Item 1'), findsNothing);
    for (int i = 2; i <= 6; i++) {
      expect(find.text('Item $i'), findsOneWidget);
    }
    Rect rect4 = tester.getRect(getItem(4));
    expect(rect4.center.dx, 400.0);
    expect(rect4.center.dy, 300.0);

    await tester.pumpWidget(buildCarousel(<int>[7, 1]));
    await tester.pumpAndSettle();

    rect4 = tester.getRect(getItem(4));
    expect(rect4, const Rect.fromLTRB(0.0, 0.0, 700.0, 600.0));
    final Rect rect5 = tester.getRect(getItem(5));
    expect(rect5, const Rect.fromLTRB(700.0, 0.0, 800.0, 600.0));
  });

  testWidgets('The initialItem stays when the itemExtent is updated', (WidgetTester tester) async {
    final CarouselController controller = CarouselController(initialItem: 3);
    addTearDown(controller.dispose);

    Widget buildCarousel(double itemExtent) {
      return MaterialApp(
        home: Scaffold(
          body: CarouselView(
            controller: controller,
            itemExtent: itemExtent,
            itemSnapping: true,
            children: List<Widget>.generate(20, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCarousel(234.0));
    await tester.pumpAndSettle();

    Offset rect3BottomRight = tester.getRect(getItem(3)).bottomRight;
    expect(rect3BottomRight.dx, 234.0);
    expect(rect3BottomRight.dy, 600.0);

    await tester.pumpWidget(buildCarousel(400.0));
    await tester.pumpAndSettle();

    rect3BottomRight = tester.getRect(getItem(3)).bottomRight;
    expect(rect3BottomRight.dx, 400.0);
    expect(rect3BottomRight.dy, 600.0);

    await tester.pumpWidget(buildCarousel(100.0));
    await tester.pumpAndSettle();

    rect3BottomRight = tester.getRect(getItem(3)).bottomRight;
    expect(rect3BottomRight.dx, 100.0);
    expect(rect3BottomRight.dy, 600.0);
  });

  testWidgets(
    'While scrolling, one extra item will show at the end of the screen during items transition',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CarouselView.weighted(
              flexWeights: const <int>[1, 2, 4, 2, 1],
              consumeMaxWeight: false,
              children: List<Widget>.generate(10, (int index) {
                return Center(child: Text('Item $index'));
              }),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      for (int i = 0; i < 5; i++) {
        expect(getItem(i), findsOneWidget);
      }

      // Drag the first item to the middle. So the progress for the first item size change
      // is 50%, original width is 80.
      await tester.drag(getItem(0), const Offset(-40.0, 0.0), kind: PointerDeviceKind.trackpad);
      await tester.pump();
      expect(tester.getRect(getItem(0)).width, 40.0);

      // The size of item 1 is changing to the size of item 0, so the size of item 1
      // now should be item1.originalExtent - 50% * (item1.extent - item0.extent).
      // Item1 originally should be 2/(1+2+4+2+1) * 800 = 160.0.
      expect(tester.getRect(getItem(1)).width, 160 - 0.5 * (160 - 80));

      // The extent of item 2 should be: item2.originalExtent - 50% * (item2.extent - item1.extent).
      // the extent of item 2 originally should be 4/(1+2+4+2+1) * 800 = 320.0.
      expect(tester.getRect(getItem(2)).width, 320 - 0.5 * (320 - 160));

      // The extent of item 3 should be: item3.originalExtent + 50% * (item2.extent - item3.extent).
      // the extent of item 3 originally should be 2/(1+2+4+2+1) * 800 = 160.0.
      expect(tester.getRect(getItem(3)).width, 160 + 0.5 * (320 - 160));

      // The extent of item 4 should be: item4.originalExtent + 50% * (item3.extent - item4.extent).
      // the extent of item 4 originally should be 1/(1+2+4+2+1) * 800 = 80.0.
      expect(tester.getRect(getItem(4)).width, 80 + 0.5 * (160 - 80));

      // The sum of the first 5 items during transition is less than the screen width.
      double sum = 0;
      for (int i = 0; i < 5; i++) {
        sum += tester.getRect(getItem(i)).width;
      }
      expect(sum, lessThan(MediaQuery.of(tester.element(find.byType(CarouselView))).size.width));
      final double difference =
          MediaQuery.of(tester.element(find.byType(CarouselView))).size.width - sum;

      // One more item should show on screen to fill the rest of the viewport.
      expect(getItem(5), findsOneWidget);
      expect(tester.getRect(getItem(5)).width, difference);
    },
  );

  testWidgets('Updating CarouselView does not cause exception', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/152787
    bool isLight = true;
    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            theme: Theme.of(
              context,
            ).copyWith(brightness: isLight ? Brightness.light : Brightness.dark),
            home: Scaffold(
              appBar: AppBar(
                actions: <Widget>[
                  Switch(
                    value: isLight,
                    onChanged: (bool value) {
                      setState(() {
                        isLight = value;
                      });
                    },
                  ),
                ],
              ),
              body: CarouselView(
                itemExtent: 100,
                children: List<Widget>.generate(10, (int index) {
                  return Center(child: Text('Item $index'));
                }),
              ),
            ),
          );
        },
      ),
    );
    await tester.pumpAndSettle();
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    // No exception.
    expect(tester.takeException(), isNull);
  });

  testWidgets('The shrinkExtent should keep the same when the item is tapped', (
    WidgetTester tester,
  ) async {
    final List<Widget> children = List<Widget>.generate(20, (int index) {
      return Center(child: Text('Item $index'));
    });

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Scaffold(
              body: Center(
                child: ConstrainedBox(
                  constraints: const BoxConstraints(maxHeight: 200),
                  child: CarouselView(
                    itemExtent: 330,
                    onTap: (int idx) => setState(() {}),
                    children: children,
                  ),
                ),
              ),
            ),
          );
        },
      ),
    );

    await tester.pumpAndSettle();

    expect(tester.getRect(getItem(0)).width, 330.0);

    final Finder item1 = find.text('Item 1');
    await tester.tap(find.ancestor(of: item1, matching: find.byType(Stack)));

    await tester.pumpAndSettle();

    expect(tester.getRect(getItem(0)).width, 330.0);
    expect(tester.getRect(getItem(1)).width, 330.0);
    // This should be less than 330.0 because the item is shrunk; width is 800.0 - 330.0 - 330.0
    expect(tester.getRect(getItem(2)).width, 140.0);
  });

  testWidgets('CarouselView onTap is clickable', (WidgetTester tester) async {
    int tappedIndex = -1;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 350,
            onTap: (int index) {
              tappedIndex = index;
            },
            children: List<Widget>.generate(3, (int index) {
              return Center(child: Text('Item $index'));
            }),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    final Finder carouselItem = find.text('Item 1');
    await tester.tap(carouselItem, warnIfMissed: false);
    await tester.pumpAndSettle();

    // Verify that the onTap callback was called with the correct index.
    expect(tappedIndex, 1);

    // Tap another item.
    final Finder anotherCarouselItem = find.text('Item 2');
    await tester.tap(anotherCarouselItem, warnIfMissed: false);
    await tester.pumpAndSettle();

    // Verify that the onTap callback was called with the new index.
    expect(tappedIndex, 2);
  });

  testWidgets('CarouselView with enableSplash true - children are not directly interactive', (
    WidgetTester tester,
  ) async {
    bool buttonPressed = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 350,
            children: List<Widget>.generate(3, (int index) {
              return Center(
                child: ElevatedButton(
                  onPressed: () => buttonPressed = true,
                  child: Text('Button $index'),
                ),
              );
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    await tester.tap(find.text('Button 1'), warnIfMissed: false);
    expect(buttonPressed, isFalse);
  });

  testWidgets('CarouselView with enableSplash false - children are directly interactive', (
    WidgetTester tester,
  ) async {
    bool buttonPressed = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CarouselView(
            itemExtent: 350,
            enableSplash: false,
            children: List<Widget>.generate(3, (int index) {
              return Center(
                child: ElevatedButton(
                  onPressed: () => buttonPressed = true,
                  child: Text('Button $index'),
                ),
              );
            }),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    await tester.tap(find.text('Button 1'));
    expect(buttonPressed, isTrue);
  });

  testWidgets(
    'CarouselView with enableSplash false - container is clickable without triggering children onTap',
    (WidgetTester tester) async {
      int tappedIndex = -1;
      bool buttonPressed = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CarouselView(
              itemExtent: 350,
              enableSplash: false,
              onTap: (int index) {
                tappedIndex = index;
              },
              children: List<Widget>.generate(3, (int index) {
                return Column(
                  children: <Widget>[
                    Text('Item $index'),
                    ElevatedButton(
                      onPressed: () => buttonPressed = true,
                      child: Text('Button $index'),
                    ),
                  ],
                );
              }),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final Finder carouselItem = find.text('Item 1');
      await tester.tap(carouselItem, warnIfMissed: false);
      await tester.pumpAndSettle();

      expect(tappedIndex, 1);
      expect(buttonPressed, false);

      final Finder anotherCarouselItem = find.text('Item 2');
      await tester.tap(anotherCarouselItem, warnIfMissed: false);
      await tester.pumpAndSettle();

      expect(tappedIndex, 2);
      expect(buttonPressed, false);

      await tester.tap(find.text('Button 1'), warnIfMissed: false);
      expect(buttonPressed, isTrue);
    },
  );
}

Finder getItem(int index) {
  return find.descendant(
    of: find.byType(CarouselView),
    matching: find.ancestor(of: find.text('Item $index'), matching: find.byType(Padding)),
  );
}

Future<TestGesture> hoverPointerOverCarouselItem(WidgetTester tester, Key key) async {
  final Offset center = tester.getCenter(find.byKey(key));
  final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);

  // On hovered.
  await gesture.addPointer();
  await gesture.moveTo(center);
  return gesture;
}
