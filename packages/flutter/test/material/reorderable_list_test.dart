// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('$ReorderableListView', () {
    const double itemHeight = 48.0;
    const List<String> originalListItems = <String>['Item 1', 'Item 2', 'Item 3', 'Item 4'];
    late List<String> listItems;

    void onReorder(int oldIndex, int newIndex) {
      if (oldIndex < newIndex) {
        newIndex -= 1;
      }
      final String element = listItems.removeAt(oldIndex);
      listItems.insert(newIndex, element);
    }

    Widget listItemToWidget(String listItem) {
      return SizedBox(
        key: Key(listItem),
        height: itemHeight,
        width: itemHeight,
        child: Text(listItem),
      );
    }

    Widget build({
      Widget? header,
      Widget? footer,
      Axis scrollDirection = Axis.vertical,
      bool reverse = false,
      EdgeInsets padding = EdgeInsets.zero,
      TextDirection textDirection = TextDirection.ltr,
      TargetPlatform? platform,
    }) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Directionality(
          textDirection: textDirection,
          child: SizedBox(
            height: itemHeight * 10,
            width: itemHeight * 10,
            child: ReorderableListView(
              header: header,
              footer: footer,
              scrollDirection: scrollDirection,
              onReorder: onReorder,
              reverse: reverse,
              padding: padding,
              children: listItems.map<Widget>(listItemToWidget).toList(),
            ),
          ),
        ),
      );
    }

    setUp(() {
      // Copy the original list into listItems.
      listItems = originalListItems.toList();
    });

    group('in vertical mode', () {
      testWidgets('reorder is not triggered when children length is less or equals to 1', (
        WidgetTester tester,
      ) async {
        bool onReorderWasCalled = false;
        final List<String> currentListItems = listItems.take(1).toList();
        final ReorderableListView reorderableListView = ReorderableListView(
          header: const Text('Header'),
          onReorder: (_, _) => onReorderWasCalled = true,
          children: currentListItems.map<Widget>(listItemToWidget).toList(),
        );
        final List<String> currentOriginalListItems = originalListItems.take(1).toList();
        await tester.pumpWidget(
          MaterialApp(home: SizedBox(height: itemHeight * 10, child: reorderableListView)),
        );
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 1')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        await drag.moveTo(tester.getBottomLeft(find.text('Item 1')) * 2);
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        await drag.up();
        expect(onReorderWasCalled, false);
        expect(currentListItems, orderedEquals(<String>['Item 1']));
      });

      testWidgets('reorders its contents only when a drag finishes', (WidgetTester tester) async {
        await tester.pumpWidget(build());
        expect(listItems, orderedEquals(originalListItems));
        final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 1')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        expect(listItems, orderedEquals(originalListItems));
        await drag.moveTo(tester.getCenter(find.text('Item 4')));
        expect(listItems, orderedEquals(originalListItems));
        await drag.up();
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 1', 'Item 4']));
      });

      testWidgets('allows reordering from the very top to the very bottom', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(build());
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(0.0, itemHeight * 2),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
      });

      testWidgets('allows reordering from the very bottom to the very top', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(build());
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 4')),
          tester.getCenter(find.text('Item 1')),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 2', 'Item 3']));
      });

      testWidgets('allows reordering inside the middle of the widget', (WidgetTester tester) async {
        await tester.pumpWidget(build());
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 3')),
          tester.getCenter(find.text('Item 2')),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 2', 'Item 4']));
      });

      testWidgets('properly reorders with a header', (WidgetTester tester) async {
        await tester.pumpWidget(build(header: const Text('Header Text')));
        expect(find.text('Header Text'), findsOneWidget);
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(0.0, itemHeight * 2),
        );
        await tester.pumpAndSettle();
        expect(find.text('Header Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
      });

      testWidgets('properly reorders with a footer', (WidgetTester tester) async {
        await tester.pumpWidget(build(footer: const Text('Footer Text')));
        expect(find.text('Footer Text'), findsOneWidget);
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(0.0, itemHeight * 2),
        );
        await tester.pumpAndSettle();
        expect(find.text('Footer Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
      });

      testWidgets('properly determines the vertical drop area extents', (
        WidgetTester tester,
      ) async {
        final Widget reorderableListView = ReorderableListView(
          onReorder: (int oldIndex, int newIndex) {},
          children: const <Widget>[
            SizedBox(key: Key('Normal item'), height: itemHeight, child: Text('Normal item')),
            SizedBox(key: Key('Tall item'), height: itemHeight * 2, child: Text('Tall item')),
            SizedBox(key: Key('Last item'), height: itemHeight, child: Text('Last item')),
          ],
        );
        await tester.pumpWidget(
          MaterialApp(home: SizedBox(height: itemHeight * 10, child: reorderableListView)),
        );

        double getListHeight() {
          final RenderSliverList listScrollView = tester.renderObject(find.byType(SliverList));
          return listScrollView.geometry!.maxPaintExtent;
        }

        const double kDraggingListHeight = 4 * itemHeight;
        // Drag a normal text item
        expect(getListHeight(), kDraggingListHeight);
        TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Normal item')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);

        // Move it
        await drag.moveTo(tester.getCenter(find.text('Last item')));
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);

        // Drop it
        await drag.up();
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);

        // Drag a tall item
        drag = await tester.startGesture(tester.getCenter(find.text('Tall item')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);
        // Move it
        await drag.moveTo(tester.getCenter(find.text('Last item')));
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);

        // Drop it
        await drag.up();
        await tester.pumpAndSettle();
        expect(getListHeight(), kDraggingListHeight);
      });

      testWidgets('Vertical drag in progress golden image', (WidgetTester tester) async {
        debugDisableShadows = false;
        final Widget reorderableListView = ReorderableListView(
          children: <Widget>[
            Container(
              key: const Key('pink'),
              width: double.infinity,
              height: itemHeight,
              color: Colors.pink,
            ),
            Container(
              key: const Key('blue'),
              width: double.infinity,
              height: itemHeight,
              color: Colors.blue,
            ),
            Container(
              key: const Key('green'),
              width: double.infinity,
              height: itemHeight,
              color: Colors.green,
            ),
          ],
          onReorder: (int oldIndex, int newIndex) {},
        );

        late final OverlayEntry entry;
        addTearDown(
          () =>
              entry
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          MaterialApp(
            home: Container(
              color: Colors.white,
              height: itemHeight * 3,
              // Wrap in an overlay so that the golden image includes the dragged item.
              child: Overlay(
                initialEntries: <OverlayEntry>[
                  entry = OverlayEntry(
                    builder: (BuildContext context) {
                      // Wrap the list in padding to test that the positioning
                      // is correct when the origin of the overlay is different
                      // from the list.
                      return Padding(padding: const EdgeInsets.all(24), child: reorderableListView);
                    },
                  ),
                ],
              ),
            ),
          ),
        );

        // Start dragging the second item.
        final TestGesture drag = await tester.startGesture(
          tester.getCenter(find.byKey(const Key('blue'))),
        );
        await tester.pump(kLongPressTimeout + kPressTimeout);

        // Drag it up to be partially over the top item.
        await drag.moveBy(const Offset(0, -itemHeight / 3));
        await tester.pumpAndSettle();

        // Should be an image of the second item overlapping the bottom of the
        // first with a gap between the first and third and a drop shadow on
        // the dragged item.
        await expectLater(
          find.byType(Overlay).last,
          matchesGoldenFile('reorderable_list_test.vertical.drop_area.png'),
        );
        debugDisableShadows = true;
      });

      testWidgets('Preserves children states when the list parent changes the order', (
        WidgetTester tester,
      ) async {
        _StatefulState findState(Key key) {
          return find
              .byElementPredicate(
                (Element element) => element.findAncestorWidgetOfExactType<_Stateful>()?.key == key,
              )
              .evaluate()
              .first
              .findAncestorStateOfType<_StatefulState>()!;
        }

        await tester.pumpWidget(
          MaterialApp(
            home: ReorderableListView(
              children: <Widget>[
                _Stateful(key: const Key('A')),
                _Stateful(key: const Key('B')),
                _Stateful(key: const Key('C')),
              ],
              onReorder: (int oldIndex, int newIndex) {},
            ),
          ),
        );
        await tester.tap(find.byKey(const Key('A')));
        await tester.pumpAndSettle();
        // Only the 'A' widget should be checked.
        expect(findState(const Key('A')).checked, true);
        expect(findState(const Key('B')).checked, false);
        expect(findState(const Key('C')).checked, false);

        await tester.pumpWidget(
          MaterialApp(
            home: ReorderableListView(
              children: <Widget>[
                _Stateful(key: const Key('B')),
                _Stateful(key: const Key('C')),
                _Stateful(key: const Key('A')),
              ],
              onReorder: (int oldIndex, int newIndex) {},
            ),
          ),
        );
        // Only the 'A' widget should be checked.
        expect(findState(const Key('B')).checked, false);
        expect(findState(const Key('C')).checked, false);
        expect(findState(const Key('A')).checked, true);
      });

      testWidgets('Preserves children states when rebuilt', (WidgetTester tester) async {
        const Key firstBox = Key('key');
        Widget build() {
          return MaterialApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: SizedBox(
                width: 100,
                height: 100,
                child: ReorderableListView(
                  children: const <Widget>[SizedBox(key: firstBox, width: 10, height: 10)],
                  onReorder: (_, _) {},
                ),
              ),
            ),
          );
        }

        // When the widget is rebuilt, the state of child should be consistent.
        await tester.pumpWidget(build());
        final Element e0 = tester.element(find.byKey(firstBox));
        await tester.pumpWidget(build());
        final Element e1 = tester.element(find.byKey(firstBox));
        expect(e0, equals(e1));
      });

      testWidgets('Uses the PrimaryScrollController when available', (WidgetTester tester) async {
        final ScrollController primary = ScrollController();
        addTearDown(primary.dispose);
        final Widget reorderableList = ReorderableListView(
          children: const <Widget>[
            SizedBox(width: 100.0, height: 100.0, key: Key('C'), child: Text('C')),
            SizedBox(width: 100.0, height: 100.0, key: Key('B'), child: Text('B')),
            SizedBox(width: 100.0, height: 100.0, key: Key('A'), child: Text('A')),
          ],
          onReorder: (int oldIndex, int newIndex) {},
        );

        Widget buildWithScrollController(ScrollController controller) {
          return MaterialApp(
            home: PrimaryScrollController(
              controller: controller,
              child: SizedBox(height: 100.0, width: 100.0, child: reorderableList),
            ),
          );
        }

        await tester.pumpWidget(buildWithScrollController(primary));
        Scrollable scrollView = tester.widget(find.byType(Scrollable));
        expect(scrollView.controller, primary);

        // Now try changing the primary scroll controller and checking that the scroll view gets updated.
        final ScrollController primary2 = ScrollController();
        addTearDown(primary2.dispose);

        await tester.pumpWidget(buildWithScrollController(primary2));
        scrollView = tester.widget(find.byType(Scrollable));
        expect(scrollView.controller, primary2);
      });

      testWidgets('Test custom ScrollController behavior when set', (WidgetTester tester) async {
        const Key firstBox = Key('C');
        const Key secondBox = Key('B');
        const Key thirdBox = Key('A');
        final ScrollController customController = ScrollController();
        addTearDown(customController.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: SizedBox(
                height: 150,
                child: ReorderableListView(
                  scrollController: customController,
                  onReorder: (int oldIndex, int newIndex) {},
                  children: const <Widget>[
                    SizedBox(width: 100.0, height: 100.0, key: firstBox, child: Text('C')),
                    SizedBox(width: 100.0, height: 100.0, key: secondBox, child: Text('B')),
                    SizedBox(width: 100.0, height: 100.0, key: thirdBox, child: Text('A')),
                  ],
                ),
              ),
            ),
          ),
        );

        // Check initial scroll offset of first list item relative to
        // the offset of the list view.
        customController.animateTo(
          40.0,
          duration: const Duration(milliseconds: 200),
          curve: Curves.linear,
        );
        await tester.pumpAndSettle();
        Offset listViewTopLeft = tester.getTopLeft(find.byType(ReorderableListView));
        Offset firstBoxTopLeft = tester.getTopLeft(find.byKey(firstBox));
        expect(firstBoxTopLeft.dy, listViewTopLeft.dy - 40.0);

        // Drag the UI to see if the scroll controller updates accordingly
        await tester.drag(find.text('B'), const Offset(0.0, -100.0));
        listViewTopLeft = tester.getTopLeft(find.byType(ReorderableListView));
        firstBoxTopLeft = tester.getTopLeft(find.byKey(firstBox));
        // Initial scroll controller offset: 40.0
        // Drag UI by 100.0 upwards vertically
        // First 20.0 px always ignored, so scroll offset is only
        // shifted by 80.0.
        // Final offset: 40.0 + 80.0 = 120.0
        // The total distance available to scroll is 300.0 - 150.0 = 150.0, or
        // height of the ReorderableListView minus height of the SizedBox. Since
        // The final offset is less than this, it's not limited.
        expect(customController.offset, 120.0);
      });

      testWidgets('ReorderableList auto scrolling is fast enough', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/121603.
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: ReorderableListView.builder(
                scrollController: controller,
                itemCount: 100,
                itemBuilder: (BuildContext context, int index) {
                  return Text('data', key: ValueKey<int>(index));
                },
                onReorder: (int oldIndex, int newIndex) {},
              ),
            ),
          ),
        );

        // Start gesture on first item.
        final TestGesture drag = await tester.startGesture(
          tester.getCenter(find.byKey(const ValueKey<int>(0))),
        );
        await tester.pump(kLongPressTimeout + kPressTimeout);
        final Offset bottomRight = tester.getBottomRight(find.byType(ReorderableListView));
        // Drag enough for move to start.
        await drag.moveTo(Offset(bottomRight.dx / 2, bottomRight.dy));
        await tester.pump();
        // Use a fixed value to make sure the default velocity scalar is bigger
        // than a certain amount.
        const double kMinimumAllowedAutoScrollDistancePer5ms = 1.7;

        await tester.pump(const Duration(milliseconds: 5));
        expect(controller.offset, greaterThan(kMinimumAllowedAutoScrollDistancePer5ms));
        await tester.pump(const Duration(milliseconds: 5));
        expect(controller.offset, greaterThan(kMinimumAllowedAutoScrollDistancePer5ms * 2));
        await tester.pump(const Duration(milliseconds: 5));
        expect(controller.offset, greaterThan(kMinimumAllowedAutoScrollDistancePer5ms * 3));
        await tester.pump(const Duration(milliseconds: 5));
        expect(controller.offset, greaterThan(kMinimumAllowedAutoScrollDistancePer5ms * 4));
      });

      testWidgets('Still builds when no PrimaryScrollController is available', (
        WidgetTester tester,
      ) async {
        final Widget reorderableList = ReorderableListView(
          children: const <Widget>[
            SizedBox(width: 100.0, height: 100.0, key: Key('C'), child: Text('C')),
            SizedBox(width: 100.0, height: 100.0, key: Key('B'), child: Text('B')),
            SizedBox(width: 100.0, height: 100.0, key: Key('A'), child: Text('A')),
          ],
          onReorder: (int oldIndex, int newIndex) {},
        );

        late final OverlayEntry entry;
        addTearDown(
          () =>
              entry
                ..remove()
                ..dispose(),
        );

        final Widget overlay = Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(builder: (BuildContext context) => reorderableList),
          ],
        );
        final Widget boilerplate = Localizations(
          locale: const Locale('en'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultMaterialLocalizations.delegate,
            DefaultWidgetsLocalizations.delegate,
          ],
          child: SizedBox(
            width: 100.0,
            height: 100.0,
            child: Directionality(textDirection: TextDirection.ltr, child: overlay),
          ),
        );
        await expectLater(() => tester.pumpWidget(boilerplate), returnsNormally);
      });

      group('Accessibility (a11y/Semantics)', () {
        Map<CustomSemanticsAction, VoidCallback> getSemanticsActions(int index) {
          final Semantics semantics =
              find
                      .ancestor(
                        of: find.byKey(Key(listItems[index])),
                        matching: find.byType(Semantics),
                      )
                      .evaluate()
                      .first
                      .widget
                  as Semantics;
          return semantics.properties.customSemanticsActions!;
        }

        const CustomSemanticsAction moveToStart = CustomSemanticsAction(label: 'Move to the start');
        const CustomSemanticsAction moveToEnd = CustomSemanticsAction(label: 'Move to the end');
        const CustomSemanticsAction moveUp = CustomSemanticsAction(label: 'Move up');
        const CustomSemanticsAction moveDown = CustomSemanticsAction(label: 'Move down');

        testWidgets('Provides the correct accessibility actions in LTR and RTL modes', (
          WidgetTester tester,
        ) async {
          // The a11y actions for a vertical list are the same in LTR and RTL modes.
          final SemanticsHandle handle = tester.ensureSemantics();
          for (final TextDirection direction in TextDirection.values) {
            await tester.pumpWidget(build());

            // The first item can be moved down or to the end.
            final Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions =
                getSemanticsActions(0);
            expect(
              firstSemanticsActions.length,
              2,
              reason: 'The first list item should have 2 custom actions with $direction.',
            );
            expect(
              firstSemanticsActions.containsKey(moveToStart),
              false,
              reason: 'The first item cannot `Move to the start` with $direction.',
            );
            expect(
              firstSemanticsActions.containsKey(moveUp),
              false,
              reason: 'The first item cannot `Move up` with $direction.',
            );
            expect(
              firstSemanticsActions.containsKey(moveDown),
              true,
              reason: 'The first item should be able to `Move down` with $direction.',
            );
            expect(
              firstSemanticsActions.containsKey(moveToEnd),
              true,
              reason: 'The first item should be able to `Move to the end` with $direction.',
            );

            // Items in the middle can be moved to the start, end, up or down.
            for (int i = 1; i < listItems.length - 1; i += 1) {
              final Map<CustomSemanticsAction, VoidCallback> ithSemanticsActions =
                  getSemanticsActions(i);
              expect(
                ithSemanticsActions.length,
                4,
                reason: 'List item $i should have 4 custom actions with $direction.',
              );
              expect(
                ithSemanticsActions.containsKey(moveToStart),
                true,
                reason: 'List item $i should be able to `Move to the start` with $direction.',
              );
              expect(
                ithSemanticsActions.containsKey(moveUp),
                true,
                reason: 'List item $i should be able to `Move up` with $direction.',
              );
              expect(
                ithSemanticsActions.containsKey(moveDown),
                true,
                reason: 'List item $i should be able to `Move down` with $direction.',
              );
              expect(
                ithSemanticsActions.containsKey(moveToEnd),
                true,
                reason: 'List item $i should be able to `Move to the end` with $direction.',
              );
            }

            // The last item can be moved up or to the start.
            final Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions =
                getSemanticsActions(listItems.length - 1);
            expect(
              lastSemanticsActions.length,
              2,
              reason: 'The last list item should have 2 custom actions with $direction.',
            );
            expect(
              lastSemanticsActions.containsKey(moveToStart),
              true,
              reason: 'The last item should be able to `Move to the start` with $direction.',
            );
            expect(
              lastSemanticsActions.containsKey(moveUp),
              true,
              reason: 'The last item should be able to `Move up` with $direction.',
            );
            expect(
              lastSemanticsActions.containsKey(moveDown),
              false,
              reason: 'The last item cannot `Move down` with $direction.',
            );
            expect(
              lastSemanticsActions.containsKey(moveToEnd),
              false,
              reason: 'The last item cannot `Move to the end` with $direction.',
            );
          }
          handle.dispose();
        });

        testWidgets('First item accessibility (a11y) actions work', (WidgetTester tester) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 1 to the end of the list.
          await tester.pumpWidget(build());
          Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));

          // Test out move after: move Item 2 (the current first item) one space down.
          await tester.pumpWidget(build());
          firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveDown]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 3', 'Item 2', 'Item 4', 'Item 1']));

          handle.dispose();
        });

        testWidgets('Middle item accessibility (a11y) actions work', (WidgetTester tester) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 2 to the end of the list.
          await tester.pumpWidget(build());
          Map<CustomSemanticsAction, VoidCallback> middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move after: move Item 3 (the current second item) one space down.
          await tester.pumpWidget(build());
          middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveDown]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 4', 'Item 3', 'Item 2']));

          // Test out move after: move Item 3 (the current third item) one space up.
          await tester.pumpWidget(build());
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveUp]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move to start: move Item 4 (the current third item) to the start of the list.
          await tester.pumpWidget(build());
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });

        testWidgets('Last item accessibility (a11y) actions work', (WidgetTester tester) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to start: move Item 4 to the start of the list.
          await tester.pumpWidget(build());
          Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions = getSemanticsActions(
            listItems.length - 1,
          );
          lastSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 2', 'Item 3']));

          // Test out move up: move Item 3 (the current last item) one space up.
          await tester.pumpWidget(build());
          lastSemanticsActions = getSemanticsActions(listItems.length - 1);
          lastSemanticsActions[moveUp]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });

        testWidgets("Doesn't hide accessibility when a child declares its own semantics", (
          WidgetTester tester,
        ) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          final Widget reorderableListView = ReorderableListView(
            onReorder: (int oldIndex, int newIndex) {},
            children: <Widget>[
              const SizedBox(
                key: Key('List tile 1'),
                height: itemHeight,
                child: Text('List tile 1'),
              ),
              SizedBox(
                key: const Key('Switch tile'),
                height: itemHeight,
                child: Material(
                  child: SwitchListTile(
                    title: const Text('Switch tile'),
                    value: true,
                    onChanged: (bool? newValue) {},
                    internalAddSemanticForOnTap: true,
                  ),
                ),
              ),
              const SizedBox(
                key: Key('List tile 2'),
                height: itemHeight,
                child: Text('List tile 2'),
              ),
            ],
          );
          await tester.pumpWidget(
            MaterialApp(home: SizedBox(height: itemHeight * 10, child: reorderableListView)),
          );

          // Get the switch tile's semantics:
          final SemanticsNode semanticsNode = tester.getSemantics(
            find.byKey(const Key('Switch tile')),
          );

          // Check for ReorderableListView custom semantics actions.
          expect(
            semanticsNode,
            matchesSemantics(
              customActions: const <CustomSemanticsAction>[
                CustomSemanticsAction(label: 'Move up'),
                CustomSemanticsAction(label: 'Move down'),
                CustomSemanticsAction(label: 'Move to the end'),
                CustomSemanticsAction(label: 'Move to the start'),
              ],
            ),
          );

          // Check for properties of SwitchTile semantics.
          late SemanticsNode child;
          semanticsNode.visitChildren((SemanticsNode node) {
            child = node;
            return false;
          });
          expect(
            child,
            matchesSemantics(
              isButton: true,
              hasToggledState: true,
              isToggled: true,
              isEnabled: true,
              isFocusable: true,
              hasEnabledState: true,
              hasSelectedState: true,
              label: 'Switch tile',
              hasTapAction: true,
              hasFocusAction: true,
            ),
          );
          handle.dispose();
        });
      });
    });

    group('in horizontal mode', () {
      testWidgets('reorder is not triggered when children length is less or equals to 1', (
        WidgetTester tester,
      ) async {
        bool onReorderWasCalled = false;
        final List<String> currentListItems = listItems.take(1).toList();
        final ReorderableListView reorderableListView = ReorderableListView(
          header: const Text('Header'),
          scrollDirection: Axis.horizontal,
          onReorder: (_, _) => onReorderWasCalled = true,
          children: currentListItems.map<Widget>(listItemToWidget).toList(),
        );
        final List<String> currentOriginalListItems = originalListItems.take(1).toList();
        await tester.pumpWidget(
          MaterialApp(home: SizedBox(height: itemHeight * 10, child: reorderableListView)),
        );
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 1')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        await drag.moveTo(tester.getBottomLeft(find.text('Item 1')) * 2);
        expect(currentListItems, orderedEquals(currentOriginalListItems));
        await drag.up();
        expect(onReorderWasCalled, false);
        expect(currentListItems, orderedEquals(<String>['Item 1']));
      });

      testWidgets('allows reordering from the very top to the very bottom', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(itemHeight * 2, 0.0),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
      });

      testWidgets('allows reordering from the very bottom to the very top', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 4')),
          tester.getCenter(find.text('Item 1')),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 2', 'Item 3']));
      });

      testWidgets('allows reordering inside the middle of the widget', (WidgetTester tester) async {
        await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 3')),
          tester.getCenter(find.text('Item 2')),
        );
        await tester.pumpAndSettle();
        expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 2', 'Item 4']));
      });

      testWidgets('properly reorders with a header', (WidgetTester tester) async {
        await tester.pumpWidget(
          build(header: const Text('Header Text'), scrollDirection: Axis.horizontal),
        );
        expect(find.text('Header Text'), findsOneWidget);
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(itemHeight * 2, 0.0),
        );
        await tester.pumpAndSettle();
        expect(find.text('Header Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
        await tester.pumpWidget(
          build(header: const Text('Header Text'), scrollDirection: Axis.horizontal),
        );
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 4')),
          tester.getCenter(find.text('Item 3')),
        );
        await tester.pumpAndSettle();
        expect(find.text('Header Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 4', 'Item 3', 'Item 1']));
      });

      testWidgets('properly reorders with a footer', (WidgetTester tester) async {
        await tester.pumpWidget(
          build(footer: const Text('Footer Text'), scrollDirection: Axis.horizontal),
        );
        expect(find.text('Footer Text'), findsOneWidget);
        expect(listItems, orderedEquals(originalListItems));
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 1')),
          tester.getCenter(find.text('Item 4')) + const Offset(itemHeight * 2, 0.0),
        );
        await tester.pumpAndSettle();
        expect(find.text('Footer Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));
        await tester.pumpWidget(
          build(footer: const Text('Footer Text'), scrollDirection: Axis.horizontal),
        );
        await longPressDrag(
          tester,
          tester.getCenter(find.text('Item 4')),
          tester.getCenter(find.text('Item 3')),
        );
        await tester.pumpAndSettle();
        expect(find.text('Footer Text'), findsOneWidget);
        expect(listItems, orderedEquals(<String>['Item 2', 'Item 4', 'Item 3', 'Item 1']));
      });

      testWidgets('properly determines the horizontal drop area extents', (
        WidgetTester tester,
      ) async {
        final Widget reorderableListView = ReorderableListView(
          scrollDirection: Axis.horizontal,
          onReorder: (int oldIndex, int newIndex) {},
          children: const <Widget>[
            SizedBox(key: Key('Normal item'), width: itemHeight, child: Text('Normal item')),
            SizedBox(key: Key('Tall item'), width: itemHeight * 2, child: Text('Tall item')),
            SizedBox(key: Key('Last item'), width: itemHeight, child: Text('Last item')),
          ],
        );
        await tester.pumpWidget(
          MaterialApp(home: SizedBox(width: itemHeight * 10, child: reorderableListView)),
        );

        double getListWidth() {
          final RenderSliverList listScrollView = tester.renderObject(find.byType(SliverList));
          return listScrollView.geometry!.maxPaintExtent;
        }

        const double kDraggingListWidth = 4 * itemHeight;
        // Drag a normal text item
        expect(getListWidth(), kDraggingListWidth);
        TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Normal item')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);

        // Move it
        await drag.moveTo(tester.getCenter(find.text('Last item')));
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);

        // Drop it
        await drag.up();
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);

        // Drag a tall item
        drag = await tester.startGesture(tester.getCenter(find.text('Tall item')));
        await tester.pump(kLongPressTimeout + kPressTimeout);
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);
        // Move it
        await drag.moveTo(tester.getCenter(find.text('Last item')));
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);

        // Drop it
        await drag.up();
        await tester.pumpAndSettle();
        expect(getListWidth(), kDraggingListWidth);
      });

      testWidgets('Horizontal drag in progress golden image', (WidgetTester tester) async {
        debugDisableShadows = false;
        final Widget reorderableListView = ReorderableListView(
          scrollDirection: Axis.horizontal,
          onReorder: (int oldIndex, int newIndex) {},
          children: <Widget>[
            Container(
              key: const Key('pink'),
              height: double.infinity,
              width: itemHeight,
              color: Colors.pink,
            ),
            Container(
              key: const Key('blue'),
              height: double.infinity,
              width: itemHeight,
              color: Colors.blue,
            ),
            Container(
              key: const Key('green'),
              height: double.infinity,
              width: itemHeight,
              color: Colors.green,
            ),
          ],
        );

        late final OverlayEntry entry;
        addTearDown(
          () =>
              entry
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          MaterialApp(
            home: Container(
              color: Colors.white,
              width: itemHeight * 3,
              // Wrap in an overlay so that the golden image includes the dragged item.
              child: Overlay(
                initialEntries: <OverlayEntry>[
                  entry = OverlayEntry(
                    builder: (BuildContext context) {
                      // Wrap the list in padding to test that the positioning
                      // is correct when the origin of the overlay is different
                      // from the list.
                      return Padding(padding: const EdgeInsets.all(24), child: reorderableListView);
                    },
                  ),
                ],
              ),
            ),
          ),
        );

        // Start dragging the second item.
        final TestGesture drag = await tester.startGesture(
          tester.getCenter(find.byKey(const Key('blue'))),
        );
        await tester.pump(kLongPressTimeout + kPressTimeout);

        // Drag it left to be partially over the first item.
        await drag.moveBy(const Offset(-itemHeight / 3, 0));
        await tester.pumpAndSettle();

        // Should be an image of the second item overlapping the right of the
        // first with a gap between the first and third and a drop shadow on
        // the dragged item.
        await expectLater(
          find.byType(Overlay).last,
          matchesGoldenFile('reorderable_list_test.horizontal.drop_area.png'),
        );
        debugDisableShadows = true;
      });

      testWidgets('Preserves children states when the list parent changes the order', (
        WidgetTester tester,
      ) async {
        _StatefulState findState(Key key) {
          return find
              .byElementPredicate(
                (Element element) => element.findAncestorWidgetOfExactType<_Stateful>()?.key == key,
              )
              .evaluate()
              .first
              .findAncestorStateOfType<_StatefulState>()!;
        }

        await tester.pumpWidget(
          MaterialApp(
            home: ReorderableListView(
              onReorder: (int oldIndex, int newIndex) {},
              scrollDirection: Axis.horizontal,
              children: <Widget>[
                _Stateful(key: const Key('A')),
                _Stateful(key: const Key('B')),
                _Stateful(key: const Key('C')),
              ],
            ),
          ),
        );
        await tester.tap(find.byKey(const Key('A')));
        await tester.pumpAndSettle();
        // Only the 'A' widget should be checked.
        expect(findState(const Key('A')).checked, true);
        expect(findState(const Key('B')).checked, false);
        expect(findState(const Key('C')).checked, false);

        await tester.pumpWidget(
          MaterialApp(
            home: ReorderableListView(
              onReorder: (int oldIndex, int newIndex) {},
              scrollDirection: Axis.horizontal,
              children: <Widget>[
                _Stateful(key: const Key('B')),
                _Stateful(key: const Key('C')),
                _Stateful(key: const Key('A')),
              ],
            ),
          ),
        );
        // Only the 'A' widget should be checked.
        expect(findState(const Key('B')).checked, false);
        expect(findState(const Key('C')).checked, false);
        expect(findState(const Key('A')).checked, true);
      });

      testWidgets('Preserves children states when rebuilt', (WidgetTester tester) async {
        const Key firstBox = Key('key');
        Widget build() {
          return MaterialApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: SizedBox(
                width: 100,
                height: 100,
                child: ReorderableListView(
                  scrollDirection: Axis.horizontal,
                  children: const <Widget>[SizedBox(key: firstBox, width: 10, height: 10)],
                  onReorder: (_, _) {},
                ),
              ),
            ),
          );
        }

        // When the widget is rebuilt, the state of child should be consistent.
        await tester.pumpWidget(build());
        final Element e0 = tester.element(find.byKey(firstBox));
        await tester.pumpWidget(build());
        final Element e1 = tester.element(find.byKey(firstBox));
        expect(e0, equals(e1));
      });

      group('Accessibility (a11y/Semantics)', () {
        Map<CustomSemanticsAction, VoidCallback> getSemanticsActions(int index) {
          final Semantics semantics =
              find
                      .ancestor(
                        of: find.byKey(Key(listItems[index])),
                        matching: find.byType(Semantics),
                      )
                      .evaluate()
                      .first
                      .widget
                  as Semantics;
          return semantics.properties.customSemanticsActions!;
        }

        const CustomSemanticsAction moveToStart = CustomSemanticsAction(label: 'Move to the start');
        const CustomSemanticsAction moveToEnd = CustomSemanticsAction(label: 'Move to the end');
        const CustomSemanticsAction moveLeft = CustomSemanticsAction(label: 'Move left');
        const CustomSemanticsAction moveRight = CustomSemanticsAction(label: 'Move right');

        testWidgets('Provides the correct accessibility actions in LTR mode', (
          WidgetTester tester,
        ) async {
          final SemanticsHandle handle = tester.ensureSemantics();

          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));

          // The first item can be moved right or to the end.
          final Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions =
              getSemanticsActions(0);
          expect(
            firstSemanticsActions.length,
            2,
            reason: 'The first list item should have 2 custom actions.',
          );
          expect(
            firstSemanticsActions.containsKey(moveToStart),
            false,
            reason: 'The first item cannot `Move to the start`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveLeft),
            false,
            reason: 'The first item cannot `Move left`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveRight),
            true,
            reason: 'The first item should be able to `Move right`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveToEnd),
            true,
            reason: 'The first item should be able to `Move to the end`.',
          );

          // Items in the middle can be moved to the start, end, left or right.
          for (int i = 1; i < listItems.length - 1; i += 1) {
            final Map<CustomSemanticsAction, VoidCallback> ithSemanticsActions =
                getSemanticsActions(i);
            expect(
              ithSemanticsActions.length,
              4,
              reason: 'List item $i should have 4 custom actions.',
            );
            expect(
              ithSemanticsActions.containsKey(moveToStart),
              true,
              reason: 'List item $i should be able to `Move to the start`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveLeft),
              true,
              reason: 'List item $i should be able to `Move left`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveRight),
              true,
              reason: 'List item $i should be able to `Move right`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveToEnd),
              true,
              reason: 'List item $i should be able to `Move to the end`.',
            );
          }

          // The last item can be moved left or to the start.
          final Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions = getSemanticsActions(
            listItems.length - 1,
          );
          expect(
            lastSemanticsActions.length,
            2,
            reason: 'The last list item should have 2 custom actions.',
          );
          expect(
            lastSemanticsActions.containsKey(moveToStart),
            true,
            reason: 'The last item should be able to `Move to the start`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveLeft),
            true,
            reason: 'The last item should be able to `Move left`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveRight),
            false,
            reason: 'The last item cannot `Move right`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveToEnd),
            false,
            reason: 'The last item cannot `Move to the end`.',
          );
          handle.dispose();
        });

        testWidgets('Provides the correct accessibility actions in Right-To-Left directionality', (
          WidgetTester tester,
        ) async {
          // In RTL mode, the right is the start and the left is the end.
          // The array representation is unchanged (LTR), but the direction of the motion actions is reversed.
          final SemanticsHandle handle = tester.ensureSemantics();

          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );

          // The first item can be moved right or to the end.
          final Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions =
              getSemanticsActions(0);
          expect(
            firstSemanticsActions.length,
            2,
            reason: 'The first list item should have 2 custom actions.',
          );
          expect(
            firstSemanticsActions.containsKey(moveToStart),
            false,
            reason: 'The first item cannot `Move to the start`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveRight),
            false,
            reason: 'The first item cannot `Move right`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveLeft),
            true,
            reason: 'The first item should be able to `Move left`.',
          );
          expect(
            firstSemanticsActions.containsKey(moveToEnd),
            true,
            reason: 'The first item should be able to `Move to the end`.',
          );

          // Items in the middle can be moved to the start, end, left or right.
          for (int i = 1; i < listItems.length - 1; i += 1) {
            final Map<CustomSemanticsAction, VoidCallback> ithSemanticsActions =
                getSemanticsActions(i);
            expect(
              ithSemanticsActions.length,
              4,
              reason: 'List item $i should have 4 custom actions.',
            );
            expect(
              ithSemanticsActions.containsKey(moveToStart),
              true,
              reason: 'List item $i should be able to `Move to the start`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveRight),
              true,
              reason: 'List item $i should be able to `Move right`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveLeft),
              true,
              reason: 'List item $i should be able to `Move left`.',
            );
            expect(
              ithSemanticsActions.containsKey(moveToEnd),
              true,
              reason: 'List item $i should be able to `Move to the end`.',
            );
          }

          // The last item can be moved left or to the start.
          final Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions = getSemanticsActions(
            listItems.length - 1,
          );
          expect(
            lastSemanticsActions.length,
            2,
            reason: 'The last list item should have 2 custom actions.',
          );
          expect(
            lastSemanticsActions.containsKey(moveToStart),
            true,
            reason: 'The last item should be able to `Move to the start`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveRight),
            true,
            reason: 'The last item should be able to `Move right`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveLeft),
            false,
            reason: 'The last item cannot `Move left`.',
          );
          expect(
            lastSemanticsActions.containsKey(moveToEnd),
            false,
            reason: 'The last item cannot `Move to the end`.',
          );
          handle.dispose();
        });

        testWidgets('First item accessibility (a11y) actions work in LTR mode', (
          WidgetTester tester,
        ) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 1 to the end of the list.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));

          // Test out move after: move Item 2 (the current first item) one space to the right.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveRight]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 3', 'Item 2', 'Item 4', 'Item 1']));

          handle.dispose();
        });

        testWidgets('First item accessibility (a11y) actions work in Right-To-Left directionality', (
          WidgetTester tester,
        ) async {
          // In RTL mode, the right is the start and the left is the end.
          // The array representation is unchanged (LTR), but the direction of the motion actions is reversed.
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 1 to the end of the list.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          Map<CustomSemanticsAction, VoidCallback> firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 2', 'Item 3', 'Item 4', 'Item 1']));

          // Test out move after: move Item 2 (the current first item) one space to the left.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          firstSemanticsActions = getSemanticsActions(0);
          firstSemanticsActions[moveLeft]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 3', 'Item 2', 'Item 4', 'Item 1']));

          handle.dispose();
        });

        testWidgets('Middle item accessibility (a11y) actions work in LTR mode', (
          WidgetTester tester,
        ) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 2 to the end of the list.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          Map<CustomSemanticsAction, VoidCallback> middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move after: move Item 3 (the current second item) one space to the right.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveRight]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 4', 'Item 3', 'Item 2']));

          // Test out move after: move Item 3 (the current third item) one space to the left.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveLeft]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move to start: move Item 4 (the current third item) to the start of the list.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });

        testWidgets('Middle item accessibility (a11y) actions work in Right-To-Left directionality', (
          WidgetTester tester,
        ) async {
          // In RTL mode, the right is the start and the left is the end.
          // The array representation is unchanged (LTR), but the direction of the motion actions is reversed.
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to end: move Item 2 to the end of the list.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          Map<CustomSemanticsAction, VoidCallback> middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveToEnd]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move after: move Item 3 (the current second item) one space to the left.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          middleSemanticsActions = getSemanticsActions(1);
          middleSemanticsActions[moveLeft]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 4', 'Item 3', 'Item 2']));

          // Test out move after: move Item 3 (the current third item) one space to the right.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveRight]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 1', 'Item 3', 'Item 4', 'Item 2']));

          // Test out move to start: move Item 4 (the current third item) to the start of the list.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          middleSemanticsActions = getSemanticsActions(2);
          middleSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });

        testWidgets('Last item accessibility (a11y) actions work in LTR mode', (
          WidgetTester tester,
        ) async {
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to start: move Item 4 to the start of the list.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions = getSemanticsActions(
            listItems.length - 1,
          );
          lastSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 2', 'Item 3']));

          // Test out move before: move Item 3 (the current last item) one space to the left.
          await tester.pumpWidget(build(scrollDirection: Axis.horizontal));
          lastSemanticsActions = getSemanticsActions(listItems.length - 1);
          lastSemanticsActions[moveLeft]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });

        testWidgets('Last item accessibility (a11y) actions work in Right-To-Left directionality', (
          WidgetTester tester,
        ) async {
          // In RTL mode, the right is the start and the left is the end.
          // The array representation is unchanged (LTR), but the direction of the motion actions is reversed.
          final SemanticsHandle handle = tester.ensureSemantics();
          expect(listItems, orderedEquals(originalListItems));

          // Test out move to start: move Item 4 to the start of the list.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          Map<CustomSemanticsAction, VoidCallback> lastSemanticsActions = getSemanticsActions(
            listItems.length - 1,
          );
          lastSemanticsActions[moveToStart]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 2', 'Item 3']));

          // Test out move before: move Item 3 (the current last item) one space to the right.
          await tester.pumpWidget(
            build(scrollDirection: Axis.horizontal, textDirection: TextDirection.rtl),
          );
          lastSemanticsActions = getSemanticsActions(listItems.length - 1);
          lastSemanticsActions[moveRight]!();
          await tester.pumpAndSettle();
          expect(listItems, orderedEquals(<String>['Item 4', 'Item 1', 'Item 3', 'Item 2']));

          handle.dispose();
        });
      });
    });

    testWidgets('ReorderableListView.builder asserts on negative childCount', (
      WidgetTester tester,
    ) async {
      expect(
        () => ReorderableListView.builder(
          itemBuilder: (BuildContext context, int index) {
            return const SizedBox();
          },
          itemCount: -1,
          onReorder: (int from, int to) {},
        ),
        throwsAssertionError,
      );
    });

    testWidgets('ReorderableListView.builder only creates the children it needs', (
      WidgetTester tester,
    ) async {
      final Set<int> itemsCreated = <int>{};
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableListView.builder(
            itemBuilder: (BuildContext context, int index) {
              itemsCreated.add(index);
              return Text(index.toString(), key: ValueKey<int>(index));
            },
            itemCount: 1000,
            onReorder: (int from, int to) {},
          ),
        ),
      );

      // Should have only created the first 18 items.
      expect(itemsCreated, <int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17});
    });

    group('Padding', () {
      testWidgets('Padding with no header & footer', (WidgetTester tester) async {
        const EdgeInsets padding = EdgeInsets.fromLTRB(10, 20, 30, 40);

        // Vertical
        await tester.pumpWidget(build(padding: padding));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 20, 770, 68),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(10, 164, 770, 212),
        );

        // Horizontal
        await tester.pumpWidget(build(padding: padding, scrollDirection: Axis.horizontal));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 20, 58, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(154, 20, 202, 560),
        );
      });

      testWidgets('Padding with header or footer', (WidgetTester tester) async {
        const EdgeInsets padding = EdgeInsets.fromLTRB(10, 20, 30, 40);
        const Key headerKey = Key('Header');
        const Key footerKey = Key('Footer');
        const Widget verticalHeader = SizedBox(key: headerKey, height: 10);
        const Widget horizontalHeader = SizedBox(key: headerKey, width: 10);
        const Widget verticalFooter = SizedBox(key: footerKey, height: 10);
        const Widget horizontalFooter = SizedBox(key: footerKey, width: 10);

        // Vertical Header
        await tester.pumpWidget(build(padding: padding, header: verticalHeader));
        expect(tester.getRect(find.byKey(headerKey)), const Rect.fromLTRB(10, 20, 770, 30));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 30, 770, 78),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(10, 174, 770, 222),
        );

        // Vertical Footer
        await tester.pumpWidget(build(padding: padding, footer: verticalFooter));
        expect(tester.getRect(find.byKey(footerKey)), const Rect.fromLTRB(10, 212, 770, 222));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 20, 770, 68),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(10, 164, 770, 212),
        );

        // Vertical Header, reversed
        await tester.pumpWidget(build(padding: padding, header: verticalHeader, reverse: true));
        expect(tester.getRect(find.byKey(headerKey)), const Rect.fromLTRB(10, 550, 770, 560));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 502, 770, 550),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(10, 358, 770, 406),
        );

        // Vertical Footer, reversed
        await tester.pumpWidget(build(padding: padding, footer: verticalFooter, reverse: true));
        expect(tester.getRect(find.byKey(footerKey)), const Rect.fromLTRB(10, 358, 770, 368));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 512, 770, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(10, 368, 770, 416),
        );

        // Horizontal Header
        await tester.pumpWidget(
          build(padding: padding, header: horizontalHeader, scrollDirection: Axis.horizontal),
        );
        expect(tester.getRect(find.byKey(headerKey)), const Rect.fromLTRB(10, 20, 20, 560));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(20, 20, 68, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(164, 20, 212, 560),
        );

        // // Horizontal Footer
        await tester.pumpWidget(
          build(padding: padding, footer: horizontalFooter, scrollDirection: Axis.horizontal),
        );
        expect(tester.getRect(find.byKey(footerKey)), const Rect.fromLTRB(202, 20, 212, 560));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(10, 20, 58, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(154, 20, 202, 560),
        );

        // Horizontal Header, reversed
        await tester.pumpWidget(
          build(
            padding: padding,
            header: horizontalHeader,
            scrollDirection: Axis.horizontal,
            reverse: true,
          ),
        );
        expect(tester.getRect(find.byKey(headerKey)), const Rect.fromLTRB(760, 20, 770, 560));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(712, 20, 760, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(568, 20, 616, 560),
        );

        // // Horizontal Footer, reversed
        await tester.pumpWidget(
          build(
            padding: padding,
            footer: horizontalFooter,
            scrollDirection: Axis.horizontal,
            reverse: true,
          ),
        );
        expect(tester.getRect(find.byKey(footerKey)), const Rect.fromLTRB(568, 20, 578, 560));
        expect(
          tester.getRect(find.byKey(const Key('Item 1'))),
          const Rect.fromLTRB(722, 20, 770, 560),
        );
        expect(
          tester.getRect(find.byKey(const Key('Item 4'))),
          const Rect.fromLTRB(578, 20, 626, 560),
        );
      });
    });

    testWidgets('ReorderableListView can be reversed', (WidgetTester tester) async {
      final Widget reorderableListView = ReorderableListView(
        reverse: true,
        onReorder: (int oldIndex, int newIndex) {},
        children: const <Widget>[
          SizedBox(key: Key('A'), child: Text('A')),
          SizedBox(key: Key('B'), child: Text('B')),
          SizedBox(key: Key('C'), child: Text('C')),
        ],
      );
      await tester.pumpWidget(MaterialApp(home: reorderableListView));
      expect(tester.getCenter(find.text('A')).dy, greaterThan(tester.getCenter(find.text('B')).dy));
    });

    testWidgets('Animation test when placing an item in place', (WidgetTester tester) async {
      const Key testItemKey = Key('Test item');
      final Widget reorderableListView = ReorderableListView(
        onReorder: (int oldIndex, int newIndex) {},
        children: const <Widget>[
          SizedBox(key: Key('First item'), height: itemHeight, child: Text('First item')),
          SizedBox(key: testItemKey, height: itemHeight, child: Text('Test item')),
          SizedBox(key: Key('Last item'), height: itemHeight, child: Text('Last item')),
        ],
      );
      await tester.pumpWidget(
        MaterialApp(home: SizedBox(height: itemHeight * 10, child: reorderableListView)),
      );

      Offset getTestItemPosition() {
        final RenderBox testItem = tester.renderObject<RenderBox>(find.byKey(testItemKey));
        return testItem.localToGlobal(Offset.zero);
      }

      // Before pick it up.
      final Offset startPosition = getTestItemPosition();

      // Pick it up.
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(testItemKey)),
      );
      await tester.pump(kLongPressTimeout + kPressTimeout);
      expect(getTestItemPosition(), startPosition);

      // Put it down.
      await gesture.up();
      await tester.pump();
      expect(getTestItemPosition(), startPosition);

      // After put it down.
      await tester.pumpAndSettle();
      expect(getTestItemPosition(), startPosition);
    });
    // TODO(djshuckerow): figure out how to write a test for scrolling the list.

    testWidgets(
      'ReorderableListView on desktop platforms should have drag handles',
      (WidgetTester tester) async {
        await tester.pumpWidget(build());
        // All four items should have drag handles and not delayed listeners.
        expect(find.byIcon(Icons.drag_handle), findsNWidgets(4));
        expect(find.byType(ReorderableDelayedDragStartListener), findsNothing);
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets(
      'ReorderableListView on mobile platforms should not have drag handles',
      (WidgetTester tester) async {
        await tester.pumpWidget(build());
        // All four items should have delayed listeners and not drag handles.
        expect(find.byType(ReorderableDelayedDragStartListener), findsNWidgets(4));
        expect(find.byIcon(Icons.drag_handle), findsNothing);
      },
      variant: TargetPlatformVariant.mobile(),
    );

    testWidgets('Vertical list renders drag handle in correct position', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(build(platform: TargetPlatform.macOS));
      final Finder listView = find.byType(ReorderableListView);
      final Finder item1 = find.byKey(const Key('Item 1'));
      final Finder dragHandle = find.byIcon(Icons.drag_handle).first;

      // Should be centered vertically within the item and 8 pixels from the right edge of the list.
      expect(tester.getCenter(dragHandle).dy, tester.getCenter(item1).dy);
      expect(tester.getTopRight(dragHandle).dx, tester.getSize(listView).width - 8);
    });

    testWidgets('Horizontal list renders drag handle in correct position', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        build(scrollDirection: Axis.horizontal, platform: TargetPlatform.macOS),
      );
      final Finder listView = find.byType(ReorderableListView);
      final Finder item1 = find.byKey(const Key('Item 1'));
      final Finder dragHandle = find.byIcon(Icons.drag_handle).first;

      // Should be centered horizontally within the item and 8 pixels from the bottom of the list.
      expect(tester.getCenter(dragHandle).dx, tester.getCenter(item1).dx);
      expect(tester.getBottomRight(dragHandle).dy, tester.getSize(listView).height - 8);
    });
  });

  testWidgets(
    'ReorderableListView, can deal with the dragged item getting unmounted and rebuilt during drag',
    (WidgetTester tester) async {
      // See https://github.com/flutter/flutter/issues/74840 for more details.
      final List<int> items = List<int>.generate(100, (int index) => index);

      void handleReorder(int fromIndex, int toIndex) {
        if (toIndex > fromIndex) {
          toIndex -= 1;
        }
        items.insert(toIndex, items.removeAt(fromIndex));
      }

      // The list is 800x600, 8 items, each item is 800x100 with
      // an "item $index" text widget at the item's origin.  Drags are initiated by
      // a simple press on the text widget.
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableListView.builder(
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                key: ValueKey<int>(items[index]),
                height: 100,
                child: ReorderableDragStartListener(
                  index: index,
                  child: Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[Text('item ${items[index]}')],
                  ),
                ),
              );
            },
            buildDefaultDragHandles: false,
            itemCount: items.length,
            onReorder: handleReorder,
          ),
        ),
      );

      // Drag item 0 downwards and force an auto scroll off the end of the list
      // far enough that item zeros original entry in the list is unmounted.
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
      await tester.pump(kPressTimeout);
      // Off the bottom of the screen, which should autoscroll until we hit the
      // end of the list
      await drag.moveBy(const Offset(0, 700));
      await tester.pump(const Duration(seconds: 30));
      await tester.pumpAndSettle();
      // Ensure we made it to the bottom (only 4 should be showing as there should
      // be a gap at the end for the drop area of the dragged item.
      for (final int i in <int>[95, 96, 97, 98, 99]) {
        expect(find.text('item $i'), findsOneWidget);
      }

      // Drag back to off the top of the list, which should autoscroll until
      // we hit the beginning of the list. This should cause the first item's
      // entry to be rebuilt. However, the contents should not be in both places.
      await drag.moveBy(const Offset(0, -1400));
      await tester.pump(const Duration(seconds: 30));
      await tester.pumpAndSettle();
      // Release back at the top so item 0 should drop where it was
      await drag.up();
      await tester.pumpAndSettle();

      // Should not have changed anything
      for (final int i in <int>[0, 1, 2, 3, 4, 5]) {
        expect(find.text('item $i'), findsOneWidget);
      }
      expect(items.take(8), orderedEquals(<int>[0, 1, 2, 3, 4, 5, 6, 7]));
    },
  );

  testWidgets('ReorderableListView calls onReorderStart and onReorderEnd correctly', (
    WidgetTester tester,
  ) async {
    final List<int> items = List<int>.generate(8, (int index) => index);
    int? startIndex, endIndex;
    final Finder item0 = find.textContaining('item 0');

    void handleReorder(int fromIndex, int toIndex) {
      if (toIndex > fromIndex) {
        toIndex -= 1;
      }
      items.insert(toIndex, items.removeAt(fromIndex));
    }

    await tester.pumpWidget(
      MaterialApp(
        home: ReorderableListView.builder(
          itemBuilder: (BuildContext context, int index) {
            return SizedBox(
              key: ValueKey<int>(items[index]),
              height: 100,
              child: ReorderableDragStartListener(
                index: index,
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[Text('item ${items[index]}')],
                ),
              ),
            );
          },
          buildDefaultDragHandles: false,
          itemCount: items.length,
          onReorder: handleReorder,
          onReorderStart: (int index) {
            startIndex = index;
          },
          onReorderEnd: (int index) {
            endIndex = index;
          },
        ),
      ),
    );

    TestGesture drag = await tester.startGesture(tester.getCenter(item0));
    await tester.pump(kPressTimeout);
    // Drag enough for move to start.
    await drag.moveBy(const Offset(0, 20));

    expect(startIndex, equals(0));
    expect(endIndex, isNull);

    // Move item0 from index 0 to index 3
    await drag.moveBy(const Offset(0, 300));
    await tester.pumpAndSettle();
    await drag.up();
    await tester.pumpAndSettle();

    expect(endIndex, equals(3));

    startIndex = null;
    endIndex = null;

    drag = await tester.startGesture(tester.getCenter(item0));
    await tester.pump(kPressTimeout);
    // Drag enough for move to start.
    await drag.moveBy(const Offset(0, 20));

    expect(startIndex, equals(2));
    expect(endIndex, isNull);

    // Move item0 from index 2 to index 0
    await drag.moveBy(const Offset(0, -200));
    await tester.pumpAndSettle();
    await drag.up();
    await tester.pumpAndSettle();

    expect(endIndex, equals(0));
  });

  testWidgets('ReorderableListView throws an error when key is not passed to its children', (
    WidgetTester tester,
  ) async {
    final Widget reorderableListView = ReorderableListView.builder(
      itemBuilder: (BuildContext context, int index) {
        return SizedBox(child: Text('Item $index'));
      },
      itemCount: 3,
      onReorder: (int oldIndex, int newIndex) {},
    );
    await tester.pumpWidget(MaterialApp(home: reorderableListView));
    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(exception.toString(), contains('Every item of ReorderableListView must have a key.'));
  });

  testWidgets('Throws an error if no overlay present', (WidgetTester tester) async {
    final Widget reorderableList = ReorderableListView(
      children: const <Widget>[
        SizedBox(width: 100.0, height: 100.0, key: Key('C'), child: Text('C')),
        SizedBox(width: 100.0, height: 100.0, key: Key('B'), child: Text('B')),
        SizedBox(width: 100.0, height: 100.0, key: Key('A'), child: Text('A')),
      ],
      onReorder: (int oldIndex, int newIndex) {},
    );
    final Widget boilerplate = Localizations(
      locale: const Locale('en'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultMaterialLocalizations.delegate,
        DefaultWidgetsLocalizations.delegate,
      ],
      child: SizedBox(
        width: 100.0,
        height: 100.0,
        child: Directionality(textDirection: TextDirection.ltr, child: reorderableList),
      ),
    );
    await tester.pumpWidget(boilerplate);

    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(exception.toString(), contains('No Overlay widget found'));
    expect(
      exception.toString(),
      contains('ReorderableListView widgets require an Overlay widget ancestor'),
    );
  });

  testWidgets('ReorderableListView asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    expect(
      () => ReorderableListView(
        itemExtent: 30,
        prototypeItem: const SizedBox(),
        onReorder: (int fromIndex, int toIndex) {},
        children: const <Widget>[],
      ),
      throwsAssertionError,
    );
  });

  testWidgets('ReorderableListView.builder asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];
    expect(
      () => ReorderableListView.builder(
        itemBuilder: (BuildContext context, int index) {
          return SizedBox(
            key: ValueKey<int>(numbers[index]),
            height: 20 + numbers[index] * 10,
            child: ReorderableDragStartListener(
              index: index,
              child: Text(numbers[index].toString()),
            ),
          );
        },
        itemCount: numbers.length,
        itemExtent: 30,
        prototypeItem: const SizedBox(),
        onReorder: (int fromIndex, int toIndex) {},
      ),
      throwsAssertionError,
    );
  });

  testWidgets('if itemExtent is non-null, children have same extent in the scroll direction', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ReorderableListView.builder(
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(numbers[index]),
                    // children with different heights
                    height: 20 + numbers[index] * 10,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text(numbers[index].toString()),
                    ),
                  );
                },
                itemCount: numbers.length,
                itemExtent: 30,
                onReorder: (int fromIndex, int toIndex) {},
              );
            },
          ),
        ),
      ),
    );

    final double item0Height = tester.getSize(find.text('0').hitTestable()).height;
    final double item1Height = tester.getSize(find.text('1').hitTestable()).height;
    final double item2Height = tester.getSize(find.text('2').hitTestable()).height;

    expect(item0Height, 30.0);
    expect(item1Height, 30.0);
    expect(item2Height, 30.0);
  });

  testWidgets('if prototypeItem is non-null, children have same extent in the scroll direction', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ReorderableListView.builder(
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(numbers[index]),
                    // children with different heights
                    height: 20 + numbers[index] * 10,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text(numbers[index].toString()),
                    ),
                  );
                },
                itemCount: numbers.length,
                prototypeItem: const SizedBox(height: 30, child: Text('3')),
                onReorder: (int oldIndex, int newIndex) {},
              );
            },
          ),
        ),
      ),
    );

    final double item0Height = tester.getSize(find.text('0').hitTestable()).height;
    final double item1Height = tester.getSize(find.text('1').hitTestable()).height;
    final double item2Height = tester.getSize(find.text('2').hitTestable()).height;

    expect(item0Height, 30.0);
    expect(item1Height, 30.0);
    expect(item2Height, 30.0);
  });

  testWidgets('ReorderableListView auto scrolls speed is configurable', (
    WidgetTester tester,
  ) async {
    Future<void> pumpFor({
      required Duration duration,
      Duration interval = const Duration(milliseconds: 50),
    }) async {
      await tester.pump();

      int times = (duration.inMilliseconds / interval.inMilliseconds).ceil();
      while (times > 0) {
        await tester.pump(interval + const Duration(milliseconds: 1));
        await tester.idle();
        times--;
      }
    }

    Future<double> pumpListAndDrag({required double autoScrollerVelocityScalar}) async {
      final List<int> items = List<int>.generate(10, (int index) => index);
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableListView.builder(
            itemBuilder: (BuildContext context, int index) {
              return Container(
                key: ValueKey<int>(items[index]),
                height: 100,
                color: items[index].isOdd ? Colors.red : Colors.green,
                child: ReorderableDragStartListener(
                  index: index,
                  child: Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[Text('item ${items[index]}'), const Icon(Icons.drag_handle)],
                  ),
                ),
              );
            },
            itemCount: items.length,
            onReorder: (int fromIndex, int toIndex) {},
            scrollController: scrollController,
            autoScrollerVelocityScalar: autoScrollerVelocityScalar,
          ),
        ),
      );

      expect(scrollController.offset, 0);

      final Finder item = find.text('item 0');
      final TestGesture drag = await tester.startGesture(tester.getCenter(item));

      // Drag just enough to touch the edge but not surpass it, so the
      // auto scroller is not yet triggered
      await drag.moveBy(const Offset(0, 500));
      await pumpFor(duration: const Duration(milliseconds: 200));

      expect(scrollController.offset, 0);

      // Now drag a little bit more so the auto scroller triggers
      await drag.moveBy(const Offset(0, 50));
      await pumpFor(
        duration: const Duration(milliseconds: 600),
        interval: Duration(milliseconds: (1000 / autoScrollerVelocityScalar).round()),
      );
      await drag.up();

      return scrollController.offset;
    }

    const double fastVelocityScalar = 20;
    final double offsetForFastScroller = await pumpListAndDrag(
      autoScrollerVelocityScalar: fastVelocityScalar,
    );

    // Reset widget tree
    await tester.pumpWidget(const SizedBox());

    const double slowVelocityScalar = 5;
    final double offsetForSlowScroller = await pumpListAndDrag(
      autoScrollerVelocityScalar: slowVelocityScalar,
    );

    expect(offsetForFastScroller / offsetForSlowScroller, fastVelocityScalar / slowVelocityScalar);
  });

  testWidgets('DragBoundary defines the boundary for ReorderableListView.', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Container(
            margin: const EdgeInsets.only(top: 100),
            height: 300,
            child: DragBoundary(
              child: ReorderableListView.builder(
                itemBuilder: (BuildContext context, int index) {
                  return ReorderableDragStartListener(
                    key: ValueKey<int>(index),
                    index: index,
                    child: Text('$index'),
                  );
                },
                itemCount: 5,
                onReorder: (int fromIndex, int toIndex) {},
              ),
            ),
          ),
        ),
      ),
    );
    TestGesture drag = await tester.startGesture(tester.getCenter(find.text('0')));
    await tester.pump(kLongPressTimeout);
    await drag.moveBy(const Offset(0, -400));
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.text('0')), const Offset(0, 100));
    await drag.up();
    await tester.pumpAndSettle();

    drag = await tester.startGesture(tester.getCenter(find.text('0')));
    await tester.pump(kLongPressTimeout);
    await drag.moveBy(const Offset(0, 800));
    await tester.pumpAndSettle();
    expect(tester.getBottomLeft(find.text('0')), const Offset(0, 400));
    await drag.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Mouse cursor behavior on the drag handle', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ReorderableListView.builder(
            itemBuilder: (BuildContext context, int index) {
              return ReorderableDragStartListener(
                key: ValueKey<int>(index),
                index: index,
                child: Text('$index'),
              );
            },
            itemCount: 5,
            onReorder: (int fromIndex, int toIndex) {},
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byIcon(Icons.drag_handle).first));
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );
    await gesture.down(tester.getCenter(find.byIcon(Icons.drag_handle).first));
    await tester.pump(kLongPressTimeout);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grabbing,
    );
    await gesture.up();
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );
  }, variant: TargetPlatformVariant.desktop());

  testWidgets(
    'Mouse cursor behavior on the drag handle can be provided',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ReorderableListView.builder(
              mouseCursor:
                  const WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
                    WidgetState.dragged: SystemMouseCursors.copy,
                    WidgetState.any: SystemMouseCursors.resizeColumn,
                  }),
              itemBuilder: (BuildContext context, int index) {
                return ReorderableDragStartListener(
                  key: ValueKey<int>(index),
                  index: index,
                  child: Text('$index'),
                );
              },
              itemCount: 5,
              onReorder: (int fromIndex, int toIndex) {},
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.mouse,
        pointer: 1,
      );
      await gesture.addPointer(location: tester.getCenter(find.byIcon(Icons.drag_handle).first));
      await tester.pump();
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.resizeColumn,
      );
      await gesture.down(tester.getCenter(find.byIcon(Icons.drag_handle).first));
      await tester.pump(kLongPressTimeout);
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.copy,
      );
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.resizeColumn,
      );
    },
    variant: TargetPlatformVariant.desktop(),
  );
}

Future<void> longPressDrag(WidgetTester tester, Offset start, Offset end) async {
  final TestGesture drag = await tester.startGesture(start);
  await tester.pump(kLongPressTimeout + kPressTimeout);
  await drag.moveTo(end);
  await tester.pump(kPressTimeout);
  await drag.up();
}

class _Stateful extends StatefulWidget {
  // Ignoring the preference for const constructors because we want to test with regular non-const instances.
  // ignore:prefer_const_constructors_in_immutables
  _Stateful({super.key});

  @override
  State<StatefulWidget> createState() => _StatefulState();
}

class _StatefulState extends State<_Stateful> {
  bool? checked = false;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 48.0,
      height: 48.0,
      child: Material(
        child: Checkbox(value: checked, onChanged: (bool? newValue) => checked = newValue),
      ),
    );
  }
}
