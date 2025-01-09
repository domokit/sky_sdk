// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('SliverReorderableList works well when having gestureSettings', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/103404
    const int itemCount = 5;
    int onReorderCallCount = 0;
    final List<int> items = List<int>.generate(itemCount, (int index) => index);

    void handleReorder(int fromIndex, int toIndex) {
      onReorderCallCount += 1;
      if (toIndex > fromIndex) {
        toIndex -= 1;
      }
      items.insert(toIndex, items.removeAt(fromIndex));
    }

    // The list has five elements of height 100
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(gestureSettings: DeviceGestureSettings(touchSlop: 8.0)),
          child: CustomScrollView(
            slivers: <Widget>[
              SliverReorderableList(
                itemCount: itemCount,
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(items[index]),
                    height: 100,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text('item ${items[index]}'),
                    ),
                  );
                },
                onReorder: handleReorder,
              ),
            ],
          ),
        ),
      ),
    );

    // Start gesture on first item
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
    await tester.pump(kPressTimeout);

    // Drag a little bit to make `ImmediateMultiDragGestureRecognizer` compete with `VerticalDragGestureRecognizer`
    await drag.moveBy(const Offset(0, 10));
    await tester.pump();
    // Drag enough to move down the first item
    await drag.moveBy(const Offset(0, 40));
    await tester.pump();
    await drag.up();
    await tester.pumpAndSettle();

    expect(onReorderCallCount, 1);
    expect(items, orderedEquals(<int>[1, 0, 2, 3, 4]));
  });

  testWidgets('SliverReorderableList item has correct semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const int itemCount = 5;
    int onReorderCallCount = 0;
    final List<int> items = List<int>.generate(itemCount, (int index) => index);

    void handleReorder(int fromIndex, int toIndex) {
      onReorderCallCount += 1;
      if (toIndex > fromIndex) {
        toIndex -= 1;
      }
      items.insert(toIndex, items.removeAt(fromIndex));
    }

    // The list has five elements of height 100
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(gestureSettings: DeviceGestureSettings(touchSlop: 8.0)),
          child: CustomScrollView(
            slivers: <Widget>[
              SliverReorderableList(
                itemCount: itemCount,
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(items[index]),
                    height: 100,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text('item ${items[index]}'),
                    ),
                  );
                },
                onReorder: handleReorder,
              ),
            ],
          ),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(label: 'item 0', actions: <SemanticsAction>[SemanticsAction.customAction]),
    );
    final SemanticsNode node = tester.getSemantics(find.text('item 0'));

    // perform custom action 'move down'.
    final int customActionId = CustomSemanticsAction.getIdentifier(
      const CustomSemanticsAction(label: 'Move down'),
    );
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      node.id,
      SemanticsAction.customAction,
      customActionId,
    );
    await tester.pumpAndSettle();

    expect(onReorderCallCount, 1);
    expect(items, orderedEquals(<int>[1, 0, 2, 3, 4]));

    semantics.dispose();
  });

  testWidgets('SliverReorderableList custom semantics action has correct label', (
    WidgetTester tester,
  ) async {
    const int itemCount = 5;
    final List<int> items = List<int>.generate(itemCount, (int index) => index);
    // The list has five elements of height 100
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(gestureSettings: DeviceGestureSettings(touchSlop: 8.0)),
          child: CustomScrollView(
            slivers: <Widget>[
              SliverReorderableList(
                itemCount: itemCount,
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(items[index]),
                    height: 100,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text('item ${items[index]}'),
                    ),
                  );
                },
                onReorder: (int _, int __) {},
              ),
            ],
          ),
        ),
      ),
    );
    final SemanticsNode node = tester.getSemantics(find.text('item 0'));
    final SemanticsData data = node.getSemanticsData();
    expect(data.customSemanticsActionIds!.length, 2);
    final CustomSemanticsAction action1 =
        CustomSemanticsAction.getAction(data.customSemanticsActionIds![0])!;
    expect(action1.label, 'Move down');
    final CustomSemanticsAction action2 =
        CustomSemanticsAction.getAction(data.customSemanticsActionIds![1])!;
    expect(action2.label, 'Move to the end');
  });

  // Regression test for https://github.com/flutter/flutter/issues/100451
  testWidgets('SliverReorderableList.builder respects findChildIndexCallback', (
    WidgetTester tester,
  ) async {
    bool finderCalled = false;
    int itemCount = 7;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return CustomScrollView(
              slivers: <Widget>[
                SliverReorderableList(
                  itemCount: itemCount,
                  itemBuilder:
                      (BuildContext _, int index) => Container(key: Key('$index'), height: 2000.0),
                  findChildIndexCallback: (Key key) {
                    finderCalled = true;
                    return null;
                  },
                  onReorder: (int oldIndex, int newIndex) {},
                ),
              ],
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

  // Regression test for https://github.com/flutter/flutter/issues/88191
  testWidgets('Do not crash when dragging with two fingers simultaneously', (
    WidgetTester tester,
  ) async {
    final List<int> items = List<int>.generate(3, (int index) => index);
    void handleReorder(int fromIndex, int toIndex) {
      if (toIndex > fromIndex) {
        toIndex -= 1;
      }
      items.insert(toIndex, items.removeAt(fromIndex));
    }

    await tester.pumpWidget(
      MaterialApp(
        home: ReorderableList(
          itemBuilder: (BuildContext context, int index) {
            return ReorderableDragStartListener(
              index: index,
              key: ValueKey<int>(items[index]),
              child: SizedBox(
                height: 100,
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[Text('item ${items[index]}')],
                ),
              ),
            );
          },
          itemCount: items.length,
          onReorder: handleReorder,
        ),
      ),
    );

    final TestGesture drag1 = await tester.startGesture(tester.getCenter(find.text('item 0')));
    final TestGesture drag2 = await tester.startGesture(tester.getCenter(find.text('item 0')));
    await tester.pump(kLongPressTimeout);

    await drag1.moveBy(const Offset(0, 100));
    await drag2.moveBy(const Offset(0, 100));
    await tester.pumpAndSettle();

    await drag1.up();
    await drag2.up();
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
  });

  testWidgets('negative itemCount should assert', (WidgetTester tester) async {
    final List<int> items = <int>[1, 2, 3];
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext outerContext, StateSetter setState) {
            return CustomScrollView(
              slivers: <Widget>[
                SliverReorderableList(
                  itemCount: -1,
                  onReorder: (int fromIndex, int toIndex) {
                    setState(() {
                      if (toIndex > fromIndex) {
                        toIndex -= 1;
                      }
                      items.insert(toIndex, items.removeAt(fromIndex));
                    });
                  },
                  itemBuilder: (BuildContext context, int index) {
                    return SizedBox(height: 100, child: Text('item ${items[index]}'));
                  },
                ),
              ],
            );
          },
        ),
      ),
    );
    expect(tester.takeException(), isA<AssertionError>());
  });

  testWidgets('zero itemCount should not build widget', (WidgetTester tester) async {
    final List<int> items = <int>[1, 2, 3];
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext outerContext, StateSetter setState) {
            return CustomScrollView(
              slivers: <Widget>[
                SliverFixedExtentList(
                  itemExtent: 50.0,
                  delegate: SliverChildListDelegate(<Widget>[const Text('before')]),
                ),
                SliverReorderableList(
                  itemCount: 0,
                  onReorder: (int fromIndex, int toIndex) {
                    setState(() {
                      if (toIndex > fromIndex) {
                        toIndex -= 1;
                      }
                      items.insert(toIndex, items.removeAt(fromIndex));
                    });
                  },
                  itemBuilder: (BuildContext context, int index) {
                    return SizedBox(height: 100, child: Text('item ${items[index]}'));
                  },
                ),
                SliverFixedExtentList(
                  itemExtent: 50.0,
                  delegate: SliverChildListDelegate(<Widget>[const Text('after')]),
                ),
              ],
            );
          },
        ),
      ),
    );

    expect(find.text('before'), findsOneWidget);
    expect(find.byType(SliverReorderableList), findsNothing);
    expect(find.text('after'), findsOneWidget);
  });

  testWidgets('SliverReorderableList, drag and drop, fixed height items', (
    WidgetTester tester,
  ) async {
    final List<int> items = List<int>.generate(8, (int index) => index);

    Future<void> pressDragRelease(Offset start, Offset delta) async {
      final TestGesture drag = await tester.startGesture(start);
      await tester.pump(kPressTimeout);
      await drag.moveBy(delta);
      await tester.pump(kPressTimeout);
      await drag.up();
      await tester.pumpAndSettle();
    }

    void check({List<int> visible = const <int>[], List<int> hidden = const <int>[]}) {
      for (final int i in visible) {
        expect(find.text('item $i'), findsOneWidget);
      }
      for (final int i in hidden) {
        expect(find.text('item $i'), findsNothing);
      }
    }

    // The SliverReorderableList is 800x600, 8 items, each item is 800x100 with
    // an "item $index" text widget at the item's origin.  Drags are initiated by
    // a simple press on the text widget.
    await tester.pumpWidget(TestList(items: items));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);

    // Drag item 0 downwards less than halfway and let it snap back. List
    // should remain as it is.
    await pressDragRelease(const Offset(12, 50), const Offset(12, 60));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 100));
    expect(items, orderedEquals(<int>[0, 1, 2, 3, 4, 5, 6, 7]));

    // Drag item 0 downwards more than halfway to displace item 1.
    await pressDragRelease(tester.getCenter(find.text('item 0')), const Offset(0, 51));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);
    expect(tester.getTopLeft(find.text('item 1')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 100));
    expect(items, orderedEquals(<int>[1, 0, 2, 3, 4, 5, 6, 7]));

    // Drag item 0 back to where it was.
    await pressDragRelease(tester.getCenter(find.text('item 0')), const Offset(0, -51));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 100));
    expect(items, orderedEquals(<int>[0, 1, 2, 3, 4, 5, 6, 7]));

    // Drag item 1 to item 3
    await pressDragRelease(tester.getCenter(find.text('item 1')), const Offset(0, 151));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 300));
    expect(tester.getTopLeft(find.text('item 3')), const Offset(0, 200));
    expect(items, orderedEquals(<int>[0, 2, 3, 1, 4, 5, 6, 7]));

    // Drag item 1 back to where it was
    await pressDragRelease(tester.getCenter(find.text('item 1')), const Offset(0, -200));
    check(visible: <int>[0, 1, 2, 3, 4, 5], hidden: <int>[6, 7]);
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 100));
    expect(tester.getTopLeft(find.text('item 3')), const Offset(0, 300));
    expect(items, orderedEquals(<int>[0, 1, 2, 3, 4, 5, 6, 7]));
  });

  testWidgets('SliverReorderableList, items inherit DefaultTextStyle, IconTheme', (
    WidgetTester tester,
  ) async {
    const Color textColor = Color(0xffffffff);
    const Color iconColor = Color(0xff0000ff);

    TextStyle getIconStyle() {
      return tester
          .widget<RichText>(find.descendant(of: find.byType(Icon), matching: find.byType(RichText)))
          .text
          .style!;
    }

    TextStyle getTextStyle() {
      return tester
          .widget<RichText>(
            find.descendant(of: find.text('item 0'), matching: find.byType(RichText)),
          )
          .text
          .style!;
    }

    // This SliverReorderableList has just one item: "item 0".
    await tester.pumpWidget(
      TestList(items: List<int>.from(<int>[0]), textColor: textColor, iconColor: iconColor),
    );
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(getIconStyle().color, iconColor);
    expect(getTextStyle().color, textColor);

    // Dragging item 0 causes it to be reparented in the overlay. The item
    // should still inherit the IconTheme and DefaultTextStyle because they are
    // InheritedThemes.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
    await tester.pump(kPressTimeout);
    await drag.moveBy(const Offset(0, 50));
    await tester.pump(kPressTimeout);
    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 50));
    expect(getIconStyle().color, iconColor);
    expect(getTextStyle().color, textColor);

    // Drag is complete, item 0 returns to where it was.
    await drag.up();
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(getIconStyle().color, iconColor);
    expect(getTextStyle().color, textColor);
  });

  testWidgets('SliverReorderableList - custom proxyDecorator', (WidgetTester tester) async {
    const ValueKey<String> fadeTransitionKey = ValueKey<String>('reordered-fade');

    await tester.pumpWidget(
      TestList(
        items: List<int>.from(<int>[0, 1, 2, 3]),
        proxyDecorator: (Widget child, int index, Animation<double> animation) {
          return AnimatedBuilder(
            animation: animation,
            builder: (BuildContext context, Widget? child) {
              final Tween<double> fadeValues = Tween<double>(begin: 1.0, end: 0.5);
              final Animation<double> fadeAnimation = animation.drive(fadeValues);
              return FadeTransition(key: fadeTransitionKey, opacity: fadeAnimation, child: child);
            },
            child: child,
          );
        },
      ),
    );

    Finder getItemFadeTransition() => find.byKey(fadeTransitionKey);

    expect(getItemFadeTransition(), findsNothing);

    // Start gesture on first item
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
    await tester.pump(kPressTimeout);

    // Drag enough for transition animation defined in proxyDecorator to start.
    await drag.moveBy(const Offset(0, 50));
    await tester.pump();

    // At the start, opacity should be at 1.0.
    expect(getItemFadeTransition(), findsOneWidget);
    FadeTransition fadeTransition = tester.widget(getItemFadeTransition());
    expect(fadeTransition.opacity.value, 1.0);

    // Let animation run halfway.
    await tester.pump(const Duration(milliseconds: 125));
    fadeTransition = tester.widget(getItemFadeTransition());
    expect(fadeTransition.opacity.value, greaterThan(0.5));
    expect(fadeTransition.opacity.value, lessThan(1.0));

    // Allow animation to run to the end.
    await tester.pumpAndSettle();
    expect(find.byKey(fadeTransitionKey), findsOneWidget);
    fadeTransition = tester.widget(getItemFadeTransition());
    expect(fadeTransition.opacity.value, 0.5);

    // Finish reordering.
    await drag.up();
    await tester.pumpAndSettle();
    expect(getItemFadeTransition(), findsNothing);
  });

  testWidgets(
    'ReorderableList supports items with nested list views without throwing layout exception.',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Ensure there is always a top padding to simulate a phone with
              // safe area at the top. If the nested list doesn't have the
              // padding removed before it is put into the overlay it will
              // overflow the layout by the top padding.
              data: MediaQuery.of(context).copyWith(padding: const EdgeInsets.only(top: 50)),
              child: child!,
            );
          },
          home: Scaffold(
            appBar: AppBar(title: const Text('Nested Lists')),
            body: ReorderableList(
              itemCount: 10,
              itemBuilder: (BuildContext context, int index) {
                return ReorderableDragStartListener(
                  index: index,
                  key: ValueKey<int>(index),
                  child: Column(
                    children: <Widget>[
                      ListView(
                        shrinkWrap: true,
                        physics: const ClampingScrollPhysics(),
                        children: const <Widget>[
                          Text('Other data'),
                          Text('Other data'),
                          Text('Other data'),
                        ],
                      ),
                    ],
                  ),
                );
              },
              onReorder: (int oldIndex, int newIndex) {},
            ),
          ),
        ),
      );

      // Start gesture on first item
      final TestGesture drag = await tester.startGesture(
        tester.getCenter(find.byKey(const ValueKey<int>(0))),
      );
      await tester.pump(kPressTimeout);

      // Drag enough for move to start
      await drag.moveBy(const Offset(0, 50));
      await tester.pumpAndSettle();

      // There shouldn't be a layout overflow exception.
      expect(tester.takeException(), isNull);
    },
  );

  testWidgets(
    'ReorderableList supports items with nested list views without throwing layout exception.',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/83224.
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Ensure there is always a top padding to simulate a phone with
              // safe area at the top. If the nested list doesn't have the
              // padding removed before it is put into the overlay it will
              // overflow the layout by the top padding.
              data: MediaQuery.of(context).copyWith(padding: const EdgeInsets.only(top: 50)),
              child: child!,
            );
          },
          home: Scaffold(
            appBar: AppBar(title: const Text('Nested Lists')),
            body: ReorderableList(
              itemCount: 10,
              itemBuilder: (BuildContext context, int index) {
                return ReorderableDragStartListener(
                  index: index,
                  key: ValueKey<int>(index),
                  child: Column(
                    children: <Widget>[
                      ListView(
                        shrinkWrap: true,
                        physics: const ClampingScrollPhysics(),
                        children: const <Widget>[
                          Text('Other data'),
                          Text('Other data'),
                          Text('Other data'),
                        ],
                      ),
                    ],
                  ),
                );
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
      await tester.pump(kPressTimeout);

      // Drag enough for move to start.
      await drag.moveBy(const Offset(0, 50));
      await tester.pumpAndSettle();

      // There shouldn't be a layout overflow exception.
      expect(tester.takeException(), isNull);
    },
  );

  testWidgets('SliverReorderableList - properly animates the drop in a reversed list', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/110949
    final List<int> items = List<int>.generate(8, (int index) => index);

    Future<void> pressDragRelease(Offset start, Offset delta) async {
      final TestGesture drag = await tester.startGesture(start);
      await tester.pump(kPressTimeout);
      await drag.moveBy(delta);
      await tester.pumpAndSettle();
      await drag.up();
      await tester.pump();
    }

    // The TestList is 800x600 SliverReorderableList with 8 items 800x100 each.
    // Each item has a text widget with 'item $index' that can be moved by a
    // press and drag gesture. For this test we are reversing the order so
    // the first item is at the bottom.
    await tester.pumpWidget(TestList(items: items, reverse: true));

    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 500));
    expect(tester.getTopLeft(find.text('item 2')), const Offset(0, 300));

    // Drag item 0 up and insert it between item 1 and item 2. It should
    // smoothly animate.
    await pressDragRelease(tester.getCenter(find.text('item 0')), const Offset(0, -50));
    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 450));
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 500));
    expect(tester.getTopLeft(find.text('item 2')), const Offset(0, 300));

    // After the first several frames we should be moving closer to the final position,
    // not further away as was the case with the original bug.
    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(450));
    expect(tester.getTopLeft(find.text('item 0')).dy, greaterThan(400));

    // Sample the middle (don't use exact values as it depends on the internal
    // curve being used).
    await tester.pump(const Duration(milliseconds: 125));
    expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(450));
    expect(tester.getTopLeft(find.text('item 0')).dy, greaterThan(400));

    // Sample the end of the animation.
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(450));
    expect(tester.getTopLeft(find.text('item 0')).dy, greaterThan(400));

    // Wait for it to finish, it should be back to the original position
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 400));
  });

  testWidgets(
    'SliverReorderableList - properly animates the drop at starting position in a reversed list',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/84625
      final List<int> items = List<int>.generate(8, (int index) => index);

      Future<void> pressDragRelease(Offset start, Offset delta) async {
        final TestGesture drag = await tester.startGesture(start);
        await tester.pump(kPressTimeout);
        await drag.moveBy(delta);
        await tester.pumpAndSettle();
        await drag.up();
        await tester.pump();
      }

      // The TestList is 800x600 SliverReorderableList with 8 items 800x100 each.
      // Each item has a text widget with 'item $index' that can be moved by a
      // press and drag gesture. For this test we are reversing the order so
      // the first item is at the bottom.
      await tester.pumpWidget(TestList(items: items, reverse: true));

      expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 500));
      expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 400));

      // Drag item 0 downwards off the edge and let it snap back. It should
      // smoothly animate back up.
      await pressDragRelease(tester.getCenter(find.text('item 0')), const Offset(0, 50));
      expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 550));
      expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 400));

      // After the first several frames we should be moving closer to the final position,
      // not further away as was the case with the original bug.
      await tester.pump(const Duration(milliseconds: 10));
      expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(550));

      // Sample the middle (don't use exact values as it depends on the internal
      // curve being used).
      await tester.pump(const Duration(milliseconds: 125));
      expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(550));

      // Wait for it to finish, it should be back to the original position
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('item 0')), const Offset(0, 500));
    },
  );

  testWidgets('SliverReorderableList - properly animates the drop at starting position', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/150843
    final List<int> items = List<int>.generate(3, (int index) => index);

    // The TestList is 300x100 SliverReorderableList with 3 items 100x100 each.
    // Each item has a text widget with 'item $index' that can be moved by a
    // press and drag gesture. For this test the first item is at the top
    await tester.pumpWidget(TestList(items: items));

    expect(tester.getTopLeft(find.text('item 0')), Offset.zero);
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 100));

    // Drag item 0 downwards and then upwards.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
    await tester.pump(kPressTimeout);
    await drag.moveBy(const Offset(0, 100));
    await tester.pumpAndSettle();
    await drag.moveBy(const Offset(0, -110));
    await tester.pump();
    expect(tester.getTopLeft(find.text('item 0')), const Offset(0, -10));
    expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 100));

    // Now leave the drag, it should go to index 0.
    await drag.up();
    await tester.pump();

    // It should not go to index 1 and come back
    await tester.pump(const Duration(milliseconds: 200));
    expect(tester.getTopLeft(find.text('item 0')).dy, lessThan(50));
  });

  testWidgets(
    'SliverReorderableList - properly animates the drop at starting position with reverse:true',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/150843
      final List<int> items = List<int>.generate(3, (int index) => index);

      // The TestList is 300x100 SliverReorderableList with 3 items 100x100 each.
      // Each item has a text widget with 'item $index' that can be moved by a
      // press and drag gesture. For this test the first item is at the top
      await tester.pumpWidget(TestList(items: items, reverse: true));

      expect(tester.getTopLeft(find.text('item 2')), const Offset(0, 300.0));
      expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 400.0));

      // Drag item 2 downwards and then upwards.
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 2')));
      await tester.pump(kPressTimeout);
      await drag.moveBy(const Offset(0, 100));
      await tester.pumpAndSettle();
      await drag.moveBy(const Offset(0, -110));
      await tester.pump();
      expect(tester.getTopLeft(find.text('item 2')), const Offset(0, 290));
      expect(tester.getTopLeft(find.text('item 1')), const Offset(0, 300));

      // Now leave the drag, it should go to index 1.
      await drag.up();
      await tester.pump();

      // It should not go to index 0 and come back
      await tester.pump(const Duration(milliseconds: 200));
      expect(tester.getTopLeft(find.text('item 2')).dy, greaterThan(350));
    },
  );

  testWidgets('SliverReorderableList calls onReorderStart and onReorderEnd correctly', (
    WidgetTester tester,
  ) async {
    final List<int> items = List<int>.generate(8, (int index) => index);
    int? startIndex, endIndex;
    final Finder item0 = find.textContaining('item 0');

    await tester.pumpWidget(
      TestList(
        items: items,
        onReorderStart: (int index) {
          startIndex = index;
        },
        onReorderEnd: (int index) {
          endIndex = index;
        },
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

  testWidgets('ReorderableList calls onReorderStart and onReorderEnd correctly', (
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
        home: ReorderableList(
          itemCount: items.length,
          itemBuilder: (BuildContext context, int index) {
            return SizedBox(
              key: ValueKey<int>(items[index]),
              height: 100,
              child: ReorderableDelayedDragStartListener(
                index: index,
                child: Text('item ${items[index]}'),
              ),
            );
          },
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
    await tester.pump(kLongPressTimeout);
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
    await tester.pump(kLongPressTimeout);
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

  testWidgets('ReorderableList asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];
    expect(
      () => ReorderableList(
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

  testWidgets('ReorderableList passes itemExtentBuilder to SliverReorderableList', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/155936
    const int itemCount = 5;
    const List<double> items = <double>[10.0, 20.0, 30.0, 40.0, 50.0];

    void handleReorder(int fromIndex, int toIndex) {
      if (toIndex > fromIndex) {
        toIndex -= 1;
      }
      items.insert(toIndex, items.removeAt(fromIndex));
    }

    // The list has five elements, that indicate the extent for the item at the given index.
    await tester.pumpWidget(
      MaterialApp(
        home: ReorderableList(
          itemBuilder:
              (BuildContext context, int index) =>
                  SizedBox(key: ValueKey<double>(items[index]), child: Text('Item $index')),
          itemCount: itemCount,
          onReorder: handleReorder,
          itemExtentBuilder: (int index, SliverLayoutDimensions dimensions) {
            return items[index];
          },
        ),
      ),
    );

    const Map<int, double> expectedExtents = <int, double>{
      0: 10.0,
      1: 20.0,
      2: 30.0,
      3: 40.0,
      4: 50.0,
    };

    final Map<int, double> itemExtents = <int, double>{
      for (int i = 0; i < itemCount; i++) i: tester.getSize(find.text('Item $i')).height,
    };

    expect(const MapEquality<int, double>().equals(itemExtents, expectedExtents), isTrue);
  });

  testWidgets('SliverReorderableList asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];
    expect(
      () => SliverReorderableList(
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
              return ReorderableList(
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
                onReorder: (int fromIndex, int toIndex) {
                  if (fromIndex < toIndex) {
                    toIndex--;
                  }
                  final int value = numbers.removeAt(fromIndex);
                  numbers.insert(toIndex, value);
                },
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
              return ReorderableList(
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

  group('ReorderableDragStartListener', () {
    testWidgets('It should allow the item to be dragged when enabled is true', (
      WidgetTester tester,
    ) async {
      const int itemCount = 5;
      int onReorderCallCount = 0;
      final List<int> items = List<int>.generate(itemCount, (int index) => index);

      void handleReorder(int fromIndex, int toIndex) {
        onReorderCallCount += 1;
        if (toIndex > fromIndex) {
          toIndex -= 1;
        }
        items.insert(toIndex, items.removeAt(fromIndex));
      }

      // The list has five elements of height 100
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableList(
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                key: ValueKey<int>(items[index]),
                height: 100,
                child: ReorderableDragStartListener(
                  index: index,
                  child: Text('item ${items[index]}'),
                ),
              );
            },
            onReorder: handleReorder,
          ),
        ),
      );

      // Start gesture on first item
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
      await tester.pump(kPressTimeout);

      // Drag enough to move down the first item
      await drag.moveBy(const Offset(0, 50));
      await tester.pump();
      await drag.up();
      await tester.pumpAndSettle();

      expect(onReorderCallCount, 1);
      expect(items, orderedEquals(<int>[1, 0, 2, 3, 4]));
    });

    testWidgets('It should not allow the item to be dragged when enabled is false', (
      WidgetTester tester,
    ) async {
      const int itemCount = 5;
      int onReorderCallCount = 0;
      final List<int> items = List<int>.generate(itemCount, (int index) => index);

      void handleReorder(int fromIndex, int toIndex) {
        onReorderCallCount += 1;
        if (toIndex > fromIndex) {
          toIndex -= 1;
        }
        items.insert(toIndex, items.removeAt(fromIndex));
      }

      // The list has five elements of height 100
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableList(
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                key: ValueKey<int>(items[index]),
                height: 100,
                child: ReorderableDragStartListener(
                  index: index,
                  enabled: false,
                  child: Text('item ${items[index]}'),
                ),
              );
            },
            onReorder: handleReorder,
          ),
        ),
      );

      // Start gesture on first item
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
      await tester.pump(kLongPressTimeout);

      // Drag enough to move down the first item
      await drag.moveBy(const Offset(0, 50));
      await tester.pump();
      await drag.up();
      await tester.pumpAndSettle();

      expect(onReorderCallCount, 0);
      expect(items, orderedEquals(<int>[0, 1, 2, 3, 4]));
    });
  });

  group('ReorderableDelayedDragStartListener', () {
    testWidgets('It should allow the item to be dragged when enabled is true', (
      WidgetTester tester,
    ) async {
      const int itemCount = 5;
      int onReorderCallCount = 0;
      final List<int> items = List<int>.generate(itemCount, (int index) => index);

      void handleReorder(int fromIndex, int toIndex) {
        onReorderCallCount += 1;
        if (toIndex > fromIndex) {
          toIndex -= 1;
        }
        items.insert(toIndex, items.removeAt(fromIndex));
      }

      // The list has five elements of height 100
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableList(
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                key: ValueKey<int>(items[index]),
                height: 100,
                child: ReorderableDelayedDragStartListener(
                  index: index,
                  child: Text('item ${items[index]}'),
                ),
              );
            },
            onReorder: handleReorder,
          ),
        ),
      );
      await tester.pumpAndSettle();

      // Start gesture on first item
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
      await tester.pump(kLongPressTimeout);

      // Drag enough to move down the first item
      await drag.moveBy(const Offset(0, 50));
      await tester.pump();
      await drag.up();
      await tester.pumpAndSettle();

      expect(onReorderCallCount, 1);
      expect(items, orderedEquals(<int>[1, 0, 2, 3, 4]));
    });

    testWidgets('It should not allow the item to be dragged when enabled is false', (
      WidgetTester tester,
    ) async {
      const int itemCount = 5;
      int onReorderCallCount = 0;
      final List<int> items = List<int>.generate(itemCount, (int index) => index);

      void handleReorder(int fromIndex, int toIndex) {
        onReorderCallCount += 1;
        if (toIndex > fromIndex) {
          toIndex -= 1;
        }
        items.insert(toIndex, items.removeAt(fromIndex));
      }

      // The list has five elements of height 100
      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableList(
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                key: ValueKey<int>(items[index]),
                height: 100,
                child: ReorderableDelayedDragStartListener(
                  index: index,
                  enabled: false,
                  child: Text('item ${items[index]}'),
                ),
              );
            },
            onReorder: handleReorder,
          ),
        ),
      );

      // Start gesture on first item
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('item 0')));
      await tester.pump(kLongPressTimeout);

      // Drag enough to move down the first item
      await drag.moveBy(const Offset(0, 50));
      await tester.pump();
      await drag.up();
      await tester.pumpAndSettle();

      expect(onReorderCallCount, 0);
      expect(items, orderedEquals(<int>[0, 1, 2, 3, 4]));
    });
  });

  testWidgets('SliverReorderableList properly disposes items', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/105010
    const int itemCount = 5;
    final List<int> items = List<int>.generate(itemCount, (int index) => index);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          drawer: Drawer(
            child: Builder(
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    Expanded(
                      child: CustomScrollView(
                        slivers: <Widget>[
                          SliverReorderableList(
                            itemCount: itemCount,
                            itemBuilder: (BuildContext context, int index) {
                              return Material(
                                key: ValueKey<String>('item-$index'),
                                child: ReorderableDragStartListener(
                                  index: index,
                                  child: ListTile(title: Text('item ${items[index]}')),
                                ),
                              );
                            },
                            onReorder: (int oldIndex, int newIndex) {},
                          ),
                        ],
                      ),
                    ),
                    TextButton(
                      onPressed: () {
                        Scaffold.of(context).closeDrawer();
                      },
                      child: const Text('Close drawer'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.menu));
    await tester.pumpAndSettle();

    final Finder item0 = find.text('item 0');
    expect(item0, findsOneWidget);

    // Start gesture on first item without drag up event.
    final TestGesture drag = await tester.startGesture(tester.getCenter(item0));
    await drag.moveBy(const Offset(0, 200));
    await tester.pump();

    await tester.tap(find.text('Close drawer'));
    await tester.pumpAndSettle();

    expect(item0, findsNothing);
  });

  testWidgets('SliverReorderableList auto scrolls speed is configurable', (
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
          home: CustomScrollView(
            controller: scrollController,
            slivers: <Widget>[
              SliverReorderableList(
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    key: ValueKey<int>(items[index]),
                    height: 100,
                    color: items[index].isOdd ? Colors.red : Colors.green,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: <Widget>[
                          Text('item ${items[index]}'),
                          const Icon(Icons.drag_handle),
                        ],
                      ),
                    ),
                  );
                },
                itemCount: items.length,
                onReorder: (int fromIndex, int toIndex) {},
                autoScrollerVelocityScalar: autoScrollerVelocityScalar,
              ),
            ],
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

  testWidgets(
    'Null check error when dragging and dropping last element into last index with reverse:true',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/132077
      const int itemCount = 5;
      final List<String> items = List<String>.generate(
        itemCount,
        (int index) => 'Item ${index + 1}',
      );

      await tester.pumpWidget(
        MaterialApp(
          home: ReorderableList(
            onReorder: (int oldIndex, int newIndex) {
              if (newIndex > oldIndex) {
                newIndex -= 1;
              }
              final String item = items.removeAt(oldIndex);
              items.insert(newIndex, item);
            },
            itemCount: items.length,
            reverse: true,
            itemBuilder: (BuildContext context, int index) {
              return ReorderableDragStartListener(
                key: Key('$index'),
                index: index,
                child: Material(child: ListTile(title: Text(items[index]))),
              );
            },
          ),
        ),
      );

      // Start gesture on last item
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 5')));
      await tester.pump(kLongPressTimeout);

      // Drag to move up the last item, and drop at the last index
      await drag.moveBy(const Offset(0, -50));
      await tester.pump();
      await drag.up();
      await tester.pumpAndSettle();

      expect(tester.takeException(), null);
    },
  );

  testWidgets('When creating a new item, be in the correct position', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: LayoutBuilder(
          builder: (_, BoxConstraints view) {
            // The third one just appears on the screen
            final double itemSize = view.maxWidth / 2 - 20;
            return Scaffold(
              body: CustomScrollView(
                scrollDirection: Axis.horizontal,
                cacheExtent: 0, // The fourth one will not be created in the initial state.
                slivers: <Widget>[
                  SliverReorderableList(
                    itemBuilder: (BuildContext context, int index) {
                      return ReorderableDragStartListener(
                        key: ValueKey<int>(index),
                        index: index,
                        child: Builder(
                          builder: (BuildContext context) {
                            return SizedBox(width: itemSize, child: Text('$index'));
                          },
                        ),
                      );
                    },
                    itemCount: 4,
                    onReorder: (int fromIndex, int toIndex) {},
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('0')));
    await tester.pump(kLongPressTimeout);
    await drag.moveBy(const Offset(20, 0));
    await tester.pump();
    expect(find.text('3').hitTestable(at: Alignment.topLeft), findsNothing);
    await drag.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Tests the correctness of the drop animation in various scenarios', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/138994
    late Size screenSize;
    final List<double> itemSizes = <double>[20, 50, 30, 80, 100, 30];
    Future<void> pumpFor(bool reverse, Axis scrollDirection) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              screenSize = MediaQuery.sizeOf(context);
              return Scaffold(
                body: CustomScrollView(
                  reverse: reverse,
                  scrollDirection: scrollDirection,
                  slivers: <Widget>[
                    SliverReorderableList(
                      itemBuilder: (BuildContext context, int index) {
                        return ReorderableDragStartListener(
                          key: ValueKey<int>(index),
                          index: index,
                          child: Builder(
                            builder: (BuildContext context) {
                              return SizedBox(
                                height:
                                    scrollDirection == Axis.vertical
                                        ? itemSizes[index]
                                        : double.infinity,
                                width:
                                    scrollDirection == Axis.horizontal
                                        ? itemSizes[index]
                                        : double.infinity,
                                child: Text('$index'),
                              );
                            },
                          ),
                        );
                      },
                      itemCount: itemSizes.length,
                      onReorder: (int fromIndex, int toIndex) {},
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      );
    }

    Future<void> testMove(
      int from,
      int to, {
      bool reverse = false,
      Axis scrollDirection = Axis.vertical,
    }) async {
      await pumpFor(reverse, scrollDirection);
      final double targetOffset = (List<double>.of(itemSizes)..removeAt(from)).sublist(0, to).sum;
      final Offset targetPosition =
          reverse
              ? (scrollDirection == Axis.vertical
                  ? Offset(0, screenSize.height - targetOffset - itemSizes[from])
                  : Offset(screenSize.width - targetOffset - itemSizes[from], 0))
              : (scrollDirection == Axis.vertical
                  ? Offset(0, targetOffset)
                  : Offset(targetOffset, 0));
      final Offset moveOffset = targetPosition - tester.getTopLeft(find.text('$from'));
      await tester.timedDrag(find.text('$from'), moveOffset, const Duration(seconds: 1));
      // Before the drop animation starts
      final Offset animationBeginOffset = tester.getTopLeft(find.text('$from'));
      // Halfway through the animation
      await tester.pump(const Duration(milliseconds: 125));
      expect(
        tester.getTopLeft(find.text('$from')),
        Offset.lerp(animationBeginOffset, targetPosition, 0.5),
      );
      // Animation ends
      await tester.pump(const Duration(milliseconds: 125));
      expect(tester.getTopLeft(find.text('$from')), targetPosition);
      await tester.pumpAndSettle();
    }

    final List<(int, int)> testCases = <(int, int)>[(3, 1), (3, 3), (3, 5), (0, 5), (5, 0)];
    for (final (int, int) element in testCases) {
      await testMove(element.$1, element.$2);
      await testMove(element.$1, element.$2, reverse: true);
      await testMove(element.$1, element.$2, scrollDirection: Axis.horizontal);
      await testMove(element.$1, element.$2, reverse: true, scrollDirection: Axis.horizontal);
    }
  });

  testWidgets('Tests that the item position is correct when prototypeItem or itemExtent are set', (
    WidgetTester tester,
  ) async {
    Future<void> pumpFor({Widget? prototypeItem, double? itemExtent}) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              slivers: <Widget>[
                SliverReorderableList(
                  itemBuilder: (BuildContext context, int index) {
                    return ReorderableDragStartListener(
                      key: ValueKey<int>(index),
                      index: index,
                      child: SizedBox(height: 100, child: Text('$index')),
                    );
                  },
                  itemCount: 5,
                  itemExtent: itemExtent,
                  prototypeItem: prototypeItem,
                  onReorder: (int fromIndex, int toIndex) {},
                ),
              ],
            ),
          ),
        ),
      );
    }

    Future<void> testFor({Widget? prototypeItem, double? itemExtent}) async {
      await pumpFor(prototypeItem: prototypeItem, itemExtent: itemExtent);
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('0')));
      await tester.pump(kLongPressTimeout);
      await drag.moveBy(const Offset(0, 20));
      await tester.pump();
      expect(tester.getTopLeft(find.text('1')), const Offset(0, 100));
      await drag.up();
      await tester.pumpAndSettle();
    }

    await testFor();
    await testFor(prototypeItem: const SizedBox(height: 100, width: 100, child: Text('prototype')));
    await testFor(itemExtent: 100);
  });

  testWidgets('The item being dragged will not be affected by layout constraints.', (
    WidgetTester tester,
  ) async {
    final Map<int, BoxConstraints> itemLayoutConstraints = <int, BoxConstraints>{};
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverReorderableList(
                itemBuilder: (BuildContext context, int index) {
                  return LayoutBuilder(
                    key: ValueKey<int>(index),
                    builder: (BuildContext context, BoxConstraints constraints) {
                      itemLayoutConstraints[index] = constraints;
                      return SizedBox(
                        height: 100,
                        child: ReorderableDragStartListener(index: index, child: Text('$index')),
                      );
                    },
                  );
                },
                itemCount: 5,
                onReorder: (int fromIndex, int toIndex) {},
              ),
            ],
          ),
        ),
      ),
    );
    final Map<int, BoxConstraints> preDragLayoutConstraints = Map<int, BoxConstraints>.of(
      itemLayoutConstraints,
    );
    itemLayoutConstraints.clear();
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('0')));
    await tester.pump(kLongPressTimeout);
    await drag.moveBy(const Offset(0, 20));
    await tester.pump();
    expect(itemLayoutConstraints, preDragLayoutConstraints);
    await drag.up();
    await tester.pumpAndSettle();
  });

  testWidgets('DragBoundary defines the boundary for ReorderableList.', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Container(
            margin: const EdgeInsets.only(top: 100),
            height: 300,
            child: DragBoundary(
              child: CustomScrollView(
                slivers: <Widget>[
                  SliverReorderableList(
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
                ],
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
}

class TestList extends StatelessWidget {
  const TestList({
    super.key,
    this.textColor,
    this.iconColor,
    this.proxyDecorator,
    required this.items,
    this.reverse = false,
    this.onReorderStart,
    this.onReorderEnd,
    this.autoScrollerVelocityScalar,
  });

  final List<int> items;
  final Color? textColor;
  final Color? iconColor;
  final ReorderItemProxyDecorator? proxyDecorator;
  final bool reverse;
  final void Function(int)? onReorderStart, onReorderEnd;
  final double? autoScrollerVelocityScalar;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: DefaultTextStyle(
          style: TextStyle(color: textColor),
          child: IconTheme(
            data: IconThemeData(color: iconColor),
            child: StatefulBuilder(
              builder: (BuildContext outerContext, StateSetter setState) {
                final List<int> items = this.items;
                return CustomScrollView(
                  reverse: reverse,
                  slivers: <Widget>[
                    SliverReorderableList(
                      itemBuilder: (BuildContext context, int index) {
                        return Container(
                          key: ValueKey<int>(items[index]),
                          height: 100,
                          color: items[index].isOdd ? Colors.red : Colors.green,
                          child: ReorderableDragStartListener(
                            index: index,
                            child: Row(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: <Widget>[
                                Text('item ${items[index]}'),
                                const Icon(Icons.drag_handle),
                              ],
                            ),
                          ),
                        );
                      },
                      itemCount: items.length,
                      onReorder: (int fromIndex, int toIndex) {
                        setState(() {
                          if (toIndex > fromIndex) {
                            toIndex -= 1;
                          }
                          items.insert(toIndex, items.removeAt(fromIndex));
                        });
                      },
                      proxyDecorator: proxyDecorator,
                      onReorderStart: onReorderStart,
                      onReorderEnd: onReorderEnd,
                      autoScrollerVelocityScalar: autoScrollerVelocityScalar,
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );
  }
}
