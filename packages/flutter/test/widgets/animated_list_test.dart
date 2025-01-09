// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/src/foundation/diagnostics.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/100451
  testWidgets('SliverAnimatedList.builder respects findChildIndexCallback', (
    WidgetTester tester,
  ) async {
    bool finderCalled = false;
    int itemCount = 7;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return CustomScrollView(
              slivers: <Widget>[
                SliverAnimatedList(
                  initialItemCount: itemCount,
                  itemBuilder:
                      (BuildContext context, int index, Animation<double> animation) =>
                          Container(key: Key('$index'), height: 2000.0),
                  findChildIndexCallback: (Key key) {
                    finderCalled = true;
                    return null;
                  },
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

  testWidgets('AnimatedList', (WidgetTester tester) async {
    Widget builder(BuildContext context, int index, Animation<double> animation) {
      return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
    }

    final GlobalKey<AnimatedListState> listKey = GlobalKey<AnimatedListState>();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedList(key: listKey, initialItemCount: 2, itemBuilder: builder),
      ),
    );

    expect(
      find.byWidgetPredicate((Widget widget) {
        return widget is SliverAnimatedList &&
            widget.initialItemCount == 2 &&
            widget.itemBuilder == builder;
      }),
      findsOneWidget,
    );

    listKey.currentState!.insertItem(0);
    await tester.pump();
    expect(find.text('item 2'), findsOneWidget);

    listKey.currentState!.removeItem(2, (BuildContext context, Animation<double> animation) {
      return const SizedBox(height: 100.0, child: Center(child: Text('removing item')));
    }, duration: const Duration(milliseconds: 100));

    await tester.pump();
    expect(find.text('removing item'), findsOneWidget);
    expect(find.text('item 2'), findsNothing);

    await tester.pumpAndSettle();
    expect(find.text('removing item'), findsNothing);

    // Test for insertAllItems
    listKey.currentState!.insertAllItems(0, 2);
    await tester.pump();
    expect(find.text('item 2'), findsOneWidget);
    expect(find.text('item 3'), findsOneWidget);

    // Test for removeAllItems
    listKey.currentState!.removeAllItems((BuildContext context, Animation<double> animation) {
      return const SizedBox(height: 100.0, child: Center(child: Text('removing item')));
    }, duration: const Duration(milliseconds: 100));

    await tester.pump();
    expect(find.text('removing item'), findsWidgets);
    expect(find.text('item 0'), findsNothing);
    expect(find.text('item 1'), findsNothing);
    expect(find.text('item 2'), findsNothing);
    expect(find.text('item 3'), findsNothing);

    await tester.pumpAndSettle();
    expect(find.text('removing item'), findsNothing);
  });

  group('SliverAnimatedList', () {
    testWidgets('initialItemCount', (WidgetTester tester) async {
      final Map<int, Animation<double>> animations = <int, Animation<double>>{};

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedList(
                initialItemCount: 2,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  animations[index] = animation;
                  return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
                },
              ),
            ],
          ),
        ),
      );

      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(animations.containsKey(0), true);
      expect(animations.containsKey(1), true);
      expect(animations[0]!.value, 1.0);
      expect(animations[1]!.value, 1.0);
    });

    testWidgets('insert', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedList(
                key: listKey,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return SizeTransition(
                    key: ValueKey<int>(index),
                    sizeFactor: animation,
                    child: SizedBox(height: 100.0, child: Center(child: Text('item $index'))),
                  );
                },
              ),
            ],
          ),
        ),
      );

      double itemHeight(int index) =>
          tester.getSize(find.byKey(ValueKey<int>(index), skipOffstage: false)).height;
      double itemTop(int index) =>
          tester.getTopLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dy;
      double itemBottom(int index) =>
          tester.getBottomLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dy;

      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // Newly inserted item 0's height should animate from 0 to 100
      expect(itemHeight(0), 0.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 50.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 100.0);

      // The list now contains one fully expanded item at the top:
      expect(find.text('item 0'), findsOneWidget);
      expect(itemTop(0), 0.0);
      expect(itemBottom(0), 100.0);

      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // The height of the newly inserted items at index 0 and 1 should animate
      // from 0 to 100.
      // The height of the original item, now at index 2, should remain 100.
      expect(itemHeight(0), 0.0);
      expect(itemHeight(1), 0.0);
      expect(itemHeight(2), 100.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 50.0);
      expect(itemHeight(1), 50.0);
      expect(itemHeight(2), 100.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 100.0);
      expect(itemHeight(1), 100.0);
      expect(itemHeight(2), 100.0);

      // The newly inserted "item 1" and "item 2" appear above "item 0"
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(find.text('item 2'), findsOneWidget);
      expect(itemTop(0), 0.0);
      expect(itemBottom(0), 100.0);
      expect(itemTop(1), 100.0);
      expect(itemBottom(1), 200.0);
      expect(itemTop(2), 200.0);
      expect(itemBottom(2), 300.0);
    });

    // Test for insertAllItems with SliverAnimatedList
    testWidgets('insertAll', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedList(
                key: listKey,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return SizeTransition(
                    key: ValueKey<int>(index),
                    sizeFactor: animation,
                    child: SizedBox(height: 100.0, child: Center(child: Text('item $index'))),
                  );
                },
              ),
            ],
          ),
        ),
      );

      double itemHeight(int index) =>
          tester.getSize(find.byKey(ValueKey<int>(index), skipOffstage: false)).height;
      double itemTop(int index) =>
          tester.getTopLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dy;
      double itemBottom(int index) =>
          tester.getBottomLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dy;

      listKey.currentState!.insertAllItems(0, 2, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // Newly inserted item 0 & 1's height should animate from 0 to 100
      expect(itemHeight(0), 0.0);
      expect(itemHeight(1), 0.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 50.0);
      expect(itemHeight(1), 50.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemHeight(0), 100.0);
      expect(itemHeight(1), 100.0);

      // The list now contains two fully expanded items at the top:
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(itemTop(0), 0.0);
      expect(itemBottom(0), 100.0);
      expect(itemTop(1), 100.0);
      expect(itemBottom(1), 200.0);
    });

    // Test for removeAllItems with SliverAnimatedList
    testWidgets('remove', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();
      final List<int> items = <int>[0, 1, 2];

      Widget buildItem(BuildContext context, int item, Animation<double> animation) {
        return SizeTransition(
          key: ValueKey<int>(item),
          sizeFactor: animation,
          child: SizedBox(
            height: 100.0,
            child: Center(child: Text('item $item', textDirection: TextDirection.ltr)),
          ),
        );
      }

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedList(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return buildItem(context, items[index], animation);
                },
              ),
            ],
          ),
        ),
      );

      double itemTop(int index) => tester.getTopLeft(find.byKey(ValueKey<int>(index))).dy;
      double itemBottom(int index) => tester.getBottomLeft(find.byKey(ValueKey<int>(index))).dy;

      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(find.text('item 2'), findsOneWidget);

      items.removeAt(0);
      listKey.currentState!.removeItem(
        0,
        (BuildContext context, Animation<double> animation) => buildItem(context, 0, animation),
        duration: const Duration(milliseconds: 100),
      );

      // Items 0, 1, 2 at 0, 100, 200. All heights 100.
      expect(itemTop(0), 0.0);
      expect(itemBottom(0), 100.0);
      expect(itemTop(1), 100.0);
      expect(itemBottom(1), 200.0);
      expect(itemTop(2), 200.0);
      expect(itemBottom(2), 300.0);

      // Newly removed item 0's height should animate from 100 to 0 over 100ms

      // Items 0, 1, 2 at 0, 50, 150. Item 0's height is 50.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemTop(0), 0.0);
      expect(itemBottom(0), 50.0);
      expect(itemTop(1), 50.0);
      expect(itemBottom(1), 150.0);
      expect(itemTop(2), 150.0);
      expect(itemBottom(2), 250.0);

      // Items 1, 2 at 0, 100.
      await tester.pumpAndSettle();
      expect(itemTop(1), 0.0);
      expect(itemBottom(1), 100.0);
      expect(itemTop(2), 100.0);
      expect(itemBottom(2), 200.0);
    });

    // Test for removeAllItems with SliverAnimatedList
    testWidgets('removeAll', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();
      final List<int> items = <int>[0, 1, 2];

      Widget buildItem(BuildContext context, int item, Animation<double> animation) {
        return SizeTransition(
          key: ValueKey<int>(item),
          sizeFactor: animation,
          child: SizedBox(
            height: 100.0,
            child: Center(child: Text('item $item', textDirection: TextDirection.ltr)),
          ),
        );
      }

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedList(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return buildItem(context, items[index], animation);
                },
              ),
            ],
          ),
        ),
      );

      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(find.text('item 2'), findsOneWidget);

      items.clear();
      listKey.currentState!.removeAllItems(
        (BuildContext context, Animation<double> animation) => buildItem(context, 0, animation),
        duration: const Duration(milliseconds: 100),
      );

      await tester.pumpAndSettle();

      expect(find.text('item 0'), findsNothing);
      expect(find.text('item 1'), findsNothing);
      expect(find.text('item 2'), findsNothing);
    });

    testWidgets('works in combination with other slivers', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  const SizedBox(height: 100),
                  const SizedBox(height: 100),
                ]),
              ),
              SliverAnimatedList(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return SizedBox(height: 100, child: Text('item $index'));
                },
              ),
            ],
          ),
        ),
      );

      expect(tester.getTopLeft(find.text('item 0')).dy, 200);
      expect(tester.getTopLeft(find.text('item 1')).dy, 300);

      listKey.currentState!.insertItem(3);
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('item 3')).dy, 500);

      listKey.currentState!.removeItem(0, (BuildContext context, Animation<double> animation) {
        return SizeTransition(
          sizeFactor: animation,
          key: const ObjectKey('removing'),
          child: const SizedBox(height: 100, child: Text('removing')),
        );
      }, duration: const Duration(seconds: 1));

      await tester.pump();
      expect(find.text('item 3'), findsNothing);

      await tester.pump(const Duration(milliseconds: 500));
      expect(tester.getSize(find.byKey(const ObjectKey('removing'))).height, 50);
      expect(tester.getTopLeft(find.text('item 0')).dy, 250);

      await tester.pumpAndSettle();
      expect(find.text('removing'), findsNothing);
      expect(tester.getTopLeft(find.text('item 0')).dy, 200);
    });

    testWidgets(
      'passes correctly derived index of findChildIndexCallback to the inner SliverChildBuilderDelegate',
      (WidgetTester tester) async {
        final List<int> items = <int>[0, 1, 2, 3];
        final GlobalKey<SliverAnimatedListState> listKey = GlobalKey<SliverAnimatedListState>();

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CustomScrollView(
              slivers: <Widget>[
                SliverAnimatedList(
                  key: listKey,
                  initialItemCount: items.length,
                  itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                    return _StatefulListItem(key: ValueKey<int>(items[index]), index: index);
                  },
                  findChildIndexCallback: (Key key) {
                    final int index = items.indexOf((key as ValueKey<int>).value);
                    return index == -1 ? null : index;
                  },
                ),
              ],
            ),
          ),
        );

        // get all list entries in order
        final List<Text> listEntries =
            find.byType(Text).evaluate().map((Element e) => e.widget as Text).toList();

        // check that the list is rendered in the correct order
        expect(listEntries[0].data, equals('item 0'));
        expect(listEntries[1].data, equals('item 1'));
        expect(listEntries[2].data, equals('item 2'));
        expect(listEntries[3].data, equals('item 3'));

        // delete one item
        listKey.currentState?.removeItem(0, (BuildContext context, Animation<double> animation) {
          return Container();
        });

        // delete from list
        items.removeAt(0);

        // reorder list
        items.insert(0, items.removeLast());

        // render with new list order
        await tester.pumpAndSettle();

        // get all list entries in order
        final List<Text> reorderedListEntries =
            find.byType(Text).evaluate().map((Element e) => e.widget as Text).toList();

        // check that the stateful items of the list are rendered in the order provided by findChildIndexCallback
        expect(reorderedListEntries[0].data, equals('item 3'));
        expect(reorderedListEntries[1].data, equals('item 1'));
        expect(reorderedListEntries[2].data, equals('item 2'));
      },
    );
  });

  testWidgets(
    'AnimatedList.of() and maybeOf called with a context that does not contain AnimatedList',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(Container(key: key));
      late FlutterError error;
      expect(AnimatedList.maybeOf(key.currentContext!), isNull);
      try {
        AnimatedList.of(key.currentContext!);
      } on FlutterError catch (e) {
        error = e;
      }
      expect(error.diagnostics.length, 4);
      expect(error.diagnostics[2].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[2].toStringDeep(),
        equalsIgnoringHashCodes(
          'This can happen when the context provided is from the same\n'
          'StatefulWidget that built the AnimatedList. Please see the\n'
          'AnimatedList documentation for examples of how to refer to an\n'
          'AnimatedListState object:\n'
          '  https://api.flutter.dev/flutter/widgets/AnimatedListState-class.html\n',
        ),
      );
      expect(error.diagnostics[3], isA<DiagnosticsProperty<Element>>());
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   AnimatedList.of() called with a context that does not contain an\n'
          '   AnimatedList.\n'
          '   No AnimatedList ancestor could be found starting from the context\n'
          '   that was passed to AnimatedList.of().\n'
          '   This can happen when the context provided is from the same\n'
          '   StatefulWidget that built the AnimatedList. Please see the\n'
          '   AnimatedList documentation for examples of how to refer to an\n'
          '   AnimatedListState object:\n'
          '     https://api.flutter.dev/flutter/widgets/AnimatedListState-class.html\n'
          '   The context used was:\n'
          '     Container-[GlobalKey#32cc6]\n',
        ),
      );
    },
  );

  testWidgets('AnimatedList.clipBehavior is forwarded to its inner CustomScrollView', (
    WidgetTester tester,
  ) async {
    const Clip clipBehavior = Clip.none;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedList(
          initialItemCount: 2,
          clipBehavior: clipBehavior,
          itemBuilder: (BuildContext context, int index, Animation<double> _) {
            return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
          },
        ),
      ),
    );

    expect(
      tester.widget<CustomScrollView>(find.byType(CustomScrollView)).clipBehavior,
      clipBehavior,
    );
  });

  testWidgets('AnimatedList.shrinkwrap is forwarded to its inner CustomScrollView', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/115040
    final ScrollController controller = ScrollController();

    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedList(
          controller: controller,
          initialItemCount: 2,
          shrinkWrap: true,
          itemBuilder: (BuildContext context, int index, Animation<double> _) {
            return SizedBox(height: 100.0, child: Center(child: Text('Item $index')));
          },
        ),
      ),
    );

    expect(tester.widget<CustomScrollView>(find.byType(CustomScrollView)).shrinkWrap, true);
  });

  testWidgets('AnimatedList applies MediaQuery padding', (WidgetTester tester) async {
    const EdgeInsets padding = EdgeInsets.all(30.0);
    EdgeInsets? innerMediaQueryPadding;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.all(30.0)),
          child: AnimatedList(
            initialItemCount: 3,
            itemBuilder: (BuildContext context, int index, Animation<double> animation) {
              innerMediaQueryPadding = MediaQuery.paddingOf(context);
              return const Placeholder();
            },
          ),
        ),
      ),
    );
    final Offset topLeft = tester.getTopLeft(find.byType(Placeholder).first);
    // Automatically apply the top padding into sliver.
    expect(topLeft, Offset(0.0, padding.top));

    // Scroll to the bottom.
    await tester.drag(find.byType(AnimatedList), const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();

    final Offset bottomLeft = tester.getBottomLeft(find.byType(Placeholder).last);
    // Automatically apply the bottom padding into sliver.
    expect(bottomLeft, Offset(0.0, 600.0 - padding.bottom));

    // Verify that the left/right padding is not applied.
    expect(innerMediaQueryPadding, const EdgeInsets.symmetric(horizontal: 30.0));
  });

  testWidgets('AnimatedList.separated', (WidgetTester tester) async {
    tester.view.physicalSize = const Size(600, 1800);
    tester.view.devicePixelRatio = 1.0;
    addTearDown(tester.view.reset);

    Widget builder(BuildContext context, int index, Animation<double> animation) {
      return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
    }

    Widget separatorBuilder(BuildContext context, int index, Animation<double> animation) {
      return SizedBox(height: 100.0, child: Center(child: Text('separator after item $index')));
    }

    Widget itemRemovalBuilder(BuildContext context, int? index, Animation<double> animation) {
      final String text = index != null ? 'removing item $index' : 'removing item';
      return SizedBox(height: 100.0, child: Center(child: Text(text)));
    }

    // Helper function to wrap itemRemovalBuilder with index
    // to allow testing removal of an item at the expected index.
    // Null index is necessary for removeAllItems.
    AnimatedRemovedItemBuilder itemRemovalBuilderWrapper({int? index}) {
      return (BuildContext context, Animation<double> animation) {
        return itemRemovalBuilder(context, index, animation);
      };
    }

    Widget separatorRemovalBuilder(BuildContext context, int index, Animation<double> animation) {
      return SizedBox(
        height: 100.0,
        child: Center(child: Text('removing separator after item $index')),
      );
    }

    List<Text> getItemsSeparatorsTexts(WidgetTester tester) {
      final Finder itemsSeparators = find.descendant(
        of: find.byType(SliverAnimatedList),
        matching: find.byType(Text),
      );
      return itemsSeparators.allCandidates.map((Element e) => e.widget).whereType<Text>().toList();
    }

    final GlobalKey<AnimatedListState> listKey = GlobalKey<AnimatedListState>();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedList.separated(
          key: listKey,
          initialItemCount: 2,
          itemBuilder: builder,
          separatorBuilder: separatorBuilder,
          removedSeparatorBuilder: separatorRemovalBuilder,
        ),
      ),
    );

    final Finder sliverAnimatedList = find.byType(SliverAnimatedList);
    expect(sliverAnimatedList, findsOneWidget);
    expect(
      (sliverAnimatedList.evaluate().first.widget as SliverAnimatedList).initialItemCount,
      3,
    ); // 2 items + 1 separator

    List<Text> itemsSeparatorsTexts;

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 3);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');

    await tester.pumpAndSettle();

    // Begin testing

    // insertItem - Insert at beginning of list
    listKey.currentState!.insertItem(0);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 5);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');

    await tester.pumpAndSettle();

    // insertItem - Insert at end of list
    listKey.currentState!.insertItem(3);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 7);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');

    await tester.pumpAndSettle();

    // insertItem - Insert in middle of list
    listKey.currentState!.insertItem(2);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 9);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');
    expect(itemsSeparatorsTexts[7].data, 'separator after item 3');
    expect(itemsSeparatorsTexts[8].data, 'item 4');

    await tester.pumpAndSettle();

    // insertItem - Insert at negative index
    expect(() => listKey.currentState!.insertItem(-1), throwsAssertionError);

    // insertItem - Insert at index greater than itemCount
    expect(() => listKey.currentState!.insertItem(42), throwsAssertionError);

    // removeItem - Remove at beginning of list
    listKey.currentState!.removeItem(
      0,
      itemRemovalBuilderWrapper(index: 0),
      duration: const Duration(milliseconds: 100),
    );
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 9);
    expect(itemsSeparatorsTexts[0].data, 'removing item 0');
    expect(itemsSeparatorsTexts[1].data, 'removing separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 0');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[4].data, 'item 1');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[6].data, 'item 2');
    expect(itemsSeparatorsTexts[7].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[8].data, 'item 3');

    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 7);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');

    // removeItem - Remove at end of list
    listKey.currentState!.removeItem(
      3,
      itemRemovalBuilderWrapper(index: 3),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 7);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'removing separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'removing item 3');

    await tester.pumpAndSettle();

    // removeItem - Remove in middle of list
    listKey.currentState!.removeItem(
      1,
      itemRemovalBuilderWrapper(index: 1),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 5);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'removing item 1');
    expect(itemsSeparatorsTexts[3].data, 'removing separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 1');

    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 3);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');

    // removeItem - Remove at negative index
    expect(
      () => listKey.currentState!.removeItem(
        -1,
        itemRemovalBuilderWrapper(index: -1),
        duration: const Duration(milliseconds: 100),
      ),
      throwsAssertionError,
    );

    // removeItem - Remove at index greater than itemCount
    expect(
      () => listKey.currentState!.removeItem(
        42,
        itemRemovalBuilderWrapper(index: 42),
        duration: const Duration(milliseconds: 100),
      ),
      throwsAssertionError,
    );

    // insertAllItems - Insert no items
    listKey.currentState!.insertAllItems(0, 0);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 3);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');

    await tester.pumpAndSettle();

    // insertAllItems - Insert negative number of items
    listKey.currentState!.insertAllItems(0, -1);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 3);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');

    await tester.pumpAndSettle();

    // insertAllItems - Insert at beginning of list
    listKey.currentState!.insertAllItems(0, 2);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 7);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');

    await tester.pumpAndSettle();

    // insertAllItems - Insert at end of list
    listKey.currentState!.insertAllItems(4, 2);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 11);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');
    expect(itemsSeparatorsTexts[7].data, 'separator after item 3');
    expect(itemsSeparatorsTexts[8].data, 'item 4');
    expect(itemsSeparatorsTexts[9].data, 'separator after item 4');
    expect(itemsSeparatorsTexts[10].data, 'item 5');

    await tester.pumpAndSettle();

    // insertAllItems - Insert in middle of list
    listKey.currentState!.insertAllItems(3, 2);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 15);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
    expect(itemsSeparatorsTexts[3].data, 'separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'item 2');
    expect(itemsSeparatorsTexts[5].data, 'separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'item 3');
    expect(itemsSeparatorsTexts[7].data, 'separator after item 3');
    expect(itemsSeparatorsTexts[8].data, 'item 4');
    expect(itemsSeparatorsTexts[9].data, 'separator after item 4');
    expect(itemsSeparatorsTexts[10].data, 'item 5');
    expect(itemsSeparatorsTexts[11].data, 'separator after item 5');
    expect(itemsSeparatorsTexts[12].data, 'item 6');
    expect(itemsSeparatorsTexts[13].data, 'separator after item 6');
    expect(itemsSeparatorsTexts[14].data, 'item 7');

    await tester.pumpAndSettle();

    // insertAllItems - Insert at negative index
    expect(() => listKey.currentState!.insertAllItems(-1, 2), throwsAssertionError);

    // insertAllItems - Insert at index greater than itemCount
    expect(() => listKey.currentState!.insertAllItems(42, 2), throwsAssertionError);

    // removeAllItems - Remove all items from list with multiple items
    listKey.currentState!.removeAllItems(
      itemRemovalBuilderWrapper(),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 15);
    expect(itemsSeparatorsTexts[0].data, 'removing item');
    expect(itemsSeparatorsTexts[1].data, 'removing separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'removing item');
    expect(itemsSeparatorsTexts[3].data, 'removing separator after item 1');
    expect(itemsSeparatorsTexts[4].data, 'removing item');
    expect(itemsSeparatorsTexts[5].data, 'removing separator after item 2');
    expect(itemsSeparatorsTexts[6].data, 'removing item');
    expect(itemsSeparatorsTexts[7].data, 'removing separator after item 3');
    expect(itemsSeparatorsTexts[8].data, 'removing item');
    expect(itemsSeparatorsTexts[9].data, 'removing separator after item 4');
    expect(itemsSeparatorsTexts[10].data, 'removing item');
    expect(itemsSeparatorsTexts[11].data, 'removing separator after item 5');
    expect(itemsSeparatorsTexts[12].data, 'removing item');
    expect(itemsSeparatorsTexts[13].data, 'removing separator after item 6');
    expect(itemsSeparatorsTexts[14].data, 'removing item');

    await tester.pumpAndSettle();

    // removeItem - Remove from empty list
    expect(
      () => listKey.currentState!.removeItem(
        0,
        itemRemovalBuilderWrapper(index: 0),
        duration: const Duration(milliseconds: 100),
      ),
      throwsAssertionError,
    );

    // removeItem - Remove item from list with single item
    // Prepare
    listKey.currentState!.insertItem(0);

    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 1);
    expect(itemsSeparatorsTexts[0].data, 'item 0');

    // Test
    listKey.currentState!.removeItem(
      0,
      itemRemovalBuilderWrapper(index: 0),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 1);
    expect(itemsSeparatorsTexts[0].data, 'removing item 0');

    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 0);

    // removeAllItems - Remove all items from empty list
    listKey.currentState!.removeAllItems(
      itemRemovalBuilderWrapper(),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 0);

    await tester.pumpAndSettle();

    // removeAllItems - Remove all items from list with single item
    // Prepare
    listKey.currentState!.insertItem(0);
    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 1);
    expect(itemsSeparatorsTexts[0].data, 'item 0');

    // Test
    listKey.currentState!.removeAllItems(
      itemRemovalBuilderWrapper(),
      duration: const Duration(milliseconds: 100),
    );

    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 1);
    expect(itemsSeparatorsTexts[0].data, 'removing item');

    await tester.pumpAndSettle();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 0);

    // insertAllItems - Insert into empty list
    listKey.currentState!.insertAllItems(0, 2);
    await tester.pump();

    itemsSeparatorsTexts = getItemsSeparatorsTexts(tester);

    expect(itemsSeparatorsTexts.length, 3);
    expect(itemsSeparatorsTexts[0].data, 'item 0');
    expect(itemsSeparatorsTexts[1].data, 'separator after item 0');
    expect(itemsSeparatorsTexts[2].data, 'item 1');
  });
}

class _StatefulListItem extends StatefulWidget {
  const _StatefulListItem({super.key, required this.index});

  final int index;

  @override
  _StatefulListItemState createState() => _StatefulListItemState();
}

class _StatefulListItemState extends State<_StatefulListItem> {
  late final int number = widget.index;

  @override
  Widget build(BuildContext context) {
    return Text('item $number');
  }
}
