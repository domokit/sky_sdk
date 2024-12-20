// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/src/foundation/diagnostics.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/100451
  testWidgets('SliverAnimatedGrid.builder respects findChildIndexCallback', (
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
                SliverAnimatedGrid(
                  initialItemCount: itemCount,
                  itemBuilder:
                      (BuildContext context, int index, Animation<double> animation) =>
                          Container(key: Key('$index'), height: 2000.0),
                  gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                    maxCrossAxisExtent: 100.0,
                    mainAxisSpacing: 10.0,
                    crossAxisSpacing: 10.0,
                  ),
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

  testWidgets('AnimatedGrid', (WidgetTester tester) async {
    Widget builder(BuildContext context, int index, Animation<double> animation) {
      return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
    }

    final GlobalKey<AnimatedGridState> listKey = GlobalKey<AnimatedGridState>();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedGrid(
          key: listKey,
          initialItemCount: 2,
          itemBuilder: builder,
          gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
            maxCrossAxisExtent: 100.0,
            mainAxisSpacing: 10.0,
            crossAxisSpacing: 10.0,
          ),
        ),
      ),
    );

    expect(
      find.byWidgetPredicate((Widget widget) {
        return widget is SliverAnimatedGrid &&
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

    listKey.currentState!.insertAllItems(0, 2);
    await tester.pump();
    expect(find.text('item 2'), findsOneWidget);
    expect(find.text('item 3'), findsOneWidget);

    // Test for removeAllItems.
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

  group('SliverAnimatedGrid', () {
    testWidgets('initialItemCount', (WidgetTester tester) async {
      final Map<int, Animation<double>> animations = <int, Animation<double>>{};

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedGrid(
                initialItemCount: 2,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  animations[index] = animation;
                  return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                  mainAxisSpacing: 10.0,
                  crossAxisSpacing: 10.0,
                ),
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
      final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedGrid(
                key: listKey,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return ScaleTransition(
                    key: ValueKey<int>(index),
                    scale: animation,
                    child: SizedBox(height: 100.0, child: Center(child: Text('item $index'))),
                  );
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                ),
              ),
            ],
          ),
        ),
      );

      double itemScale(int index) =>
          tester
              .widget<ScaleTransition>(find.byKey(ValueKey<int>(index), skipOffstage: false))
              .scale
              .value;
      double itemLeft(int index) =>
          tester.getTopLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;
      double itemRight(int index) =>
          tester.getTopRight(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;

      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // Newly inserted item 0's scale should animate from 0 to 1
      expect(itemScale(0), 0.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 0.5);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 1.0);

      // The list now contains one fully expanded item at the top:
      expect(find.text('item 0'), findsOneWidget);
      expect(itemLeft(0), 0.0);
      expect(itemRight(0), 100.0);

      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      listKey.currentState!.insertItem(0, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // The scale of the newly inserted items at index 0 and 1 should animate
      // from 0 to 1.
      // The scale of the original item, now at index 2, should remain 1.
      expect(itemScale(0), 0.0);
      expect(itemScale(1), 0.0);
      expect(itemScale(2), 1.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 0.5);
      expect(itemScale(1), 0.5);
      expect(itemScale(2), 1.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 1.0);
      expect(itemScale(1), 1.0);
      expect(itemScale(2), 1.0);

      // The newly inserted "item 1" and "item 2" appear above "item 0"
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(find.text('item 2'), findsOneWidget);
      expect(itemLeft(0), 0.0);
      expect(itemRight(0), 100.0);
      expect(itemLeft(1), 100.0);
      expect(itemRight(1), 200.0);
      expect(itemLeft(2), 200.0);
      expect(itemRight(2), 300.0);
    });

    testWidgets('insertAll', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAnimatedGrid(
                key: listKey,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return ScaleTransition(
                    key: ValueKey<int>(index),
                    scale: animation,
                    child: SizedBox(height: 100.0, child: Center(child: Text('item $index'))),
                  );
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                ),
              ),
            ],
          ),
        ),
      );

      double itemScale(int index) =>
          tester
              .widget<ScaleTransition>(find.byKey(ValueKey<int>(index), skipOffstage: false))
              .scale
              .value;
      double itemLeft(int index) =>
          tester.getTopLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;
      double itemRight(int index) =>
          tester.getTopRight(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;

      listKey.currentState!.insertAllItems(0, 2, duration: const Duration(milliseconds: 100));
      await tester.pump();

      // Newly inserted items 0 & 1's scale should animate from 0 to 1
      expect(itemScale(0), 0.0);
      expect(itemScale(1), 0.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 0.5);
      expect(itemScale(1), 0.5);
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 1.0);
      expect(itemScale(1), 1.0);

      // The list now contains two fully expanded items at the top:
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item 1'), findsOneWidget);
      expect(itemLeft(0), 0.0);
      expect(itemRight(0), 100.0);
      expect(itemLeft(1), 100.0);
      expect(itemRight(1), 200.0);
    });

    testWidgets('remove', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();
      final List<int> items = <int>[0, 1, 2];

      Widget buildItem(BuildContext context, int item, Animation<double> animation) {
        return ScaleTransition(
          key: ValueKey<int>(item),
          scale: animation,
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
              SliverAnimatedGrid(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return buildItem(context, items[index], animation);
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                ),
              ),
            ],
          ),
        ),
      );

      double itemScale(int index) =>
          tester
              .widget<ScaleTransition>(find.byKey(ValueKey<int>(index), skipOffstage: false))
              .scale
              .value;
      double itemLeft(int index) =>
          tester.getTopLeft(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;
      double itemRight(int index) =>
          tester.getTopRight(find.byKey(ValueKey<int>(index), skipOffstage: false)).dx;

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
      expect(itemLeft(0), 0.0);
      expect(itemRight(0), 100.0);
      expect(itemLeft(1), 100.0);
      expect(itemRight(1), 200.0);
      expect(itemLeft(2), 200.0);
      expect(itemRight(2), 300.0);

      // Newly removed item 0's height should animate from 100 to 0 over 100ms

      // Items 0, 1, 2 at 0, 50, 150. Item 0's height is 50.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(itemScale(0), 0.5);
      expect(itemScale(1), 1.0);
      expect(itemScale(2), 1.0);

      // Items 1, 2 at 0, 100.
      await tester.pumpAndSettle();
      expect(itemLeft(1), 0.0);
      expect(itemRight(1), 100.0);
      expect(itemLeft(2), 100.0);
      expect(itemRight(2), 200.0);
    });

    testWidgets('removeAll', (WidgetTester tester) async {
      final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();
      final List<int> items = <int>[0, 1, 2];

      Widget buildItem(BuildContext context, int item, Animation<double> animation) {
        return ScaleTransition(
          key: ValueKey<int>(item),
          scale: animation,
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
              SliverAnimatedGrid(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return buildItem(context, items[index], animation);
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                ),
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
      final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();

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
              SliverAnimatedGrid(
                key: listKey,
                initialItemCount: 3,
                itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                  return SizedBox(height: 100, child: Text('item $index'));
                },
                gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                  maxCrossAxisExtent: 100.0,
                ),
              ),
            ],
          ),
        ),
      );

      expect(tester.getTopLeft(find.text('item 0')).dx, 0);
      expect(tester.getTopLeft(find.text('item 1')).dx, 100);

      listKey.currentState!.insertItem(3);
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('item 3')).dx, 300);

      listKey.currentState!.removeItem(0, (BuildContext context, Animation<double> animation) {
        return ScaleTransition(
          scale: animation,
          key: const ObjectKey('removing'),
          child: const SizedBox(height: 100, child: Text('removing')),
        );
      }, duration: const Duration(seconds: 1));

      await tester.pump();
      expect(find.text('item 3'), findsNothing);

      await tester.pump(const Duration(milliseconds: 500));
      expect(
        tester
            .widget<ScaleTransition>(find.byKey(const ObjectKey('removing'), skipOffstage: false))
            .scale
            .value,
        0.5,
      );
      expect(tester.getTopLeft(find.text('item 0')).dx, 100);

      await tester.pumpAndSettle();
      expect(find.text('removing'), findsNothing);
      expect(tester.getTopLeft(find.text('item 0')).dx, 0);
    });

    testWidgets(
      'passes correctly derived index of findChildIndexCallback to the inner SliverChildBuilderDelegate',
      (WidgetTester tester) async {
        final List<int> items = <int>[0, 1, 2, 3];
        final GlobalKey<SliverAnimatedGridState> listKey = GlobalKey<SliverAnimatedGridState>();

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CustomScrollView(
              slivers: <Widget>[
                SliverAnimatedGrid(
                  key: listKey,
                  initialItemCount: items.length,
                  itemBuilder: (BuildContext context, int index, Animation<double> animation) {
                    return _StatefulListItem(key: ValueKey<int>(items[index]), index: index);
                  },
                  gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                    maxCrossAxisExtent: 100.0,
                    mainAxisSpacing: 10.0,
                    crossAxisSpacing: 10.0,
                  ),
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
    'AnimatedGrid.of() and maybeOf called with a context that does not contain AnimatedGrid',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(Container(key: key));
      late FlutterError error;
      expect(AnimatedGrid.maybeOf(key.currentContext!), isNull);
      try {
        AnimatedGrid.of(key.currentContext!);
      } on FlutterError catch (e) {
        error = e;
      }
      expect(error.diagnostics.length, 4);
      expect(error.diagnostics[2].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[2].toStringDeep(),
        equalsIgnoringHashCodes(
          'This can happen when the context provided is from the same\n'
          'StatefulWidget that built the AnimatedGrid. Please see the\n'
          'AnimatedGrid documentation for examples of how to refer to an\n'
          'AnimatedGridState object:\n'
          '  https://api.flutter.dev/flutter/widgets/AnimatedGridState-class.html\n',
        ),
      );
      expect(error.diagnostics[3], isA<DiagnosticsProperty<Element>>());
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   AnimatedGrid.of() called with a context that does not contain an\n'
          '   AnimatedGrid.\n'
          '   No AnimatedGrid ancestor could be found starting from the context\n'
          '   that was passed to AnimatedGrid.of().\n'
          '   This can happen when the context provided is from the same\n'
          '   StatefulWidget that built the AnimatedGrid. Please see the\n'
          '   AnimatedGrid documentation for examples of how to refer to an\n'
          '   AnimatedGridState object:\n'
          '     https://api.flutter.dev/flutter/widgets/AnimatedGridState-class.html\n'
          '   The context used was:\n'
          '     Container-[GlobalKey#32cc6]\n',
        ),
      );
    },
  );

  testWidgets('AnimatedGrid.clipBehavior is forwarded to its inner CustomScrollView', (
    WidgetTester tester,
  ) async {
    const Clip clipBehavior = Clip.none;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedGrid(
          initialItemCount: 2,
          clipBehavior: clipBehavior,
          itemBuilder: (BuildContext context, int index, Animation<double> _) {
            return SizedBox(height: 100.0, child: Center(child: Text('item $index')));
          },
          gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
            maxCrossAxisExtent: 100.0,
            mainAxisSpacing: 10.0,
            crossAxisSpacing: 10.0,
          ),
        ),
      ),
    );

    expect(
      tester.widget<CustomScrollView>(find.byType(CustomScrollView)).clipBehavior,
      clipBehavior,
    );
  });

  testWidgets('AnimatedGrid applies MediaQuery padding', (WidgetTester tester) async {
    const EdgeInsets padding = EdgeInsets.all(30.0);
    EdgeInsets? innerMediaQueryPadding;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.all(30.0)),
          child: AnimatedGrid(
            initialItemCount: 6,
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
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
    await tester.drag(find.byType(AnimatedGrid), const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();

    final Offset bottomRight = tester.getBottomRight(find.byType(Placeholder).last);
    // Automatically apply the bottom padding into sliver.
    expect(bottomRight, Offset(800.0, 600.0 - padding.bottom));

    // Verify that the left/right padding is not applied.
    expect(innerMediaQueryPadding, const EdgeInsets.symmetric(horizontal: 30.0));
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
