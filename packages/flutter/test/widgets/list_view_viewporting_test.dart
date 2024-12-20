// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_widgets.dart';

void main() {
  testWidgets('ListView mount/dismount smoke test', (WidgetTester tester) async {
    final List<int> callbackTracker = <int>[];

    // the root view is 800x600 in the test environment
    // so if our widget is 100 pixels tall, it should fit exactly 6 times.

    Widget builder() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: FlipWidget(
          left: ListView.builder(
            itemBuilder: (BuildContext context, int index) {
              callbackTracker.add(index);
              return SizedBox(key: ValueKey<int>(index), height: 100.0, child: Text('$index'));
            },
          ),
          right: const Text('Not Today'),
        ),
      );
    }

    await tester.pumpWidget(builder());

    final FlipWidgetState testWidget = tester.state(find.byType(FlipWidget));

    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2, 3, 4, 5, // visible
        6, 7, 8, // in cached area
      ]),
    );

    callbackTracker.clear();
    testWidget.flip();
    await tester.pump();

    expect(callbackTracker, equals(<int>[]));

    callbackTracker.clear();
    testWidget.flip();
    await tester.pump();

    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2, 3, 4, 5, // visible
        6, 7, 8, // in cached area
      ]),
    );
  });

  testWidgets('ListView vertical', (WidgetTester tester) async {
    final List<int> callbackTracker = <int>[];

    // the root view is 800x600 in the test environment
    // so if our widget is 200 pixels tall, it should fit exactly 3 times.
    // but if we are offset by 300 pixels, there will be 4, numbered 1-4.

    Widget itemBuilder(BuildContext context, int index) {
      callbackTracker.add(index);
      return SizedBox(
        key: ValueKey<int>(index),
        width: 500.0, // this should be ignored
        height: 200.0,
        child: Text('$index', textDirection: TextDirection.ltr),
      );
    }

    Widget builder() {
      final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
      addTearDown(controller.dispose);

      return Directionality(
        textDirection: TextDirection.ltr,
        child: FlipWidget(
          left: ListView.builder(controller: controller, itemBuilder: itemBuilder),
          right: const Text('Not Today'),
        ),
      );
    }

    await tester.pumpWidget(builder());

    // 0 is built to find its height
    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2, 3, 4,
        5, // in cached area
      ]),
    );
    callbackTracker.clear();

    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    scrollable.position.jumpTo(600.0); // now only 3 should fit, numbered 3-5.

    await tester.pumpWidget(builder());

    // We build the visible children to find their new size.
    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2,
        3, 4, 5, //visible
        6, 7,
      ]),
    );
    callbackTracker.clear();

    await tester.pumpWidget(builder());

    // 0 isn't built because they're not visible.
    expect(
      callbackTracker,
      equals(<int>[
        1, 2,
        3, 4, 5, // visible
        6, 7,
      ]),
    );
    callbackTracker.clear();
  });

  testWidgets('ListView horizontal', (WidgetTester tester) async {
    final List<int> callbackTracker = <int>[];

    // the root view is 800x600 in the test environment
    // so if our widget is 200 pixels wide, it should fit exactly 4 times.
    // but if we are offset by 300 pixels, there will be 5, numbered 1-5.

    Widget itemBuilder(BuildContext context, int index) {
      callbackTracker.add(index);
      return SizedBox(
        key: ValueKey<int>(index),
        height: 500.0, // this should be ignored
        width: 200.0,
        child: Text('$index', textDirection: TextDirection.ltr),
      );
    }

    Widget builder() {
      final ScrollController controller = ScrollController(initialScrollOffset: 500.0);
      addTearDown(controller.dispose);

      return Directionality(
        textDirection: TextDirection.ltr,
        child: FlipWidget(
          left: ListView.builder(
            scrollDirection: Axis.horizontal,
            controller: controller,
            itemBuilder: itemBuilder,
          ),
          right: const Text('Not Today'),
        ),
      );
    }

    await tester.pumpWidget(builder());

    // 0 is built to find its width
    expect(callbackTracker, equals(<int>[0, 1, 2, 3, 4, 5, 6, 7]));

    callbackTracker.clear();

    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    scrollable.position.jumpTo(600.0); // now only 4 should fit, numbered 2-5.

    await tester.pumpWidget(builder());

    // We build the visible children to find their new size.
    expect(callbackTracker, equals(<int>[1, 2, 3, 4, 5, 6, 7, 8]));
    callbackTracker.clear();

    await tester.pumpWidget(builder());

    // 0 isn't built because they're not visible.
    expect(callbackTracker, equals(<int>[1, 2, 3, 4, 5, 6, 7, 8]));
    callbackTracker.clear();
  });

  testWidgets('ListView reinvoke builders', (WidgetTester tester) async {
    final List<int> callbackTracker = <int>[];
    final List<String?> text = <String?>[];

    Widget itemBuilder(BuildContext context, int index) {
      callbackTracker.add(index);
      return SizedBox(
        key: ValueKey<int>(index),
        width: 500.0, // this should be ignored
        height: 220.0,
        child: Text('$index', textDirection: TextDirection.ltr),
      );
    }

    void collectText(Widget widget) {
      if (widget is Text) {
        text.add(widget.data);
      }
    }

    Widget builder() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(itemBuilder: itemBuilder),
      );
    }

    await tester.pumpWidget(builder());

    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2,
        3, // in cached area
      ]),
    );
    callbackTracker.clear();
    tester.allWidgets.forEach(collectText);
    expect(text, equals(<String>['0', '1', '2', '3']));
    text.clear();

    await tester.pumpWidget(builder());

    expect(
      callbackTracker,
      equals(<int>[
        0, 1, 2,
        3, // in cached area
      ]),
    );
    callbackTracker.clear();
    tester.allWidgets.forEach(collectText);
    expect(text, equals(<String>['0', '1', '2', '3']));
    text.clear();
  });

  testWidgets('ListView reinvoke builders', (WidgetTester tester) async {
    late StateSetter setState;
    ThemeData themeData = ThemeData.light(useMaterial3: false);

    Widget itemBuilder(BuildContext context, int index) {
      return Container(
        key: ValueKey<int>(index),
        width: 500.0, // this should be ignored
        height: 220.0,
        color: Theme.of(context).primaryColor,
        child: Text('$index', textDirection: TextDirection.ltr),
      );
    }

    final Widget viewport = ListView.builder(itemBuilder: itemBuilder);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            setState = setter;
            return Theme(data: themeData, child: viewport);
          },
        ),
      ),
    );

    Container widget = tester.firstWidget(find.byType(Container));
    expect(widget.color, equals(Colors.blue));

    setState(() {
      themeData = ThemeData(primarySwatch: Colors.green, useMaterial3: false);
    });

    await tester.pump();

    widget = tester.firstWidget(find.byType(Container));
    expect(widget.color, equals(Colors.green));
  });

  testWidgets('ListView padding', (WidgetTester tester) async {
    Widget itemBuilder(BuildContext context, int index) {
      return Container(
        key: ValueKey<int>(index),
        width: 500.0, // this should be ignored
        height: 220.0,
        color: Colors.green[500],
        child: Text('$index', textDirection: TextDirection.ltr),
      );
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          padding: const EdgeInsets.fromLTRB(7.0, 3.0, 5.0, 11.0),
          itemBuilder: itemBuilder,
        ),
      ),
    );

    final RenderBox firstBox = tester.renderObject(find.text('0'));
    final Offset upperLeft = firstBox.localToGlobal(Offset.zero);
    expect(upperLeft, equals(const Offset(7.0, 3.0)));
    expect(firstBox.size.width, equals(800.0 - 12.0));
  });

  testWidgets('ListView underflow extents', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addSemanticIndexes: false,
          children: <Widget>[
            Container(height: 100.0),
            Container(height: 100.0),
            Container(height: 100.0),
          ],
        ),
      ),
    );

    final RenderSliverList list = tester.renderObject(find.byType(SliverList));

    expect(list.indexOf(list.firstChild!), equals(0));
    expect(list.indexOf(list.lastChild!), equals(2));
    expect(list.childScrollOffset(list.firstChild!), equals(0.0));
    expect(list.geometry!.scrollExtent, equals(300.0));

    expect(list, hasAGoodToStringDeep);
    expect(
      list.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderSliverList#00000 relayoutBoundary=up2\n'
        ' │ needs compositing\n'
        ' │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        ' │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │   0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0,\n'
        ' │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │   remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        ' │ geometry: SliverGeometry(scrollExtent: 300.0, paintExtent: 300.0,\n'
        ' │   maxPaintExtent: 300.0, cacheExtent: 300.0)\n'
        ' │ currently live children: 0 to 2\n'
        ' │\n'
        ' ├─child with index 0: RenderRepaintBoundary#00000 relayoutBoundary=up3\n'
        ' │ │ needs compositing\n'
        ' │ │ parentData: index=0; layoutOffset=0.0 (can use size)\n'
        ' │ │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │ │ layer: OffsetLayer#00000\n'
        ' │ │ size: Size(800.0, 100.0)\n'
        ' │ │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        ' │ │ diagnosis: insufficient data to draw conclusion (less than five\n'
        ' │ │   repaints)\n'
        ' │ │\n'
        ' │ └─child: RenderConstrainedBox#00000 relayoutBoundary=up4\n'
        ' │   │ parentData: <none> (can use size)\n'
        ' │   │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │   │ size: Size(800.0, 100.0)\n'
        ' │   │ additionalConstraints: BoxConstraints(0.0<=w<=Infinity, h=100.0)\n'
        ' │   │\n'
        ' │   └─child: RenderLimitedBox#00000\n'
        ' │     │ parentData: <none> (can use size)\n'
        ' │     │ constraints: BoxConstraints(w=800.0, h=100.0)\n'
        ' │     │ size: Size(800.0, 100.0)\n'
        ' │     │ maxWidth: 0.0\n'
        ' │     │ maxHeight: 0.0\n'
        ' │     │\n'
        ' │     └─child: RenderConstrainedBox#00000\n'
        ' │         parentData: <none> (can use size)\n'
        ' │         constraints: BoxConstraints(w=800.0, h=100.0)\n'
        ' │         size: Size(800.0, 100.0)\n'
        ' │         additionalConstraints: BoxConstraints(biggest)\n'
        ' │\n'
        ' ├─child with index 1: RenderRepaintBoundary#00000 relayoutBoundary=up3\n'
        ' │ │ needs compositing\n'
        ' │ │ parentData: index=1; layoutOffset=100.0 (can use size)\n'
        ' │ │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │ │ layer: OffsetLayer#00000\n'
        ' │ │ size: Size(800.0, 100.0)\n'
        ' │ │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        ' │ │ diagnosis: insufficient data to draw conclusion (less than five\n'
        ' │ │   repaints)\n'
        ' │ │\n'
        ' │ └─child: RenderConstrainedBox#00000 relayoutBoundary=up4\n'
        ' │   │ parentData: <none> (can use size)\n'
        ' │   │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │   │ size: Size(800.0, 100.0)\n'
        ' │   │ additionalConstraints: BoxConstraints(0.0<=w<=Infinity, h=100.0)\n'
        ' │   │\n'
        ' │   └─child: RenderLimitedBox#00000\n'
        ' │     │ parentData: <none> (can use size)\n'
        ' │     │ constraints: BoxConstraints(w=800.0, h=100.0)\n'
        ' │     │ size: Size(800.0, 100.0)\n'
        ' │     │ maxWidth: 0.0\n'
        ' │     │ maxHeight: 0.0\n'
        ' │     │\n'
        ' │     └─child: RenderConstrainedBox#00000\n'
        ' │         parentData: <none> (can use size)\n'
        ' │         constraints: BoxConstraints(w=800.0, h=100.0)\n'
        ' │         size: Size(800.0, 100.0)\n'
        ' │         additionalConstraints: BoxConstraints(biggest)\n'
        ' │\n'
        ' └─child with index 2: RenderRepaintBoundary#00000 relayoutBoundary=up3\n'
        '   │ needs compositing\n'
        '   │ parentData: index=2; layoutOffset=200.0 (can use size)\n'
        '   │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        '   │ layer: OffsetLayer#00000\n'
        '   │ size: Size(800.0, 100.0)\n'
        '   │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        '   │ diagnosis: insufficient data to draw conclusion (less than five\n'
        '   │   repaints)\n'
        '   │\n'
        '   └─child: RenderConstrainedBox#00000 relayoutBoundary=up4\n'
        '     │ parentData: <none> (can use size)\n'
        '     │ constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        '     │ size: Size(800.0, 100.0)\n'
        '     │ additionalConstraints: BoxConstraints(0.0<=w<=Infinity, h=100.0)\n'
        '     │\n'
        '     └─child: RenderLimitedBox#00000\n'
        '       │ parentData: <none> (can use size)\n'
        '       │ constraints: BoxConstraints(w=800.0, h=100.0)\n'
        '       │ size: Size(800.0, 100.0)\n'
        '       │ maxWidth: 0.0\n'
        '       │ maxHeight: 0.0\n'
        '       │\n'
        '       └─child: RenderConstrainedBox#00000\n'
        '           parentData: <none> (can use size)\n'
        '           constraints: BoxConstraints(w=800.0, h=100.0)\n'
        '           size: Size(800.0, 100.0)\n'
        '           additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    expect(position.viewportDimension, equals(600.0));
    expect(position.minScrollExtent, equals(0.0));
  });

  testWidgets('ListView should not paint hidden children', (WidgetTester tester) async {
    const Text text = Text('test');
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            child: ListView(
              cacheExtent: 500.0,
              controller: controller,
              children: const <Widget>[
                SizedBox(height: 140.0, child: text),
                SizedBox(height: 160.0, child: text),
                SizedBox(height: 90.0, child: text),
                SizedBox(height: 110.0, child: text),
                SizedBox(height: 80.0, child: text),
                SizedBox(height: 70.0, child: text),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderSliverList list = tester.renderObject(find.byType(SliverList));
    expect(list, paintsExactlyCountTimes(#drawParagraph, 2));
  });

  testWidgets('ListView should paint with offset', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 120.0);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SizedBox(
            height: 500.0,
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                const SliverAppBar(expandedHeight: 250.0),
                SliverList(
                  delegate:
                      ListView.builder(
                        itemExtent: 100.0,
                        itemCount: 100,
                        itemBuilder: (_, __) => const SizedBox(height: 40.0, child: Text('hey')),
                      ).childrenDelegate,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderObject renderObject = tester.renderObject(find.byType(Scrollable));
    expect(renderObject, paintsExactlyCountTimes(#drawParagraph, 10));
  });

  testWidgets('ListView should paint with rtl', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: SizedBox(
          height: 200.0,
          child: ListView.builder(
            padding: EdgeInsets.zero,
            scrollDirection: Axis.horizontal,
            itemExtent: 200.0,
            itemCount: 10,
            itemBuilder:
                (_, int i) => Container(
                  height: 200.0,
                  width: 200.0,
                  color: i.isEven ? Colors.black : Colors.red,
                ),
          ),
        ),
      ),
    );

    final RenderObject renderObject = tester.renderObject(find.byType(Scrollable));
    expect(renderObject, paintsExactlyCountTimes(#drawRect, 4));
  });
}
