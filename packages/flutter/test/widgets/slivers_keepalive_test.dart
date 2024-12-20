// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Sliver with keep alive without key - should dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      const WidgetTest0(text: 'child 0', keepAlive: true),
      const WidgetTest1(text: 'child 1', keepAlive: true),
      const WidgetTest2(text: 'child 2', keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver without keep alive without key - should dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      const WidgetTest0(text: 'child 0'),
      const WidgetTest1(text: 'child 1'),
      const WidgetTest2(text: 'child 2'),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver without keep alive with key - should dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: GlobalKey()),
      WidgetTest1(text: 'child 1', key: GlobalKey()),
      WidgetTest2(text: 'child 2', key: GlobalKey()),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver with keep alive with key - should not dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: GlobalKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: GlobalKey(), keepAlive: true),
      WidgetTest2(text: 'child 2', key: GlobalKey(), keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(state0.hasBeenDisposed, false);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver with keep alive with Unique key - should not dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: UniqueKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: UniqueKey(), keepAlive: true),
      WidgetTest2(text: 'child 2', key: UniqueKey(), keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(state0.hasBeenDisposed, false);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver with keep alive with Value key - should not dispose after reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      const WidgetTest0(text: 'child 0', key: ValueKey<int>(0), keepAlive: true),
      const WidgetTest1(text: 'child 1', key: ValueKey<int>(1), keepAlive: true),
      const WidgetTest2(text: 'child 2', key: ValueKey<int>(2), keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(state0.hasBeenDisposed, false);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('Sliver complex case 1', (WidgetTester tester) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: GlobalKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: GlobalKey(), keepAlive: true),
      const WidgetTest2(text: 'child 2', keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, false);
    // Child 2 does not have a key.
    expect(state2.hasBeenDisposed, true);
  });

  testWidgets('Sliver complex case 2', (WidgetTester tester) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: GlobalKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: UniqueKey()),
      const WidgetTest2(text: 'child 2', keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(children: childList));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, true);
  });

  testWidgets('Sliver with SliverChildBuilderDelegate', (WidgetTester tester) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: UniqueKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: GlobalKey()),
      const WidgetTest2(text: 'child 2', keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildBuilderTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildBuilderTest(children: childList));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 2'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildBuilderTest(children: childList));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingChildBuilderTest(children: childList));
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 0', skipOffstage: false), findsOneWidget);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildBuilderTest(children: childList));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1', skipOffstage: false), findsNothing);
    expect(find.text('child 2', skipOffstage: false), findsNothing);

    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, true);
  });

  testWidgets('SliverFillViewport should not dispose widget with key during in screen reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: UniqueKey(), keepAlive: true),
      WidgetTest1(text: 'child 1', key: UniqueKey()),
      const WidgetTest2(text: 'child 2', keepAlive: true),
    ];
    await tester.pumpWidget(SwitchingChildListTest(viewportFraction: 0.1, children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2'), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingChildListTest(viewportFraction: 0.1, children: childList));

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(viewportFraction: 0.1, children: childList));

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingChildListTest(viewportFraction: 0.1, children: childList));

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingChildListTest(viewportFraction: 0.1, children: childList));

    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, false);
    expect(state2.hasBeenDisposed, true);
  });

  testWidgets('SliverList should not dispose widget with key during in screen reordering', (
    WidgetTester tester,
  ) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: UniqueKey(), keepAlive: true),
      const WidgetTest1(text: 'child 1', keepAlive: true),
      WidgetTest2(text: 'child 2', key: UniqueKey()),
    ];
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2'), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));

    childList = createSwitchedChildList(childList, 1, 2);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));

    childList = createSwitchedChildList(childList, 0, 2);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));

    childList = createSwitchedChildList(childList, 0, 1);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));
    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, false);
  });

  testWidgets('SliverList remove child from child list', (WidgetTester tester) async {
    List<Widget> childList = <Widget>[
      WidgetTest0(text: 'child 0', key: UniqueKey(), keepAlive: true),
      const WidgetTest1(text: 'child 1', keepAlive: true),
      WidgetTest2(text: 'child 2', key: UniqueKey()),
    ];
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));
    final _WidgetTest0State state0 = tester.state(find.byType(WidgetTest0));
    final _WidgetTest1State state1 = tester.state(find.byType(WidgetTest1));
    final _WidgetTest2State state2 = tester.state(find.byType(WidgetTest2));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2'), findsOneWidget);

    childList = createSwitchedChildList(childList, 0, 1);
    childList.removeAt(2);
    await tester.pumpWidget(SwitchingSliverListTest(children: childList));
    expect(find.text('child 0'), findsOneWidget);
    expect(find.text('child 1'), findsOneWidget);
    expect(find.text('child 2'), findsNothing);
    expect(state0.hasBeenDisposed, false);
    expect(state1.hasBeenDisposed, true);
    expect(state2.hasBeenDisposed, true);
  });
}

List<Widget> createSwitchedChildList(List<Widget> childList, int i, int j) {
  final Widget w = childList[i];
  childList[i] = childList[j];
  childList[j] = w;
  return List<Widget>.from(childList);
}

class SwitchingChildBuilderTest extends StatefulWidget {
  const SwitchingChildBuilderTest({required this.children, super.key});

  final List<Widget> children;

  @override
  State<SwitchingChildBuilderTest> createState() => _SwitchingChildBuilderTest();
}

class _SwitchingChildBuilderTest extends State<SwitchingChildBuilderTest> {
  late List<Widget> children;
  late Map<Key, int> _mapKeyToIndex;

  @override
  void initState() {
    super.initState();
    children = widget.children;
    _mapKeyToIndex = <Key, int>{};
    for (int index = 0; index < children.length; index += 1) {
      final Key? key = children[index].key;
      if (key != null) {
        _mapKeyToIndex[key] = index;
      }
    }
  }

  @override
  void didUpdateWidget(SwitchingChildBuilderTest oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.children != widget.children) {
      children = widget.children;
      _mapKeyToIndex = <Key, int>{};
      for (int index = 0; index < children.length; index += 1) {
        final Key? key = children[index].key;
        if (key != null) {
          _mapKeyToIndex[key] = index;
        }
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: SizedBox(
          height: 100,
          child: CustomScrollView(
            cacheExtent: 0,
            slivers: <Widget>[
              SliverFillViewport(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) {
                    return children[index];
                  },
                  childCount: children.length,
                  findChildIndexCallback: (Key key) => _mapKeyToIndex[key] ?? -1,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class SwitchingChildListTest extends StatelessWidget {
  const SwitchingChildListTest({required this.children, this.viewportFraction = 1.0, super.key});

  final List<Widget> children;
  final double viewportFraction;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: SizedBox(
          height: 100,
          child: CustomScrollView(
            cacheExtent: 0,
            slivers: <Widget>[
              SliverFillViewport(
                viewportFraction: viewportFraction,
                delegate: SliverChildListDelegate(children),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class SwitchingSliverListTest extends StatelessWidget {
  const SwitchingSliverListTest({required this.children, super.key});

  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: SizedBox(
          height: 100,
          child: CustomScrollView(
            cacheExtent: 0,
            slivers: <Widget>[SliverList(delegate: SliverChildListDelegate(children))],
          ),
        ),
      ),
    );
  }
}

class WidgetTest0 extends StatefulWidget {
  const WidgetTest0({required this.text, this.keepAlive = false, super.key});

  final String text;
  final bool keepAlive;

  @override
  State<WidgetTest0> createState() => _WidgetTest0State();
}

class _WidgetTest0State extends State<WidgetTest0> with AutomaticKeepAliveClientMixin {
  bool hasBeenDisposed = false;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Text(widget.text);
  }

  @override
  void dispose() {
    hasBeenDisposed = true;
    super.dispose();
  }

  @override
  bool get wantKeepAlive => widget.keepAlive;
}

class WidgetTest1 extends StatefulWidget {
  const WidgetTest1({required this.text, this.keepAlive = false, super.key});

  final String text;
  final bool keepAlive;

  @override
  State<WidgetTest1> createState() => _WidgetTest1State();
}

class _WidgetTest1State extends State<WidgetTest1> with AutomaticKeepAliveClientMixin {
  bool hasBeenDisposed = false;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Text(widget.text);
  }

  @override
  void dispose() {
    hasBeenDisposed = true;
    super.dispose();
  }

  @override
  bool get wantKeepAlive => widget.keepAlive;
}

class WidgetTest2 extends StatefulWidget {
  const WidgetTest2({required this.text, this.keepAlive = false, super.key});

  final String text;
  final bool keepAlive;

  @override
  State<WidgetTest2> createState() => _WidgetTest2State();
}

class _WidgetTest2State extends State<WidgetTest2> with AutomaticKeepAliveClientMixin {
  bool hasBeenDisposed = false;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Text(widget.text);
  }

  @override
  void dispose() {
    hasBeenDisposed = true;
    super.dispose();
  }

  @override
  bool get wantKeepAlive => widget.keepAlive;
}
