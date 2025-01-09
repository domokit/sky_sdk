// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class Leaf extends StatefulWidget {
  const Leaf({required Key super.key, required this.child});
  final Widget child;
  @override
  State<Leaf> createState() => _LeafState();
}

class _LeafState extends State<Leaf> {
  bool _keepAlive = false;
  KeepAliveHandle? _handle;

  @override
  void deactivate() {
    _handle?.dispose();
    _handle = null;
    super.deactivate();
  }

  void setKeepAlive(bool value) {
    _keepAlive = value;
    if (_keepAlive) {
      if (_handle == null) {
        _handle = KeepAliveHandle();
        KeepAliveNotification(_handle!).dispatch(context);
      }
    } else {
      _handle?.dispose();
      _handle = null;
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_keepAlive && _handle == null) {
      _handle = KeepAliveHandle();
      KeepAliveNotification(_handle!).dispatch(context);
    }
    return widget.child;
  }
}

List<Widget> generateList(Widget child, {required bool impliedMode}) {
  return List<Widget>.generate(100, (int index) {
    final Widget result = Leaf(key: GlobalObjectKey<_LeafState>(index), child: child);
    if (impliedMode) {
      return result;
    }
    return AutomaticKeepAlive(child: result);
  }, growable: false);
}

void tests({required bool impliedMode}) {
  testWidgets('AutomaticKeepAlive with ListView with itemExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: impliedMode,
          addRepaintBoundaries: impliedMode,
          addSemanticIndexes: false,
          itemExtent: 12.3, // about 50 widgets visible
          cacheExtent: 0.0,
          children: generateList(const Placeholder(), impliedMode: impliedMode),
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });

  testWidgets('AutomaticKeepAlive with ListView without itemExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: impliedMode,
          addRepaintBoundaries: impliedMode,
          addSemanticIndexes: false,
          cacheExtent: 0.0,
          children: generateList(
            const SizedBox(height: 12.3, child: Placeholder()), // about 50 widgets visible
            impliedMode: impliedMode,
          ),
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });

  testWidgets('AutomaticKeepAlive with GridView', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          addAutomaticKeepAlives: impliedMode,
          addRepaintBoundaries: impliedMode,
          addSemanticIndexes: false,
          crossAxisCount: 2,
          childAspectRatio: 400.0 / 24.6, // about 50 widgets visible
          cacheExtent: 0.0,
          children: generateList(const Placeholder(), impliedMode: impliedMode),
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(GridView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(GridView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });
}

void main() {
  group('Explicit automatic keep-alive', () {
    tests(impliedMode: false);
  });
  group('Implied automatic keep-alive', () {
    tests(impliedMode: true);
  });

  testWidgets('AutomaticKeepAlive double', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          cacheExtent: 0.0,
          children: const <Widget>[
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(0), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(1), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(child: SizedBox(key: GlobalObjectKey<_LeafState>(2), height: 400.0)),
            AutomaticKeepAlive(child: SizedBox(key: GlobalObjectKey<_LeafState>(3), height: 400.0)),
          ],
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0)); // move to bottom
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    await tester.drag(find.byType(ListView), const Offset(0.0, 1000.0)); // move to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(0).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0)); // move to bottom
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    const GlobalObjectKey<_LeafState>(1).currentState!.setKeepAlive(true);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    const GlobalObjectKey<_LeafState>(0).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    const GlobalObjectKey<_LeafState>(1).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
  });

  testWidgets('AutomaticKeepAlive double 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          cacheExtent: 0.0,
          children: const <Widget>[
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(0), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(1), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(2), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(3), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(4), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(5), child: Placeholder()),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(0).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0)); // move to bottom
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5)), findsOneWidget);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          cacheExtent: 0.0,
          children: const <Widget>[
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(1), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(2), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(3), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(4), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(5), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(0), child: Placeholder()),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
    await tester.pump(); // Sometimes AutomaticKeepAlive needs an extra pump to clean things up.
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsOneWidget);
    await tester.drag(find.byType(ListView), const Offset(0.0, 1000.0)); // move to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0)), findsNothing);
    const GlobalObjectKey<_LeafState>(0).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsNothing);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          cacheExtent: 0.0,
          children: const <Widget>[
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(1), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(2), child: Placeholder()),
                  ],
                ),
              ),
            ),
            AutomaticKeepAlive(child: SizedBox(height: 400.0, child: Stack())),
            AutomaticKeepAlive(
              child: SizedBox(
                height: 400.0,
                child: Stack(
                  children: <Widget>[
                    Leaf(key: GlobalObjectKey<_LeafState>(3), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(4), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(5), child: Placeholder()),
                    Leaf(key: GlobalObjectKey<_LeafState>(0), child: Placeholder()),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
    await tester.pump(); // Sometimes AutomaticKeepAlive needs an extra pump to clean things up.
    expect(find.byKey(const GlobalObjectKey<_LeafState>(1)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(2)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(4), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(5), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(0), skipOffstage: false), findsNothing);
  });

  testWidgets('AutomaticKeepAlive with keepAlive set to true before initState', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          dragStartBehavior: DragStartBehavior.down,
          addSemanticIndexes: false,
          itemCount: 50,
          itemBuilder: (BuildContext context, int index) {
            if (index == 0) {
              return const _AlwaysKeepAlive(key: GlobalObjectKey<_AlwaysKeepAliveState>(0));
            }
            return SizedBox(height: 44.0, child: Text('FooBar $index'));
          },
        ),
      ),
    );

    expect(find.text('keep me alive'), findsOneWidget);
    expect(find.text('FooBar 1'), findsOneWidget);
    expect(find.text('FooBar 2'), findsOneWidget);

    expect(find.byKey(const GlobalObjectKey<_AlwaysKeepAliveState>(0)), findsOneWidget);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0)); // move to bottom
    await tester.pump();
    expect(
      find.byKey(const GlobalObjectKey<_AlwaysKeepAliveState>(0), skipOffstage: false),
      findsOneWidget,
    );

    expect(find.text('keep me alive', skipOffstage: false), findsOneWidget);
    expect(find.text('FooBar 1'), findsNothing);
    expect(find.text('FooBar 2'), findsNothing);
  });

  testWidgets(
    'AutomaticKeepAlive with keepAlive set to true before initState and widget goes out of scope',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            addSemanticIndexes: false,
            itemCount: 250,
            itemBuilder: (BuildContext context, int index) {
              if (index.isEven) {
                return _AlwaysKeepAlive(key: GlobalObjectKey<_AlwaysKeepAliveState>(index));
              }
              return SizedBox(height: 44.0, child: Text('FooBar $index'));
            },
          ),
        ),
      );

      expect(find.text('keep me alive'), findsNWidgets(7));
      expect(find.text('FooBar 1'), findsOneWidget);
      expect(find.text('FooBar 3'), findsOneWidget);

      expect(find.byKey(const GlobalObjectKey<_AlwaysKeepAliveState>(0)), findsOneWidget);

      final ScrollableState state = tester.state(find.byType(Scrollable));
      final ScrollPosition position = state.position;
      position.jumpTo(3025.0);

      await tester.pump();
      expect(
        find.byKey(const GlobalObjectKey<_AlwaysKeepAliveState>(0), skipOffstage: false),
        findsOneWidget,
      );

      expect(find.text('keep me alive', skipOffstage: false), findsNWidgets(23));
      expect(find.text('FooBar 1'), findsNothing);
      expect(find.text('FooBar 3'), findsNothing);
      expect(find.text('FooBar 73'), findsOneWidget);
    },
  );

  testWidgets('AutomaticKeepAlive with SliverKeepAliveWidget', (WidgetTester tester) async {
    // We're just doing a basic test here to make sure that the functionality of
    // RenderSliverWithKeepAliveMixin doesn't get regressed or deleted. As testing
    // the full functionality would be cumbersome.
    final RenderSliverMultiBoxAdaptorAlt alternate = RenderSliverMultiBoxAdaptorAlt();
    addTearDown(alternate.dispose);
    final RenderBox child = RenderBoxKeepAlive();
    addTearDown(child.dispose);
    alternate.insert(child);

    expect(alternate.children.length, 1);
  });

  testWidgets('Keep alive Listenable has its listener removed once called', (
    WidgetTester tester,
  ) async {
    final LeakCheckerHandle handle = LeakCheckerHandle();
    addTearDown(handle.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          itemCount: 1,
          itemBuilder: (BuildContext context, int index) {
            return const KeepAliveListenableLeakChecker(
              key: GlobalObjectKey<_KeepAliveListenableLeakCheckerState>(0),
            );
          },
        ),
      ),
    );
    final _KeepAliveListenableLeakCheckerState state =
        const GlobalObjectKey<_KeepAliveListenableLeakCheckerState>(0).currentState!;

    expect(handle.hasListeners, false);
    state.dispatch(handle);
    expect(handle.hasListeners, true);
    handle.notifyListeners();
    expect(handle.hasListeners, false);
  });
}

class _AlwaysKeepAlive extends StatefulWidget {
  const _AlwaysKeepAlive({required Key super.key});

  @override
  State<StatefulWidget> createState() => _AlwaysKeepAliveState();
}

class _AlwaysKeepAliveState extends State<_AlwaysKeepAlive>
    with AutomaticKeepAliveClientMixin<_AlwaysKeepAlive> {
  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return const SizedBox(height: 48.0, child: Text('keep me alive'));
  }
}

class RenderBoxKeepAlive extends RenderBox {}

mixin KeepAliveParentDataMixinAlt implements KeepAliveParentDataMixin {
  @override
  bool keptAlive = false;

  @override
  bool keepAlive = false;
}

class RenderSliverMultiBoxAdaptorAlt extends RenderSliver
    with KeepAliveParentDataMixinAlt, RenderSliverHelpers, RenderSliverWithKeepAliveMixin {
  final List<RenderBox> children = <RenderBox>[];

  void insert(RenderBox child, {RenderBox? after}) {
    children.add(child);
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    children.forEach(visitor);
  }

  @override
  void performLayout() {}
}

class LeakCheckerHandle with ChangeNotifier {
  LeakCheckerHandle() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  @override
  bool get hasListeners => super.hasListeners;
}

class KeepAliveListenableLeakChecker extends StatefulWidget {
  const KeepAliveListenableLeakChecker({super.key});

  @override
  State<KeepAliveListenableLeakChecker> createState() => _KeepAliveListenableLeakCheckerState();
}

class _KeepAliveListenableLeakCheckerState extends State<KeepAliveListenableLeakChecker> {
  void dispatch(Listenable handle) {
    KeepAliveNotification(handle).dispatch(context);
  }

  @override
  Widget build(BuildContext context) {
    return const Placeholder();
  }
}
