// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

class TestState extends StatefulWidget {
  const TestState({super.key, required this.child, required this.log});
  final Widget child;
  final List<String> log;
  @override
  State<TestState> createState() => _TestStateState();
}

class _TestStateState extends State<TestState> {
  @override
  void initState() {
    super.initState();
    widget.log.add('created new state');
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}

void main() {
  testWidgets('Visibility', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final List<String> log = <String>[];

    final Widget testChild = GestureDetector(
      onTap: () {
        log.add('tap');
      },
      child: Builder(
        builder: (BuildContext context) {
          final bool animating = TickerMode.of(context);
          return TestState(log: log, child: Text('a $animating', textDirection: TextDirection.rtl));
        },
      ),
    );

    final Matcher expectedSemanticsWhenPresent = hasSemantics(
      TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics.rootChild(
            label: 'a true',
            textDirection: TextDirection.rtl,
            actions: <SemanticsAction>[SemanticsAction.tap],
          ),
        ],
      ),
      ignoreId: true,
      ignoreRect: true,
      ignoreTransform: true,
    );

    final Matcher expectedSemanticsWhenPresentWithIgnorePointer = hasSemantics(
      TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics.rootChild(label: 'a true', textDirection: TextDirection.rtl),
        ],
      ),
      ignoreId: true,
      ignoreRect: true,
      ignoreTransform: true,
    );

    final Matcher expectedSemanticsWhenAbsent = hasSemantics(TestSemantics.root());

    // We now run a sequence of pumpWidget calls one after the other. In
    // addition to verifying that the right behavior is seen in each case, this
    // also verifies that the widget can dynamically change from state to state.

    await tester.pumpWidget(Visibility(child: testChild));
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(800.0, 600.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['created new state', 'tap']);
    log.clear();

    await tester.pumpWidget(Visibility(visible: false, child: testChild));
    expect(find.byType(Text, skipOffstage: false), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), const Size(800.0, 600.0));
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(Center(child: Visibility(visible: false, child: testChild)));
    expect(find.byType(Text, skipOffstage: false), findsNothing);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(
      Center(child: Visibility(replacement: const Placeholder(), visible: false, child: testChild)),
    );
    expect(find.byType(Text, skipOffstage: false), findsNothing);
    expect(find.byType(Placeholder), findsOneWidget);
    expect(find.byType(Visibility), paints..path());
    expect(tester.getSize(find.byType(Visibility)), const Size(800.0, 600.0));
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(
      Center(child: Visibility(replacement: const Placeholder(), child: testChild)),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['created new state', 'tap']);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          maintainState: true,
          maintainAnimation: true,
          maintainSize: true,
          maintainInteractivity: true,
          maintainSemantics: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['created new state', 'tap']);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          visible: false,
          maintainState: true,
          maintainAnimation: true,
          maintainSize: true,
          maintainInteractivity: true,
          maintainSemantics: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['tap']);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          visible: false,
          maintainState: true,
          maintainAnimation: true,
          maintainSize: true,
          maintainInteractivity: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['tap']);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          visible: false,
          maintainState: true,
          maintainAnimation: true,
          maintainSize: true,
          maintainSemantics: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresentWithIgnorePointer);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          visible: false,
          maintainState: true,
          maintainAnimation: true,
          maintainSize: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(
      Center(
        child: Visibility(
          visible: false,
          maintainState: true,
          maintainAnimation: true,
          child: testChild,
        ),
      ),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.byType(Text), findsNothing);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>['created new state']);
    log.clear();

    await tester.pumpWidget(
      Center(child: Visibility(visible: false, maintainState: true, child: testChild)),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.byType(Text), findsNothing);
    expect(find.text('a false', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>['created new state']);
    log.clear();

    // Now we toggle the visibility off and on a few times to make sure that works.

    await tester.pumpWidget(Center(child: Visibility(maintainState: true, child: testChild)));
    expect(find.byType(Text), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>['tap']);
    log.clear();

    await tester.pumpWidget(
      Center(child: Visibility(visible: false, maintainState: true, child: testChild)),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.byType(Text), findsNothing);
    expect(find.text('a false', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(Center(child: Visibility(maintainState: true, child: testChild)));
    expect(find.byType(Text), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['tap']);
    log.clear();

    await tester.pumpWidget(
      Center(child: Visibility(visible: false, maintainState: true, child: testChild)),
    );
    expect(find.byType(Text, skipOffstage: false), findsOneWidget);
    expect(find.byType(Text), findsNothing);
    expect(find.text('a false', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    // Same but without maintainState.

    await tester.pumpWidget(Center(child: Visibility(visible: false, child: testChild)));
    expect(find.byType(Text, skipOffstage: false), findsNothing);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(Center(child: Visibility(child: testChild)));
    expect(find.byType(Text), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['created new state', 'tap']);
    log.clear();

    await tester.pumpWidget(Center(child: Visibility(visible: false, child: testChild)));
    expect(find.byType(Text, skipOffstage: false), findsNothing);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paintsNothing);
    expect(tester.getSize(find.byType(Visibility)), Size.zero);
    expect(semantics, expectedSemanticsWhenAbsent);
    expect(log, <String>[]);
    await tester.tap(find.byType(Visibility), warnIfMissed: false);
    expect(log, <String>[]);
    log.clear();

    await tester.pumpWidget(Center(child: Visibility(child: testChild)));
    expect(find.byType(Text), findsOneWidget);
    expect(find.text('a true', skipOffstage: false), findsOneWidget);
    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Visibility), paints..paragraph());
    expect(tester.getSize(find.byType(Visibility)), const Size(84.0, 14.0));
    expect(semantics, expectedSemanticsWhenPresent);
    expect(log, <String>['created new state']);
    await tester.tap(find.byType(Visibility));
    expect(log, <String>['created new state', 'tap']);
    log.clear();

    semantics.dispose();
  });

  testWidgets('Visibility does not force compositing when visible and maintain*', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Visibility(
        maintainSize: true,
        maintainAnimation: true,
        maintainState: true,
        child: Text('hello', textDirection: TextDirection.ltr),
      ),
    );

    // Root transform from the tester and then the picture created by the text.
    expect(tester.layers, hasLength(2));
    expect(tester.layers, isNot(contains(isA<OpacityLayer>())));
    expect(tester.layers.last, isA<PictureLayer>());
  });

  testWidgets('SliverVisibility does not force compositing when visible and maintain*', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverVisibility(
              maintainSize: true,
              maintainAnimation: true,
              maintainState: true,
              sliver: SliverList(
                delegate: SliverChildListDelegate.fixed(addRepaintBoundaries: false, <Widget>[
                  Text('hello'),
                ]),
              ),
            ),
          ],
        ),
      ),
    );

    // This requires a lot more layers due to including sliver lists which do manage additional
    // offset layers. Just trust me this is one fewer layers than before...
    expect(tester.layers, hasLength(6));
    expect(tester.layers, isNot(contains(isA<OpacityLayer>())));
    expect(tester.layers.last, isA<PictureLayer>());
  });

  testWidgets('Visibility.of returns correct value', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: _ShowVisibility()),
    );
    expect(find.text('is visible ? true', skipOffstage: false), findsOneWidget);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Visibility(maintainState: true, child: _ShowVisibility()),
      ),
    );
    expect(find.text('is visible ? true', skipOffstage: false), findsOneWidget);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Visibility(visible: false, maintainState: true, child: _ShowVisibility()),
      ),
    );
    expect(find.text('is visible ? false', skipOffstage: false), findsOneWidget);
  });

  testWidgets('Visibility.of works when multiple Visibility widgets are in hierarchy', (
    WidgetTester tester,
  ) async {
    bool didChangeDependencies = false;
    void handleDidChangeDependencies() {
      didChangeDependencies = true;
    }

    Widget newWidget({required bool ancestorIsVisible, required bool descendantIsVisible}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Visibility(
          visible: ancestorIsVisible,
          maintainState: true,
          child: Center(
            child: Visibility(
              visible: descendantIsVisible,
              maintainState: true,
              child: _ShowVisibility(onDidChangeDependencies: handleDidChangeDependencies),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(newWidget(ancestorIsVisible: true, descendantIsVisible: true));
    expect(didChangeDependencies, isTrue);
    expect(find.text('is visible ? true', skipOffstage: false), findsOneWidget);
    didChangeDependencies = false;

    await tester.pumpWidget(newWidget(ancestorIsVisible: true, descendantIsVisible: false));
    expect(didChangeDependencies, isTrue);
    expect(find.text('is visible ? false', skipOffstage: false), findsOneWidget);
    didChangeDependencies = false;

    await tester.pumpWidget(newWidget(ancestorIsVisible: true, descendantIsVisible: false));
    expect(didChangeDependencies, isFalse);

    await tester.pumpWidget(newWidget(ancestorIsVisible: false, descendantIsVisible: false));
    expect(didChangeDependencies, isTrue);
    didChangeDependencies = false;

    await tester.pumpWidget(newWidget(ancestorIsVisible: false, descendantIsVisible: true));
    expect(didChangeDependencies, isTrue);
    expect(find.text('is visible ? false', skipOffstage: false), findsOneWidget);
  });
}

class _ShowVisibility extends StatefulWidget {
  const _ShowVisibility({this.onDidChangeDependencies});

  final VoidCallback? onDidChangeDependencies;

  @override
  State<_ShowVisibility> createState() => _ShowVisibilityState();
}

class _ShowVisibilityState extends State<_ShowVisibility> {
  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (widget.onDidChangeDependencies != null) {
      widget.onDidChangeDependencies!();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Text('is visible ? ${Visibility.of(context)}');
  }
}
