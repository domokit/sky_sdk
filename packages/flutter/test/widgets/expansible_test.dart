// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class TestExpansibleWidget extends StatefulWidget {
  const TestExpansibleWidget({
    super.key,
    this.initiallyExpanded = false,
    this.maintainState = false,
    this.onExpansionChanged,
    this.expansionDuration = const Duration(milliseconds: 200),
    this.expansionCurve = Curves.linear,
    required this.controller,
    required this.children,
  });
  final bool initiallyExpanded;
  final List<Widget> children;
  final bool maintainState;
  final ValueChanged<bool>? onExpansionChanged;
  final Duration expansionDuration;
  final Curve expansionCurve;
  final ExpansibleController<TestExpansibleWidget> controller;

  @override
  State<TestExpansibleWidget> createState() => _TestExpansibleWidgetState();
}

class _TestExpansibleWidgetState extends State<TestExpansibleWidget>
    with TickerProviderStateMixin, ExpansibleStateMixin<TestExpansibleWidget> {
  @override
  List<Widget> get children => widget.children;

  @override
  ValueChanged<bool>? get onExpansionChanged => widget.onExpansionChanged;

  @override
  Duration get expansionDuration => widget.expansionDuration;

  @override
  Curve get expansionCurve => widget.expansionCurve;

  @override
  bool get initiallyExpanded => widget.initiallyExpanded;

  @override
  bool get maintainState => widget.maintainState;

  @override
  ExpansibleController<TestExpansibleWidget> get controller => widget.controller;

  @override
  Widget buildHeader(BuildContext context) {
    return GestureDetector(onTap: toggleExpansion, child: const Text('Header'));
  }

  @override
  Widget buildBody(BuildContext context) {
    return Column(children: children);
  }
}

void main() {
  testWidgets('Controller expands and collapses the widget', (WidgetTester tester) async {
    final ExpansibleController<TestExpansibleWidget> controller =
        ExpansibleController<TestExpansibleWidget>();
    await tester.pumpWidget(
      MaterialApp(
        color: const Color(0xffffffff),
        home: TestExpansibleWidget(controller: controller, children: const <Widget>[Text('Body')]),
      ),
    );

    expect(find.text('Body'), findsNothing);
    controller.expand();
    await tester.pumpAndSettle();
    expect(find.text('Body'), findsOneWidget);

    controller.collapse();
    await tester.pumpAndSettle();
    expect(find.text('Body'), findsNothing);
  });

  testWidgets('onExpansionChanged callback', (WidgetTester tester) async {
    final ExpansibleController<TestExpansibleWidget> controller =
        ExpansibleController<TestExpansibleWidget>();
    bool? expansionState;
    await tester.pumpWidget(
      MaterialApp(
        color: const Color(0xffffffff),
        home: TestExpansibleWidget(
          controller: controller,
          children: const <Widget>[Text('Body')],
          onExpansionChanged: (bool expanded) {
            expansionState = expanded;
          },
        ),
      ),
    );

    // Tap on the header to toggle the expansion.
    await tester.tap(find.text('Header'));
    await tester.pumpAndSettle();
    expect(expansionState, true);

    await tester.tap(find.text('Header'));
    await tester.pumpAndSettle();
    expect(expansionState, false);

    // Use the controller to toggle the expansion.
    controller.expand();
    await tester.pumpAndSettle();
    expect(expansionState, true);

    controller.collapse();
    await tester.pumpAndSettle();
    expect(expansionState, false);
  });

  testWidgets('Respects initiallyExpanded', (WidgetTester tester) async {
    final ExpansibleController<TestExpansibleWidget> controller =
        ExpansibleController<TestExpansibleWidget>();
    await tester.pumpWidget(
      MaterialApp(
        home: SingleChildScrollView(
          child: Column(
            children: <Widget>[
              TestExpansibleWidget(
                controller: controller,
                initiallyExpanded: true,
                children: const <Widget>[Text('Body')],
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text('Body'), findsOneWidget);

    await tester.tap(find.text('Header'));
    await tester.pumpAndSettle();

    expect(find.text('Body'), findsNothing);
  });

  testWidgets('ExpansionTile maintainState', (WidgetTester tester) async {
    final ExpansibleController<TestExpansibleWidget> controller1 =
        ExpansibleController<TestExpansibleWidget>();
    final ExpansibleController<TestExpansibleWidget> controller2 =
        ExpansibleController<TestExpansibleWidget>();
    await tester.pumpWidget(
      MaterialApp(
        home: SingleChildScrollView(
          child: Column(
            children: <Widget>[
              TestExpansibleWidget(
                controller: controller1,
                maintainState: true,
                children: const <Widget>[Text('Maintaining State')],
              ),
              TestExpansibleWidget(
                controller: controller2,
                children: const <Widget>[Text('Discarding State')],
              ),
            ],
          ),
        ),
      ),
    );

    // This text should be offstage while the expansible widget is collapsed.
    expect(find.text('Maintaining State', skipOffstage: false), findsOneWidget);
    expect(find.text('Maintaining State'), findsNothing);
    // This text is not displayed while the expansible widget is collapsed.
    expect(find.text('Discarding State'), findsNothing);
  });

  testWidgets('Respects expansionDuration and expansionCurve', (WidgetTester tester) async {
    final ExpansibleController<TestExpansibleWidget> controller =
        ExpansibleController<TestExpansibleWidget>();
    await tester.pumpWidget(
      MaterialApp(
        home: TestExpansibleWidget(
          controller: controller,
          expansionDuration: const Duration(milliseconds: 120),
          expansionCurve: Curves.easeOut,
          children: const <Widget>[SizedBox(height: 50.0, child: Placeholder())],
        ),
      ),
    );

    expect(find.byType(Placeholder), findsNothing);

    await tester.tap(find.text('Header'));

    // One pump to start the animation, and more pumps to get to different
    // points in the animation.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 60));
    expect(tester.getBottomLeft(find.byType(Placeholder)).dy, 90.08984375);

    // The animation has completed.
    await tester.pump(const Duration(milliseconds: 60) + const Duration(microseconds: 1));
    expect(tester.getBottomLeft(find.byType(Placeholder)).dy, 98.0);

    // SInce the animation has completed, the vertical position doesn't change.
    await tester.pump(const Duration(milliseconds: 60));
    expect(tester.getBottomLeft(find.byType(Placeholder)).dy, 98.0);
  });
}
