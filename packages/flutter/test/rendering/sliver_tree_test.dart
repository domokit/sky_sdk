// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

List<TreeSliverNode<String>> _setUpNodes() {
  return <TreeSliverNode<String>>[
    TreeSliverNode<String>('First'),
    TreeSliverNode<String>(
      'Second',
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>(
          'alpha',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('uno'),
            TreeSliverNode<String>('dos'),
            TreeSliverNode<String>('tres'),
          ],
        ),
        TreeSliverNode<String>('beta'),
        TreeSliverNode<String>('kappa'),
      ],
    ),
    TreeSliverNode<String>(
      'Third',
      expanded: true,
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>('gamma'),
        TreeSliverNode<String>('delta'),
        TreeSliverNode<String>('epsilon'),
      ],
    ),
    TreeSliverNode<String>('Fourth'),
  ];
}

List<TreeSliverNode<String>> treeNodes = _setUpNodes();

void main() {
  testWidgets('asserts proper axis directions', (WidgetTester tester) async {
    final List<Object?> exceptions = <Object?>[];
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      exceptions.add(details.exception);
    };
    addTearDown(() {
      FlutterError.onError = oldHandler;
    });

    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          reverse: true,
          slivers: <Widget>[TreeSliver<String>(tree: treeNodes)],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exceptions.isNotEmpty, isTrue);
    expect(
      exceptions[0].toString(),
      contains('TreeSliver is only supported in Viewports with an AxisDirection.down.'),
    );

    exceptions.clear();
    await tester.pumpWidget(Container());
    FlutterError.onError = (FlutterErrorDetails details) {
      exceptions.add(details.exception);
    };

    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          scrollDirection: Axis.horizontal,
          reverse: true,
          slivers: <Widget>[TreeSliver<String>(tree: treeNodes)],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exceptions.isNotEmpty, isTrue);
    expect(
      exceptions[0].toString(),
      contains('TreeSliver is only supported in Viewports with an AxisDirection.down.'),
    );

    exceptions.clear();
    await tester.pumpWidget(Container());
    FlutterError.onError = (FlutterErrorDetails details) {
      exceptions.add(details.exception);
    };

    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          scrollDirection: Axis.horizontal,
          slivers: <Widget>[TreeSliver<String>(tree: treeNodes)],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exceptions.isNotEmpty, isTrue);
    expect(
      exceptions[0].toString(),
      contains('TreeSliver is only supported in Viewports with an AxisDirection.down.'),
    );
  });

  testWidgets('Basic layout', (WidgetTester tester) async {
    treeNodes = _setUpNodes();
    // Default layout, custom indentation values, row extents.
    TreeSliver<String> treeSliver = TreeSliver<String>(tree: treeNodes);
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('First'), findsOneWidget);
    expect(tester.getRect(find.text('First')), const Rect.fromLTRB(46.0, 8.0, 286.0, 32.0));
    expect(find.text('Second'), findsOneWidget);
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    expect(find.text('Third'), findsOneWidget);
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(find.text('gamma'), findsOneWidget);
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    expect(find.text('delta'), findsOneWidget);
    expect(tester.getRect(find.text('delta')), const Rect.fromLTRB(46.0, 168.0, 286.0, 192.0));
    expect(find.text('epsilon'), findsOneWidget);
    expect(tester.getRect(find.text('epsilon')), const Rect.fromLTRB(46.0, 208.0, 382.0, 232.0));
    expect(find.text('Fourth'), findsOneWidget);
    expect(tester.getRect(find.text('Fourth')), const Rect.fromLTRB(46.0, 248.0, 334.0, 272.0));

    treeSliver = TreeSliver<String>(tree: treeNodes, indentation: TreeSliverIndentationType.none);
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(46.0, 128.0))
        ..paragraph(offset: const Offset(46.0, 168.0))
        ..paragraph(offset: const Offset(46.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('First'), findsOneWidget);
    expect(tester.getRect(find.text('First')), const Rect.fromLTRB(46.0, 8.0, 286.0, 32.0));
    expect(find.text('Second'), findsOneWidget);
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    expect(find.text('Third'), findsOneWidget);
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(find.text('gamma'), findsOneWidget);
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    expect(find.text('delta'), findsOneWidget);
    expect(tester.getRect(find.text('delta')), const Rect.fromLTRB(46.0, 168.0, 286.0, 192.0));
    expect(find.text('epsilon'), findsOneWidget);
    expect(tester.getRect(find.text('epsilon')), const Rect.fromLTRB(46.0, 208.0, 382.0, 232.0));
    expect(find.text('Fourth'), findsOneWidget);
    expect(tester.getRect(find.text('Fourth')), const Rect.fromLTRB(46.0, 248.0, 334.0, 272.0));

    treeSliver = TreeSliver<String>(
      tree: treeNodes,
      indentation: TreeSliverIndentationType.custom(50.0),
    );
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(96.0, 128.0))
        ..paragraph(offset: const Offset(96.0, 168.0))
        ..paragraph(offset: const Offset(96.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('First'), findsOneWidget);
    expect(tester.getRect(find.text('First')), const Rect.fromLTRB(46.0, 8.0, 286.0, 32.0));
    expect(find.text('Second'), findsOneWidget);
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    expect(find.text('Third'), findsOneWidget);
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(find.text('gamma'), findsOneWidget);
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    expect(find.text('delta'), findsOneWidget);
    expect(tester.getRect(find.text('delta')), const Rect.fromLTRB(46.0, 168.0, 286.0, 192.0));
    expect(find.text('epsilon'), findsOneWidget);
    expect(tester.getRect(find.text('epsilon')), const Rect.fromLTRB(46.0, 208.0, 382.0, 232.0));
    expect(find.text('Fourth'), findsOneWidget);
    expect(tester.getRect(find.text('Fourth')), const Rect.fromLTRB(46.0, 248.0, 334.0, 272.0));

    treeSliver = TreeSliver<String>(tree: treeNodes, treeRowExtentBuilder: (_, __) => 100);
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 26.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 126.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 226.0))
        ..paragraph(offset: const Offset(56.0, 326.0))
        ..paragraph(offset: const Offset(56.0, 426.0))
        ..paragraph(offset: const Offset(56.0, 526.0)),
    );
    expect(find.text('First'), findsOneWidget);
    expect(tester.getRect(find.text('First')), const Rect.fromLTRB(46.0, 26.0, 286.0, 74.0));
    expect(find.text('Second'), findsOneWidget);
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 126.0, 334.0, 174.0));
    expect(find.text('Third'), findsOneWidget);
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 226.0, 286.0, 274.0));
    expect(find.text('gamma'), findsOneWidget);
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 326.0, 286.0, 374.0));
    expect(find.text('delta'), findsOneWidget);
    expect(tester.getRect(find.text('delta')), const Rect.fromLTRB(46.0, 426.0, 286.0, 474.0));
    expect(find.text('epsilon'), findsOneWidget);
    expect(tester.getRect(find.text('epsilon')), const Rect.fromLTRB(46.0, 526.0, 382.0, 574.0));
    expect(find.text('Fourth'), findsNothing);
  });

  testWidgets('Animating node segment', (WidgetTester tester) async {
    treeNodes = _setUpNodes();
    TreeSliver<String> treeSliver = TreeSliver<String>(tree: treeNodes);
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('alpha'), findsNothing);
    await tester.tap(find.byType(Icon).first);
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph(offset: const Offset(56.0, 8.0)) // beta animating in
        ..paragraph(offset: const Offset(56.0, 48.0)) // kappa animating in
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    // New nodes have been inserted into the tree, alpha
    // is not visible yet.
    expect(find.text('alpha'), findsNothing);
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')), const Rect.fromLTRB(46.0, 8.0, 238.0, 32.0));
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')), const Rect.fromLTRB(46.0, 48.0, 286.0, 72.0));
    // Progress the animation.
    await tester.pump(const Duration(milliseconds: 50));
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // alpha icon
        ..paragraph(offset: const Offset(56.0, 8.0)) // alpha animating in
        ..paragraph(offset: const Offset(56.0, 48.0)) // beta animating in
        ..paragraph(offset: const Offset(56.0, 88.0)) // kappa animating in
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(56.0, 248.0))
        ..paragraph(offset: const Offset(46.0, 288.0)),
    );
    expect(tester.getRect(find.text('alpha')).top.floor(), 8.0);
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')).top.floor(), 48.0);
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')).top.floor(), 88.0);
    // Complete the animation
    await tester.pumpAndSettle();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0)) // First
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0)) // Second
        ..paragraph() // alpha icon
        ..paragraph(offset: const Offset(56.0, 88.0)) // alpha
        ..paragraph(offset: const Offset(56.0, 128.0)) // beta
        ..paragraph(offset: const Offset(56.0, 168.0)) // kappa
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 208.0)) // Third
        ..paragraph(offset: const Offset(56.0, 248.0)) // gamma
        ..paragraph(offset: const Offset(56.0, 288.0)) // delta
        ..paragraph(offset: const Offset(56.0, 328.0)) // epsilon
        ..paragraph(offset: const Offset(46.0, 368.0)), // Fourth
    );
    expect(find.text('alpha'), findsOneWidget);
    expect(tester.getRect(find.text('alpha')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')), const Rect.fromLTRB(46.0, 128.0, 238.0, 152.0));
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')), const Rect.fromLTRB(46.0, 168.0, 286.0, 192.0));

    // Customize the animation
    treeSliver = TreeSliver<String>(
      tree: treeNodes,
      toggleAnimationStyle: AnimationStyle(
        duration: const Duration(milliseconds: 500),
        curve: Curves.bounceIn,
      ),
    );
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0)) // First
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0)) // Second
        ..paragraph() // alpha icon
        ..paragraph(offset: const Offset(56.0, 88.0)) // alpha
        ..paragraph(offset: const Offset(56.0, 128.0)) // beta
        ..paragraph(offset: const Offset(56.0, 168.0)) // kappa
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 208.0)) // Third
        ..paragraph(offset: const Offset(56.0, 248.0)) // gamma
        ..paragraph(offset: const Offset(56.0, 288.0)) // delta
        ..paragraph(offset: const Offset(56.0, 328.0)) // epsilon
        ..paragraph(offset: const Offset(46.0, 368.0)), // Fourth
    );
    // Still visible from earlier.
    expect(find.text('alpha'), findsOneWidget);
    expect(tester.getRect(find.text('alpha')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    // Collapse the node now
    await tester.tap(find.byType(Icon).first);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 200));
    expect(find.text('alpha'), findsOneWidget);
    expect(tester.getRect(find.text('alpha')).top.floor(), -22);
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')).top.floor(), 18);
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')).top.floor(), 58);
    // Progress the animation.
    await tester.pump(const Duration(milliseconds: 200));
    expect(find.text('alpha'), findsOneWidget);
    expect(tester.getRect(find.text('alpha')).top.floor(), -25);
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')).top.floor(), 15);
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')).top.floor(), 55.0);
    // Complete the animation
    await tester.pumpAndSettle();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('alpha'), findsNothing);

    // Disable the animation
    treeSliver = TreeSliver<String>(
      tree: treeNodes,
      toggleAnimationStyle: AnimationStyle.noAnimation,
    );
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    // Not in the tree.
    expect(find.text('alpha'), findsNothing);
    // Collapse the node now
    await tester.tap(find.byType(Icon).first);
    await tester.pump();
    // No animating, straight to positions.
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0)) // First
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0)) // Second
        ..paragraph() // alpha icon
        ..paragraph(offset: const Offset(56.0, 88.0)) // alpha
        ..paragraph(offset: const Offset(56.0, 128.0)) // beta
        ..paragraph(offset: const Offset(56.0, 168.0)) // kappa
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 208.0)) // Third
        ..paragraph(offset: const Offset(56.0, 248.0)) // gamma
        ..paragraph(offset: const Offset(56.0, 288.0)) // delta
        ..paragraph(offset: const Offset(56.0, 328.0)) // epsilon
        ..paragraph(offset: const Offset(46.0, 368.0)), // Fourth
    );
    expect(find.text('alpha'), findsOneWidget);
    expect(tester.getRect(find.text('alpha')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(find.text('beta'), findsOneWidget);
    expect(tester.getRect(find.text('beta')), const Rect.fromLTRB(46.0, 128.0, 238.0, 152.0));
    expect(find.text('kappa'), findsOneWidget);
    expect(tester.getRect(find.text('kappa')), const Rect.fromLTRB(46.0, 168.0, 286.0, 192.0));
  });

  testWidgets('Multiple animating node segments', (WidgetTester tester) async {
    treeNodes = _setUpNodes();
    final TreeSliver<String> treeSliver = TreeSliver<String>(tree: treeNodes);
    await tester.pumpWidget(MaterialApp(home: CustomScrollView(slivers: <Widget>[treeSliver])));
    await tester.pump();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(find.text('Second'), findsOneWidget);
    expect(find.text('alpha'), findsNothing); // Second is collapsed
    expect(find.text('Third'), findsOneWidget);
    expect(find.text('gamma'), findsOneWidget); // Third is expanded

    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));

    // Trigger two animations to run together.
    // Collapse Third
    await tester.tap(find.byType(Icon).last);
    // Expand Second
    await tester.tap(find.byType(Icon).first);
    await tester.pump(const Duration(milliseconds: 15));
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph(offset: const Offset(56.0, 8.0)) // beta entering
        ..paragraph(offset: const Offset(56.0, 48.0)) // kappa entering
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    // Third is collapsing
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    // Second is expanding
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    // beta has been added and is animating into view.
    expect(tester.getRect(find.text('beta')).top.floor(), 8.0);
    await tester.pump(const Duration(milliseconds: 15));
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // alpha icon animating
        ..paragraph(offset: const Offset(56.0, -20.0)) // alpha animating
        ..paragraph(offset: const Offset(56.0, 20.0)) // beta
        ..paragraph(offset: const Offset(56.0, 60.0)) // kappa
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 100.0)) // Third
        // Children of Third are animating, but the expand and
        // collapse counter each other, so their position is unchanged.
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    // Third is still collapsing. Third is sliding down
    // as Seconds's children slide in, gamma is still exiting.
    expect(tester.getRect(find.text('Third')).top.floor(), 100.0);
    // gamma appears to not have moved, this is because it is
    // intersecting both animations, the positive offset of
    // Second animation == the negative offset of Third
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    // Second is still expanding
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    // alpha is still animating into view.
    expect(tester.getRect(find.text('alpha')).top.floor(), -20.0);
    // Progress the animation further
    await tester.pump(const Duration(milliseconds: 15));
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // alpha icon animating
        ..paragraph(offset: const Offset(56.0, -8.0)) // alpha animating
        ..paragraph(offset: const Offset(56.0, 32.0)) // beta
        ..paragraph(offset: const Offset(56.0, 72.0)) // kappa
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 112.0)) // Third
        // Children of Third are animating, but the expand and
        // collapse counter each other, so their position is unchanged.
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph(offset: const Offset(56.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    // Third is still collapsing. Third is sliding down
    // as Seconds's children slide in, gamma is still exiting.
    expect(tester.getRect(find.text('Third')).top.floor(), 112.0);
    // gamma appears to not have moved, this is because it is
    // intersecting both animations, the positive offset of
    // Second animation == the negative offset of Third
    expect(tester.getRect(find.text('gamma')), const Rect.fromLTRB(46.0, 128.0, 286.0, 152.0));
    // Second is still expanding
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    // alpha is still animating into view.
    expect(tester.getRect(find.text('alpha')).top.floor(), -8.0);
    // Complete the animations
    await tester.pumpAndSettle();
    expect(
      find.byType(TreeSliver<String>),
      paints
        ..paragraph(offset: const Offset(46.0, 8.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 48.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(56.0, 88.0))
        ..paragraph(offset: const Offset(56.0, 128.0))
        ..paragraph(offset: const Offset(56.0, 168.0))
        ..paragraph() // Icon
        ..paragraph(offset: const Offset(46.0, 208.0))
        ..paragraph(offset: const Offset(46.0, 248.0)),
    );
    expect(tester.getRect(find.text('Third')), const Rect.fromLTRB(46.0, 208.0, 286.0, 232.0));
    // gamma has left the building
    expect(find.text('gamma'), findsNothing);
    expect(tester.getRect(find.text('Second')), const Rect.fromLTRB(46.0, 48.0, 334.0, 72.0));
    // alpha is in place.
    expect(tester.getRect(find.text('alpha')), const Rect.fromLTRB(46.0, 88.0, 286.0, 112.0));
  });

  testWidgets('only paints visible rows', (WidgetTester tester) async {
    treeNodes = _setUpNodes();
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    treeNodes = _setUpNodes();
    final TreeSliver<String> treeSliver = TreeSliver<String>(
      treeRowExtentBuilder: (_, __) => 200,
      tree: treeNodes,
    );
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(controller: scrollController, slivers: <Widget>[treeSliver]),
      ),
    );
    await tester.pump();
    expect(scrollController.position.pixels, 0.0);
    expect(scrollController.position.maxScrollExtent, 800.0);
    bool rowNeedsPaint(String row) {
      return find.text(row).evaluate().first.renderObject!.debugNeedsPaint;
    }

    expect(rowNeedsPaint('First'), isFalse);
    expect(rowNeedsPaint('Second'), isFalse);
    expect(rowNeedsPaint('Third'), isFalse);
    expect(find.text('gamma'), findsNothing); // Not visible

    // Change the scroll offset
    scrollController.jumpTo(200);
    await tester.pump();
    expect(find.text('First'), findsNothing);
    expect(rowNeedsPaint('Second'), isFalse);
    expect(rowNeedsPaint('Third'), isFalse);
    expect(rowNeedsPaint('gamma'), isFalse); // Now visible
  });
}
