// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group(WidgetOrderTraversalPolicy, () {
    testWidgets('Find the initial focus if there is none yet.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(),
          child: FocusScope(
            key: key1,
            child: Column(
              children: <Widget>[
                Focus(key: key2, child: SizedBox(key: key3, width: 100, height: 100)),
                Focus(key: key4, child: SizedBox(key: key5, width: 100, height: 100)),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key3));
      final Element secondChild = tester.element(find.byKey(key5));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      secondFocusNode.nextFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Find the initial focus if there is none yet and traversing backwards.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(),
          child: FocusScope(
            key: key1,
            child: Column(
              children: <Widget>[
                Focus(key: key2, child: SizedBox(key: key3, width: 100, height: 100)),
                Focus(key: key4, child: SizedBox(key: key5, width: 100, height: 100)),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key3));
      final Element secondChild = tester.element(find.byKey(key5));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isFalse);

      secondFocusNode.previousFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('focus traversal should work case 1', (WidgetTester tester) async {
      final FocusNode outer1 = FocusNode(debugLabel: 'outer1', skipTraversal: true);
      final FocusNode outer2 = FocusNode(debugLabel: 'outer2', skipTraversal: true);
      final FocusNode inner1 = FocusNode(debugLabel: 'inner1');
      final FocusNode inner2 = FocusNode(debugLabel: 'inner2');
      addTearDown(() {
        outer1.dispose();
        outer2.dispose();
        inner1.dispose();
        inner2.dispose();
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            child: Row(
              children: <Widget>[
                FocusScope(
                  child: Focus(
                    focusNode: outer1,
                    child: Focus(focusNode: inner1, child: const SizedBox(width: 10, height: 10)),
                  ),
                ),
                FocusScope(
                  child: Focus(
                    focusNode: outer2,
                    // Add a padding to ensure both Focus widgets have different
                    // sizes.
                    child: Padding(
                      padding: const EdgeInsets.all(5),
                      child: Focus(focusNode: inner2, child: const SizedBox(width: 10, height: 10)),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      expect(FocusManager.instance.primaryFocus, isNull);
      inner1.requestFocus();
      await tester.pump();
      expect(FocusManager.instance.primaryFocus, inner1);
      outer2.nextFocus();
      await tester.pump();
      expect(FocusManager.instance.primaryFocus, inner2);
    });

    testWidgets('focus traversal should work case 2', (WidgetTester tester) async {
      final FocusNode outer1 = FocusNode(debugLabel: 'outer1', skipTraversal: true);
      final FocusNode outer2 = FocusNode(debugLabel: 'outer2', skipTraversal: true);
      final FocusNode inner1 = FocusNode(debugLabel: 'inner1');
      final FocusNode inner2 = FocusNode(debugLabel: 'inner2');
      addTearDown(() {
        outer1.dispose();
        outer2.dispose();
        inner1.dispose();
        inner2.dispose();
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            child: Row(
              children: <Widget>[
                FocusScope(
                  child: Focus(
                    focusNode: outer1,
                    child: Focus(focusNode: inner1, child: const SizedBox(width: 10, height: 10)),
                  ),
                ),
                FocusScope(
                  child: Focus(
                    focusNode: outer2,
                    child: Focus(focusNode: inner2, child: const SizedBox(width: 10, height: 10)),
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      expect(FocusManager.instance.primaryFocus, isNull);
      inner1.requestFocus();
      await tester.pump();
      expect(FocusManager.instance.primaryFocus, inner1);
      outer2.nextFocus();
      await tester.pump();
      expect(FocusManager.instance.primaryFocus, inner2);
    });

    testWidgets('Move focus to next node.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      bool? focus1;
      bool? focus2;
      bool? focus3;
      bool? focus5;
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(),
          child: FocusScope(
            debugLabel: 'key1',
            key: key1,
            onFocusChange: (bool focus) => focus1 = focus,
            child: Column(
              children: <Widget>[
                FocusScope(
                  debugLabel: 'key2',
                  key: key2,
                  onFocusChange: (bool focus) => focus2 = focus,
                  child: Column(
                    children: <Widget>[
                      Focus(
                        debugLabel: 'key3',
                        key: key3,
                        onFocusChange: (bool focus) => focus3 = focus,
                        child: Container(key: key4),
                      ),
                      Focus(
                        debugLabel: 'key5',
                        key: key5,
                        onFocusChange: (bool focus) => focus5 = focus,
                        child: Container(key: key6),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key4));
      final Element secondChild = tester.element(find.byKey(key6));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      firstFocusNode.requestFocus();

      await tester.pump();

      expect(focus1, isTrue);
      expect(focus2, isTrue);
      expect(focus3, isTrue);
      expect(focus5, isNull);
      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);

      focus1 = null;
      focus2 = null;
      focus3 = null;
      focus5 = null;

      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isFalse);
      expect(focus5, isTrue);
      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      focus1 = null;
      focus2 = null;
      focus3 = null;
      focus5 = null;

      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isTrue);
      expect(focus5, isFalse);
      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);

      focus1 = null;
      focus2 = null;
      focus3 = null;
      focus5 = null;

      // Tests that can still move back to original node.
      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isFalse);
      expect(focus5, isTrue);
      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Move focus to previous node.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(),
          child: FocusScope(
            key: key1,
            child: Column(
              children: <Widget>[
                FocusScope(
                  key: key2,
                  child: Column(
                    children: <Widget>[
                      Focus(key: key3, child: Container(key: key4)),
                      Focus(key: key5, child: Container(key: key6)),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key4));
      final Element secondChild = tester.element(find.byKey(key6));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      secondFocusNode.requestFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);

      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      // Tests that can still move back to original node.
      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Move focus to next/previous node while skipping nodes in policy', (
      WidgetTester tester,
    ) async {
      final List<FocusNode> nodes = List<FocusNode>.generate(
        7,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: SkipAllButFirstAndLastPolicy(),
          child: Column(
            children: List<Widget>.generate(
              nodes.length,
              (int index) => Focus(focusNode: nodes[index], child: const SizedBox()),
            ),
          ),
        ),
      );

      nodes[2].requestFocus();
      await tester.pump();

      expect(nodes[2].hasPrimaryFocus, isTrue);

      primaryFocus!.nextFocus();
      await tester.pump();

      expect(nodes[6].hasPrimaryFocus, isTrue);

      primaryFocus!.previousFocus();
      await tester.pump();

      expect(nodes[0].hasPrimaryFocus, isTrue);
    });

    testWidgets('Find the initial focus when a route is pushed or popped.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode testNode1 = FocusNode(debugLabel: 'First Focus Node');
      addTearDown(testNode1.dispose);
      final FocusNode testNode2 = FocusNode(debugLabel: 'Second Focus Node');
      addTearDown(testNode2.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    key: key1,
                    focusNode: testNode1,
                    autofocus: true,
                    onPressed: () {
                      Navigator.of(context).push<void>(
                        MaterialPageRoute<void>(
                          builder: (BuildContext context) {
                            return Center(
                              child: ElevatedButton(
                                key: key2,
                                focusNode: testNode2,
                                autofocus: true,
                                onPressed: () {
                                  Navigator.of(context).pop();
                                },
                                child: const Text('Go Back'),
                              ),
                            );
                          },
                        ),
                      );
                    },
                    child: const Text('Go Forward'),
                  );
                },
              ),
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.text('Go Forward'));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      await tester.tap(find.text('Go Forward'));
      await tester.pumpAndSettle();

      final Element secondChild = tester.element(find.text('Go Back'));
      final FocusNode secondFocusNode = Focus.of(secondChild);

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);

      await tester.tap(find.text('Go Back'));
      await tester.pumpAndSettle();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Custom requestFocusCallback gets called on the next/previous focus.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode testNode1 = FocusNode(debugLabel: 'Focus Node');
      addTearDown(testNode1.dispose);
      bool calledCallback = false;

      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(
            requestFocusCallback: (
              FocusNode node, {
              double? alignment,
              ScrollPositionAlignmentPolicy? alignmentPolicy,
              Curve? curve,
              Duration? duration,
            }) {
              calledCallback = true;
            },
          ),
          child: FocusScope(
            debugLabel: 'key1',
            child: Focus(key: key1, focusNode: testNode1, child: Container()),
          ),
        ),
      );

      final Element element = tester.element(find.byKey(key1));
      final FocusNode scope = FocusScope.of(element);
      scope.nextFocus();

      await tester.pump();

      expect(calledCallback, isTrue);

      calledCallback = false;

      scope.previousFocus();
      await tester.pump();

      expect(calledCallback, isTrue);
    });
  });

  testWidgets('Nested navigator does not trap focus', (WidgetTester tester) async {
    final FocusNode node1 = FocusNode();
    addTearDown(node1.dispose);
    final FocusNode node2 = FocusNode();
    addTearDown(node2.dispose);
    final FocusNode node3 = FocusNode();
    addTearDown(node3.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(),
          child: FocusScope(
            child: Column(
              children: <Widget>[
                Focus(focusNode: node1, child: const SizedBox(width: 100, height: 100)),
                SizedBox(
                  width: 100,
                  height: 100,
                  child: Navigator(
                    pages: <Page<void>>[
                      MaterialPage<void>(
                        child: Focus(
                          focusNode: node2,
                          child: const SizedBox(width: 100, height: 100),
                        ),
                      ),
                    ],
                    onPopPage: (_, _) => false,
                  ),
                ),
                Focus(focusNode: node3, child: const SizedBox(width: 100, height: 100)),
              ],
            ),
          ),
        ),
      ),
    );

    node1.requestFocus();
    await tester.pump();

    expect(node1.hasFocus, isTrue);
    expect(node2.hasFocus, isFalse);
    expect(node3.hasFocus, isFalse);

    node1.nextFocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isTrue);
    expect(node3.hasFocus, isFalse);

    node2.nextFocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);
    expect(node3.hasFocus, isTrue);

    node3.nextFocus();
    await tester.pump();
    expect(node1.hasFocus, isTrue);
    expect(node2.hasFocus, isFalse);
    expect(node3.hasFocus, isFalse);

    node1.previousFocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);
    expect(node3.hasFocus, isTrue);

    node3.previousFocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isTrue);
    expect(node3.hasFocus, isFalse);

    node2.previousFocus();
    await tester.pump();
    expect(node1.hasFocus, isTrue);
    expect(node2.hasFocus, isFalse);
    expect(node3.hasFocus, isFalse);
  });

  group(ReadingOrderTraversalPolicy, () {
    testWidgets('Find the initial focus if there is none yet.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(),
          child: FocusScope(
            key: key1,
            child: Column(
              children: <Widget>[
                Focus(key: key2, child: SizedBox(key: key3, width: 100, height: 100)),
                Focus(key: key4, child: SizedBox(key: key5, width: 100, height: 100)),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key3));
      final Element secondChild = tester.element(find.byKey(key5));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      secondFocusNode.nextFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Move reading focus to next node.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      bool? focus1;
      bool? focus2;
      bool? focus3;
      bool? focus5;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'key1',
              key: key1,
              onFocusChange: (bool focus) => focus1 = focus,
              child: Column(
                children: <Widget>[
                  FocusScope(
                    debugLabel: 'key2',
                    key: key2,
                    onFocusChange: (bool focus) => focus2 = focus,
                    child: Row(
                      children: <Widget>[
                        Focus(
                          debugLabel: 'key3',
                          key: key3,
                          onFocusChange: (bool focus) => focus3 = focus,
                          child: Container(key: key4),
                        ),
                        Focus(
                          debugLabel: 'key5',
                          key: key5,
                          onFocusChange: (bool focus) => focus5 = focus,
                          child: Container(key: key6),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focus1 = null;
        focus2 = null;
        focus3 = null;
        focus5 = null;
      }

      final Element firstChild = tester.element(find.byKey(key4));
      final Element secondChild = tester.element(find.byKey(key6));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      firstFocusNode.requestFocus();

      await tester.pump();

      expect(focus1, isTrue);
      expect(focus2, isTrue);
      expect(focus3, isTrue);
      expect(focus5, isNull);
      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
      clear();

      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isFalse);
      expect(focus5, isTrue);
      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
      clear();

      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isTrue);
      expect(focus5, isFalse);
      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
      clear();

      // Tests that can still move back to original node.
      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(focus1, isNull);
      expect(focus2, isNull);
      expect(focus3, isFalse);
      expect(focus5, isTrue);
      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Requesting nextFocus on node focuses its descendant', (WidgetTester tester) async {
      for (final bool canRequestFocus in <bool>{true, false}) {
        final FocusNode node1 = FocusNode();
        final FocusNode node2 = FocusNode();
        addTearDown(() {
          node1.dispose();
          node2.dispose();
        });
        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: FocusTraversalGroup(
              policy: ReadingOrderTraversalPolicy(),
              child: FocusScope(
                child: Focus(
                  focusNode: node1,
                  canRequestFocus: canRequestFocus,
                  child: Focus(focusNode: node2, child: Container()),
                ),
              ),
            ),
          ),
        );

        final bool didFindNode = node1.nextFocus();
        await tester.pump();
        expect(didFindNode, isTrue);
        if (canRequestFocus) {
          expect(node1.hasPrimaryFocus, isTrue);
          expect(node2.hasPrimaryFocus, isFalse);
        } else {
          expect(node1.hasPrimaryFocus, isFalse);
          expect(node2.hasPrimaryFocus, isTrue);
        }
      }
    });

    testWidgets('Move reading focus to previous node.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(),
          child: FocusScope(
            key: key1,
            child: Column(
              children: <Widget>[
                FocusScope(
                  key: key2,
                  child: Column(
                    children: <Widget>[
                      Focus(key: key3, child: Container(key: key4)),
                      Focus(key: key5, child: Container(key: key6)),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key4));
      final Element secondChild = tester.element(find.byKey(key6));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      secondFocusNode.requestFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);

      Focus.of(firstChild).previousFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      // Tests that can still move back to original node.
      Focus.of(firstChild).nextFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(secondFocusNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Focus order is correct in the presence of different directionalities.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;
      final FocusScopeNode scopeNode = FocusScopeNode();
      addTearDown(scopeNode.dispose);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      Widget buildTest(TextDirection topDirection) {
        return Directionality(
          textDirection: topDirection,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(),
            child: FocusScope(
              node: scopeNode,
              child: Column(
                children: <Widget>[
                  Directionality(
                    textDirection: TextDirection.ltr,
                    child: Row(
                      children: <Widget>[
                        Focus(focusNode: nodes[0], child: const SizedBox(width: 10, height: 10)),
                        Focus(focusNode: nodes[1], child: const SizedBox(width: 10, height: 10)),
                        Focus(focusNode: nodes[2], child: const SizedBox(width: 10, height: 10)),
                      ],
                    ),
                  ),
                  Directionality(
                    textDirection: TextDirection.ltr,
                    child: Row(
                      children: <Widget>[
                        Directionality(
                          textDirection: TextDirection.rtl,
                          child: Focus(
                            focusNode: nodes[3],
                            child: const SizedBox(width: 10, height: 10),
                          ),
                        ),
                        Directionality(
                          textDirection: TextDirection.rtl,
                          child: Focus(
                            focusNode: nodes[4],
                            child: const SizedBox(width: 10, height: 10),
                          ),
                        ),
                        Directionality(
                          textDirection: TextDirection.ltr,
                          child: Focus(
                            focusNode: nodes[5],
                            child: const SizedBox(width: 10, height: 10),
                          ),
                        ),
                      ],
                    ),
                  ),
                  Row(
                    children: <Widget>[
                      Directionality(
                        textDirection: TextDirection.ltr,
                        child: Focus(
                          focusNode: nodes[6],
                          child: const SizedBox(width: 10, height: 10),
                        ),
                      ),
                      Directionality(
                        textDirection: TextDirection.rtl,
                        child: Focus(
                          focusNode: nodes[7],
                          child: const SizedBox(width: 10, height: 10),
                        ),
                      ),
                      Directionality(
                        textDirection: TextDirection.rtl,
                        child: Focus(
                          focusNode: nodes[8],
                          child: const SizedBox(width: 10, height: 10),
                        ),
                      ),
                      Directionality(
                        textDirection: TextDirection.ltr,
                        child: Focus(
                          focusNode: nodes[9],
                          child: const SizedBox(width: 10, height: 10),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildTest(TextDirection.rtl));

      // The last four *are* correct: the Row is sensitive to the directionality
      // too, so it swaps the positions of 7 and 8.
      final List<int> order = <int>[];
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(<int>[0, 1, 2, 4, 3, 5, 6, 7, 8, 9]));

      await tester.pumpWidget(buildTest(TextDirection.ltr));

      order.clear();
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(<int>[0, 1, 2, 4, 3, 5, 6, 8, 7, 9]));
    });

    testWidgets('Focus order is reading order regardless of widget order, even when overlapping.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(),
            child: Stack(
              alignment: Alignment.topLeft,
              children: List<Widget>.generate(nodeCount, (int index) {
                // Boxes that all have the same upper left origin corner.
                return Focus(
                  focusNode: nodes[index],
                  child: SizedBox(width: 10.0 * (index + 1), height: 10.0 * (index + 1)),
                );
              }),
            ),
          ),
        ),
      );

      final List<int> order = <int>[];
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(<int>[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]));

      // Concentric boxes.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(),
            child: Stack(
              alignment: Alignment.center,
              children: List<Widget>.generate(nodeCount, (int index) {
                return Focus(
                  focusNode: nodes[index],
                  child: SizedBox(width: 10.0 * (index + 1), height: 10.0 * (index + 1)),
                );
              }),
            ),
          ),
        ),
      );

      order.clear();
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(<int>[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]));

      // Stacked (vertically) and centered (horizontally, on each other)
      // widgets, not overlapping.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(),
            child: Stack(
              alignment: Alignment.center,
              children: List<Widget>.generate(nodeCount, (int index) {
                return Positioned(
                  top: 5.0 * index * (index + 1),
                  left: 5.0 * (9 - index),
                  child: Focus(
                    focusNode: nodes[index],
                    child: Container(
                      decoration: BoxDecoration(border: Border.all()),
                      width: 10.0 * (index + 1),
                      height: 10.0 * (index + 1),
                    ),
                  ),
                );
              }),
            ),
          ),
        ),
      );

      order.clear();
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(<int>[1, 2, 3, 4, 5, 6, 7, 8, 9, 0]));
    });

    testWidgets('Custom requestFocusCallback gets called on the next/previous focus.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode testNode1 = FocusNode(debugLabel: 'Focus Node');
      addTearDown(testNode1.dispose);
      bool calledCallback = false;

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: ReadingOrderTraversalPolicy(
              requestFocusCallback: (
                FocusNode node, {
                double? alignment,
                ScrollPositionAlignmentPolicy? alignmentPolicy,
                Curve? curve,
                Duration? duration,
              }) {
                calledCallback = true;
              },
            ),
            child: FocusScope(
              debugLabel: 'key1',
              child: Focus(key: key1, focusNode: testNode1, child: Container()),
            ),
          ),
        ),
      );

      final Element element = tester.element(find.byKey(key1));
      final FocusNode scope = FocusScope.of(element);
      scope.nextFocus();

      await tester.pump();

      expect(calledCallback, isTrue);

      calledCallback = false;

      scope.previousFocus();
      await tester.pump();

      expect(calledCallback, isTrue);
    });
  });

  group(OrderedTraversalPolicy, () {
    testWidgets('Find the initial focus if there is none yet.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: OrderedTraversalPolicy(secondary: ReadingOrderTraversalPolicy()),
          child: FocusScope(
            child: Column(
              children: <Widget>[
                FocusTraversalOrder(
                  order: const NumericFocusOrder(2),
                  child: Focus(child: SizedBox(key: key1, width: 100, height: 100)),
                ),
                FocusTraversalOrder(
                  order: const NumericFocusOrder(1),
                  child: Focus(child: SizedBox(key: key2, width: 100, height: 100)),
                ),
              ],
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.byKey(key1));
      final Element secondChild = tester.element(find.byKey(key2));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode secondFocusNode = Focus.of(secondChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      secondFocusNode.nextFocus();

      await tester.pump();

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Fall back to the secondary sort if no FocusTraversalOrder exists.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: FocusTraversalGroup(
            policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
            child: FocusScope(
              child: Row(
                children: List<Widget>.generate(
                  nodeCount,
                  (int index) =>
                      Focus(focusNode: nodes[index], child: const SizedBox(width: 10, height: 10)),
                ),
              ),
            ),
          ),
        ),
      );

      // Because it should be using widget order, this shouldn't be affected by
      // the directionality.
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        expect(nodes[i].hasPrimaryFocus, isTrue, reason: "node $i doesn't have focus, but should");
      }

      // Now check backwards.
      for (int i = nodeCount - 1; i > 0; --i) {
        nodes.first.previousFocus();
        await tester.pump();
        expect(
          nodes[i - 1].hasPrimaryFocus,
          isTrue,
          reason: "node ${i - 1} doesn't have focus, but should",
        );
      }
    });

    testWidgets('Move focus to next/previous node using numerical order.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
            child: FocusScope(
              child: Row(
                children: List<Widget>.generate(
                  nodeCount,
                  (int index) => FocusTraversalOrder(
                    order: NumericFocusOrder(nodeCount - index.toDouble()),
                    child: Focus(
                      focusNode: nodes[index],
                      child: const SizedBox(width: 10, height: 10),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      // The orders are assigned to be backwards from normal, so should go backwards.
      for (int i = nodeCount - 1; i >= 0; --i) {
        nodes.first.nextFocus();
        await tester.pump();
        expect(nodes[i].hasPrimaryFocus, isTrue, reason: "node $i doesn't have focus, but should");
      }

      // Now check backwards.
      for (int i = 1; i < nodeCount; ++i) {
        nodes.first.previousFocus();
        await tester.pump();
        expect(nodes[i].hasPrimaryFocus, isTrue, reason: "node $i doesn't have focus, but should");
      }
    });

    testWidgets('Move focus to next/previous node using lexical order.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;

      /// Generate ['J' ... 'A'];
      final List<String> keys = List<String>.generate(
        nodeCount,
        (int index) => String.fromCharCode('A'.codeUnits[0] + nodeCount - index - 1),
      );
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node ${keys[index]}'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
            child: FocusScope(
              child: Row(
                children: List<Widget>.generate(
                  nodeCount,
                  (int index) => FocusTraversalOrder(
                    order: LexicalFocusOrder(keys[index]),
                    child: Focus(
                      focusNode: nodes[index],
                      child: const SizedBox(width: 10, height: 10),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      // The orders are assigned to be backwards from normal, so should go backwards.
      for (int i = nodeCount - 1; i >= 0; --i) {
        nodes.first.nextFocus();
        await tester.pump();
        expect(nodes[i].hasPrimaryFocus, isTrue, reason: "node $i doesn't have focus, but should");
      }

      // Now check backwards.
      for (int i = 1; i < nodeCount; ++i) {
        nodes.first.previousFocus();
        await tester.pump();
        expect(nodes[i].hasPrimaryFocus, isTrue, reason: "node $i doesn't have focus, but should");
      }
    });

    testWidgets('Focus order is correct in the presence of FocusTraversalPolicyGroups.', (
      WidgetTester tester,
    ) async {
      const int nodeCount = 10;
      final FocusScopeNode scopeNode = FocusScopeNode();
      addTearDown(scopeNode.dispose);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        nodeCount,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              node: scopeNode,
              child: FocusTraversalGroup(
                policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
                child: Row(
                  children: <Widget>[
                    FocusTraversalOrder(
                      order: const NumericFocusOrder(0),
                      child: FocusTraversalGroup(
                        policy: WidgetOrderTraversalPolicy(),
                        child: Row(
                          children: <Widget>[
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(9),
                              child: Focus(
                                focusNode: nodes[9],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(8),
                              child: Focus(
                                focusNode: nodes[8],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(7),
                              child: Focus(
                                focusNode: nodes[7],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    FocusTraversalOrder(
                      order: const NumericFocusOrder(1),
                      child: FocusTraversalGroup(
                        policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
                        child: Row(
                          children: <Widget>[
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(4),
                              child: Focus(
                                focusNode: nodes[4],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(5),
                              child: Focus(
                                focusNode: nodes[5],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const NumericFocusOrder(6),
                              child: Focus(
                                focusNode: nodes[6],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    FocusTraversalOrder(
                      order: const NumericFocusOrder(2),
                      child: FocusTraversalGroup(
                        policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
                        child: Row(
                          children: <Widget>[
                            FocusTraversalOrder(
                              order: const LexicalFocusOrder('D'),
                              child: Focus(
                                focusNode: nodes[3],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const LexicalFocusOrder('C'),
                              child: Focus(
                                focusNode: nodes[2],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const LexicalFocusOrder('B'),
                              child: Focus(
                                focusNode: nodes[1],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                            FocusTraversalOrder(
                              order: const LexicalFocusOrder('A'),
                              child: Focus(
                                focusNode: nodes[0],
                                child: const SizedBox(width: 10, height: 10),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      );

      final List<int> expectedOrder = <int>[9, 8, 7, 4, 5, 6, 0, 1, 2, 3];
      final List<int> order = <int>[];
      for (int i = 0; i < nodeCount; ++i) {
        nodes.first.nextFocus();
        await tester.pump();
        order.add(nodes.indexOf(primaryFocus!));
      }
      expect(order, orderedEquals(expectedOrder));
    });

    testWidgets('Find the initial focus when a route is pushed or popped.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode testNode1 = FocusNode(debugLabel: 'First Focus Node');
      addTearDown(testNode1.dispose);
      final FocusNode testNode2 = FocusNode(debugLabel: 'Second Focus Node');
      addTearDown(testNode2.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: FocusTraversalGroup(
            policy: OrderedTraversalPolicy(secondary: WidgetOrderTraversalPolicy()),
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return FocusTraversalOrder(
                    order: const NumericFocusOrder(0),
                    child: ElevatedButton(
                      key: key1,
                      focusNode: testNode1,
                      autofocus: true,
                      onPressed: () {
                        Navigator.of(context).push<void>(
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return Center(
                                child: FocusTraversalOrder(
                                  order: const NumericFocusOrder(0),
                                  child: ElevatedButton(
                                    key: key2,
                                    focusNode: testNode2,
                                    autofocus: true,
                                    onPressed: () {
                                      Navigator.of(context).pop();
                                    },
                                    child: const Text('Go Back'),
                                  ),
                                ),
                              );
                            },
                          ),
                        );
                      },
                      child: const Text('Go Forward'),
                    ),
                  );
                },
              ),
            ),
          ),
        ),
      );

      final Element firstChild = tester.element(find.text('Go Forward'));
      final FocusNode firstFocusNode = Focus.of(firstChild);
      final FocusNode scope = Focus.of(firstChild).enclosingScope!;
      await tester.pump();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);

      await tester.tap(find.text('Go Forward'));
      await tester.pumpAndSettle();

      final Element secondChild = tester.element(find.text('Go Back'));
      final FocusNode secondFocusNode = Focus.of(secondChild);

      expect(firstFocusNode.hasFocus, isFalse);
      expect(secondFocusNode.hasFocus, isTrue);

      await tester.tap(find.text('Go Back'));
      await tester.pumpAndSettle();

      expect(firstFocusNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Custom requestFocusCallback gets called on the next/previous focus.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode testNode1 = FocusNode(debugLabel: 'Focus Node');
      addTearDown(testNode1.dispose);
      bool calledCallback = false;

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: OrderedTraversalPolicy(
              requestFocusCallback: (
                FocusNode node, {
                double? alignment,
                ScrollPositionAlignmentPolicy? alignmentPolicy,
                Curve? curve,
                Duration? duration,
              }) {
                calledCallback = true;
              },
            ),
            child: FocusScope(
              debugLabel: 'key1',
              child: Focus(key: key1, focusNode: testNode1, child: Container()),
            ),
          ),
        ),
      );

      final Element element = tester.element(find.byKey(key1));
      final FocusNode scope = FocusScope.of(element);
      scope.nextFocus();

      await tester.pump();

      expect(calledCallback, isTrue);

      calledCallback = false;

      scope.previousFocus();
      await tester.pump();

      expect(calledCallback, isTrue);
    });
  });

  group(DirectionalFocusTraversalPolicyMixin, () {
    testWidgets('Move focus in all directions.', (WidgetTester tester) async {
      final GlobalKey upperLeftKey = GlobalKey(debugLabel: 'upperLeftKey');
      final GlobalKey upperRightKey = GlobalKey(debugLabel: 'upperRightKey');
      final GlobalKey lowerLeftKey = GlobalKey(debugLabel: 'lowerLeftKey');
      final GlobalKey lowerRightKey = GlobalKey(debugLabel: 'lowerRightKey');
      bool? focusUpperLeft;
      bool? focusUpperRight;
      bool? focusLowerLeft;
      bool? focusLowerRight;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'Scope',
              child: Column(
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Focus(
                        debugLabel: 'upperLeft',
                        onFocusChange: (bool focus) => focusUpperLeft = focus,
                        child: SizedBox(width: 100, height: 100, key: upperLeftKey),
                      ),
                      Focus(
                        debugLabel: 'upperRight',
                        onFocusChange: (bool focus) => focusUpperRight = focus,
                        child: SizedBox(width: 100, height: 100, key: upperRightKey),
                      ),
                    ],
                  ),
                  Row(
                    children: <Widget>[
                      Focus(
                        debugLabel: 'lowerLeft',
                        onFocusChange: (bool focus) => focusLowerLeft = focus,
                        child: SizedBox(width: 100, height: 100, key: lowerLeftKey),
                      ),
                      Focus(
                        debugLabel: 'lowerRight',
                        onFocusChange: (bool focus) => focusLowerRight = focus,
                        child: SizedBox(width: 100, height: 100, key: lowerRightKey),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focusUpperLeft = null;
        focusUpperRight = null;
        focusLowerLeft = null;
        focusLowerRight = null;
      }

      final FocusNode upperLeftNode = Focus.of(tester.element(find.byKey(upperLeftKey)));
      final FocusNode upperRightNode = Focus.of(tester.element(find.byKey(upperRightKey)));
      final FocusNode lowerLeftNode = Focus.of(tester.element(find.byKey(lowerLeftKey)));
      final FocusNode lowerRightNode = Focus.of(tester.element(find.byKey(lowerRightKey)));
      final FocusNode scope = upperLeftNode.enclosingScope!;
      upperLeftNode.requestFocus();

      await tester.pump();

      expect(focusUpperLeft, isTrue);
      expect(focusUpperRight, isNull);
      expect(focusLowerLeft, isNull);
      expect(focusLowerRight, isNull);
      expect(upperLeftNode.hasFocus, isTrue);
      expect(upperRightNode.hasFocus, isFalse);
      expect(lowerLeftNode.hasFocus, isFalse);
      expect(lowerRightNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
      clear();

      expect(scope.focusInDirection(TraversalDirection.right), isTrue);

      await tester.pump();

      expect(focusUpperLeft, isFalse);
      expect(focusUpperRight, isTrue);
      expect(focusLowerLeft, isNull);
      expect(focusLowerRight, isNull);
      expect(upperLeftNode.hasFocus, isFalse);
      expect(upperRightNode.hasFocus, isTrue);
      expect(lowerLeftNode.hasFocus, isFalse);
      expect(lowerRightNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);

      await tester.pump();

      expect(focusUpperLeft, isNull);
      expect(focusUpperRight, isFalse);
      expect(focusLowerLeft, isNull);
      expect(focusLowerRight, isTrue);
      expect(upperLeftNode.hasFocus, isFalse);
      expect(upperRightNode.hasFocus, isFalse);
      expect(lowerLeftNode.hasFocus, isFalse);
      expect(lowerRightNode.hasFocus, isTrue);
      expect(scope.hasFocus, isTrue);
      clear();

      expect(scope.focusInDirection(TraversalDirection.left), isTrue);

      await tester.pump();

      expect(focusUpperLeft, isNull);
      expect(focusUpperRight, isNull);
      expect(focusLowerLeft, isTrue);
      expect(focusLowerRight, isFalse);
      expect(upperLeftNode.hasFocus, isFalse);
      expect(upperRightNode.hasFocus, isFalse);
      expect(lowerLeftNode.hasFocus, isTrue);
      expect(lowerRightNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);

      await tester.pump();

      expect(focusUpperLeft, isTrue);
      expect(focusUpperRight, isNull);
      expect(focusLowerLeft, isFalse);
      expect(focusLowerRight, isNull);
      expect(upperLeftNode.hasFocus, isTrue);
      expect(upperRightNode.hasFocus, isFalse);
      expect(lowerLeftNode.hasFocus, isFalse);
      expect(lowerRightNode.hasFocus, isFalse);
      expect(scope.hasFocus, isTrue);
    });

    testWidgets('Directional focus avoids hysteresis.', (WidgetTester tester) async {
      List<bool?> focus = List<bool?>.generate(6, (int _) => null);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        6,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      Focus makeFocus(int index) {
        return Focus(
          debugLabel: '[$index]',
          focusNode: nodes[index],
          onFocusChange: (bool isFocused) => focus[index] = isFocused,
          child: const SizedBox(width: 100, height: 100),
        );
      }

      /// Layout is:
      ///          [0]
      ///       [1]   [2]
      ///    [3]   [4]   [5]
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'Scope',
              child: Column(
                children: <Widget>[
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[makeFocus(0)],
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[makeFocus(1), makeFocus(2)],
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[makeFocus(3), makeFocus(4), makeFocus(5)],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focus = List<bool?>.generate(focus.length, (int _) => null);
      }

      final FocusNode scope = nodes[0].enclosingScope!;
      nodes[4].requestFocus();

      // Test to make sure that the same path is followed backwards and forwards.
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, true, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[null, null, true, null, false, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[true, null, false, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[false, null, true, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, false, null, true, null]));
      clear();

      // Make sure that moving in a different axis clears the history.
      expect(scope.focusInDirection(TraversalDirection.left), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, true, false, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[null, true, null, false, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[true, false, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();

      expect(focus, orderedEquals(<bool?>[false, true, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, false, null, true, null, null]));
      clear();
    });

    testWidgets('Directional prefers the closest node even on irregular grids', (
      WidgetTester tester,
    ) async {
      const int cols = 3;
      const int rows = 3;
      List<bool?> focus = List<bool?>.generate(rows * cols, (int _) => null);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        rows * cols,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      Widget makeFocus(int row, int col) {
        final int index = row * rows + col;
        return Focus(
          focusNode: nodes[index],
          onFocusChange: (bool isFocused) => focus[index] = isFocused,
          child: Container(
            // Make some of the items a different size to test the code that
            // checks for the closest node.
            width: index == 3 ? 150 : 100,
            height: index == 1 ? 150 : 100,
            color: Colors.primaries[index],
            child: Text('[$row, $col]'),
          ),
        );
      }

      /// Layout is:
      ///           [0, 1]
      ///    [0, 0] [    ] [0, 2]
      ///    [  1,  0 ] [1, 1] [1, 2]
      ///    [2, 0] [2, 1] [2, 2]
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'Scope',
              child: Column(
                children: <Widget>[
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    crossAxisAlignment: CrossAxisAlignment.end,
                    children: <Widget>[makeFocus(0, 0), makeFocus(0, 1), makeFocus(0, 2)],
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[makeFocus(1, 0), makeFocus(1, 1), makeFocus(1, 2)],
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[makeFocus(2, 0), makeFocus(2, 1), makeFocus(2, 2)],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focus = List<bool?>.generate(focus.length, (int _) => null);
      }

      final FocusNode scope = nodes[0].enclosingScope!;

      // Go down the center column and make sure that the focus stays in that
      // column, even though the second row is irregular.
      nodes[1].requestFocus();
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, true, null, null, null, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, false, null, null, true, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, false, null, null, true, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isFalse);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null, null, null, null]));
      clear();

      // Go back up the right column and make sure that the focus stays in that
      // column, even though the second row is irregular.
      expect(scope.focusInDirection(TraversalDirection.right), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null, null, false, true]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, null, true, null, null, false]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, true, null, null, false, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.up), isFalse);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null, null, null, null]));
      clear();

      // Go left on the top row and make sure that the focus stays in that
      // row, even though the second column is irregular.
      expect(scope.focusInDirection(TraversalDirection.left), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, true, false, null, null, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.left), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[true, false, null, null, null, null, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.left), isFalse);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null, null, null, null]));
      clear();
    });

    testWidgets('Closest vertical is picked when only out of band items are considered', (
      WidgetTester tester,
    ) async {
      const int rows = 4;
      List<bool?> focus = List<bool?>.generate(rows, (int _) => null);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        rows,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      Widget makeFocus(int row) {
        return Padding(
          padding: EdgeInsetsDirectional.only(end: row != 0 ? 110.0 : 0),
          child: Focus(
            focusNode: nodes[row],
            onFocusChange: (bool isFocused) => focus[row] = isFocused,
            child: Container(
              width: row == 1 ? 150 : 100,
              height: 100,
              color: Colors.primaries[row],
              child: Text('[$row]'),
            ),
          ),
        );
      }

      /// Layout is:
      ///           [0]
      ///    [  1]
      ///     [ 2]
      ///     [ 3]
      ///
      /// The important feature is that nothing is in the vertical band defined
      /// by widget [0]. We want it to traverse to 1, 2, 3 in order, even though
      /// the center of [2] is horizontally closer to the vertical axis of [0]'s
      /// center than [1]'s.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'Scope',
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.end,
                children: <Widget>[makeFocus(0), makeFocus(1), makeFocus(2), makeFocus(3)],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focus = List<bool?>.generate(focus.length, (int _) => null);
      }

      final FocusNode scope = nodes[0].enclosingScope!;

      // Go down the column and make sure that the focus goes to the next
      // closest one.
      nodes[0].requestFocus();
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[true, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[false, true, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, false, true, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, false, true]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.down), isFalse);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null]));
      clear();
    });

    testWidgets('Closest horizontal is picked when only out of band items are considered', (
      WidgetTester tester,
    ) async {
      const int cols = 4;
      List<bool?> focus = List<bool?>.generate(cols, (int _) => null);
      final List<FocusNode> nodes = List<FocusNode>.generate(
        cols,
        (int index) => FocusNode(debugLabel: 'Node $index'),
      );
      addTearDown(() {
        for (final FocusNode node in nodes) {
          node.dispose();
        }
      });

      Widget makeFocus(int col) {
        return Padding(
          padding: EdgeInsetsDirectional.only(top: col != 0 ? 110.0 : 0),
          child: Focus(
            focusNode: nodes[col],
            onFocusChange: (bool isFocused) => focus[col] = isFocused,
            child: Container(
              width: 100,
              height: col == 1 ? 150 : 100,
              color: Colors.primaries[col],
              child: Text('[$col]'),
            ),
          ),
        );
      }

      /// Layout is:
      ///    [0]
      ///        [ ][2][3]
      ///        [1]
      /// ([ ] is part of [1], [1] is just taller than [2] and [3]).
      ///
      /// The important feature is that nothing is in the horizontal band
      /// defined by widget [0]. We want it to traverse to 1, 2, 3 in order,
      /// even though the center of [2] is vertically closer to the horizontal
      /// axis of [0]'s center than [1]'s.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'Scope',
              child: Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[makeFocus(0), makeFocus(1), makeFocus(2), makeFocus(3)],
              ),
            ),
          ),
        ),
      );

      void clear() {
        focus = List<bool?>.generate(focus.length, (int _) => null);
      }

      final FocusNode scope = nodes[0].enclosingScope!;

      // Go down the row and make sure that the focus goes to the next
      // closest one.
      nodes[0].requestFocus();
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[true, null, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.right), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[false, true, null, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.right), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, false, true, null]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.right), isTrue);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, false, true]));
      clear();

      expect(scope.focusInDirection(TraversalDirection.right), isFalse);
      await tester.pump();
      expect(focus, orderedEquals(<bool?>[null, null, null, null]));
      clear();
    });

    testWidgets('Can find first focus in all directions.', (WidgetTester tester) async {
      final GlobalKey upperLeftKey = GlobalKey(debugLabel: 'upperLeftKey');
      final GlobalKey upperRightKey = GlobalKey(debugLabel: 'upperRightKey');
      final GlobalKey lowerLeftKey = GlobalKey(debugLabel: 'lowerLeftKey');

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FocusTraversalGroup(
            policy: WidgetOrderTraversalPolicy(),
            child: FocusScope(
              debugLabel: 'scope',
              child: Column(
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Focus(
                        debugLabel: 'upperLeft',
                        child: SizedBox(width: 100, height: 100, key: upperLeftKey),
                      ),
                      Focus(
                        debugLabel: 'upperRight',
                        child: SizedBox(width: 100, height: 100, key: upperRightKey),
                      ),
                    ],
                  ),
                  Row(
                    children: <Widget>[
                      Focus(
                        debugLabel: 'lowerLeft',
                        child: SizedBox(width: 100, height: 100, key: lowerLeftKey),
                      ),
                      const Focus(
                        debugLabel: 'lowerRight',
                        child: SizedBox(width: 100, height: 100),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      final FocusNode upperLeftNode = Focus.of(tester.element(find.byKey(upperLeftKey)));
      final FocusNode upperRightNode = Focus.of(tester.element(find.byKey(upperRightKey)));
      final FocusNode lowerLeftNode = Focus.of(tester.element(find.byKey(lowerLeftKey)));
      final FocusNode scope = upperLeftNode.enclosingScope!;

      await tester.pump();

      final FocusTraversalPolicy policy = FocusTraversalGroup.of(upperLeftKey.currentContext!);

      expect(policy.findFirstFocusInDirection(scope, TraversalDirection.up), equals(lowerLeftNode));
      expect(
        policy.findFirstFocusInDirection(scope, TraversalDirection.down),
        equals(upperLeftNode),
      );
      expect(
        policy.findFirstFocusInDirection(scope, TraversalDirection.left),
        equals(upperRightNode),
      );
      expect(
        policy.findFirstFocusInDirection(scope, TraversalDirection.right),
        equals(upperLeftNode),
      );
    });

    testWidgets('Can find focus when policy data dirty', (WidgetTester tester) async {
      final FocusNode focusTop = FocusNode(debugLabel: 'top');
      addTearDown(focusTop.dispose);
      final FocusNode focusCenter = FocusNode(debugLabel: 'center');
      addTearDown(focusCenter.dispose);
      final FocusNode focusBottom = FocusNode(debugLabel: 'bottom');
      addTearDown(focusBottom.dispose);

      final FocusTraversalPolicy policy = ReadingOrderTraversalPolicy();
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: policy,
          child: FocusScope(
            debugLabel: 'Scope',
            child: Column(
              children: <Widget>[
                Focus(focusNode: focusTop, child: const SizedBox(width: 100, height: 100)),
                Focus(focusNode: focusCenter, child: const SizedBox(width: 100, height: 100)),
                Focus(focusNode: focusBottom, child: const SizedBox(width: 100, height: 100)),
              ],
            ),
          ),
        ),
      );

      focusTop.requestFocus();
      final FocusNode scope = focusTop.enclosingScope!;

      scope.focusInDirection(TraversalDirection.down);
      scope.focusInDirection(TraversalDirection.down);

      await tester.pump();
      expect(focusBottom.hasFocus, isTrue);

      // Remove center focus node.
      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: policy,
          child: FocusScope(
            debugLabel: 'Scope',
            child: Column(
              children: <Widget>[
                Focus(focusNode: focusTop, child: const SizedBox(width: 100, height: 100)),
                Focus(focusNode: focusBottom, child: const SizedBox(width: 100, height: 100)),
              ],
            ),
          ),
        ),
      );

      expect(focusBottom.hasFocus, isTrue);
      scope.focusInDirection(TraversalDirection.up);
      await tester.pump();

      expect(focusCenter.hasFocus, isFalse);
      expect(focusTop.hasFocus, isTrue);
    });

    testWidgets(
      'Focus traversal actions are invoked when shortcuts are used.',
      (WidgetTester tester) async {
        final GlobalKey upperLeftKey = GlobalKey(debugLabel: 'upperLeftKey');
        final GlobalKey upperRightKey = GlobalKey(debugLabel: 'upperRightKey');
        final GlobalKey lowerLeftKey = GlobalKey(debugLabel: 'lowerLeftKey');
        final GlobalKey lowerRightKey = GlobalKey(debugLabel: 'lowerRightKey');

        await tester.pumpWidget(
          WidgetsApp(
            color: const Color(0xFFFFFFFF),
            onGenerateRoute: (RouteSettings settings) {
              return TestRoute(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: FocusScope(
                    debugLabel: 'scope',
                    child: Column(
                      children: <Widget>[
                        Row(
                          children: <Widget>[
                            Focus(
                              autofocus: true,
                              debugLabel: 'upperLeft',
                              child: SizedBox(width: 100, height: 100, key: upperLeftKey),
                            ),
                            Focus(
                              debugLabel: 'upperRight',
                              child: SizedBox(width: 100, height: 100, key: upperRightKey),
                            ),
                          ],
                        ),
                        Row(
                          children: <Widget>[
                            Focus(
                              debugLabel: 'lowerLeft',
                              child: SizedBox(width: 100, height: 100, key: lowerLeftKey),
                            ),
                            Focus(
                              debugLabel: 'lowerRight',
                              child: SizedBox(width: 100, height: 100, key: lowerRightKey),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          ),
        );

        expect(Focus.of(upperLeftKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(upperRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(lowerLeftKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(lowerRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(upperLeftKey.currentContext!).hasPrimaryFocus, isTrue);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        expect(Focus.of(lowerRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        expect(Focus.of(lowerLeftKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        expect(Focus.of(upperRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        expect(Focus.of(upperLeftKey.currentContext!).hasPrimaryFocus, isTrue);

        // Traverse in a direction
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        expect(Focus.of(upperRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        expect(Focus.of(lowerRightKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        expect(Focus.of(lowerLeftKey.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        expect(Focus.of(upperLeftKey.currentContext!).hasPrimaryFocus, isTrue);
      },
      // https://github.com/flutter/flutter/issues/35347
      skip: isBrowser,
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets(
      'Focus traversal actions works when current focus skip traversal',
      (WidgetTester tester) async {
        final GlobalKey key1 = GlobalKey(debugLabel: 'key1');
        final GlobalKey key2 = GlobalKey(debugLabel: 'key2');
        final GlobalKey key3 = GlobalKey(debugLabel: 'key3');

        await tester.pumpWidget(
          WidgetsApp(
            color: const Color(0xFFFFFFFF),
            onGenerateRoute: (RouteSettings settings) {
              return TestRoute(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: FocusScope(
                    debugLabel: 'scope',
                    child: Column(
                      children: <Widget>[
                        Row(
                          children: <Widget>[
                            Focus(
                              autofocus: true,
                              skipTraversal: true,
                              debugLabel: '1',
                              child: SizedBox(width: 100, height: 100, key: key1),
                            ),
                            Focus(
                              debugLabel: '2',
                              child: SizedBox(width: 100, height: 100, key: key2),
                            ),
                            Focus(
                              debugLabel: '3',
                              child: SizedBox(width: 100, height: 100, key: key3),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          ),
        );

        expect(Focus.of(key1.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(key2.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(key3.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        // Skips key 1 because it skips traversal.
        expect(Focus.of(key2.currentContext!).hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        expect(Focus.of(key3.currentContext!).hasPrimaryFocus, isTrue);
      },
      // https://github.com/flutter/flutter/issues/35347
      skip: isBrowser,
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets(
      'Focus traversal inside a vertical scrollable scrolls to stay visible.',
      (WidgetTester tester) async {
        final List<int> items = List<int>.generate(11, (int index) => index).toList();
        final List<FocusNode> nodes =
            List<FocusNode>.generate(
              11,
              (int index) => FocusNode(debugLabel: 'Item ${index + 1}'),
            ).toList();
        addTearDown(() {
          for (final FocusNode node in nodes) {
            node.dispose();
          }
        });
        final FocusNode topNode = FocusNode(debugLabel: 'Header');
        addTearDown(topNode.dispose);
        final FocusNode bottomNode = FocusNode(debugLabel: 'Footer');
        addTearDown(bottomNode.dispose);
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Column(
              children: <Widget>[
                Focus(focusNode: topNode, child: Container(height: 100)),
                Expanded(
                  child: ListView(
                    controller: controller,
                    children:
                        items.map<Widget>((int item) {
                          return Focus(focusNode: nodes[item], child: Container(height: 100));
                        }).toList(),
                  ),
                ),
                Focus(focusNode: bottomNode, child: Container(height: 100)),
              ],
            ),
          ),
        );

        // Start at the top
        expect(controller.offset, equals(0.0));
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pump();
        expect(topNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Enter the list.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pump();
        expect(nodes[0].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Go down until we hit the bottom of the visible area.
        for (int i = 1; i <= 4; ++i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
          await tester.pump();
          expect(controller.offset, equals(0.0), reason: 'Focusing item $i caused a scroll');
        }

        // Now keep going down, and the scrollable should scroll automatically.
        for (int i = 5; i <= 10; ++i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
          await tester.pump();
          final double expectedOffset = 100.0 * (i - 5) + 200.0;
          expect(
            controller.offset,
            equals(expectedOffset),
            reason: "Focusing item $i didn't cause a scroll to $expectedOffset",
          );
        }

        // Now go one more, and see that the footer gets focused.

        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pump();
        expect(bottomNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(100.0 * (10 - 5) + 200.0));

        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pump();
        expect(nodes[10].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(100.0 * (10 - 5) + 200.0));

        // Now reverse directions and go back to the top.

        // These should not cause a scroll.
        final double lowestOffset = controller.offset;
        for (int i = 10; i >= 8; --i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
          await tester.pump();
          expect(
            controller.offset,
            equals(lowestOffset),
            reason: 'Focusing item $i caused a scroll',
          );
        }

        // These should all cause a scroll.
        for (int i = 7; i >= 1; --i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
          await tester.pump();
          final double expectedOffset = 100.0 * (i - 1);
          expect(
            controller.offset,
            equals(expectedOffset),
            reason: "Focusing item $i didn't cause a scroll",
          );
        }

        // Back at the top.
        expect(nodes[0].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Now we jump to the header.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pump();
        expect(topNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));
      },
      // https://github.com/flutter/flutter/issues/35347
      skip: isBrowser,
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets(
      'Focus traversal inside a horizontal scrollable scrolls to stay visible.',
      (WidgetTester tester) async {
        final List<int> items = List<int>.generate(11, (int index) => index).toList();
        final List<FocusNode> nodes =
            List<FocusNode>.generate(
              11,
              (int index) => FocusNode(debugLabel: 'Item ${index + 1}'),
            ).toList();
        addTearDown(() {
          for (final FocusNode node in nodes) {
            node.dispose();
          }
        });
        final FocusNode leftNode = FocusNode(debugLabel: 'Left Side');
        addTearDown(leftNode.dispose);
        final FocusNode rightNode = FocusNode(debugLabel: 'Right Side');
        addTearDown(rightNode.dispose);
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Row(
              children: <Widget>[
                Focus(focusNode: leftNode, child: Container(width: 100)),
                Expanded(
                  child: ListView(
                    scrollDirection: Axis.horizontal,
                    controller: controller,
                    children:
                        items.map<Widget>((int item) {
                          return Focus(focusNode: nodes[item], child: Container(width: 100));
                        }).toList(),
                  ),
                ),
                Focus(focusNode: rightNode, child: Container(width: 100)),
              ],
            ),
          ),
        );

        // Start at the right
        expect(controller.offset, equals(0.0));
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pump();
        expect(leftNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Enter the list.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pump();
        expect(nodes[0].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Go right until we hit the right of the visible area.
        for (int i = 1; i <= 6; ++i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
          await tester.pump();
          expect(controller.offset, equals(0.0), reason: 'Focusing item $i caused a scroll');
        }

        // Now keep going right, and the scrollable should scroll automatically.
        for (int i = 7; i <= 10; ++i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
          await tester.pump();
          final double expectedOffset = 100.0 * (i - 5);
          expect(
            controller.offset,
            equals(expectedOffset),
            reason: "Focusing item $i didn't cause a scroll to $expectedOffset",
          );
        }

        // Now go one more, and see that the right edge gets focused.

        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pump();
        expect(rightNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(100.0 * 5));

        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pump();
        expect(nodes[10].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(100.0 * 5));

        // Now reverse directions and go back to the left.

        // These should not cause a scroll.
        final double lowestOffset = controller.offset;
        for (int i = 10; i >= 7; --i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
          await tester.pump();
          expect(
            controller.offset,
            equals(lowestOffset),
            reason: 'Focusing item $i caused a scroll',
          );
        }

        // These should all cause a scroll.
        for (int i = 6; i >= 1; --i) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
          await tester.pump();
          final double expectedOffset = 100.0 * (i - 1);
          expect(
            controller.offset,
            equals(expectedOffset),
            reason: "Focusing item $i didn't cause a scroll",
          );
        }

        // Back at the left side of the scrollable.
        expect(nodes[0].hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));

        // Now we jump to the left edge of the app.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pump();
        expect(leftNode.hasPrimaryFocus, isTrue);
        expect(controller.offset, equals(0.0));
      },
      // https://github.com/flutter/flutter/issues/35347
      skip: isBrowser,
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets(
      'Arrow focus traversal actions can be re-enabled for text fields.',
      (WidgetTester tester) async {
        final GlobalKey upperLeftKey = GlobalKey(debugLabel: 'upperLeftKey');
        final GlobalKey upperRightKey = GlobalKey(debugLabel: 'upperRightKey');
        final GlobalKey lowerLeftKey = GlobalKey(debugLabel: 'lowerLeftKey');
        final GlobalKey lowerRightKey = GlobalKey(debugLabel: 'lowerRightKey');

        final TextEditingController controller1 = TextEditingController();
        addTearDown(controller1.dispose);
        final TextEditingController controller2 = TextEditingController();
        addTearDown(controller2.dispose);
        final TextEditingController controller3 = TextEditingController();
        addTearDown(controller3.dispose);
        final TextEditingController controller4 = TextEditingController();
        addTearDown(controller4.dispose);

        final FocusNode focusNodeUpperLeft = FocusNode(debugLabel: 'upperLeft');
        addTearDown(focusNodeUpperLeft.dispose);
        final FocusNode focusNodeUpperRight = FocusNode(debugLabel: 'upperRight');
        addTearDown(focusNodeUpperRight.dispose);
        final FocusNode focusNodeLowerLeft = FocusNode(debugLabel: 'lowerLeft');
        addTearDown(focusNodeLowerLeft.dispose);
        final FocusNode focusNodeLowerRight = FocusNode(debugLabel: 'lowerRight');
        addTearDown(focusNodeLowerRight.dispose);

        Widget generatetestWidgets(bool ignoreTextFields) {
          final Map<ShortcutActivator, Intent> shortcuts = <ShortcutActivator, Intent>{
            const SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(
              TraversalDirection.left,
              ignoreTextFields: ignoreTextFields,
            ),
            const SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(
              TraversalDirection.right,
              ignoreTextFields: ignoreTextFields,
            ),
            const SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(
              TraversalDirection.down,
              ignoreTextFields: ignoreTextFields,
            ),
            const SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(
              TraversalDirection.up,
              ignoreTextFields: ignoreTextFields,
            ),
          };

          return MaterialApp(
            home: Shortcuts(
              shortcuts: shortcuts,
              child: FocusScope(
                debugLabel: 'scope',
                child: Column(
                  children: <Widget>[
                    Row(
                      children: <Widget>[
                        SizedBox(
                          width: 100,
                          height: 100,
                          child: EditableText(
                            autofocus: true,
                            key: upperLeftKey,
                            controller: controller1,
                            focusNode: focusNodeUpperLeft,
                            cursorColor: const Color(0xffffffff),
                            backgroundCursorColor: const Color(0xff808080),
                            style: const TextStyle(),
                          ),
                        ),
                        SizedBox(
                          width: 100,
                          height: 100,
                          child: EditableText(
                            key: upperRightKey,
                            controller: controller2,
                            focusNode: focusNodeUpperRight,
                            cursorColor: const Color(0xffffffff),
                            backgroundCursorColor: const Color(0xff808080),
                            style: const TextStyle(),
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: <Widget>[
                        SizedBox(
                          width: 100,
                          height: 100,
                          child: EditableText(
                            key: lowerLeftKey,
                            controller: controller3,
                            focusNode: focusNodeLowerLeft,
                            cursorColor: const Color(0xffffffff),
                            backgroundCursorColor: const Color(0xff808080),
                            style: const TextStyle(),
                          ),
                        ),
                        SizedBox(
                          width: 100,
                          height: 100,
                          child: EditableText(
                            key: lowerRightKey,
                            controller: controller4,
                            focusNode: focusNodeLowerRight,
                            cursorColor: const Color(0xffffffff),
                            backgroundCursorColor: const Color(0xff808080),
                            style: const TextStyle(),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
          );
        }

        await tester.pumpWidget(generatetestWidgets(false));

        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        expect(focusNodeUpperRight.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        expect(focusNodeLowerRight.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        expect(focusNodeLowerLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);

        await tester.pumpWidget(generatetestWidgets(true));

        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        expect(focusNodeUpperRight.hasPrimaryFocus, isFalse);
        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        expect(focusNodeLowerRight.hasPrimaryFocus, isFalse);
        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        expect(focusNodeLowerLeft.hasPrimaryFocus, isFalse);
        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        expect(focusNodeUpperLeft.hasPrimaryFocus, isTrue);
      },
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets(
      'Focus traversal does not break when no focusable is available on a MaterialApp',
      (WidgetTester tester) async {
        final List<Object> events = <Object>[];

        await tester.pumpWidget(MaterialApp(home: Container()));

        HardwareKeyboard.instance.addHandler((KeyEvent event) {
          events.add(event);
          return true;
        });

        await tester.idle();
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.idle();

        expect(events.length, 2);
      },
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets('Focus traversal does not throw when no focusable is available in a group', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        const MaterialApp(home: Scaffold(body: ListTile(title: Text('title')))),
      );
      final FocusNode? initialFocus = primaryFocus;
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(primaryFocus, equals(initialFocus));
    });

    testWidgets(
      'Focus traversal does not break when no focusable is available on a WidgetsApp',
      (WidgetTester tester) async {
        final List<KeyEvent> events = <KeyEvent>[];

        await tester.pumpWidget(
          WidgetsApp(
            color: Colors.white,
            onGenerateRoute:
                (RouteSettings settings) => PageRouteBuilder<void>(
                  settings: settings,
                  pageBuilder: (
                    BuildContext context,
                    Animation<double> animation1,
                    Animation<double> animation2,
                  ) {
                    return const Placeholder();
                  },
                ),
          ),
        );

        HardwareKeyboard.instance.addHandler((KeyEvent event) {
          events.add(event);
          return true;
        });

        await tester.idle();
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.idle();

        expect(events.length, 2);
      },
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets('Custom requestFocusCallback gets called on focusInDirection up/down/left/right.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode testNode1 = FocusNode(debugLabel: 'Focus Node');
      addTearDown(testNode1.dispose);
      bool calledCallback = false;

      await tester.pumpWidget(
        FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(
            requestFocusCallback: (
              FocusNode node, {
              double? alignment,
              ScrollPositionAlignmentPolicy? alignmentPolicy,
              Curve? curve,
              Duration? duration,
            }) {
              calledCallback = true;
            },
          ),
          child: FocusScope(
            debugLabel: 'key1',
            child: Focus(key: key1, focusNode: testNode1, child: Container()),
          ),
        ),
      );

      final Element element = tester.element(find.byKey(key1));
      final FocusNode scope = FocusScope.of(element);
      scope.focusInDirection(TraversalDirection.up);

      await tester.pump();

      expect(calledCallback, isTrue);

      calledCallback = false;

      scope.focusInDirection(TraversalDirection.down);
      await tester.pump();

      expect(calledCallback, isTrue);

      calledCallback = false;

      scope.focusInDirection(TraversalDirection.left);
      await tester.pump();

      expect(calledCallback, isTrue);

      scope.focusInDirection(TraversalDirection.right);
      await tester.pump();

      expect(calledCallback, isTrue);
    });
  });

  group(FocusTraversalGroup, () {
    testWidgets("Focus traversal group doesn't introduce a Semantics node", (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(FocusTraversalGroup(child: Container()));
      final TestSemantics expectedSemantics = TestSemantics.root();
      expect(semantics, hasSemantics(expectedSemantics));
      semantics.dispose();
    });

    testWidgets(
      "Descendants of FocusTraversalGroup aren't focusable if descendantsAreFocusable is false.",
      (WidgetTester tester) async {
        final GlobalKey key1 = GlobalKey(debugLabel: '1');
        final GlobalKey key2 = GlobalKey(debugLabel: '2');
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);
        bool? gotFocus;

        await tester.pumpWidget(
          FocusTraversalGroup(
            descendantsAreFocusable: false,
            child: Focus(
              onFocusChange: (bool focused) => gotFocus = focused,
              child: Focus(key: key1, focusNode: focusNode, child: Container(key: key2)),
            ),
          ),
        );

        final Element childWidget = tester.element(find.byKey(key1));
        final FocusNode unfocusableNode = Focus.of(childWidget);
        final Element containerWidget = tester.element(find.byKey(key2));
        final FocusNode containerNode = Focus.of(containerWidget);

        unfocusableNode.requestFocus();
        await tester.pump();

        expect(gotFocus, isNull);
        expect(containerNode.hasFocus, isFalse);
        expect(unfocusableNode.hasFocus, isFalse);

        containerNode.requestFocus();
        await tester.pump();

        expect(gotFocus, isNull);
        expect(containerNode.hasFocus, isFalse);
        expect(unfocusableNode.hasFocus, isFalse);
      },
    );

    testWidgets('Group applies correct policy if focus tree is different from widget tree.', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final FocusNode focusNode = FocusNode(debugLabel: 'child');
      addTearDown(focusNode.dispose);
      final FocusNode parentFocusNode = FocusNode(debugLabel: 'parent');
      addTearDown(parentFocusNode.dispose);

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            FocusTraversalGroup(
              policy: WidgetOrderTraversalPolicy(),
              child: Focus(
                child: Focus.withExternalFocusNode(
                  key: key1,
                  // This makes focusNode be a child of parentFocusNode instead
                  // of the surrounding Focus.
                  parentNode: parentFocusNode,
                  focusNode: focusNode,
                  child: Container(key: key2),
                ),
              ),
            ),
            FocusTraversalGroup(
              policy: SkipAllButFirstAndLastPolicy(),
              child: FocusScope(
                child: Focus.withExternalFocusNode(
                  key: key3,
                  focusNode: parentFocusNode,
                  child: Container(key: key4),
                ),
              ),
            ),
          ],
        ),
      );

      expect(focusNode.parent, equals(parentFocusNode));
      expect(
        FocusTraversalGroup.maybeOf(key2.currentContext!),
        const TypeMatcher<SkipAllButFirstAndLastPolicy>(),
      );
      expect(
        FocusTraversalGroup.of(key2.currentContext!),
        const TypeMatcher<SkipAllButFirstAndLastPolicy>(),
      );
    });

    testWidgets(
      "Descendants of FocusTraversalGroup aren't traversable if descendantsAreTraversable is false.",
      (WidgetTester tester) async {
        final FocusNode node1 = FocusNode();
        addTearDown(node1.dispose);
        final FocusNode node2 = FocusNode();
        addTearDown(node2.dispose);

        await tester.pumpWidget(
          FocusTraversalGroup(
            descendantsAreTraversable: false,
            child: Column(
              children: <Widget>[
                Focus(focusNode: node1, child: Container()),
                Focus(focusNode: node2, child: Container()),
              ],
            ),
          ),
        );

        node1.requestFocus();
        await tester.pump();

        expect(node1.hasPrimaryFocus, isTrue);
        expect(node2.hasPrimaryFocus, isFalse);

        expect(primaryFocus!.nextFocus(), isFalse);
        await tester.pump();

        expect(node1.hasPrimaryFocus, isTrue);
        expect(node2.hasPrimaryFocus, isFalse);
      },
    );

    testWidgets(
      "FocusTraversalGroup with skipTraversal for all descendants set to true doesn't cause an exception.",
      (WidgetTester tester) async {
        final FocusNode node1 = FocusNode();
        addTearDown(node1.dispose);
        final FocusNode node2 = FocusNode();
        addTearDown(node2.dispose);

        await tester.pumpWidget(
          FocusTraversalGroup(
            child: Column(
              children: <Widget>[
                Focus(skipTraversal: true, focusNode: node1, child: Container()),
                Focus(skipTraversal: true, focusNode: node2, child: Container()),
              ],
            ),
          ),
        );

        node1.requestFocus();
        await tester.pump();

        expect(node1.hasPrimaryFocus, isTrue);
        expect(node2.hasPrimaryFocus, isFalse);

        expect(primaryFocus!.nextFocus(), isFalse);
        await tester.pump();

        expect(node1.hasPrimaryFocus, isTrue);
        expect(node2.hasPrimaryFocus, isFalse);
      },
    );

    testWidgets("Nested FocusTraversalGroup with unfocusable children doesn't assert.", (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? gotFocus;

      await tester.pumpWidget(
        FocusTraversalGroup(
          child: Column(
            children: <Widget>[
              Focus(autofocus: true, child: Container()),
              FocusTraversalGroup(
                descendantsAreFocusable: false,
                child: Focus(
                  onFocusChange: (bool focused) => gotFocus = focused,
                  child: Focus(key: key1, focusNode: focusNode, child: Container(key: key2)),
                ),
              ),
            ],
          ),
        ),
      );

      final Element childWidget = tester.element(find.byKey(key1));
      final FocusNode unfocusableNode = Focus.of(childWidget);
      final Element containerWidget = tester.element(find.byKey(key2));
      final FocusNode containerNode = Focus.of(containerWidget);

      await tester.pump();
      primaryFocus!.nextFocus();

      expect(gotFocus, isNull);
      expect(containerNode.hasFocus, isFalse);
      expect(unfocusableNode.hasFocus, isFalse);

      containerNode.requestFocus();
      await tester.pump();

      expect(gotFocus, isNull);
      expect(containerNode.hasFocus, isFalse);
      expect(unfocusableNode.hasFocus, isFalse);
    });

    testWidgets("Empty FocusTraversalGroup doesn't cause an exception.", (
      WidgetTester tester,
    ) async {
      final GlobalKey key = GlobalKey(debugLabel: 'Test Key');
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        FocusTraversalGroup(
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: Column(
              children: <Widget>[
                FocusTraversalGroup(child: Container(key: key)),
                Focus(focusNode: focusNode, autofocus: true, child: Container()),
              ],
            ),
          ),
        ),
      );

      await tester.pump();
      primaryFocus!.nextFocus();
      await tester.pump();
      expect(primaryFocus, equals(focusNode));
    });
  });

  group(RawKeyboardListener, () {
    testWidgets('Raw keyboard listener introduces a Semantics node by default', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(RawKeyboardListener(focusNode: focusNode, child: Container()));
      final TestSemantics expectedSemantics = TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics.rootChild(
            flags: <SemanticsFlag>[SemanticsFlag.isFocusable],
            actions: <SemanticsAction>[SemanticsAction.focus],
          ),
        ],
      );
      expect(
        semantics,
        hasSemantics(expectedSemantics, ignoreId: true, ignoreRect: true, ignoreTransform: true),
      );
      semantics.dispose();
    });

    testWidgets("Raw keyboard listener doesn't introduce a Semantics node when specified", (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        RawKeyboardListener(focusNode: focusNode, includeSemantics: false, child: Container()),
      );
      final TestSemantics expectedSemantics = TestSemantics.root();
      expect(semantics, hasSemantics(expectedSemantics));
      semantics.dispose();
    });
  });

  group(ExcludeFocusTraversal, () {
    testWidgets("Descendants aren't traversable", (WidgetTester tester) async {
      final FocusNode node1 = FocusNode(debugLabel: 'node 1');
      addTearDown(node1.dispose);
      final FocusNode node2 = FocusNode(debugLabel: 'node 2');
      addTearDown(node2.dispose);
      final FocusNode node3 = FocusNode(debugLabel: 'node 3');
      addTearDown(node3.dispose);
      final FocusNode node4 = FocusNode(debugLabel: 'node 4');
      addTearDown(node4.dispose);

      await tester.pumpWidget(
        FocusTraversalGroup(
          child: Column(
            children: <Widget>[
              Focus(autofocus: true, focusNode: node1, child: Container()),
              ExcludeFocusTraversal(
                child: Focus(focusNode: node2, child: Focus(focusNode: node3, child: Container())),
              ),
              Focus(focusNode: node4, child: Container()),
            ],
          ),
        ),
      );
      await tester.pump();

      expect(node1.hasPrimaryFocus, isTrue);
      expect(node2.hasPrimaryFocus, isFalse);
      expect(node3.hasPrimaryFocus, isFalse);
      expect(node4.hasPrimaryFocus, isFalse);

      node1.nextFocus();
      await tester.pump();

      expect(node1.hasPrimaryFocus, isFalse);
      expect(node2.hasPrimaryFocus, isFalse);
      expect(node3.hasPrimaryFocus, isFalse);
      expect(node4.hasPrimaryFocus, isTrue);
    });

    testWidgets("Doesn't introduce a Semantics node", (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(ExcludeFocusTraversal(child: Container()));
      final TestSemantics expectedSemantics = TestSemantics.root();
      expect(semantics, hasSemantics(expectedSemantics));
      semantics.dispose();
    });
  });

  // Tests that Flutter allows the focus to escape the app. This is the default
  // behavior on the web, since on the web the app is always embedded into some
  // surrounding UI. There's at least the browser UI for the address bar and
  // tabs. If Flutter Web is embedded into a custom element, there could be
  // other focusable HTML elements surrounding Flutter.
  //
  // See also: https://github.com/flutter/flutter/issues/114463
  testWidgets('Default route edge traversal behavior', (WidgetTester tester) async {
    final FocusNode nodeA = FocusNode();
    addTearDown(nodeA.dispose);
    final FocusNode nodeB = FocusNode();
    addTearDown(nodeB.dispose);

    Future<bool> nextFocus() async {
      final bool result = Actions.invoke(primaryFocus!.context!, const NextFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    Future<bool> previousFocus() async {
      final bool result =
          Actions.invoke(primaryFocus!.context!, const PreviousFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            TextButton(focusNode: nodeA, child: const Text('A'), onPressed: () {}),
            TextButton(focusNode: nodeB, child: const Text('B'), onPressed: () {}),
          ],
        ),
      ),
    );

    nodeA.requestFocus();
    await tester.pump();

    expect(nodeA.hasFocus, true);
    expect(nodeB.hasFocus, false);

    // A -> B
    expect(await nextFocus(), isTrue);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);

    // A <- B
    expect(await previousFocus(), isTrue);
    expect(nodeA.hasFocus, true);
    expect(nodeB.hasFocus, false);

    // A -> B
    expect(await nextFocus(), isTrue);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);

    // B ->
    //   * on mobile: cycle back to A
    //   * on web: let the focus escape the app
    expect(await nextFocus(), !kIsWeb);
    expect(nodeA.hasFocus, !kIsWeb);
    expect(nodeB.hasFocus, false);

    // Start with A again, but wrap around in the opposite direction
    nodeA.requestFocus();
    await tester.pump();
    expect(await previousFocus(), !kIsWeb);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, !kIsWeb);
  });

  // This test creates a FocusScopeNode configured to traverse focus in a closed
  // loop. After traversing one loop, it changes the behavior to `leaveFlutterView` and `stop`,
  // then verifies that the new behavior did indeed take effect.
  testWidgets('FocusScopeNode.traversalEdgeBehavior takes effect after update', (
    WidgetTester tester,
  ) async {
    final FocusScopeNode scope = FocusScopeNode();
    addTearDown(scope.dispose);
    expect(scope.traversalEdgeBehavior, TraversalEdgeBehavior.closedLoop);

    final FocusNode nodeA = FocusNode();
    addTearDown(nodeA.dispose);
    final FocusNode nodeB = FocusNode();
    addTearDown(nodeB.dispose);

    Future<bool> nextFocus() async {
      final bool result = Actions.invoke(primaryFocus!.context!, const NextFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    Future<bool> previousFocus() async {
      final bool result =
          Actions.invoke(primaryFocus!.context!, const PreviousFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Focus(
          focusNode: scope,
          child: Column(
            children: <Widget>[
              TextButton(focusNode: nodeA, child: const Text('A'), onPressed: () {}),
              TextButton(focusNode: nodeB, child: const Text('B'), onPressed: () {}),
            ],
          ),
        ),
      ),
    );

    nodeA.requestFocus();
    await tester.pump();

    expect(nodeA.hasFocus, true);
    expect(nodeB.hasFocus, false);

    // A -> B
    expect(await nextFocus(), isTrue);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);

    // A <- B (wrap around)
    expect(await nextFocus(), isTrue);
    expect(nodeA.hasFocus, true);
    expect(nodeB.hasFocus, false);

    // Change the behavior and verify that the new behavior is in effect.
    scope.traversalEdgeBehavior = TraversalEdgeBehavior.leaveFlutterView;
    expect(scope.traversalEdgeBehavior, TraversalEdgeBehavior.leaveFlutterView);

    // A -> B
    expect(await nextFocus(), isTrue);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);

    // B -> escape the view
    expect(await nextFocus(), false);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, false);

    // Change the behavior and verify that the new behavior is in effect.
    scope.traversalEdgeBehavior = TraversalEdgeBehavior.stop;
    expect(scope.traversalEdgeBehavior, TraversalEdgeBehavior.stop);

    // B -> A, but stop at the edge
    nodeB.requestFocus();
    await tester.pump();
    expect(nodeB.hasFocus, true);
    expect(await nextFocus(), false);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);

    // Change the behavior back to closedLoop and verify it's in effect. Also,
    // this time traverse in the opposite direction.
    nodeA.requestFocus();
    await tester.pump();
    expect(nodeA.hasFocus, true);
    scope.traversalEdgeBehavior = TraversalEdgeBehavior.closedLoop;
    expect(scope.traversalEdgeBehavior, TraversalEdgeBehavior.closedLoop);
    expect(await previousFocus(), true);
    expect(nodeA.hasFocus, false);
    expect(nodeB.hasFocus, true);
  });

  testWidgets('NextFocusAction converts invoke result to KeyEventResult', (
    WidgetTester tester,
  ) async {
    expect(
      NextFocusAction().toKeyEventResult(const NextFocusIntent(), true),
      KeyEventResult.handled,
    );
    expect(
      NextFocusAction().toKeyEventResult(const NextFocusIntent(), false),
      KeyEventResult.skipRemainingHandlers,
    );
  });

  testWidgets('PreviousFocusAction converts invoke result to KeyEventResult', (
    WidgetTester tester,
  ) async {
    expect(
      PreviousFocusAction().toKeyEventResult(const PreviousFocusIntent(), true),
      KeyEventResult.handled,
    );
    expect(
      PreviousFocusAction().toKeyEventResult(const PreviousFocusIntent(), false),
      KeyEventResult.skipRemainingHandlers,
    );
  });

  testWidgets('RequestFocusAction calls the RequestFocusIntent.requestFocusCallback', (
    WidgetTester tester,
  ) async {
    bool calledCallback = false;
    final FocusNode nodeA = FocusNode();
    addTearDown(nodeA.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: SingleChildScrollView(
          child: TextButton(focusNode: nodeA, child: const Text('A'), onPressed: () {}),
        ),
      ),
    );

    RequestFocusAction().invoke(RequestFocusIntent(nodeA));
    await tester.pump();
    expect(nodeA.hasFocus, isTrue);

    nodeA.unfocus();
    await tester.pump();
    expect(nodeA.hasFocus, isFalse);

    final RequestFocusIntent focusIntentWithCallback = RequestFocusIntent(
      nodeA,
      requestFocusCallback:
          (
            FocusNode node, {
            double? alignment,
            ScrollPositionAlignmentPolicy? alignmentPolicy,
            Curve? curve,
            Duration? duration,
          }) => calledCallback = true,
    );

    RequestFocusAction().invoke(focusIntentWithCallback);
    await tester.pump();
    expect(calledCallback, isTrue);
  });

  testWidgets('Edge cases for inDirection', (WidgetTester tester) async {
    List<bool?> focus = List<bool?>.generate(6, (int _) => null);
    final List<FocusNode> nodes = List<FocusNode>.generate(
      6,
      (int index) => FocusNode(debugLabel: 'Node $index'),
    );
    final FocusScopeNode childScope = FocusScopeNode(debugLabel: 'Child Scope');
    addTearDown(() {
      for (final FocusNode node in nodes) {
        node.dispose();
      }
      childScope.dispose();
    });

    Focus makeFocus(int index) {
      return Focus(
        debugLabel: '[$index]',
        focusNode: nodes[index],
        onFocusChange: (bool isFocused) => focus[index] = isFocused,
        child: const SizedBox(width: 100, height: 100),
      );
    }

    Future<void> pumpApp() async {
      /// Layout is:
      ///          [0]
      /// ---------Child FocusScope---------
      ///          [1]
      ///          [2]
      ///              [3]
      /// ---------Child FocusScope End---------
      ///          [4] [5]
      Widget home = Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Row(mainAxisAlignment: MainAxisAlignment.center, children: <Widget>[makeFocus(0)]),
          FocusScope(
            node: childScope,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                makeFocus(1),
                Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    makeFocus(2),
                    Padding(padding: const EdgeInsets.only(top: 100), child: makeFocus(3)),
                  ],
                ),
              ],
            ),
          ),
          Row(children: <Widget>[makeFocus(4), makeFocus(5)]),
        ],
      );
      // Prevent the arrow keys from scrolling on the web.
      if (isBrowser) {
        home = Shortcuts(
          shortcuts: const <ShortcutActivator, Intent>{
            SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(
              TraversalDirection.up,
            ),
            SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(
              TraversalDirection.down,
            ),
          },
          child: home,
        );
      }
      await tester.pumpWidget(MaterialApp(home: home));
    }

    await pumpApp();

    void clear() {
      focus = List<bool?>.generate(focus.length, (int _) => null);
    }

    Future<void> resetTo(int index) async {
      nodes[index].requestFocus();
      await tester.pump();
      clear();
    }

    // childScope's directionalTraversalEdgeBehavior is TraversalEdgeBehavior.stop
    // focus is should not change
    childScope.directionalTraversalEdgeBehavior = TraversalEdgeBehavior.stop;
    await resetTo(3);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null]));
    clear();
    await resetTo(1);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, null, null, null, null, null]));
    clear();

    // childScope's directionalTraversalEdgeBehavior is TraversalEdgeBehavior.closedLoop
    // focus is should change in a loop
    childScope.directionalTraversalEdgeBehavior = TraversalEdgeBehavior.closedLoop;
    await resetTo(3);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, true, null, false, null, null]));
    clear();
    await resetTo(1);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, false, true, null, null, null]));
    clear();

    // childScope's directionalTraversalEdgeBehavior is TraversalEdgeBehavior.parentScope
    // focus can change to the parent scope
    childScope.directionalTraversalEdgeBehavior = TraversalEdgeBehavior.parentScope;
    await resetTo(3);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, null, null, false, null, true]));
    clear();
    await resetTo(1);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[true, false, null, null, null, null]));
    clear();

    // childScope's directionalTraversalEdgeBehavior is TraversalEdgeBehavior.leaveFlutterView
    // focus will be lost
    childScope.directionalTraversalEdgeBehavior = TraversalEdgeBehavior.leaveFlutterView;
    await resetTo(3);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, null, null, false, null, null]));
    clear();
    await resetTo(1);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(focus, orderedEquals(<bool?>[null, false, null, null, null, null]));
    clear();
  });

  testWidgets('When there is no focused node, the focus can be set to the FocusScopeNode.', (
    WidgetTester tester,
  ) async {
    final FocusScopeNode scope = FocusScopeNode();
    final FocusScopeNode childScope = FocusScopeNode();
    final FocusNode nodeA = FocusNode();
    addTearDown(() {
      scope.dispose();
      childScope.dispose();
      nodeA.dispose();
    });
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FocusScope(
          node: scope,
          child: FocusScope(
            node: childScope,
            child: Padding(
              padding: const EdgeInsets.only(top: 10),
              child: Focus(focusNode: nodeA, child: const Text('A')),
            ),
          ),
        ),
      ),
    );
    expect(scope.focusInDirection(TraversalDirection.down), isTrue);
    await tester.pump();
    expect(childScope.hasFocus, isTrue);
    expect(nodeA.hasFocus, isFalse);
  });
}

class TestRoute extends PageRouteBuilder<void> {
  TestRoute({required Widget child})
    : super(
        pageBuilder: (BuildContext _, Animation<double> _, Animation<double> _) {
          return child;
        },
      );
}

/// Used to test removal of nodes while sorting.
class SkipAllButFirstAndLastPolicy extends FocusTraversalPolicy
    with DirectionalFocusTraversalPolicyMixin {
  @override
  Iterable<FocusNode> sortDescendants(Iterable<FocusNode> descendants, FocusNode currentNode) {
    return <FocusNode>[
      descendants.first,
      if (currentNode != descendants.first && currentNode != descendants.last) currentNode,
      descendants.last,
    ];
  }
}
