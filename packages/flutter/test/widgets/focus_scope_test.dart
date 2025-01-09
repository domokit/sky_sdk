// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group('FocusScope', () {
    testWidgets('Can focus', (WidgetTester tester) async {
      final GlobalKey<TestFocusState> key = GlobalKey();

      await tester.pumpWidget(TestFocus(key: key));

      expect(key.currentState!.focusNode.hasFocus, isFalse);

      FocusScope.of(key.currentContext!).requestFocus(key.currentState!.focusNode);
      await tester.pumpAndSettle();

      expect(key.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
    });

    testWidgets('Can unfocus', (WidgetTester tester) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();
      await tester.pumpWidget(
        Column(children: <Widget>[TestFocus(key: keyA), TestFocus(key: keyB, name: 'b')]),
      );

      expect(keyA.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      // Set focus to the "B" node to unfocus the "A" node.
      FocusScope.of(keyB.currentContext!).requestFocus(keyB.currentState!.focusNode);
      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('B FOCUSED'), findsOneWidget);
    });

    testWidgets('Autofocus works', (WidgetTester tester) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();
      await tester.pumpWidget(
        Column(
          children: <Widget>[
            TestFocus(key: keyA),
            TestFocus(key: keyB, name: 'b', autofocus: true),
          ],
        ),
      );

      await tester.pump();

      expect(keyA.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('B FOCUSED'), findsOneWidget);
    });

    testWidgets('Can have multiple focused children and they update accordingly', (
      WidgetTester tester,
    ) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            TestFocus(key: keyA, autofocus: true),
            TestFocus(key: keyB, name: 'b'),
          ],
        ),
      );

      // Autofocus is delayed one frame.
      await tester.pump();
      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
      await tester.tap(find.text('A FOCUSED'));
      await tester.pump();
      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
      await tester.tap(find.text('b'));
      await tester.pump();
      expect(keyA.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('B FOCUSED'), findsOneWidget);
      await tester.tap(find.text('a'));
      await tester.pump();
      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
    });

    // This moves a focus node first into a focus scope that is added to its
    // parent, and then out of that focus scope again.
    testWidgets('Can move focus in and out of FocusScope', (WidgetTester tester) async {
      final FocusScopeNode parentFocusScope = FocusScopeNode(debugLabel: 'Parent Scope Node');
      addTearDown(parentFocusScope.dispose);
      final FocusScopeNode childFocusScope = FocusScopeNode(debugLabel: 'Child Scope Node');
      addTearDown(childFocusScope.dispose);
      final GlobalKey<TestFocusState> key = GlobalKey();

      // Initially create the focus inside of the parent FocusScope.
      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          autofocus: true,
          child: Column(children: <Widget>[TestFocus(key: key, debugLabel: 'Child')]),
        ),
      );

      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);
      FocusScope.of(key.currentContext!).requestFocus(key.currentState!.focusNode);
      await tester.pumpAndSettle();

      expect(key.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);

      expect(parentFocusScope, hasAGoodToStringDeep);
      expect(
        parentFocusScope.toStringDeep(),
        equalsIgnoringHashCodes(
          'FocusScopeNode#00000(Parent Scope Node [IN FOCUS PATH])\n'
          ' │ context: FocusScope\n'
          ' │ IN FOCUS PATH\n'
          ' │ focusedChildren: FocusNode#00000(Child [PRIMARY FOCUS])\n'
          ' │\n'
          ' └─Child 1: FocusNode#00000(Child [PRIMARY FOCUS])\n'
          '     context: Focus\n'
          '     PRIMARY FOCUS\n',
        ),
      );

      expect(FocusManager.instance.rootScope, hasAGoodToStringDeep);
      expect(
        FocusManager.instance.rootScope.toStringDeep(minLevel: DiagnosticLevel.info),
        equalsIgnoringHashCodes(
          'FocusScopeNode#00000(Root Focus Scope [IN FOCUS PATH])\n'
          ' │ IN FOCUS PATH\n'
          ' │ focusedChildren: FocusScopeNode#00000(View Scope [IN FOCUS PATH])\n'
          ' │\n'
          ' └─Child 1: _FocusTraversalGroupNode#00000(FocusTraversalGroup [IN FOCUS PATH])\n'
          '   │ context: Focus\n'
          '   │ NOT FOCUSABLE\n'
          '   │ IN FOCUS PATH\n'
          '   │\n'
          '   └─Child 1: FocusScopeNode#00000(View Scope [IN FOCUS PATH])\n'
          '     │ context: _FocusScopeWithExternalFocusNode\n'
          '     │ IN FOCUS PATH\n'
          '     │ focusedChildren: FocusScopeNode#00000(Parent Scope Node [IN FOCUS\n'
          '     │   PATH])\n'
          '     │\n'
          '     └─Child 1: FocusScopeNode#00000(Parent Scope Node [IN FOCUS PATH])\n'
          '       │ context: FocusScope\n'
          '       │ IN FOCUS PATH\n'
          '       │ focusedChildren: FocusNode#00000(Child [PRIMARY FOCUS])\n'
          '       │\n'
          '       └─Child 1: FocusNode#00000(Child [PRIMARY FOCUS])\n'
          '           context: Focus\n'
          '           PRIMARY FOCUS\n',
        ),
      );

      // Add the child focus scope to the focus tree.
      final FocusAttachment childAttachment = childFocusScope.attach(key.currentContext);
      parentFocusScope.setFirstFocus(childFocusScope);
      await tester.pumpAndSettle();
      expect(childFocusScope.isFirstFocus, isTrue);

      // Now add the child focus scope with no child focusable in it to the tree.
      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          child: Column(
            children: <Widget>[
              TestFocus(key: key, debugLabel: 'Child'),
              FocusScope(debugLabel: 'Child Scope', node: childFocusScope, child: Container()),
            ],
          ),
        ),
      );

      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);

      // Now move the existing focus node into the child focus scope.
      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          child: Column(
            children: <Widget>[
              FocusScope(
                debugLabel: 'Child Scope',
                node: childFocusScope,
                child: TestFocus(key: key, debugLabel: 'Child'),
              ),
            ],
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);

      // Now remove the child focus scope.
      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          child: Column(children: <Widget>[TestFocus(key: key, debugLabel: 'Child')]),
        ),
      );

      await tester.pumpAndSettle();
      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);

      // Must detach the child because we had to attach it in order to call
      // setFirstFocus before adding to the widget.
      childAttachment.detach();
    });

    testWidgets('Setting first focus requests focus for the scope properly.', (
      WidgetTester tester,
    ) async {
      final FocusScopeNode parentFocusScope = FocusScopeNode(debugLabel: 'Parent Scope Node');
      addTearDown(parentFocusScope.dispose);
      final FocusScopeNode childFocusScope1 = FocusScopeNode(debugLabel: 'Child Scope Node 1');
      addTearDown(childFocusScope1.dispose);
      final FocusScopeNode childFocusScope2 = FocusScopeNode(debugLabel: 'Child Scope Node 2');
      addTearDown(childFocusScope2.dispose);
      final GlobalKey<TestFocusState> keyA = GlobalKey(debugLabel: 'Key A');
      final GlobalKey<TestFocusState> keyB = GlobalKey(debugLabel: 'Key B');
      final GlobalKey<TestFocusState> keyC = GlobalKey(debugLabel: 'Key C');

      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          child: Column(
            children: <Widget>[
              FocusScope(
                debugLabel: 'Child Scope 1',
                node: childFocusScope1,
                child: Column(
                  children: <Widget>[
                    TestFocus(key: keyA, autofocus: true, debugLabel: 'Child A'),
                    TestFocus(key: keyB, name: 'b', debugLabel: 'Child B'),
                  ],
                ),
              ),
              FocusScope(
                debugLabel: 'Child Scope 2',
                node: childFocusScope2,
                child: TestFocus(key: keyC, name: 'c', debugLabel: 'Child C'),
              ),
            ],
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);

      parentFocusScope.setFirstFocus(childFocusScope2);
      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('a'), findsOneWidget);

      parentFocusScope.setFirstFocus(childFocusScope1);
      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);

      keyB.currentState!.focusNode.requestFocus();
      await tester.pumpAndSettle();

      expect(keyB.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('B FOCUSED'), findsOneWidget);
      expect(parentFocusScope.isFirstFocus, isTrue);
      expect(childFocusScope1.isFirstFocus, isTrue);

      parentFocusScope.setFirstFocus(childFocusScope2);
      await tester.pumpAndSettle();

      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
      expect(parentFocusScope.isFirstFocus, isTrue);
      expect(childFocusScope1.isFirstFocus, isFalse);
      expect(childFocusScope2.isFirstFocus, isTrue);

      keyC.currentState!.focusNode.requestFocus();
      await tester.pumpAndSettle();

      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
      expect(keyC.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('C FOCUSED'), findsOneWidget);
      expect(parentFocusScope.isFirstFocus, isTrue);
      expect(childFocusScope1.isFirstFocus, isFalse);
      expect(childFocusScope2.isFirstFocus, isTrue);

      childFocusScope1.requestFocus();
      await tester.pumpAndSettle();
      expect(keyB.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('B FOCUSED'), findsOneWidget);
      expect(keyC.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('c'), findsOneWidget);
      expect(parentFocusScope.isFirstFocus, isTrue);
      expect(childFocusScope1.isFirstFocus, isTrue);
      expect(childFocusScope2.isFirstFocus, isFalse);
    });

    testWidgets('Removing focused widget moves focus to next widget', (WidgetTester tester) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();

      await tester.pumpWidget(
        Column(children: <Widget>[TestFocus(key: keyA), TestFocus(key: keyB, name: 'b')]),
      );

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      await tester.pumpWidget(Column(children: <Widget>[TestFocus(key: keyB, name: 'b')]));

      await tester.pump();

      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
    });

    testWidgets('Adding a new FocusScope attaches the child to its parent.', (
      WidgetTester tester,
    ) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final FocusScopeNode parentFocusScope = FocusScopeNode(debugLabel: 'Parent Scope Node');
      addTearDown(parentFocusScope.dispose);
      final FocusScopeNode childFocusScope = FocusScopeNode(debugLabel: 'Child Scope Node');
      addTearDown(childFocusScope.dispose);

      await tester.pumpWidget(
        FocusScope(node: childFocusScope, child: TestFocus(debugLabel: 'Child', key: keyA)),
      );

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      expect(FocusScope.of(keyA.currentContext!), equals(childFocusScope));
      expect(Focus.of(keyA.currentContext!, scopeOk: true), equals(childFocusScope));
      FocusManager.instance.rootScope.setFirstFocus(FocusScope.of(keyA.currentContext!));

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(childFocusScope.isFirstFocus, isTrue);

      await tester.pumpWidget(
        FocusScope(
          node: parentFocusScope,
          child: FocusScope(
            node: childFocusScope,
            child: TestFocus(debugLabel: 'Child', key: keyA),
          ),
        ),
      );

      await tester.pump();
      expect(childFocusScope.isFirstFocus, isTrue);
      // Node keeps it's focus when moved to the new scope.
      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
    });

    testWidgets('Setting parentNode determines focus tree hierarchy.', (WidgetTester tester) async {
      final FocusNode topNode = FocusNode(debugLabel: 'Top');
      addTearDown(topNode.dispose);
      final FocusNode parentNode = FocusNode(debugLabel: 'Parent');
      addTearDown(parentNode.dispose);
      final FocusNode childNode = FocusNode(debugLabel: 'Child');
      addTearDown(childNode.dispose);
      final FocusNode insertedNode = FocusNode(debugLabel: 'Inserted');
      addTearDown(insertedNode.dispose);

      await tester.pumpWidget(
        FocusScope(
          child: Focus.withExternalFocusNode(
            focusNode: topNode,
            child: Column(
              children: <Widget>[
                Focus.withExternalFocusNode(focusNode: parentNode, child: const SizedBox()),
                Focus.withExternalFocusNode(
                  focusNode: childNode,
                  parentNode: parentNode,
                  autofocus: true,
                  child: const SizedBox(),
                ),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(childNode.hasPrimaryFocus, isTrue);
      expect(parentNode.hasFocus, isTrue);
      expect(topNode.hasFocus, isTrue);

      // Check that inserting a Focus in between doesn't reparent the child.
      await tester.pumpWidget(
        FocusScope(
          child: Focus.withExternalFocusNode(
            focusNode: topNode,
            child: Column(
              children: <Widget>[
                Focus.withExternalFocusNode(focusNode: parentNode, child: const SizedBox()),
                Focus.withExternalFocusNode(
                  focusNode: insertedNode,
                  child: Focus.withExternalFocusNode(
                    focusNode: childNode,
                    parentNode: parentNode,
                    autofocus: true,
                    child: const SizedBox(),
                  ),
                ),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(childNode.hasPrimaryFocus, isTrue);
      expect(parentNode.hasFocus, isTrue);
      expect(topNode.hasFocus, isTrue);
      expect(insertedNode.hasFocus, isFalse);
    });

    testWidgets('Setting parentNode determines focus scope tree hierarchy.', (
      WidgetTester tester,
    ) async {
      final FocusScopeNode topNode = FocusScopeNode(debugLabel: 'Top');
      addTearDown(topNode.dispose);
      final FocusScopeNode parentNode = FocusScopeNode(debugLabel: 'Parent');
      addTearDown(parentNode.dispose);
      final FocusScopeNode childNode = FocusScopeNode(debugLabel: 'Child');
      addTearDown(childNode.dispose);
      final FocusScopeNode insertedNode = FocusScopeNode(debugLabel: 'Inserted');
      addTearDown(insertedNode.dispose);

      await tester.pumpWidget(
        FocusScope.withExternalFocusNode(
          focusScopeNode: topNode,
          child: Column(
            children: <Widget>[
              FocusScope.withExternalFocusNode(focusScopeNode: parentNode, child: const SizedBox()),
              FocusScope.withExternalFocusNode(
                focusScopeNode: childNode,
                parentNode: parentNode,
                child: const Focus(autofocus: true, child: SizedBox()),
              ),
            ],
          ),
        ),
      );
      await tester.pump();

      expect(childNode.hasFocus, isTrue);
      expect(parentNode.hasFocus, isTrue);
      expect(topNode.hasFocus, isTrue);

      // Check that inserting a Focus in between doesn't reparent the child.
      await tester.pumpWidget(
        FocusScope.withExternalFocusNode(
          focusScopeNode: topNode,
          child: Column(
            children: <Widget>[
              FocusScope.withExternalFocusNode(focusScopeNode: parentNode, child: const SizedBox()),
              FocusScope.withExternalFocusNode(
                focusScopeNode: insertedNode,
                child: FocusScope.withExternalFocusNode(
                  focusScopeNode: childNode,
                  parentNode: parentNode,
                  child: const Focus(autofocus: true, child: SizedBox()),
                ),
              ),
            ],
          ),
        ),
      );
      await tester.pump();

      expect(childNode.hasFocus, isTrue);
      expect(parentNode.hasFocus, isTrue);
      expect(topNode.hasFocus, isTrue);
      expect(insertedNode.hasFocus, isFalse);
    });

    // Arguably, this isn't correct behavior, but it is what happens now.
    testWidgets("Removing focused widget doesn't move focus to next widget within FocusScope", (
      WidgetTester tester,
    ) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();
      final FocusScopeNode parentFocusScope = FocusScopeNode(debugLabel: 'Parent Scope');
      addTearDown(parentFocusScope.dispose);

      await tester.pumpWidget(
        FocusScope(
          debugLabel: 'Parent Scope',
          node: parentFocusScope,
          autofocus: true,
          child: Column(
            children: <Widget>[
              TestFocus(debugLabel: 'Widget A', key: keyA),
              TestFocus(debugLabel: 'Widget B', key: keyB, name: 'b'),
            ],
          ),
        ),
      );

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      final FocusScopeNode scope = FocusScope.of(keyA.currentContext!);
      FocusManager.instance.rootScope.setFirstFocus(scope);

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      await tester.pumpWidget(
        FocusScope(
          node: parentFocusScope,
          child: Column(children: <Widget>[TestFocus(key: keyB, name: 'b')]),
        ),
      );

      await tester.pump();

      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
    });

    testWidgets('Removing a FocusScope removes its node from the tree', (
      WidgetTester tester,
    ) async {
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();
      final GlobalKey<TestFocusState> scopeKeyA = GlobalKey();
      final GlobalKey<TestFocusState> scopeKeyB = GlobalKey();
      final FocusScopeNode parentFocusScope = FocusScopeNode(debugLabel: 'Parent Scope');
      addTearDown(parentFocusScope.dispose);

      // This checks both FocusScopes that have their own nodes, as well as those
      // that use external nodes.
      await tester.pumpWidget(
        FocusTraversalGroup(
          child: Column(
            children: <Widget>[
              FocusScope(
                key: scopeKeyA,
                node: parentFocusScope,
                child: Column(children: <Widget>[TestFocus(debugLabel: 'Child A', key: keyA)]),
              ),
              FocusScope(
                key: scopeKeyB,
                child: Column(
                  children: <Widget>[TestFocus(debugLabel: 'Child B', key: keyB, name: 'b')],
                ),
              ),
            ],
          ),
        ),
      );

      FocusScope.of(keyB.currentContext!).requestFocus(keyB.currentState!.focusNode);
      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      final FocusScopeNode aScope = FocusScope.of(keyA.currentContext!);
      final FocusScopeNode bScope = FocusScope.of(keyB.currentContext!);
      FocusManager.instance.rootScope.setFirstFocus(bScope);
      FocusManager.instance.rootScope.setFirstFocus(aScope);

      await tester.pumpAndSettle();

      expect(FocusScope.of(keyA.currentContext!).isFirstFocus, isTrue);
      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      expect(FocusManager.instance.rootScope.descendants.length, equals(7));
      await tester.pumpWidget(Container());
      expect(FocusManager.instance.rootScope.descendants.length, equals(2));
      expect(FocusManager.instance.rootScope.descendants, isNot(contains(aScope)));
      expect(FocusManager.instance.rootScope.descendants, isNot(contains(bScope)));
    });

    // By "pinned", it means kept in the tree by a GlobalKey.
    testWidgets(
      "Removing pinned focused scope doesn't move focus to focused widget within next FocusScope",
      (WidgetTester tester) async {
        final GlobalKey<TestFocusState> keyA = GlobalKey();
        final GlobalKey<TestFocusState> keyB = GlobalKey();
        final GlobalKey<TestFocusState> scopeKeyA = GlobalKey();
        final GlobalKey<TestFocusState> scopeKeyB = GlobalKey();
        final FocusScopeNode parentFocusScope1 = FocusScopeNode(debugLabel: 'Parent Scope 1');
        addTearDown(parentFocusScope1.dispose);
        final FocusScopeNode parentFocusScope2 = FocusScopeNode(debugLabel: 'Parent Scope 2');
        addTearDown(parentFocusScope2.dispose);

        await tester.pumpWidget(
          FocusTraversalGroup(
            child: Column(
              children: <Widget>[
                FocusScope(
                  key: scopeKeyA,
                  node: parentFocusScope1,
                  child: Column(children: <Widget>[TestFocus(debugLabel: 'Child A', key: keyA)]),
                ),
                FocusScope(
                  key: scopeKeyB,
                  node: parentFocusScope2,
                  child: Column(
                    children: <Widget>[TestFocus(debugLabel: 'Child B', key: keyB, name: 'b')],
                  ),
                ),
              ],
            ),
          ),
        );

        FocusScope.of(keyB.currentContext!).requestFocus(keyB.currentState!.focusNode);
        FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
        final FocusScopeNode bScope = FocusScope.of(keyB.currentContext!);
        final FocusScopeNode aScope = FocusScope.of(keyA.currentContext!);
        FocusManager.instance.rootScope.setFirstFocus(bScope);
        FocusManager.instance.rootScope.setFirstFocus(aScope);

        await tester.pumpAndSettle();

        expect(FocusScope.of(keyA.currentContext!).isFirstFocus, isTrue);
        expect(keyA.currentState!.focusNode.hasFocus, isTrue);
        expect(find.text('A FOCUSED'), findsOneWidget);
        expect(keyB.currentState!.focusNode.hasFocus, isFalse);
        expect(find.text('b'), findsOneWidget);

        await tester.pumpWidget(
          FocusTraversalGroup(
            child: Column(
              children: <Widget>[
                FocusScope(
                  key: scopeKeyB,
                  node: parentFocusScope2,
                  child: Column(
                    children: <Widget>[
                      TestFocus(debugLabel: 'Child B', key: keyB, name: 'b', autofocus: true),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );

        await tester.pump();

        expect(keyB.currentState!.focusNode.hasFocus, isTrue);
        expect(find.text('B FOCUSED'), findsOneWidget);
      },
    );

    testWidgets(
      "Removing unpinned focused scope doesn't move focus to focused widget within next FocusScope",
      (WidgetTester tester) async {
        final GlobalKey<TestFocusState> keyA = GlobalKey();
        final GlobalKey<TestFocusState> keyB = GlobalKey();
        final FocusScopeNode parentFocusScope1 = FocusScopeNode(debugLabel: 'Parent Scope 1');
        addTearDown(parentFocusScope1.dispose);
        final FocusScopeNode parentFocusScope2 = FocusScopeNode(debugLabel: 'Parent Scope 2');
        addTearDown(parentFocusScope2.dispose);

        await tester.pumpWidget(
          FocusTraversalGroup(
            child: Column(
              children: <Widget>[
                FocusScope(
                  node: parentFocusScope1,
                  child: Column(children: <Widget>[TestFocus(debugLabel: 'Child A', key: keyA)]),
                ),
                FocusScope(
                  node: parentFocusScope2,
                  child: Column(
                    children: <Widget>[TestFocus(debugLabel: 'Child B', key: keyB, name: 'b')],
                  ),
                ),
              ],
            ),
          ),
        );

        FocusScope.of(keyB.currentContext!).requestFocus(keyB.currentState!.focusNode);
        FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
        final FocusScopeNode bScope = FocusScope.of(keyB.currentContext!);
        final FocusScopeNode aScope = FocusScope.of(keyA.currentContext!);
        FocusManager.instance.rootScope.setFirstFocus(bScope);
        FocusManager.instance.rootScope.setFirstFocus(aScope);

        await tester.pumpAndSettle();

        expect(FocusScope.of(keyA.currentContext!).isFirstFocus, isTrue);
        expect(keyA.currentState!.focusNode.hasFocus, isTrue);
        expect(find.text('A FOCUSED'), findsOneWidget);
        expect(keyB.currentState!.focusNode.hasFocus, isFalse);
        expect(find.text('b'), findsOneWidget);

        await tester.pumpWidget(
          FocusTraversalGroup(
            child: Column(
              children: <Widget>[
                FocusScope(
                  node: parentFocusScope2,
                  child: Column(
                    children: <Widget>[
                      TestFocus(debugLabel: 'Child B', key: keyB, name: 'b', autofocus: true),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
        await tester.pump();

        expect(keyB.currentState!.focusNode.hasFocus, isTrue);
        expect(find.text('B FOCUSED'), findsOneWidget);
      },
    );

    testWidgets('Moving widget from one scope to another retains focus', (
      WidgetTester tester,
    ) async {
      final FocusScopeNode parentFocusScope1 = FocusScopeNode();
      addTearDown(parentFocusScope1.dispose);
      final FocusScopeNode parentFocusScope2 = FocusScopeNode();
      addTearDown(parentFocusScope2.dispose);
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            FocusScope(
              node: parentFocusScope1,
              child: Column(children: <Widget>[TestFocus(key: keyA)]),
            ),
            FocusScope(
              node: parentFocusScope2,
              child: Column(children: <Widget>[TestFocus(key: keyB, name: 'b')]),
            ),
          ],
        ),
      );

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      final FocusScopeNode aScope = FocusScope.of(keyA.currentContext!);
      FocusManager.instance.rootScope.setFirstFocus(aScope);

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            FocusScope(
              node: parentFocusScope1,
              child: Column(children: <Widget>[TestFocus(key: keyB, name: 'b')]),
            ),
            FocusScope(
              node: parentFocusScope2,
              child: Column(children: <Widget>[TestFocus(key: keyA)]),
            ),
          ],
        ),
      );

      await tester.pump();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
    });

    testWidgets('Moving FocusScopeNodes retains focus', (WidgetTester tester) async {
      final FocusScopeNode parentFocusScope1 = FocusScopeNode(debugLabel: 'Scope 1');
      addTearDown(parentFocusScope1.dispose);
      final FocusScopeNode parentFocusScope2 = FocusScopeNode(debugLabel: 'Scope 2');
      addTearDown(parentFocusScope2.dispose);
      final GlobalKey<TestFocusState> keyA = GlobalKey();
      final GlobalKey<TestFocusState> keyB = GlobalKey();

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            FocusScope(
              node: parentFocusScope1,
              child: Column(children: <Widget>[TestFocus(debugLabel: 'Child A', key: keyA)]),
            ),
            FocusScope(
              node: parentFocusScope2,
              child: Column(
                children: <Widget>[TestFocus(debugLabel: 'Child B', key: keyB, name: 'b')],
              ),
            ),
          ],
        ),
      );

      FocusScope.of(keyA.currentContext!).requestFocus(keyA.currentState!.focusNode);
      final FocusScopeNode aScope = FocusScope.of(keyA.currentContext!);
      FocusManager.instance.rootScope.setFirstFocus(aScope);

      await tester.pumpAndSettle();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);

      // This just swaps the FocusScopeNodes that the FocusScopes have in them.
      await tester.pumpWidget(
        Column(
          children: <Widget>[
            FocusScope(
              node: parentFocusScope2,
              child: Column(children: <Widget>[TestFocus(debugLabel: 'Child A', key: keyA)]),
            ),
            FocusScope(
              node: parentFocusScope1,
              child: Column(
                children: <Widget>[TestFocus(debugLabel: 'Child B', key: keyB, name: 'b')],
              ),
            ),
          ],
        ),
      );

      await tester.pump();

      expect(keyA.currentState!.focusNode.hasFocus, isTrue);
      expect(find.text('A FOCUSED'), findsOneWidget);
      expect(keyB.currentState!.focusNode.hasFocus, isFalse);
      expect(find.text('b'), findsOneWidget);
    });

    testWidgets('Can focus root node.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      await tester.pumpWidget(Focus(key: key1, child: Container()));

      final Element firstElement = tester.element(find.byKey(key1));
      final FocusScopeNode rootNode = FocusScope.of(firstElement);
      rootNode.requestFocus();

      await tester.pump();

      expect(rootNode.hasFocus, isTrue);
      expect(rootNode, equals(FocusManager.instance.rootScope.descendants.toList()[1]));
    });

    testWidgets('Can autofocus a node.', (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(Focus(focusNode: focusNode, child: Container()));

      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isFalse);

      await tester.pumpWidget(Focus(autofocus: true, focusNode: focusNode, child: Container()));

      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);
    });

    testWidgets("Won't autofocus a node if one is already focused.", (WidgetTester tester) async {
      final FocusNode focusNodeA = FocusNode(debugLabel: 'Test Node A');
      addTearDown(focusNodeA.dispose);
      final FocusNode focusNodeB = FocusNode(debugLabel: 'Test Node B');
      addTearDown(focusNodeB.dispose);
      await tester.pumpWidget(
        Column(
          children: <Widget>[Focus(focusNode: focusNodeA, autofocus: true, child: Container())],
        ),
      );

      await tester.pump();
      expect(focusNodeA.hasPrimaryFocus, isTrue);

      await tester.pumpWidget(
        Column(
          children: <Widget>[
            Focus(focusNode: focusNodeA, child: Container()),
            Focus(focusNode: focusNodeB, autofocus: true, child: Container()),
          ],
        ),
      );

      await tester.pump();
      expect(focusNodeB.hasPrimaryFocus, isFalse);
      expect(focusNodeA.hasPrimaryFocus, isTrue);
    });

    testWidgets(
      "FocusScope doesn't update the focusNode attributes when the widget updates if withExternalFocusNode is used",
      (WidgetTester tester) async {
        final GlobalKey key1 = GlobalKey(debugLabel: '1');
        final FocusScopeNode focusScopeNode = FocusScopeNode();
        addTearDown(focusScopeNode.dispose);
        bool? keyEventHandled;
        KeyEventResult handleCallback(FocusNode node, RawKeyEvent event) {
          keyEventHandled = true;
          return KeyEventResult.handled;
        }

        KeyEventResult handleEventCallback(FocusNode node, KeyEvent event) {
          keyEventHandled = true;
          return KeyEventResult.handled;
        }

        KeyEventResult ignoreCallback(FocusNode node, RawKeyEvent event) => KeyEventResult.ignored;
        KeyEventResult ignoreEventCallback(FocusNode node, KeyEvent event) =>
            KeyEventResult.ignored;
        focusScopeNode.onKey = ignoreCallback;
        focusScopeNode.onKeyEvent = ignoreEventCallback;
        focusScopeNode.descendantsAreFocusable = false;
        focusScopeNode.descendantsAreTraversable = false;
        focusScopeNode.skipTraversal = false;
        focusScopeNode.canRequestFocus = true;
        FocusScope focusScopeWidget = FocusScope.withExternalFocusNode(
          focusScopeNode: focusScopeNode,
          child: Container(key: key1),
        );
        await tester.pumpWidget(focusScopeWidget);
        expect(focusScopeNode.onKey, equals(ignoreCallback));
        expect(focusScopeNode.onKeyEvent, equals(ignoreEventCallback));
        expect(focusScopeNode.descendantsAreFocusable, isFalse);
        expect(focusScopeNode.descendantsAreTraversable, isFalse);
        expect(focusScopeNode.skipTraversal, isFalse);
        expect(focusScopeNode.canRequestFocus, isTrue);
        expect(focusScopeWidget.onKey, equals(focusScopeNode.onKey));
        expect(focusScopeWidget.onKeyEvent, equals(focusScopeNode.onKeyEvent));
        expect(
          focusScopeWidget.descendantsAreFocusable,
          equals(focusScopeNode.descendantsAreFocusable),
        );
        expect(
          focusScopeWidget.descendantsAreTraversable,
          equals(focusScopeNode.descendantsAreTraversable),
        );
        expect(focusScopeWidget.skipTraversal, equals(focusScopeNode.skipTraversal));
        expect(focusScopeWidget.canRequestFocus, equals(focusScopeNode.canRequestFocus));

        FocusScope.of(key1.currentContext!).requestFocus();
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.enter);
        expect(keyEventHandled, isNull);

        focusScopeNode.onKey = handleCallback;
        focusScopeNode.onKeyEvent = handleEventCallback;
        focusScopeNode.descendantsAreFocusable = true;
        focusScopeNode.descendantsAreTraversable = true;
        focusScopeWidget = FocusScope.withExternalFocusNode(
          focusScopeNode: focusScopeNode,
          child: Container(key: key1),
        );
        await tester.pumpWidget(focusScopeWidget);
        expect(focusScopeNode.onKey, equals(handleCallback));
        expect(focusScopeNode.onKeyEvent, equals(handleEventCallback));
        expect(focusScopeNode.descendantsAreFocusable, isTrue);
        expect(focusScopeNode.descendantsAreTraversable, isTrue);
        expect(focusScopeNode.skipTraversal, isFalse);
        expect(focusScopeNode.canRequestFocus, isTrue);
        expect(focusScopeWidget.onKey, equals(focusScopeNode.onKey));
        expect(focusScopeWidget.onKeyEvent, equals(focusScopeNode.onKeyEvent));
        expect(
          focusScopeWidget.descendantsAreFocusable,
          equals(focusScopeNode.descendantsAreFocusable),
        );
        expect(
          focusScopeWidget.descendantsAreTraversable,
          equals(focusScopeNode.descendantsAreTraversable),
        );
        expect(focusScopeWidget.skipTraversal, equals(focusScopeNode.skipTraversal));
        expect(focusScopeWidget.canRequestFocus, equals(focusScopeNode.canRequestFocus));

        await tester.sendKeyEvent(LogicalKeyboardKey.enter);
        expect(keyEventHandled, isTrue);
      },
    );
  });

  group('Focus', () {
    testWidgets('Focus.of stops at the nearest Focus widget.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      final FocusScopeNode scopeNode = FocusScopeNode();
      addTearDown(scopeNode.dispose);
      await tester.pumpWidget(
        FocusScope(
          key: key1,
          node: scopeNode,
          debugLabel: 'Key 1',
          child: Container(
            key: key2,
            child: Focus(
              debugLabel: 'Key 3',
              key: key3,
              child: Container(
                key: key4,
                child: Focus(debugLabel: 'Key 5', key: key5, child: Container(key: key6)),
              ),
            ),
          ),
        ),
      );
      final Element element1 = tester.element(find.byKey(key1));
      final Element element2 = tester.element(find.byKey(key2));
      final Element element3 = tester.element(find.byKey(key3));
      final Element element4 = tester.element(find.byKey(key4));
      final Element element5 = tester.element(find.byKey(key5));
      final Element element6 = tester.element(find.byKey(key6));
      final FocusNode root = element1.owner!.focusManager.rootScope;

      expect(Focus.maybeOf(element1), isNull);
      expect(Focus.maybeOf(element2), isNull);
      expect(Focus.maybeOf(element3), isNull);
      expect(Focus.of(element4).parent!.parent!.parent!.parent, equals(root));
      expect(Focus.of(element5).parent!.parent!.parent!.parent, equals(root));
      expect(Focus.of(element6).parent!.parent!.parent!.parent!.parent, equals(root));
    });
    testWidgets('Can traverse Focus children.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final GlobalKey key3 = GlobalKey(debugLabel: '3');
      final GlobalKey key4 = GlobalKey(debugLabel: '4');
      final GlobalKey key5 = GlobalKey(debugLabel: '5');
      final GlobalKey key6 = GlobalKey(debugLabel: '6');
      final GlobalKey key7 = GlobalKey(debugLabel: '7');
      final GlobalKey key8 = GlobalKey(debugLabel: '8');
      await tester.pumpWidget(
        Focus(
          child: Column(
            key: key1,
            children: <Widget>[
              Focus(key: key2, child: Focus(key: key3, child: Container())),
              Focus(key: key4, child: Focus(key: key5, child: Container())),
              Focus(
                key: key6,
                child: Column(
                  children: <Widget>[
                    Focus(key: key7, child: Container()),
                    Focus(key: key8, child: Container()),
                  ],
                ),
              ),
            ],
          ),
        ),
      );

      final Element firstScope = tester.element(find.byKey(key1));
      final List<FocusNode> nodes = <FocusNode>[];
      final List<Key> keys = <Key>[];
      bool visitor(FocusNode node) {
        nodes.add(node);
        keys.add(node.context!.widget.key!);
        return true;
      }

      await tester.pump();

      Focus.of(firstScope).descendants.forEach(visitor);
      expect(nodes.length, equals(7));
      expect(keys.length, equals(7));
      // Depth first.
      expect(keys, equals(<Key>[key3, key2, key5, key4, key7, key8, key6]));

      // Just traverses a sub-tree.
      final Element secondScope = tester.element(find.byKey(key7));
      nodes.clear();
      keys.clear();
      Focus.of(secondScope).descendants.forEach(visitor);
      expect(nodes.length, equals(2));
      expect(keys, equals(<Key>[key7, key8]));
    });

    testWidgets('Can set focus.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      late bool gotFocus;
      await tester.pumpWidget(
        Focus(onFocusChange: (bool focused) => gotFocus = focused, child: Container(key: key1)),
      );

      final Element firstNode = tester.element(find.byKey(key1));
      final FocusNode node = Focus.of(firstNode);
      node.requestFocus();

      await tester.pump();

      expect(gotFocus, isTrue);
      expect(node.hasFocus, isTrue);
    });

    testWidgets('Focus is ignored when set to not focusable.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      bool? gotFocus;
      await tester.pumpWidget(
        Focus(
          canRequestFocus: false,
          onFocusChange: (bool focused) => gotFocus = focused,
          child: Container(key: key1),
        ),
      );

      final Element firstNode = tester.element(find.byKey(key1));
      final FocusNode node = Focus.of(firstNode);
      node.requestFocus();

      await tester.pump();

      expect(gotFocus, isNull);
      expect(node.hasFocus, isFalse);
    });

    testWidgets('Focus is lost when set to not focusable.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      bool? gotFocus;
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          canRequestFocus: true,
          onFocusChange: (bool focused) => gotFocus = focused,
          child: Container(key: key1),
        ),
      );

      Element firstNode = tester.element(find.byKey(key1));
      FocusNode node = Focus.of(firstNode);
      node.requestFocus();

      await tester.pump();

      expect(gotFocus, isTrue);
      expect(node.hasFocus, isTrue);

      gotFocus = null;
      await tester.pumpWidget(
        Focus(
          canRequestFocus: false,
          onFocusChange: (bool focused) => gotFocus = focused,
          child: Container(key: key1),
        ),
      );

      firstNode = tester.element(find.byKey(key1));
      node = Focus.of(firstNode);
      node.requestFocus();

      await tester.pump();

      expect(gotFocus, false);
      expect(node.hasFocus, isFalse);
    });

    testWidgets('Child of unfocusable Focus can get focus.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? gotFocus;
      await tester.pumpWidget(
        Focus(
          canRequestFocus: false,
          onFocusChange: (bool focused) => gotFocus = focused,
          child: Focus(key: key1, focusNode: focusNode, child: Container(key: key2)),
        ),
      );

      final Element childWidget = tester.element(find.byKey(key1));
      final FocusNode unfocusableNode = Focus.of(childWidget);
      unfocusableNode.requestFocus();

      await tester.pump();

      expect(gotFocus, isNull);
      expect(unfocusableNode.hasFocus, isFalse);

      final Element containerWidget = tester.element(find.byKey(key2));
      final FocusNode focusableNode = Focus.of(containerWidget);
      focusableNode.requestFocus();

      await tester.pump();

      expect(gotFocus, isTrue);
      expect(unfocusableNode.hasFocus, isTrue);
    });

    testWidgets('Nodes are removed when all Focuses are removed.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      late bool gotFocus;
      await tester.pumpWidget(
        FocusScope(
          child: Focus(
            onFocusChange: (bool focused) => gotFocus = focused,
            child: Container(key: key1),
          ),
        ),
      );

      final Element firstNode = tester.element(find.byKey(key1));
      final FocusNode node = Focus.of(firstNode);
      node.requestFocus();

      await tester.pump();

      expect(gotFocus, isTrue);
      expect(node.hasFocus, isTrue);

      await tester.pumpWidget(Container());
      // Even with no other focusable widgets, there will be the top level focus
      // traversal and view focus nodes.
      expect(FocusManager.instance.rootScope.descendants, hasLength(2));
    });

    testWidgets('Focus widgets set Semantics information about focus', (WidgetTester tester) async {
      final GlobalKey<TestFocusState> key = GlobalKey();

      await tester.pumpWidget(TestFocus(key: key));

      final SemanticsNode semantics = tester.getSemantics(find.byKey(key));

      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(semantics.hasFlag(SemanticsFlag.isFocused), isFalse);
      expect(semantics.hasFlag(SemanticsFlag.isFocusable), isTrue);

      FocusScope.of(key.currentContext!).requestFocus(key.currentState!.focusNode);
      await tester.pumpAndSettle();

      expect(key.currentState!.focusNode.hasFocus, isTrue);
      expect(semantics.hasFlag(SemanticsFlag.isFocused), isTrue);
      expect(semantics.hasFlag(SemanticsFlag.isFocusable), isTrue);

      key.currentState!.focusNode.canRequestFocus = false;
      await tester.pumpAndSettle();

      expect(key.currentState!.focusNode.hasFocus, isFalse);
      expect(key.currentState!.focusNode.canRequestFocus, isFalse);
      expect(semantics.hasFlag(SemanticsFlag.isFocused), isFalse);
      expect(semantics.hasFlag(SemanticsFlag.isFocusable), isFalse);
    });

    testWidgets('Setting canRequestFocus on focus node causes update.', (
      WidgetTester tester,
    ) async {
      final GlobalKey<TestFocusState> key = GlobalKey();

      final TestFocus testFocus = TestFocus(key: key);
      await tester.pumpWidget(testFocus);

      await tester.pumpAndSettle();
      key.currentState!.built = false;
      key.currentState!.focusNode.canRequestFocus = false;
      await tester.pumpAndSettle();
      key.currentState!.built = true;

      expect(key.currentState!.focusNode.canRequestFocus, isFalse);
    });

    testWidgets('canRequestFocus causes descendants of scope to be skipped.', (
      WidgetTester tester,
    ) async {
      final GlobalKey scope1 = GlobalKey(debugLabel: 'scope1');
      final GlobalKey scope2 = GlobalKey(debugLabel: 'scope2');
      final GlobalKey focus1 = GlobalKey(debugLabel: 'focus1');
      final GlobalKey focus2 = GlobalKey(debugLabel: 'focus2');
      final GlobalKey container1 = GlobalKey(debugLabel: 'container');
      Future<void> pumpTest({
        bool allowScope1 = true,
        bool allowScope2 = true,
        bool allowFocus1 = true,
        bool allowFocus2 = true,
      }) async {
        await tester.pumpWidget(
          FocusScope(
            key: scope1,
            canRequestFocus: allowScope1,
            child: FocusScope(
              key: scope2,
              canRequestFocus: allowScope2,
              child: Focus(
                key: focus1,
                canRequestFocus: allowFocus1,
                child: Focus(
                  key: focus2,
                  canRequestFocus: allowFocus2,
                  child: Container(key: container1),
                ),
              ),
            ),
          ),
        );
        await tester.pump();
      }

      // Check childless node (focus2).
      await pumpTest();
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);
      await pumpTest(allowFocus2: false);
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      await pumpTest();
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);

      // Check FocusNode with child (focus1). Shouldn't affect children.
      await pumpTest(allowFocus1: false);
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue); // focus2 has focus.
      Focus.of(focus2.currentContext!).requestFocus(); // Try to focus focus1
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue); // focus2 still has focus.
      Focus.of(container1.currentContext!).requestFocus(); // Now try to focus focus2
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);
      await pumpTest();
      // Try again, now that we've set focus1's canRequestFocus to true again.
      Focus.of(container1.currentContext!).unfocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);

      // Check FocusScopeNode with only FocusNode children (scope2). Should affect children.
      await pumpTest(allowScope2: false);
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      FocusScope.of(focus1.currentContext!).requestFocus(); // Try to focus scope2
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(focus2.currentContext!).requestFocus(); // Try to focus focus1
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(container1.currentContext!).requestFocus(); // Try to focus focus2
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      await pumpTest();
      // Try again, now that we've set scope2's canRequestFocus to true again.
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);

      // Check FocusScopeNode with both FocusNode children and FocusScope children (scope1). Should affect children.
      await pumpTest(allowScope1: false);
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      FocusScope.of(scope2.currentContext!).requestFocus(); // Try to focus scope1
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      FocusScope.of(focus1.currentContext!).requestFocus(); // Try to focus scope2
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(focus2.currentContext!).requestFocus(); // Try to focus focus1
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      Focus.of(container1.currentContext!).requestFocus(); // Try to focus focus2
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isFalse);
      await pumpTest();
      // Try again, now that we've set scope1's canRequestFocus to true again.
      Focus.of(container1.currentContext!).requestFocus();
      await tester.pump();
      expect(Focus.of(container1.currentContext!).hasFocus, isTrue);
    });

    testWidgets('skipTraversal works as expected.', (WidgetTester tester) async {
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1');
      addTearDown(scope1.dispose);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusNode focus1 = FocusNode(debugLabel: 'focus1');
      addTearDown(focus1.dispose);
      final FocusNode focus2 = FocusNode(debugLabel: 'focus2');
      addTearDown(focus2.dispose);

      Future<void> pumpTest({
        bool traverseScope1 = false,
        bool traverseScope2 = false,
        bool traverseFocus1 = false,
        bool traverseFocus2 = false,
      }) async {
        await tester.pumpWidget(
          FocusScope(
            node: scope1,
            skipTraversal: traverseScope1,
            child: FocusScope(
              node: scope2,
              skipTraversal: traverseScope2,
              child: Focus(
                focusNode: focus1,
                skipTraversal: traverseFocus1,
                child: Focus(focusNode: focus2, skipTraversal: traverseFocus2, child: Container()),
              ),
            ),
          ),
        );
        await tester.pump();
      }

      await pumpTest();
      expect(scope1.traversalDescendants, equals(<FocusNode>[focus2, focus1, scope2]));

      // Check childless node (focus2).
      await pumpTest(traverseFocus2: true);
      expect(scope1.traversalDescendants, equals(<FocusNode>[focus1, scope2]));

      // Check FocusNode with child (focus1). Shouldn't affect children.
      await pumpTest(traverseFocus1: true);
      expect(scope1.traversalDescendants, equals(<FocusNode>[focus2, scope2]));

      // Check FocusScopeNode with only FocusNode children (scope2). Should affect children.
      await pumpTest(traverseScope2: true);
      expect(scope1.traversalDescendants, equals(<FocusNode>[focus2, focus1]));

      // Check FocusScopeNode with both FocusNode children and FocusScope children (scope1). Should affect children.
      await pumpTest(traverseScope1: true);
      expect(scope1.traversalDescendants, equals(<FocusNode>[focus2, focus1, scope2]));
    });

    testWidgets('descendantsAreFocusable works as expected.', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? gotFocus;
      await tester.pumpWidget(
        Focus(
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
    });

    testWidgets('descendantsAreTraversable works as expected.', (WidgetTester tester) async {
      final FocusScopeNode scopeNode = FocusScopeNode(debugLabel: 'scope');
      addTearDown(scopeNode.dispose);
      final FocusNode node1 = FocusNode(debugLabel: 'node 1');
      addTearDown(node1.dispose);
      final FocusNode node2 = FocusNode(debugLabel: 'node 2');
      addTearDown(node2.dispose);
      final FocusNode node3 = FocusNode(debugLabel: 'node 3');
      addTearDown(node3.dispose);

      await tester.pumpWidget(
        FocusScope(
          node: scopeNode,
          child: Column(
            children: <Widget>[
              Focus(focusNode: node1, child: Container()),
              Focus(
                focusNode: node2,
                descendantsAreTraversable: false,
                child: Focus(focusNode: node3, child: Container()),
              ),
            ],
          ),
        ),
      );
      await tester.pump();

      expect(scopeNode.traversalDescendants, equals(<FocusNode>[node1, node2]));
      expect(node2.traversalDescendants, equals(<FocusNode>[]));
    });

    testWidgets("Focus doesn't introduce a Semantics node when includeSemantics is false", (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(Focus(includeSemantics: false, child: Container()));
      final TestSemantics expectedSemantics = TestSemantics.root();
      expect(semantics, hasSemantics(expectedSemantics));
      semantics.dispose();
    });

    testWidgets('Focus updates the onKey handler when the widget updates', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? keyEventHandled;
      KeyEventResult handleCallback(FocusNode node, RawKeyEvent event) {
        keyEventHandled = true;
        return KeyEventResult.handled;
      }

      KeyEventResult ignoreCallback(FocusNode node, RawKeyEvent event) => KeyEventResult.ignored;
      Focus focusWidget = Focus(
        onKey: ignoreCallback, // This one does nothing.
        focusNode: focusNode,
        skipTraversal: true,
        canRequestFocus: true,
        child: Container(key: key1),
      );
      focusNode.onKeyEvent = null;
      await tester.pumpWidget(focusWidget);
      expect(focusNode.onKey, equals(ignoreCallback));
      expect(focusWidget.onKey, equals(focusNode.onKey));
      expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
      expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
      expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
      expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

      Focus.of(key1.currentContext!).requestFocus();
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      expect(keyEventHandled, isNull);

      focusWidget = Focus(
        onKey: handleCallback,
        focusNode: focusNode,
        skipTraversal: true,
        canRequestFocus: true,
        child: Container(key: key1),
      );
      await tester.pumpWidget(focusWidget);
      expect(focusNode.onKey, equals(handleCallback));
      expect(focusWidget.onKey, equals(focusNode.onKey));
      expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
      expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
      expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
      expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      expect(keyEventHandled, isTrue);
    });

    testWidgets('Focus updates the onKeyEvent handler when the widget updates', (
      WidgetTester tester,
    ) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? keyEventHandled;
      KeyEventResult handleEventCallback(FocusNode node, KeyEvent event) {
        keyEventHandled = true;
        return KeyEventResult.handled;
      }

      KeyEventResult ignoreEventCallback(FocusNode node, KeyEvent event) => KeyEventResult.ignored;
      Focus focusWidget = Focus(
        onKeyEvent: ignoreEventCallback, // This one does nothing.
        focusNode: focusNode,
        skipTraversal: true,
        canRequestFocus: true,
        child: Container(key: key1),
      );
      focusNode.onKeyEvent = null;
      await tester.pumpWidget(focusWidget);
      expect(focusNode.onKeyEvent, equals(ignoreEventCallback));
      expect(focusWidget.onKey, equals(focusNode.onKey));
      expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
      expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
      expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
      expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

      Focus.of(key1.currentContext!).requestFocus();
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      expect(keyEventHandled, isNull);

      focusWidget = Focus(
        onKeyEvent: handleEventCallback,
        focusNode: focusNode,
        skipTraversal: true,
        canRequestFocus: true,
        child: Container(key: key1),
      );
      await tester.pumpWidget(focusWidget);
      expect(focusNode.onKeyEvent, equals(handleEventCallback));
      expect(focusWidget.onKey, equals(focusNode.onKey));
      expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
      expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
      expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
      expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      expect(keyEventHandled, isTrue);
    });

    testWidgets(
      "Focus doesn't update the focusNode attributes when the widget updates if withExternalFocusNode is used",
      (WidgetTester tester) async {
        final GlobalKey key1 = GlobalKey(debugLabel: '1');
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);
        bool? keyEventHandled;
        KeyEventResult handleCallback(FocusNode node, RawKeyEvent event) {
          keyEventHandled = true;
          return KeyEventResult.handled;
        }

        KeyEventResult handleEventCallback(FocusNode node, KeyEvent event) {
          keyEventHandled = true;
          return KeyEventResult.handled;
        }

        KeyEventResult ignoreCallback(FocusNode node, RawKeyEvent event) => KeyEventResult.ignored;
        KeyEventResult ignoreEventCallback(FocusNode node, KeyEvent event) =>
            KeyEventResult.ignored;
        focusNode.onKey = ignoreCallback;
        focusNode.onKeyEvent = ignoreEventCallback;
        focusNode.descendantsAreFocusable = false;
        focusNode.descendantsAreTraversable = false;
        focusNode.skipTraversal = false;
        focusNode.canRequestFocus = true;
        Focus focusWidget = Focus.withExternalFocusNode(
          focusNode: focusNode,
          child: Container(key: key1),
        );
        await tester.pumpWidget(focusWidget);
        expect(focusNode.onKey, equals(ignoreCallback));
        expect(focusNode.onKeyEvent, equals(ignoreEventCallback));
        expect(focusNode.descendantsAreFocusable, isFalse);
        expect(focusNode.descendantsAreTraversable, isFalse);
        expect(focusNode.skipTraversal, isFalse);
        expect(focusNode.canRequestFocus, isTrue);
        expect(focusWidget.onKey, equals(focusNode.onKey));
        expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
        expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
        expect(focusWidget.descendantsAreTraversable, equals(focusNode.descendantsAreTraversable));
        expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
        expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

        Focus.of(key1.currentContext!).requestFocus();
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.enter);
        expect(keyEventHandled, isNull);

        focusNode.onKey = handleCallback;
        focusNode.onKeyEvent = handleEventCallback;
        focusNode.descendantsAreFocusable = true;
        focusNode.descendantsAreTraversable = true;
        focusWidget = Focus.withExternalFocusNode(
          focusNode: focusNode,
          child: Container(key: key1),
        );
        await tester.pumpWidget(focusWidget);
        expect(focusNode.onKey, equals(handleCallback));
        expect(focusNode.onKeyEvent, equals(handleEventCallback));
        expect(focusNode.descendantsAreFocusable, isTrue);
        expect(focusNode.descendantsAreTraversable, isTrue);
        expect(focusNode.skipTraversal, isFalse);
        expect(focusNode.canRequestFocus, isTrue);
        expect(focusWidget.onKey, equals(focusNode.onKey));
        expect(focusWidget.onKeyEvent, equals(focusNode.onKeyEvent));
        expect(focusWidget.descendantsAreFocusable, equals(focusNode.descendantsAreFocusable));
        expect(focusWidget.descendantsAreTraversable, equals(focusNode.descendantsAreTraversable));
        expect(focusWidget.skipTraversal, equals(focusNode.skipTraversal));
        expect(focusWidget.canRequestFocus, equals(focusNode.canRequestFocus));

        await tester.sendKeyEvent(LogicalKeyboardKey.enter);
        expect(keyEventHandled, isTrue);
      },
    );

    testWidgets(
      'Focus does not update the focusNode attributes when the widget updates if withExternalFocusNode is used 2',
      (WidgetTester tester) async {
        final TestExternalFocusNode focusNode = TestExternalFocusNode();
        assert(!focusNode.isModified);
        addTearDown(focusNode.dispose);

        final Focus focusWidget = Focus.withExternalFocusNode(
          focusNode: focusNode,
          child: Container(),
        );

        await tester.pumpWidget(focusWidget);
        expect(focusNode.isModified, isFalse);
        await tester.pumpWidget(const SizedBox());
      },
    );

    testWidgets('Focus passes changes in attribute values to its focus node', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(Focus(child: Container()));
    });

    testWidgets('Focus widget gains input focus when it gains accessibility focus', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: Focus(focusNode: focusNode, child: const Text('Test')),
        ),
      );

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                flags: <SemanticsFlag>[SemanticsFlag.isFocusable],
                actions: <SemanticsAction>[SemanticsAction.focus],
                label: 'Test',
                textDirection: TextDirection.rtl,
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      expect(focusNode.hasFocus, isFalse);
      semanticsOwner.performAction(1, SemanticsAction.focus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);
      semantics.dispose();
    });
  });

  group('ExcludeFocus', () {
    testWidgets("Descendants of ExcludeFocus aren't focusable.", (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey(debugLabel: '1');
      final GlobalKey key2 = GlobalKey(debugLabel: '2');
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      bool? gotFocus;
      await tester.pumpWidget(
        ExcludeFocus(
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
    });

    // Regression test for https://github.com/flutter/flutter/issues/61700
    testWidgets("ExcludeFocus doesn't transfer focus to another descendant.", (
      WidgetTester tester,
    ) async {
      final FocusNode parentFocusNode = FocusNode(debugLabel: 'group');
      addTearDown(parentFocusNode.dispose);
      final FocusNode focusNode1 = FocusNode(debugLabel: 'node 1');
      addTearDown(focusNode1.dispose);
      final FocusNode focusNode2 = FocusNode(debugLabel: 'node 2');
      addTearDown(focusNode2.dispose);
      await tester.pumpWidget(
        ExcludeFocus(
          excluding: false,
          child: Focus(
            focusNode: parentFocusNode,
            child: Column(
              children: <Widget>[
                Focus(autofocus: true, focusNode: focusNode1, child: Container()),
                Focus(focusNode: focusNode2, child: Container()),
              ],
            ),
          ),
        ),
      );

      await tester.pump();

      expect(parentFocusNode.hasFocus, isTrue);
      expect(focusNode1.hasPrimaryFocus, isTrue);
      expect(focusNode2.hasFocus, isFalse);

      // Move focus to the second node to create some focus history for the scope.
      focusNode2.requestFocus();
      await tester.pump();

      expect(parentFocusNode.hasFocus, isTrue);
      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasPrimaryFocus, isTrue);

      // Now turn off the focus for the subtree.
      await tester.pumpWidget(
        ExcludeFocus(
          child: Focus(
            focusNode: parentFocusNode,
            child: Column(
              children: <Widget>[
                Focus(autofocus: true, focusNode: focusNode1, child: Container()),
                Focus(focusNode: focusNode2, child: Container()),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasFocus, isFalse);
      expect(parentFocusNode.hasFocus, isFalse);
      expect(parentFocusNode.enclosingScope!.hasPrimaryFocus, isTrue);
    });

    testWidgets("ExcludeFocus doesn't introduce a Semantics node", (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(ExcludeFocus(child: Container()));
      final TestSemantics expectedSemantics = TestSemantics.root();
      expect(semantics, hasSemantics(expectedSemantics));
      semantics.dispose();
    });

    // Regression test for https://github.com/flutter/flutter/issues/92693
    testWidgets(
      'Setting parent FocusScope.canRequestFocus to false, does not set descendant Focus._internalNode._canRequestFocus to false',
      (WidgetTester tester) async {
        final FocusNode childFocusNode = FocusNode(debugLabel: 'node 1');
        addTearDown(childFocusNode.dispose);

        Widget buildFocusTree({required bool parentCanRequestFocus}) {
          return FocusScope(
            canRequestFocus: parentCanRequestFocus,
            child: Column(children: <Widget>[Focus(focusNode: childFocusNode, child: Container())]),
          );
        }

        // childFocusNode.canRequestFocus is true when parent canRequestFocus is true
        await tester.pumpWidget(buildFocusTree(parentCanRequestFocus: true));
        expect(childFocusNode.canRequestFocus, isTrue);

        // childFocusNode.canRequestFocus is false when parent canRequestFocus is false
        await tester.pumpWidget(buildFocusTree(parentCanRequestFocus: false));
        expect(childFocusNode.canRequestFocus, isFalse);

        // childFocusNode.canRequestFocus is true again when parent canRequestFocus is changed back to true
        await tester.pumpWidget(buildFocusTree(parentCanRequestFocus: true));
        expect(childFocusNode.canRequestFocus, isTrue);
      },
    );
  });
}

class TestFocus extends StatefulWidget {
  const TestFocus({
    super.key,
    this.debugLabel,
    this.name = 'a',
    this.autofocus = false,
    this.parentNode,
  });

  final String? debugLabel;
  final String name;
  final bool autofocus;
  final FocusNode? parentNode;

  @override
  TestFocusState createState() => TestFocusState();
}

class TestFocusState extends State<TestFocus> {
  late FocusNode focusNode;
  late String _label;
  bool built = false;

  @override
  void dispose() {
    focusNode.removeListener(_updateLabel);
    focusNode.dispose();
    super.dispose();
  }

  String get label =>
      focusNode.hasFocus ? '${widget.name.toUpperCase()} FOCUSED' : widget.name.toLowerCase();

  @override
  void initState() {
    super.initState();
    focusNode = FocusNode(debugLabel: widget.debugLabel);
    _label = label;
    focusNode.addListener(_updateLabel);
  }

  void _updateLabel() {
    setState(() {
      _label = label;
    });
  }

  @override
  Widget build(BuildContext context) {
    built = true;
    return GestureDetector(
      onTap: () {
        FocusScope.of(context).requestFocus(focusNode);
      },
      child: Focus(
        autofocus: widget.autofocus,
        focusNode: focusNode,
        parentNode: widget.parentNode,
        debugLabel: widget.debugLabel,
        child: Text(_label, textDirection: TextDirection.ltr),
      ),
    );
  }
}

class TestExternalFocusNode extends FocusNode {
  TestExternalFocusNode();

  bool isModified = false;

  @override
  FocusOnKeyEventCallback? get onKeyEvent => _onKeyEvent;
  FocusOnKeyEventCallback? _onKeyEvent;
  @override
  set onKeyEvent(FocusOnKeyEventCallback? newValue) {
    if (newValue != _onKeyEvent) {
      _onKeyEvent = newValue;
      isModified = true;
    }
  }

  @override
  set descendantsAreFocusable(bool newValue) {
    super.descendantsAreFocusable = newValue;
    isModified = true;
  }

  @override
  set descendantsAreTraversable(bool newValue) {
    super.descendantsAreTraversable = newValue;
    isModified = true;
  }

  @override
  set skipTraversal(bool newValue) {
    super.skipTraversal = newValue;
    isModified = true;
  }

  @override
  set canRequestFocus(bool newValue) {
    super.canRequestFocus = newValue;
    isModified = true;
  }
}
