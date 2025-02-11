// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  final GlobalKey widgetKey = GlobalKey();
  Future<BuildContext> setupWidget(WidgetTester tester) async {
    await tester.pumpWidget(Container(key: widgetKey));
    return widgetKey.currentContext!;
  }

  group(FocusNode, () {
    testWidgets('Can add children.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusNode parent = FocusNode();
      addTearDown(parent.dispose);
      final FocusAttachment parentAttachment = parent.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      parentAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      child1Attachment.reparent(parent: parent);
      expect(child1.parent, equals(parent));
      expect(parent.children.first, equals(child1));
      expect(parent.children.last, equals(child1));
      child2Attachment.reparent(parent: parent);
      expect(child1.parent, equals(parent));
      expect(child2.parent, equals(parent));
      expect(parent.children.first, equals(child1));
      expect(parent.children.last, equals(child2));
    });

    testWidgets('Can remove children.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusNode parent = FocusNode();
      addTearDown(parent.dispose);
      final FocusAttachment parentAttachment = parent.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      parentAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      child1Attachment.reparent(parent: parent);
      child2Attachment.reparent(parent: parent);
      expect(child1.parent, equals(parent));
      expect(child2.parent, equals(parent));
      expect(parent.children.first, equals(child1));
      expect(parent.children.last, equals(child2));
      child1Attachment.detach();
      expect(child1.parent, isNull);
      expect(child2.parent, equals(parent));
      expect(parent.children.first, equals(child2));
      expect(parent.children.last, equals(child2));
      child2Attachment.detach();
      expect(child1.parent, isNull);
      expect(child2.parent, isNull);
      expect(parent.children, isEmpty);
    });

    testWidgets('Geometry is transformed properly.', (WidgetTester tester) async {
      final FocusNode focusNode1 = FocusNode(debugLabel: 'Test Node 1');
      addTearDown(focusNode1.dispose);
      final FocusNode focusNode2 = FocusNode(debugLabel: 'Test Node 2');
      addTearDown(focusNode2.dispose);

      await tester.pumpWidget(
        Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            children: <Widget>[
              Focus(focusNode: focusNode1, child: const SizedBox(width: 200, height: 100)),
              Transform.translate(
                offset: const Offset(10, 20),
                child: Transform.scale(
                  scale: 0.33,
                  child: Transform.rotate(
                    angle: math.pi,
                    child: Focus(
                      focusNode: focusNode2,
                      child: const SizedBox(width: 200, height: 100),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      );
      focusNode2.requestFocus();
      await tester.pump();

      expect(focusNode1.rect, equals(const Rect.fromLTRB(300.0, 8.0, 500.0, 108.0)));
      expect(focusNode2.rect, equals(const Rect.fromLTRB(443.0, 194.5, 377.0, 161.5)));
      expect(focusNode1.size, equals(const Size(200.0, 100.0)));
      expect(focusNode2.size, equals(const Size(-66.0, -33.0)));
      expect(focusNode1.offset, equals(const Offset(300.0, 8.0)));
      expect(focusNode2.offset, equals(const Offset(443.0, 194.5)));
    });

    testWidgets('descendantsAreFocusable disables focus for descendants.', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent2);
      child1.requestFocus();
      await tester.pump();

      expect(tester.binding.focusManager.primaryFocus, equals(child1));
      expect(scope.focusedChild, equals(child1));
      expect(scope.traversalDescendants.contains(child1), isTrue);
      expect(scope.traversalDescendants.contains(child2), isTrue);

      parent2.descendantsAreFocusable = false;
      // Node should still be focusable, even if descendants are not.
      parent2.requestFocus();
      await tester.pump();
      expect(parent2.hasPrimaryFocus, isTrue);

      child2.requestFocus();
      await tester.pump();
      expect(tester.binding.focusManager.primaryFocus, isNot(equals(child2)));
      expect(tester.binding.focusManager.primaryFocus, equals(parent2));
      expect(scope.focusedChild, equals(parent2));
      expect(scope.traversalDescendants.contains(child1), isTrue);
      expect(scope.traversalDescendants.contains(child2), isFalse);

      parent1.descendantsAreFocusable = false;
      await tester.pump();
      expect(tester.binding.focusManager.primaryFocus, isNot(equals(child2)));
      expect(tester.binding.focusManager.primaryFocus, isNot(equals(child1)));
      expect(scope.focusedChild, equals(parent2));
      expect(scope.traversalDescendants.contains(child1), isFalse);
      expect(scope.traversalDescendants.contains(child2), isFalse);
    });

    testWidgets('descendantsAreTraversable disables traversal for descendants.', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);

      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent2);

      expect(scope.traversalDescendants, equals(<FocusNode>[child1, parent1, child2, parent2]));

      parent2.descendantsAreTraversable = false;
      expect(scope.traversalDescendants, equals(<FocusNode>[child1, parent1, parent2]));

      parent1.descendantsAreTraversable = false;
      expect(scope.traversalDescendants, equals(<FocusNode>[parent1, parent2]));

      parent1.descendantsAreTraversable = true;
      parent2.descendantsAreTraversable = true;
      scope.descendantsAreTraversable = false;
      expect(scope.traversalDescendants, equals(<FocusNode>[]));
    });

    testWidgets("canRequestFocus doesn't affect traversalChildren", (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent2);
      child1.requestFocus();
      await tester.pump();

      expect(tester.binding.focusManager.primaryFocus, equals(child1));
      expect(scope.focusedChild, equals(child1));
      expect(parent2.traversalChildren.contains(child2), isTrue);
      expect(scope.traversalChildren.contains(parent2), isTrue);

      parent2.canRequestFocus = false;
      await tester.pump();
      expect(parent2.traversalChildren.contains(child2), isTrue);
      expect(scope.traversalChildren.contains(parent2), isFalse);
    });

    testWidgets('implements debugFillProperties', (WidgetTester tester) async {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      final FocusNode focusNode = FocusNode(debugLabel: 'Label');
      addTearDown(focusNode.dispose);
      focusNode.debugFillProperties(builder);
      final List<String> description =
          builder.properties.map((DiagnosticsNode n) => n.toString()).toList();
      expect(description, <String>[
        'context: null',
        'descendantsAreFocusable: true',
        'descendantsAreTraversable: true',
        'canRequestFocus: true',
        'hasFocus: false',
        'hasPrimaryFocus: false',
      ]);
    });

    testWidgets('onKeyEvent and onKey correctly cooperate', (WidgetTester tester) async {
      final FocusNode focusNode1 = FocusNode(debugLabel: 'Test Node 1');
      addTearDown(focusNode1.dispose);
      final FocusNode focusNode2 = FocusNode(debugLabel: 'Test Node 2');
      addTearDown(focusNode2.dispose);
      final FocusNode focusNode3 = FocusNode(debugLabel: 'Test Node 3');
      addTearDown(focusNode3.dispose);
      List<List<KeyEventResult>> results = <List<KeyEventResult>>[
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
      ];
      final List<int> logs = <int>[];

      await tester.pumpWidget(
        Focus(
          focusNode: focusNode1,
          onKeyEvent: (_, KeyEvent event) {
            logs.add(0);
            return results[0][0];
          },
          onKey: (_, RawKeyEvent event) {
            logs.add(1);
            return results[0][1];
          },
          child: Focus(
            focusNode: focusNode2,
            onKeyEvent: (_, KeyEvent event) {
              logs.add(10);
              return results[1][0];
            },
            onKey: (_, RawKeyEvent event) {
              logs.add(11);
              return results[1][1];
            },
            child: Focus(
              focusNode: focusNode3,
              onKeyEvent: (_, KeyEvent event) {
                logs.add(20);
                return results[2][0];
              },
              onKey: (_, RawKeyEvent event) {
                logs.add(21);
                return results[2][1];
              },
              child: const SizedBox(width: 200, height: 100),
            ),
          ),
        ),
      );
      focusNode3.requestFocus();
      await tester.pump();

      // All ignored.
      results = <List<KeyEventResult>>[
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
      ];
      expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
      expect(logs, <int>[20, 21, 10, 11, 0, 1]);
      logs.clear();

      // The onKeyEvent should be able to stop propagation.
      results = <List<KeyEventResult>>[
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.handled, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
      ];
      expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), true);
      expect(logs, <int>[20, 21, 10, 11]);
      logs.clear();

      // The onKey should be able to stop propagation.
      results = <List<KeyEventResult>>[
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.handled],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
      ];
      expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
      expect(logs, <int>[20, 21, 10, 11]);
      logs.clear();

      // KeyEventResult.skipRemainingHandlers works.
      results = <List<KeyEventResult>>[
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.skipRemainingHandlers, KeyEventResult.ignored],
        <KeyEventResult>[KeyEventResult.ignored, KeyEventResult.ignored],
      ];
      expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
      expect(logs, <int>[20, 21, 10, 11]);
      logs.clear();
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets(
      'FocusManager ignores app lifecycle changes on Android and iOS.',
      (WidgetTester tester) async {
        Future<void> setAppLifecycleState(AppLifecycleState state) async {
          final ByteData? message = const StringCodec().encodeMessage(state.toString());
          await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
              .handlePlatformMessage('flutter/lifecycle', message, (_) {});
        }

        final BuildContext context = await setupWidget(tester);
        final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
        addTearDown(scope.dispose);
        final FocusAttachment scopeAttachment = scope.attach(context);
        final FocusNode focusNode = FocusNode(debugLabel: 'Focus Node');
        addTearDown(focusNode.dispose);
        final FocusAttachment focusNodeAttachment = focusNode.attach(context);
        scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
        focusNodeAttachment.reparent(parent: scope);
        focusNode.requestFocus();
        await tester.pump();
        expect(focusNode.hasPrimaryFocus, isTrue);

        await setAppLifecycleState(AppLifecycleState.paused);
        expect(focusNode.hasPrimaryFocus, isTrue);

        await setAppLifecycleState(AppLifecycleState.resumed);
        expect(focusNode.hasPrimaryFocus, isTrue);
      },
      skip: kIsWeb, // [intended]
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
    );

    testWidgets('FocusManager responds to app lifecycle changes.', (WidgetTester tester) async {
      Future<void> setAppLifecycleState(AppLifecycleState state) async {
        final ByteData? message = const StringCodec().encodeMessage(state.toString());
        await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
            .handlePlatformMessage('flutter/lifecycle', message, (_) {});
      }

      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode focusNode = FocusNode(debugLabel: 'Focus Node');
      addTearDown(focusNode.dispose);
      final FocusAttachment focusNodeAttachment = focusNode.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      focusNodeAttachment.reparent(parent: scope);
      focusNode.requestFocus();
      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      await setAppLifecycleState(AppLifecycleState.paused);
      expect(focusNode.hasPrimaryFocus, isFalse);

      await setAppLifecycleState(AppLifecycleState.resumed);
      expect(focusNode.hasPrimaryFocus, isTrue);
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('Node is removed completely even if app is paused.', (WidgetTester tester) async {
      Future<void> setAppLifecycleState(AppLifecycleState state) async {
        final ByteData? message = const StringCodec().encodeMessage(state.toString());
        await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
            .handlePlatformMessage('flutter/lifecycle', message, (_) {});
      }

      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode focusNode = FocusNode(debugLabel: 'Focus Node');
      addTearDown(focusNode.dispose);
      final FocusAttachment focusNodeAttachment = focusNode.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      focusNodeAttachment.reparent(parent: scope);
      focusNode.requestFocus();
      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      await setAppLifecycleState(AppLifecycleState.paused);
      focusNodeAttachment.detach();
      expect(focusNode.hasPrimaryFocus, isFalse);

      await setAppLifecycleState(AppLifecycleState.resumed);
      expect(focusNode.hasPrimaryFocus, isFalse);
    });
  });

  group(FocusScopeNode, () {
    testWidgets('Can setFirstFocus on a scope with no manager.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      scope.attach(context);
      final FocusScopeNode parent = FocusScopeNode(debugLabel: 'Parent');
      addTearDown(parent.dispose);
      parent.attach(context);
      final FocusScopeNode child1 = FocusScopeNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusScopeNode child2 = FocusScopeNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      child2.attach(context);
      scope.setFirstFocus(parent);
      parent.setFirstFocus(child1);
      parent.setFirstFocus(child2);
      child1.requestFocus();
      await tester.pump();
      expect(scope.hasFocus, isFalse);
      expect(child1.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(scope.focusedChild, equals(parent));
      expect(parent.focusedChild, equals(child1));
      child1Attachment.detach();
      expect(scope.hasFocus, isFalse);
      expect(scope.focusedChild, equals(parent));
    });

    testWidgets('Removing a node removes it from scope.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode();
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent = FocusNode();
      addTearDown(parent.dispose);
      final FocusAttachment parentAttachment = parent.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parentAttachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent);
      child2Attachment.reparent(parent: parent);
      child1.requestFocus();
      await tester.pump();
      expect(scope.hasFocus, isTrue);
      expect(child1.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(scope.focusedChild, equals(child1));
      child1Attachment.detach();
      expect(scope.hasFocus, isFalse);
      expect(scope.focusedChild, isNull);
    });

    testWidgets('Can add children to scope and focus', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode();
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent = FocusNode();
      addTearDown(parent.dispose);
      final FocusAttachment parentAttachment = parent.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parentAttachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent);
      child2Attachment.reparent(parent: parent);
      expect(scope.children.first, equals(parent));
      expect(parent.parent, equals(scope));
      expect(child1.parent, equals(parent));
      expect(child2.parent, equals(parent));
      expect(parent.children.first, equals(child1));
      expect(parent.children.last, equals(child2));
      child1.requestFocus();
      await tester.pump();
      expect(scope.focusedChild, equals(child1));
      expect(parent.hasFocus, isTrue);
      expect(parent.hasPrimaryFocus, isFalse);
      expect(child1.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(child2.hasFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);
      child2.requestFocus();
      await tester.pump();
      expect(scope.focusedChild, equals(child2));
      expect(parent.hasFocus, isTrue);
      expect(parent.hasPrimaryFocus, isFalse);
      expect(child1.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasFocus, isTrue);
      expect(child2.hasPrimaryFocus, isTrue);
    });

    // Regression test for https://github.com/flutter/flutter/issues/136758
    testWidgets('removing grandchildren from scope updates focusedChild', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);

      // Sets up this focus node tree:
      //
      //  root
      //   |
      // scope1
      //   |
      // child1
      //   |
      // child2
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1');
      addTearDown(scope1.dispose);
      final FocusAttachment scope2Attachment = scope1.attach(context);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);

      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child2Attachment = child1.attach(context);

      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child3Attachment = child2.attach(context);

      child2Attachment.reparent(parent: scope1);
      child3Attachment.reparent(parent: child1);
      expect(child1.parent, equals(scope1));
      expect(scope1.children.first, equals(child1));
      child2.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child2));

      // Detach the middle child and make sure that the scope is updated so that
      // it no longer references child2 as the focused child.
      child2Attachment.detach();
      expect(scope1.focusedChild, isNull);
    });

    testWidgets('Requesting focus before adding to tree results in a request after adding', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode();
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode child = FocusNode();
      addTearDown(child.dispose);
      child.requestFocus();
      expect(child.hasPrimaryFocus, isFalse); // not attached yet.

      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      await tester.pump();
      expect(scope.focusedChild, isNull);
      expect(child.hasPrimaryFocus, isFalse); // not attached yet.

      final FocusAttachment childAttachment = child.attach(context);
      expect(child.hasPrimaryFocus, isFalse); // not parented yet.
      childAttachment.reparent(parent: scope);
      await tester.pump();
      expect(
        child.hasPrimaryFocus,
        isTrue,
      ); // now attached and parented, so focus finally happened.
    });

    testWidgets('Autofocus works.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent = FocusNode(debugLabel: 'Parent');
      addTearDown(parent.dispose);
      final FocusAttachment parentAttachment = parent.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parentAttachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent);
      child2Attachment.reparent(parent: parent);

      scope.autofocus(child2);
      await tester.pump();

      expect(scope.focusedChild, equals(child2));
      expect(parent.hasFocus, isTrue);
      expect(child1.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasFocus, isTrue);
      expect(child2.hasPrimaryFocus, isTrue);
      child1.requestFocus();
      scope.autofocus(child2);

      await tester.pump();

      expect(scope.focusedChild, equals(child1));
      expect(parent.hasFocus, isTrue);
      expect(child1.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(child2.hasFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);
    });

    testWidgets('Adding a focusedChild to a scope sets scope as focusedChild in parent scope', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode();
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode();
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: scope1);
      child1Attachment.reparent(parent: scope1);
      child2Attachment.reparent(parent: scope2);
      child2.requestFocus();
      await tester.pump();
      expect(scope2.focusedChild, equals(child2));
      expect(scope1.focusedChild, equals(scope2));
      expect(child1.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasFocus, isTrue);
      expect(child2.hasPrimaryFocus, isTrue);
      child1.requestFocus();
      await tester.pump();
      expect(scope2.focusedChild, equals(child2));
      expect(scope1.focusedChild, equals(child1));
      expect(child1.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(child2.hasFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);
    });

    testWidgets('Can move node with focus without losing focus', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      expect(scope.children.first, equals(parent1));
      expect(scope.children.last, equals(parent2));
      expect(parent1.parent, equals(scope));
      expect(parent2.parent, equals(scope));
      expect(child1.parent, equals(parent1));
      expect(child2.parent, equals(parent1));
      expect(parent1.children.first, equals(child1));
      expect(parent1.children.last, equals(child2));
      child1.requestFocus();
      await tester.pump();
      child1Attachment.reparent(parent: parent2);
      await tester.pump();

      expect(scope.focusedChild, equals(child1));
      expect(child1.parent, equals(parent2));
      expect(child2.parent, equals(parent1));
      expect(parent1.children.first, equals(child2));
      expect(parent2.children.first, equals(child1));
    });

    test('FocusScopeNode.canRequestFocus affects descendantsAreFocusable', () {
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');

      scope.descendantsAreFocusable = false;
      expect(scope.descendantsAreFocusable, isFalse);
      expect(scope.canRequestFocus, isTrue);

      scope.descendantsAreFocusable = true;
      expect(scope.descendantsAreFocusable, isTrue);
      expect(scope.canRequestFocus, isTrue);

      scope.canRequestFocus = false;
      expect(scope.descendantsAreFocusable, isFalse);
      expect(scope.canRequestFocus, isFalse);

      scope.canRequestFocus = true;
      expect(scope.descendantsAreFocusable, isTrue);
      expect(scope.canRequestFocus, isTrue);
    });

    testWidgets('canRequestFocus affects children.', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child1.requestFocus();
      await tester.pump();

      expect(tester.binding.focusManager.primaryFocus, equals(child1));
      expect(scope.focusedChild, equals(child1));
      expect(scope.traversalDescendants.contains(child1), isTrue);
      expect(scope.traversalDescendants.contains(child2), isTrue);
      expect(scope.traversalChildren.contains(parent1), isTrue);
      expect(parent1.traversalChildren.contains(child2), isTrue);

      scope.canRequestFocus = false;
      await tester.pump();
      child2.requestFocus();
      await tester.pump();
      expect(tester.binding.focusManager.primaryFocus, isNot(equals(child2)));
      expect(tester.binding.focusManager.primaryFocus, isNot(equals(child1)));
      expect(scope.focusedChild, equals(child1));
      expect(scope.traversalDescendants.contains(child1), isFalse);
      expect(scope.traversalDescendants.contains(child2), isFalse);
      expect(scope.traversalChildren.contains(parent1), isFalse);
      expect(parent1.traversalChildren.contains(child2), isFalse);
    });

    testWidgets("skipTraversal doesn't affect children.", (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope');
      addTearDown(scope.dispose);
      final FocusAttachment scopeAttachment = scope.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      scopeAttachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope);
      parent2Attachment.reparent(parent: scope);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child1.requestFocus();
      await tester.pump();

      expect(tester.binding.focusManager.primaryFocus, equals(child1));
      expect(scope.focusedChild, equals(child1));
      expect(tester.binding.focusManager.rootScope.traversalDescendants.contains(scope), isTrue);
      expect(scope.traversalDescendants.contains(child1), isTrue);
      expect(scope.traversalDescendants.contains(child2), isTrue);

      scope.skipTraversal = true;
      await tester.pump();
      expect(tester.binding.focusManager.primaryFocus, equals(child1));
      expect(scope.focusedChild, equals(child1));
      expect(tester.binding.focusManager.rootScope.traversalDescendants.contains(scope), isFalse);
      expect(scope.traversalDescendants.contains(child1), isTrue);
      expect(scope.traversalDescendants.contains(child2), isTrue);
    });

    testWidgets('Can move node between scopes and lose scope focus', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1')..attach(context);
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'child3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'child4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);

      child1.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(parent2.children.contains(child1), isFalse);

      child1Attachment.reparent(parent: parent2);
      await tester.pump();
      expect(scope1.focusedChild, isNull);
      expect(parent2.children.contains(child1), isTrue);
    });

    testWidgets('ancestors and descendants are computed and recomputed properly', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1');
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'child3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'child4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);
      child4.requestFocus();
      await tester.pump();
      final FocusScopeNode rootScope = tester.binding.focusManager.rootScope;
      final List<FocusNode> preamble = <FocusNode>[
        rootScope.children.first.children.first, // The View Node,
        rootScope.children.first, // The FocusTraversal node above the view
      ];
      expect(
        child4.ancestors,
        equals(<FocusNode>[parent2, scope2, tester.binding.focusManager.rootScope]),
      );
      expect(
        rootScope.descendants,
        equals(<FocusNode>[
          ...preamble,
          child1,
          child2,
          parent1,
          scope1,
          child3,
          child4,
          parent2,
          scope2,
        ]),
      );
      scope2Attachment.reparent(parent: child2);
      await tester.pump();
      expect(
        child4.ancestors,
        equals(<FocusNode>[parent2, scope2, child2, parent1, scope1, rootScope]),
      );
      expect(
        tester.binding.focusManager.rootScope.descendants,
        equals(<FocusNode>[
          ...preamble,
          child1,
          child3,
          child4,
          parent2,
          scope2,
          child2,
          parent1,
          scope1,
        ]),
      );
    });

    testWidgets('Can move focus between scopes and keep focus', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode();
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode();
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode();
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode();
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode();
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode();
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode();
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode();
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);
      child4.requestFocus();
      await tester.pump();
      child1.requestFocus();
      await tester.pump();
      expect(child4.hasFocus, isFalse);
      expect(child4.hasPrimaryFocus, isFalse);
      expect(child1.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(scope1.hasFocus, isTrue);
      expect(scope1.hasPrimaryFocus, isFalse);
      expect(scope2.hasFocus, isFalse);
      expect(scope2.hasPrimaryFocus, isFalse);
      expect(parent1.hasFocus, isTrue);
      expect(parent2.hasFocus, isFalse);
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child4));
      scope2.requestFocus();
      await tester.pump();
      expect(child4.hasFocus, isTrue);
      expect(child4.hasPrimaryFocus, isTrue);
      expect(child1.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(scope1.hasFocus, isFalse);
      expect(scope1.hasPrimaryFocus, isFalse);
      expect(scope2.hasFocus, isTrue);
      expect(scope2.hasPrimaryFocus, isFalse);
      expect(parent1.hasFocus, isFalse);
      expect(parent2.hasFocus, isTrue);
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child4));
    });

    testWidgets('Unfocus with disposition previouslyFocusedChild works properly', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1')..attach(context);
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'child3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'child4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);

      // Build up a history.
      child4.requestFocus();
      await tester.pump();
      child2.requestFocus();
      await tester.pump();
      child3.requestFocus();
      await tester.pump();
      child1.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));

      child1.unfocus(disposition: UnfocusDisposition.previouslyFocusedChild);
      await tester.pump();
      expect(scope1.focusedChild, equals(child2));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isTrue);

      // Can re-focus child.
      child1.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(child3.hasPrimaryFocus, isFalse);

      // The same thing happens when unfocusing a second time.
      child1.unfocus(disposition: UnfocusDisposition.previouslyFocusedChild);
      await tester.pump();
      expect(scope1.focusedChild, equals(child2));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isTrue);

      // When the scope gets unfocused, then the sibling scope gets focus.
      child1.requestFocus();
      await tester.pump();
      scope1.unfocus(disposition: UnfocusDisposition.previouslyFocusedChild);
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isFalse);
      expect(scope2.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child3.hasPrimaryFocus, isTrue);
    });

    testWidgets('Unfocus with disposition scope works properly', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1')..attach(context);
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'child3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'child4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);

      // Build up a history.
      child4.requestFocus();
      await tester.pump();
      child2.requestFocus();
      await tester.pump();
      child3.requestFocus();
      await tester.pump();
      child1.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));

      child1.unfocus();
      await tester.pump();
      // Focused child doesn't change.
      expect(scope1.focusedChild, isNull);
      expect(scope2.focusedChild, equals(child3));
      // Focus does change.
      expect(scope1.hasPrimaryFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);

      // Can re-focus child.
      child1.requestFocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isTrue);
      expect(child3.hasPrimaryFocus, isFalse);

      // The same thing happens when unfocusing a second time.
      child1.unfocus();
      await tester.pump();
      expect(scope1.focusedChild, isNull);
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasPrimaryFocus, isTrue);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);

      // When the scope gets unfocused, then its parent scope (the root scope)
      // gets focus, but it doesn't mess with the focused children.
      child1.requestFocus();
      await tester.pump();
      scope1.unfocus();
      await tester.pump();
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasFocus, isFalse);
      expect(scope2.hasFocus, isFalse);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child3.hasPrimaryFocus, isFalse);
      expect(FocusManager.instance.rootScope.hasPrimaryFocus, isTrue);
    });

    testWidgets('Unfocus works properly when some nodes are unfocusable', (
      WidgetTester tester,
    ) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1')..attach(context);
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(debugLabel: 'child2');
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'child3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'child4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);

      // Build up a history.
      child4.requestFocus();
      await tester.pump();
      child2.requestFocus();
      await tester.pump();
      child3.requestFocus();
      await tester.pump();
      child1.requestFocus();
      await tester.pump();
      expect(child1.hasPrimaryFocus, isTrue);

      scope1.canRequestFocus = false;
      await tester.pump();

      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(child3.hasPrimaryFocus, isTrue);

      child1.unfocus();
      await tester.pump();
      expect(child3.hasPrimaryFocus, isTrue);
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasPrimaryFocus, isFalse);
      expect(scope2.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);

      child1.unfocus(disposition: UnfocusDisposition.previouslyFocusedChild);
      await tester.pump();
      expect(child3.hasPrimaryFocus, isTrue);
      expect(scope1.focusedChild, equals(child1));
      expect(scope2.focusedChild, equals(child3));
      expect(scope1.hasPrimaryFocus, isFalse);
      expect(scope2.hasFocus, isTrue);
      expect(child1.hasPrimaryFocus, isFalse);
      expect(child2.hasPrimaryFocus, isFalse);
    });

    testWidgets(
      'Requesting focus on a scope works properly when some focusedChild nodes are unfocusable',
      (WidgetTester tester) async {
        final BuildContext context = await setupWidget(tester);
        final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1')..attach(context);
        addTearDown(scope1.dispose);
        final FocusAttachment scope1Attachment = scope1.attach(context);
        final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
        addTearDown(scope2.dispose);
        final FocusAttachment scope2Attachment = scope2.attach(context);
        final FocusNode parent1 = FocusNode(debugLabel: 'parent1');
        addTearDown(parent1.dispose);
        final FocusAttachment parent1Attachment = parent1.attach(context);
        final FocusNode parent2 = FocusNode(debugLabel: 'parent2');
        addTearDown(parent2.dispose);
        final FocusAttachment parent2Attachment = parent2.attach(context);
        final FocusNode child1 = FocusNode(debugLabel: 'child1');
        addTearDown(child1.dispose);
        final FocusAttachment child1Attachment = child1.attach(context);
        final FocusNode child2 = FocusNode(debugLabel: 'child2');
        addTearDown(child2.dispose);
        final FocusAttachment child2Attachment = child2.attach(context);
        final FocusNode child3 = FocusNode(debugLabel: 'child3');
        addTearDown(child3.dispose);
        final FocusAttachment child3Attachment = child3.attach(context);
        final FocusNode child4 = FocusNode(debugLabel: 'child4');
        addTearDown(child4.dispose);
        final FocusAttachment child4Attachment = child4.attach(context);
        scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
        scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
        parent1Attachment.reparent(parent: scope1);
        parent2Attachment.reparent(parent: scope2);
        child1Attachment.reparent(parent: parent1);
        child2Attachment.reparent(parent: parent1);
        child3Attachment.reparent(parent: parent2);
        child4Attachment.reparent(parent: parent2);

        // Build up a history.
        child4.requestFocus();
        await tester.pump();
        child2.requestFocus();
        await tester.pump();
        child3.requestFocus();
        await tester.pump();
        child1.requestFocus();
        await tester.pump();
        expect(child1.hasPrimaryFocus, isTrue);

        child1.canRequestFocus = false;
        child3.canRequestFocus = false;
        await tester.pump();
        scope1.requestFocus();
        await tester.pump();

        expect(scope1.focusedChild, equals(child2));
        expect(child2.hasPrimaryFocus, isTrue);

        scope2.requestFocus();
        await tester.pump();

        expect(scope2.focusedChild, equals(child4));
        expect(child4.hasPrimaryFocus, isTrue);
      },
    );

    testWidgets(
      'Key handling bubbles up and terminates when handled.',
      (WidgetTester tester) async {
        final Set<FocusNode> receivedAnEvent = <FocusNode>{};
        final Set<FocusNode> shouldHandle = <FocusNode>{};
        KeyEventResult handleEvent(FocusNode node, RawKeyEvent event) {
          if (shouldHandle.contains(node)) {
            receivedAnEvent.add(node);
            return KeyEventResult.handled;
          }
          return KeyEventResult.ignored;
        }

        Future<void> sendEvent() async {
          receivedAnEvent.clear();
          await tester.sendKeyEvent(LogicalKeyboardKey.metaLeft, platform: 'fuchsia');
        }

        final BuildContext context = await setupWidget(tester);
        final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'Scope 1');
        addTearDown(scope1.dispose);
        final FocusAttachment scope1Attachment = scope1.attach(context, onKey: handleEvent);
        final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'Scope 2');
        addTearDown(scope2.dispose);
        final FocusAttachment scope2Attachment = scope2.attach(context, onKey: handleEvent);
        final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1', onKey: handleEvent);
        addTearDown(parent1.dispose);
        final FocusAttachment parent1Attachment = parent1.attach(context);
        final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2', onKey: handleEvent);
        addTearDown(parent2.dispose);
        final FocusAttachment parent2Attachment = parent2.attach(context);
        final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
        addTearDown(child1.dispose);
        final FocusAttachment child1Attachment = child1.attach(context, onKey: handleEvent);
        final FocusNode child2 = FocusNode(debugLabel: 'Child 2');
        addTearDown(child2.dispose);
        final FocusAttachment child2Attachment = child2.attach(context, onKey: handleEvent);
        final FocusNode child3 = FocusNode(debugLabel: 'Child 3');
        addTearDown(child3.dispose);
        final FocusAttachment child3Attachment = child3.attach(context, onKey: handleEvent);
        final FocusNode child4 = FocusNode(debugLabel: 'Child 4');
        addTearDown(child4.dispose);
        final FocusAttachment child4Attachment = child4.attach(context, onKey: handleEvent);
        scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
        scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
        parent1Attachment.reparent(parent: scope1);
        parent2Attachment.reparent(parent: scope2);
        child1Attachment.reparent(parent: parent1);
        child2Attachment.reparent(parent: parent1);
        child3Attachment.reparent(parent: parent2);
        child4Attachment.reparent(parent: parent2);
        child4.requestFocus();
        await tester.pump();
        shouldHandle.addAll(<FocusNode>{scope2, parent2, child2, child4});
        await sendEvent();
        expect(receivedAnEvent, equals(<FocusNode>{child4}));
        shouldHandle.remove(child4);
        await sendEvent();
        expect(receivedAnEvent, equals(<FocusNode>{parent2}));
        shouldHandle.remove(parent2);
        await sendEvent();
        expect(receivedAnEvent, equals(<FocusNode>{scope2}));
        shouldHandle.clear();
        await sendEvent();
        expect(receivedAnEvent, isEmpty);
        child1.requestFocus();
        await tester.pump();
        shouldHandle.addAll(<FocusNode>{scope2, parent2, child2, child4});
        await sendEvent();
        // Since none of the focused nodes handle this event, nothing should
        // receive it.
        expect(receivedAnEvent, isEmpty);
      },
      variant: KeySimulatorTransitModeVariant.all(),
    );

    testWidgets('Initial highlight mode guesses correctly.', (WidgetTester tester) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.automatic;
      switch (defaultTargetPlatform) {
        case TargetPlatform.fuchsia:
        case TargetPlatform.android:
        case TargetPlatform.iOS:
          expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
      }
    }, variant: TargetPlatformVariant.all());

    testWidgets(
      'Mouse events change initial focus highlight mode on mobile.',
      (WidgetTester tester) async {
        expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
        RendererBinding.instance.initMouseTracker(); // Clear out the mouse state.
        final TestGesture gesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
          pointer: 0,
        );
        await gesture.moveTo(Offset.zero);
        expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
      },
      variant: TargetPlatformVariant.mobile(),
    );

    testWidgets(
      'Mouse events change initial focus highlight mode on desktop.',
      (WidgetTester tester) async {
        expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
        RendererBinding.instance.initMouseTracker(); // Clear out the mouse state.
        final TestGesture gesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
          pointer: 0,
        );
        await gesture.moveTo(Offset.zero);
        expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets('Keyboard events change initial focus highlight mode.', (
      WidgetTester tester,
    ) async {
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
    }, variant: TargetPlatformVariant.all());

    testWidgets('Events change focus highlight mode.', (WidgetTester tester) async {
      await setupWidget(tester);
      int callCount = 0;
      FocusHighlightMode? lastMode;
      void handleModeChange(FocusHighlightMode mode) {
        lastMode = mode;
        callCount++;
      }

      FocusManager.instance.addHighlightModeListener(handleModeChange);
      addTearDown(() => FocusManager.instance.removeHighlightModeListener(handleModeChange));
      expect(callCount, equals(0));
      expect(lastMode, isNull);
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.automatic;
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
      await tester.sendKeyEvent(LogicalKeyboardKey.metaLeft, platform: 'fuchsia');
      expect(callCount, equals(1));
      expect(lastMode, FocusHighlightMode.traditional);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
      await tester.tap(find.byType(Container), warnIfMissed: false);
      expect(callCount, equals(2));
      expect(lastMode, FocusHighlightMode.touch);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
      final TestGesture gesture = await tester.startGesture(
        Offset.zero,
        kind: PointerDeviceKind.mouse,
      );
      await gesture.up();
      expect(callCount, equals(2));
      expect(lastMode, FocusHighlightMode.touch);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
      await tester.tap(find.byType(Container), warnIfMissed: false);
      expect(callCount, equals(2));
      expect(lastMode, FocusHighlightMode.touch);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      expect(callCount, equals(3));
      expect(lastMode, FocusHighlightMode.traditional);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
      expect(callCount, equals(4));
      expect(lastMode, FocusHighlightMode.touch);
      expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
    });

    testWidgets('Scopes can be focused without sending focus to descendants.', (
      WidgetTester tester,
    ) async {
      final FocusScopeNode scopeNode = FocusScopeNode(debugLabel: 'Scope1');
      addTearDown(scopeNode.dispose);
      final FocusNode childFocusNode = FocusNode(debugLabel: 'Child1');
      addTearDown(childFocusNode.dispose);
      await tester.pumpWidget(
        FocusScope.withExternalFocusNode(
          focusScopeNode: scopeNode,
          child: Focus(
            debugLabel: 'Parent1',
            child: FocusScope(
              debugLabel: 'Scope2',
              child: Focus.withExternalFocusNode(
                focusNode: childFocusNode,
                child: const SizedBox(),
              ),
            ),
          ),
        ),
      );

      childFocusNode.requestFocus();
      await tester.pump();
      expect(scopeNode.hasFocus, isTrue);
      expect(childFocusNode.hasPrimaryFocus, isTrue);

      scopeNode.requestScopeFocus();
      await tester.pump();
      expect(scopeNode.hasPrimaryFocus, isTrue);
      expect(childFocusNode.hasPrimaryFocus, isFalse);
    });

    testWidgets('implements debugFillProperties', (WidgetTester tester) async {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      final FocusScopeNode scope = FocusScopeNode(debugLabel: 'Scope Label');
      addTearDown(scope.dispose);
      scope.debugFillProperties(builder);
      final List<String> description =
          builder.properties.map((DiagnosticsNode n) => n.toString()).toList();
      expect(description, <String>[
        'context: null',
        'descendantsAreFocusable: true',
        'descendantsAreTraversable: true',
        'canRequestFocus: true',
        'hasFocus: false',
        'hasPrimaryFocus: false',
      ]);
    });

    testWidgets('debugDescribeFocusTree produces correct output', (WidgetTester tester) async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'Scope 1');
      addTearDown(scope1.dispose);
      final FocusAttachment scope1Attachment = scope1.attach(context);
      final FocusScopeNode scope2 = FocusScopeNode(); // No label, Just to test that it works.
      addTearDown(scope2.dispose);
      final FocusAttachment scope2Attachment = scope2.attach(context);
      final FocusNode parent1 = FocusNode(debugLabel: 'Parent 1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode parent2 = FocusNode(debugLabel: 'Parent 2');
      addTearDown(parent2.dispose);
      final FocusAttachment parent2Attachment = parent2.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'Child 1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      final FocusNode child2 = FocusNode(); // No label, Just to test that it works.
      addTearDown(child2.dispose);
      final FocusAttachment child2Attachment = child2.attach(context);
      final FocusNode child3 = FocusNode(debugLabel: 'Child 3');
      addTearDown(child3.dispose);
      final FocusAttachment child3Attachment = child3.attach(context);
      final FocusNode child4 = FocusNode(debugLabel: 'Child 4');
      addTearDown(child4.dispose);
      final FocusAttachment child4Attachment = child4.attach(context);
      scope1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      scope2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      parent1Attachment.reparent(parent: scope1);
      parent2Attachment.reparent(parent: scope2);
      child1Attachment.reparent(parent: parent1);
      child2Attachment.reparent(parent: parent1);
      child3Attachment.reparent(parent: parent2);
      child4Attachment.reparent(parent: parent2);
      child4.requestFocus();
      await tester.pump();
      final String description = debugDescribeFocusTree();
      expect(
        description,
        equalsIgnoringHashCodes(
          'FocusManager#00000\n'
          ' │ primaryFocus: FocusNode#00000(Child 4 [PRIMARY FOCUS])\n'
          ' │ primaryFocusCreator: Container-[GlobalKey#00000] ←\n'
          ' │   _FocusInheritedScope ← _FocusScopeWithExternalFocusNode ←\n'
          ' │   _FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery\n'
          ' │   ← _MediaQueryFromView ← _PipelineOwnerScope ← _ViewScope ←\n'
          ' │   _RawViewInternal-[_DeprecatedRawViewKey TestFlutterView#00000]\n'
          ' │   ← RawView ← View ← [root]\n'
          ' │\n'
          ' └─rootScope: FocusScopeNode#00000(Root Focus Scope [IN FOCUS PATH])\n'
          '   │ IN FOCUS PATH\n'
          '   │ focusedChildren: FocusScopeNode#00000([IN FOCUS PATH])\n'
          '   │\n'
          '   ├─Child 1: _FocusTraversalGroupNode#00000(FocusTraversalGroup)\n'
          '   │ │ context: Focus\n'
          '   │ │ NOT FOCUSABLE\n'
          '   │ │\n'
          '   │ └─Child 1: FocusScopeNode#00000(View Scope)\n'
          '   │     context: _FocusScopeWithExternalFocusNode\n'
          '   │\n'
          '   ├─Child 2: FocusScopeNode#00000(Scope 1)\n'
          '   │ │ context: Container-[GlobalKey#00000]\n'
          '   │ │\n'
          '   │ └─Child 1: FocusNode#00000(Parent 1)\n'
          '   │   │ context: Container-[GlobalKey#00000]\n'
          '   │   │\n'
          '   │   ├─Child 1: FocusNode#00000(Child 1)\n'
          '   │   │   context: Container-[GlobalKey#00000]\n'
          '   │   │\n'
          '   │   └─Child 2: FocusNode#00000\n'
          '   │       context: Container-[GlobalKey#00000]\n'
          '   │\n'
          '   └─Child 3: FocusScopeNode#00000([IN FOCUS PATH])\n'
          '     │ context: Container-[GlobalKey#00000]\n'
          '     │ IN FOCUS PATH\n'
          '     │ focusedChildren: FocusNode#00000(Child 4 [PRIMARY FOCUS])\n'
          '     │\n'
          '     └─Child 1: FocusNode#00000(Parent 2 [IN FOCUS PATH])\n'
          '       │ context: Container-[GlobalKey#00000]\n'
          '       │ IN FOCUS PATH\n'
          '       │\n'
          '       ├─Child 1: FocusNode#00000(Child 3)\n'
          '       │   context: Container-[GlobalKey#00000]\n'
          '       │\n'
          '       └─Child 2: FocusNode#00000(Child 4 [PRIMARY FOCUS])\n'
          '           context: Container-[GlobalKey#00000]\n'
          '           PRIMARY FOCUS\n',
        ),
      );
    });
  });

  group('Autofocus', () {
    testWidgets('works when the previous focused node is detached', (WidgetTester tester) async {
      final FocusNode node1 = FocusNode();
      addTearDown(node1.dispose);
      final FocusNode node2 = FocusNode();
      addTearDown(node2.dispose);

      await tester.pumpWidget(
        FocusScope(child: Focus(autofocus: true, focusNode: node1, child: const Placeholder())),
      );
      await tester.pump();
      expect(node1.hasPrimaryFocus, isTrue);

      await tester.pumpWidget(
        FocusScope(
          child: SizedBox(
            child: Focus(autofocus: true, focusNode: node2, child: const Placeholder()),
          ),
        ),
      );
      await tester.pump();
      expect(node2.hasPrimaryFocus, isTrue);
    });

    testWidgets('node detached before autofocus is applied', (WidgetTester tester) async {
      final FocusScopeNode scopeNode = FocusScopeNode();
      addTearDown(scopeNode.dispose);
      final FocusNode node1 = FocusNode();
      addTearDown(node1.dispose);

      await tester.pumpWidget(
        FocusScope(
          node: scopeNode,
          child: Focus(autofocus: true, focusNode: node1, child: const Placeholder()),
        ),
      );
      await tester.pumpWidget(
        FocusScope(node: scopeNode, child: const Focus(child: Placeholder())),
      );

      await tester.pump();
      expect(node1.hasPrimaryFocus, isFalse);
      expect(scopeNode.hasPrimaryFocus, isTrue);
    });

    testWidgets('autofocus the first candidate', (WidgetTester tester) async {
      final FocusNode node1 = FocusNode();
      addTearDown(node1.dispose);
      final FocusNode node2 = FocusNode();
      addTearDown(node2.dispose);
      final FocusNode node3 = FocusNode();
      addTearDown(node3.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Column(
            children: <Focus>[
              Focus(autofocus: true, focusNode: node1, child: const SizedBox()),
              Focus(autofocus: true, focusNode: node2, child: const SizedBox()),
              Focus(autofocus: true, focusNode: node3, child: const SizedBox()),
            ],
          ),
        ),
      );

      expect(node1.hasPrimaryFocus, isTrue);
    });

    testWidgets('Autofocus works with global key reparenting', (WidgetTester tester) async {
      final FocusNode node = FocusNode();
      addTearDown(node.dispose);
      final FocusScopeNode scope1 = FocusScopeNode(debugLabel: 'scope1');
      addTearDown(scope1.dispose);
      final FocusScopeNode scope2 = FocusScopeNode(debugLabel: 'scope2');
      addTearDown(scope2.dispose);
      final GlobalKey key = GlobalKey();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Column(
            children: <Focus>[
              FocusScope(
                node: scope1,
                child: Focus(key: key, focusNode: node, child: const SizedBox()),
              ),
              FocusScope(node: scope2, child: const SizedBox()),
            ],
          ),
        ),
      );

      // _applyFocusChange will be called before persistentCallbacks,
      // guaranteeing the focus changes are applied before the BuildContext
      // `node` attaches to gets reparented.
      scope1.autofocus(node);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Column(
            children: <Focus>[
              FocusScope(node: scope1, child: const SizedBox()),
              FocusScope(
                node: scope2,
                child: Focus(key: key, focusNode: node, child: const SizedBox()),
              ),
            ],
          ),
        ),
      );

      expect(node.hasPrimaryFocus, isTrue);
      expect(scope2.hasFocus, isTrue);
    });
  });

  testWidgets("Doesn't lose focused child when reparenting if the nearestScope doesn't change.", (
    WidgetTester tester,
  ) async {
    final BuildContext context = await setupWidget(tester);
    final FocusScopeNode parent1 = FocusScopeNode(debugLabel: 'parent1');
    addTearDown(parent1.dispose);
    final FocusScopeNode parent2 = FocusScopeNode(debugLabel: 'parent2');
    addTearDown(parent2.dispose);
    final FocusAttachment parent1Attachment = parent1.attach(context);
    final FocusAttachment parent2Attachment = parent2.attach(context);
    final FocusNode child1 = FocusNode(debugLabel: 'child1');
    addTearDown(child1.dispose);
    final FocusAttachment child1Attachment = child1.attach(context);
    final FocusNode child2 = FocusNode(debugLabel: 'child2');
    addTearDown(child2.dispose);
    final FocusAttachment child2Attachment = child2.attach(context);
    parent1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
    child1Attachment.reparent(parent: parent1);
    child2Attachment.reparent(parent: child1);
    parent1.autofocus(child2);
    await tester.pump();
    parent2Attachment.reparent(parent: tester.binding.focusManager.rootScope);
    parent2.requestFocus();
    await tester.pump();
    expect(parent1.focusedChild, equals(child2));
    child2Attachment.reparent(parent: parent1);
    expect(parent1.focusedChild, equals(child2));
    parent1.requestFocus();
    await tester.pump();
    expect(parent1.focusedChild, equals(child2));
  });

  testWidgets('Ancestors get notified exactly as often as needed if focused child changes focus.', (
    WidgetTester tester,
  ) async {
    bool topFocus = false;
    bool parent1Focus = false;
    bool parent2Focus = false;
    bool child1Focus = false;
    bool child2Focus = false;
    int topNotify = 0;
    int parent1Notify = 0;
    int parent2Notify = 0;
    int child1Notify = 0;
    int child2Notify = 0;
    void clear() {
      topFocus = false;
      parent1Focus = false;
      parent2Focus = false;
      child1Focus = false;
      child2Focus = false;
      topNotify = 0;
      parent1Notify = 0;
      parent2Notify = 0;
      child1Notify = 0;
      child2Notify = 0;
    }

    final BuildContext context = await setupWidget(tester);
    final FocusScopeNode top = FocusScopeNode(debugLabel: 'top');
    addTearDown(top.dispose);
    final FocusAttachment topAttachment = top.attach(context);
    final FocusScopeNode parent1 = FocusScopeNode(debugLabel: 'parent1');
    addTearDown(parent1.dispose);
    final FocusAttachment parent1Attachment = parent1.attach(context);
    final FocusScopeNode parent2 = FocusScopeNode(debugLabel: 'parent2');
    addTearDown(parent2.dispose);
    final FocusAttachment parent2Attachment = parent2.attach(context);
    final FocusNode child1 = FocusNode(debugLabel: 'child1');
    addTearDown(child1.dispose);
    final FocusAttachment child1Attachment = child1.attach(context);
    final FocusNode child2 = FocusNode(debugLabel: 'child2');
    addTearDown(child2.dispose);
    final FocusAttachment child2Attachment = child2.attach(context);
    topAttachment.reparent(parent: tester.binding.focusManager.rootScope);
    parent1Attachment.reparent(parent: top);
    parent2Attachment.reparent(parent: top);
    child1Attachment.reparent(parent: parent1);
    child2Attachment.reparent(parent: parent2);
    top.addListener(() {
      topNotify++;
      topFocus = top.hasFocus;
    });
    parent1.addListener(() {
      parent1Notify++;
      parent1Focus = parent1.hasFocus;
    });
    parent2.addListener(() {
      parent2Notify++;
      parent2Focus = parent2.hasFocus;
    });
    child1.addListener(() {
      child1Notify++;
      child1Focus = child1.hasFocus;
    });
    child2.addListener(() {
      child2Notify++;
      child2Focus = child2.hasFocus;
    });
    child1.requestFocus();
    await tester.pump();
    expect(topFocus, isTrue);
    expect(parent1Focus, isTrue);
    expect(child1Focus, isTrue);
    expect(parent2Focus, isFalse);
    expect(child2Focus, isFalse);
    expect(topNotify, equals(1));
    expect(parent1Notify, equals(1));
    expect(child1Notify, equals(1));
    expect(parent2Notify, equals(0));
    expect(child2Notify, equals(0));

    clear();
    child1.unfocus();
    await tester.pump();
    expect(topFocus, isFalse);
    expect(parent1Focus, isTrue);
    expect(child1Focus, isFalse);
    expect(parent2Focus, isFalse);
    expect(child2Focus, isFalse);
    expect(topNotify, equals(0));
    expect(parent1Notify, equals(1));
    expect(child1Notify, equals(1));
    expect(parent2Notify, equals(0));
    expect(child2Notify, equals(0));

    clear();
    child1.requestFocus();
    await tester.pump();
    expect(topFocus, isFalse);
    expect(parent1Focus, isTrue);
    expect(child1Focus, isTrue);
    expect(parent2Focus, isFalse);
    expect(child2Focus, isFalse);
    expect(topNotify, equals(0));
    expect(parent1Notify, equals(1));
    expect(child1Notify, equals(1));
    expect(parent2Notify, equals(0));
    expect(child2Notify, equals(0));

    clear();
    child2.requestFocus();
    await tester.pump();
    expect(topFocus, isFalse);
    expect(parent1Focus, isFalse);
    expect(child1Focus, isFalse);
    expect(parent2Focus, isTrue);
    expect(child2Focus, isTrue);
    expect(topNotify, equals(0));
    expect(parent1Notify, equals(1));
    expect(child1Notify, equals(1));
    expect(parent2Notify, equals(1));
    expect(child2Notify, equals(1));

    // Changing the focus back before the pump shouldn't cause notifications.
    clear();
    child1.requestFocus();
    child2.requestFocus();
    await tester.pump();
    expect(topFocus, isFalse);
    expect(parent1Focus, isFalse);
    expect(child1Focus, isFalse);
    expect(parent2Focus, isFalse);
    expect(child2Focus, isFalse);
    expect(topNotify, equals(0));
    expect(parent1Notify, equals(0));
    expect(child1Notify, equals(0));
    expect(parent2Notify, equals(0));
    expect(child2Notify, equals(0));
  });

  testWidgets('Focus changes notify listeners.', (WidgetTester tester) async {
    final BuildContext context = await setupWidget(tester);
    final FocusScopeNode parent1 = FocusScopeNode(debugLabel: 'parent1');
    addTearDown(parent1.dispose);
    final FocusAttachment parent1Attachment = parent1.attach(context);
    final FocusNode child1 = FocusNode(debugLabel: 'child1');
    addTearDown(child1.dispose);
    final FocusAttachment child1Attachment = child1.attach(context);
    final FocusNode child2 = FocusNode(debugLabel: 'child2');
    addTearDown(child2.dispose);
    final FocusAttachment child2Attachment = child2.attach(context);
    parent1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
    child1Attachment.reparent(parent: parent1);
    child2Attachment.reparent(parent: child1);

    int notifyCount = 0;
    void handleFocusChange() {
      notifyCount++;
    }

    tester.binding.focusManager.addListener(handleFocusChange);

    parent1.autofocus(child2);
    expect(notifyCount, equals(0));
    await tester.pump();
    expect(notifyCount, equals(1));
    notifyCount = 0;

    child1.requestFocus();
    child2.requestFocus();
    child1.requestFocus();
    await tester.pump();
    expect(notifyCount, equals(1));
    notifyCount = 0;

    child2.requestFocus();
    await tester.pump();
    expect(notifyCount, equals(1));
    notifyCount = 0;

    child2.unfocus();
    await tester.pump();
    expect(notifyCount, equals(1));
    notifyCount = 0;

    tester.binding.focusManager.removeListener(handleFocusChange);
  });

  test('$FocusManager dispatches object creation in constructor', () async {
    await expectLater(
      await memoryEvents(() => FocusManager().dispose(), FocusManager),
      areCreateAndDispose,
    );
  });

  test('$FocusNode dispatches object creation in constructor', () async {
    await expectLater(
      await memoryEvents(() => FocusNode().dispose(), FocusNode),
      areCreateAndDispose,
    );
  });

  testWidgets('FocusManager.addEarlyKeyEventHandler works', (WidgetTester tester) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: 'Test Node 1');
    addTearDown(focusNode1.dispose);
    final List<int> logs = <int>[];
    KeyEventResult earlyResult = KeyEventResult.ignored;
    KeyEventResult focusResult = KeyEventResult.ignored;

    await tester.pumpWidget(
      Focus(
        focusNode: focusNode1,
        onKeyEvent: (_, KeyEvent event) {
          logs.add(0);
          if (event is KeyDownEvent) {
            return focusResult;
          }
          return KeyEventResult.ignored;
        },
        onKey: (_, RawKeyEvent event) {
          logs.add(1);
          if (event is KeyDownEvent) {
            return focusResult;
          }
          return KeyEventResult.ignored;
        },
        child: const SizedBox(),
      ),
    );
    focusNode1.requestFocus();
    await tester.pump();

    KeyEventResult earlyHandler(KeyEvent event) {
      if (event is KeyDownEvent) {
        return earlyResult;
      }
      return KeyEventResult.ignored;
    }

    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    FocusManager.instance.addEarlyKeyEventHandler(earlyHandler);
    logs.clear();
    focusResult = KeyEventResult.ignored;
    earlyResult = KeyEventResult.handled;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1]);

    logs.clear();
    focusResult = KeyEventResult.ignored;
    earlyResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    logs.clear();
    focusResult = KeyEventResult.handled;
    earlyResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    FocusManager.instance.removeEarlyKeyEventHandler(earlyHandler);
    logs.clear();
    focusResult = KeyEventResult.ignored;
    earlyResult = KeyEventResult.handled;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    logs.clear();
    focusResult = KeyEventResult.handled;
    earlyResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('FocusManager.addLateKeyEventHandler works', (WidgetTester tester) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: 'Test Node 1');
    addTearDown(focusNode1.dispose);
    final List<int> logs = <int>[];
    KeyEventResult lateResult = KeyEventResult.ignored;
    KeyEventResult focusResult = KeyEventResult.ignored;

    await tester.pumpWidget(
      Focus(
        focusNode: focusNode1,
        onKeyEvent: (_, KeyEvent event) {
          logs.add(0);
          if (event is KeyDownEvent) {
            return focusResult;
          }
          return KeyEventResult.ignored;
        },
        onKey: (_, RawKeyEvent event) {
          logs.add(1);
          if (event is KeyDownEvent) {
            return focusResult;
          }
          return KeyEventResult.ignored;
        },
        child: const SizedBox(),
      ),
    );
    focusNode1.requestFocus();
    await tester.pump();

    KeyEventResult lateHandler(KeyEvent event) {
      if (event is KeyDownEvent) {
        return lateResult;
      }
      return KeyEventResult.ignored;
    }

    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    FocusManager.instance.addLateKeyEventHandler(lateHandler);
    logs.clear();
    focusResult = KeyEventResult.ignored;
    lateResult = KeyEventResult.handled;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    logs.clear();
    focusResult = KeyEventResult.ignored;
    lateResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    logs.clear();
    focusResult = KeyEventResult.handled;
    lateResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    FocusManager.instance.removeLateKeyEventHandler(lateHandler);
    logs.clear();
    focusResult = KeyEventResult.ignored;
    lateResult = KeyEventResult.handled;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), false);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);

    logs.clear();
    focusResult = KeyEventResult.handled;
    lateResult = KeyEventResult.ignored;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.digit1), true);
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.digit1), false);
    expect(logs, <int>[0, 1, 0, 1]);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('FocusManager notifies listeners when a widget loses focus because it was removed.', (
    WidgetTester tester,
  ) async {
    final FocusNode nodeA = FocusNode(debugLabel: 'a');
    addTearDown(nodeA.dispose);
    final FocusNode nodeB = FocusNode(debugLabel: 'b');
    addTearDown(nodeB.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Column(
          children: <Widget>[
            Focus(focusNode: nodeA, child: const Text('a')),
            Focus(focusNode: nodeB, child: const Text('b')),
          ],
        ),
      ),
    );
    int notifyCount = 0;
    void handleFocusChange() {
      notifyCount++;
    }

    tester.binding.focusManager.addListener(handleFocusChange);
    addTearDown(() => tester.binding.focusManager.removeListener(handleFocusChange));

    nodeA.requestFocus();
    await tester.pump();
    expect(nodeA.hasPrimaryFocus, isTrue);
    expect(notifyCount, equals(1));
    notifyCount = 0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Column(children: <Widget>[Focus(focusNode: nodeB, child: const Text('b'))]),
      ),
    );

    await tester.pump();
    expect(nodeA.hasPrimaryFocus, isFalse);
    expect(nodeB.hasPrimaryFocus, isFalse);
    expect(notifyCount, equals(1));
    notifyCount = 0;
  });

  testWidgets('debugFocusChanges causes logging of focus changes', (WidgetTester tester) async {
    final bool oldDebugFocusChanges = debugFocusChanges;
    final DebugPrintCallback oldDebugPrint = debugPrint;
    final StringBuffer messages = StringBuffer();
    debugPrint = (String? message, {int? wrapWidth}) {
      messages.writeln(message ?? '');
    };
    debugFocusChanges = true;
    try {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode parent1 = FocusScopeNode(debugLabel: 'parent1');
      addTearDown(parent1.dispose);
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode child1 = FocusNode(debugLabel: 'child1');
      addTearDown(child1.dispose);
      final FocusAttachment child1Attachment = child1.attach(context);
      parent1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      child1Attachment.reparent(parent: parent1);

      int notifyCount = 0;
      void handleFocusChange() {
        notifyCount++;
      }

      tester.binding.focusManager.addListener(handleFocusChange);

      parent1.requestFocus();
      expect(notifyCount, equals(0));
      await tester.pump();
      expect(notifyCount, equals(1));
      notifyCount = 0;

      child1.requestFocus();
      await tester.pump();
      expect(notifyCount, equals(1));
      notifyCount = 0;

      tester.binding.focusManager.removeListener(handleFocusChange);
    } finally {
      debugFocusChanges = oldDebugFocusChanges;
      debugPrint = oldDebugPrint;
    }
    final String messagesStr = messages.toString();
    expect(
      messagesStr,
      contains(RegExp(r'   └─Child \d+: FocusScopeNode#[a-f0-9]{5}\(parent1 \[PRIMARY FOCUS\]\)')),
    );
    expect(messagesStr, contains('FOCUS: Notified 2 dirty nodes'));
    expect(
      messagesStr,
      contains(
        RegExp(
          r'FOCUS: Scheduling update, current focus is null, next focus will be FocusScopeNode#.*parent1',
        ),
      ),
    );
  });

  testWidgets("doesn't call toString on a focus node when debugFocusChanges is false", (
    WidgetTester tester,
  ) async {
    final bool oldDebugFocusChanges = debugFocusChanges;
    final DebugPrintCallback oldDebugPrint = debugPrint;
    final StringBuffer messages = StringBuffer();
    debugPrint = (String? message, {int? wrapWidth}) {
      messages.writeln(message ?? '');
    };
    Future<void> testDebugFocusChanges() async {
      final BuildContext context = await setupWidget(tester);
      final FocusScopeNode parent1 = FocusScopeNode(debugLabel: 'parent1');
      final FocusAttachment parent1Attachment = parent1.attach(context);
      final FocusNode child1 =
          debugFocusChanges
              ? FocusNode(debugLabel: 'child1')
              : _LoggingTestFocusNode(debugLabel: 'child1');
      final FocusAttachment child1Attachment = child1.attach(context);
      parent1Attachment.reparent(parent: tester.binding.focusManager.rootScope);
      child1Attachment.reparent(parent: parent1);

      child1.requestFocus();
      await tester.pump();
      child1.dispose();
      parent1.dispose();
      await tester.pump();
    }

    try {
      debugFocusChanges = false;
      await testDebugFocusChanges();
      expect(messages, isEmpty);
      expect(tester.takeException(), isNull);
      debugFocusChanges = true;
      await testDebugFocusChanges();
      expect(messages.toString(), contains('FOCUS: Notified 3 dirty nodes:'));
      expect(tester.takeException(), isNull);
    } finally {
      debugFocusChanges = oldDebugFocusChanges;
      debugPrint = oldDebugPrint;
    }
  });
}

class _LoggingTestFocusNode extends FocusNode {
  _LoggingTestFocusNode({super.debugLabel});

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.debug}) {
    throw StateError("Shouldn't call toString here");
  }

  @override
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines,
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 65,
  }) {
    throw StateError("Shouldn't call toStringDeep here");
  }
}
