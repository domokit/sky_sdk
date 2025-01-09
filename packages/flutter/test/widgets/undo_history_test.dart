// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'editable_text_utils.dart';

final FocusNode _focusNode = FocusNode(debugLabel: 'UndoHistory Node');

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  group('UndoHistory', () {
    Future<void> sendUndoRedo(WidgetTester tester, [bool redo = false]) {
      return sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.keyZ],
        shortcutModifier: true,
        shift: redo,
        targetPlatform: defaultTargetPlatform,
      );
    }

    Future<void> sendUndo(WidgetTester tester) => sendUndoRedo(tester);
    Future<void> sendRedo(WidgetTester tester) => sendUndoRedo(tester, true);

    testWidgets('UndoHistory widget registers as global undo/redo client', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'UndoHistory Node');
      addTearDown(focusNode.dispose);
      final GlobalKey undoHistoryGlobalKey = GlobalKey();
      final ValueNotifier<int> value = ValueNotifier<int>(0);
      addTearDown(value.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: UndoHistory<int>(
            key: undoHistoryGlobalKey,
            value: value,
            onTriggered: (_) {},
            focusNode: focusNode,
            child: Focus(focusNode: focusNode, child: Container()),
          ),
        ),
      );

      // Initially the UndoHistory doesn't have focus, therefore it should
      // not be the global undo/redo client. Ensure that's the case.
      expect(UndoManager.client, isNull);

      // Give focus to the UndoHistory widget.
      focusNode.requestFocus();
      await tester.pump();

      // Now that the UndoHistory widget has focus, it should have registered
      // itself as the global undo/redo client.
      final State? undoHistoryState = undoHistoryGlobalKey.currentState;
      expect(UndoManager.client, undoHistoryState);
    });

    testWidgets('UndoHistory widget deregisters as global undo/redo client when it loses focus', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'UndoHistory Node');
      addTearDown(focusNode.dispose);
      final GlobalKey undoHistoryGlobalKey = GlobalKey();
      final ValueNotifier<int> value = ValueNotifier<int>(0);
      addTearDown(value.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: UndoHistory<int>(
            key: undoHistoryGlobalKey,
            value: value,
            onTriggered: (_) {},
            focusNode: focusNode,
            child: Focus(focusNode: focusNode, child: Container()),
          ),
        ),
      );

      // Give focus to the UndoHistory widget.
      focusNode.requestFocus();
      await tester.pump();

      // Ensure that UndoHistory is the global undo/redo client.
      final State? undoHistoryState = undoHistoryGlobalKey.currentState;
      expect(UndoManager.client, undoHistoryState);

      // Remove focus from UndoHistory widget.
      focusNode.unfocus();
      await tester.pump();

      // Ensure the UndoHistory widget is no longer the global client
      expect(UndoManager.client, null);
    });

    testWidgets('UndoHistory widget deregisters as global undo/redo client when disposed', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'UndoHistory Node');
      addTearDown(focusNode.dispose);
      final GlobalKey undoHistoryGlobalKey = GlobalKey();
      final ValueNotifier<int> value = ValueNotifier<int>(0);
      addTearDown(value.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: UndoHistory<int>(
            key: undoHistoryGlobalKey,
            value: value,
            onTriggered: (_) {},
            focusNode: focusNode,
            child: Focus(focusNode: focusNode, child: Container()),
          ),
        ),
      );

      // Give focus to the UndoHistory widget.
      focusNode.requestFocus();
      await tester.pump();

      // Ensure that UndoHistory is the global undo/redo client.
      final State? undoHistoryState = undoHistoryGlobalKey.currentState;
      expect(UndoManager.client, undoHistoryState);

      // Cause the UndoHistory widget to dispose its state.
      await tester.pumpWidget(const MaterialApp(home: SizedBox()));

      // Ensure that the disposed UndoHistory state is not still the global
      // undo/redo history client.
      expect(UndoManager.client, isNull);
    });

    testWidgets(
      'allows undo and redo to be called programmatically from the UndoHistoryController',
      (WidgetTester tester) async {
        final ValueNotifier<int> value = ValueNotifier<int>(0);
        addTearDown(value.dispose);
        final UndoHistoryController controller = UndoHistoryController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: UndoHistory<int>(
              value: value,
              controller: controller,
              onTriggered: (int newValue) {
                value.value = newValue;
              },
              focusNode: _focusNode,
              child: Container(),
            ),
          ),
        );

        await tester.pump(const Duration(milliseconds: 500));

        // Undo/redo have no effect if the value has never changed.
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, false);
        controller.undo();
        expect(value.value, 0);
        controller.redo();
        expect(value.value, 0);

        _focusNode.requestFocus();
        await tester.pump();
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, false);
        controller.undo();
        expect(value.value, 0);
        controller.redo();
        expect(value.value, 0);

        value.value = 1;

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Can undo/redo a single change.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        controller.undo();
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        controller.redo();
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        value.value = 2;
        await tester.pump(const Duration(milliseconds: 500));

        // And can undo/redo multiple changes.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        controller.undo();
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        controller.undo();
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        controller.redo();
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        controller.redo();
        expect(value.value, 2);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        // Changing the value again clears the redo stack.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        controller.undo();
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        value.value = 3;
        await tester.pump(const Duration(milliseconds: 500));
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'allows undo and redo to be called using the keyboard',
      (WidgetTester tester) async {
        final ValueNotifier<int> value = ValueNotifier<int>(0);
        addTearDown(value.dispose);
        final UndoHistoryController controller = UndoHistoryController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: UndoHistory<int>(
              controller: controller,
              value: value,
              onTriggered: (int newValue) {
                value.value = newValue;
              },
              focusNode: _focusNode,
              child: Focus(focusNode: _focusNode, child: Container()),
            ),
          ),
        );

        await tester.pump(const Duration(milliseconds: 500));

        // Undo/redo have no effect if the value has never changed.
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, false);
        await sendUndo(tester);
        expect(value.value, 0);
        await sendRedo(tester);
        expect(value.value, 0);

        _focusNode.requestFocus();
        await tester.pump();
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, false);
        await sendUndo(tester);
        expect(value.value, 0);
        await sendRedo(tester);
        expect(value.value, 0);

        value.value = 1;

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Can undo/redo a single change.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        await sendUndo(tester);
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        await sendRedo(tester);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        value.value = 2;
        await tester.pump(const Duration(milliseconds: 500));

        // And can undo/redo multiple changes.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        await sendUndo(tester);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        await sendUndo(tester);
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        await sendRedo(tester);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        await sendRedo(tester);
        expect(value.value, 2);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        // Changing the value again clears the redo stack.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        await sendUndo(tester);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        value.value = 3;
        await tester.pump(const Duration(milliseconds: 500));
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets('duplicate changes do not affect the undo history', (WidgetTester tester) async {
      final ValueNotifier<int> value = ValueNotifier<int>(0);
      addTearDown(value.dispose);
      final UndoHistoryController controller = UndoHistoryController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: UndoHistory<int>(
            controller: controller,
            value: value,
            onTriggered: (int newValue) {
              value.value = newValue;
            },
            focusNode: _focusNode,
            child: Container(),
          ),
        ),
      );

      _focusNode.requestFocus();

      // Wait for the throttling.
      await tester.pump(const Duration(milliseconds: 500));

      value.value = 1;

      // Wait for the throttling.
      await tester.pump(const Duration(milliseconds: 500));

      // Can undo/redo a single change.
      expect(controller.value.canUndo, true);
      expect(controller.value.canRedo, false);
      controller.undo();
      expect(value.value, 0);
      expect(controller.value.canUndo, false);
      expect(controller.value.canRedo, true);
      controller.redo();
      expect(value.value, 1);
      expect(controller.value.canUndo, true);
      expect(controller.value.canRedo, false);

      // Changes that result in the same state won't be saved on the undo stack.
      value.value = 1;
      await tester.pump(const Duration(milliseconds: 500));
      expect(controller.value.canUndo, true);
      expect(controller.value.canRedo, false);
      controller.undo();
      expect(value.value, 0);
      expect(controller.value.canUndo, false);
      expect(controller.value.canRedo, true);
    }, variant: TargetPlatformVariant.all());

    testWidgets('ignores value changes pushed during onTriggered', (WidgetTester tester) async {
      final ValueNotifier<int> value = ValueNotifier<int>(0);
      addTearDown(value.dispose);
      final UndoHistoryController controller = UndoHistoryController();
      addTearDown(controller.dispose);
      int Function(int newValue) valueToUse = (int value) => value;
      final GlobalKey<UndoHistoryState<int>> key = GlobalKey<UndoHistoryState<int>>();

      await tester.pumpWidget(
        MaterialApp(
          home: UndoHistory<int>(
            key: key,
            value: value,
            controller: controller,
            onTriggered: (int newValue) {
              value.value = valueToUse(newValue);
            },
            focusNode: _focusNode,
            child: Container(),
          ),
        ),
      );

      await tester.pump(const Duration(milliseconds: 500));

      // Undo/redo have no effect if the value has never changed.
      expect(controller.value.canUndo, false);
      expect(controller.value.canRedo, false);
      controller.undo();
      expect(value.value, 0);
      controller.redo();
      expect(value.value, 0);

      _focusNode.requestFocus();
      await tester.pump();
      expect(controller.value.canUndo, false);
      expect(controller.value.canRedo, false);
      controller.undo();
      expect(value.value, 0);
      controller.redo();
      expect(value.value, 0);

      value.value = 1;

      // Wait for the throttling.
      await tester.pump(const Duration(milliseconds: 500));

      valueToUse = (int value) => 3;
      expect(() => key.currentState!.undo(), throwsAssertionError);
    }, variant: TargetPlatformVariant.all());

    testWidgets(
      'changes should send setUndoState to the UndoManagerConnection on iOS',
      (WidgetTester tester) async {
        final List<MethodCall> log = <MethodCall>[];
        tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.undoManager, (
          MethodCall methodCall,
        ) async {
          log.add(methodCall);
          return null;
        });
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);

        final ValueNotifier<int> value = ValueNotifier<int>(0);
        addTearDown(value.dispose);
        final UndoHistoryController controller = UndoHistoryController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: UndoHistory<int>(
              controller: controller,
              value: value,
              onTriggered: (int newValue) {
                value.value = newValue;
              },
              focusNode: focusNode,
              child: Focus(focusNode: focusNode, child: Container()),
            ),
          ),
        );

        await tester.pump();

        focusNode.requestFocus();
        await tester.pump();

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Undo and redo should both be disabled.
        MethodCall methodCall = log.lastWhere(
          (MethodCall m) => m.method == 'UndoManager.setUndoState',
        );
        expect(methodCall.method, 'UndoManager.setUndoState');
        expect(methodCall.arguments as Map<String, dynamic>, <String, bool>{
          'canUndo': false,
          'canRedo': false,
        });

        // Making a change should enable undo.
        value.value = 1;
        await tester.pump(const Duration(milliseconds: 500));

        methodCall = log.lastWhere((MethodCall m) => m.method == 'UndoManager.setUndoState');
        expect(methodCall.method, 'UndoManager.setUndoState');
        expect(methodCall.arguments as Map<String, dynamic>, <String, bool>{
          'canUndo': true,
          'canRedo': false,
        });

        // Undo should remain enabled after another change.
        value.value = 2;
        await tester.pump(const Duration(milliseconds: 500));

        methodCall = log.lastWhere((MethodCall m) => m.method == 'UndoManager.setUndoState');
        expect(methodCall.method, 'UndoManager.setUndoState');
        expect(methodCall.arguments as Map<String, dynamic>, <String, bool>{
          'canUndo': true,
          'canRedo': false,
        });

        // Undo and redo should be enabled after one undo.
        controller.undo();
        methodCall = log.lastWhere((MethodCall m) => m.method == 'UndoManager.setUndoState');
        expect(methodCall.method, 'UndoManager.setUndoState');
        expect(methodCall.arguments as Map<String, dynamic>, <String, bool>{
          'canUndo': true,
          'canRedo': true,
        });

        // Only redo should be enabled after a second undo.
        controller.undo();
        methodCall = log.lastWhere((MethodCall m) => m.method == 'UndoManager.setUndoState');
        expect(methodCall.method, 'UndoManager.setUndoState');
        expect(methodCall.arguments as Map<String, dynamic>, <String, bool>{
          'canUndo': false,
          'canRedo': true,
        });
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'handlePlatformUndo should undo or redo appropriately on iOS',
      (WidgetTester tester) async {
        final ValueNotifier<int> value = ValueNotifier<int>(0);
        addTearDown(value.dispose);
        final UndoHistoryController controller = UndoHistoryController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: UndoHistory<int>(
              controller: controller,
              value: value,
              onTriggered: (int newValue) {
                value.value = newValue;
              },
              focusNode: _focusNode,
              child: Focus(focusNode: _focusNode, child: Container()),
            ),
          ),
        );

        await tester.pump(const Duration(milliseconds: 500));
        _focusNode.requestFocus();
        await tester.pump();

        // Undo/redo have no effect if the value has never changed.
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, false);
        UndoManager.client!.handlePlatformUndo(UndoDirection.undo);
        expect(value.value, 0);
        UndoManager.client!.handlePlatformUndo(UndoDirection.redo);
        expect(value.value, 0);

        value.value = 1;

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Can undo/redo a single change.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        UndoManager.client!.handlePlatformUndo(UndoDirection.undo);
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        UndoManager.client!.handlePlatformUndo(UndoDirection.redo);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        value.value = 2;
        await tester.pump(const Duration(milliseconds: 500));

        // And can undo/redo multiple changes.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        UndoManager.client!.handlePlatformUndo(UndoDirection.undo);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        UndoManager.client!.handlePlatformUndo(UndoDirection.undo);
        expect(value.value, 0);
        expect(controller.value.canUndo, false);
        expect(controller.value.canRedo, true);
        UndoManager.client!.handlePlatformUndo(UndoDirection.redo);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        UndoManager.client!.handlePlatformUndo(UndoDirection.redo);
        expect(value.value, 2);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);

        // Changing the value again clears the redo stack.
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
        UndoManager.client!.handlePlatformUndo(UndoDirection.undo);
        expect(value.value, 1);
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, true);
        value.value = 3;
        await tester.pump(const Duration(milliseconds: 500));
        expect(controller.value.canUndo, true);
        expect(controller.value.canRedo, false);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
      skip: kIsWeb, // [intended]
    );
  });

  group('UndoHistoryController', () {
    testWidgets('UndoHistoryController notifies onUndo listeners onUndo', (
      WidgetTester tester,
    ) async {
      int calls = 0;
      final UndoHistoryController controller = UndoHistoryController();
      addTearDown(controller.dispose);
      controller.onUndo.addListener(() {
        calls++;
      });

      // Does not notify the listener if canUndo is false.
      controller.undo();
      expect(calls, 0);

      // Does notify the listener if canUndo is true.
      controller.value = const UndoHistoryValue(canUndo: true);
      controller.undo();
      expect(calls, 1);
    });

    testWidgets('UndoHistoryController notifies onRedo listeners onRedo', (
      WidgetTester tester,
    ) async {
      int calls = 0;
      final UndoHistoryController controller = UndoHistoryController();
      addTearDown(controller.dispose);
      controller.onRedo.addListener(() {
        calls++;
      });

      // Does not notify the listener if canUndo is false.
      controller.redo();
      expect(calls, 0);

      // Does notify the listener if canRedo is true.
      controller.value = const UndoHistoryValue(canRedo: true);
      controller.redo();
      expect(calls, 1);
    });

    testWidgets('UndoHistoryController notifies listeners on value change', (
      WidgetTester tester,
    ) async {
      int calls = 0;
      final UndoHistoryController controller = UndoHistoryController(
        value: const UndoHistoryValue(canUndo: true),
      );
      addTearDown(controller.dispose);
      controller.addListener(() {
        calls++;
      });

      // Does not notify if the value is the same.
      controller.value = const UndoHistoryValue(canUndo: true);
      expect(calls, 0);

      // Does notify if the value has changed.
      controller.value = const UndoHistoryValue(canRedo: true);
      expect(calls, 1);
    });
  });
}
