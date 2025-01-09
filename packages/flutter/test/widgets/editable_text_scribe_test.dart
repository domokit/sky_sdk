// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show PointerDeviceKind;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

  const TextStyle textStyle = TextStyle();
  const Color cursorColor = Color.fromARGB(0xFF, 0xFF, 0x00, 0x00);
  final List<MethodCall> calls = <MethodCall>[];
  bool isFeatureAvailableReturnValue = true;
  late TextEditingController controller;
  late FocusNode focusNode;

  setUp(() async {
    calls.clear();
    isFeatureAvailableReturnValue = true;
    binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.scribe, (
      MethodCall methodCall,
    ) {
      calls.add(methodCall);

      return switch (methodCall.method) {
        'Scribe.isFeatureAvailable' => Future<bool>.value(isFeatureAvailableReturnValue),
        'Scribe.startStylusHandwriting' => Future<void>.value(),
        _ => throw FlutterError('Unexpected method call: ${methodCall.method}'),
      };
    });

    controller = TextEditingController(text: 'Lorem ipsum dolor sit amet');
    focusNode = FocusNode(debugLabel: 'EditableText Node');
  });

  tearDown(() {
    controller.dispose();
    focusNode.dispose();
  });

  Future<void> pumpTextSelectionGestureDetectorBuilder(
    WidgetTester tester, {
    bool forcePressEnabled = true,
    bool selectionEnabled = true,
  }) async {
    final GlobalKey<EditableTextState> editableTextKey = GlobalKey<EditableTextState>();
    final FakeTextSelectionGestureDetectorBuilderDelegate delegate =
        FakeTextSelectionGestureDetectorBuilderDelegate(
          editableTextKey: editableTextKey,
          forcePressEnabled: forcePressEnabled,
          selectionEnabled: selectionEnabled,
        );

    final TextSelectionGestureDetectorBuilder provider = TextSelectionGestureDetectorBuilder(
      delegate: delegate,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: provider.buildGestureDetector(
          behavior: HitTestBehavior.translucent,
          child: EditableText(
            key: editableTextKey,
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            showSelectionHandles: true,
          ),
        ),
      ),
    );
  }

  testWidgets(
    'when Scribe is available, starts handwriting on tap down',
    (WidgetTester tester) async {
      isFeatureAvailableReturnValue = true;

      await pumpTextSelectionGestureDetectorBuilder(tester);

      expect(focusNode.hasFocus, isFalse);

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.stylus,
        pointer: 1,
      );
      await gesture.down(tester.getCenter(find.byType(EditableText)));

      // Wait for the gesture arena.
      await tester.pumpAndSettle();

      expect(calls, hasLength(2));
      expect(calls.first.method, 'Scribe.isFeatureAvailable');
      expect(calls[1].method, 'Scribe.startStylusHandwriting');

      await gesture.up();
      expect(focusNode.hasFocus, isTrue);

      // On web, let the browser handle handwriting input.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'when Scribe is unavailable, does not start handwriting on tap down',
    (WidgetTester tester) async {
      isFeatureAvailableReturnValue = false;

      await pumpTextSelectionGestureDetectorBuilder(tester);

      expect(focusNode.hasFocus, isFalse);

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.stylus,
        pointer: 1,
      );
      await gesture.down(tester.getCenter(find.byType(EditableText)));

      // Wait for the gesture arena.
      await tester.pumpAndSettle();

      expect(calls, hasLength(1));
      expect(calls.first.method, 'Scribe.isFeatureAvailable');

      await gesture.up();
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'tap down event must be from a stylus in order to start handwriting',
    (WidgetTester tester) async {
      isFeatureAvailableReturnValue = true;

      await pumpTextSelectionGestureDetectorBuilder(tester);

      expect(focusNode.hasFocus, isFalse);

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.mouse,
        pointer: 1,
      );
      await gesture.down(tester.getCenter(find.byType(EditableText)));

      expect(calls, isEmpty);

      await gesture.up();
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'tap down event on a collapsed selection handle is handled by the handle and does not start handwriting',
    (WidgetTester tester) async {
      isFeatureAvailableReturnValue = true;

      await pumpTextSelectionGestureDetectorBuilder(tester);

      expect(focusNode.hasFocus, isFalse);
      expect(find.byType(CompositedTransformFollower), findsNothing);

      // Tap to show the collapsed selection handle.
      final Offset fieldOffset = tester.getTopLeft(find.byType(EditableText));
      await tester.tapAt(fieldOffset + const Offset(20.0, 10.0));
      await tester.pump();
      expect(find.byType(CompositedTransformFollower), findsOneWidget);

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.stylus,
        pointer: 1,
      );
      final Finder handleFinder = find.descendant(
        of: find.byType(CompositedTransformFollower),
        matching: find.byType(CustomPaint),
      );
      await gesture.down(tester.getCenter(handleFinder));

      // Wait for the gesture arena.
      await tester.pumpAndSettle();

      expect(calls, hasLength(0));
      expect(controller.selection.isCollapsed, isTrue);
      final int cursorStart = controller.selection.start;

      // Dragging on top of the handle moves it like normal.
      await gesture.moveBy(const Offset(20.0, 0.0));
      expect(controller.selection.isCollapsed, isTrue);
      expect(controller.selection.start, greaterThan(cursorStart));
      expect(calls, hasLength(0));

      await gesture.up();
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'tap down event on the end selection handle is handled by the handle and does not start handwriting',
    (WidgetTester tester) async {
      isFeatureAvailableReturnValue = true;

      await pumpTextSelectionGestureDetectorBuilder(tester);

      expect(focusNode.hasFocus, isFalse);
      expect(find.byType(CompositedTransformFollower), findsNothing);

      // Long press to select the first word and show both handles.
      final Offset fieldOffset = tester.getTopLeft(find.byType(EditableText));
      await tester.longPressAt(fieldOffset + const Offset(20.0, 10.0));
      await tester.pump();
      expect(find.byType(CompositedTransformFollower), findsNWidgets(2));

      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.stylus,
        pointer: 1,
      );
      final Finder endHandleFinder = find.descendant(
        of: find.byType(CompositedTransformFollower).at(1),
        matching: find.byType(CustomPaint),
      );
      await gesture.down(tester.getCenter(endHandleFinder));

      // Wait for the gesture arena.
      await tester.pumpAndSettle();

      expect(calls, isEmpty);
      expect(controller.selection.isCollapsed, isFalse);
      final TextSelection selectionStart = controller.selection;

      // Dragging on top of the handle extends selection like normal.
      await gesture.moveBy(const Offset(20.0, 0.0));
      expect(controller.selection.isCollapsed, isFalse);
      expect(controller.selection.start, equals(selectionStart.start));
      expect(controller.selection.end, greaterThan(selectionStart.end));
      expect(calls, isEmpty);

      await gesture.up();
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );
}

class FakeTextSelectionGestureDetectorBuilderDelegate
    implements TextSelectionGestureDetectorBuilderDelegate {
  FakeTextSelectionGestureDetectorBuilderDelegate({
    required this.editableTextKey,
    required this.forcePressEnabled,
    required this.selectionEnabled,
  });

  @override
  final GlobalKey<EditableTextState> editableTextKey;

  @override
  final bool forcePressEnabled;

  @override
  final bool selectionEnabled;
}
