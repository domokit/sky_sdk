// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// reduced-test-set:
//   This file is run as part of a reduced test set in CI on Mac and Windows
//   machines.
// no-shuffle:
// TODO(122950): Remove this tag once this test's state leaks/test
// dependencies have been fixed.
// https://github.com/flutter/flutter/issues/122950
// Fails with "flutter test --test-randomize-ordering-seed=20230318"
@Tags(<String>['reduced-test-set', 'no-shuffle'])
library;

import 'dart:math' as math;
import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';
import '../widgets/editable_text_utils.dart';
import '../widgets/feedback_tester.dart';
import '../widgets/live_text_utils.dart';
import '../widgets/process_text_utils.dart';
import '../widgets/semantics_tester.dart';
import '../widgets/text_selection_toolbar_utils.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  const String kThreeLines =
      'First line of text is\n'
      'Second line goes until\n'
      'Third line of stuff';
  const String kMoreThanFourLines =
      '$kThreeLines\n'
      "Fourth line won't display and ends at";
  // Gap between caret and edge of input, defined in editable.dart.
  const int kCaretGap = 1;

  setUp(() async {
    debugResetSemanticsIdCounter();
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  final Key textFieldKey = UniqueKey();
  Widget textFieldBuilder({int? maxLines = 1, int? minLines}) {
    return boilerplate(
      child: TextField(
        key: textFieldKey,
        style: const TextStyle(color: Colors.black, fontSize: 34.0),
        maxLines: maxLines,
        minLines: minLines,
        decoration: const InputDecoration(hintText: 'Placeholder'),
      ),
    );
  }

  testWidgets('Live Text button shows and hides correctly when LiveTextStatus changes', (
    WidgetTester tester,
  ) async {
    final LiveTextInputTester liveTextInputTester = LiveTextInputTester();
    addTearDown(liveTextInputTester.dispose);
    final TextEditingController controller = _textEditingController();
    const Key key = ValueKey<String>('TextField');
    final FocusNode focusNode = _focusNode();
    final Widget app = MaterialApp(
      theme: ThemeData(platform: TargetPlatform.iOS),
      home: Scaffold(
        body: Center(child: TextField(key: key, controller: controller, focusNode: focusNode)),
      ),
    );

    liveTextInputTester.mockLiveTextInputEnabled = true;
    await tester.pumpWidget(app);
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    final Finder textFinder = find.byType(EditableText);
    await tester.longPress(textFinder);
    await tester.pumpAndSettle();
    expect(findLiveTextButton(), kIsWeb ? findsNothing : findsOneWidget);

    liveTextInputTester.mockLiveTextInputEnabled = false;
    await tester.longPress(textFinder);
    await tester.pumpAndSettle();
    expect(findLiveTextButton(), findsNothing);
  });

  testWidgets(
    'text field selection toolbar should hide when the user starts typing',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: TextField(decoration: InputDecoration(hintText: 'Placeholder')),
              ),
            ),
          ),
        ),
      );

      await tester.showKeyboard(find.byType(TextField));

      const String testValue = 'A B C';
      tester.testTextInput.updateEditingValue(const TextEditingValue(text: testValue));
      await tester.pump();

      // The selectWordsInRange with SelectionChangedCause.tap seems to be needed to show the toolbar.
      // (This is true even if we provide selection parameter to the TextEditingValue above.)
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);

      expect(state.showToolbar(), true);

      // This is needed for the AnimatedOpacity to turn from 0 to 1 so the toolbar is visible.
      await tester.pumpAndSettle();

      // Sanity check that the toolbar widget exists.
      expect(find.text('Paste'), findsOneWidget);

      const String newValue = 'A B C D';
      tester.testTextInput.updateEditingValue(const TextEditingValue(text: newValue));
      await tester.pump();

      expect(state.selectionOverlay!.toolbarIsVisible, isFalse);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('Composing change does not hide selection handle caret', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/108673
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'I Love Flutter!';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    // Handle not shown.
    expect(controller.selection.isCollapsed, true);
    final Finder fadeFinder = find.byType(FadeTransition);
    FadeTransition handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(0.0));

    // Tap on the text field to show the handle.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    expect(fadeFinder, findsNWidgets(1));
    handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));
    final RenderObject handleRenderObjectBegin = tester.renderObject(fadeFinder.at(0));

    expect(
      controller.value,
      const TextEditingValue(
        text: 'I Love Flutter!',
        selection: TextSelection.collapsed(offset: 15, affinity: TextAffinity.upstream),
      ),
    );

    // Simulate text composing change.
    tester.testTextInput.updateEditingValue(
      controller.value.copyWith(composing: const TextRange(start: 7, end: 15)),
    );
    await skipPastScrollingAnimation(tester);

    expect(
      controller.value,
      const TextEditingValue(
        text: 'I Love Flutter!',
        selection: TextSelection.collapsed(offset: 15, affinity: TextAffinity.upstream),
        composing: TextRange(start: 7, end: 15),
      ),
    );

    // Handle still shown.
    expect(controller.selection.isCollapsed, true);
    handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));

    // Simulate text composing and affinity change.
    tester.testTextInput.updateEditingValue(
      controller.value.copyWith(
        selection: controller.value.selection.copyWith(affinity: TextAffinity.downstream),
        composing: const TextRange(start: 8, end: 15),
      ),
    );
    await skipPastScrollingAnimation(tester);

    expect(
      controller.value,
      const TextEditingValue(
        text: 'I Love Flutter!',
        selection: TextSelection.collapsed(offset: 15, affinity: TextAffinity.upstream),
        composing: TextRange(start: 8, end: 15),
      ),
    );

    // Handle still shown.
    expect(controller.selection.isCollapsed, true);
    handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));

    final RenderObject handleRenderObjectEnd = tester.renderObject(fadeFinder.at(0));
    // The RenderObject sub-tree should not be unmounted.
    expect(identical(handleRenderObjectBegin, handleRenderObjectEnd), true);
  });

  testWidgets(
    'can use the desktop cut/copy/paste buttons on Mac',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Initially, the menu is not shown and there is no selection.
      expectNoCupertinoToolbar();
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Right clicking shows the menu.
      final TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);

      // Copy the first word.
      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expectNoCupertinoToolbar();

      // Paste it at the end.
      await gesture.down(textOffsetToPosition(tester, controller.text.length));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Paste'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2blah1');
      expect(controller.selection, const TextSelection.collapsed(offset: 16));

      // Cut the first word.
      await gesture.down(midBlah1);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      await tester.tap(find.text('Cut'));
      await tester.pumpAndSettle();
      expect(controller.text, ' blah2blah1');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 0));
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'can use the desktop cut/copy/paste buttons on Windows and Linux',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Initially, the menu is not shown and there is no selection.
      expectNoCupertinoToolbar();
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Right clicking shows the menu.
      TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 2));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);

      // Double tap to select the first word, then right click to show the menu.
      final Offset startBlah1 = textOffsetToPosition(tester, 0);
      gesture = await tester.startGesture(startBlah1, kind: PointerDeviceKind.mouse);
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.down(startBlah1);
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Select all'), findsNothing);
      gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);

      // Copy the first word.
      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expectNoCupertinoToolbar();

      // Paste it at the end.
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Paste'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2blah1');
      expect(controller.selection, const TextSelection.collapsed(offset: 16));

      // Cut the first word.
      gesture = await tester.startGesture(midBlah1, kind: PointerDeviceKind.mouse);
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.down(startBlah1);
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Select all'), findsNothing);
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Cut'));
      await tester.pumpAndSettle();
      expect(controller.text, ' blah2blah1');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 0));
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Look Up shows up on iOS only',
    (WidgetTester tester) async {
      String? lastLookUp;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'LookUp.invoke') {
            expect(methodCall.arguments, isA<String>());
            lastLookUp = methodCall.arguments as String;
          }
          return null;
        },
      );

      final TextEditingController controller = _textEditingController(text: 'Test ');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final bool isTargetPlatformiOS = defaultTargetPlatform == TargetPlatform.iOS;

      // Long press to put the cursor after the "s".
      const int index = 3;
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 4));
      expect(find.text('Look Up'), isTargetPlatformiOS ? findsOneWidget : findsNothing);

      if (isTargetPlatformiOS) {
        await tester.tap(find.text('Look Up'));
        expect(lastLookUp, 'Test');
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Search Web shows up on iOS only',
    (WidgetTester tester) async {
      String? lastSearch;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'SearchWeb.invoke') {
            expect(methodCall.arguments, isA<String>());
            lastSearch = methodCall.arguments as String;
          }
          return null;
        },
      );

      final TextEditingController controller = _textEditingController(text: 'Test ');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final bool isTargetPlatformiOS = defaultTargetPlatform == TargetPlatform.iOS;

      // Long press to put the cursor after the "s".
      const int index = 3;
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 4));
      expect(find.text('Search Web'), isTargetPlatformiOS ? findsOneWidget : findsNothing);

      if (isTargetPlatformiOS) {
        await tester.tap(find.text('Search Web'));
        expect(lastSearch, 'Test');
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Share shows up on iOS and Android',
    (WidgetTester tester) async {
      String? lastShare;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'Share.invoke') {
            expect(methodCall.arguments, isA<String>());
            lastShare = methodCall.arguments as String;
          }
          return null;
        },
      );

      final TextEditingController controller = _textEditingController(text: 'Test ');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final bool isTargetPlatformiOS = defaultTargetPlatform == TargetPlatform.iOS;

      // Long press to put the cursor after the "s".
      const int index = 3;
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 4));

      if (isTargetPlatformiOS) {
        expect(find.text('Share...'), findsOneWidget);
        await tester.tap(find.text('Share...'));
      } else {
        expect(find.text('Share'), findsOneWidget);
        await tester.tap(find.text('Share'));
      }
      expect(lastShare, 'Test');
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('uses DefaultSelectionStyle for selection and cursor colors if provided', (
    WidgetTester tester,
  ) async {
    const Color selectionColor = Colors.orange;
    const Color cursorColor = Colors.red;

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: DefaultSelectionStyle(
            selectionColor: selectionColor,
            cursorColor: cursorColor,
            child: TextField(autofocus: true),
          ),
        ),
      ),
    );
    await tester.pump();
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.widget.selectionColor, selectionColor);
    expect(state.widget.cursorColor, cursorColor);
  });

  testWidgets(
    'Use error cursor color when an InputDecoration with an errorText or error widget is provided',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: TextField(
              autofocus: true,
              decoration: InputDecoration(
                error: Text('error'),
                errorStyle: TextStyle(color: Colors.teal),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.widget.cursorColor, Colors.teal);

      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: TextField(
              autofocus: true,
              decoration: InputDecoration(
                errorText: 'error',
                errorStyle: TextStyle(color: Colors.teal),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.widget.cursorColor, Colors.teal);
    },
  );

  testWidgets('sets cursorOpacityAnimates on EditableText correctly', (WidgetTester tester) async {
    // True

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: TextField(autofocus: true, cursorOpacityAnimates: true)),
      ),
    );
    await tester.pump();
    EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.cursorOpacityAnimates, true);

    // False

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: TextField(autofocus: true, cursorOpacityAnimates: false)),
      ),
    );
    await tester.pump();
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.cursorOpacityAnimates, false);
  });

  testWidgets(
    'Activates the text field when receives semantics focus on desktops',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = _focusNode();
      await tester.pumpWidget(MaterialApp(home: Material(child: TextField(focusNode: focusNode))));
      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.isTextField,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                            ],
                            actions: <SemanticsAction>[
                              SemanticsAction.tap,
                              SemanticsAction.focus,
                              SemanticsAction.didGainAccessibilityFocus,
                              SemanticsAction.didLoseAccessibilityFocus,
                            ],
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      expect(focusNode.hasFocus, isFalse);
      semanticsOwner.performAction(4, SemanticsAction.didGainAccessibilityFocus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);

      semanticsOwner.performAction(4, SemanticsAction.didLoseAccessibilityFocus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isFalse);
      semantics.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets('TextField passes onEditingComplete to EditableText', (WidgetTester tester) async {
    void onEditingComplete() {}

    await tester.pumpWidget(
      MaterialApp(home: Material(child: TextField(onEditingComplete: onEditingComplete))),
    );

    final Finder editableTextFinder = find.byType(EditableText);
    expect(editableTextFinder, findsOneWidget);

    final EditableText editableTextWidget = tester.widget(editableTextFinder);
    expect(editableTextWidget.onEditingComplete, onEditingComplete);
  });

  // Regression test for https://github.com/flutter/flutter/issues/127597.
  testWidgets(
    'The second TextField is clicked, triggers the onTapOutside callback of the previous TextField',
    (WidgetTester tester) async {
      final GlobalKey keyA = GlobalKey();
      final GlobalKey keyB = GlobalKey();
      final GlobalKey keyC = GlobalKey();
      bool outsideClickA = false;
      bool outsideClickB = false;
      bool outsideClickC = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: Column(
              children: <Widget>[
                const Text('Outside'),
                Material(
                  child: TextField(
                    key: keyA,
                    groupId: 'Group A',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickA = true;
                    },
                  ),
                ),
                Material(
                  child: TextField(
                    key: keyB,
                    groupId: 'Group B',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickB = true;
                    },
                  ),
                ),
                Material(
                  child: TextField(
                    key: keyC,
                    groupId: 'Group C',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickC = true;
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.pump();

      Future<void> click(Finder finder) async {
        await tester.tap(finder);
        await tester.enterText(finder, 'Hello');
        await tester.pump();
      }

      expect(outsideClickA, false);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyA));
      await tester.showKeyboard(find.byKey(keyA));
      await tester.idle();
      expect(outsideClickA, false);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyB));
      expect(outsideClickA, true);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyC));
      expect(outsideClickA, true);
      expect(outsideClickB, true);
      expect(outsideClickC, false);

      await tester.tap(find.text('Outside'));
      expect(outsideClickA, true);
      expect(outsideClickB, true);
      expect(outsideClickC, true);
    },
  );

  testWidgets('TextField has consistent size', (WidgetTester tester) async {
    final Key textFieldKey = UniqueKey();
    String? textFieldValue;

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: textFieldKey,
          decoration: const InputDecoration(hintText: 'Placeholder'),
          onChanged: (String value) {
            textFieldValue = value;
          },
        ),
      ),
    );

    RenderBox findTextFieldBox() => tester.renderObject(find.byKey(textFieldKey));

    final RenderBox inputBox = findTextFieldBox();
    final Size emptyInputSize = inputBox.size;

    Future<void> checkText(String testValue) async {
      return TestAsyncUtils.guard(() async {
        expect(textFieldValue, isNull);
        await tester.enterText(find.byType(TextField), testValue);
        // Check that the onChanged event handler fired.
        expect(textFieldValue, equals(testValue));
        textFieldValue = null;
        await skipPastScrollingAnimation(tester);
      });
    }

    await checkText(' ');
    expect(findTextFieldBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));

    await checkText('Test');
    expect(findTextFieldBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));
  });

  testWidgets('Cursor blinks', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(child: const TextField(decoration: InputDecoration(hintText: 'Placeholder'))),
    );
    await tester.showKeyboard(find.byType(TextField));

    final EditableTextState editableText = tester.state(find.byType(EditableText));

    // Check that the cursor visibility toggles after each blink interval.
    Future<void> checkCursorToggle() async {
      final bool initialShowCursor = editableText.cursorCurrentlyVisible;
      await tester.pump(editableText.cursorBlinkInterval);
      expect(editableText.cursorCurrentlyVisible, equals(!initialShowCursor));
      await tester.pump(editableText.cursorBlinkInterval);
      expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
      await tester.pump(editableText.cursorBlinkInterval ~/ 10);
      expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
      await tester.pump(editableText.cursorBlinkInterval);
      expect(editableText.cursorCurrentlyVisible, equals(!initialShowCursor));
      await tester.pump(editableText.cursorBlinkInterval);
      expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
    }

    await checkCursorToggle();
    await tester.showKeyboard(find.byType(TextField));

    // Try the test again with a nonempty EditableText.
    tester.testTextInput.updateEditingValue(
      const TextEditingValue(text: 'X', selection: TextSelection.collapsed(offset: 1)),
    );
    await tester.idle();
    expect(tester.state(find.byType(EditableText)), editableText);
    await checkCursorToggle();
  });

  // Regression test for https://github.com/flutter/flutter/issues/78918.
  testWidgets('RenderEditable sets correct text editing value', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'how are you');
    final UniqueKey icon = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(
            controller: controller,
            decoration: InputDecoration(
              suffixIcon: IconButton(
                key: icon,
                icon: const Icon(Icons.cancel),
                onPressed: () => controller.clear(),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(icon));
    await tester.pump();
    expect(controller.text, '');
    expect(controller.selection, const TextSelection.collapsed(offset: 0));
  });

  testWidgets(
    'Cursor radius is 2.0',
    (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Material(child: TextField())));

      final EditableTextState editableTextState = tester.firstState(find.byType(EditableText));
      final RenderEditable renderEditable = editableTextState.renderEditable;

      expect(renderEditable.cursorRadius, const Radius.circular(2.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('cursor has expected defaults', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const TextField()));

    final TextField textField = tester.firstWidget(find.byType(TextField));
    expect(textField.cursorWidth, 2.0);
    expect(textField.cursorHeight, null);
    expect(textField.cursorRadius, null);
  });

  testWidgets('cursor has expected radius value', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const TextField(cursorRadius: Radius.circular(3.0))));

    final TextField textField = tester.firstWidget(find.byType(TextField));
    expect(textField.cursorWidth, 2.0);
    expect(textField.cursorRadius, const Radius.circular(3.0));
  });

  testWidgets('clipBehavior has expected defaults', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const TextField()));

    final TextField textField = tester.firstWidget(find.byType(TextField));
    expect(textField.clipBehavior, Clip.hardEdge);
  });

  testWidgets('Overflow clipBehavior none golden', (WidgetTester tester) async {
    final OverflowWidgetTextEditingController controller = OverflowWidgetTextEditingController();
    addTearDown(controller.dispose);
    final Widget widget = Theme(
      data: ThemeData(useMaterial3: false),
      child: overlay(
        child: RepaintBoundary(
          key: const ValueKey<int>(1),
          child: SizedBox(
            height: 200,
            width: 200,
            child: Center(
              child: SizedBox(
                // Make sure the input field is not high enough for the WidgetSpan.
                height: 50,
                child: TextField(controller: controller, clipBehavior: Clip.none),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpWidget(widget);

    final TextField textField = tester.firstWidget(find.byType(TextField));
    expect(textField.clipBehavior, Clip.none);

    final EditableText editableText = tester.firstWidget(find.byType(EditableText));
    expect(editableText.clipBehavior, Clip.none);

    await expectLater(
      find.byKey(const ValueKey<int>(1)),
      matchesGoldenFile('overflow_clipbehavior_none.material.0.png'),
    );
  });

  testWidgets('Material cursor android golden', (WidgetTester tester) async {
    final Widget widget = Theme(
      data: ThemeData(useMaterial3: false),
      child: overlay(
        child: const RepaintBoundary(
          key: ValueKey<int>(1),
          child: TextField(
            cursorColor: Colors.blue,
            cursorWidth: 15,
            cursorRadius: Radius.circular(3.0),
          ),
        ),
      ),
    );
    await tester.pumpWidget(widget);

    const String testValue = 'A short phrase';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    await tester.tapAt(textOffsetToPosition(tester, testValue.length));
    await tester.pump();

    await expectLater(
      find.byKey(const ValueKey<int>(1)),
      matchesGoldenFile('text_field_cursor_test.material.0.png'),
    );
  });

  testWidgets(
    'Material cursor golden',
    (WidgetTester tester) async {
      final Widget widget = Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: const RepaintBoundary(
            key: ValueKey<int>(1),
            child: TextField(
              cursorColor: Colors.blue,
              cursorWidth: 15,
              cursorRadius: Radius.circular(3.0),
            ),
          ),
        ),
      );
      await tester.pumpWidget(widget);

      const String testValue = 'A short phrase';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      await tester.tapAt(textOffsetToPosition(tester, testValue.length));
      await tester.pump();

      await expectLater(
        find.byKey(const ValueKey<int>(1)),
        matchesGoldenFile(
          'text_field_cursor_test_${debugDefaultTargetPlatformOverride!.name.toLowerCase()}.material.1.png',
        ),
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'TextInputFormatter gets correct selection value',
    (WidgetTester tester) async {
      late TextEditingValue actualOldValue;
      late TextEditingValue actualNewValue;
      void callBack(TextEditingValue oldValue, TextEditingValue newValue) {
        actualOldValue = oldValue;
        actualNewValue = newValue;
      }

      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController(text: '123');
      await tester.pumpWidget(
        boilerplate(
          child: TextField(
            controller: controller,
            focusNode: focusNode,
            inputFormatters: <TextInputFormatter>[TestFormatter(callBack)],
          ),
        ),
      );

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pumpAndSettle();

      expect(
        actualOldValue,
        const TextEditingValue(
          text: '123',
          selection: TextSelection.collapsed(offset: 3, affinity: TextAffinity.upstream),
        ),
      );
      expect(
        actualNewValue,
        const TextEditingValue(text: '12', selection: TextSelection.collapsed(offset: 2)),
      );
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
  );

  testWidgets(
    'text field selection toolbar renders correctly inside opacity',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: const Scaffold(
            body: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: Opacity(
                  opacity: 0.5,
                  child: TextField(decoration: InputDecoration(hintText: 'Placeholder')),
                ),
              ),
            ),
          ),
        ),
      );

      await tester.showKeyboard(find.byType(TextField));

      const String testValue = 'A B C';
      tester.testTextInput.updateEditingValue(const TextEditingValue(text: testValue));
      await tester.pump();

      // The selectWordsInRange with SelectionChangedCause.tap seems to be needed to show the toolbar.
      // (This is true even if we provide selection parameter to the TextEditingValue above.)
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);

      expect(state.showToolbar(), true);

      // This is needed for the AnimatedOpacity to turn from 0 to 1 so the toolbar is visible.
      await tester.pumpAndSettle();
      await tester.pump(const Duration(seconds: 1));

      // Sanity check that the toolbar widget exists.
      expect(find.text('Paste'), findsOneWidget);

      await expectLater(
        // The toolbar exists in the Overlay above the MaterialApp.
        find.byType(Overlay),
        matchesGoldenFile('text_field_opacity_test.0.png'),
      );
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'text field toolbar options correctly changes options',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      // On macOS, we select the precise position of the tap.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(
                controller: controller,
                toolbarOptions: const ToolbarOptions(copy: true),
              ),
            ),
          ),
        ),
      );

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(textOffsetToPosition(tester, 3));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(textOffsetToPosition(tester, 8));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 8));
      await tester.tapAt(textOffsetToPosition(tester, 8));
      await tester.pump();

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      // Selected text shows 'Copy', and not 'Paste', 'Cut', 'Select All'.
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Select All'), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('text selection style 1', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure\nhi\nwasssup!',
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: RepaintBoundary(
              child: Container(
                width: 650.0,
                height: 600.0,
                decoration: const BoxDecoration(color: Color(0xff00ff00)),
                child: Column(
                  children: <Widget>[
                    TextField(
                      key: const Key('field0'),
                      controller: controller,
                      style: const TextStyle(height: 4, color: Colors.black45),
                      toolbarOptions: const ToolbarOptions(copy: true, selectAll: true),
                      selectionHeightStyle: ui.BoxHeightStyle.includeLineSpacingTop,
                      selectionWidthStyle: ui.BoxWidthStyle.max,
                      maxLines: 3,
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    final Offset textfieldStart = tester.getTopLeft(find.byKey(const Key('field0')));

    await tester.longPressAt(textfieldStart + const Offset(50.0, 2.0));
    await tester.pumpAndSettle(const Duration(milliseconds: 50));
    await tester.tapAt(textfieldStart + const Offset(100.0, 107.0));
    await tester.pump(const Duration(milliseconds: 300));

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('text_field_golden.TextSelectionStyle.1.png'),
    );
  });

  testWidgets('text selection style 2', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure\nhi\nwasssup!',
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: RepaintBoundary(
              child: Container(
                width: 650.0,
                height: 600.0,
                decoration: const BoxDecoration(color: Color(0xff00ff00)),
                child: Column(
                  children: <Widget>[
                    TextField(
                      key: const Key('field0'),
                      controller: controller,
                      style: const TextStyle(height: 4, color: Colors.black45),
                      toolbarOptions: const ToolbarOptions(copy: true, selectAll: true),
                      selectionHeightStyle: ui.BoxHeightStyle.includeLineSpacingBottom,
                      maxLines: 3,
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final EditableTextState editableTextState = tester.state(find.byType(EditableText));

    // Double tap to select the first word.
    const int index = 4;
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pumpAndSettle();
    expect(editableTextState.selectionOverlay!.handlesAreVisible, isTrue);
    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 7);

    // Select all text.  Use the toolbar if possible. iOS only shows the toolbar
    // when the selection is collapsed.
    if (isContextMenuProvidedByPlatform || defaultTargetPlatform == TargetPlatform.iOS) {
      controller.selection = TextSelection(baseOffset: 0, extentOffset: controller.text.length);
      expect(controller.selection.extentOffset, controller.text.length);
    } else {
      await tester.tap(find.text('Select all'));
      await tester.pump();
      expect(controller.selection.baseOffset, 0);
      expect(controller.selection.extentOffset, controller.text.length);
    }

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('text_field_golden.TextSelectionStyle.2.png'),
    );
    // Text selection styles are not fully supported on web.
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/93723

  testWidgets(
    'text field toolbar options correctly changes options',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(
                controller: controller,
                toolbarOptions: const ToolbarOptions(copy: true),
              ),
            ),
          ),
        ),
      );

      final Offset pos = textOffsetToPosition(tester, 9); // Index of 'P|eel'

      await tester.tapAt(pos);
      await tester.pump(const Duration(milliseconds: 50));

      await tester.tapAt(pos);
      await tester.pump();

      // Selected text shows 'Copy', and not 'Paste', 'Cut', 'Select all'.
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Select all'), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('cursor layout has correct width', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController.fromValue(
      const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
    );
    addTearDown(controller.dispose);
    final FocusNode focusNode = _focusNode();
    EditableText.debugDeterministicCursor = true;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: RepaintBoundary(
            child: TextField(cursorWidth: 15.0, controller: controller, focusNode: focusNode),
          ),
        ),
      ),
    );
    focusNode.requestFocus();
    await tester.pump();

    await expectLater(
      find.byType(TextField),
      matchesGoldenFile('text_field_cursor_width_test.0.png'),
    );
    EditableText.debugDeterministicCursor = false;
  });

  testWidgets('cursor layout has correct radius', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController.fromValue(
      const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
    );
    addTearDown(controller.dispose);
    final FocusNode focusNode = _focusNode();
    EditableText.debugDeterministicCursor = true;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: RepaintBoundary(
            child: TextField(
              cursorWidth: 15.0,
              cursorRadius: const Radius.circular(3.0),
              controller: controller,
              focusNode: focusNode,
            ),
          ),
        ),
      ),
    );
    focusNode.requestFocus();
    await tester.pump();

    await expectLater(
      find.byType(TextField),
      matchesGoldenFile('text_field_cursor_width_test.1.png'),
    );
    EditableText.debugDeterministicCursor = false;
  });

  testWidgets('cursor layout has correct height', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController.fromValue(
      const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
    );
    addTearDown(controller.dispose);
    final FocusNode focusNode = _focusNode();

    EditableText.debugDeterministicCursor = true;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: RepaintBoundary(
            child: TextField(
              cursorWidth: 15.0,
              cursorHeight: 30.0,
              controller: controller,
              focusNode: focusNode,
            ),
          ),
        ),
      ),
    );
    focusNode.requestFocus();
    await tester.pump();

    await expectLater(
      find.byType(TextField),
      matchesGoldenFile('text_field_cursor_width_test.2.png'),
    );
    EditableText.debugDeterministicCursor = false;
  });

  testWidgets('Overflowing a line with spaces stops the cursor at the end', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(child: TextField(key: textFieldKey, controller: controller, maxLines: null)),
      ),
    );
    expect(controller.selection.baseOffset, -1);
    expect(controller.selection.extentOffset, -1);

    const String testValueOneLine = 'enough text to be exactly at the end of the line.';
    await tester.enterText(find.byType(TextField), testValueOneLine);
    await skipPastScrollingAnimation(tester);

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));

    RenderBox inputBox = findInputBox();
    final Size oneLineInputSize = inputBox.size;

    await tester.tapAt(textOffsetToPosition(tester, testValueOneLine.length));
    await tester.pump();

    const String testValueTwoLines = 'enough text to overflow the first line and go to the second';
    await tester.enterText(find.byType(TextField), testValueTwoLines);
    await skipPastScrollingAnimation(tester);

    expect(inputBox, findInputBox());
    inputBox = findInputBox();
    expect(inputBox.size.height, greaterThan(oneLineInputSize.height));
    final Size twoLineInputSize = inputBox.size;

    // Enter a string with the same number of characters as testValueTwoLines,
    // but where the overflowing part is all spaces. Assert that it only renders
    // on one line.
    const String testValueSpaces = '$testValueOneLine          ';
    expect(testValueSpaces.length, testValueTwoLines.length);
    await tester.enterText(find.byType(TextField), testValueSpaces);
    await skipPastScrollingAnimation(tester);

    expect(inputBox, findInputBox());
    inputBox = findInputBox();
    expect(inputBox.size.height, oneLineInputSize.height);

    // Swapping the final space for a letter causes it to wrap to 2 lines.
    const String testValueSpacesOverflow = '$testValueOneLine         a';
    expect(testValueSpacesOverflow.length, testValueTwoLines.length);
    await tester.enterText(find.byType(TextField), testValueSpacesOverflow);
    await skipPastScrollingAnimation(tester);

    expect(inputBox, findInputBox());
    inputBox = findInputBox();
    expect(inputBox.size.height, twoLineInputSize.height);

    // Positioning the cursor at the end of a line overflowing with spaces puts
    // it inside the input still.
    await tester.enterText(find.byType(TextField), testValueSpaces);
    await skipPastScrollingAnimation(tester);
    await tester.tapAt(textOffsetToPosition(tester, testValueSpaces.length));
    await tester.pump();

    final double inputWidth = findRenderEditable(tester).size.width;
    final Offset cursorOffsetSpaces =
        findRenderEditable(
          tester,
        ).getLocalRectForCaret(const TextPosition(offset: testValueSpaces.length)).bottomRight;

    expect(cursorOffsetSpaces.dx, inputWidth - kCaretGap);
  });

  testWidgets('Overflowing a line with spaces stops the cursor at the end (rtl direction)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      overlay(child: const TextField(textDirection: TextDirection.rtl, maxLines: null)),
    );

    const String testValueOneLine = 'enough text to be exactly at the end of the line.';
    const String testValueSpaces = '$testValueOneLine          ';

    // Positioning the cursor at the end of a line overflowing with spaces puts
    // it inside the input still.
    await tester.enterText(find.byType(TextField), testValueSpaces);
    await skipPastScrollingAnimation(tester);
    await tester.tapAt(textOffsetToPosition(tester, testValueSpaces.length));
    await tester.pump();

    final Offset cursorOffsetSpaces =
        findRenderEditable(
          tester,
        ).getLocalRectForCaret(const TextPosition(offset: testValueSpaces.length)).topLeft;

    expect(cursorOffsetSpaces.dx >= 0, isTrue);
  });

  testWidgets(
    'mobile obscureText control test',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        overlay(
          child: const TextField(
            obscureText: true,
            decoration: InputDecoration(hintText: 'Placeholder'),
          ),
        ),
      );
      await tester.showKeyboard(find.byType(TextField));

      const String testValue = 'ABC';
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testValue,
          selection: TextSelection.collapsed(offset: testValue.length),
        ),
      );

      await tester.pump();

      // Enter a character into the obscured field and verify that the character
      // is temporarily shown to the user and then changed to a bullet.
      const String newChar = 'X';
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testValue + newChar,
          selection: TextSelection.collapsed(offset: testValue.length + 1),
        ),
      );

      await tester.pump();

      String editText = (findRenderEditable(tester).text! as TextSpan).text!;
      expect(editText.substring(editText.length - 1), newChar);

      await tester.pump(const Duration(seconds: 2));

      editText = (findRenderEditable(tester).text! as TextSpan).text!;
      expect(editText.substring(editText.length - 1), '\u2022');
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'desktop obscureText control test',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        overlay(
          child: const TextField(
            obscureText: true,
            decoration: InputDecoration(hintText: 'Placeholder'),
          ),
        ),
      );
      await tester.showKeyboard(find.byType(TextField));

      const String testValue = 'ABC';
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testValue,
          selection: TextSelection.collapsed(offset: testValue.length),
        ),
      );

      await tester.pump();

      // Enter a character into the obscured field and verify that the character
      // isn't shown to the user.
      const String newChar = 'X';
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testValue + newChar,
          selection: TextSelection.collapsed(offset: testValue.length + 1),
        ),
      );

      await tester.pump();

      final String editText = (findRenderEditable(tester).text! as TextSpan).text!;
      expect(editText.substring(editText.length - 1), '\u2022');
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets('Caret position is updated on tap', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));
    expect(controller.selection.baseOffset, -1);
    expect(controller.selection.extentOffset, -1);

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    // Tap to reposition the caret.
    final int tapIndex = testValue.indexOf('e');
    final Offset ePos = textOffsetToPosition(tester, tapIndex);
    await tester.tapAt(ePos);
    await tester.pump();

    expect(controller.selection.baseOffset, tapIndex);
    expect(controller.selection.extentOffset, tapIndex);
  });

  testWidgets('enableInteractiveSelection = false, tap', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      overlay(child: TextField(controller: controller, enableInteractiveSelection: false)),
    );
    expect(controller.selection.baseOffset, -1);
    expect(controller.selection.extentOffset, -1);

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    // Tap would ordinarily reposition the caret.
    final int tapIndex = testValue.indexOf('e');
    final Offset ePos = textOffsetToPosition(tester, tapIndex);
    await tester.tapAt(ePos);
    await tester.pump();

    expect(controller.selection.baseOffset, testValue.length);
    expect(controller.selection.isCollapsed, isTrue);
  });

  testWidgets('Can long press to select', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    expect(controller.selection.isCollapsed, true);

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    await tester.longPressAt(ePos, pointer: 7);
    await tester.pumpAndSettle();

    // 'def' is selected.
    expect(controller.selection.baseOffset, testValue.indexOf('d'));
    expect(controller.selection.extentOffset, testValue.indexOf('f') + 1);

    // Tapping elsewhere immediately collapses and moves the cursor.
    await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('h')));
    await tester.pump();

    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, testValue.indexOf('h'));
  });

  testWidgets("Slight movements in longpress don't hide/show handles", (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    expect(controller.selection.isCollapsed, true);

    // Long press the 'e' to select 'def', but don't release the gesture.
    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await tester.pumpAndSettle();

    // Handles are shown
    final Finder fadeFinder = find.byType(FadeTransition);
    expect(fadeFinder, findsNWidgets(2)); // 2 handles, 1 toolbar
    FadeTransition handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));

    // Move the gesture very slightly
    await gesture.moveBy(const Offset(1.0, 1.0));
    await tester.pump(SelectionOverlay.fadeDuration * 0.5);
    handle = tester.widget(fadeFinder.at(0));

    // The handle should still be fully opaque.
    expect(handle.opacity.value, equals(1.0));
  });

  testWidgets(
    'Long pressing a field with selection 0,0 shows the selection menu',
    (WidgetTester tester) async {
      late final TextEditingController controller;
      addTearDown(() => controller.dispose());

      await tester.pumpWidget(
        overlay(
          child: TextField(
            controller:
                controller = TextEditingController.fromValue(
                  const TextEditingValue(selection: TextSelection(baseOffset: 0, extentOffset: 0)),
                ),
          ),
        ),
      );

      expect(find.text('Paste'), findsNothing);
      final Offset emptyPos = textOffsetToPosition(tester, 0);
      await tester.longPressAt(emptyPos, pointer: 7);
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('infinite multi-line text hint text is not ellipsized by default', (
    WidgetTester tester,
  ) async {
    const String kLongString =
        'Enter your email Enter your email Enter your '
        'email Enter your email Enter your email Enter '
        'your email Enter your email';
    const double defaultLineHeight = 24;
    await tester.pumpWidget(
      overlay(
        child: const TextField(
          maxLines: null,
          decoration: InputDecoration(labelText: 'Email', hintText: kLongString),
        ),
      ),
    );
    final Text hintText = tester.widget<Text>(find.text(kLongString));
    expect(hintText.overflow, isNull);
    final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(find.text(kLongString));
    expect(paragraph.size.height > defaultLineHeight * 2, isTrue);
  });

  testWidgets('non-infinite multi-line hint text is  ellipsized by default', (
    WidgetTester tester,
  ) async {
    const String kLongString =
        'Enter your email Enter your email Enter your '
        'email Enter your email Enter your email Enter '
        'your email Enter your email';
    const double defaultLineHeight = 24;
    await tester.pumpWidget(
      overlay(
        child: const TextField(
          maxLines: 2,
          decoration: InputDecoration(labelText: 'Email', hintText: kLongString),
        ),
      ),
    );
    final Text hintText = tester.widget<Text>(find.text(kLongString));
    expect(hintText.overflow, TextOverflow.ellipsis);
    final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(find.text(kLongString));
    expect(paragraph.size.height < defaultLineHeight * 2 + precisionErrorTolerance, isTrue);
  });

  testWidgets('Entering text hides selection handle caret', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'abcdefghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    // Handle not shown.
    expect(controller.selection.isCollapsed, true);
    final Finder fadeFinder = find.byType(FadeTransition);
    FadeTransition handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(0.0));

    // Tap on the text field to show the handle.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    expect(controller.selection.isCollapsed, true);
    expect(fadeFinder, findsNWidgets(1));
    handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));

    // Enter more text.
    const String testValueAddition = 'jklmni';
    await tester.enterText(find.byType(TextField), testValueAddition);
    expect(controller.value.text, testValueAddition);
    await skipPastScrollingAnimation(tester);

    // Handle not shown.
    expect(controller.selection.isCollapsed, true);
    handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(0.0));
  });

  testWidgets('multiple text fields with prefix and suffix have correct semantics order.', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller1 = _textEditingController(text: 'abc');
    final TextEditingController controller2 = _textEditingController(text: 'def');
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              TextField(
                decoration: const InputDecoration(prefixText: 'prefix1', suffixText: 'suffix1'),
                enabled: false,
                controller: controller1,
              ),
              TextField(
                decoration: const InputDecoration(prefixText: 'prefix2', suffixText: 'suffix2'),
                enabled: false,
                controller: controller2,
              ),
            ],
          ),
        ),
      ),
    );
    final List<String> orders =
        tester.semantics
            .simulatedAccessibilityTraversal(startNode: find.semantics.byLabel('prefix1'))
            .map((SemanticsNode node) => node.label + node.value)
            .toList();

    expect(orders, <String>['prefix1', 'abc', 'suffix1', 'prefix2', 'def', 'suffix2']);
  });

  testWidgets('selection handles are excluded from the semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'abcdefghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);
    // Tap on the text field to show the handle.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    // The semantics should only have the text field.
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.setSelection,
                SemanticsAction.paste,
                SemanticsAction.setText,
                SemanticsAction.moveCursorBackwardByWord,
              ],
              value: 'abcdefghi',
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection.collapsed(offset: 9),
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Mouse long press is just like a tap', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller)));

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    expect(controller.selection.isCollapsed, true);

    // Long press the 'e' using a mouse device.
    final int eIndex = testValue.indexOf('e');
    final Offset ePos = textOffsetToPosition(tester, eIndex);
    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    // The cursor is placed just like a regular tap.
    expect(controller.selection.baseOffset, eIndex);
    expect(controller.selection.extentOffset, eIndex);
  });

  testWidgets('Read only text field basic', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'readonly');

    await tester.pumpWidget(overlay(child: TextField(controller: controller, readOnly: true)));
    // Read only text field cannot open keyboard.
    await tester.showKeyboard(find.byType(TextField));
    // On web, we always create a client connection to the engine.
    expect(tester.testTextInput.hasAnyClients, isBrowser ? isTrue : isFalse);
    await skipPastScrollingAnimation(tester);

    expect(controller.selection.isCollapsed, true);

    await tester.tap(find.byType(TextField));
    await tester.pump();
    // On web, we always create a client connection to the engine.
    expect(tester.testTextInput.hasAnyClients, isBrowser ? isTrue : isFalse);
    final EditableTextState editableText = tester.state(find.byType(EditableText));
    // Collapse selection should not paint.
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
    // Long press on the 'd' character of text 'readOnly' to show context menu.
    const int dIndex = 3;
    final Offset dPos = textOffsetToPosition(tester, dIndex);
    await tester.longPressAt(dPos);
    await tester.pumpAndSettle();

    // Context menu should not have paste and cut.
    expect(find.text('Copy'), isContextMenuProvidedByPlatform ? findsNothing : findsOneWidget);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets(
    'does not paint toolbar when no options available',
    (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Material(child: TextField(readOnly: true))));

      await tester.tap(find.byType(TextField));
      await tester.pump(const Duration(milliseconds: 50));

      await tester.tap(find.byType(TextField));
      // Wait for context menu to be built.
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoTextSelectionToolbar), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'text field build empty toolbar when no options available',
    (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Material(child: TextField(readOnly: true))));

      await tester.tap(find.byType(TextField));
      await tester.pump(const Duration(milliseconds: 50));

      await tester.tap(find.byType(TextField));
      // Wait for context menu to be built.
      await tester.pumpAndSettle();
      final RenderBox container = tester.renderObject(
        find.descendant(of: find.byType(SnapshotWidget), matching: find.byType(SizedBox)).first,
      );
      expect(container.size, Size.zero);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets('Swapping controllers should update selection', (WidgetTester tester) async {
    TextEditingController controller = _textEditingController(text: 'readonly');
    final OverlayEntry entry = OverlayEntry(
      builder: (BuildContext context) {
        return Center(child: Material(child: TextField(controller: controller, readOnly: true)));
      },
    );
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );
    await tester.pumpWidget(overlayWithEntry(entry));
    const int dIndex = 3;
    final Offset dPos = textOffsetToPosition(tester, dIndex);
    await tester.longPressAt(dPos);
    await tester.pumpAndSettle();
    final EditableTextState state = tester.state(find.byType(EditableText));
    TextSelection currentOverlaySelection = state.selectionOverlay!.value.selection;
    expect(currentOverlaySelection.baseOffset, 0);
    expect(currentOverlaySelection.extentOffset, 8);

    // Update selection from [0 to 8] to [1 to 7].
    controller = TextEditingController.fromValue(
      controller.value.copyWith(selection: const TextSelection(baseOffset: 1, extentOffset: 7)),
    );
    addTearDown(controller.dispose);

    // Mark entry to be dirty in order to trigger overlay update.
    entry.markNeedsBuild();

    await tester.pump();
    currentOverlaySelection = state.selectionOverlay!.value.selection;
    expect(currentOverlaySelection.baseOffset, 1);
    expect(currentOverlaySelection.extentOffset, 7);
  });

  testWidgets('Read only text should not compose', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController.fromValue(
      const TextEditingValue(
        text: 'readonly',
        composing: TextRange(start: 0, end: 8), // Simulate text composing.
      ),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(overlay(child: TextField(controller: controller, readOnly: true)));

    final RenderEditable renderEditable = findRenderEditable(tester);
    // There should be no composing.
    expect(renderEditable.text, TextSpan(text: 'readonly', style: renderEditable.text!.style));
  });

  testWidgets(
    'Dynamically switching between read only and not read only should hide or show collapse cursor',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'readonly');
      bool readOnly = true;
      final OverlayEntry entry = OverlayEntry(
        builder: (BuildContext context) {
          return Center(
            child: Material(child: TextField(controller: controller, readOnly: readOnly)),
          );
        },
      );
      addTearDown(
        () =>
            entry
              ..remove()
              ..dispose(),
      );
      await tester.pumpWidget(overlayWithEntry(entry));
      await tester.tap(find.byType(TextField));
      await tester.pump();

      final EditableTextState editableText = tester.state(find.byType(EditableText));
      // Collapse selection should not paint.
      expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);

      readOnly = false;
      // Mark entry to be dirty in order to trigger overlay update.
      entry.markNeedsBuild();
      await tester.pumpAndSettle();
      expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);

      readOnly = true;
      entry.markNeedsBuild();
      await tester.pumpAndSettle();
      expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
    },
  );

  testWidgets('Dynamically switching to read only should close input connection', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(text: 'readonly');
    bool readOnly = false;
    final OverlayEntry entry = OverlayEntry(
      builder: (BuildContext context) {
        return Center(
          child: Material(child: TextField(controller: controller, readOnly: readOnly)),
        );
      },
    );
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );
    await tester.pumpWidget(overlayWithEntry(entry));
    await tester.tap(find.byType(TextField));
    await tester.pump();
    expect(tester.testTextInput.hasAnyClients, true);

    readOnly = true;
    // Mark entry to be dirty in order to trigger overlay update.
    entry.markNeedsBuild();
    await tester.pump();
    // On web, we always have a client connection to the engine.
    expect(tester.testTextInput.hasAnyClients, isBrowser ? isTrue : isFalse);
  });

  testWidgets('Dynamically switching to non read only should open input connection', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(text: 'readonly');
    bool readOnly = true;
    final OverlayEntry entry = OverlayEntry(
      builder: (BuildContext context) {
        return Center(
          child: Material(child: TextField(controller: controller, readOnly: readOnly)),
        );
      },
    );
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );
    await tester.pumpWidget(overlayWithEntry(entry));
    await tester.tap(find.byType(TextField));
    await tester.pump();
    // On web, we always have a client connection to the engine.
    expect(tester.testTextInput.hasAnyClients, isBrowser ? isTrue : isFalse);

    readOnly = false;
    // Mark entry to be dirty in order to trigger overlay update.
    entry.markNeedsBuild();
    await tester.pump();
    expect(tester.testTextInput.hasAnyClients, true);
  });

  testWidgets('enableInteractiveSelection = false, long-press', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      overlay(child: TextField(controller: controller, enableInteractiveSelection: false)),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(controller.value.text, testValue);
    await skipPastScrollingAnimation(tester);

    expect(controller.selection.isCollapsed, true);

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    await tester.longPressAt(ePos, pointer: 7);
    await tester.pumpAndSettle();

    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, testValue.length);
  });

  testWidgets('Selection updates on tap down (Desktop platforms)', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, 5);
    final Offset gPos = textOffsetToPosition(tester, 8);

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pumpAndSettle();
    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 5);

    await gesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);

    await gesture.down(gPos);
    await tester.pumpAndSettle();
    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 8);

    // This should do nothing. The selection is set on tap down on desktop platforms.
    await gesture.up();
    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 8);
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Selection updates on tap up (Mobile platforms)', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();
    final bool isTargetPlatformApple = defaultTargetPlatform == TargetPlatform.iOS;

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, 5);
    final Offset gPos = textOffsetToPosition(tester, 8);

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await gesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);

    await gesture.down(gPos);
    await tester.pumpAndSettle();
    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 5);

    await gesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 8);

    final TestGesture touchGesture = await tester.startGesture(ePos);
    await touchGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    // On iOS a tap to select, selects the word edge instead of the exact tap position.
    expect(controller.selection.baseOffset, isTargetPlatformApple ? 7 : 5);
    expect(controller.selection.extentOffset, isTargetPlatformApple ? 7 : 5);

    // Selection should stay the same since it is set on tap up for mobile platforms.
    await touchGesture.down(gPos);
    await tester.pump();
    expect(controller.selection.baseOffset, isTargetPlatformApple ? 7 : 5);
    expect(controller.selection.extentOffset, isTargetPlatformApple ? 7 : 5);

    await touchGesture.up();
    await tester.pumpAndSettle();
    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 8);
  }, variant: TargetPlatformVariant.mobile());

  testWidgets(
    'Can select text with a mouse when wrapped in a GestureDetector with tap/double tap callbacks',
    (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/129161.
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: GestureDetector(
              onTap: () {},
              onDoubleTap: () {},
              child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

      final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
      await tester.pump();
      await gesture.up();
      // This is to allow the GestureArena to decide a winner between TapGestureRecognizer,
      // DoubleTapGestureRecognizer, and BaseTapAndDragGestureRecognizer.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('e'));

      await gesture.down(ePos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(controller.selection.baseOffset, testValue.indexOf('e'));
      expect(controller.selection.extentOffset, testValue.indexOf('g'));
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Can select text by dragging with a mouse', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      ),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(gPos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, testValue.indexOf('e'));
    expect(controller.selection.extentOffset, testValue.indexOf('g'));
  });

  testWidgets(
    'Can move cursor when dragging, when tap is on collapsed selection (iOS)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|g'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(ePos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 7);

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|g', where our selection was previously, and move to '|i'.
      await gesture.down(textOffsetToPosition(tester, 7));
      await tester.pump();
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('i'));

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Cursor should not move on a quick touch drag when touch does not begin on previous selection (iOS)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|a'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);

      // The position we tap during a drag start is not on the collapsed selection,
      // so the cursor should not move.
      await gesture.down(textOffsetToPosition(tester, 7));
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Can move cursor when dragging, when tap is on collapsed selection (iOS) - multiline',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: TextField(
              dragStartBehavior: DragStartBehavior.down,
              controller: controller,
              maxLines: null,
            ),
          ),
        ),
      );

      const String testValue = 'abc\ndef\nghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|a'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|a', where our selection was previously, and move to '|i'.
      await gesture.down(aPos);
      await tester.pump();
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('i'));

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Can move cursor when dragging, when tap is on collapsed selection (iOS) - PageView',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/142624.
      final TextEditingController controller = _textEditingController();
      final PageController pageController = PageController();
      addTearDown(pageController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: PageView(
              controller: pageController,
              children: <Widget>[
                Center(
                  child: TextField(
                    dragStartBehavior: DragStartBehavior.down,
                    controller: controller,
                  ),
                ),
                const SizedBox(height: 200.0, child: Center(child: Text('Page 2'))),
              ],
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi jkl mno pqr stu vwx yz';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|a'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|a', where our selection was previously, and attempt move
      // to '|g'.
      await gesture.down(aPos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));

      // Release the pointer.
      await gesture.up();
      await tester.pumpAndSettle();

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|g', where our selection was previously, and move to '|i'.
      await gesture.down(gPos);
      await tester.pump();
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('i'));

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();

      expect(pageController.page, isNotNull);
      expect(pageController.page, 0.0);
      // A horizontal drag directly on the TextField, but not on the current
      // collapsed selection should move the page view to the next page.
      final Rect textFieldRect = tester.getRect(find.byType(TextField));
      await tester.dragFrom(
        textFieldRect.centerRight - const Offset(0.1, 0.0),
        const Offset(-500.0, 0.0),
      );
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, testValue.indexOf('i'));
      expect(pageController.page, isNotNull);
      expect(pageController.page, 1.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Can move cursor when dragging, when tap is on collapsed selection (iOS) - TextField in Dismissible',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/124421.
      final TextEditingController controller = _textEditingController();
      bool dismissed = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: ListView(
              children: <Widget>[
                Dismissible(
                  key: UniqueKey(),
                  onDismissed: (DismissDirection? direction) {
                    dismissed = true;
                  },
                  child: TextField(
                    dragStartBehavior: DragStartBehavior.down,
                    controller: controller,
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi jkl mno pqr stu vwx yz';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|a'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|a', where our selection was previously, and attempt move
      // to '|g'.
      await gesture.down(aPos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));

      // Release the pointer.
      await gesture.up();
      await tester.pumpAndSettle();

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|g', where our selection was previously, and move to '|i'.
      await gesture.down(gPos);
      await tester.pump();
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('i'));

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();

      expect(dismissed, false);
      // A horizontal drag directly on the TextField, but not on the current
      // collapsed selection should allow for the Dismissible to be dismissed.
      await tester.dragFrom(
        tester.getRect(find.byType(TextField)).centerRight - const Offset(0.1, 0.0),
        const Offset(-400.0, 0.0),
      );
      await tester.pumpAndSettle();
      expect(dismissed, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Can move cursor when dragging, when tap is on collapsed selection (iOS) - ListView',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/122519
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: ListView(
              children: <Widget>[
                TextField(
                  dragStartBehavior: DragStartBehavior.down,
                  controller: controller,
                  maxLines: null,
                ),
              ],
            ),
          ),
        ),
      );

      const String testValue = 'abc\ndef\nghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));
      final Offset iPos = textOffsetToPosition(tester, testValue.indexOf('i'));

      // Tap on text field to gain focus, and set selection to '|a'. On iOS
      // the selection is set to the word edge closest to the tap position.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|a', where our selection was previously, and attempt move
      // to '|g'.
      await gesture.down(aPos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));

      // Release the pointer.
      await gesture.up();
      await tester.pumpAndSettle();

      // If the position we tap during a drag start is on the collapsed selection, then
      // we can move the cursor with a drag.
      // Here we tap on '|g', where our selection was previously, and move to '|i'.
      await gesture.down(gPos);
      await tester.pump();
      await gesture.moveTo(iPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('i'));

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Can move cursor when dragging (Android)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

      // Tap on text field to gain focus, and set selection to '|e'.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(ePos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('e'));

      // Here we tap on '|d', and move to '|g'.
      await gesture.down(textOffsetToPosition(tester, testValue.indexOf('d')));
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'Can move cursor when dragging (Android) - multiline',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: TextField(
              dragStartBehavior: DragStartBehavior.down,
              controller: controller,
              maxLines: null,
            ),
          ),
        ),
      );

      const String testValue = 'abc\ndef\nghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

      // Tap on text field to gain focus, and set selection to '|a'.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(aPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('a'));

      // Here we tap on '|c', and move down to '|g'.
      await gesture.down(textOffsetToPosition(tester, testValue.indexOf('c')));
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'Can move cursor when dragging (Android) - ListView',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/122519
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: ListView(
              children: <Widget>[
                TextField(
                  dragStartBehavior: DragStartBehavior.down,
                  controller: controller,
                  maxLines: null,
                ),
              ],
            ),
          ),
        ),
      );

      const String testValue = 'abc\ndef\nghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
      final Offset cPos = textOffsetToPosition(tester, testValue.indexOf('c'));
      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

      // Tap on text field to gain focus, and set selection to '|c'.
      // We await for kDoubleTapTimeout after the up event, so our next down event
      // does not register as a double tap.
      final TestGesture gesture = await tester.startGesture(cPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('c'));

      // Here we tap on '|a', and attempt move to '|g'. The cursor will not move
      // because the `VerticalDragGestureRecognizer` in the scrollable will beat
      // the `TapAndHorizontalDragGestureRecognizer` in the TextField. This is
      // because moving from `|a` to `|g` is a completely vertical movement.
      await gesture.down(aPos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('c'));

      // Release the pointer.
      await gesture.up();
      await tester.pumpAndSettle();

      // Here we tap on '|c', and move to '|g'. Unlike our previous attempt to
      // drag to `|g`, this works because moving from `|c` to `|g` includes a
      // horizontal movement so the `TapAndHorizontalDragGestureRecognizer`
      // in TextField can beat the `VerticalDragGestureRecognizer` in the scrollable.
      await gesture.down(cPos);
      await tester.pump();
      await gesture.moveTo(gPos);
      await tester.pumpAndSettle();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('g'));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets('Continuous dragging does not cause flickering', (WidgetTester tester) async {
    int selectionChangedCount = 0;
    const String testValue = 'abc def ghi';
    final TextEditingController controller = _textEditingController(text: testValue);

    controller.addListener(() {
      selectionChangedCount++;
    });

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(
            dragStartBehavior: DragStartBehavior.down,
            controller: controller,
            style: const TextStyle(fontSize: 10.0),
          ),
        ),
      ),
    );

    final Offset cPos = textOffsetToPosition(tester, 2); // Index of 'c'.
    final Offset gPos = textOffsetToPosition(tester, 8); // Index of 'g'.
    final Offset hPos = textOffsetToPosition(tester, 9); // Index of 'h'.

    // Drag from 'c' to 'g'.
    final TestGesture gesture = await tester.startGesture(cPos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(gPos);
    await tester.pumpAndSettle();

    expect(selectionChangedCount, isNonZero);
    selectionChangedCount = 0;
    expect(controller.selection.baseOffset, 2);
    expect(controller.selection.extentOffset, 8);

    // Tiny movement shouldn't cause text selection to change.
    await gesture.moveTo(gPos + const Offset(2.0, 0.0));
    await tester.pumpAndSettle();
    expect(selectionChangedCount, 0);

    // Now a text selection change will occur after a significant movement.
    await gesture.moveTo(hPos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(selectionChangedCount, 1);
    expect(controller.selection.baseOffset, 2);
    expect(controller.selection.extentOffset, 9);
  });

  testWidgets('Dragging in opposite direction also works', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      ),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

    final TestGesture gesture = await tester.startGesture(gPos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(ePos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, testValue.indexOf('g'));
    expect(controller.selection.extentOffset, testValue.indexOf('e'));
  });

  testWidgets('Slow mouse dragging also selects text', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      ),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump(const Duration(seconds: 2));
    await gesture.moveTo(gPos);
    await tester.pump();
    await gesture.up();

    expect(controller.selection.baseOffset, testValue.indexOf('e'));
    expect(controller.selection.extentOffset, testValue.indexOf('g'));
  });

  testWidgets(
    'Can drag handles to change selection on Apple platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        overlay(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Double tap the 'e' to select 'def'.
      final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
      // The first tap.
      TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      // The second tap.
      await gesture.down(ePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      final TextSelection selection = controller.selection;
      expect(selection.baseOffset, 4);
      expect(selection.extentOffset, 7);

      final RenderEditable renderEditable = findRenderEditable(tester);
      List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      // Drag the right handle 2 letters to the right.
      // We use a small offset because the endpoint is on the very corner
      // of the handle.
      Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      Offset newHandlePos = textOffsetToPosition(tester, testValue.length);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 11);

      // Drag the left handle 2 letters to the left.
      handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
      newHandlePos = textOffsetToPosition(tester, 2);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      switch (defaultTargetPlatform) {
        // On Apple platforms, dragging the base handle makes it the extent.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection.baseOffset, 11);
          expect(controller.selection.extentOffset, 2);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection.baseOffset, 2);
          expect(controller.selection.extentOffset, 11);
      }

      // Drag the left handle 2 letters to the left again.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
      newHandlePos = textOffsetToPosition(tester, 0);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          // The left handle was already the extent, and it remains so.
          expect(controller.selection.baseOffset, 11);
          expect(controller.selection.extentOffset, 0);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection.baseOffset, 0);
          expect(controller.selection.extentOffset, 11);
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Can drag handles to change selection on non-Apple platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        overlay(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Long press the 'e' to select 'def'.
      final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
      TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
      await tester.pump(const Duration(seconds: 2));
      await gesture.up();
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      final TextSelection selection = controller.selection;
      expect(selection.baseOffset, 4);
      expect(selection.extentOffset, 7);

      final RenderEditable renderEditable = findRenderEditable(tester);
      List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      // Drag the right handle 2 letters to the right.
      // We use a small offset because the endpoint is on the very corner
      // of the handle.
      Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      Offset newHandlePos = textOffsetToPosition(tester, testValue.length);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 11);

      // Drag the left handle 2 letters to the left.
      handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
      newHandlePos = textOffsetToPosition(tester, 2);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      switch (defaultTargetPlatform) {
        // On Apple platforms, dragging the base handle makes it the extent.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection.baseOffset, 11);
          expect(controller.selection.extentOffset, 2);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection.baseOffset, 2);
          expect(controller.selection.extentOffset, 11);
      }

      // Drag the left handle 2 letters to the left again.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
      newHandlePos = textOffsetToPosition(tester, 0);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          // The left handle was already the extent, and it remains so.
          expect(controller.selection.baseOffset, 11);
          expect(controller.selection.extentOffset, 0);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection.baseOffset, 0);
          expect(controller.selection.extentOffset, 11);
      }
    },
    variant: TargetPlatformVariant.all(
      excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.macOS},
    ),
  );

  testWidgets('Can drag the left handle while the right handle remains off-screen', (
    WidgetTester tester,
  ) async {
    // Text is longer than textfield width.
    const String testValue = 'aaaaaaaaaaaaaaaaaaaaaaaaaaa bbbbbbbbbbbbbbbbbbbbbbbbbbb';
    final TextEditingController controller = _textEditingController(text: testValue);
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MediaQuery(
            data: const MediaQueryData(size: Size(800.0, 600.0)),
            child: TextField(
              dragStartBehavior: DragStartBehavior.down,
              controller: controller,
              scrollController: scrollController,
            ),
          ),
        ),
      ),
    );

    // Double tap 'b' to show handles.
    final Offset bPos = textOffsetToPosition(tester, testValue.indexOf('b'));
    await tester.tapAt(bPos);
    await tester.pump(kDoubleTapTimeout ~/ 2);
    await tester.tapAt(bPos);
    await tester.pumpAndSettle();

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 28);
    expect(selection.extentOffset, testValue.length);

    // Move to the left edge.
    scrollController.jumpTo(0);
    await tester.pumpAndSettle();

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Left handle should appear between textfield's left and right position.
    final Offset textFieldLeftPosition = tester.getTopLeft(find.byType(TextField));
    expect(endpoints[0].point.dx - textFieldLeftPosition.dx, isPositive);
    final Offset textFieldRightPosition = tester.getTopRight(find.byType(TextField));
    expect(textFieldRightPosition.dx - endpoints[0].point.dx, isPositive);
    // Right handle should remain off-screen.
    expect(endpoints[1].point.dx - textFieldRightPosition.dx, isPositive);

    // Drag the left handle to the right by 25 offset.
    const int toOffset = 25;
    final double beforeScrollOffset = scrollController.offset;
    final Offset handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, toOffset);
    final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // On Apple platforms, dragging the base handle makes it the extent.
        expect(controller.selection.baseOffset, testValue.length);
        expect(controller.selection.extentOffset, toOffset);
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        expect(controller.selection.baseOffset, toOffset);
        expect(controller.selection.extentOffset, testValue.length);
    }

    // The scroll area of text field should not move.
    expect(scrollController.offset, beforeScrollOffset);
  });

  testWidgets('Can drag the right handle while the left handle remains off-screen', (
    WidgetTester tester,
  ) async {
    // Text is longer than textfield width.
    const String testValue = 'aaaaaaaaaaaaaaaaaaaaaaaaaaa bbbbbbbbbbbbbbbbbbbbbbbbbbb';
    final TextEditingController controller = _textEditingController(text: testValue);
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MediaQuery(
            data: const MediaQueryData(size: Size(800.0, 600.0)),
            child: TextField(
              dragStartBehavior: DragStartBehavior.down,
              controller: controller,
              scrollController: scrollController,
            ),
          ),
        ),
      ),
    );

    // Double tap 'a' to show handles.
    final Offset aPos = textOffsetToPosition(tester, testValue.indexOf('a'));
    await tester.tapAt(aPos);
    await tester.pump(kDoubleTapTimeout ~/ 2);
    await tester.tapAt(aPos);
    await tester.pumpAndSettle();

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 0);
    expect(selection.extentOffset, 27);

    // Move to the right edge.
    scrollController.jumpTo(800);
    await tester.pumpAndSettle();

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Right handle should appear between textfield's left and right position.
    final Offset textFieldLeftPosition = tester.getTopLeft(find.byType(TextField));
    expect(endpoints[1].point.dx - textFieldLeftPosition.dx, isPositive);
    final Offset textFieldRightPosition = tester.getTopRight(find.byType(TextField));
    expect(textFieldRightPosition.dx - endpoints[1].point.dx, isPositive);
    // Left handle should remain off-screen.
    expect(endpoints[0].point.dx, isNegative);

    // Drag the right handle to the left by 50 offset.
    const int toOffset = 50;
    final double beforeScrollOffset = scrollController.offset;
    final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, toOffset);
    final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, toOffset);

    // The scroll area of text field should not move.
    expect(scrollController.offset, beforeScrollOffset);
  });

  testWidgets('Drag handles trigger feedback', (WidgetTester tester) async {
    final FeedbackTester feedback = FeedbackTester();
    addTearDown(feedback.dispose);
    final TextEditingController controller = _textEditingController();
    await tester.pumpWidget(
      overlay(child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller)),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(feedback.hapticCount, 0);
    await skipPastScrollingAnimation(tester);

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 4);
    expect(selection.extentOffset, 7);
    expect(feedback.hapticCount, 1);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle 2 letters to the right.
    // Use a small offset because the endpoint is on the very corner
    // of the handle.
    final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, testValue.length);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 11);
    expect(feedback.hapticCount, 2);
  });

  testWidgets('Dragging a collapsed handle should trigger feedback.', (WidgetTester tester) async {
    final FeedbackTester feedback = FeedbackTester();
    addTearDown(feedback.dispose);
    final TextEditingController controller = _textEditingController();
    await tester.pumpWidget(
      overlay(child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller)),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    expect(feedback.hapticCount, 0);
    await skipPastScrollingAnimation(tester);

    // Tap the 'e' to bring up a collapsed handle.
    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 5);
    expect(selection.extentOffset, 5);
    expect(feedback.hapticCount, 0);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 1);

    // Drag the right handle 3 letters to the right.
    // Use a small offset because the endpoint is on the very corner
    // of the handle.
    final Offset handlePos = endpoints[0].point + const Offset(1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, testValue.indexOf('g'));
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 8);
    expect(feedback.hapticCount, 1);
  });

  testWidgets('Cannot drag one handle past the other', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      overlay(child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller)),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5); // Position before 'e'.
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 4);
    expect(selection.extentOffset, 7);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle until there's only 1 char selected.
    // We use a small offset because the endpoint is on the very corner
    // of the handle.
    final Offset handlePos = endpoints[1].point + const Offset(4.0, 0.0);
    Offset newHandlePos = textOffsetToPosition(tester, 5); // Position before 'e'.
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 5);

    newHandlePos = textOffsetToPosition(tester, 2); // Position before 'c'.
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    // The selection doesn't move beyond the left handle. There's always at
    // least 1 char selected.
    expect(controller.selection.extentOffset, 5);
  });

  testWidgets(
    'Dragging between multiple lines keeps the contact point at the same place on the handle on Android',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        // 11 first line, 19 second line, 17 third line = length 49
        text: 'a big house\njumped over a mouse\nOne more line yay',
      );

      await tester.pumpWidget(
        overlay(
          child: TextField(
            dragStartBehavior: DragStartBehavior.down,
            controller: controller,
            maxLines: 3,
            minLines: 3,
          ),
        ),
      );

      // Double tap to select 'over'.
      final Offset pos = textOffsetToPosition(tester, controller.text.indexOf('v'));
      // The first tap.
      TestGesture gesture = await tester.startGesture(pos, pointer: 7);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      // The second tap.
      await gesture.down(pos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      final TextSelection selection = controller.selection;
      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 23));

      final RenderEditable renderEditable = findRenderEditable(tester);
      List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      // Drag the right handle 4 letters to the right.
      // The adjustment moves the tap from the text position to the handle.
      const Offset endHandleAdjustment = Offset(1.0, 6.0);
      Offset handlePos = endpoints[1].point + endHandleAdjustment;
      Offset newHandlePos = textOffsetToPosition(tester, 27) + endHandleAdjustment;
      await tester.pump();
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 27));

      // Drag the right handle 1 line down.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[1].point + endHandleAdjustment;
      final Offset toNextLine = Offset(0.0, findRenderEditable(tester).preferredLineHeight + 3.0);
      newHandlePos = handlePos + toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 47));

      // Drag the right handle back up 1 line.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[1].point + endHandleAdjustment;
      newHandlePos = handlePos - toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 27));

      // Drag the left handle 4 letters to the left.
      // The adjustment moves the tap from the text position to the handle.
      const Offset startHandleAdjustment = Offset(-1.0, 6.0);
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      newHandlePos = textOffsetToPosition(tester, 15) + startHandleAdjustment;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 15, extentOffset: 27));

      // Drag the left handle 1 line up.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      newHandlePos = handlePos - toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 3, extentOffset: 27));

      // Drag the left handle 1 line back down.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      newHandlePos = handlePos + toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 15, extentOffset: 27));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'Dragging between multiple lines keeps the contact point at the same place on the handle on iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        // 11 first line, 19 second line, 17 third line = length 49
        text: 'a big house\njumped over a mouse\nOne more line yay',
      );

      await tester.pumpWidget(
        Theme(
          data: ThemeData(useMaterial3: false),
          child: overlay(
            child: TextField(
              dragStartBehavior: DragStartBehavior.down,
              controller: controller,
              maxLines: 3,
              minLines: 3,
            ),
          ),
        ),
      );

      // Double tap to select 'over'.
      final Offset pos = textOffsetToPosition(tester, controller.text.indexOf('v'));
      // The first tap.
      TestGesture gesture = await tester.startGesture(pos, pointer: 7);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      // The second tap.
      await gesture.down(pos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      final TextSelection selection = controller.selection;
      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 23));

      final RenderEditable renderEditable = findRenderEditable(tester);
      List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      // Drag the right handle 4 letters to the right.
      // The adjustment moves the tap from the text position to the handle.
      const Offset endHandleAdjustment = Offset(1.0, 6.0);
      Offset handlePos = endpoints[1].point + endHandleAdjustment;
      Offset newHandlePos = textOffsetToPosition(tester, 27) + endHandleAdjustment;
      await tester.pump();
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 27));

      // Drag the right handle 1 line down.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[1].point + endHandleAdjustment;
      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      final Offset toNextLine = Offset(0.0, lineHeight + 3.0);
      newHandlePos = handlePos + toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 47));

      // Drag the right handle back up 1 line.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[1].point + endHandleAdjustment;
      newHandlePos = handlePos - toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 27));

      // Drag the left handle 4 letters to the left.
      // The adjustment moves the tap from the text position to the handle.
      final Offset startHandleAdjustment = Offset(-1.0, -lineHeight + 6.0);
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      newHandlePos = textOffsetToPosition(tester, 15) + startHandleAdjustment;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      // On Apple platforms, dragging the base handle makes it the extent.
      expect(controller.selection, const TextSelection(baseOffset: 27, extentOffset: 15));

      // Drag the left handle 1 line up.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      // Move handle a sufficient global distance so it can be considered a drag
      // by the selection handle's [PanGestureRecognizer].
      newHandlePos = handlePos - (toNextLine * 2);
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 27, extentOffset: 3));

      // Drag the left handle 1 line back down.
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      handlePos = endpoints[0].point + startHandleAdjustment;
      newHandlePos = handlePos + toNextLine;
      gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      // Move handle up a small amount before dragging it down so the total global
      // distance travelled can be accepted by the selection handle's [PanGestureRecognizer] as a drag.
      // This way it can declare itself the winner before the [TapAndDragGestureRecognizer] that
      // is on the selection overlay.
      await gesture.moveTo(handlePos - toNextLine);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection, const TextSelection(baseOffset: 27, extentOffset: 15));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    "dragging caret within a word doesn't affect composing region",
    (WidgetTester tester) async {
      const String testValue = 'abc def ghi';
      final TextEditingController controller = TextEditingController.fromValue(
        const TextEditingValue(
          text: testValue,
          selection: TextSelection(baseOffset: 4, extentOffset: 4, affinity: TextAffinity.upstream),
          composing: TextRange(start: 4, end: 7),
        ),
      );
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        overlay(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      );

      await tester.pump();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 4);
      expect(controller.value.composing.start, 4);
      expect(controller.value.composing.end, 7);

      // Tap the caret to show the handle.
      final Offset ePos = textOffsetToPosition(tester, 4);
      await tester.tapAt(ePos);
      await tester.pumpAndSettle();

      final TextSelection selection = controller.selection;
      expect(controller.selection.isCollapsed, true);
      expect(selection.baseOffset, 4);
      expect(controller.value.composing.start, 4);
      expect(controller.value.composing.end, 7);

      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 1);

      // Drag the right handle 2 letters to the right.
      // We use a small offset because the endpoint is on the very corner
      // of the handle.
      final Offset handlePos = endpoints[0].point + const Offset(1.0, 1.0);
      final Offset newHandlePos = textOffsetToPosition(tester, 7);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 7);
      expect(controller.value.composing.start, 4);
      expect(controller.value.composing.end, 7);
    },
    skip: kIsWeb, // [intended] text selection is handled by the browser
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'Can use selection toolbar',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(overlay(child: TextField(controller: controller)));

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Tap the selection handle to bring up the "paste / select all" menu.
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero
      RenderEditable renderEditable = findRenderEditable(tester);
      List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      // Tapping on the part of the handle's GestureDetector where it overlaps
      // with the text itself does not show the menu, so add a small vertical
      // offset to tap below the text.
      await tester.tapAt(endpoints[0].point + const Offset(1.0, 13.0));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      // Select all should select all the text.
      await tester.tap(find.text('Select all'));
      await tester.pump();
      expect(controller.selection.baseOffset, 0);
      expect(controller.selection.extentOffset, testValue.length);

      // Copy should reset the selection.
      await tester.tap(find.text('Copy'));
      await skipPastScrollingAnimation(tester);
      expect(controller.selection.isCollapsed, true);

      // Tap again to bring back the menu.
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump();
      // Allow time for handle to appear and double tap to time out.
      await tester.pump(const Duration(milliseconds: 300));
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('e'));
      expect(controller.selection.extentOffset, testValue.indexOf('e'));
      renderEditable = findRenderEditable(tester);
      endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      await tester.tapAt(endpoints[0].point + const Offset(1.0, 1.0));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, testValue.indexOf('e'));
      expect(controller.selection.extentOffset, testValue.indexOf('e'));

      // Paste right before the 'e'.
      await tester.tap(find.text('Paste'));
      await tester.pump();
      expect(controller.text, 'abc d${testValue}ef ghi');
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  // Show the selection menu at the given index into the text by tapping to
  // place the cursor and then tapping on the handle.
  Future<void> showSelectionMenuAt(
    WidgetTester tester,
    TextEditingController controller,
    int index,
  ) async {
    await tester.tapAt(tester.getCenter(find.byType(EditableText)));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero
    expect(find.text('Select all'), findsNothing);

    // Tap the selection handle to bring up the "paste / select all" menu for
    // the last line of text.
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero
    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    // Tapping on the part of the handle's GestureDetector where it overlaps
    // with the text itself does not show the menu, so add a small vertical
    // offset to tap below the text.
    await tester.tapAt(endpoints[0].point + const Offset(1.0, 13.0));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero
  }

  testWidgets(
    'Check the toolbar appears below the TextField when there is not enough space above the TextField to show it',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/29808
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Padding(
              padding: const EdgeInsets.all(30.0),
              child: TextField(controller: controller),
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      await showSelectionMenuAt(tester, controller, testValue.indexOf('e'));

      // Verify the selection toolbar position is below the text.
      Offset toolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      Offset textFieldTopLeft = tester.getTopLeft(find.byType(TextField));
      expect(textFieldTopLeft.dy, lessThan(toolbarTopLeft.dy));

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Padding(
              padding: const EdgeInsets.all(150.0),
              child: TextField(controller: controller),
            ),
          ),
        ),
      );

      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      await showSelectionMenuAt(tester, controller, testValue.indexOf('e'));

      // Verify the selection toolbar position
      toolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      textFieldTopLeft = tester.getTopLeft(find.byType(TextField));
      expect(toolbarTopLeft.dy, lessThan(textFieldTopLeft.dy));
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'the toolbar adjusts its position above/below when bottom inset changes',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 48.0),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    IntrinsicHeight(
                      child: TextField(controller: controller, expands: true, maxLines: null),
                    ),
                    const SizedBox(height: 325.0),
                  ],
                ),
              ),
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      await showSelectionMenuAt(tester, controller, testValue.indexOf('e'));

      // Verify the selection toolbar position is above the text.
      expect(find.text('Select all'), findsOneWidget);
      Offset toolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      Offset textFieldTopLeft = tester.getTopLeft(find.byType(TextField));
      expect(toolbarTopLeft.dy, lessThan(textFieldTopLeft.dy));

      // Add a viewInset tall enough to push the field to the top, where there
      // is no room to display the toolbar above. This is similar to when the
      // keyboard is shown.
      tester.view.viewInsets = const FakeViewPadding(bottom: 500.0);
      addTearDown(tester.view.reset);
      await tester.pumpAndSettle();

      // Verify the selection toolbar position is below the text.
      toolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      textFieldTopLeft = tester.getTopLeft(find.byType(TextField));
      expect(toolbarTopLeft.dy, greaterThan(textFieldTopLeft.dy));

      // Remove the viewInset, as if the keyboard were hidden.
      tester.view.resetViewInsets();
      await tester.pumpAndSettle();

      // Verify the selection toolbar position is below the text.
      toolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      textFieldTopLeft = tester.getTopLeft(find.byType(TextField));
      expect(toolbarTopLeft.dy, lessThan(textFieldTopLeft.dy));
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Toolbar appears in the right places in multiline inputs',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/36749
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Padding(
              padding: const EdgeInsets.all(30.0),
              child: TextField(controller: controller, minLines: 6, maxLines: 6),
            ),
          ),
        ),
      );

      expect(find.text('Select all'), findsNothing);
      const String testValue = 'abc\ndef\nghi\njkl\nmno\npqr';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Show the selection menu on the first line and verify the selection
      // toolbar position is below the first line.
      await showSelectionMenuAt(tester, controller, testValue.indexOf('c'));
      expect(find.text('Select all'), findsOneWidget);
      final Offset firstLineToolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      final Offset firstLineTopLeft = textOffsetToPosition(tester, testValue.indexOf('a'));
      expect(firstLineTopLeft.dy, lessThan(firstLineToolbarTopLeft.dy));

      // Show the selection menu on the second to last line and verify the
      // selection toolbar position is above that line and above the first
      // line's toolbar.
      await showSelectionMenuAt(tester, controller, testValue.indexOf('o'));
      expect(find.text('Select all'), findsOneWidget);
      final Offset penultimateLineToolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      final Offset penultimateLineTopLeft = textOffsetToPosition(tester, testValue.indexOf('p'));
      expect(penultimateLineToolbarTopLeft.dy, lessThan(penultimateLineTopLeft.dy));
      expect(penultimateLineToolbarTopLeft.dy, lessThan(firstLineToolbarTopLeft.dy));

      // Show the selection menu on the last line and verify the selection
      // toolbar position is above that line and below the position of the
      // second to last line's toolbar.
      await showSelectionMenuAt(tester, controller, testValue.indexOf('r'));
      expect(find.text('Select all'), findsOneWidget);
      final Offset lastLineToolbarTopLeft = tester.getTopLeft(find.text('Select all'));
      final Offset lastLineTopLeft = textOffsetToPosition(tester, testValue.indexOf('p'));
      expect(lastLineToolbarTopLeft.dy, lessThan(lastLineTopLeft.dy));
      expect(lastLineToolbarTopLeft.dy, greaterThan(penultimateLineToolbarTopLeft.dy));
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Selection toolbar fades in',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(overlay(child: TextField(controller: controller)));

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Tap the selection handle to bring up the "paste / select all" menu.
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump();
      // Allow time for the handle to appear and for a double tap to time out.
      await tester.pump(const Duration(milliseconds: 600));
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      await tester.tapAt(endpoints[0].point + const Offset(1.0, 1.0));
      // Pump an extra frame to allow the selection menu to read the clipboard.
      await tester.pump();
      await tester.pump();

      // Toolbar should fade in. Starting at 0% opacity.
      expect(find.text('Select all'), findsOneWidget);
      final Element target = tester.element(find.text('Select all'));
      final FadeTransition opacity = target.findAncestorWidgetOfExactType<FadeTransition>()!;
      expect(opacity.opacity.value, equals(0.0));

      // Still fading in.
      await tester.pump(const Duration(milliseconds: 50));
      final FadeTransition opacity2 = target.findAncestorWidgetOfExactType<FadeTransition>()!;
      expect(opacity, same(opacity2));
      expect(opacity.opacity.value, greaterThan(0.0));
      expect(opacity.opacity.value, lessThan(1.0));

      // End the test here to ensure the animation is properly disposed of.
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('An obscured TextField is selectable by default', (WidgetTester tester) async {
    // This is a regression test for
    // https://github.com/flutter/flutter/issues/32845

    final TextEditingController controller = _textEditingController();
    Widget buildFrame(bool obscureText) {
      return overlay(child: TextField(controller: controller, obscureText: obscureText));
    }

    // Obscure text and don't enable or disable selection.
    await tester.pumpWidget(buildFrame(true));
    await tester.enterText(find.byType(TextField), 'abcdefghi');
    await skipPastScrollingAnimation(tester);
    expect(controller.selection.isCollapsed, true);

    // Long press does select text.
    final Offset ePos = textOffsetToPosition(tester, 1);
    await tester.longPressAt(ePos, pointer: 7);
    await tester.pumpAndSettle();
    expect(controller.selection.isCollapsed, false);
  });

  testWidgets('An obscured TextField is not selectable when disabled', (WidgetTester tester) async {
    // This is a regression test for
    // https://github.com/flutter/flutter/issues/32845

    final TextEditingController controller = _textEditingController();
    Widget buildFrame(bool obscureText, bool enableInteractiveSelection) {
      return overlay(
        child: TextField(
          controller: controller,
          obscureText: obscureText,
          enableInteractiveSelection: enableInteractiveSelection,
        ),
      );
    }

    // Explicitly disabled selection on obscured text.
    await tester.pumpWidget(buildFrame(true, false));
    await tester.enterText(find.byType(TextField), 'abcdefghi');
    await skipPastScrollingAnimation(tester);
    expect(controller.selection.isCollapsed, true);

    // Long press doesn't select text.
    final Offset ePos2 = textOffsetToPosition(tester, 1);
    await tester.longPressAt(ePos2, pointer: 7);
    await tester.pumpAndSettle();
    expect(controller.selection.isCollapsed, true);
  });

  testWidgets('An obscured TextField is not selectable when read-only', (
    WidgetTester tester,
  ) async {
    // This is a regression test for
    // https://github.com/flutter/flutter/issues/32845

    final TextEditingController controller = _textEditingController();
    Widget buildFrame(bool obscureText, bool readOnly) {
      return overlay(
        child: TextField(controller: controller, obscureText: obscureText, readOnly: readOnly),
      );
    }

    // Explicitly disabled selection on obscured text that is read-only.
    await tester.pumpWidget(buildFrame(true, true));
    await tester.enterText(find.byType(TextField), 'abcdefghi');
    await skipPastScrollingAnimation(tester);
    expect(controller.selection.isCollapsed, true);

    // Long press doesn't select text.
    final Offset ePos2 = textOffsetToPosition(tester, 1);
    await tester.longPressAt(ePos2, pointer: 7);
    await tester.pumpAndSettle();
    expect(controller.selection.isCollapsed, true);
  });

  testWidgets('An obscured TextField is selected as one word', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller, obscureText: true)));
    await tester.enterText(find.byType(TextField), 'abcde fghi');
    await skipPastScrollingAnimation(tester);

    // Long press does select text.
    final Offset bPos = textOffsetToPosition(tester, 1);
    await tester.longPressAt(bPos, pointer: 7);
    await tester.pumpAndSettle();
    final TextSelection selection = controller.selection;
    expect(selection.isCollapsed, false);
    expect(selection.baseOffset, 0);
    expect(selection.extentOffset, 10);
  });

  testWidgets(
    'An obscured TextField has correct default context menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();

      await tester.pumpWidget(overlay(child: TextField(controller: controller, obscureText: true)));
      await tester.enterText(find.byType(TextField), 'abcde fghi');
      await skipPastScrollingAnimation(tester);

      // Long press to select text.
      final Offset bPos = textOffsetToPosition(tester, 1);
      await tester.longPressAt(bPos, pointer: 7);
      await tester.pumpAndSettle();

      // Should only have paste option when whole obscure text is selected.
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Select all'), findsNothing);

      // Long press at the end
      final Offset iPos = textOffsetToPosition(tester, 10);
      final Offset slightRight = iPos + const Offset(30.0, 0.0);
      await tester.longPressAt(slightRight, pointer: 7);
      await tester.pumpAndSettle();

      // Should have paste and select all options when collapse.
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Cut'), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'create selection overlay if none exists when toggleToolbar is called',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/111660
      final Widget testWidget = MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            title: const Text('Test'),
            actions: <Widget>[
              PopupMenuButton<String>(
                itemBuilder: (BuildContext context) {
                  return <String>{'About'}.map((String value) {
                    return PopupMenuItem<String>(value: value, child: Text(value));
                  }).toList();
                },
              ),
            ],
          ),
          body: const TextField(),
        ),
      );

      await tester.pumpWidget(testWidget);

      // Tap on TextField.
      final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));
      final TestGesture gesture = await tester.startGesture(textFieldStart);
      await tester.pump(const Duration(milliseconds: 300));
      await gesture.up();
      await tester.pumpAndSettle();

      // Tap on 3 dot menu.
      await tester.tap(find.byType(PopupMenuButton<String>));
      await tester.pumpAndSettle();

      // Tap on TextField.
      await gesture.down(textFieldStart);
      await tester.pump(const Duration(milliseconds: 300));
      await gesture.up();
      await tester.pumpAndSettle();

      // Tap on TextField again.
      await tester.tapAt(textFieldStart);
      await tester.pumpAndSettle();

      expect(tester.takeException(), isNull);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets('TextField height with minLines unset', (WidgetTester tester) async {
    await tester.pumpWidget(textFieldBuilder());

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));

    final RenderBox inputBox = findInputBox();
    final Size emptyInputSize = inputBox.size;

    await tester.enterText(find.byType(TextField), 'No wrapping here.');
    await tester.pumpWidget(textFieldBuilder());
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));

    // Even when entering multiline text, TextField doesn't grow. It's a single
    // line input.
    await tester.enterText(find.byType(TextField), kThreeLines);
    await tester.pumpWidget(textFieldBuilder());
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));

    // maxLines: 3 makes the TextField 3 lines tall
    await tester.enterText(find.byType(TextField), '');
    await tester.pumpWidget(textFieldBuilder(maxLines: 3));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size.height, greaterThan(emptyInputSize.height));
    expect(inputBox.size.width, emptyInputSize.width);

    final Size threeLineInputSize = inputBox.size;

    // Filling with 3 lines of text stays the same size
    await tester.enterText(find.byType(TextField), kThreeLines);
    await tester.pumpWidget(textFieldBuilder(maxLines: 3));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, threeLineInputSize);

    // An extra line won't increase the size because we max at 3.
    await tester.enterText(find.byType(TextField), kMoreThanFourLines);
    await tester.pumpWidget(textFieldBuilder(maxLines: 3));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, threeLineInputSize);

    // But now it will... but it will max at four
    await tester.enterText(find.byType(TextField), kMoreThanFourLines);
    await tester.pumpWidget(textFieldBuilder(maxLines: 4));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size.height, greaterThan(threeLineInputSize.height));
    expect(inputBox.size.width, threeLineInputSize.width);

    final Size fourLineInputSize = inputBox.size;

    // Now it won't max out until the end
    await tester.enterText(find.byType(TextField), '');
    await tester.pumpWidget(textFieldBuilder(maxLines: null));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));
    await tester.enterText(find.byType(TextField), kThreeLines);
    await tester.pump();
    expect(inputBox.size, equals(threeLineInputSize));
    await tester.enterText(find.byType(TextField), kMoreThanFourLines);
    await tester.pump();
    expect(inputBox.size.height, greaterThan(fourLineInputSize.height));
    expect(inputBox.size.width, fourLineInputSize.width);
  });

  testWidgets('TextField height with minLines and maxLines', (WidgetTester tester) async {
    await tester.pumpWidget(textFieldBuilder());

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));

    final RenderBox inputBox = findInputBox();
    final Size emptyInputSize = inputBox.size;

    await tester.enterText(find.byType(TextField), 'No wrapping here.');
    await tester.pumpWidget(textFieldBuilder());
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, equals(emptyInputSize));

    // min and max set to same value locks height to value.
    await tester.pumpWidget(textFieldBuilder(minLines: 3, maxLines: 3));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size.height, greaterThan(emptyInputSize.height));
    expect(inputBox.size.width, emptyInputSize.width);

    final Size threeLineInputSize = inputBox.size;

    // maxLines: null with minLines set grows beyond minLines
    await tester.pumpWidget(textFieldBuilder(minLines: 3, maxLines: null));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, threeLineInputSize);
    await tester.enterText(find.byType(TextField), kMoreThanFourLines);
    await tester.pump();
    expect(inputBox.size.height, greaterThan(threeLineInputSize.height));
    expect(inputBox.size.width, threeLineInputSize.width);

    // With minLines and maxLines set, input will expand through the range
    await tester.enterText(find.byType(TextField), '');
    await tester.pumpWidget(textFieldBuilder(minLines: 3, maxLines: 4));
    expect(findInputBox(), equals(inputBox));
    expect(inputBox.size, equals(threeLineInputSize));
    await tester.enterText(find.byType(TextField), kMoreThanFourLines);
    await tester.pump();
    expect(inputBox.size.height, greaterThan(threeLineInputSize.height));
    expect(inputBox.size.width, threeLineInputSize.width);

    // minLines can't be greater than maxLines.
    expect(() async {
      await tester.pumpWidget(textFieldBuilder(minLines: 3, maxLines: 2));
    }, throwsAssertionError);

    // maxLines defaults to 1 and can't be less than minLines
    expect(() async {
      await tester.pumpWidget(textFieldBuilder(minLines: 3));
    }, throwsAssertionError);
  });

  testWidgets('Multiline text when wrapped in Expanded', (WidgetTester tester) async {
    Widget expandedTextFieldBuilder({int? maxLines = 1, int? minLines, bool expands = false}) {
      return boilerplate(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Expanded(
              child: TextField(
                key: textFieldKey,
                style: const TextStyle(color: Colors.black, fontSize: 34.0),
                maxLines: maxLines,
                minLines: minLines,
                expands: expands,
                decoration: const InputDecoration(hintText: 'Placeholder'),
              ),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(expandedTextFieldBuilder());

    RenderBox findBorder() {
      return tester.renderObject(
        find.descendant(
          of: find.byType(InputDecorator),
          matching: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_BorderContainer'),
        ),
      );
    }

    final RenderBox border = findBorder();

    // Without expanded: true and maxLines: null, the TextField does not expand
    // to fill its parent when wrapped in an Expanded widget.
    final Size unexpandedInputSize = border.size;

    // It does expand to fill its parent when expands: true, maxLines: null, and
    // it's wrapped in an Expanded widget.
    await tester.pumpWidget(expandedTextFieldBuilder(expands: true, maxLines: null));
    expect(border.size.height, greaterThan(unexpandedInputSize.height));
    expect(border.size.width, unexpandedInputSize.width);

    // min/maxLines that is not null and expands: true contradict each other.
    expect(() async {
      await tester.pumpWidget(expandedTextFieldBuilder(expands: true, maxLines: 4));
    }, throwsAssertionError);
    expect(() async {
      await tester.pumpWidget(expandedTextFieldBuilder(expands: true, minLines: 1, maxLines: null));
    }, throwsAssertionError);
  });

  // Regression test for https://github.com/flutter/flutter/pull/29093
  testWidgets('Multiline text when wrapped in IntrinsicHeight', (WidgetTester tester) async {
    final Key intrinsicHeightKey = UniqueKey();
    Widget intrinsicTextFieldBuilder(bool wrapInIntrinsic) {
      final TextFormField textField = TextFormField(
        key: textFieldKey,
        style: const TextStyle(color: Colors.black, fontSize: 34.0),
        maxLines: null,
        decoration: const InputDecoration(counterText: 'I am counter'),
      );
      final Widget widget =
          wrapInIntrinsic ? IntrinsicHeight(key: intrinsicHeightKey, child: textField) : textField;
      return boilerplate(
        child: Column(mainAxisAlignment: MainAxisAlignment.center, children: <Widget>[widget]),
      );
    }

    await tester.pumpWidget(intrinsicTextFieldBuilder(false));
    expect(find.byKey(intrinsicHeightKey), findsNothing);

    RenderBox findEditableText() => tester.renderObject(find.byType(EditableText));
    RenderBox editableText = findEditableText();
    final Size unwrappedEditableTextSize = editableText.size;

    // Wrapping in IntrinsicHeight should not affect the height of the input
    await tester.pumpWidget(intrinsicTextFieldBuilder(true));
    editableText = findEditableText();
    expect(editableText.size.height, unwrappedEditableTextSize.height);
    expect(editableText.size.width, unwrappedEditableTextSize.width);
  });

  // Regression test for https://github.com/flutter/flutter/pull/29093
  testWidgets('errorText empty string', (WidgetTester tester) async {
    Widget textFormFieldBuilder(String? errorText) {
      return boilerplate(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextFormField(
              key: textFieldKey,
              maxLength: 3,
              maxLengthEnforcement: MaxLengthEnforcement.none,
              decoration: InputDecoration(counterText: '', errorText: errorText),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(textFormFieldBuilder(null));

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));
    final RenderBox inputBox = findInputBox();
    final Size errorNullInputSize = inputBox.size;

    // Setting errorText causes the input's height to increase to accommodate it
    await tester.pumpWidget(textFormFieldBuilder('im errorText'));
    expect(inputBox, findInputBox());
    expect(inputBox.size.height, greaterThan(errorNullInputSize.height));
    expect(inputBox.size.width, errorNullInputSize.width);
    final Size errorInputSize = inputBox.size;

    // Setting errorText to an empty string causes the input's height to
    // increase to accommodate it, even though it's not displayed.
    // This may or may not be ideal behavior, but it is legacy behavior and
    // there are visual tests that rely on it (see Github issue referenced at
    // the top of this test). A counterText of empty string does not affect
    // input height, however.
    await tester.pumpWidget(textFormFieldBuilder(''));
    expect(inputBox, findInputBox());
    expect(inputBox.size.height, errorInputSize.height);
    expect(inputBox.size.width, errorNullInputSize.width);
  });

  testWidgets('Growable TextField when content height exceeds parent', (WidgetTester tester) async {
    const double height = 200.0;
    const double padding = 24.0;

    Widget containedTextFieldBuilder({
      Widget? counter,
      String? helperText,
      String? labelText,
      Widget? prefix,
    }) {
      return boilerplate(
        theme: ThemeData(useMaterial3: false),
        child: SizedBox(
          height: height,
          child: TextField(
            key: textFieldKey,
            maxLines: null,
            decoration: InputDecoration(
              counter: counter,
              helperText: helperText,
              labelText: labelText,
              prefix: prefix,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(containedTextFieldBuilder());
    RenderBox findEditableText() => tester.renderObject(find.byType(EditableText));

    final RenderBox inputBox = findEditableText();

    // With no decoration and when overflowing with content, the EditableText
    // takes up the full height minus the padding, so the input fits perfectly
    // inside the parent.
    await tester.enterText(find.byType(TextField), 'a\n' * 11);
    await tester.pump();
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding);

    // Adding a counter causes the EditableText to shrink to fit the counter
    // inside the parent as well.
    const double counterHeight = 40.0;
    const double subtextGap = 8.0;
    const double counterSpace = counterHeight + subtextGap;
    await tester.pumpWidget(containedTextFieldBuilder(counter: Container(height: counterHeight)));
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding - counterSpace);

    // Including helperText causes the EditableText to shrink to fit the text
    // inside the parent as well.
    await tester.pumpWidget(containedTextFieldBuilder(helperText: 'I am helperText'));
    expect(findEditableText(), equals(inputBox));
    const double helperTextSpace = 12.0;
    expect(inputBox.size.height, height - padding - helperTextSpace - subtextGap);

    // When both helperText and counter are present, EditableText shrinks by the
    // height of the taller of the two in order to fit both within the parent.
    await tester.pumpWidget(
      containedTextFieldBuilder(
        counter: Container(height: counterHeight),
        helperText: 'I am helperText',
      ),
    );
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding - counterSpace);

    // When a label is present, EditableText shrinks to fit it at the top so
    // that the bottom of the input still lines up perfectly with the parent.
    await tester.pumpWidget(containedTextFieldBuilder(labelText: 'I am labelText'));
    const double labelSpace = 16.0;
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding - labelSpace);

    // When decoration is present on the top and bottom, EditableText shrinks to
    // fit both inside the parent independently.
    await tester.pumpWidget(
      containedTextFieldBuilder(
        counter: Container(height: counterHeight),
        labelText: 'I am labelText',
      ),
    );
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding - counterSpace - labelSpace);

    // When a prefix or suffix is present in an input that's full of content,
    // it is ignored and allowed to expand beyond the top of the input. Other
    // top and bottom decoration is still respected.
    await tester.pumpWidget(
      containedTextFieldBuilder(
        counter: Container(height: counterHeight),
        labelText: 'I am labelText',
        prefix: const SizedBox(width: 10, height: 60),
      ),
    );
    expect(findEditableText(), equals(inputBox));
    expect(inputBox.size.height, height - padding - labelSpace - counterSpace);
  });

  testWidgets('Multiline hint text will wrap up to maxLines', (WidgetTester tester) async {
    final Key textFieldKey = UniqueKey();

    Widget builder(int? maxLines, final String hintMsg) {
      return boilerplate(
        child: TextField(
          key: textFieldKey,
          style: const TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: maxLines,
          decoration: InputDecoration(hintText: hintMsg),
        ),
      );
    }

    const String hintPlaceholder = 'Placeholder';
    const String multipleLineText =
        "Here's a text, which is more than one line, to demonstrate the multiple line hint text";
    await tester.pumpWidget(builder(null, hintPlaceholder));

    RenderBox findHintText(String hint) => tester.renderObject(find.text(hint));

    final RenderBox hintTextBox = findHintText(hintPlaceholder);
    final Size oneLineHintSize = hintTextBox.size;

    await tester.pumpWidget(builder(null, hintPlaceholder));
    expect(findHintText(hintPlaceholder), equals(hintTextBox));
    expect(hintTextBox.size, equals(oneLineHintSize));

    const int maxLines = 3;
    await tester.pumpWidget(builder(maxLines, multipleLineText));
    final Text hintTextWidget = tester.widget(find.text(multipleLineText));
    expect(hintTextWidget.maxLines, equals(maxLines));
    expect(findHintText(multipleLineText).size.width, greaterThanOrEqualTo(oneLineHintSize.width));
    expect(
      findHintText(multipleLineText).size.height,
      greaterThanOrEqualTo(oneLineHintSize.height),
    );
  });

  testWidgets('Can drag handles to change selection in multiline', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: TextField(
            dragStartBehavior: DragStartBehavior.down,
            controller: controller,
            style: const TextStyle(color: Colors.black, fontSize: 34.0),
            maxLines: 3,
          ),
        ),
      ),
    );

    const String testValue = kThreeLines;
    const String cutValue = 'First line of stuff';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    // Check that the text spans multiple lines.
    final Offset firstPos = textOffsetToPosition(tester, testValue.indexOf('First'));
    final Offset secondPos = textOffsetToPosition(tester, testValue.indexOf('Second'));
    final Offset thirdPos = textOffsetToPosition(tester, testValue.indexOf('Third'));
    final Offset middleStringPos = textOffsetToPosition(tester, testValue.indexOf('irst'));
    expect(firstPos.dx, lessThan(middleStringPos.dx));
    expect(firstPos.dx, secondPos.dx);
    expect(firstPos.dx, thirdPos.dx);
    expect(firstPos.dy, lessThan(secondPos.dy));
    expect(secondPos.dy, lessThan(thirdPos.dy));

    // Long press the 'n' in 'until' to select the word.
    final Offset untilPos = textOffsetToPosition(tester, testValue.indexOf('until') + 1);
    TestGesture gesture = await tester.startGesture(untilPos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    expect(controller.selection, const TextSelection(baseOffset: 39, extentOffset: 44));

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle to the third line, just after 'Third'.
    Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    // The distance below the y value returned by textOffsetToPosition required
    // to register a full vertical line drag.
    const Offset downLineOffset = Offset(0.0, 3.0);
    Offset newHandlePos =
        textOffsetToPosition(tester, testValue.indexOf('Third') + 5) + downLineOffset;
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection, const TextSelection(baseOffset: 39, extentOffset: 50));

    // Drag the left handle to the first line, just after 'First'.
    handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
    newHandlePos = textOffsetToPosition(tester, testValue.indexOf('First') + 5);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 50);

    if (!isContextMenuProvidedByPlatform) {
      await tester.tap(find.text('Cut'));
      await tester.pump();
      expect(controller.selection.isCollapsed, true);
      expect(controller.text, cutValue);
    }
  });

  testWidgets('Can scroll multiline input', (WidgetTester tester) async {
    final Key textFieldKey = UniqueKey();
    final TextEditingController controller = _textEditingController(text: kMoreThanFourLines);

    await tester.pumpWidget(
      overlay(
        child: TextField(
          dragStartBehavior: DragStartBehavior.down,
          key: textFieldKey,
          controller: controller,
          style: const TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 2,
        ),
      ),
    );

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));
    final RenderBox inputBox = findInputBox();

    // Check that the last line of text is not displayed.
    final Offset firstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    final Offset fourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));
    expect(firstPos.dx, fourthPos.dx);
    expect(firstPos.dy, lessThan(fourthPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(firstPos)),
      isTrue,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(fourthPos)),
      isFalse,
    );

    TestGesture gesture = await tester.startGesture(firstPos, pointer: 7);
    await tester.pump();
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pump(const Duration(seconds: 1));
    // Wait and drag again to trigger https://github.com/flutter/flutter/issues/6329
    // (No idea why this is necessary, but the bug wouldn't repro without it.)
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Now the first line is scrolled up, and the fourth line is visible.
    Offset newFirstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    Offset newFourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));

    expect(newFirstPos.dy, lessThan(firstPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFirstPos)),
      isFalse,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFourthPos)),
      isTrue,
    );

    // Now try scrolling by dragging the selection handle.
    // Long press the middle of the word "won't" in the fourth line.
    final Offset selectedWordPos = textOffsetToPosition(
      tester,
      kMoreThanFourLines.indexOf('Fourth line') + 14,
    );

    gesture = await tester.startGesture(selectedWordPos, pointer: 7);
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(controller.selection.base.offset, 77);
    expect(controller.selection.extent.offset, 82);
    // Sanity check for the word selected is the intended one.
    expect(
      controller.text.substring(controller.selection.baseOffset, controller.selection.extentOffset),
      "won't",
    );

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the left handle to the first line, just after 'First'.
    final Offset handlePos = endpoints[0].point + const Offset(-1, 1);
    final Offset newHandlePos = textOffsetToPosition(
      tester,
      kMoreThanFourLines.indexOf('First') + 5,
    );
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump(const Duration(seconds: 1));
    await gesture.moveTo(newHandlePos + const Offset(0.0, -10.0));
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump(const Duration(seconds: 1));

    // The text should have scrolled up with the handle to keep the active
    // cursor visible, back to its original position.
    newFirstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    newFourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));
    expect(newFirstPos.dy, firstPos.dy);
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFirstPos)),
      isTrue,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFourthPos)),
      isFalse,
    );
  });

  testWidgets('TextField smoke test', (WidgetTester tester) async {
    late String textFieldValue;

    await tester.pumpWidget(
      overlay(
        child: TextField(
          decoration: null,
          onChanged: (String value) {
            textFieldValue = value;
          },
        ),
      ),
    );

    Future<void> checkText(String testValue) {
      return TestAsyncUtils.guard(() async {
        await tester.enterText(find.byType(TextField), testValue);

        // Check that the onChanged event handler fired.
        expect(textFieldValue, equals(testValue));

        await tester.pump();
      });
    }

    await checkText('Hello World');
  });

  testWidgets('TextField with global key', (WidgetTester tester) async {
    final GlobalKey textFieldKey = GlobalKey(debugLabel: 'textFieldKey');
    late String textFieldValue;

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: textFieldKey,
          decoration: const InputDecoration(hintText: 'Placeholder'),
          onChanged: (String value) {
            textFieldValue = value;
          },
        ),
      ),
    );

    Future<void> checkText(String testValue) async {
      return TestAsyncUtils.guard(() async {
        await tester.enterText(find.byType(TextField), testValue);

        // Check that the onChanged event handler fired.
        expect(textFieldValue, equals(testValue));

        await tester.pump();
      });
    }

    await checkText('Hello World');
  });

  testWidgets('TextField errorText trumps helperText', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: const TextField(
            decoration: InputDecoration(errorText: 'error text', helperText: 'helper text'),
          ),
        ),
      ),
    );
    expect(find.text('helper text'), findsNothing);
    expect(find.text('error text'), findsOneWidget);
  });

  testWidgets('TextField with default helperStyle', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData(hintColor: Colors.blue[500], useMaterial3: false);
    await tester.pumpWidget(
      overlay(
        child: Theme(
          data: themeData,
          child: const TextField(decoration: InputDecoration(helperText: 'helper text')),
        ),
      ),
    );
    final Text helperText = tester.widget(find.text('helper text'));
    expect(helperText.style!.color, themeData.hintColor);
    expect(helperText.style!.fontSize, Typography.englishLike2014.bodySmall!.fontSize);
  });

  testWidgets('TextField with specified helperStyle', (WidgetTester tester) async {
    final TextStyle style = TextStyle(inherit: false, color: Colors.pink[500], fontSize: 10.0);

    await tester.pumpWidget(
      overlay(
        child: TextField(
          decoration: InputDecoration(helperText: 'helper text', helperStyle: style),
        ),
      ),
    );
    final Text helperText = tester.widget(find.text('helper text'));
    expect(helperText.style, style);
  });

  testWidgets('TextField with default hintStyle', (WidgetTester tester) async {
    final TextStyle style = TextStyle(color: Colors.pink[500], fontSize: 10.0);
    final ThemeData themeData = ThemeData();

    await tester.pumpWidget(
      overlay(
        child: Theme(
          data: themeData,
          child: TextField(
            decoration: const InputDecoration(hintText: 'Placeholder'),
            style: style,
          ),
        ),
      ),
    );

    final Text hintText = tester.widget(find.text('Placeholder'));
    expect(hintText.style!.color, themeData.colorScheme.onSurfaceVariant);
    expect(hintText.style!.fontSize, style.fontSize);
  });

  testWidgets('Material2 - TextField with default hintStyle', (WidgetTester tester) async {
    final TextStyle style = TextStyle(color: Colors.pink[500], fontSize: 10.0);
    final ThemeData themeData = ThemeData(useMaterial3: false, hintColor: Colors.blue[500]);

    await tester.pumpWidget(
      overlay(
        child: Theme(
          data: themeData,
          child: TextField(
            decoration: const InputDecoration(hintText: 'Placeholder'),
            style: style,
          ),
        ),
      ),
    );

    final Text hintText = tester.widget(find.text('Placeholder'));
    expect(hintText.style!.color, themeData.hintColor);
    expect(hintText.style!.fontSize, style.fontSize);
  });

  testWidgets('TextField with specified hintStyle', (WidgetTester tester) async {
    final TextStyle hintStyle = TextStyle(inherit: false, color: Colors.pink[500], fontSize: 10.0);

    await tester.pumpWidget(
      overlay(
        child: TextField(
          decoration: InputDecoration(hintText: 'Placeholder', hintStyle: hintStyle),
        ),
      ),
    );

    final Text hintText = tester.widget(find.text('Placeholder'));
    expect(hintText.style, hintStyle);
  });

  testWidgets('TextField with specified prefixStyle', (WidgetTester tester) async {
    final TextStyle prefixStyle = TextStyle(
      inherit: false,
      color: Colors.pink[500],
      fontSize: 10.0,
    );

    await tester.pumpWidget(
      overlay(
        child: TextField(
          decoration: InputDecoration(prefixText: 'Prefix:', prefixStyle: prefixStyle),
        ),
      ),
    );

    final Text prefixText = tester.widget(find.text('Prefix:'));
    expect(prefixText.style, prefixStyle);
  });

  testWidgets('TextField prefix and suffix create a sibling node', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      overlay(
        child: TextField(
          controller: _textEditingController(text: 'some text'),
          decoration: const InputDecoration(prefixText: 'Prefix', suffixText: 'Suffix'),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 2, textDirection: TextDirection.ltr, label: 'Prefix'),
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              value: 'some text',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
            ),
            TestSemantics.rootChild(id: 3, textDirection: TextDirection.ltr, label: 'Suffix'),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('TextField with specified suffixStyle', (WidgetTester tester) async {
    final TextStyle suffixStyle = TextStyle(color: Colors.pink[500], fontSize: 10.0);

    await tester.pumpWidget(
      overlay(
        child: TextField(decoration: InputDecoration(suffixText: '.com', suffixStyle: suffixStyle)),
      ),
    );

    final Text suffixText = tester.widget(find.text('.com'));
    expect(suffixText.style, suffixStyle);
  });

  testWidgets('TextField prefix and suffix appear correctly with no hint or label', (
    WidgetTester tester,
  ) async {
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Column(
          children: <Widget>[
            const TextField(decoration: InputDecoration(labelText: 'First')),
            TextField(
              key: secondKey,
              decoration: const InputDecoration(prefixText: 'Prefix', suffixText: 'Suffix'),
            ),
          ],
        ),
      ),
    );

    expect(find.text('Prefix'), findsOneWidget);
    expect(find.text('Suffix'), findsOneWidget);

    // Focus the Input. The prefix should still display.
    await tester.tap(find.byKey(secondKey));
    await tester.pump();

    expect(find.text('Prefix'), findsOneWidget);
    expect(find.text('Suffix'), findsOneWidget);

    // Enter some text, and the prefix should still display.
    await tester.enterText(find.byKey(secondKey), 'Hi');
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Prefix'), findsOneWidget);
    expect(find.text('Suffix'), findsOneWidget);
  });

  testWidgets('TextField prefix and suffix appear correctly with hint text', (
    WidgetTester tester,
  ) async {
    final TextStyle hintStyle = TextStyle(inherit: false, color: Colors.pink[500], fontSize: 10.0);
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Column(
          children: <Widget>[
            const TextField(decoration: InputDecoration(labelText: 'First')),
            TextField(
              key: secondKey,
              decoration: InputDecoration(
                hintText: 'Hint',
                hintStyle: hintStyle,
                prefixText: 'Prefix',
                suffixText: 'Suffix',
              ),
            ),
          ],
        ),
      ),
    );

    // Neither the prefix or the suffix should initially be visible, only the hint.
    expect(getOpacity(tester, find.text('Prefix')), 0.0);
    expect(getOpacity(tester, find.text('Suffix')), 0.0);
    expect(getOpacity(tester, find.text('Hint')), 1.0);

    await tester.tap(find.byKey(secondKey));
    await tester.pumpAndSettle();

    // Focus the Input. The hint, prefix, and suffix should appear
    expect(getOpacity(tester, find.text('Prefix')), 1.0);
    expect(getOpacity(tester, find.text('Suffix')), 1.0);
    expect(getOpacity(tester, find.text('Hint')), 1.0);

    // Enter some text, and the hint should disappear and the prefix and suffix
    // should continue to be visible
    await tester.enterText(find.byKey(secondKey), 'Hi');
    await tester.pumpAndSettle();

    expect(getOpacity(tester, find.text('Prefix')), 1.0);
    expect(getOpacity(tester, find.text('Suffix')), 1.0);
    expect(getOpacity(tester, find.text('Hint')), 0.0);

    // Check and make sure that the right styles were applied.
    final Text prefixText = tester.widget(find.text('Prefix'));
    expect(prefixText.style, hintStyle);
    final Text suffixText = tester.widget(find.text('Suffix'));
    expect(suffixText.style, hintStyle);
  });

  testWidgets('TextField prefix and suffix appear correctly with label text', (
    WidgetTester tester,
  ) async {
    final TextStyle prefixStyle = TextStyle(color: Colors.pink[500], fontSize: 10.0);
    final TextStyle suffixStyle = TextStyle(color: Colors.green[500], fontSize: 12.0);
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Column(
          children: <Widget>[
            const TextField(decoration: InputDecoration(labelText: 'First')),
            TextField(
              key: secondKey,
              decoration: InputDecoration(
                labelText: 'Label',
                prefixText: 'Prefix',
                prefixStyle: prefixStyle,
                suffixText: 'Suffix',
                suffixStyle: suffixStyle,
              ),
            ),
          ],
        ),
      ),
    );

    // Not focused. The prefix and suffix should not appear, but the label should.
    expect(getOpacity(tester, find.text('Prefix')), 0.0);
    expect(getOpacity(tester, find.text('Suffix')), 0.0);
    expect(find.text('Label'), findsOneWidget);

    // Focus the input. The label, prefix, and suffix should appear.
    await tester.tap(find.byKey(secondKey));
    await tester.pumpAndSettle();

    expect(getOpacity(tester, find.text('Prefix')), 1.0);
    expect(getOpacity(tester, find.text('Suffix')), 1.0);
    expect(find.text('Label'), findsOneWidget);

    // Enter some text. The label, prefix, and suffix should remain visible.
    await tester.enterText(find.byKey(secondKey), 'Hi');
    await tester.pumpAndSettle();

    expect(getOpacity(tester, find.text('Prefix')), 1.0);
    expect(getOpacity(tester, find.text('Suffix')), 1.0);
    expect(find.text('Label'), findsOneWidget);

    // Check and make sure that the right styles were applied.
    final Text prefixText = tester.widget(find.text('Prefix'));
    expect(prefixText.style, prefixStyle);
    final Text suffixText = tester.widget(find.text('Suffix'));
    expect(suffixText.style, suffixStyle);
  });

  testWidgets('TextField label text animates', (WidgetTester tester) async {
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Column(
          children: <Widget>[
            const TextField(decoration: InputDecoration(labelText: 'First')),
            TextField(key: secondKey, decoration: const InputDecoration(labelText: 'Second')),
          ],
        ),
      ),
    );

    Offset pos = tester.getTopLeft(find.text('Second'));

    // Focus the Input. The label should start animating upwards.
    await tester.tap(find.byKey(secondKey));
    await tester.idle();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));

    Offset newPos = tester.getTopLeft(find.text('Second'));
    expect(newPos.dy, lessThan(pos.dy));

    // Label should still be sliding upward.
    await tester.pump(const Duration(milliseconds: 50));
    pos = newPos;
    newPos = tester.getTopLeft(find.text('Second'));
    expect(newPos.dy, lessThan(pos.dy));
  });

  testWidgets('Icon is separated from input/label by 16+12', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const TextField(
          decoration: InputDecoration(icon: Icon(Icons.phone), labelText: 'label', filled: true),
        ),
      ),
    );
    final double iconRight = tester.getTopRight(find.byType(Icon)).dx;
    // Per https://material.io/go/design-text-fields#text-fields-layout
    // There's a 16 dps gap between the right edge of the icon and the text field's
    // container, and the 12dps more padding between the left edge of the container
    // and the left edge of the input and label.
    expect(iconRight + 28.0, equals(tester.getTopLeft(find.text('label')).dx));
    expect(iconRight + 28.0, equals(tester.getTopLeft(find.byType(EditableText)).dx));
  });

  testWidgets('Collapsed hint text placement', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: const TextField(
            decoration: InputDecoration.collapsed(hintText: 'hint'),
            strutStyle: StrutStyle.disabled,
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.text('hint')),
      equals(tester.getTopLeft(find.byType(EditableText))),
    );
  });

  testWidgets('Can align to center', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: TextField(textAlign: TextAlign.center, decoration: null),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);
    assert(editable.size.width == 300);
    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 0)).topLeft,
    );

    // The overlay() function centers its child within a 800x600 view.
    // Default cursorWidth is 2.0, test viewWidth is 800
    // Centered cursor topLeft.dx: 399 == viewWidth/2 - cursorWidth/2
    expect(topLeft.dx, equals(399.0));

    await tester.enterText(find.byType(TextField), 'abcd');
    await tester.pump();

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );

    // TextPosition(offset: 2) - center of 'abcd'
    expect(topLeft.dx, equals(399.0));
  });

  testWidgets('Can align to center within center', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: Center(child: TextField(textAlign: TextAlign.center, decoration: null)),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);
    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 0)).topLeft,
    );

    // The overlay() function centers its child within a 800x600 view.
    // Default cursorWidth is 2.0, test viewWidth is 800
    // Centered cursor topLeft.dx: 399 == viewWidth/2 - cursorWidth/2
    expect(topLeft.dx, equals(399.0));

    await tester.enterText(find.byType(TextField), 'abcd');
    await tester.pump();

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );

    // TextPosition(offset: 2) - center of 'abcd'
    expect(topLeft.dx, equals(399.0));
  });

  testWidgets('Controller can update server', (WidgetTester tester) async {
    final TextEditingController controller1 = _textEditingController(text: 'Initial Text');
    final TextEditingController controller2 = _textEditingController(text: 'More Text');

    TextEditingController? currentController;
    late StateSetter setState;

    await tester.pumpWidget(
      overlay(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            setState = setter;
            return TextField(controller: currentController);
          },
        ),
      ),
    );
    expect(tester.testTextInput.editingState, isNull);

    // Initial state with null controller.
    await tester.tap(find.byType(TextField));
    await tester.pump();
    expect(tester.testTextInput.editingState!['text'], isEmpty);

    // Update the controller from null to controller1.
    setState(() {
      currentController = controller1;
    });
    await tester.pump();
    expect(tester.testTextInput.editingState!['text'], equals('Initial Text'));

    // Verify that updates to controller1 are handled.
    controller1.text = 'Updated Text';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('Updated Text'));

    // Verify that switching from controller1 to controller2 is handled.
    setState(() {
      currentController = controller2;
    });
    await tester.pump();
    expect(tester.testTextInput.editingState!['text'], equals('More Text'));

    // Verify that updates to controller1 are ignored.
    controller1.text = 'Ignored Text';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('More Text'));

    // Verify that updates to controller text are handled.
    controller2.text = 'Additional Text';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('Additional Text'));

    // Verify that updates to controller selection are handled.
    controller2.selection = const TextSelection(baseOffset: 0, extentOffset: 5);
    await tester.idle();
    expect(tester.testTextInput.editingState!['selectionBase'], equals(0));
    expect(tester.testTextInput.editingState!['selectionExtent'], equals(5));

    // Verify that calling clear() clears the text.
    controller2.clear();
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals(''));

    // Verify that switching from controller2 to null preserves current text.
    controller2.text = 'The Final Cut';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('The Final Cut'));
    setState(() {
      currentController = null;
    });
    await tester.pump();
    expect(tester.testTextInput.editingState!['text'], equals('The Final Cut'));

    // Verify that changes to controller2 are ignored.
    controller2.text = 'Goodbye Cruel World';
    expect(tester.testTextInput.editingState!['text'], equals('The Final Cut'));
  });

  testWidgets('Cannot enter new lines onto single line TextField', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, decoration: null)),
    );

    await tester.enterText(find.byType(TextField), 'abc\ndef');

    expect(textController.text, 'abcdef');
  });

  testWidgets('Injected formatters are chained', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          decoration: null,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '#'),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a一b二c三\nd四e五f六');
    // The default single line formatter replaces \n with empty string.
    expect(textController.text, '#一#二#三#四#五#六');
  });

  testWidgets('Injected formatters are chained (deprecated names)', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          decoration: null,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '#'),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a一b二c三\nd四e五f六');
    // The default single line formatter replaces \n with empty string.
    expect(textController.text, '#一#二#三#四#五#六');
  });

  testWidgets('Chained formatters are in sequence', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          decoration: null,
          maxLines: 2,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '12\n'),
            FilteringTextInputFormatter.allow(RegExp(r'\n[0-9]')),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a1b2c3');
    // The first formatter turns it into
    // 12\n112\n212\n3
    // The second formatter turns it into
    // \n1\n2\n3
    // Multiline is allowed since maxLine != 1.
    expect(textController.text, '\n1\n2\n3');
  });

  testWidgets('Chained formatters are in sequence (deprecated names)', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          decoration: null,
          maxLines: 2,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '12\n'),
            FilteringTextInputFormatter.allow(RegExp(r'\n[0-9]')),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a1b2c3');
    // The first formatter turns it into
    // 12\n112\n212\n3
    // The second formatter turns it into
    // \n1\n2\n3
    // Multiline is allowed since maxLine != 1.
    expect(textController.text, '\n1\n2\n3');
  });

  testWidgets(
    'Pasted values are formatted',
    (WidgetTester tester) async {
      final TextEditingController textController = _textEditingController();

      await tester.pumpWidget(
        overlay(
          child: TextField(
            controller: textController,
            decoration: null,
            inputFormatters: <TextInputFormatter>[FilteringTextInputFormatter.digitsOnly],
          ),
        ),
      );

      await tester.enterText(find.byType(TextField), 'a1b\n2c3');
      expect(textController.text, '123');
      await skipPastScrollingAnimation(tester);

      await tester.tapAt(textOffsetToPosition(tester, '123'.indexOf('2')));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(textController.selection),
        renderEditable,
      );
      await tester.tapAt(endpoints[0].point + const Offset(1.0, 1.0));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      Clipboard.setData(const ClipboardData(text: '一4二\n5三6'));
      await tester.tap(find.text('Paste'));
      await tester.pump();
      // Puts 456 before the 2 in 123.
      expect(textController.text, '145623');
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Pasted values are formatted (deprecated names)',
    (WidgetTester tester) async {
      final TextEditingController textController = _textEditingController();

      await tester.pumpWidget(
        overlay(
          child: TextField(
            controller: textController,
            decoration: null,
            inputFormatters: <TextInputFormatter>[FilteringTextInputFormatter.digitsOnly],
          ),
        ),
      );

      await tester.enterText(find.byType(TextField), 'a1b\n2c3');
      expect(textController.text, '123');
      await skipPastScrollingAnimation(tester);

      await tester.tapAt(textOffsetToPosition(tester, '123'.indexOf('2')));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(textController.selection),
        renderEditable,
      );
      await tester.tapAt(endpoints[0].point + const Offset(1.0, 1.0));
      await tester.pump();
      await tester.pump(
        const Duration(milliseconds: 200),
      ); // skip past the frame where the opacity is zero

      Clipboard.setData(const ClipboardData(text: '一4二\n5三6'));
      await tester.tap(find.text('Paste'));
      await tester.pump();
      // Puts 456 before the 2 in 123.
      expect(textController.text, '145623');
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('Do not add LengthLimiting formatter to the user supplied list', (
    WidgetTester tester,
  ) async {
    final List<TextInputFormatter> formatters = <TextInputFormatter>[];

    await tester.pumpWidget(
      overlay(child: TextField(decoration: null, maxLength: 5, inputFormatters: formatters)),
    );

    expect(formatters.isEmpty, isTrue);
  });

  testWidgets('Text field scrolls the caret into view', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(child: SizedBox(width: 100.0, child: TextField(controller: controller))),
      ),
    );

    final String longText = 'a' * 20;
    await tester.enterText(find.byType(TextField), longText);
    await skipPastScrollingAnimation(tester);

    ScrollableState scrollableState = tester.firstState(find.byType(Scrollable));
    expect(scrollableState.position.pixels, equals(0.0));

    // Move the caret to the end of the text and check that the text field
    // scrolls to make the caret visible.
    scrollableState = tester.firstState(find.byType(Scrollable));
    final EditableTextState editableTextState = tester.firstState(find.byType(EditableText));
    editableTextState.userUpdateTextEditingValue(
      editableTextState.textEditingValue.copyWith(
        selection: TextSelection.collapsed(offset: longText.length),
      ),
      null,
    );

    await tester.pump(); // TODO(ianh): Figure out why this extra pump is needed.
    await skipPastScrollingAnimation(tester);

    scrollableState = tester.firstState(find.byType(Scrollable));
    // For a horizontal input, scrolls to the exact position of the caret.
    expect(scrollableState.position.pixels, equals(222.0));
  });

  testWidgets('Multiline text field scrolls the caret into view', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller, maxLines: 6)));

    const String tallText = 'a\nb\nc\nd\ne\nf\ng'; // One line over max
    await tester.enterText(find.byType(TextField), tallText);
    await skipPastScrollingAnimation(tester);

    ScrollableState scrollableState = tester.firstState(find.byType(Scrollable));
    expect(scrollableState.position.pixels, equals(0.0));

    // Move the caret to the end of the text and check that the text field
    // scrolls to make the caret visible.
    final EditableTextState editableTextState = tester.firstState(find.byType(EditableText));
    editableTextState.userUpdateTextEditingValue(
      editableTextState.textEditingValue.copyWith(
        selection: const TextSelection.collapsed(offset: tallText.length),
      ),
      null,
    );
    await tester.pump();
    await skipPastScrollingAnimation(tester);

    // Should have scrolled down exactly one line height (7 lines of text in 6
    // line text field).
    final double lineHeight = findRenderEditable(tester).preferredLineHeight;
    scrollableState = tester.firstState(find.byType(Scrollable));
    expect(scrollableState.position.pixels, moreOrLessEquals(lineHeight, epsilon: 0.1));
  });

  testWidgets('haptic feedback', (WidgetTester tester) async {
    final FeedbackTester feedback = FeedbackTester();
    addTearDown(feedback.dispose);
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      overlay(child: SizedBox(width: 100.0, child: TextField(controller: controller))),
    );

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 0);
    expect(feedback.hapticCount, 0);

    await tester.longPress(find.byType(TextField));
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 0);
    expect(feedback.hapticCount, 1);
  });

  testWidgets('Text field drops selection color when losing focus', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/103341.
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();
    final TextEditingController controller1 = _textEditingController();
    const Color selectionColor = Colors.orange;
    const Color cursorColor = Colors.red;

    await tester.pumpWidget(
      overlay(
        child: DefaultSelectionStyle(
          selectionColor: selectionColor,
          cursorColor: cursorColor,
          child: Column(
            children: <Widget>[TextField(key: key1, controller: controller1), TextField(key: key2)],
          ),
        ),
      ),
    );

    const TextSelection selection = TextSelection(baseOffset: 0, extentOffset: 4);
    final EditableTextState state1 = tester.state<EditableTextState>(
      find.byType(EditableText).first,
    );
    final EditableTextState state2 = tester.state<EditableTextState>(
      find.byType(EditableText).last,
    );

    await tester.tap(find.byKey(key1));
    await tester.enterText(find.byKey(key1), 'abcd');
    await tester.pump();

    await tester.tap(find.byKey(key2));
    await tester.enterText(find.byKey(key2), 'dcba');
    await tester.pump();

    // Focus and selection is active on first TextField, so the second TextFields
    // selectionColor should be dropped.
    await tester.tap(find.byKey(key1));
    controller1.selection = const TextSelection(baseOffset: 0, extentOffset: 4);
    await tester.pump();
    expect(controller1.selection, selection);
    expect(state1.widget.selectionColor, selectionColor);
    expect(state2.widget.selectionColor, null);

    // Focus and selection is active on second TextField, so the first TextFields
    // selectionColor should be dropped.
    await tester.tap(find.byKey(key2));
    await tester.pump();
    expect(state1.widget.selectionColor, null);
    expect(state2.widget.selectionColor, selectionColor);
  });

  testWidgets('Selection is consistent with text length', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    controller.text = 'abcde';
    controller.selection = const TextSelection.collapsed(offset: 5);

    controller.text = '';
    expect(controller.selection.start, lessThanOrEqualTo(0));
    expect(controller.selection.end, lessThanOrEqualTo(0));

    late FlutterError error;
    try {
      controller.selection = const TextSelection.collapsed(offset: 10);
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error.diagnostics.length, 1);
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   invalid text selection: TextSelection.collapsed(offset: 10,\n'
          '   affinity: TextAffinity.downstream, isDirectional: false)\n',
        ),
      );
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/35848
  testWidgets('Clearing text field with suffixIcon does not cause text selection exception', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(text: 'Prefilled text.');

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: controller,
          decoration: InputDecoration(
            suffixIcon: IconButton(icon: const Icon(Icons.close), onPressed: controller.clear),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(IconButton));
    expect(controller.text, '');
  });

  testWidgets('maxLength limits input.', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    await tester.enterText(find.byType(TextField), '0123456789101112');
    expect(textController.text, '0123456789');
  });

  testWidgets('maxLength limits input with surrogate pairs.', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    const String surrogatePair = '😆';
    await tester.enterText(find.byType(TextField), '${surrogatePair}0123456789101112');
    expect(textController.text, '${surrogatePair}012345678');
  });

  testWidgets('maxLength limits input with grapheme clusters.', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    const String graphemeCluster = '👨‍👩‍👦';
    await tester.enterText(find.byType(TextField), '${graphemeCluster}0123456789101112');
    expect(textController.text, '${graphemeCluster}012345678');
  });

  testWidgets('maxLength limits input in the center of a maxed-out field.', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/37420.
    final TextEditingController textController = _textEditingController();
    const String testValue = '0123456789';

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    // Max out the character limit in the field.
    await tester.enterText(find.byType(TextField), testValue);
    expect(textController.text, testValue);

    // Entering more characters at the end does nothing.
    await tester.enterText(find.byType(TextField), '${testValue}9999999');
    expect(textController.text, testValue);

    // Entering text in the middle of the field also does nothing.
    await tester.enterText(find.byType(TextField), '0123455555555556789');
    expect(textController.text, testValue);
  });

  testWidgets(
    'maxLength limits input in the center of a maxed-out field, with collapsed selection',
    (WidgetTester tester) async {
      final TextEditingController textController = _textEditingController();
      const String testValue = '0123456789';

      await tester.pumpWidget(
        boilerplate(child: TextField(controller: textController, maxLength: 10)),
      );

      // Max out the character limit in the field.
      await tester.showKeyboard(find.byType(TextField));
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(text: testValue, selection: TextSelection.collapsed(offset: 10)),
      );
      await tester.pump();
      expect(textController.text, testValue);

      // Entering more characters at the end does nothing.
      await tester.showKeyboard(find.byType(TextField));
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: '${testValue}9999999',
          selection: TextSelection.collapsed(offset: 10 + 7),
        ),
      );
      await tester.pump();

      expect(textController.text, testValue);

      // Entering text in the middle of the field also does nothing.
      // Entering more characters at the end does nothing.
      await tester.showKeyboard(find.byType(TextField));
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: '0123455555555556789',
          selection: TextSelection.collapsed(offset: 19),
        ),
      );
      await tester.pump();

      expect(textController.text, testValue);
    },
  );

  testWidgets(
    'maxLength limits input in the center of a maxed-out field, with non-collapsed selection',
    (WidgetTester tester) async {
      final TextEditingController textController = _textEditingController();
      const String testValue = '0123456789';

      await tester.pumpWidget(
        boilerplate(
          child: TextField(
            controller: textController,
            maxLength: 10,
            maxLengthEnforcement: MaxLengthEnforcement.enforced,
          ),
        ),
      );

      // Max out the character limit in the field.
      await tester.showKeyboard(find.byType(TextField));
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testValue,
          selection: TextSelection(baseOffset: 8, extentOffset: 10),
        ),
      );
      await tester.pump();
      expect(textController.text, testValue);

      // Entering more characters at the end does nothing.
      await tester.showKeyboard(find.byType(TextField));
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: '01234569999999',
          selection: TextSelection.collapsed(offset: 14),
        ),
      );
      await tester.pump();

      expect(textController.text, '0123456999');
    },
  );

  testWidgets('maxLength limits input length even if decoration is null.', (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, decoration: null, maxLength: 10)),
    );

    await tester.enterText(find.byType(TextField), '0123456789101112');
    expect(textController.text, '0123456789');
  });

  testWidgets('maxLength still works with other formatters', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          maxLength: 10,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '#'),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a一b二c三\nd四e五f六');
    // The default single line formatter replaces \n with empty string.
    expect(textController.text, '#一#二#三#四#五');
  });

  testWidgets('maxLength still works with other formatters (deprecated names)', (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          maxLength: 10,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.deny(RegExp(r'[a-z]'), replacementString: '#'),
          ],
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'a一b二c三\nd四e五f六');
    // The default single line formatter replaces \n with empty string.
    expect(textController.text, '#一#二#三#四#五');
  });

  testWidgets("maxLength isn't enforced when maxLengthEnforcement.none.", (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          controller: textController,
          maxLength: 10,
          maxLengthEnforcement: MaxLengthEnforcement.none,
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), '0123456789101112');
    expect(textController.text, '0123456789101112');
  });

  testWidgets('maxLength shows warning when maxLengthEnforcement.none.', (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();
    const TextStyle testStyle = TextStyle(color: Colors.deepPurpleAccent);

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          decoration: const InputDecoration(errorStyle: testStyle),
          controller: textController,
          maxLength: 10,
          maxLengthEnforcement: MaxLengthEnforcement.none,
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), '0123456789101112');
    await tester.pump();

    expect(textController.text, '0123456789101112');
    expect(find.text('16/10'), findsOneWidget);
    Text counterTextWidget = tester.widget(find.text('16/10'));
    expect(counterTextWidget.style!.color, equals(Colors.deepPurpleAccent));

    await tester.enterText(find.byType(TextField), '0123456789');
    await tester.pump();

    expect(textController.text, '0123456789');
    expect(find.text('10/10'), findsOneWidget);
    counterTextWidget = tester.widget(find.text('10/10'));
    expect(counterTextWidget.style!.color, isNot(equals(Colors.deepPurpleAccent)));
  });

  testWidgets('maxLength shows warning in Material 3', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();
    final ThemeData theme = ThemeData.from(
      colorScheme: const ColorScheme.light().copyWith(error: Colors.deepPurpleAccent),
      useMaterial3: true,
    );
    await tester.pumpWidget(
      boilerplate(
        theme: theme,
        child: TextField(
          controller: textController,
          maxLength: 10,
          maxLengthEnforcement: MaxLengthEnforcement.none,
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), '0123456789101112');
    await tester.pump();

    expect(textController.text, '0123456789101112');
    expect(find.text('16/10'), findsOneWidget);
    Text counterTextWidget = tester.widget(find.text('16/10'));
    expect(counterTextWidget.style!.color, equals(Colors.deepPurpleAccent));

    await tester.enterText(find.byType(TextField), '0123456789');
    await tester.pump();

    expect(textController.text, '0123456789');
    expect(find.text('10/10'), findsOneWidget);
    counterTextWidget = tester.widget(find.text('10/10'));
    expect(counterTextWidget.style!.color, isNot(equals(Colors.deepPurpleAccent)));
  });

  testWidgets('maxLength shows warning when maxLengthEnforcement.none with surrogate pairs.', (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();
    const TextStyle testStyle = TextStyle(color: Colors.deepPurpleAccent);

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          decoration: const InputDecoration(errorStyle: testStyle),
          controller: textController,
          maxLength: 10,
          maxLengthEnforcement: MaxLengthEnforcement.none,
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), '😆012345678910111');
    await tester.pump();

    expect(textController.text, '😆012345678910111');
    expect(find.text('16/10'), findsOneWidget);
    Text counterTextWidget = tester.widget(find.text('16/10'));
    expect(counterTextWidget.style!.color, equals(Colors.deepPurpleAccent));

    await tester.enterText(find.byType(TextField), '😆012345678');
    await tester.pump();

    expect(textController.text, '😆012345678');
    expect(find.text('10/10'), findsOneWidget);
    counterTextWidget = tester.widget(find.text('10/10'));
    expect(counterTextWidget.style!.color, isNot(equals(Colors.deepPurpleAccent)));
  });

  testWidgets('maxLength shows warning when maxLengthEnforcement.none with grapheme clusters.', (
    WidgetTester tester,
  ) async {
    final TextEditingController textController = _textEditingController();
    const TextStyle testStyle = TextStyle(color: Colors.deepPurpleAccent);

    await tester.pumpWidget(
      boilerplate(
        child: TextField(
          decoration: const InputDecoration(errorStyle: testStyle),
          controller: textController,
          maxLength: 10,
          maxLengthEnforcement: MaxLengthEnforcement.none,
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), '👨‍👩‍👦012345678910111');
    await tester.pump();

    expect(textController.text, '👨‍👩‍👦012345678910111');
    expect(find.text('16/10'), findsOneWidget);
    Text counterTextWidget = tester.widget(find.text('16/10'));
    expect(counterTextWidget.style!.color, equals(Colors.deepPurpleAccent));

    await tester.enterText(find.byType(TextField), '👨‍👩‍👦012345678');
    await tester.pump();

    expect(textController.text, '👨‍👩‍👦012345678');
    expect(find.text('10/10'), findsOneWidget);
    counterTextWidget = tester.widget(find.text('10/10'));
    expect(counterTextWidget.style!.color, isNot(equals(Colors.deepPurpleAccent)));
  });

  testWidgets('maxLength limits input with surrogate pairs.', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    const String surrogatePair = '😆';
    await tester.enterText(find.byType(TextField), '${surrogatePair}0123456789101112');
    expect(textController.text, '${surrogatePair}012345678');
  });

  testWidgets('maxLength limits input with grapheme clusters.', (WidgetTester tester) async {
    final TextEditingController textController = _textEditingController();

    await tester.pumpWidget(
      boilerplate(child: TextField(controller: textController, maxLength: 10)),
    );

    const String graphemeCluster = '👨‍👩‍👦';
    await tester.enterText(find.byType(TextField), '${graphemeCluster}0123456789101112');
    expect(textController.text, '${graphemeCluster}012345678');
  });

  testWidgets('setting maxLength shows counter', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: TextField(maxLength: 10)))),
    );

    expect(find.text('0/10'), findsOneWidget);

    await tester.enterText(find.byType(TextField), '01234');
    await tester.pump();

    expect(find.text('5/10'), findsOneWidget);
  });

  testWidgets('maxLength counter measures surrogate pairs as one character', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: TextField(maxLength: 10)))),
    );

    expect(find.text('0/10'), findsOneWidget);

    const String surrogatePair = '😆';
    await tester.enterText(find.byType(TextField), surrogatePair);
    await tester.pump();

    expect(find.text('1/10'), findsOneWidget);
  });

  testWidgets('maxLength counter measures grapheme clusters as one character', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: TextField(maxLength: 10)))),
    );

    expect(find.text('0/10'), findsOneWidget);

    const String familyEmoji = '👨‍👩‍👦';
    await tester.enterText(find.byType(TextField), familyEmoji);
    await tester.pump();

    expect(find.text('1/10'), findsOneWidget);
  });

  testWidgets('setting maxLength to TextField.noMaxLength shows only entered length', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: TextField(maxLength: TextField.noMaxLength))),
      ),
    );

    expect(find.text('0'), findsOneWidget);

    await tester.enterText(find.byType(TextField), '01234');
    await tester.pump();

    expect(find.text('5'), findsOneWidget);
  });

  testWidgets('passing a buildCounter shows returned widget', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(
              buildCounter: (
                BuildContext context, {
                required int currentLength,
                int? maxLength,
                required bool isFocused,
              }) {
                return Text('$currentLength of $maxLength');
              },
              maxLength: 10,
            ),
          ),
        ),
      ),
    );

    expect(find.text('0 of 10'), findsOneWidget);

    await tester.enterText(find.byType(TextField), '01234');
    await tester.pump();

    expect(find.text('5 of 10'), findsOneWidget);
  });

  testWidgets('TextField identifies as text field in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: TextField(maxLength: 10)))),
    );

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Can scroll multiline input when disabled', (WidgetTester tester) async {
    final Key textFieldKey = UniqueKey();
    final TextEditingController controller = _textEditingController(text: kMoreThanFourLines);

    await tester.pumpWidget(
      overlay(
        child: TextField(
          dragStartBehavior: DragStartBehavior.down,
          key: textFieldKey,
          controller: controller,
          ignorePointers: false,
          enabled: false,
          style: const TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 2,
        ),
      ),
    );

    RenderBox findInputBox() => tester.renderObject(find.byKey(textFieldKey));
    final RenderBox inputBox = findInputBox();

    // Check that the last line of text is not displayed.
    final Offset firstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    final Offset fourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));
    expect(firstPos.dx, fourthPos.dx);
    expect(firstPos.dy, lessThan(fourthPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(firstPos)),
      isTrue,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(fourthPos)),
      isFalse,
    );

    final TestGesture gesture = await tester.startGesture(firstPos, pointer: 7);
    await tester.pump();
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();
    await gesture.up();
    await tester.pumpAndSettle();

    // Now the first line is scrolled up, and the fourth line is visible.
    final Offset finalFirstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    final Offset finalFourthPos = textOffsetToPosition(
      tester,
      kMoreThanFourLines.indexOf('Fourth'),
    );

    expect(finalFirstPos.dy, lessThan(firstPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(finalFirstPos)),
      isFalse,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(finalFourthPos)),
      isTrue,
    );
  });

  testWidgets('Disabled text field does not have tap action', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: TextField(maxLength: 10, enabled: false))),
      ),
    );

    expect(
      semantics,
      isNot(
        includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus]),
      ),
    );
    semantics.dispose();
  });

  testWidgets('Disabled text field semantics node still contains value', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(
              controller: _textEditingController(text: 'text'),
              maxLength: 10,
              enabled: false,
            ),
          ),
        ),
      ),
    );

    expect(semantics, includesNodeWith(actions: <SemanticsAction>[], value: 'text'));
    semantics.dispose();
  });

  testWidgets('Readonly text field does not have tap action', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: TextField(maxLength: 10, readOnly: true))),
      ),
    );

    expect(
      semantics,
      isNot(
        includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus]),
      ),
    );

    semantics.dispose();
  });

  testWidgets('Disabled text field hides helper and counter', (WidgetTester tester) async {
    const String helperText = 'helper text';
    const String counterText = 'counter text';
    const String errorText = 'error text';
    Widget buildFrame(bool enabled, bool hasError) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: TextField(
              decoration: InputDecoration(
                labelText: 'label text',
                helperText: helperText,
                counterText: counterText,
                errorText: hasError ? errorText : null,
                enabled: enabled,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(true, false));
    Text helperWidget = tester.widget(find.text(helperText));
    Text counterWidget = tester.widget(find.text(counterText));
    expect(helperWidget.style!.color, isNot(equals(Colors.transparent)));
    expect(counterWidget.style!.color, isNot(equals(Colors.transparent)));
    await tester.pumpWidget(buildFrame(true, true));
    counterWidget = tester.widget(find.text(counterText));
    Text errorWidget = tester.widget(find.text(errorText));
    expect(helperWidget.style!.color, isNot(equals(Colors.transparent)));
    expect(errorWidget.style!.color, isNot(equals(Colors.transparent)));

    // When enabled is false, the helper/error and counter are not visible.
    await tester.pumpWidget(buildFrame(false, false));
    helperWidget = tester.widget(find.text(helperText));
    counterWidget = tester.widget(find.text(counterText));
    expect(helperWidget.style!.color, equals(Colors.transparent));
    expect(counterWidget.style!.color, equals(Colors.transparent));
    await tester.pumpWidget(buildFrame(false, true));
    errorWidget = tester.widget(find.text(errorText));
    counterWidget = tester.widget(find.text(counterText));
    expect(counterWidget.style!.color, equals(Colors.transparent));
    expect(errorWidget.style!.color, equals(Colors.transparent));
  });

  testWidgets('Disabled text field has default M2 disabled text style for the input text', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(child: Center(child: TextField(controller: controller, enabled: false))),
      ),
    );
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(
      editableText.style.color,
      Colors.black38,
    ); // Colors.black38 is the default disabled color for ThemeData.light().
  });

  testWidgets('Disabled text field has default M3 disabled text style for the input text', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );

    final ThemeData theme = ThemeData.light(useMaterial3: true);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(child: Center(child: TextField(controller: controller, enabled: false))),
      ),
    );
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, theme.textTheme.bodyLarge!.color!.withOpacity(0.38));
  });

  testWidgets('Enabled TextField statesController', (WidgetTester tester) async {
    final TextEditingController textEditingController = TextEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );
    addTearDown(textEditingController.dispose);

    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController statesController = MaterialStatesController();
    addTearDown(statesController.dispose);

    statesController.addListener(valueChanged);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(statesController: statesController, controller: textEditingController),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    final Offset center = tester.getCenter(find.byType(EditableText).first);
    await gesture.moveTo(center);
    await tester.pump();
    expect(statesController.value, <MaterialState>{MaterialState.hovered});
    expect(count, 1);

    await gesture.moveTo(Offset.zero);
    await tester.pump();
    expect(statesController.value, <MaterialState>{});
    expect(count, 2);

    await gesture.down(center);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(statesController.value, <MaterialState>{MaterialState.hovered, MaterialState.focused});
    expect(count, 4); // adds hovered and pressed - two changes.

    await gesture.moveTo(Offset.zero);
    await tester.pump();
    expect(statesController.value, <MaterialState>{MaterialState.focused});
    expect(count, 5);

    await gesture.down(Offset.zero);
    await tester.pump();
    expect(statesController.value, <MaterialState>{});
    expect(count, 6);
    await gesture.up();
    await tester.pump();

    await gesture.down(center);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(statesController.value, <MaterialState>{MaterialState.hovered, MaterialState.focused});
    expect(count, 8); // adds hovered and pressed - two changes.

    // If the text field is rebuilt disabled, then the focused state is
    // removed.
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(
              statesController: statesController,
              controller: textEditingController,
              enabled: false,
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(statesController.value, <MaterialState>{MaterialState.hovered, MaterialState.disabled});
    expect(count, 10); // removes focused and adds disabled - two changes.

    await gesture.moveTo(Offset.zero);
    await tester.pump();
    expect(statesController.value, <MaterialState>{MaterialState.disabled});
    expect(count, 11);

    // If the text field is rebuilt enabled and in an error state, then the error
    // state is added.
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(
              statesController: statesController,
              controller: textEditingController,
              decoration: const InputDecoration(errorText: 'error'),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(statesController.value, <MaterialState>{MaterialState.error});
    expect(count, 13); // removes disabled and adds error - two changes.

    // If the text field is rebuilt without an error, then the error
    // state is removed.
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextField(statesController: statesController, controller: textEditingController),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(statesController.value, <MaterialState>{});
    expect(count, 14);
  });

  testWidgets('Disabled TextField statesController', (WidgetTester tester) async {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController controller = MaterialStatesController();
    addTearDown(controller.dispose);
    controller.addListener(valueChanged);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextField(statesController: controller, enabled: false)),
        ),
      ),
    );
    expect(controller.value, <MaterialState>{MaterialState.disabled});
    expect(count, 1);
  });

  testWidgets('Provided style correctly resolves for material states', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );

    final ThemeData theme = ThemeData.light(useMaterial3: true);

    Widget buildFrame(bool enabled) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: TextField(
              controller: controller,
              enabled: enabled,
              style: MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
                if (states.contains(MaterialState.disabled)) {
                  return const TextStyle(color: Colors.red);
                }
                return const TextStyle(color: Colors.blue);
              }),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(false));
    EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, Colors.red);
    await tester.pumpWidget(buildFrame(true));
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, Colors.blue);
  });

  testWidgets('currentValueLength/maxValueLength are in the tree', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextField(controller: controller, maxLength: 10))),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
        ],
        maxValueLength: 10,
        currentValueLength: 0,
      ),
    );

    await tester.showKeyboard(find.byType(TextField));
    const String testValue = '123';
    tester.testTextInput.updateEditingValue(
      const TextEditingValue(
        text: testValue,
        selection: TextSelection.collapsed(offset: 3),
        composing: TextRange(start: 0, end: testValue.length),
      ),
    );
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isFocused,
        ],
        maxValueLength: 10,
        currentValueLength: 3,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Read only TextField identifies as read only text field in semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: TextField(maxLength: 10, readOnly: true))),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isReadOnly,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets("Disabled TextField can't be traversed to.", (WidgetTester tester) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: 'TextField 1');
    addTearDown(focusNode1.dispose);
    final FocusNode focusNode2 = FocusNode(debugLabel: 'TextField 2');
    addTearDown(focusNode2.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: FocusScope(
            child: Center(
              child: Column(
                children: <Widget>[
                  TextField(focusNode: focusNode1, autofocus: true, maxLength: 10, enabled: true),
                  TextField(focusNode: focusNode2, maxLength: 10, enabled: false),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    expect(focusNode1.nextFocus(), isFalse);
    await tester.pump();

    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);
  });

  group(
    'Keyboard Tests',
    () {
      late TextEditingController controller;

      setUp(() {
        controller = _textEditingController();
      });

      Future<void> setupWidget(WidgetTester tester) async {
        final FocusNode focusNode = _focusNode();
        controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: KeyboardListener(
                focusNode: focusNode,
                child: TextField(controller: controller, maxLines: 3),
              ),
            ),
          ),
        );
        await tester.pump();
      }

      testWidgets('Shift test 1', (WidgetTester tester) async {
        await setupWidget(tester);
        const String testValue = 'a big house';
        await tester.enterText(find.byType(TextField), testValue);

        await tester.idle();
        // Need to wait for selection to catch up.
        await tester.pump();
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        expect(controller.selection.extentOffset - controller.selection.baseOffset, -1);
      }, variant: KeySimulatorTransitModeVariant.all());

      testWidgets('Shift test 2', (WidgetTester tester) async {
        await setupWidget(tester);

        const String testValue = 'abcdefghi';
        await tester.showKeyboard(find.byType(TextField));
        tester.testTextInput.updateEditingValue(
          const TextEditingValue(
            text: testValue,
            selection: TextSelection.collapsed(offset: 3),
            composing: TextRange(start: 0, end: testValue.length),
          ),
        );
        await tester.pump();

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();
        expect(controller.selection.extentOffset - controller.selection.baseOffset, 1);
      }, variant: KeySimulatorTransitModeVariant.all());

      testWidgets('Control Shift test', (WidgetTester tester) async {
        await setupWidget(tester);
        const String testValue = 'their big house';
        await tester.enterText(find.byType(TextField), testValue);

        await tester.idle();
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();
        await tester.pumpAndSettle();
        await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 5);
      }, variant: KeySimulatorTransitModeVariant.all());

      testWidgets('Down and up test', (WidgetTester tester) async {
        await setupWidget(tester);
        const String testValue = 'a big house';
        await tester.enterText(find.byType(TextField), testValue);

        await tester.idle();
        // Need to wait for selection to catch up.
        await tester.pump();
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, -11);

        await tester.sendKeyUpEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowDown);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 0);
      }, variant: KeySimulatorTransitModeVariant.all());

      testWidgets('Down and up test 2', (WidgetTester tester) async {
        await setupWidget(tester);
        const String testValue = 'a big house\njumped over a mouse\nOne more line yay'; // 11 \n 19
        await tester.enterText(find.byType(TextField), testValue);

        await tester.idle();
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();

        for (int i = 0; i < 5; i += 1) {
          await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowRight);
          await tester.pumpAndSettle();
          await tester.sendKeyUpEvent(LogicalKeyboardKey.arrowRight);
          await tester.pumpAndSettle();
        }
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 12);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 32);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 12);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, 0);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();

        expect(controller.selection.extentOffset - controller.selection.baseOffset, -5);
      }, variant: KeySimulatorTransitModeVariant.all());

      testWidgets('Read only keyboard selection test', (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: 'readonly');
        await tester.pumpWidget(overlay(child: TextField(controller: controller, readOnly: true)));

        await tester.idle();
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();

        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowLeft);
        expect(controller.selection.extentOffset - controller.selection.baseOffset, -1);
      }, variant: KeySimulatorTransitModeVariant.all());
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
  );

  testWidgets(
    'Copy paste test',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController();
      final TextField textField = TextField(controller: controller, maxLines: 3);

      String clipboardContent = '';
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        if (methodCall.method == 'Clipboard.setData') {
          // ignore: avoid_dynamic_calls
          clipboardContent = methodCall.arguments['text'] as String;
        } else if (methodCall.method == 'Clipboard.getData') {
          return <String, dynamic>{'text': clipboardContent};
        }
        return null;
      });

      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: KeyboardListener(focusNode: focusNode, child: textField)),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();

      const String testValue = 'a big house\njumped over a mouse'; // 11 \n 19
      await tester.enterText(find.byType(TextField), testValue);

      await tester.idle();
      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Select the first 5 characters
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

      // Copy them
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      await tester.sendKeyEvent(LogicalKeyboardKey.keyC);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      await tester.pumpAndSettle();

      expect(clipboardContent, 'a big');

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();

      // Paste them
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyV);
      await tester.pumpAndSettle();
      await tester.pump(const Duration(milliseconds: 200));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyV);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      await tester.pumpAndSettle();

      const String expected = 'a biga big house\njumped over a mouse';
      expect(
        find.text(expected),
        findsOneWidget,
        reason: 'Because text contains ${controller.text}',
      );
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  // Regression test for https://github.com/flutter/flutter/issues/78219
  testWidgets(
    'Paste does not crash after calling TextController.text setter',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController();
      final TextField textField = TextField(controller: controller, obscureText: true);

      const String clipboardContent = 'I love Flutter!';
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        if (methodCall.method == 'Clipboard.getData') {
          return <String, dynamic>{'text': clipboardContent};
        }
        return null;
      });

      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: KeyboardListener(focusNode: focusNode, child: textField)),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Clear the text.
      controller.text = '';

      // Paste clipboardContent to the text field.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyV);
      await tester.pumpAndSettle();
      await tester.pump(const Duration(milliseconds: 200));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyV);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      await tester.pumpAndSettle();

      // Clipboard content is correctly pasted.
      expect(find.text(clipboardContent), findsOneWidget);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Cut test',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController();
      final TextField textField = TextField(controller: controller, maxLines: 3);
      String clipboardContent = '';
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        if (methodCall.method == 'Clipboard.setData') {
          // ignore: avoid_dynamic_calls
          clipboardContent = methodCall.arguments['text'] as String;
        } else if (methodCall.method == 'Clipboard.getData') {
          return <String, dynamic>{'text': clipboardContent};
        }
        return null;
      });

      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: KeyboardListener(focusNode: focusNode, child: textField)),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();

      const String testValue = 'a big house\njumped over a mouse'; // 11 \n 19
      await tester.enterText(find.byType(TextField), testValue);

      await tester.idle();
      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Select the first 5 characters
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
        await tester.pumpAndSettle();
      }

      // Cut them
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      await tester.sendKeyEvent(LogicalKeyboardKey.keyX);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      await tester.pumpAndSettle();

      expect(clipboardContent, 'a big');

      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();
      }

      // Paste them
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyV);
      await tester.pumpAndSettle();
      await tester.pump(const Duration(milliseconds: 200));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyV);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      await tester.pumpAndSettle();

      const String expected = ' housa bige\njumped over a mouse';
      expect(find.text(expected), findsOneWidget);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Select all test',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController();
      final TextField textField = TextField(controller: controller, maxLines: 3);

      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: KeyboardListener(focusNode: focusNode, child: textField)),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();

      const String testValue = 'a big house\njumped over a mouse'; // 11 \n 19
      await tester.enterText(find.byType(TextField), testValue);

      await tester.idle();
      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Select All
      await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
      await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
      await tester.pumpAndSettle();

      // Delete them
      await tester.sendKeyDownEvent(LogicalKeyboardKey.delete);
      await tester.pumpAndSettle();
      await tester.pump(const Duration(milliseconds: 200));

      await tester.sendKeyUpEvent(LogicalKeyboardKey.delete);
      await tester.pumpAndSettle();

      const String expected = '';
      expect(find.text(expected), findsOneWidget);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Delete test',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final TextEditingController controller = _textEditingController();
      final TextField textField = TextField(controller: controller, maxLines: 3);

      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: KeyboardListener(focusNode: focusNode, child: textField)),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();

      const String testValue = 'a big house\njumped over a mouse'; // 11 \n 19
      await tester.enterText(find.byType(TextField), testValue);

      await tester.idle();
      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Delete
      for (int i = 0; i < 6; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.delete);
        await tester.pumpAndSettle();
      }

      const String expected = 'house\njumped over a mouse';
      expect(find.text(expected), findsOneWidget);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
      await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.delete);
      await tester.pumpAndSettle();

      const String expected2 = '';
      expect(find.text(expected2), findsOneWidget);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Changing positions of text fields',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final List<KeyEvent> events = <KeyEvent>[];

      final TextEditingController c1 = _textEditingController();
      final TextEditingController c2 = _textEditingController();
      final Key key1 = UniqueKey();
      final Key key2 = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: KeyboardListener(
              focusNode: focusNode,
              onKeyEvent: events.add,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  TextField(key: key1, controller: c1, maxLines: 3),
                  TextField(key: key2, controller: c2, maxLines: 3),
                ],
              ),
            ),
          ),
        ),
      );

      const String testValue = 'a big house';
      await tester.enterText(find.byType(TextField).first, testValue);

      await tester.idle();
      // Need to wait for selection to catch up.
      await tester.pump();
      await tester.tap(find.byType(TextField).first);
      await tester.pumpAndSettle();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

      expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: KeyboardListener(
              focusNode: focusNode,
              onKeyEvent: events.add,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  TextField(key: key2, controller: c2, maxLines: 3),
                  TextField(key: key1, controller: c1, maxLines: 3),
                ],
              ),
            ),
          ),
        ),
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

      expect(c1.selection.extentOffset - c1.selection.baseOffset, -10);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Changing focus test',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      final List<KeyEvent> events = <KeyEvent>[];

      final TextEditingController c1 = _textEditingController();
      final TextEditingController c2 = _textEditingController();
      final Key key1 = UniqueKey();
      final Key key2 = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: KeyboardListener(
              focusNode: focusNode,
              onKeyEvent: events.add,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  TextField(key: key1, controller: c1, maxLines: 3),
                  TextField(key: key2, controller: c2, maxLines: 3),
                ],
              ),
            ),
          ),
        ),
      );

      const String testValue = 'a big house';
      await tester.enterText(find.byType(TextField).first, testValue);
      await tester.idle();
      await tester.pump();

      await tester.idle();
      await tester.tap(find.byType(TextField).first);
      await tester.pumpAndSettle();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

      expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);
      expect(c2.selection.extentOffset - c2.selection.baseOffset, 0);

      await tester.enterText(find.byType(TextField).last, testValue);
      await tester.idle();
      await tester.pump();

      await tester.idle();
      await tester.tap(find.byType(TextField).last);
      await tester.pumpAndSettle();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

      expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);
      expect(c2.selection.extentOffset - c2.selection.baseOffset, -5);
    },
    // [intended] only applies to platforms where we handle key events.
    skip: areKeyEventsHandledByPlatform,
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets('Caret works when maxLines is null', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(overlay(child: TextField(controller: controller, maxLines: null)));

    const String testValue = 'x';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);
    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, testValue.length);

    // Tap the selection handle to bring up the "paste / select all" menu.
    await tester.tapAt(textOffsetToPosition(tester, 0));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is

    // Confirm that the selection was updated.
    expect(controller.selection.baseOffset, 0);
  });

  testWidgets('TextField baseline alignment no-strut', (WidgetTester tester) async {
    final TextEditingController controllerA = _textEditingController(text: 'A');
    final TextEditingController controllerB = _textEditingController(text: 'B');
    final Key keyA = UniqueKey();
    final Key keyB = UniqueKey();

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.baseline,
            textBaseline: TextBaseline.alphabetic,
            children: <Widget>[
              Expanded(
                child: TextField(
                  key: keyA,
                  decoration: null,
                  controller: controllerA,
                  // The point size of the font must be a multiple of 4 until
                  // https://github.com/flutter/flutter/issues/122066 is resolved.
                  style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 12.0),
                  strutStyle: StrutStyle.disabled,
                ),
              ),
              const Text(
                'abc',
                // The point size of the font must be a multiple of 4 until
                // https://github.com/flutter/flutter/issues/122066 is resolved.
                style: TextStyle(fontFamily: 'FlutterTest', fontSize: 24.0),
              ),
              Expanded(
                child: TextField(
                  key: keyB,
                  decoration: null,
                  controller: controllerB,
                  // The point size of the font must be a multiple of 4 until
                  // https://github.com/flutter/flutter/issues/122066 is resolved.
                  style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 36.0),
                  strutStyle: StrutStyle.disabled,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    // The test font extends 0.25 * fontSize below the baseline.
    // So the three row elements line up like this:
    //
    //  A  abc  B
    //  ---------  baseline
    //  3   6   9  space below the baseline = 0.25 * fontSize
    //  ---------  rowBottomY

    final double rowBottomY = tester.getBottomLeft(find.byType(Row)).dy;
    expect(tester.getBottomLeft(find.byKey(keyA)).dy, rowBottomY - 6.0);
    expect(tester.getBottomLeft(find.text('abc')).dy, rowBottomY - 3.0);
    expect(tester.getBottomLeft(find.byKey(keyB)).dy, rowBottomY);
  });

  testWidgets('TextField baseline alignment', (WidgetTester tester) async {
    final TextEditingController controllerA = _textEditingController(text: 'A');
    final TextEditingController controllerB = _textEditingController(text: 'B');
    final Key keyA = UniqueKey();
    final Key keyB = UniqueKey();

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: overlay(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.baseline,
            textBaseline: TextBaseline.alphabetic,
            children: <Widget>[
              Expanded(
                child: TextField(
                  key: keyA,
                  decoration: null,
                  controller: controllerA,
                  // The point size of the font must be a multiple of 4 until
                  // https://github.com/flutter/flutter/issues/122066 is resolved.
                  style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 12.0),
                ),
              ),
              const Text(
                'abc',
                // The point size of the font must be a multiple of 4 until
                // https://github.com/flutter/flutter/issues/122066 is resolved.
                style: TextStyle(fontFamily: 'FlutterTest', fontSize: 24.0),
              ),
              Expanded(
                child: TextField(
                  key: keyB,
                  decoration: null,
                  controller: controllerB,
                  // The point size of the font must be a multiple of 4 until
                  // https://github.com/flutter/flutter/issues/122066 is resolved.
                  style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 36.0),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    // The test font extends 0.25 * fontSize below the baseline.
    // So the three row elements line up like this:
    //
    //  A  abc  B
    //  ---------  baseline
    //  3   6   9  space below the baseline = 0.25 * fontSize
    //  ---------  rowBottomY

    final double rowBottomY = tester.getBottomLeft(find.byType(Row)).dy;
    // The values here should match the version with strut disabled ('TextField baseline alignment no-strut')
    expect(tester.getBottomLeft(find.byKey(keyA)).dy, rowBottomY - 6.0);
    expect(tester.getBottomLeft(find.text('abc')).dy, rowBottomY - 3.0);
    expect(tester.getBottomLeft(find.byKey(keyB)).dy, rowBottomY);
  });

  testWidgets(
    'TextField semantics include label when unfocused and label/hint when focused if input is empty',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final TextEditingController controller = _textEditingController();
      final Key key = UniqueKey();

      await tester.pumpWidget(
        overlay(
          child: TextField(
            key: key,
            controller: controller,
            decoration: const InputDecoration(hintText: 'hint', labelText: 'label'),
          ),
        ),
      );

      final SemanticsNode node = tester.getSemantics(find.byKey(key));

      expect(node.label, 'label');
      expect(node.value, '');

      // Focus text field.
      await tester.tap(find.byKey(key));
      await tester.pump();

      expect(node.label, 'label');
      expect(node.value, '');
      semantics.dispose();
    },
  );

  testWidgets(
    'TextField semantics always include label and not hint when input value is not empty',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final TextEditingController controller = _textEditingController(text: 'value');
      final Key key = UniqueKey();

      await tester.pumpWidget(
        overlay(
          child: TextField(
            key: key,
            controller: controller,
            decoration: const InputDecoration(hintText: 'hint', labelText: 'label'),
          ),
        ),
      );

      final SemanticsNode node = tester.getSemantics(find.byKey(key));

      expect(node.label, 'label');
      expect(node.value, 'value');

      // Focus text field.
      await tester.tap(find.byKey(key));
      await tester.pump();

      expect(node.label, 'label');
      expect(node.value, 'value');
      semantics.dispose();
    },
  );

  testWidgets('TextField semantics always include label when no hint is given', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController(text: 'value');
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: key,
          controller: controller,
          decoration: const InputDecoration(labelText: 'label'),
        ),
      ),
    );

    final SemanticsNode node = tester.getSemantics(find.byKey(key));

    expect(node.label, 'label');
    expect(node.value, 'value');

    // Focus text field.
    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(node.label, 'label');
    expect(node.value, 'value');
    semantics.dispose();
  });

  testWidgets('TextField semantics only include hint when it is visible', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController(text: 'value');
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: key,
          controller: controller,
          decoration: const InputDecoration(hintText: 'hint'),
        ),
      ),
    );

    final SemanticsNode node = tester.getSemantics(find.byKey(key));

    expect(node.label, '');
    expect(node.value, 'value');

    // Focus text field.
    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(node.label, '');
    expect(node.value, 'value');

    // Clear the Text.
    await tester.enterText(find.byType(TextField), '');
    await tester.pumpAndSettle();

    expect(node.value, '');
    expect(node.label, 'hint');

    semantics.dispose();
  });

  testWidgets('TextField semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: TextField(key: key, controller: controller)));

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.text = 'Guten Tag';
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              value: 'Guten Tag',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              value: 'Guten Tag',
              textSelection: const TextSelection.collapsed(offset: 9),
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.selection = const TextSelection.collapsed(offset: 4);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection.collapsed(offset: 4),
              value: 'Guten Tag',
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.text = 'Schönen Feierabend';
    controller.selection = const TextSelection.collapsed(offset: 0);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection.collapsed(offset: 0),
              value: 'Schönen Feierabend',
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  // Regressing test for https://github.com/flutter/flutter/issues/99763
  testWidgets('Update textField semantics when obscureText changes', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    await tester.pumpWidget(_ObscureTextTestWidget(controller: controller));

    controller.text = 'Hello';
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
        textDirection: TextDirection.ltr,
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
        ],
        value: 'Hello',
      ),
    );

    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
        textDirection: TextDirection.ltr,
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isObscured,
        ],
      ),
    );

    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
        textDirection: TextDirection.ltr,
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
        ],
        value: 'Hello',
      ),
    );

    semantics.dispose();
  });

  testWidgets('TextField semantics, enableInteractiveSelection = false', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(key: key, controller: controller, enableInteractiveSelection: false),
      ),
    );

    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.setText,
                // Absent the following because enableInteractiveSelection: false
                // SemanticsAction.moveCursorBackwardByCharacter,
                // SemanticsAction.moveCursorBackwardByWord,
                // SemanticsAction.setSelection,
                // SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('TextField semantics for selections', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController()..text = 'Hello';
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: TextField(key: key, controller: controller)));

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    // Focus the text field
    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textSelection: const TextSelection.collapsed(offset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.selection = const TextSelection(baseOffset: 5, extentOffset: 3);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textSelection: const TextSelection(baseOffset: 5, extentOffset: 3),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
                SemanticsAction.cut,
                SemanticsAction.copy,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('TextField change selection with semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final TextEditingController controller = _textEditingController()..text = 'Hello';
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: TextField(key: key, controller: controller)));

    // Focus the text field
    await tester.tap(find.byKey(key));
    await tester.pump();

    const int inputFieldId = 1;

    expect(
      controller.selection,
      const TextSelection.collapsed(offset: 5, affinity: TextAffinity.upstream),
    );
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: inputFieldId,
              value: 'Hello',
              textSelection: const TextSelection.collapsed(offset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    // move cursor back once
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 4,
      'extent': 4,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 4));

    // move cursor to front
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 0,
      'extent': 0,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 0));

    // select all
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 0,
      'extent': 5,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: inputFieldId,
              value: 'Hello',
              textSelection: const TextSelection(baseOffset: 0, extentOffset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
                SemanticsAction.cut,
                SemanticsAction.copy,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Can activate TextField with explicit controller via semantics ', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/17801

    const String textInTextField = 'Hello';

    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final TextEditingController controller = _textEditingController()..text = textInTextField;
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: TextField(key: key, controller: controller)));

    const int inputFieldId = 1;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              value: textInTextField,
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semanticsOwner.performAction(inputFieldId, SemanticsAction.tap);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              value: textInTextField,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection(
                baseOffset: textInTextField.length,
                extentOffset: textInTextField.length,
              ),
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('When clipboard empty, no semantics paste option', (WidgetTester tester) async {
    const String textInTextField = 'Hello';

    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final TextEditingController controller = _textEditingController()..text = textInTextField;
    final Key key = UniqueKey();

    // Clear the clipboard.
    await Clipboard.setData(const ClipboardData(text: ''));

    await tester.pumpWidget(overlay(child: TextField(key: key, controller: controller)));

    const int inputFieldId = 1;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              value: textInTextField,
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semanticsOwner.performAction(inputFieldId, SemanticsAction.tap);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                // No paste option.
              ],
              value: textInTextField,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection(
                baseOffset: textInTextField.length,
                extentOffset: textInTextField.length,
              ),
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();

    // On web, we don't check for pasteability because that triggers a
    // permission dialog in the browser.
    // https://github.com/flutter/flutter/pull/57139#issuecomment-629048058
  }, skip: isBrowser); // [intended] see above.

  testWidgets('TextField throws when not descended from a Material widget', (
    WidgetTester tester,
  ) async {
    const Widget textField = TextField();
    await tester.pumpWidget(textField);
    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(exception.toString(), startsWith('No Material widget found.'));
  });

  testWidgets('TextField loses focus when disabled', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'TextField Focus Node');
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      boilerplate(child: TextField(focusNode: focusNode, autofocus: true, enabled: true)),
    );
    expect(focusNode.hasFocus, isTrue);

    await tester.pumpWidget(
      boilerplate(child: TextField(focusNode: focusNode, autofocus: true, enabled: false)),
    );
    expect(focusNode.hasFocus, isFalse);

    await tester.pumpWidget(
      boilerplate(
        child: Builder(
          builder: (BuildContext context) {
            return MediaQuery(
              data: MediaQuery.of(context).copyWith(navigationMode: NavigationMode.directional),
              child: TextField(focusNode: focusNode, autofocus: true, enabled: true),
            );
          },
        ),
      ),
    );
    focusNode.requestFocus();
    await tester.pump();

    expect(focusNode.hasFocus, isTrue);

    await tester.pumpWidget(
      boilerplate(
        child: Builder(
          builder: (BuildContext context) {
            return MediaQuery(
              data: MediaQuery.of(context).copyWith(navigationMode: NavigationMode.directional),
              child: TextField(focusNode: focusNode, autofocus: true, enabled: false),
            );
          },
        ),
      ),
    );
    await tester.pump();

    expect(focusNode.hasFocus, isTrue);
  });

  testWidgets('TextField displays text with text direction', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(child: TextField(textDirection: TextDirection.rtl)),
      ),
    );

    RenderEditable editable = findRenderEditable(tester);

    await tester.enterText(find.byType(TextField), '0123456789101112');
    await tester.pumpAndSettle();
    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 10)).topLeft,
    );

    expect(topLeft.dx, equals(701));

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(child: TextField(textDirection: TextDirection.ltr)),
      ),
    );

    editable = findRenderEditable(tester);

    await tester.enterText(find.byType(TextField), '0123456789101112');
    await tester.pumpAndSettle();
    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 10)).topLeft,
    );

    expect(topLeft.dx, equals(160.0));
  });

  testWidgets('TextField semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: key,
          controller: controller,
          maxLength: 10,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            helperText: 'helper',
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'label',
              id: 1,
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
              children: <TestSemantics>[
                TestSemantics(id: 2, label: 'helper', textDirection: TextDirection.ltr),
                TestSemantics(
                  id: 3,
                  label: '10 characters remaining',
                  textDirection: TextDirection.ltr,
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    await tester.tap(find.byType(TextField));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'label',
              id: 1,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection(baseOffset: 0, extentOffset: 0),
              actions: <SemanticsAction>[
                SemanticsAction.tap,
                SemanticsAction.focus,
                SemanticsAction.setSelection,
                SemanticsAction.setText,
                SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocused,
              ],
              children: <TestSemantics>[
                TestSemantics(id: 2, label: 'helper', textDirection: TextDirection.ltr),
                TestSemantics(
                  id: 3,
                  label: '10 characters remaining',
                  flags: <SemanticsFlag>[SemanticsFlag.isLiveRegion],
                  textDirection: TextDirection.ltr,
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.text = 'hello';
    await tester.pump();
    semantics.dispose();
  });

  testWidgets('InputDecoration counterText can have a semanticCounterText', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: key,
          controller: controller,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            helperText: 'helper',
            counterText: '0/10',
            semanticCounterText: '0 out of 10',
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'label',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
              children: <TestSemantics>[
                TestSemantics(label: 'helper', textDirection: TextDirection.ltr),
                TestSemantics(label: '0 out of 10', textDirection: TextDirection.ltr),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('InputDecoration errorText semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = _textEditingController();
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: TextField(
          key: key,
          controller: controller,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            errorText: 'oh no!',
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'label',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
              ],
              children: <TestSemantics>[
                TestSemantics(label: 'oh no!', textDirection: TextDirection.ltr),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('floating label does not overlap with value at large textScaleFactors', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(text: 'Just some text');
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: MediaQuery(
            data: const MediaQueryData(textScaler: TextScaler.linear(4.0)),
            child: Center(
              child: TextField(
                decoration: const InputDecoration(
                  labelText: 'Label',
                  border: UnderlineInputBorder(),
                ),
                controller: controller,
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(TextField));
    final Rect labelRect = tester.getRect(find.text('Label'));
    final Rect fieldRect = tester.getRect(find.text('Just some text'));
    expect(labelRect.bottom, lessThanOrEqualTo(fieldRect.top));
  });

  testWidgets('TextField scrolls into view but does not bounce (SingleChildScrollView)', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/20485

    final Key textField1 = UniqueKey();
    final Key textField2 = UniqueKey();
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    double? minOffset;
    double? maxOffset;

    scrollController.addListener(() {
      final double offset = scrollController.offset;
      minOffset = math.min(minOffset ?? offset, offset);
      maxOffset = math.max(maxOffset ?? offset, offset);
    });

    Widget buildFrame(Axis scrollDirection) {
      return MaterialApp(
        home: Scaffold(
          body: SafeArea(
            child: SingleChildScrollView(
              physics: const BouncingScrollPhysics(),
              controller: scrollController,
              child: Column(
                children: <Widget>[
                  SizedBox(
                    // visible when scrollOffset is 0.0
                    height: 100.0,
                    width: 100.0,
                    child: TextField(key: textField1, scrollPadding: const EdgeInsets.all(200.0)),
                  ),
                  const SizedBox(
                    height: 600.0, // Same size as the frame. Initially
                    width: 800.0, // textField2 is not visible
                  ),
                  SizedBox(
                    // visible when scrollOffset is 200.0
                    height: 100.0,
                    width: 100.0,
                    child: TextField(key: textField2, scrollPadding: const EdgeInsets.all(200.0)),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Axis.vertical));
    await tester.enterText(find.byKey(textField1), '1');
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField2), '2'); //scroll textField2 into view
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField1), '3'); //scroll textField1 back into view
    await tester.pumpAndSettle();

    expect(minOffset, 0.0);
    expect(maxOffset, 200.0);

    minOffset = null;
    maxOffset = null;

    await tester.pumpWidget(buildFrame(Axis.horizontal));
    await tester.enterText(find.byKey(textField1), '1');
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField2), '2'); //scroll textField2 into view
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField1), '3'); //scroll textField1 back into view
    await tester.pumpAndSettle();

    expect(minOffset, 0.0);
    expect(maxOffset, 200.0);
  });

  testWidgets('TextField scrolls into view but does not bounce (ListView)', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/20485

    final Key textField1 = UniqueKey();
    final Key textField2 = UniqueKey();
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    double? minOffset;
    double? maxOffset;

    scrollController.addListener(() {
      final double offset = scrollController.offset;
      minOffset = math.min(minOffset ?? offset, offset);
      maxOffset = math.max(maxOffset ?? offset, offset);
    });

    Widget buildFrame(Axis scrollDirection) {
      return MaterialApp(
        home: Scaffold(
          body: SafeArea(
            child: ListView(
              physics: const BouncingScrollPhysics(),
              controller: scrollController,
              children: <Widget>[
                SizedBox(
                  // visible when scrollOffset is 0.0
                  height: 100.0,
                  width: 100.0,
                  child: TextField(key: textField1, scrollPadding: const EdgeInsets.all(200.0)),
                ),
                const SizedBox(
                  height: 450.0, // 50.0 smaller than the overall frame so that both
                  width: 650.0, // textfields are always partially visible.
                ),
                SizedBox(
                  // visible when scrollOffset = 50.0
                  height: 100.0,
                  width: 100.0,
                  child: TextField(key: textField2, scrollPadding: const EdgeInsets.all(200.0)),
                ),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Axis.vertical));
    await tester.enterText(find.byKey(textField1), '1'); // textfield1 is visible
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField2), '2'); //scroll textField2 into view
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField1), '3'); //scroll textField1 back into view
    await tester.pumpAndSettle();

    expect(minOffset, 0.0);
    expect(maxOffset, 50.0);

    minOffset = null;
    maxOffset = null;

    await tester.pumpWidget(buildFrame(Axis.horizontal));
    await tester.enterText(find.byKey(textField1), '1'); // textfield1 is visible
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField2), '2'); //scroll textField2 into view
    await tester.pumpAndSettle();
    await tester.enterText(find.byKey(textField1), '3'); //scroll textField1 back into view
    await tester.pumpAndSettle();

    expect(minOffset, 0.0);
    expect(maxOffset, 50.0);
  });

  testWidgets('onTap is called upon tap', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      overlay(
        child: TextField(
          onTap: () {
            tapCount += 1;
          },
        ),
      ),
    );

    expect(tapCount, 0);
    await tester.tap(find.byType(TextField));
    // Wait a bit so they're all single taps and not double taps.
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 300));
    expect(tapCount, 3);
  });

  testWidgets('onTap is not called, field is disabled', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      overlay(
        child: TextField(
          enabled: false,
          onTap: () {
            tapCount += 1;
          },
        ),
      ),
    );

    expect(tapCount, 0);
    await tester.tap(find.byType(TextField));
    await tester.tap(find.byType(TextField));
    await tester.tap(find.byType(TextField));
    expect(tapCount, 0);
  });

  testWidgets('Includes cursor for TextField', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/24612

    Widget buildFrame({
      double? stepWidth,
      required double cursorWidth,
      required TextAlign textAlign,
    }) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                IntrinsicWidth(
                  stepWidth: stepWidth,
                  child: TextField(textAlign: textAlign, cursorWidth: cursorWidth),
                ),
              ],
            ),
          ),
        ),
      );
    }

    // A cursor of default size doesn't cause the TextField to increase its
    // width.
    const String text = '1234';
    double? stepWidth = 80.0;
    await tester.pumpWidget(
      buildFrame(stepWidth: 80.0, cursorWidth: 2.0, textAlign: TextAlign.left),
    );
    await tester.enterText(find.byType(TextField), text);
    await tester.pumpAndSettle();
    expect(tester.getSize(find.byType(TextField)).width, stepWidth);

    // A wide cursor is counted in the width of the text and causes the
    // TextField to increase to twice the stepWidth.
    await tester.pumpWidget(
      buildFrame(stepWidth: stepWidth, cursorWidth: 18.0, textAlign: TextAlign.left),
    );
    await tester.enterText(find.byType(TextField), text);
    await tester.pumpAndSettle();
    expect(tester.getSize(find.byType(TextField)).width, 2 * stepWidth);

    // A null stepWidth causes the TextField to perfectly wrap the text plus
    // the cursor regardless of alignment.
    stepWidth = null;
    const double WIDTH_OF_CHAR = 16.0;
    const double CARET_GAP = 1.0;
    await tester.pumpWidget(
      buildFrame(stepWidth: stepWidth, cursorWidth: 18.0, textAlign: TextAlign.left),
    );
    await tester.enterText(find.byType(TextField), text);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byType(TextField)).width,
      WIDTH_OF_CHAR * text.length + 18.0 + CARET_GAP,
    );
    await tester.pumpWidget(
      buildFrame(stepWidth: stepWidth, cursorWidth: 18.0, textAlign: TextAlign.right),
    );
    await tester.enterText(find.byType(TextField), text);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byType(TextField)).width,
      WIDTH_OF_CHAR * text.length + 18.0 + CARET_GAP,
    );
  });

  testWidgets('TextField style is merged with theme', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/23994

    final ThemeData themeData = ThemeData(
      useMaterial3: false,
      textTheme: TextTheme(titleMedium: TextStyle(color: Colors.blue[500])),
    );

    Widget buildFrame(TextStyle style) {
      return MaterialApp(
        theme: themeData,
        home: Material(child: Center(child: TextField(style: style))),
      );
    }

    // Empty TextStyle is overridden by theme
    await tester.pumpWidget(buildFrame(const TextStyle()));
    EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, themeData.textTheme.titleMedium!.color);
    expect(editableText.style.background, themeData.textTheme.titleMedium!.background);
    expect(editableText.style.shadows, themeData.textTheme.titleMedium!.shadows);
    expect(editableText.style.decoration, themeData.textTheme.titleMedium!.decoration);
    expect(editableText.style.locale, themeData.textTheme.titleMedium!.locale);
    expect(editableText.style.wordSpacing, themeData.textTheme.titleMedium!.wordSpacing);

    // Properties set on TextStyle override theme
    const Color setColor = Colors.red;
    await tester.pumpWidget(buildFrame(const TextStyle(color: setColor)));
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, setColor);

    // inherit: false causes nothing to be merged in from theme
    await tester.pumpWidget(
      buildFrame(
        const TextStyle(fontSize: 24.0, textBaseline: TextBaseline.alphabetic, inherit: false),
      ),
    );
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, isNull);
  });

  testWidgets('TextField style is merged with theme in Material 3', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/23994

    final ThemeData themeData = ThemeData(
      useMaterial3: true,
      textTheme: TextTheme(bodyLarge: TextStyle(color: Colors.blue[500])),
    );

    Widget buildFrame(TextStyle style) {
      return MaterialApp(
        theme: themeData,
        home: Material(child: Center(child: TextField(style: style))),
      );
    }

    // Empty TextStyle is overridden by theme
    await tester.pumpWidget(buildFrame(const TextStyle()));
    EditableText editableText = tester.widget(find.byType(EditableText));

    // According to material 3 spec, the input text should be the color of onSurface.
    // https://github.com/flutter/flutter/issues/107686 is tracking this issue.
    expect(editableText.style.color, themeData.textTheme.bodyLarge!.color);

    expect(editableText.style.background, themeData.textTheme.bodyLarge!.background);
    expect(editableText.style.shadows, themeData.textTheme.bodyLarge!.shadows);
    expect(editableText.style.decoration, themeData.textTheme.bodyLarge!.decoration);
    expect(editableText.style.locale, themeData.textTheme.bodyLarge!.locale);
    expect(editableText.style.wordSpacing, themeData.textTheme.bodyLarge!.wordSpacing);

    // Properties set on TextStyle override theme
    const Color setColor = Colors.red;
    await tester.pumpWidget(buildFrame(const TextStyle(color: setColor)));
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, setColor);

    // inherit: false causes nothing to be merged in from theme
    await tester.pumpWidget(
      buildFrame(
        const TextStyle(fontSize: 24.0, textBaseline: TextBaseline.alphabetic, inherit: false),
      ),
    );
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, isNull);
  });

  testWidgets(
    'selection handles color respects Theme',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/74890.
      const Color expectedSelectionHandleColor = Color.fromARGB(255, 10, 200, 255);

      final TextEditingController controller = TextEditingController(text: 'Some text.');
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            textSelectionTheme: const TextSelectionThemeData(selectionHandleColor: Colors.red),
          ),
          home: Material(
            child: Theme(
              data: ThemeData(
                textSelectionTheme: const TextSelectionThemeData(
                  selectionHandleColor: expectedSelectionHandleColor,
                ),
              ),
              child: TextField(controller: controller),
            ),
          ),
        ),
      );

      await tester.longPressAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      final Iterable<RenderBox> boxes = tester.renderObjectList<RenderBox>(
        find.descendant(
          of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay'),
          matching: find.byType(CustomPaint),
        ),
      );
      expect(boxes.length, 2);

      for (final RenderBox box in boxes) {
        expect(box, paints..path(color: expectedSelectionHandleColor));
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets('style enforces required fields', (WidgetTester tester) async {
    Widget buildFrame(TextStyle style) {
      return MaterialApp(home: Material(child: TextField(style: style)));
    }

    await tester.pumpWidget(
      buildFrame(
        const TextStyle(inherit: false, fontSize: 12.0, textBaseline: TextBaseline.alphabetic),
      ),
    );
    expect(tester.takeException(), isNull);

    // With inherit not set to false, will pickup required fields from theme
    await tester.pumpWidget(buildFrame(const TextStyle(fontSize: 12.0)));
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(buildFrame(const TextStyle(inherit: false, fontSize: 12.0)));
    expect(tester.takeException(), isNotNull);
  });

  testWidgets(
    'tap moves cursor to the edge of the word it tapped',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pump();

      // We moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 7, affinity: TextAffinity.upstream),
      );

      // But don't trigger the toolbar.
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'tap with a mouse does not move cursor to the edge of the word',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(
        textfieldStart + const Offset(50.0, 9.0),
        pointer: 1,
        kind: PointerDeviceKind.mouse,
      );
      await gesture.up();

      // Cursor at tap position, not at word edge.
      expect(controller.selection, const TextSelection.collapsed(offset: 3));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'tap moves cursor to the position tapped',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pump();

      // We moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 3));

      // But don't trigger the toolbar.
      expectNoMaterialToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'two slow taps do not trigger a word selection',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      // On macOS, we select the precise position of the tap.
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset pos = textOffsetToPosition(tester, 6); // Index of 'Atwate|r'.

      await tester.tapAt(pos);
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(pos);
      await tester.pump();

      // Plain collapsed selection.
      expect(controller.selection.isCollapsed, isTrue);
      expect(controller.selection.baseOffset, isTargetPlatformMobile ? 7 : 6);

      // Toolbar shows on mobile only.
      if (isTargetPlatformMobile) {
        expectCupertinoToolbarForCollapsedSelection();
      } else {
        // After a tap, macOS does not show a selection toolbar for a collapsed selection.
        expectNoCupertinoToolbar();
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Tapping on a collapsed selection toggles the toolbar',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text:
            'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neigse Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(controller: controller, maxLines: 2))),
        ),
      );

      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      final Offset begPos = textOffsetToPosition(tester, 0);
      final Offset endPos =
          textOffsetToPosition(tester, 35) +
          const Offset(
            200.0,
            0.0,
          ); // Index of 'Bonaventure|' + Offset(200.0,0), which is at the end of the first line.
      final Offset vPos = textOffsetToPosition(tester, 29); // Index of 'Bonav|enture'.
      final Offset wPos = textOffsetToPosition(tester, 3); // Index of 'Atw|ater'.

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(wPos);
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(vPos);
      await tester.pump(const Duration(milliseconds: 500));
      // First tap moved the cursor. Here we tap the position where 'v' is located.
      // On iOS this will select the closest word edge, in this case the cursor is placed
      // at the end of the word 'Bonaventure|'.
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 35);
      expectNoCupertinoToolbar();

      await tester.tapAt(vPos);
      await tester.pumpAndSettle(const Duration(milliseconds: 500));
      // Second tap toggles the toolbar. Here we tap on 'v' again, and select the word edge. Since
      // the selection has not changed we toggle the toolbar.
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 35);
      expectCupertinoToolbarForCollapsedSelection();

      // Tap the 'v' position again to hide the toolbar.
      await tester.tapAt(vPos);
      await tester.pumpAndSettle();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 35);
      expectNoCupertinoToolbar();

      // Long press at the end of the first line to move the cursor to the end of the first line
      // where the word wrap is. Since there is a word wrap here, and the direction of the text is LTR,
      // the TextAffinity will be upstream and against the natural direction. The toolbar is also
      // shown after a long press.
      await tester.longPressAt(endPos);
      await tester.pumpAndSettle(const Duration(milliseconds: 500));
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 46);
      expect(controller.selection.affinity, TextAffinity.upstream);
      expectCupertinoToolbarForCollapsedSelection();

      // Tap at the same position to toggle the toolbar.
      await tester.tapAt(endPos);
      await tester.pumpAndSettle();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 46);
      expect(controller.selection.affinity, TextAffinity.upstream);
      expectNoCupertinoToolbar();

      // Tap at the beginning of the second line to move the cursor to the front of the first word on the
      // second line, where the word wrap is. Since there is a word wrap here, and the direction of the text is LTR,
      // the TextAffinity will be downstream and following the natural direction. The toolbar will be hidden after this tap.
      await tester.tapAt(begPos + Offset(0.0, lineHeight));
      await tester.pumpAndSettle();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 46);
      expect(controller.selection.affinity, TextAffinity.downstream);
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Tapping on a non-collapsed selection toggles the toolbar and retains the selection',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset vPos = textOffsetToPosition(tester, 29); // Index of 'Bonav|enture'.
      final Offset ePos =
          textOffsetToPosition(tester, 35) +
          const Offset(
            7.0,
            0.0,
          ); // Index of 'Bonaventure|' + Offset(7.0,0), which taps slightly to the right of the end of the text.
      final Offset wPos = textOffsetToPosition(tester, 3); // Index of 'Atw|ater'.

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(wPos);
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(vPos);
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 35);
      await tester.tapAt(vPos);
      await tester.pumpAndSettle(const Duration(milliseconds: 500));

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 24, extentOffset: 35));

      // The toolbar shows up.
      expectCupertinoToolbarForPartialSelection();

      // Tap the selected word to hide the toolbar and retain the selection.
      await tester.tapAt(vPos);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 24, extentOffset: 35));
      expectNoCupertinoToolbar();

      // Tap the selected word to show the toolbar and retain the selection.
      await tester.tapAt(vPos);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 24, extentOffset: 35));
      expectCupertinoToolbarForPartialSelection();

      // Tap past the selected word to move the cursor and hide the toolbar.
      await tester.tapAt(ePos);
      await tester.pumpAndSettle();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 35);
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'double tap selects word and first tap of double tap moves cursor (iOS)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset pPos = textOffsetToPosition(tester, 9); // Index of 'P|eel'.
      final Offset wPos = textOffsetToPosition(tester, 3); // Index of 'Atw|ater'.

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(wPos);
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(pPos);
      await tester.pumpAndSettle();

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      // The toolbar shows up.
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets('iOS selectWordEdge works correctly', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Initially, the menu is not shown and there is no selection.
    expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));
    final Offset pos1 = textOffsetToPosition(tester, 1);
    TestGesture gesture = await tester.startGesture(pos1);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();
    expect(
      controller.selection,
      const TextSelection.collapsed(offset: 5, affinity: TextAffinity.upstream),
    );

    final Offset pos0 = textOffsetToPosition(tester, 0);
    gesture = await tester.startGesture(pos0);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection.collapsed(offset: 0));
  }, variant: TargetPlatformVariant.only(TargetPlatform.iOS));

  testWidgets(
    'double tap does not select word on read-only obscured field',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(obscureText: true, readOnly: true, controller: controller),
            ),
          ),
        ),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 35));
      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pumpAndSettle();

      // Second tap doesn't select anything.
      expect(controller.selection, const TextSelection.collapsed(offset: 35));

      // Selected text shows no toolbar.
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'double tap selects word and first tap of double tap moves cursor and shows toolbar',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pumpAndSettle();

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      // The toolbar shows up.
      expectMaterialToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'Custom toolbar test - Android text selection controls',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(
                controller: controller,
                selectionControls: materialTextSelectionControls,
              ),
            ),
          ),
        ),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pumpAndSettle();

      // Selected text shows 4 toolbar buttons: cut, copy, paste, select all
      expect(find.byType(TextButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    },
    variant: TargetPlatformVariant.all(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Custom toolbar test - Cupertino text selection controls',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(
                controller: controller,
                selectionControls: cupertinoTextSelectionControls,
              ),
            ),
          ),
        ),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pumpAndSettle();

      // Selected text shows 3 toolbar buttons: cut, copy, paste
      expect(find.byType(CupertinoButton), findsNWidgets(3));
    },
    variant: TargetPlatformVariant.all(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('selectionControls is passed to EditableText', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Scaffold(body: TextField(selectionControls: materialTextSelectionControls)),
        ),
      ),
    );

    final EditableText widget = tester.widget(find.byType(EditableText));
    expect(widget.selectionControls, equals(materialTextSelectionControls));
  });

  testWidgets('Can double click + drag with a mouse to select word by word', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      ),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final Offset hPos = textOffsetToPosition(tester, testValue.indexOf('h'));

    // Tap on text field to gain focus, and set selection to '|e'.
    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, testValue.indexOf('e'));

    // Here we tap on '|e' again, to register a double tap. This will select
    // the word at the tapped position.
    await gesture.down(ePos);
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 7);

    // Drag, right after the double tap, to select word by word.
    // Moving to the position of 'h', will extend the selection to 'ghi'.
    await gesture.moveTo(hPos);
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, testValue.indexOf('d'));
    expect(controller.selection.extentOffset, testValue.indexOf('i') + 1);
  });

  testWidgets('Can double tap + drag to select word by word', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
        ),
      ),
    );

    const String testValue = 'abc def ghi';
    await tester.enterText(find.byType(TextField), testValue);
    await skipPastScrollingAnimation(tester);

    final Offset ePos = textOffsetToPosition(tester, testValue.indexOf('e'));
    final Offset hPos = textOffsetToPosition(tester, testValue.indexOf('h'));

    // Tap on text field to gain focus, and set selection to '|e'.
    final TestGesture gesture = await tester.startGesture(ePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, testValue.indexOf('e'));

    // Here we tap on '|e' again, to register a double tap. This will select
    // the word at the tapped position.
    await gesture.down(ePos);
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 7);

    // Drag, right after the double tap, to select word by word.
    // Moving to the position of 'h', will extend the selection to 'ghi'.
    await gesture.moveTo(hPos);
    await tester.pumpAndSettle();

    // Toolbar should be hidden during a drag.
    expectNoMaterialToolbar();
    expect(controller.selection.baseOffset, testValue.indexOf('d'));
    expect(controller.selection.extentOffset, testValue.indexOf('i') + 1);

    // Toolbar should re-appear after a drag.
    await gesture.up();
    await tester.pump();
    expectMaterialToolbarForPartialSelection();
  });

  group('Triple tap/click', () {
    const String testValueA =
        'Now is the time for\n' // 20
        'all good people\n' // 20 + 16 => 36
        'to come to the aid\n' // 36 + 19 => 55
        'of their country.'; // 55 + 17 => 72
    const String testValueB =
        'Today is the time for\n' // 22
        'all good people\n' // 22 + 16 => 38
        'to come to the aid\n' // 38 + 19 => 57
        'of their country.'; // 57 + 17 => 74
    testWidgets(
      'Can triple tap to select a paragraph on mobile platforms when tapping at a word edge',
      (WidgetTester tester) async {
        // TODO(Renzo-Olivares): Enable for iOS, currently broken because selection overlay blocks the TextSelectionGestureDetector https://github.com/flutter/flutter/issues/123415.
        final TextEditingController controller = _textEditingController();
        final bool isTargetPlatformApple = defaultTargetPlatform == TargetPlatform.iOS;

        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueA);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueA);

        final Offset firstLinePos = textOffsetToPosition(tester, 6);

        // Tap on text field to gain focus, and set selection to 'is|' on the first line.
        final TestGesture gesture = await tester.startGesture(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 6);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, isTargetPlatformApple ? 4 : 6);
        expect(controller.selection.extentOffset, isTargetPlatformApple ? 6 : 7);

        // Here we tap on same position again, to register a triple tap. This will select
        // the paragraph at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 20);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
      }),
    );

    testWidgets(
      'Can triple tap to select a paragraph on mobile platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();
        final bool isTargetPlatformApple = defaultTargetPlatform == TargetPlatform.iOS;

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueB);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueB);

        final Offset firstLinePos =
            tester.getTopLeft(find.byType(TextField)) + const Offset(50.0, 9.0);

        // Tap on text field to gain focus, and move the selection.
        final TestGesture gesture = await tester.startGesture(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, isTargetPlatformApple ? 5 : 3);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 5);

        // Here we tap on same position again, to register a triple tap. This will select
        // the paragraph at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 22);
      },
      variant: TargetPlatformVariant.mobile(),
    );

    testWidgets(
      'Triple click at the beginning of a line should not select the previous paragraph',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/132126
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueB);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueB);

        final Offset thirdLinePos = textOffsetToPosition(tester, 38);

        // Click on text field to gain focus, and move the selection.
        final TestGesture gesture = await tester.startGesture(
          thirdLinePos,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 38);

        // Here we click on same position again, to register a double click. This will select
        // the word at the clicked position.
        await gesture.down(thirdLinePos);
        await gesture.up();

        expect(controller.selection.baseOffset, 38);
        expect(controller.selection.extentOffset, 40);

        // Here we click on same position again, to register a triple click. This will select
        // the paragraph at the clicked position.
        await gesture.down(thirdLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 38);
        expect(controller.selection.extentOffset, 57);
      },
      variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.linux}),
    );

    testWidgets(
      'Triple click at the end of text should select the previous paragraph',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/132126.
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueB);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueB);

        final Offset endOfTextPos = textOffsetToPosition(tester, 74);

        // Click on text field to gain focus, and move the selection.
        final TestGesture gesture = await tester.startGesture(
          endOfTextPos,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 74);

        // Here we click on same position again, to register a double click.
        await gesture.down(endOfTextPos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 74);
        expect(controller.selection.extentOffset, 74);

        // Here we click on same position again, to register a triple click. This will select
        // the paragraph at the clicked position.
        await gesture.down(endOfTextPos);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 57);
        expect(controller.selection.extentOffset, 74);
      },
      variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.linux}),
    );

    testWidgets(
      'triple tap chains work on Non-Apple mobile platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(
          text: 'Atwater Peel Sherbrooke Bonaventure',
        );
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(child: Center(child: TextField(controller: controller))),
          ),
        );

        final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 3);
        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
        expectMaterialToolbarForPartialSelection();

        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 35));
        // Triple tap selecting the same paragraph somewhere else is fine.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pump(const Duration(milliseconds: 50));
        // First tap hides the toolbar and moves the selection.
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 6);
        expectNoMaterialToolbar();
        // Second tap shows the toolbar and selects the word.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
        expectMaterialToolbarForPartialSelection();

        // Third tap shows the toolbar and selects the paragraph.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 35));
        expectMaterialToolbarForFullSelection();

        await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
        await tester.pump(const Duration(milliseconds: 50));
        // First tap moved the cursor and hid the toolbar.
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 9);
        expectNoMaterialToolbar();
        // Second tap selects the word.
        await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
        expectMaterialToolbarForPartialSelection();

        // Third tap selects the paragraph and shows the toolbar.
        await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 35));
        expectMaterialToolbarForFullSelection();
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
      }),
    );

    testWidgets(
      'triple tap chains work on Apple platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(
          text: 'Atwater Peel Sherbrooke Bonaventure\nThe fox jumped over the fence.',
        );
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
          ),
        );

        final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 7);
        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
        expectCupertinoToolbarForPartialSelection();

        await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
        await tester.pumpAndSettle(kDoubleTapTimeout);
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 36));
        // Triple tap selecting the same paragraph somewhere else is fine.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pump(const Duration(milliseconds: 50));
        // First tap hides the toolbar and retains the selection.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 36));
        expectNoCupertinoToolbar();
        // Second tap shows the toolbar and selects the word.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
        expectCupertinoToolbarForPartialSelection();

        // Third tap shows the toolbar and selects the paragraph.
        await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
        await tester.pumpAndSettle(kDoubleTapTimeout);
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 36));
        expectCupertinoToolbarForPartialSelection();

        await tester.tapAt(textfieldStart + const Offset(150.0, 50.0));
        await tester.pump(const Duration(milliseconds: 50));
        // First tap moved the cursor and hid the toolbar.
        expect(
          controller.selection,
          const TextSelection.collapsed(offset: 50, affinity: TextAffinity.upstream),
        );
        expectNoCupertinoToolbar();
        // Second tap selects the word.
        await tester.tapAt(textfieldStart + const Offset(150.0, 50.0));
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 44, extentOffset: 50));
        expectCupertinoToolbarForPartialSelection();

        // Third tap selects the paragraph and shows the toolbar.
        await tester.tapAt(textfieldStart + const Offset(150.0, 50.0));
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 36, extentOffset: 66));
        expectCupertinoToolbarForPartialSelection();
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets('triple click chains work', (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: testValueA);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
        ),
      );

      final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));
      final bool platformSelectsByLine = defaultTargetPlatform == TargetPlatform.linux;

      // First click moves the cursor to the point of the click, not the edge of
      // the clicked word.
      final TestGesture gesture = await tester.startGesture(
        textFieldStart + const Offset(210.0, 9.0),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 13);

      // Second click selects the word.
      await gesture.down(textFieldStart + const Offset(210.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

      // Triple click selects the paragraph.
      await gesture.down(textFieldStart + const Offset(210.0, 9.0));
      await tester.pump();
      await gesture.up();
      // Wait for the consecutive tap timer to timeout so the next
      // tap is not detected as a triple tap.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(
        controller.selection,
        TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
      );

      // Triple click selecting the same paragraph somewhere else is fine.
      await gesture.down(textFieldStart + const Offset(100.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      // First click moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 6));
      await gesture.down(textFieldStart + const Offset(100.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      // Second click selected the word.
      expect(controller.selection, const TextSelection(baseOffset: 6, extentOffset: 7));

      await gesture.down(textFieldStart + const Offset(100.0, 9.0));
      await tester.pump();
      await gesture.up();
      // Wait for the consecutive tap timer to timeout so the tap count
      // is reset.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      // Third click selected the paragraph.
      expect(
        controller.selection,
        TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
      );

      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      // First click moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      // Second click selected the word.
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 10));

      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      // Third click selects the paragraph.
      expect(
        controller.selection,
        TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
      );
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('triple click after a click on desktop platforms', (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: testValueA);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
        ),
      );

      final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));
      final bool platformSelectsByLine = defaultTargetPlatform == TargetPlatform.linux;

      final TestGesture gesture = await tester.startGesture(
        textFieldStart + const Offset(50.0, 9.0),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.selection, const TextSelection.collapsed(offset: 3));
      // First click moves the selection.
      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      expect(controller.selection, const TextSelection.collapsed(offset: 9));

      // Double click selection to select a word.
      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 10));

      // Triple click selection to select a paragraph.
      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
      );
    }, variant: TargetPlatformVariant.desktop());

    testWidgets(
      'Can triple tap to select all on a single-line textfield on mobile platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: testValueB);
        final bool isTargetPlatformApple = defaultTargetPlatform == TargetPlatform.iOS;

        await tester.pumpWidget(
          MaterialApp(home: Material(child: TextField(controller: controller))),
        );

        final Offset firstLinePos =
            tester.getTopLeft(find.byType(TextField)) + const Offset(50.0, 9.0);

        // Tap on text field to gain focus, and set selection somewhere on the first word.
        final TestGesture gesture = await tester.startGesture(firstLinePos, pointer: 7);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, isTargetPlatformApple ? 5 : 3);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 5);

        // Here we tap on same position again, to register a triple tap. This will select
        // the entire text field if it is a single-line field.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 74);
      },
      variant: TargetPlatformVariant.mobile(),
    );

    testWidgets(
      'Can triple click to select all on a single-line textfield on desktop platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: testValueA);

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(dragStartBehavior: DragStartBehavior.down, controller: controller),
            ),
          ),
        );

        final Offset firstLinePos = textOffsetToPosition(tester, 5);

        // Tap on text field to gain focus, and set selection to 'i|s' on the first line.
        final TestGesture gesture = await tester.startGesture(
          firstLinePos,
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 4);
        expect(controller.selection.extentOffset, 6);

        // Here we tap on same position again, to register a triple tap. This will select
        // the entire text field if it is a single-line field.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 72);
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets(
      'Can triple click to select a line on Linux',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueA);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueA);

        final Offset firstLinePos = textOffsetToPosition(tester, 5);

        // Tap on text field to gain focus, and set selection to 'i|s' on the first line.
        final TestGesture gesture = await tester.startGesture(
          firstLinePos,
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 4);
        expect(controller.selection.extentOffset, 6);

        // Here we tap on same position again, to register a triple tap. This will select
        // the paragraph at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 19);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.linux),
    );

    testWidgets(
      'Can triple click to select a paragraph',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueA);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueA);

        final Offset firstLinePos = textOffsetToPosition(tester, 5);

        // Tap on text field to gain focus, and set selection to 'i|s' on the first line.
        final TestGesture gesture = await tester.startGesture(
          firstLinePos,
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 4);
        expect(controller.selection.extentOffset, 6);

        // Here we tap on same position again, to register a triple tap. This will select
        // the paragraph at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 20);
      },
      variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.linux}),
    );

    testWidgets(
      'Can triple click + drag to select line by line on Linux',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueA);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueA);

        final Offset firstLinePos = textOffsetToPosition(tester, 5);
        final double lineHeight = findRenderEditable(tester).preferredLineHeight;

        // Tap on text field to gain focus, and set selection to 'i|s' on the first line.
        final TestGesture gesture = await tester.startGesture(
          firstLinePos,
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 4);
        expect(controller.selection.extentOffset, 6);

        // Here we tap on the same position again, to register a triple tap. This will select
        // the line at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 19);

        // Drag, down after the triple tap, to select line by line.
        // Moving down will extend the selection to the second line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 35);

        // Moving down will extend the selection to the third line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 2));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 54);

        // Moving down will extend the selection to the last line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 4));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 72);

        // Moving up will extend the selection to the third line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 2));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 54);

        // Moving up will extend the selection to the second line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 1));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 35);

        // Moving up will extend the selection to the first line.
        await gesture.moveTo(firstLinePos);
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 19);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.linux),
    );

    testWidgets(
      'Can triple click + drag to select paragraph by paragraph',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();

        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: TextField(
                dragStartBehavior: DragStartBehavior.down,
                controller: controller,
                maxLines: null,
              ),
            ),
          ),
        );

        await tester.enterText(find.byType(TextField), testValueA);
        await skipPastScrollingAnimation(tester);
        expect(controller.value.text, testValueA);

        final Offset firstLinePos = textOffsetToPosition(tester, 5);
        final double lineHeight = findRenderEditable(tester).preferredLineHeight;

        // Tap on text field to gain focus, and set selection to 'i|s' on the first line.
        final TestGesture gesture = await tester.startGesture(
          firstLinePos,
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);

        // Here we tap on same position again, to register a double tap. This will select
        // the word at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(controller.selection.baseOffset, 4);
        expect(controller.selection.extentOffset, 6);

        // Here we tap on the same position again, to register a triple tap. This will select
        // the paragraph at the tapped position.
        await gesture.down(firstLinePos);
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 20);

        // Drag, down after the triple tap, to select paragraph by paragraph.
        // Moving down will extend the selection to the second line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 36);

        // Moving down will extend the selection to the third line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 2));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 55);

        // Moving down will extend the selection to the last line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 4));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 72);

        // Moving up will extend the selection to the third line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight * 2));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 55);

        // Moving up will extend the selection to the second line.
        await gesture.moveTo(firstLinePos + Offset(0, lineHeight));
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 36);

        // Moving up will extend the selection to the first line.
        await gesture.moveTo(firstLinePos);
        await tester.pumpAndSettle();

        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 20);
      },
      variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.linux}),
    );

    testWidgets(
      'Going past triple click retains the selection on Apple platforms',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: testValueA);
        await tester.pumpWidget(
          MaterialApp(
            home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
          ),
        );

        final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));

        // First click moves the cursor to the point of the click, not the edge of
        // the clicked word.
        final TestGesture gesture = await tester.startGesture(
          textFieldStart + const Offset(210.0, 9.0),
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 13);

        // Second click selects the word.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        // Triple click selects the paragraph.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        // Clicking again retains the selection.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Clicking again retains the selection.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        // Clicking again retains the selection.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'Tap count resets when going past a triple tap on Android, Fuchsia, and Linux',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: testValueA);
        await tester.pumpWidget(
          MaterialApp(
            home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
          ),
        );

        final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));
        final bool platformSelectsByLine = defaultTargetPlatform == TargetPlatform.linux;

        // First click moves the cursor to the point of the click, not the edge of
        // the clicked word.
        final TestGesture gesture = await tester.startGesture(
          textFieldStart + const Offset(210.0, 9.0),
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 13);

        // Second click selects the word.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        // Triple click selects the paragraph.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(
          controller.selection,
          TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
        );

        // Clicking again moves the caret to the tapped position.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 13);

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Clicking again selects the word.
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        // Clicking again selects the paragraph.
        expect(
          controller.selection,
          TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
        );

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        // Clicking again moves the caret to the tapped position.
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 13);

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Clicking again selects the word.
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        // Clicking again selects the paragraph.
        expect(
          controller.selection,
          TextSelection(baseOffset: 0, extentOffset: platformSelectsByLine ? 19 : 20),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
        TargetPlatform.linux,
      }),
    );

    testWidgets(
      'Double click and triple click alternate on Windows',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController(text: testValueA);
        await tester.pumpWidget(
          MaterialApp(
            home: Material(child: Center(child: TextField(controller: controller, maxLines: null))),
          ),
        );

        final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));

        // First click moves the cursor to the point of the click, not the edge of
        // the clicked word.
        final TestGesture gesture = await tester.startGesture(
          textFieldStart + const Offset(210.0, 9.0),
          pointer: 7,
          kind: PointerDeviceKind.mouse,
        );
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 13);

        // Second click selects the word.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        // Triple click selects the paragraph.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        // Clicking again selects the word.
        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Clicking again selects the paragraph.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        // Clicking again selects the word.
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump(const Duration(milliseconds: 50));
        // Clicking again selects the paragraph.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Clicking again selects the word.
        expect(controller.selection, const TextSelection(baseOffset: 11, extentOffset: 15));

        await gesture.down(textFieldStart + const Offset(210.0, 9.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        // Clicking again selects the paragraph.
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 20));
      },
      variant: TargetPlatformVariant.only(TargetPlatform.windows),
    );
  });

  testWidgets(
    'double tap on top of cursor also selects word',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      // Tap to put the cursor after the "w".
      const int index = 3;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 500));
      expect(controller.selection, const TextSelection.collapsed(offset: index));

      // Double tap on the same location.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));

      // First tap doesn't change the selection
      expect(controller.selection, const TextSelection.collapsed(offset: index));

      // Second tap selects the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      // The toolbar shows up.
      expectMaterialToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'double double tap just shows the selection menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      // Double tap on the same location shows the selection menu.
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);

      // Double tap again keeps the selection menu visible.
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'double long press just shows the selection menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      // Long press shows the selection menu.
      await tester.longPressAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);

      // Long press again keeps the selection menu visible.
      await tester.longPressAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'A single tap hides the selection menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      // Long press shows the selection menu.
      await tester.longPress(find.byType(TextField));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);

      // Tap hides the selection menu.
      await tester.tap(find.byType(TextField));
      await tester.pump();
      expect(find.text('Paste'), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Drag selection hides the selection menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));
      final Offset midBlah1 = textOffsetToPosition(tester, 2);
      final Offset midBlah2 = textOffsetToPosition(tester, 8);

      // Right click the second word.
      final TestGesture gesture = await tester.startGesture(
        midBlah2,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // The toolbar is shown.
      expect(find.text('Paste'), findsOneWidget);

      // Drag the mouse to the first word.
      final TestGesture gesture2 = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture2.moveTo(midBlah2);
      await tester.pump();
      await gesture2.up();
      await tester.pumpAndSettle();

      // The toolbar is hidden.
      expect(find.text('Paste'), findsNothing);
    },
    variant: TargetPlatformVariant.desktop(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Long press on an autofocused field shows the selection menu',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );
      // This extra pump allows the selection set by autofocus to propagate to
      // the RenderEditable.
      await tester.pump();

      // Long press shows the selection menu.
      expect(find.text('Paste'), findsNothing);
      await tester.longPress(find.byType(TextField));
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'double tap hold selects word',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      final TestGesture gesture = await tester.startGesture(
        textfieldStart + const Offset(150.0, 9.0),
      );
      // Hold the press.
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      // The toolbar shows up.
      expectCupertinoToolbarForPartialSelection();

      await gesture.up();
      await tester.pump();

      // Still selected.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      // The toolbar is still showing.
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'tap after a double tap select is not affected',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      // On macOS, we select the precise position of the tap.
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset pPos = textOffsetToPosition(tester, 9); // Index of 'P|eel'.
      final Offset ePos = textOffsetToPosition(tester, 6); // Index of 'Atwate|r'

      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(
        controller.selection,
        isTargetPlatformMobile
            ? const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream)
            : const TextSelection.collapsed(offset: 9),
      );
      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(ePos);
      await tester.pump();

      // Plain collapsed selection at the edge of first word on iOS. In iOS 12,
      // the first tap after a double tap ends up putting the cursor at where
      // you tapped instead of the edge like every other single tap. This is
      // likely a bug in iOS 12 and not present in other versions.
      expect(controller.selection.isCollapsed, isTrue);
      expect(controller.selection.baseOffset, isTargetPlatformMobile ? 7 : 6);

      // No toolbar.
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press moves cursor to the exact long press position and shows toolbar when the field is focused',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );

      // This extra pump allows the selection set by autofocus to propagate to
      // the RenderEditable.
      await tester.pump();

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Collapsed cursor for iOS long press.
      expect(controller.selection, const TextSelection.collapsed(offset: 3));

      expectCupertinoToolbarForCollapsedSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press that starts on an unfocused TextField selects the word at the exact long press position and shows toolbar',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Collapsed cursor for iOS long press.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      // The toolbar shows up.
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press selects word and shows toolbar',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      // The toolbar shows up.
      expectMaterialToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'Toolbar hides on scroll start and re-appears on scroll end on Android and iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure ' * 20,
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final RenderEditable renderEditable = state.renderEditable;

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Long press should select word at position and show toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      final bool targetPlatformIsiOS = defaultTargetPlatform == TargetPlatform.iOS;
      final Finder contextMenuButtonFinder =
          targetPlatformIsiOS ? find.byType(CupertinoButton) : find.byType(TextButton);
      // Context menu shows 5 buttons: cut, copy, paste, select all, share on Android.
      // Context menu shows 6 buttons: cut, copy, paste, select all, lookup, share on iOS.
      final int numberOfContextMenuButtons = targetPlatformIsiOS ? 6 : 5;

      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );

      // Scroll to the left, the toolbar should be hidden since we are scrolling.
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(TextField)),
      );
      await tester.pump();
      await gesture.moveTo(tester.getBottomLeft(find.byType(TextField)));
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);

      // Scroll back to center, the toolbar should still be hidden since
      // we are still scrolling.
      await gesture.moveTo(tester.getCenter(find.byType(TextField)));
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);

      // Release finger to end scroll, toolbar should now be visible.
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'Toolbar hides on parent scrollable scroll start and re-appears on scroll end on Android and iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure ' * 20,
      );
      final Key key1 = UniqueKey();
      final Key key2 = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: ListView(
                children: <Widget>[
                  Container(height: 400, key: key1),
                  TextField(controller: controller),
                  Container(height: 1000, key: key2),
                ],
              ),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final RenderEditable renderEditable = state.renderEditable;

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Long press should select word at position and show toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      final bool targetPlatformIsiOS = defaultTargetPlatform == TargetPlatform.iOS;
      final Finder contextMenuButtonFinder =
          targetPlatformIsiOS ? find.byType(CupertinoButton) : find.byType(TextButton);
      // Context menu shows 5 buttons: cut, copy, paste, select all, share on Android.
      // Context menu shows 6 buttons: cut, copy, paste, select all, lookup, share on iOS.
      final int numberOfContextMenuButtons = targetPlatformIsiOS ? 6 : 5;
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );

      // Scroll down, the toolbar should be hidden since we are scrolling.
      final TestGesture gesture = await tester.startGesture(tester.getBottomLeft(find.byKey(key1)));
      await tester.pump();
      await gesture.moveTo(tester.getTopLeft(find.byKey(key1)));
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);

      // Release finger to end scroll, toolbar should now be visible.
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'Toolbar can re-appear after being scrolled out of view on Android and iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure ' * 20,
      );
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(controller: controller, scrollController: scrollController),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final RenderEditable renderEditable = state.renderEditable;

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      expect(renderEditable.selectionStartInViewport.value, false);
      expect(renderEditable.selectionEndInViewport.value, false);

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Long press should select word at position and show toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      final bool targetPlatformIsiOS = defaultTargetPlatform == TargetPlatform.iOS;
      final Finder contextMenuButtonFinder =
          targetPlatformIsiOS ? find.byType(CupertinoButton) : find.byType(TextButton);
      // Context menu shows 5 buttons: cut, copy, paste, select all, share on Android.
      // Context menu shows 6 buttons: cut, copy, paste, select all, lookup, share on iOS.
      final int numberOfContextMenuButtons = targetPlatformIsiOS ? 6 : 5;

      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);

      // Scroll to the end so the selection is no longer visible. This should
      // hide the toolbar, but schedule it to be shown once the selection is
      // visible again.
      scrollController.animateTo(
        500.0,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);
      expect(renderEditable.selectionStartInViewport.value, false);
      expect(renderEditable.selectionEndInViewport.value, false);

      // Scroll to the beginning where the selection is in view
      // and the toolbar should show again.
      scrollController.animateTo(
        0.0,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);

      final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 0));
      await tester.pump();
      await gesture.up();
      await gesture.down(textOffsetToPosition(tester, 0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Double tap should select word at position and show toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);

      // Scroll to the end so the selection is no longer visible. This should
      // hide the toolbar, but schedule it to be shown once the selection is
      // visible again.
      scrollController.animateTo(
        500.0,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);
      expect(renderEditable.selectionStartInViewport.value, false);
      expect(renderEditable.selectionEndInViewport.value, false);

      // Tap to change the selection. This will invalidate the scheduled
      // toolbar.
      await gesture.down(tester.getCenter(find.byType(TextField)));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Scroll to the beginning where the selection was previously
      // and the toolbar should not show because it was invalidated.
      scrollController.animateTo(
        0.0,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(contextMenuButtonFinder, findsNothing);
      expect(renderEditable.selectionStartInViewport.value, false);
      expect(renderEditable.selectionEndInViewport.value, false);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'Toolbar can re-appear after parent scrollable scrolls selection out of view on Android and iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      final Key key1 = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: ListView(
                controller: scrollController,
                children: <Widget>[
                  TextField(controller: controller),
                  Container(height: 1500.0, key: key1),
                ],
              ),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final RenderEditable renderEditable = state.renderEditable;

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.longPressAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();

      // Long press should select word at position and show toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      final bool targetPlatformIsiOS = defaultTargetPlatform == TargetPlatform.iOS;
      final Finder contextMenuButtonFinder =
          targetPlatformIsiOS ? find.byType(CupertinoButton) : find.byType(TextButton);
      // Context menu shows 5 buttons: cut, copy, paste, select all, share on Android.
      // Context menu shows 6 buttons: cut, copy, paste, select all, lookup, share on iOS.
      final int numberOfContextMenuButtons = targetPlatformIsiOS ? 6 : 5;
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );

      // Scroll down, the TextField should no longer be in the viewport.
      scrollController.animateTo(
        scrollController.position.maxScrollExtent,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(find.byType(TextField), findsNothing);
      expect(contextMenuButtonFinder, findsNothing);

      // Scroll back up so the TextField is inside the viewport.
      scrollController.animateTo(
        0.0,
        duration: const Duration(milliseconds: 100),
        curve: Curves.linear,
      );
      await tester.pumpAndSettle();
      expect(find.byType(TextField), findsOneWidget);
      expect(
        contextMenuButtonFinder,
        isContextMenuProvidedByPlatform ? findsNothing : findsNWidgets(numberOfContextMenuButtons),
      );
      expect(renderEditable.selectionStartInViewport.value, true);
      expect(renderEditable.selectionEndInViewport.value, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'long press tap cannot initiate a double tap',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );

      // This extra pump is so autofocus can propagate to renderEditable.
      await tester.pump();

      final Offset ePos = textOffsetToPosition(tester, 6); // Index of 'Atwate|r'

      await tester.longPressAt(ePos);
      await tester.pumpAndSettle(const Duration(milliseconds: 50));

      // Tap slightly behind the previous tap to avoid tapping the context menu
      // on desktop.
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      final Offset secondTapPos = isTargetPlatformMobile ? ePos : ePos + const Offset(-1.0, 0.0);
      await tester.tapAt(secondTapPos);
      await tester.pump();

      // The cursor does not move and the toolbar is toggled.
      expect(controller.selection.isCollapsed, isTrue);
      expect(controller.selection.baseOffset, 6);

      // The toolbar from the long press is now dismissed by the second tap.
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press drag extends the selection to the word under the drag and shows toolbar on lift on non-Apple platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 18));
      await tester.pump(const Duration(milliseconds: 500));

      // Long press selects the word at the long presses position.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));
      // Cursor move doesn't trigger a toolbar initially.
      expectNoMaterialToolbar();

      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 35));
      // Still no toolbar.
      expectNoMaterialToolbar();

      // The selection is moved on a backwards drag.
      await gesture.moveBy(const Offset(-200, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 8));
      // Still no toolbar.
      expectNoMaterialToolbar();

      await gesture.moveBy(const Offset(-100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));
      // Still no toolbar.
      expectNoMaterialToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));
      // The toolbar now shows up.
      expectMaterialToolbarForPartialSelection();
    },
    variant: TargetPlatformVariant.all(
      excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.macOS},
    ),
  );

  testWidgets(
    'long press drag on a focused TextField moves the cursor under the drag and shows toolbar on lift',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );

      // This extra pump is so autofocus can propagate to renderEditable.
      await tester.pump();

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(
        textfieldStart + const Offset(50.0, 9.0),
      );
      await tester.pump(const Duration(milliseconds: 500));

      // Long press on iOS shows collapsed selection cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 3));
      // Cursor move doesn't trigger a toolbar initially.
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(50, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection.collapsed(offset: 6));
      // Still no toolbar.
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(50, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      // Still no toolbar.
      expectNoCupertinoToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      // The toolbar now shows up.
      expectCupertinoToolbarForCollapsedSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press drag on an unfocused TextField selects word-by-word and shows toolbar on lift',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(
        textfieldStart + const Offset(50.0, 9.0),
      );
      await tester.pump(const Duration(milliseconds: 500));

      // Long press on iOS shows collapsed selection cursor.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      // Cursor move doesn't trigger a toolbar initially.
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 12));
      // Still no toolbar.
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 23));
      // Still no toolbar.
      expectNoCupertinoToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 23));
      // The toolbar now shows up.
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press drag can edge scroll on non-Apple platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      final RenderEditable renderEditable = findRenderEditable(tester);

      List<TextSelectionPoint> lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // Just testing the text and making sure that the last character is off
      // the right side of the screen.
      expect(lastCharEndpoint[0].point.dx, 1056);

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(textfieldStart);
      await tester.pump(const Duration(milliseconds: 500));

      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 7, affinity: TextAffinity.upstream),
      );
      expectNoMaterialToolbar();

      await gesture.moveBy(const Offset(900, 5));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 59));
      // Keep moving out.
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 66));
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 66, affinity: TextAffinity.upstream),
      ); // We're at the edge now.
      expectNoMaterialToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 66, affinity: TextAffinity.upstream),
      );
      // The toolbar now shows up.
      expectMaterialToolbarForFullSelection();

      lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // The last character is now on screen near the right edge.
      expect(lastCharEndpoint[0].point.dx, moreOrLessEquals(798, epsilon: 1));

      final List<TextSelectionPoint> firstCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 0), // First character's position.
      );
      expect(firstCharEndpoint.length, 1);
      // The first character is now offscreen to the left.
      expect(firstCharEndpoint[0].point.dx, moreOrLessEquals(-257.0, epsilon: 1));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'long press drag can edge scroll on Apple platforms - unfocused TextField',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      final RenderEditable renderEditable = findRenderEditable(tester);

      List<TextSelectionPoint> lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // Just testing the test and making sure that the last character is off
      // the right side of the screen.
      expect(lastCharEndpoint[0].point.dx, 1056);

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(textfieldStart);
      await tester.pump(const Duration(milliseconds: 500));

      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 7, affinity: TextAffinity.upstream),
      );
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(900, 5));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 59));
      // Keep moving out.
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 66));
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 66, affinity: TextAffinity.upstream),
      ); // We're at the edge now.
      expectNoCupertinoToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(
        controller.selection,
        const TextSelection(baseOffset: 0, extentOffset: 66, affinity: TextAffinity.upstream),
      );
      // The toolbar now shows up.
      expectCupertinoToolbarForFullSelection();

      lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // The last character is now on screen near the right edge.
      expect(lastCharEndpoint[0].point.dx, moreOrLessEquals(798, epsilon: 1));

      final List<TextSelectionPoint> firstCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 0), // First character's position.
      );
      expect(firstCharEndpoint.length, 1);
      // The first character is now offscreen to the left.
      expect(firstCharEndpoint[0].point.dx, moreOrLessEquals(-257.0, epsilon: 1));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'long press drag can edge scroll on Apple platforms - focused TextField',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );

      // This extra pump is so autofocus can propagate to renderEditable.
      await tester.pump();

      final RenderEditable renderEditable = findRenderEditable(tester);

      List<TextSelectionPoint> lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // Just testing the test and making sure that the last character is off
      // the right side of the screen.
      expect(lastCharEndpoint[0].point.dx, 1056);

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final TestGesture gesture = await tester.startGesture(textfieldStart + const Offset(300, 5));
      await tester.pump(const Duration(milliseconds: 500));

      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 19, affinity: TextAffinity.upstream),
      );
      expectNoCupertinoToolbar();

      await gesture.moveBy(const Offset(600, 0));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection.collapsed(offset: 56));
      // Keep moving out.
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection.collapsed(offset: 62));
      await gesture.moveBy(const Offset(1, 0));
      await tester.pump();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 66, affinity: TextAffinity.upstream),
      ); // We're at the edge now.
      expectNoCupertinoToolbar();

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 66, affinity: TextAffinity.upstream),
      );
      // The toolbar now shows up.
      expectCupertinoToolbarForCollapsedSelection();

      lastCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 66), // Last character's position.
      );

      expect(lastCharEndpoint.length, 1);
      // The last character is now on screen near the right edge.
      expect(lastCharEndpoint[0].point.dx, moreOrLessEquals(798, epsilon: 1));

      final List<TextSelectionPoint> firstCharEndpoint = renderEditable.getEndpointsForSelection(
        const TextSelection.collapsed(offset: 0), // First character's position.
      );
      expect(firstCharEndpoint.length, 1);
      // The first character is now offscreen to the left.
      expect(firstCharEndpoint[0].point.dx, moreOrLessEquals(-257.0, epsilon: 1));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('mouse click and drag can edge scroll', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
    );
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
    );
    final Size screenSize = MediaQuery.of(tester.element(find.byType(TextField))).size;
    // Just testing the test and making sure that the last character is off
    // the right side of the screen.
    expect(textOffsetToPosition(tester, 66).dx, greaterThan(screenSize.width));

    final TestGesture gesture = await tester.startGesture(
      textOffsetToPosition(tester, 19),
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );

    await gesture.moveTo(textOffsetToPosition(tester, 56));
    // To the edge of the screen basically.
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 56));

    // Keep moving out.
    await gesture.moveTo(textOffsetToPosition(tester, 62));
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 62));
    await gesture.moveTo(textOffsetToPosition(tester, 66));
    await tester.pumpAndSettle();
    expect(
      controller.selection,
      const TextSelection(baseOffset: 19, extentOffset: 66),
    ); // We're at the edge now.
    expectNoCupertinoToolbar();

    await gesture.up();
    await tester.pumpAndSettle();

    // The selection isn't affected by the gesture lift.
    expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 66));

    // The last character is now on screen near the right edge.
    expect(
      textOffsetToPosition(tester, 66).dx,
      moreOrLessEquals(TestSemantics.fullScreen.width, epsilon: 2.0),
    );

    // The first character is now offscreen to the left.
    expect(textOffsetToPosition(tester, 0).dx, lessThan(-100.0));
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'keyboard selection change scrolls the field',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      // Just testing the test and making sure that the last character is off
      // the right side of the screen.
      expect(textOffsetToPosition(tester, 66).dx, 1056);

      await tester.tapAt(textOffsetToPosition(tester, 13));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 13));

      // Move to position 56 with the right arrow (near the edge of the screen).
      for (int i = 0; i < (56 - 13); i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      }
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        // arrowRight always sets the affinity to downstream.
        const TextSelection.collapsed(offset: 56),
      );

      // Keep moving out.
      for (int i = 0; i < (62 - 56); i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      }
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 62));
      for (int i = 0; i < (66 - 62); i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      }
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 66),
      ); // We're at the edge now.

      await tester.pumpAndSettle();

      // The last character is now on screen near the right edge.
      expect(
        textOffsetToPosition(tester, 66).dx,
        moreOrLessEquals(TestSemantics.fullScreen.width, epsilon: 2.0),
      );

      // The first character is now offscreen to the left.
      expect(textOffsetToPosition(tester, 0).dx, moreOrLessEquals(-257.0, epsilon: 1));
    },
    variant: TargetPlatformVariant.all(),
    skip: isBrowser, // [intended] Browser handles arrow keys differently.
  );

  testWidgets(
    'long press drag can edge scroll vertically',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text:
            'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neigse Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(child: TextField(autofocus: true, maxLines: 2, controller: controller)),
          ),
        ),
      );

      // This extra pump is so autofocus can propagate to renderEditable.
      await tester.pump();

      // Just testing the test and making sure that the last character is outside
      // the bottom of the field.
      final int textLength = controller.text.length;
      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      final double firstCharY = textOffsetToPosition(tester, 0).dy;
      expect(
        textOffsetToPosition(tester, textLength).dy,
        moreOrLessEquals(firstCharY + lineHeight * 2, epsilon: 1),
      );

      // Start long pressing on the first line.
      final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 19));
      await tester.pump(const Duration(milliseconds: 500));
      expect(controller.selection, const TextSelection.collapsed(offset: 19));
      await tester.pumpAndSettle();

      // Move down to the second line.
      await gesture.moveBy(Offset(0.0, lineHeight));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 65));

      // Still hasn't scrolled.
      expect(
        textOffsetToPosition(tester, 65).dy,
        moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
      );

      // Keep selecting down to the third and final line.
      await gesture.moveBy(Offset(0.0, lineHeight));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 110));

      // The last character is no longer three line heights down from the top of
      // the field, it's now only two line heights down, because it has scrolled
      // down by one line.
      expect(
        textOffsetToPosition(tester, 110).dy,
        moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
      );

      // Likewise, the first character is now scrolled out of the top of the field
      // by one line.
      expect(
        textOffsetToPosition(tester, 0).dy,
        moreOrLessEquals(firstCharY - lineHeight, epsilon: 1),
      );

      // End gesture and skip the magnifier hide animation, so it can release
      // resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'keyboard selection change scrolls the field vertically',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text:
            'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(maxLines: 2, controller: controller))),
        ),
      );

      // Just testing the test and making sure that the last character is outside
      // the bottom of the field.
      final int textLength = controller.text.length;
      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      final double firstCharY = textOffsetToPosition(tester, 0).dy;
      expect(
        textOffsetToPosition(tester, textLength).dy,
        moreOrLessEquals(firstCharY + lineHeight * 2, epsilon: 1),
      );

      await tester.tapAt(textOffsetToPosition(tester, 13));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 13));

      // Move down to the second line.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 59));

      // Still hasn't scrolled.
      expect(
        textOffsetToPosition(tester, 66).dy,
        moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
      );

      // Move down to the third and final line.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 104));

      // The last character is no longer three line heights down from the top of
      // the field, it's now only two line heights down, because it has scrolled
      // down by one line.
      expect(
        textOffsetToPosition(tester, textLength).dy,
        moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
      );

      // Likewise, the first character is now scrolled out of the top of the field
      // by one line.
      expect(
        textOffsetToPosition(tester, 0).dy,
        moreOrLessEquals(firstCharY - lineHeight, epsilon: 1),
      );
    },
    variant: TargetPlatformVariant.all(),
    skip: isBrowser, // [intended] Browser handles arrow keys differently.
  );

  testWidgets('mouse click and drag can edge scroll vertically', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text:
          'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges',
    );
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextField(maxLines: 2, controller: controller))),
      ),
    );

    // Just testing the test and making sure that the last character is outside
    // the bottom of the field.
    final int textLength = controller.text.length;
    final double lineHeight = findRenderEditable(tester).preferredLineHeight;
    final double firstCharY = textOffsetToPosition(tester, 0).dy;
    expect(
      textOffsetToPosition(tester, textLength).dy,
      moreOrLessEquals(firstCharY + lineHeight * 2, epsilon: 1),
    );

    // Start selecting on the first line.
    final TestGesture gesture = await tester.startGesture(
      textOffsetToPosition(tester, 19),
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );

    // Still hasn't scrolled.
    expect(
      textOffsetToPosition(tester, 60).dy,
      moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
    );

    // Select down to the second line.
    await gesture.moveBy(Offset(0.0, lineHeight));
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 65));

    // Still hasn't scrolled.
    expect(
      textOffsetToPosition(tester, 60).dy,
      moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
    );

    // Keep selecting down to the third and final line.
    await gesture.moveBy(Offset(0.0, lineHeight));
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection(baseOffset: 19, extentOffset: 110));

    // The last character is no longer three line heights down from the top of
    // the field, it's now only two line heights down, because it has scrolled
    // down by one line.
    expect(
      textOffsetToPosition(tester, textLength).dy,
      moreOrLessEquals(firstCharY + lineHeight, epsilon: 1),
    );

    // Likewise, the first character is now scrolled out of the top of the field
    // by one line.
    expect(
      textOffsetToPosition(tester, 0).dy,
      moreOrLessEquals(firstCharY - lineHeight, epsilon: 1),
    );
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'long tap after a double tap select is not affected',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      // On macOS, we select the precise position of the tap.
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset pPos = textOffsetToPosition(tester, 9); // Index of 'P|eel'
      final Offset ePos = textOffsetToPosition(tester, 6); // Index of 'Atwate|r'

      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor to the beginning of the second word.
      expect(
        controller.selection,
        isTargetPlatformMobile
            ? const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream)
            : const TextSelection.collapsed(offset: 9),
      );
      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 500));

      await tester.longPressAt(ePos);
      await tester.pumpAndSettle();

      // Plain collapsed selection at the exact tap position.
      expect(controller.selection, const TextSelection.collapsed(offset: 6));

      // The toolbar shows up.
      expectCupertinoToolbarForCollapsedSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'double tap after a long tap is not affected',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      // On iOS/iPadOS, during a tap we select the edge of the word closest to the tap.
      // On macOS, we select the precise position of the tap.
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(autofocus: true, controller: controller))),
        ),
      );

      // This extra pump is so autofocus can propagate to renderEditable.
      await tester.pump();

      // The second tap is slightly higher to avoid tapping the context menu on
      // desktop.
      final Offset pPos =
          textOffsetToPosition(tester, 9) + const Offset(0.0, -20.0); // Index of 'P|eel'
      final Offset wPos = textOffsetToPosition(tester, 3); // Index of 'Atw|ater'

      await tester.longPressAt(wPos);
      await tester.pumpAndSettle(const Duration(milliseconds: 50));
      expect(controller.selection, const TextSelection.collapsed(offset: 3));

      await tester.tapAt(pPos);
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(
        controller.selection,
        isTargetPlatformMobile
            ? const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream)
            : const TextSelection.collapsed(offset: 9),
      );
      await tester.tapAt(pPos);
      await tester.pumpAndSettle();

      // Double tap selection.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('double click after a click on desktop platforms', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
    );

    final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));

    final TestGesture gesture = await tester.startGesture(
      textFieldStart + const Offset(50.0, 9.0),
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(controller.selection, const TextSelection.collapsed(offset: 3));

    await gesture.down(textFieldStart + const Offset(150.0, 9.0));
    await tester.pump();
    await gesture.up();
    await tester.pump(const Duration(milliseconds: 50));
    // First click moved the cursor to the precise location, not the start of
    // the word.
    expect(controller.selection, const TextSelection.collapsed(offset: 9));

    // Double click selection.
    await gesture.down(textFieldStart + const Offset(150.0, 9.0));
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();
    expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
    // The text selection toolbar isn't shown on Mac without a right click.
    expectNoCupertinoToolbar();
  }, variant: TargetPlatformVariant.desktop());

  testWidgets(
    'double tap chains work',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 7, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(textfieldStart + const Offset(50.0, 9.0));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectCupertinoToolbarForPartialSelection();

      // Double tap selecting the same word somewhere else is fine.
      await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap hides the toolbar and retains the selection.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectNoCupertinoToolbar();

      // Second tap shows the toolbar and retains the selection.
      await tester.tapAt(textfieldStart + const Offset(100.0, 9.0));
      // Wait for the consecutive tap timer to timeout so the next
      // tap is not detected as a triple tap.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectCupertinoToolbarForPartialSelection();

      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor and hid the toolbar.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );
      expectNoCupertinoToolbar();
      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'double click chains work',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset textFieldStart = tester.getTopLeft(find.byType(TextField));

      // First click moves the cursor to the point of the click, not the edge of
      // the clicked word.
      final TestGesture gesture = await tester.startGesture(
        textFieldStart + const Offset(50.0, 9.0),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      expect(controller.selection, const TextSelection.collapsed(offset: 3));

      // Second click selects.
      await gesture.down(textFieldStart + const Offset(50.0, 9.0));
      await tester.pump();
      await gesture.up();
      // Wait for the consecutive tap timer to timeout so the next
      // tap is not detected as a triple tap.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectNoCupertinoToolbar();

      // Double tap selecting the same word somewhere else is fine.
      await gesture.down(textFieldStart + const Offset(100.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 6));
      await gesture.down(textFieldStart + const Offset(100.0, 9.0));
      await tester.pump();
      await gesture.up();
      // Wait for the consecutive tap timer to timeout so the next
      // tap is not detected as a triple tap.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectNoCupertinoToolbar();

      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      await gesture.down(textFieldStart + const Offset(150.0, 9.0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
      expectNoCupertinoToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets(
    'double tapping a space selects the previous word on iOS',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: ' blah blah  \n  blah');
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(maxLines: null, controller: controller))),
        ),
      );

      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, -1);
      expect(controller.value.selection.extentOffset, -1);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 19));
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 19);
      expect(controller.value.selection.extentOffset, 19);

      // Double tapping does the same thing.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.extentOffset, 5);
      expect(controller.value.selection.baseOffset, 1);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 19));
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 19);
      expect(controller.value.selection.extentOffset, 19);

      // Double tapping does the same thing for the first space.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 0);
      expect(controller.value.selection.extentOffset, 1);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 19));
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 19);
      expect(controller.value.selection.extentOffset, 19);

      // Double tapping the last space selects all previous contiguous spaces on
      // both lines and the previous word.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 14));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 14));
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 6);
      expect(controller.value.selection.extentOffset, 14);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'selecting a space selects the space on non-iOS platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: ' blah blah');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, -1);
      expect(controller.value.selection.extentOffset, -1);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 10));
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 10);
      expect(controller.value.selection.extentOffset, 10);

      // Double tapping the second space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 5);
      expect(controller.value.selection.extentOffset, 6);

      // Tap at the end of the text to move the selection to the end. On some
      // platforms, the context menu "Cut" button blocks this tap, so move it out
      // of the way by an Offset.
      await tester.tapAt(textOffsetToPosition(tester, 10) + const Offset(200.0, 0.0));
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 10);
      expect(controller.value.selection.extentOffset, 10);

      // Double tapping the second space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 0);
      expect(controller.value.selection.extentOffset, 1);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.linux,
      TargetPlatform.fuchsia,
      TargetPlatform.android,
    }),
  );

  testWidgets(
    'selecting a space selects the space on Desktop platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: ' blah blah');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, -1);
      expect(controller.value.selection.extentOffset, -1);

      // Put the cursor at the end of the field.
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 10),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 10);
      expect(controller.value.selection.extentOffset, 10);

      // Double clicking the second space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.down(textOffsetToPosition(tester, 5));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      await gesture.down(textOffsetToPosition(tester, 5));
      await tester.pump();
      await gesture.up();
      // Wait for the consecutive tap timer to timeout so our next tap is not
      // detected as a triple tap.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 5);
      expect(controller.value.selection.extentOffset, 6);

      // Put the cursor at the end of the field.
      await gesture.down(textOffsetToPosition(tester, 10));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 10);
      expect(controller.value.selection.extentOffset, 10);

      // Double tapping the second space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.down(textOffsetToPosition(tester, 0));
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 50));
      await gesture.down(textOffsetToPosition(tester, 0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.value.selection, isNotNull);
      expect(controller.value.selection.baseOffset, 0);
      expect(controller.value.selection.extentOffset, 1);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets(
    'Force press does not set selection on Android or Fuchsia touch devices',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final Offset offset = tester.getTopLeft(find.byType(TextField)) + const Offset(150.0, 9.0);

      final int pointerValue = tester.nextPointer;
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          pressure: 0.0,
          pressureMax: 6.0,
          pressureMin: 0.0,
        ),
      );
      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: offset + const Offset(150.0, 9.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );

      await gesture.up();
      await tester.pump();

      // We don't want this gesture to select any word on Android.
      expect(controller.selection, const TextSelection.collapsed(offset: -1));
      expectNoMaterialToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'Force press sets selection on desktop platforms that do not support it',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final Offset offset = tester.getTopLeft(find.byType(TextField)) + const Offset(150.0, 9.0);

      final int pointerValue = tester.nextPointer;
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          pressure: 0.0,
          pressureMax: 6.0,
          pressureMin: 0.0,
        ),
      );
      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: offset + const Offset(150.0, 9.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );

      await gesture.up();
      await tester.pump();

      // We don't want this gesture to select any word on Android.
      expect(controller.selection, const TextSelection.collapsed(offset: 9));
      expectNoMaterialToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'force press selects word',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final int pointerValue = tester.nextPointer;
      final Offset offset = textfieldStart + const Offset(150.0, 9.0);
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          pressure: 0.0,
          pressureMax: 6.0,
          pressureMin: 0.0,
        ),
      );

      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: textfieldStart + const Offset(150.0, 9.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );
      // We expect the force press to select a word at the given location.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      await gesture.up();
      await tester.pumpAndSettle();
      expectCupertinoToolbarForPartialSelection();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'tap on non-force-press-supported devices work',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(Container(key: GlobalKey()));
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      final int pointerValue = tester.nextPointer;
      final Offset offset = textfieldStart + const Offset(150.0, 9.0);
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          // iPhone 6 and below report 0 across the board.
          pressure: 0,
          pressureMax: 0,
          pressureMin: 0,
        ),
      );

      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: textfieldStart + const Offset(150.0, 9.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );
      await gesture.up();
      // The event should fallback to a normal tap and move the cursor.
      // Single taps selects the edge of the word.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );

      await tester.pump();
      // Single taps shouldn't trigger the toolbar.
      expectNoCupertinoToolbar();

      // TODO(gspencergoog): Add in TargetPlatform.macOS in the line below when we figure out what global state is leaking.
      // https://github.com/flutter/flutter/issues/43445
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('default TextField debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    const TextField().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('TextField implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    // Not checking controller, inputFormatters, focusNode
    const TextField(
      decoration: InputDecoration(labelText: 'foo'),
      keyboardType: TextInputType.text,
      textInputAction: TextInputAction.done,
      style: TextStyle(color: Color(0xff00ff00)),
      textAlign: TextAlign.end,
      textDirection: TextDirection.ltr,
      autofocus: true,
      autocorrect: false,
      maxLines: 10,
      maxLength: 100,
      maxLengthEnforcement: MaxLengthEnforcement.none,
      smartDashesType: SmartDashesType.disabled,
      smartQuotesType: SmartQuotesType.disabled,
      enabled: false,
      cursorWidth: 1.0,
      cursorHeight: 1.0,
      cursorRadius: Radius.zero,
      cursorColor: Color(0xff00ff00),
      keyboardAppearance: Brightness.dark,
      scrollPadding: EdgeInsets.zero,
      scrollPhysics: ClampingScrollPhysics(),
      enableInteractiveSelection: false,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'enabled: false',
      'decoration: InputDecoration(labelText: "foo")',
      'style: TextStyle(inherit: true, color: ${const Color(0xff00ff00)})',
      'autofocus: true',
      'autocorrect: false',
      'smartDashesType: disabled',
      'smartQuotesType: disabled',
      'maxLines: 10',
      'maxLength: 100',
      'maxLengthEnforcement: none',
      'textInputAction: done',
      'textAlign: end',
      'textDirection: ltr',
      'cursorWidth: 1.0',
      'cursorHeight: 1.0',
      'cursorRadius: Radius.circular(0.0)',
      'cursorColor: ${const Color(0xff00ff00)}',
      'keyboardAppearance: Brightness.dark',
      'scrollPadding: EdgeInsets.zero',
      'selection disabled',
      'scrollPhysics: ClampingScrollPhysics',
    ]);
  });

  testWidgets('strut basic single line', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android),
        home: const Material(child: Center(child: TextField())),
      ),
    );

    expect(
      tester.getSize(find.byType(TextField)),
      // The TextField will be as tall as the decoration (24) plus the metrics
      // from the default TextStyle of the theme (16), or 40 altogether.
      // Because this is less than the kMinInteractiveDimension, it will be
      // increased to that value (48).
      const Size(800, kMinInteractiveDimension),
    );
  });

  testWidgets('strut TextStyle increases height', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(child: Center(child: TextField(style: TextStyle(fontSize: 20)))),
      ),
    );

    expect(
      tester.getSize(find.byType(TextField)),
      // Strut should inherit the TextStyle.fontSize by default and produce the
      // same height as if it were disabled.
      const Size(800, kMinInteractiveDimension), // Because 44 < 48.
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android),
        home: const Material(
          child: Center(
            child: TextField(style: TextStyle(fontSize: 20), strutStyle: StrutStyle.disabled),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(TextField)),
      // The height here should match the previous version with strut enabled.
      const Size(800, kMinInteractiveDimension), // Because 44 < 48.
    );
  });

  testWidgets('strut basic multi line', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(child: Center(child: TextField(maxLines: 6))),
      ),
    );

    expect(
      tester.getSize(find.byType(TextField)),
      // The height should be the input decoration (24) plus 6x the strut height (16).
      const Size(800, 120),
    );
  });

  testWidgets('strut no force small strut', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: TextField(
              maxLines: 6,
              strutStyle: StrutStyle(
                // The small strut is overtaken by the larger
                // TextStyle fontSize.
                fontSize: 5,
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(TextField)),
      // When the strut's height is smaller than TextStyle's and forceStrutHeight
      // is disabled, then the TextStyle takes precedence. Should be the same height
      // as 'strut basic multi line'.
      const Size(800, 120),
    );
  });

  testWidgets(
    'strut no force large strut',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
          home: const Material(
            child: Center(child: TextField(maxLines: 6, strutStyle: StrutStyle(fontSize: 25))),
          ),
        ),
      );

      expect(
        tester.getSize(find.byType(TextField)),
        // When the strut's height is larger than TextStyle's and forceStrutHeight
        // is disabled, then the StrutStyle takes precedence.
        const Size(800, 174),
      );
    },
    skip: isBrowser, // TODO(mdebbar): https://github.com/flutter/flutter/issues/32243
  );

  testWidgets(
    'strut height override',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
          home: const Material(
            child: Center(
              child: TextField(
                maxLines: 3,
                strutStyle: StrutStyle(fontSize: 8, forceStrutHeight: true),
              ),
            ),
          ),
        ),
      );

      expect(
        tester.getSize(find.byType(TextField)),
        // The smaller font size of strut make the field shorter than normal.
        const Size(800, 48),
      );
    },
    skip: isBrowser, // TODO(mdebbar): https://github.com/flutter/flutter/issues/32243
  );

  testWidgets(
    'strut forces field taller',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
          home: const Material(
            child: Center(
              child: TextField(
                maxLines: 3,
                style: TextStyle(fontSize: 10),
                strutStyle: StrutStyle(fontSize: 18, forceStrutHeight: true),
              ),
            ),
          ),
        ),
      );

      expect(
        tester.getSize(find.byType(TextField)),
        // When the strut fontSize is larger than a provided TextStyle, the
        // strut's height takes precedence.
        const Size(800, 78),
      );
    },
    skip: isBrowser, // TODO(mdebbar): https://github.com/flutter/flutter/issues/32243
  );

  testWidgets('Caret center position', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: Theme(
          data: ThemeData(useMaterial3: false),
          child: const SizedBox(
            width: 300.0,
            child: TextField(textAlign: TextAlign.center, decoration: null),
          ),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    await tester.enterText(find.byType(TextField), 'abcd');
    await tester.pump();

    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 4)).topLeft,
    );
    expect(topLeft.dx, equals(431));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 3)).topLeft,
    );
    expect(topLeft.dx, equals(415));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );
    expect(topLeft.dx, equals(399));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 1)).topLeft,
    );
    expect(topLeft.dx, equals(383));
  });

  testWidgets('Caret indexes into trailing whitespace center align', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: Theme(
          data: ThemeData(useMaterial3: false),
          child: const SizedBox(
            width: 300.0,
            child: TextField(textAlign: TextAlign.center, decoration: null),
          ),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    await tester.enterText(find.byType(TextField), 'abcd    ');
    await tester.pump();

    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 7)).topLeft,
    );
    expect(topLeft.dx, equals(479));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 8)).topLeft,
    );
    expect(topLeft.dx, equals(495));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 4)).topLeft,
    );
    expect(topLeft.dx, equals(431));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 3)).topLeft,
    );
    expect(topLeft.dx, equals(415)); // Should be same as equivalent in 'Caret center position'

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );
    expect(topLeft.dx, equals(399)); // Should be same as equivalent in 'Caret center position'

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 1)).topLeft,
    );
    expect(topLeft.dx, equals(383)); // Should be same as equivalent in 'Caret center position'
  });

  testWidgets('selection handles are rendered and not faded away', (WidgetTester tester) async {
    const String testText = 'lorem ipsum';
    final TextEditingController controller = _textEditingController(text: testText);

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable renderEditable = state.renderEditable;

    await tester.tapAt(const Offset(20, 10));
    renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pumpAndSettle();

    final List<FadeTransition> transitions =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay',
              ),
              matching: find.byType(FadeTransition),
            )
            .evaluate()
            .map((Element e) => e.widget)
            .cast<FadeTransition>()
            .toList();
    expect(transitions.length, 2);
    final FadeTransition left = transitions[0];
    final FadeTransition right = transitions[1];
    expect(left.opacity.value, equals(1.0));
    expect(right.opacity.value, equals(1.0));
  });

  testWidgets(
    'iOS selection handles are rendered and not faded away',
    (WidgetTester tester) async {
      const String testText = 'lorem ipsum';
      final TextEditingController controller = _textEditingController(text: testText);

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final RenderEditable renderEditable =
          tester.state<EditableTextState>(find.byType(EditableText)).renderEditable;

      await tester.tapAt(const Offset(20, 10));
      renderEditable.selectWord(cause: SelectionChangedCause.longPress);
      await tester.pumpAndSettle();

      final List<FadeTransition> transitions =
          find
              .byType(FadeTransition)
              .evaluate()
              .map((Element e) => e.widget)
              .cast<FadeTransition>()
              .toList();
      expect(transitions.length, 2);
      final FadeTransition left = transitions[0];
      final FadeTransition right = transitions[1];

      expect(left.opacity.value, equals(1.0));
      expect(right.opacity.value, equals(1.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'iPad Scribble selection change shows selection handles',
    (WidgetTester tester) async {
      const String testText = 'lorem ipsum';
      final TextEditingController controller = _textEditingController(text: testText);

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      await tester.showKeyboard(find.byType(EditableText));
      await tester.testTextInput.startScribbleInteraction();
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: testText,
          selection: TextSelection(baseOffset: 2, extentOffset: 7),
        ),
      );
      await tester.pumpAndSettle();

      final List<FadeTransition> transitions =
          find
              .byType(FadeTransition)
              .evaluate()
              .map((Element e) => e.widget)
              .cast<FadeTransition>()
              .toList();
      expect(transitions.length, 2);
      final FadeTransition left = transitions[0];
      final FadeTransition right = transitions[1];

      expect(left.opacity.value, equals(1.0));
      expect(right.opacity.value, equals(1.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets('Tap shows handles but not toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Tap to trigger the text field.
    await tester.tap(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
  });

  testWidgets('Tap in empty text field does not show handles nor toolbar', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Tap to trigger the text field.
    await tester.tap(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
  });

  testWidgets('Long press shows handles and toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Long press to trigger the text field.
    await tester.longPress(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(
      editableText.selectionOverlay!.toolbarIsVisible,
      isContextMenuProvidedByPlatform ? isFalse : isTrue,
    );
  });

  testWidgets('Long press in empty text field shows handles and toolbar', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Tap to trigger the text field.
    await tester.longPress(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(
      editableText.selectionOverlay!.toolbarIsVisible,
      isContextMenuProvidedByPlatform ? isFalse : isTrue,
    );
  });

  testWidgets('Double tap shows handles and toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Double tap to trigger the text field.
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.tap(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(
      editableText.selectionOverlay!.toolbarIsVisible,
      isContextMenuProvidedByPlatform ? isFalse : isTrue,
    );
  });

  testWidgets('Double tap in empty text field shows toolbar but not handles', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController();

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Double tap to trigger the text field.
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.tap(find.byType(TextField));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
    expect(
      editableText.selectionOverlay!.toolbarIsVisible,
      isContextMenuProvidedByPlatform ? isFalse : isTrue,
    );
  });

  testWidgets('Mouse tap does not show handles nor toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Long press to trigger the text field.
    final Offset textFieldPos = tester.getCenter(find.byType(TextField));
    final TestGesture gesture = await tester.startGesture(
      textFieldPos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump();
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('Mouse long press does not show handles nor toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Long press to trigger the text field.
    final Offset textFieldPos = tester.getCenter(find.byType(TextField));
    final TestGesture gesture = await tester.startGesture(
      textFieldPos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('Mouse double tap does not show handles nor toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Double tap to trigger the text field.
    final Offset textFieldPos = tester.getCenter(find.byType(TextField));
    final TestGesture gesture = await tester.startGesture(
      textFieldPos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(milliseconds: 50));
    await gesture.up();
    await tester.pump();
    await gesture.down(textFieldPos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('Does not show handles when updated from the web engine', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Interact with the text field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    final TestGesture gesture = await tester.startGesture(
      topLeft + const Offset(0.0, 5.0),
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(milliseconds: 50));
    await gesture.up();
    await tester.pumpAndSettle();

    final EditableTextState state = tester.state(find.byType(EditableText));
    expect(state.selectionOverlay!.handlesAreVisible, isFalse);
    expect(controller.selection, const TextSelection.collapsed(offset: 0));

    if (kIsWeb) {
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: 'abc def ghi',
          selection: TextSelection(baseOffset: 2, extentOffset: 7),
        ),
      );
      // Wait for all the `setState` calls to be flushed.
      await tester.pumpAndSettle();
      expect(
        state.currentTextEditingValue.selection,
        const TextSelection(baseOffset: 2, extentOffset: 7),
      );
      expect(state.selectionOverlay!.handlesAreVisible, isFalse);
    }
  });

  testWidgets('Tapping selection handles toggles the toolbar', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(text: 'abc def ghi');

    await tester.pumpWidget(MaterialApp(home: Material(child: TextField(controller: controller))));

    // Tap to position the cursor and show the selection handles.
    final Offset ePos = textOffsetToPosition(tester, 5); // Index of 'e'.
    await tester.tapAt(ePos, pointer: 7);
    await tester.pumpAndSettle();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    expect(endpoints.length, 1);

    // Tap the handle to show the toolbar.
    final Offset handlePos = endpoints[0].point + const Offset(0.0, 1.0);
    await tester.tapAt(handlePos, pointer: 7);
    await tester.pump();
    expect(
      editableText.selectionOverlay!.toolbarIsVisible,
      isContextMenuProvidedByPlatform ? isFalse : isTrue,
    );

    // Tap the handle again to hide the toolbar.
    await tester.tapAt(handlePos, pointer: 7);
    await tester.pump();
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
  });

  testWidgets(
    'when TextField would be blocked by keyboard, it is shown with enough space for the selection handle',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Center(
              child: ListView(
                controller: scrollController,
                children: <Widget>[
                  Container(height: 579), // Push field almost off screen.
                  const TextField(),
                  Container(height: 1000),
                ],
              ),
            ),
          ),
        ),
      );

      // Tap the TextField to put the cursor into it and bring it into view.
      expect(scrollController.offset, 0.0);
      await tester.tapAt(tester.getTopLeft(find.byType(TextField)));
      await tester.pumpAndSettle();

      // The ListView has scrolled to keep the TextField and cursor handle
      // visible.
      expect(scrollController.offset, 50.0);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/74566
  testWidgets(
    'TextField and last input character are visible on the screen when the cursor is not shown',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      final ScrollController textFieldScrollController = ScrollController();
      addTearDown(() {
        scrollController.dispose();
        textFieldScrollController.dispose();
      });

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Center(
              child: ListView(
                controller: scrollController,
                children: <Widget>[
                  Container(height: 579), // Push field almost off screen.
                  TextField(scrollController: textFieldScrollController, showCursor: false),
                  Container(height: 1000),
                ],
              ),
            ),
          ),
        ),
      );

      // Tap the TextField to bring it into view.
      expect(scrollController.offset, 0.0);
      await tester.tapAt(tester.getTopLeft(find.byType(TextField)));
      await tester.pumpAndSettle();

      // The ListView has scrolled to keep the TextField visible.
      expect(scrollController.offset, 50.0);
      expect(textFieldScrollController.offset, 0.0);

      // After entering some long text, the last input character remains on the screen.
      final String testValue = 'I love Flutter!' * 10;
      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: testValue,
          selection: TextSelection.collapsed(offset: testValue.length),
        ),
      );
      await tester.pump();
      await tester.pumpAndSettle(); // Text scroll animation.

      expect(textFieldScrollController.offset, 1602.0);
    },
  );

  group('height', () {
    testWidgets('By default, TextField is at least kMinInteractiveDimension high', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(theme: ThemeData(), home: const Scaffold(body: Center(child: TextField()))),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(TextField));
      expect(renderBox.size.height, greaterThanOrEqualTo(kMinInteractiveDimension));
    });

    testWidgets(
      "When text is very small, TextField still doesn't go below kMinInteractiveDimension height",
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(),
            home: const Scaffold(body: Center(child: TextField(style: TextStyle(fontSize: 2.0)))),
          ),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(TextField));
        expect(renderBox.size.height, kMinInteractiveDimension);
      },
    );

    testWidgets('When isDense, TextField can go below kMinInteractiveDimension height', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(),
          home: const Scaffold(
            body: Center(child: TextField(decoration: InputDecoration(isDense: true))),
          ),
        ),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(TextField));
      expect(renderBox.size.height, lessThan(kMinInteractiveDimension));
    });

    group('intrinsics', () {
      Widget buildTest({required bool isDense}) {
        return MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              slivers: <Widget>[
                SliverFillRemaining(
                  hasScrollBody: false,
                  child: Column(
                    children: <Widget>[
                      TextField(decoration: InputDecoration(isDense: isDense)),
                      Container(height: 1000),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
      }

      testWidgets('By default, intrinsic height is at least kMinInteractiveDimension high', (
        WidgetTester tester,
      ) async {
        // Regression test for https://github.com/flutter/flutter/issues/54729
        // If the intrinsic height does not match that of the height after
        // performLayout, this will fail.
        await tester.pumpWidget(buildTest(isDense: false));
      });

      testWidgets('When isDense, intrinsic height can go below kMinInteractiveDimension height', (
        WidgetTester tester,
      ) async {
        // Regression test for https://github.com/flutter/flutter/issues/54729
        // If the intrinsic height does not match that of the height after
        // performLayout, this will fail.
        await tester.pumpWidget(buildTest(isDense: true));
      });
    });
  });
  testWidgets("Arrow keys don't move input focus", (WidgetTester tester) async {
    final TextEditingController controller1 = _textEditingController();
    final TextEditingController controller2 = _textEditingController();
    final TextEditingController controller3 = _textEditingController();
    final TextEditingController controller4 = _textEditingController();
    final TextEditingController controller5 = _textEditingController();
    final FocusNode focusNode1 = FocusNode(debugLabel: 'Field 1');
    final FocusNode focusNode2 = FocusNode(debugLabel: 'Field 2');
    final FocusNode focusNode3 = FocusNode(debugLabel: 'Field 3');
    final FocusNode focusNode4 = FocusNode(debugLabel: 'Field 4');
    final FocusNode focusNode5 = FocusNode(debugLabel: 'Field 5');

    addTearDown(() {
      focusNode1.dispose();
      focusNode2.dispose();
      focusNode3.dispose();
      focusNode4.dispose();
      focusNode5.dispose();
    });

    // Lay out text fields in a "+" formation, and focus the center one.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(),
        home: Scaffold(
          body: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                SizedBox(
                  width: 100.0,
                  child: TextField(controller: controller1, focusNode: focusNode1),
                ),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    SizedBox(
                      width: 100.0,
                      child: TextField(controller: controller2, focusNode: focusNode2),
                    ),
                    SizedBox(
                      width: 100.0,
                      child: TextField(controller: controller3, focusNode: focusNode3),
                    ),
                    SizedBox(
                      width: 100.0,
                      child: TextField(controller: controller4, focusNode: focusNode4),
                    ),
                  ],
                ),
                SizedBox(
                  width: 100.0,
                  child: TextField(controller: controller5, focusNode: focusNode5),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    focusNode3.requestFocus();
    await tester.pump();
    expect(focusNode3.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(focusNode3.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(focusNode3.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(focusNode3.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(focusNode3.hasPrimaryFocus, isTrue);
  });

  testWidgets('Scrolling shortcuts are disabled in text fields', (WidgetTester tester) async {
    bool scrollInvoked = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Actions(
          actions: <Type, Action<Intent>>{
            ScrollIntent: CallbackAction<ScrollIntent>(
              onInvoke: (Intent intent) {
                scrollInvoked = true;
                return null;
              },
            ),
          },
          child: Material(
            child: ListView(
              children: const <Widget>[
                Padding(padding: EdgeInsets.symmetric(vertical: 200)),
                TextField(),
                Padding(padding: EdgeInsets.symmetric(vertical: 800)),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    expect(scrollInvoked, isFalse);

    // Set focus on the text field.
    await tester.tapAt(tester.getTopLeft(find.byType(TextField)));

    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    expect(scrollInvoked, isFalse);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(scrollInvoked, isFalse);
  });

  testWidgets("A buildCounter that returns null doesn't affect the size of the TextField", (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/44909

    final GlobalKey textField1Key = GlobalKey();
    final GlobalKey textField2Key = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Column(
            children: <Widget>[
              TextField(key: textField1Key),
              TextField(
                key: textField2Key,
                maxLength: 1,
                buildCounter:
                    (
                      BuildContext context, {
                      required int currentLength,
                      required bool isFocused,
                      int? maxLength,
                    }) => null,
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();
    final Size textFieldSize1 = tester.getSize(find.byKey(textField1Key));
    final Size textFieldSize2 = tester.getSize(find.byKey(textField2Key));

    expect(textFieldSize1, equals(textFieldSize2));
  });

  testWidgets('The selection menu displays in an Overlay without error', (
    WidgetTester tester,
  ) async {
    // This is a regression test for
    // https://github.com/flutter/flutter/issues/43787
    final TextEditingController controller = _textEditingController(
      text: 'This is a test that shows some odd behavior with Text Selection!',
    );

    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ColoredBox(
            color: Colors.grey,
            child: Center(
              child: Container(
                color: Colors.red,
                width: 300,
                height: 600,
                child: Overlay(
                  initialEntries: <OverlayEntry>[
                    overlayEntry = OverlayEntry(
                      builder:
                          (BuildContext context) =>
                              Center(child: TextField(controller: controller)),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await showSelectionMenuAt(tester, controller, controller.text.indexOf('test'));
    await tester.pumpAndSettle();
    expect(tester.takeException(), isNull);
  });

  testWidgets(
    'clipboard status is checked via hasStrings without getting the full clipboard contents',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      bool calledGetData = false;
      bool calledHasStrings = false;
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        switch (methodCall.method) {
          case 'Clipboard.getData':
            calledGetData = true;
          case 'Clipboard.hasStrings':
            calledHasStrings = true;
          default:
            break;
        }
        return null;
      });

      final Offset textfieldStart = tester.getTopLeft(find.byType(TextField));

      // Double tap like when showing the text selection menu on Android/iOS.
      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textfieldStart + const Offset(150.0, 9.0));
      await tester.pump();

      // getData is not called unless something is pasted.  hasStrings is used to
      // check the status of the clipboard.
      expect(calledGetData, false);
      // hasStrings is checked in order to decide if the content can be pasted.
      expect(calledHasStrings, true);
    },
    skip: kIsWeb, // [intended] web doesn't call hasStrings.
  );

  testWidgets('TextField changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextField(
              mouseCursor: SystemMouseCursors.grab,
              decoration: InputDecoration(
                // Add an icon so that the left edge is not the text area
                icon: Icon(Icons.person),
              ),
            ),
          ),
        ),
      ),
    );

    // Center, which is within the text area
    final Offset center = tester.getCenter(find.byType(TextField));
    // Top left, which is not the text area
    final Offset edge = tester.getTopLeft(find.byType(TextField)) + const Offset(1, 1);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: center);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    // Test default cursor
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextField(decoration: InputDecoration(icon: Icon(Icons.person))),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    await gesture.moveTo(edge);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    await gesture.moveTo(center);

    // Test default cursor when disabled
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextField(enabled: false, decoration: InputDecoration(icon: Icon(Icons.person))),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(edge);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(center);
  });

  testWidgets('TextField icons change mouse cursor when hovered', (WidgetTester tester) async {
    // Test default cursor in icons area.
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextField(
              decoration: InputDecoration(
                icon: Icon(Icons.label),
                prefixIcon: Icon(Icons.cabin),
                suffixIcon: Icon(Icons.person),
              ),
            ),
          ),
        ),
      ),
    );

    // Center, which is within the text area
    final Offset center = tester.getCenter(find.byType(TextField));
    // The Icon area
    final Offset iconArea = tester.getCenter(find.byIcon(Icons.label));
    // The prefix Icon area
    final Offset prefixIconArea = tester.getCenter(find.byIcon(Icons.cabin));
    // The suffix Icon area
    final Offset suffixIconArea = tester.getCenter(find.byIcon(Icons.person));

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: center);

    await tester.pump();

    await gesture.moveTo(center);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    await gesture.moveTo(iconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    await gesture.moveTo(prefixIconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    await gesture.moveTo(suffixIconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(center);

    // Test click cursor in icons area for buttons.
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextField(
              decoration: InputDecoration(
                icon: IconButton(icon: const Icon(Icons.label), onPressed: () {}),
                prefixIcon: IconButton(icon: const Icon(Icons.cabin), onPressed: () {}),
                suffixIcon: IconButton(icon: const Icon(Icons.person), onPressed: () {}),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pump();

    await gesture.moveTo(center);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    await gesture.moveTo(iconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    await gesture.moveTo(prefixIconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    await gesture.moveTo(suffixIconArea);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets(
    'Text selection menu does not change mouse cursor when hovered',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MouseRegion(
              cursor: SystemMouseCursors.forbidden,
              child: TextField(controller: controller),
            ),
          ),
        ),
      );

      expect(find.text('Copy'), findsNothing);

      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 3),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.text,
      );
      expect(find.text('Paste'), findsOneWidget);

      await gesture.moveTo(tester.getCenter(find.text('Paste')));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.basic,
      );
    },
    variant: TargetPlatformVariant.desktop(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('Caret rtl with changing width', (WidgetTester tester) async {
    late StateSetter setState;
    bool isWide = false;
    const double wideWidth = 300.0;
    const double narrowWidth = 200.0;
    const TextStyle style = TextStyle(
      fontSize: 10,
      height: 1.0,
      letterSpacing: 0.0,
      wordSpacing: 0.0,
    );
    const double caretWidth = 2.0;
    final TextEditingController controller = _textEditingController();
    await tester.pumpWidget(
      boilerplate(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            setState = setter;
            return SizedBox(
              width: isWide ? wideWidth : narrowWidth,
              child: TextField(
                key: textFieldKey,
                controller: controller,
                textDirection: TextDirection.rtl,
                style: style,
              ),
            );
          },
        ),
      ),
    );

    // The cursor is on the right of the input because it's RTL.
    RenderEditable editable = findRenderEditable(tester);
    double cursorRight =
        editable
            .getLocalRectForCaret(TextPosition(offset: controller.value.text.length))
            .topRight
            .dx;
    double inputWidth = editable.size.width;
    expect(inputWidth, narrowWidth);
    expect(cursorRight, inputWidth - kCaretGap);

    const String text = '12345';
    // After entering some text, the cursor is placed to the left of the text
    // because the paragraph's writing direction is RTL.
    await tester.enterText(find.byType(TextField), text);
    await tester.pump();
    editable = findRenderEditable(tester);
    cursorRight =
        editable
            .getLocalRectForCaret(TextPosition(offset: controller.value.text.length))
            .topRight
            .dx;
    inputWidth = editable.size.width;
    expect(cursorRight, inputWidth - kCaretGap - text.length * 10 - caretWidth);

    // Since increasing the width of the input moves its right edge further to
    // the right, the cursor has followed this change and still appears on the
    // right of the input.
    setState(() {
      isWide = true;
    });
    await tester.pump();
    editable = findRenderEditable(tester);
    cursorRight =
        editable
            .getLocalRectForCaret(TextPosition(offset: controller.value.text.length))
            .topRight
            .dx;
    inputWidth = editable.size.width;
    expect(inputWidth, wideWidth);
    expect(cursorRight, inputWidth - kCaretGap - text.length * 10 - caretWidth);
  });

  testWidgets(
    'Text selection menu hides after select all on desktop',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      final String selectAll =
          defaultTargetPlatform == TargetPlatform.macOS ? 'Select All' : 'Select all';

      expect(find.text(selectAll), findsNothing);
      expect(find.text('Copy'), findsNothing);

      final TestGesture gesture = await tester.startGesture(
        const Offset(10.0, 0.0) + textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(
        controller.value.selection,
        TextSelection.collapsed(offset: controller.text.length, affinity: TextAffinity.upstream),
      );
      expect(find.text(selectAll), findsOneWidget);

      await tester.tapAt(tester.getCenter(find.text(selectAll)));

      await tester.pump();
      expect(find.text(selectAll), findsNothing);
      expect(find.text('Copy'), findsNothing);
    },
    // All desktop platforms except MacOS, which has no select all button.
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  // Regressing test for https://github.com/flutter/flutter/issues/70625
  testWidgets('TextFields can inherit [FloatingLabelBehaviour] from InputDecorationTheme.', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = _focusNode();
    Widget textFieldBuilder({FloatingLabelBehavior behavior = FloatingLabelBehavior.auto}) {
      return MaterialApp(
        theme: ThemeData(
          useMaterial3: false,
          inputDecorationTheme: InputDecorationTheme(floatingLabelBehavior: behavior),
        ),
        home: Scaffold(
          body: TextField(
            focusNode: focusNode,
            decoration: const InputDecoration(labelText: 'Label'),
          ),
        ),
      );
    }

    await tester.pumpWidget(textFieldBuilder());
    // The label will be positioned within the content when unfocused.
    expect(tester.getTopLeft(find.text('Label')).dy, 20.0);

    focusNode.requestFocus();
    await tester.pumpAndSettle(); // label animation.
    // The label will float above the content when focused.
    expect(tester.getTopLeft(find.text('Label')).dy, 12.0);

    focusNode.unfocus();
    await tester.pumpAndSettle(); // label animation.

    await tester.pumpWidget(textFieldBuilder(behavior: FloatingLabelBehavior.never));
    await tester.pumpAndSettle(); // theme animation.
    // The label will be positioned within the content.
    expect(tester.getTopLeft(find.text('Label')).dy, 20.0);

    focusNode.requestFocus();
    await tester.pumpAndSettle(); // label animation.
    // The label will always be positioned within the content.
    expect(tester.getTopLeft(find.text('Label')).dy, 20.0);

    await tester.pumpWidget(textFieldBuilder(behavior: FloatingLabelBehavior.always));
    await tester.pumpAndSettle(); // theme animation.
    // The label will always float above the content.
    expect(tester.getTopLeft(find.text('Label')).dy, 12.0);

    focusNode.unfocus();
    await tester.pumpAndSettle(); // label animation.
    // The label will always float above the content.
    expect(tester.getTopLeft(find.text('Label')).dy, 12.0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/140607.
  testWidgets('TextFields can inherit errorStyle color from InputDecorationTheme.', (
    WidgetTester tester,
  ) async {
    Widget textFieldBuilder() {
      return MaterialApp(
        theme: ThemeData(
          inputDecorationTheme: const InputDecorationTheme(
            errorStyle: TextStyle(color: Colors.green),
          ),
        ),
        home: const Scaffold(body: TextField(decoration: InputDecoration(errorText: 'error'))),
      );
    }

    await tester.pumpWidget(textFieldBuilder());
    await tester.pumpAndSettle();
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.widget.cursorColor, Colors.green);
  });

  group('MaxLengthEnforcement', () {
    const int maxLength = 5;

    Future<void> setupWidget(WidgetTester tester, MaxLengthEnforcement? enforcement) async {
      final Widget widget = MaterialApp(
        home: Material(child: TextField(maxLength: maxLength, maxLengthEnforcement: enforcement)),
      );

      await tester.pumpWidget(widget);
      await tester.pumpAndSettle();
    }

    testWidgets('using none enforcement.', (WidgetTester tester) async {
      const MaxLengthEnforcement enforcement = MaxLengthEnforcement.none;

      await setupWidget(tester, enforcement);

      final EditableTextState state = tester.state(find.byType(EditableText));

      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.text, 'abc');
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.text, 'abcdef');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 6));

      state.updateEditingValue(const TextEditingValue(text: 'abcdef'));
      expect(state.currentTextEditingValue.text, 'abcdef');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    testWidgets('using enforced.', (WidgetTester tester) async {
      const MaxLengthEnforcement enforcement = MaxLengthEnforcement.enforced;

      await setupWidget(tester, enforcement);

      final EditableTextState state = tester.state(find.byType(EditableText));

      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.text, 'abc');
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      state.updateEditingValue(const TextEditingValue(text: 'abcdef'));
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));
    });

    testWidgets('using truncateAfterCompositionEnds.', (WidgetTester tester) async {
      const MaxLengthEnforcement enforcement = MaxLengthEnforcement.truncateAfterCompositionEnds;

      await setupWidget(tester, enforcement);

      final EditableTextState state = tester.state(find.byType(EditableText));

      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.text, 'abc');
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.text, 'abcdef');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 6));

      state.updateEditingValue(const TextEditingValue(text: 'abcdef'));
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    testWidgets('using default behavior for different platforms.', (WidgetTester tester) async {
      await setupWidget(tester, null);

      final EditableTextState state = tester.state(find.byType(EditableText));

      state.updateEditingValue(const TextEditingValue(text: '侬好啊'));
      expect(state.currentTextEditingValue.text, '侬好啊');
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      state.updateEditingValue(
        const TextEditingValue(text: '侬好啊旁友', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.text, '侬好啊旁友');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      state.updateEditingValue(
        const TextEditingValue(text: '侬好啊旁友们', composing: TextRange(start: 3, end: 6)),
      );
      if (kIsWeb ||
          defaultTargetPlatform == TargetPlatform.iOS ||
          defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.linux ||
          defaultTargetPlatform == TargetPlatform.fuchsia) {
        expect(state.currentTextEditingValue.text, '侬好啊旁友们');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 6));
      } else {
        expect(state.currentTextEditingValue.text, '侬好啊旁友');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));
      }

      state.updateEditingValue(const TextEditingValue(text: '侬好啊旁友'));
      expect(state.currentTextEditingValue.text, '侬好啊旁友');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });
  });

  testWidgets('TextField does not leak touch events when deadline has exceeded', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/118340.
    int textFieldTapCount = 0;
    int prefixTapCount = 0;
    int suffixTapCount = 0;

    final FocusNode focusNode = _focusNode();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: TextField(
            focusNode: focusNode,
            onTap: () {
              textFieldTapCount += 1;
            },
            decoration: InputDecoration(
              labelText: 'Label',
              prefix: ElevatedButton(
                onPressed: () {
                  prefixTapCount += 1;
                },
                child: const Text('prefix'),
              ),
              suffix: ElevatedButton(
                onPressed: () {
                  suffixTapCount += 1;
                },
                child: const Text('suffix'),
              ),
            ),
          ),
        ),
      ),
    );

    // Focus to show the prefix and suffix buttons.
    focusNode.requestFocus();
    await tester.pump();

    TestGesture gesture = await tester.startGesture(
      tester.getRect(find.text('prefix')).center,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pumpAndSettle();
    await gesture.up();
    expect(textFieldTapCount, 0);
    expect(prefixTapCount, 1);
    expect(suffixTapCount, 0);

    gesture = await tester.startGesture(
      tester.getRect(find.text('suffix')).center,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pumpAndSettle();
    await gesture.up();
    expect(textFieldTapCount, 0);
    expect(prefixTapCount, 1);
    expect(suffixTapCount, 1);
  });

  testWidgets('prefix/suffix buttons do not leak touch events', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/39376.

    int textFieldTapCount = 0;
    int prefixTapCount = 0;
    int suffixTapCount = 0;

    final FocusNode focusNode = _focusNode();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: TextField(
            focusNode: focusNode,
            onTap: () {
              textFieldTapCount += 1;
            },
            decoration: InputDecoration(
              labelText: 'Label',
              prefix: ElevatedButton(
                onPressed: () {
                  prefixTapCount += 1;
                },
                child: const Text('prefix'),
              ),
              suffix: ElevatedButton(
                onPressed: () {
                  suffixTapCount += 1;
                },
                child: const Text('suffix'),
              ),
            ),
          ),
        ),
      ),
    );

    // Focus to show the prefix and suffix buttons.
    focusNode.requestFocus();
    await tester.pump();

    await tester.tap(find.text('prefix'));
    expect(textFieldTapCount, 0);
    expect(prefixTapCount, 1);
    expect(suffixTapCount, 0);

    await tester.tap(find.text('suffix'));
    expect(textFieldTapCount, 0);
    expect(prefixTapCount, 1);
    expect(suffixTapCount, 1);
  });

  testWidgets('autofill info has hint text', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: TextField(decoration: InputDecoration(hintText: 'placeholder text')),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(TextField));

    expect(
      tester.testTextInput.setClientArgs?['autofill'],
      containsPair('hintText', 'placeholder text'),
    );
  });

  testWidgets('TextField at rest does not push any layers with alwaysNeedsAddToScene', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const MaterialApp(home: Material(child: Center(child: TextField()))));

    expect(tester.layers.any((Layer layer) => layer.debugSubtreeNeedsAddToScene!), isFalse);
  });

  testWidgets('Focused TextField does not push any layers with alwaysNeedsAddToScene', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = _focusNode();
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextField(focusNode: focusNode)))),
    );
    await tester.showKeyboard(find.byType(TextField));

    expect(focusNode.hasFocus, isTrue);
    expect(tester.layers.any((Layer layer) => layer.debugSubtreeNeedsAddToScene!), isFalse);
  });

  testWidgets(
    'TextField does not push any layers with alwaysNeedsAddToScene after toolbar is dismissed',
    (WidgetTester tester) async {
      final FocusNode focusNode = _focusNode();
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(focusNode: focusNode)))),
      );

      await tester.showKeyboard(find.byType(TextField));

      // Bring up the toolbar.
      const String testValue = 'A B C';
      tester.testTextInput.updateEditingValue(const TextEditingValue(text: testValue));
      await tester.pump();
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
      expect(state.showToolbar(), true);
      await tester.pumpAndSettle();
      await tester.pump(const Duration(seconds: 1));
      expect(find.text('Copy'), findsOneWidget); // Toolbar is visible

      // Hide the toolbar
      focusNode.unfocus();
      await tester.pumpAndSettle();
      await tester.pump(const Duration(seconds: 1));
      expect(find.text('Copy'), findsNothing); // Toolbar is not visible

      expect(tester.layers.any((Layer layer) => layer.debugSubtreeNeedsAddToScene!), isFalse);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('cursor blinking respects TickerMode', (WidgetTester tester) async {
    final FocusNode focusNode = _focusNode();
    Widget builder({required bool tickerMode}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: TickerMode(enabled: tickerMode, child: TextField(focusNode: focusNode)),
          ),
        ),
      );
    }

    // TickerMode is on, cursor is blinking.
    await tester.pumpWidget(builder(tickerMode: true));
    await tester.showKeyboard(find.byType(TextField));
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable editable = state.renderEditable;
    expect(editable.showCursor.value, isTrue);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isTrue);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);

    // TickerMode is off, cursor does not blink.
    await tester.pumpWidget(builder(tickerMode: false));
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);

    // TickerMode is on, cursor blinks again.
    await tester.pumpWidget(builder(tickerMode: true));
    expect(editable.showCursor.value, isTrue);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isTrue);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);

    // Dismissing focus while tickerMode is off does not start cursor blinking
    // when tickerMode is turned on again.
    await tester.pumpWidget(builder(tickerMode: false));
    focusNode.unfocus();
    await tester.pump();
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pumpWidget(builder(tickerMode: true));
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);

    // Focusing while tickerMode is off does not start cursor blinking...
    await tester.pumpWidget(builder(tickerMode: false));
    await tester.showKeyboard(find.byType(TextField));
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);

    // ... but it does start when tickerMode is switched on again.
    await tester.pumpWidget(builder(tickerMode: true));
    expect(editable.showCursor.value, isTrue);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isFalse);
    await tester.pump(state.cursorBlinkInterval);
    expect(editable.showCursor.value, isTrue);
  });

  testWidgets(
    'can shift + tap to select with a keyboard (Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      await tester.tapAt(textOffsetToPosition(tester, 13));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 13);

      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.tapAt(textOffsetToPosition(tester, 20));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 20);

      await tester.pump(kDoubleTapTimeout);
      await tester.tapAt(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 23);

      await tester.pump(kDoubleTapTimeout);
      await tester.tapAt(textOffsetToPosition(tester, 4));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 4);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 4);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'can shift + tap to select with a keyboard (non-Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      await tester.tapAt(textOffsetToPosition(tester, 13));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 13);

      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.tapAt(textOffsetToPosition(tester, 20));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 20);

      await tester.pump(kDoubleTapTimeout);
      await tester.tapAt(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 23);

      await tester.pump(kDoubleTapTimeout);
      await tester.tapAt(textOffsetToPosition(tester, 4));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 4);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 13);
      expect(controller.selection.extentOffset, 4);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets('shift tapping an unfocused field', (WidgetTester tester) async {
    final TextEditingController controller = _textEditingController(
      text: 'Atwater Peel Sherbrooke Bonaventure',
    );
    final FocusNode focusNode = _focusNode();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextField(controller: controller, focusNode: focusNode)),
        ),
      ),
    );
    expect(focusNode.hasFocus, isFalse);

    // Put the cursor at the end of the field.
    await tester.tapAt(textOffsetToPosition(tester, controller.text.length));
    await tester.pump(kDoubleTapTimeout);
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    expect(controller.selection.baseOffset, 35);
    expect(controller.selection.extentOffset, 35);

    // Unfocus the field, but the selection remains.
    focusNode.unfocus();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isFalse);
    expect(controller.selection.baseOffset, 35);
    expect(controller.selection.extentOffset, 35);

    // Shift tap in the middle of the field.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.tapAt(textOffsetToPosition(tester, 20));
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    switch (defaultTargetPlatform) {
      // Apple platforms start the selection from 0.
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        expect(controller.selection.baseOffset, 0);

      // Other platforms start from the previous selection.
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        expect(controller.selection.baseOffset, 35);
    }
    expect(controller.selection.extentOffset, 20);
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'can shift + tap + drag to select with a keyboard (Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      await tester.tapAt(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 8);

      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 23),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      if (isTargetPlatformMobile) {
        await gesture.up();
      }
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 23);

      // Expand the selection a bit.
      if (isTargetPlatformMobile) {
        await gesture.down(textOffsetToPosition(tester, 24));
      }
      await tester.pumpAndSettle();
      await gesture.moveTo(textOffsetToPosition(tester, 28));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 28);

      // Move back to the original selection.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 23);

      // Collapse the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 8);

      // Invert the selection. The base jumps to the original extent.
      await gesture.moveTo(textOffsetToPosition(tester, 7));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 7);

      // Continuing to move in the inverted direction expands the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 4));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 4);

      // Move back to the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 8);

      // Continue to move past the original base, which will cause the selection
      // to invert back to the original orientation.
      await gesture.moveTo(textOffsetToPosition(tester, 9));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 9);

      // Continuing to select in this direction selects just like it did
      // originally.
      await gesture.moveTo(textOffsetToPosition(tester, 24));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 24);

      // Releasing the shift key has no effect; the selection continues as the
      // mouse continues to move.
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 24);
      await gesture.moveTo(textOffsetToPosition(tester, 26));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 26);

      await gesture.up();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 26);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'can shift + tap + drag to select with a keyboard (non-Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      final bool isTargetPlatformMobile =
          defaultTargetPlatform == TargetPlatform.android ||
          defaultTargetPlatform == TargetPlatform.fuchsia;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      await tester.tapAt(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 8);

      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 23),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pumpAndSettle();
      if (isTargetPlatformMobile) {
        await gesture.up();
        // Not a double tap + drag.
        await tester.pumpAndSettle(kDoubleTapTimeout);
      }
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 23);

      // Expand the selection a bit.
      if (isTargetPlatformMobile) {
        await gesture.down(textOffsetToPosition(tester, 23));
        await tester.pumpAndSettle();
      }
      await gesture.moveTo(textOffsetToPosition(tester, 28));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 28);

      // Move back to the original selection.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 23);

      // Collapse the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 8);

      // Invert the selection. The original selection is not restored like on iOS
      // and Mac.
      await gesture.moveTo(textOffsetToPosition(tester, 7));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 7);

      // Continuing to move in the inverted direction expands the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 4));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 4);

      // Move back to the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 8);

      // Continue to move past the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 9));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 9);

      // Continuing to select in this direction selects just like it did
      // originally.
      await gesture.moveTo(textOffsetToPosition(tester, 24));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 24);

      // Releasing the shift key has no effect; the selection continues as the
      // mouse continues to move.
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 24);
      await gesture.moveTo(textOffsetToPosition(tester, 26));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 26);

      await gesture.up();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 26);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'can shift + tap + drag to select with a keyboard, reversed (Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      final bool isTargetPlatformMobile = defaultTargetPlatform == TargetPlatform.iOS;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      // Make a selection from right to left.
      await tester.tapAt(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 23);
      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 8),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      if (isTargetPlatformMobile) {
        await gesture.up();
      }
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 8);

      // Expand the selection a bit.
      if (isTargetPlatformMobile) {
        await gesture.down(textOffsetToPosition(tester, 7));
      }
      await gesture.moveTo(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 5);

      // Move back to the original selection.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 8);

      // Collapse the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 23);

      // Invert the selection. The base jumps to the original extent.
      await gesture.moveTo(textOffsetToPosition(tester, 24));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 24);

      // Continuing to move in the inverted direction expands the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 27));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 27);

      // Move back to the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 8);
      expect(controller.selection.extentOffset, 23);

      // Continue to move past the original base, which will cause the selection
      // to invert back to the original orientation.
      await gesture.moveTo(textOffsetToPosition(tester, 22));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 22);

      // Continuing to select in this direction selects just like it did
      // originally.
      await gesture.moveTo(textOffsetToPosition(tester, 16));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 16);

      // Releasing the shift key has no effect; the selection continues as the
      // mouse continues to move.
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 16);
      await gesture.moveTo(textOffsetToPosition(tester, 14));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 14);

      await gesture.up();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 14);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'can shift + tap + drag to select with a keyboard, reversed (non-Apple platforms)',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      final bool isTargetPlatformMobile =
          defaultTargetPlatform == TargetPlatform.android ||
          defaultTargetPlatform == TargetPlatform.fuchsia;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(child: Center(child: TextField(controller: controller))),
        ),
      );

      // Make a selection from right to left.
      await tester.tapAt(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 23);
      await tester.pump(kDoubleTapTimeout);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 8),
        pointer: 7,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pumpAndSettle();
      if (isTargetPlatformMobile) {
        await gesture.up();
        // Not a double tap + drag.
        await tester.pumpAndSettle(kDoubleTapTimeout);
      }
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 8);

      // Expand the selection a bit.
      if (isTargetPlatformMobile) {
        await gesture.down(textOffsetToPosition(tester, 8));
        await tester.pumpAndSettle();
      }
      await gesture.moveTo(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 5);

      // Move back to the original selection.
      await gesture.moveTo(textOffsetToPosition(tester, 8));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 8);

      // Collapse the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 23);

      // Invert the selection. The selection is not restored like it would be on
      // iOS and Mac.
      await gesture.moveTo(textOffsetToPosition(tester, 24));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 24);

      // Continuing to move in the inverted direction expands the selection.
      await gesture.moveTo(textOffsetToPosition(tester, 27));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 27);

      // Move back to the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 23));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 23);

      // Continue to move past the original base.
      await gesture.moveTo(textOffsetToPosition(tester, 22));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 22);

      // Continuing to select in this direction selects just like it did
      // originally.
      await gesture.moveTo(textOffsetToPosition(tester, 16));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 16);

      // Releasing the shift key has no effect; the selection continues as the
      // mouse continues to move.
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 16);
      await gesture.moveTo(textOffsetToPosition(tester, 14));
      await tester.pumpAndSettle();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 14);

      await gesture.up();
      expect(controller.selection.baseOffset, 23);
      expect(controller.selection.extentOffset, 14);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.windows,
    }),
  );

  // Regression test for https://github.com/flutter/flutter/issues/101587.
  testWidgets(
    'Right clicking menu behavior',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Initially, the menu is not shown and there is no selection.
      expectNoCupertinoToolbar();
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);
      final Offset midBlah2 = textOffsetToPosition(tester, 8);

      // Right click the second word.
      final TestGesture gesture = await tester.startGesture(
        midBlah2,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection, const TextSelection(baseOffset: 6, extentOffset: 11));
          expect(find.text('Cut'), findsOneWidget);
          expect(find.text('Copy'), findsOneWidget);
          expect(find.text('Paste'), findsOneWidget);

        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection, const TextSelection.collapsed(offset: 8));
          expect(find.text('Cut'), findsNothing);
          expect(find.text('Copy'), findsNothing);
          expect(find.text('Paste'), findsOneWidget);
          expect(find.text('Select all'), findsOneWidget);
      }

      // Right click the first word.
      await gesture.down(midBlah1);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
          expect(find.text('Cut'), findsOneWidget);
          expect(find.text('Copy'), findsOneWidget);
          expect(find.text('Paste'), findsOneWidget);

        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection, const TextSelection.collapsed(offset: 8));
          expect(find.text('Cut'), findsNothing);
          expect(find.text('Copy'), findsNothing);
          expect(find.text('Paste'), findsNothing);
          expect(find.text('Select all'), findsNothing);
      }
    },
    variant: TargetPlatformVariant.all(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('Cannot request focus when canRequestFocus is false', (WidgetTester tester) async {
    final FocusNode focusNode = _focusNode();

    // Default test. The canRequestFocus is true by default and the text field can be focused
    await tester.pumpWidget(boilerplate(child: TextField(focusNode: focusNode)));
    expect(focusNode.hasFocus, isFalse);
    focusNode.requestFocus();
    await tester.pump();
    expect(focusNode.hasFocus, isTrue);

    // Set canRequestFocus to false: the text field cannot be focused when it is tapped/long pressed.
    await tester.pumpWidget(
      boilerplate(child: TextField(focusNode: focusNode, canRequestFocus: false)),
    );

    expect(focusNode.hasFocus, isFalse);
    focusNode.requestFocus();
    await tester.pump();
    expect(focusNode.hasFocus, isFalse);

    // The text field cannot be focused if it is tapped.
    await tester.tap(find.byType(TextField));
    await tester.pump();
    expect(focusNode.hasFocus, isFalse);

    // The text field cannot be focused if it is long pressed.
    await tester.longPress(find.byType(TextField));
    await tester.pump();
    expect(focusNode.hasFocus, isFalse);
  });

  group('Right click focus', () {
    testWidgets('Can right click to focus multiple times', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/pull/103228
      final FocusNode focusNode1 = _focusNode();
      final FocusNode focusNode2 = _focusNode();
      final UniqueKey key1 = UniqueKey();
      final UniqueKey key2 = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                TextField(key: key1, focusNode: focusNode1),
                const SizedBox(height: 100.0),
                TextField(key: key2, focusNode: focusNode2),
              ],
            ),
          ),
        ),
      );

      // Interact with the field to establish the input connection.
      await tester.tapAt(tester.getCenter(find.byKey(key1)), buttons: kSecondaryButton);
      await tester.pump();

      expect(focusNode1.hasFocus, isTrue);
      expect(focusNode2.hasFocus, isFalse);

      await tester.tapAt(tester.getCenter(find.byKey(key2)), buttons: kSecondaryButton);
      await tester.pump();

      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasFocus, isTrue);

      await tester.tapAt(tester.getCenter(find.byKey(key1)), buttons: kSecondaryButton);
      await tester.pump();

      expect(focusNode1.hasFocus, isTrue);
      expect(focusNode2.hasFocus, isFalse);
    });

    testWidgets(
      'Can right click to focus on previously selected word on Apple platforms',
      (WidgetTester tester) async {
        final FocusNode focusNode1 = _focusNode();
        final FocusNode focusNode2 = _focusNode();
        final TextEditingController controller = _textEditingController(text: 'first second');
        final UniqueKey key1 = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Column(
                children: <Widget>[
                  TextField(key: key1, controller: controller, focusNode: focusNode1),
                  Focus(focusNode: focusNode2, child: const Text('focusable')),
                ],
              ),
            ),
          ),
        );

        // Interact with the field to establish the input connection.
        await tester.tapAt(tester.getCenter(find.byKey(key1)), buttons: kSecondaryButton);
        await tester.pump();

        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);

        // Select the second word.
        controller.selection = const TextSelection(baseOffset: 6, extentOffset: 12);
        await tester.pump();

        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);
        expect(controller.selection.isCollapsed, isFalse);
        expect(controller.selection.baseOffset, 6);
        expect(controller.selection.extentOffset, 12);

        // Unfocus the first field.
        focusNode2.requestFocus();
        await tester.pumpAndSettle();

        expect(focusNode1.hasFocus, isFalse);
        expect(focusNode2.hasFocus, isTrue);

        // Right click the second word in the first field, which is still selected
        // even though the selection is not visible.
        await tester.tapAt(textOffsetToPosition(tester, 8), buttons: kSecondaryButton);
        await tester.pump();

        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);
        expect(controller.selection.baseOffset, 6);
        expect(controller.selection.extentOffset, 12);

        // Select everything.
        controller.selection = const TextSelection(baseOffset: 0, extentOffset: 12);
        await tester.pump();

        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 12);

        // Unfocus the first field.
        focusNode2.requestFocus();
        await tester.pumpAndSettle();

        // Right click the first word in the first field.
        await tester.tapAt(textOffsetToPosition(tester, 2), buttons: kSecondaryButton);
        await tester.pump();

        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 5);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets('Right clicking cannot request focus if canRequestFocus is false', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = _focusNode();
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[TextField(key: key, focusNode: focusNode, canRequestFocus: false)],
            ),
          ),
        ),
      );

      await tester.tapAt(tester.getCenter(find.byKey(key)), buttons: kSecondaryButton);
      await tester.pump();

      expect(focusNode.hasFocus, isFalse);
    });
  });

  group('context menu', () {
    testWidgets(
      'builds AdaptiveTextSelectionToolbar by default',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();
        await tester.pumpWidget(
          MaterialApp(
            home: Material(child: Column(children: <Widget>[TextField(controller: controller)])),
          ),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

        // Long-press to bring up the context menu.
        final Finder textFinder = find.byType(EditableText);
        await tester.longPress(textFinder);
        tester.state<EditableTextState>(textFinder).showToolbar();
        await tester.pumpAndSettle();

        expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
      },
      skip: kIsWeb, // [intended] on web the browser handles the context menu.
    );

    testWidgets(
      'contextMenuBuilder is used in place of the default text selection toolbar',
      (WidgetTester tester) async {
        final GlobalKey key = GlobalKey();
        final TextEditingController controller = _textEditingController();
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Column(
                children: <Widget>[
                  TextField(
                    controller: controller,
                    contextMenuBuilder: (
                      BuildContext context,
                      EditableTextState editableTextState,
                    ) {
                      return Placeholder(key: key);
                    },
                  ),
                ],
              ),
            ),
          ),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        expect(find.byKey(key), findsNothing);

        // Long-press to bring up the context menu.
        final Finder textFinder = find.byType(EditableText);
        await tester.longPress(textFinder);
        tester.state<EditableTextState>(textFinder).showToolbar();
        await tester.pumpAndSettle();

        expect(find.byKey(key), findsOneWidget);
      },
      skip: kIsWeb, // [intended] on web the browser handles the context menu.
    );

    testWidgets(
      'contextMenuBuilder changes from default to null',
      (WidgetTester tester) async {
        final GlobalKey key = GlobalKey();
        final TextEditingController controller = _textEditingController();
        await tester.pumpWidget(
          MaterialApp(home: Material(child: TextField(key: key, controller: controller))),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        // Long-press to bring up the context menu.
        final Finder textFinder = find.byType(EditableText);
        await tester.longPress(textFinder);
        tester.state<EditableTextState>(textFinder).showToolbar();
        await tester.pump();

        expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

        // Set contextMenuBuilder to null.
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: TextField(key: key, controller: controller, contextMenuBuilder: null),
            ),
          ),
        );

        // Trigger build one more time...
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Padding(
                padding: EdgeInsets.zero,
                child: TextField(key: key, controller: controller, contextMenuBuilder: null),
              ),
            ),
          ),
        );
      },
      skip: kIsWeb, // [intended] on web the browser handles the context menu.
    );
  });

  group('magnifier builder', () {
    testWidgets('should build custom magnifier if given', (WidgetTester tester) async {
      final Widget customMagnifier = Container(key: UniqueKey());
      final TextField textField = TextField(
        magnifierConfiguration: TextMagnifierConfiguration(
          magnifierBuilder:
              (
                BuildContext context,
                MagnifierController controller,
                ValueNotifier<MagnifierInfo>? info,
              ) => customMagnifier,
        ),
      );

      await tester.pumpWidget(const MaterialApp(home: Placeholder()));

      final BuildContext context = tester.firstElement(find.byType(Placeholder));
      final ValueNotifier<MagnifierInfo> magnifierInfo = ValueNotifier<MagnifierInfo>(
        MagnifierInfo.empty,
      );
      addTearDown(magnifierInfo.dispose);

      expect(
        textField.magnifierConfiguration!.magnifierBuilder(
          context,
          MagnifierController(),
          magnifierInfo,
        ),
        isA<Widget>().having(
          (Widget widget) => widget.key,
          'built magnifier key equal to passed in magnifier key',
          equals(customMagnifier.key),
        ),
      );
    });

    group('defaults', () {
      testWidgets(
        'should build Magnifier on Android',
        (WidgetTester tester) async {
          await tester.pumpWidget(const MaterialApp(home: Scaffold(body: TextField())));

          final BuildContext context = tester.firstElement(find.byType(TextField));
          final EditableText editableText = tester.widget(find.byType(EditableText));
          final ValueNotifier<MagnifierInfo> magnifierInfo = ValueNotifier<MagnifierInfo>(
            MagnifierInfo.empty,
          );
          addTearDown(magnifierInfo.dispose);

          expect(
            editableText.magnifierConfiguration.magnifierBuilder(
              context,
              MagnifierController(),
              magnifierInfo,
            ),
            isA<TextMagnifier>(),
          );
        },
        variant: TargetPlatformVariant.only(TargetPlatform.android),
      );

      testWidgets(
        'should build CupertinoMagnifier on iOS',
        (WidgetTester tester) async {
          await tester.pumpWidget(const MaterialApp(home: Scaffold(body: TextField())));

          final BuildContext context = tester.firstElement(find.byType(TextField));
          final EditableText editableText = tester.widget(find.byType(EditableText));
          final ValueNotifier<MagnifierInfo> magnifierInfo = ValueNotifier<MagnifierInfo>(
            MagnifierInfo.empty,
          );
          addTearDown(magnifierInfo.dispose);

          expect(
            editableText.magnifierConfiguration.magnifierBuilder(
              context,
              MagnifierController(),
              magnifierInfo,
            ),
            isA<CupertinoTextMagnifier>(),
          );
        },
        variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      );

      testWidgets(
        'should build nothing on Android and iOS',
        (WidgetTester tester) async {
          await tester.pumpWidget(const MaterialApp(home: Scaffold(body: TextField())));

          final BuildContext context = tester.firstElement(find.byType(TextField));
          final EditableText editableText = tester.widget(find.byType(EditableText));
          final ValueNotifier<MagnifierInfo> magnifierInfo = ValueNotifier<MagnifierInfo>(
            MagnifierInfo.empty,
          );
          addTearDown(magnifierInfo.dispose);

          expect(
            editableText.magnifierConfiguration.magnifierBuilder(
              context,
              MagnifierController(),
              magnifierInfo,
            ),
            isNull,
          );
        },
        variant: TargetPlatformVariant.all(
          excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.android},
        ),
      );
    });
  });

  group('magnifier', () {
    late ValueNotifier<MagnifierInfo> magnifierInfo;
    final Widget fakeMagnifier = Container(key: UniqueKey());

    testWidgets('Can drag handles to show, unshow, and update magnifier', (
      WidgetTester tester,
    ) async {
      final TextEditingController controller = _textEditingController();
      await tester.pumpWidget(
        overlay(
          child: TextField(
            dragStartBehavior: DragStartBehavior.down,
            controller: controller,
            magnifierConfiguration: TextMagnifierConfiguration(
              magnifierBuilder: (
                BuildContext context,
                MagnifierController controller,
                ValueNotifier<MagnifierInfo> localMagnifierInfo,
              ) {
                magnifierInfo = localMagnifierInfo;
                return fakeMagnifier;
              },
            ),
          ),
        ),
      );

      const String testValue = 'abc def ghi';
      await tester.enterText(find.byType(TextField), testValue);
      await skipPastScrollingAnimation(tester);

      // Double tap the 'e' to select 'def'.
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump(const Duration(milliseconds: 30));
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump(const Duration(milliseconds: 30));

      final TextSelection selection = controller.selection;

      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );

      // Drag the right handle 2 letters to the right.
      final Offset handlePos = endpoints.last.point + const Offset(1.0, 1.0);
      final TestGesture gesture = await tester.startGesture(handlePos);

      await gesture.moveTo(textOffsetToPosition(tester, testValue.length - 2));
      await tester.pump();

      expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
      final Offset firstDragGesturePosition = magnifierInfo.value.globalGesturePosition;

      await gesture.moveTo(textOffsetToPosition(tester, testValue.length));
      await tester.pump();

      // Expect the position the magnifier gets to have moved.
      expect(firstDragGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

      await gesture.up();
      await tester.pump();

      expect(find.byKey(fakeMagnifier.key!), findsNothing);
    });

    testWidgets(
      'Can drag to show, unshow, and update magnifier',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Center(
                child: TextField(
                  dragStartBehavior: DragStartBehavior.down,
                  controller: controller,
                  magnifierConfiguration: TextMagnifierConfiguration(
                    magnifierBuilder: (
                      BuildContext context,
                      MagnifierController controller,
                      ValueNotifier<MagnifierInfo> localMagnifierInfo,
                    ) {
                      magnifierInfo = localMagnifierInfo;
                      return fakeMagnifier;
                    },
                  ),
                ),
              ),
            ),
          ),
        );

        const String testValue = 'abc def ghi';
        await tester.enterText(find.byType(TextField), testValue);
        await skipPastScrollingAnimation(tester);

        // Tap at '|a' to move the selection to position 0.
        await tester.tapAt(textOffsetToPosition(tester, 0));
        await tester.pumpAndSettle(kDoubleTapTimeout);
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 0);
        expect(find.byKey(fakeMagnifier.key!), findsNothing);

        // Start a drag gesture to move the selection to the dragged position, showing
        // the magnifier.
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 0));
        await tester.pump();

        await gesture.moveTo(textOffsetToPosition(tester, 5));
        await tester.pump();
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 5);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);

        Offset firstDragGesturePosition = magnifierInfo.value.globalGesturePosition;

        await gesture.moveTo(textOffsetToPosition(tester, 10));
        await tester.pump();
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 10);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
        // Expect the position the magnifier gets to have moved.
        expect(firstDragGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

        // The magnifier should hide when the drag ends.
        await gesture.up();
        await tester.pump();
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 10);
        expect(find.byKey(fakeMagnifier.key!), findsNothing);

        // Start a double-tap select the word at the tapped position.
        await gesture.down(textOffsetToPosition(tester, 1));
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(tester, 1));
        await tester.pumpAndSettle();
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 3);

        // Start a drag gesture to extend the selection word-by-word, showing the
        // magnifier.
        await gesture.moveTo(textOffsetToPosition(tester, 5));
        await tester.pump();
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 7);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);

        firstDragGesturePosition = magnifierInfo.value.globalGesturePosition;

        await gesture.moveTo(textOffsetToPosition(tester, 10));
        await tester.pump();
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 11);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
        // Expect the position the magnifier gets to have moved.
        expect(firstDragGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

        // The magnifier should hide when the drag ends.
        await gesture.up();
        await tester.pump();
        expect(controller.selection.baseOffset, 0);
        expect(controller.selection.extentOffset, 11);
        expect(find.byKey(fakeMagnifier.key!), findsNothing);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
    );

    testWidgets(
      'Can long press to show, unshow, and update magnifier',
      (WidgetTester tester) async {
        final TextEditingController controller = _textEditingController();
        final bool isTargetPlatformAndroid = defaultTargetPlatform == TargetPlatform.android;
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Center(
                child: TextField(
                  dragStartBehavior: DragStartBehavior.down,
                  controller: controller,
                  magnifierConfiguration: TextMagnifierConfiguration(
                    magnifierBuilder: (
                      BuildContext context,
                      MagnifierController controller,
                      ValueNotifier<MagnifierInfo> localMagnifierInfo,
                    ) {
                      magnifierInfo = localMagnifierInfo;
                      return fakeMagnifier;
                    },
                  ),
                ),
              ),
            ),
          ),
        );

        const String testValue = 'abc def ghi';
        await tester.enterText(find.byType(TextField), testValue);
        await skipPastScrollingAnimation(tester);

        // Tap at 'e' to set the selection to position 5 on Android.
        // Tap at 'e' to set the selection to the closest word edge, which is position 4 on iOS.
        await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
        await tester.pumpAndSettle(const Duration(milliseconds: 300));
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, isTargetPlatformAndroid ? 5 : 7);
        expect(find.byKey(fakeMagnifier.key!), findsNothing);

        // Long press the 'e' to select 'def' on Android and show magnifier.
        // Long press the 'e' to move the cursor in front of the 'e' on iOS and show the magnifier.
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(tester, testValue.indexOf('e')),
        );
        await tester.pumpAndSettle(const Duration(milliseconds: 1000));
        expect(controller.selection.baseOffset, isTargetPlatformAndroid ? 4 : 5);
        expect(controller.selection.extentOffset, isTargetPlatformAndroid ? 7 : 5);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);

        final Offset firstLongPressGesturePosition = magnifierInfo.value.globalGesturePosition;

        // Move the gesture to 'h' on Android to update the magnifier and select 'ghi'.
        // Move the gesture to 'h' on iOS to update the magnifier and move the cursor to 'h'.
        await gesture.moveTo(textOffsetToPosition(tester, testValue.indexOf('h')));
        await tester.pumpAndSettle();
        expect(controller.selection.baseOffset, isTargetPlatformAndroid ? 4 : 9);
        expect(controller.selection.extentOffset, isTargetPlatformAndroid ? 11 : 9);
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
        // Expect the position the magnifier gets to have moved.
        expect(firstLongPressGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

        // End the long press to hide the magnifier.
        await gesture.up();
        await tester.pumpAndSettle();
        expect(find.byKey(fakeMagnifier.key!), findsNothing);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
    );

    testWidgets(
      'magnifier does not show when tapping outside field',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/128321
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Padding(
                padding: const EdgeInsets.all(20),
                child: TextField(
                  magnifierConfiguration: TextMagnifierConfiguration(
                    magnifierBuilder: (
                      BuildContext context,
                      MagnifierController controller,
                      ValueNotifier<MagnifierInfo> localMagnifierInfo,
                    ) {
                      magnifierInfo = localMagnifierInfo;
                      return fakeMagnifier;
                    },
                  ),
                  onTapOutside: (PointerDownEvent event) {
                    FocusManager.instance.primaryFocus?.unfocus();
                  },
                ),
              ),
            ),
          ),
        );

        await tester.tapAt(tester.getCenter(find.byType(TextField)));
        await tester.pump();

        expect(find.byKey(fakeMagnifier.key!), findsNothing);

        final TestGesture gesture = await tester.startGesture(
          tester.getBottomLeft(find.byType(TextField)) - const Offset(10.0, 20.0),
        );
        await tester.pump();
        expect(find.byKey(fakeMagnifier.key!), findsNothing);
        await gesture.up();
        await tester.pump();

        expect(find.byKey(fakeMagnifier.key!), findsNothing);
      },
      // [intended] only applies to platforms where we supply the context menu.
      skip: isContextMenuProvidedByPlatform,
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );
  });

  group('TapRegion integration', () {
    testWidgets('Tapping outside loses focus on desktop', (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: Opacity(
                  opacity: 0.5,
                  child: TextField(
                    autofocus: true,
                    focusNode: focusNode,
                    decoration: const InputDecoration(
                      hintText: 'Placeholder',
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      await tester.tapAt(const Offset(10, 10));
      await tester.pump();

      expect(focusNode.hasPrimaryFocus, isFalse);
    }, variant: TargetPlatformVariant.desktop());

    testWidgets("Tapping outside doesn't lose focus on mobile", (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: Opacity(
                  opacity: 0.5,
                  child: TextField(
                    autofocus: true,
                    focusNode: focusNode,
                    decoration: const InputDecoration(
                      hintText: 'Placeholder',
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      await tester.tapAt(const Offset(10, 10));
      await tester.pump();

      // Focus is lost on mobile browsers, but not mobile apps.
      expect(focusNode.hasPrimaryFocus, kIsWeb ? isFalse : isTrue);
    }, variant: TargetPlatformVariant.mobile());

    testWidgets(
      "Tapping on toolbar doesn't lose focus",
      (WidgetTester tester) async {
        final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
        final TextEditingController controller = _textEditingController(text: 'A B C');
        addTearDown(focusNode.dispose);
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: SizedBox(
                  width: 100,
                  height: 100,
                  child: Opacity(
                    opacity: 0.5,
                    child: TextField(
                      controller: controller,
                      focusNode: focusNode,
                      decoration: const InputDecoration(hintText: 'Placeholder'),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );

        // The selectWordsInRange with SelectionChangedCause.tap seems to be needed to show the toolbar.
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.renderEditable.selectWordsInRange(
          from: Offset.zero,
          cause: SelectionChangedCause.tap,
        );

        final Offset aPosition = textOffsetToPosition(tester, 1);

        // Right clicking shows the menu.
        final TestGesture gesture = await tester.startGesture(
          aPosition,
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Sanity check that the toolbar widget exists.
        expect(find.text('Copy'), findsOneWidget);
        expect(focusNode.hasPrimaryFocus, isTrue);

        // Now tap on it to see if we lose focus.
        await tester.tap(find.text('Copy'));
        await tester.pumpAndSettle();

        expect(focusNode.hasPrimaryFocus, isTrue);
      },
      variant: TargetPlatformVariant.all(),
      skip: isBrowser, // [intended] On the web, the toolbar isn't rendered by Flutter.
    );

    testWidgets("Tapping on input decorator doesn't lose focus", (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: Opacity(
                  opacity: 0.5,
                  child: TextField(
                    autofocus: true,
                    focusNode: focusNode,
                    decoration: const InputDecoration(
                      hintText: 'Placeholder',
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      final Rect decorationBox = tester.getRect(find.byType(TextField));
      // Tap just inside the decoration, but not inside the EditableText.
      await tester.tapAt(decorationBox.topLeft + const Offset(1, 1));
      await tester.pump();

      expect(focusNode.hasPrimaryFocus, isTrue);
    }, variant: TargetPlatformVariant.all());

    // PointerDownEvents can't be trackpad events, apparently, so we skip that one.
    for (final PointerDeviceKind pointerDeviceKind
        in PointerDeviceKind.values.toSet()..remove(PointerDeviceKind.trackpad)) {
      testWidgets(
        'Default TextField handling of onTapOutside follows platform conventions for ${pointerDeviceKind.name}',
        (WidgetTester tester) async {
          final FocusNode focusNode = FocusNode(debugLabel: 'Test');
          addTearDown(focusNode.dispose);
          await tester.pumpWidget(
            MaterialApp(
              home: Scaffold(
                body: Column(
                  children: <Widget>[
                    const Text('Outside'),
                    TextField(autofocus: true, focusNode: focusNode),
                  ],
                ),
              ),
            ),
          );
          await tester.pump();

          Future<void> click(Finder finder) async {
            final TestGesture gesture = await tester.startGesture(
              tester.getCenter(finder),
              kind: pointerDeviceKind,
            );
            await gesture.up();
            await gesture.removePointer();
          }

          expect(focusNode.hasPrimaryFocus, isTrue);

          await click(find.text('Outside'));

          switch (pointerDeviceKind) {
            case PointerDeviceKind.touch:
              switch (defaultTargetPlatform) {
                case TargetPlatform.iOS:
                case TargetPlatform.android:
                case TargetPlatform.fuchsia:
                  expect(focusNode.hasPrimaryFocus, equals(!kIsWeb));
                case TargetPlatform.linux:
                case TargetPlatform.macOS:
                case TargetPlatform.windows:
                  expect(focusNode.hasPrimaryFocus, isFalse);
              }
            case PointerDeviceKind.mouse:
            case PointerDeviceKind.stylus:
            case PointerDeviceKind.invertedStylus:
            case PointerDeviceKind.trackpad:
            case PointerDeviceKind.unknown:
              expect(focusNode.hasPrimaryFocus, isFalse);
          }
        },
        variant: TargetPlatformVariant.all(),
      );
    }
  });

  testWidgets(
    'Builds the corresponding default spell check toolbar by platform',
    (WidgetTester tester) async {
      tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
      late final BuildContext builderContext;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  builderContext = context;
                  return const TextField(
                    autofocus: true,
                    spellCheckConfiguration: SpellCheckConfiguration(),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Allow the autofocus to take effect.
      await tester.pump();

      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );
      editableTextState.spellCheckResults = const SpellCheckResults('', <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 0, end: 0), <String>['something']),
      ]);
      final Widget spellCheckToolbar = TextField.defaultSpellCheckSuggestionsToolbarBuilder(
        builderContext,
        editableTextState,
      );

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(spellCheckToolbar, isA<CupertinoSpellCheckSuggestionsToolbar>());
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(spellCheckToolbar, isA<SpellCheckSuggestionsToolbar>());
      }
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'Builds the corresponding default spell check configuration by platform',
    (WidgetTester tester) async {
      tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;

      final SpellCheckConfiguration expectedConfiguration;
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expectedConfiguration = SpellCheckConfiguration(
            misspelledTextStyle: CupertinoTextField.cupertinoMisspelledTextStyle,
            misspelledSelectionColor: CupertinoTextField.kMisspelledSelectionColor,
            spellCheckService: DefaultSpellCheckService(),
            spellCheckSuggestionsToolbarBuilder:
                CupertinoTextField.defaultSpellCheckSuggestionsToolbarBuilder,
          );
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expectedConfiguration = SpellCheckConfiguration(
            misspelledTextStyle: TextField.materialMisspelledTextStyle,
            spellCheckService: DefaultSpellCheckService(),
            spellCheckSuggestionsToolbarBuilder:
                TextField.defaultSpellCheckSuggestionsToolbarBuilder,
          );
      }
      await tester.pumpWidget(
        const MaterialApp(
          home: Scaffold(
            body: Center(
              child: TextField(autofocus: true, spellCheckConfiguration: SpellCheckConfiguration()),
            ),
          ),
        ),
      );

      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );

      expect(
        editableTextState.spellCheckConfiguration.misspelledTextStyle,
        expectedConfiguration.misspelledTextStyle,
      );
      expect(
        editableTextState.spellCheckConfiguration.misspelledSelectionColor,
        expectedConfiguration.misspelledSelectionColor,
      );
      expect(
        editableTextState.spellCheckConfiguration.spellCheckService.runtimeType,
        expectedConfiguration.spellCheckService.runtimeType,
      );
      expect(
        editableTextState.spellCheckConfiguration.spellCheckSuggestionsToolbarBuilder,
        expectedConfiguration.spellCheckSuggestionsToolbarBuilder,
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'text selection toolbar is hidden on tap down on desktop platforms',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'blah1 blah2');
      await tester.pumpWidget(
        MaterialApp(home: Scaffold(body: Center(child: TextField(controller: controller)))),
      );

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 8),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      gesture = await tester.startGesture(
        textOffsetToPosition(tester, 2),
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();

      // After the gesture is down but not up, the toolbar is already gone.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
    variant: TargetPlatformVariant.all(excluding: TargetPlatformVariant.mobile().values),
  );

  testWidgets(
    'Text processing actions are added to the toolbar',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Long press to put the cursor after the "F".
      final int index = initialText.indexOf('F');
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pump();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 14));

      // The toolbar is visible and the text processing actions are visible on Android.
      final bool areTextActionsSupported = defaultTargetPlatform == TargetPlatform.android;
      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
      expect(find.text(fakeAction1Label), areTextActionsSupported ? findsOneWidget : findsNothing);
      expect(find.text(fakeAction2Label), areTextActionsSupported ? findsOneWidget : findsNothing);
    },
    variant: TargetPlatformVariant.all(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Text processing actions are not added to the toolbar for obscured text',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(obscureText: true, controller: controller))),
      );

      // Long press to put the cursor after the "F".
      final int index = initialText.indexOf('F');
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pump();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 14));

      // The toolbar is visible but does not contain the text processing actions.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
      expect(find.text(fakeAction1Label), findsNothing);
      expect(find.text(fakeAction2Label), findsNothing);
    },
    variant: TargetPlatformVariant.all(),
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Text processing actions are not added to the toolbar if selection is collapsed (Android only)',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Open the text selection toolbar.
      await showSelectionMenuAt(tester, controller, initialText.indexOf('F'));
      await skipPastScrollingAnimation(tester);

      // The toolbar is visible but does not contain the text processing actions.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
      expect(controller.selection.isCollapsed, true);

      expect(find.text(fakeAction1Label), findsNothing);
      expect(find.text(fakeAction2Label), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Invoke a text processing action that does not return a value (Android only)',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Long press to put the cursor after the "F".
      final int index = initialText.indexOf('F');
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pump();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 14));

      // Run an action that does not return a processed text.
      await tester.tap(find.text(fakeAction2Label));
      await tester.pump(const Duration(milliseconds: 200));

      // The action was correctly called.
      expect(mockProcessTextHandler.lastCalledActionId, fakeAction2Id);
      expect(mockProcessTextHandler.lastTextToProcess, 'Flutter');

      // The text field was not updated.
      expect(controller.text, initialText);

      // The toolbar is no longer visible.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Invoking a text processing action that returns a value replaces the selection (Android only)',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(controller: controller))),
      );

      // Long press to put the cursor after the "F".
      final int index = initialText.indexOf('F');
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pump();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 14));

      // Run an action that returns a processed text.
      await tester.tap(find.text(fakeAction1Label));
      await tester.pump(const Duration(milliseconds: 200));

      // The action was correctly called.
      expect(mockProcessTextHandler.lastCalledActionId, fakeAction1Id);
      expect(mockProcessTextHandler.lastTextToProcess, 'Flutter');

      // The text field was updated.
      expect(controller.text, 'I love Flutter!!!');

      // The toolbar is no longer visible.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets(
    'Invoking a text processing action that returns a value does not replace the selection of a readOnly text field (Android only)',
    (WidgetTester tester) async {
      const String initialText = 'I love Flutter';
      final TextEditingController controller = _textEditingController(text: initialText);
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      await tester.pumpWidget(
        MaterialApp(home: Material(child: TextField(readOnly: true, controller: controller))),
      );

      // Long press to put the cursor after the "F".
      final int index = initialText.indexOf('F');
      await tester.longPressAt(textOffsetToPosition(tester, index));
      await tester.pump();

      // Double tap on the same location to select the word around the cursor.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 14));

      // Run an action that returns a processed text.
      await tester.tap(find.text(fakeAction1Label));
      await tester.pump(const Duration(milliseconds: 200));

      // The Action was correctly called.
      expect(mockProcessTextHandler.lastCalledActionId, fakeAction1Id);
      expect(mockProcessTextHandler.lastTextToProcess, 'Flutter');

      // The text field was not updated.
      expect(controller.text, initialText);

      // The toolbar is no longer visible.
      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    },
    // [intended] only applies to platforms where we supply the context menu.
    skip: isContextMenuProvidedByPlatform,
  );

  testWidgets('Start the floating cursor on long tap', (WidgetTester tester) async {
    EditableText.debugDeterministicCursor = true;
    final TextEditingController controller = _textEditingController(text: 'abcd');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: RepaintBoundary(
              key: const ValueKey<int>(1),
              child: TextField(autofocus: true, controller: controller),
            ),
          ),
        ),
      ),
    );
    // Wait for autofocus.
    await tester.pumpAndSettle();
    final Offset textFieldCenter = tester.getCenter(find.byType(TextField));
    final TestGesture gesture = await tester.startGesture(textFieldCenter);
    await tester.pump(kLongPressTimeout);
    await expectLater(
      find.byKey(const ValueKey<int>(1)),
      matchesGoldenFile('text_field_floating_cursor.regular_and_floating_both.material.0.png'),
    );
    await gesture.moveTo(Offset(10, textFieldCenter.dy));
    await tester.pump();
    await expectLater(
      find.byKey(const ValueKey<int>(1)),
      matchesGoldenFile('text_field_floating_cursor.only_floating_cursor.material.0.png'),
    );
    await gesture.up();
    EditableText.debugDeterministicCursor = false;
  }, variant: TargetPlatformVariant.only(TargetPlatform.iOS));

  testWidgets(
    'Cursor should not blink when long-pressing to show floating cursor.',
    (WidgetTester tester) async {
      final TextEditingController controller = _textEditingController(text: 'abcdefghijklmnopqr');
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextField(
                autofocus: true,
                controller: controller,
                cursorOpacityAnimates: false,
              ),
            ),
          ),
        ),
      );
      final EditableTextState state = tester.state(find.byType(EditableText));
      Future<void> checkCursorBlinking({bool isBlinking = true}) async {
        bool initialShowCursor = true;
        if (isBlinking) {
          initialShowCursor = state.renderEditable.showCursor.value;
        }
        await tester.pump(state.cursorBlinkInterval);
        expect(
          state.cursorCurrentlyVisible,
          equals(isBlinking ? !initialShowCursor : initialShowCursor),
        );
        await tester.pump(state.cursorBlinkInterval);
        expect(state.cursorCurrentlyVisible, equals(initialShowCursor));
        await tester.pump(state.cursorBlinkInterval);
        expect(
          state.cursorCurrentlyVisible,
          equals(isBlinking ? !initialShowCursor : initialShowCursor),
        );
        await tester.pump(state.cursorBlinkInterval);
        expect(state.cursorCurrentlyVisible, equals(initialShowCursor));
      }

      // Wait for autofocus.
      await tester.pumpAndSettle();
      // Before long-pressing, the cursor should blink.
      await checkCursorBlinking();

      final TestGesture gesture = await tester.startGesture(
        tester.getTopLeft(find.byType(TextField)),
      );
      await tester.pump(kLongPressTimeout);
      // When long-pressing, the cursor shouldn't blink.
      await checkCursorBlinking(isBlinking: false);
      await gesture.moveBy(const Offset(20, 0));
      await tester.pump();
      // When long-pressing and dragging to move the cursor, the cursor shouldn't blink.
      await checkCursorBlinking(isBlinking: false);
      await gesture.up();
      // After finishing the long-press, the cursor should blink.
      await checkCursorBlinking();
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('when enabled listens to onFocus events and gains focus', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextField(focusNode: focusNode)))),
    );
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.isTextField,
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isEnabled,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.tap,
                            if (defaultTargetPlatform == TargetPlatform.windows ||
                                defaultTargetPlatform == TargetPlatform.macOS ||
                                defaultTargetPlatform == TargetPlatform.linux)
                              SemanticsAction.didGainAccessibilityFocus,
                            if (defaultTargetPlatform == TargetPlatform.windows ||
                                defaultTargetPlatform == TargetPlatform.macOS ||
                                defaultTargetPlatform == TargetPlatform.linux)
                              SemanticsAction.didLoseAccessibilityFocus,
                            SemanticsAction.focus,
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    expect(focusNode.hasFocus, isFalse);
    semanticsOwner.performAction(4, SemanticsAction.focus);
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    semantics.dispose();
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'when disabled does not listen to onFocus events or gain focus',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(focusNode: focusNode, enabled: false))),
        ),
      );
      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.isTextField,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isReadOnly,
                            ],
                            actions: <SemanticsAction>[
                              if (defaultTargetPlatform == TargetPlatform.windows ||
                                  defaultTargetPlatform == TargetPlatform.macOS ||
                                  defaultTargetPlatform == TargetPlatform.linux)
                                SemanticsAction.didGainAccessibilityFocus,
                              if (defaultTargetPlatform == TargetPlatform.windows ||
                                  defaultTargetPlatform == TargetPlatform.macOS ||
                                  defaultTargetPlatform == TargetPlatform.linux)
                                SemanticsAction.didLoseAccessibilityFocus,
                            ],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      expect(focusNode.hasFocus, isFalse);
      semanticsOwner.performAction(4, SemanticsAction.focus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isFalse);
      semantics.dispose();
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'when receives SemanticsAction.focus while already focused, shows keyboard',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(focusNode: focusNode)))),
      );
      focusNode.requestFocus();
      await tester.pumpAndSettle();

      tester.testTextInput.log.clear();
      expect(focusNode.hasFocus, isTrue);
      semanticsOwner.performAction(4, SemanticsAction.focus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);
      expect(tester.testTextInput.log.single.method, 'TextInput.show');

      semantics.dispose();
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'when receives SemanticsAction.focus while focused but read-only, does not show keyboard',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Center(child: TextField(focusNode: focusNode, readOnly: true))),
        ),
      );
      focusNode.requestFocus();
      await tester.pumpAndSettle();

      tester.testTextInput.log.clear();
      expect(focusNode.hasFocus, isTrue);
      semanticsOwner.performAction(4, SemanticsAction.focus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);
      expect(tester.testTextInput.log, isEmpty);

      semantics.dispose();
    },
    variant: TargetPlatformVariant.all(),
  );
}

/// A Simple widget for testing the obscure text.
class _ObscureTextTestWidget extends StatefulWidget {
  const _ObscureTextTestWidget({required this.controller});

  final TextEditingController controller;
  @override
  _ObscureTextTestWidgetState createState() => _ObscureTextTestWidgetState();
}

class _ObscureTextTestWidgetState extends State<_ObscureTextTestWidget> {
  bool _obscureText = false;
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Builder(
          builder: (BuildContext context) {
            return Column(
              children: <Widget>[
                TextField(obscureText: _obscureText, controller: widget.controller),
                ElevatedButton(
                  onPressed:
                      () => setState(() {
                        _obscureText = !_obscureText;
                      }),
                  child: const SizedBox.shrink(),
                ),
              ],
            );
          },
        ),
      ),
    );
  }
}

typedef FormatEditUpdateCallback =
    void Function(TextEditingValue oldValue, TextEditingValue newValue);

// On web, key events in text fields are handled by the browser.
const bool areKeyEventsHandledByPlatform = isBrowser;

class CupertinoLocalizationsDelegate extends LocalizationsDelegate<CupertinoLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<CupertinoLocalizations> load(Locale locale) => DefaultCupertinoLocalizations.load(locale);

  @override
  bool shouldReload(CupertinoLocalizationsDelegate old) => false;
}

class MaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<MaterialLocalizations> load(Locale locale) => DefaultMaterialLocalizations.load(locale);

  @override
  bool shouldReload(MaterialLocalizationsDelegate old) => false;
}

class WidgetsLocalizationsDelegate extends LocalizationsDelegate<WidgetsLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<WidgetsLocalizations> load(Locale locale) => DefaultWidgetsLocalizations.load(locale);

  @override
  bool shouldReload(WidgetsLocalizationsDelegate old) => false;
}

Widget overlay({required Widget child}) {
  final OverlayEntry entry = OverlayEntry(
    builder: (BuildContext context) {
      return Center(child: Material(child: child));
    },
  );
  addTearDown(
    () =>
        entry
          ..remove()
          ..dispose(),
  );
  return overlayWithEntry(entry);
}

Widget overlayWithEntry(OverlayEntry entry) {
  return Localizations(
    locale: const Locale('en', 'US'),
    delegates: <LocalizationsDelegate<dynamic>>[
      WidgetsLocalizationsDelegate(),
      MaterialLocalizationsDelegate(),
      CupertinoLocalizationsDelegate(),
    ],
    child: DefaultTextEditingShortcuts(
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(800.0, 600.0)),
          child: Overlay(initialEntries: <OverlayEntry>[entry]),
        ),
      ),
    ),
  );
}

Widget boilerplate({required Widget child, ThemeData? theme}) {
  return MaterialApp(
    theme: theme,
    home: Localizations(
      locale: const Locale('en', 'US'),
      delegates: <LocalizationsDelegate<dynamic>>[
        WidgetsLocalizationsDelegate(),
        MaterialLocalizationsDelegate(),
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(800.0, 600.0)),
          child: Center(child: Material(child: child)),
        ),
      ),
    ),
  );
}

Future<void> skipPastScrollingAnimation(WidgetTester tester) async {
  await tester.pump();
  await tester.pump(const Duration(milliseconds: 200));
}

double getOpacity(WidgetTester tester, Finder finder) {
  return tester
      .widget<FadeTransition>(find.ancestor(of: finder, matching: find.byType(FadeTransition)))
      .opacity
      .value;
}

class TestFormatter extends TextInputFormatter {
  TestFormatter(this.onFormatEditUpdate);
  FormatEditUpdateCallback onFormatEditUpdate;
  @override
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue) {
    onFormatEditUpdate(oldValue, newValue);
    return newValue;
  }
}

FocusNode _focusNode() {
  final FocusNode result = FocusNode();
  addTearDown(result.dispose);
  return result;
}

TextEditingController _textEditingController({String text = ''}) {
  final TextEditingController result = TextEditingController(text: text);
  addTearDown(result.dispose);
  return result;
}
