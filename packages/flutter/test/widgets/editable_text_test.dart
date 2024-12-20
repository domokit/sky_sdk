// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show jsonDecode;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../widgets/clipboard_utils.dart';
import 'editable_text_utils.dart';
import 'live_text_utils.dart';
import 'semantics_tester.dart';

Matcher matchesMethodCall(String method, {dynamic args}) =>
    _MatchesMethodCall(method, arguments: args == null ? null : wrapMatcher(args));

class _MatchesMethodCall extends Matcher {
  const _MatchesMethodCall(this.name, {this.arguments});

  final String name;
  final Matcher? arguments;

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    if (item is MethodCall && item.method == name) {
      return arguments?.matches(item.arguments, matchState) ?? true;
    }
    return false;
  }

  @override
  Description describe(Description description) {
    final Description newDescription = description.add('has method name: ').addDescriptionOf(name);
    if (arguments != null) {
      newDescription.add(' with arguments: ').addDescriptionOf(arguments);
    }
    return newDescription;
  }
}

const TextStyle textStyle = TextStyle();
const Color cursorColor = Color.fromARGB(0xFF, 0xFF, 0x00, 0x00);

enum HandlePositionInViewport { leftEdge, rightEdge, within }

typedef _VoidFutureCallback = Future<void> Function();

TextEditingValue collapsedAtEnd(String text) {
  return TextEditingValue(text: text, selection: TextSelection.collapsed(offset: text.length));
}

void main() {
  late TextEditingController controller;
  late FocusNode focusNode;
  late FocusScopeNode focusScopeNode;

  setUp(() async {
    final MockClipboard mockClipboard = MockClipboard();
    TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    debugResetSemanticsIdCounter();
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
    controller = TextEditingController();
    focusNode = FocusNode(debugLabel: 'EditableText Node');
    focusScopeNode = FocusScopeNode(debugLabel: 'EditableText Scope Node');
  });

  tearDown(() {
    TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
    controller.dispose();
    focusNode.dispose();
    focusScopeNode.dispose();
  });

  // Tests that the desired keyboard action button is requested.
  //
  // More technically, when an EditableText is given a particular [action], Flutter
  // requests [serializedActionName] when attaching to the platform's input
  // system.
  Future<void> desiredKeyboardActionIsRequested({
    required WidgetTester tester,
    TextInputAction? action,
    String serializedActionName = '',
  }) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              textInputAction: action,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals(serializedActionName));
  }

  testWidgets(
    'Tapping the Live Text button calls onLiveTextInput',
    (WidgetTester tester) async {
      bool invokedLiveTextInputSuccessfully = false;
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                selectionControls: materialTextSelectionHandleControls,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  return CupertinoAdaptiveTextSelectionToolbar.editable(
                    key: key,
                    clipboardStatus: ClipboardStatus.pasteable,
                    onCopy: null,
                    onCut: null,
                    onPaste: null,
                    onSelectAll: null,
                    onLookUp: null,
                    onSearchWeb: null,
                    onShare: null,
                    onLiveTextInput: () {
                      invokedLiveTextInputSuccessfully = true;
                    },
                    anchors: const TextSelectionToolbarAnchors(primaryAnchor: Offset.zero),
                  );
                },
              ),
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
      expect(findLiveTextButton(), findsOneWidget);
      await tester.tap(findLiveTextButton());
      await tester.pump();
      expect(invokedLiveTextInputSuccessfully, isTrue);
    },
    skip: kIsWeb, // [intended]
  );

  group('Check the passed groupId value', () {
    testWidgets('The value of the passed-in groupId should match the groupId of the EditableText', (
      WidgetTester tester,
    ) async {
      final List<String> groupIds = <String>['Group A', 'Group B', 'Group C'];
      final List<GlobalKey> keys = List<GlobalKey>.generate(3, (_) => GlobalKey());
      final List<Widget> inputFields = <Widget>[
        TextFormField(key: keys[0], groupId: groupIds[0]),
        CupertinoTextField(key: keys[1], groupId: groupIds[1]),
        TextField(key: keys[2], groupId: groupIds[2]),
      ];

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: Column(
              children:
                  inputFields.map((Widget child) {
                    return Material(child: child);
                  }).toList(),
            ),
          ),
        ),
      );

      await tester.pump();

      for (int i = 0; i < 3; i++) {
        final EditableText editableText = tester.widget(
          find.descendant(of: find.byKey(keys[i]), matching: find.byType(EditableText)),
        );
        expect(editableText.groupId, groupIds[i]);
      }
    });

    testWidgets(
      'When the value of groupId is not passed in, the default type should be EditableText',
      (WidgetTester tester) async {
        final List<GlobalKey> keys = List<GlobalKey>.generate(3, (_) => GlobalKey());
        final List<Widget> inputFields = <Widget>[
          TextFormField(key: keys[0]),
          CupertinoTextField(key: keys[1]),
          TextField(key: keys[2]),
        ];

        await tester.pumpWidget(
          MaterialApp(
            home: Align(
              alignment: Alignment.topLeft,
              child: Column(
                children:
                    inputFields.map((Widget child) {
                      return Material(child: child);
                    }).toList(),
              ),
            ),
          ),
        );

        await tester.pump();

        for (int i = 0; i < 3; i++) {
          final EditableText editableText = tester.widget(
            find.descendant(of: find.byKey(keys[i]), matching: find.byType(EditableText)),
          );
          expect(editableText.groupId == EditableText, true);
        }
      },
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/126312.
  testWidgets('when open input connection in didUpdateWidget, should not throw', (
    WidgetTester tester,
  ) async {
    final Key key = GlobalKey();

    final TextEditingController controller1 = TextEditingController(text: 'blah blah');
    addTearDown(controller1.dispose);
    final TextEditingController controller2 = TextEditingController(text: 'blah blah');
    addTearDown(controller2.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          key: key,
          backgroundCursorColor: Colors.grey,
          controller: controller1,
          focusNode: focusNode,
          readOnly: true,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    focusNode.requestFocus();
    await tester.pump();

    // Reparent the EditableText, so that the parent has not yet been laid
    // out when didUpdateWidget is called.
    await tester.pumpWidget(
      MaterialApp(
        home: FractionalTranslation(
          translation: const Offset(0.1, 0.1),
          child: EditableText(
            key: key,
            backgroundCursorColor: Colors.grey,
            controller: controller2,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      ),
    );
  });

  testWidgets('Text with selection can be shown on the screen when the keyboard shown', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/119628
    addTearDown(tester.view.reset);

    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    controller.value = const TextEditingValue(text: 'I love flutter');

    final Widget widget = MaterialApp(
      home: Scaffold(
        body: SingleChildScrollView(
          controller: scrollController,
          child: Column(
            children: <Widget>[
              const SizedBox(height: 1000.0),
              SizedBox(
                height: 20.0,
                child: EditableText(
                  controller: controller,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(),
                  cursorColor: Colors.red,
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await tester.pumpWidget(widget);

    await tester.showKeyboard(find.byType(EditableText));
    tester.view.viewInsets = const FakeViewPadding(bottom: 500);
    controller.selection = TextSelection(baseOffset: 0, extentOffset: controller.text.length);

    await tester.pump();

    // The offset of the scrollController should change immediately after view changes its metrics.
    final double offsetAfter = scrollController.offset;
    expect(offsetAfter, isNot(0.0));
  });

  // Related issue: https://github.com/flutter/flutter/issues/98115
  testWidgets('ScheduleShowCaretOnScreen with no animation when the view changes metrics', (
    WidgetTester tester,
  ) async {
    addTearDown(tester.view.reset);

    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    final Widget widget = MaterialApp(
      home: Scaffold(
        body: SingleChildScrollView(
          controller: scrollController,
          child: Column(
            children: <Widget>[
              Column(
                children: List<Widget>.generate(5, (_) {
                  return Container(height: 1200.0, color: Colors.black12);
                }),
              ),
              SizedBox(
                height: 20,
                child: EditableText(
                  controller: controller,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(),
                  cursorColor: Colors.red,
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await tester.pumpWidget(widget);
    await tester.showKeyboard(find.byType(EditableText));
    tester.view.viewInsets = const FakeViewPadding(bottom: 500);
    await tester.pump();

    // The offset of the scrollController should change immediately after view changes its metrics.
    final double offsetAfter = scrollController.offset;
    expect(offsetAfter, isNot(0.0));
  });

  // Regression test for https://github.com/flutter/flutter/issues/34538.
  testWidgets('RTL arabic correct caret placement after trailing whitespace', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.blue,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Simulates Gboard Persian input.
    state.updateEditingValue(
      const TextEditingValue(text: 'گ', selection: TextSelection.collapsed(offset: 1)),
    );
    await tester.pump();
    double previousCaretXPosition =
        state.renderEditable.getLocalRectForCaret(state.textEditingValue.selection.base).left;

    state.updateEditingValue(
      const TextEditingValue(text: 'گی', selection: TextSelection.collapsed(offset: 2)),
    );
    await tester.pump();
    double caretXPosition =
        state.renderEditable.getLocalRectForCaret(state.textEditingValue.selection.base).left;
    expect(caretXPosition, lessThan(previousCaretXPosition));
    previousCaretXPosition = caretXPosition;

    state.updateEditingValue(
      const TextEditingValue(text: 'گیگ', selection: TextSelection.collapsed(offset: 3)),
    );
    await tester.pump();
    caretXPosition =
        state.renderEditable.getLocalRectForCaret(state.textEditingValue.selection.base).left;
    expect(caretXPosition, lessThan(previousCaretXPosition));
    previousCaretXPosition = caretXPosition;

    // Enter a whitespace in a RTL input field moves the caret to the left.
    state.updateEditingValue(
      const TextEditingValue(text: 'گیگ ', selection: TextSelection.collapsed(offset: 4)),
    );
    await tester.pump();
    caretXPosition =
        state.renderEditable.getLocalRectForCaret(state.textEditingValue.selection.base).left;
    expect(caretXPosition, lessThan(previousCaretXPosition));

    expect(state.currentTextEditingValue.text, equals('گیگ '));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/78550.

  testWidgets('has expected defaults', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    final EditableText editableText = tester.firstWidget(find.byType(EditableText));
    expect(editableText.maxLines, equals(1));
    expect(editableText.obscureText, isFalse);
    expect(editableText.autocorrect, isTrue);
    expect(editableText.enableSuggestions, isTrue);
    expect(editableText.enableIMEPersonalizedLearning, isTrue);
    expect(editableText.textAlign, TextAlign.start);
    expect(editableText.cursorWidth, 2.0);
    expect(editableText.cursorHeight, isNull);
    expect(editableText.textHeightBehavior, isNull);
  });

  testWidgets('when backgroundCursorColor is updated, RenderEditable should be updated', (
    WidgetTester tester,
  ) async {
    Widget buildWidget(Color backgroundCursorColor) {
      return MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            backgroundCursorColor: backgroundCursorColor,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget(Colors.red));
    await tester.pumpWidget(buildWidget(Colors.green));

    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    expect(render.backgroundCursorColor, Colors.green);
  });

  testWidgets('text keyboard is requested when maxLines is default', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    final EditableText editableText = tester.firstWidget(find.byType(EditableText));
    expect(editableText.maxLines, equals(1));
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.text'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.done'));
  });

  testWidgets('Keyboard is configured for "unspecified" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.unspecified,
      serializedActionName: 'TextInputAction.unspecified',
    );
  });

  testWidgets('Keyboard is configured for "none" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.none,
      serializedActionName: 'TextInputAction.none',
    );
  });

  testWidgets('Keyboard is configured for "done" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.done,
      serializedActionName: 'TextInputAction.done',
    );
  });

  testWidgets('Keyboard is configured for "send" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.send,
      serializedActionName: 'TextInputAction.send',
    );
  });

  testWidgets('Keyboard is configured for "go" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.go,
      serializedActionName: 'TextInputAction.go',
    );
  });

  testWidgets('Keyboard is configured for "search" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.search,
      serializedActionName: 'TextInputAction.search',
    );
  });

  testWidgets('Keyboard is configured for "send" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.send,
      serializedActionName: 'TextInputAction.send',
    );
  });

  testWidgets('Keyboard is configured for "next" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.next,
      serializedActionName: 'TextInputAction.next',
    );
  });

  testWidgets('Keyboard is configured for "previous" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.previous,
      serializedActionName: 'TextInputAction.previous',
    );
  });

  testWidgets('Keyboard is configured for "continue" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.continueAction,
      serializedActionName: 'TextInputAction.continueAction',
    );
  });

  testWidgets('Keyboard is configured for "join" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.join,
      serializedActionName: 'TextInputAction.join',
    );
  });

  testWidgets('Keyboard is configured for "route" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.route,
      serializedActionName: 'TextInputAction.route',
    );
  });

  testWidgets('Keyboard is configured for "emergencyCall" action when explicitly requested', (
    WidgetTester tester,
  ) async {
    await desiredKeyboardActionIsRequested(
      tester: tester,
      action: TextInputAction.emergencyCall,
      serializedActionName: 'TextInputAction.emergencyCall',
    );
  });

  testWidgets('insertContent does not throw and parses data correctly', (
    WidgetTester tester,
  ) async {
    String? latestUri;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              contentInsertionConfiguration: ContentInsertionConfiguration(
                onContentInserted: (KeyboardInsertedContent content) {
                  latestUri = content.uri;
                },
                allowedMimeTypes: const <String>['image/gif'],
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.enterText(find.byType(EditableText), 'test');
    await tester.idle();

    const String uri = 'content://com.google.android.inputmethod.latin.fileprovider/test.gif';
    final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
      'args': <dynamic>[
        -1,
        'TextInputAction.commitContent',
        jsonDecode('{"mimeType": "image/gif", "data": [0,1,0,1,0,1,0,0,0], "uri": "$uri"}'),
      ],
      'method': 'TextInputClient.performAction',
    });

    Object? error;
    try {
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );
    } catch (e) {
      error = e;
    }
    expect(error, isNull);
    expect(latestUri, equals(uri));
  });

  testWidgets('onAppPrivateCommand does not throw', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();

    final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
      'args': <dynamic>[
        -1, // The magic clint id that points to the current client.
        jsonDecode('{"action": "actionCommand", "data": {"input_context" : "abcdefg"}}'),
      ],
      'method': 'TextInputClient.performPrivateCommand',
    });

    Object? error;
    try {
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );
    } catch (e) {
      error = e;
    }
    expect(error, isNull);
  });

  group('Infer keyboardType from autofillHints', () {
    testWidgets(
      'infer keyboard types from autofillHints: ios',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: FocusScope(
                node: focusScopeNode,
                autofocus: true,
                child: EditableText(
                  controller: controller,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                  autofillHints: const <String>[AutofillHints.streetAddressLine1],
                ),
              ),
            ),
          ),
        );

        await tester.tap(find.byType(EditableText));
        await tester.showKeyboard(find.byType(EditableText));
        controller.text = 'test';
        await tester.idle();
        expect(tester.testTextInput.editingState!['text'], equals('test'));
        expect(
          (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
          // On web, we don't infer the keyboard type as "name". We only infer
          // on iOS and macOS.
          kIsWeb ? equals('TextInputType.address') : equals('TextInputType.name'),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets('infer keyboard types from autofillHints: non-ios', (WidgetTester tester) async {
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                autofillHints: const <String>[AutofillHints.streetAddressLine1],
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.showKeyboard(find.byType(EditableText));
      controller.text = 'test';
      await tester.idle();
      expect(tester.testTextInput.editingState!['text'], equals('test'));
      expect(
        (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
        equals('TextInputType.address'),
      );
    });

    testWidgets(
      'inferred keyboard types can be overridden: ios',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: FocusScope(
                node: focusScopeNode,
                autofocus: true,
                child: EditableText(
                  controller: controller,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                  keyboardType: TextInputType.text,
                  autofillHints: const <String>[AutofillHints.streetAddressLine1],
                ),
              ),
            ),
          ),
        );

        await tester.tap(find.byType(EditableText));
        await tester.showKeyboard(find.byType(EditableText));
        controller.text = 'test';
        await tester.idle();
        expect(tester.testTextInput.editingState!['text'], equals('test'));
        expect(
          (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
          equals('TextInputType.text'),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets('inferred keyboard types can be overridden: non-ios', (WidgetTester tester) async {
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                keyboardType: TextInputType.text,
                autofillHints: const <String>[AutofillHints.streetAddressLine1],
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.showKeyboard(find.byType(EditableText));
      controller.text = 'test';
      await tester.idle();
      expect(tester.testTextInput.editingState!['text'], equals('test'));
      expect(
        (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
        equals('TextInputType.text'),
      );
    });
  });

  testWidgets('multiline keyboard is requested when set explicitly', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              keyboardType: TextInputType.multiline,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.multiline'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.newline'));
  });

  testWidgets('EditableText sends enableInteractiveSelection to config', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              enableInteractiveSelection: true,
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              keyboardType: TextInputType.multiline,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.textInputConfiguration.enableInteractiveSelection, isTrue);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              enableInteractiveSelection: false,
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              keyboardType: TextInputType.multiline,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.textInputConfiguration.enableInteractiveSelection, isFalse);
  });

  testWidgets('EditableText sends viewId to config', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapWithView: false,
      View(
        view: FakeFlutterView(tester.view, viewId: 77),
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      ),
    );

    EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.textInputConfiguration.viewId, 77);

    await tester.pumpWidget(
      wrapWithView: false,
      View(
        view: FakeFlutterView(tester.view, viewId: 88),
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                enableInteractiveSelection: false,
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                keyboardType: TextInputType.multiline,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      ),
    );

    state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.textInputConfiguration.viewId, 88);
  });

  testWidgets('selection persists when unfocused', (WidgetTester tester) async {
    const TextEditingValue value = TextEditingValue(
      text: 'test test',
      selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 5, extentOffset: 7),
    );
    controller.value = value;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            keyboardType: TextInputType.multiline,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    expect(controller.value, value);
    expect(focusNode.hasFocus, isFalse);

    focusNode.requestFocus();
    await tester.pump();

    // On web, focusing a single-line input selects the entire field.
    final TextEditingValue webValue = value.copyWith(
      selection: TextSelection(baseOffset: 0, extentOffset: controller.value.text.length),
    );
    if (kIsWeb) {
      expect(controller.value, webValue);
    } else {
      expect(controller.value, value);
    }
    expect(focusNode.hasFocus, isTrue);

    focusNode.unfocus();
    await tester.pump();

    if (kIsWeb) {
      expect(controller.value, webValue);
    } else {
      expect(controller.value, value);
    }
    expect(focusNode.hasFocus, isFalse);
  });

  testWidgets('EditableText does not derive selection color from DefaultSelectionStyle', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/103341.
    const TextEditingValue value = TextEditingValue(
      text: 'test test',
      selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 5, extentOffset: 7),
    );
    const Color selectionColor = Colors.orange;
    controller.value = value;
    await tester.pumpWidget(
      DefaultSelectionStyle(
        selectionColor: selectionColor,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              keyboardType: TextInputType.multiline,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.renderEditable.selectionColor, null);
  });

  testWidgets('visiblePassword keyboard is requested when set explicitly', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              keyboardType: TextInputType.visiblePassword,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.visiblePassword'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.done'));
  });

  testWidgets('enableSuggestions flag is sent to the engine properly', (WidgetTester tester) async {
    const bool enableSuggestions = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              enableSuggestions: enableSuggestions,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.idle();
    expect(tester.testTextInput.setClientArgs!['enableSuggestions'], enableSuggestions);
  });

  testWidgets('enableIMEPersonalizedLearning flag is sent to the engine properly', (
    WidgetTester tester,
  ) async {
    const bool enableIMEPersonalizedLearning = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              enableIMEPersonalizedLearning: enableIMEPersonalizedLearning,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.idle();
    expect(
      tester.testTextInput.setClientArgs!['enableIMEPersonalizedLearning'],
      enableIMEPersonalizedLearning,
    );
  });

  group('smartDashesType and smartQuotesType', () {
    testWidgets('sent to the engine properly', (WidgetTester tester) async {
      const SmartDashesType smartDashesType = SmartDashesType.disabled;
      const SmartQuotesType smartQuotesType = SmartQuotesType.disabled;
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                smartDashesType: smartDashesType,
                smartQuotesType: smartQuotesType,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.showKeyboard(find.byType(EditableText));
      await tester.idle();
      expect(
        tester.testTextInput.setClientArgs!['smartDashesType'],
        smartDashesType.index.toString(),
      );
      expect(
        tester.testTextInput.setClientArgs!['smartQuotesType'],
        smartQuotesType.index.toString(),
      );
    });

    testWidgets('default to true when obscureText is false', (WidgetTester tester) async {
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.showKeyboard(find.byType(EditableText));
      await tester.idle();
      expect(tester.testTextInput.setClientArgs!['smartDashesType'], '1');
      expect(tester.testTextInput.setClientArgs!['smartQuotesType'], '1');
    });

    testWidgets('default to false when obscureText is true', (WidgetTester tester) async {
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                obscureText: true,
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.showKeyboard(find.byType(EditableText));
      await tester.idle();
      expect(tester.testTextInput.setClientArgs!['smartDashesType'], '0');
      expect(tester.testTextInput.setClientArgs!['smartQuotesType'], '0');
    });
  });

  testWidgets('selection overlay will update when text grow bigger', (WidgetTester tester) async {
    controller.value = const TextEditingValue(text: 'initial value');

    Future<void> pumpEditableTextWithTextStyle(TextStyle style) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: style,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            showSelectionHandles: true,
          ),
        ),
      );
    }

    await pumpEditableTextWithTextStyle(const TextStyle(fontSize: 18));
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.renderEditable.selectWordsInRange(
      from: Offset.zero,
      cause: SelectionChangedCause.longPress,
    );
    await tester.pumpAndSettle();
    await tester.idle();

    List<RenderBox> handles = List<RenderBox>.from(
      tester.renderObjectList<RenderBox>(
        find.descendant(
          of: find.byType(CompositedTransformFollower),
          matching: find.byType(Padding),
        ),
      ),
    );

    expect(handles[0].localToGlobal(Offset.zero), const Offset(-35.0, 5.0));
    expect(handles[1].localToGlobal(Offset.zero), const Offset(113.0, 5.0));

    await pumpEditableTextWithTextStyle(const TextStyle(fontSize: 30));
    await tester.pumpAndSettle();

    // Handles should be updated with bigger font size.
    handles = List<RenderBox>.from(
      tester.renderObjectList<RenderBox>(
        find.descendant(
          of: find.byType(CompositedTransformFollower),
          matching: find.byType(Padding),
        ),
      ),
    );
    // First handle should have the same dx but bigger dy.
    expect(handles[0].localToGlobal(Offset.zero), const Offset(-35.0, 17.0));
    expect(handles[1].localToGlobal(Offset.zero), const Offset(197.0, 17.0));
  });

  testWidgets('can update style of previous activated EditableText', (WidgetTester tester) async {
    final TextEditingController controller1 = TextEditingController();
    addTearDown(controller1.dispose);
    final TextEditingController controller2 = TextEditingController();
    addTearDown(controller2.dispose);
    final TextEditingController controller3 = TextEditingController();
    addTearDown(controller3.dispose);
    final TextEditingController controller4 = TextEditingController();
    addTearDown(controller4.dispose);
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: Column(
              children: <Widget>[
                EditableText(
                  key: key1,
                  controller: controller1,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(fontSize: 9),
                  cursorColor: cursorColor,
                ),
                EditableText(
                  key: key2,
                  controller: controller2,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(fontSize: 9),
                  cursorColor: cursorColor,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key1));
    await tester.showKeyboard(find.byKey(key1));
    controller.text = 'test';
    await tester.idle();
    RenderBox renderEditable = tester.renderObject(find.byKey(key1));
    expect(renderEditable.size.height, 9.0);
    // Taps the other EditableText to deactivate the first one.
    await tester.tap(find.byKey(key2));
    await tester.showKeyboard(find.byKey(key2));
    // Updates the style.
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: Column(
              children: <Widget>[
                EditableText(
                  key: key1,
                  controller: controller3,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(fontSize: 20),
                  cursorColor: cursorColor,
                ),
                EditableText(
                  key: key2,
                  controller: controller4,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(fontSize: 9),
                  cursorColor: cursorColor,
                ),
              ],
            ),
          ),
        ),
      ),
    );
    renderEditable = tester.renderObject(find.byKey(key1));
    expect(renderEditable.size.height, 20.0);
    expect(tester.takeException(), null);
  });

  testWidgets('Multiline keyboard with newline action is requested when maxLines = null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              maxLines: null,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.multiline'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.newline'));
  });

  testWidgets('Text keyboard is requested when explicitly set and maxLines = null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              maxLines: null,
              keyboardType: TextInputType.text,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.text'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.done'));
  });

  testWidgets('Correct keyboard is requested when set explicitly and maxLines > 1', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              keyboardType: TextInputType.phone,
              maxLines: 3,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.phone'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.done'));
  });

  testWidgets('multiline keyboard is requested when set implicitly', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              maxLines: 3, // Sets multiline keyboard implicitly.
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.multiline'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.newline'));
  });

  testWidgets('single line inputs have correct default keyboard', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.text'),
    );
    expect(tester.testTextInput.setClientArgs!['inputAction'], equals('TextInputAction.done'));
  });

  // Test case for
  // https://github.com/flutter/flutter/issues/123523
  // https://github.com/flutter/flutter/issues/134846 .
  testWidgets(
    'The focus and callback behavior are correct when TextInputClient.onConnectionClosed message received',
    (WidgetTester tester) async {
      bool onSubmittedInvoked = false;
      bool onEditingCompleteInvoked = false;
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: FocusScope(
              node: focusScopeNode,
              autofocus: true,
              child: EditableText(
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                style: textStyle,
                autofocus: true,
                cursorColor: cursorColor,
                onSubmitted: (String text) {
                  onSubmittedInvoked = true;
                },
                onEditingComplete: () {
                  onEditingCompleteInvoked = true;
                },
              ),
            ),
          ),
        ),
      );

      expect(focusNode.hasFocus, isTrue);
      final EditableTextState editableText = tester.state(find.byType(EditableText));
      editableText.connectionClosed();
      await tester.pump();

      expect(focusNode.hasFocus, isFalse);
      expect(onEditingCompleteInvoked, isFalse);
      expect(onSubmittedInvoked, isFalse);
    },
  );

  testWidgets('connection is closed when TextInputClient.onConnectionClosed message received', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(state.wantKeepAlive, true);

    tester.testTextInput.log.clear();
    tester.testTextInput.closeConnection();
    await tester.idle();

    // Widget does not have focus anymore.
    expect(state.wantKeepAlive, false);
    // No method calls are sent from the framework.
    // This makes sure hide/clearClient methods are not called after connection
    // closed.
    expect(tester.testTextInput.log, isEmpty);
  });

  testWidgets('closed connection reopened when user focused', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test3';
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(tester.testTextInput.editingState!['text'], equals('test3'));
    expect(state.wantKeepAlive, true);

    tester.testTextInput.log.clear();
    tester.testTextInput.closeConnection();
    await tester.pumpAndSettle();

    // Widget does not have focus anymore.
    expect(state.wantKeepAlive, false);
    // No method calls are sent from the framework.
    // This makes sure hide/clearClient methods are not called after connection
    // closed.
    expect(tester.testTextInput.log, isEmpty);

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pump();
    controller.text = 'test2';
    expect(tester.testTextInput.editingState!['text'], equals('test2'));
    // Widget regained the focus.
    expect(state.wantKeepAlive, true);
  });

  testWidgets('closed connection reopened when user focused on another field', (
    WidgetTester tester,
  ) async {
    final EditableText testNameField = EditableText(
      backgroundCursorColor: Colors.grey,
      controller: controller,
      focusNode: focusNode,
      maxLines: null,
      keyboardType: TextInputType.text,
      style: textStyle,
      cursorColor: cursorColor,
    );

    final EditableText testPhoneField = EditableText(
      backgroundCursorColor: Colors.grey,
      controller: controller,
      focusNode: focusNode,
      keyboardType: TextInputType.phone,
      maxLines: 3,
      style: textStyle,
      cursorColor: cursorColor,
    );

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: ListView(children: <Widget>[testNameField, testPhoneField]),
          ),
        ),
      ),
    );

    // Tap, enter text.
    await tester.tap(find.byWidget(testNameField));
    await tester.showKeyboard(find.byWidget(testNameField));
    controller.text = 'test';
    await tester.idle();

    expect(tester.testTextInput.editingState!['text'], equals('test'));
    final EditableTextState state = tester.state<EditableTextState>(find.byWidget(testNameField));
    expect(state.wantKeepAlive, true);

    tester.testTextInput.log.clear();
    tester.testTextInput.closeConnection();
    // A pump is needed to allow the focus change (unfocus) to be resolved.
    await tester.pump();

    // Widget does not have focus anymore.
    expect(state.wantKeepAlive, false);
    // No method calls are sent from the framework.
    // This makes sure hide/clearClient methods are not called after connection
    // closed.
    expect(tester.testTextInput.log, isEmpty);

    // For the next fields, tap, enter text.
    await tester.tap(find.byWidget(testPhoneField));
    await tester.showKeyboard(find.byWidget(testPhoneField));
    controller.text = '650123123';
    await tester.idle();
    expect(tester.testTextInput.editingState!['text'], equals('650123123'));
    // Widget regained the focus.
    expect(state.wantKeepAlive, true);
  });

  testWidgets('kept-alive EditableText does not crash when layout is skipped', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/84896.
    EditableText.debugDeterministicCursor = true;
    const Key key = ValueKey<String>('EditableText');
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: ListView(
            children: <Widget>[
              EditableText(
                key: key,
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                autofocus: true,
                maxLines: null,
                keyboardType: TextInputType.text,
                style: textStyle,
                textAlign: TextAlign.left,
                cursorColor: cursorColor,
                showCursor: false,
              ),
            ],
          ),
        ),
      ),
    );

    // Wait for autofocus.
    await tester.pump();
    expect(focusNode.hasFocus, isTrue);

    // Prepend an additional item to make EditableText invisible. It's still
    // kept in the tree via the keepalive mechanism. Change the text alignment
    // and showCursor. The RenderEditable now needs to relayout and repaint.
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: ListView(
            children: <Widget>[
              const SizedBox(height: 6000),
              EditableText(
                key: key,
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                autofocus: true,
                maxLines: null,
                keyboardType: TextInputType.text,
                style: textStyle,
                textAlign: TextAlign.right,
                cursorColor: cursorColor,
                showCursor: true,
              ),
            ],
          ),
        ),
      ),
    );

    EditableText.debugDeterministicCursor = false;
    expect(tester.takeException(), isNull);
  });

  // Toolbar is not used in Flutter Web unless the browser context menu is
  // explicitly disabled. Skip this check.
  //
  // Web is using native DOM elements (it is also used as platform input)
  // to enable clipboard functionality of the toolbar: copy, paste, select,
  // cut. It might also provide additional functionality depending on the
  // browser (such as translation). Due to this, in browsers, we should not
  // show a Flutter toolbar for the editable text elements.
  testWidgets('can show toolbar when there is text and a selection', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Can't show the toolbar when there's no focus.
    expect(state.showToolbar(), false);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), findsNothing);

    // Can show the toolbar when focused even though there's no text.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);

    // Hide the menu again.
    state.hideToolbar();
    await tester.pump();
    expect(find.text('Paste'), findsNothing);

    // Can show the menu with text and a selection.
    controller.text = 'blah';
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);
  });

  group('BrowserContextMenu', () {
    setUp(() async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        (MethodCall call) {
          // Just complete successfully, so that BrowserContextMenu thinks that
          // the engine successfully received its call.
          return Future<void>.value();
        },
      );
      await BrowserContextMenu.disableContextMenu();
    });

    tearDown(() async {
      await BrowserContextMenu.enableContextMenu();
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        null,
      );
    });

    testWidgets(
      'web can show flutter context menu when the browser context menu is disabled',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        // Can't show the toolbar when there's no focus.
        expect(state.showToolbar(), false);
        await tester.pumpAndSettle();
        expect(find.text('Paste'), findsNothing);

        // Can show the toolbar when focused even though there's no text.
        state.renderEditable.selectWordsInRange(
          from: Offset.zero,
          cause: SelectionChangedCause.tap,
        );
        await tester.pump();
        expect(state.showToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('Paste'), findsOneWidget);

        // Hide the menu again.
        state.hideToolbar();
        await tester.pump();
        expect(find.text('Paste'), findsNothing);

        // Can show the menu with text and a selection.
        controller.text = 'blah';
        await tester.pump();
        expect(state.showToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('Paste'), findsOneWidget);
      },
      skip: !kIsWeb, // [intended]
    );
  });

  testWidgets('can hide toolbar with DismissIntent', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Show the toolbar
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();

    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);

    // Hide the menu using the DismissIntent.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.text('Paste'), findsNothing);
  });

  testWidgets(
    'toolbar hidden on mobile when orientation changes',
    (WidgetTester tester) async {
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Show the toolbar
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
      await tester.pump();

      expect(state.showToolbar(), true);
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsOneWidget);

      // Hide the menu by changing orientation.
      tester.view.physicalSize = const Size(1800.0, 2400.0);
      await tester.pumpAndSettle();
      expect(find.text('Paste'), findsNothing);

      // Handles should be hidden as well on Android
      expect(
        find.descendant(
          of: find.byType(CompositedTransformFollower),
          matching: find.byType(Padding),
        ),
        defaultTargetPlatform == TargetPlatform.android ? findsNothing : findsOneWidget,
      );

      // On web, we don't show the Flutter toolbar and instead rely on the browser
      // toolbar. Until we change that, this test should remain skipped.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
  );

  testWidgets('Paste is shown only when there is something to paste', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Make sure the clipboard has a valid string on it.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));

    // Show the toolbar.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();

    // The Paste button is shown (except on web, which doesn't show the Flutter
    // toolbar).
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);

    // Hide the menu again.
    state.hideToolbar();
    await tester.pump();
    expect(find.text('Paste'), findsNothing);

    // Clear the clipboard
    await Clipboard.setData(const ClipboardData(text: ''));

    // Show the toolbar again.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();

    // Paste is not shown.
    await tester.pumpAndSettle();
    expect(find.text('Paste'), findsNothing);
  });

  testWidgets(
    'Copy selection does not collapse selection on desktop and iOS',
    (WidgetTester tester) async {
      final TextEditingController localController = TextEditingController(text: 'Hello world');
      addTearDown(localController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: localController,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Show the toolbar.
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
      await tester.pump();

      final TextSelection copySelectionRange = localController.selection;

      state.showToolbar();
      await tester.pumpAndSettle();

      expect(find.text('Copy'), findsOneWidget);

      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(copySelectionRange, localController.selection);
      expect(find.text('Copy'), findsNothing);
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'Copy selection collapses selection and hides the toolbar on Android and Fuchsia',
    (WidgetTester tester) async {
      final TextEditingController localController = TextEditingController(text: 'Hello world');
      addTearDown(localController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: localController,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Show the toolbar.
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
      await tester.pump();

      final TextSelection copySelectionRange = localController.selection;

      expect(find.byType(TextSelectionToolbar), findsNothing);
      state.showToolbar();
      await tester.pumpAndSettle();

      expect(find.byType(TextSelectionToolbar), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);

      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(
        localController.selection,
        TextSelection.collapsed(offset: copySelectionRange.extentOffset),
      );
      expect(find.text('Copy'), findsNothing);
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets('can show the toolbar after clearing all text', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/35998.
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Add text and an empty selection.
    controller.text = 'blah';
    await tester.pump();
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();

    // Clear the text and selection.
    expect(find.text('Paste'), findsNothing);
    state.updateEditingValue(TextEditingValue.empty);
    await tester.pump();

    // Should be able to show the toolbar.
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);
  });

  testWidgets('can dynamically disable options in toolbar', (WidgetTester tester) async {
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          toolbarOptions: const ToolbarOptions(copy: true, selectAll: true),
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Select something. Doesn't really matter what.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pump();
    expect(find.text('Select all'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Copy'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('can dynamically disable select all option in toolbar - cupertino', (
    WidgetTester tester,
  ) async {
    // Regression test: https://github.com/flutter/flutter/issues/40711
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          toolbarOptions: ToolbarOptions.empty,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: cupertinoTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    await tester.tap(find.byType(EditableText));
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pump();
    expect(find.text('Select All'), findsNothing);
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('can dynamically disable select all option in toolbar - material', (
    WidgetTester tester,
  ) async {
    // Regression test: https://github.com/flutter/flutter/issues/40711
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          toolbarOptions: const ToolbarOptions(copy: true),
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Select something. Doesn't really matter what.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pump();
    expect(find.text('Select all'), findsNothing);
    expect(find.text('Copy'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('cut and paste are disabled in read only mode even if explicitly set', (
    WidgetTester tester,
  ) async {
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          readOnly: true,
          toolbarOptions: const ToolbarOptions(copy: true, cut: true, paste: true, selectAll: true),
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Select something. Doesn't really matter what.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();
    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pump();
    expect(find.text('Select all'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Copy'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('cut and copy are disabled in obscured mode even if explicitly set', (
    WidgetTester tester,
  ) async {
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          obscureText: true,
          toolbarOptions: const ToolbarOptions(copy: true, cut: true, paste: true, selectAll: true),
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    await tester.tap(find.byType(EditableText));
    await tester.pump();
    // Select something, but not the whole thing.
    state.renderEditable.selectWord(cause: SelectionChangedCause.tap);
    await tester.pump();
    expect(state.selectAllEnabled, isTrue);
    expect(state.pasteEnabled, isTrue);
    expect(state.cutEnabled, isFalse);
    expect(state.copyEnabled, isFalse);

    // On web, we don't let Flutter show the toolbar.
    expect(state.showToolbar(), kIsWeb ? isFalse : isTrue);
    await tester.pump();
    expect(find.text('Select all'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Paste'), kIsWeb ? findsNothing : findsOneWidget);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('cut and copy do nothing in obscured mode even if explicitly called', (
    WidgetTester tester,
  ) async {
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          obscureText: true,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.selectAllEnabled, isTrue);
    expect(state.pasteEnabled, isTrue);
    expect(state.cutEnabled, isFalse);
    expect(state.copyEnabled, isFalse);

    // Select all.
    state.selectAll(SelectionChangedCause.toolbar);
    await tester.pump();
    await Clipboard.setData(const ClipboardData(text: ''));
    state.cutSelection(SelectionChangedCause.toolbar);
    ClipboardData? data = await Clipboard.getData('text/plain');
    expect(data, isNotNull);
    expect(data!.text, isEmpty);

    state.selectAll(SelectionChangedCause.toolbar);
    await tester.pump();
    await Clipboard.setData(const ClipboardData(text: ''));
    state.copySelection(SelectionChangedCause.toolbar);
    data = await Clipboard.getData('text/plain');
    expect(data, isNotNull);
    expect(data!.text, isEmpty);
  });

  testWidgets('select all does nothing if obscured and read-only, even if explicitly called', (
    WidgetTester tester,
  ) async {
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          obscureText: true,
          readOnly: true,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Select all.
    state.selectAll(SelectionChangedCause.toolbar);
    expect(state.selectAllEnabled, isFalse);
    expect(state.textEditingValue.selection.isCollapsed, isTrue);
  });

  group('buttonItemsForToolbarOptions', () {
    testWidgets('returns null when toolbarOptions are empty', (WidgetTester tester) async {
      controller.text = 'TEXT';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            toolbarOptions: ToolbarOptions.empty,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      expect(state.buttonItemsForToolbarOptions(), isNull);
    });

    testWidgets(
      'returns empty array when only cut is selected in toolbarOptions but cut is not enabled',
      (WidgetTester tester) async {
        controller.text = 'TEXT';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(cut: true),
              readOnly: true,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        expect(state.cutEnabled, isFalse);
        expect(state.buttonItemsForToolbarOptions(), isEmpty);
      },
    );

    testWidgets(
      'returns only cut button when only cut is selected in toolbarOptions and cut is enabled',
      (WidgetTester tester) async {
        const String text = 'TEXT';
        controller.text = text;

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(cut: true),
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        // Selecting all.
        controller.selection = TextSelection(baseOffset: 0, extentOffset: controller.text.length);
        expect(state.cutEnabled, isTrue);

        final List<ContextMenuButtonItem>? items = state.buttonItemsForToolbarOptions();

        expect(items, isNotNull);
        expect(items, hasLength(1));

        final ContextMenuButtonItem cutButton = items!.first;
        expect(cutButton.type, ContextMenuButtonType.cut);

        cutButton.onPressed?.call();
        await tester.pump();

        expect(controller.text, isEmpty);
        final ClipboardData? data = await Clipboard.getData('text/plain');
        expect(data, isNotNull);
        expect(data!.text, equals(text));
      },
    );

    testWidgets(
      'returns empty array when only copy is selected in toolbarOptions but copy is not enabled',
      (WidgetTester tester) async {
        controller.text = 'TEXT';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(copy: true),
              obscureText: true,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        expect(state.copyEnabled, isFalse);
        expect(state.buttonItemsForToolbarOptions(), isEmpty);
      },
    );

    testWidgets(
      'returns only copy button when only copy is selected in toolbarOptions and copy is enabled',
      (WidgetTester tester) async {
        const String text = 'TEXT';
        controller.text = text;

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(copy: true),
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        // Selecting all.
        controller.selection = TextSelection(baseOffset: 0, extentOffset: controller.text.length);
        expect(state.copyEnabled, isTrue);

        final List<ContextMenuButtonItem>? items = state.buttonItemsForToolbarOptions();

        expect(items, isNotNull);
        expect(items, hasLength(1));

        final ContextMenuButtonItem copyButton = items!.first;
        expect(copyButton.type, ContextMenuButtonType.copy);

        copyButton.onPressed?.call();
        await tester.pump();

        expect(controller.text, equals(text));
        final ClipboardData? data = await Clipboard.getData('text/plain');
        expect(data, isNotNull);
        expect(data!.text, equals(text));
      },
    );

    testWidgets(
      'returns empty array when only paste is selected in toolbarOptions but paste is not enabled',
      (WidgetTester tester) async {
        controller.text = 'TEXT';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(paste: true),
              readOnly: true,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        expect(state.pasteEnabled, isFalse);
        expect(state.buttonItemsForToolbarOptions(), isEmpty);
      },
    );

    testWidgets(
      'returns only paste button when only paste is selected in toolbarOptions and paste is enabled',
      (WidgetTester tester) async {
        const String text = 'TEXT';
        controller.text = text;

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(paste: true),
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        // Moving caret to the end.
        controller.selection = TextSelection.collapsed(offset: controller.text.length);
        expect(state.pasteEnabled, isTrue);

        final List<ContextMenuButtonItem>? items = state.buttonItemsForToolbarOptions();

        expect(items, isNotNull);
        expect(items, hasLength(1));

        final ContextMenuButtonItem pasteButton = items!.first;
        expect(pasteButton.type, ContextMenuButtonType.paste);

        // Setting data which will be pasted into the clipboard.
        await Clipboard.setData(const ClipboardData(text: text));

        pasteButton.onPressed?.call();
        await tester.pump();

        expect(controller.text, equals(text + text));
      },
    );

    testWidgets(
      'returns empty array when only selectAll is selected in toolbarOptions but selectAll is not enabled',
      (WidgetTester tester) async {
        controller.text = 'TEXT';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(selectAll: true),
              readOnly: true,
              obscureText: true,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        expect(state.selectAllEnabled, isFalse);
        expect(state.buttonItemsForToolbarOptions(), isEmpty);
      },
    );

    testWidgets(
      'returns only selectAll button when only selectAll is selected in toolbarOptions and selectAll is enabled',
      (WidgetTester tester) async {
        const String text = 'TEXT';
        controller.text = text;

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              toolbarOptions: const ToolbarOptions(selectAll: true),
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        final List<ContextMenuButtonItem>? items = state.buttonItemsForToolbarOptions();

        expect(items, isNotNull);
        expect(items, hasLength(1));

        final ContextMenuButtonItem selectAllButton = items!.first;
        expect(selectAllButton.type, ContextMenuButtonType.selectAll);

        selectAllButton.onPressed?.call();
        await tester.pump();

        expect(controller.text, equals(text));
        expect(state.textEditingValue.selection.textInside(text), equals(text));
      },
    );
  });

  testWidgets('Handles the read-only flag correctly', (WidgetTester tester) async {
    controller.text = 'Lorem ipsum dolor sit amet';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          readOnly: true,
          controller: controller,
          backgroundCursorColor: Colors.grey,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    // Interact with the field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    await tester.tapAt(topLeft + const Offset(0.0, 5.0));
    await tester.pump();

    controller.selection = const TextSelection(baseOffset: 0, extentOffset: 5);
    await tester.pump();

    if (kIsWeb) {
      // On the web, a regular connection to the platform should've been made
      // with the `readOnly` flag set to true.
      expect(tester.testTextInput.hasAnyClients, isTrue);
      expect(tester.testTextInput.setClientArgs!['readOnly'], isTrue);
      expect(tester.testTextInput.editingState!['text'], 'Lorem ipsum dolor sit amet');
      expect(tester.testTextInput.editingState!['selectionBase'], 0);
      expect(tester.testTextInput.editingState!['selectionExtent'], 5);
    } else {
      // On non-web platforms, a read-only field doesn't need a connection with
      // the platform.
      expect(tester.testTextInput.hasAnyClients, isFalse);
    }
  });

  testWidgets('Does not accept updates when read-only', (WidgetTester tester) async {
    controller.text = 'Lorem ipsum dolor sit amet';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          readOnly: true,
          controller: controller,
          backgroundCursorColor: Colors.grey,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    // Interact with the field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    await tester.tapAt(topLeft + const Offset(0.0, 5.0));
    await tester.pump();

    expect(tester.testTextInput.hasAnyClients, kIsWeb ? isTrue : isFalse);
    if (kIsWeb) {
      // On the web, the input connection exists, but text updates should be
      // ignored.
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: 'Foo bar',
          selection: TextSelection(baseOffset: 0, extentOffset: 3),
          composing: TextRange(start: 3, end: 4),
        ),
      );
      // Only selection should change.
      expect(
        controller.value,
        const TextEditingValue(
          text: 'Lorem ipsum dolor sit amet',
          selection: TextSelection(baseOffset: 0, extentOffset: 3),
        ),
      );
    }
  });

  testWidgets('Read-only fields do not format text', (WidgetTester tester) async {
    controller.text = 'Lorem ipsum dolor sit amet';
    late SelectionChangedCause selectionCause;

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          readOnly: true,
          controller: controller,
          backgroundCursorColor: Colors.grey,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
          onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
            selectionCause = cause!;
          },
        ),
      ),
    );

    // Interact with the field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    await tester.tapAt(topLeft + const Offset(0.0, 5.0));
    await tester.pump();

    expect(tester.testTextInput.hasAnyClients, kIsWeb ? isTrue : isFalse);
    if (kIsWeb) {
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(
          text: 'Foo bar',
          selection: TextSelection(baseOffset: 0, extentOffset: 3),
        ),
      );
      // On web, the only way a text field can be updated from the engine is if
      // a keyboard is used.
      expect(selectionCause, SelectionChangedCause.keyboard);
    }
  });

  testWidgets('Sends "updateConfig" when read-only flag is flipped', (WidgetTester tester) async {
    bool readOnly = true;
    late StateSetter setState;
    controller.text = 'Lorem ipsum dolor sit amet';

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            return EditableText(
              readOnly: readOnly,
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            );
          },
        ),
      ),
    );

    // Interact with the field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    await tester.tapAt(topLeft + const Offset(0.0, 5.0));
    await tester.pump();

    expect(tester.testTextInput.hasAnyClients, kIsWeb ? isTrue : isFalse);
    if (kIsWeb) {
      expect(tester.testTextInput.setClientArgs!['readOnly'], isTrue);
    }

    setState(() {
      readOnly = false;
    });
    await tester.pump();

    expect(tester.testTextInput.hasAnyClients, isTrue);
    expect(tester.testTextInput.setClientArgs!['readOnly'], isFalse);
  });

  testWidgets('Sends "updateConfig" when obscureText is flipped', (WidgetTester tester) async {
    bool obscureText = true;
    late StateSetter setState;
    controller.text = 'Lorem';

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            return EditableText(
              obscureText: obscureText,
              controller: controller,
              backgroundCursorColor: Colors.grey,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            );
          },
        ),
      ),
    );

    // Interact with the field to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    await tester.tapAt(topLeft + const Offset(0.0, 5.0));
    await tester.pump();

    expect(tester.testTextInput.setClientArgs!['obscureText'], isTrue);

    setState(() {
      obscureText = false;
    });
    await tester.pump();

    expect(tester.testTextInput.setClientArgs!['obscureText'], isFalse);
  });

  testWidgets('Sends viewId and updates config when it changes', (WidgetTester tester) async {
    int viewId = 14;
    late StateSetter setState;
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(
      wrapWithView: false,
      StatefulBuilder(
        builder: (BuildContext context, StateSetter stateSetter) {
          setState = stateSetter;
          return View(
            view: FakeFlutterView(tester.view, viewId: viewId),
            child: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: EditableText(
                  key: key,
                  controller: controller,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
              ),
            ),
          );
        },
      ),
    );

    // Focus the field to establish the input connection.
    focusNode.requestFocus();
    await tester.pump();

    expect(tester.testTextInput.setClientArgs!['viewId'], 14);
    expect(tester.testTextInput.log, contains(matchesMethodCall('TextInput.setClient')));
    tester.testTextInput.log.clear();

    setState(() {
      viewId = 15;
    });
    await tester.pump();

    expect(tester.testTextInput.setClientArgs!['viewId'], 15);
    expect(tester.testTextInput.log, contains(matchesMethodCall('TextInput.updateConfig')));
    tester.testTextInput.log.clear();
  });

  testWidgets('Fires onChanged when text changes via TextSelectionOverlay', (
    WidgetTester tester,
  ) async {
    late String changedValue;
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        selectionControls: materialTextSelectionControls,
        keyboardType: TextInputType.text,
        onChanged: (String value) {
          changedValue = value;
        },
      ),
    );
    await tester.pumpWidget(widget);

    // Populate a fake clipboard.
    const String clipboardContent = 'Dobunezumi mitai ni utsukushiku naritai';
    Clipboard.setData(const ClipboardData(text: clipboardContent));

    // Long-press to bring up the text editing controls.
    final Finder textFinder = find.byType(EditableText);
    await tester.longPress(textFinder);
    tester.state<EditableTextState>(textFinder).showToolbar();
    await tester.pumpAndSettle();

    await tester.tap(find.text('Paste'));
    await tester.pump();

    expect(changedValue, clipboardContent);

    // On web, we don't show the Flutter toolbar and instead rely on the browser
    // toolbar. Until we change that, this test should remain skipped.
  }, skip: kIsWeb); // [intended]

  // The variants to test in the focus handling test.
  final ValueVariant<TextInputAction> focusVariants = ValueVariant<TextInputAction>(
    TextInputAction.values.toSet(),
  );

  testWidgets('Handles focus correctly when action is invoked', (WidgetTester tester) async {
    // The expectations for each of the types of TextInputAction.
    const Map<TextInputAction, bool> actionShouldLoseFocus = <TextInputAction, bool>{
      TextInputAction.none: false,
      TextInputAction.unspecified: false,
      TextInputAction.done: true,
      TextInputAction.go: true,
      TextInputAction.search: true,
      TextInputAction.send: true,
      TextInputAction.continueAction: false,
      TextInputAction.join: false,
      TextInputAction.route: false,
      TextInputAction.emergencyCall: false,
      TextInputAction.newline: true,
      TextInputAction.next: true,
      TextInputAction.previous: true,
    };

    final TextInputAction action = focusVariants.currentValue!;
    expect(actionShouldLoseFocus.containsKey(action), isTrue);

    Future<void> ensureCorrectFocusHandlingForAction(
      TextInputAction action, {
      required bool shouldLoseFocus,
      bool shouldFocusNext = false,
      bool shouldFocusPrevious = false,
    }) async {
      final GlobalKey previousKey = GlobalKey();
      final GlobalKey nextKey = GlobalKey();

      final Widget widget = MaterialApp(
        home: Column(
          children: <Widget>[
            TextButton(child: Text('Previous Widget', key: previousKey), onPressed: () {}),
            EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
              autofocus: true,
            ),
            TextButton(child: Text('Next Widget', key: nextKey), onPressed: () {}),
          ],
        ),
      );
      await tester.pumpWidget(widget);

      assert(focusNode.hasFocus);

      await tester.testTextInput.receiveAction(action);
      await tester.pump();

      expect(Focus.of(nextKey.currentContext!).hasFocus, equals(shouldFocusNext));
      expect(Focus.of(previousKey.currentContext!).hasFocus, equals(shouldFocusPrevious));
      expect(focusNode.hasFocus, equals(!shouldLoseFocus));
    }

    try {
      await ensureCorrectFocusHandlingForAction(
        action,
        shouldLoseFocus: actionShouldLoseFocus[action]!,
        shouldFocusNext: action == TextInputAction.next,
        shouldFocusPrevious: action == TextInputAction.previous,
      );
    } on PlatformException {
      // on Android, continueAction isn't supported.
      expect(action, equals(TextInputAction.continueAction));
    }
  }, variant: focusVariants);

  testWidgets(
    'Does not lose focus by default when "done" action is pressed and onEditingComplete is provided',
    (WidgetTester tester) async {
      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          selectionControls: materialTextSelectionControls,
          keyboardType: TextInputType.text,
          onEditingComplete: () {
            // This prevents the default focus change behavior on submission.
          },
        ),
      );
      await tester.pumpWidget(widget);

      // Select EditableText to give it focus.
      final Finder textFinder = find.byType(EditableText);
      await tester.tap(textFinder);
      await tester.pump();

      assert(focusNode.hasFocus);

      await tester.testTextInput.receiveAction(TextInputAction.done);
      await tester.pump();

      // Still has focus even though "done" was pressed because onEditingComplete
      // was provided and it overrides the default behavior.
      expect(focusNode.hasFocus, true);
    },
  );

  testWidgets('When "done" is pressed callbacks are invoked: onEditingComplete > onSubmitted', (
    WidgetTester tester,
  ) async {
    bool onEditingCompleteCalled = false;
    bool onSubmittedCalled = false;

    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        onEditingComplete: () {
          onEditingCompleteCalled = true;
          expect(onSubmittedCalled, false);
        },
        onSubmitted: (String value) {
          onSubmittedCalled = true;
          expect(onEditingCompleteCalled, true);
        },
      ),
    );
    await tester.pumpWidget(widget);

    // Select EditableText to give it focus.
    final Finder textFinder = find.byType(EditableText);
    await tester.tap(textFinder);
    await tester.pump();

    assert(focusNode.hasFocus);

    // The execution path starting with receiveAction() will trigger the
    // onEditingComplete and onSubmission callbacks.
    await tester.testTextInput.receiveAction(TextInputAction.done);

    // The expectations we care about are up above in the onEditingComplete
    // and onSubmission callbacks.
  });

  testWidgets('When "next" is pressed callbacks are invoked: onEditingComplete > onSubmitted', (
    WidgetTester tester,
  ) async {
    bool onEditingCompleteCalled = false;
    bool onSubmittedCalled = false;

    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        onEditingComplete: () {
          onEditingCompleteCalled = true;
          assert(!onSubmittedCalled);
        },
        onSubmitted: (String value) {
          onSubmittedCalled = true;
          assert(onEditingCompleteCalled);
        },
      ),
    );
    await tester.pumpWidget(widget);

    // Select EditableText to give it focus.
    final Finder textFinder = find.byType(EditableText);
    await tester.tap(textFinder);
    await tester.pump();

    assert(focusNode.hasFocus);

    // The execution path starting with receiveAction() will trigger the
    // onEditingComplete and onSubmission callbacks.
    await tester.testTextInput.receiveAction(TextInputAction.done);

    // The expectations we care about are up above in the onEditingComplete
    // and onSubmission callbacks.
  });

  testWidgets(
    'When "newline" action is called on a Editable text with maxLines == 1 callbacks are invoked: onEditingComplete > onSubmitted',
    (WidgetTester tester) async {
      bool onEditingCompleteCalled = false;
      bool onSubmittedCalled = false;

      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          onEditingComplete: () {
            onEditingCompleteCalled = true;
            assert(!onSubmittedCalled);
          },
          onSubmitted: (String value) {
            onSubmittedCalled = true;
            assert(onEditingCompleteCalled);
          },
        ),
      );
      await tester.pumpWidget(widget);

      // Select EditableText to give it focus.
      final Finder textFinder = find.byType(EditableText);
      await tester.tap(textFinder);
      await tester.pump();

      assert(focusNode.hasFocus);

      // The execution path starting with receiveAction() will trigger the
      // onEditingComplete and onSubmission callbacks.
      await tester.testTextInput.receiveAction(TextInputAction.newline);
      // The expectations we care about are up above in the onEditingComplete
      // and onSubmission callbacks.
    },
  );

  testWidgets(
    'When "newline" action is called on a Editable text with maxLines != 1, onEditingComplete and onSubmitted callbacks are not invoked.',
    (WidgetTester tester) async {
      bool onEditingCompleteCalled = false;
      bool onSubmittedCalled = false;

      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          maxLines: 3,
          onEditingComplete: () {
            onEditingCompleteCalled = true;
          },
          onSubmitted: (String value) {
            onSubmittedCalled = true;
          },
        ),
      );
      await tester.pumpWidget(widget);

      // Select EditableText to give it focus.
      final Finder textFinder = find.byType(EditableText);
      await tester.tap(textFinder);
      await tester.pump();

      assert(focusNode.hasFocus);

      // The execution path starting with receiveAction() will trigger the
      // onEditingComplete and onSubmission callbacks.
      await tester.testTextInput.receiveAction(TextInputAction.newline);

      // These callbacks shouldn't have been triggered.
      assert(!onSubmittedCalled);
      assert(!onEditingCompleteCalled);
    },
  );

  testWidgets(
    'finalizeEditing should reset the input connection when shouldUnfocus is true but the unfocus is cancelled',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/84240 .
      Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          focusNode: focusNode,
          controller: controller,
          onSubmitted: (String value) {},
        ),
      );
      await tester.pumpWidget(widget);
      focusNode.requestFocus();
      await tester.pump();

      assert(focusNode.hasFocus);
      tester.testTextInput.log.clear();

      // This should unfocus the field. Don't restart the input.
      await tester.testTextInput.receiveAction(TextInputAction.done);
      expect(
        tester.testTextInput.log,
        isNot(
          containsAllInOrder(<Matcher>[
            matchesMethodCall('TextInput.clearClient'),
            matchesMethodCall('TextInput.setClient'),
          ]),
        ),
      );

      widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          focusNode: focusNode,
          controller: controller,
          onSubmitted: (String value) {
            focusNode.requestFocus();
          },
        ),
      );
      await tester.pumpWidget(widget);

      focusNode.requestFocus();
      await tester.pump();

      assert(focusNode.hasFocus);
      tester.testTextInput.log.clear();

      // This will attempt to unfocus the field but the onSubmitted callback
      // will cancel that. Restart the input connection in this case.
      await tester.testTextInput.receiveAction(TextInputAction.done);
      expect(
        tester.testTextInput.log,
        containsAllInOrder(<Matcher>[
          matchesMethodCall('TextInput.clearClient'),
          matchesMethodCall('TextInput.setClient'),
        ]),
      );

      tester.testTextInput.log.clear();
      // TextInputAction.unspecified does not unfocus the input field by default.
      await tester.testTextInput.receiveAction(TextInputAction.unspecified);
      expect(
        tester.testTextInput.log,
        isNot(
          containsAllInOrder(<Matcher>[
            matchesMethodCall('TextInput.clearClient'),
            matchesMethodCall('TextInput.setClient'),
          ]),
        ),
      );
    },
  );

  testWidgets(
    'requesting focus in the onSubmitted callback should keep the onscreen keyboard visible',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/95154 .
      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          focusNode: focusNode,
          controller: controller,
          onSubmitted: (String value) {
            focusNode.requestFocus();
          },
        ),
      );
      await tester.pumpWidget(widget);

      focusNode.requestFocus();
      await tester.pump();

      assert(focusNode.hasFocus);
      tester.testTextInput.log.clear();

      // This will attempt to unfocus the field but the onSubmitted callback
      // will cancel that. Restart the input connection in this case.
      await tester.testTextInput.receiveAction(TextInputAction.done);
      expect(
        tester.testTextInput.log,
        containsAllInOrder(<Matcher>[
          matchesMethodCall('TextInput.clearClient'),
          matchesMethodCall('TextInput.setClient'),
          matchesMethodCall('TextInput.show'),
        ]),
      );

      tester.testTextInput.log.clear();
      // TextInputAction.unspecified does not unfocus the input field by default.
      await tester.testTextInput.receiveAction(TextInputAction.unspecified);
      expect(
        tester.testTextInput.log,
        isNot(
          containsAllInOrder(<Matcher>[
            matchesMethodCall('TextInput.clearClient'),
            matchesMethodCall('TextInput.setClient'),
            matchesMethodCall('TextInput.show'),
          ]),
        ),
      );
    },
  );

  testWidgets('does not request keyboard after the keyboard changes the selection', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/154156.
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        focusNode: focusNode,
        controller: controller,
      ),
    );
    controller.value = const TextEditingValue(
      text: '123',
      selection: TextSelection.collapsed(offset: 0),
    );
    await tester.pumpWidget(widget);

    focusNode.requestFocus();
    await tester.pump();

    assert(focusNode.hasFocus);
    tester.testTextInput.log.clear();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.userUpdateTextEditingValue(
      const TextEditingValue(text: '123', selection: TextSelection.collapsed(offset: 1)),
      SelectionChangedCause.keyboard,
    );

    expect(
      tester.testTextInput.log.map((MethodCall m) => m.method),
      isNot(contains('TextInput.show')),
    );
  });

  testWidgets(
    'iOS autocorrection rectangle should appear on demand and dismiss when the text changes or when focus is lost',
    (WidgetTester tester) async {
      const Color rectColor = Color(0xFFFF0000);
      controller.text = 'ABCDEFG';

      void verifyAutocorrectionRectVisibility({required bool expectVisible}) {
        PaintPattern evaluate() {
          if (expectVisible) {
            return paints..something((Symbol method, List<dynamic> arguments) {
              if (method != #drawRect) {
                return false;
              }
              final Paint paint = arguments[1] as Paint;
              return paint.color == rectColor;
            });
          } else {
            return paints..everything((Symbol method, List<dynamic> arguments) {
              if (method != #drawRect) {
                return true;
              }
              final Paint paint = arguments[1] as Paint;
              if (paint.color != rectColor) {
                return true;
              }
              throw 'Expected: autocorrection rect not visible, found: ${arguments[0]}';
            });
          }
        }

        expect(findRenderEditable(tester), evaluate());
      }

      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          autocorrectionTextRectColor: rectColor,
          showCursor: false,
          onEditingComplete: () {},
        ),
      );

      await tester.pumpWidget(widget);

      await tester.tap(find.byType(EditableText));
      await tester.pump();
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      assert(focusNode.hasFocus);

      // The prompt rect should be invisible initially.
      verifyAutocorrectionRectVisibility(expectVisible: false);

      state.showAutocorrectionPromptRect(0, 1);
      await tester.pump();

      // Show prompt rect when told to.
      verifyAutocorrectionRectVisibility(expectVisible: true);

      await tester.enterText(find.byType(EditableText), '12345');
      await tester.pump();
      verifyAutocorrectionRectVisibility(expectVisible: false);

      state.showAutocorrectionPromptRect(0, 1);
      await tester.pump();

      verifyAutocorrectionRectVisibility(expectVisible: true);

      // Unfocus, prompt rect should go away.
      focusNode.unfocus();
      await tester.pumpAndSettle();

      verifyAutocorrectionRectVisibility(expectVisible: false);
    },
  );

  testWidgets('Changing controller updates EditableText', (WidgetTester tester) async {
    final TextEditingController controller1 = TextEditingController(text: 'Wibble');
    addTearDown(controller1.dispose);
    final TextEditingController controller2 = TextEditingController(text: 'Wobble');
    addTearDown(controller2.dispose);
    TextEditingController currentController = controller1;
    late StateSetter setState;

    Widget builder() {
      return StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          setState = setter;
          return MaterialApp(
            home: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Center(
                  child: Material(
                    child: EditableText(
                      backgroundCursorColor: Colors.grey,
                      controller: currentController,
                      focusNode: focusNode,
                      style: Typography.material2018().black.titleMedium!,
                      cursorColor: Colors.blue,
                      selectionControls: materialTextSelectionControls,
                      keyboardType: TextInputType.text,
                      onChanged: (String value) {},
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    await tester.pumpWidget(builder());
    await tester.pump(); // An extra pump to allow focus request to go through.

    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.showKeyboard(find.byType(EditableText));

    // Verify TextInput.setEditingState and TextInput.setEditableSizeAndTransform are
    // both fired with updated text when controller is replaced.
    setState(() {
      currentController = controller2;
    });
    await tester.pump();

    expect(
      log.lastWhere((MethodCall m) => m.method == 'TextInput.setEditingState'),
      isMethodCall(
        'TextInput.setEditingState',
        arguments: const <String, dynamic>{
          'text': 'Wobble',
          'selectionBase': -1,
          'selectionExtent': -1,
          'selectionAffinity': 'TextAffinity.downstream',
          'selectionIsDirectional': false,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ),
    );
    expect(
      log.lastWhere((MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform'),
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 14,
          'transform': Matrix4.translationValues(0.0, 293.0, 0.0).storage.toList(),
        },
      ),
    );
  });

  testWidgets('EditableText identifies as text field (w/ focus) in semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    expect(semantics, includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.isTextField]));

    await tester.tap(find.byType(EditableText));
    await tester.idle();
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.isTextField, SemanticsFlag.isFocused]),
    );

    semantics.dispose();
  });

  testWidgets('EditableText sets multi-line flag in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    expect(semantics, includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.isTextField]));

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              maxLines: 3,
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[SemanticsFlag.isTextField, SemanticsFlag.isMultiline],
      ),
    );

    semantics.dispose();
  });

  testWidgets('EditableText includes text as value in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    const String value1 = 'EditableText content';
    controller.text = value1;

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.isTextField], value: value1),
    );

    const String value2 = 'Changed the EditableText content';
    controller.text = value2;
    await tester.idle();
    await tester.pump();

    expect(
      semantics,
      includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.isTextField], value: value2),
    );

    semantics.dispose();
  });

  testWidgets('exposes correct cursor movement semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    controller.text = 'test';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    focusNode.requestFocus();
    await tester.pump();

    expect(semantics, includesNodeWith(value: 'test'));

    controller.selection = TextSelection.collapsed(offset: controller.text.length);
    await tester.pumpAndSettle();

    // At end, can only go backwards.
    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.setSelection,
          SemanticsAction.setText,
        ],
      ),
    );

    controller.selection = TextSelection.collapsed(offset: controller.text.length - 2);
    await tester.pumpAndSettle();

    // Somewhere in the middle, can go in both directions.
    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.moveCursorForwardByWord,
          SemanticsAction.setSelection,
          SemanticsAction.setText,
        ],
      ),
    );

    controller.selection = const TextSelection.collapsed(offset: 0);
    await tester.pumpAndSettle();

    // At beginning, can only go forward.
    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorForwardByWord,
          SemanticsAction.setSelection,
          SemanticsAction.setText,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('can move cursor with a11y means - character', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const bool doNotExtendSelection = false;

    controller.text = 'test';
    controller.selection = TextSelection.collapsed(offset: controller.text.length);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
        ],
      ),
    );

    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    final int semanticsId = render.debugSemantics!.id;

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 4);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 3);
    expect(controller.selection.extentOffset, 3);

    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 0);

    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 1);
    expect(controller.selection.extentOffset, 1);

    semantics.dispose();
  });

  testWidgets('can move cursor with a11y means - word', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const bool doNotExtendSelection = false;

    controller.text = 'test for words';
    controller.selection = TextSelection.collapsed(offset: controller.text.length);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
        ],
      ),
    );

    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    final int semanticsId = render.debugSemantics!.id;

    expect(controller.selection.baseOffset, 14);
    expect(controller.selection.extentOffset, 14);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 9);
    expect(controller.selection.extentOffset, 9);

    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 5);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 0);

    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 5);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 9);
    expect(controller.selection.extentOffset, 9);

    semantics.dispose();
  });

  testWidgets('can extend selection with a11y means - character', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const bool extendSelection = true;
    const bool doNotExtendSelection = false;

    controller.text = 'test';
    controller.selection = TextSelection.collapsed(offset: controller.text.length);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
        ],
      ),
    );

    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    final int semanticsId = render.debugSemantics!.id;

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 4);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 3);

    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      extendSelection,
    );
    await tester.pumpAndSettle();
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      extendSelection,
    );
    await tester.pumpAndSettle();
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByCharacter,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 0);

    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(
        value: 'test',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByCharacter,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 1);
    expect(controller.selection.extentOffset, 1);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByCharacter,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 1);
    expect(controller.selection.extentOffset, 2);

    semantics.dispose();
  });

  testWidgets('can extend selection with a11y means - word', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const bool extendSelection = true;
    const bool doNotExtendSelection = false;

    controller.text = 'test for words';
    controller.selection = TextSelection.collapsed(offset: controller.text.length);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
        ],
      ),
    );

    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    final int semanticsId = render.debugSemantics!.id;

    expect(controller.selection.baseOffset, 14);
    expect(controller.selection.extentOffset, 14);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 14);
    expect(controller.selection.extentOffset, 9);

    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorBackwardByCharacter,
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorBackwardByWord,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 14);
    expect(controller.selection.extentOffset, 5);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorBackwardByWord,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 14);
    expect(controller.selection.extentOffset, 0);

    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(
        value: 'test for words',
        actions: <SemanticsAction>[
          SemanticsAction.moveCursorForwardByCharacter,
          SemanticsAction.moveCursorForwardByWord,
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByWord,
      doNotExtendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 5);

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      semanticsId,
      SemanticsAction.moveCursorForwardByWord,
      extendSelection,
    );
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 9);

    semantics.dispose();
  });

  testWidgets('password fields have correct semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    controller.text = 'super-secret-password!!1';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          obscureText: true,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    final String expectedValue = '•' * controller.text.length;

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[
                            SemanticsFlag.isTextField,
                            SemanticsFlag.isObscured,
                          ],
                          value: expectedValue,
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
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('password fields become obscured with the right semantics when set', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    const String originalText = 'super-secret-password!!1';
    controller.text = originalText;

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    final String expectedValue = '•' * originalText.length;

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isTextField],
                          value: originalText,
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
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );

    focusNode.requestFocus();

    // Now change it to make it obscure text.
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          obscureText: true,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    expect((findRenderEditable(tester).text! as TextSpan).text, expectedValue);

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[
                            SemanticsFlag.isTextField,
                            SemanticsFlag.isObscured,
                            SemanticsFlag.isFocused,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.moveCursorBackwardByCharacter,
                            SemanticsAction.setSelection,
                            SemanticsAction.setText,
                            SemanticsAction.moveCursorBackwardByWord,
                          ],
                          value: expectedValue,
                          textDirection: TextDirection.ltr,
                          // Focusing a single-line field on web selects it.
                          textSelection:
                              kIsWeb
                                  ? const TextSelection(baseOffset: 0, extentOffset: 24)
                                  : const TextSelection.collapsed(offset: 24),
                        ),
                      ],
                    ),
                  ],
                ),
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

  testWidgets('password fields can have their obscuring character customized', (
    WidgetTester tester,
  ) async {
    const String originalText = 'super-secret-password!!1';
    controller.text = originalText;

    const String obscuringCharacter = '#';
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          obscuringCharacter: obscuringCharacter,
          obscureText: true,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    final String expectedValue = obscuringCharacter * originalText.length;
    expect((findRenderEditable(tester).text! as TextSpan).text, expectedValue);
  });

  testWidgets(
    'password briefly shows last character when entered on mobile',
    (WidgetTester tester) async {
      final bool debugDeterministicCursor = EditableText.debugDeterministicCursor;
      EditableText.debugDeterministicCursor = false;
      addTearDown(() {
        EditableText.debugDeterministicCursor = debugDeterministicCursor;
      });

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            obscureText: true,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      );

      await tester.enterText(find.byType(EditableText), 'AA');
      await tester.pump();
      await tester.enterText(find.byType(EditableText), 'AAA');
      await tester.pump();

      expect((findRenderEditable(tester).text! as TextSpan).text, '••A');
      await tester.pump(const Duration(milliseconds: 500));
      await tester.pump(const Duration(milliseconds: 500));
      await tester.pump(const Duration(milliseconds: 500));
      expect((findRenderEditable(tester).text! as TextSpan).text, '•••');
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  group('a11y copy/cut/paste', () {
    Future<void> buildApp(MockTextSelectionControls controls, WidgetTester tester) {
      return tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: controls,
          ),
        ),
      );
    }

    late MockTextSelectionControls controls;

    setUp(() {
      controller.text = 'test';
      controller.selection = TextSelection.collapsed(offset: controller.text.length);

      controls = MockTextSelectionControls();
    });

    testWidgets('are exposed', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      controls.testCanCopy = false;
      controls.testCanCut = false;
      controls.testCanPaste = false;

      await buildApp(controls, tester);
      await tester.tap(find.byType(EditableText));
      await tester.pump();

      expect(
        semantics,
        includesNodeWith(
          value: 'test',
          actions: <SemanticsAction>[
            SemanticsAction.moveCursorBackwardByCharacter,
            SemanticsAction.moveCursorBackwardByWord,
            SemanticsAction.setSelection,
            SemanticsAction.setText,
          ],
        ),
      );

      controls.testCanCopy = true;
      await buildApp(controls, tester);
      expect(
        semantics,
        includesNodeWith(
          value: 'test',
          actions: <SemanticsAction>[
            SemanticsAction.moveCursorBackwardByCharacter,
            SemanticsAction.moveCursorBackwardByWord,
            SemanticsAction.setSelection,
            SemanticsAction.setText,
            SemanticsAction.copy,
          ],
        ),
      );

      controls.testCanCopy = false;
      controls.testCanPaste = true;
      await buildApp(controls, tester);
      await tester.pumpAndSettle();
      expect(
        semantics,
        includesNodeWith(
          value: 'test',
          actions: <SemanticsAction>[
            SemanticsAction.moveCursorBackwardByCharacter,
            SemanticsAction.moveCursorBackwardByWord,
            SemanticsAction.setSelection,
            SemanticsAction.setText,
            SemanticsAction.paste,
          ],
        ),
      );

      controls.testCanPaste = false;
      controls.testCanCut = true;
      await buildApp(controls, tester);
      expect(
        semantics,
        includesNodeWith(
          value: 'test',
          actions: <SemanticsAction>[
            SemanticsAction.moveCursorBackwardByCharacter,
            SemanticsAction.moveCursorBackwardByWord,
            SemanticsAction.setSelection,
            SemanticsAction.setText,
            SemanticsAction.cut,
          ],
        ),
      );

      controls.testCanCopy = true;
      controls.testCanCut = true;
      controls.testCanPaste = true;
      await buildApp(controls, tester);
      expect(
        semantics,
        includesNodeWith(
          value: 'test',
          actions: <SemanticsAction>[
            SemanticsAction.moveCursorBackwardByCharacter,
            SemanticsAction.moveCursorBackwardByWord,
            SemanticsAction.setSelection,
            SemanticsAction.setText,
            SemanticsAction.cut,
            SemanticsAction.copy,
            SemanticsAction.paste,
          ],
        ),
      );
      semantics.dispose();
    });

    testWidgets('can copy/cut/paste with a11y', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      controls.testCanCopy = true;
      controls.testCanCut = true;
      controls.testCanPaste = true;
      await buildApp(controls, tester);
      await tester.tap(find.byType(EditableText));
      await tester.pump();

      final SemanticsOwner owner = tester.binding.pipelineOwner.semanticsOwner!;
      const int expectedNodeId = 5;

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics.rootChild(
                id: 1,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics.rootChild(
                            id: expectedNodeId,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.isTextField,
                              SemanticsFlag.isFocused,
                            ],
                            actions: <SemanticsAction>[
                              SemanticsAction.moveCursorBackwardByCharacter,
                              SemanticsAction.moveCursorBackwardByWord,
                              SemanticsAction.setSelection,
                              SemanticsAction.setText,
                              SemanticsAction.copy,
                              SemanticsAction.cut,
                              SemanticsAction.paste,
                            ],
                            value: 'test',
                            textSelection: TextSelection.collapsed(offset: controller.text.length),
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

      owner.performAction(expectedNodeId, SemanticsAction.copy);
      expect(controls.copyCount, 1);

      owner.performAction(expectedNodeId, SemanticsAction.cut);
      expect(controls.cutCount, 1);

      owner.performAction(expectedNodeId, SemanticsAction.paste);
      expect(controls.pasteCount, 1);

      semantics.dispose();
    });

    // Regression test for b/201218542.
    testWidgets('copying with a11y works even when toolbar is hidden', (WidgetTester tester) async {
      Future<void> testByControls(TextSelectionControls controls) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final TextEditingController controller = TextEditingController(text: 'ABCDEFG');
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: controls,
            ),
          ),
        );
        await tester.tap(find.byType(EditableText));
        await tester.pump();

        final SemanticsOwner owner = tester.binding.pipelineOwner.semanticsOwner!;
        const int expectedNodeId = 5;

        expect(controller.value.selection.isCollapsed, isTrue);

        controller.selection = TextSelection(
          baseOffset: 0,
          extentOffset: controller.value.text.length,
        );
        await tester.pump();

        expect(find.text('Copy'), findsNothing);

        owner.performAction(expectedNodeId, SemanticsAction.copy);
        expect(tester.takeException(), isNull);
        expect((await Clipboard.getData(Clipboard.kTextPlain))!.text, equals('ABCDEFG'));

        semantics.dispose();
      }

      await testByControls(materialTextSelectionControls);
      await testByControls(cupertinoTextSelectionControls);
    });
  });

  testWidgets('can set text with a11y', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );
    await tester.tap(find.byType(EditableText));
    await tester.pump();

    final SemanticsOwner owner = tester.binding.pipelineOwner.semanticsOwner!;
    const int expectedNodeId = 4;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics.rootChild(
                          id: expectedNodeId,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.isTextField,
                            SemanticsFlag.isFocused,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.setSelection,
                            SemanticsAction.setText,
                          ],
                          textSelection: TextSelection.collapsed(offset: controller.text.length),
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

    expect(controller.text, '');
    owner.performAction(expectedNodeId, SemanticsAction.setText, 'how are you');
    expect(controller.text, 'how are you');

    semantics.dispose();
  });

  testWidgets('allows customizing text style in subclasses', (WidgetTester tester) async {
    controller.text = 'Hello World';

    await tester.pumpWidget(
      MaterialApp(
        home: CustomStyleEditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
        ),
      ),
    );

    // Simulate selection change via tap to show handles.
    final RenderEditable render = tester.allRenderObjects.whereType<RenderEditable>().first;
    expect(render.text!.style!.fontStyle, FontStyle.italic);
  });

  testWidgets('onChanged callback only invoked on text changes', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/111651 .
    int onChangedCount = 0;
    bool preventInput = false;
    final TextInputFormatter formatter = TextInputFormatter.withFunction((
      TextEditingValue oldValue,
      TextEditingValue newValue,
    ) {
      return preventInput ? oldValue : newValue;
    });

    final Widget widget = MediaQuery(
      data: const MediaQueryData(),
      child: EditableText(
        controller: controller,
        backgroundCursorColor: Colors.red,
        cursorColor: Colors.red,
        focusNode: focusNode,
        style: textStyle,
        onChanged: (String newString) {
          onChangedCount += 1;
        },
        inputFormatters: <TextInputFormatter>[formatter],
        textDirection: TextDirection.ltr,
      ),
    );
    await tester.pumpWidget(widget);
    final EditableTextState state = tester.firstState(find.byType(EditableText));
    state.updateEditingValue(
      const TextEditingValue(text: 'a', composing: TextRange(start: 0, end: 1)),
    );
    expect(onChangedCount, 1);

    state.updateEditingValue(const TextEditingValue(text: 'a'));
    expect(onChangedCount, 1);

    state.updateEditingValue(const TextEditingValue(text: 'ab'));
    expect(onChangedCount, 2);

    preventInput = true;
    state.updateEditingValue(const TextEditingValue(text: 'abc'));
    expect(onChangedCount, 2);
  });

  testWidgets('Formatters are skipped if text has not changed', (WidgetTester tester) async {
    int called = 0;
    final TextInputFormatter formatter = TextInputFormatter.withFunction((
      TextEditingValue oldValue,
      TextEditingValue newValue,
    ) {
      called += 1;
      return newValue;
    });
    final MediaQuery mediaQuery = MediaQuery(
      data: const MediaQueryData(),
      child: EditableText(
        controller: controller,
        backgroundCursorColor: Colors.red,
        cursorColor: Colors.red,
        focusNode: focusNode,
        style: textStyle,
        inputFormatters: <TextInputFormatter>[formatter],
        textDirection: TextDirection.ltr,
      ),
    );
    await tester.pumpWidget(mediaQuery);
    final EditableTextState state = tester.firstState(find.byType(EditableText));
    state.updateEditingValue(const TextEditingValue(text: 'a'));
    expect(called, 1);
    // same value.
    state.updateEditingValue(const TextEditingValue(text: 'a'));
    expect(called, 1);
    // same value with different selection.
    state.updateEditingValue(
      const TextEditingValue(text: 'a', selection: TextSelection.collapsed(offset: 1)),
    );
    // different value.
    state.updateEditingValue(const TextEditingValue(text: 'b'));
    expect(called, 2);
  });

  testWidgets('default keyboardAppearance is respected', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/22212.

    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: Typography.material2018().black.titleMedium!,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    final MethodCall setClient = log.first;
    expect(setClient.method, 'TextInput.setClient');
    expect(
      ((setClient.arguments as Iterable<dynamic>).last
          as Map<String, dynamic>)['keyboardAppearance'],
      'Brightness.light',
    );
  });

  testWidgets('location of widget is sent on show keyboard', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: Typography.material2018().black.titleMedium!,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    final MethodCall methodCall = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 600,
          'transform': Matrix4.identity().storage.toList(),
        },
      ),
    );
  });

  testWidgets('transform and size is reset when text connection opens', (
    WidgetTester tester,
  ) async {
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    final TextEditingController controller1 = TextEditingController();
    addTearDown(controller1.dispose);
    final FocusNode focusNode1 = FocusNode();
    addTearDown(focusNode1.dispose);
    final TextEditingController controller2 = TextEditingController();
    addTearDown(controller2.dispose);
    final FocusNode focusNode2 = FocusNode();
    addTearDown(focusNode2.dispose);
    controller1.text = 'Text1';
    controller2.text = 'Text2';

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              EditableText(
                key: ValueKey<String>(controller1.text),
                controller: controller1,
                focusNode: focusNode1,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
              ),
              const SizedBox(height: 200.0),
              EditableText(
                key: ValueKey<String>(controller2.text),
                controller: controller2,
                focusNode: focusNode2,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                minLines: 10,
                maxLines: 20,
              ),
              const SizedBox(height: 100.0),
            ],
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byKey(ValueKey<String>(controller1.text)));
    final MethodCall methodCall = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 14,
          'transform': Matrix4.identity().storage.toList(),
        },
      ),
    );

    log.clear();

    // Move to the next editable text.
    await tester.showKeyboard(find.byKey(ValueKey<String>(controller2.text)));
    final MethodCall methodCall2 = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall2,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 140.0,
          'transform': <double>[
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            214.0,
            0.0,
            1.0,
          ],
        },
      ),
    );

    log.clear();

    // Move back to the first editable text.
    await tester.showKeyboard(find.byKey(ValueKey<String>(controller1.text)));
    final MethodCall methodCall3 = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall3,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 14,
          'transform': Matrix4.identity().storage.toList(),
        },
      ),
    );
  });

  testWidgets('size and transform are sent when they change', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    const Offset offset = Offset(10.0, 20.0);
    const Key transformButtonKey = Key('transformButton');
    await tester.pumpWidget(
      const TransformedEditableText(offset: offset, transformButtonKey: transformButtonKey),
    );

    await tester.showKeyboard(find.byType(EditableText));
    MethodCall methodCall = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 14,
          'transform': Matrix4.identity().storage.toList(),
        },
      ),
    );

    log.clear();
    await tester.tap(find.byKey(transformButtonKey));
    await tester.pumpAndSettle();

    // There should be a new platform message updating the transform.
    methodCall = log.firstWhere(
      (MethodCall m) => m.method == 'TextInput.setEditableSizeAndTransform',
    );
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditableSizeAndTransform',
        arguments: <String, dynamic>{
          'width': 800,
          'height': 14,
          'transform': Matrix4.translationValues(offset.dx, offset.dy, 0.0).storage.toList(),
        },
      ),
    );
  });

  testWidgets('text styling info is sent on show keyboard', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: EditableText(
          textDirection: TextDirection.rtl,
          controller: controller,
          focusNode: focusNode,
          style: const TextStyle(fontSize: 20.0, fontFamily: 'Roboto', fontWeight: FontWeight.w600),
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    final MethodCall setStyle = log.firstWhere((MethodCall m) => m.method == 'TextInput.setStyle');
    expect(
      setStyle,
      isMethodCall(
        'TextInput.setStyle',
        arguments: <String, dynamic>{
          'fontSize': 20.0,
          'fontFamily': 'Roboto',
          'fontWeightIndex': 5,
          'textAlignIndex': 4,
          'textDirectionIndex': 0,
        },
      ),
    );
  });

  testWidgets('text styling info is sent on show keyboard (bold override)', (
    WidgetTester tester,
  ) async {
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(boldText: true),
        child: EditableText(
          textDirection: TextDirection.rtl,
          controller: controller,
          focusNode: focusNode,
          style: const TextStyle(fontSize: 20.0, fontFamily: 'Roboto', fontWeight: FontWeight.w600),
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    final MethodCall setStyle = log.firstWhere((MethodCall m) => m.method == 'TextInput.setStyle');
    expect(
      setStyle,
      isMethodCall(
        'TextInput.setStyle',
        arguments: <String, dynamic>{
          'fontSize': 20.0,
          'fontFamily': 'Roboto',
          'fontWeightIndex': FontWeight.bold.index,
          'textAlignIndex': 4,
          'textDirectionIndex': 0,
        },
      ),
    );
  });

  testWidgets('text styling info is sent on style update', (WidgetTester tester) async {
    final GlobalKey<EditableTextState> editableTextKey = GlobalKey<EditableTextState>();
    late StateSetter setState;
    const TextStyle textStyle1 = TextStyle(
      fontSize: 20.0,
      fontFamily: 'RobotoMono',
      fontWeight: FontWeight.w600,
    );
    const TextStyle textStyle2 = TextStyle(
      fontSize: 20.0,
      fontFamily: 'Raleway',
      fontWeight: FontWeight.w700,
    );
    TextStyle currentTextStyle = textStyle1;

    Widget builder() {
      return StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          setState = setter;
          return MaterialApp(
            home: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Center(
                  child: Material(
                    child: EditableText(
                      backgroundCursorColor: Colors.grey,
                      key: editableTextKey,
                      controller: controller,
                      focusNode: focusNode,
                      style: currentTextStyle,
                      cursorColor: Colors.blue,
                      selectionControls: materialTextSelectionControls,
                      keyboardType: TextInputType.text,
                      onChanged: (String value) {},
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    await tester.pumpWidget(builder());
    await tester.showKeyboard(find.byType(EditableText));

    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });
    setState(() {
      currentTextStyle = textStyle2;
    });
    await tester.pump();

    // Updated styling information should be sent via TextInput.setStyle method.
    final MethodCall setStyle = log.firstWhere((MethodCall m) => m.method == 'TextInput.setStyle');
    expect(
      setStyle,
      isMethodCall(
        'TextInput.setStyle',
        arguments: <String, dynamic>{
          'fontSize': 20.0,
          'fontFamily': 'Raleway',
          'fontWeightIndex': 6,
          'textAlignIndex': 4,
          'textDirectionIndex': 1,
        },
      ),
    );
  });

  group('setCaretRect', () {
    Widget builder() {
      return MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Material(
                child: EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: Colors.blue,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  onChanged: (String value) {},
                ),
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('called with proper coordinates', (WidgetTester tester) async {
      controller.value = TextEditingValue(text: 'a' * 50);
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setCaretRect',
            args: allOf(
              // No composing text so the width should not be too wide because
              // it's empty.
              containsPair('x', equals(700)),
              containsPair('y', equals(0)),
              containsPair('width', equals(2)),
              containsPair('height', equals(14)),
            ),
          ),
        ),
      );

      tester.testTextInput.log.clear();

      controller.value = TextEditingValue(
        text: 'a' * 50,
        selection: const TextSelection(baseOffset: 0, extentOffset: 0),
      );
      await tester.pump();

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setCaretRect',
            // Now the composing range is not empty.
            args: allOf(containsPair('x', equals(0)), containsPair('y', equals(0))),
          ),
        ),
      );
    });

    testWidgets('only send updates when necessary', (WidgetTester tester) async {
      controller.value = TextEditingValue(text: 'a' * 100);
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(tester.testTextInput.log, contains(matchesMethodCall('TextInput.setCaretRect')));

      tester.testTextInput.log.clear();

      // Should not send updates every frame.
      await tester.pump();

      expect(
        tester.testTextInput.log,
        isNot(contains(matchesMethodCall('TextInput.setCaretRect'))),
      );
    });

    testWidgets('set to selection start on forward selection', (WidgetTester tester) async {
      controller.value = TextEditingValue(
        text: 'a' * 100,
        selection: const TextSelection(baseOffset: 10, extentOffset: 30),
      );
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setCaretRect',
            // Now the composing range is not empty.
            args: allOf(containsPair('x', equals(140)), containsPair('y', equals(0))),
          ),
        ),
      );
    });

    testWidgets('set to selection start on reversed selection', (WidgetTester tester) async {
      controller.value = TextEditingValue(
        text: 'a' * 100,
        selection: const TextSelection(baseOffset: 30, extentOffset: 10),
      );
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setCaretRect',
            // Now the composing range is not empty.
            args: allOf(containsPair('x', equals(140)), containsPair('y', equals(0))),
          ),
        ),
      );
    });
  });

  group('setMarkedTextRect', () {
    Widget builder() {
      return MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Material(
                child: EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: Colors.blue,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  onChanged: (String value) {},
                ),
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('called when the composing range changes', (WidgetTester tester) async {
      controller.value = TextEditingValue(text: 'a' * 100);
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setMarkedTextRect',
            args: allOf(
              // No composing text so the width should not be too wide because
              // it's empty.
              containsPair('width', lessThanOrEqualTo(5)),
              containsPair('x', lessThanOrEqualTo(1)),
            ),
          ),
        ),
      );

      tester.testTextInput.log.clear();

      controller.value = collapsedAtEnd(
        'a' * 100,
      ).copyWith(composing: const TextRange(start: 0, end: 10));
      await tester.pump();

      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setMarkedTextRect',
            // Now the composing range is not empty.
            args: containsPair('width', greaterThanOrEqualTo(10)),
          ),
        ),
      );
    });

    testWidgets('only send updates when necessary', (WidgetTester tester) async {
      controller.value = TextEditingValue(
        text: 'a' * 100,
        composing: const TextRange(start: 0, end: 10),
      );
      await tester.pumpWidget(builder());
      await tester.showKeyboard(find.byType(EditableText));

      expect(tester.testTextInput.log, contains(matchesMethodCall('TextInput.setMarkedTextRect')));

      tester.testTextInput.log.clear();

      // Should not send updates every frame.
      await tester.pump();

      expect(
        tester.testTextInput.log,
        isNot(contains(matchesMethodCall('TextInput.setMarkedTextRect'))),
      );
    });

    testWidgets('zero matrix paint transform', (WidgetTester tester) async {
      controller.value = TextEditingValue(
        text: 'a' * 100,
        composing: const TextRange(start: 0, end: 10),
      );
      // Use a FittedBox with an zero-sized child to set the paint transform
      // to the zero matrix.
      await tester.pumpWidget(
        FittedBox(child: SizedBox.fromSize(size: Size.zero, child: builder())),
      );
      await tester.showKeyboard(find.byType(EditableText));
      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setMarkedTextRect',
            args: allOf(
              containsPair('width', isNotNaN),
              containsPair('height', isNotNaN),
              containsPair('x', isNotNaN),
              containsPair('y', isNotNaN),
            ),
          ),
        ),
      );
    });
  });

  testWidgets('custom keyboardAppearance is respected', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/22212.

    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: Typography.material2018().black.titleMedium!,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            keyboardAppearance: Brightness.dark,
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    final MethodCall setClient = log.first;
    expect(setClient.method, 'TextInput.setClient');
    expect(
      ((setClient.arguments as Iterable<dynamic>).last
          as Map<String, dynamic>)['keyboardAppearance'],
      'Brightness.dark',
    );
  });

  testWidgets('Composing text is underlined and underline is cleared when losing focus', (
    WidgetTester tester,
  ) async {
    controller.value = const TextEditingValue(
      text: 'text composing text',
      selection: TextSelection.collapsed(offset: 14),
      composing: TextRange(start: 5, end: 14),
    );

    await tester.pumpWidget(
      MaterialApp(
        // So we can show overlays.
        home: EditableText(
          autofocus: true,
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
          keyboardType: TextInputType.text,
          onEditingComplete: () {
            // This prevents the default focus change behavior on submission.
          },
        ),
      ),
    );

    assert(focusNode.hasFocus);
    // Autofocus has a one frame delay.
    await tester.pump();

    final RenderEditable renderEditable = findRenderEditable(tester);
    // The actual text span is split into 3 parts with the middle part underlined.
    expect((renderEditable.text! as TextSpan).children!.length, 3);
    final TextSpan textSpan = (renderEditable.text! as TextSpan).children![1] as TextSpan;
    expect(textSpan.text, 'composing');
    expect(textSpan.style!.decoration, TextDecoration.underline);

    focusNode.unfocus();
    // Drain microtasks.
    await tester.idle();
    await tester.pump();

    expect((renderEditable.text! as TextSpan).children, isNull);
    // Everything's just formatted the same way now.
    expect((renderEditable.text! as TextSpan).text, 'text composing text');
    expect(renderEditable.text!.style!.decoration, isNull);
  });

  testWidgets('text selection toolbar visibility', (WidgetTester tester) async {
    controller.text = 'hello \n world \n this \n is \n text';

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: Container(
            height: 50,
            color: Colors.white,
            child: EditableText(
              showSelectionHandles: true,
              controller: controller,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
              selectionColor: Colors.lightBlueAccent,
              maxLines: 3,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable renderEditable = state.renderEditable;
    final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));

    // Select the first word. And show the toolbar.
    await tester.tapAt(const Offset(20, 10));
    renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    expect(state.showToolbar(), true);
    await tester.pumpAndSettle();

    // Find the toolbar fade transition while the toolbar is still visible.
    final List<FadeTransition> transitionsBefore =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_SelectionToolbarWrapper',
              ),
              matching: find.byType(FadeTransition),
            )
            .evaluate()
            .map((Element e) => e.widget)
            .cast<FadeTransition>()
            .toList();

    expect(transitionsBefore.length, 1);

    final FadeTransition toolbarBefore = transitionsBefore[0];

    expect(toolbarBefore.opacity.value, 1.0);

    // Scroll until the selection is no longer within view.
    scrollable.controller!.jumpTo(50.0);
    await tester.pumpAndSettle();

    // Try to find the toolbar fade transition after the toolbar has been hidden
    // as a result of a scroll. This removes the toolbar overlay entry so no fade
    // transition should be found.
    final List<FadeTransition> transitionsAfter =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_SelectionToolbarWrapper',
              ),
              matching: find.byType(FadeTransition),
            )
            .evaluate()
            .map((Element e) => e.widget)
            .cast<FadeTransition>()
            .toList();
    expect(transitionsAfter.length, 0);
    expect(state.selectionOverlay, isNotNull);
    expect(state.selectionOverlay!.toolbarIsVisible, false);

    // On web, we don't show the Flutter toolbar and instead rely on the browser
    // toolbar. Until we change that, this test should remain skipped.
  }, skip: kIsWeb); // [intended]

  testWidgets('text selection handle visibility', (WidgetTester tester) async {
    // Text with two separate words to select.
    controller.text = 'XXXXX          XXXXX';

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 100,
            child: EditableText(
              showSelectionHandles: true,
              controller: controller,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable renderEditable = state.renderEditable;
    final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));

    bool expectedLeftVisibleBefore = false;
    bool expectedRightVisibleBefore = false;

    Future<void> verifyVisibility(
      HandlePositionInViewport leftPosition,
      bool expectedLeftVisible,
      HandlePositionInViewport rightPosition,
      bool expectedRightVisible,
    ) async {
      await tester.pump();

      // Check the signal from RenderEditable about whether they're within the
      // viewport.

      expect(renderEditable.selectionStartInViewport.value, equals(expectedLeftVisible));
      expect(renderEditable.selectionEndInViewport.value, equals(expectedRightVisible));

      // Check that the animations are functional and going in the right
      // direction.

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

      if (expectedLeftVisibleBefore) {
        expect(left.opacity.value, equals(1.0));
      }
      if (expectedRightVisibleBefore) {
        expect(right.opacity.value, equals(1.0));
      }

      await tester.pump(SelectionOverlay.fadeDuration ~/ 2);

      if (expectedLeftVisible != expectedLeftVisibleBefore) {
        expect(left.opacity.value, equals(0.5));
      }
      if (expectedRightVisible != expectedRightVisibleBefore) {
        expect(right.opacity.value, equals(0.5));
      }

      await tester.pump(SelectionOverlay.fadeDuration ~/ 2);

      if (expectedLeftVisible) {
        expect(left.opacity.value, equals(1.0));
      }
      if (expectedRightVisible) {
        expect(right.opacity.value, equals(1.0));
      }

      expectedLeftVisibleBefore = expectedLeftVisible;
      expectedRightVisibleBefore = expectedRightVisible;

      // Check that the handles' positions are correct.

      final List<RenderBox> handles = List<RenderBox>.from(
        tester.renderObjectList<RenderBox>(
          find.descendant(
            of: find.byType(CompositedTransformFollower),
            matching: find.byType(Padding),
          ),
        ),
      );

      final Size viewport = renderEditable.size;

      void testPosition(double pos, HandlePositionInViewport expected) {
        switch (expected) {
          case HandlePositionInViewport.leftEdge:
            expect(
              pos,
              inExclusiveRange(0 - kMinInteractiveDimension, 0 + kMinInteractiveDimension),
            );
          case HandlePositionInViewport.rightEdge:
            expect(
              pos,
              inExclusiveRange(
                viewport.width - kMinInteractiveDimension,
                viewport.width + kMinInteractiveDimension,
              ),
            );
          case HandlePositionInViewport.within:
            expect(
              pos,
              inExclusiveRange(
                0 - kMinInteractiveDimension,
                viewport.width + kMinInteractiveDimension,
              ),
            );
        }
      }

      expect(state.selectionOverlay!.handlesAreVisible, isTrue);
      testPosition(handles[0].localToGlobal(Offset.zero).dx, leftPosition);
      testPosition(handles[1].localToGlobal(Offset.zero).dx, rightPosition);
    }

    // Select the first word. Both handles should be visible.
    await tester.tapAt(const Offset(20, 10));
    renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pump();
    await verifyVisibility(
      HandlePositionInViewport.leftEdge,
      true,
      HandlePositionInViewport.within,
      true,
    );

    // Drag the text slightly so the first word is partially visible. Only the
    // right handle should be visible.
    scrollable.controller!.jumpTo(20.0);
    await verifyVisibility(
      HandlePositionInViewport.leftEdge,
      false,
      HandlePositionInViewport.within,
      true,
    );

    // Drag the text all the way to the left so the first word is not visible at
    // all (and the second word is fully visible). Both handles should be
    // invisible now.
    scrollable.controller!.jumpTo(200.0);
    await verifyVisibility(
      HandlePositionInViewport.leftEdge,
      false,
      HandlePositionInViewport.leftEdge,
      false,
    );

    // Tap to unselect.
    await tester.tap(find.byType(EditableText));
    await tester.pump();

    // Now that the second word has been dragged fully into view, select it.
    await tester.tapAt(const Offset(80, 10));
    renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pump();
    await verifyVisibility(
      HandlePositionInViewport.within,
      true,
      HandlePositionInViewport.within,
      true,
    );

    // Drag the text slightly to the right. Only the left handle should be
    // visible.
    scrollable.controller!.jumpTo(150);
    await verifyVisibility(
      HandlePositionInViewport.within,
      true,
      HandlePositionInViewport.rightEdge,
      false,
    );

    // Drag the text all the way to the right, so the second word is not visible
    // at all. Again, both handles should be invisible.
    scrollable.controller!.jumpTo(0);
    await verifyVisibility(
      HandlePositionInViewport.rightEdge,
      false,
      HandlePositionInViewport.rightEdge,
      false,
    );

    // On web, we don't show the Flutter toolbar and instead rely on the browser
    // toolbar. Until we change that, this test should remain skipped.
  }, skip: kIsWeb); // [intended]

  testWidgets('text selection handle visibility RTL', (WidgetTester tester) async {
    // Text with two separate words to select.
    controller.text = 'XXXXX          XXXXX';

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 100,
            child: EditableText(
              controller: controller,
              showSelectionHandles: true,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
              textAlign: TextAlign.right,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Select the first word. Both handles should be visible.
    await tester.tapAt(const Offset(20, 10));
    state.renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pump();
    final List<RenderBox> handles = List<RenderBox>.from(
      tester.renderObjectList<RenderBox>(
        find.descendant(
          of: find.byType(CompositedTransformFollower),
          matching: find.byType(Padding),
        ),
      ),
    );
    expect(
      handles[0].localToGlobal(Offset.zero).dx,
      inExclusiveRange(-kMinInteractiveDimension, kMinInteractiveDimension),
    );
    expect(
      handles[1].localToGlobal(Offset.zero).dx,
      inExclusiveRange(70.0 - kMinInteractiveDimension, 70.0 + kMinInteractiveDimension),
    );
    expect(state.selectionOverlay!.handlesAreVisible, isTrue);
    expect(controller.selection.base.offset, 0);
    expect(controller.selection.extent.offset, 5);

    // On web, we don't show the Flutter toolbar and instead rely on the browser
    // toolbar. Until we change that, this test should remain skipped.
  }, skip: kIsWeb); // [intended]

  const String testText =
      'Now is the time for\n' // 20
      'all good people\n' // 20 + 16 => 36
      'to come to the aid\n' // 36 + 19 => 55
      'of their country.'; // 55 + 17 => 72

  Future<void> testTextEditing(
    WidgetTester tester, {
    required TargetPlatform targetPlatform,
  }) async {
    final String targetPlatformString = targetPlatform.toString();
    final String platform =
        targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
    controller.text = testText;
    controller.selection = const TextSelection(
      baseOffset: 0,
      extentOffset: 0,
      affinity: TextAffinity.upstream,
    );
    late TextSelection selection;
    late SelectionChangedCause cause;
    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 400,
            child: EditableText(
              maxLines: 10,
              controller: controller,
              showSelectionHandles: true,
              autofocus: true,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
              textAlign: TextAlign.right,
              onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                selection = newSelection;
                cause = newCause!;
              },
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Wait for autofocus to take effect.

    // Select a few characters using shift right arrow
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 3, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );

    // Select fewer characters using shift left arrow
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowLeft,
        LogicalKeyboardKey.arrowLeft,
        LogicalKeyboardKey.arrowLeft,
      ],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 0)),
      reason: 'on $platform',
    );

    // Try to select before the first character, nothing should change.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 0)),
      reason: 'on $platform',
    );

    // Select the first two words.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight, LogicalKeyboardKey.arrowRight],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 6)),
      reason: 'on $platform',
    );

    // Unselect the second word.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 4, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );

    // Select the next line.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 20, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );

    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.arrowRight,
    ], targetPlatform: defaultTargetPlatform);

    expect(
      selection,
      equals(const TextSelection(baseOffset: 20, extentOffset: 20)),
      reason: 'on $platform',
    );

    // Select the next line.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 20, extentOffset: 39)),
      reason: 'on $platform',
    );

    // Select to the end of the string by going down.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowDown,
      ],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 20, extentOffset: testText.length)),
      reason: 'on $platform',
    );

    // Go back up one line to set selection up to part of the last line.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 20, extentOffset: 39)),
      reason: 'on $platform',
    );

    // Select to the end of the selection.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
      lineModifier: true,
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(
        const TextSelection(baseOffset: 20, extentOffset: 54, affinity: TextAffinity.upstream),
      ),
      reason: 'on $platform',
    );

    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
      lineModifier: true,
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    switch (defaultTargetPlatform) {
      // These platforms extend by line.
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        expect(
          selection,
          equals(
            const TextSelection(baseOffset: 20, extentOffset: 36, affinity: TextAffinity.upstream),
          ),
          reason: 'on $platform',
        );

      // Mac and iOS expand by line.
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        expect(
          selection,
          equals(
            const TextSelection(baseOffset: 20, extentOffset: 54, affinity: TextAffinity.upstream),
          ),
          reason: 'on $platform',
        );
    }

    // Select All
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.keyA],
      shortcutModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(
        const TextSelection(
          baseOffset: 0,
          extentOffset: testText.length,
          affinity: TextAffinity.upstream,
        ),
      ),
      reason: 'on $platform',
    );

    // Jump to beginning of selection.
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.arrowLeft,
    ], targetPlatform: defaultTargetPlatform);

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 0)),
      reason: 'on $platform',
    );

    // Jump to end.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      lineModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection.collapsed(offset: testText.length)),
      reason: 'on $platform',
    );
    expect(controller.text, equals(testText), reason: 'on $platform');

    // Jump to start.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      lineModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(selection, equals(const TextSelection.collapsed(offset: 0)), reason: 'on $platform');
    expect(controller.text, equals(testText), reason: 'on $platform');

    // Move forward a few letters
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.arrowRight,
      LogicalKeyboardKey.arrowRight,
      LogicalKeyboardKey.arrowRight,
    ], targetPlatform: defaultTargetPlatform);

    expect(selection, equals(const TextSelection.collapsed(offset: 3)), reason: 'on $platform');

    // Select to end.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      lineModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 3, extentOffset: testText.length)),
      reason: 'on $platform',
    );

    // Select to start, which extends the selection.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      lineModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    switch (defaultTargetPlatform) {
      // Extend selection.
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        expect(
          selection,
          equals(
            const TextSelection(baseOffset: 3, extentOffset: 0, affinity: TextAffinity.upstream),
          ),
          reason: 'on $platform',
        );
      // On macOS/iOS expand selection.
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        expect(
          selection,
          equals(const TextSelection(baseOffset: 72, extentOffset: 0)),
          reason: 'on $platform',
        );
    }

    // Move to start again.
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.arrowUp,
    ], targetPlatform: defaultTargetPlatform);

    expect(selection, equals(const TextSelection.collapsed(offset: 0)), reason: 'on $platform');

    // Move down by page.
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.pageDown,
    ], targetPlatform: defaultTargetPlatform);

    // On macOS, pageDown/Up don't change selection.
    expect(
      selection,
      equals(
        defaultTargetPlatform == TargetPlatform.macOS || defaultTargetPlatform == TargetPlatform.iOS
            ? const TextSelection.collapsed(offset: 0)
            : const TextSelection.collapsed(offset: 55),
      ),
      reason: 'on $platform',
    );

    // Move up by page (to start).
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.pageUp,
    ], targetPlatform: defaultTargetPlatform);

    expect(selection, equals(const TextSelection.collapsed(offset: 0)), reason: 'on $platform');

    // Select towards end by page.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.pageDown],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 55, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );

    // Change selection extent towards start by page.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.pageUp],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(selection, equals(const TextSelection.collapsed(offset: 0)), reason: 'on $platform');

    // Jump forward three words.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ],
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 10)),
      reason: 'on $platform',
    );

    // Select some characters backward.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowLeft,
        LogicalKeyboardKey.arrowLeft,
        LogicalKeyboardKey.arrowLeft,
      ],
      shift: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 7, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );

    // Select a word backward.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 4, affinity: TextAffinity.upstream)),
      reason: 'on $platform',
    );
    expect(controller.text, equals(testText), reason: 'on $platform');

    // Cut
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.keyX],
      shortcutModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 4, extentOffset: 4)),
      reason: 'on $platform',
    );
    expect(
      controller.text,
      equals(
        'Now  time for\n'
        'all good people\n'
        'to come to the aid\n'
        'of their country.',
      ),
      reason: 'on $platform',
    );
    expect(
      (await Clipboard.getData(Clipboard.kTextPlain))!.text,
      equals('is the'),
      reason: 'on $platform',
    );

    // Paste
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.keyV],
      shortcutModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 10)),
      reason: 'on $platform',
    );
    expect(controller.text, equals(testText), reason: 'on $platform');

    final bool platformIsApple =
        defaultTargetPlatform == TargetPlatform.iOS ||
        defaultTargetPlatform == TargetPlatform.macOS;
    // Move down one paragraph.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 20)),
      reason: 'on $platform',
    );

    // Move down another paragraph.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 36)),
      reason: 'on $platform',
    );

    // Move down another paragraph.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 55)),
      reason: 'on $platform',
    );

    // Move up a paragraph.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 36)),
      reason: 'on $platform',
    );

    // Move up a paragraph.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 20)),
      reason: 'on $platform',
    );

    // Move up. This will collapse the selection to the origin on Apple platforms, and
    // extend to the previous paragraph boundary on other platforms.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(TextSelection(baseOffset: 10, extentOffset: platformIsApple ? 10 : 0)),
      reason: 'on $platform',
    );

    // Move up, extending the selection backwards to the previous paragraph on Apple platforms.
    // On other platforms this does nothing since our extent is already at 0 from the previous
    // set of keys sent.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowUp],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(const TextSelection(baseOffset: 10, extentOffset: 0)),
      reason: 'on $platform',
    );

    // Move down, collapsing the selection to the origin on Apple platforms.
    // On other platforms this moves the selection's extent to the next paragraph boundary.
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.arrowDown],
      shift: true,
      wordModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(
        TextSelection(
          baseOffset: 10,
          extentOffset: platformIsApple ? 10 : 20,
          affinity: platformIsApple ? TextAffinity.upstream : TextAffinity.downstream,
        ),
      ),
      reason: 'on $platform',
    );

    // Copy All
    await sendKeys(
      tester,
      <LogicalKeyboardKey>[LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyC],
      shortcutModifier: true,
      targetPlatform: defaultTargetPlatform,
    );

    expect(
      selection,
      equals(
        const TextSelection(
          baseOffset: 0,
          extentOffset: testText.length,
          affinity: TextAffinity.upstream,
        ),
      ),
      reason: 'on $platform',
    );
    expect(controller.text, equals(testText), reason: 'on $platform');
    expect((await Clipboard.getData(Clipboard.kTextPlain))!.text, equals(testText));

    if (defaultTargetPlatform != TargetPlatform.iOS) {
      // Delete
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.delete,
      ], targetPlatform: defaultTargetPlatform);
      expect(
        selection,
        equals(const TextSelection(baseOffset: 0, extentOffset: 0)),
        reason: 'on $platform',
      );
      expect(controller.text, isEmpty, reason: 'on $platform');

      controller.text = 'abc';
      controller.selection = const TextSelection(baseOffset: 2, extentOffset: 2);

      // Backspace
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.backspace,
      ], targetPlatform: defaultTargetPlatform);
      expect(
        selection,
        equals(const TextSelection(baseOffset: 1, extentOffset: 1)),
        reason: 'on $platform',
      );
      expect(controller.text, 'ac', reason: 'on $platform');

      // Shift-backspace (same as backspace)
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.backspace],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        selection,
        equals(const TextSelection(baseOffset: 0, extentOffset: 0)),
        reason: 'on $platform',
      );
      expect(controller.text, 'c', reason: 'on $platform');
    }
  }

  testWidgets(
    'keyboard text selection works (RawKeyEvent)',
    (WidgetTester tester) async {
      debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.rawKeyData;

      await testTextEditing(tester, targetPlatform: defaultTargetPlatform);

      debugKeyEventSimulatorTransitModeOverride = null;

      // On web, using keyboard for selection is handled by the browser.
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'keyboard text selection works (ui.KeyData then RawKeyEvent)',
    (WidgetTester tester) async {
      debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.keyDataThenRawKeyData;

      await testTextEditing(tester, targetPlatform: defaultTargetPlatform);

      debugKeyEventSimulatorTransitModeOverride = null;

      // On web, using keyboard for selection is handled by the browser.
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'keyboard shortcuts respect read-only',
    (WidgetTester tester) async {
      final String platform = defaultTargetPlatform.name.toLowerCase();
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: testText.length ~/ 2,
        affinity: TextAffinity.upstream,
      );
      TextSelection? selection;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                readOnly: true,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      const String clipboardContent = 'read-only';
      await Clipboard.setData(const ClipboardData(text: clipboardContent));

      // Paste
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.keyV],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(selection, isNull, reason: 'on $platform');
      expect(controller.text, equals(testText), reason: 'on $platform');

      // Select All
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.keyA],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(
        selection,
        equals(
          const TextSelection(
            baseOffset: 0,
            extentOffset: testText.length,
            affinity: TextAffinity.upstream,
          ),
        ),
        reason: 'on $platform',
      );
      expect(controller.text, equals(testText), reason: 'on $platform');

      // Cut
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.keyX],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(
        selection,
        equals(
          const TextSelection(
            baseOffset: 0,
            extentOffset: testText.length,
            affinity: TextAffinity.upstream,
          ),
        ),
        reason: 'on $platform',
      );
      expect(controller.text, equals(testText), reason: 'on $platform');
      expect(
        (await Clipboard.getData(Clipboard.kTextPlain))!.text,
        equals(clipboardContent),
        reason: 'on $platform',
      );

      // Copy
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.keyC],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(
        selection,
        equals(
          const TextSelection(
            baseOffset: 0,
            extentOffset: testText.length,
            affinity: TextAffinity.upstream,
          ),
        ),
        reason: 'on $platform',
      );
      expect(controller.text, equals(testText), reason: 'on $platform');
      expect(
        (await Clipboard.getData(Clipboard.kTextPlain))!.text,
        equals(testText),
        reason: 'on $platform',
      );

      // Delete
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.delete,
      ], targetPlatform: defaultTargetPlatform);
      expect(
        selection,
        equals(
          const TextSelection(
            baseOffset: 0,
            extentOffset: testText.length,
            affinity: TextAffinity.upstream,
          ),
        ),
        reason: 'on $platform',
      );
      expect(controller.text, equals(testText), reason: 'on $platform');

      // Backspace
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.backspace,
      ], targetPlatform: defaultTargetPlatform);
      expect(
        selection,
        equals(
          const TextSelection(
            baseOffset: 0,
            extentOffset: testText.length,
            affinity: TextAffinity.upstream,
          ),
        ),
        reason: 'on $platform',
      );
      expect(controller.text, equals(testText), reason: 'on $platform');
    },
    // On web, using keyboard for selection is handled by the browser.
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'home/end keys',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 23)), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.home,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 23)),
            reason: 'on $platform',
          );

        // These platforms go to the line start/end.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 20)),
            reason: 'on $platform',
          );
      }

      expect(controller.text, equals(testText), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.end,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 23)),
            reason: 'on $platform',
          );

        // These platforms go to the line start/end.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 35, affinity: TextAffinity.upstream)),
            reason: 'on $platform',
          );
      }
      expect(controller.text, equals(testText), reason: 'on $platform');
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'home keys and wordwraps',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 32)), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.home,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // These platforms go to the line start/end.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 29)),
            reason: 'on $platform',
          );
      }

      expect(controller.text, equals(testText), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.home,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all still.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Linux does nothing at a wordwrap with subsequent presses.
        case TargetPlatform.linux:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 29)),
            reason: 'on $platform',
          );

        // Windows jumps to the previous wordwrapped line.
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 0)),
            reason: 'on $platform',
          );
      }
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'end keys and wordwraps',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 32)), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.end,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // These platforms go to the line start/end.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 58, affinity: TextAffinity.upstream)),
            reason: 'on $platform',
          );
      }
      expect(controller.text, equals(testText), reason: 'on $platform');

      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.end,
      ], targetPlatform: defaultTargetPlatform);

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all still.
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Linux does nothing at a wordwrap with subsequent presses.
        case TargetPlatform.linux:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 58, affinity: TextAffinity.upstream)),
            reason: 'on $platform',
          );

        // Windows jumps to the next wordwrapped line.
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 84, affinity: TextAffinity.upstream)),
            reason: 'on $platform',
          );
      }
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'shift + home/end keys',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump();

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 23)), reason: 'on $platform');

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(controller.text, equals(testText), reason: 'on $platform');
      final TextSelection selectionAfterHome = selection;

      // Move back to position 23.
      controller.selection = const TextSelection.collapsed(offset: 23);
      await tester.pump();

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      expect(controller.text, equals(testText), reason: 'on $platform');
      final TextSelection selectionAfterEnd = selection;

      switch (defaultTargetPlatform) {
        // These platforms don't handle shift + home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(
            selectionAfterHome,
            equals(const TextSelection(baseOffset: 23, extentOffset: 23)),
            reason: 'on $platform',
          );
          expect(
            selectionAfterEnd,
            equals(const TextSelection(baseOffset: 23, extentOffset: 23)),
            reason: 'on $platform',
          );

        // Linux extends to the line start/end.
        case TargetPlatform.linux:
          expect(
            selectionAfterHome,
            equals(const TextSelection(baseOffset: 23, extentOffset: 20)),
            reason: 'on $platform',
          );
          expect(
            selectionAfterEnd,
            equals(
              const TextSelection(
                baseOffset: 23,
                extentOffset: 35,
                affinity: TextAffinity.upstream,
              ),
            ),
            reason: 'on $platform',
          );

        // Windows expands to the line start/end.
        case TargetPlatform.windows:
          expect(
            selectionAfterHome,
            equals(const TextSelection(baseOffset: 23, extentOffset: 20)),
            reason: 'on $platform',
          );
          expect(
            selectionAfterEnd,
            equals(const TextSelection(baseOffset: 23, extentOffset: 35)),
            reason: 'on $platform',
          );

        // Mac and iOS go to the start/end of the document.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            selectionAfterHome,
            equals(
              const TextSelection(baseOffset: 23, extentOffset: 0, affinity: TextAffinity.upstream),
            ),
            reason: 'on $platform',
          );
          expect(
            selectionAfterEnd,
            equals(const TextSelection(baseOffset: 23, extentOffset: 72)),
            reason: 'on $platform',
          );
      }
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'shift + home/end keys (Windows only)',
    (WidgetTester tester) async {
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump();

      // Move the selection away from the start so it can invert.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);
      await tester.pump();
      expect(controller.selection, equals(const TextSelection.collapsed(offset: 4)));

      // Press shift + end and extend the selection to the end of the line.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 4, extentOffset: 19, affinity: TextAffinity.upstream),
        ),
      );

      // Press shift + home and the selection inverts and extends to the start, it
      // does not collapse and stop at the inversion.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(controller.selection, equals(const TextSelection(baseOffset: 4, extentOffset: 0)));

      // Press shift + end again and the selection inverts and extends to the end,
      // again it does not stop at the inversion.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(controller.selection, equals(const TextSelection(baseOffset: 4, extentOffset: 19)));
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.windows}),
  );

  testWidgets(
    'home/end keys scrolling (Mac only)',
    (WidgetTester tester) async {
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));

      expect(scrollable.controller!.offset, 0.0);

      // Scroll to the end of the document with the end key.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.end,
      ], targetPlatform: defaultTargetPlatform);
      final double maxScrollExtent = scrollable.controller!.position.maxScrollExtent;
      expect(scrollable.controller!.offset, maxScrollExtent);

      // Scroll back to the beginning of the document with the home key.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.home,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollable.controller!.offset, 0.0);
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets(
    'shift + home keys and wordwraps',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 32)), reason: 'on $platform');

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with shift + home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Mac and iOS select to the start of the document.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 0)),
            reason: 'on $platform',
          );

        // These platforms select to the line start.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 29)),
            reason: 'on $platform',
          );
      }

      expect(controller.text, equals(testText), reason: 'on $platform');

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all still.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Mac and iOS select to the start of the document.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 0)),
            reason: 'on $platform',
          );

        // Linux does nothing at a wordwrap with subsequent presses.
        case TargetPlatform.linux:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 29)),
            reason: 'on $platform',
          );

        // Windows jumps to the previous wordwrapped line.
        case TargetPlatform.windows:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 0)),
            reason: 'on $platform',
          );
      }
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'shift + end keys and wordwraps',
    (WidgetTester tester) async {
      final String targetPlatformString = defaultTargetPlatform.toString();
      final String platform =
          targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      late SelectionChangedCause cause;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                  cause = newCause!;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);

      expect(cause, equals(SelectionChangedCause.keyboard), reason: 'on $platform');
      expect(selection, equals(const TextSelection.collapsed(offset: 32)), reason: 'on $platform');

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Mac and iOS select to the end of the document.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 145)),
            reason: 'on $platform',
          );

        // These platforms select to the line end.
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            selection,
            equals(
              const TextSelection(
                baseOffset: 32,
                extentOffset: 58,
                affinity: TextAffinity.upstream,
              ),
            ),
            reason: 'on $platform',
          );
      }
      expect(controller.text, equals(testText), reason: 'on $platform');

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      switch (defaultTargetPlatform) {
        // These platforms don't move the selection with home/end at all still.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(
            selection,
            equals(const TextSelection.collapsed(offset: 32)),
            reason: 'on $platform',
          );

        // Mac and iOS stay at the end of the document.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            selection,
            equals(const TextSelection(baseOffset: 32, extentOffset: 145)),
            reason: 'on $platform',
          );

        // Linux does nothing at a wordwrap with subsequent presses.
        case TargetPlatform.linux:
          expect(
            selection,
            equals(
              const TextSelection(
                baseOffset: 32,
                extentOffset: 58,
                affinity: TextAffinity.upstream,
              ),
            ),
            reason: 'on $platform',
          );

        // Windows jumps to the previous wordwrapped line.
        case TargetPlatform.windows:
          expect(
            selection,
            equals(
              const TextSelection(
                baseOffset: 32,
                extentOffset: 84,
                affinity: TextAffinity.upstream,
              ),
            ),
            reason: 'on $platform',
          );
      }
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'shift + home/end keys to document boundary (Mac only)',
    (WidgetTester tester) async {
      const String testText =
          'Now is the time for all good people to come to the aid of their country. Now is the time for all good people to come to the aid of their country.';
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      late TextSelection selection;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? newCause) {
                  selection = newSelection;
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));
      expect(scrollable.controller!.offset, 0.0);

      // Move near the middle of the document.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);
      expect(selection, equals(const TextSelection.collapsed(offset: 32)));

      // Expand to the start of the document with the home key.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(scrollable.controller!.offset, 0.0);
      expect(selection, equals(const TextSelection(baseOffset: 32, extentOffset: 0)));

      // Expand to the end of the document with the end key.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      final double maxScrollExtent = scrollable.controller!.position.maxScrollExtent;
      expect(scrollable.controller!.offset, maxScrollExtent);
      expect(selection, equals(const TextSelection(baseOffset: 0, extentOffset: 145)));
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets(
    'control + home/end keys (Windows only)',
    (WidgetTester tester) async {
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump();

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(controller.selection, equals(const TextSelection.collapsed(offset: testText.length)));

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shortcutModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(controller.selection, equals(const TextSelection.collapsed(offset: 0)));
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.windows}),
  );

  testWidgets(
    'control + shift + home/end keys (Windows only)',
    (WidgetTester tester) async {
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump();

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shortcutModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(
        controller.selection,
        equals(const TextSelection(baseOffset: 0, extentOffset: testText.length)),
      );

      // Collapse the selection at the end.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowRight,
      ], targetPlatform: defaultTargetPlatform);
      await tester.pump();
      expect(controller.selection, equals(const TextSelection.collapsed(offset: testText.length)));

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shortcutModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      expect(
        controller.selection,
        equals(const TextSelection(baseOffset: testText.length, extentOffset: 0)),
      );
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.windows}),
  );

  testWidgets(
    'pageup/pagedown keys on Apple platforms',
    (WidgetTester tester) async {
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      const int lines = 2;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                minLines: lines,
                maxLines: lines,
                controller: controller,
                scrollController: scrollController,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      expect(controller.value.selection.isCollapsed, isTrue);
      expect(controller.value.selection.baseOffset, 0);
      expect(scrollController.position.pixels, 0.0);
      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      expect(scrollController.position.viewportDimension, lineHeight * lines);

      // Page Up does nothing at the top.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageUp,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, 0.0);

      // Page Down scrolls proportionally to the height of the viewport.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageDown,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, lineHeight * lines * 0.8);

      // Another Page Down reaches the bottom.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageDown,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, lineHeight * lines);

      // Page Up now scrolls back up proportionally to the height of the viewport.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageUp,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, lineHeight * lines - lineHeight * lines * 0.8);

      // Another Page Up reaches the top.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageUp,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, 0.0);
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'pageup/pagedown keys in a one line field on Apple platforms',
    (WidgetTester tester) async {
      controller.text = testText;
      controller.selection = const TextSelection(
        baseOffset: 0,
        extentOffset: 0,
        affinity: TextAffinity.upstream,
      );
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                minLines: 1,
                controller: controller,
                scrollController: scrollController,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      expect(controller.value.selection.isCollapsed, isTrue);
      expect(controller.value.selection.baseOffset, 0);
      expect(scrollController.position.pixels, 0.0);

      // Page Up scrolls to the end.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageUp,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, scrollController.position.maxScrollExtent);
      expect(controller.value.selection.isCollapsed, isTrue);
      expect(controller.value.selection.baseOffset, 0);

      // Return scroll to the start.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.home,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, 0.0);
      expect(controller.value.selection.isCollapsed, isTrue);
      expect(controller.value.selection.baseOffset, 0);

      // Page Down also scrolls to the end.
      await sendKeys(tester, <LogicalKeyboardKey>[
        LogicalKeyboardKey.pageDown,
      ], targetPlatform: defaultTargetPlatform);
      expect(scrollController.position.pixels, scrollController.position.maxScrollExtent);
      expect(controller.value.selection.isCollapsed, isTrue);
      expect(controller.value.selection.baseOffset, 0);
    },
    skip: kIsWeb, // [intended] on web these keys are handled by the browser.
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  // Regression test for https://github.com/flutter/flutter/issues/31287
  testWidgets(
    'text selection handle visibility',
    (WidgetTester tester) async {
      // Text with two separate words to select.
      controller.text = 'XXXXX          XXXXX';

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              child: EditableText(
                showSelectionHandles: true,
                controller: controller,
                focusNode: focusNode,
                style: Typography.material2018(platform: TargetPlatform.iOS).black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: cupertinoTextSelectionControls,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final RenderEditable renderEditable = state.renderEditable;
      final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));

      bool expectedLeftVisibleBefore = false;
      bool expectedRightVisibleBefore = false;

      Future<void> verifyVisibility(
        HandlePositionInViewport leftPosition,
        bool expectedLeftVisible,
        HandlePositionInViewport rightPosition,
        bool expectedRightVisible,
      ) async {
        await tester.pump();

        // Check the signal from RenderEditable about whether they're within the
        // viewport.

        expect(renderEditable.selectionStartInViewport.value, equals(expectedLeftVisible));
        expect(renderEditable.selectionEndInViewport.value, equals(expectedRightVisible));

        // Check that the animations are functional and going in the right
        // direction.

        final List<FadeTransition> transitions =
            find
                .byType(FadeTransition)
                .evaluate()
                .map((Element e) => e.widget)
                .cast<FadeTransition>()
                .toList();
        final FadeTransition left = transitions[0];
        final FadeTransition right = transitions[1];

        if (expectedLeftVisibleBefore) {
          expect(left.opacity.value, equals(1.0));
        }
        if (expectedRightVisibleBefore) {
          expect(right.opacity.value, equals(1.0));
        }

        await tester.pump(SelectionOverlay.fadeDuration ~/ 2);

        if (expectedLeftVisible != expectedLeftVisibleBefore) {
          expect(left.opacity.value, equals(0.5));
        }
        if (expectedRightVisible != expectedRightVisibleBefore) {
          expect(right.opacity.value, equals(0.5));
        }

        await tester.pump(SelectionOverlay.fadeDuration ~/ 2);

        if (expectedLeftVisible) {
          expect(left.opacity.value, equals(1.0));
        }
        if (expectedRightVisible) {
          expect(right.opacity.value, equals(1.0));
        }

        expectedLeftVisibleBefore = expectedLeftVisible;
        expectedRightVisibleBefore = expectedRightVisible;

        // Check that the handles' positions are correct.

        final List<RenderBox> handles = List<RenderBox>.from(
          tester.renderObjectList<RenderBox>(
            find.descendant(
              of: find.byType(CompositedTransformFollower),
              matching: find.byType(Padding),
            ),
          ),
        );

        final Size viewport = renderEditable.size;

        void testPosition(double pos, HandlePositionInViewport expected) {
          switch (expected) {
            case HandlePositionInViewport.leftEdge:
              expect(
                pos,
                inExclusiveRange(0 - kMinInteractiveDimension, 0 + kMinInteractiveDimension),
              );
            case HandlePositionInViewport.rightEdge:
              expect(
                pos,
                inExclusiveRange(
                  viewport.width - kMinInteractiveDimension,
                  viewport.width + kMinInteractiveDimension,
                ),
              );
            case HandlePositionInViewport.within:
              expect(
                pos,
                inExclusiveRange(
                  0 - kMinInteractiveDimension,
                  viewport.width + kMinInteractiveDimension,
                ),
              );
          }
        }

        expect(state.selectionOverlay!.handlesAreVisible, isTrue);
        testPosition(handles[0].localToGlobal(Offset.zero).dx, leftPosition);
        testPosition(handles[1].localToGlobal(Offset.zero).dx, rightPosition);
      }

      // Select the first word. Both handles should be visible.
      await tester.tapAt(const Offset(20, 10));
      renderEditable.selectWord(cause: SelectionChangedCause.longPress);
      await tester.pump();
      await verifyVisibility(
        HandlePositionInViewport.leftEdge,
        true,
        HandlePositionInViewport.within,
        true,
      );

      // Drag the text slightly so the first word is partially visible. Only the
      // right handle should be visible.
      scrollable.controller!.jumpTo(20.0);
      await verifyVisibility(
        HandlePositionInViewport.leftEdge,
        false,
        HandlePositionInViewport.within,
        true,
      );

      // Drag the text all the way to the left so the first word is not visible at
      // all (and the second word is fully visible). Both handles should be
      // invisible now.
      scrollable.controller!.jumpTo(200.0);
      await verifyVisibility(
        HandlePositionInViewport.leftEdge,
        false,
        HandlePositionInViewport.leftEdge,
        false,
      );

      // Tap to unselect.
      await tester.tap(find.byType(EditableText));
      await tester.pump();

      // Now that the second word has been dragged fully into view, select it.
      await tester.tapAt(const Offset(80, 10));
      renderEditable.selectWord(cause: SelectionChangedCause.longPress);
      await tester.pump();
      await verifyVisibility(
        HandlePositionInViewport.within,
        true,
        HandlePositionInViewport.within,
        true,
      );

      // Drag the text slightly to the right. Only the left handle should be
      // visible.
      scrollable.controller!.jumpTo(150);
      await verifyVisibility(
        HandlePositionInViewport.within,
        true,
        HandlePositionInViewport.rightEdge,
        false,
      );

      // Drag the text all the way to the right, so the second word is not visible
      // at all. Again, both handles should be invisible.
      scrollable.controller!.jumpTo(0);
      await verifyVisibility(
        HandlePositionInViewport.rightEdge,
        false,
        HandlePositionInViewport.rightEdge,
        false,
      );
    },
    // On web, we don't show the Flutter toolbar and instead rely on the browser
    // toolbar. Until we change that, this test should remain skipped.
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets("scrolling doesn't bounce", (WidgetTester tester) async {
    // 3 lines of text, where the last line overflows and requires scrolling.
    controller.text = 'XXXXX\nXXXXX\nXXXXX';

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 100,
            child: EditableText(
              showSelectionHandles: true,
              maxLines: 2,
              controller: controller,
              focusNode: focusNode,
              style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              selectionControls: materialTextSelectionControls,
              keyboardType: TextInputType.text,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable renderEditable = state.renderEditable;
    final Scrollable scrollable = tester.widget<Scrollable>(find.byType(Scrollable));

    expect(scrollable.controller!.position.viewportDimension, equals(28));
    expect(scrollable.controller!.position.pixels, equals(0));

    expect(renderEditable.maxScrollExtent, equals(14));

    scrollable.controller!.jumpTo(20.0);
    await tester.pump();
    expect(scrollable.controller!.position.pixels, equals(20));

    state.bringIntoView(const TextPosition(offset: 0));
    await tester.pump();
    expect(scrollable.controller!.position.pixels, equals(0));

    state.bringIntoView(const TextPosition(offset: 13));
    await tester.pump();
    expect(scrollable.controller!.position.pixels, equals(14));
    expect(scrollable.controller!.position.pixels, equals(renderEditable.maxScrollExtent));
  });

  testWidgets(
    'Deleting text with keyboard backspace does not trigger assertion on CupertinoPageRoute',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/153003.
      controller.text = testText * 20;
      final ScrollController editableScrollController = ScrollController();
      addTearDown(editableScrollController.dispose);
      final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

      await tester.pumpWidget(
        MaterialApp(
          navigatorKey: navigatorKey,
          home: Center(
            child: TextButton(
              onPressed: () async {
                if (navigatorKey.currentState == null) {
                  return;
                }
                await navigatorKey.currentState!.push(
                  CupertinoPageRoute<void>(
                    settings: const RouteSettings(name: '/TestCupertinoRoute'),
                    builder: (BuildContext innerContext) {
                      return Align(
                        alignment: Alignment.topLeft,
                        child: SizedBox(
                          width: 200,
                          height: 200,
                          child: EditableText(
                            maxLines: null,
                            controller: controller,
                            scrollController: editableScrollController,
                            focusNode: focusNode,
                            style: textStyle,
                            cursorColor: Colors.blue,
                            backgroundCursorColor: Colors.grey,
                            showSelectionHandles: true,
                            selectionControls: materialTextSelectionControls,
                            selectionColor: Colors.lightBlueAccent,
                          ),
                        ),
                      );
                    },
                  ),
                );
              },
              child: const Text('Push Route'),
            ),
          ),
        ),
      );

      // Push cupertino route.
      await tester.tap(find.text('Push Route'));
      await tester.pumpAndSettle();

      expect(editableScrollController.offset, 0);

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.bringIntoView(TextPosition(offset: controller.text.length));

      await tester.pumpAndSettle();
      expect(editableScrollController.offset, editableScrollController.position.maxScrollExtent);

      // Select a word near the end of the text. And show the toolbar.
      await tester.tapAt(textOffsetToPosition(tester, controller.text.length - 10));
      state.renderEditable.selectWord(cause: SelectionChangedCause.longPress);
      expect(state.showToolbar(), true);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 1426, extentOffset: 1431));
      expect(state.selectionOverlay, isNotNull);
      expect(state.selectionOverlay!.toolbarIsVisible, true);

      // Send backspace key event to delete the selected word. This will cause
      // the EditableText to scroll the new position into view, but this
      // should not cause an exception, and the toolbar should no longer be visible.
      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 1426));
      expect(tester.takeException(), isNull);
      expect(state.selectionOverlay, isNotNull);
      expect(state.selectionOverlay!.toolbarIsVisible, false);
      // On web, we don't show the Flutter toolbar and instead rely on the browser
      // toolbar. Until we change that, this test should remain skipped.
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets('bringIntoView brings the caret into view when in a viewport', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/55547.
    controller.text = testText * 20;
    final ScrollController editableScrollController = ScrollController();
    addTearDown(editableScrollController.dispose);
    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200,
            height: 200,
            child: SingleChildScrollView(
              controller: outerController,
              child: EditableText(
                maxLines: null,
                controller: controller,
                scrollController: editableScrollController,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
              ),
            ),
          ),
        ),
      ),
    );

    expect(outerController.offset, 0);
    expect(editableScrollController.offset, 0);

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.bringIntoView(TextPosition(offset: controller.text.length));

    await tester.pumpAndSettle();
    // The SingleChildScrollView is scrolled instead of the EditableText to
    // reveal the caret.
    expect(outerController.offset, outerController.position.maxScrollExtent);
    expect(editableScrollController.offset, 0);
  });

  testWidgets('bringIntoView does nothing if the physics prohibits implicit scrolling', (
    WidgetTester tester,
  ) async {
    controller.text = testText * 20;
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    Future<void> buildWithPhysics({ScrollPhysics? physics}) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 200,
              height: 200,
              child: EditableText(
                maxLines: null,
                controller: controller,
                scrollController: scrollController,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                scrollPhysics: physics,
              ),
            ),
          ),
        ),
      );
    }

    await buildWithPhysics();
    expect(scrollController.offset, 0);

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.bringIntoView(TextPosition(offset: controller.text.length));

    await tester.pumpAndSettle();
    // Scrolled to the maxScrollExtent to reveal to caret.
    expect(scrollController.offset, scrollController.position.maxScrollExtent);

    scrollController.jumpTo(0);
    await buildWithPhysics(physics: const NoImplicitScrollPhysics());
    expect(scrollController.offset, 0);

    state.bringIntoView(TextPosition(offset: controller.text.length));

    await tester.pumpAndSettle();
    expect(scrollController.offset, 0);
  });

  testWidgets('can change scroll controller', (WidgetTester tester) async {
    controller.text = 'A' * 1000;
    final _TestScrollController scrollController1 = _TestScrollController();
    addTearDown(scrollController1.dispose);
    final _TestScrollController scrollController2 = _TestScrollController();
    addTearDown(scrollController2.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          scrollController: scrollController1,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.widget.scrollController, scrollController1);

    // Change scrollController to controller 2.
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          scrollController: scrollController2,
        ),
      ),
    );

    expect(state.widget.scrollController, scrollController2);

    // Changing scrollController to null.
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
        ),
      ),
    );

    expect(state.widget.scrollController, isNull);

    // Change scrollController to back controller 2.
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          scrollController: scrollController2,
        ),
      ),
    );

    expect(state.widget.scrollController, scrollController2);
  });

  testWidgets('getLocalRectForCaret does not throw when it sees an infinite point', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: SkipPainting(
          child: Transform(
            transform: Matrix4.zero(),
            child: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final Rect rect = state.renderEditable.getLocalRectForCaret(const TextPosition(offset: 0));
    expect(rect.isFinite, true);
    expect(tester.takeException(), isNull);
  });

  testWidgets('obscured multiline fields throw an exception', (WidgetTester tester) async {
    expect(() {
      EditableText(
        backgroundCursorColor: cursorColor,
        controller: controller,
        cursorColor: cursorColor,
        focusNode: focusNode,
        obscureText: true,
        style: textStyle,
      );
    }, returnsNormally);
    expect(() {
      EditableText(
        backgroundCursorColor: cursorColor,
        controller: controller,
        cursorColor: cursorColor,
        focusNode: focusNode,
        maxLines: 2,
        obscureText: true,
        style: textStyle,
      );
    }, throwsAssertionError);
  });

  group('batch editing', () {
    Widget buildWidget() {
      return MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
          ),
        ),
      );
    }

    testWidgets('batch editing works', (WidgetTester tester) async {
      await tester.pumpWidget(buildWidget());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.updateEditingValue(const TextEditingValue(text: 'remote value'));
      tester.testTextInput.log.clear();

      state.beginBatchEdit();

      controller.text = 'new change 1';
      expect(state.currentTextEditingValue.text, 'new change 1');
      expect(tester.testTextInput.log, isEmpty);

      // Nesting.
      state.beginBatchEdit();
      controller.text = 'new change 2';
      expect(state.currentTextEditingValue.text, 'new change 2');
      expect(tester.testTextInput.log, isEmpty);

      // End the innermost batch edit. Not yet.
      state.endBatchEdit();
      expect(tester.testTextInput.log, isEmpty);

      controller.text = 'new change 3';
      expect(state.currentTextEditingValue.text, 'new change 3');
      expect(tester.testTextInput.log, isEmpty);

      // Finish the outermost batch edit.
      state.endBatchEdit();
      expect(tester.testTextInput.log, hasLength(1));
      expect(
        tester.testTextInput.log,
        contains(
          matchesMethodCall(
            'TextInput.setEditingState',
            args: containsPair('text', 'new change 3'),
          ),
        ),
      );
    });

    testWidgets('batch edits need to be nested properly', (WidgetTester tester) async {
      await tester.pumpWidget(buildWidget());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.updateEditingValue(const TextEditingValue(text: 'remote value'));
      tester.testTextInput.log.clear();

      String? errorString;
      try {
        state.endBatchEdit();
      } catch (e) {
        errorString = e.toString();
      }

      expect(errorString, contains('Unbalanced call to endBatchEdit'));
    });

    testWidgets(
      'catch unfinished batch edits on disposal',
      experimentalLeakTesting:
          LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
      (WidgetTester tester) async {
        await tester.pumpWidget(buildWidget());

        // Connect.
        await tester.showKeyboard(find.byType(EditableText));

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.updateEditingValue(const TextEditingValue(text: 'remote value'));
        tester.testTextInput.log.clear();

        state.beginBatchEdit();
        expect(tester.takeException(), isNull);

        await tester.pumpWidget(Container());
        expect(tester.takeException(), isAssertionError);
      },
    );
  });

  group('EditableText does not send editing values more than once', () {
    Widget boilerplate() {
      final EditableText editableText = EditableText(
        showSelectionHandles: true,
        maxLines: 2,
        controller: controller,
        focusNode: focusNode,
        cursorColor: Colors.red,
        backgroundCursorColor: Colors.blue,
        style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
        keyboardType: TextInputType.text,
        inputFormatters: <TextInputFormatter>[LengthLimitingTextInputFormatter(6)],
        onChanged: (String s) {
          controller.value = collapsedAtEnd('${controller.text} onChanged');
        },
      );

      controller.addListener(() {
        if (!controller.text.endsWith('listener')) {
          controller.value = collapsedAtEnd('${controller.text} listener');
        }
      });

      return MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(textDirection: TextDirection.ltr, child: editableText),
      );
    }

    testWidgets('input from text input plugin', (WidgetTester tester) async {
      controller.text = testText;
      await tester.pumpWidget(boilerplate());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));
      tester.testTextInput.log.clear();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.updateEditingValue(collapsedAtEnd('remoteremoteremote'));

      // Apply in order: length formatter -> listener -> onChanged -> listener.
      const String expectedText = 'remote listener onChanged listener';
      expect(controller.text, expectedText);
      final List<TextEditingValue> updates = tester.testTextInput.log
          .where((MethodCall call) => call.method == 'TextInput.setEditingState')
          .map(
            (MethodCall call) => TextEditingValue.fromJSON(call.arguments as Map<String, dynamic>),
          )
          .toList(growable: false);

      expect(updates, <TextEditingValue>[collapsedAtEnd(expectedText)]);

      tester.testTextInput.log.clear();

      // If by coincidence the text input plugin sends the same value back,
      // do nothing.
      state.updateEditingValue(collapsedAtEnd(expectedText));
      expect(controller.text, 'remote listener onChanged listener');
      expect(tester.testTextInput.log, isEmpty);
    });

    testWidgets('input from text selection menu', (WidgetTester tester) async {
      controller.text = testText;
      await tester.pumpWidget(boilerplate());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));
      tester.testTextInput.log.clear();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.userUpdateTextEditingValue(
        collapsedAtEnd('remoteremoteremote'),
        SelectionChangedCause.keyboard,
      );

      final List<TextEditingValue> updates = tester.testTextInput.log
          .where((MethodCall call) => call.method == 'TextInput.setEditingState')
          .map(
            (MethodCall call) => TextEditingValue.fromJSON(call.arguments as Map<String, dynamic>),
          )
          .toList(growable: false);

      const String expectedText = 'remote listener onChanged listener';
      expect(updates, <TextEditingValue>[collapsedAtEnd(expectedText)]);

      tester.testTextInput.log.clear();
    });

    testWidgets('input from controller', (WidgetTester tester) async {
      controller.text = testText;
      await tester.pumpWidget(boilerplate());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));
      tester.testTextInput.log.clear();

      controller.text = 'remoteremoteremote';
      final List<TextEditingValue> updates = tester.testTextInput.log
          .where((MethodCall call) => call.method == 'TextInput.setEditingState')
          .map(
            (MethodCall call) => TextEditingValue.fromJSON(call.arguments as Map<String, dynamic>),
          )
          .toList(growable: false);

      expect(updates, <TextEditingValue>[collapsedAtEnd('remoteremoteremote listener')]);
    });

    testWidgets('input from changing controller', (WidgetTester tester) async {
      Widget build({TextEditingController? textEditingController}) {
        return MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: EditableText(
              showSelectionHandles: true,
              maxLines: 2,
              controller: textEditingController ?? controller,
              focusNode: focusNode,
              cursorColor: Colors.red,
              backgroundCursorColor: Colors.blue,
              style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
              keyboardType: TextInputType.text,
              inputFormatters: <TextInputFormatter>[LengthLimitingTextInputFormatter(6)],
            ),
          ),
        );
      }

      await tester.pumpWidget(build());

      // Connect.
      await tester.showKeyboard(find.byType(EditableText));
      tester.testTextInput.log.clear();
      final TextEditingController controller1 = TextEditingController(text: 'new text');
      addTearDown(controller1.dispose);
      await tester.pumpWidget(build(textEditingController: controller1));

      List<TextEditingValue> updates = tester.testTextInput.log
          .where((MethodCall call) => call.method == 'TextInput.setEditingState')
          .map(
            (MethodCall call) => TextEditingValue.fromJSON(call.arguments as Map<String, dynamic>),
          )
          .toList(growable: false);

      expect(updates, const <TextEditingValue>[TextEditingValue(text: 'new text')]);

      tester.testTextInput.log.clear();
      final TextEditingController controller2 = TextEditingController(text: 'new new text');
      addTearDown(controller2.dispose);
      await tester.pumpWidget(build(textEditingController: controller2));

      updates = tester.testTextInput.log
          .where((MethodCall call) => call.method == 'TextInput.setEditingState')
          .map(
            (MethodCall call) => TextEditingValue.fromJSON(call.arguments as Map<String, dynamic>),
          )
          .toList(growable: false);

      expect(updates, const <TextEditingValue>[TextEditingValue(text: 'new new text')]);
    });
  });

  testWidgets('input imm channel calls are ordered correctly', (WidgetTester tester) async {
    controller.text = 'flutter is the best!';
    final EditableText et = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller,
      focusNode: focusNode,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Align(alignment: Alignment.topLeft, child: SizedBox(width: 100, child: et)),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    // TextInput.show should be after TextInput.setEditingState.
    // On Android setEditingState triggers an IME restart which may prevent
    // the keyboard from showing if the show keyboard request comes before the
    // restart.
    // See: https://github.com/flutter/flutter/issues/68571.
    final List<String> logOrder = <String>[
      'TextInput.setClient',
      'TextInput.setEditableSizeAndTransform',
      'TextInput.setMarkedTextRect',
      'TextInput.setStyle',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.requestAutofill',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.setCaretRect',
    ];
    expect(tester.testTextInput.log.map((MethodCall m) => m.method), logOrder);
  });

  testWidgets('keyboard is requested after setEditingState after switching to a new text field', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/68571.
    final TextEditingController controller1 = TextEditingController();
    addTearDown(controller1.dispose);
    final FocusNode focusNode1 = FocusNode();
    addTearDown(focusNode1.dispose);
    final EditableText editableText1 = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller1,
      focusNode: focusNode1,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    final TextEditingController controller2 = TextEditingController();
    addTearDown(controller2.dispose);
    final FocusNode focusNode2 = FocusNode();
    addTearDown(focusNode2.dispose);
    final EditableText editableText2 = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller2,
      focusNode: focusNode2,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    await tester.pumpWidget(
      MaterialApp(home: Center(child: Column(children: <Widget>[editableText1, editableText2]))),
    );

    await tester.tap(find.byWidget(editableText1));
    await tester.pumpAndSettle();

    tester.testTextInput.log.clear();
    await tester.tap(find.byWidget(editableText2));
    await tester.pumpAndSettle();

    // Send TextInput.show after TextInput.setEditingState. Otherwise
    // some Android keyboards ignore the "show keyboard" request, as the
    // Android text input plugin restarts the input method when setEditingState
    // is sent by the framework.
    final List<String> logOrder = <String>[
      'TextInput.clearClient',
      'TextInput.setClient',
      'TextInput.setEditableSizeAndTransform',
      'TextInput.setMarkedTextRect',
      'TextInput.setStyle',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.requestAutofill',
      'TextInput.setCaretRect',
    ];
    expect(tester.testTextInput.log.map((MethodCall m) => m.method), logOrder);
  });

  testWidgets('Autofill does not request focus', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/91354 .
    final TextEditingController controller1 = TextEditingController();
    addTearDown(controller1.dispose);
    final FocusNode focusNode1 = FocusNode();
    addTearDown(focusNode1.dispose);
    final EditableText editableText1 = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller1,
      focusNode: focusNode1,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    final TextEditingController controller2 = TextEditingController();
    addTearDown(controller2.dispose);
    final FocusNode focusNode2 = FocusNode();
    addTearDown(focusNode2.dispose);
    final EditableText editableText2 = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller2,
      focusNode: focusNode2,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    await tester.pumpWidget(
      MaterialApp(home: Center(child: Column(children: <Widget>[editableText1, editableText2]))),
    );

    // editableText1 has the focus.
    await tester.tap(find.byWidget(editableText1));
    await tester.pumpAndSettle();

    final EditableTextState state2 = tester.state<EditableTextState>(find.byWidget(editableText2));
    // Update editableText2 when it's not focused. It should not request focus.
    state2.updateEditingValue(
      const TextEditingValue(text: 'password', selection: TextSelection.collapsed(offset: 8)),
    );
    await tester.pumpAndSettle();

    expect(focusNode1.hasFocus, isTrue);
    expect(focusNode2.hasFocus, isFalse);
  });

  testWidgets('setEditingState is not called when text changes', (WidgetTester tester) async {
    // We shouldn't get a message here because this change is owned by the platform side.
    controller.text = 'flutter is the best!';
    final EditableText et = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller,
      focusNode: focusNode,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Align(alignment: Alignment.topLeft, child: SizedBox(width: 100, child: et)),
      ),
    );

    await tester.enterText(find.byType(EditableText), '...');

    final List<String> logOrder = <String>[
      'TextInput.setClient',
      'TextInput.setEditableSizeAndTransform',
      'TextInput.setMarkedTextRect',
      'TextInput.setStyle',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.requestAutofill',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.setCaretRect',
    ];
    expect(tester.testTextInput.log.map((MethodCall m) => m.method), logOrder);
    expect(tester.testTextInput.editingState!['text'], 'flutter is the best!');
  });

  testWidgets('setEditingState is called when text changes on controller', (
    WidgetTester tester,
  ) async {
    // We should get a message here because this change is owned by the framework side.
    controller.text = 'flutter is the best!';
    final EditableText et = EditableText(
      showSelectionHandles: true,
      maxLines: 2,
      controller: controller,
      focusNode: focusNode,
      cursorColor: Colors.red,
      backgroundCursorColor: Colors.blue,
      style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
      keyboardType: TextInputType.text,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Align(alignment: Alignment.topLeft, child: SizedBox(width: 100, child: et)),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    controller.value = collapsedAtEnd('${controller.text}...');
    await tester.idle();

    final List<String> logOrder = <String>[
      'TextInput.setClient',
      'TextInput.setEditableSizeAndTransform',
      'TextInput.setMarkedTextRect',
      'TextInput.setStyle',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.requestAutofill',
      'TextInput.setEditingState',
      'TextInput.show',
      'TextInput.setCaretRect',
      'TextInput.setEditingState',
    ];

    expect(tester.testTextInput.log.map((MethodCall m) => m.method), logOrder);
    expect(tester.testTextInput.editingState!['text'], 'flutter is the best!...');
  });

  testWidgets('Synchronous test of local and remote editing values', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/65059
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });
    final TextInputFormatter formatter = TextInputFormatter.withFunction((
      TextEditingValue oldValue,
      TextEditingValue newValue,
    ) {
      if (newValue.text == 'I will be modified by the formatter.') {
        newValue = collapsedAtEnd('Flutter is the best!');
      }
      return newValue;
    });
    late StateSetter setState;

    Widget builder() {
      return StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          setState = setter;
          return MaterialApp(
            home: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Center(
                  child: Material(
                    child: EditableText(
                      controller: controller,
                      focusNode: focusNode,
                      style: textStyle,
                      cursorColor: Colors.red,
                      backgroundCursorColor: Colors.red,
                      keyboardType: TextInputType.multiline,
                      inputFormatters: <TextInputFormatter>[formatter],
                      onChanged: (String value) {},
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    await tester.pumpWidget(builder());
    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pump();

    log.clear();

    final EditableTextState state = tester.firstState(find.byType(EditableText));
    // setEditingState is not called when only the remote changes
    state.updateEditingValue(TextEditingValue(text: 'a', selection: controller.selection));

    expect(log.length, 0);

    // setEditingState is called when remote value modified by the formatter.
    state.updateEditingValue(
      TextEditingValue(
        text: 'I will be modified by the formatter.',
        selection: controller.selection,
      ),
    );
    expect(log.length, 1);
    MethodCall methodCall = log[0];
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditingState',
        arguments: <String, dynamic>{
          'text': 'Flutter is the best!',
          'selectionBase': 20,
          'selectionExtent': 20,
          'selectionAffinity': 'TextAffinity.downstream',
          'selectionIsDirectional': false,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ),
    );

    log.clear();

    // setEditingState is called when the [controller.value] is modified by local.
    String text = 'I love flutter!';
    setState(() {
      controller.value = collapsedAtEnd(text);
    });
    expect(log.length, 1);
    methodCall = log[0];
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditingState',
        arguments: <String, dynamic>{
          'text': 'I love flutter!',
          'selectionBase': text.length,
          'selectionExtent': text.length,
          'selectionAffinity': 'TextAffinity.downstream',
          'selectionIsDirectional': false,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ),
    );

    log.clear();

    // Currently `_receivedRemoteTextEditingValue` equals 'I will be modified by the formatter.',
    // setEditingState will be called when set the [controller.value] to `_receivedRemoteTextEditingValue` by local.
    text = 'I will be modified by the formatter.';
    setState(() {
      controller.value = collapsedAtEnd(text);
    });
    expect(log.length, 1);
    methodCall = log[0];
    expect(
      methodCall,
      isMethodCall(
        'TextInput.setEditingState',
        arguments: <String, dynamic>{
          'text': 'I will be modified by the formatter.',
          'selectionBase': text.length,
          'selectionExtent': text.length,
          'selectionAffinity': 'TextAffinity.downstream',
          'selectionIsDirectional': false,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ),
    );
  });

  testWidgets('Send text input state to engine when the input formatter rejects user input', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/67828
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });
    final TextInputFormatter formatter = TextInputFormatter.withFunction((
      TextEditingValue oldValue,
      TextEditingValue newValue,
    ) {
      return collapsedAtEnd('Flutter is the best!');
    });

    Widget builder() {
      return StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          return MaterialApp(
            home: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Center(
                  child: Material(
                    child: EditableText(
                      controller: controller,
                      focusNode: focusNode,
                      style: textStyle,
                      cursorColor: Colors.red,
                      backgroundCursorColor: Colors.red,
                      keyboardType: TextInputType.multiline,
                      inputFormatters: <TextInputFormatter>[formatter],
                      onChanged: (String value) {},
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    await tester.pumpWidget(builder());
    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pump();

    log.clear();

    final EditableTextState state = tester.firstState(find.byType(EditableText));

    // setEditingState is called when remote value modified by the formatter.
    state.updateEditingValue(collapsedAtEnd('I will be modified by the formatter.'));
    expect(
      log,
      contains(
        matchesMethodCall(
          'TextInput.setEditingState',
          args: allOf(containsPair('text', 'Flutter is the best!')),
        ),
      ),
    );

    log.clear();

    state.updateEditingValue(collapsedAtEnd('I will be modified by the formatter.'));
    expect(
      log,
      contains(
        matchesMethodCall(
          'TextInput.setEditingState',
          args: allOf(containsPair('text', 'Flutter is the best!')),
        ),
      ),
    );
  });

  testWidgets('Repeatedly receiving [TextEditingValue] will not trigger a keyboard request', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/66036
    final List<MethodCall> log = <MethodCall>[];
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    Widget builder() {
      return StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          return MaterialApp(
            home: MediaQuery(
              data: const MediaQueryData(),
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Center(
                  child: Material(
                    child: EditableText(
                      controller: controller,
                      focusNode: focusNode,
                      style: textStyle,
                      cursorColor: Colors.red,
                      backgroundCursorColor: Colors.red,
                      keyboardType: TextInputType.multiline,
                      onChanged: (String value) {},
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    await tester.pumpWidget(builder());
    await tester.tap(find.byType(EditableText));
    await tester.pump();

    // The keyboard is shown after tap the EditableText.
    expect(focusNode.hasFocus, true);

    log.clear();

    final EditableTextState state = tester.firstState(find.byType(EditableText));

    state.updateEditingValue(TextEditingValue(text: 'a', selection: controller.selection));
    await tester.pump();

    // Nothing called when only the remote changes.
    expect(log.length, 0);

    // Hide the keyboard.
    focusNode.unfocus();
    await tester.pump();

    expect(log.length, 2);
    MethodCall methodCall = log[0];
    // Close the InputConnection.
    expect(methodCall, isMethodCall('TextInput.clearClient', arguments: null));
    methodCall = log[1];
    expect(methodCall, isMethodCall('TextInput.hide', arguments: null));
    // The keyboard loses focus.
    expect(focusNode.hasFocus, false);

    log.clear();

    // Send repeat value from the engine.
    state.updateEditingValue(TextEditingValue(text: 'a', selection: controller.selection));
    await tester.pump();

    // Nothing called when only the remote changes.
    expect(log.length, 0);
    // The keyboard is not be requested after a repeat value from the engine.
    expect(focusNode.hasFocus, false);
  });

  group('TextEditingController', () {
    testWidgets('TextEditingController.text set to empty string clears field', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: Material(
                  child: EditableText(
                    controller: controller,
                    focusNode: focusNode,
                    style: textStyle,
                    cursorColor: Colors.red,
                    backgroundCursorColor: Colors.red,
                    keyboardType: TextInputType.multiline,
                    onChanged: (String value) {},
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      controller.text = '...';
      await tester.pump();
      expect(find.text('...'), findsOneWidget);

      controller.text = '';
      await tester.pump();
      expect(find.text('...'), findsNothing);
    });

    testWidgets('TextEditingController.clear() behavior test', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/66316
      final List<MethodCall> log = <MethodCall>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });

      Widget builder() {
        return StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            return MaterialApp(
              home: MediaQuery(
                data: const MediaQueryData(),
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: Center(
                    child: Material(
                      child: EditableText(
                        controller: controller,
                        focusNode: focusNode,
                        style: textStyle,
                        cursorColor: Colors.red,
                        backgroundCursorColor: Colors.red,
                        keyboardType: TextInputType.multiline,
                        onChanged: (String value) {},
                      ),
                    ),
                  ),
                ),
              ),
            );
          },
        );
      }

      await tester.pumpWidget(builder());
      await tester.tap(find.byType(EditableText));
      await tester.pump();

      // The keyboard is shown after tap the EditableText.
      expect(focusNode.hasFocus, true);

      log.clear();

      final EditableTextState state = tester.firstState(find.byType(EditableText));

      state.updateEditingValue(TextEditingValue(text: 'a', selection: controller.selection));
      await tester.pump();

      // Nothing called when only the remote changes.
      expect(log, isEmpty);

      controller.clear();

      expect(log.length, 1);
      expect(
        log[0],
        isMethodCall(
          'TextInput.setEditingState',
          arguments: <String, dynamic>{
            'text': '',
            'selectionBase': 0,
            'selectionExtent': 0,
            'selectionAffinity': 'TextAffinity.downstream',
            'selectionIsDirectional': false,
            'composingBase': -1,
            'composingExtent': -1,
          },
        ),
      );
    });

    testWidgets('TextEditingController.buildTextSpan receives build context', (
      WidgetTester tester,
    ) async {
      final _AccentColorTextEditingController controller = _AccentColorTextEditingController('a');
      addTearDown(controller.dispose);
      const Color color = Color.fromARGB(255, 1, 2, 3);
      final ThemeData lightTheme = ThemeData.light();
      await tester.pumpWidget(
        MaterialApp(
          theme: lightTheme.copyWith(
            colorScheme: lightTheme.colorScheme.copyWith(secondary: color),
          ),
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: Typography.material2018().black.titleMedium!,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      );

      final RenderEditable renderEditable = findRenderEditable(tester);
      final TextSpan textSpan = renderEditable.text! as TextSpan;
      expect(textSpan.style!.color, color);
    });

    testWidgets('controller listener changes value', (WidgetTester tester) async {
      const double maxValue = 5.5555;

      controller.addListener(() {
        final double value = double.tryParse(controller.text.trim()) ?? .0;
        if (value > maxValue) {
          controller.text = maxValue.toString();
          controller.selection = TextSelection.fromPosition(
            TextPosition(offset: maxValue.toString().length),
          );
        }
      });
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      state.updateEditingValue(
        const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
      );
      await tester.pump();
      state.updateEditingValue(
        const TextEditingValue(text: '12', selection: TextSelection.collapsed(offset: 2)),
      );
      await tester.pump();

      expect(controller.text, '5.5555');
      expect(controller.selection.baseOffset, 6);
      expect(controller.selection.extentOffset, 6);
    });
  });

  testWidgets('autofocus:true on first frame does not throw', (WidgetTester tester) async {
    controller.text = testText;
    controller.selection = const TextSelection(
      baseOffset: 0,
      extentOffset: 0,
      affinity: TextAffinity.upstream,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          maxLines: 10,
          controller: controller,
          showSelectionHandles: true,
          autofocus: true,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          selectionControls: materialTextSelectionControls,
          keyboardType: TextInputType.text,
          textAlign: TextAlign.right,
        ),
      ),
    );

    await tester.pumpAndSettle(); // Wait for autofocus to take effect.

    final dynamic exception = tester.takeException();
    expect(exception, isNull);
  });

  testWidgets('updateEditingValue filters multiple calls from formatter', (
    WidgetTester tester,
  ) async {
    final MockTextFormatter formatter = MockTextFormatter();
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              inputFormatters: <TextInputFormatter>[formatter],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = '';
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(tester.testTextInput.editingState!['text'], equals(''));
    expect(state.wantKeepAlive, true);

    state.updateEditingValue(TextEditingValue.empty);
    state.updateEditingValue(const TextEditingValue(text: 'a'));
    state.updateEditingValue(const TextEditingValue(text: 'aa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaa'));
    state.updateEditingValue(const TextEditingValue(text: 'aa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaaa'));
    state.updateEditingValue(const TextEditingValue(text: 'aa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaaaaaa'));
    state.updateEditingValue(const TextEditingValue(text: 'aa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaaaaaaaa'));
    state.updateEditingValue(const TextEditingValue(text: 'aaaaaaaaa')); // Skipped

    const List<String> referenceLog = <String>[
      '[1]: , a',
      '[1]: normal aa',
      '[2]: a, aa',
      '[2]: normal aaaa',
      '[3]: aa, aaa',
      '[3]: normal aaaaaa',
      '[4]: aaa, aa',
      '[4]: deleting aa',
      '[5]: aa, aaa',
      '[5]: normal aaaaaaaaaa',
      '[6]: aaa, aaaa',
      '[6]: normal aaaaaaaaaaaa',
      '[7]: aaaa, aa',
      '[7]: deleting aaaaa',
      '[8]: aa, aaaaaaa',
      '[8]: normal aaaaaaaaaaaaaaaa',
      '[9]: aaaaaaa, aa',
      '[9]: deleting aaaaaaa',
      '[10]: aa, aaaaaaaaa',
      '[10]: normal aaaaaaaaaaaaaaaaaaaa',
    ];

    expect(formatter.log, referenceLog);
  });

  testWidgets('formatter logic handles repeat filtering', (WidgetTester tester) async {
    final MockTextFormatter formatter = MockTextFormatter();
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              inputFormatters: <TextInputFormatter>[formatter],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = '';
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(tester.testTextInput.editingState!['text'], equals(''));
    expect(state.wantKeepAlive, true);

    // We no longer perform full repeat filtering in framework, it is now left
    // to the engine to prevent repeat calls from being sent in the first place.
    // Engine preventing repeats is far more reliable and avoids many of the ambiguous
    // filtering we performed before.
    expect(formatter.formatCallCount, 0);
    state.updateEditingValue(const TextEditingValue(text: '01'));
    expect(formatter.formatCallCount, 1);
    state.updateEditingValue(const TextEditingValue(text: '012'));
    expect(formatter.formatCallCount, 2);
    state.updateEditingValue(const TextEditingValue(text: '0123')); // Text change causes reformat
    expect(formatter.formatCallCount, 3);
    state.updateEditingValue(
      const TextEditingValue(text: '0123'),
    ); // No text change, does not format
    expect(formatter.formatCallCount, 3);
    state.updateEditingValue(
      const TextEditingValue(text: '0123'),
    ); // No text change, does not format
    expect(formatter.formatCallCount, 3);
    state.updateEditingValue(
      const TextEditingValue(text: '0123', selection: TextSelection.collapsed(offset: 2)),
    ); // Selection change does not reformat
    expect(formatter.formatCallCount, 3);
    state.updateEditingValue(
      const TextEditingValue(text: '0123', selection: TextSelection.collapsed(offset: 2)),
    ); // No text change, does not format
    expect(formatter.formatCallCount, 3);
    state.updateEditingValue(
      const TextEditingValue(text: '0123', selection: TextSelection.collapsed(offset: 2)),
    ); // No text change, does not format
    expect(formatter.formatCallCount, 3);

    // Composing changes should not trigger reformat, as it could cause infinite loops on some IMEs.
    state.updateEditingValue(
      const TextEditingValue(
        text: '0123',
        selection: TextSelection.collapsed(offset: 2),
        composing: TextRange(start: 1, end: 2),
      ),
    );
    expect(formatter.formatCallCount, 3);
    expect(formatter.lastOldValue.composing, TextRange.empty);
    expect(
      formatter.lastNewValue.composing,
      TextRange.empty,
    ); // The new composing was registered in formatter.
    // Clearing composing region should trigger reformat.
    state.updateEditingValue(
      const TextEditingValue(text: '01234', selection: TextSelection.collapsed(offset: 2)),
    ); // Formats, with oldValue containing composing region.
    expect(formatter.formatCallCount, 4);
    expect(formatter.lastOldValue.composing, const TextRange(start: 1, end: 2));
    expect(formatter.lastNewValue.composing, TextRange.empty);

    const List<String> referenceLog = <String>[
      '[1]: , 01',
      '[1]: normal aa',
      '[2]: 01, 012',
      '[2]: normal aaaa',
      '[3]: 012, 0123',
      '[3]: normal aaaaaa',
      '[4]: 0123, 01234',
      '[4]: normal aaaaaaaa',
    ];

    expect(formatter.log, referenceLog);
  });

  // Regression test for https://github.com/flutter/flutter/issues/53612
  testWidgets('formatter logic handles initial repeat edge case', (WidgetTester tester) async {
    final MockTextFormatter formatter = MockTextFormatter();
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              inputFormatters: <TextInputFormatter>[formatter],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.showKeyboard(find.byType(EditableText));
    controller.text = 'test';
    await tester.idle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(tester.testTextInput.editingState!['text'], equals('test'));
    expect(state.wantKeepAlive, true);

    expect(formatter.formatCallCount, 0);
    state.updateEditingValue(collapsedAtEnd('test'));
    state.updateEditingValue(
      collapsedAtEnd('test').copyWith(composing: const TextRange(start: 1, end: 2)),
    );
    // Pass to formatter once to check the values.
    state.updateEditingValue(collapsedAtEnd('test'));
    expect(formatter.lastOldValue.composing, const TextRange(start: 1, end: 2));
    expect(formatter.lastOldValue.text, 'test');
  });

  testWidgets('EditableText changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            child: MouseRegion(
              cursor: SystemMouseCursors.forbidden,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                mouseCursor: SystemMouseCursors.click,
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(EditableText)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            child: MouseRegion(
              cursor: SystemMouseCursors.forbidden,
              child: EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('Can access characters on editing string', (WidgetTester tester) async {
    late int charactersLength;
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        selectionControls: materialTextSelectionControls,
        keyboardType: TextInputType.text,
        onChanged: (String value) {
          charactersLength = value.characters.length;
        },
      ),
    );
    await tester.pumpWidget(widget);

    // Enter an extended grapheme cluster whose string length is different than
    // its characters length.
    await tester.enterText(find.byType(EditableText), '👨‍👩‍👦');
    await tester.pump();

    expect(charactersLength, 1);
  });

  testWidgets('EditableText can set and update clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );
    final RenderEditable renderObject = tester.allRenderObjects.whereType<RenderEditable>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              clipBehavior: Clip.antiAlias,
            ),
          ),
        ),
      ),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('EditableText inherits DefaultTextHeightBehavior', (WidgetTester tester) async {
    const TextHeightBehavior customTextHeightBehavior = TextHeightBehavior(
      applyHeightToFirstAscent: false,
    );
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: DefaultTextHeightBehavior(
              textHeightBehavior: customTextHeightBehavior,
              child: EditableText(
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        ),
      ),
    );
    final RenderEditable renderObject = tester.allRenderObjects.whereType<RenderEditable>().first;
    expect(renderObject.textHeightBehavior, equals(customTextHeightBehavior));
  });

  testWidgets('EditableText defaultTextHeightBehavior is used over inherited widget', (
    WidgetTester tester,
  ) async {
    const TextHeightBehavior inheritedTextHeightBehavior = TextHeightBehavior(
      applyHeightToFirstAscent: false,
    );
    const TextHeightBehavior customTextHeightBehavior = TextHeightBehavior(
      applyHeightToLastDescent: false,
      applyHeightToFirstAscent: false,
    );
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: FocusScope(
            node: focusScopeNode,
            autofocus: true,
            child: DefaultTextHeightBehavior(
              textHeightBehavior: inheritedTextHeightBehavior,
              child: EditableText(
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                textHeightBehavior: customTextHeightBehavior,
              ),
            ),
          ),
        ),
      ),
    );
    final RenderEditable renderObject = tester.allRenderObjects.whereType<RenderEditable>().first;
    expect(renderObject.textHeightBehavior, isNot(equals(inheritedTextHeightBehavior)));
    expect(renderObject.textHeightBehavior, equals(customTextHeightBehavior));
  });

  test('Asserts if composing text is not valid', () async {
    void expectToAssert(TextEditingValue value, bool shouldAssert) {
      dynamic initException;
      dynamic updateException;

      TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);
      try {
        controller = TextEditingController.fromValue(value);
      } catch (e) {
        initException = e;
      }

      controller = TextEditingController();
      addTearDown(controller.dispose);
      try {
        controller.value = value;
      } catch (e) {
        updateException = e;
      }

      expect(initException?.toString(), shouldAssert ? contains('composing range') : isNull);
      expect(updateException?.toString(), shouldAssert ? contains('composing range') : isNull);
    }

    expectToAssert(TextEditingValue.empty, false);
    expectToAssert(
      const TextEditingValue(text: 'test', composing: TextRange(start: 1, end: 0)),
      true,
    );
    expectToAssert(
      const TextEditingValue(text: 'test', composing: TextRange(start: 1, end: 9)),
      true,
    );
    expectToAssert(
      const TextEditingValue(text: 'test', composing: TextRange(start: -1, end: 9)),
      false,
    );
  });

  testWidgets('Preserves composing range if cursor moves within that range', (
    WidgetTester tester,
  ) async {
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: textStyle,
        cursorColor: cursorColor,
        selectionControls: materialTextSelectionControls,
      ),
    );
    await tester.pumpWidget(widget);

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.updateEditingValue(
      const TextEditingValue(text: 'foo composing bar', composing: TextRange(start: 4, end: 12)),
    );
    controller.selection = const TextSelection.collapsed(offset: 5);
    expect(state.currentTextEditingValue.composing, const TextRange(start: 4, end: 12));
  });

  testWidgets('Clears composing range if cursor moves outside that range', (
    WidgetTester tester,
  ) async {
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: textStyle,
        cursorColor: cursorColor,
        selectionControls: materialTextSelectionControls,
      ),
    );
    await tester.pumpWidget(widget);

    // Positioning cursor before the composing range should clear the composing range.
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    controller.selection = const TextSelection.collapsed(offset: 2);
    expect(state.currentTextEditingValue.composing, TextRange.empty);

    // Reset the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    expect(state.currentTextEditingValue.composing, const TextRange(start: 4, end: 12));

    // Positioning cursor after the composing range should clear the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    controller.selection = const TextSelection.collapsed(offset: 14);
    expect(state.currentTextEditingValue.composing, TextRange.empty);
  });

  testWidgets('Clears composing range if cursor moves outside that range - case two', (
    WidgetTester tester,
  ) async {
    final Widget widget = MaterialApp(
      home: EditableText(
        backgroundCursorColor: Colors.grey,
        controller: controller,
        focusNode: focusNode,
        style: textStyle,
        cursorColor: cursorColor,
        selectionControls: materialTextSelectionControls,
      ),
    );
    await tester.pumpWidget(widget);

    // Setting a selection before the composing range clears the composing range.
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    controller.selection = const TextSelection(baseOffset: 1, extentOffset: 2);
    expect(state.currentTextEditingValue.composing, TextRange.empty);

    // Reset the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    expect(state.currentTextEditingValue.composing, const TextRange(start: 4, end: 12));

    // Setting a selection within the composing range doesn't clear the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    controller.selection = const TextSelection(baseOffset: 5, extentOffset: 7);
    expect(state.currentTextEditingValue.composing, const TextRange(start: 4, end: 12));
    expect(
      state.currentTextEditingValue.selection,
      const TextSelection(baseOffset: 5, extentOffset: 7),
    );

    // Reset the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    expect(state.currentTextEditingValue.composing, const TextRange(start: 4, end: 12));

    // Setting a selection after the composing range clears the composing range.
    state.updateEditingValue(
      const TextEditingValue(
        text: 'foo composing bar',
        selection: TextSelection.collapsed(offset: 4),
        composing: TextRange(start: 4, end: 12),
      ),
    );
    controller.selection = const TextSelection(baseOffset: 13, extentOffset: 15);
    expect(state.currentTextEditingValue.composing, TextRange.empty);
  });

  group('Length formatter', () {
    const int maxLength = 5;

    Future<void> setupWidget(
      WidgetTester tester,
      LengthLimitingTextInputFormatter formatter,
    ) async {
      final Widget widget = MaterialApp(
        home: EditableText(
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          inputFormatters: <TextInputFormatter>[formatter],
          style: textStyle,
          cursorColor: cursorColor,
          selectionControls: materialTextSelectionControls,
        ),
      );

      await tester.pumpWidget(widget);
      await tester.pumpAndSettle();
    }

    // Regression test for https://github.com/flutter/flutter/issues/65374.
    testWidgets('will not cause crash while the TextEditingValue is composing', (
      WidgetTester tester,
    ) async {
      await setupWidget(
        tester,
        LengthLimitingTextInputFormatter(
          maxLength,
          maxLengthEnforcement: MaxLengthEnforcement.truncateAfterCompositionEnds,
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      state.updateEditingValue(const TextEditingValue(text: 'abcde'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);
      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 2, end: 4)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 2, end: 4));

      // Formatter will not update format while the editing value is composing.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 2, end: 5)),
      );
      expect(state.currentTextEditingValue.text, 'abcdef');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 2, end: 5));

      // After composing ends, formatter will update.
      state.updateEditingValue(const TextEditingValue(text: 'abcdef'));
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    testWidgets('handles composing text correctly, continued', (WidgetTester tester) async {
      await setupWidget(
        tester,
        LengthLimitingTextInputFormatter(
          maxLength,
          maxLengthEnforcement: MaxLengthEnforcement.truncateAfterCompositionEnds,
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're at maxLength with no composing text.
      controller.text = 'abcde';
      assert(state.currentTextEditingValue == const TextEditingValue(text: 'abcde'));

      // Should be able to change the editing value if the new value is still shorter
      // than maxLength.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 2, end: 4)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 2, end: 4));

      // Reset.
      controller.text = 'abcde';
      assert(state.currentTextEditingValue == const TextEditingValue(text: 'abcde'));

      // The text should not change when trying to insert when the text is already
      // at maxLength.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 5, end: 6)),
      );
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    // Regression test for https://github.com/flutter/flutter/issues/68086.
    testWidgets('enforced composing truncated', (WidgetTester tester) async {
      await setupWidget(
        tester,
        LengthLimitingTextInputFormatter(
          maxLength,
          maxLengthEnforcement: MaxLengthEnforcement.enforced,
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: 'abcde'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // When it's not longer than `maxLength`, it can still start composing.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      // `newValue` will be truncated if `composingMaxLengthEnforced`.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcdef', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.text, 'abcde');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      // Reset the value.
      state.updateEditingValue(const TextEditingValue(text: 'abcde'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Change the value in order to take effects on web test.
      state.updateEditingValue(const TextEditingValue(text: '你好啊朋友'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing with a longer value, it should be the same state.
      state.updateEditingValue(
        const TextEditingValue(text: '你好啊朋友们', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    // Regression test for https://github.com/flutter/flutter/issues/68086.
    testWidgets('default truncate behaviors with different platforms', (WidgetTester tester) async {
      await setupWidget(tester, LengthLimitingTextInputFormatter(maxLength));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: '你好啊朋友'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // When it's not longer than `maxLength`, it can still start composing.
      state.updateEditingValue(
        const TextEditingValue(text: '你好啊朋友', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      state.updateEditingValue(
        const TextEditingValue(text: '你好啊朋友们', composing: TextRange(start: 3, end: 6)),
      );
      if (kIsWeb ||
          defaultTargetPlatform == TargetPlatform.iOS ||
          defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.linux ||
          defaultTargetPlatform == TargetPlatform.fuchsia) {
        // `newValue` will not be truncated on couple platforms.
        expect(state.currentTextEditingValue.text, '你好啊朋友们');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 6));
      } else {
        // `newValue` on other platforms will be truncated.
        expect(state.currentTextEditingValue.text, '你好啊朋友');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));
      }

      // Reset the value.
      state.updateEditingValue(const TextEditingValue(text: '你好啊朋友'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing with a longer value, it should be the same state.
      state.updateEditingValue(
        const TextEditingValue(text: '你好啊朋友们', composing: TextRange(start: 3, end: 6)),
      );
      expect(state.currentTextEditingValue.text, '你好啊朋友');
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    // Regression test for https://github.com/flutter/flutter/issues/68086.
    testWidgets("composing range removed if it's overflowed the truncated value's length", (
      WidgetTester tester,
    ) async {
      await setupWidget(
        tester,
        LengthLimitingTextInputFormatter(
          maxLength,
          maxLengthEnforcement: MaxLengthEnforcement.enforced,
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're not at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      // Reset the value.
      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing with a range already overflowed the truncated length.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcdefgh', composing: TextRange(start: 5, end: 7)),
      );
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    // Regression test for https://github.com/flutter/flutter/issues/68086.
    testWidgets('composing range removed with different platforms', (WidgetTester tester) async {
      await setupWidget(tester, LengthLimitingTextInputFormatter(maxLength));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're not at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcde', composing: TextRange(start: 3, end: 5)),
      );
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));

      // Reset the value.
      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start composing with a range already overflowed the truncated length.
      state.updateEditingValue(
        const TextEditingValue(text: 'abcdefgh', composing: TextRange(start: 5, end: 7)),
      );
      if (kIsWeb ||
          defaultTargetPlatform == TargetPlatform.iOS ||
          defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.linux ||
          defaultTargetPlatform == TargetPlatform.fuchsia) {
        expect(state.currentTextEditingValue.composing, const TextRange(start: 5, end: 7));
      } else {
        expect(state.currentTextEditingValue.composing, TextRange.empty);
      }
    });

    testWidgets("composing range handled correctly when it's overflowed", (
      WidgetTester tester,
    ) async {
      const String string = '👨‍👩‍👦0123456';

      await setupWidget(tester, LengthLimitingTextInputFormatter(maxLength));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're not at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: string));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Clearing composing range if collapsed.
      state.updateEditingValue(
        const TextEditingValue(text: string, composing: TextRange(start: 10, end: 10)),
      );
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Clearing composing range if overflowed.
      state.updateEditingValue(
        const TextEditingValue(text: string, composing: TextRange(start: 10, end: 11)),
      );
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });

    // Regression test for https://github.com/flutter/flutter/issues/68086.
    testWidgets('typing in the middle with different platforms.', (WidgetTester tester) async {
      await setupWidget(tester, LengthLimitingTextInputFormatter(maxLength));

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Initially we're not at maxLength with no composing text.
      state.updateEditingValue(const TextEditingValue(text: 'abc'));
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      // Start typing in the middle.
      state.updateEditingValue(
        const TextEditingValue(text: 'abDEc', composing: TextRange(start: 3, end: 4)),
      );
      expect(state.currentTextEditingValue.text, 'abDEc');
      expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 4));

      // Keep typing when the value has exceed the limitation.
      state.updateEditingValue(
        const TextEditingValue(text: 'abDEFc', composing: TextRange(start: 3, end: 5)),
      );
      if (kIsWeb ||
          defaultTargetPlatform == TargetPlatform.iOS ||
          defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.linux ||
          defaultTargetPlatform == TargetPlatform.fuchsia) {
        expect(state.currentTextEditingValue.text, 'abDEFc');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 5));
      } else {
        expect(state.currentTextEditingValue.text, 'abDEc');
        expect(state.currentTextEditingValue.composing, const TextRange(start: 3, end: 4));
      }

      // Reset the value according to the limit.
      state.updateEditingValue(const TextEditingValue(text: 'abDEc'));
      expect(state.currentTextEditingValue.text, 'abDEc');
      expect(state.currentTextEditingValue.composing, TextRange.empty);

      state.updateEditingValue(
        const TextEditingValue(text: 'abDEFc', composing: TextRange(start: 4, end: 5)),
      );
      expect(state.currentTextEditingValue.composing, TextRange.empty);
    });
  });

  group('callback errors', () {
    const String errorText = 'Test EditableText callback error';

    testWidgets('onSelectionChanged can throw errors', (WidgetTester tester) async {
      controller.text = 'flutter is the best!';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
            selectionControls: materialTextSelectionControls,
            onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
              throw FlutterError(errorText);
            },
          ),
        ),
      );

      // Interact with the field to establish the input connection.
      await tester.tap(find.byType(EditableText));
      final dynamic error = tester.takeException();
      expect(error, isFlutterError);
      expect(error.toString(), contains(errorText));
    });

    testWidgets('onChanged can throw errors', (WidgetTester tester) async {
      controller.text = 'flutter is the best!';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
            onChanged: (String text) {
              throw FlutterError(errorText);
            },
          ),
        ),
      );

      // Modify the text and expect an error from onChanged.
      await tester.enterText(find.byType(EditableText), '...');
      final dynamic error = tester.takeException();
      expect(error, isFlutterError);
      expect(error.toString(), contains(errorText));
    });

    testWidgets('onEditingComplete can throw errors', (WidgetTester tester) async {
      controller.text = 'flutter is the best!';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
            onEditingComplete: () {
              throw FlutterError(errorText);
            },
          ),
        ),
      );

      // Interact with the field to establish the input connection.
      final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
      await tester.tapAt(topLeft + const Offset(0.0, 5.0));
      await tester.pump();

      // Submit and expect an error from onEditingComplete.
      await tester.testTextInput.receiveAction(TextInputAction.done);
      final dynamic error = tester.takeException();
      expect(error, isFlutterError);
      expect(error.toString(), contains(errorText));
    });

    testWidgets('onSubmitted can throw errors', (WidgetTester tester) async {
      controller.text = 'flutter is the best!';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
            onSubmitted: (String text) {
              throw FlutterError(errorText);
            },
          ),
        ),
      );

      // Interact with the field to establish the input connection.
      final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
      await tester.tapAt(topLeft + const Offset(0.0, 5.0));
      await tester.pump();

      // Submit and expect an error from onSubmitted.
      await tester.testTextInput.receiveAction(TextInputAction.done);
      final dynamic error = tester.takeException();
      expect(error, isFlutterError);
      expect(error.toString(), contains(errorText));
    });

    testWidgets('input formatters can throw errors', (WidgetTester tester) async {
      final TextInputFormatter badFormatter = TextInputFormatter.withFunction(
        (TextEditingValue oldValue, TextEditingValue newValue) => throw FlutterError(errorText),
      );
      controller.text = 'flutter is the best!';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            showSelectionHandles: true,
            maxLines: 2,
            controller: controller,
            inputFormatters: <TextInputFormatter>[badFormatter],
            focusNode: focusNode,
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.blue,
            style: Typography.material2018().black.titleMedium!.copyWith(fontFamily: 'Roboto'),
            keyboardType: TextInputType.text,
          ),
        ),
      );

      // Interact with the field to establish the input connection.
      await tester.tap(find.byType(EditableText));
      await tester.pump();

      await tester.enterText(find.byType(EditableText), 'text');

      final dynamic error = tester.takeException();
      expect(error, isFlutterError);
      expect(error.toString(), contains(errorText));
      expect(controller.text, 'text');
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/72400.
  testWidgets("delete doesn't cause crash when selection is -1,-1", (WidgetTester tester) async {
    final UnsettableController unsettableController = UnsettableController();
    addTearDown(unsettableController.dispose);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            autofocus: true,
            controller: unsettableController,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    await tester.pump(); // Wait for the autofocus to take effect.

    // Delete
    await sendKeys(tester, <LogicalKeyboardKey>[
      LogicalKeyboardKey.delete,
    ], targetPlatform: TargetPlatform.android);

    expect(tester.takeException(), null);
  });

  testWidgets('can change behavior by overriding text editing shortcuts', (
    WidgetTester tester,
  ) async {
    const Map<SingleActivator, Intent> testShortcuts = <SingleActivator, Intent>{
      SingleActivator(LogicalKeyboardKey.arrowLeft): ExtendSelectionByCharacterIntent(
        forward: true,
        collapseSelection: true,
      ),
      SingleActivator(LogicalKeyboardKey.keyX, control: true): ExtendSelectionByCharacterIntent(
        forward: true,
        collapseSelection: true,
      ),
      SingleActivator(LogicalKeyboardKey.keyC, control: true): ExtendSelectionByCharacterIntent(
        forward: true,
        collapseSelection: true,
      ),
      SingleActivator(LogicalKeyboardKey.keyV, control: true): ExtendSelectionByCharacterIntent(
        forward: true,
        collapseSelection: true,
      ),
      SingleActivator(LogicalKeyboardKey.keyA, control: true): ExtendSelectionByCharacterIntent(
        forward: true,
        collapseSelection: true,
      ),
    };
    controller.text = testText;
    controller.selection = const TextSelection(
      baseOffset: 0,
      extentOffset: 0,
      affinity: TextAffinity.upstream,
    );
    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 400,
            child: Shortcuts(
              shortcuts: testShortcuts,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Wait for autofocus to take effect.

    // The right arrow key moves to the right as usual.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(controller.selection.isCollapsed, isTrue);
    expect(controller.selection.baseOffset, 1);

    // And the testShortcuts also moves to the right due to the Shortcuts override.
    for (final SingleActivator singleActivator in testShortcuts.keys) {
      controller.selection = const TextSelection.collapsed(offset: 0);
      await tester.pump();

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[singleActivator.trigger],
        shortcutModifier: singleActivator.control,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, isTrue);
      expect(controller.selection.baseOffset, 1);
    }

    // On web, using keyboard for selection is handled by the browser.
  }, skip: kIsWeb); // [intended]

  testWidgets(
    'navigating by word',
    (WidgetTester tester) async {
      controller.text = 'word word word';
      // word wo|rd| word
      controller.selection = const TextSelection(
        baseOffset: 7,
        extentOffset: 9,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 9);

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      // word wo|rd word|
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 14);

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      // word wo|rd |word
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 10);
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      if (defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.iOS) {
        // word wo|rd word
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 7);
        expect(controller.selection.extentOffset, 7);

        await sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
          shift: true,
          wordModifier: true,
          targetPlatform: defaultTargetPlatform,
        );
      }

      // word |wo|rd word
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 5);

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      // |word wo|rd word
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 0);

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      // word| wo|rd word
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 4);

      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        wordModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      if (defaultTargetPlatform == TargetPlatform.macOS ||
          defaultTargetPlatform == TargetPlatform.iOS) {
        // word wo|rd word
        expect(controller.selection.isCollapsed, true);
        expect(controller.selection.baseOffset, 7);
        expect(controller.selection.extentOffset, 7);

        await sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
          shift: true,
          wordModifier: true,
          targetPlatform: defaultTargetPlatform,
        );
      }

      // word wo|rd| word
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 9);

      // On web, using keyboard for selection is handled by the browser.
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'navigating multiline text',
    (WidgetTester tester) async {
      controller.text = 'word word word\nword word\nword'; // 15 + 10 + 4;
      // wo|rd wo|rd
      controller.selection = const TextSelection(
        baseOffset: 17,
        extentOffset: 22,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 17);
      expect(controller.selection.extentOffset, 22);

      // Multiple expandRightByLine shortcuts only move to the end of the line and
      // not to the next line.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[
          LogicalKeyboardKey.arrowRight,
          LogicalKeyboardKey.arrowRight,
          LogicalKeyboardKey.arrowRight,
        ],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 17);
      expect(controller.selection.extentOffset, 24);

      // Multiple expandLeftByLine shortcuts only move to the start of the line
      // and not to the previous line.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[
          LogicalKeyboardKey.arrowLeft,
          LogicalKeyboardKey.arrowLeft,
          LogicalKeyboardKey.arrowLeft,
        ],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      switch (defaultTargetPlatform) {
        // These platforms extend by line.
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection.baseOffset, 17);
          expect(controller.selection.extentOffset, 15);

        // Mac and iOS expand by line.
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection.baseOffset, 15);
          expect(controller.selection.extentOffset, 24);
      }

      // Set the caret to the end of a line.
      controller.selection = const TextSelection(
        baseOffset: 24,
        extentOffset: 24,
        affinity: TextAffinity.upstream,
      );
      await tester.pump();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 24);
      expect(controller.selection.extentOffset, 24);

      // Can't expand right by line any further.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 24);
      expect(controller.selection.extentOffset, 24);

      // Can select the entire line from the end.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 24);
      expect(controller.selection.extentOffset, 15);

      // Set the caret to the start of a line.
      controller.selection = const TextSelection(baseOffset: 15, extentOffset: 15);
      await tester.pump();
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 15);
      expect(controller.selection.extentOffset, 15);

      // Can't expand let any further.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 15);
      expect(controller.selection.extentOffset, 15);

      // Can select the entire line from the start.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 15);
      expect(controller.selection.extentOffset, 24);
      // On web, using keyboard for selection is handled by the browser.
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    "Mac's expand by line behavior on multiple lines",
    (WidgetTester tester) async {
      controller.text = 'word word word\nword word\nword'; // 15 + 10 + 4;
      // word word word
      // wo|rd word
      // w|ord
      controller.selection = const TextSelection(
        baseOffset: 17,
        extentOffset: 26,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 17);
      expect(controller.selection.extentOffset, 26);

      // Expanding right to the end of the line moves the extent on the second
      // selected line.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 17);
      expect(controller.selection.extentOffset, 29);

      // Expanding right again does nothing.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[
          LogicalKeyboardKey.arrowRight,
          LogicalKeyboardKey.arrowRight,
          LogicalKeyboardKey.arrowRight,
        ],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 17);
      expect(controller.selection.extentOffset, 29);

      // Expanding left by line moves the base on the first selected line to the
      // beginning of that line.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 15);
      expect(controller.selection.extentOffset, 29);

      // Expanding left again does nothing.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[
          LogicalKeyboardKey.arrowLeft,
          LogicalKeyboardKey.arrowLeft,
          LogicalKeyboardKey.arrowLeft,
        ],
        shift: true,
        lineModifier: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 15);
      expect(controller.selection.extentOffset, 29);
    },
    // On web, using keyboard for selection is handled by the browser.
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets(
    "Mac's expand extent position",
    (WidgetTester tester) async {
      controller.text = 'Now is the time for all good people to come to the aid of their country';
      // Start the selection in the middle somewhere.
      controller.selection = const TextSelection.collapsed(offset: 10);
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 10);

      // With cursor in the middle of the line, cmd + left. Left end is the extent.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 10, extentOffset: 0, affinity: TextAffinity.upstream),
        ),
      );

      // With cursor in the middle of the line, cmd + right. Right end is the extent.
      controller.selection = const TextSelection.collapsed(offset: 10);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 10, extentOffset: 29, affinity: TextAffinity.upstream),
        ),
      );

      // With cursor in the middle of the line, cmd + left then cmd + right. Left end is the extent.
      controller.selection = const TextSelection.collapsed(offset: 10);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(controller.selection, equals(const TextSelection(baseOffset: 29, extentOffset: 0)));

      // With cursor in the middle of the line, cmd + right then cmd + left. Right end is the extent.
      controller.selection = const TextSelection.collapsed(offset: 10);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 0, extentOffset: 29, affinity: TextAffinity.upstream),
        ),
      );

      // With an RTL selection in the middle of the line, cmd + left. Left end is the extent.
      controller.selection = const TextSelection(baseOffset: 12, extentOffset: 8);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 12, extentOffset: 0, affinity: TextAffinity.upstream),
        ),
      );

      // With an RTL selection in the middle of the line, cmd + right. Left end is the extent.
      controller.selection = const TextSelection(baseOffset: 12, extentOffset: 8);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 29, extentOffset: 8, affinity: TextAffinity.upstream),
        ),
      );

      // With an LTR selection in the middle of the line, cmd + right. Right end is the extent.
      controller.selection = const TextSelection(baseOffset: 8, extentOffset: 12);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 8, extentOffset: 29, affinity: TextAffinity.upstream),
        ),
      );

      // With an LTR selection in the middle of the line, cmd + left. Right end is the extent.
      controller.selection = const TextSelection(baseOffset: 8, extentOffset: 12);
      await tester.pump();
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.arrowLeft],
        lineModifier: true,
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 0, extentOffset: 12, affinity: TextAffinity.upstream),
        ),
      );
    },
    // On web, using keyboard for selection is handled by the browser.
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets(
    'expanding selection to start/end single line',
    (WidgetTester tester) async {
      controller.text = 'word word word';
      // word wo|rd| word
      controller.selection = const TextSelection(
        baseOffset: 7,
        extentOffset: 9,
        affinity: TextAffinity.upstream,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.
      expect(controller.selection.isCollapsed, false);
      expect(controller.selection.baseOffset, 7);
      expect(controller.selection.extentOffset, 9);

      final String targetPlatform = defaultTargetPlatform.toString();
      final String platform =
          targetPlatform.substring(targetPlatform.indexOf('.') + 1).toLowerCase();

      // Select to the start.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.home],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      // |word word| word
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 9, extentOffset: 0, affinity: TextAffinity.upstream),
        ),
        reason: 'on $platform',
      );

      // Select to the end.
      await sendKeys(
        tester,
        <LogicalKeyboardKey>[LogicalKeyboardKey.end],
        shift: true,
        targetPlatform: defaultTargetPlatform,
      );

      // |word word word|
      expect(
        controller.selection,
        equals(
          const TextSelection(baseOffset: 0, extentOffset: 14, affinity: TextAffinity.upstream),
        ),
        reason: 'on $platform',
      );
    },
    // On web, using keyboard for selection is handled by the browser.
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets('can change text editing behavior by overriding actions', (
    WidgetTester tester,
  ) async {
    controller.text = testText;
    controller.selection = const TextSelection(
      baseOffset: 0,
      extentOffset: 0,
      affinity: TextAffinity.upstream,
    );
    bool myIntentWasCalled = false;
    final CallbackAction<ExtendSelectionByCharacterIntent> overrideAction =
        CallbackAction<ExtendSelectionByCharacterIntent>(
          onInvoke: (ExtendSelectionByCharacterIntent intent) {
            myIntentWasCalled = true;
            return null;
          },
        );
    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 400,
            child: Actions(
              actions: <Type, Action<Intent>>{ExtendSelectionByCharacterIntent: overrideAction},
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pump(); // Wait for autofocus to take effect.

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(controller.selection.isCollapsed, isTrue);
    expect(controller.selection.baseOffset, 0);
    expect(myIntentWasCalled, isTrue);

    // On web, using keyboard for selection is handled by the browser.
  }, skip: kIsWeb); // [intended]

  testWidgets('can change tap outside behavior by overriding actions', (WidgetTester tester) async {
    bool myIntentWasCalled = false;
    final CallbackAction<EditableTextTapOutsideIntent> overrideAction =
        CallbackAction<EditableTextTapOutsideIntent>(
          onInvoke: (EditableTextTapOutsideIntent intent) {
            myIntentWasCalled = true;
            return null;
          },
        );
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            SizedBox(key: key, width: 200, height: 200),
            Actions(
              actions: <Type, Action<Intent>>{EditableTextTapOutsideIntent: overrideAction},
              child: EditableText(
                autofocus: true,
                controller: controller,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
              ),
            ),
          ],
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byKey(key), warnIfMissed: false);
    await tester.pumpAndSettle();
    expect(myIntentWasCalled, isTrue);
    expect(focusNode.hasFocus, true);
  });

  testWidgets('ignore key event from web platform', (WidgetTester tester) async {
    controller.text = 'test\ntest';
    controller.selection = const TextSelection(
      baseOffset: 0,
      extentOffset: 0,
      affinity: TextAffinity.upstream,
    );
    bool myIntentWasCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 400,
            child: Actions(
              actions: <Type, Action<Intent>>{
                ExtendSelectionByCharacterIntent: CallbackAction<ExtendSelectionByCharacterIntent>(
                  onInvoke: (ExtendSelectionByCharacterIntent intent) {
                    myIntentWasCalled = true;
                    return null;
                  },
                ),
              },
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Wait for autofocus to take effect.

    if (kIsWeb) {
      await simulateKeyDownEvent(LogicalKeyboardKey.arrowRight, platform: 'web');
      await tester.pump();
      expect(myIntentWasCalled, isFalse);
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);
    } else {
      await simulateKeyDownEvent(LogicalKeyboardKey.arrowRight, platform: 'android');
      await tester.pump();
      expect(myIntentWasCalled, isTrue);
      expect(controller.selection.isCollapsed, true);
      expect(controller.selection.baseOffset, 0);
    }
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('the toolbar is disposed when selection changes and there is no selectionControls', (
    WidgetTester tester,
  ) async {
    late StateSetter setState;
    bool enableInteractiveSelection = true;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return EditableText(
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  selectionControls:
                      enableInteractiveSelection ? materialTextSelectionControls : null,
                  controller: controller,
                  enableInteractiveSelection: enableInteractiveSelection,
                );
              },
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Can't show the toolbar when there's no focus.
    expect(state.showToolbar(), false);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), findsNothing);

    // Can show the toolbar when focused even though there's no text.
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);
    await tester.pump();
    expect(state.showToolbar(), isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), findsOneWidget);

    // Find the FadeTransition in the toolbar and expect that it has not been
    // disposed.
    final FadeTransition fadeTransition = find
        .byType(FadeTransition)
        .evaluate()
        .map((Element element) => element.widget as FadeTransition)
        .firstWhere((FadeTransition fadeTransition) {
          return fadeTransition.child is CompositedTransformFollower;
        });
    expect(fadeTransition.toString(), isNot(contains('DISPOSED')));

    // Turn off interactive selection and change the text, which triggers the
    // toolbar to be disposed.
    setState(() {
      enableInteractiveSelection = false;
    });
    await tester.pump();
    await tester.enterText(find.byType(EditableText), 'abc');
    await tester.pump();

    expect(fadeTransition.toString(), contains('DISPOSED'));
    // On web, using keyboard for selection is handled by the browser.
  }, skip: kIsWeb); // [intended]

  testWidgets('EditableText does not leak animation controllers', (WidgetTester tester) async {
    controller.text = 'A';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          autofocus: true,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          cursorOpacityAnimates: true,
        ),
      ),
    );

    expect(focusNode.hasPrimaryFocus, isTrue);
    final EditableTextState state = tester.state(find.byType(EditableText));

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );

    // Start the cursor blink opacity animation controller.
    // _kCursorBlinkWaitForStart
    await tester.pump(const Duration(milliseconds: 150));
    // _kCursorBlinkHalfPeriod
    await tester.pump(const Duration(milliseconds: 500));

    // Start the floating cursor reset animation controller.
    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.End, offset: Offset.zero),
    );

    expect(tester.binding.transientCallbackCount, 2);

    await tester.pumpWidget(const SizedBox());
    expect(tester.hasRunningAnimations, isFalse);
  });

  testWidgets('Floating cursor affinity', (WidgetTester tester) async {
    EditableText.debugDeterministicCursor = true;
    final GlobalKey key = GlobalKey();
    // Set it up so that there will be word-wrap.
    controller.text = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz';

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 500),
            child: EditableText(
              key: key,
              autofocus: true,
              maxLines: 2,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    final EditableTextState state = tester.state(find.byType(EditableText));

    // Select after the first word, with default affinity (downstream).
    controller.selection = const TextSelection.collapsed(offset: 27);
    await tester.pump();
    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The floating cursor should be drawn at the end of the first line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(0.5, 15, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    // Select after the first word, with upstream affinity.
    controller.selection = const TextSelection.collapsed(
      offset: 27,
      affinity: TextAffinity.upstream,
    );
    await tester.pump();

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The floating cursor should be drawn at the beginning of the second line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(378.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    EditableText.debugDeterministicCursor = false;
  });

  testWidgets('Floating cursor ending with selection', (WidgetTester tester) async {
    EditableText.debugDeterministicCursor = true;
    final GlobalKey key = GlobalKey();
    SelectionChangedCause? lastSelectionChangedCause;
    controller.text = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ\n1234567890';
    controller.selection = const TextSelection.collapsed(offset: 0);

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          key: key,
          autofocus: true,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          cursorOpacityAnimates: true,
          onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
            lastSelectionChangedCause = cause;
          },
          maxLines: 2,
        ),
      ),
    );

    await tester.pump();
    final EditableTextState state = tester.state(find.byType(EditableText));

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The cursor should be drawn at the start of the line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(0.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Update, offset: const Offset(50, 0)),
    );
    await tester.pump();

    // The cursor should be drawn somewhere in the middle of the line
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(50.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.End, offset: Offset.zero),
    );
    await tester.pumpAndSettle(
      const Duration(milliseconds: 125),
    ); // Floating cursor has an end animation.

    // Selection should be updated based on the floating cursor location.
    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, 4);
    expect(lastSelectionChangedCause, SelectionChangedCause.forcePress);
    lastSelectionChangedCause = null;

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The cursor should be drawn near to the previous position.
    // It's different because it's snapped to exactly between characters.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(56.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Update, offset: const Offset(-56, 0)),
    );
    await tester.pump();

    // The cursor should be drawn at the start of the line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(0.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    // Simulate UIKit setting the selection using keyboard selection.
    state.updateEditingValue(
      state.currentTextEditingValue.copyWith(
        selection: const TextSelection(baseOffset: 0, extentOffset: 4),
      ),
    );
    await tester.pump();

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.End, offset: Offset.zero),
    );
    await tester.pump();

    // Selection should not be changed since it wasn't previously collapsed.
    expect(controller.selection.isCollapsed, false);
    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 4);
    expect(lastSelectionChangedCause, SelectionChangedCause.forcePress);
    lastSelectionChangedCause = null;

    // Now test using keyboard selection in a forwards direction.
    state.updateEditingValue(
      state.currentTextEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 0)),
    );
    await tester.pump();
    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The cursor should be drawn in the same (start) position.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(0.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Update, offset: const Offset(56, 0)),
    );
    await tester.pump();

    // The cursor should be drawn somewhere in the middle of the line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(56.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    // Simulate UIKit setting the selection using keyboard selection.
    state.updateEditingValue(
      state.currentTextEditingValue.copyWith(
        selection: const TextSelection(baseOffset: 0, extentOffset: 4),
      ),
    );
    await tester.pump();

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.End, offset: Offset.zero),
    );
    await tester.pump();

    // Selection should not be changed since it wasn't previously collapsed.
    expect(controller.selection.isCollapsed, false);
    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 4);
    expect(lastSelectionChangedCause, SelectionChangedCause.forcePress);
    lastSelectionChangedCause = null;

    // Test that the affinity is updated in case the floating cursor ends at the same offset.

    // Put the selection at the beginning of the second line.
    state.updateEditingValue(
      state.currentTextEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 27)),
    );
    await tester.pump();

    // Now test using keyboard selection in a forwards direction.
    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.Start, offset: Offset.zero),
    );
    await tester.pump();

    // The cursor should be drawn at the start of the second line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(0.5, 15, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    // Move the cursor to the end of the first line.

    state.updateFloatingCursor(
      RawFloatingCursorPoint(
        state: FloatingCursorDragState.Update,
        offset: const Offset(9999, -14),
      ),
    );
    await tester.pump();

    // The cursor should be drawn at the end of the first line.
    expect(
      key.currentContext!.findRenderObject(),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTWH(800.5, 1, 3, 12),
          const Radius.circular(1),
        ),
      ),
    );

    state.updateFloatingCursor(
      RawFloatingCursorPoint(state: FloatingCursorDragState.End, offset: Offset.zero),
    );
    await tester.pump();

    // Selection should be changed as it was previously collapsed.
    expect(controller.selection.isCollapsed, true);
    expect(controller.selection.baseOffset, 27);
    expect(controller.selection.extentOffset, 27);
    expect(lastSelectionChangedCause, SelectionChangedCause.forcePress);
    lastSelectionChangedCause = null;

    EditableText.debugDeterministicCursor = false;
  });

  group('Selection changed scroll into view', () {
    final String text = List<int>.generate(64, (int index) => index).join('\n');
    final TextEditingController controller = TextEditingController(text: text);
    final ScrollController scrollController = ScrollController();
    late double maxScrollExtent;

    tearDownAll(() {
      controller.dispose();
      scrollController.dispose();
    });

    Future<void> resetSelectionAndScrollOffset(
      WidgetTester tester, {
      required bool setMaxScrollExtent,
    }) async {
      controller.value = controller.value.copyWith(
        text: text,
        selection: controller.selection.copyWith(baseOffset: 0, extentOffset: 1),
      );
      await tester.pump();
      final double targetOffset =
          setMaxScrollExtent ? scrollController.position.maxScrollExtent : 0.0;
      scrollController.jumpTo(targetOffset);
      await tester.pumpAndSettle();
      maxScrollExtent = scrollController.position.maxScrollExtent;
      expect(scrollController.offset, targetOffset);
    }

    Future<TextSelectionDelegate> pumpLongScrollableText(WidgetTester tester) async {
      final GlobalKey<EditableTextState> key = GlobalKey<EditableTextState>();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                height: 32,
                child: EditableText(
                  key: key,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  scrollController: scrollController,
                  maxLines: 2,
                ),
              ),
            ),
          ),
        ),
      );

      // Populate [maxScrollExtent].
      await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: false);
      return key.currentState!;
    }

    testWidgets(
      'SelectAll toolbar action will not set max scroll on designated platforms',
      (WidgetTester tester) async {
        final TextSelectionDelegate textSelectionDelegate = await pumpLongScrollableText(tester);

        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: false);
        textSelectionDelegate.selectAll(SelectionChangedCause.toolbar);
        await tester.pump();
        expect(scrollController.offset, 0.0);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'Selection will be scrolled into view with SelectionChangedCause',
      (WidgetTester tester) async {
        final TextSelectionDelegate textSelectionDelegate = await pumpLongScrollableText(tester);

        // Cut
        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        textSelectionDelegate.cutSelection(SelectionChangedCause.keyboard);
        await tester.pump();
        expect(scrollController.offset, maxScrollExtent);

        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        textSelectionDelegate.cutSelection(SelectionChangedCause.toolbar);
        await tester.pump();
        expect(scrollController.offset.roundToDouble(), 0.0);

        // Paste
        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        await textSelectionDelegate.pasteText(SelectionChangedCause.keyboard);
        await tester.pump();
        expect(scrollController.offset, maxScrollExtent);

        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        await textSelectionDelegate.pasteText(SelectionChangedCause.toolbar);
        await tester.pump();
        expect(scrollController.offset.roundToDouble(), 0.0);

        // Select all
        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: false);
        textSelectionDelegate.selectAll(SelectionChangedCause.keyboard);
        await tester.pump();
        expect(scrollController.offset, 0.0);

        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: false);
        textSelectionDelegate.selectAll(SelectionChangedCause.toolbar);
        await tester.pump();
        expect(scrollController.offset.roundToDouble(), maxScrollExtent);

        // Copy
        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        textSelectionDelegate.copySelection(SelectionChangedCause.keyboard);
        await tester.pump();
        expect(scrollController.offset, maxScrollExtent);

        await resetSelectionAndScrollOffset(tester, setMaxScrollExtent: true);
        textSelectionDelegate.copySelection(SelectionChangedCause.toolbar);
        await tester.pump();
        expect(scrollController.offset.roundToDouble(), 0.0);
      },
      variant: TargetPlatformVariant.all(
        excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.macOS},
      ),
    );
  });

  testWidgets('Should not scroll on paste if caret already visible', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/96658.
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    controller.text = 'Lorem ipsum please paste here: \n${".\n" * 50}';

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 600.0,
            width: 600.0,
            child: EditableText(
              controller: controller,
              scrollController: scrollController,
              focusNode: focusNode,
              maxLines: null,
              style: const TextStyle(fontSize: 36.0),
              backgroundCursorColor: Colors.grey,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    await Clipboard.setData(const ClipboardData(text: 'Fairly long text to be pasted'));
    focusNode.requestFocus();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    expect(scrollController.offset, 0.0);

    controller.selection = const TextSelection.collapsed(offset: 31);
    await state.pasteText(SelectionChangedCause.toolbar);
    await tester.pumpAndSettle();

    // No scroll should happen as the caret is in the viewport all the time.
    expect(scrollController.offset, 0.0);
  });

  testWidgets('Autofill enabled by default', (WidgetTester tester) async {
    controller.text = 'A';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          autofocus: true,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          cursorOpacityAnimates: true,
        ),
      ),
    );

    assert(focusNode.hasFocus);
    expect(tester.testTextInput.log, contains(matchesMethodCall('TextInput.requestAutofill')));
  });

  testWidgets('Autofill can be disabled', (WidgetTester tester) async {
    controller.text = 'A';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          autofocus: true,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          cursorOpacityAnimates: true,
          autofillHints: null,
        ),
      ),
    );

    assert(focusNode.hasFocus);
    expect(
      tester.testTextInput.log,
      isNot(contains(matchesMethodCall('TextInput.requestAutofill'))),
    );
  });

  group('TextEditingHistory', () {
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

    TextEditingValue emptyComposingOnAndroid(TextEditingValue value) {
      if (defaultTargetPlatform == TargetPlatform.android) {
        return value.copyWith(composing: TextRange.empty);
      }
      return value;
    }

    Widget boilerplate() {
      return MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          cursorOpacityAnimates: true,
          autofillHints: null,
        ),
      );
    }

    // Wait for the throttling. This is used to ensure a new history entry is created.
    Future<void> waitForThrottling(WidgetTester tester) async {
      await tester.pump(const Duration(milliseconds: 500));
    }

    // Empty text editing value with a collapsed selection.
    const TextEditingValue emptyTextCollapsed = TextEditingValue(
      selection: TextSelection.collapsed(offset: 0),
    );

    // Texts and text editing values used repeatedly in undo/redo tests.
    const String textA = 'A';
    const String textAB = 'AB';
    const String textAC = 'AC';

    const TextEditingValue textACollapsedAtEnd = TextEditingValue(
      text: textA,
      selection: TextSelection.collapsed(offset: textA.length),
    );

    const TextEditingValue textASelected = TextEditingValue(
      text: textA,
      selection: TextSelection(baseOffset: 0, extentOffset: textA.length),
    );

    const TextEditingValue textABCollapsedAtEnd = TextEditingValue(
      text: textAB,
      selection: TextSelection.collapsed(offset: textAB.length),
    );

    const TextEditingValue textACCollapsedAtEnd = TextEditingValue(
      text: textAC,
      selection: TextSelection.collapsed(offset: textAC.length),
    );

    bool isDesktop() {
      return debugDefaultTargetPlatformOverride == TargetPlatform.macOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.windows ||
          debugDefaultTargetPlatformOverride == TargetPlatform.linux;
    }

    testWidgets(
      'Should have no effect on an empty and non-focused field',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());
        expect(controller.value, TextEditingValue.empty);

        // Undo/redo have no effect on an empty field that has never been edited.
        await sendUndo(tester);
        expect(controller.value, TextEditingValue.empty);
        await sendRedo(tester);
        expect(controller.value, TextEditingValue.empty);
        await tester.pump();
        expect(controller.value, TextEditingValue.empty);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'Should have no effect on an empty and focused field',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());
        await waitForThrottling(tester);
        expect(controller.value, TextEditingValue.empty);

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        expect(controller.value, emptyTextCollapsed);
        await waitForThrottling(tester);

        // Undo/redo should have no effect. The field is focused and the value has
        // changed, but the text remains empty.
        await sendUndo(tester);
        expect(controller.value, emptyTextCollapsed);

        await sendRedo(tester);
        expect(controller.value, emptyTextCollapsed);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'Can undo/redo a single insertion',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        await waitForThrottling(tester);
        expect(controller.value, emptyTextCollapsed);

        // First insertion.
        await tester.enterText(find.byType(EditableText), textA);
        await waitForThrottling(tester);
        expect(controller.value, textACollapsedAtEnd);

        // A redo before any undo has no effect.
        await sendRedo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Can undo a single insertion.
        await sendUndo(tester);
        expect(controller.value, emptyTextCollapsed);

        // A second undo has no effect.
        await sendUndo(tester);
        expect(controller.value, emptyTextCollapsed);

        // Can redo a single insertion.
        await sendRedo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // A second redo has no effect.
        await sendRedo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'Can undo/redo multiple insertions',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        await waitForThrottling(tester);
        expect(controller.value, emptyTextCollapsed);

        // First insertion.
        await tester.enterText(find.byType(EditableText), textA);
        await waitForThrottling(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Second insertion.
        await tester.enterText(find.byType(EditableText), textAB);
        await waitForThrottling(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // Undo the first insertion.
        await sendUndo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Undo the second insertion.
        await sendUndo(tester);
        expect(controller.value, emptyTextCollapsed);

        // Redo the second insertion.
        await sendRedo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Redo the first insertion.
        await sendRedo(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    // Regression test for https://github.com/flutter/flutter/issues/120794.
    // This is only reproducible on Android platform because it is the only
    // platform where composing changes are saved in the editing history.
    testWidgets(
      'Can undo as intented when adding a delay between undos',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        await waitForThrottling(tester);
        expect(controller.value, emptyTextCollapsed);

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        const TextEditingValue composingStep1 = TextEditingValue(
          text: '1 ni',
          composing: TextRange(start: 2, end: 4),
          selection: TextSelection.collapsed(offset: 4),
        );

        const TextEditingValue composingStep2 = TextEditingValue(
          text: '1 nihao',
          composing: TextRange(start: 2, end: 7),
          selection: TextSelection.collapsed(offset: 7),
        );

        const TextEditingValue composingStep3 = TextEditingValue(
          text: '1 你好',
          selection: TextSelection.collapsed(offset: 4),
        );

        // Enter some composing text.
        state.userUpdateTextEditingValue(composingStep1, SelectionChangedCause.keyboard);
        await waitForThrottling(tester);

        state.userUpdateTextEditingValue(composingStep2, SelectionChangedCause.keyboard);
        await waitForThrottling(tester);

        state.userUpdateTextEditingValue(composingStep3, SelectionChangedCause.keyboard);
        await waitForThrottling(tester);

        // Undo first insertion.
        await sendUndo(tester);
        expect(controller.value, emptyComposingOnAndroid(composingStep2));

        // Waiting for the throttling between undos should have no effect.
        await tester.pump(const Duration(milliseconds: 500));

        // Undo second insertion.
        await sendUndo(tester);
        expect(controller.value, emptyComposingOnAndroid(composingStep1));

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
      skip: kIsWeb, // [intended]
    );

    // Regression test for https://github.com/flutter/flutter/issues/120194.
    testWidgets(
      'Cursor does not jump after undo',
      (WidgetTester tester) async {
        // Initialize the controller with a non empty text.
        controller.text = textA;
        await tester.pumpWidget(boilerplate());

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        await waitForThrottling(tester);
        expect(controller.value, isDesktop() ? textASelected : textACollapsedAtEnd);

        // Insert some text.
        await tester.enterText(find.byType(EditableText), textAB);
        expect(controller.value, textABCollapsedAtEnd);

        // Undo the insertion without waiting for the throttling delay.
        await sendUndo(tester);
        expect(controller.value.selection.isValid, true);
        expect(controller.value, isDesktop() ? textASelected : textACollapsedAtEnd);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'Initial value is recorded when an undo is received just after getting the focus',
      (WidgetTester tester) async {
        // Initialize the controller with a non empty text.
        controller.text = textA;
        await tester.pumpWidget(boilerplate());

        // Focus the field and do not wait for throttling delay before calling undo.
        focusNode.requestFocus();
        await tester.pump();
        await sendUndo(tester);
        await waitForThrottling(tester);
        expect(controller.value, isDesktop() ? textASelected : textACollapsedAtEnd);

        // Insert some text.
        await tester.enterText(find.byType(EditableText), textAB);
        expect(controller.value, textABCollapsedAtEnd);

        // Undo the insertion.
        await sendUndo(tester);

        // Initial text should have been recorded and restored.
        expect(controller.value, isDesktop() ? textASelected : textACollapsedAtEnd);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'Can make changes in the middle of the history',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate());

        // Focus the field and wait for throttling delay to get the initial
        // state saved in text editing history.
        focusNode.requestFocus();
        await tester.pump();
        await waitForThrottling(tester);
        expect(controller.value, emptyTextCollapsed);

        // First insertion.
        await tester.enterText(find.byType(EditableText), textA);
        await waitForThrottling(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Second insertion.
        await tester.enterText(find.byType(EditableText), textAC);
        await waitForThrottling(tester);
        expect(controller.value, textACCollapsedAtEnd);

        // Undo and make a change.
        await sendUndo(tester);
        expect(controller.value, textACollapsedAtEnd);
        await tester.enterText(find.byType(EditableText), textAB);
        await waitForThrottling(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // Try a redo, state should not change because of the previous undo.
        await sendRedo(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // Trying again will have no effect.
        await sendRedo(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // Undo should restore state as it was before second insertion.
        await sendUndo(tester);
        expect(controller.value, textACollapsedAtEnd);

        // Another undo will restore state as before first insertion.
        await sendUndo(tester);
        expect(controller.value, emptyTextCollapsed);

        // Redo all changes.
        await sendRedo(tester);
        expect(controller.value, textACollapsedAtEnd);
        await sendRedo(tester);
        expect(controller.value, textABCollapsedAtEnd);

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'inside EditableText, duplicate changes',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
            ),
          ),
        );

        expect(controller.value, TextEditingValue.empty);

        focusNode.requestFocus();
        expect(controller.value, TextEditingValue.empty);
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        await tester.enterText(find.byType(EditableText), '1');
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Can undo/redo a single insertion.
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );

        // Changes that result in the same state won't be saved on the undo stack.
        await tester.enterText(find.byType(EditableText), '12');
        expect(
          controller.value,
          const TextEditingValue(text: '12', selection: TextSelection.collapsed(offset: 2)),
        );
        await tester.enterText(find.byType(EditableText), '1');
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await tester.pump(const Duration(milliseconds: 500));
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'inside EditableText, autofocus',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              autofocus: true,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
            ),
          ),
        );

        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));
        await tester.enterText(find.byType(EditableText), '1');
        await tester.pump(const Duration(milliseconds: 500));
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1', selection: TextSelection.collapsed(offset: 1)),
        );
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'does not save composing changes (except Android)',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
            ),
          ),
        );

        expect(controller.value, TextEditingValue.empty);

        focusNode.requestFocus();
        expect(controller.value, TextEditingValue.empty);
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some regular non-composing text that is undoable.
        await tester.enterText(find.byType(EditableText), '1 ');
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some composing text.
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.userUpdateTextEditingValue(
          const TextEditingValue(
            text: '1 ni',
            composing: TextRange(start: 2, end: 4),
            selection: TextSelection.collapsed(offset: 4),
          ),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some more composing text.
        state.userUpdateTextEditingValue(
          const TextEditingValue(
            text: '1 nihao',
            composing: TextRange(start: 2, end: 7),
            selection: TextSelection.collapsed(offset: 7),
          ),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Commit the composing text.
        state.userUpdateTextEditingValue(
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        // Undo/redo ignores the composing changes.
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );

        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.android}),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'does save composing changes on Android',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
            ),
          ),
        );

        expect(controller.value, TextEditingValue.empty);

        focusNode.requestFocus();
        expect(controller.value, TextEditingValue.empty);
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some regular non-composing text that is undoable.
        await tester.enterText(find.byType(EditableText), '1 ');
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some composing text.
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.userUpdateTextEditingValue(
          const TextEditingValue(
            text: '1 ni',
            composing: TextRange(start: 2, end: 4),
            selection: TextSelection.collapsed(offset: 4),
          ),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some more composing text.
        state.userUpdateTextEditingValue(
          const TextEditingValue(
            text: '1 nihao',
            composing: TextRange(start: 2, end: 7),
            selection: TextSelection.collapsed(offset: 7),
          ),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Commit the composing text.
        state.userUpdateTextEditingValue(
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        // Undo/redo includes the composing changes.
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 nihao', selection: TextSelection.collapsed(offset: 7)),
        );

        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ni', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );

        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ni', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 nihao', selection: TextSelection.collapsed(offset: 7)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 nihao', selection: TextSelection.collapsed(offset: 7)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ni', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ni', selection: TextSelection.collapsed(offset: 4)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 nihao', selection: TextSelection.collapsed(offset: 7)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 你好', selection: TextSelection.collapsed(offset: 4)),
        );

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'saves right up to composing change even when throttled',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
            ),
          ),
        );

        expect(controller.value, TextEditingValue.empty);

        focusNode.requestFocus();
        expect(controller.value, TextEditingValue.empty);
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        // Wait for the throttling.
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some regular non-composing text that is undoable.
        await tester.enterText(find.byType(EditableText), '1 ');
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Enter some regular non-composing text and then immediately enter some
        // composing text.
        await tester.enterText(find.byType(EditableText), '1 2 ');
        expect(
          controller.value,
          const TextEditingValue(text: '1 2 ', selection: TextSelection.collapsed(offset: 4)),
        );
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.userUpdateTextEditingValue(
          const TextEditingValue(
            text: '1 2 ni',
            composing: TextRange(start: 4, end: 6),
            selection: TextSelection.collapsed(offset: 6),
          ),
          SelectionChangedCause.keyboard,
        );
        expect(
          controller.value,
          const TextEditingValue(
            text: '1 2 ni',
            composing: TextRange(start: 4, end: 6),
            selection: TextSelection.collapsed(offset: 6),
          ),
        );
        await tester.pump(const Duration(milliseconds: 500));

        // Commit the composing text.
        state.userUpdateTextEditingValue(
          const TextEditingValue(text: '1 2 你', selection: TextSelection.collapsed(offset: 5)),
          SelectionChangedCause.keyboard,
        );
        await tester.pump(const Duration(milliseconds: 500));

        expect(
          controller.value,
          const TextEditingValue(text: '1 2 你', selection: TextSelection.collapsed(offset: 5)),
        );

        // Undo/redo still gets the second non-composing change.
        await sendUndo(tester);
        switch (defaultTargetPlatform) {
          // Android includes composing changes.
          case TargetPlatform.android:
            expect(
              controller.value,
              emptyComposingOnAndroid(
                const TextEditingValue(
                  text: '1 2 ni',
                  composing: TextRange(start: 4, end: 6),
                  selection: TextSelection.collapsed(offset: 6),
                ),
              ),
            );
          // Composing changes are ignored on all other platforms.
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            expect(
              controller.value,
              const TextEditingValue(text: '1 2 ', selection: TextSelection.collapsed(offset: 4)),
            );
        }
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );
        await sendUndo(tester);
        expect(
          controller.value,
          const TextEditingValue(selection: TextSelection.collapsed(offset: 0)),
        );

        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 ', selection: TextSelection.collapsed(offset: 2)),
        );
        await sendRedo(tester);
        switch (defaultTargetPlatform) {
          // Android includes composing changes.
          case TargetPlatform.android:
            expect(
              controller.value,
              emptyComposingOnAndroid(
                const TextEditingValue(
                  text: '1 2 ni',
                  composing: TextRange(start: 4, end: 6),
                  selection: TextSelection.collapsed(offset: 6),
                ),
              ),
            );
          // Composing changes are ignored on all other platforms.
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            expect(
              controller.value,
              const TextEditingValue(text: '1 2 ', selection: TextSelection.collapsed(offset: 4)),
            );
        }
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 2 你', selection: TextSelection.collapsed(offset: 5)),
        );
        await sendRedo(tester);
        expect(
          controller.value,
          const TextEditingValue(text: '1 2 你', selection: TextSelection.collapsed(offset: 5)),
        );

        // On web, these keyboard shortcuts are handled by the browser.
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // [intended]
    );
  });

  testWidgets(
    'pasting with the keyboard collapses the selection and places it after the pasted content',
    (WidgetTester tester) async {
      Future<void> testPasteSelection(WidgetTester tester, _VoidFutureCallback paste) async {
        final TextEditingController controller = TextEditingController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
            ),
          ),
        );

        await tester.pump();
        expect(controller.text, '');

        await tester.enterText(find.byType(EditableText), '12345');
        expect(
          controller.value,
          const TextEditingValue(text: '12345', selection: TextSelection.collapsed(offset: 5)),
        );

        await sendKeys(
          tester,
          <LogicalKeyboardKey>[
            LogicalKeyboardKey.arrowLeft,
            LogicalKeyboardKey.arrowLeft,
            LogicalKeyboardKey.arrowLeft,
            LogicalKeyboardKey.arrowLeft,
            LogicalKeyboardKey.arrowLeft,
          ],
          shift: true,
          targetPlatform: defaultTargetPlatform,
        );

        expect(
          controller.value,
          const TextEditingValue(
            text: '12345',
            selection: TextSelection(baseOffset: 5, extentOffset: 0),
          ),
        );

        await sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.keyC],
          shortcutModifier: true,
          targetPlatform: defaultTargetPlatform,
        );
        expect(
          controller.value,
          const TextEditingValue(
            text: '12345',
            selection: TextSelection(baseOffset: 5, extentOffset: 0),
          ),
        );

        // Pasting content of equal length, reversed selection.
        await paste();
        expect(
          controller.value,
          const TextEditingValue(text: '12345', selection: TextSelection.collapsed(offset: 5)),
        );

        // Pasting content of longer length, forward selection.
        await sendKeys(tester, <LogicalKeyboardKey>[
          LogicalKeyboardKey.arrowLeft,
        ], targetPlatform: defaultTargetPlatform);
        await sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.arrowRight],
          shift: true,
          targetPlatform: defaultTargetPlatform,
        );
        expect(
          controller.value,
          const TextEditingValue(
            text: '12345',
            selection: TextSelection(baseOffset: 4, extentOffset: 5),
          ),
        );
        await paste();
        expect(
          controller.value,
          const TextEditingValue(text: '123412345', selection: TextSelection.collapsed(offset: 9)),
        );

        // Pasting content of shorter length, forward selection.
        await sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.keyA],
          shortcutModifier: true,
          targetPlatform: defaultTargetPlatform,
        );
        expect(
          controller.value,
          const TextEditingValue(
            text: '123412345',
            selection: TextSelection(baseOffset: 0, extentOffset: 9),
          ),
        );
        await paste();
        // Pump to allow postFrameCallbacks to finish before dispose.
        await tester.pump();
        expect(
          controller.value,
          const TextEditingValue(text: '12345', selection: TextSelection.collapsed(offset: 5)),
        );
      }

      // Test pasting with the keyboard.
      await testPasteSelection(tester, () {
        return sendKeys(
          tester,
          <LogicalKeyboardKey>[LogicalKeyboardKey.keyV],
          shortcutModifier: true,
          targetPlatform: defaultTargetPlatform,
        );
      });

      // Test pasting with the toolbar.
      await testPasteSelection(tester, () async {
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        expect(state.showToolbar(), true);
        await tester.pumpAndSettle();
        expect(find.text('Paste'), findsOneWidget);
        return tester.tap(find.text('Paste'));
      });
    },
    skip: kIsWeb, // [intended]
  );

  // Regression test for https://github.com/flutter/flutter/issues/98322.
  testWidgets('EditableText consumes ActivateIntent and ButtonActivateIntent', (
    WidgetTester tester,
  ) async {
    bool receivedIntent = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Actions(
          actions: <Type, Action<Intent>>{
            ActivateIntent: CallbackAction<ActivateIntent>(
              onInvoke: (_) {
                receivedIntent = true;
                return;
              },
            ),
            ButtonActivateIntent: CallbackAction<ActivateIntent>(
              onInvoke: (_) {
                receivedIntent = true;
                return;
              },
            ),
          },
          child: EditableText(
            autofocus: true,
            backgroundCursorColor: Colors.blue,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode.hasFocus, isTrue);

    // ActivateIntent, which is triggered by space and enter in WidgetsApp, is
    // consumed by EditableText so that the space/enter reach the IME.
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();
    expect(receivedIntent, isFalse);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    expect(receivedIntent, isFalse);
  });

  // Regression test for https://github.com/flutter/flutter/issues/100585.
  testWidgets('can paste and remove field', (WidgetTester tester) async {
    controller.text = 'text';
    late StateSetter setState;
    bool showField = true;
    final _CustomTextSelectionControls controls = _CustomTextSelectionControls(
      onPaste: () {
        setState(() {
          showField = false;
        });
      },
    );
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            if (!showField) {
              return const Placeholder();
            }
            return EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: controls,
            );
          },
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.pump();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    await tester.longPress(find.byType(EditableText));
    await tester.pump();
    expect(state.showToolbar(), isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Paste'), findsOneWidget);

    await tester.tap(find.text('Paste'));
    await tester.pumpAndSettle();

    expect(tester.takeException(), null);
    // On web, the text selection toolbar paste button is handled by the browser.
  }, skip: kIsWeb); // [intended]

  // Regression test for https://github.com/flutter/flutter/issues/100585.
  testWidgets('can cut and remove field', (WidgetTester tester) async {
    controller.text = 'text';
    late StateSetter setState;
    bool showField = true;
    final _CustomTextSelectionControls controls = _CustomTextSelectionControls(
      onCut: () {
        setState(() {
          showField = false;
        });
      },
    );
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            if (!showField) {
              return const Placeholder();
            }
            return EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: controls,
            );
          },
        ),
      ),
    );

    await tester.tap(find.byType(EditableText));
    await tester.pump();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    await tester.tapAt(textOffsetToPosition(tester, 2));
    state.renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pump();
    expect(state.showToolbar(), isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Cut'), findsOneWidget);

    await tester.tap(find.text('Cut'));
    await tester.pumpAndSettle();

    expect(tester.takeException(), null);
    // On web, the text selection toolbar cut button is handled by the browser.
  }, skip: kIsWeb); // [intended]

  group('Mac document shortcuts', () {
    testWidgets(
      'ctrl-A/E',
      (WidgetTester tester) async {
        final String targetPlatformString = defaultTargetPlatform.toString();
        final String platform =
            targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
        controller.text = testText;
        controller.selection = const TextSelection(
          baseOffset: 0,
          extentOffset: 0,
          affinity: TextAffinity.upstream,
        );
        await tester.pumpWidget(
          MaterialApp(
            home: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 400,
                child: EditableText(
                  maxLines: 10,
                  controller: controller,
                  showSelectionHandles: true,
                  autofocus: true,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.right,
                ),
              ),
            ),
          ),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 0);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyE, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(
          controller.selection,
          equals(const TextSelection.collapsed(offset: 19, affinity: TextAffinity.upstream)),
          reason: 'on $platform',
        );

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyA, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(
          controller.selection,
          equals(const TextSelection.collapsed(offset: 0)),
          reason: 'on $platform',
        );
      },
      skip: kIsWeb, // [intended] on web these keys are handled by the browser.
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'ctrl-F/B',
      (WidgetTester tester) async {
        final String targetPlatformString = defaultTargetPlatform.toString();
        final String platform =
            targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
        controller.text = testText;
        controller.selection = const TextSelection(
          baseOffset: 0,
          extentOffset: 0,
          affinity: TextAffinity.upstream,
        );
        await tester.pumpWidget(
          MaterialApp(
            home: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 400,
                child: EditableText(
                  maxLines: 10,
                  controller: controller,
                  showSelectionHandles: true,
                  autofocus: true,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.right,
                ),
              ),
            ),
          ),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 0);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyF, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 1);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyB, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 0);
      },
      skip: kIsWeb, // [intended] on web these keys are handled by the browser.
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'ctrl-N/P',
      (WidgetTester tester) async {
        final String targetPlatformString = defaultTargetPlatform.toString();
        final String platform =
            targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
        controller.text = testText;
        controller.selection = const TextSelection(
          baseOffset: 0,
          extentOffset: 0,
          affinity: TextAffinity.upstream,
        );
        await tester.pumpWidget(
          MaterialApp(
            home: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 400,
                child: EditableText(
                  maxLines: 10,
                  controller: controller,
                  showSelectionHandles: true,
                  autofocus: true,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.right,
                ),
              ),
            ),
          ),
        );

        await tester.pump(); // Wait for autofocus to take effect.

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 0);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyN, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 20);

        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyP, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();

        expect(controller.selection.isCollapsed, isTrue);
        expect(controller.selection.baseOffset, 0);
      },
      skip: kIsWeb, // [intended] on web these keys are handled by the browser.
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    group('ctrl-T to transpose', () {
      Future<void> ctrlT(WidgetTester tester, String platform) async {
        await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
        await tester.sendKeyEvent(LogicalKeyboardKey.keyT, platform: platform);
        await tester.pump();
        await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft, platform: platform);
        await tester.pump();
      }

      testWidgets(
        'with normal characters',
        (WidgetTester tester) async {
          final String targetPlatformString = defaultTargetPlatform.toString();
          final String platform =
              targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();

          controller.text = testText;
          controller.selection = const TextSelection(
            baseOffset: 0,
            extentOffset: 0,
            affinity: TextAffinity.upstream,
          );
          await tester.pumpWidget(
            MaterialApp(
              home: Align(
                alignment: Alignment.topLeft,
                child: SizedBox(
                  width: 400,
                  child: EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    selectionControls: materialTextSelectionControls,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                  ),
                ),
              ),
            ),
          );

          await tester.pump(); // Wait for autofocus to take effect.

          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 0);

          // ctrl-T does nothing at the start of the field.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 0);

          controller.selection = const TextSelection(baseOffset: 1, extentOffset: 4);
          await tester.pump();
          expect(controller.selection.isCollapsed, isFalse);
          expect(controller.selection.baseOffset, 1);
          expect(controller.selection.extentOffset, 4);

          // ctrl-T does nothing when the selection isn't collapsed.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isFalse);
          expect(controller.selection.baseOffset, 1);
          expect(controller.selection.extentOffset, 4);

          controller.selection = const TextSelection.collapsed(offset: 5);
          await tester.pump();
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 5);

          // ctrl-T swaps the previous and next characters when they exist.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 6);
          expect(controller.text.substring(0, 19), 'Now si the time for');

          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 7);
          expect(controller.text.substring(0, 19), 'Now s ithe time for');

          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 8);
          expect(controller.text.substring(0, 19), 'Now s tihe time for');

          controller.selection = TextSelection.collapsed(offset: controller.text.length);
          await tester.pump();
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, controller.text.length);
          expect(controller.text.substring(55, 72), 'of their country.');

          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, controller.text.length);
          expect(controller.text.substring(55, 72), 'of their countr.y');
        },
        skip: kIsWeb, // [intended] on web these keys are handled by the browser.
        variant: const TargetPlatformVariant(<TargetPlatform>{
          TargetPlatform.iOS,
          TargetPlatform.macOS,
        }),
      );

      testWidgets(
        'with extended grapheme clusters',
        (WidgetTester tester) async {
          final String targetPlatformString = defaultTargetPlatform.toString();
          final String platform =
              targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();

          // One extended grapheme cluster of length 8 and one surrogate pair of
          // length 2.
          controller.text = '👨‍👩‍👦😆';
          controller.selection = const TextSelection(
            baseOffset: 0,
            extentOffset: 0,
            affinity: TextAffinity.upstream,
          );
          await tester.pumpWidget(
            MaterialApp(
              home: Align(
                alignment: Alignment.topLeft,
                child: SizedBox(
                  width: 400,
                  child: EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    selectionControls: materialTextSelectionControls,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                  ),
                ),
              ),
            ),
          );

          await tester.pump(); // Wait for autofocus to take effect.

          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 0);

          // ctrl-T does nothing at the start of the field.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 0);
          expect(controller.text, '👨‍👩‍👦😆');

          controller.selection = const TextSelection(baseOffset: 8, extentOffset: 10);
          await tester.pump();
          expect(controller.selection.isCollapsed, isFalse);
          expect(controller.selection.baseOffset, 8);
          expect(controller.selection.extentOffset, 10);

          // ctrl-T does nothing when the selection isn't collapsed.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isFalse);
          expect(controller.selection.baseOffset, 8);
          expect(controller.selection.extentOffset, 10);
          expect(controller.text, '👨‍👩‍👦😆');

          controller.selection = const TextSelection.collapsed(offset: 8);
          await tester.pump();
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 8);

          // ctrl-T swaps the previous and next characters when they exist.
          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 10);
          expect(controller.text, '😆👨‍👩‍👦');

          await ctrlT(tester, platform);
          expect(controller.selection.isCollapsed, isTrue);
          expect(controller.selection.baseOffset, 10);
          expect(controller.text, '👨‍👩‍👦😆');
        },
        skip: kIsWeb, // [intended] on web these keys are handled by the browser.
        variant: const TargetPlatformVariant(<TargetPlatform>{
          TargetPlatform.iOS,
          TargetPlatform.macOS,
        }),
      );
    });

    testWidgets('macOS selectors work', (WidgetTester tester) async {
      controller.text = 'test\nline2';
      controller.selection = TextSelection.collapsed(offset: controller.text.length);

      final GlobalKey<EditableTextState> key = GlobalKey<EditableTextState>();

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                key: key,
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                selectionControls: materialTextSelectionControls,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
              ),
            ),
          ),
        ),
      );

      key.currentState!.performSelector('moveLeft:');
      await tester.pump();

      expect(controller.selection, const TextSelection.collapsed(offset: 9));

      key.currentState!.performSelector('moveToBeginningOfParagraph:');
      await tester.pump();

      expect(controller.selection, const TextSelection.collapsed(offset: 5));

      // These both need to be handled, first moves cursor to the end of previous
      // paragraph, second moves to the beginning of paragraph.
      key.currentState!.performSelector('moveBackward:');
      key.currentState!.performSelector('moveToBeginningOfParagraph:');
      await tester.pump();

      expect(controller.selection, const TextSelection.collapsed(offset: 0));
    });
  });

  testWidgets(
    'contextMenuBuilder is used in place of the default text selection toolbar',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: EditableText(
                maxLines: 10,
                controller: controller,
                showSelectionHandles: true,
                autofocus: true,
                focusNode: focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                keyboardType: TextInputType.text,
                textAlign: TextAlign.right,
                selectionControls: materialTextSelectionHandleControls,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  return SizedBox(key: key, width: 10.0, height: 10.0);
                },
              ),
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
    'contextMenuBuilder can be updated to display a new menu',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142077.
      late StateSetter setState;
      final GlobalKey keyOne = GlobalKey();
      final GlobalKey keyTwo = GlobalKey();
      GlobalKey key = keyOne;

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter localSetState) {
                  setState = localSetState;
                  return EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                    selectionControls: materialTextSelectionHandleControls,
                    contextMenuBuilder: (
                      BuildContext context,
                      EditableTextState editableTextState,
                    ) {
                      return SizedBox(key: key, width: 10.0, height: 10.0);
                    },
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      expect(find.byKey(keyOne), findsNothing);
      expect(find.byKey(keyTwo), findsNothing);

      // Long-press to bring up the context menu.
      final Finder textFinder = find.byType(EditableText);
      await tester.longPress(textFinder);
      tester.state<EditableTextState>(textFinder).showToolbar();
      await tester.pumpAndSettle();

      expect(find.byKey(keyOne), findsOneWidget);
      expect(find.byKey(keyTwo), findsNothing);

      setState(() {
        key = keyTwo;
      });
      await tester.pumpAndSettle();

      expect(find.byKey(keyOne), findsNothing);
      expect(find.byKey(keyTwo), findsOneWidget);
    },
    skip: kIsWeb, // [intended] on web the browser handles the context menu.
  );

  testWidgets(
    'selectionControls can be updated',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142077.
      controller.text = 'test';
      late StateSetter setState;
      TextSelectionControls selectionControls = materialTextSelectionControls;

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter localSetState) {
                  setState = localSetState;
                  return EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                    selectionControls: selectionControls,
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      final Finder materialHandleFinder = find.byWidgetPredicate((Widget widget) {
        if (widget.runtimeType != CustomPaint) {
          return false;
        }
        final CustomPaint customPaint = widget as CustomPaint;
        return '${customPaint.painter.runtimeType}' == '_TextSelectionHandlePainter';
      });
      final Finder cupertinoHandleFinder = find.byWidgetPredicate((Widget widget) {
        if (widget.runtimeType != CustomPaint) {
          return false;
        }
        final CustomPaint customPaint = widget as CustomPaint;
        return '${customPaint.painter.runtimeType}' == '_CupertinoTextSelectionHandlePainter';
      });
      expect(materialHandleFinder, findsOneWidget);
      expect(cupertinoHandleFinder, findsNothing);

      // Long-press to select the text because Cupertino doesn't show a selection
      // handle when the selection is collapsed.
      final Finder textFinder = find.byType(EditableText);
      await tester.longPress(textFinder);
      tester.state<EditableTextState>(textFinder).showToolbar();
      await tester.pumpAndSettle();

      expect(materialHandleFinder, findsNWidgets(2));
      expect(cupertinoHandleFinder, findsNothing);

      setState(() {
        selectionControls = cupertinoTextSelectionControls;
      });
      await tester.pumpAndSettle();

      expect(materialHandleFinder, findsNothing);
      expect(cupertinoHandleFinder, findsNWidgets(2));
    },
    skip: kIsWeb, // [intended] on web the browser handles the context menu.
  );

  testWidgets(
    'onSelectionHandleTapped can be updated',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142077.
      late StateSetter setState;
      int tapCount = 0;
      VoidCallback? onSelectionHandleTapped;

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter localSetState) {
                  setState = localSetState;
                  return EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                    selectionControls: materialTextSelectionControls,
                    onSelectionHandleTapped: onSelectionHandleTapped,
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      final Finder materialHandleFinder = find.byWidgetPredicate((Widget widget) {
        if (widget.runtimeType != CustomPaint) {
          return false;
        }
        final CustomPaint customPaint = widget as CustomPaint;
        return '${customPaint.painter.runtimeType}' == '_TextSelectionHandlePainter';
      });
      expect(materialHandleFinder, findsOneWidget);
      expect(tapCount, equals(0));

      await tester.tap(materialHandleFinder);
      await tester.pump();
      expect(tapCount, equals(0));

      setState(() {
        onSelectionHandleTapped = () => tapCount += 1;
      });
      await tester.pumpAndSettle();

      await tester.tap(materialHandleFinder);
      await tester.pump();
      expect(tapCount, equals(1));
    },
    skip: kIsWeb, // [intended] on web the browser handles the context menu.
  );

  testWidgets(
    'dragStartBehavior can be updated',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142077.
      late StateSetter setState;
      DragStartBehavior dragStartBehavior = DragStartBehavior.down;

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter localSetState) {
                  setState = localSetState;
                  return EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                    selectionControls: materialTextSelectionControls,
                    dragStartBehavior: dragStartBehavior,
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      final Finder handleOverlayFinder = find.descendant(
        of: find.byType(Overlay),
        matching: find.byWidgetPredicate(
          (Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay',
        ),
      );
      expect(handleOverlayFinder, findsOneWidget);

      // Expects that the selection handle has the given DragStartBehavior.
      void checkDragStartBehavior(DragStartBehavior dragStartBehavior) {
        final RawGestureDetector rawGestureDetector = tester.widget(
          find.descendant(of: handleOverlayFinder, matching: find.byType(RawGestureDetector)).first,
        );
        final GestureRecognizerFactory<GestureRecognizer>? recognizerFactory =
            rawGestureDetector.gestures[PanGestureRecognizer];
        final PanGestureRecognizer recognizer = PanGestureRecognizer();
        recognizerFactory?.initializer(recognizer);
        expect(recognizer.dragStartBehavior, dragStartBehavior);
        recognizer.dispose();
      }

      checkDragStartBehavior(DragStartBehavior.down);

      setState(() {
        dragStartBehavior = DragStartBehavior.start;
      });
      await tester.pumpAndSettle();

      expect(handleOverlayFinder, findsOneWidget);
      checkDragStartBehavior(DragStartBehavior.start);
    },
    skip: kIsWeb, // [intended] on web the browser handles the context menu.
  );

  testWidgets(
    'magnifierConfiguration can be updated to display a new magnifier',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142077.
      late StateSetter setState;
      final GlobalKey keyOne = GlobalKey();
      final GlobalKey keyTwo = GlobalKey();
      GlobalKey key = keyOne;

      final TextMagnifierConfiguration magnifierConfiguration = TextMagnifierConfiguration(
        magnifierBuilder: (
          BuildContext context,
          MagnifierController controller,
          ValueNotifier<MagnifierInfo>? info,
        ) {
          return Placeholder(key: key);
        },
      );

      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 400,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter localSetState) {
                  setState = localSetState;
                  return EditableText(
                    maxLines: 10,
                    controller: controller,
                    showSelectionHandles: true,
                    autofocus: true,
                    focusNode: focusNode,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                    keyboardType: TextInputType.text,
                    textAlign: TextAlign.right,
                    selectionControls: materialTextSelectionHandleControls,
                    magnifierConfiguration: magnifierConfiguration,
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Wait for autofocus to take effect.

      void checkMagnifierKey(Key testKey) {
        final EditableText editableText = tester.widget(find.byType(EditableText));
        final BuildContext context = tester.firstElement(find.byType(EditableText));
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
          isA<Widget>().having(
            (Widget widget) => widget.key,
            'built magnifier key equal to passed in magnifier key',
            equals(testKey),
          ),
        );
      }

      checkMagnifierKey(keyOne);

      setState(() {
        key = keyTwo;
      });
      await tester.pumpAndSettle();

      checkMagnifierKey(keyTwo);
    },
    skip: kIsWeb, // [intended] on web the browser handles the context menu.
  );

  group('Spell check', () {
    testWidgets('Spell check configured properly when spell check disabled by default', (
      WidgetTester tester,
    ) async {
      controller.text = 'A';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.spellCheckEnabled, isFalse);
    });

    testWidgets('Spell check configured properly when spell check disabled manually', (
      WidgetTester tester,
    ) async {
      controller.text = 'A';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
            spellCheckConfiguration: const SpellCheckConfiguration.disabled(),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.spellCheckEnabled, isFalse);
    });

    testWidgets(
      'Error thrown when spell check configuration defined without specifying misspelled text style',
      (WidgetTester tester) async {
        controller.text = 'A';

        expect(() {
          EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
            spellCheckConfiguration: const SpellCheckConfiguration(),
          );
        }, throwsAssertionError);
      },
    );

    testWidgets(
      'Spell check configured properly when spell check enabled without specified spell check service and native spell check service defined',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        controller.text = 'A';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: const TextStyle(),
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        expect(state.spellCheckEnabled, isTrue);
        expect(
          state.spellCheckConfiguration.spellCheckService.runtimeType,
          equals(DefaultSpellCheckService),
        );
        tester.binding.platformDispatcher.clearNativeSpellCheckServiceDefined();
      },
    );

    testWidgets('Spell check configured properly with specified spell check service', (
      WidgetTester tester,
    ) async {
      final FakeSpellCheckService fakeSpellCheckService = FakeSpellCheckService();
      controller.text = 'A';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
            spellCheckConfiguration: SpellCheckConfiguration(
              spellCheckService: fakeSpellCheckService,
              misspelledTextStyle: TextField.materialMisspelledTextStyle,
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(
        state.spellCheckConfiguration.spellCheckService.runtimeType,
        equals(FakeSpellCheckService),
      );
    });

    testWidgets(
      'Spell check disabled when spell check configuration specified but no default spell check service available',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = false;
        controller.text = 'A';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: const TextStyle(),
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
              ),
            ),
          ),
        );

        expect(tester.takeException(), isA<AssertionError>());
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        expect(state.spellCheckConfiguration, equals(const SpellCheckConfiguration.disabled()));
        tester.binding.platformDispatcher.clearNativeSpellCheckServiceDefined();
      },
    );

    testWidgets(
      'findSuggestionSpanAtCursorIndex finds correct span with cursor in middle of a word',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        controller.text = 'A';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: const TextStyle(),
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
              ),
            ),
          ),
        );
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        const int cursorIndex = 21;
        const SuggestionSpan expectedSpan = SuggestionSpan(TextRange(start: 20, end: 23), <String>[
          'Hey',
          'He',
        ]);
        const List<SuggestionSpan> suggestionSpans = <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
          expectedSpan,
          SuggestionSpan(TextRange(start: 25, end: 30), <String>['green', 'grey', 'great']),
        ];

        // Omitting actual text in results for brevity. Same for following tests that test the findSuggestionSpanAtCursorIndex method.
        state.spellCheckResults = const SpellCheckResults('', suggestionSpans);
        final SuggestionSpan? suggestionSpan = state.findSuggestionSpanAtCursorIndex(cursorIndex);

        expect(suggestionSpan, equals(expectedSpan));
      },
    );

    testWidgets(
      'findSuggestionSpanAtCursorIndex finds correct span with cursor on edge of a word',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        controller.text = 'A';

        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              controller: controller,
              focusNode: focusNode,
              style: const TextStyle(),
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
              cursorOpacityAnimates: true,
              autofillHints: null,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
              ),
            ),
          ),
        );
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        const int cursorIndex = 23;
        const SuggestionSpan expectedSpan = SuggestionSpan(TextRange(start: 20, end: 23), <String>[
          'Hey',
          'He',
        ]);
        const List<SuggestionSpan> suggestionSpans = <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
          expectedSpan,
          SuggestionSpan(TextRange(start: 25, end: 30), <String>['green', 'grey', 'great']),
        ];

        state.spellCheckResults = const SpellCheckResults('', suggestionSpans);
        final SuggestionSpan? suggestionSpan = state.findSuggestionSpanAtCursorIndex(cursorIndex);

        expect(suggestionSpan, equals(expectedSpan));
      },
    );

    testWidgets('findSuggestionSpanAtCursorIndex finds no span when cursor out of range of spans', (
      WidgetTester tester,
    ) async {
      tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
      controller.text = 'A';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
            spellCheckConfiguration: const SpellCheckConfiguration(
              misspelledTextStyle: TextField.materialMisspelledTextStyle,
            ),
          ),
        ),
      );
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      const int cursorIndex = 33;
      const SuggestionSpan expectedSpan = SuggestionSpan(TextRange(start: 20, end: 23), <String>[
        'Hey',
        'He',
      ]);
      const List<SuggestionSpan> suggestionSpans = <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
        expectedSpan,
        SuggestionSpan(TextRange(start: 25, end: 30), <String>['green', 'grey', 'great']),
      ];

      state.spellCheckResults = const SpellCheckResults('', suggestionSpans);
      final SuggestionSpan? suggestionSpan = state.findSuggestionSpanAtCursorIndex(cursorIndex);

      expect(suggestionSpan, isNull);
    });

    testWidgets('findSuggestionSpanAtCursorIndex finds no span when word correctly spelled', (
      WidgetTester tester,
    ) async {
      tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
      controller.text = 'A';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(),
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
            cursorOpacityAnimates: true,
            autofillHints: null,
            spellCheckConfiguration: const SpellCheckConfiguration(
              misspelledTextStyle: TextField.materialMisspelledTextStyle,
            ),
          ),
        ),
      );
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      const int cursorIndex = 5;
      const SuggestionSpan expectedSpan = SuggestionSpan(TextRange(start: 20, end: 23), <String>[
        'Hey',
        'He',
      ]);
      const List<SuggestionSpan> suggestionSpans = <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
        expectedSpan,
        SuggestionSpan(TextRange(start: 25, end: 30), <String>['green', 'grey', 'great']),
      ];

      state.spellCheckResults = const SpellCheckResults('', suggestionSpans);
      final SuggestionSpan? suggestionSpan = state.findSuggestionSpanAtCursorIndex(cursorIndex);

      expect(suggestionSpan, isNull);
    });

    testWidgets('can show spell check suggestions toolbar when there are spell check results', (
      WidgetTester tester,
    ) async {
      tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
      const TextEditingValue value = TextEditingValue(
        text: 'tset test test',
        selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
      );
      controller.value = value;
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            spellCheckConfiguration: const SpellCheckConfiguration(
              misspelledTextStyle: TextField.materialMisspelledTextStyle,
              spellCheckSuggestionsToolbarBuilder:
                  TextField.defaultSpellCheckSuggestionsToolbarBuilder,
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      // Can't show the toolbar when there's no focus.
      expect(state.showSpellCheckSuggestionsToolbar(), false);
      await tester.pumpAndSettle();
      expect(find.text('DELETE'), findsNothing);

      // Can't show the toolbar when there are no spell check results.
      expect(state.showSpellCheckSuggestionsToolbar(), false);
      await tester.pumpAndSettle();
      expect(find.text('test'), findsNothing);
      expect(find.text('sets'), findsNothing);
      expect(find.text('set'), findsNothing);
      expect(find.text('DELETE'), findsNothing);

      // Can show the toolbar when there are spell check results.
      state.spellCheckResults = const SpellCheckResults('test tset test', <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 0, end: 4), <String>['test', 'sets', 'set']),
      ]);
      state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);

      await tester.pumpAndSettle();
      // Toolbar will only show on non-web platforms.
      expect(state.showSpellCheckSuggestionsToolbar(), !kIsWeb);
      await tester.pumpAndSettle();

      const Matcher matcher = kIsWeb ? findsNothing : findsOneWidget;
      expect(find.text('test'), matcher);
      expect(find.text('sets'), matcher);
      expect(find.text('set'), matcher);
      expect(find.text('DELETE'), matcher);
    });

    testWidgets(
      'can show spell check suggestions toolbar when there are no spell check results on iOS',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        const TextEditingValue value = TextEditingValue(
          text: 'tset test test',
          selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
        );
        controller.value = value;
        await tester.pumpWidget(
          CupertinoApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: CupertinoTextField.cupertinoMisspelledTextStyle,
                spellCheckSuggestionsToolbarBuilder:
                    CupertinoTextField.defaultSpellCheckSuggestionsToolbarBuilder,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        // Can't show the toolbar when there's no focus.
        expect(state.showSpellCheckSuggestionsToolbar(), false);
        await tester.pumpAndSettle();
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNothing);

        // Can't show the toolbar when there are no spell check results.
        expect(state.showSpellCheckSuggestionsToolbar(), false);
        await tester.pumpAndSettle();
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNothing);

        // Shows 'No Replacements Found' when there are spell check results but no
        // suggestions.
        state.spellCheckResults = const SpellCheckResults('test tset test', <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 0, end: 4), <String>[]),
        ]);
        state.renderEditable.selectWordsInRange(
          from: Offset.zero,
          cause: SelectionChangedCause.tap,
        );

        await tester.pumpAndSettle();
        // Toolbar will only show on non-web platforms.
        expect(state.showSpellCheckSuggestionsToolbar(), isTrue);
        await tester.pumpAndSettle();

        expect(find.byType(CupertinoTextSelectionToolbarButton), findsOneWidget);
        expect(find.byType(CupertinoButton), findsOneWidget);
        expect(find.text('No Replacements Found'), findsOneWidget);
        final CupertinoButton button = tester.widget(find.byType(CupertinoButton));
        expect(button.enabled, isFalse);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'cupertino spell check suggestions toolbar buttons correctly change the composing region',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        const TextEditingValue value = TextEditingValue(
          text: 'tset test test',
          selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
        );
        controller.value = value;
        await tester.pumpWidget(
          CupertinoApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: cupertinoTextSelectionControls,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: CupertinoTextField.cupertinoMisspelledTextStyle,
                spellCheckSuggestionsToolbarBuilder:
                    CupertinoTextField.defaultSpellCheckSuggestionsToolbarBuilder,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.spellCheckResults = const SpellCheckResults('tset test test', <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 0, end: 4), <String>['test', 'sets', 'set']),
        ]);
        state.renderEditable.selectWordsInRange(
          from: Offset.zero,
          cause: SelectionChangedCause.tap,
        );
        await tester.pumpAndSettle();

        // Set last tap down position so that selecting the word edge will be
        // a valid operation.
        final Offset pos1 = textOffsetToPosition(tester, 1);
        final TestGesture gesture = await tester.startGesture(pos1);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(state.currentTextEditingValue.selection.baseOffset, equals(1));

        // Test that tapping misspelled word replacement buttons will replace
        // the correct word and select the word edge.
        state.showSpellCheckSuggestionsToolbar();
        await tester.pumpAndSettle();

        if (kIsWeb) {
          expect(find.text('sets'), findsNothing);
        } else {
          expect(find.text('sets'), findsOneWidget);
          await tester.tap(find.text('sets'));
          await tester.pumpAndSettle();
          expect(state.currentTextEditingValue.text, equals('sets test test'));
          expect(state.currentTextEditingValue.selection.baseOffset, equals(4));
        }
      },
    );

    testWidgets(
      'material spell check suggestions toolbar buttons correctly change the composing region',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        const TextEditingValue value = TextEditingValue(
          text: 'tset test test',
          composing: TextRange(start: 0, end: 4),
          selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
        );
        controller.value = value;
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
                spellCheckSuggestionsToolbarBuilder:
                    TextField.defaultSpellCheckSuggestionsToolbarBuilder,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        state.spellCheckResults = const SpellCheckResults('tset test test', <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 0, end: 4), <String>['test', 'sets', 'set']),
        ]);
        state.renderEditable.selectWordsInRange(
          from: Offset.zero,
          cause: SelectionChangedCause.tap,
        );
        await tester.pumpAndSettle();
        expect(state.currentTextEditingValue.selection.baseOffset, equals(0));

        // Test misspelled word replacement buttons.
        state.showSpellCheckSuggestionsToolbar();
        await tester.pumpAndSettle();

        if (kIsWeb) {
          expect(find.text('sets'), findsNothing);
        } else {
          expect(find.text('sets'), findsOneWidget);
          await tester.tap(find.text('sets'));
          await tester.pumpAndSettle();
          expect(state.currentTextEditingValue.text, equals('sets test test'));
          expect(state.currentTextEditingValue.selection.baseOffset, equals(0));
        }

        // Test delete button.
        state.showSpellCheckSuggestionsToolbar();
        await tester.pumpAndSettle();
        if (kIsWeb) {
          expect(find.text('DELETE'), findsNothing);
        } else {
          expect(find.text('DELETE'), findsOneWidget);
          await tester.tap(find.text('DELETE'));
          await tester.pumpAndSettle();
          expect(state.currentTextEditingValue.text, equals(' test test'));
          expect(state.currentTextEditingValue.selection.baseOffset, equals(0));
        }
      },
    );

    testWidgets(
      'replacing puts cursor at the end of the word',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        controller.value = const TextEditingValue(
          // All misspellings of "test". One the same length, one shorter, and one
          // longer.
          text: 'tset tst testt',
          selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
        );
        await tester.pumpWidget(
          CupertinoApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: CupertinoTextField.cupertinoMisspelledTextStyle,
                spellCheckSuggestionsToolbarBuilder:
                    CupertinoTextField.defaultSpellCheckSuggestionsToolbarBuilder,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        state.spellCheckResults = SpellCheckResults(controller.value.text, const <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 0, end: 4), <String>['test']),
          SuggestionSpan(TextRange(start: 5, end: 8), <String>['test']),
          SuggestionSpan(TextRange(start: 9, end: 13), <String>['test']),
        ]);
        await tester.tapAt(textOffsetToPosition(tester, 0));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsOneWidget);

        // Replacing a word of the same length as the replacement puts the cursor
        // at the end of the new word.
        await tester.tap(find.text('test'));
        await tester.pumpAndSettle();
        expect(
          controller.value,
          equals(
            const TextEditingValue(
              text: 'test tst testt',
              selection: TextSelection.collapsed(offset: 4),
            ),
          ),
        );

        state.spellCheckResults = SpellCheckResults(controller.value.text, const <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 5, end: 8), <String>['test']),
          SuggestionSpan(TextRange(start: 9, end: 13), <String>['test']),
        ]);
        await tester.tapAt(textOffsetToPosition(tester, 5));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsOneWidget);

        // Replacing a word of less length as the replacement puts the cursor at
        // the end of the new word.
        await tester.tap(find.text('test'));
        await tester.pumpAndSettle();
        expect(
          controller.value,
          equals(
            const TextEditingValue(
              text: 'test test testt',
              selection: TextSelection.collapsed(offset: 9),
            ),
          ),
        );

        state.spellCheckResults = SpellCheckResults(controller.value.text, const <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 10, end: 15), <String>['test']),
        ]);
        await tester.tapAt(textOffsetToPosition(tester, 10));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsOneWidget);

        // Replacing a word of greater length as the replacement puts the cursor
        // at the end of the new word.
        await tester.tap(find.text('test'));
        await tester.pumpAndSettle();
        expect(
          controller.value,
          equals(
            const TextEditingValue(
              text: 'test test test',
              selection: TextSelection.collapsed(offset: 14),
            ),
          ),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.android,
      }),
      skip: kIsWeb, // [intended]
    );

    testWidgets(
      'tapping on a misspelled word hides the handles',
      (WidgetTester tester) async {
        tester.binding.platformDispatcher.nativeSpellCheckServiceDefinedTestValue = true;
        controller.value = const TextEditingValue(
          // All misspellings of "test". One the same length, one shorter, and one
          // longer.
          text: 'test test testt',
          selection: TextSelection(affinity: TextAffinity.upstream, baseOffset: 0, extentOffset: 4),
        );
        await tester.pumpWidget(
          MaterialApp(
            home: EditableText(
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              selectionControls: materialTextSelectionControls,
              showSelectionHandles: true,
              spellCheckConfiguration: const SpellCheckConfiguration(
                misspelledTextStyle: TextField.materialMisspelledTextStyle,
                spellCheckSuggestionsToolbarBuilder:
                    TextField.defaultSpellCheckSuggestionsToolbarBuilder,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

        state.spellCheckResults = SpellCheckResults(controller.value.text, const <SuggestionSpan>[
          SuggestionSpan(TextRange(start: 10, end: 15), <String>['test']),
        ]);
        await tester.tapAt(textOffsetToPosition(tester, 0));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isFalse);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsNothing);
        expect(state.selectionOverlay!.handlesAreVisible, isTrue);

        await tester.tapAt(textOffsetToPosition(tester, 12));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isTrue);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsOneWidget);
        expect(state.selectionOverlay!.handlesAreVisible, isFalse);

        await tester.tapAt(textOffsetToPosition(tester, 5));
        await tester.pumpAndSettle();
        expect(state.showSpellCheckSuggestionsToolbar(), isFalse);
        await tester.pumpAndSettle();
        expect(find.text('test'), findsNothing);
        expect(state.selectionOverlay!.handlesAreVisible, isTrue);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
      skip: kIsWeb, // [intended]
    );
  });

  group('magnifier', () {
    testWidgets('should build nothing by default', (WidgetTester tester) async {
      final EditableText editableText = EditableText(
        controller: controller,
        showSelectionHandles: true,
        autofocus: true,
        focusNode: focusNode,
        style: Typography.material2018().black.titleMedium!,
        cursorColor: Colors.blue,
        backgroundCursorColor: Colors.grey,
        selectionControls: materialTextSelectionControls,
        keyboardType: TextInputType.text,
        textAlign: TextAlign.right,
      );

      await tester.pumpWidget(MaterialApp(home: editableText));

      final BuildContext context = tester.firstElement(find.byType(EditableText));
      final ValueNotifier<MagnifierInfo> notifier = ValueNotifier<MagnifierInfo>(
        MagnifierInfo.empty,
      );
      addTearDown(notifier.dispose);

      expect(
        editableText.magnifierConfiguration.magnifierBuilder(
          context,
          MagnifierController(),
          notifier,
        ),
        isNull,
      );
    });

    testWidgets('magnifier is in correct position when EditableText is scaled', (
      WidgetTester tester,
    ) async {
      controller.text = 'hello \n world \n this \n is \n text';
      final GlobalKey magnifierKey = GlobalKey();
      const double scale = 0.5;
      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Transform.scale(
                scale: scale,
                child: EditableText(
                  controller: controller,
                  maxLines: null,
                  showSelectionHandles: true,
                  autofocus: true,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  selectionControls: materialTextSelectionControls,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.right,
                  magnifierConfiguration: TextMagnifierConfiguration(
                    shouldDisplayHandlesInMagnifier: false,
                    magnifierBuilder: (
                      BuildContext context,
                      MagnifierController controller,
                      ValueNotifier<MagnifierInfo>? notifier,
                    ) {
                      return TextMagnifier(key: magnifierKey, magnifierInfo: notifier!);
                    },
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      await tester.tapAt(textOffsetToPosition(tester, 3));
      await tester.pumpAndSettle();
      final List<RenderBox> handles = List<RenderBox>.from(
        tester.renderObjectList<RenderBox>(
          find.descendant(
            of: find.byType(CompositedTransformFollower),
            matching: find.byType(Padding),
          ),
        ),
      );
      expect(handles, hasLength(1));
      final RenderBox handle = handles.first;
      expect(find.byKey(magnifierKey), findsNothing);

      final TestGesture gesture = await tester.startGesture(
        handle.localToGlobal(Offset(handle.size.width / 2, handle.size.height / 2)),
      );
      await tester.pump(const Duration(milliseconds: 200));
      expect(find.byKey(magnifierKey), findsOneWidget);
      final Offset magnifierStart = tester.getTopLeft(find.byKey(magnifierKey));

      // Dragging by a quarter of a line height does not move the magnifier.
      // Typically, when not scaled, you need to drag by a full line height to
      // get the magnifier to move vertically.
      final double lineHeight = findRenderEditable(tester).preferredLineHeight;
      await gesture.moveBy(Offset(0.0, lineHeight / 4));
      await tester.pump(const Duration(milliseconds: 20));
      await tester.pumpAndSettle();
      expect(find.byKey(magnifierKey), findsOneWidget);
      expect(tester.getTopLeft(find.byKey(magnifierKey)), magnifierStart);

      // Dragging by another quarter line height (total half a line height) does
      // move the magnifier, because the text is scaled down by half.
      await gesture.moveBy(Offset(0.0, lineHeight / 4));
      await tester.pump(const Duration(milliseconds: 20));
      await tester.pumpAndSettle();
      expect(find.byKey(magnifierKey), findsOneWidget);
      expect(tester.getTopLeft(find.byKey(magnifierKey)).dy, magnifierStart.dy + lineHeight / 2);

      // Drag back up by a quarter line height, cursor doesn't move.
      await gesture.moveBy(Offset(0.0, -lineHeight / 4));
      await tester.pump(const Duration(milliseconds: 20));
      await tester.pumpAndSettle();
      expect(find.byKey(magnifierKey), findsOneWidget);
      expect(tester.getTopLeft(find.byKey(magnifierKey)).dy, magnifierStart.dy + lineHeight / 2);

      // Continuing the drag up to a half line height (whole line height scaled)
      // does move the cursor.
      await gesture.moveBy(Offset(0.0, -lineHeight / 4));
      await tester.pump(const Duration(milliseconds: 20));
      await tester.pumpAndSettle();
      expect(find.byKey(magnifierKey), findsOneWidget);
      expect(tester.getTopLeft(find.byKey(magnifierKey)), magnifierStart);

      await gesture.up();
      await tester.pump(const Duration(milliseconds: 20));
      expect(find.byKey(magnifierKey), findsNothing);

      await tester.pumpAndSettle();
    });
  });

  // Regression test for: https://github.com/flutter/flutter/issues/117418.
  testWidgets('can handle the partial selection of a multi-code-unit glyph', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          showSelectionHandles: true,
          autofocus: true,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
          selectionControls: materialTextSelectionControls,
          keyboardType: TextInputType.text,
          textAlign: TextAlign.right,
          minLines: 2,
          maxLines: 2,
        ),
      ),
    );

    await tester.enterText(find.byType(EditableText), '12345');
    await tester.pumpAndSettle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.userUpdateTextEditingValue(
      const TextEditingValue(
        // This is an extended grapheme cluster made up of several code units,
        // which has length 8.  A selection from 0-1 does not fully select it.
        text: '👨‍👩‍👦',
        selection: TextSelection(baseOffset: 0, extentOffset: 1),
      ),
      SelectionChangedCause.keyboard,
    );

    await tester.pumpAndSettle();

    expect(tester.takeException(), null);
  });

  testWidgets('does not crash when didChangeMetrics is called after unmounting', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          controller: controller,
          focusNode: focusNode,
          style: Typography.material2018().black.titleMedium!,
          cursorColor: Colors.blue,
          backgroundCursorColor: Colors.grey,
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    // Disposes the EditableText.
    await tester.pumpWidget(const Placeholder());

    // Shouldn't crash.
    state.didChangeMetrics();
  });

  testWidgets('_CompositionCallback widget does not skip frames', (WidgetTester tester) async {
    EditableText.debugDeterministicCursor = true;
    controller.value = const TextEditingValue(selection: TextSelection.collapsed(offset: 0));
    Offset offset = Offset.zero;
    late StateSetter setState;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            return Transform.translate(
              offset: offset,
              // The EditableText is configured in a way that the it doesn't
              // explicitly request repaint on focus change.
              child: TickerMode(
                enabled: false,
                child: RepaintBoundary(
                  child: EditableText(
                    controller: controller,
                    focusNode: focusNode,
                    style: const TextStyle(),
                    showCursor: false,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    focusNode.requestFocus();
    await tester.pump();
    tester.testTextInput.log.clear();

    // The composition callback should be registered. To verify, change the
    // parent layer's transform.
    setState(() {
      offset = const Offset(42, 0);
    });
    await tester.pump();

    expect(
      tester.testTextInput.log,
      contains(
        matchesMethodCall(
          'TextInput.setEditableSizeAndTransform',
          args: containsPair(
            'transform',
            Matrix4.translationValues(offset.dx, offset.dy, 0).storage,
          ),
        ),
      ),
    );

    EditableText.debugDeterministicCursor = false;
  });

  group('selection behavior when receiving focus', () {
    testWidgets('tabbing between fields', (WidgetTester tester) async {
      final bool isDesktop =
          debugDefaultTargetPlatformOverride == TargetPlatform.macOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.windows ||
          debugDefaultTargetPlatformOverride == TargetPlatform.linux;

      final TextEditingController controller1 = TextEditingController();
      addTearDown(controller1.dispose);
      final TextEditingController controller2 = TextEditingController();
      addTearDown(controller2.dispose);
      controller1.text = 'Text1';
      controller2.text = 'Text2\nLine2';
      final FocusNode focusNode1 = FocusNode();
      addTearDown(focusNode1.dispose);
      final FocusNode focusNode2 = FocusNode();
      addTearDown(focusNode2.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              EditableText(
                key: ValueKey<String>(controller1.text),
                controller: controller1,
                focusNode: focusNode1,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
              ),
              const SizedBox(height: 200.0),
              EditableText(
                key: ValueKey<String>(controller2.text),
                controller: controller2,
                focusNode: focusNode2,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
                minLines: 10,
                maxLines: 20,
              ),
              const SizedBox(height: 100.0),
            ],
          ),
        ),
      );

      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasFocus, isFalse);
      expect(controller1.selection, const TextSelection.collapsed(offset: -1));
      expect(controller2.selection, const TextSelection.collapsed(offset: -1));

      // Tab to the first field (single line).
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(focusNode1.hasFocus, isTrue);
      expect(focusNode2.hasFocus, isFalse);
      expect(
        controller1.selection,
        kIsWeb || isDesktop
            ? TextSelection(baseOffset: 0, extentOffset: controller1.text.length)
            : TextSelection.collapsed(offset: controller1.text.length),
      );

      // Move the cursor to another position in the first field.
      await tester.tapAt(textOffsetToPosition(tester, controller1.text.length - 1));
      await tester.pumpAndSettle();
      expect(controller1.selection, TextSelection.collapsed(offset: controller1.text.length - 1));

      // Tab to the second field (multiline).
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasFocus, isTrue);
      expect(controller2.selection, TextSelection.collapsed(offset: controller2.text.length));

      // Move the cursor to another position in the second field.
      await tester.tapAt(textOffsetToPosition(tester, controller2.text.length - 1, index: 1));
      await tester.pumpAndSettle();
      expect(controller2.selection, TextSelection.collapsed(offset: controller2.text.length - 1));

      // On web, the document root is also focusable.
      if (kIsWeb) {
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();
        expect(focusNode1.hasFocus, isFalse);
        expect(focusNode2.hasFocus, isFalse);
      }

      // Tabbing again goes back to the first field and reselects the field.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(focusNode1.hasFocus, isTrue);
      expect(focusNode2.hasFocus, isFalse);
      expect(
        controller1.selection,
        kIsWeb || isDesktop
            ? TextSelection(baseOffset: 0, extentOffset: controller1.text.length)
            : TextSelection.collapsed(offset: controller1.text.length - 1),
      );

      // Tabbing to the second field again retains the moved selection.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(focusNode1.hasFocus, isFalse);
      expect(focusNode2.hasFocus, isTrue);
      expect(controller2.selection, TextSelection.collapsed(offset: controller2.text.length - 1));
    }, variant: TargetPlatformVariant.all());

    testWidgets('Selection is updated when the field has focus and the new selection is invalid', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/120631.
      controller.text = 'Text';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            key: ValueKey<String>(controller.text),
            controller: controller,
            focusNode: focusNode,
            style: Typography.material2018().black.titleMedium!,
            cursorColor: Colors.blue,
            backgroundCursorColor: Colors.grey,
          ),
        ),
      );

      expect(focusNode.hasFocus, isFalse);
      expect(controller.selection, const TextSelection.collapsed(offset: -1));

      // Tab to focus the field.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);
      expect(
        controller.selection,
        kIsWeb
            ? TextSelection(baseOffset: 0, extentOffset: controller.text.length)
            : TextSelection.collapsed(offset: controller.text.length),
      );

      // Update text without specifying the selection.
      controller.text = 'Updated';

      // As the TextField is focused the selection should be automatically adjusted.
      expect(focusNode.hasFocus, isTrue);
      expect(
        controller.selection,
        kIsWeb
            ? TextSelection(baseOffset: 0, extentOffset: controller.text.length)
            : TextSelection.collapsed(offset: controller.text.length),
      );
    });

    testWidgets(
      'when having focus stolen between frames on web',
      (WidgetTester tester) async {
        controller.text = 'Text1';
        final FocusNode focusNode1 = FocusNode();
        addTearDown(focusNode1.dispose);
        final FocusNode focusNode2 = FocusNode();
        addTearDown(focusNode2.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                EditableText(
                  key: ValueKey<String>(controller.text),
                  controller: controller,
                  focusNode: focusNode1,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                ),
                const SizedBox(height: 200.0),
                Focus(focusNode: focusNode2, child: const SizedBox.shrink()),
                const SizedBox(height: 100.0),
              ],
            ),
          ),
        );

        expect(focusNode1.hasFocus, isFalse);
        expect(focusNode2.hasFocus, isFalse);
        expect(controller.selection, const TextSelection.collapsed(offset: -1));

        final EditableTextState state = tester.state<EditableTextState>(
          find.byType(EditableText).first,
        );

        // Set the text editing value in order to trigger an internal call to
        // requestFocus.
        state.userUpdateTextEditingValue(controller.value, SelectionChangedCause.keyboard);
        // Focus takes a frame to update, so it hasn't changed yet.
        expect(focusNode1.hasFocus, isFalse);
        expect(focusNode2.hasFocus, isFalse);

        // Before EditableText's listener on widget.focusNode can be called, change
        // the focus again
        focusNode2.requestFocus();
        await tester.pump();
        expect(focusNode1.hasFocus, isFalse);
        expect(focusNode2.hasFocus, isTrue);

        // Focus the EditableText again, which should cause the field to be selected
        // on web.
        focusNode1.requestFocus();
        await tester.pumpAndSettle();
        expect(focusNode1.hasFocus, isTrue);
        expect(focusNode2.hasFocus, isFalse);
        expect(
          controller.selection,
          TextSelection(baseOffset: 0, extentOffset: controller.text.length),
        );
      },
      skip: !kIsWeb, // [intended]
    );

    // Regression test for https://github.com/flutter/flutter/issues/156078.
    testWidgets('when having focus regained after the app resumed', (WidgetTester tester) async {
      Future<void> setAppLifeCycleState(AppLifecycleState state) async {
        final ByteData? message = const StringCodec().encodeMessage(state.toString());
        await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
            .handlePlatformMessage('flutter/lifecycle', message, (_) {});
      }

      final TextEditingController controller = TextEditingController(text: 'Flutter!');
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: EditableText(
              key: ValueKey<String>(controller.text),
              controller: controller,
              focusNode: focusNode,
              autofocus: true,
              style: Typography.material2018().black.titleMedium!,
              cursorColor: Colors.blue,
              backgroundCursorColor: Colors.grey,
            ),
          ),
        ),
      );

      expect(focusNode.hasFocus, true);
      expect(controller.selection, collapsedAtEnd('Flutter!').selection);

      await setAppLifeCycleState(AppLifecycleState.inactive);
      await setAppLifeCycleState(AppLifecycleState.resumed);

      expect(focusNode.hasFocus, true);
      expect(controller.selection, collapsedAtEnd('Flutter!').selection);
    }, variant: TargetPlatformVariant.all());
  });

  testWidgets('EditableText respects MediaQuery.boldText', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(boldText: true),
          child: EditableText(
            controller: controller,
            focusNode: focusNode,
            style: const TextStyle(fontWeight: FontWeight.normal),
            cursorColor: Colors.red,
            backgroundCursorColor: Colors.green,
          ),
        ),
      ),
    );

    controller.text = 'foo';
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

    expect(state.buildTextSpan().style!.fontWeight, FontWeight.bold);
  });

  testWidgets(
    'code points are treated as single characters in obscure mode',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.enterText(find.byType(EditableText), '👨‍👩‍👦');
      await tester.pump();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.textEditingValue.text, '👨‍👩‍👦');
      // 👨‍👩‍👦|
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 8));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      // 👨‍👩‍|👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      // 👨‍👩|‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      // 👨‍|👩‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 3));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      // 👨|‍👩‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 2));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      // |👨‍👩‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 0));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      // 👨|‍👩‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 2));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      // 👨‍|👩‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 3));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      // 👨‍👩|‍👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      // 👨‍👩‍|👦
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      // 👨‍👩‍👦|
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 8));

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨‍👩‍');

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨‍👩');

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨‍');

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨');

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '');
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'when manually placing the cursor in the middle of a code point',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      await tester.enterText(find.byType(EditableText), '👨‍👩‍👦');
      await tester.pump();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.textEditingValue.text, '👨‍👩‍👦');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 8));

      // Place the cursor in the middle of the last code point, which consists of
      // two code units.
      await tester.tapAt(textOffsetToPosition(tester, 7));
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 7));

      // Using the arrow keys moves out of the code unit.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.tapAt(textOffsetToPosition(tester, 7));
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 7));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 8));

      // Pressing delete doesn't delete only the left code unit, it deletes the
      // entire code point (both code units, one to the left and one to the right
      // of the cursor).
      await tester.tapAt(textOffsetToPosition(tester, 7));
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 7));

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨‍👩‍');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'when inserting a malformed string',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      // This malformed string is the result of removing the final code unit from
      // the extended grapheme cluster "👨‍👩‍👦", so that the final
      // surrogate pair (the "👦" emoji or "\uD83D\uDC66"), only has its high
      // surrogate.
      await tester.enterText(find.byType(EditableText), '👨‍👩‍\uD83D');
      await tester.pump();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.textEditingValue.text, '👨‍👩‍\uD83D');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 7));

      // The dangling high surrogate is treated as a single rune.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 7));

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '👨‍👩‍');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'when inserting a malformed string that is a sequence of dangling high surrogates',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      // This string is the high surrogate from the emoji "👦" ("\uD83D\uDC66"),
      // repeated.
      await tester.enterText(find.byType(EditableText), '\uD83D\uD83D\uD83D\uD83D\uD83D\uD83D');
      await tester.pump();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.textEditingValue.text, '\uD83D\uD83D\uD83D\uD83D\uD83D\uD83D');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      // Each dangling high surrogate is treated as a single character.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '\uD83D\uD83D\uD83D\uD83D\uD83D');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'when inserting a malformed string that is a sequence of dangling low surrogates',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.tap(find.byType(EditableText));
      // This string is the low surrogate from the emoji "👦" ("\uD83D\uDC66"),
      // repeated.
      await tester.enterText(find.byType(EditableText), '\uDC66\uDC66\uDC66\uDC66\uDC66\uDC66');
      await tester.pump();

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.textEditingValue.text, '\uDC66\uDC66\uDC66\uDC66\uDC66\uDC66');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      // Each dangling high surrogate is treated as a single character.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 6));

      await tester.sendKeyEvent(LogicalKeyboardKey.backspace);
      await tester.pump();
      expect(state.textEditingValue.text, '\uDC66\uDC66\uDC66\uDC66\uDC66');
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: 5));
    },
    skip: kIsWeb, // [intended]
  );

  group('hasStrings', () {
    late int calls;
    setUp(() {
      calls = 0;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) {
          if (methodCall.method == 'Clipboard.hasStrings') {
            calls += 1;
          }
          return Future<void>.value();
        },
      );
    });
    tearDown(() {
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        null,
      );
    });

    testWidgets('web avoids the paste permissions prompt by not calling hasStrings', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            obscureText: true,
            toolbarOptions: const ToolbarOptions(
              copy: true,
              cut: true,
              paste: true,
              selectAll: true,
            ),
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      expect(calls, equals(kIsWeb ? 0 : 1));

      // Long-press to bring up the context menu.
      final Finder textFinder = find.byType(EditableText);
      await tester.longPress(textFinder);
      tester.state<EditableTextState>(textFinder).showToolbar();
      await tester.pumpAndSettle();

      expect(calls, equals(kIsWeb ? 0 : 2));
    });
  });

  testWidgets('Cursor color with an opacity is respected', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    const double opacity = 0.55;
    controller.text = 'blah blah';

    await tester.pumpWidget(
      MaterialApp(
        home: EditableText(
          key: key,
          cursorColor: cursorColor.withOpacity(opacity),
          backgroundCursorColor: Colors.grey,
          controller: controller,
          focusNode: focusNode,
          style: textStyle,
        ),
      ),
    );

    // Tap to show the cursor.
    await tester.tap(find.byKey(key));
    await tester.pumpAndSettle();

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.renderEditable.cursorColor, cursorColor.withOpacity(opacity));
  });

  testWidgets('should notify on size change', (WidgetTester tester) async {
    int notifyCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NotificationListener<SizeChangedLayoutNotification>(
            onNotification: (SizeChangedLayoutNotification notification) {
              notifyCount += 1;
              return false;
            },
            child: EditableText(
              backgroundCursorColor: Colors.grey,
              cursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              maxLines: 3,
              minLines: 1,
              style: textStyle,
            ),
          ),
        ),
      ),
    );

    expect(notifyCount, equals(0));
    await tester.enterText(find.byType(EditableText), '\n');
    await tester.pumpAndSettle();
    expect(notifyCount, equals(1));
  });

  testWidgets('ShowCaretOnScreen is correctly scheduled within a SliverMainAxisGroup', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    final Widget widget = MaterialApp(
      home: Scaffold(
        body: CustomScrollView(
          controller: scrollController,
          slivers: const <Widget>[
            SliverMainAxisGroup(
              slivers: <Widget>[
                SliverToBoxAdapter(child: SizedBox(height: 600)),
                SliverToBoxAdapter(child: SizedBox(height: 44, child: TextField())),
                SliverToBoxAdapter(child: SizedBox(height: 500)),
              ],
            ),
          ],
        ),
      ),
    );
    await tester.pumpWidget(widget);
    await tester.showKeyboard(find.byType(EditableText, skipOffstage: false));
    await tester.pumpAndSettle();
    expect(scrollController.offset, 75.0);
  });

  testWidgets(
    'getPositionForPoint is correct when EditableText is scaled',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      controller.text = 'Line1\nLine2\nLine3\nLine4\nLine5\nLine6\nLine7\nLine8';

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Transform.scale(
              scale: 0.5,
              child: EditableText(
                key: key,
                cursorColor: cursorColor,
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                maxLines: 2,
                minLines: 2,
                style: textStyle,
              ),
            ),
          ),
        ),
      );

      // With no scroll, the top left is the first character.
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
      expect(state.renderEditable.getPositionForPoint(topLeft), const TextPosition(offset: 0));

      // After scrolling to view the fourth line, the top left is the start of the
      // third line.
      state.bringIntoView(const TextPosition(offset: 18));
      await tester.pumpAndSettle();
      expect(state.renderEditable.getPositionForPoint(topLeft), const TextPosition(offset: 12));
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'selectPositionAt is correct when EditableText is scaled',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      controller.text = 'Line1\nLine2\nLine3\nLine4\nLine5\nLine6\nLine7\nLine8';

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Transform.scale(
              scale: 0.5,
              child: EditableText(
                key: key,
                cursorColor: cursorColor,
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                maxLines: 2,
                minLines: 2,
                style: textStyle,
              ),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: -1));

      // Scroll to the fourth line and select the full line above that.
      state.bringIntoView(const TextPosition(offset: 18));
      await tester.pumpAndSettle();
      state.renderEditable.selectPositionAt(
        from: topLeft,
        to: topLeft + const Offset(100.0, 0.0),
        cause: SelectionChangedCause.drag,
      );
      await tester.pumpAndSettle();
      expect(
        state.textEditingValue.selection,
        const TextSelection(baseOffset: 12, extentOffset: 17),
      );
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'selectWordsInRange is correct when EditableText is scaled',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      controller.text = 'Line1\nLine2\nLine3\nLine4\nLine5\nLine6\nLine7\nLine8';

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Transform.scale(
              scale: 0.5,
              child: EditableText(
                key: key,
                cursorColor: cursorColor,
                backgroundCursorColor: Colors.grey,
                controller: controller,
                focusNode: focusNode,
                maxLines: 2,
                minLines: 2,
                style: textStyle,
              ),
            ),
          ),
        ),
      );

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
      expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: -1));

      // Scroll to the fourth line and select the full line above that.
      state.bringIntoView(const TextPosition(offset: 18));
      await tester.pumpAndSettle();
      state.renderEditable.selectWordsInRange(
        from: topLeft,
        to: topLeft + const Offset(100.0, 0.0),
        cause: SelectionChangedCause.drag,
      );
      await tester.pumpAndSettle();
      expect(
        state.textEditingValue.selection,
        const TextSelection(baseOffset: 12, extentOffset: 17),
      );
    },
    skip: kIsWeb, // [intended]
  );

  testWidgets('selectWordEdge is correct when EditableText is scaled', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    controller.text = 'Line1\nLine2\nLine3\nLine4\nLine5\nLine6\nLine7\nLine8';

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Transform.scale(
            scale: 0.5,
            child: EditableText(
              key: key,
              cursorColor: cursorColor,
              backgroundCursorColor: Colors.grey,
              controller: controller,
              focusNode: focusNode,
              maxLines: 2,
              minLines: 2,
              style: textStyle,
            ),
          ),
        ),
      ),
    );

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    //final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: -1));

    // Scroll to the fourth line.
    state.bringIntoView(const TextPosition(offset: 18));
    await tester.pumpAndSettle();

    // Secondary tap inside of the 3rd line.
    state.renderEditable.handleSecondaryTapDown(
      TapDownDetails(globalPosition: textOffsetToPosition(tester, 13)),
    );
    expect(state.textEditingValue.selection, const TextSelection.collapsed(offset: -1));

    // selectWordEdge moves the selection to the end of the 3rd line.
    state.renderEditable.selectWordEdge(cause: SelectionChangedCause.tap);
    expect(
      state.textEditingValue.selection,
      const TextSelection.collapsed(offset: 17, affinity: TextAffinity.upstream),
    );
  });

  testWidgets('Composing region can truncate grapheme', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            autofocus: true,
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();
    assert(focusNode.hasFocus);

    controller.value = const TextEditingValue(
      text: 'Á',
      selection: TextSelection(baseOffset: 1, extentOffset: 2),
      composing: TextSelection(baseOffset: 1, extentOffset: 2),
    );
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
  });

  testWidgets('Can implement TextEditingController', (WidgetTester tester) async {
    final _TextEditingControllerImpl controller = _TextEditingControllerImpl();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: EditableText(
            autofocus: true,
            backgroundCursorColor: Colors.grey,
            controller: controller,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/159259.
  testWidgets(
    'showToolbar does nothing and returns false when already shown',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';
      final GlobalKey<EditableTextState> editableTextKey = GlobalKey();

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            key: editableTextKey,
            autofocus: true,
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
              return AdaptiveTextSelectionToolbar.editableText(
                editableTextState: editableTextState,
              );
            },
          ),
        ),
      );

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      expect(editableTextKey.currentState!.showToolbar(), isTrue);
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      expect(editableTextKey.currentState!.showToolbar(), isFalse);
      await tester.pump();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    skip: kIsWeb, // [intended]
  );
}

class UnsettableController extends TextEditingController {
  @override
  set value(TextEditingValue v) {
    // Do nothing for set, which causes selection to remain as -1, -1.
  }
}

class MockTextFormatter extends TextInputFormatter {
  MockTextFormatter() : formatCallCount = 0, log = <String>[];

  int formatCallCount;
  List<String> log;
  late TextEditingValue lastOldValue;
  late TextEditingValue lastNewValue;

  @override
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue) {
    lastOldValue = oldValue;
    lastNewValue = newValue;
    formatCallCount++;
    log.add('[$formatCallCount]: ${oldValue.text}, ${newValue.text}');
    TextEditingValue finalValue;
    if (newValue.text.length < oldValue.text.length) {
      finalValue = _handleTextDeletion(oldValue, newValue);
    } else {
      finalValue = _formatText(newValue);
    }
    return finalValue;
  }

  TextEditingValue _handleTextDeletion(TextEditingValue oldValue, TextEditingValue newValue) {
    final String result = 'a' * (formatCallCount - 2);
    log.add('[$formatCallCount]: deleting $result');
    return TextEditingValue(
      text: newValue.text,
      selection: newValue.selection,
      composing: newValue.composing,
    );
  }

  TextEditingValue _formatText(TextEditingValue value) {
    final String result = 'a' * formatCallCount * 2;
    log.add('[$formatCallCount]: normal $result');
    return TextEditingValue(
      text: value.text,
      selection: value.selection,
      composing: value.composing,
    );
  }
}

class MockTextSelectionControls extends Fake implements TextSelectionControls {
  @override
  Widget buildToolbar(
    BuildContext context,
    Rect globalEditableRegion,
    double textLineHeight,
    Offset position,
    List<TextSelectionPoint> endpoints,
    TextSelectionDelegate delegate,
    ValueListenable<ClipboardStatus>? clipboardStatus,
    Offset? lastSecondaryTapDownPosition,
  ) {
    return const SizedBox();
  }

  @override
  Widget buildHandle(
    BuildContext context,
    TextSelectionHandleType type,
    double textLineHeight, [
    VoidCallback? onTap,
  ]) {
    return const SizedBox();
  }

  @override
  Size getHandleSize(double textLineHeight) {
    return Size.zero;
  }

  @override
  Offset getHandleAnchor(TextSelectionHandleType type, double textLineHeight) {
    return Offset.zero;
  }

  bool testCanCut = false;
  bool testCanCopy = false;
  bool testCanPaste = false;

  int cutCount = 0;
  int pasteCount = 0;
  int copyCount = 0;

  @override
  void handleCopy(TextSelectionDelegate delegate) {
    copyCount += 1;
  }

  @override
  Future<void> handlePaste(TextSelectionDelegate delegate) async {
    pasteCount += 1;
  }

  @override
  void handleCut(TextSelectionDelegate delegate) {
    cutCount += 1;
  }

  @override
  bool canCut(TextSelectionDelegate delegate) {
    return testCanCut;
  }

  @override
  bool canCopy(TextSelectionDelegate delegate) {
    return testCanCopy;
  }

  @override
  bool canPaste(TextSelectionDelegate delegate) {
    return testCanPaste;
  }
}

// Fake text selection controls that call a callback when paste happens.
class _CustomTextSelectionControls extends TextSelectionControls {
  _CustomTextSelectionControls({this.onPaste, this.onCut});

  static const double _kToolbarContentDistance = 8.0;

  final VoidCallback? onPaste;
  final VoidCallback? onCut;

  @override
  Widget buildToolbar(
    BuildContext context,
    Rect globalEditableRegion,
    double textLineHeight,
    Offset position,
    List<TextSelectionPoint> endpoints,
    TextSelectionDelegate delegate,
    ValueListenable<ClipboardStatus>? clipboardStatus,
    Offset? lastSecondaryTapDownPosition,
  ) {
    final Offset selectionMidpoint = position;
    final TextSelectionPoint startTextSelectionPoint = endpoints[0];
    final TextSelectionPoint endTextSelectionPoint =
        endpoints.length > 1 ? endpoints[1] : endpoints[0];
    final Offset anchorAbove = Offset(
      globalEditableRegion.left + selectionMidpoint.dx,
      globalEditableRegion.top +
          startTextSelectionPoint.point.dy -
          textLineHeight -
          _kToolbarContentDistance,
    );
    final Offset anchorBelow = Offset(
      globalEditableRegion.left + selectionMidpoint.dx,
      globalEditableRegion.top +
          endTextSelectionPoint.point.dy +
          TextSelectionToolbar.kToolbarContentDistanceBelow,
    );
    return _CustomTextSelectionToolbar(
      anchorAbove: anchorAbove,
      anchorBelow: anchorBelow,
      handlePaste: () => handlePaste(delegate),
      handleCut: () => handleCut(delegate),
    );
  }

  @override
  Widget buildHandle(
    BuildContext context,
    TextSelectionHandleType type,
    double textLineHeight, [
    VoidCallback? onTap,
  ]) {
    return Container();
  }

  @override
  Size getHandleSize(double textLineHeight) {
    return Size.zero;
  }

  @override
  Offset getHandleAnchor(TextSelectionHandleType type, double textLineHeight) {
    return Offset.zero;
  }

  @override
  bool canCut(TextSelectionDelegate delegate) {
    return true;
  }

  @override
  bool canPaste(TextSelectionDelegate delegate) {
    return true;
  }

  @override
  Future<void> handlePaste(TextSelectionDelegate delegate) {
    onPaste?.call();
    return super.handlePaste(delegate);
  }

  @override
  void handleCut(TextSelectionDelegate delegate, [ClipboardStatusNotifier? clipboardStatus]) {
    onCut?.call();
    return super.handleCut(delegate);
  }
}

// A fake text selection toolbar with only a paste button.
class _CustomTextSelectionToolbar extends StatelessWidget {
  const _CustomTextSelectionToolbar({
    required this.anchorAbove,
    required this.anchorBelow,
    this.handlePaste,
    this.handleCut,
  });

  final Offset anchorAbove;
  final Offset anchorBelow;
  final VoidCallback? handlePaste;
  final VoidCallback? handleCut;

  @override
  Widget build(BuildContext context) {
    return TextSelectionToolbar(
      anchorAbove: anchorAbove,
      anchorBelow: anchorBelow,
      toolbarBuilder: (BuildContext context, Widget child) {
        return ColoredBox(color: Colors.pink, child: child);
      },
      children: <Widget>[
        TextSelectionToolbarTextButton(
          padding: TextSelectionToolbarTextButton.getPadding(0, 2),
          onPressed: handleCut,
          child: const Text('Cut'),
        ),
        TextSelectionToolbarTextButton(
          padding: TextSelectionToolbarTextButton.getPadding(1, 2),
          onPressed: handlePaste,
          child: const Text('Paste'),
        ),
      ],
    );
  }
}

class CustomStyleEditableText extends EditableText {
  CustomStyleEditableText({
    super.key,
    required super.controller,
    required super.cursorColor,
    required super.focusNode,
    required super.style,
  }) : super(backgroundCursorColor: Colors.grey);
  @override
  CustomStyleEditableTextState createState() => CustomStyleEditableTextState();
}

class CustomStyleEditableTextState extends EditableTextState {
  @override
  TextSpan buildTextSpan() {
    return TextSpan(
      style: const TextStyle(fontStyle: FontStyle.italic),
      text: widget.controller.value.text,
    );
  }
}

class TransformedEditableText extends StatefulWidget {
  const TransformedEditableText({
    super.key,
    required this.offset,
    required this.transformButtonKey,
  });

  final Offset offset;
  final Key transformButtonKey;

  @override
  State<TransformedEditableText> createState() => _TransformedEditableTextState();
}

class _TransformedEditableTextState extends State<TransformedEditableText> {
  bool _isTransformed = false;

  final TextEditingController _controller = TextEditingController();
  final FocusNode _focusNode = FocusNode();

  @override
  void dispose() {
    _controller.dispose();
    _focusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MediaQuery(
      data: const MediaQueryData(),
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Transform.translate(
              offset: _isTransformed ? widget.offset : Offset.zero,
              child: EditableText(
                controller: _controller,
                focusNode: _focusNode,
                style: Typography.material2018().black.titleMedium!,
                cursorColor: Colors.blue,
                backgroundCursorColor: Colors.grey,
              ),
            ),
            ElevatedButton(
              key: widget.transformButtonKey,
              onPressed: () {
                setState(() {
                  _isTransformed = !_isTransformed;
                });
              },
              child: const Text('Toggle Transform'),
            ),
          ],
        ),
      ),
    );
  }
}

class NoImplicitScrollPhysics extends AlwaysScrollableScrollPhysics {
  const NoImplicitScrollPhysics({super.parent});

  @override
  bool get allowImplicitScrolling => false;

  @override
  NoImplicitScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return NoImplicitScrollPhysics(parent: buildParent(ancestor));
  }
}

class SkipPainting extends SingleChildRenderObjectWidget {
  const SkipPainting({super.key, required Widget super.child});

  @override
  SkipPaintingRenderObject createRenderObject(BuildContext context) => SkipPaintingRenderObject();
}

class SkipPaintingRenderObject extends RenderProxyBox {
  @override
  void paint(PaintingContext context, Offset offset) {}
}

class _AccentColorTextEditingController extends TextEditingController {
  _AccentColorTextEditingController(String text) : super(text: text);

  @override
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
    SpellCheckConfiguration? spellCheckConfiguration,
  }) {
    final Color color = Theme.of(context).colorScheme.secondary;
    return super.buildTextSpan(
      context: context,
      style: TextStyle(color: color),
      withComposing: withComposing,
    );
  }
}

class _TextEditingControllerImpl extends ChangeNotifier implements TextEditingController {
  final TextEditingController _innerController = TextEditingController();

  @override
  void clear() => _innerController.clear();

  @override
  void clearComposing() => _innerController.clearComposing();

  @override
  TextSelection get selection => _innerController.selection;
  @override
  set selection(TextSelection newSelection) => _innerController.selection = newSelection;

  @override
  String get text => _innerController.text;
  @override
  set text(String newText) => _innerController.text = newText;

  @override
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
  }) {
    return _innerController.buildTextSpan(
      context: context,
      style: style,
      withComposing: withComposing,
    );
  }

  @override
  TextEditingValue get value => _innerController.value;
  @override
  set value(TextEditingValue newValue) => _innerController.value = newValue;
}

class _TestScrollController extends ScrollController {
  bool get attached => hasListeners;
}

class FakeSpellCheckService extends DefaultSpellCheckService {}

class FakeFlutterView extends TestFlutterView {
  FakeFlutterView(TestFlutterView view, {required this.viewId})
    : super(view: view, display: view.display, platformDispatcher: view.platformDispatcher);

  @override
  final int viewId;
}
