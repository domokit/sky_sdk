// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(gspencergoog): Remove this tag once this test's state leaks/test
// dependencies have been fixed.
// https://github.com/flutter/flutter/issues/85160
// Fails with "flutter test --test-randomize-ordering-seed=20210721"
@Tags(<String>['no-shuffle'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('enterText works', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: Material(child: TextField())));

    final EditableTextState state = tester.state(find.byType(EditableText));
    expect(state.textEditingValue.text, '');

    await tester.enterText(find.byType(EditableText), 'let there be text');
    expect(state.textEditingValue.text, 'let there be text');
    expect(state.textEditingValue.selection.isCollapsed, isTrue);
    expect(state.textEditingValue.selection.baseOffset, 17);
  });

  testWidgets('receiveAction() forwards exception when exception occurs during action processing', (
    WidgetTester tester,
  ) async {
    // Setup a widget that can receive focus so that we can open the keyboard.
    const Widget widget = MaterialApp(home: Material(child: TextField()));
    await tester.pumpWidget(widget);

    // Keyboard must be shown for receiveAction() to function.
    await tester.showKeyboard(find.byType(TextField));

    // Register a handler for the text input channel that throws an error. This
    // error should be reported within a PlatformException by TestTextInput.
    SystemChannels.textInput.setMethodCallHandler((MethodCall call) {
      throw FlutterError('A fake error occurred during action processing.');
    });

    await expectLater(
      () => tester.testTextInput.receiveAction(TextInputAction.done),
      throwsA(isA<PlatformException>()),
    );
  });

  testWidgets('selectors are called on macOS', (WidgetTester tester) async {
    List<dynamic>? selectorNames;
    await SystemChannels.textInput.invokeMethod('TextInput.setClient', <dynamic>[
      1,
      <String, dynamic>{},
    ]);
    await SystemChannels.textInput.invokeMethod('TextInput.show');
    SystemChannels.textInput.setMethodCallHandler((MethodCall call) async {
      if (call.method == 'TextInputClient.performSelectors') {
        selectorNames = (call.arguments as List<dynamic>)[1] as List<dynamic>;
      }
    });
    await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
    await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowUp);
    await SystemChannels.textInput.invokeMethod('TextInput.clearClient');

    if (defaultTargetPlatform == TargetPlatform.macOS) {
      expect(selectorNames, <dynamic>['moveBackward:', 'moveToBeginningOfParagraph:']);
    } else {
      expect(selectorNames, isNull);
    }
  }, variant: TargetPlatformVariant.all());

  testWidgets('selector is called for ctrl + backspace on macOS', (WidgetTester tester) async {
    List<dynamic>? selectorNames;
    await SystemChannels.textInput.invokeMethod('TextInput.setClient', <dynamic>[
      1,
      <String, dynamic>{},
    ]);
    await SystemChannels.textInput.invokeMethod('TextInput.show');
    SystemChannels.textInput.setMethodCallHandler((MethodCall call) async {
      if (call.method == 'TextInputClient.performSelectors') {
        selectorNames = (call.arguments as List<dynamic>)[1] as List<dynamic>;
      }
    });
    await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
    await tester.sendKeyDownEvent(LogicalKeyboardKey.backspace);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.backspace);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
    await SystemChannels.textInput.invokeMethod('TextInput.clearClient');

    if (defaultTargetPlatform == TargetPlatform.macOS) {
      expect(selectorNames, <dynamic>['deleteBackwardByDecomposingPreviousCharacter:']);
    } else {
      expect(selectorNames, isNull);
    }
  }, variant: TargetPlatformVariant.all());
}
