// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_util' as js_util;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:web/web.dart' as web;
import 'package:web_e2e_tests/common.dart';
import 'package:web_e2e_tests/text_editing_main.dart' as app;

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('Focused text field creates a native input element', (WidgetTester tester) async {
    app.main();
    await tester.pumpAndSettle();

    // Focus on a TextFormField.
    final Finder finder = find.byKey(const Key('input'));
    expect(finder, findsOneWidget);
    await tester.tap(find.byKey(const Key('input')));

    // A native input element will be appended to the DOM.
    final web.NodeList nodeList = findElements('input');
    expect(nodeList.length, equals(1));
    final web.HTMLInputElement input = nodeList.item(0)! as web.HTMLInputElement;
    // The element's value will be the same as the textFormField's value.
    expect(input.value, 'Text1');

    // Change the value of the TextFormField.
    final TextFormField textFormField = tester.widget(finder);
    textFormField.controller?.text = 'New Value';
    // DOM element's value also changes.
    expect(input.value, 'New Value');
  }, semanticsEnabled: false);

  testWidgets('Input field with no initial value works', (WidgetTester tester) async {
    app.main();
    await tester.pumpAndSettle();

    // Focus on a TextFormField.
    final Finder finder = find.byKey(const Key('empty-input'));
    expect(finder, findsOneWidget);
    await tester.tap(find.byKey(const Key('empty-input')));

    // A native input element will be appended to the DOM.
    final web.NodeList nodeList = findElements('input');
    expect(nodeList.length, equals(1));
    final web.HTMLInputElement input = nodeList.item(0)! as web.HTMLInputElement;
    // The element's value will be empty.
    expect(input.value, '');

    // Change the value of the TextFormField.
    final TextFormField textFormField = tester.widget(finder);
    textFormField.controller?.text = 'New Value';
    // DOM element's value also changes.
    expect(input.value, 'New Value');
  }, semanticsEnabled: false);

  testWidgets('Pressing enter on the text field triggers submit', (WidgetTester tester) async {
    app.main();
    await tester.pumpAndSettle();

    // This text will show no-enter initially. It will have 'enter-pressed'
    // after `onFieldSubmitted` of TextField is triggered.
    final Finder textFinder = find.byKey(const Key('text'));
    expect(textFinder, findsOneWidget);
    final Text text = tester.widget(textFinder);
    expect(text.data, 'no-enter');

    // Focus on a TextFormField.
    final Finder textFormFieldsFinder = find.byKey(const Key('input2'));
    expect(textFormFieldsFinder, findsOneWidget);
    await tester.tap(find.byKey(const Key('input2')));

    // Press Tab. This should trigger `onFieldSubmitted` of TextField.
    final web.HTMLInputElement input = findElements('input').item(0)! as web.HTMLInputElement;
    dispatchKeyboardEvent(input, 'keydown', <String, dynamic>{
      'keyCode': 13, // Enter.
      'cancelable': true,
    });
    // Release Tab.
    dispatchKeyboardEvent(input, 'keyup', <String, dynamic>{
      'keyCode': 13, // Enter.
      'cancelable': true,
    });

    await tester.pumpAndSettle();

    final Finder textFinder2 = find.byKey(const Key('text'));
    expect(textFinder2, findsOneWidget);
    final Text text2 = tester.widget(textFinder2);
    expect(text2.data, 'enter pressed');
  }, semanticsEnabled: false);

  testWidgets('Jump between TextFormFields with tab key', (WidgetTester tester) async {
    app.main();
    await tester.pumpAndSettle();

    // Focus on a TextFormField.
    final Finder finder = find.byKey(const Key('input'));
    expect(finder, findsOneWidget);
    await tester.tap(find.byKey(const Key('input')));

    // A native input element will be appended to the DOM.
    final web.NodeList nodeList = findElements('input');
    expect(nodeList.length, equals(1));
    final web.HTMLInputElement input = nodeList.item(0)! as web.HTMLInputElement;

    // Press Tab. The focus should move to the next TextFormField.
    dispatchKeyboardEvent(input, 'keydown', <String, dynamic>{
      'key': 'Tab',
      'code': 'Tab',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });
    // Release tab.
    dispatchKeyboardEvent(input, 'keyup', <String, dynamic>{
      'key': 'Tab',
      'code': 'Tab',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });

    await tester.pumpAndSettle();

    // A native input element for the next TextField should be attached to the
    // DOM.
    final web.HTMLInputElement input2 = findElements('input').item(0)! as web.HTMLInputElement;
    expect(input2.value, 'Text2');
  }, semanticsEnabled: false);

  testWidgets('Jump between TextFormFields with tab key after CapsLock is activated', (
    WidgetTester tester,
  ) async {
    app.main();
    await tester.pumpAndSettle();

    // Focus on a TextFormField.
    final Finder finder = find.byKey(const Key('input'));
    expect(finder, findsOneWidget);
    await tester.tap(find.byKey(const Key('input')));

    // A native input element will be appended to the DOM.
    final web.NodeList nodeList = findElements('input');
    expect(nodeList.length, equals(1));
    final web.HTMLInputElement input = nodeList.item(0)! as web.HTMLInputElement;

    // Press and release CapsLock.
    dispatchKeyboardEvent(input, 'keydown', <String, dynamic>{
      'key': 'CapsLock',
      'code': 'CapsLock',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });
    dispatchKeyboardEvent(input, 'keyup', <String, dynamic>{
      'key': 'CapsLock',
      'code': 'CapsLock',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });

    // Press Tab. The focus should move to the next TextFormField.
    dispatchKeyboardEvent(input, 'keydown', <String, dynamic>{
      'key': 'Tab',
      'code': 'Tab',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });
    // Release Tab.
    dispatchKeyboardEvent(input, 'keyup', <String, dynamic>{
      'key': 'Tab',
      'code': 'Tab',
      'bubbles': true,
      'cancelable': true,
      'composed': true,
    });

    await tester.pumpAndSettle();

    // A native input element for the next TextField should be attached to the
    // DOM.
    final web.HTMLInputElement input2 = findElements('input').item(0)! as web.HTMLInputElement;
    expect(input2.value, 'Text2');
  }, semanticsEnabled: false);

  testWidgets('Read-only fields work', (WidgetTester tester) async {
    const String text = 'Lorem ipsum dolor sit amet';
    app.main();
    await tester.pumpAndSettle();

    // Select something from the selectable text.
    final Finder finder = find.byKey(const Key('selectable'));
    expect(finder, findsOneWidget);
    final RenderBox selectable = tester.renderObject(finder);
    final Offset topLeft = selectable.localToGlobal(Offset.zero);
    final Offset topRight = selectable.localToGlobal(Offset(selectable.size.width, 0.0));

    // Drag by mouse to select the entire selectable text.
    TestGesture gesture = await tester.startGesture(topLeft, kind: PointerDeviceKind.mouse);
    await gesture.moveTo(topRight);
    await gesture.up();

    // A native input element will be appended to the DOM.
    final web.NodeList nodeList = findElements('textarea');
    expect(nodeList.length, equals(1));
    final web.HTMLTextAreaElement input = nodeList.item(0)! as web.HTMLTextAreaElement;
    // The element's value should contain the selectable text.
    expect(input.value, text);
    expect(input.hasAttribute('readonly'), isTrue);

    // Make sure the entire text is selected.
    TextRange? range = TextRange(start: input.selectionStart, end: input.selectionEnd);
    expect(range.textInside(text), text);

    // Double tap to select the first word.
    final Offset firstWordOffset = topLeft.translate(10.0, 0.0);
    gesture = await tester.startGesture(firstWordOffset, kind: PointerDeviceKind.mouse);
    await gesture.up();
    await gesture.down(firstWordOffset);
    await gesture.up();
    range = TextRange(start: input.selectionStart, end: input.selectionEnd);
    expect(range.textInside(text), 'Lorem');

    // Double tap to select the last word.
    final Offset lastWordOffset = topRight.translate(-10.0, 0.0);
    gesture = await tester.startGesture(lastWordOffset, kind: PointerDeviceKind.mouse);
    await gesture.up();
    await gesture.down(lastWordOffset);
    await gesture.up();
    range = TextRange(start: input.selectionStart, end: input.selectionEnd);
    expect(range.textInside(text), 'amet');
  }, semanticsEnabled: false);
}

web.KeyboardEvent dispatchKeyboardEvent(
  web.EventTarget target,
  String type,
  Map<String, dynamic> args,
) {
  final Object jsKeyboardEvent = js_util.getProperty(web.window, 'KeyboardEvent') as Object;
  final List<dynamic> eventArgs = <dynamic>[type, args];
  final web.KeyboardEvent event =
      js_util.callConstructor(jsKeyboardEvent, js_util.jsify(eventArgs) as List<dynamic>)
          as web.KeyboardEvent;
  target.dispatchEvent(event);

  return event;
}
