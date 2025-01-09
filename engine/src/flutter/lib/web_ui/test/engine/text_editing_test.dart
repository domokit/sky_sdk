// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_util' as js_util;
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/spy.dart';
import '../common/test_initialization.dart';

/// The `keyCode` of the "Enter" key.
const int _kReturnKeyCode = 13;

const MethodCodec codec = JSONMethodCodec();

EnginePlatformDispatcher get dispatcher => EnginePlatformDispatcher.instance;

DomElement get defaultTextEditingRoot => dispatcher.implicitView!.dom.textEditingHost;

DomElement get implicitViewRootElement => dispatcher.implicitView!.dom.rootElement;

/// Add unit tests for [FirefoxTextEditingStrategy].
// TODO(mdebbar): https://github.com/flutter/flutter/issues/46891

DefaultTextEditingStrategy? editingStrategy;
EditingState? lastEditingState;
TextEditingDeltaState? editingDeltaState;
String? lastInputAction;

final InputConfiguration singlelineConfig = InputConfiguration(viewId: kImplicitViewId);
final Map<String, dynamic> flutterSinglelineConfig = createFlutterConfig('text');

final InputConfiguration multilineConfig = InputConfiguration(
  viewId: kImplicitViewId,
  inputType: EngineInputType.multiline,
  inputAction: 'TextInputAction.newline',
);
final Map<String, dynamic> flutterMultilineConfig = createFlutterConfig('multiline');

void trackEditingState(EditingState? editingState, TextEditingDeltaState? textEditingDeltaState) {
  lastEditingState = editingState;
  editingDeltaState = textEditingDeltaState;
}

void trackInputAction(String? inputAction) {
  lastInputAction = inputAction;
}

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpImplicitView();

  setUp(() {
    domDocument.activeElement?.blur();
  });

  tearDown(() async {
    lastEditingState = null;
    editingDeltaState = null;
    lastInputAction = null;
    cleanTextEditingStrategy();
    cleanTestFlags();
    clearBackUpDomElementIfExists();
    await waitForTextStrategyStopPropagation();
  });

  group('$GloballyPositionedTextEditingStrategy', () {
    late HybridTextEditing testTextEditing;

    setUp(() {
      testTextEditing = HybridTextEditing();
      editingStrategy = GloballyPositionedTextEditingStrategy(testTextEditing);
      testTextEditing.debugTextEditingStrategyOverride = editingStrategy;
      testTextEditing.configuration = singlelineConfig;
    });

    test('Creates element when enabled and removes it when disabled', () async {
      expect(domDocument.getElementsByTagName('input'), hasLength(0));
      expect(
        domDocument.activeElement,
        domDocument.body,
        reason: 'The focus should initially be on the body',
      );
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, domDocument.body);

      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      // Now the editing element should have focus.

      expect(domDocument.activeElement, input);
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, input);

      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('type'), null);
      expect(input.tabIndex, -1, reason: 'The input should not be reachable by keyboard');

      // Input is appended to the right point of the DOM.
      expect(defaultTextEditingRoot.contains(editingStrategy!.domElement), isTrue);

      editingStrategy!.disable();
      await waitForTextStrategyStopPropagation();
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(0));
      // The focus is back to the flutter view.
      expect(domDocument.activeElement, implicitViewRootElement);
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, implicitViewRootElement);
    });

    test('inserts element in the correct view', () async {
      final DomElement host = createDomElement('div');
      domDocument.body!.append(host);
      final EngineFlutterView view = EngineFlutterView(dispatcher, host);
      dispatcher.viewManager.registerView(view);
      final DomElement textEditingHost = view.dom.textEditingHost;

      expect(domDocument.getElementsByTagName('input'), hasLength(0));
      expect(textEditingHost.getElementsByTagName('input'), hasLength(0));

      final InputConfiguration config = InputConfiguration(viewId: view.viewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      final DomElement input = editingStrategy!.domElement!;

      // Input is appended to the right view.
      expect(textEditingHost.contains(input), isTrue);

      // Cleanup.
      editingStrategy!.disable();
      await waitForTextStrategyStopPropagation();
      expect(textEditingHost.querySelectorAll('input'), hasLength(0));
      dispatcher.viewManager.unregisterView(view.viewId);
      view.dispose();
      host.remove();
    });

    test('Respects read-only config', () {
      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId, readOnly: true);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('readonly'), 'readonly');

      editingStrategy!.disable();
    });

    test('Knows how to create password fields', () {
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        obscureText: true,
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('type'), 'password');

      editingStrategy!.disable();
    });

    test('Knows how to create non-default text actions', () {
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        inputAction: 'TextInputAction.send',
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs ||
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.android) {
        expect(input.getAttribute('enterkeyhint'), 'send');
      } else {
        expect(input.getAttribute('enterkeyhint'), null);
      }

      editingStrategy!.disable();
    });

    test('Knows to turn autocorrect off', () {
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        autocorrect: false,
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('autocorrect'), 'off');

      editingStrategy!.disable();
    });

    test('Knows to turn autocorrect on', () {
      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('autocorrect'), 'on');

      editingStrategy!.disable();
    });

    test('Knows to turn autofill off', () {
      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      final DomElement input = defaultTextEditingRoot.querySelector('input')!;
      expect(editingStrategy!.domElement, input);
      expect(input.getAttribute('autocomplete'), 'off');

      editingStrategy!.disable();
    });

    test('Can read editing state correctly', () {
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      final DomHTMLInputElement input = editingStrategy!.domElement! as DomHTMLInputElement;
      input.value = 'foo bar';
      input.dispatchEvent(createDomEvent('Event', 'input'));
      expect(lastEditingState, EditingState(text: 'foo bar', baseOffset: 7, extentOffset: 7));

      input.setSelectionRange(4, 6);
      domDocument.dispatchEvent(createDomEvent('Event', 'selectionchange'));
      expect(lastEditingState, EditingState(text: 'foo bar', baseOffset: 4, extentOffset: 6));

      // There should be no input action.
      expect(lastInputAction, isNull);
    });

    test('Can set editing state correctly', () {
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      editingStrategy!.setEditingState(
        EditingState(text: 'foo bar baz', baseOffset: 2, extentOffset: 7),
      );

      checkInputEditingState(editingStrategy!.domElement, 'foo bar baz', 2, 7);

      // There should be no input action.
      expect(lastInputAction, isNull);
    });

    test('Multi-line mode also works', () async {
      // The textarea element is created lazily.
      expect(domDocument.getElementsByTagName('textarea'), hasLength(0));
      editingStrategy!.enable(
        multilineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(1));

      final DomHTMLTextAreaElement textarea =
          defaultTextEditingRoot.querySelector('textarea')! as DomHTMLTextAreaElement;
      // Now the textarea should have focus.
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, textarea);
      expect(editingStrategy!.domElement, textarea);

      textarea.value = 'foo\nbar';
      textarea.dispatchEvent(createDomEvent('Event', 'input'));
      textarea.setSelectionRange(4, 6);
      domDocument.dispatchEvent(createDomEvent('Event', 'selectionchange'));
      // Can read textarea state correctly (and preserves new lines).
      expect(lastEditingState, EditingState(text: 'foo\nbar', baseOffset: 4, extentOffset: 6));

      // Can set textarea state correctly (and preserves new lines).
      editingStrategy!.setEditingState(
        EditingState(text: 'bar\nbaz', baseOffset: 2, extentOffset: 7),
      );
      checkTextAreaEditingState(textarea, 'bar\nbaz', 2, 7);

      editingStrategy!.disable();

      await waitForTextStrategyStopPropagation();

      // The textarea should be cleaned up.
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(0));

      expect(
        defaultTextEditingRoot.ownerDocument?.activeElement,
        implicitViewRootElement,
        reason: 'The focus should be back to the body',
      );

      // There should be no input action.
      expect(lastInputAction, isNull);
    });

    test('Same instance can be re-enabled with different config', () async {
      // Make sure there's nothing in the DOM yet.
      expect(domDocument.getElementsByTagName('input'), hasLength(0));
      expect(domDocument.getElementsByTagName('textarea'), hasLength(0));

      // Use single-line config and expect an `<input>` to be created.
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(1));
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(0));

      // Disable and check that all DOM elements were removed.
      editingStrategy!.disable();
      await waitForTextStrategyStopPropagation();
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(0));
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(0));

      // Use multi-line config and expect an `<textarea>` to be created.
      editingStrategy!.enable(
        multilineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      await waitForTextStrategyStopPropagation();
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(0));
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(1));

      // Disable again and check that all DOM elements were removed.
      editingStrategy!.disable();
      await waitForTextStrategyStopPropagation();
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(0));
      expect(defaultTextEditingRoot.querySelectorAll('textarea'), hasLength(0));

      // There should be no input action.
      expect(lastInputAction, isNull);
    });

    test('Triggers input action', () {
      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      dispatchKeyboardEvent(editingStrategy!.domElement!, 'keydown', keyCode: _kReturnKeyCode);
      expect(lastInputAction, 'TextInputAction.done');
    });

    test('handling keyboard event prevents triggering input action', () {
      final ui.PlatformMessageCallback? savedCallback = dispatcher.onPlatformMessage;

      bool markTextEventHandled = false;
      dispatcher.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        final ByteData response =
            const JSONMessageCodec().encodeMessage(<String, dynamic>{
              'handled': markTextEventHandled,
            })!;
        callback!(response);
      };
      RawKeyboard.initialize();

      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      markTextEventHandled = true;
      dispatchKeyboardEvent(editingStrategy!.domElement!, 'keydown', keyCode: _kReturnKeyCode);

      // Input action prevented by platform message callback.
      expect(lastInputAction, isNull);

      markTextEventHandled = false;
      dispatchKeyboardEvent(editingStrategy!.domElement!, 'keydown', keyCode: _kReturnKeyCode);

      // Input action received.
      expect(lastInputAction, 'TextInputAction.done');

      dispatcher.onPlatformMessage = savedCallback;
      RawKeyboard.instance?.dispose();
    });

    test('Triggers input action in multi-line mode', () {
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        inputType: EngineInputType.multiline,
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      final DomKeyboardEvent event = dispatchKeyboardEvent(
        editingStrategy!.domElement!,
        'keydown',
        keyCode: _kReturnKeyCode,
      );

      // Input action is triggered!
      expect(lastInputAction, 'TextInputAction.done');
      // And default behavior of keyboard event should have been prevented.
      // Only TextInputAction.newline should not prevent default behavior
      // for a multiline field.
      expect(event.defaultPrevented, isTrue);
    });

    test('Does not prevent default behavior when TextInputAction.newline', () {
      // Regression test for https://github.com/flutter/flutter/issues/145051.
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        inputAction: 'TextInputAction.newline',
        inputType: EngineInputType.multilineNone,
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      final DomKeyboardEvent event = dispatchKeyboardEvent(
        editingStrategy!.domElement!,
        'keydown',
        keyCode: _kReturnKeyCode,
      );
      expect(lastInputAction, 'TextInputAction.newline');
      // And default behavior of keyboard event should't have been prevented.
      expect(event.defaultPrevented, isFalse);
    });

    test('Triggers input action in multiline-none mode', () {
      final InputConfiguration config = InputConfiguration(
        viewId: kImplicitViewId,
        inputType: EngineInputType.multilineNone,
      );
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      final DomKeyboardEvent event = dispatchKeyboardEvent(
        editingStrategy!.domElement!,
        'keydown',
        keyCode: _kReturnKeyCode,
      );

      // Input action is triggered!
      expect(lastInputAction, 'TextInputAction.done');
      // And default behavior of keyboard event should have been prevented.
      // Only TextInputAction.newline should not prevent default behavior
      // for a multiline field.
      expect(event.defaultPrevented, isTrue);
    });

    test('Triggers input action and prevent new line key event for single line field', () {
      // Regression test for https://github.com/flutter/flutter/issues/113559
      final InputConfiguration config = InputConfiguration(viewId: kImplicitViewId);
      editingStrategy!.enable(config, onChange: trackEditingState, onAction: trackInputAction);

      // No input action so far.
      expect(lastInputAction, isNull);

      final DomKeyboardEvent event = dispatchKeyboardEvent(
        editingStrategy!.domElement!,
        'keydown',
        keyCode: _kReturnKeyCode,
      );
      expect(lastInputAction, 'TextInputAction.done');
      // And default behavior of keyboard event should have been prevented.
      expect(event.defaultPrevented, isTrue);
    });

    test('globally positions and sizes its DOM element', () {
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      expect(editingStrategy!.isEnabled, isTrue);

      // No geometry should be set until setEditableSizeAndTransform is called.
      expect(editingStrategy!.domElement!.style.transform, '');
      expect(editingStrategy!.domElement!.style.width, '');
      expect(editingStrategy!.domElement!.style.height, '');

      testTextEditing.acceptCommand(
        TextInputSetEditableSizeAndTransform(
          geometry: EditableTextGeometry(
            width: 13,
            height: 12,
            globalTransform: Matrix4.translationValues(14, 15, 0).storage,
          ),
        ),
        () {},
      );

      // setEditableSizeAndTransform calls placeElement, so expecting geometry to be applied.
      expect(editingStrategy!.domElement!.style.transform, 'matrix(1, 0, 0, 1, 14, 15)');
      expect(editingStrategy!.domElement!.style.width, '13px');
      expect(editingStrategy!.domElement!.style.height, '12px');
    });

    test('updateElementPlacement() should not call placeElement() when in mid-composition', () {
      final HybridTextEditing testTextEditing = HybridTextEditing();
      final GlobalTextEditingStrategySpy editingStrategy = GlobalTextEditingStrategySpy(
        testTextEditing,
      );
      testTextEditing.debugTextEditingStrategyOverride = editingStrategy;
      testTextEditing.configuration = singlelineConfig;

      editingStrategy.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      expect(editingStrategy.isEnabled, isTrue);

      // placeElement() called from enable()
      expect(editingStrategy.placeElementCount, 1);

      // No geometry should be set until setEditableSizeAndTransform is called.
      expect(editingStrategy.domElement!.style.transform, '');
      expect(editingStrategy.domElement!.style.width, '');
      expect(editingStrategy.domElement!.style.height, '');

      // set some composing text.
      editingStrategy.composingText = '뮤';

      testTextEditing.acceptCommand(
        TextInputSetEditableSizeAndTransform(
          geometry: EditableTextGeometry(
            width: 13,
            height: 12,
            globalTransform: Matrix4.translationValues(14, 15, 0).storage,
          ),
        ),
        () {},
      );

      // placeElement() should not be called again.
      expect(editingStrategy.placeElementCount, 1);

      // geometry should be applied.
      expect(editingStrategy.domElement!.style.transform, 'matrix(1, 0, 0, 1, 14, 15)');
      expect(editingStrategy.domElement!.style.width, '13px');
      expect(editingStrategy.domElement!.style.height, '12px');

      // set composing text to null.
      editingStrategy.composingText = null;

      testTextEditing.acceptCommand(
        TextInputSetEditableSizeAndTransform(
          geometry: EditableTextGeometry(
            width: 10,
            height: 10,
            globalTransform: Matrix4.translationValues(11, 12, 0).storage,
          ),
        ),
        () {},
      );

      // placeElement() should be called again.
      expect(editingStrategy.placeElementCount, 2);

      // geometry should be updated.
      expect(editingStrategy.domElement!.style.transform, 'matrix(1, 0, 0, 1, 11, 12)');
      expect(editingStrategy.domElement!.style.width, '10px');
      expect(editingStrategy.domElement!.style.height, '10px');
    });
  });

  group('$HybridTextEditing', () {
    HybridTextEditing? textEditing;
    final PlatformMessagesSpy spy = PlatformMessagesSpy();

    int clientId = 0;

    /// Emulates sending of a message by the framework to the engine.
    void sendFrameworkMessage(ByteData? message) {
      textEditing!.channel.handleTextInput(message, (ByteData? data) {});
    }

    /// Sends the necessary platform messages to activate a text field and show
    /// the keyboard.
    ///
    /// Returns the `clientId` used in the platform message.
    int showKeyboard({
      required String inputType,
      int? viewId,
      String? inputAction,
      bool decimal = false,
      bool isMultiline = false,
      bool autofillEnabled = true,
    }) {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        ++clientId,
        createFlutterConfig(
          inputType,
          viewId: viewId,
          inputAction: inputAction,
          decimal: decimal,
          isMultiline: isMultiline,
          autofillEnabled: autofillEnabled,
        ),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      return clientId;
    }

    void hideKeyboard() {
      const MethodCall hide = MethodCall('TextInput.hide');
      sendFrameworkMessage(codec.encodeMethodCall(hide));

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));
    }

    String? getEditingInputMode() {
      return textEditing!.strategy.domElement!.getAttribute('inputmode');
    }

    setUp(() {
      textEditing = HybridTextEditing();
      spy.setUp();
    });

    tearDown(() {
      spy.tearDown();
      if (textEditing!.isEditing) {
        textEditing!.stopEditing();
      }
      textEditing = null;
    });

    test('TextInput.requestAutofill', () async {
      const MethodCall requestAutofill = MethodCall('TextInput.requestAutofill');
      sendFrameworkMessage(codec.encodeMethodCall(requestAutofill));

      //No-op and without crashing.
    });

    test('setClient, show, setEditingState, hide', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      // Editing shouldn't have started yet.
      expect(domDocument.activeElement, domDocument.body);

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      checkInputEditingState(textEditing!.strategy.domElement, '', 0, 0);

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      const MethodCall hide = MethodCall('TextInput.hide');
      sendFrameworkMessage(codec.encodeMethodCall(hide));

      await waitForTextStrategyStopPropagation();

      expect(
        domDocument.activeElement,
        implicitViewRootElement,
        reason: 'Text editing should have stopped',
      );

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
    });

    test('setClient, setEditingState, show, clearClient', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      expect(
        domDocument.activeElement,
        domDocument.body,
        reason: 'Editing should not have started yet',
      );

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      await waitForTextStrategyStopPropagation();

      expect(
        domDocument.activeElement,
        implicitViewRootElement,
        reason: 'Text editing should have stopped',
      );

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
    });

    test(
      'setClient, setEditingState, setSizeAndTransform, show - input element is put into the DOM Safari Desktop',
      () async {
        editingStrategy = SafariDesktopTextEditingStrategy(textEditing!);
        textEditing!.debugTextEditingStrategyOverride = editingStrategy;
        final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
          123,
          flutterSinglelineConfig,
        ]);
        sendFrameworkMessage(codec.encodeMethodCall(setClient));

        const MethodCall show = MethodCall('TextInput.show');
        sendFrameworkMessage(codec.encodeMethodCall(show));

        // Editing shouldn't have started yet.
        expect(domDocument.activeElement, domDocument.body);

        // The "setSizeAndTransform" message has to be here before we call
        // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
        // we don't put the input element into the DOM until we get its correct
        // dimensions from the framework.
        final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
          150,
          50,
          Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
        );
        sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

        const MethodCall setEditingState = MethodCall(
          'TextInput.setEditingState',
          <String, dynamic>{'text': 'abcd', 'selectionBase': 2, 'selectionExtent': 3},
        );
        sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

        expect(
          defaultTextEditingRoot.ownerDocument?.activeElement,
          textEditing!.strategy.domElement,
        );
      },
      skip: !isSafari,
    );

    test('setClient, setEditingState, show, updateConfig, clearClient', () {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('text', readOnly: true),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      final DomElement element = textEditing!.strategy.domElement!;
      expect(element.getAttribute('readonly'), 'readonly');

      // Update the read-only config.
      final MethodCall updateConfig = MethodCall(
        'TextInput.updateConfig',
        createFlutterConfig('text'),
      );
      sendFrameworkMessage(codec.encodeMethodCall(updateConfig));

      expect(element.hasAttribute('readonly'), isFalse);

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
    });

    test('focus and connection with blur', () async {
      // In all the desktop browsers we are keeping the connection
      // open, keep the text editing element focused if it receives a blur
      // event.
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      expect(
        defaultTextEditingRoot.ownerDocument?.activeElement,
        domDocument.body,
        reason: 'Editing should not have started yet',
      );

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);
      expect(textEditing!.isEditing, isTrue);

      // No connection close message sent.
      expect(spy.messages, hasLength(0));
      await Future<void>.delayed(Duration.zero);

      // DOM element still keeps the focus.
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, textEditing!.strategy.domElement);
    });

    test(
      'focus and disconnection with delaying blur in iOS',
      () async {
        final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
          123,
          flutterSinglelineConfig,
        ]);
        sendFrameworkMessage(codec.encodeMethodCall(setClient));

        const MethodCall setEditingState = MethodCall(
          'TextInput.setEditingState',
          <String, dynamic>{'text': 'abcd', 'selectionBase': 2, 'selectionExtent': 3},
        );
        sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

        // Editing shouldn't have started yet.
        expect(defaultTextEditingRoot.ownerDocument?.activeElement, domDocument.body);

        const MethodCall show = MethodCall('TextInput.show');
        sendFrameworkMessage(codec.encodeMethodCall(show));

        // The "setSizeAndTransform" message has to be here before we call
        // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
        // we don't put the input element into the DOM until we get its correct
        // dimensions from the framework.
        final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
          150,
          50,
          Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
        );
        sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

        checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);
        expect(textEditing!.isEditing, isTrue);

        // Delay for not to be a fast callback with blur.
        await Future<void>.delayed(const Duration(milliseconds: 200));
        // DOM element is blurred.
        textEditing!.strategy.domElement!.blur();

        expect(spy.messages, hasLength(1));
        expect(spy.messages[0].channel, 'flutter/textinput');
        expect(spy.messages[0].methodName, 'TextInputClient.onConnectionClosed');
        await Future<void>.delayed(Duration.zero);
        // DOM element loses the focus.
        expect(defaultTextEditingRoot.ownerDocument?.activeElement, domDocument.body);
      },
      // Test on ios-safari only.
      skip:
          ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit ||
          ui_web.browser.operatingSystem != ui_web.OperatingSystem.iOs,
    );

    test('finishAutofillContext closes connection no autofill element', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      // Editing shouldn't have started yet.
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, domDocument.body);

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      const MethodCall finishAutofillContext = MethodCall('TextInput.finishAutofillContext', false);
      sendFrameworkMessage(codec.encodeMethodCall(finishAutofillContext));

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.onConnectionClosed');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
      ]);
      spy.messages.clear();
      // Input element is removed from DOM.
      expect(defaultTextEditingRoot.querySelectorAll('input'), hasLength(0));
    });

    test('finishAutofillContext removes form from DOM', () async {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name', 'telephoneNumber'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The transform is changed. For example after a validation error, red
      // line appeared under the input field.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // Form is added to DOM.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
      // Form stays on the DOM until autofill context is finalized.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      expect(formsOnTheDom, hasLength(1));

      const MethodCall finishAutofillContext = MethodCall('TextInput.finishAutofillContext', false);
      sendFrameworkMessage(codec.encodeMethodCall(finishAutofillContext));

      // Form element is removed from DOM.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isEmpty);
      expect(formsOnTheDom, hasLength(0));
    });

    test('finishAutofillContext with save submits forms', () async {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name', 'telephoneNumber'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The transform is changed. For example after a validation error, red
      // line appeared under the input field.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // Form is added to DOM.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      final DomHTMLFormElement formElement =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      final Completer<bool> submittedForm = Completer<bool>();
      formElement.addEventListener(
        'submit',
        createDomEventListener((DomEvent event) => submittedForm.complete(true)),
      );

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      const MethodCall finishAutofillContext = MethodCall('TextInput.finishAutofillContext', true);
      sendFrameworkMessage(codec.encodeMethodCall(finishAutofillContext));

      // `submit` action is called on form.
      await expectLater(await submittedForm.future, isTrue);
    });

    test('forms submits for focused input', () async {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name', 'telephoneNumber'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The transform is changed. For example after a validation error, red
      // line appeared under the input field.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // Form is added to DOM.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      final DomHTMLFormElement formElement =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      final Completer<bool> submittedForm = Completer<bool>();
      formElement.addEventListener(
        'submit',
        createDomEventListener((DomEvent event) => submittedForm.complete(true)),
      );

      // Clear client is not called. The used requested context to be finalized.
      const MethodCall finishAutofillContext = MethodCall('TextInput.finishAutofillContext', true);
      sendFrameworkMessage(codec.encodeMethodCall(finishAutofillContext));

      // Connection is closed by the engine.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.onConnectionClosed');

      // `submit` action is called on form.
      await expectLater(await submittedForm.future, isTrue);
      // Form element is removed from DOM.
      expect(defaultTextEditingRoot.querySelectorAll('form'), hasLength(0));
      expect(formsOnTheDom, hasLength(0));
    });

    test('Moves the focus across input elements', () async {
      final List<DomEvent> focusinEvents = <DomEvent>[];
      final DomEventListener handleFocusIn = createDomEventListener(focusinEvents.add);

      final MethodCall setClient1 = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      final MethodCall setClient2 = MethodCall('TextInput.setClient', <dynamic>[
        567,
        flutterSinglelineConfig,
      ]);
      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      const MethodCall show = MethodCall('TextInput.show');
      const MethodCall clearClient = MethodCall('TextInput.clearClient');

      domDocument.body!.addEventListener('focusin', handleFocusIn);
      sendFrameworkMessage(codec.encodeMethodCall(setClient1));
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));
      sendFrameworkMessage(codec.encodeMethodCall(show));
      final DomElement firstInput = textEditing!.strategy.domElement!;
      expect(domDocument.activeElement, firstInput);

      sendFrameworkMessage(codec.encodeMethodCall(setClient2));
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));
      sendFrameworkMessage(codec.encodeMethodCall(show));
      final DomElement secondInput = textEditing!.strategy.domElement!;
      expect(domDocument.activeElement, secondInput);
      expect(firstInput, isNot(secondInput));

      sendFrameworkMessage(codec.encodeMethodCall(clearClient));
      await waitForTextStrategyStopPropagation();
      domDocument.body!.removeEventListener('focusin', handleFocusIn);

      expect(focusinEvents, hasLength(3));
      expect(focusinEvents[0].target, firstInput);
      expect(focusinEvents[1].target, secondInput);
      expect(focusinEvents[2].target, implicitViewRootElement);
    });

    test('setClient, setEditingState, show, setClient', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      expect(
        domDocument.activeElement,
        domDocument.body,
        reason: 'Editing should not have started yet.',
      );

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      final MethodCall setClient2 = MethodCall('TextInput.setClient', <dynamic>[
        567,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient2));

      await waitForTextStrategyStopPropagation();

      expect(
        domDocument.activeElement,
        implicitViewRootElement,
        reason:
            'Receiving another client via setClient should stop editing, hence should remove the previous active element.',
      );

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);

      hideKeyboard();
    });

    test('setClient, setEditingState, show, setEditingState, clearClient', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      const MethodCall setEditingState2 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'xyz',
        'selectionBase': 0,
        'selectionExtent': 2,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState2));

      // The second [setEditingState] should override the first one.
      checkInputEditingState(textEditing!.strategy.domElement, 'xyz', 0, 2);

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
    });

    test('singleTextField Autofill: setClient, setEditingState, show, '
        'setSizeAndTransform, setEditingState, clearClient', () async {
      // Create a configuration with focused element has autofil hint.
      final Map<String, dynamic> flutterSingleAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSingleAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // The second [setEditingState] should override the first one.
      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      final DomHTMLFormElement formElement =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      // The form has one input element and one submit button.
      expect(formElement.childNodes, hasLength(2));

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
      // Form stays on the DOM until autofill context is finalized.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      expect(formsOnTheDom, hasLength(1));
    });

    test('singleTextField Autofill setEditableSizeAndTransform preserves'
        'editing state', () async {
      // Create a configuration with focused element has autofil hint.
      final Map<String, dynamic> flutterSingleAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSingleAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        10,
        10,
        Matrix4.translationValues(10.0, 10.0, 10.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomHTMLInputElement inputElement =
          textEditing!.strategy.domElement! as DomHTMLInputElement;
      expect(inputElement.value, 'abcd');
      if (!(ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.macOs)) {
        // In Safari Desktop Autofill menu appears as soon as an element is
        // focused, therefore the input element is only focused after the
        // location is received.
        expect(defaultTextEditingRoot.ownerDocument?.activeElement, inputElement);
        expect(inputElement.selectionStart, 2);
        expect(inputElement.selectionEnd, 3);
      }

      // The transform is changed. For example after a validation error, red
      // line appeared under the input field.
      final MethodCall updateSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(updateSizeAndTransform));

      // Check the element still has focus. User can keep editing.
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, textEditing!.strategy.domElement);

      // Check the cursor location is the same.
      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
      // Form stays on the DOM until autofill context is finalized.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      expect(formsOnTheDom, hasLength(1));
    });

    test('multiTextField Autofill: setClient, setEditingState, show, '
        'setSizeAndTransform setEditingState, clearClient', () async {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name', 'telephoneNumber'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // The second [setEditingState] should override the first one.
      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      final DomHTMLFormElement formElement =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      // The form has 4 input elements and one submit button.
      expect(formElement.childNodes, hasLength(5));

      const MethodCall clearClient = MethodCall('TextInput.clearClient');
      sendFrameworkMessage(codec.encodeMethodCall(clearClient));

      // Confirm that [HybridTextEditing] didn't send any messages.
      expect(spy.messages, isEmpty);
      // Form stays on the DOM until autofill context is finalized.
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      expect(formsOnTheDom, hasLength(1));
    });

    test('No capitalization: setClient, setEditingState, show', () {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> capitalizeWordsConfig = createFlutterConfig('text');
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        capitalizeWordsConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': '',
        'selectionBase': 0,
        'selectionExtent': 0,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));
      spy.messages.clear();

      // Test for mobile Safari. `sentences` is the default attribute for
      // mobile browsers. Check if `off` is added to the input element.
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs) {
        expect(textEditing!.strategy.domElement!.getAttribute('autocapitalize'), 'off');
      } else {
        expect(textEditing!.strategy.domElement!.getAttribute('autocapitalize'), isNull);
      }

      spy.messages.clear();
      hideKeyboard();
    });

    test('All characters capitalization: setClient, setEditingState, show', () {
      // Create a configuration with an AutofillGroup of four text fields.
      final Map<String, dynamic> capitalizeWordsConfig = createFlutterConfig(
        'text',
        textCapitalization: 'TextCapitalization.characters',
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        capitalizeWordsConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': '',
        'selectionBase': 0,
        'selectionExtent': 0,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));
      spy.messages.clear();

      // Test for mobile Safari.
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs) {
        expect(textEditing!.strategy.domElement!.getAttribute('autocapitalize'), 'characters');
      }

      spy.messages.clear();
      hideKeyboard();
    });

    test(
      'setClient, setEditableSizeAndTransform, setStyle, setEditingState, show, clearClient',
      () {
        final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
          123,
          flutterSinglelineConfig,
        ]);
        sendFrameworkMessage(codec.encodeMethodCall(setClient));

        final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
          150,
          50,
          Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
        );
        sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

        final MethodCall setStyle = configureSetStyleMethodCall(12, 'sans-serif', 4, 4, 1);
        sendFrameworkMessage(codec.encodeMethodCall(setStyle));

        const MethodCall setEditingState = MethodCall(
          'TextInput.setEditingState',
          <String, dynamic>{'text': 'abcd', 'selectionBase': 2, 'selectionExtent': 3},
        );
        sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

        const MethodCall show = MethodCall('TextInput.show');
        sendFrameworkMessage(codec.encodeMethodCall(show));

        final DomElement domElement = textEditing!.strategy.domElement!;

        checkInputEditingState(domElement, 'abcd', 2, 3);

        // Check if the location and styling is correct.
        final DomRect boundingRect = domElement.getBoundingClientRect();
        expect(boundingRect.left, 10.0);
        expect(boundingRect.top, 20.0);
        expect(boundingRect.right, 160.0);
        expect(boundingRect.bottom, 70.0);
        expect(
          domElement.style.transform,
          'matrix3d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 20, 30, 1)',
        );
        expect(textEditing!.strategy.domElement!.style.font, '500 12px sans-serif');

        const MethodCall clearClient = MethodCall('TextInput.clearClient');
        sendFrameworkMessage(codec.encodeMethodCall(clearClient));

        // Confirm that [HybridTextEditing] didn't send any messages.
        expect(spy.messages, isEmpty);
      },
      // TODO(mdebbar): https://github.com/flutter/flutter/issues/50590
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test(
      'setClient, show, setEditableSizeAndTransform, setStyle, setEditingState, clearClient',
      () {
        final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
          123,
          flutterSinglelineConfig,
        ]);
        sendFrameworkMessage(codec.encodeMethodCall(setClient));

        const MethodCall show = MethodCall('TextInput.show');
        sendFrameworkMessage(codec.encodeMethodCall(show));

        // The "setSizeAndTransform" message has to be here before we call
        // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
        // we don't put the input element into the DOM until we get its correct
        // dimensions from the framework.
        final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
          150,
          50,
          Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
        );
        sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

        final MethodCall setStyle = configureSetStyleMethodCall(12, 'sans-serif', 4, 4, 1);
        sendFrameworkMessage(codec.encodeMethodCall(setStyle));

        const MethodCall setEditingState = MethodCall(
          'TextInput.setEditingState',
          <String, dynamic>{'text': 'abcd', 'selectionBase': 2, 'selectionExtent': 3},
        );
        sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

        final DomHTMLElement domElement = textEditing!.strategy.domElement!;

        checkInputEditingState(domElement, 'abcd', 2, 3);

        // Check if the position is correct.
        final DomRect boundingRect = domElement.getBoundingClientRect();
        expect(boundingRect.left, 10.0);
        expect(boundingRect.top, 20.0);
        expect(boundingRect.right, 160.0);
        expect(boundingRect.bottom, 70.0);
        expect(
          domElement.style.transform,
          'matrix3d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 20, 30, 1)',
        );
        expect(textEditing!.strategy.domElement!.style.font, '500 12px sans-serif');

        // For `blink` and `webkit` browser engines the overlay would be hidden.
        if (ui_web.browser.browserEngine == ui_web.BrowserEngine.blink ||
            ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
          expect(
            textEditing!.strategy.domElement!.classList.contains('transparentTextEditing'),
            isTrue,
          );
        } else {
          expect(
            textEditing!.strategy.domElement!.classList.contains('transparentTextEditing'),
            isFalse,
          );
        }

        const MethodCall clearClient = MethodCall('TextInput.clearClient');
        sendFrameworkMessage(codec.encodeMethodCall(clearClient));
      },
      // TODO(mdebbar): https://github.com/flutter/flutter/issues/50590
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test(
      'input font set successfully with null fontWeightIndex',
      () {
        final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
          123,
          flutterSinglelineConfig,
        ]);
        sendFrameworkMessage(codec.encodeMethodCall(setClient));

        final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
          150,
          50,
          Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
        );
        sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

        final MethodCall setStyle = configureSetStyleMethodCall(
          12,
          'sans-serif',
          4,
          null /* fontWeightIndex */,
          1,
        );
        sendFrameworkMessage(codec.encodeMethodCall(setStyle));

        const MethodCall setEditingState = MethodCall(
          'TextInput.setEditingState',
          <String, dynamic>{'text': 'abcd', 'selectionBase': 2, 'selectionExtent': 3},
        );
        sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

        const MethodCall show = MethodCall('TextInput.show');
        sendFrameworkMessage(codec.encodeMethodCall(show));

        final DomHTMLElement domElement = textEditing!.strategy.domElement!;

        checkInputEditingState(domElement, 'abcd', 2, 3);

        // Check if the location and styling is correct.
        final DomRect boundingRect = domElement.getBoundingClientRect();
        expect(boundingRect.left, 10.0);
        expect(boundingRect.top, 20.0);
        expect(boundingRect.right, 160.0);
        expect(boundingRect.bottom, 70.0);
        expect(
          domElement.style.transform,
          'matrix3d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 20, 30, 1)',
        );
        expect(textEditing!.strategy.domElement!.style.font, '12px sans-serif');

        hideKeyboard();
      },
      // TODO(mdebbar): https://github.com/flutter/flutter/issues/50590
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test('Canonicalizes font family', () {
      showKeyboard(inputType: 'text');

      final DomHTMLElement input = textEditing!.strategy.domElement!;

      MethodCall setStyle;

      setStyle = configureSetStyleMethodCall(12, 'sans-serif', 4, 4, 1);
      sendFrameworkMessage(codec.encodeMethodCall(setStyle));
      expect(input.style.fontFamily, canonicalizeFontFamily('sans-serif'));

      setStyle = configureSetStyleMethodCall(12, '.SF Pro Text', 4, 4, 1);
      sendFrameworkMessage(codec.encodeMethodCall(setStyle));
      expect(input.style.fontFamily, canonicalizeFontFamily('.SF Pro Text'));

      setStyle = configureSetStyleMethodCall(12, 'foo bar baz', 4, 4, 1);
      sendFrameworkMessage(codec.encodeMethodCall(setStyle));
      expect(input.style.fontFamily, canonicalizeFontFamily('foo bar baz'));

      hideKeyboard();
    });

    test('negative base offset and selection extent values in editing state is handled', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'xyz',
        'selectionBase': 1,
        'selectionExtent': 2,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // Check if the selection range is correct.
      checkInputEditingState(textEditing!.strategy.domElement, 'xyz', 1, 2);

      const MethodCall setEditingState2 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'xyz',
        'selectionBase': -1,
        'selectionExtent': -1,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState2));

      // The negative offset values are applied to the dom element as 0.
      checkInputEditingState(textEditing!.strategy.domElement, 'xyz', 0, 0);

      hideKeyboard();
    });

    test('Syncs the editing state back to Flutter', () {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterSinglelineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      final DomHTMLInputElement input = textEditing!.strategy.domElement! as DomHTMLInputElement;

      input.value = 'something';
      input.dispatchEvent(createDomEvent('Event', 'input'));

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingState');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'text': 'something',
          'selectionBase': 9,
          'selectionExtent': 9,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ]);
      spy.messages.clear();

      input.setSelectionRange(2, 5);
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        final DomEvent keyup = createDomEvent('Event', 'keyup');
        textEditing!.strategy.domElement!.dispatchEvent(keyup);
      } else {
        domDocument.dispatchEvent(createDomEvent('Event', 'selectionchange'));
      }

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingState');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'text': 'something',
          'selectionBase': 2,
          'selectionExtent': 5,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ]);
      spy.messages.clear();

      hideKeyboard();
    });

    test('Syncs the editing state back to Flutter - delta model', () {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('text', enableDeltaModel: true),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': '',
        'selectionBase': -1,
        'selectionExtent': -1,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      final DomHTMLInputElement input = textEditing!.strategy.domElement! as DomHTMLInputElement;

      input.value = 'something';
      input.dispatchEvent(createDomEvent('Event', 'input'));

      spy.messages.clear();

      input.setSelectionRange(2, 5);
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        final DomEvent keyup = createDomEvent('Event', 'keyup');
        textEditing!.strategy.domElement!.dispatchEvent(keyup);
      } else {
        domDocument.dispatchEvent(createDomEvent('Event', 'selectionchange'));
      }

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingStateWithDeltas');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'deltas': <Map<String, dynamic>>[
            <String, dynamic>{
              'oldText': 'something',
              'deltaText': '',
              'deltaStart': -1,
              'deltaEnd': -1,
              'selectionBase': 2,
              'selectionExtent': 5,
              'composingBase': -1,
              'composingExtent': -1,
            },
          ],
        },
      ]);
      spy.messages.clear();

      hideKeyboard();
    });

    test('Supports deletion at inverted selection', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('text', enableDeltaModel: true),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'Hello world',
        'selectionBase': 9,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomHTMLInputElement input = textEditing!.strategy.domElement! as DomHTMLInputElement;

      final DomInputEvent testEvent = createDomInputEvent('beforeinput', <Object?, Object?>{
        'inputType': 'deleteContentBackward',
      });
      input.dispatchEvent(testEvent);

      final EditingState editingState = EditingState(text: 'Helld', baseOffset: 3, extentOffset: 3);
      editingState.applyToDomElement(input);
      input.dispatchEvent(createDomEvent('Event', 'input'));

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingStateWithDeltas');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'deltas': <Map<String, dynamic>>[
            <String, dynamic>{
              'oldText': 'Hello world',
              'deltaText': '',
              'deltaStart': 3,
              'deltaEnd': 9,
              'selectionBase': 3,
              'selectionExtent': 3,
              'composingBase': -1,
              'composingExtent': -1,
            },
          ],
        },
      ]);
      spy.messages.clear();

      hideKeyboard();
      // TODO(Renzo-Olivares): https://github.com/flutter/flutter/issues/134271
    }, skip: isSafari);

    test('Supports new line at inverted selection', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('text', enableDeltaModel: true),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'Hello world',
        'selectionBase': 9,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomHTMLInputElement input = textEditing!.strategy.domElement! as DomHTMLInputElement;

      final DomInputEvent testEvent = createDomInputEvent('beforeinput', <Object?, Object?>{
        'inputType': 'insertLineBreak',
      });
      input.dispatchEvent(testEvent);

      final EditingState editingState = EditingState(
        text: 'Hel\nld',
        baseOffset: 3,
        extentOffset: 3,
      );
      editingState.applyToDomElement(input);
      input.dispatchEvent(createDomEvent('Event', 'input'));

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingStateWithDeltas');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'deltas': <Map<String, dynamic>>[
            <String, dynamic>{
              'oldText': 'Hello world',
              'deltaText': '\n',
              'deltaStart': 3,
              'deltaEnd': 9,
              'selectionBase': 3,
              'selectionExtent': 3,
              'composingBase': -1,
              'composingExtent': -1,
            },
          ],
        },
      ]);
      spy.messages.clear();

      hideKeyboard();
      // TODO(Renzo-Olivares): https://github.com/flutter/flutter/issues/134271
    }, skip: isSafari);

    test('multiTextField Autofill sync updates back to Flutter', () async {
      // Create a configuration with an AutofillGroup of four text fields.
      const String hintForFirstElement = 'familyName';
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        autofillHint: 'email',
        autofillHintsForFields: <String>[
          hintForFirstElement,
          'email',
          'givenName',
          'telephoneNumber',
        ],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall setEditingState1 = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'abcd',
        'selectionBase': 2,
        'selectionExtent': 3,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState1));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      // The second [setEditingState] should override the first one.
      checkInputEditingState(textEditing!.strategy.domElement, 'abcd', 2, 3);

      final DomHTMLFormElement formElement =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      // The form has 4 input elements and one submit button.
      expect(formElement.childNodes, hasLength(5));

      // Autofill one of the form elements.
      final DomHTMLInputElement element = formElement.childNodes.toList()[0] as DomHTMLInputElement;
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        expect(element.name, BrowserAutofillHints.instance.flutterToEngine(hintForFirstElement));
      } else {
        expect(
          element.autocomplete,
          BrowserAutofillHints.instance.flutterToEngine(hintForFirstElement),
        );
      }
      element.value = 'something';
      element.dispatchEvent(createDomEvent('Event', 'input'));

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingStateWithTag');
      expect(spy.messages[0].methodArguments, <dynamic>[
        0, // Client ID
        <String, dynamic>{
          hintForFirstElement: <String, dynamic>{
            'text': 'something',
            'selectionBase': 9,
            'selectionExtent': 9,
            'composingBase': -1,
            'composingExtent': -1,
          },
        },
      ]);

      spy.messages.clear();
      hideKeyboard();
    });

    test('Multi-line mode also works', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultilineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      expect(
        defaultTextEditingRoot.ownerDocument?.activeElement,
        domDocument.body,
        reason: 'Editing should have not started yet',
      );

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomHTMLTextAreaElement textarea =
          textEditing!.strategy.domElement! as DomHTMLTextAreaElement;
      checkTextAreaEditingState(textarea, '', 0, 0);

      // Can set editing state and preserve new lines.
      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': 'foo\nbar',
        'selectionBase': 2,
        'selectionExtent': 3,
        'composingBase': null,
        'composingExtent': null,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));
      checkTextAreaEditingState(textarea, 'foo\nbar', 2, 3);

      textarea.value = 'something\nelse';

      textarea.dispatchEvent(createDomEvent('Event', 'input'));
      textarea.setSelectionRange(2, 5);
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        textEditing!.strategy.domElement!.dispatchEvent(createDomEvent('Event', 'keyup'));
      } else {
        domDocument.dispatchEvent(createDomEvent('Event', 'selectionchange'));
      }

      // Two messages should've been sent. One for the 'input' event and one for
      // the 'selectionchange' event.
      expect(spy.messages, hasLength(2));

      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.updateEditingState');
      expect(spy.messages[0].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'text': 'something\nelse',
          'selectionBase': 14,
          'selectionExtent': 14,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ]);

      expect(spy.messages[1].channel, 'flutter/textinput');
      expect(spy.messages[1].methodName, 'TextInputClient.updateEditingState');
      expect(spy.messages[1].methodArguments, <dynamic>[
        123, // Client ID
        <String, dynamic>{
          'text': 'something\nelse',
          'selectionBase': 2,
          'selectionExtent': 5,
          'composingBase': -1,
          'composingExtent': -1,
        },
      ]);
      spy.messages.clear();

      const MethodCall hide = MethodCall('TextInput.hide');
      sendFrameworkMessage(codec.encodeMethodCall(hide));

      await waitForTextStrategyStopPropagation();

      expect(
        domDocument.activeElement,
        implicitViewRootElement,
        reason: 'Text editing should have stopped',
      );

      // Confirm that [HybridTextEditing] didn't send any more messages.
      expect(spy.messages, isEmpty);
    });

    test('none mode works', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('none'),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      expect(textEditing!.strategy.domElement!.tagName, 'INPUT');
      expect(getEditingInputMode(), 'none');
    });

    test('none multiline mode works', () async {
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        createFlutterConfig('none', isMultiline: true),
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList(),
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      expect(textEditing!.strategy.domElement!.tagName, 'TEXTAREA');
      expect(getEditingInputMode(), 'none');
    });

    test('sets correct input type in Android', () {
      ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.android;
      ui_web.browser.debugBrowserEngineOverride = ui_web.BrowserEngine.blink;

      /// During initialization [HybridTextEditing] will pick the correct
      /// text editing strategy for [OperatingSystem.android].
      textEditing = HybridTextEditing();

      showKeyboard(inputType: 'text');
      expect(getEditingInputMode(), null);

      showKeyboard(inputType: 'number');
      expect(getEditingInputMode(), 'numeric');

      showKeyboard(inputType: 'number');
      expect(getEditingInputMode(), 'numeric');

      showKeyboard(inputType: 'number', decimal: true);
      expect(getEditingInputMode(), 'decimal');

      showKeyboard(inputType: 'phone');
      expect(getEditingInputMode(), 'tel');

      showKeyboard(inputType: 'emailAddress');
      expect(getEditingInputMode(), 'email');

      showKeyboard(inputType: 'url');
      expect(getEditingInputMode(), 'url');

      showKeyboard(inputType: 'none');
      expect(getEditingInputMode(), 'none');
      expect(textEditing!.strategy.domElement!.tagName, 'INPUT');

      showKeyboard(inputType: 'none', isMultiline: true);
      expect(getEditingInputMode(), 'none');
      expect(textEditing!.strategy.domElement!.tagName, 'TEXTAREA');

      hideKeyboard();
    });

    test('sets correct input type for Firefox on Android', () {
      ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.android;
      ui_web.browser.debugBrowserEngineOverride = ui_web.BrowserEngine.firefox;

      /// During initialization [HybridTextEditing] will pick the correct
      /// text editing strategy for [OperatingSystem.android].
      textEditing = HybridTextEditing();

      showKeyboard(inputType: 'text');
      expect(getEditingInputMode(), null);

      showKeyboard(inputType: 'number');
      expect(getEditingInputMode(), 'numeric');

      showKeyboard(inputType: 'number');
      expect(getEditingInputMode(), 'numeric');

      showKeyboard(inputType: 'number', decimal: true);
      expect(getEditingInputMode(), 'decimal');

      showKeyboard(inputType: 'phone');
      expect(getEditingInputMode(), 'tel');

      showKeyboard(inputType: 'emailAddress');
      expect(getEditingInputMode(), 'email');

      showKeyboard(inputType: 'url');
      expect(getEditingInputMode(), 'url');

      showKeyboard(inputType: 'none');
      expect(getEditingInputMode(), 'none');

      hideKeyboard();
    });

    test('prevent mouse events on Android', () {
      // Regression test for https://github.com/flutter/flutter/issues/124483.
      ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.android;
      ui_web.browser.debugBrowserEngineOverride = ui_web.BrowserEngine.blink;

      /// During initialization [HybridTextEditing] will pick the correct
      /// text editing strategy for [OperatingSystem.android].
      textEditing = HybridTextEditing();

      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultilineConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      // Editing shouldn't have started yet.
      expect(defaultTextEditingRoot.ownerDocument?.activeElement, domDocument.body);

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      // The "setSizeAndTransform" message has to be here before we call
      // checkInputEditingState, since on some platforms (e.g. Desktop Safari)
      // we don't put the input element into the DOM until we get its correct
      // dimensions from the framework.
      final List<double> transform = Matrix4.translationValues(10.0, 20.0, 30.0).storage.toList();
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        150,
        50,
        transform,
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomHTMLTextAreaElement textarea =
          textEditing!.strategy.domElement! as DomHTMLTextAreaElement;
      checkTextAreaEditingState(textarea, '', 0, 0);

      // Can set editing state and preserve new lines.
      const MethodCall setEditingState = MethodCall('TextInput.setEditingState', <String, dynamic>{
        'text': '1\n2\n3\n4\n',
        'selectionBase': 8,
        'selectionExtent': 8,
        'composingBase': null,
        'composingExtent': null,
      });
      sendFrameworkMessage(codec.encodeMethodCall(setEditingState));
      checkTextAreaEditingState(textarea, '1\n2\n3\n4\n', 8, 8);

      // 'mousedown' event should be prevented.
      final DomEvent event = createDomEvent('Event', 'mousedown');
      textarea.dispatchEvent(event);
      expect(event.defaultPrevented, isTrue);

      hideKeyboard();
    });

    test('sets correct input type in iOS', () {
      // Test on ios-safari only.
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs) {
        /// During initialization [HybridTextEditing] will pick the correct
        /// text editing strategy for [OperatingSystem.iOs].
        textEditing = HybridTextEditing();

        showKeyboard(inputType: 'text');
        expect(getEditingInputMode(), null);

        showKeyboard(inputType: 'number');
        expect(getEditingInputMode(), 'numeric');

        showKeyboard(inputType: 'number');
        expect(getEditingInputMode(), 'numeric');

        showKeyboard(inputType: 'number', decimal: true);
        expect(getEditingInputMode(), 'decimal');

        showKeyboard(inputType: 'phone');
        expect(getEditingInputMode(), 'tel');

        showKeyboard(inputType: 'emailAddress');
        expect(getEditingInputMode(), 'email');

        showKeyboard(inputType: 'url');
        expect(getEditingInputMode(), 'url');

        showKeyboard(inputType: 'none');
        expect(getEditingInputMode(), 'none');
        expect(textEditing!.strategy.domElement!.tagName, 'INPUT');

        showKeyboard(inputType: 'none', isMultiline: true);
        expect(getEditingInputMode(), 'none');
        expect(textEditing!.strategy.domElement!.tagName, 'TEXTAREA');

        hideKeyboard();
      }
    });

    test('sends the correct input action as a platform message', () {
      final int clientId = showKeyboard(inputType: 'text', inputAction: 'TextInputAction.next');

      // There should be no input action yet.
      expect(lastInputAction, isNull);

      dispatchKeyboardEvent(textEditing!.strategy.domElement!, 'keydown', keyCode: _kReturnKeyCode);

      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.performAction');
      expect(spy.messages[0].methodArguments, <dynamic>[clientId, 'TextInputAction.next']);
    });

    test('sends input action in multi-line mode', () {
      showKeyboard(inputType: 'multiline', inputAction: 'TextInputAction.next');

      final DomKeyboardEvent event = dispatchKeyboardEvent(
        textEditing!.strategy.domElement!,
        'keydown',
        keyCode: _kReturnKeyCode,
      );

      // Input action is sent as a platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/textinput');
      expect(spy.messages[0].methodName, 'TextInputClient.performAction');
      expect(spy.messages[0].methodArguments, <dynamic>[clientId, 'TextInputAction.next']);
      // And default behavior of keyboard event should have been prevented.
      // Only TextInputAction.newline should not prevent default behavior
      // for a multiline field.
      expect(event.defaultPrevented, isTrue);
    });

    test('inserts element in the correct view', () async {
      final DomElement host = createDomElement('div');
      domDocument.body!.append(host);
      final EngineFlutterView view = EngineFlutterView(dispatcher, host);
      dispatcher.viewManager.registerView(view);

      textEditing = HybridTextEditing();
      showKeyboard(inputType: 'text', viewId: view.viewId);
      // The Safari strategy doesn't insert the input element into the DOM until
      // it has received the geometry information.
      final List<double> transform = Matrix4.identity().storage.toList();
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        10,
        10,
        transform,
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomElement input = textEditing!.strategy.domElement!;

      // Input is appended to the right view.
      expect(view.dom.textEditingHost.contains(input), isTrue);

      // Cleanup.
      hideKeyboard();
      dispatcher.viewManager.unregisterView(view.viewId);
      view.dispose();
      host.remove();
    });

    test('moves element to correct view', () {
      final DomElement host1 = createDomElement('div');
      domDocument.body!.append(host1);
      final EngineFlutterView view1 = EngineFlutterView(dispatcher, host1);
      dispatcher.viewManager.registerView(view1);

      final DomElement host2 = createDomElement('div');
      domDocument.body!.append(host2);
      final EngineFlutterView view2 = EngineFlutterView(dispatcher, host2);
      dispatcher.viewManager.registerView(view2);

      textEditing = HybridTextEditing();
      showKeyboard(inputType: 'text', viewId: view1.viewId, autofillEnabled: false);

      final DomElement input = textEditing!.strategy.domElement!;

      // Input is appended to view1.
      expect(view1.dom.textEditingHost.contains(input), isTrue);

      sendFrameworkMessage(
        codec.encodeMethodCall(
          MethodCall(
            'TextInput.updateConfig',
            createFlutterConfig('text', viewId: view2.viewId, autofillEnabled: false),
          ),
        ),
      );

      // The input element is the same (no new element was created), but it has
      // moved to view2.
      expect(textEditing!.strategy.domElement, input);
      expect(view2.dom.textEditingHost.contains(input), isTrue);

      // Cleanup.
      hideKeyboard();
      dispatcher.viewManager.unregisterView(view1.viewId);
      view1.dispose();
      dispatcher.viewManager.unregisterView(view2.viewId);
      view2.dispose();
      host1.remove();
      host2.remove();
    });

    test('places autofill form in the correct view', () async {
      final DomElement host = createDomElement('div');
      domDocument.body!.append(host);
      final EngineFlutterView view = EngineFlutterView(dispatcher, host);
      dispatcher.viewManager.registerView(view);

      textEditing = HybridTextEditing();

      // Create a configuration with an AutofillGroup of three text fields.
      final Map<String, dynamic> flutterMultiAutofillElementConfig = createFlutterConfig(
        'text',
        viewId: view.viewId,
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        flutterMultiAutofillElementConfig,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));
      // The Safari strategy doesn't insert the input element into the DOM until
      // it has received the geometry information.
      final List<double> transform = Matrix4.identity().storage.toList();
      final MethodCall setSizeAndTransform = configureSetSizeAndTransformMethodCall(
        10,
        10,
        transform,
      );
      sendFrameworkMessage(codec.encodeMethodCall(setSizeAndTransform));

      final DomElement input = textEditing!.strategy.domElement!;
      final DomElement form = textEditing!.configuration!.autofillGroup!.formElement;

      // Input and form are appended to the right view.
      expect(view.dom.textEditingHost.contains(input), isTrue);
      expect(view.dom.textEditingHost.contains(form), isTrue);

      // Cleanup.
      hideKeyboard();
      dispatcher.viewManager.unregisterView(view.viewId);
      view.dispose();
      host.remove();
    });

    test('moves autofill form to the correct view', () async {
      final DomElement host1 = createDomElement('div');
      domDocument.body!.append(host1);
      final EngineFlutterView view1 = EngineFlutterView(dispatcher, host1);
      dispatcher.viewManager.registerView(view1);

      final DomElement host2 = createDomElement('div');
      domDocument.body!.append(host2);
      final EngineFlutterView view2 = EngineFlutterView(dispatcher, host2);
      dispatcher.viewManager.registerView(view2);

      textEditing = HybridTextEditing();

      // Create a configuration with an AutofillGroup of three text fields.
      final Map<String, dynamic> autofillConfig1 = createFlutterConfig(
        'text',
        viewId: view1.viewId,
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name'],
      );
      final MethodCall setClient = MethodCall('TextInput.setClient', <dynamic>[
        123,
        autofillConfig1,
      ]);
      sendFrameworkMessage(codec.encodeMethodCall(setClient));

      const MethodCall show = MethodCall('TextInput.show');
      sendFrameworkMessage(codec.encodeMethodCall(show));

      final DomElement input = textEditing!.strategy.domElement!;
      final DomElement form = textEditing!.configuration!.autofillGroup!.formElement;

      // Input and form are appended to view1.
      expect(view1.dom.textEditingHost.contains(input), isTrue);
      expect(view1.dom.textEditingHost.contains(form), isTrue);

      // Move the input and form to view2.
      final Map<String, dynamic> autofillConfig2 = createFlutterConfig(
        'text',
        viewId: view2.viewId,
        autofillHint: 'username',
        autofillHintsForFields: <String>['username', 'email', 'name'],
      );
      sendFrameworkMessage(
        codec.encodeMethodCall(MethodCall('TextInput.updateConfig', autofillConfig2)),
      );

      // Input and form are in view2.
      expect(view2.dom.textEditingHost.contains(input), isTrue);
      expect(view2.dom.textEditingHost.contains(form), isTrue);

      // Cleanup.
      hideKeyboard();
      dispatcher.viewManager.unregisterView(view1.viewId);
      view1.dispose();
      dispatcher.viewManager.unregisterView(view2.viewId);
      view2.dispose();
      host1.remove();
      host2.remove();
      // TODO(mdebbar): Autofill forms don't get updated in the current system.
      //                https://github.com/flutter/flutter/issues/145101
    }, skip: true);

    tearDown(() {
      clearForms();
    });
  });

  group('EngineAutofillForm', () {
    test('validate multi element form', () {
      final List<dynamic> fields = createFieldValues(
        <String>['username', 'password', 'newPassword'],
        <String>['field1', 'field2', 'field3'],
      );
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('username', 'field1'),
            fields,
          )!;

      // Number of elements if number of fields sent to the constructor minus
      // one (for the focused text element).
      expect(autofillForm.elements, hasLength(2));
      expect(autofillForm.items, hasLength(2));
      expect(autofillForm.formElement, isNotNull);

      expect(autofillForm.formIdentifier, 'field1*field2*field3');

      final DomHTMLFormElement form = autofillForm.formElement;
      // Note that we also add a submit button. Therefore the form element has
      // 3 child nodes.
      expect(form.childNodes, hasLength(3));

      final DomHTMLInputElement firstElement = form.childNodes.toList()[0] as DomHTMLInputElement;
      // Autofill value is applied to the element.
      expect(firstElement.name, BrowserAutofillHints.instance.flutterToEngine('password'));
      expect(firstElement.id, BrowserAutofillHints.instance.flutterToEngine('password'));
      expect(firstElement.type, 'password');
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        expect(firstElement.name, BrowserAutofillHints.instance.flutterToEngine('password'));
      } else {
        expect(
          firstElement.autocomplete,
          BrowserAutofillHints.instance.flutterToEngine('password'),
        );
      }

      // Editing state is applied to the element.
      expect(firstElement.value, 'Test');
      expect(firstElement.selectionStart, 0);
      expect(firstElement.selectionEnd, 0);

      // Element is hidden.
      final DomCSSStyleDeclaration css = firstElement.style;
      expect(css.color, 'transparent');
      expect(css.backgroundColor, 'transparent');

      // For `blink` and `webkit` browser engines the overlay would be hidden.
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.blink ||
          ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
        expect(firstElement.classList.contains('transparentTextEditing'), isTrue);
      } else {
        expect(firstElement.classList.contains('transparentTextEditing'), isFalse);
      }
    });

    test('validate multi element form ids sorted for form id', () {
      final List<dynamic> fields = createFieldValues(
        <String>['username', 'password', 'newPassword'],
        <String>['zzyyxx', 'aabbcc', 'jjkkll'],
      );
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('username', 'field1'),
            fields,
          )!;

      expect(autofillForm.formIdentifier, 'aabbcc*jjkkll*zzyyxx');
    });

    test('place and store form', () {
      expect(defaultTextEditingRoot.querySelectorAll('form'), isEmpty);

      final List<dynamic> fields = createFieldValues(
        <String>['username', 'password', 'newPassword'],
        <String>['field1', 'fields2', 'field3'],
      );
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('username', 'field1'),
            fields,
          )!;

      final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
      autofillForm.placeForm(testInputElement);

      // The focused element is appended to the form, form also has the button
      // so in total it shoould have 4 elements.
      final DomHTMLFormElement form = autofillForm.formElement;
      expect(form.childNodes, hasLength(4));

      final DomHTMLFormElement formOnDom =
          defaultTextEditingRoot.querySelector('form')! as DomHTMLFormElement;
      // Form is attached to the DOM.
      expect(form, equals(formOnDom));

      autofillForm.storeForm();
      expect(defaultTextEditingRoot.querySelectorAll('form'), isNotEmpty);
      expect(formsOnTheDom, hasLength(1));
    });

    test('Validate single element form', () {
      final List<dynamic> fields = createFieldValues(<String>['username'], <String>['field1']);
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('username', 'field1'),
            fields,
          )!;

      // The focused element is the only field. Form should be empty after
      // the initialization (focus element is appended later).
      expect(autofillForm.elements, isEmpty);
      expect(autofillForm.items, isEmpty);
      expect(autofillForm.formElement, isNotNull);

      final DomHTMLFormElement form = autofillForm.formElement;
      // Submit button is added to the form.
      expect(form.childNodes, isNotEmpty);
      final DomHTMLInputElement inputElement = form.childNodes.toList()[0] as DomHTMLInputElement;
      expect(inputElement.type, 'submit');
      expect(inputElement.tabIndex, -1, reason: 'The input should not be reachable by keyboard');

      // The submit button should have class `submitBtn`.
      expect(inputElement.className, 'submitBtn');
    });

    test('Return null if no focused element', () {
      final List<dynamic> fields = createFieldValues(<String>['username'], <String>['field1']);
      final EngineAutofillForm? autofillForm = EngineAutofillForm.fromFrameworkMessage(
        kImplicitViewId,
        null,
        fields,
      );

      expect(autofillForm, isNull);
    });

    test('placeForm() should place element in correct position', () {
      final List<dynamic> fields = createFieldValues(
        <String>['email', 'username', 'password'],
        <String>['field1', 'field2', 'field3'],
      );
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('email', 'field1'),
            fields,
          )!;

      expect(autofillForm.elements, hasLength(2));

      List<DomHTMLInputElement> formChildNodes =
          autofillForm.formElement.childNodes.toList() as List<DomHTMLInputElement>;

      // Only username, password, submit nodes are created
      expect(formChildNodes, hasLength(3));
      expect(formChildNodes[0].name, 'username');
      expect(formChildNodes[1].name, 'current-password');
      expect(formChildNodes[2].type, 'submit');
      // insertion point for email should be before username
      expect(autofillForm.insertionReferenceNode, formChildNodes[0]);

      final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
      testInputElement.name = 'email';
      autofillForm.placeForm(testInputElement);

      formChildNodes = autofillForm.formElement.childNodes.toList() as List<DomHTMLInputElement>;
      // email node should be placed before username
      expect(formChildNodes, hasLength(4));
      expect(formChildNodes[0].name, 'email');
      expect(formChildNodes[1].name, 'username');
      expect(formChildNodes[2].name, 'current-password');
      expect(formChildNodes[3].type, 'submit');
    });

    test(
      'hidden autofill elements should have a width and height of 0 on non-Safari browsers',
      () {
        final List<dynamic> fields = createFieldValues(
          <String>['email', 'username', 'password'],
          <String>['field1', 'field2', 'field3'],
        );
        final EngineAutofillForm autofillForm =
            EngineAutofillForm.fromFrameworkMessage(
              kImplicitViewId,
              createAutofillInfo('email', 'field1'),
              fields,
            )!;
        final List<DomHTMLInputElement> formChildNodes =
            autofillForm.formElement.childNodes.toList() as List<DomHTMLInputElement>;
        final DomHTMLInputElement username = formChildNodes[0];
        final DomHTMLInputElement password = formChildNodes[1];

        expect(username.name, 'username');
        expect(password.name, 'current-password');
        expect(username.style.width, '0px');
        expect(username.style.height, '0px');
        expect(username.style.pointerEvents, isNot('none'));
        expect(password.style.width, '0px');
        expect(password.style.height, '0px');
        expect(password.style.pointerEvents, isNot('none'));
        expect(autofillForm.formElement.style.pointerEvents, isNot('none'));
      },
      skip: isSafari,
    );

    test('hidden autofill elements should not have a width and height of 0 on Safari', () {
      final List<dynamic> fields = createFieldValues(
        <String>['email', 'username', 'password'],
        <String>['field1', 'field2', 'field3'],
      );
      final EngineAutofillForm autofillForm =
          EngineAutofillForm.fromFrameworkMessage(
            kImplicitViewId,
            createAutofillInfo('email', 'field1'),
            fields,
          )!;
      final List<DomHTMLInputElement> formChildNodes =
          autofillForm.formElement.childNodes.toList() as List<DomHTMLInputElement>;
      final DomHTMLInputElement username = formChildNodes[0];
      final DomHTMLInputElement password = formChildNodes[1];
      expect(username.name, 'username');
      expect(password.name, 'current-password');
      expect(username.style.width, isNot('0px'));
      expect(username.style.height, isNot('0px'));
      expect(username.style.pointerEvents, 'none');
      expect(password.style.width, isNot('0px'));
      expect(password.style.height, isNot('0px'));
      expect(password.style.pointerEvents, 'none');
      expect(autofillForm.formElement.style.pointerEvents, 'none');
    }, skip: !isSafari);

    test(
      'the focused element within a form should explicitly set pointer events on Safari',
      () {
        final List<dynamic> fields = createFieldValues(
          <String>['email', 'username', 'password'],
          <String>['field1', 'field2', 'field3'],
        );
        final EngineAutofillForm autofillForm =
            EngineAutofillForm.fromFrameworkMessage(
              kImplicitViewId,
              createAutofillInfo('email', 'field1'),
              fields,
            )!;

        final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
        testInputElement.name = 'email';
        autofillForm.placeForm(testInputElement);

        final List<DomHTMLInputElement> formChildNodes =
            autofillForm.formElement.childNodes.toList() as List<DomHTMLInputElement>;
        final DomHTMLInputElement email = formChildNodes[0];
        final DomHTMLInputElement username = formChildNodes[1];
        final DomHTMLInputElement password = formChildNodes[2];

        expect(email.name, 'email');
        expect(username.name, 'username');
        expect(password.name, 'current-password');

        // pointer events are none on the form and all non-focused elements
        expect(autofillForm.formElement.style.pointerEvents, 'none');
        expect(username.style.pointerEvents, 'none');
        expect(password.style.pointerEvents, 'none');

        // pointer events are set to all on the activeDomElement
        expect(email.style.pointerEvents, 'all');
      },
      skip: !isSafari,
    );

    tearDown(() {
      clearForms();
    });
  });

  group('AutofillInfo', () {
    const String testHint = 'streetAddressLine2';
    const String testId = 'EditableText-659836579';
    const String testPasswordHint = 'password';

    test('autofill has correct value', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(testHint, testId),
      );

      // Hint sent from the framework is converted to the hint compatible with
      // browsers.
      expect(autofillInfo.autofillHint, BrowserAutofillHints.instance.flutterToEngine(testHint));
      expect(autofillInfo.uniqueIdentifier, testId);
    });

    test('input with autofill hint', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(testHint, testId),
      );

      final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
      autofillInfo.applyToDomElement(testInputElement);

      // Hint sent from the framework is converted to the hint compatible with
      // browsers.
      expect(testInputElement.name, BrowserAutofillHints.instance.flutterToEngine(testHint));
      expect(testInputElement.id, BrowserAutofillHints.instance.flutterToEngine(testHint));
      expect(testInputElement.type, 'text');
      if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
        expect(testInputElement.name, BrowserAutofillHints.instance.flutterToEngine(testHint));
      } else {
        expect(
          testInputElement.autocomplete,
          BrowserAutofillHints.instance.flutterToEngine(testHint),
        );
      }
    });

    test('textarea with autofill hint', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(testHint, testId),
      );

      final DomHTMLTextAreaElement testInputElement = createDomHTMLTextAreaElement();
      autofillInfo.applyToDomElement(testInputElement);

      // Hint sent from the framework is converted to the hint compatible with
      // browsers.
      expect(testInputElement.name, BrowserAutofillHints.instance.flutterToEngine(testHint));
      expect(testInputElement.id, BrowserAutofillHints.instance.flutterToEngine(testHint));
      expect(
        testInputElement.getAttribute('autocomplete'),
        BrowserAutofillHints.instance.flutterToEngine(testHint),
      );
    });

    test('password autofill hint', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(testPasswordHint, testId),
      );

      final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
      autofillInfo.applyToDomElement(testInputElement);

      // Hint sent from the framework is converted to the hint compatible with
      // browsers.
      expect(
        testInputElement.name,
        BrowserAutofillHints.instance.flutterToEngine(testPasswordHint),
      );
      expect(testInputElement.id, BrowserAutofillHints.instance.flutterToEngine(testPasswordHint));
      expect(testInputElement.type, 'password');
      expect(
        testInputElement.getAttribute('autocomplete'),
        BrowserAutofillHints.instance.flutterToEngine(testPasswordHint),
      );
    });

    test('autofill with no hints', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(null, testId),
      );

      final DomHTMLInputElement testInputElement = createDomHTMLInputElement();
      autofillInfo.applyToDomElement(testInputElement);

      expect(testInputElement.autocomplete, 'on');
      expect(testInputElement.placeholder, isEmpty);
    });

    test('TextArea autofill with no hints', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(null, testId),
      );

      final DomHTMLTextAreaElement testInputElement = createDomHTMLTextAreaElement();
      autofillInfo.applyToDomElement(testInputElement);

      expect(testInputElement.getAttribute('autocomplete'), 'on');
      expect(testInputElement.placeholder, isEmpty);
    });

    test('autofill with only placeholder', () {
      final AutofillInfo autofillInfo = AutofillInfo.fromFrameworkMessage(
        createAutofillInfo(null, testId, placeholder: 'enter your password'),
      );

      final DomHTMLTextAreaElement testInputElement = createDomHTMLTextAreaElement();
      autofillInfo.applyToDomElement(testInputElement);

      expect(testInputElement.getAttribute('autocomplete'), 'on');
      expect(testInputElement.placeholder, 'enter your password');
    });

    // Regression test for https://github.com/flutter/flutter/issues/135542
    test('autofill with middleName hint', () {
      expect(BrowserAutofillHints.instance.flutterToEngine('middleName'), 'additional-name');
    });
  });

  group('EditingState', () {
    EditingState editingState;

    setUp(() {
      editingStrategy = GloballyPositionedTextEditingStrategy(HybridTextEditing());
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
    });

    test('Fix flipped base and extent offsets', () {
      expect(
        EditingState(baseOffset: 10, extentOffset: 4),
        EditingState(baseOffset: 4, extentOffset: 10),
      );

      expect(
        EditingState.fromFrameworkMessage(<String, dynamic>{
          'selectionBase': 10,
          'selectionExtent': 4,
        }),
        EditingState.fromFrameworkMessage(<String, dynamic>{
          'selectionBase': 4,
          'selectionExtent': 10,
        }),
      );
    });

    test('Sets default composing offsets if none given', () {
      final EditingState editingState = EditingState(text: 'Test', baseOffset: 2, extentOffset: 4);
      final EditingState editingStateFromFrameworkMsg = EditingState.fromFrameworkMessage(
        <String, dynamic>{'selectionBase': 10, 'selectionExtent': 4},
      );

      expect(editingState.composingBaseOffset, -1);
      expect(editingState.composingExtentOffset, -1);

      expect(editingStateFromFrameworkMsg.composingBaseOffset, -1);
      expect(editingStateFromFrameworkMsg.composingExtentOffset, -1);
    });

    test('Correctly identifies min and max offsets', () {
      final EditingState flippedEditingState = EditingState(baseOffset: 10, extentOffset: 4);
      final EditingState normalEditingState = EditingState(baseOffset: 2, extentOffset: 6);

      expect(flippedEditingState.minOffset, 4);
      expect(flippedEditingState.maxOffset, 10);
      expect(normalEditingState.minOffset, 2);
      expect(normalEditingState.maxOffset, 6);
    });

    test('Configure input element from the editing state', () {
      final DomHTMLInputElement input =
          defaultTextEditingRoot.querySelector('input')! as DomHTMLInputElement;
      editingState = EditingState(text: 'Test', baseOffset: 1, extentOffset: 2);

      editingState.applyToDomElement(input);

      expect(input.value, 'Test');
      expect(input.selectionStart, 1);
      expect(input.selectionEnd, 2);
    });

    test('Configure text area element from the editing state', () {
      cleanTextEditingStrategy();
      editingStrategy!.enable(
        multilineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      final DomHTMLTextAreaElement textArea =
          defaultTextEditingRoot.querySelector('textarea')! as DomHTMLTextAreaElement;
      editingState = EditingState(text: 'Test', baseOffset: 1, extentOffset: 2);

      editingState.applyToDomElement(textArea);

      expect(textArea.value, 'Test');
      expect(textArea.selectionStart, 1);
      expect(textArea.selectionEnd, 2);
    });

    test('Configure input element editing state for a flipped base and extent', () {
      final DomHTMLInputElement input =
          defaultTextEditingRoot.querySelector('input')! as DomHTMLInputElement;
      editingState = EditingState(text: 'Hello World', baseOffset: 10, extentOffset: 2);

      editingState.applyToDomElement(input);

      expect(input.value, 'Hello World');
      expect(input.selectionStart, 2);
      expect(input.selectionEnd, 10);
    });

    test('Get Editing State from input element', () {
      final DomHTMLInputElement input =
          defaultTextEditingRoot.querySelector('input')! as DomHTMLInputElement;
      input.value = 'Test';

      input.setSelectionRange(1, 2);
      editingState = EditingState.fromDomElement(input);
      expect(editingState.text, 'Test');
      expect(editingState.baseOffset, 1);
      expect(editingState.extentOffset, 2);

      input.setSelectionRange(1, 2, 'backward');
      editingState = EditingState.fromDomElement(input);
      expect(editingState.baseOffset, 2);
      expect(editingState.extentOffset, 1);
    });

    test('Get Editing State from text area element', () {
      cleanTextEditingStrategy();
      editingStrategy!.enable(
        multilineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      final DomHTMLTextAreaElement input =
          defaultTextEditingRoot.querySelector('textarea')! as DomHTMLTextAreaElement;
      input.value = 'Test';

      input.setSelectionRange(1, 2);
      editingState = EditingState.fromDomElement(input);
      expect(editingState.text, 'Test');
      expect(editingState.baseOffset, 1);
      expect(editingState.extentOffset, 2);

      input.setSelectionRange(1, 2, 'backward');
      editingState = EditingState.fromDomElement(input);
      expect(editingState.baseOffset, 2);
      expect(editingState.extentOffset, 1);
    });

    group('comparing editing states', () {
      test('From dom element', () {
        final DomHTMLInputElement input =
            defaultTextEditingRoot.querySelector('input')! as DomHTMLInputElement;
        input.value = 'Test';
        input.selectionStart = 1;
        input.selectionEnd = 2;

        final EditingState editingState1 = EditingState.fromDomElement(input);
        final EditingState editingState2 = EditingState.fromDomElement(input);

        input.setSelectionRange(1, 3);

        final EditingState editingState3 = EditingState.fromDomElement(input);

        expect(editingState1 == editingState2, isTrue);
        expect(editingState1 != editingState3, isTrue);
      });

      test('Takes flipped base and extent offsets into account', () {
        final EditingState flippedEditingState = EditingState(baseOffset: 10, extentOffset: 4);
        final EditingState normalEditingState = EditingState(baseOffset: 4, extentOffset: 10);

        expect(normalEditingState, flippedEditingState);

        expect(normalEditingState == flippedEditingState, isTrue);
      });

      test('takes composition range into account', () {
        final EditingState editingState1 = EditingState(
          composingBaseOffset: 1,
          composingExtentOffset: 2,
        );
        final EditingState editingState2 = EditingState(
          composingBaseOffset: 1,
          composingExtentOffset: 2,
        );
        final EditingState editingState3 = EditingState(
          composingBaseOffset: 4,
          composingExtentOffset: 8,
        );

        expect(editingState1, editingState2);
        expect(editingState1, isNot(editingState3));
      });
    });
  });

  group('TextEditingDeltaState', () {
    // The selection baseOffset and extentOffset are not inferred by
    // TextEditingDeltaState.inferDeltaState so we do not verify them here.
    test('Verify correct delta is inferred - insertion', () {
      final EditingState newEditState = EditingState(text: 'world', baseOffset: 5, extentOffset: 5);
      final EditingState lastEditState = EditingState(text: 'worl', baseOffset: 4, extentOffset: 4);
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'worl',
        deltaText: 'd',
        deltaStart: 4,
        deltaEnd: 4,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'worl');
      expect(textEditingDeltaState.deltaText, 'd');
      expect(textEditingDeltaState.deltaStart, 4);
      expect(textEditingDeltaState.deltaEnd, 4);
      expect(textEditingDeltaState.baseOffset, 5);
      expect(textEditingDeltaState.extentOffset, 5);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Verify correct delta is inferred - Backward deletion - Empty selection', () {
      final EditingState newEditState = EditingState(text: 'worl', baseOffset: 4, extentOffset: 4);
      final EditingState lastEditState = EditingState(
        text: 'world',
        baseOffset: 5,
        extentOffset: 5,
      );
      // `deltaState.deltaEnd` is initialized accordingly to what is done in `DefaultTextEditingStrategy.handleBeforeInput`
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'world',
        deltaEnd: 5,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'world');
      expect(textEditingDeltaState.deltaText, '');
      expect(textEditingDeltaState.deltaStart, 4);
      expect(textEditingDeltaState.deltaEnd, 5);
      expect(textEditingDeltaState.baseOffset, 4);
      expect(textEditingDeltaState.extentOffset, 4);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Verify correct delta is inferred - Forward deletion - Empty selection', () {
      final EditingState newEditState = EditingState(text: 'worl', baseOffset: 4, extentOffset: 4);
      final EditingState lastEditState = EditingState(
        text: 'world',
        baseOffset: 4,
        extentOffset: 4,
      );
      // `deltaState.deltaEnd` is initialized accordingly to what is done in `DefaultTextEditingStrategy.handleBeforeInput`
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'world',
        deltaEnd: 4,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'world');
      expect(textEditingDeltaState.deltaText, '');
      expect(textEditingDeltaState.deltaStart, 4);
      expect(textEditingDeltaState.deltaEnd, 5);
      expect(textEditingDeltaState.baseOffset, 4);
      expect(textEditingDeltaState.extentOffset, 4);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Verify correct delta is inferred - Deletion - Non-empty selection', () {
      final EditingState newEditState = EditingState(text: 'w', baseOffset: 1, extentOffset: 1);
      final EditingState lastEditState = EditingState(
        text: 'world',
        baseOffset: 1,
        extentOffset: 5,
      );
      // `deltaState.deltaEnd` is initialized accordingly to what is done in `DefaultTextEditingStrategy.handleBeforeInput`
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'world',
        deltaEnd: 5,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'world');
      expect(textEditingDeltaState.deltaText, '');
      expect(textEditingDeltaState.deltaStart, 1);
      expect(textEditingDeltaState.deltaEnd, 5);
      expect(textEditingDeltaState.baseOffset, 1);
      expect(textEditingDeltaState.extentOffset, 1);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Verify correct delta is inferred - composing region replacement', () {
      final EditingState newEditState = EditingState(text: '你好吗', baseOffset: 3, extentOffset: 3);
      final EditingState lastEditState = EditingState(
        text: 'ni hao ma',
        baseOffset: 9,
        extentOffset: 9,
      );
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'ni hao ma',
        deltaText: '你好吗',
        deltaStart: 9,
        deltaEnd: 9,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: 0,
        composingExtent: 9,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'ni hao ma');
      expect(textEditingDeltaState.deltaText, '你好吗');
      expect(textEditingDeltaState.deltaStart, 0);
      expect(textEditingDeltaState.deltaEnd, 9);
      expect(textEditingDeltaState.baseOffset, 3);
      expect(textEditingDeltaState.extentOffset, 3);
      expect(textEditingDeltaState.composingOffset, 0);
      expect(textEditingDeltaState.composingExtent, 9);
    });

    test('Verify correct delta is inferred for double space to insert a period', () {
      final EditingState newEditState = EditingState(
        text: 'hello. ',
        baseOffset: 7,
        extentOffset: 7,
      );
      final EditingState lastEditState = EditingState(
        text: 'hello ',
        baseOffset: 6,
        extentOffset: 6,
      );
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'hello ',
        deltaText: '. ',
        deltaStart: 6,
        deltaEnd: 6,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'hello ');
      expect(textEditingDeltaState.deltaText, '. ');
      expect(textEditingDeltaState.deltaStart, 5);
      expect(textEditingDeltaState.deltaEnd, 6);
      expect(textEditingDeltaState.baseOffset, 7);
      expect(textEditingDeltaState.extentOffset, 7);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Verify correct delta is inferred for accent menu', () {
      final EditingState newEditState = EditingState(text: 'à', baseOffset: 1, extentOffset: 1);
      final EditingState lastEditState = EditingState(text: 'a', baseOffset: 1, extentOffset: 1);
      final TextEditingDeltaState deltaState = TextEditingDeltaState(
        oldText: 'a',
        deltaText: 'à',
        deltaStart: 1,
        deltaEnd: 1,
        baseOffset: -1,
        extentOffset: -1,
        composingOffset: -1,
        composingExtent: -1,
      );

      final TextEditingDeltaState textEditingDeltaState = TextEditingDeltaState.inferDeltaState(
        newEditState,
        lastEditState,
        deltaState,
      );

      expect(textEditingDeltaState.oldText, 'a');
      expect(textEditingDeltaState.deltaText, 'à');
      expect(textEditingDeltaState.deltaStart, 0);
      expect(textEditingDeltaState.deltaEnd, 1);
      expect(textEditingDeltaState.baseOffset, 1);
      expect(textEditingDeltaState.extentOffset, 1);
      expect(textEditingDeltaState.composingOffset, -1);
      expect(textEditingDeltaState.composingExtent, -1);
    });

    test('Delta state is cleared after setting editing state', () {
      editingStrategy!.enable(
        multilineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );
      final DomHTMLInputElement input = editingStrategy!.domElement! as DomHTMLInputElement;
      input.value = 'foo bar';
      input.dispatchEvent(createDomEvent('Event', 'input'));
      expect(lastEditingState, EditingState(text: 'foo bar', baseOffset: 7, extentOffset: 7));
      expect(editingStrategy!.editingDeltaState.oldText, 'foo bar');

      editingStrategy!.setEditingState(
        EditingState(text: 'foo bar baz', baseOffset: 11, extentOffset: 11),
      );
      input.dispatchEvent(createDomEvent('Event', 'input'));
      expect(editingStrategy?.editingDeltaState.oldText, 'foo bar baz');
    });
  });

  group('text editing styles', () {
    test('invisible element', () {
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      final DomHTMLElement input = editingStrategy!.activeDomElement;
      expect(domDocument.activeElement, input, reason: 'the input element should be focused');

      expect(input.style.color, contains('transparent'));
      if (isSafari) {
        // macOS 13 returns different values than macOS 12.
        expect(input.style.background, anyOf(contains('transparent'), contains('none')));
        expect(input.style.outline, anyOf(contains('none'), contains('currentcolor')));
        expect(input.style.border, anyOf(contains('none'), contains('medium')));
      } else {
        expect(input.style.background, contains('transparent'));
        expect(input.style.outline, contains('none'));
        expect(input.style.border, anyOf(contains('none'), contains('medium')));
      }
      expect(input.style.backgroundColor, contains('transparent'));
      expect(input.style.caretColor, contains('transparent'));
      expect(input.style.textShadow, contains('none'));
    });

    test('prevents effect of (forced-colors: active)', () {
      editingStrategy!.enable(
        singlelineConfig,
        onChange: trackEditingState,
        onAction: trackInputAction,
      );

      final DomHTMLElement input = editingStrategy!.activeDomElement;
      expect(input.style.getPropertyValue('forced-color-adjust'), 'none');
      // TODO(hterkelsen): Firefox does not support forced-color-adjust even
      // though it supports forced-colors. Safari doesn't support forced-colors
      // so this isn't a problem there.
    }, skip: isFirefox || isSafari);
  });
}

DomKeyboardEvent dispatchKeyboardEvent(DomEventTarget target, String type, {required int keyCode}) {
  final Object jsKeyboardEvent = js_util.getProperty<Object>(domWindow, 'KeyboardEvent');
  final List<dynamic> eventArgs = <dynamic>[
    type,
    js_util.jsify(<String, dynamic>{'keyCode': keyCode, 'cancelable': true}),
  ];
  final DomKeyboardEvent event = js_util.callConstructor<DomKeyboardEvent>(
    jsKeyboardEvent,
    eventArgs,
  );
  target.dispatchEvent(event);

  return event;
}

MethodCall configureSetStyleMethodCall(
  int fontSize,
  String fontFamily,
  int textAlignIndex,
  int? fontWeightIndex,
  int textDirectionIndex,
) {
  return MethodCall('TextInput.setStyle', <String, dynamic>{
    'fontSize': fontSize,
    'fontFamily': fontFamily,
    'textAlignIndex': textAlignIndex,
    'fontWeightIndex': fontWeightIndex,
    'textDirectionIndex': textDirectionIndex,
  });
}

MethodCall configureSetSizeAndTransformMethodCall(int width, int height, List<double> transform) {
  return MethodCall('TextInput.setEditableSizeAndTransform', <String, dynamic>{
    'width': width,
    'height': height,
    'transform': transform,
  });
}

/// Will disable editing element which will also clean the backup DOM
/// element from the page.
void cleanTextEditingStrategy() {
  if (editingStrategy != null && editingStrategy!.isEnabled) {
    // Clean up all the DOM elements and event listeners.
    editingStrategy!.disable();
  }
}

void cleanTestFlags() {
  ui_web.browser.debugBrowserEngineOverride = null;
  ui_web.browser.debugOperatingSystemOverride = null;
}

void checkInputEditingState(DomElement? element, String text, int start, int end) {
  expect(element, isNotNull);
  expect(domInstanceOfString(element, 'HTMLInputElement'), true);
  final DomHTMLInputElement input = element! as DomHTMLInputElement;
  expect(defaultTextEditingRoot.ownerDocument?.activeElement, input);
  expect(input.value, text);
  expect(input.selectionStart, start);
  expect(input.selectionEnd, end);
}

/// In case of an exception backup DOM element(s) can still stay on the DOM.
void clearBackUpDomElementIfExists() {
  final List<DomElement> domElementsToRemove = <DomElement>[];
  if (defaultTextEditingRoot.querySelectorAll('input').isNotEmpty) {
    domElementsToRemove.addAll(defaultTextEditingRoot.querySelectorAll('input').cast<DomElement>());
  }
  if (defaultTextEditingRoot.querySelectorAll('textarea').isNotEmpty) {
    domElementsToRemove.addAll(
      defaultTextEditingRoot.querySelectorAll('textarea').cast<DomElement>(),
    );
  }
  domElementsToRemove.forEach(_removeNode);
}

void _removeNode(DomElement n) => n.remove();

void checkTextAreaEditingState(DomHTMLTextAreaElement textarea, String text, int start, int end) {
  expect(defaultTextEditingRoot.ownerDocument?.activeElement, textarea);
  expect(textarea.value, text);
  expect(textarea.selectionStart, start);
  expect(textarea.selectionEnd, end);
}

/// Creates an [InputConfiguration] for using in the tests.
///
/// For simplicity this method is using `autofillHint` as the `uniqueId` for
/// simplicity.
Map<String, dynamic> createFlutterConfig(
  String inputType, {
  int? viewId,
  bool readOnly = false,
  bool obscureText = false,
  bool autocorrect = true,
  String textCapitalization = 'TextCapitalization.none',
  String? inputAction,
  bool autofillEnabled = true,
  String? autofillHint,
  String? placeholderText,
  List<String>? autofillHintsForFields,
  bool decimal = false,
  bool enableDeltaModel = false,
  bool isMultiline = false,
}) {
  return <String, dynamic>{
    'inputType': <String, dynamic>{
      'name': 'TextInputType.$inputType',
      if (decimal) 'decimal': true,
      if (isMultiline) 'isMultiline': true,
    },
    if (viewId != null) 'viewId': viewId,
    'readOnly': readOnly,
    'obscureText': obscureText,
    'autocorrect': autocorrect,
    'inputAction': inputAction ?? 'TextInputAction.done',
    'textCapitalization': textCapitalization,
    if (autofillEnabled)
      'autofill': createAutofillInfo(
        autofillHint,
        autofillHint ?? 'bogusId',
        placeholder: placeholderText,
      ),
    if (autofillEnabled && autofillHintsForFields != null)
      'fields': createFieldValues(autofillHintsForFields, autofillHintsForFields),
    'enableDeltaModel': enableDeltaModel,
  };
}

Map<String, dynamic> createAutofillInfo(String? hint, String uniqueId, {String? placeholder}) =>
    <String, dynamic>{
      'uniqueIdentifier': uniqueId,
      if (hint != null) 'hints': <String>[hint],
      if (placeholder != null) 'hintText': placeholder,
      'editingValue': <String, dynamic>{
        'text': 'Test',
        'selectionBase': 0,
        'selectionExtent': 0,
        'selectionAffinity': 'TextAffinity.downstream',
        'selectionIsDirectional': false,
        'composingBase': -1,
        'composingExtent': -1,
      },
    };

List<dynamic> createFieldValues(List<String> hints, List<String> uniqueIds) {
  final List<dynamic> testFields = <dynamic>[];

  expect(hints.length, equals(uniqueIds.length));

  for (int i = 0; i < hints.length; i++) {
    testFields.add(createOneFieldValue(hints[i], uniqueIds[i]));
  }

  return testFields;
}

Map<String, dynamic> createOneFieldValue(String hint, String uniqueId) => <String, dynamic>{
  'inputType': <String, dynamic>{'name': 'TextInputType.text', 'signed': null, 'decimal': null},
  'textCapitalization': 'TextCapitalization.none',
  'autofill': createAutofillInfo(hint, uniqueId),
};

/// In order to not leak test state, clean up the forms from dom if any remains.
void clearForms() {
  while (defaultTextEditingRoot.querySelectorAll('form').isNotEmpty) {
    defaultTextEditingRoot.querySelectorAll('form').last.remove();
  }
  formsOnTheDom.clear();
}

/// Waits until the text strategy closes and moves the focus accordingly.
Future<void> waitForTextStrategyStopPropagation() async {
  await Future<void>.delayed(Duration.zero);
}

class GlobalTextEditingStrategySpy extends GloballyPositionedTextEditingStrategy {
  GlobalTextEditingStrategySpy(super.owner);

  int placeElementCount = 0;

  @override
  void placeElement() {
    placeElementCount++;
    super.placeElement();
  }
}
