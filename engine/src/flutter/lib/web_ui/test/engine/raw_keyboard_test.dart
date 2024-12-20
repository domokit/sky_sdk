// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:quiver/testing/async.dart';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('RawKeyboard', () {
    /// Used to save and restore [PlatformDispatcher.onPlatformMessage] after each test.
    ui.PlatformMessageCallback? savedCallback;

    setUp(() {
      savedCallback = ui.PlatformDispatcher.instance.onPlatformMessage;
    });

    tearDown(() {
      ui.PlatformDispatcher.instance.onPlatformMessage = savedCallback;
    });

    test('initializes and disposes', () {
      expect(RawKeyboard.instance, isNull);
      RawKeyboard.initialize();
      expect(RawKeyboard.instance, isA<RawKeyboard>());
      RawKeyboard.instance!.dispose();
      expect(RawKeyboard.instance, isNull);
    });

    test('dispatches keyup to flutter/keyevent channel', () {
      RawKeyboard.initialize();

      String? channelReceived;
      Map<String, dynamic>? dataReceived;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        channelReceived = channel;
        dataReceived = const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>?;
      };

      DomKeyboardEvent event;

      // Dispatch a keydown event first so that KeyboardBinding will recognize the keyup event.
      // and will not set preventDefault on it.
      event = dispatchKeyboardEvent('keydown', key: 'SomeKey', code: 'SomeCode', keyCode: 1);

      event = dispatchKeyboardEvent('keyup', key: 'SomeKey', code: 'SomeCode', keyCode: 1);

      expect(event.defaultPrevented, isFalse);
      expect(channelReceived, 'flutter/keyevent');
      expect(dataReceived, <String, dynamic>{
        'type': 'keyup',
        'keymap': 'web',
        'code': 'SomeCode',
        'location': 0,
        'key': 'SomeKey',
        'metaState': 0x0,
        'keyCode': 1,
      });

      RawKeyboard.instance!.dispose();
    });

    test('dispatches keydown to flutter/keyevent channel', () {
      RawKeyboard.initialize();

      String? channelReceived;
      Map<String, dynamic>? dataReceived;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        channelReceived = channel;
        dataReceived = const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>?;
      };

      DomKeyboardEvent event;

      event = dispatchKeyboardEvent('keydown', key: 'SomeKey', code: 'SomeCode', keyCode: 1);

      expect(channelReceived, 'flutter/keyevent');
      expect(dataReceived, <String, dynamic>{
        'type': 'keydown',
        'keymap': 'web',
        'code': 'SomeCode',
        'key': 'SomeKey',
        'location': 0,
        'metaState': 0x0,
        'keyCode': 1,
      });
      expect(event.defaultPrevented, isFalse);

      RawKeyboard.instance!.dispose();
    });

    test('dispatches correct meta state', () {
      RawKeyboard.initialize();

      Map<String, dynamic>? dataReceived;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        dataReceived = const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>?;
      };

      DomKeyboardEvent event;

      event = dispatchKeyboardEvent(
        'keydown',
        key: 'SomeKey',
        code: 'SomeCode',
        isControlPressed: true,
      );
      expect(event.defaultPrevented, isFalse);
      expect(dataReceived, <String, dynamic>{
        'type': 'keydown',
        'keymap': 'web',
        'code': 'SomeCode',
        'key': 'SomeKey',
        'location': 0,
        //          ctrl
        'metaState': 0x4,
        'keyCode': 0,
      });

      event = dispatchKeyboardEvent(
        'keydown',
        key: 'SomeKey',
        code: 'SomeCode',
        isShiftPressed: true,
        isAltPressed: true,
        isMetaPressed: true,
      );
      expect(event.defaultPrevented, isFalse);
      expect(dataReceived, <String, dynamic>{
        'type': 'keydown',
        'keymap': 'web',
        'code': 'SomeCode',
        'key': 'SomeKey',
        'location': 0,
        //          shift  alt   meta
        'metaState': 0x1 | 0x2 | 0x8,
        'keyCode': 0,
      });

      RawKeyboard.instance!.dispose();
    });

    // Regression test for https://github.com/flutter/flutter/issues/125672.
    test(
      'updates meta state for Meta key and wrong DOM event metaKey value (Linux)',
      () {
        RawKeyboard.initialize();

        Map<String, dynamic>? dataReceived;
        ui.PlatformDispatcher.instance.onPlatformMessage = (
          String channel,
          ByteData? data,
          ui.PlatformMessageResponseCallback? callback,
        ) {
          dataReceived = const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>?;
        };

        // Purposely send an incoherent DOM event where Meta key is pressed but event.metaKey is not set to true.
        final DomKeyboardEvent event = dispatchKeyboardEvent(
          'keydown',
          key: 'Meta',
          code: 'MetaLeft',
        );
        expect(event.defaultPrevented, isFalse);
        expect(dataReceived, <String, dynamic>{
          'type': 'keydown',
          'keymap': 'web',
          'code': 'MetaLeft',
          'key': 'Meta',
          'location': 0,
          'metaState': 0x8,
          'keyCode': 0,
        });
        RawKeyboard.instance!.dispose();
      },
      skip: ui_web.browser.operatingSystem != ui_web.OperatingSystem.linux,
    );

    // Regression test for https://github.com/flutter/flutter/issues/141186.
    test('updates meta state for Meta key seen as "Process" key', () {
      RawKeyboard.initialize();

      Map<String, dynamic>? dataReceived;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        dataReceived = const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>?;
      };

      // Purposely send a DOM event where Meta key is pressed but event.metaKey is not set to true.
      // This can happen when the Meta key is seen as the 'Process' key.
      final DomKeyboardEvent event = dispatchKeyboardEvent(
        'keydown',
        key: 'Process',
        code: 'MetaLeft',
        location: 1,
        keyCode: 229,
      );
      expect(event.defaultPrevented, isFalse);
      expect(dataReceived, <String, dynamic>{
        'type': 'keydown',
        'keymap': 'web',
        'code': 'MetaLeft',
        'key': 'Process',
        'location': 1,
        'metaState': 0x8,
        'keyCode': 229,
      });
      RawKeyboard.instance!.dispose();
    });

    test('dispatches repeat events', () {
      RawKeyboard.initialize();

      final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
      };

      DomKeyboardEvent event;

      event = dispatchKeyboardEvent('keydown', key: 'SomeKey', code: 'SomeCode', repeat: true);
      expect(event.defaultPrevented, isFalse);

      event = dispatchKeyboardEvent('keydown', key: 'SomeKey', code: 'SomeCode', repeat: true);
      expect(event.defaultPrevented, isFalse);

      event = dispatchKeyboardEvent('keydown', key: 'SomeKey', code: 'SomeCode', repeat: true);
      expect(event.defaultPrevented, isFalse);

      final Map<String, dynamic> expectedMessage = <String, dynamic>{
        'type': 'keydown',
        'keymap': 'web',
        'code': 'SomeCode',
        'key': 'SomeKey',
        'location': 0,
        'metaState': 0,
        'keyCode': 0,
      };
      expect(messages, <Map<String, dynamic>>[expectedMessage, expectedMessage, expectedMessage]);

      RawKeyboard.instance!.dispose();
    });

    test('stops dispatching events after dispose', () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
      };

      dispatchKeyboardEvent('keydown');
      expect(count, 1);
      dispatchKeyboardEvent('keyup');
      expect(count, 2);

      RawKeyboard.instance!.dispose();
      expect(RawKeyboard.instance, isNull);

      // No more event dispatching.
      dispatchKeyboardEvent('keydown');
      expect(count, 2);
      dispatchKeyboardEvent('keyup');
      expect(count, 2);
    });

    test('prevents default when key is handled by the framework', () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
        final ByteData response =
            const JSONMessageCodec().encodeMessage(<String, dynamic>{'handled': true})!;
        callback!(response);
      };

      final DomKeyboardEvent event = dispatchKeyboardEvent('keydown', key: 'Tab', code: 'Tab');

      expect(event.defaultPrevented, isTrue);
      expect(count, 1);

      RawKeyboard.instance!.dispose();
    });

    test("Doesn't prevent default when key is not handled by the framework", () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
        final ByteData response =
            const JSONMessageCodec().encodeMessage(<String, dynamic>{'handled': false})!;
        callback!(response);
      };

      final DomKeyboardEvent event = dispatchKeyboardEvent('keydown', key: 'Tab', code: 'Tab');

      expect(event.defaultPrevented, isFalse);
      expect(count, 1);

      RawKeyboard.instance!.dispose();
    });

    test('keyboard events should be triggered on text fields', () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
      };

      useTextEditingElement((DomElement element) {
        final DomKeyboardEvent event = dispatchKeyboardEvent(
          'keydown',
          key: 'SomeKey',
          code: 'SomeCode',
          target: element,
        );

        expect(event.defaultPrevented, isFalse);
        expect(count, 1);
      });

      RawKeyboard.instance!.dispose();
    });

    test('the "Tab" key should never be ignored when it is not a part of IME composition', () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
        final ByteData response =
            const JSONMessageCodec().encodeMessage(<String, dynamic>{'handled': true})!;
        callback!(response);
      };

      useTextEditingElement((DomElement element) {
        final DomKeyboardEvent event = dispatchKeyboardEvent(
          'keydown',
          key: 'Tab',
          code: 'Tab',
          target: element,
        );

        expect(event.defaultPrevented, isTrue);
        expect(count, 1);
      });

      RawKeyboard.instance!.dispose();
    });

    test('Ignores event when Tab key is hit during IME composition', () {
      RawKeyboard.initialize();

      int count = 0;
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        count += 1;
        final ByteData response =
            const JSONMessageCodec().encodeMessage(<String, dynamic>{'handled': true})!;
        callback!(response);
      };

      useTextEditingElement((DomElement element) {
        dispatchKeyboardEvent(
          'keydown',
          key: 'Tab',
          code: 'Tab',
          target: element,
          isComposing: true,
        );

        expect(count, 0); // no message sent to framework
      });

      RawKeyboard.instance!.dispose();
    });

    testFakeAsync('On macOS, synthesize keyup when shortcut is handled by the system', (
      FakeAsync async,
    ) {
      // This can happen when the user clicks `cmd+alt+i` to open devtools. Here
      // is the sequence we receive from the browser in such case:
      //
      // keydown(cmd) -> keydown(alt) -> keydown(i) -> keyup(alt) -> keyup(cmd)
      //
      // There's no `keyup(i)`. The web engine is expected to synthesize a
      // `keyup(i)` event.
      RawKeyboard.initialize(onMacOs: true);

      final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
      };

      dispatchKeyboardEvent(
        'keydown',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isMetaPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'Alt',
        code: 'AltLeft',
        location: 1,
        isMetaPressed: true,
        isAltPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'i',
        code: 'KeyI',
        isMetaPressed: true,
        isAltPressed: true,
      );
      async.elapse(const Duration(milliseconds: 10));
      dispatchKeyboardEvent(
        'keyup',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isAltPressed: true,
      );
      dispatchKeyboardEvent('keyup', key: 'Alt', code: 'AltLeft', location: 1);
      // Notice no `keyup` for "i".

      expect(messages, <Map<String, dynamic>>[
        <String, dynamic>{
          'type': 'keydown',
          'keymap': 'web',
          'key': 'Meta',
          'code': 'MetaLeft',
          'location': 1,
          //           meta
          'metaState': 0x8,
          'keyCode': 0,
        },
        <String, dynamic>{
          'type': 'keydown',
          'keymap': 'web',
          'key': 'Alt',
          'code': 'AltLeft',
          'location': 1,
          //           alt   meta
          'metaState': 0x2 | 0x8,
          'keyCode': 0,
        },
        <String, dynamic>{
          'type': 'keydown',
          'keymap': 'web',
          'key': 'i',
          'code': 'KeyI',
          'location': 0,
          //           alt   meta
          'metaState': 0x2 | 0x8,
          'keyCode': 0,
        },
        <String, dynamic>{
          'type': 'keyup',
          'keymap': 'web',
          'key': 'Meta',
          'code': 'MetaLeft',
          'location': 1,
          //           alt
          'metaState': 0x2,
          'keyCode': 0,
        },
        <String, dynamic>{
          'type': 'keyup',
          'keymap': 'web',
          'key': 'Alt',
          'code': 'AltLeft',
          'location': 1,
          'metaState': 0x0,
          'keyCode': 0,
        },
      ]);
      messages.clear();

      // Still too eary to synthesize a keyup event.
      async.elapse(const Duration(milliseconds: 50));
      expect(messages, isEmpty);

      async.elapse(const Duration(seconds: 3));
      expect(messages, <Map<String, dynamic>>[
        <String, dynamic>{
          'type': 'keyup',
          'keymap': 'web',
          'key': 'i',
          'code': 'KeyI',
          'location': 0,
          'metaState': 0x0,
          'keyCode': 0,
        },
      ]);

      RawKeyboard.instance!.dispose();
    });

    testFakeAsync('On macOS, do not synthesize keyup when we receive repeat events', (
      FakeAsync async,
    ) {
      RawKeyboard.initialize(onMacOs: true);

      final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
      };

      dispatchKeyboardEvent(
        'keydown',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isMetaPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'Alt',
        code: 'AltLeft',
        location: 1,
        isMetaPressed: true,
        isAltPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'i',
        code: 'KeyI',
        isMetaPressed: true,
        isAltPressed: true,
      );
      async.elapse(const Duration(milliseconds: 10));
      dispatchKeyboardEvent(
        'keyup',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isAltPressed: true,
      );
      dispatchKeyboardEvent('keyup', key: 'Alt', code: 'AltLeft', location: 1);
      // Notice no `keyup` for "i".

      messages.clear();

      // Spend more than 2 seconds sending repeat events and make sure no
      // keyup was synthesized.
      for (int i = 0; i < 20; i++) {
        async.elapse(const Duration(milliseconds: 100));
        dispatchKeyboardEvent('keydown', key: 'i', code: 'KeyI', repeat: true);
      }

      // There should be no synthesized keyup.
      expect(messages, hasLength(20));
      for (int i = 0; i < 20; i++) {
        expect(messages[i], <String, dynamic>{
          'type': 'keydown',
          'keymap': 'web',
          'key': 'i',
          'code': 'KeyI',
          'location': 0,
          'metaState': 0x0,
          'keyCode': 0,
        });
      }
      messages.clear();

      RawKeyboard.instance!.dispose();
    });

    testFakeAsync(
      'On macOS, do not synthesize keyup when keys are not affected by meta modifiers',
      (FakeAsync async) {
        RawKeyboard.initialize();

        final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
        ui.PlatformDispatcher.instance.onPlatformMessage = (
          String channel,
          ByteData? data,
          ui.PlatformMessageResponseCallback? callback,
        ) {
          messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
        };

        dispatchKeyboardEvent('keydown', key: 'i', code: 'KeyI');
        dispatchKeyboardEvent('keydown', key: 'o', code: 'KeyO');
        messages.clear();

        // Wait for a long-enough period of time and no events
        // should be synthesized
        async.elapse(const Duration(seconds: 3));
        expect(messages, hasLength(0));

        RawKeyboard.instance!.dispose();
      },
    );

    testFakeAsync('On macOS, do not synthesize keyup for meta keys', (FakeAsync async) {
      RawKeyboard.initialize(onMacOs: true);

      final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
      };

      dispatchKeyboardEvent(
        'keydown',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isMetaPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'Alt',
        code: 'AltLeft',
        location: 1,
        isMetaPressed: true,
        isAltPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'i',
        code: 'KeyI',
        isMetaPressed: true,
        isAltPressed: true,
      );
      async.elapse(const Duration(milliseconds: 10));
      dispatchKeyboardEvent(
        'keyup',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isAltPressed: true,
      );
      // Notice no `keyup` for "AltLeft" and "i".

      messages.clear();

      // There has been no repeat events for "AltLeft" nor "i". Only "i" should
      // synthesize a keyup event.
      async.elapse(const Duration(seconds: 3));
      expect(messages, <Map<String, dynamic>>[
        <String, dynamic>{
          'type': 'keyup',
          'keymap': 'web',
          'key': 'i',
          'code': 'KeyI',
          'location': 0,
          //           alt
          'metaState': 0x2,
          'keyCode': 0,
        },
      ]);

      RawKeyboard.instance!.dispose();
    });

    testFakeAsync('On non-macOS, do not synthesize keyup for shortcuts', (FakeAsync async) {
      RawKeyboard.initialize(); // onMacOs: false

      final List<Map<String, dynamic>> messages = <Map<String, dynamic>>[];
      ui.PlatformDispatcher.instance.onPlatformMessage = (
        String channel,
        ByteData? data,
        ui.PlatformMessageResponseCallback? callback,
      ) {
        messages.add(const JSONMessageCodec().decodeMessage(data) as Map<String, dynamic>);
      };

      dispatchKeyboardEvent(
        'keydown',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isMetaPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'Alt',
        code: 'AltLeft',
        location: 1,
        isMetaPressed: true,
        isAltPressed: true,
      );
      dispatchKeyboardEvent(
        'keydown',
        key: 'i',
        code: 'KeyI',
        isMetaPressed: true,
        isAltPressed: true,
      );
      async.elapse(const Duration(milliseconds: 10));
      dispatchKeyboardEvent(
        'keyup',
        key: 'Meta',
        code: 'MetaLeft',
        location: 1,
        isAltPressed: true,
      );
      dispatchKeyboardEvent('keyup', key: 'Alt', code: 'AltLeft', location: 1);
      // Notice no `keyup` for "i".

      expect(messages, hasLength(5));
      messages.clear();

      // Never synthesize keyup events.
      async.elapse(const Duration(seconds: 3));
      expect(messages, isEmpty);

      RawKeyboard.instance!.dispose();
    });
  });
}

typedef ElementCallback = void Function(DomElement element);

void useTextEditingElement(ElementCallback callback) {
  final DomHTMLInputElement input = createDomHTMLInputElement();
  input.classList.add(HybridTextEditing.textEditingClass);

  try {
    domDocument.body!.append(input);
    callback(input);
  } finally {
    input.remove();
  }
}

DomKeyboardEvent dispatchKeyboardEvent(
  String type, {
  DomEventTarget? target,
  String? key,
  String? code,
  int location = 0,
  bool repeat = false,
  bool isShiftPressed = false,
  bool isAltPressed = false,
  bool isControlPressed = false,
  bool isMetaPressed = false,
  bool isComposing = false,
  int keyCode = 0,
}) {
  target ??= domWindow;

  final DomKeyboardEvent event = createDomKeyboardEvent(type, <String, Object>{
    if (key != null) 'key': key,
    if (code != null) 'code': code,
    'location': location,
    'repeat': repeat,
    'shiftKey': isShiftPressed,
    'altKey': isAltPressed,
    'ctrlKey': isControlPressed,
    'metaKey': isMetaPressed,
    'isComposing': isComposing,
    'keyCode': keyCode,
    'bubbles': true,
    'cancelable': true,
  });
  target.dispatchEvent(event);

  return event;
}

typedef FakeAsyncTest = void Function(FakeAsync);

void testFakeAsync(String description, FakeAsyncTest fn) {
  test(description, () {
    FakeAsync().run(fn);
  });
}
