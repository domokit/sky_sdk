// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'HardwareKeyboard records pressed keys and enabled locks',
    (WidgetTester tester) async {
      await simulateKeyDownEvent(LogicalKeyboardKey.numLock, platform: 'windows');
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock}),
      );
      expect(
        HardwareKeyboard.instance.lockModesEnabled,
        equals(<KeyboardLockMode>{KeyboardLockMode.numLock}),
      );

      await simulateKeyDownEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock, PhysicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock, LogicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.lockModesEnabled,
        equals(<KeyboardLockMode>{KeyboardLockMode.numLock}),
      );

      await simulateKeyRepeatEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock, PhysicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock, LogicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.lockModesEnabled,
        equals(<KeyboardLockMode>{KeyboardLockMode.numLock}),
      );

      await simulateKeyUpEvent(LogicalKeyboardKey.numLock);
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.lockModesEnabled,
        equals(<KeyboardLockMode>{KeyboardLockMode.numLock}),
      );

      await simulateKeyDownEvent(LogicalKeyboardKey.numLock, platform: 'windows');
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock, PhysicalKeyboardKey.numpad1}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock, LogicalKeyboardKey.numpad1}),
      );
      expect(HardwareKeyboard.instance.lockModesEnabled, equals(<KeyboardLockMode>{}));

      await simulateKeyUpEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(
        HardwareKeyboard.instance.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock}),
      );
      expect(
        HardwareKeyboard.instance.logicalKeysPressed,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock}),
      );
      expect(HardwareKeyboard.instance.lockModesEnabled, equals(<KeyboardLockMode>{}));

      await simulateKeyUpEvent(LogicalKeyboardKey.numLock, platform: 'windows');
      expect(HardwareKeyboard.instance.physicalKeysPressed, equals(<PhysicalKeyboardKey>{}));
      expect(HardwareKeyboard.instance.logicalKeysPressed, equals(<LogicalKeyboardKey>{}));
      expect(HardwareKeyboard.instance.lockModesEnabled, equals(<KeyboardLockMode>{}));
    },
    variant: KeySimulatorTransitModeVariant.keyDataThenRawKeyData(),
  );

  testWidgets(
    'KeyEvent can tell which keys are pressed',
    (WidgetTester tester) async {
      await tester.pumpWidget(const Focus(autofocus: true, child: SizedBox()));
      await tester.pump();

      await simulateKeyDownEvent(LogicalKeyboardKey.numLock, platform: 'windows');

      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numLock), isTrue);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numLock), isTrue);

      await simulateKeyDownEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numpad1), isTrue);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numpad1), isTrue);

      await simulateKeyRepeatEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numpad1), isTrue);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numpad1), isTrue);

      await simulateKeyUpEvent(LogicalKeyboardKey.numLock);
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numpad1), isTrue);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numpad1), isTrue);

      await simulateKeyDownEvent(LogicalKeyboardKey.numLock, platform: 'windows');
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numLock), isTrue);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numLock), isTrue);

      await simulateKeyUpEvent(LogicalKeyboardKey.numpad1, platform: 'windows');
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numpad1), isFalse);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numpad1), isFalse);

      await simulateKeyUpEvent(LogicalKeyboardKey.numLock, platform: 'windows');
      expect(HardwareKeyboard.instance.isPhysicalKeyPressed(PhysicalKeyboardKey.numLock), isFalse);
      expect(HardwareKeyboard.instance.isLogicalKeyPressed(LogicalKeyboardKey.numLock), isFalse);
    },
    variant: KeySimulatorTransitModeVariant.keyDataThenRawKeyData(),
  );

  testWidgets('KeyboardManager synthesizes modifier keys in rawKeyData mode', (
    WidgetTester tester,
  ) async {
    final List<KeyEvent> events = <KeyEvent>[];
    HardwareKeyboard.instance.addHandler((KeyEvent event) {
      events.add(event);
      return false;
    });
    // While ShiftLeft is held (the event of which was skipped), press keyA.
    final Map<String, dynamic> rawMessage =
        kIsWeb
            ? (KeyEventSimulator.getKeyData(LogicalKeyboardKey.keyA, platform: 'web')
              ..['metaState'] = RawKeyEventDataWeb.modifierShift)
            : (KeyEventSimulator.getKeyData(LogicalKeyboardKey.keyA, platform: 'android')
              ..['metaState'] =
                  RawKeyEventDataAndroid.modifierLeftShift | RawKeyEventDataAndroid.modifierShift);
    tester.binding.keyEventManager.handleRawKeyMessage(rawMessage);
    expect(events, hasLength(2));
    expect(events[0].physicalKey, PhysicalKeyboardKey.shiftLeft);
    expect(events[0].logicalKey, LogicalKeyboardKey.shiftLeft);
    expect(events[0].synthesized, true);
    expect(events[1].physicalKey, PhysicalKeyboardKey.keyA);
    expect(events[1].logicalKey, LogicalKeyboardKey.keyA);
    expect(events[1].synthesized, false);
  });

  testWidgets('Dispatch events to all handlers', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final List<int> logs = <int>[];

    await tester.pumpWidget(
      KeyboardListener(
        autofocus: true,
        focusNode: focusNode,
        child: Container(),
        onKeyEvent: (KeyEvent event) {
          logs.add(1);
        },
      ),
    );

    // Only the Service binding handler.

    expect(await simulateKeyDownEvent(LogicalKeyboardKey.keyA), false);
    expect(logs, <int>[1]);
    logs.clear();

    // Add a handler.

    bool handler2Result = false;
    bool handler2(KeyEvent event) {
      logs.add(2);
      return handler2Result;
    }

    HardwareKeyboard.instance.addHandler(handler2);

    expect(await simulateKeyUpEvent(LogicalKeyboardKey.keyA), false);
    expect(logs, <int>[2, 1]);
    logs.clear();

    handler2Result = true;

    expect(await simulateKeyDownEvent(LogicalKeyboardKey.keyA), true);
    expect(logs, <int>[2, 1]);
    logs.clear();

    // Add another handler.

    handler2Result = false;
    bool handler3Result = false;
    bool handler3(KeyEvent event) {
      logs.add(3);
      return handler3Result;
    }

    HardwareKeyboard.instance.addHandler(handler3);

    expect(await simulateKeyUpEvent(LogicalKeyboardKey.keyA), false);
    expect(logs, <int>[2, 3, 1]);
    logs.clear();

    handler2Result = true;

    expect(await simulateKeyDownEvent(LogicalKeyboardKey.keyA), true);
    expect(logs, <int>[2, 3, 1]);
    logs.clear();

    handler3Result = true;

    expect(await simulateKeyUpEvent(LogicalKeyboardKey.keyA), true);
    expect(logs, <int>[2, 3, 1]);
    logs.clear();

    // Add handler2 again.

    HardwareKeyboard.instance.addHandler(handler2);

    handler3Result = false;
    handler2Result = false;
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.keyA), false);
    expect(logs, <int>[2, 3, 2, 1]);
    logs.clear();

    handler2Result = true;
    expect(await simulateKeyUpEvent(LogicalKeyboardKey.keyA), true);
    expect(logs, <int>[2, 3, 2, 1]);
    logs.clear();

    // Remove handler2 once.

    HardwareKeyboard.instance.removeHandler(handler2);
    expect(await simulateKeyDownEvent(LogicalKeyboardKey.keyA), true);
    expect(logs, <int>[3, 2, 1]);
    logs.clear();
  }, variant: KeySimulatorTransitModeVariant.all());

  // Regression test for https://github.com/flutter/flutter/issues/99196 .
  //
  // In rawKeyData mode, if a key down event is dispatched but immediately
  // synthesized to be released, the old logic would trigger a Null check
  // _CastError on _hardwareKeyboard.lookUpLayout(key). The original scenario
  // that this is triggered on Android is unknown. Here we make up a scenario
  // where a ShiftLeft key down is dispatched but the modifier bit is not set.
  testWidgets(
    'Correctly convert down events that are synthesized released',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      final List<KeyEvent> events = <KeyEvent>[];

      await tester.pumpWidget(
        KeyboardListener(
          autofocus: true,
          focusNode: focusNode,
          child: Container(),
          onKeyEvent: (KeyEvent event) {
            events.add(event);
          },
        ),
      );

      // Dispatch an arbitrary event to bypass the pressedKeys check.
      await simulateKeyDownEvent(LogicalKeyboardKey.keyA, platform: 'web');

      // Dispatch an
      final Map<String, dynamic> data2 = KeyEventSimulator.getKeyData(
        LogicalKeyboardKey.shiftLeft,
        platform: 'web',
      )..['metaState'] = 0;
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        SystemChannels.keyEvent.name,
        SystemChannels.keyEvent.codec.encodeMessage(data2),
        (ByteData? data) {},
      );

      expect(events, hasLength(3));
      expect(events[1], isA<KeyDownEvent>());
      expect(events[1].logicalKey, LogicalKeyboardKey.shiftLeft);
      expect(events[1].synthesized, false);
      expect(events[2], isA<KeyUpEvent>());
      expect(events[2].logicalKey, LogicalKeyboardKey.shiftLeft);
      expect(events[2].synthesized, true);
      expect(
        ServicesBinding.instance.keyboard.physicalKeysPressed,
        equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.keyA}),
      );
    },
    variant: const KeySimulatorTransitModeVariant(<KeyDataTransitMode>{
      KeyDataTransitMode.rawKeyData,
    }),
  );

  testWidgets(
    'Instantly dispatch synthesized key events when the queue is empty',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      final List<int> logs = <int>[];

      await tester.pumpWidget(
        KeyboardListener(
          autofocus: true,
          focusNode: focusNode,
          child: Container(),
          onKeyEvent: (KeyEvent event) {
            logs.add(1);
          },
        ),
      );
      ServicesBinding.instance.keyboard.addHandler((KeyEvent event) {
        logs.add(2);
        return false;
      });

      // Dispatch a solitary synthesized event.
      expect(
        ServicesBinding.instance.keyEventManager.handleKeyData(
          ui.KeyData(
            timeStamp: Duration.zero,
            type: ui.KeyEventType.down,
            logical: LogicalKeyboardKey.keyA.keyId,
            physical: PhysicalKeyboardKey.keyA.usbHidUsage,
            character: null,
            synthesized: true,
          ),
        ),
        false,
      );
      expect(logs, <int>[2, 1]);
      logs.clear();
    },
    variant: KeySimulatorTransitModeVariant.keyDataThenRawKeyData(),
  );

  testWidgets(
    'Postpone synthesized key events when the queue is not empty',
    (WidgetTester tester) async {
      final FocusNode keyboardListenerFocusNode = FocusNode();
      addTearDown(keyboardListenerFocusNode.dispose);
      final FocusNode rawKeyboardListenerFocusNode = FocusNode();
      addTearDown(rawKeyboardListenerFocusNode.dispose);
      final List<String> logs = <String>[];

      await tester.pumpWidget(
        RawKeyboardListener(
          focusNode: rawKeyboardListenerFocusNode,
          onKey: (RawKeyEvent event) {
            logs.add('${event.runtimeType}');
          },
          child: KeyboardListener(
            autofocus: true,
            focusNode: keyboardListenerFocusNode,
            child: Container(),
            onKeyEvent: (KeyEvent event) {
              logs.add('${event.runtimeType}');
            },
          ),
        ),
      );

      // On macOS, a CapsLock tap yields a down event and a synthesized up event.
      expect(
        ServicesBinding.instance.keyEventManager.handleKeyData(
          ui.KeyData(
            timeStamp: Duration.zero,
            type: ui.KeyEventType.down,
            logical: LogicalKeyboardKey.capsLock.keyId,
            physical: PhysicalKeyboardKey.capsLock.usbHidUsage,
            character: null,
            synthesized: false,
          ),
        ),
        false,
      );
      expect(
        ServicesBinding.instance.keyEventManager.handleKeyData(
          ui.KeyData(
            timeStamp: Duration.zero,
            type: ui.KeyEventType.up,
            logical: LogicalKeyboardKey.capsLock.keyId,
            physical: PhysicalKeyboardKey.capsLock.usbHidUsage,
            character: null,
            synthesized: true,
          ),
        ),
        false,
      );
      expect(
        await ServicesBinding.instance.keyEventManager.handleRawKeyMessage(<String, dynamic>{
          'type': 'keydown',
          'keymap': 'macos',
          'keyCode': 0x00000039,
          'characters': '',
          'charactersIgnoringModifiers': '',
          'modifiers': 0x10000,
        }),
        equals(<String, dynamic>{'handled': false}),
      );

      expect(logs, <String>['RawKeyDownEvent', 'KeyDownEvent', 'KeyUpEvent']);
      logs.clear();
    },
    variant: KeySimulatorTransitModeVariant.keyDataThenRawKeyData(),
  );

  // The first key data received from the engine might be an empty key data.
  // In that case, the key data should not be converted to any [KeyEvent]s,
  // but is only used so that *a* key data comes before the raw key message
  // and makes [KeyEventManager] infer [KeyDataTransitMode.keyDataThenRawKeyData].
  testWidgets('Empty keyData yields no event but triggers inference', (WidgetTester tester) async {
    final List<KeyEvent> events = <KeyEvent>[];
    final List<RawKeyEvent> rawEvents = <RawKeyEvent>[];
    tester.binding.keyboard.addHandler((KeyEvent event) {
      events.add(event);
      return true;
    });
    RawKeyboard.instance.addListener((RawKeyEvent event) {
      rawEvents.add(event);
    });
    tester.binding.keyEventManager.handleKeyData(
      const ui.KeyData(
        type: ui.KeyEventType.down,
        timeStamp: Duration.zero,
        logical: 0,
        physical: 0,
        character: 'a',
        synthesized: false,
      ),
    );
    tester.binding.keyEventManager.handleRawKeyMessage(<String, dynamic>{
      'type': 'keydown',
      'keymap': 'windows',
      'keyCode': 0x04,
      'scanCode': 0x04,
      'characterCodePoint': 0,
      'modifiers': 0,
    });
    expect(events.length, 0);
    expect(rawEvents.length, 1);

    // Dispatch another key data to ensure it's in
    // [KeyDataTransitMode.keyDataThenRawKeyData] mode (otherwise assertion
    // will be thrown upon a KeyData).
    tester.binding.keyEventManager.handleKeyData(
      const ui.KeyData(
        type: ui.KeyEventType.down,
        timeStamp: Duration.zero,
        logical: 0x22,
        physical: 0x70034,
        character: '"',
        synthesized: false,
      ),
    );
    tester.binding.keyEventManager.handleRawKeyMessage(<String, dynamic>{
      'type': 'keydown',
      'keymap': 'windows',
      'keyCode': 0x04,
      'scanCode': 0x04,
      'characterCodePoint': 0,
      'modifiers': 0,
    });
    expect(events.length, 1);
    expect(rawEvents.length, 2);
  });

  testWidgets('Exceptions from keyMessageHandler are caught and reported', (
    WidgetTester tester,
  ) async {
    final KeyMessageHandler? oldKeyMessageHandler =
        tester.binding.keyEventManager.keyMessageHandler;
    addTearDown(() {
      tester.binding.keyEventManager.keyMessageHandler = oldKeyMessageHandler;
    });

    // When keyMessageHandler throws an error...
    tester.binding.keyEventManager.keyMessageHandler = (KeyMessage message) {
      throw 1;
    };

    // Simulate a key down event.
    FlutterErrorDetails? record;
    await _runWhileOverridingOnError(
      () => simulateKeyDownEvent(LogicalKeyboardKey.keyA),
      onError: (FlutterErrorDetails details) {
        record = details;
      },
    );

    // ... the error should be caught.
    expect(record, isNotNull);
    expect(record!.exception, 1);
    final Map<String, DiagnosticsNode> infos = _groupDiagnosticsByName(
      record!.informationCollector!(),
    );
    expect(infos['KeyMessage'], isA<DiagnosticsProperty<KeyMessage>>());

    // But the exception should not interrupt recording the state.
    // Now the keyMessageHandler no longer throws an error.
    tester.binding.keyEventManager.keyMessageHandler = null;
    record = null;

    // Simulate a key up event.
    await _runWhileOverridingOnError(
      () => simulateKeyUpEvent(LogicalKeyboardKey.keyA),
      onError: (FlutterErrorDetails details) {
        record = details;
      },
    );
    // If the previous state (key down) wasn't recorded, this key up event will
    // trigger assertions.
    expect(record, isNull);
  });

  testWidgets(
    'Exceptions from HardwareKeyboard handlers are caught and reported',
    (WidgetTester tester) async {
      bool throwingCallback(KeyEvent event) {
        throw 1;
      }

      // When the handler throws an error...
      HardwareKeyboard.instance.addHandler(throwingCallback);

      // Simulate a key down event.
      FlutterErrorDetails? record;
      await _runWhileOverridingOnError(
        () => simulateKeyDownEvent(LogicalKeyboardKey.keyA),
        onError: (FlutterErrorDetails details) {
          record = details;
        },
      );

      // ... the error should be caught.
      expect(record, isNotNull);
      expect(record!.exception, 1);
      final Map<String, DiagnosticsNode> infos = _groupDiagnosticsByName(
        record!.informationCollector!(),
      );
      expect(infos['Event'], isA<DiagnosticsProperty<KeyEvent>>());

      // But the exception should not interrupt recording the state.
      // Now the key handler no longer throws an error.
      HardwareKeyboard.instance.removeHandler(throwingCallback);
      record = null;

      // Simulate a key up event.
      await _runWhileOverridingOnError(
        () => simulateKeyUpEvent(LogicalKeyboardKey.keyA),
        onError: (FlutterErrorDetails details) {
          record = details;
        },
      );
      // If the previous state (key down) wasn't recorded, this key up event will
      // trigger assertions.
      expect(record, isNull);
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets('debugPrintKeyboardEvents causes logging of key events', (WidgetTester tester) async {
    final bool oldDebugPrintKeyboardEvents = debugPrintKeyboardEvents;
    final DebugPrintCallback oldDebugPrint = debugPrint;
    final StringBuffer messages = StringBuffer();
    debugPrint = (String? message, {int? wrapWidth}) {
      messages.writeln(message ?? '');
    };
    debugPrintKeyboardEvents = true;
    try {
      await simulateKeyDownEvent(LogicalKeyboardKey.keyA);
    } finally {
      debugPrintKeyboardEvents = oldDebugPrintKeyboardEvents;
      debugPrint = oldDebugPrint;
    }
    final String messagesStr = messages.toString();
    expect(messagesStr, contains('KEYBOARD: Key event received: '));
    expect(messagesStr, contains('KEYBOARD: Pressed state before processing the event:'));
    expect(messagesStr, contains('KEYBOARD: Pressed state after processing the event:'));
  });
}

Future<void> _runWhileOverridingOnError(
  AsyncCallback body, {
  required FlutterExceptionHandler onError,
}) async {
  final FlutterExceptionHandler? oldFlutterErrorOnError = FlutterError.onError;
  FlutterError.onError = onError;

  try {
    await body();
  } finally {
    FlutterError.onError = oldFlutterErrorOnError;
  }
}

Map<String, DiagnosticsNode> _groupDiagnosticsByName(Iterable<DiagnosticsNode> infos) {
  return Map<String, DiagnosticsNode>.fromIterable(
    infos,
    key: (dynamic node) => (node as DiagnosticsNode).name ?? '',
  );
}
