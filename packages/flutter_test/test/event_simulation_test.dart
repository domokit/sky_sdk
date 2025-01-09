// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const List<String> platforms = <String>['linux', 'macos', 'android', 'fuchsia'];

void _verifyKeyEvent<T extends KeyEvent>(
  KeyEvent event,
  PhysicalKeyboardKey physical,
  LogicalKeyboardKey logical,
  String? character,
) {
  expect(event, isA<T>());
  expect(event.physicalKey, physical);
  expect(event.logicalKey, logical);
  expect(event.character, character);
  expect(event.synthesized, false);
}

void _verifyRawKeyEvent<T extends RawKeyEvent>(
  RawKeyEvent event,
  PhysicalKeyboardKey physical,
  LogicalKeyboardKey logical,
  String? character,
) {
  expect(event, isA<T>());
  expect(event.physicalKey, physical);
  expect(event.logicalKey, logical);
  expect(event.character, character);
}

Future<void> _shouldThrow<T extends Error>(AsyncValueGetter<void> func) async {
  bool hasError = false;
  try {
    await func();
  } catch (e) {
    expect(e, isA<T>());
    hasError = true;
  } finally {
    expect(hasError, true);
  }
}

void main() {
  testWidgets('default transit mode is keyDataThenRawKeyData', (WidgetTester tester) async {
    expect(KeyEventSimulator.transitMode, KeyDataTransitMode.keyDataThenRawKeyData);
  });

  testWidgets('debugKeyEventSimulatorTransitModeOverride overrides default transit mode', (
    WidgetTester tester,
  ) async {
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.rawKeyData;
    expect(KeyEventSimulator.transitMode, KeyDataTransitMode.rawKeyData);
    // Unsetting debugKeyEventSimulatorTransitModeOverride can't be called in a
    // tear down callback because TestWidgetsFlutterBinding._verifyInvariants
    // is called before tear down callbacks.
    debugKeyEventSimulatorTransitModeOverride = null;
  });

  testWidgets('simulates keyboard events (RawEvent)', (WidgetTester tester) async {
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.rawKeyData;

    final List<RawKeyEvent> events = <RawKeyEvent>[];
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      RawKeyboardListener(focusNode: focusNode, onKey: events.add, child: Container()),
    );

    focusNode.requestFocus();
    await tester.idle();

    for (final String platform in platforms) {
      await tester.sendKeyEvent(LogicalKeyboardKey.shiftLeft, platform: platform);
      await tester.sendKeyEvent(LogicalKeyboardKey.shift, platform: platform);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA, platform: platform);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA, platform: platform);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad1, platform: platform);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad1, platform: platform);
      await tester.idle();

      expect(events.length, 8);
      for (int i = 0; i < events.length; ++i) {
        final bool isEven = i.isEven;
        if (isEven) {
          expect(events[i].runtimeType, equals(RawKeyDownEvent));
        } else {
          expect(events[i].runtimeType, equals(RawKeyUpEvent));
        }
        if (i < 4) {
          expect(
            events[i].data.isModifierPressed(ModifierKey.shiftModifier, side: KeyboardSide.left),
            equals(isEven),
          );
        }
      }
      events.clear();
    }

    await tester.pumpWidget(Container());

    debugKeyEventSimulatorTransitModeOverride = null;
  });

  testWidgets('simulates keyboard events (KeyData then RawKeyEvent)', (WidgetTester tester) async {
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.keyDataThenRawKeyData;

    final List<KeyEvent> events = <KeyEvent>[];
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      KeyboardListener(focusNode: focusNode, onKeyEvent: events.add, child: Container()),
    );

    focusNode.requestFocus();
    await tester.idle();

    // Key press shiftLeft
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
    expect(events.length, 1);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.shiftLeft,
      LogicalKeyboardKey.shiftLeft,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.shiftLeft}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.shiftLeft}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.shiftLeft);
    expect(events.length, 1);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.shiftLeft,
      LogicalKeyboardKey.shiftLeft,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.shiftLeft}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.shiftLeft}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
    expect(events.length, 1);
    _verifyKeyEvent<KeyUpEvent>(
      events[0],
      PhysicalKeyboardKey.shiftLeft,
      LogicalKeyboardKey.shiftLeft,
      null,
    );
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    // Key press keyA
    await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
    expect(events.length, 1);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.keyA}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.keyA}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
    _verifyKeyEvent<KeyUpEvent>(events[0], PhysicalKeyboardKey.keyA, LogicalKeyboardKey.keyA, null);
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    // Key press keyA with physical keyQ
    await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA, physicalKey: PhysicalKeyboardKey.keyQ);
    expect(events.length, 1);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.keyQ,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.keyQ}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA, physicalKey: PhysicalKeyboardKey.keyQ);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.keyQ,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.keyQ}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA, physicalKey: PhysicalKeyboardKey.keyQ);
    _verifyKeyEvent<KeyUpEvent>(events[0], PhysicalKeyboardKey.keyQ, LogicalKeyboardKey.keyA, null);
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    // Key press numpad1
    await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad1);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.numpad1,
      LogicalKeyboardKey.numpad1,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numpad1}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numpad1}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.numpad1);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.numpad1,
      LogicalKeyboardKey.numpad1,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numpad1}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numpad1}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad1);
    _verifyKeyEvent<KeyUpEvent>(
      events[0],
      PhysicalKeyboardKey.numpad1,
      LogicalKeyboardKey.numpad1,
      null,
    );
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    // Key press numLock (1st time)
    await tester.sendKeyDownEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
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
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
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
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyUpEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(
      HardwareKeyboard.instance.lockModesEnabled,
      equals(<KeyboardLockMode>{KeyboardLockMode.numLock}),
    );
    events.clear();

    // Key press numLock (2nd time)
    await tester.sendKeyDownEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyDownEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyRepeatEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyRepeatEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
    expect(
      HardwareKeyboard.instance.physicalKeysPressed,
      equals(<PhysicalKeyboardKey>{PhysicalKeyboardKey.numLock}),
    );
    expect(
      HardwareKeyboard.instance.logicalKeysPressed,
      equals(<LogicalKeyboardKey>{LogicalKeyboardKey.numLock}),
    );
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.sendKeyUpEvent(LogicalKeyboardKey.numLock);
    _verifyKeyEvent<KeyUpEvent>(
      events[0],
      PhysicalKeyboardKey.numLock,
      LogicalKeyboardKey.numLock,
      null,
    );
    expect(HardwareKeyboard.instance.physicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    expect(HardwareKeyboard.instance.lockModesEnabled, isEmpty);
    events.clear();

    await tester.idle();

    await tester.pumpWidget(Container());

    debugKeyEventSimulatorTransitModeOverride = null;
  });

  testWidgets('simulates using the correct transit mode: rawKeyData', (WidgetTester tester) async {
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.rawKeyData;

    final List<Object> events = <Object>[];
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      Focus(
        focusNode: focusNode,
        onKey: (FocusNode node, RawKeyEvent event) {
          events.add(event);
          return KeyEventResult.ignored;
        },
        onKeyEvent: (FocusNode node, KeyEvent event) {
          events.add(event);
          return KeyEventResult.ignored;
        },
        child: Container(),
      ),
    );

    focusNode.requestFocus();
    await tester.idle();

    // A (physical keyA, logical keyA) is pressed.
    await simulateKeyDownEvent(LogicalKeyboardKey.keyA);
    expect(events.length, 2);
    expect(events[0], isA<KeyEvent>());
    _verifyKeyEvent<KeyDownEvent>(
      events[0] as KeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(events[1], isA<RawKeyEvent>());
    _verifyRawKeyEvent<RawKeyDownEvent>(
      events[1] as RawKeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    events.clear();

    // A (physical keyA, logical keyB) is released.
    //
    // Since this event was synthesized and regularized before being sent to
    // HardwareKeyboard, this event will be accepted.
    await simulateKeyUpEvent(LogicalKeyboardKey.keyB, physicalKey: PhysicalKeyboardKey.keyA);
    expect(events.length, 2);
    expect(events[0], isA<KeyEvent>());
    _verifyKeyEvent<KeyUpEvent>(
      events[0] as KeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      null,
    );
    expect(events[1], isA<RawKeyEvent>());
    _verifyRawKeyEvent<RawKeyUpEvent>(
      events[1] as RawKeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyB,
      null,
    );
    events.clear();

    // Manually switch the transit mode to `keyDataThenRawKeyData`. This will
    // never happen in real applications so the assertion error can verify that
    // the transit mode is correctly applied.
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.keyDataThenRawKeyData;

    await _shouldThrow<AssertionError>(
      () => simulateKeyUpEvent(LogicalKeyboardKey.keyB, physicalKey: PhysicalKeyboardKey.keyA),
    );

    debugKeyEventSimulatorTransitModeOverride = null;
  });

  testWidgets('simulates using the correct transit mode: keyDataThenRawKeyData', (
    WidgetTester tester,
  ) async {
    debugKeyEventSimulatorTransitModeOverride = KeyDataTransitMode.keyDataThenRawKeyData;

    final List<Object> events = <Object>[];
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      Focus(
        focusNode: focusNode,
        onKey: (FocusNode node, RawKeyEvent event) {
          events.add(event);
          return KeyEventResult.ignored;
        },
        onKeyEvent: (FocusNode node, KeyEvent event) {
          events.add(event);
          return KeyEventResult.ignored;
        },
        child: Container(),
      ),
    );

    focusNode.requestFocus();
    await tester.idle();

    // A (physical keyA, logical keyA) is pressed.
    await simulateKeyDownEvent(LogicalKeyboardKey.keyA);
    expect(events.length, 2);
    expect(events[0], isA<KeyEvent>());
    _verifyKeyEvent<KeyDownEvent>(
      events[0] as KeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    expect(events[1], isA<RawKeyEvent>());
    _verifyRawKeyEvent<RawKeyDownEvent>(
      events[1] as RawKeyEvent,
      PhysicalKeyboardKey.keyA,
      LogicalKeyboardKey.keyA,
      'a',
    );
    events.clear();

    // A (physical keyA, logical keyB) is released.
    //
    // Since this event is transmitted to HardwareKeyboard as-is, it will be rejected due to
    // inconsistent logical key. This does not indicate behavioral difference,
    // since KeyData is will never send malformed data sequence in real applications.
    await _shouldThrow<AssertionError>(
      () => simulateKeyUpEvent(LogicalKeyboardKey.keyB, physicalKey: PhysicalKeyboardKey.keyA),
    );

    debugKeyEventSimulatorTransitModeOverride = null;
  });

  testWidgets('Key events are simulated using the default target platform', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/133955.
    final List<RawKeyEvent> events = <RawKeyEvent>[];
    final FocusNode focusNode = FocusNode();

    await tester.pumpWidget(
      RawKeyboardListener(focusNode: focusNode, onKey: events.add, child: Container()),
    );

    focusNode.requestFocus();
    await tester.idle();

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    expect(events.length, 1);
    final Type expectedType =
        isBrowser
            ? RawKeyEventDataWeb
            : switch (defaultTargetPlatform) {
              TargetPlatform.android => RawKeyEventDataAndroid,
              TargetPlatform.fuchsia => RawKeyEventDataFuchsia,
              TargetPlatform.iOS => RawKeyEventDataIos,
              TargetPlatform.linux => RawKeyEventDataLinux,
              TargetPlatform.macOS => RawKeyEventDataMacOs,
              TargetPlatform.windows => RawKeyEventDataWindows,
            };
    expect(events.first.data.runtimeType, expectedType);
  }, variant: TargetPlatformVariant.all());
}
