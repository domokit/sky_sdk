// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'semantics_tester.dart';

void main() {
  group(LogicalKeySet, () {
    test('LogicalKeySet passes parameters correctly.', () {
      final LogicalKeySet set1 = LogicalKeySet(LogicalKeyboardKey.keyA);
      final LogicalKeySet set2 = LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB);
      final LogicalKeySet set3 = LogicalKeySet(
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
      );
      final LogicalKeySet set4 = LogicalKeySet(
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyD,
      );
      final LogicalKeySet setFromSet = LogicalKeySet.fromSet(<LogicalKeyboardKey>{
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyD,
      });
      expect(set1.keys, equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA}));
      expect(
        set2.keys,
        equals(<LogicalKeyboardKey>{LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB}),
      );
      expect(
        set3.keys,
        equals(<LogicalKeyboardKey>{
          LogicalKeyboardKey.keyA,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyC,
        }),
      );
      expect(
        set4.keys,
        equals(<LogicalKeyboardKey>{
          LogicalKeyboardKey.keyA,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyC,
          LogicalKeyboardKey.keyD,
        }),
      );
      expect(
        setFromSet.keys,
        equals(<LogicalKeyboardKey>{
          LogicalKeyboardKey.keyA,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyC,
          LogicalKeyboardKey.keyD,
        }),
      );
    });

    test('LogicalKeySet works as a map key.', () {
      final LogicalKeySet set1 = LogicalKeySet(LogicalKeyboardKey.keyA);
      final LogicalKeySet set2 = LogicalKeySet(
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyD,
      );
      final LogicalKeySet set3 = LogicalKeySet(
        LogicalKeyboardKey.keyD,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyA,
      );
      final LogicalKeySet set4 = LogicalKeySet.fromSet(<LogicalKeyboardKey>{
        LogicalKeyboardKey.keyD,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyA,
      });
      final Map<LogicalKeySet, String> map = <LogicalKeySet, String>{set1: 'one'};
      expect(set2 == set3, isTrue);
      expect(set2 == set4, isTrue);
      expect(set2.hashCode, set3.hashCode);
      expect(set2.hashCode, set4.hashCode);
      expect(map.containsKey(set1), isTrue);
      expect(map.containsKey(LogicalKeySet(LogicalKeyboardKey.keyA)), isTrue);
      expect(
        set2,
        equals(
          LogicalKeySet.fromSet(<LogicalKeyboardKey>{
            LogicalKeyboardKey.keyA,
            LogicalKeyboardKey.keyB,
            LogicalKeyboardKey.keyC,
            LogicalKeyboardKey.keyD,
          }),
        ),
      );
    });

    testWidgets('handles two keys', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(LogicalKeySet(LogicalKeyboardKey.keyC, LogicalKeyboardKey.control), (
          Intent intent,
        ) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // LCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      // KeyC -> LCtrl: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      // RCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 1);
      invoked = 0;

      // LCtrl -> LShift -> KeyC: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 0);
      invoked = 0;

      // LCtrl -> KeyA -> KeyC: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invoked, 0);
      invoked = 0;

      expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    });

    test('LogicalKeySet.hashCode is stable', () {
      final LogicalKeySet set1 = LogicalKeySet(LogicalKeyboardKey.keyA);
      expect(set1.hashCode, set1.hashCode);

      final LogicalKeySet set2 = LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB);
      expect(set2.hashCode, set2.hashCode);

      final LogicalKeySet set3 = LogicalKeySet(
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
      );
      expect(set3.hashCode, set3.hashCode);

      final LogicalKeySet set4 = LogicalKeySet(
        LogicalKeyboardKey.keyA,
        LogicalKeyboardKey.keyB,
        LogicalKeyboardKey.keyC,
        LogicalKeyboardKey.keyD,
      );
      expect(set4.hashCode, set4.hashCode);
    });

    test('LogicalKeySet.hashCode is order-independent', () {
      expect(
        LogicalKeySet(LogicalKeyboardKey.keyA).hashCode,
        LogicalKeySet(LogicalKeyboardKey.keyA).hashCode,
      );
      expect(
        LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB).hashCode,
        LogicalKeySet(LogicalKeyboardKey.keyB, LogicalKeyboardKey.keyA).hashCode,
      );
      expect(
        LogicalKeySet(
          LogicalKeyboardKey.keyA,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyC,
        ).hashCode,
        LogicalKeySet(
          LogicalKeyboardKey.keyC,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyA,
        ).hashCode,
      );
      expect(
        LogicalKeySet(
          LogicalKeyboardKey.keyA,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyC,
          LogicalKeyboardKey.keyD,
        ).hashCode,
        LogicalKeySet(
          LogicalKeyboardKey.keyD,
          LogicalKeyboardKey.keyC,
          LogicalKeyboardKey.keyB,
          LogicalKeyboardKey.keyA,
        ).hashCode,
      );
    });

    testWidgets('isActivatedBy works as expected', (WidgetTester tester) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      final LogicalKeySet set = LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.control);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(set, events.last), isTrue);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(set, events.last), isTrue);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(set, events.last), isFalse);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(ShortcutActivator.isActivatedBy(set, events.last), isFalse);
    });

    test('LogicalKeySet diagnostics work.', () {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

      LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB).debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) {
                return !node.isFiltered(DiagnosticLevel.info);
              })
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description.length, equals(1));
      expect(description[0], equals('keys: Key A + Key B'));
    });
  });

  group(SingleActivator, () {
    testWidgets('handles Ctrl-C', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const SingleActivator(LogicalKeyboardKey.keyC, control: true), (
          Intent intent,
        ) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // LCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      // KeyC -> LCtrl: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      invoked = 0;

      // LShift -> LCtrl -> KeyC: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      invoked = 0;

      // With Ctrl-C pressed, KeyA -> Release KeyA: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      invoked = 0;
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      invoked = 0;

      // LCtrl -> KeyA -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      invoked = 0;

      // RCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 1);
      invoked = 0;

      // LCtrl -> RCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 1);
      invoked = 0;

      // While holding Ctrl-C, press KeyA: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('handles repeated events', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const SingleActivator(LogicalKeyboardKey.keyC, control: true), (
          Intent intent,
        ) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // LCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 2);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 2);
      invoked = 0;

      expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('rejects repeated events if requested', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(
          const SingleActivator(LogicalKeyboardKey.keyC, control: true, includeRepeats: false),
          (Intent intent) {
            invoked += 1;
          },
        ),
      );
      await tester.pump();

      // LCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('handles Shift-Ctrl-C', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(
          const SingleActivator(LogicalKeyboardKey.keyC, shift: true, control: true),
          (Intent intent) {
            invoked += 1;
          },
        ),
      );
      await tester.pump();

      // LShift -> LCtrl -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 1);
      invoked = 0;

      // LCtrl -> LShift -> KeyC: Accept
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 1);
      invoked = 0;

      // LCtrl -> KeyC -> LShift: Reject
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 0);
      invoked = 0;

      expect(HardwareKeyboard.instance.logicalKeysPressed, isEmpty);
    });

    testWidgets('isActivatedBy works as expected', (WidgetTester tester) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      const SingleActivator singleActivator = SingleActivator(
        LogicalKeyboardKey.keyA,
        control: true,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isFalse);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isFalse);

      const SingleActivator noRepeatSingleActivator = SingleActivator(
        LogicalKeyboardKey.keyA,
        control: true,
        includeRepeats: false,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatSingleActivator, events.last), isTrue);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatSingleActivator, events.last), isFalse);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatSingleActivator, events.last), isFalse);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(ShortcutActivator.isActivatedBy(noRepeatSingleActivator, events.last), isFalse);
    });

    testWidgets('numLock works as expected when set to LockState.locked', (
      WidgetTester tester,
    ) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      const SingleActivator singleActivator = SingleActivator(
        LogicalKeyboardKey.numpad4,
        numLock: LockState.locked,
      );

      // Lock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock), isTrue);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);

      // Unlock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(
        HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock),
        isFalse,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isFalse);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);
    });

    testWidgets('numLock works as expected when set to LockState.unlocked', (
      WidgetTester tester,
    ) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      const SingleActivator singleActivator = SingleActivator(
        LogicalKeyboardKey.numpad4,
        numLock: LockState.unlocked,
      );

      // Lock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock), isTrue);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isFalse);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);

      // Unlock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(
        HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock),
        isFalse,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);
    });

    testWidgets('numLock works as expected when set to LockState.ignored', (
      WidgetTester tester,
    ) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      const SingleActivator singleActivator = SingleActivator(LogicalKeyboardKey.numpad4);

      // Lock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock), isTrue);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);

      // Unlock NumLock.
      await tester.sendKeyEvent(LogicalKeyboardKey.numLock);
      expect(
        HardwareKeyboard.instance.lockModesEnabled.contains(KeyboardLockMode.numLock),
        isFalse,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.numpad4);
      expect(ShortcutActivator.isActivatedBy(singleActivator, events.last), isTrue);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.numpad4);
    });

    group('diagnostics.', () {
      test('single key', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const SingleActivator(LogicalKeyboardKey.keyA).debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(1));
        expect(description[0], equals('keys: Key A'));
      });

      test('no repeats', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const SingleActivator(
          LogicalKeyboardKey.keyA,
          includeRepeats: false,
        ).debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(2));
        expect(description[0], equals('keys: Key A'));
        expect(description[1], equals('excluding repeats'));
      });

      test('combination', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const SingleActivator(
          LogicalKeyboardKey.keyA,
          control: true,
          shift: true,
          alt: true,
          meta: true,
        ).debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(1));
        expect(description[0], equals('keys: Control + Alt + Meta + Shift + Key A'));
      });
    });
  });

  group(Shortcuts, () {
    testWidgets('Default constructed Shortcuts has empty shortcuts', (WidgetTester tester) async {
      const Shortcuts shortcuts = Shortcuts(
        shortcuts: <LogicalKeySet, Intent>{},
        child: SizedBox(),
      );
      await tester.pumpWidget(shortcuts);
      expect(shortcuts.shortcuts, isNotNull);
      expect(shortcuts.shortcuts, isEmpty);
    });

    testWidgets('Default constructed Shortcuts.manager has empty shortcuts', (
      WidgetTester tester,
    ) async {
      final ShortcutManager manager = ShortcutManager();
      addTearDown(manager.dispose);
      expect(manager.shortcuts, isNotNull);
      expect(manager.shortcuts, isEmpty);
      final Shortcuts shortcuts = Shortcuts.manager(manager: manager, child: const SizedBox());
      await tester.pumpWidget(shortcuts);
      expect(shortcuts.shortcuts, isNotNull);
      expect(shortcuts.shortcuts, isEmpty);
    });

    testWidgets('Shortcuts.manager passes on shortcuts', (WidgetTester tester) async {
      final Map<LogicalKeySet, Intent> testShortcuts = <LogicalKeySet, Intent>{
        LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
      };
      final ShortcutManager manager = ShortcutManager(shortcuts: testShortcuts);
      addTearDown(manager.dispose);
      expect(manager.shortcuts, isNotNull);
      expect(manager.shortcuts, equals(testShortcuts));
      final Shortcuts shortcuts = Shortcuts.manager(manager: manager, child: const SizedBox());
      await tester.pumpWidget(shortcuts);
      expect(shortcuts.shortcuts, isNotNull);
      expect(shortcuts.shortcuts, equals(testShortcuts));
    });

    testWidgets('ShortcutManager handles shortcuts', (WidgetTester tester) async {
      final GlobalKey containerKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        Actions(
          actions: <Type, Action<Intent>>{
            TestIntent: TestAction(
              onInvoke: (Intent intent) {
                invoked = true;
                return true;
              },
            ),
          },
          child: Shortcuts.manager(
            manager: testManager,
            child: Focus(
              autofocus: true,
              child: SizedBox(key: containerKey, width: 100, height: 100),
            ),
          ),
        ),
      );
      await tester.pump();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, isTrue);
      expect(pressedKeys, equals(<LogicalKeyboardKey>[LogicalKeyboardKey.shiftLeft]));
    });

    testWidgets('Shortcuts.manager lets manager handle shortcuts', (WidgetTester tester) async {
      final GlobalKey containerKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        Actions(
          actions: <Type, Action<Intent>>{
            TestIntent: TestAction(
              onInvoke: (Intent intent) {
                invoked = true;
                return true;
              },
            ),
          },
          child: Shortcuts.manager(
            manager: testManager,
            child: Focus(
              autofocus: true,
              child: SizedBox(key: containerKey, width: 100, height: 100),
            ),
          ),
        ),
      );
      await tester.pump();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, isTrue);
      expect(pressedKeys, equals(<LogicalKeyboardKey>[LogicalKeyboardKey.shiftLeft]));
    });

    testWidgets('ShortcutManager ignores key presses with no primary focus', (
      WidgetTester tester,
    ) async {
      final GlobalKey containerKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        Actions(
          actions: <Type, Action<Intent>>{
            TestIntent: TestAction(
              onInvoke: (Intent intent) {
                invoked = true;
                return true;
              },
            ),
          },
          child: Shortcuts.manager(
            manager: testManager,
            child: SizedBox(key: containerKey, width: 100, height: 100),
          ),
        ),
      );
      await tester.pump();
      expect(primaryFocus, isNull);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, isFalse);
      expect(pressedKeys, isEmpty);
    });

    test('$ShortcutManager dispatches object creation in constructor', () async {
      await expectLater(
        await memoryEvents(() => ShortcutManager().dispose(), ShortcutManager),
        areCreateAndDispose,
      );
    });

    testWidgets("Shortcuts passes to the next Shortcuts widget if it doesn't map the key", (
      WidgetTester tester,
    ) async {
      final GlobalKey containerKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        Shortcuts.manager(
          manager: testManager,
          child: Actions(
            actions: <Type, Action<Intent>>{
              TestIntent: TestAction(
                onInvoke: (Intent intent) {
                  invoked = true;
                  return invoked;
                },
              ),
            },
            child: Shortcuts(
              shortcuts: <LogicalKeySet, Intent>{
                LogicalKeySet(LogicalKeyboardKey.keyA): Intent.doNothing,
              },
              child: Focus(
                autofocus: true,
                child: SizedBox(key: containerKey, width: 100, height: 100),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, isTrue);
      expect(pressedKeys, equals(<LogicalKeyboardKey>[LogicalKeyboardKey.shiftLeft]));
    });

    testWidgets('Shortcuts can disable a shortcut with Intent.doNothing', (
      WidgetTester tester,
    ) async {
      final GlobalKey containerKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Shortcuts.manager(
            manager: testManager,
            child: Actions(
              actions: <Type, Action<Intent>>{
                TestIntent: TestAction(
                  onInvoke: (Intent intent) {
                    invoked = true;
                    return invoked;
                  },
                ),
              },
              child: Shortcuts(
                shortcuts: <LogicalKeySet, Intent>{
                  LogicalKeySet(LogicalKeyboardKey.shift): Intent.doNothing,
                },
                child: Focus(
                  autofocus: true,
                  child: SizedBox(key: containerKey, width: 100, height: 100),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, isFalse);
      expect(pressedKeys, isEmpty);
    });

    testWidgets(
      "Shortcuts that aren't bound to an action don't absorb keys meant for text fields",
      (WidgetTester tester) async {
        final GlobalKey textFieldKey = GlobalKey();
        final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
        final TestShortcutManager testManager = TestShortcutManager(
          pressedKeys,
          shortcuts: <LogicalKeySet, Intent>{
            LogicalKeySet(LogicalKeyboardKey.keyA): const TestIntent(),
          },
        );
        addTearDown(testManager.dispose);
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Shortcuts.manager(
                manager: testManager,
                child: TextField(key: textFieldKey, autofocus: true),
              ),
            ),
          ),
        );
        await tester.pump();
        final bool handled = await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
        expect(handled, isFalse);
        expect(pressedKeys, equals(<LogicalKeyboardKey>[LogicalKeyboardKey.keyA]));
      },
    );

    testWidgets('Shortcuts that are bound to an action do override text fields', (
      WidgetTester tester,
    ) async {
      final GlobalKey textFieldKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.keyA): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Shortcuts.manager(
              manager: testManager,
              child: Actions(
                actions: <Type, Action<Intent>>{
                  TestIntent: TestAction(
                    onInvoke: (Intent intent) {
                      invoked = true;
                      return invoked;
                    },
                  ),
                },
                child: TextField(key: textFieldKey, autofocus: true),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      final bool result = await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
      expect(result, isTrue);
      expect(pressedKeys, equals(<LogicalKeyboardKey>[LogicalKeyboardKey.keyA]));
      expect(invoked, isTrue);
    });

    testWidgets('Shortcuts can override intents that apply to text fields', (
      WidgetTester tester,
    ) async {
      final GlobalKey textFieldKey = GlobalKey();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.keyA): const TestIntent(),
        },
      );
      addTearDown(testManager.dispose);
      bool invoked = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Shortcuts.manager(
              manager: testManager,
              child: Actions(
                actions: <Type, Action<Intent>>{
                  TestIntent: TestAction(
                    onInvoke: (Intent intent) {
                      invoked = true;
                      return invoked;
                    },
                  ),
                },
                child: Actions(
                  actions: <Type, Action<Intent>>{TestIntent: DoNothingAction(consumesKey: false)},
                  child: TextField(key: textFieldKey, autofocus: true),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      final bool result = await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
      expect(result, isFalse);
      expect(invoked, isFalse);
    });

    testWidgets(
      'Shortcuts can override intents that apply to text fields with DoNothingAndStopPropagationIntent',
      (WidgetTester tester) async {
        final GlobalKey textFieldKey = GlobalKey();
        final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
        final TestShortcutManager testManager = TestShortcutManager(
          pressedKeys,
          shortcuts: <LogicalKeySet, Intent>{
            LogicalKeySet(LogicalKeyboardKey.keyA): const TestIntent(),
          },
        );
        addTearDown(testManager.dispose);
        bool invoked = false;
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: Shortcuts.manager(
                manager: testManager,
                child: Actions(
                  actions: <Type, Action<Intent>>{
                    TestIntent: TestAction(
                      onInvoke: (Intent intent) {
                        invoked = true;
                        return invoked;
                      },
                    ),
                  },
                  child: Shortcuts(
                    shortcuts: <LogicalKeySet, Intent>{
                      LogicalKeySet(LogicalKeyboardKey.keyA):
                          const DoNothingAndStopPropagationIntent(),
                    },
                    child: TextField(key: textFieldKey, autofocus: true),
                  ),
                ),
              ),
            ),
          ),
        );
        await tester.pump();
        final bool result = await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
        expect(result, isFalse);
        expect(invoked, isFalse);
      },
    );

    test('Shortcuts diagnostics work.', () {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

      Shortcuts(
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.shift, LogicalKeyboardKey.keyA): const ActivateIntent(),
          LogicalKeySet(
            LogicalKeyboardKey.shift,
            LogicalKeyboardKey.arrowRight,
          ): const DirectionalFocusIntent(TraversalDirection.right),
        },
        child: const SizedBox(),
      ).debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) {
                return !node.isFiltered(DiagnosticLevel.info);
              })
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description.length, equals(1));
      expect(
        description[0],
        equalsIgnoringHashCodes(
          'shortcuts: {{Shift + Key A}: ActivateIntent#00000, {Shift + Arrow Right}: DirectionalFocusIntent#00000(direction: right)}',
        ),
      );
    });

    test('Shortcuts diagnostics work when debugLabel specified.', () {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

      Shortcuts(
        debugLabel: '<Debug Label>',
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB): const ActivateIntent(),
        },
        child: const SizedBox(),
      ).debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) {
                return !node.isFiltered(DiagnosticLevel.info);
              })
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description.length, equals(1));
      expect(description[0], equals('shortcuts: <Debug Label>'));
    });

    test('Shortcuts diagnostics work when manager not specified.', () {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

      Shortcuts(
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB): const ActivateIntent(),
        },
        child: const SizedBox(),
      ).debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) {
                return !node.isFiltered(DiagnosticLevel.info);
              })
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description.length, equals(1));
      expect(
        description[0],
        equalsIgnoringHashCodes('shortcuts: {{Key A + Key B}: ActivateIntent#00000}'),
      );
    });

    test('Shortcuts diagnostics work when manager specified.', () {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      final List<LogicalKeyboardKey> pressedKeys = <LogicalKeyboardKey>[];
      final TestShortcutManager testManager = TestShortcutManager(
        pressedKeys,
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.keyA, LogicalKeyboardKey.keyB): const ActivateIntent(),
        },
      );
      addTearDown(testManager.dispose);

      Shortcuts.manager(manager: testManager, child: const SizedBox()).debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) {
                return !node.isFiltered(DiagnosticLevel.info);
              })
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description.length, equals(2));
      expect(
        description[0],
        equalsIgnoringHashCodes(
          'manager: TestShortcutManager#00000(shortcuts: {LogicalKeySet#00000(keys: Key A + Key B): ActivateIntent#00000})',
        ),
      );
      expect(
        description[1],
        equalsIgnoringHashCodes('shortcuts: {{Key A + Key B}: ActivateIntent#00000}'),
      );
    });

    testWidgets('Shortcuts support multiple intents', (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      bool? value = true;
      Widget buildApp() {
        return MaterialApp(
          shortcuts: <LogicalKeySet, Intent>{
            LogicalKeySet(LogicalKeyboardKey.space): const PrioritizedIntents(
              orderedIntents: <Intent>[
                ActivateIntent(),
                ScrollIntent(direction: AxisDirection.down, type: ScrollIncrementType.page),
              ],
            ),
            LogicalKeySet(LogicalKeyboardKey.tab): const NextFocusIntent(),
            LogicalKeySet(LogicalKeyboardKey.pageUp): const ScrollIntent(
              direction: AxisDirection.up,
              type: ScrollIncrementType.page,
            ),
          },
          home: Material(
            child: Center(
              child: ListView(
                primary: true,
                children: <Widget>[
                  StatefulBuilder(
                    builder: (BuildContext context, StateSetter setState) {
                      return Checkbox(
                        value: value,
                        onChanged:
                            (bool? newValue) => setState(() {
                              value = newValue;
                            }),
                        focusColor: Colors.orange[500],
                      );
                    },
                  ),
                  Container(color: Colors.blue, height: 1000),
                ],
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      expect(
        tester.binding.focusManager.primaryFocus!.toStringShort(),
        equalsIgnoringHashCodes(
          'FocusScopeNode#00000(_ModalScopeState<dynamic> Focus Scope [PRIMARY FOCUS])',
        ),
      );
      final ScrollController controller = PrimaryScrollController.of(
        tester.element(find.byType(ListView)),
      );
      expect(controller.position.pixels, 0.0);
      expect(value, isTrue);

      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pumpAndSettle();
      // ScrollView scrolls
      expect(controller.position.pixels, 448.0);
      expect(value, isTrue);

      await tester.sendKeyEvent(LogicalKeyboardKey.pageUp);
      await tester.pumpAndSettle();
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      // Focus is now on the checkbox.
      expect(
        tester.binding.focusManager.primaryFocus!.toStringShort(),
        equalsIgnoringHashCodes('FocusNode#00000([PRIMARY FOCUS])'),
      );
      expect(value, isTrue);
      expect(controller.position.pixels, 0.0);

      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pumpAndSettle();
      // Checkbox is toggled, scroll view does not scroll.
      expect(value, isFalse);
      expect(controller.position.pixels, 0.0);

      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pumpAndSettle();
      expect(value, isTrue);
      expect(controller.position.pixels, 0.0);
    });

    testWidgets('Shortcuts support activators that returns null in triggers', (
      WidgetTester tester,
    ) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(
          const DumbLogicalActivator(LogicalKeyboardKey.keyC),
          (Intent intent) {
            invoked += 1;
          },
          const SingleActivator(LogicalKeyboardKey.keyC, control: true),
          (Intent intent) {
            invoked += 10;
          },
        ),
      );
      await tester.pump();

      // Press KeyC: Accepted by DumbLogicalActivator
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      invoked = 0;

      // Press ControlLeft + KeyC: Accepted by SingleActivator
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 10);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 10);
      invoked = 0;

      // Press ControlLeft + ShiftLeft + KeyC: Accepted by DumbLogicalActivator
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 1);
      invoked = 0;
    });

    testWidgets('Shortcuts does not insert a semantics node when includeSemantics is false', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);
      addTearDown(semanticsTester.dispose);

      // By default, includeSemantics is true.
      await tester.pumpWidget(
        const Shortcuts(shortcuts: <LogicalKeySet, Intent>{}, child: SizedBox()),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(children: <TestSemantics>[TestSemantics(id: 1)]),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      await tester.pumpWidget(
        const Shortcuts(
          includeSemantics: false,
          shortcuts: <LogicalKeySet, Intent>{},
          child: SizedBox(),
        ),
      );

      expect(
        semanticsTester,
        hasSemantics(TestSemantics.root(), ignoreRect: true, ignoreTransform: true),
      );

      semanticsTester.dispose();
    });
  });

  group('CharacterActivator', () {
    testWidgets('is triggered on events with correct character', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const CharacterActivator('?'), (Intent intent) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // Press Shift + /
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 1);
      invoked = 0;
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('handles repeated events', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const CharacterActivator('?'), (Intent intent) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // Press KeyC: Accepted by DumbLogicalActivator
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 2);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 2);
      invoked = 0;
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('rejects repeated events if requested', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const CharacterActivator('?', includeRepeats: false), (Intent intent) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // Press Shift + /
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      expect(invoked, 1);
      invoked = 0;
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('handles Alt, Ctrl and Meta', (WidgetTester tester) async {
      int invoked = 0;
      await tester.pumpWidget(
        activatorTester(const CharacterActivator('?', alt: true, meta: true, control: true), (
          Intent intent,
        ) {
          invoked += 1;
        }),
      );
      await tester.pump();

      // Press Shift + /
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      expect(invoked, 0);

      // Press Left Alt + Ctrl + Meta + Shift + /
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.metaLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.metaLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      expect(invoked, 1);
      invoked = 0;

      // Press Right Alt + Ctrl + Meta + Shift + /
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.metaRight);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 0);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.slash, character: '?');
      expect(invoked, 1);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.slash);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftRight);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.metaRight);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altRight);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
      expect(invoked, 1);
      invoked = 0;
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('isActivatedBy works as expected', (WidgetTester tester) async {
      // Collect some key events to use for testing.
      final List<KeyEvent> events = <KeyEvent>[];
      await tester.pumpWidget(
        Focus(
          autofocus: true,
          onKeyEvent: (FocusNode node, KeyEvent event) {
            events.add(event);
            return KeyEventResult.ignored;
          },
          child: const SizedBox(),
        ),
      );

      const CharacterActivator characterActivator = CharacterActivator('a');

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(characterActivator, events.last), isTrue);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(characterActivator, events.last), isTrue);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(characterActivator, events.last), isFalse);

      const CharacterActivator noRepeatCharacterActivator = CharacterActivator(
        'a',
        includeRepeats: false,
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatCharacterActivator, events.last), isTrue);
      await tester.sendKeyRepeatEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatCharacterActivator, events.last), isFalse);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(ShortcutActivator.isActivatedBy(noRepeatCharacterActivator, events.last), isFalse);
    });

    group('diagnostics.', () {
      test('single key', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const CharacterActivator('A').debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(1));
        expect(description[0], equals("character: 'A'"));
      });

      test('no repeats', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const CharacterActivator('A', includeRepeats: false).debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(2));
        expect(description[0], equals("character: 'A'"));
        expect(description[1], equals('excluding repeats'));
      });

      test('combination', () {
        final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

        const CharacterActivator('A', control: true, meta: true).debugFillProperties(builder);

        final List<String> description =
            builder.properties
                .where((DiagnosticsNode node) {
                  return !node.isFiltered(DiagnosticLevel.info);
                })
                .map((DiagnosticsNode node) => node.toString())
                .toList();

        expect(description.length, equals(1));
        expect(description[0], equals("character: Control + Meta + 'A'"));
      });
    });
  });

  group('CallbackShortcuts', () {
    testWidgets('trigger on key events', (WidgetTester tester) async {
      int invokedA = 0;
      int invokedB = 0;
      await tester.pumpWidget(
        CallbackShortcuts(
          bindings: <ShortcutActivator, VoidCallback>{
            const SingleActivator(LogicalKeyboardKey.keyA): () {
              invokedA += 1;
            },
            const SingleActivator(LogicalKeyboardKey.keyB): () {
              invokedB += 1;
            },
          },
          child: const Focus(autofocus: true, child: Placeholder()),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.pump();
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      invokedA = 0;
      invokedB = 0;
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
    });

    testWidgets('nested CallbackShortcuts stop propagation', (WidgetTester tester) async {
      int invokedOuter = 0;
      int invokedInner = 0;
      await tester.pumpWidget(
        CallbackShortcuts(
          bindings: <ShortcutActivator, VoidCallback>{
            const SingleActivator(LogicalKeyboardKey.keyA): () {
              invokedOuter += 1;
            },
          },
          child: CallbackShortcuts(
            bindings: <ShortcutActivator, VoidCallback>{
              const SingleActivator(LogicalKeyboardKey.keyA): () {
                invokedInner += 1;
              },
            },
            child: const Focus(autofocus: true, child: Placeholder()),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
    });

    testWidgets('non-overlapping nested CallbackShortcuts fire appropriately', (
      WidgetTester tester,
    ) async {
      int invokedOuter = 0;
      int invokedInner = 0;
      await tester.pumpWidget(
        CallbackShortcuts(
          bindings: <ShortcutActivator, VoidCallback>{
            const CharacterActivator('b'): () {
              invokedOuter += 1;
            },
          },
          child: CallbackShortcuts(
            bindings: <ShortcutActivator, VoidCallback>{
              const CharacterActivator('a'): () {
                invokedInner += 1;
              },
            },
            child: const Focus(autofocus: true, child: Placeholder()),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedOuter, equals(1));
      expect(invokedInner, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
      expect(invokedOuter, equals(1));
      expect(invokedInner, equals(1));
    });

    testWidgets('Works correctly with Shortcuts too', (WidgetTester tester) async {
      int invokedCallbackA = 0;
      int invokedCallbackB = 0;
      int invokedActionA = 0;
      int invokedActionB = 0;

      void clear() {
        invokedCallbackA = 0;
        invokedCallbackB = 0;
        invokedActionA = 0;
        invokedActionB = 0;
      }

      await tester.pumpWidget(
        Actions(
          actions: <Type, Action<Intent>>{
            TestIntent: TestAction(
              onInvoke: (Intent intent) {
                invokedActionA += 1;
                return true;
              },
            ),
            TestIntent2: TestAction(
              onInvoke: (Intent intent) {
                invokedActionB += 1;
                return true;
              },
            ),
          },
          child: CallbackShortcuts(
            bindings: <ShortcutActivator, VoidCallback>{
              const CharacterActivator('b'): () {
                invokedCallbackB += 1;
              },
            },
            child: Shortcuts(
              shortcuts: <LogicalKeySet, Intent>{
                LogicalKeySet(LogicalKeyboardKey.keyA): const TestIntent(),
                LogicalKeySet(LogicalKeyboardKey.keyB): const TestIntent2(),
              },
              child: CallbackShortcuts(
                bindings: <ShortcutActivator, VoidCallback>{
                  const CharacterActivator('a'): () {
                    invokedCallbackA += 1;
                  },
                },
                child: const Focus(autofocus: true, child: Placeholder()),
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedCallbackA, equals(1));
      expect(invokedCallbackB, equals(0));
      expect(invokedActionA, equals(0));
      expect(invokedActionB, equals(0));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      clear();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedCallbackA, equals(0));
      expect(invokedCallbackB, equals(0));
      expect(invokedActionA, equals(0));
      expect(invokedActionB, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
    });
  });

  group('ShortcutRegistrar', () {
    testWidgets('trigger ShortcutRegistrar on key events', (WidgetTester tester) async {
      int invokedA = 0;
      int invokedB = 0;
      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            shortcuts: <ShortcutActivator, Intent>{
              const SingleActivator(LogicalKeyboardKey.keyA): VoidCallbackIntent(() {
                invokedA += 1;
              }),
              const SingleActivator(LogicalKeyboardKey.keyB): VoidCallbackIntent(() {
                invokedB += 1;
              }),
            },
            child: Actions(
              actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
              child: const Focus(autofocus: true, child: Placeholder()),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.pump();
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      invokedA = 0;
      invokedB = 0;
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
    });

    testWidgets('MaterialApp has a ShortcutRegistrar listening', (WidgetTester tester) async {
      int invokedA = 0;
      int invokedB = 0;
      await tester.pumpWidget(
        MaterialApp(
          home: TestCallbackRegistration(
            shortcuts: <ShortcutActivator, Intent>{
              const SingleActivator(LogicalKeyboardKey.keyA): VoidCallbackIntent(() {
                invokedA += 1;
              }),
              const SingleActivator(LogicalKeyboardKey.keyB): VoidCallbackIntent(() {
                invokedB += 1;
              }),
            },
            child: Actions(
              actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
              child: const Focus(autofocus: true, child: Placeholder()),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.pump();
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invokedA, equals(1));
      expect(invokedB, equals(0));
      invokedA = 0;
      invokedB = 0;
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
      expect(invokedA, equals(0));
      expect(invokedB, equals(1));
    });

    testWidgets("doesn't override text field shortcuts", (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ShortcutRegistrar(
              child: TestCallbackRegistration(
                shortcuts: const <ShortcutActivator, Intent>{
                  SingleActivator(LogicalKeyboardKey.keyA, control: true): SelectAllTextIntent(
                    SelectionChangedCause.keyboard,
                  ),
                },
                child: TextField(autofocus: true, controller: controller),
              ),
            ),
          ),
        ),
      );
      controller.text = 'Testing';
      await tester.pump();

      // Send a "Ctrl-A", which should be bound to select all by default.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
      await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
      await tester.pump();

      expect(controller.selection.baseOffset, equals(0));
      expect(controller.selection.extentOffset, equals(7));
    });

    testWidgets('nested ShortcutRegistrars stop propagation', (WidgetTester tester) async {
      int invokedOuter = 0;
      int invokedInner = 0;
      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            shortcuts: <ShortcutActivator, Intent>{
              const SingleActivator(LogicalKeyboardKey.keyA): VoidCallbackIntent(() {
                invokedOuter += 1;
              }),
            },
            child: ShortcutRegistrar(
              child: TestCallbackRegistration(
                shortcuts: <ShortcutActivator, Intent>{
                  const SingleActivator(LogicalKeyboardKey.keyA): VoidCallbackIntent(() {
                    invokedInner += 1;
                  }),
                },
                child: Actions(
                  actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
                  child: const Focus(autofocus: true, child: Placeholder()),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
    });

    testWidgets('non-overlapping nested ShortcutRegistrars fire appropriately', (
      WidgetTester tester,
    ) async {
      int invokedOuter = 0;
      int invokedInner = 0;
      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            shortcuts: <ShortcutActivator, Intent>{
              const CharacterActivator('b'): VoidCallbackIntent(() {
                invokedOuter += 1;
              }),
            },
            child: ShortcutRegistrar(
              child: TestCallbackRegistration(
                shortcuts: <ShortcutActivator, Intent>{
                  const CharacterActivator('a'): VoidCallbackIntent(() {
                    invokedInner += 1;
                  }),
                },
                child: Actions(
                  actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
                  child: const Focus(autofocus: true, child: Placeholder()),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedOuter, equals(0));
      expect(invokedInner, equals(1));
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedOuter, equals(1));
      expect(invokedInner, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
      expect(invokedOuter, equals(1));
      expect(invokedInner, equals(1));
    });

    testWidgets('Works correctly with Shortcuts too', (WidgetTester tester) async {
      int invokedCallbackA = 0;
      int invokedCallbackB = 0;
      int invokedActionA = 0;
      int invokedActionB = 0;

      void clear() {
        invokedCallbackA = 0;
        invokedCallbackB = 0;
        invokedActionA = 0;
        invokedActionB = 0;
      }

      await tester.pumpWidget(
        Actions(
          actions: <Type, Action<Intent>>{
            TestIntent: TestAction(
              onInvoke: (Intent intent) {
                invokedActionA += 1;
                return true;
              },
            ),
            TestIntent2: TestAction(
              onInvoke: (Intent intent) {
                invokedActionB += 1;
                return true;
              },
            ),
            VoidCallbackIntent: VoidCallbackAction(),
          },
          child: ShortcutRegistrar(
            child: TestCallbackRegistration(
              shortcuts: <ShortcutActivator, Intent>{
                const CharacterActivator('b'): VoidCallbackIntent(() {
                  invokedCallbackB += 1;
                }),
              },
              child: Shortcuts(
                shortcuts: const <ShortcutActivator, Intent>{
                  SingleActivator(LogicalKeyboardKey.keyA): TestIntent(),
                  SingleActivator(LogicalKeyboardKey.keyB): TestIntent2(),
                },
                child: ShortcutRegistrar(
                  child: TestCallbackRegistration(
                    shortcuts: <ShortcutActivator, Intent>{
                      const CharacterActivator('a'): VoidCallbackIntent(() {
                        invokedCallbackA += 1;
                      }),
                    },
                    child: const Focus(autofocus: true, child: Placeholder()),
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      expect(invokedCallbackA, equals(1));
      expect(invokedCallbackB, equals(0));
      expect(invokedActionA, equals(0));
      expect(invokedActionB, equals(0));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      clear();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyB);
      expect(invokedCallbackA, equals(0));
      expect(invokedCallbackB, equals(0));
      expect(invokedActionA, equals(0));
      expect(invokedActionB, equals(1));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyB);
    });

    testWidgets('Updating shortcuts triggers dependency rebuild', (WidgetTester tester) async {
      final List<Map<ShortcutActivator, Intent>> shortcutsChanged =
          <Map<ShortcutActivator, Intent>>[];
      void dependenciesUpdated(Map<ShortcutActivator, Intent> shortcuts) {
        shortcutsChanged.add(shortcuts);
      }

      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            onDependencyUpdate: dependenciesUpdated,
            shortcuts: const <ShortcutActivator, Intent>{
              SingleActivator(LogicalKeyboardKey.keyA): SelectAllTextIntent(
                SelectionChangedCause.keyboard,
              ),
              SingleActivator(LogicalKeyboardKey.keyB): ActivateIntent(),
            },
            child: Actions(
              actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
              child: const Focus(autofocus: true, child: Placeholder()),
            ),
          ),
        ),
      );

      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            onDependencyUpdate: dependenciesUpdated,
            shortcuts: const <ShortcutActivator, Intent>{
              SingleActivator(LogicalKeyboardKey.keyA): SelectAllTextIntent(
                SelectionChangedCause.keyboard,
              ),
            },
            child: Actions(
              actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
              child: const Focus(autofocus: true, child: Placeholder()),
            ),
          ),
        ),
      );

      await tester.pumpWidget(
        ShortcutRegistrar(
          child: TestCallbackRegistration(
            onDependencyUpdate: dependenciesUpdated,
            shortcuts: const <ShortcutActivator, Intent>{
              SingleActivator(LogicalKeyboardKey.keyA): SelectAllTextIntent(
                SelectionChangedCause.keyboard,
              ),
              SingleActivator(LogicalKeyboardKey.keyB): ActivateIntent(),
            },
            child: Actions(
              actions: <Type, Action<Intent>>{VoidCallbackIntent: VoidCallbackAction()},
              child: const Focus(autofocus: true, child: Placeholder()),
            ),
          ),
        ),
      );

      expect(shortcutsChanged.length, equals(2));
      expect(
        shortcutsChanged.last,
        equals(const <ShortcutActivator, Intent>{
          SingleActivator(LogicalKeyboardKey.keyA): SelectAllTextIntent(
            SelectionChangedCause.keyboard,
          ),
          SingleActivator(LogicalKeyboardKey.keyB): ActivateIntent(),
        }),
      );
    });

    testWidgets('using a disposed token asserts', (WidgetTester tester) async {
      final ShortcutRegistry registry = ShortcutRegistry();
      addTearDown(registry.dispose);
      final ShortcutRegistryEntry token = registry.addAll(const <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.keyA): DoNothingIntent(),
      });
      token.dispose();
      expect(() {
        token.replaceAll(<ShortcutActivator, Intent>{});
      }, throwsFlutterError);
    });

    testWidgets('setting duplicate bindings asserts', (WidgetTester tester) async {
      final ShortcutRegistry registry = ShortcutRegistry();
      addTearDown(registry.dispose);
      final ShortcutRegistryEntry token = registry.addAll(const <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.keyA): DoNothingIntent(),
      });
      expect(() {
        final ShortcutRegistryEntry token2 = registry.addAll(const <ShortcutActivator, Intent>{
          SingleActivator(LogicalKeyboardKey.keyA): ActivateIntent(),
        });
        token2.dispose();
      }, throwsAssertionError);
      token.dispose();
    });

    test('dispatches object creation in constructor', () async {
      await expectLater(
        await memoryEvents(() => ShortcutRegistry().dispose(), ShortcutRegistry),
        areCreateAndDispose,
      );
    });
  });
}

class TestCallbackRegistration extends StatefulWidget {
  const TestCallbackRegistration({
    super.key,
    required this.shortcuts,
    this.onDependencyUpdate,
    required this.child,
  });

  final Map<ShortcutActivator, Intent> shortcuts;
  final void Function(Map<ShortcutActivator, Intent> shortcuts)? onDependencyUpdate;
  final Widget child;

  @override
  State<TestCallbackRegistration> createState() => _TestCallbackRegistrationState();
}

class _TestCallbackRegistrationState extends State<TestCallbackRegistration> {
  ShortcutRegistryEntry? _registryToken;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _registryToken?.dispose();
    _registryToken = ShortcutRegistry.of(context).addAll(widget.shortcuts);
  }

  @override
  void didUpdateWidget(TestCallbackRegistration oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.shortcuts != oldWidget.shortcuts || _registryToken == null) {
      _registryToken?.dispose();
      _registryToken = ShortcutRegistry.of(context).addAll(widget.shortcuts);
    }
    widget.onDependencyUpdate?.call(ShortcutRegistry.of(context).shortcuts);
  }

  @override
  void dispose() {
    _registryToken?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}

class TestAction extends CallbackAction<Intent> {
  TestAction({required super.onInvoke});
}

/// An activator that accepts down events that has [key] as the logical key.
///
/// This class is used only to tests. It is intentionally designed poorly by
/// returning null in [triggers], and checks [key] in [accepts].
class DumbLogicalActivator extends ShortcutActivator {
  const DumbLogicalActivator(this.key);

  final LogicalKeyboardKey key;

  @override
  Iterable<LogicalKeyboardKey>? get triggers => null;

  @override
  bool accepts(KeyEvent event, HardwareKeyboard state) {
    return (event is KeyDownEvent || event is KeyRepeatEvent) && event.logicalKey == key;
  }

  /// Returns a short and readable description of the key combination.
  ///
  /// Intended to be used in debug mode for logging purposes. In release mode,
  /// [debugDescribeKeys] returns an empty string.
  @override
  String debugDescribeKeys() {
    String result = '';
    assert(() {
      result = key.keyLabel;
      return true;
    }());
    return result;
  }
}

class TestIntent extends Intent {
  const TestIntent();
}

class TestIntent2 extends Intent {
  const TestIntent2();
}

class TestShortcutManager extends ShortcutManager {
  TestShortcutManager(this.keys, {super.shortcuts});

  List<LogicalKeyboardKey> keys;

  @override
  KeyEventResult handleKeypress(BuildContext context, KeyEvent event) {
    if (event is KeyDownEvent || event is KeyRepeatEvent) {
      keys.add(event.logicalKey);
    }
    return super.handleKeypress(context, event);
  }
}

Widget activatorTester(
  ShortcutActivator activator,
  ValueSetter<Intent> onInvoke, [
  ShortcutActivator? activator2,
  ValueSetter<Intent>? onInvoke2,
]) {
  final bool hasSecond = activator2 != null && onInvoke2 != null;
  return Actions(
    key: GlobalKey(),
    actions: <Type, Action<Intent>>{
      TestIntent: TestAction(
        onInvoke: (Intent intent) {
          onInvoke(intent);
          return true;
        },
      ),
      if (hasSecond)
        TestIntent2: TestAction(
          onInvoke: (Intent intent) {
            onInvoke2(intent);
            return null;
          },
        ),
    },
    child: Shortcuts(
      shortcuts: <ShortcutActivator, Intent>{
        activator: const TestIntent(),
        if (hasSecond) activator2: const TestIntent2(),
      },
      child: const Focus(autofocus: true, child: SizedBox(width: 100, height: 100)),
    ),
  );
}
