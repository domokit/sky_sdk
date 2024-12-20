// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:fake_async/fake_async.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

class TestGestureArenaMember extends GestureArenaMember {
  @override
  void acceptGesture(int key) {
    accepted = true;
  }

  @override
  void rejectGesture(int key) {
    rejected = true;
  }

  bool accepted = false;
  bool rejected = false;
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late DoubleTapGestureRecognizer tap;
  bool doubleTapRecognized = false;
  TapDownDetails? doubleTapDownDetails;
  bool doubleTapCanceled = false;

  setUp(() {
    tap = DoubleTapGestureRecognizer();
    addTearDown(tap.dispose);

    doubleTapRecognized = false;
    tap.onDoubleTap = () {
      expect(doubleTapRecognized, isFalse);
      doubleTapRecognized = true;
    };

    doubleTapDownDetails = null;
    tap.onDoubleTapDown = (TapDownDetails details) {
      expect(doubleTapDownDetails, isNull);
      doubleTapDownDetails = details;
    };

    doubleTapCanceled = false;
    tap.onDoubleTapCancel = () {
      expect(doubleTapCanceled, isFalse);
      doubleTapCanceled = true;
    };
  });

  tearDown(() {
    tap.dispose();
  });

  // Down/up pair 1: normal tap sequence
  const PointerDownEvent down1 = PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));

  const PointerUpEvent up1 = PointerUpEvent(pointer: 1, position: Offset(11.0, 9.0));

  // Down/up pair 2: normal tap sequence close to pair 1
  const PointerDownEvent down2 = PointerDownEvent(pointer: 2, position: Offset(12.0, 12.0));

  const PointerUpEvent up2 = PointerUpEvent(pointer: 2, position: Offset(13.0, 11.0));

  // Down/up pair 3: normal tap sequence far away from pair 1
  const PointerDownEvent down3 = PointerDownEvent(pointer: 3, position: Offset(130.0, 130.0));

  const PointerUpEvent up3 = PointerUpEvent(pointer: 3, position: Offset(131.0, 129.0));

  // Down/move/up sequence 4: intervening motion
  const PointerDownEvent down4 = PointerDownEvent(pointer: 4, position: Offset(10.0, 10.0));

  const PointerMoveEvent move4 = PointerMoveEvent(pointer: 4, position: Offset(25.0, 25.0));

  const PointerUpEvent up4 = PointerUpEvent(pointer: 4, position: Offset(25.0, 25.0));

  // Down/up pair 5: normal tap sequence identical to pair 1
  const PointerDownEvent down5 = PointerDownEvent(pointer: 5, position: Offset(10.0, 10.0));

  const PointerUpEvent up5 = PointerUpEvent(pointer: 5, position: Offset(11.0, 9.0));

  // Down/up pair 6: normal tap sequence close to pair 1 but on secondary button
  const PointerDownEvent down6 = PointerDownEvent(
    pointer: 6,
    position: Offset(10.0, 10.0),
    buttons: kSecondaryMouseButton,
  );

  const PointerUpEvent up6 = PointerUpEvent(pointer: 6, position: Offset(11.0, 9.0));

  testGesture('Should recognize double tap', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down2);
    tester.closeArena(2);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down2.position);
    expect(doubleTapDownDetails!.localPosition, down2.localPosition);
    tester.route(down2);
    expect(doubleTapRecognized, isFalse);

    tester.route(up2);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should recognize double tap with secondaryButton', (GestureTester tester) {
    final DoubleTapGestureRecognizer tapSecondary = DoubleTapGestureRecognizer(
      allowedButtonsFilter: (int buttons) => buttons == kSecondaryButton,
    );
    addTearDown(tapSecondary.dispose);
    tapSecondary.onDoubleTap = () {
      doubleTapRecognized = true;
    };
    tapSecondary.onDoubleTapDown = (TapDownDetails details) {
      doubleTapDownDetails = details;
    };
    tapSecondary.onDoubleTapCancel = () {
      doubleTapCanceled = true;
    };

    // Down/up pair 7: normal tap sequence close to pair 6
    const PointerDownEvent down7 = PointerDownEvent(
      pointer: 7,
      position: Offset(10.0, 10.0),
      buttons: kSecondaryMouseButton,
    );

    const PointerUpEvent up7 = PointerUpEvent(pointer: 7, position: Offset(11.0, 9.0));

    tapSecondary.addPointer(down6);
    tester.closeArena(6);
    tester.route(down6);
    tester.route(up6);
    GestureBinding.instance.gestureArena.sweep(6);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tapSecondary.addPointer(down7);
    tester.closeArena(7);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down7.position);
    expect(doubleTapDownDetails!.localPosition, down7.localPosition);
    tester.route(down7);
    expect(doubleTapRecognized, isFalse);

    tester.route(up7);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Inter-tap distance cancels double tap', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tap.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);
    tester.route(up3);
    GestureBinding.instance.gestureArena.sweep(3);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Intra-tap distance cancels double tap', (GestureTester tester) {
    tap.addPointer(down4);
    tester.closeArena(4);
    tester.route(down4);

    tester.route(move4);
    tester.route(up4);
    GestureBinding.instance.gestureArena.sweep(4);

    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down2);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Inter-tap delay cancels double tap', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.async.elapse(const Duration(milliseconds: 5000));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Inter-tap delay resets double tap, allowing third tap to be a double-tap', (
    GestureTester tester,
  ) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.async.elapse(const Duration(milliseconds: 5000));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down5);
    tester.closeArena(5);
    tester.route(down5);
    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down5.position);
    expect(doubleTapDownDetails!.localPosition, down5.localPosition);

    tester.route(up5);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(5);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Intra-tap delay does not cancel double tap', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.async.elapse(const Duration(milliseconds: 1000));
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(doubleTapDownDetails, isNull);

    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down2.position);
    expect(doubleTapDownDetails!.localPosition, down2.localPosition);

    tester.route(up2);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should not recognize two overlapping taps', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);

    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down1);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should recognize one tap of group followed by second tap', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);

    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down1);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down1.position);
    expect(doubleTapDownDetails!.localPosition, down1.localPosition);

    tester.route(up1);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should cancel on arena reject during first tap', (GestureTester tester) {
    tap.addPointer(down1);
    final TestGestureArenaMember member = TestGestureArenaMember();
    final GestureArenaEntry entry = GestureBinding.instance.gestureArena.add(1, member);
    tester.closeArena(1);
    tester.route(down1);

    tester.route(up1);
    entry.resolve(GestureDisposition.accepted);
    expect(member.accepted, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);

    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should cancel on arena reject between taps', (GestureTester tester) {
    tap.addPointer(down1);
    final TestGestureArenaMember member = TestGestureArenaMember();
    final GestureArenaEntry entry = GestureBinding.instance.gestureArena.add(1, member);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    entry.resolve(GestureDisposition.accepted);
    expect(member.accepted, isTrue);

    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Should cancel on arena reject during last tap', (GestureTester tester) {
    tap.addPointer(down1);
    final TestGestureArenaMember member = TestGestureArenaMember();
    final GestureArenaEntry entry = GestureBinding.instance.gestureArena.add(1, member);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down2.position);
    expect(doubleTapDownDetails!.localPosition, down2.localPosition);
    expect(doubleTapCanceled, isFalse);

    entry.resolve(GestureDisposition.accepted);
    expect(member.accepted, isTrue);
    expect(doubleTapCanceled, isTrue);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapRecognized, isFalse);
  });

  testGesture('Passive gesture should trigger on double tap cancel', (GestureTester tester) {
    FakeAsync().run((FakeAsync async) {
      tap.addPointer(down1);
      final TestGestureArenaMember member = TestGestureArenaMember();
      GestureBinding.instance.gestureArena.add(1, member);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);

      expect(member.accepted, isFalse);
      async.elapse(const Duration(milliseconds: 5000));
      expect(member.accepted, isTrue);

      expect(doubleTapRecognized, isFalse);
      expect(doubleTapDownDetails, isNull);
      expect(doubleTapCanceled, isFalse);
    });
  });

  testGesture('Should not recognize two over-rapid taps', (GestureTester tester) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.async.elapse(const Duration(milliseconds: 10));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNull);
    expect(doubleTapCanceled, isFalse);
  });

  testGesture('Over-rapid taps resets double tap, allowing third tap to be a double-tap', (
    GestureTester tester,
  ) {
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.async.elapse(const Duration(milliseconds: 10));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(doubleTapDownDetails, isNull);

    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down5);
    tester.closeArena(5);
    tester.route(down5);
    expect(doubleTapRecognized, isFalse);
    expect(doubleTapDownDetails, isNotNull);
    expect(doubleTapDownDetails!.globalPosition, down5.position);
    expect(doubleTapDownDetails!.localPosition, down5.localPosition);

    tester.route(up5);
    expect(doubleTapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(5);
    expect(doubleTapCanceled, isFalse);
  });

  group('Enforce consistent-button restriction:', () {
    testGesture('Button change should interrupt existing sequence', (GestureTester tester) {
      // Down1 -> down6 (different button from 1) -> down2 (same button as 1)
      // Down1 and down2 could've been a double tap, but is interrupted by down 6.

      const Duration interval = Duration(milliseconds: 100);
      assert(interval * 2 < kDoubleTapTimeout);
      assert(interval > kDoubleTapMinTime);

      tap.addPointer(down1);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);

      tester.async.elapse(interval);

      tap.addPointer(down6);
      tester.closeArena(6);
      tester.route(down6);
      tester.route(up6);
      GestureBinding.instance.gestureArena.sweep(6);

      tester.async.elapse(interval);
      expect(doubleTapRecognized, isFalse);

      tap.addPointer(down2);
      tester.closeArena(2);
      tester.route(down2);
      tester.route(up2);
      GestureBinding.instance.gestureArena.sweep(2);

      expect(doubleTapRecognized, isFalse);
      expect(doubleTapDownDetails, isNull);
      expect(doubleTapCanceled, isFalse);
    });

    testGesture('Button change with allowedButtonsFilter should interrupt existing sequence', (
      GestureTester tester,
    ) {
      final DoubleTapGestureRecognizer tapPrimary = DoubleTapGestureRecognizer(
        allowedButtonsFilter: (int buttons) => buttons == kPrimaryButton,
      );
      addTearDown(tapPrimary.dispose);
      tapPrimary.onDoubleTap = () {
        doubleTapRecognized = true;
      };
      tapPrimary.onDoubleTapDown = (TapDownDetails details) {
        doubleTapDownDetails = details;
      };
      tapPrimary.onDoubleTapCancel = () {
        doubleTapCanceled = true;
      };

      // Down1 -> down6 (different button from 1) -> down2 (same button as 1)
      // Down1 and down2 could've been a double tap, but is interrupted by down 6.
      // Down6 gets ignored because it's not a primary button. Regardless, the state
      // is reset.
      const Duration interval = Duration(milliseconds: 100);
      assert(interval * 2 < kDoubleTapTimeout);
      assert(interval > kDoubleTapMinTime);

      tapPrimary.addPointer(down1);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);

      tester.async.elapse(interval);

      tapPrimary.addPointer(down6);
      tester.closeArena(6);
      tester.route(down6);
      tester.route(up6);
      GestureBinding.instance.gestureArena.sweep(6);

      tester.async.elapse(interval);
      expect(doubleTapRecognized, isFalse);

      tapPrimary.addPointer(down2);
      tester.closeArena(2);
      tester.route(down2);
      tester.route(up2);
      GestureBinding.instance.gestureArena.sweep(2);

      expect(doubleTapRecognized, isFalse);
      expect(doubleTapDownDetails, isNull);
      expect(doubleTapCanceled, isFalse);
    });

    testGesture('Button change should start a valid sequence', (GestureTester tester) {
      // Down6 -> down1 (different button from 6) -> down2 (same button as 1)

      const Duration interval = Duration(milliseconds: 100);
      assert(interval * 2 < kDoubleTapTimeout);
      assert(interval > kDoubleTapMinTime);

      tap.addPointer(down6);
      tester.closeArena(6);
      tester.route(down6);
      tester.route(up6);
      GestureBinding.instance.gestureArena.sweep(6);

      tester.async.elapse(interval);

      tap.addPointer(down1);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);

      expect(doubleTapRecognized, isFalse);
      expect(doubleTapDownDetails, isNull);
      tester.async.elapse(interval);

      tap.addPointer(down2);
      tester.closeArena(2);
      tester.route(down2);
      expect(doubleTapDownDetails, isNotNull);
      expect(doubleTapDownDetails!.globalPosition, down2.position);
      expect(doubleTapDownDetails!.localPosition, down2.localPosition);
      tester.route(up2);
      GestureBinding.instance.gestureArena.sweep(2);

      expect(doubleTapRecognized, isTrue);
      expect(doubleTapCanceled, isFalse);
    });
  });

  group('Recognizers listening on different buttons do not form competition:', () {
    // This test is assisted by tap recognizers. If a tap gesture has
    // no competing recognizers, a pointer down event triggers its onTapDown
    // immediately; if there are competitors, onTapDown is triggered after a
    // timeout.
    // The following tests make sure that double tap recognizers do not form
    // competition with a tap gesture recognizer listening on a different button.

    final List<String> recognized = <String>[];
    late TapGestureRecognizer tapPrimary;
    late TapGestureRecognizer tapSecondary;
    late DoubleTapGestureRecognizer doubleTap;
    setUp(() {
      tapPrimary =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('tapPrimary');
            };
      addTearDown(tapPrimary.dispose);
      tapSecondary =
          TapGestureRecognizer()
            ..onSecondaryTapDown = (TapDownDetails details) {
              recognized.add('tapSecondary');
            };
      addTearDown(tapSecondary.dispose);
      doubleTap =
          DoubleTapGestureRecognizer()
            ..onDoubleTap = () {
              recognized.add('doubleTap');
            };
      addTearDown(doubleTap.dispose);
    });

    tearDown(() {
      recognized.clear();
      tapPrimary.dispose();
      tapSecondary.dispose();
      doubleTap.dispose();
    });

    testGesture(
      'A primary double tap recognizer does not form competition with a secondary tap recognizer',
      (GestureTester tester) {
        doubleTap.addPointer(down6);
        tapSecondary.addPointer(down6);
        tester.closeArena(down6.pointer);

        tester.route(down6);
        expect(recognized, <String>['tapSecondary']);
      },
    );

    testGesture('A primary double tap recognizer forms competition with a primary tap recognizer', (
      GestureTester tester,
    ) {
      doubleTap.addPointer(down1);
      tapPrimary.addPointer(down1);
      tester.closeArena(down1.pointer);

      tester.route(down1);
      expect(recognized, <String>[]);

      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, <String>['tapPrimary']);
    });
  });

  testGesture('A secondary double tap should not trigger primary', (GestureTester tester) {
    final List<String> recognized = <String>[];
    final DoubleTapGestureRecognizer doubleTap =
        DoubleTapGestureRecognizer()
          ..onDoubleTap = () {
            recognized.add('primary');
          };
    addTearDown(doubleTap.dispose);

    // Down/up pair 7: normal tap sequence close to pair 6
    const PointerDownEvent down7 = PointerDownEvent(
      pointer: 7,
      position: Offset(10.0, 10.0),
      buttons: kSecondaryMouseButton,
    );

    const PointerUpEvent up7 = PointerUpEvent(pointer: 7, position: Offset(11.0, 9.0));

    doubleTap.addPointer(down6);
    tester.closeArena(6);
    tester.route(down6);
    tester.route(up6);
    GestureBinding.instance.gestureArena.sweep(6);

    tester.async.elapse(const Duration(milliseconds: 100));
    doubleTap.addPointer(down7);
    tester.closeArena(7);
    tester.route(down7);
    tester.route(up7);
    expect(recognized, <String>[]);

    recognized.clear();
    doubleTap.dispose();
  });

  testGesture('Buttons filter should cancel invalid taps', (GestureTester tester) {
    final List<String> recognized = <String>[];
    final DoubleTapGestureRecognizer doubleTap = DoubleTapGestureRecognizer(
        allowedButtonsFilter: (int buttons) => false,
      )
      ..onDoubleTap = () {
        recognized.add('primary');
      };
    addTearDown(doubleTap.dispose);

    // Down/up pair 7: normal tap sequence close to pair 6
    const PointerDownEvent down7 = PointerDownEvent(pointer: 7, position: Offset(10.0, 10.0));

    const PointerUpEvent up7 = PointerUpEvent(pointer: 7, position: Offset(11.0, 9.0));

    doubleTap.addPointer(down7);
    tester.closeArena(7);
    tester.route(down7);
    tester.route(up7);
    GestureBinding.instance.gestureArena.sweep(7);

    tester.async.elapse(const Duration(milliseconds: 100));
    doubleTap.addPointer(down6);
    tester.closeArena(6);
    tester.route(down6);
    tester.route(up6);

    expect(recognized, <String>[]);

    recognized.clear();
    doubleTap.dispose();
  });

  // Regression test for https://github.com/flutter/flutter/issues/73667
  testGesture('Unfinished DoubleTap does not prevent competing Tap', (GestureTester tester) {
    int tapCount = 0;
    final DoubleTapGestureRecognizer doubleTap = DoubleTapGestureRecognizer()..onDoubleTap = () {};
    addTearDown(doubleTap.dispose);
    final TapGestureRecognizer tap = TapGestureRecognizer()..onTap = () => tapCount++;
    addTearDown(tap.dispose);

    // Open a arena with 2 members and holding.
    doubleTap.addPointer(down1);
    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    // Open a new arena with only one TapGestureRecognizer.
    tester.async.elapse(const Duration(milliseconds: 100));
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    final PointerMoveEvent move2 = PointerMoveEvent(pointer: 2, position: down2.position);
    tester.route(move2);
    tester.route(up2);
    expect(tapCount, 1); // The second tap will win immediately.
    GestureBinding.instance.gestureArena.sweep(2);

    // Finish the previous gesture arena.
    tester.async.elapse(const Duration(milliseconds: 300));
    expect(
      tapCount,
      1,
    ); // The first tap should not trigger onTap callback though it wins the arena.

    tap.dispose();
    doubleTap.dispose();
  });
}
