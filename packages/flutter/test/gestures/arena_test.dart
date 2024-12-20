// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

const int primaryKey = 4;

class TestGestureArenaMember extends GestureArenaMember {
  bool acceptRan = false;

  @override
  void acceptGesture(int key) {
    expect(key, equals(primaryKey));
    acceptRan = true;
  }

  bool rejectRan = false;

  @override
  void rejectGesture(int key) {
    expect(key, equals(primaryKey));
    rejectRan = true;
  }
}

class GestureTester {
  GestureArenaManager arena = GestureArenaManager();
  TestGestureArenaMember first = TestGestureArenaMember();
  TestGestureArenaMember second = TestGestureArenaMember();

  late GestureArenaEntry firstEntry;
  void addFirst() {
    firstEntry = arena.add(primaryKey, first);
  }

  late GestureArenaEntry secondEntry;
  void addSecond() {
    secondEntry = arena.add(primaryKey, second);
  }

  void expectNothing() {
    expect(first.acceptRan, isFalse);
    expect(first.rejectRan, isFalse);
    expect(second.acceptRan, isFalse);
    expect(second.rejectRan, isFalse);
  }

  void expectFirstWin() {
    expect(first.acceptRan, isTrue);
    expect(first.rejectRan, isFalse);
    expect(second.acceptRan, isFalse);
    expect(second.rejectRan, isTrue);
  }

  void expectSecondWin() {
    expect(first.acceptRan, isFalse);
    expect(first.rejectRan, isTrue);
    expect(second.acceptRan, isTrue);
    expect(second.rejectRan, isFalse);
  }
}

void main() {
  test('Should win by accepting', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.firstEntry.resolve(GestureDisposition.accepted);
    tester.expectFirstWin();
  });

  test('Should win by sweep', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.arena.sweep(primaryKey);
    tester.expectFirstWin();
  });

  test('Should win on release after hold sweep release', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.arena.hold(primaryKey);
    tester.expectNothing();
    tester.arena.sweep(primaryKey);
    tester.expectNothing();
    tester.arena.release(primaryKey);
    tester.expectFirstWin();
  });

  test('Should win on sweep after hold release sweep', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.arena.hold(primaryKey);
    tester.expectNothing();
    tester.arena.release(primaryKey);
    tester.expectNothing();
    tester.arena.sweep(primaryKey);
    tester.expectFirstWin();
  });

  test('Only first winner should win', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.firstEntry.resolve(GestureDisposition.accepted);
    tester.secondEntry.resolve(GestureDisposition.accepted);
    tester.expectFirstWin();
  });

  test('Only first winner should win, regardless of order', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.arena.close(primaryKey);
    tester.expectNothing();
    tester.secondEntry.resolve(GestureDisposition.accepted);
    tester.firstEntry.resolve(GestureDisposition.accepted);
    tester.expectSecondWin();
  });

  test('Win before close is delayed to close', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.expectNothing();
    tester.firstEntry.resolve(GestureDisposition.accepted);
    tester.expectNothing();
    tester.arena.close(primaryKey);
    tester.expectFirstWin();
  });

  test('Win before close is delayed to close, and only first winner should win', () {
    final GestureTester tester = GestureTester();
    tester.addFirst();
    tester.addSecond();
    tester.expectNothing();
    tester.firstEntry.resolve(GestureDisposition.accepted);
    tester.secondEntry.resolve(GestureDisposition.accepted);
    tester.expectNothing();
    tester.arena.close(primaryKey);
    tester.expectFirstWin();
  });

  test(
    'Win before close is delayed to close, and only first winner should win, regardless of order',
    () {
      final GestureTester tester = GestureTester();
      tester.addFirst();
      tester.addSecond();
      tester.expectNothing();
      tester.secondEntry.resolve(GestureDisposition.accepted);
      tester.firstEntry.resolve(GestureDisposition.accepted);
      tester.expectNothing();
      tester.arena.close(primaryKey);
      tester.expectSecondWin();
    },
  );
}
