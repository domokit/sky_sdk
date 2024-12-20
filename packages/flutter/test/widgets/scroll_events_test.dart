// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Widget _buildScroller({required List<String> log}) {
  return NotificationListener<ScrollNotification>(
    onNotification: (ScrollNotification notification) {
      if (notification is ScrollStartNotification) {
        log.add('scroll-start');
      } else if (notification is ScrollUpdateNotification) {
        log.add('scroll-update');
      } else if (notification is ScrollEndNotification) {
        log.add('scroll-end');
      }
      return false;
    },
    child: const SingleChildScrollView(child: SizedBox(width: 1000.0, height: 1000.0)),
  );
}

void main() {
  Completer<void> animateTo(
    WidgetTester tester,
    double newScrollOffset, {
    required Duration duration,
  }) {
    final Completer<void> completer = Completer<void>();
    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    scrollable.position
        .animateTo(newScrollOffset, duration: duration, curve: Curves.linear)
        .whenComplete(completer.complete);
    return completer;
  }

  void jumpTo(WidgetTester tester, double newScrollOffset) {
    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    scrollable.position.jumpTo(newScrollOffset);
  }

  testWidgets('Scroll event drag', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    final TestGesture gesture = await tester.startGesture(const Offset(100.0, 100.0));
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(seconds: 1));
    expect(log, equals(<String>['scroll-start']));
    await gesture.moveBy(const Offset(-10.0, -10.0));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    await tester.pump(const Duration(seconds: 1));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    await gesture.up();
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-end']));
    await tester.pump(const Duration(seconds: 1));
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-end']));
  });

  testWidgets('Scroll animateTo', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    final Completer<void> completer = animateTo(
      tester,
      100.0,
      duration: const Duration(seconds: 1),
    );
    expect(completer.isCompleted, isFalse);
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    await tester.pump(const Duration(milliseconds: 1500));
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-update', 'scroll-end']));
    expect(completer.isCompleted, isTrue);
  });

  testWidgets('Scroll jumpTo', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    jumpTo(tester, 100.0);
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-end']));
    await tester.pump();
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-end']));
  });

  testWidgets('Scroll jumpTo during animation', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    final Completer<void> completer = animateTo(
      tester,
      100.0,
      duration: const Duration(seconds: 1),
    );
    expect(completer.isCompleted, isFalse);
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    expect(completer.isCompleted, isFalse);

    jumpTo(tester, 100.0);
    expect(completer.isCompleted, isFalse);
    expect(
      log,
      equals(<String>[
        'scroll-start',
        'scroll-update',
        'scroll-end',
        'scroll-start',
        'scroll-update',
        'scroll-end',
      ]),
    );
    await tester.pump(const Duration(milliseconds: 100));
    expect(
      log,
      equals(<String>[
        'scroll-start',
        'scroll-update',
        'scroll-end',
        'scroll-start',
        'scroll-update',
        'scroll-end',
      ]),
    );
    expect(completer.isCompleted, isTrue);
    await tester.pump(const Duration(milliseconds: 1500));
    expect(
      log,
      equals(<String>[
        'scroll-start',
        'scroll-update',
        'scroll-end',
        'scroll-start',
        'scroll-update',
        'scroll-end',
      ]),
    );
    expect(completer.isCompleted, isTrue);
  });

  testWidgets('Scroll scrollTo during animation', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    Completer<void> completer = animateTo(tester, 100.0, duration: const Duration(seconds: 1));
    expect(completer.isCompleted, isFalse);
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    expect(completer.isCompleted, isFalse);

    completer = animateTo(tester, 100.0, duration: const Duration(seconds: 1));
    expect(completer.isCompleted, isFalse);
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['scroll-start', 'scroll-update']));
    await tester.pump(const Duration(milliseconds: 1500));
    expect(log, equals(<String>['scroll-start', 'scroll-update', 'scroll-update', 'scroll-end']));
    expect(completer.isCompleted, isTrue);
  });

  testWidgets('fling, fling generates two start/end pairs', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    // The ideal behavior here would be a single start/end pair, but for
    // simplicity of implementation we compromise here and accept two. Should
    // you find a way to make this work with just one without complicating the
    // API, feel free to change the expectation here.

    expect(log, equals(<String>[]));
    await tester.flingFrom(const Offset(100.0, 100.0), const Offset(-50.0, -50.0), 500.0);
    await tester.pump(const Duration(seconds: 1));
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start']));
    await tester.flingFrom(const Offset(100.0, 100.0), const Offset(-50.0, -50.0), 500.0);
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start', 'scroll-end', 'scroll-start']));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start', 'scroll-end', 'scroll-start', 'scroll-end']));
  });

  testWidgets('fling, pause, fling generates two start/end pairs', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    await tester.flingFrom(const Offset(100.0, 100.0), const Offset(-50.0, -50.0), 500.0);
    await tester.pump(const Duration(seconds: 1));
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start']));
    await tester.pump(const Duration(minutes: 1));
    await tester.flingFrom(const Offset(100.0, 100.0), const Offset(-50.0, -50.0), 500.0);
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start', 'scroll-end', 'scroll-start']));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log, equals(<String>['scroll-start', 'scroll-end', 'scroll-start', 'scroll-end']));
  });

  testWidgets('fling up ends', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(_buildScroller(log: log));

    expect(log, equals(<String>[]));
    await tester.flingFrom(const Offset(100.0, 100.0), const Offset(50.0, 50.0), 500.0);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(log.first, equals('scroll-start'));
    expect(log.last, equals('scroll-end'));
    log.removeWhere((String value) => value == 'scroll-update');
    expect(log.length, equals(2));
    expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, equals(0.0));
  });
}
