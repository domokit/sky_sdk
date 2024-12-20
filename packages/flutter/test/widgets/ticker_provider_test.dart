// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  testWidgets('TickerMode', (WidgetTester tester) async {
    const Widget widget = TickerMode(enabled: false, child: CircularProgressIndicator());
    expect(widget.toString, isNot(throwsException));

    await tester.pumpWidget(widget);

    expect(tester.binding.transientCallbackCount, 0);

    await tester.pumpWidget(const TickerMode(enabled: true, child: CircularProgressIndicator()));

    expect(tester.binding.transientCallbackCount, 1);

    await tester.pumpWidget(const TickerMode(enabled: false, child: CircularProgressIndicator()));

    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('Navigation with TickerMode', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: const LinearProgressIndicator(),
        routes: <String, WidgetBuilder>{'/test': (BuildContext context) => const Text('hello')},
      ),
    );
    expect(tester.binding.transientCallbackCount, 1);
    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/test');
    expect(tester.binding.transientCallbackCount, 2);
    await tester.pump();
    expect(tester.binding.transientCallbackCount, 2);
    await tester.pump(const Duration(seconds: 5));
    expect(tester.binding.transientCallbackCount, 0);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    expect(tester.binding.transientCallbackCount, 1);
    await tester.pump();
    expect(tester.binding.transientCallbackCount, 2);
    await tester.pump(const Duration(seconds: 5));
    expect(tester.binding.transientCallbackCount, 1);
  });

  testWidgets('SingleTickerProviderStateMixin can handle not being used', (
    WidgetTester tester,
  ) async {
    const Widget widget = BoringTickerTest();
    expect(widget.toString, isNot(throwsException));

    await tester.pumpWidget(widget);
    await tester.pumpWidget(Container());
    // the test is that this doesn't crash, like it used to...
  });

  group('TickerProviderStateMixin assertion control test', () {
    testWidgets(
      'SingleTickerProviderStateMixin create multiple tickers',
      experimentalLeakTesting:
          LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
      (WidgetTester tester) async {
        const Widget widget = _SingleTickerCreateMultipleTicker();
        await tester.pumpWidget(widget);
        final dynamic exception = tester.takeException();
        expect(exception, isNotNull);
        expect(exception, isFlutterError);
        final FlutterError error = exception as FlutterError;
        expect(error.diagnostics.length, 3);
        expect(error.diagnostics[2].level, DiagnosticLevel.hint);
        expect(
          error.diagnostics[2].toStringDeep(),
          equalsIgnoringHashCodes(
            'If a State is used for multiple AnimationController objects, or\n'
            'if it is passed to other objects and those objects might use it\n'
            'more than one time in total, then instead of mixing in a\n'
            'SingleTickerProviderStateMixin, use a regular\n'
            'TickerProviderStateMixin.\n',
          ),
        );
        expect(
          error.toStringDeep(),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   _SingleTickerCreateMultipleTickerState is a\n'
            '   SingleTickerProviderStateMixin but multiple tickers were created.\n'
            '   A SingleTickerProviderStateMixin can only be used as a\n'
            '   TickerProvider once.\n'
            '   If a State is used for multiple AnimationController objects, or\n'
            '   if it is passed to other objects and those objects might use it\n'
            '   more than one time in total, then instead of mixing in a\n'
            '   SingleTickerProviderStateMixin, use a regular\n'
            '   TickerProviderStateMixin.\n',
          ),
        );
      },
    );

    testWidgets('SingleTickerProviderStateMixin dispose while active', (WidgetTester tester) async {
      final GlobalKey<_SingleTickerTestState> key = GlobalKey<_SingleTickerTestState>();
      final Widget widget = _SingleTickerTest(key: key);
      await tester.pumpWidget(widget);
      FlutterError? error;
      key.currentState!.controller.repeat();
      try {
        key.currentState!.dispose();
      } on FlutterError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(error!.diagnostics.length, 4);
        expect(error.diagnostics[2].level, DiagnosticLevel.hint);
        expect(
          error.diagnostics[2].toStringDeep(),
          'Tickers used by AnimationControllers should be disposed by\n'
          'calling dispose() on the AnimationController itself. Otherwise,\n'
          'the ticker will leak.\n',
        );
        expect(error.diagnostics[3], isA<DiagnosticsProperty<Ticker>>());
        expect(
          error.toStringDeep().split('\n').take(13).join('\n'),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   _SingleTickerTestState#00000(ticker active) was disposed with an\n'
            '   active Ticker.\n'
            '   _SingleTickerTestState created a Ticker via its\n'
            '   SingleTickerProviderStateMixin, but at the time dispose() was\n'
            '   called on the mixin, that Ticker was still active. The Ticker\n'
            '   must be disposed before calling super.dispose().\n'
            '   Tickers used by AnimationControllers should be disposed by\n'
            '   calling dispose() on the AnimationController itself. Otherwise,\n'
            '   the ticker will leak.\n'
            '   The offending ticker was:\n'
            '     Ticker(created by _SingleTickerTestState#00000)\n'
            '     The stack trace when the Ticker was actually created was:',
          ),
        );
        key.currentState!.controller.stop();
      }
    });

    testWidgets('SingleTickerProviderStateMixin dispose while active', (WidgetTester tester) async {
      final GlobalKey<_SingleTickerTestState> key = GlobalKey<_SingleTickerTestState>();
      final Widget widget = _SingleTickerTest(key: key);
      await tester.pumpWidget(widget);
      FlutterError? error;
      key.currentState!.controller.repeat();
      try {
        key.currentState!.dispose();
      } on FlutterError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(error!.diagnostics.length, 4);
        expect(error.diagnostics[2].level, DiagnosticLevel.hint);
        expect(
          error.diagnostics[2].toStringDeep(),
          'Tickers used by AnimationControllers should be disposed by\n'
          'calling dispose() on the AnimationController itself. Otherwise,\n'
          'the ticker will leak.\n',
        );
        expect(error.diagnostics[3], isA<DiagnosticsProperty<Ticker>>());
        expect(
          error.toStringDeep().split('\n').take(13).join('\n'),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   _SingleTickerTestState#00000(ticker active) was disposed with an\n'
            '   active Ticker.\n'
            '   _SingleTickerTestState created a Ticker via its\n'
            '   SingleTickerProviderStateMixin, but at the time dispose() was\n'
            '   called on the mixin, that Ticker was still active. The Ticker\n'
            '   must be disposed before calling super.dispose().\n'
            '   Tickers used by AnimationControllers should be disposed by\n'
            '   calling dispose() on the AnimationController itself. Otherwise,\n'
            '   the ticker will leak.\n'
            '   The offending ticker was:\n'
            '     Ticker(created by _SingleTickerTestState#00000)\n'
            '     The stack trace when the Ticker was actually created was:',
          ),
        );
        key.currentState!.controller.stop();
      }
    });

    testWidgets('TickerProviderStateMixin dispose while any ticker is active', (
      WidgetTester tester,
    ) async {
      final GlobalKey<_MultipleTickerTestState> key = GlobalKey<_MultipleTickerTestState>();
      final Widget widget = _MultipleTickerTest(key: key);
      await tester.pumpWidget(widget);
      FlutterError? error;
      key.currentState!.controllers.first.repeat();
      try {
        key.currentState!.dispose();
      } on FlutterError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(error!.diagnostics.length, 4);
        expect(error.diagnostics[2].level, DiagnosticLevel.hint);
        expect(
          error.diagnostics[2].toStringDeep(),
          'Tickers used by AnimationControllers should be disposed by\n'
          'calling dispose() on the AnimationController itself. Otherwise,\n'
          'the ticker will leak.\n',
        );
        expect(error.diagnostics[3], isA<DiagnosticsProperty<Ticker>>());
        expect(
          error.toStringDeep().split('\n').take(12).join('\n'),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   _MultipleTickerTestState#00000(tickers: tracking 2 tickers) was\n'
            '   disposed with an active Ticker.\n'
            '   _MultipleTickerTestState created a Ticker via its\n'
            '   TickerProviderStateMixin, but at the time dispose() was called on\n'
            '   the mixin, that Ticker was still active. All Tickers must be\n'
            '   disposed before calling super.dispose().\n'
            '   Tickers used by AnimationControllers should be disposed by\n'
            '   calling dispose() on the AnimationController itself. Otherwise,\n'
            '   the ticker will leak.\n'
            '   The offending ticker was:\n'
            '     _WidgetTicker(created by _MultipleTickerTestState#00000)',
          ),
        );
        key.currentState!.controllers.first.stop();
      }
    });
  });

  testWidgets('SingleTickerProviderStateMixin does not call State.toString', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const _SingleTickerTest());
    expect(tester.state<_SingleTickerTestState>(find.byType(_SingleTickerTest)).toStringCount, 0);
  });

  testWidgets('TickerProviderStateMixin does not call State.toString', (WidgetTester tester) async {
    await tester.pumpWidget(const _MultipleTickerTest());
    expect(
      tester.state<_MultipleTickerTestState>(find.byType(_MultipleTickerTest)).toStringCount,
      0,
    );
  });
}

class BoringTickerTest extends StatefulWidget {
  const BoringTickerTest({super.key});
  @override
  State<BoringTickerTest> createState() => _BoringTickerTestState();
}

class _BoringTickerTestState extends State<BoringTickerTest> with SingleTickerProviderStateMixin {
  @override
  Widget build(BuildContext context) => Container();
}

class _SingleTickerTest extends StatefulWidget {
  const _SingleTickerTest({super.key});

  @override
  _SingleTickerTestState createState() => _SingleTickerTestState();
}

class _SingleTickerTestState extends State<_SingleTickerTest> with SingleTickerProviderStateMixin {
  late AnimationController controller;

  @override
  void initState() {
    super.initState();
    controller = AnimationController(vsync: this, duration: const Duration(seconds: 100));
  }

  @override
  Widget build(BuildContext context) {
    return Container();
  }

  int toStringCount = 0;

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    toStringCount += 1;
    return super.toString(minLevel: minLevel);
  }
}

class _MultipleTickerTest extends StatefulWidget {
  const _MultipleTickerTest({super.key});

  @override
  _MultipleTickerTestState createState() => _MultipleTickerTestState();
}

class _MultipleTickerTestState extends State<_MultipleTickerTest> with TickerProviderStateMixin {
  List<AnimationController> controllers = <AnimationController>[];

  @override
  void initState() {
    super.initState();
    const Duration duration = Duration(seconds: 100);
    controllers.add(AnimationController(vsync: this, duration: duration));
    controllers.add(AnimationController(vsync: this, duration: duration));
  }

  @override
  Widget build(BuildContext context) {
    return Container();
  }

  int toStringCount = 0;

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    toStringCount += 1;
    return super.toString(minLevel: minLevel);
  }
}

class _SingleTickerCreateMultipleTicker extends StatefulWidget {
  const _SingleTickerCreateMultipleTicker();

  @override
  _SingleTickerCreateMultipleTickerState createState() => _SingleTickerCreateMultipleTickerState();
}

class _SingleTickerCreateMultipleTickerState extends State<_SingleTickerCreateMultipleTicker>
    with SingleTickerProviderStateMixin {
  @override
  void initState() {
    super.initState();
    AnimationController(duration: const Duration(seconds: 5), vsync: this);
    AnimationController(duration: const Duration(seconds: 6), vsync: this);
  }

  @override
  Widget build(BuildContext context) {
    return Container();
  }
}
