// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  Widget snapshotText(BuildContext context, AsyncSnapshot<String> snapshot) {
    return Text(snapshot.toString(), textDirection: TextDirection.ltr);
  }
  group('AsyncSnapshot', () {
    test('requiring data preserves the stackTrace', () {
      final StackTrace originalStackTrace = StackTrace.current;

      try {
        AsyncSnapshot<String>.withError(
          ConnectionState.done,
          Error(),
          originalStackTrace,
        ).requireData;
        fail('requireData did not throw');
      } catch (error, stackTrace) {
        expect(stackTrace, originalStackTrace);
      }
    });
    test('requiring data succeeds if data is present', () {
      expect(
        const AsyncSnapshot<String>.withData(ConnectionState.done, 'hello').requireData,
        'hello',
      );
    });
    test('requiring data fails if there is an error', () {
      expect(
        () => const AsyncSnapshot<String>.withError(ConnectionState.done, 'error').requireData,
        throwsA(equals('error')),
      );
    });
    test('requiring data fails if snapshot has neither data nor error', () {
      expect(
        () => const AsyncSnapshot<String>.nothing().requireData,
        throwsStateError,
      );
    });
    test('AsyncSnapshot basic constructors', () {
      expect(const AsyncSnapshot<int>.nothing().connectionState, ConnectionState.none);
      expect(const AsyncSnapshot<int>.nothing().data, isNull);
      expect(const AsyncSnapshot<int>.nothing().error, isNull);
      expect(const AsyncSnapshot<int>.nothing().stackTrace, isNull);
      expect(const AsyncSnapshot<int>.waiting().connectionState, ConnectionState.waiting);
      expect(const AsyncSnapshot<int>.waiting().data, isNull);
      expect(const AsyncSnapshot<int>.waiting().error, isNull);
      expect(const AsyncSnapshot<int>.waiting().stackTrace, isNull);
    });
    test('withError uses empty stack trace if no stack trace is specified', () {
      // We need to store the error as a local variable in order for the
      // equality check on the error to be true.
      final Error error = Error();
      expect(
        AsyncSnapshot<int>.withError(ConnectionState.done, error),
        AsyncSnapshot<int>.withError(ConnectionState.done, error),
      );
    });
  });
  group('Async smoke tests', () {
    testWidgets('FutureBuilder', (WidgetTester tester) async {
      await tester.pumpWidget(FutureBuilder<String>(
        future: Future<String>.value('hello'),
        builder: snapshotText,
      ));
      await eventFiring(tester);
    });
    testWidgets('StreamBuilder', (WidgetTester tester) async {
      await tester.pumpWidget(StreamBuilder<String>(
        stream: Stream<String>.fromIterable(<String>['hello', 'world']),
        builder: snapshotText,
      ));
      await eventFiring(tester);
    });
    testWidgets('StreamFold', (WidgetTester tester) async {
      await tester.pumpWidget(StringCollector(
        stream: Stream<String>.fromIterable(<String>['hello', 'world']),
      ));
      await eventFiring(tester);
    });
  });
  group('FutureBuilder', () {
    testWidgets('gives expected snapshot with SynchronousFuture', (WidgetTester tester) async {
      final SynchronousFuture<String> future = SynchronousFuture<String>('flutter');
      await tester.pumpWidget(FutureBuilder<String>(
        future: future,
        builder: (BuildContext context, AsyncSnapshot<String> snapshot) {
          expect(snapshot.connectionState, ConnectionState.done);
          expect(snapshot.data, 'flutter');
          expect(snapshot.error, null);
          expect(snapshot.stackTrace, null);
          return const Placeholder();
        },
      ));
    });

    testWidgets('gracefully handles transition from null future', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, builder: snapshotText, future: null,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsOneWidget);
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, future: completer.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to null future', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, future: completer.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, builder: snapshotText, future: null,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsOneWidget);
      completer.complete('hello');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to other future', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final Completer<String> completerA = Completer<String>();
      final Completer<String> completerB = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, future: completerA.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      await tester.pumpWidget(FutureBuilder<String>(
        key: key, future: completerB.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      completerB.complete('B');
      completerA.complete('A');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, B, null, null)'), findsOneWidget);
    });
    testWidgets('tracks life-cycle of Future to success', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        future: completer.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      completer.complete('hello');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, hello, null, null)'), findsOneWidget);
    });
    testWidgets('tracks life-cycle of Future to error', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        future: completer.future, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      completer.completeError('bad', StackTrace.fromString('trace'));
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, null, bad, trace)'), findsOneWidget);
    });
    testWidgets('runs the builder using given initial data', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key,
        future: null,
        builder: snapshotText,
        initialData: 'I',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, I, null, null)'), findsOneWidget);
    });
    testWidgets('ignores initialData when reconfiguring', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key,
        future: null,
        builder: snapshotText,
        initialData: 'I',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, I, null, null)'), findsOneWidget);
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(FutureBuilder<String>(
        key: key,
        future: completer.future,
        builder: snapshotText,
        initialData: 'Ignored',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, I, null, null)'), findsOneWidget);
    });
    testWidgets('debugRethrowError rethrows caught error', (WidgetTester tester) async {
      FutureBuilder.debugRethrowError = true;
      final Completer<void> caughtError = Completer<void>();
      await runZonedGuarded(() async {
        final Completer<String> completer = Completer<String>();
        await tester.pumpWidget(FutureBuilder<String>(
          future: completer.future,
          builder: snapshotText,
        ), duration: const Duration(seconds: 1));
        completer.completeError('bad');
      }, (Object error, StackTrace stack) {
        expectSync(error, equals('bad'));
        caughtError.complete();
      });
      await tester.pumpAndSettle();
      expectSync(caughtError.isCompleted, isTrue);
      FutureBuilder.debugRethrowError = false;
    });
  });
  group('StreamBuilder', () {
    testWidgets('gracefully handles transition from null stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, builder: snapshotText, stream: null,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsOneWidget);
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, stream: controller.stream, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to null stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, stream: controller.stream, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, builder: snapshotText, stream: null,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to other stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controllerA = StreamController<String>();
      final StreamController<String> controllerB = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, stream: controllerA.stream, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, stream: controllerB.stream, builder: snapshotText,
      ));
      controllerB.add('B');
      controllerA.add('A');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.active, B, null, null)'), findsOneWidget);
    });
    testWidgets('tracks events and errors of stream until completion', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key, stream: controller.stream, builder: snapshotText,
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsOneWidget);
      controller.add('1');
      controller.add('2');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.active, 2, null, null)'), findsOneWidget);
      controller.add('3');
      controller.addError('bad', StackTrace.fromString('trace'));
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.active, null, bad, trace)'), findsOneWidget);
      controller.add('4');
      controller.close();
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, 4, null, null)'), findsOneWidget);
    });
    testWidgets('runs the builder using given initial data', (WidgetTester tester) async {
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        stream: controller.stream,
        builder: snapshotText,
        initialData: 'I',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, I, null, null)'), findsOneWidget);
    });
    testWidgets('ignores initialData when reconfiguring', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key,
        stream: null,
        builder: snapshotText,
        initialData: 'I',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, I, null, null)'), findsOneWidget);
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StreamBuilder<String>(
        key: key,
        stream: controller.stream,
        builder: snapshotText,
        initialData: 'Ignored',
      ));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, I, null, null)'), findsOneWidget);
    });
  });
  group('FutureBuilder and StreamBuilder behave identically on Stream from Future', () {
    testWidgets('when completing with data', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(future: completer.future, builder: snapshotText),
        StreamBuilder<String>(stream: completer.future.asStream(), builder: snapshotText),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsNWidgets(2));
      completer.complete('hello');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, hello, null, null)'), findsNWidgets(2));
    });
    testWidgets('when completing with error and with empty stack trace', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(future: completer.future, builder: snapshotText),
        StreamBuilder<String>(stream: completer.future.asStream(), builder: snapshotText),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsNWidgets(2));
      completer.completeError('bad', StackTrace.empty);
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, null, bad, )'), findsNWidgets(2));
    });
    testWidgets('when completing with error and with stack trace', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(future: completer.future, builder: snapshotText),
        StreamBuilder<String>(stream: completer.future.asStream(), builder: snapshotText),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, null, null, null)'), findsNWidgets(2));
      completer.completeError('bad', StackTrace.fromString('trace'));
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, null, bad, trace)'), findsNWidgets(2));
    });
    testWidgets('when Future is null', (WidgetTester tester) async {
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(builder: snapshotText, future: null),
        StreamBuilder<String>(builder: snapshotText, stream: null,),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, null, null, null)'), findsNWidgets(2));
    });
    testWidgets('when initialData is used with null Future and Stream', (WidgetTester tester) async {
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(builder: snapshotText, initialData: 'I', future: null),
        StreamBuilder<String>(builder: snapshotText, initialData: 'I', stream: null),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.none, I, null, null)'), findsNWidgets(2));
    });
    testWidgets('when using initialData and completing with data', (WidgetTester tester) async {
      final Completer<String> completer = Completer<String>();
      await tester.pumpWidget(Column(children: <Widget>[
        FutureBuilder<String>(future: completer.future, builder: snapshotText, initialData: 'I'),
        StreamBuilder<String>(stream: completer.future.asStream(), builder: snapshotText, initialData: 'I'),
      ]));
      expect(find.text('AsyncSnapshot<String>(ConnectionState.waiting, I, null, null)'), findsNWidgets(2));
      completer.complete('hello');
      await eventFiring(tester);
      expect(find.text('AsyncSnapshot<String>(ConnectionState.done, hello, null, null)'), findsNWidgets(2));
    });
  });
  group('StreamBuilderBase', () {
    testWidgets('gracefully handles transition from null stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(StringCollector(key: key));
      expect(find.text(''), findsOneWidget);
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StringCollector(key: key, stream: controller.stream));
      expect(find.text('conn'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to null stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StringCollector(key: key, stream: controller.stream));
      expect(find.text('conn'), findsOneWidget);
      await tester.pumpWidget(StringCollector(key: key));
      expect(find.text('conn, disc'), findsOneWidget);
    });
    testWidgets('gracefully handles transition to other stream', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controllerA = StreamController<String>();
      final StreamController<String> controllerB = StreamController<String>();
      await tester.pumpWidget(StringCollector(key: key, stream: controllerA.stream));
      await tester.pumpWidget(StringCollector(key: key, stream: controllerB.stream));
      controllerA.add('A');
      controllerB.add('B');
      await eventFiring(tester);
      expect(find.text('conn, disc, conn, data:B'), findsOneWidget);
    });
    testWidgets('tracks events and errors until completion', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final StreamController<String> controller = StreamController<String>();
      await tester.pumpWidget(StringCollector(key: key, stream: controller.stream));
      controller.add('1');
      controller.addError('bad', StackTrace.fromString('trace'));
      controller.add('2');
      controller.close();
      await eventFiring(tester);
      expect(find.text('conn, data:1, error:bad stackTrace:trace, data:2, done'), findsOneWidget);
    });
  });
}

Future<void> eventFiring(WidgetTester tester) async {
  await tester.pump(Duration.zero);
}

class StringCollector extends StreamBuilderBase<String, List<String>> {
  const StringCollector({ super.key, super.stream });

  @override
  List<String> initial() => <String>[];

  @override
  List<String> afterConnected(List<String> current) => current..add('conn');

  @override
  List<String> afterData(List<String> current, String data) => current..add('data:$data');

  @override
  List<String> afterError(List<String> current, dynamic error, StackTrace stackTrace) => current..add('error:$error stackTrace:$stackTrace');

  @override
  List<String> afterDone(List<String> current) => current..add('done');

  @override
  List<String> afterDisconnected(List<String> current) => current..add('disc');

  @override
  Widget buildFromSummary(BuildContext context, List<String> currentSummary) => Text(currentSummary.join(', '), textDirection: TextDirection.ltr);
}
