// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(gspencergoog): Remove this tag once this test's state leaks/test
// dependencies have been fixed.
// https://github.com/flutter/flutter/issues/85160
// Fails with "flutter test --test-randomize-ordering-seed=20210721"
@Tags(<String>['no-shuffle'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart' hide TextInputAction;
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart' hide TextInputAction;
import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_driver/src/extension/extension.dart';
import 'package:flutter_test/flutter_test.dart' hide TextInputAction;

import 'stubs/stub_command.dart';
import 'stubs/stub_command_extension.dart';
import 'stubs/stub_finder.dart';
import 'stubs/stub_finder_extension.dart';

Future<void> silenceDriverLogger(AsyncCallback callback) async {
  final DriverLogCallback oldLogger = driverLog;
  driverLog = (String source, String message) {};
  try {
    await callback();
  } finally {
    driverLog = oldLogger;
  }
}

void main() {
  group('waitUntilNoTransientCallbacks', () {
    late FlutterDriverExtension driverExtension;
    Map<String, dynamic>? result;
    int messageId = 0;
    final List<String?> log = <String?>[];

    setUp(() {
      result = null;
      driverExtension = FlutterDriverExtension(
        (String? message) async {
          log.add(message);
          return (messageId += 1).toString();
        },
        false,
        true,
      );
    });

    testWidgets('returns immediately when transient callback queue is empty', (
      WidgetTester tester,
    ) async {
      driverExtension
          .call(const WaitForCondition(NoTransientCallbacks()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      await tester.idle();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waits until no transient callbacks', (WidgetTester tester) async {
      SchedulerBinding.instance.scheduleFrameCallback((_) {
        // Intentionally blank. We only care about existence of a callback.
      });

      driverExtension
          .call(const WaitForCondition(NoTransientCallbacks()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('handler', (WidgetTester tester) async {
      expect(log, isEmpty);
      final Map<String, dynamic> response = await driverExtension.call(
        const RequestData('hello').serialize(),
      );
      final RequestDataResult result = RequestDataResult.fromJson(
        response['response'] as Map<String, dynamic>,
      );
      expect(log, <String>['hello']);
      expect(result.message, '1');
    });
  });

  group('waitForCondition', () {
    late FlutterDriverExtension driverExtension;
    Map<String, dynamic>? result;
    int messageId = 0;
    final List<String?> log = <String?>[];

    setUp(() {
      result = null;
      driverExtension = FlutterDriverExtension(
        (String? message) async {
          log.add(message);
          return (messageId += 1).toString();
        },
        false,
        true,
      );
    });

    testWidgets(
      'waiting for NoTransientCallbacks returns immediately when transient callback queue is empty',
      (WidgetTester tester) async {
        driverExtension
            .call(const WaitForCondition(NoTransientCallbacks()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        await tester.idle();
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );

    testWidgets('waiting for NoTransientCallbacks returns until no transient callbacks', (
      WidgetTester tester,
    ) async {
      SchedulerBinding.instance.scheduleFrameCallback((_) {
        // Intentionally blank. We only care about existence of a callback.
      });

      driverExtension
          .call(const WaitForCondition(NoTransientCallbacks()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waiting for NoPendingFrame returns immediately when frame is synced', (
      WidgetTester tester,
    ) async {
      driverExtension
          .call(const WaitForCondition(NoPendingFrame()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      await tester.idle();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waiting for NoPendingFrame returns until no pending scheduled frame', (
      WidgetTester tester,
    ) async {
      SchedulerBinding.instance.scheduleFrame();

      driverExtension
          .call(const WaitForCondition(NoPendingFrame()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waiting for combined conditions returns immediately', (WidgetTester tester) async {
      const SerializableWaitCondition combinedCondition = CombinedCondition(
        <SerializableWaitCondition>[NoTransientCallbacks(), NoPendingFrame()],
      );
      driverExtension
          .call(const WaitForCondition(combinedCondition).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      await tester.idle();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waiting for combined conditions returns until no transient callbacks', (
      WidgetTester tester,
    ) async {
      SchedulerBinding.instance.scheduleFrame();
      SchedulerBinding.instance.scheduleFrameCallback((_) {
        // Intentionally blank. We only care about existence of a callback.
      });

      const SerializableWaitCondition combinedCondition = CombinedCondition(
        <SerializableWaitCondition>[NoTransientCallbacks(), NoPendingFrame()],
      );
      driverExtension
          .call(const WaitForCondition(combinedCondition).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waiting for combined conditions returns until no pending scheduled frame', (
      WidgetTester tester,
    ) async {
      SchedulerBinding.instance.scheduleFrame();
      SchedulerBinding.instance.scheduleFrameCallback((_) {
        // Intentionally blank. We only care about existence of a callback.
      });

      const SerializableWaitCondition combinedCondition = CombinedCondition(
        <SerializableWaitCondition>[NoPendingFrame(), NoTransientCallbacks()],
      );
      driverExtension
          .call(const WaitForCondition(combinedCondition).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets(
      'waiting for NoPendingPlatformMessages returns immediately when there are no platform messages',
      (WidgetTester tester) async {
        driverExtension
            .call(const WaitForCondition(NoPendingPlatformMessages()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        await tester.idle();
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );

    testWidgets(
      'waiting for NoPendingPlatformMessages returns until a single method channel call returns',
      (WidgetTester tester) async {
        const MethodChannel channel = MethodChannel('helloChannel', JSONMethodCodec());
        const MessageCodec<dynamic> jsonMessage = JSONMessageCodec();
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 10),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });
        channel.invokeMethod<String>('sayHello', 'hello');

        driverExtension
            .call(const WaitForCondition(NoPendingPlatformMessages()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        // The channel message are delayed for 10 milliseconds, so nothing happens yet.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, isNull);

        // Now we receive the result.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );

    testWidgets(
      'waiting for NoPendingPlatformMessages returns until both method channel calls return',
      (WidgetTester tester) async {
        const MessageCodec<dynamic> jsonMessage = JSONMessageCodec();
        // Configures channel 1
        const MethodChannel channel1 = MethodChannel('helloChannel1', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel1', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 10),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        // Configures channel 2
        const MethodChannel channel2 = MethodChannel('helloChannel2', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel2', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 20),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        channel1.invokeMethod<String>('sayHello', 'hello');
        channel2.invokeMethod<String>('sayHello', 'hello');

        driverExtension
            .call(const WaitForCondition(NoPendingPlatformMessages()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        // Neither of the channel responses is received, so nothing happens yet.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, isNull);

        // Result of channel 1 is received, but channel 2 is still pending, so still waiting.
        await tester.pump(const Duration(milliseconds: 10));
        expect(result, isNull);

        // Both of the results are received. Now we receive the result.
        await tester.pump(const Duration(milliseconds: 30));
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );

    testWidgets(
      'waiting for NoPendingPlatformMessages returns until new method channel call returns',
      (WidgetTester tester) async {
        const MessageCodec<dynamic> jsonMessage = JSONMessageCodec();
        // Configures channel 1
        const MethodChannel channel1 = MethodChannel('helloChannel1', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel1', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 10),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        // Configures channel 2
        const MethodChannel channel2 = MethodChannel('helloChannel2', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel2', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 20),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        channel1.invokeMethod<String>('sayHello', 'hello');

        // Calls the waiting API before the second channel message is sent.
        driverExtension
            .call(const WaitForCondition(NoPendingPlatformMessages()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        // The first channel message is not received, so nothing happens yet.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, isNull);

        channel2.invokeMethod<String>('sayHello', 'hello');

        // Result of channel 1 is received, but channel 2 is still pending, so still waiting.
        await tester.pump(const Duration(milliseconds: 15));
        expect(result, isNull);

        // Both of the results are received. Now we receive the result.
        await tester.pump(const Duration(milliseconds: 10));
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );

    testWidgets(
      'waiting for NoPendingPlatformMessages returns until both old and new method channel calls return',
      (WidgetTester tester) async {
        const MessageCodec<dynamic> jsonMessage = JSONMessageCodec();
        // Configures channel 1
        const MethodChannel channel1 = MethodChannel('helloChannel1', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel1', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 20),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        // Configures channel 2
        const MethodChannel channel2 = MethodChannel('helloChannel2', JSONMethodCodec());
        tester.binding.defaultBinaryMessenger.setMockMessageHandler('helloChannel2', (
          ByteData? message,
        ) {
          return Future<ByteData>.delayed(
            const Duration(milliseconds: 10),
            () => jsonMessage.encodeMessage(<dynamic>['hello world'])!,
          );
        });

        channel1.invokeMethod<String>('sayHello', 'hello');

        driverExtension
            .call(const WaitForCondition(NoPendingPlatformMessages()).serialize())
            .then<void>(
              expectAsync1((Map<String, dynamic> r) {
                result = r;
              }),
            );

        // The first channel message is not received, so nothing happens yet.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, isNull);

        channel2.invokeMethod<String>('sayHello', 'hello');

        // Result of channel 2 is received, but channel 1 is still pending, so still waiting.
        await tester.pump(const Duration(milliseconds: 10));
        expect(result, isNull);

        // Now we receive the result.
        await tester.pump(const Duration(milliseconds: 5));
        expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
      },
    );
  });

  group('getSemanticsId', () {
    late FlutterDriverExtension driverExtension;
    setUp(() {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, true);
    });

    testWidgets('works when semantics are enabled', (WidgetTester tester) async {
      final SemanticsHandle semantics = tester.ensureSemantics();
      await tester.pumpWidget(const Text('hello', textDirection: TextDirection.ltr));

      final Map<String, String> arguments = GetSemanticsId(const ByText('hello')).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);
      final GetSemanticsIdResult result = GetSemanticsIdResult.fromJson(
        response['response'] as Map<String, dynamic>,
      );

      expect(result.id, 1);
      semantics.dispose();
    });

    testWidgets('throws state error if no data is found', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('hello', textDirection: TextDirection.ltr));

      final Map<String, String> arguments = GetSemanticsId(const ByText('hello')).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);

      expect(response['isError'], true);
      expect(response['response'], contains('Bad state: No semantics data found'));
    }, semanticsEnabled: false);

    testWidgets('throws state error multiple matches are found', (WidgetTester tester) async {
      final SemanticsHandle semantics = tester.ensureSemantics();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView(
            children: const <Widget>[
              SizedBox(width: 100.0, height: 100.0, child: Text('hello')),
              SizedBox(width: 100.0, height: 100.0, child: Text('hello')),
            ],
          ),
        ),
      );

      final Map<String, String> arguments = GetSemanticsId(const ByText('hello')).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);

      expect(response['isError'], true);
      expect(
        response['response'],
        contains('Bad state: Found more than one element with the same ID'),
      );
      semantics.dispose();
    });
  });

  testWidgets('getOffset', (WidgetTester tester) async {
    final FlutterDriverExtension driverExtension = FlutterDriverExtension(
      (String? arg) async => '',
      true,
      true,
    );

    Future<Offset> getOffset(OffsetType offset) async {
      final Map<String, String> arguments = GetOffset(ByValueKey(1), offset).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);
      final GetOffsetResult result = GetOffsetResult.fromJson(
        response['response'] as Map<String, dynamic>,
      );
      return Offset(result.dx, result.dy);
    }

    await tester.pumpWidget(
      Align(
        alignment: Alignment.topLeft,
        child: Transform.translate(
          offset: const Offset(40, 30),
          child: const SizedBox(key: ValueKey<int>(1), width: 100, height: 120),
        ),
      ),
    );

    expect(await getOffset(OffsetType.topLeft), const Offset(40, 30));
    expect(await getOffset(OffsetType.topRight), const Offset(40 + 100.0, 30));
    expect(await getOffset(OffsetType.bottomLeft), const Offset(40, 30 + 120.0));
    expect(await getOffset(OffsetType.bottomRight), const Offset(40 + 100.0, 30 + 120.0));
    expect(await getOffset(OffsetType.center), const Offset(40 + (100 / 2), 30 + (120 / 2)));
  });

  testWidgets('getText', (WidgetTester tester) async {
    await silenceDriverLogger(() async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
      );

      Future<String?> getTextInternal(SerializableFinder search) async {
        final Map<String, String> arguments =
            GetText(search, timeout: const Duration(seconds: 1)).serialize();
        final Map<String, dynamic> result = await driverExtension.call(arguments);
        if (result['isError'] as bool) {
          return null;
        }
        return GetTextResult.fromJson(result['response'] as Map<String, dynamic>).text;
      }

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Column(
              key: const ValueKey<String>('column'),
              children: <Widget>[
                const Text('Hello1', key: ValueKey<String>('text1')),
                SizedBox(
                  height: 25.0,
                  child: RichText(
                    key: const ValueKey<String>('text2'),
                    text: const TextSpan(text: 'Hello2'),
                  ),
                ),
                SizedBox(
                  height: 25.0,
                  child: EditableText(
                    key: const ValueKey<String>('text3'),
                    controller: TextEditingController(text: 'Hello3'),
                    focusNode: FocusNode(),
                    style: const TextStyle(),
                    cursorColor: Colors.red,
                    backgroundCursorColor: Colors.black,
                  ),
                ),
                SizedBox(
                  height: 25.0,
                  child: TextField(
                    key: const ValueKey<String>('text4'),
                    controller: TextEditingController(text: 'Hello4'),
                  ),
                ),
                SizedBox(
                  height: 25.0,
                  child: TextFormField(
                    key: const ValueKey<String>('text5'),
                    controller: TextEditingController(text: 'Hello5'),
                  ),
                ),
                SizedBox(
                  height: 25.0,
                  child: RichText(
                    key: const ValueKey<String>('text6'),
                    text: const TextSpan(
                      children: <TextSpan>[
                        TextSpan(text: 'Hello'),
                        TextSpan(text: ', '),
                        TextSpan(text: 'World'),
                        TextSpan(text: '!'),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      expect(await getTextInternal(ByValueKey('text1')), 'Hello1');
      expect(await getTextInternal(ByValueKey('text2')), 'Hello2');
      expect(await getTextInternal(ByValueKey('text3')), 'Hello3');
      expect(await getTextInternal(ByValueKey('text4')), 'Hello4');
      expect(await getTextInternal(ByValueKey('text5')), 'Hello5');
      expect(await getTextInternal(ByValueKey('text6')), 'Hello, World!');

      // Check if error thrown for other types
      final Map<String, String> arguments =
          GetText(ByValueKey('column'), timeout: const Duration(seconds: 1)).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);
      expect(response['isError'], true);
      expect(response['response'], contains('is currently not supported by getText'));
    });
  });

  testWidgets('descendant finder', (WidgetTester tester) async {
    await silenceDriverLogger(() async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
      );

      Future<String?> getDescendantText({String? of, bool matchRoot = false}) async {
        final Map<String, String> arguments =
            GetText(
              Descendant(of: ByValueKey(of), matching: ByValueKey('text2'), matchRoot: matchRoot),
              timeout: const Duration(seconds: 1),
            ).serialize();
        final Map<String, dynamic> result = await driverExtension.call(arguments);
        if (result['isError'] as bool) {
          return null;
        }
        return GetTextResult.fromJson(result['response'] as Map<String, dynamic>).text;
      }

      await tester.pumpWidget(
        const MaterialApp(
          home: Column(
            key: ValueKey<String>('column'),
            children: <Widget>[
              Text('Hello1', key: ValueKey<String>('text1')),
              Text('Hello2', key: ValueKey<String>('text2')),
              Text('Hello3', key: ValueKey<String>('text3')),
            ],
          ),
        ),
      );

      expect(await getDescendantText(of: 'column'), 'Hello2');
      expect(await getDescendantText(of: 'column', matchRoot: true), 'Hello2');
      expect(await getDescendantText(of: 'text2', matchRoot: true), 'Hello2');

      // Find nothing
      Future<String?> result = getDescendantText(of: 'text1', matchRoot: true);
      await tester.pump(const Duration(seconds: 2));
      expect(await result, null);

      result = getDescendantText(of: 'text2');
      await tester.pump(const Duration(seconds: 2));
      expect(await result, null);
    });
  });

  testWidgets('descendant finder firstMatchOnly', (WidgetTester tester) async {
    await silenceDriverLogger(() async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
      );

      Future<String?> getDescendantText() async {
        final Map<String, String> arguments =
            GetText(
              Descendant(
                of: ByValueKey('column'),
                matching: const ByType('Text'),
                firstMatchOnly: true,
              ),
              timeout: const Duration(seconds: 1),
            ).serialize();
        final Map<String, dynamic> result = await driverExtension.call(arguments);
        if (result['isError'] as bool) {
          return null;
        }
        return GetTextResult.fromJson(result['response'] as Map<String, dynamic>).text;
      }

      await tester.pumpWidget(
        const MaterialApp(
          home: Column(
            key: ValueKey<String>('column'),
            children: <Widget>[
              Text('Hello1', key: ValueKey<String>('text1')),
              Text('Hello2', key: ValueKey<String>('text2')),
              Text('Hello3', key: ValueKey<String>('text3')),
            ],
          ),
        ),
      );

      expect(await getDescendantText(), 'Hello1');
    });
  });

  testWidgets('ancestor finder', (WidgetTester tester) async {
    await silenceDriverLogger(() async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
      );

      Future<Offset?> getAncestorTopLeft({
        String? of,
        String? matching,
        bool matchRoot = false,
      }) async {
        final Map<String, String> arguments =
            GetOffset(
              Ancestor(of: ByValueKey(of), matching: ByValueKey(matching), matchRoot: matchRoot),
              OffsetType.topLeft,
              timeout: const Duration(seconds: 1),
            ).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        if (response['isError'] as bool) {
          return null;
        }
        final GetOffsetResult result = GetOffsetResult.fromJson(
          response['response'] as Map<String, dynamic>,
        );
        return Offset(result.dx, result.dy);
      }

      await tester.pumpWidget(
        const MaterialApp(
          home: Center(
            child: SizedBox(
              key: ValueKey<String>('parent'),
              height: 100,
              width: 100,
              child: Center(
                child: Row(
                  children: <Widget>[
                    SizedBox(key: ValueKey<String>('leftchild'), width: 25, height: 25),
                    SizedBox(key: ValueKey<String>('rightchild'), width: 25, height: 25),
                  ],
                ),
              ),
            ),
          ),
        ),
      );

      expect(
        await getAncestorTopLeft(of: 'leftchild', matching: 'parent'),
        const Offset((800 - 100) / 2, (600 - 100) / 2),
      );
      expect(
        await getAncestorTopLeft(of: 'leftchild', matching: 'parent', matchRoot: true),
        const Offset((800 - 100) / 2, (600 - 100) / 2),
      );
      expect(
        await getAncestorTopLeft(of: 'parent', matching: 'parent', matchRoot: true),
        const Offset((800 - 100) / 2, (600 - 100) / 2),
      );

      // Find nothing
      Future<Offset?> result = getAncestorTopLeft(of: 'leftchild', matching: 'leftchild');
      await tester.pump(const Duration(seconds: 2));
      expect(await result, null);

      result = getAncestorTopLeft(of: 'leftchild', matching: 'rightchild');
      await tester.pump(const Duration(seconds: 2));
      expect(await result, null);
    });
  });

  testWidgets('ancestor finder firstMatchOnly', (WidgetTester tester) async {
    await silenceDriverLogger(() async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
      );

      Future<Offset?> getAncestorTopLeft() async {
        final Map<String, String> arguments =
            GetOffset(
              Ancestor(
                of: ByValueKey('leaf'),
                matching: const ByType('SizedBox'),
                firstMatchOnly: true,
              ),
              OffsetType.topLeft,
              timeout: const Duration(seconds: 1),
            ).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        if (response['isError'] as bool) {
          return null;
        }
        final GetOffsetResult result = GetOffsetResult.fromJson(
          response['response'] as Map<String, dynamic>,
        );
        return Offset(result.dx, result.dy);
      }

      await tester.pumpWidget(
        const MaterialApp(
          home: Center(
            child: SizedBox(
              height: 200,
              width: 200,
              child: Center(
                child: SizedBox(
                  height: 100,
                  width: 100,
                  child: Center(
                    child: SizedBox(key: ValueKey<String>('leaf'), height: 50, width: 50),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      expect(await getAncestorTopLeft(), const Offset((800 - 100) / 2, (600 - 100) / 2));
    });
  });

  testWidgets('GetDiagnosticsTree', (WidgetTester tester) async {
    final FlutterDriverExtension driverExtension = FlutterDriverExtension(
      (String? arg) async => '',
      true,
      true,
    );

    Future<Map<String, dynamic>> getDiagnosticsTree(
      DiagnosticsType type,
      SerializableFinder finder, {
      int depth = 0,
      bool properties = true,
    }) async {
      final Map<String, String> arguments =
          GetDiagnosticsTree(
            finder,
            type,
            subtreeDepth: depth,
            includeProperties: properties,
          ).serialize();
      final Map<String, dynamic> response = await driverExtension.call(arguments);
      final DiagnosticsTreeResult result = DiagnosticsTreeResult(
        response['response'] as Map<String, dynamic>,
      );
      return result.json;
    }

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Text('Hello World', key: ValueKey<String>('Text'))),
      ),
    );

    // Widget
    Map<String, dynamic> result = await getDiagnosticsTree(
      DiagnosticsType.widget,
      ByValueKey('Text'),
    );
    expect(result['children'], isNull); // depth: 0
    expect(result['widgetRuntimeType'], 'Text');

    List<Map<String, dynamic>> properties =
        (result['properties']! as List<Object>).cast<Map<String, dynamic>>();
    Map<String, dynamic> stringProperty = properties.singleWhere(
      (Map<String, dynamic> property) => property['name'] == 'data',
    );
    expect(stringProperty['description'], '"Hello World"');
    expect(stringProperty['propertyType'], 'String');

    result = await getDiagnosticsTree(
      DiagnosticsType.widget,
      ByValueKey('Text'),
      properties: false,
    );
    expect(result['widgetRuntimeType'], 'Text');
    expect(result['properties'], isNull); // properties: false

    result = await getDiagnosticsTree(DiagnosticsType.widget, ByValueKey('Text'), depth: 1);
    List<Map<String, dynamic>> children =
        (result['children']! as List<Object>).cast<Map<String, dynamic>>();
    expect(children.single['children'], isNull);

    result = await getDiagnosticsTree(DiagnosticsType.widget, ByValueKey('Text'), depth: 100);
    children = (result['children']! as List<Object>).cast<Map<String, dynamic>>();
    expect(children.single['children'], isEmpty);

    // RenderObject
    result = await getDiagnosticsTree(DiagnosticsType.renderObject, ByValueKey('Text'));
    expect(result['children'], isNull); // depth: 0
    expect(result['properties'], isNotNull);
    expect(result['description'], startsWith('RenderParagraph'));

    result = await getDiagnosticsTree(
      DiagnosticsType.renderObject,
      ByValueKey('Text'),
      properties: false,
    );
    expect(result['properties'], isNull); // properties: false
    expect(result['description'], startsWith('RenderParagraph'));

    result = await getDiagnosticsTree(DiagnosticsType.renderObject, ByValueKey('Text'), depth: 1);
    children = (result['children']! as List<Object>).cast<Map<String, dynamic>>();
    final Map<String, dynamic> textSpan = children.single;
    expect(textSpan['description'], 'TextSpan');
    properties = (textSpan['properties']! as List<Object>).cast<Map<String, dynamic>>();
    stringProperty = properties.singleWhere(
      (Map<String, dynamic> property) => property['name'] == 'text',
    );
    expect(stringProperty['description'], '"Hello World"');
    expect(stringProperty['propertyType'], 'String');
    expect(children.single['children'], isNull);

    result = await getDiagnosticsTree(DiagnosticsType.renderObject, ByValueKey('Text'), depth: 100);
    children = (result['children']! as List<Object>).cast<Map<String, dynamic>>();
    expect(children.single['children'], isEmpty);
  });

  group('enableTextEntryEmulation', () {
    late FlutterDriverExtension driverExtension;

    Future<Map<String, dynamic>> enterText() async {
      final Map<String, String> arguments = const EnterText('foo').serialize();
      final Map<String, dynamic> result = await driverExtension.call(arguments);
      return result;
    }

    const Widget testWidget = MaterialApp(
      home: Material(
        child: Center(child: TextField(key: ValueKey<String>('foo'), autofocus: true)),
      ),
    );

    testWidgets('enableTextEntryEmulation false', (WidgetTester tester) async {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, false);

      await tester.pumpWidget(testWidget);

      final Map<String, dynamic> enterTextResult = await enterText();
      expect(enterTextResult['isError'], isTrue);
    });

    testWidgets('enableTextEntryEmulation true', (WidgetTester tester) async {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, true);

      await tester.pumpWidget(testWidget);

      final Map<String, dynamic> enterTextResult = await enterText();
      expect(enterTextResult['isError'], isFalse);
    });
  });

  group('extension finders', () {
    final Widget debugTree = Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: Column(
          key: const ValueKey<String>('Column'),
          children: <Widget>[
            const Text('Foo', key: ValueKey<String>('Text1')),
            const Text('Bar', key: ValueKey<String>('Text2')),
            TextButton(
              key: const ValueKey<String>('Button'),
              onPressed: () {},
              child: const Text('Whatever'),
            ),
          ],
        ),
      ),
    );

    testWidgets('unknown extension finder', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        finders: <FinderExtension>[],
      );

      Future<Map<String, dynamic>> getText(SerializableFinder finder) async {
        final Map<String, String> arguments =
            GetText(finder, timeout: const Duration(seconds: 1)).serialize();
        return driverExtension.call(arguments);
      }

      await tester.pumpWidget(debugTree);

      final Map<String, dynamic> result = await getText(StubFinder('Text1'));
      expect(result['isError'], true);
      expect(result['response'] is String, true);
      expect(result['response'] as String?, contains('Unsupported search specification type Stub'));
    });

    testWidgets('simple extension finder', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        finders: <FinderExtension>[StubFinderExtension()],
      );

      Future<GetTextResult> getText(SerializableFinder finder) async {
        final Map<String, String> arguments =
            GetText(finder, timeout: const Duration(seconds: 1)).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        return GetTextResult.fromJson(response['response'] as Map<String, dynamic>);
      }

      await tester.pumpWidget(debugTree);

      final GetTextResult result = await getText(StubFinder('Text1'));
      expect(result.text, 'Foo');
    });

    testWidgets('complex extension finder', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        finders: <FinderExtension>[StubFinderExtension()],
      );

      Future<GetTextResult> getText(SerializableFinder finder) async {
        final Map<String, String> arguments =
            GetText(finder, timeout: const Duration(seconds: 1)).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        return GetTextResult.fromJson(response['response'] as Map<String, dynamic>);
      }

      await tester.pumpWidget(debugTree);

      final GetTextResult result = await getText(
        Descendant(of: StubFinder('Column'), matching: StubFinder('Text1')),
      );
      expect(result.text, 'Foo');
    });

    testWidgets('extension finder with command', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        finders: <FinderExtension>[StubFinderExtension()],
      );

      Future<Map<String, dynamic>> tap(SerializableFinder finder) async {
        final Map<String, String> arguments =
            Tap(finder, timeout: const Duration(seconds: 1)).serialize();
        return driverExtension.call(arguments);
      }

      await tester.pumpWidget(debugTree);

      final Map<String, dynamic> result = await tap(StubFinder('Button'));
      expect(result['isError'], false);
    });
  });

  group('extension commands', () {
    int invokes = 0;
    void stubCallback() => invokes++;

    final Widget debugTree = Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: Column(
          children: <Widget>[
            TextButton(
              key: const ValueKey<String>('Button'),
              onPressed: stubCallback,
              child: const Text('Whatever'),
            ),
          ],
        ),
      ),
    );

    setUp(() {
      invokes = 0;
    });

    testWidgets('unknown extension command', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        commands: <CommandExtension>[],
      );

      Future<Map<String, dynamic>> invokeCommand(SerializableFinder finder, int times) async {
        final Map<String, String> arguments = StubNestedCommand(finder, times).serialize();
        return driverExtension.call(arguments);
      }

      await tester.pumpWidget(debugTree);

      final Map<String, dynamic> result = await invokeCommand(ByValueKey('Button'), 10);
      expect(result['isError'], true);
      expect(result['response'] is String, true);
      expect(result['response'] as String?, contains('Unsupported command kind StubNestedCommand'));
    });

    testWidgets('nested command', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        commands: <CommandExtension>[StubNestedCommandExtension()],
      );

      Future<StubCommandResult> invokeCommand(SerializableFinder finder, int times) async {
        await driverExtension.call(
          const SetFrameSync(false).serialize(),
        ); // disable frame sync for test to avoid lock
        final Map<String, String> arguments =
            StubNestedCommand(finder, times, timeout: const Duration(seconds: 1)).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        final Map<String, dynamic> commandResponse = response['response'] as Map<String, dynamic>;
        return StubCommandResult(commandResponse['resultParam'] as String);
      }

      await tester.pumpWidget(debugTree);

      const int times = 10;
      final StubCommandResult result = await invokeCommand(ByValueKey('Button'), times);
      expect(result.resultParam, 'stub response');
      expect(invokes, times);
    });

    testWidgets('prober command', (WidgetTester tester) async {
      final FlutterDriverExtension driverExtension = FlutterDriverExtension(
        (String? arg) async => '',
        true,
        true,
        commands: <CommandExtension>[StubProberCommandExtension()],
      );

      Future<StubCommandResult> invokeCommand(SerializableFinder finder, int times) async {
        await driverExtension.call(
          const SetFrameSync(false).serialize(),
        ); // disable frame sync for test to avoid lock
        final Map<String, String> arguments =
            StubProberCommand(finder, times, timeout: const Duration(seconds: 1)).serialize();
        final Map<String, dynamic> response = await driverExtension.call(arguments);
        final Map<String, dynamic> commandResponse = response['response'] as Map<String, dynamic>;
        return StubCommandResult(commandResponse['resultParam'] as String);
      }

      await tester.pumpWidget(debugTree);

      const int times = 10;
      final StubCommandResult result = await invokeCommand(ByValueKey('Button'), times);
      expect(result.resultParam, 'stub response');
      expect(invokes, times);
    });
  });

  group('waitForTappable', () {
    late FlutterDriverExtension driverExtension;

    Future<Map<String, dynamic>> waitForTappable() async {
      final SerializableFinder finder = ByValueKey('widgetOne');
      final Map<String, String> arguments = WaitForTappable(finder).serialize();
      final Map<String, dynamic> result = await driverExtension.call(arguments);
      return result;
    }

    const Widget testWidget = MaterialApp(
      home: Material(
        child: Column(
          children: <Widget>[
            Text('Hello ', key: Key('widgetOne')),
            SizedBox.shrink(child: Text('World!', key: Key('widgetTwo'))),
          ],
        ),
      ),
    );

    testWidgets('returns true when widget is tappable', (WidgetTester tester) async {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, false);

      await tester.pumpWidget(testWidget);

      final Map<String, dynamic> waitForTappableResult = await waitForTappable();
      expect(waitForTappableResult['isError'], isFalse);
    });
  });

  group('waitUntilFrameSync', () {
    late FlutterDriverExtension driverExtension;
    Map<String, dynamic>? result;

    setUp(() {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, true);
      result = null;
    });

    testWidgets('returns immediately when frame is synced', (WidgetTester tester) async {
      driverExtension
          .call(const WaitForCondition(NoPendingFrame()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      await tester.idle();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waits until no transient callbacks', (WidgetTester tester) async {
      SchedulerBinding.instance.scheduleFrameCallback((_) {
        // Intentionally blank. We only care about existence of a callback.
      });

      driverExtension
          .call(const WaitForCondition(NoPendingFrame()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });

    testWidgets('waits until no pending scheduled frame', (WidgetTester tester) async {
      SchedulerBinding.instance.scheduleFrame();

      driverExtension
          .call(const WaitForCondition(NoPendingFrame()).serialize())
          .then<void>(
            expectAsync1((Map<String, dynamic> r) {
              result = r;
            }),
          );

      // Nothing should happen until the next frame.
      await tester.idle();
      expect(result, isNull);

      // NOW we should receive the result.
      await tester.pump();
      expect(result, <String, dynamic>{'isError': false, 'response': <String, dynamic>{}});
    });
  });

  group('sendTextInputAction', () {
    late FlutterDriverExtension driverExtension;

    Future<void> sendAction(TextInputAction action) async {
      final Map<String, String> arguments = SendTextInputAction(action).serialize();
      await driverExtension.call(arguments);
    }

    MaterialApp testWidget(TextEditingController controller) => MaterialApp(
      home: Material(
        child: Center(
          child: TextField(
            key: const ValueKey<String>('foo'),
            autofocus: true,
            controller: controller,
            onSubmitted: (_) {
              controller.value = const TextEditingValue(text: 'bar');
            },
          ),
        ),
      ),
    );

    testWidgets('press done trigger onSubmitted and change value', (WidgetTester tester) async {
      driverExtension = FlutterDriverExtension((String? arg) async => '', true, true);

      final TextEditingController controller = TextEditingController(text: 'foo');
      await tester.pumpWidget(testWidget(controller));

      expect(controller.value.text, 'foo');
      await sendAction(TextInputAction.done);
      expectSync(controller.value.text, 'bar');
    });
  });
}
