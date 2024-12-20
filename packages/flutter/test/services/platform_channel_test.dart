// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  group('BasicMessageChannel', () {
    const MessageCodec<String?> string = StringCodec();
    const BasicMessageChannel<String?> channel = BasicMessageChannel<String?>('ch', string);
    test('can send string message and get reply', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch',
        (ByteData? message) async =>
            string.encodeMessage('${string.decodeMessage(message)!} world'),
      );
      final String? reply = await channel.send('hello');
      expect(reply, equals('hello world'));
    });

    test('can receive string message and send reply', () async {
      channel.setMessageHandler((String? message) async => '${message!} world');
      String? reply;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch',
        const StringCodec().encodeMessage('hello'),
        (ByteData? replyBinary) {
          reply = string.decodeMessage(replyBinary);
        },
      );
      expect(reply, equals('hello world'));
    });
  });

  group('MethodChannel', () {
    const MessageCodec<dynamic> jsonMessage = JSONMessageCodec();
    const MethodCodec jsonMethod = JSONMethodCodec();

    const MethodChannel channel = MethodChannel('ch7', jsonMethod);
    const OptionalMethodChannel optionalMethodChannel = OptionalMethodChannel('ch8', jsonMethod);
    tearDown(() {
      channel.setMethodCallHandler(null);
      optionalMethodChannel.setMethodCallHandler(null);
    });

    test('can invoke method and get result', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          final Map<dynamic, dynamic> methodCall =
              jsonMessage.decodeMessage(message) as Map<dynamic, dynamic>;
          if (methodCall['method'] == 'sayHello') {
            return jsonMessage.encodeMessage(<dynamic>['${methodCall['args']} world']);
          } else {
            return jsonMessage.encodeMessage(<dynamic>['unknown', null, null]);
          }
        },
      );
      final String? result = await channel.invokeMethod('sayHello', 'hello');
      expect(result, equals('hello world'));
    });

    test('can invoke list method and get result', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          final Map<dynamic, dynamic> methodCall =
              jsonMessage.decodeMessage(message) as Map<dynamic, dynamic>;
          if (methodCall['method'] == 'sayHello') {
            return jsonMessage.encodeMessage(<dynamic>[
              <String>['${methodCall['args']}', 'world'],
            ]);
          } else {
            return jsonMessage.encodeMessage(<dynamic>['unknown', null, null]);
          }
        },
      );
      expect(channel.invokeMethod<List<String>>('sayHello', 'hello'), throwsA(isA<TypeError>()));
      expect(await channel.invokeListMethod<String>('sayHello', 'hello'), <String>[
        'hello',
        'world',
      ]);
    });

    test('can invoke list method and get null result', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          final Map<dynamic, dynamic> methodCall =
              jsonMessage.decodeMessage(message) as Map<dynamic, dynamic>;
          if (methodCall['method'] == 'sayHello') {
            return jsonMessage.encodeMessage(<dynamic>[null]);
          } else {
            return jsonMessage.encodeMessage(<dynamic>['unknown', null, null]);
          }
        },
      );
      expect(await channel.invokeListMethod<String>('sayHello', 'hello'), null);
    });

    test('can invoke map method and get result', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          final Map<dynamic, dynamic> methodCall =
              jsonMessage.decodeMessage(message) as Map<dynamic, dynamic>;
          if (methodCall['method'] == 'sayHello') {
            return jsonMessage.encodeMessage(<dynamic>[
              <String, String>{'${methodCall['args']}': 'world'},
            ]);
          } else {
            return jsonMessage.encodeMessage(<dynamic>['unknown', null, null]);
          }
        },
      );
      expect(
        channel.invokeMethod<Map<String, String>>('sayHello', 'hello'),
        throwsA(isA<TypeError>()),
      );
      expect(await channel.invokeMapMethod<String, String>('sayHello', 'hello'), <String, String>{
        'hello': 'world',
      });
    });

    test('can invoke map method and get null result', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          final Map<dynamic, dynamic> methodCall =
              jsonMessage.decodeMessage(message) as Map<dynamic, dynamic>;
          if (methodCall['method'] == 'sayHello') {
            return jsonMessage.encodeMessage(<dynamic>[null]);
          } else {
            return jsonMessage.encodeMessage(<dynamic>['unknown', null, null]);
          }
        },
      );
      expect(await channel.invokeMapMethod<String, String>('sayHello', 'hello'), null);
    });

    test('can invoke method and get error', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async {
          return jsonMessage.encodeMessage(<dynamic>[
            'bad',
            'Something happened',
            <String, dynamic>{'a': 42, 'b': 3.14},
          ]);
        },
      );
      expect(
        () => channel.invokeMethod<dynamic>('sayHello', 'hello'),
        throwsA(
          isA<PlatformException>()
              .having((PlatformException e) => e.code, 'code', equals('bad'))
              .having((PlatformException e) => e.message, 'message', equals('Something happened'))
              .having(
                (PlatformException e) => e.details,
                'details',
                equals(<String, dynamic>{'a': 42, 'b': 3.14}),
              ),
        ),
      );
    });

    test('can invoke unimplemented method', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch7',
        (ByteData? message) async => null,
      );
      expect(
        () => channel.invokeMethod<void>('sayHello', 'hello'),
        throwsA(
          isA<MissingPluginException>().having(
            (MissingPluginException e) => e.message,
            'message',
            allOf(contains('sayHello'), contains('ch7')),
          ),
        ),
      );
    });

    test('can invoke unimplemented method (optional)', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
        'ch8',
        (ByteData? message) async => null,
      );
      final String? result = await optionalMethodChannel.invokeMethod<String>('sayHello', 'hello');
      expect(result, isNull);
    });

    test('can handle method call with no registered plugin (setting before)', () async {
      channel.setMethodCallHandler(null);
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      await null; // just in case there's something async happening
      expect(envelope, isNull);
    });

    test('can handle method call with no registered plugin (setting after)', () async {
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      channel.setMethodCallHandler(null);
      await null; // just in case there's something async happening
      expect(envelope, isNull);
    });

    test('can handle method call of unimplemented method', () async {
      channel.setMethodCallHandler((MethodCall call) async {
        throw MissingPluginException();
      });
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      expect(envelope, isNull);
    });

    test('can handle method call with successful result', () async {
      channel.setMethodCallHandler((MethodCall call) async => '${call.arguments}, world');
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      expect(jsonMethod.decodeEnvelope(envelope!), equals('hello, world'));
    });

    test('can handle method call with expressive error result', () async {
      channel.setMethodCallHandler((MethodCall call) async {
        throw PlatformException(code: 'bad', message: 'sayHello failed');
      });
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      expect(
        () => jsonMethod.decodeEnvelope(envelope!),
        throwsA(
          isA<PlatformException>()
              .having((PlatformException e) => e.code, 'code', equals('bad'))
              .having((PlatformException e) => e.message, 'message', equals('sayHello failed')),
        ),
      );
    });

    test('can handle method call with other error result', () async {
      channel.setMethodCallHandler((MethodCall call) async {
        throw ArgumentError('bad');
      });
      final ByteData call = jsonMethod.encodeMethodCall(const MethodCall('sayHello', 'hello'));
      ByteData? envelope;
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'ch7',
        call,
        (ByteData? result) {
          envelope = result;
        },
      );
      expect(
        () => jsonMethod.decodeEnvelope(envelope!),
        throwsA(
          isA<PlatformException>()
              .having((PlatformException e) => e.code, 'code', equals('error'))
              .having(
                (PlatformException e) => e.message,
                'message',
                equals('Invalid argument(s): bad'),
              ),
        ),
      );
    });

    test('can check the mock handler', () async {
      Future<dynamic> handler(MethodCall call) => Future<dynamic>.value();

      const MethodChannel channel = MethodChannel('test_handler');
      expect(
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
          channel.name,
          null,
        ),
        true,
      );
      expect(
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
          channel.name,
          handler,
        ),
        false,
      );
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        channel,
        handler,
      );
      expect(
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
          channel.name,
          handler,
        ),
        true,
      );
    });
  });

  group('EventChannel', () {
    const MethodCodec jsonMethod = JSONMethodCodec();
    const EventChannel channel = EventChannel('ch', jsonMethod);

    test('can receive event stream', () async {
      bool canceled = false;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockStreamHandler(
        channel,
        MockStreamHandler.inline(
          onListen: (Object? arguments, MockStreamHandlerEventSink events) {
            events.success('${arguments}1');
            events.success('${arguments}2');
            events.endOfStream();
          },
          onCancel: (Object? arguments) {
            canceled = true;
          },
        ),
      );
      final List<Object?> events = await channel.receiveBroadcastStream('hello').toList();
      expect(events, orderedEquals(<String>['hello1', 'hello2']));
      await Future<void>.delayed(Duration.zero);
      expect(canceled, isTrue);
    });

    test('can receive error event', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockStreamHandler(
        channel,
        MockStreamHandler.inline(
          onListen: (Object? arguments, MockStreamHandlerEventSink events) {
            events.error(code: '404', message: 'Not Found.', details: arguments);
          },
        ),
      );
      final List<Object?> events = <Object?>[];
      final List<Object?> errors = <Object?>[];
      channel.receiveBroadcastStream('hello').listen(events.add, onError: errors.add);
      await Future<void>.delayed(Duration.zero);
      expect(events, isEmpty);
      expect(errors, hasLength(1));
      expect(errors[0], isA<PlatformException>());
      final PlatformException? error = errors[0] as PlatformException?;
      expect(error?.code, '404');
      expect(error?.message, 'Not Found.');
      expect(error?.details, 'hello');
    });
  });
}
