// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/daemon.dart';

import '../src/common.dart';

class FakeDaemonStreams implements DaemonStreams {
  final StreamController<DaemonMessage> inputs = StreamController<DaemonMessage>();
  final StreamController<DaemonMessage> outputs = StreamController<DaemonMessage>();

  @override
  Stream<DaemonMessage> get inputStream {
    return inputs.stream;
  }

  @override
  void send(Map<String, dynamic> message, [List<int>? binary]) {
    outputs.add(DaemonMessage(message, binary != null ? Stream<List<int>>.value(binary) : null));
  }

  @override
  Future<void> dispose() async {
    await inputs.close();
    // In some tests, outputs have no listeners. We don't wait for outputs to close.
    unawaited(outputs.close());
  }
}

void main() {
  late BufferLogger bufferLogger;
  late FakeDaemonStreams daemonStreams;
  late DaemonConnection daemonConnection;
  setUp(() {
    bufferLogger = BufferLogger.test();
    daemonStreams = FakeDaemonStreams();
    daemonConnection = DaemonConnection(daemonStreams: daemonStreams, logger: bufferLogger);
  });

  tearDown(() async {
    await daemonConnection.dispose();
  });

  group('DaemonConnection receiving end', () {
    testWithoutContext('redirects input to incoming commands', () async {
      final Map<String, dynamic> commandToSend = <String, dynamic>{
        'id': 0,
        'method': 'some_method',
      };
      daemonStreams.inputs.add(DaemonMessage(commandToSend));

      final DaemonMessage commandReceived = await daemonConnection.incomingCommands.first;
      await daemonStreams.dispose();

      expect(commandReceived.data, commandToSend);
    });

    testWithoutContext('listenToEvent can receive the right events', () async {
      final Future<List<DaemonEventData>> events =
          daemonConnection.listenToEvent('event1').toList();

      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'event': 'event1', 'params': '1'}));
      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'event': 'event2', 'params': '2'}));
      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'event': 'event1', 'params': null}));
      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'event': 'event1', 'params': 3}));

      await pumpEventQueue();
      await daemonConnection.dispose();

      expect((await events).map((DaemonEventData event) => event.data).toList(), <dynamic>[
        '1',
        null,
        3,
      ]);
    });
  });

  group('DaemonConnection sending end', () {
    testWithoutContext('sending requests', () async {
      unawaited(daemonConnection.sendRequest('some_method', 'param'));
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'some_method');
      expect(message.data['params'], 'param');
    });

    testWithoutContext('sending requests without param', () async {
      unawaited(daemonConnection.sendRequest('some_method'));
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'some_method');
      expect(message.data['params'], isNull);
    });

    testWithoutContext('sending response', () async {
      daemonConnection.sendResponse('1', 'some_data');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], '1');
      expect(message.data['method'], isNull);
      expect(message.data['error'], isNull);
      expect(message.data['result'], 'some_data');
    });

    testWithoutContext('sending response without data', () async {
      daemonConnection.sendResponse('1');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], '1');
      expect(message.data['method'], isNull);
      expect(message.data['error'], isNull);
      expect(message.data['result'], isNull);
    });

    testWithoutContext('sending error response', () async {
      daemonConnection.sendErrorResponse('1', 'error', StackTrace.fromString('stack trace'));
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], '1');
      expect(message.data['method'], isNull);
      expect(message.data['error'], 'error');
      expect(message.data['trace'], 'stack trace');
    });

    testWithoutContext('sending events', () async {
      daemonConnection.sendEvent('some_event', '123');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], isNull);
      expect(message.data['event'], 'some_event');
      expect(message.data['params'], '123');
    });

    testWithoutContext('sending events without params', () async {
      daemonConnection.sendEvent('some_event');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;
      expect(message.data['id'], isNull);
      expect(message.data['event'], 'some_event');
      expect(message.data['params'], isNull);
    });
  });

  group('DaemonConnection request and response', () {
    testWithoutContext('receiving response from requests', () async {
      final Future<dynamic> requestFuture = daemonConnection.sendRequest('some_method', 'param');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'some_method');
      expect(message.data['params'], 'param');

      final String id = message.data['id']! as String;
      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'id': id, 'result': '123'}));
      expect(await requestFuture, '123');
    });

    testWithoutContext('receiving response from requests without result', () async {
      final Future<dynamic> requestFuture = daemonConnection.sendRequest('some_method', 'param');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'some_method');
      expect(message.data['params'], 'param');

      final String id = message.data['id']! as String;
      daemonStreams.inputs.add(DaemonMessage(<String, dynamic>{'id': id}));
      expect(await requestFuture, null);
    });

    testWithoutContext('receiving error response from requests without result', () async {
      final Future<dynamic> requestFuture = daemonConnection.sendRequest('some_method', 'param');
      final DaemonMessage message = await daemonStreams.outputs.stream.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'some_method');
      expect(message.data['params'], 'param');

      final String id = message.data['id']! as String;
      daemonStreams.inputs.add(
        DaemonMessage(<String, dynamic>{'id': id, 'error': 'some_error', 'trace': 'stack trace'}),
      );

      Object? gotError;
      StackTrace? gotStackTrace;
      try {
        await requestFuture;
      } on Object catch (error, stackTrace) {
        gotError = error;
        gotStackTrace = stackTrace;
      }

      expect(gotError, 'some_error');
      expect(gotStackTrace.toString(), 'stack trace');
    });
  });

  group('DaemonInputStreamConverter', () {
    Map<String, Object?> testCommand(int id, [int? binarySize]) => <String, Object?>{
      'id': id,
      'method': 'test',
      if (binarySize != null) '_binaryLength': binarySize,
    };
    List<int> testCommandBinary(int id, [int? binarySize]) =>
        utf8.encode('[${json.encode(testCommand(id, binarySize))}]\n');

    testWithoutContext('can parse a single message', () async {
      final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
        testCommandBinary(10),
      ]);
      final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
      final Stream<DaemonMessage> outputStream = converter.convertedStream;
      final List<DaemonMessage> outputs = await outputStream.toList();
      expect(outputs, hasLength(1));
      expect(outputs[0].data, testCommand(10));
      expect(outputs[0].binary, null);
    });

    testWithoutContext('can parse multiple messages', () async {
      final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
        testCommandBinary(10),
        testCommandBinary(20),
      ]);
      final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
      final Stream<DaemonMessage> outputStream = converter.convertedStream;
      final List<DaemonMessage> outputs = await outputStream.toList();
      expect(outputs, hasLength(2));
      expect(outputs[0].data, testCommand(10));
      expect(outputs[0].binary, null);
      expect(outputs[1].data, testCommand(20));
      expect(outputs[1].binary, null);
    });

    testWithoutContext(
      'can parse multiple messages while ignoring non json data in between',
      () async {
        final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
          testCommandBinary(10),
          utf8.encode('This is not a json data...\n'),
          testCommandBinary(20),
        ]);
        final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
        final Stream<DaemonMessage> outputStream = converter.convertedStream;
        final List<DaemonMessage> outputs = await outputStream.toList();
        expect(outputs, hasLength(2));
        expect(outputs[0].data, testCommand(10));
        expect(outputs[0].binary, null);
        expect(outputs[1].data, testCommand(20));
        expect(outputs[1].binary, null);
      },
    );

    testWithoutContext(
      'can parse multiple messages even when they are split in multiple packets',
      () async {
        final List<int> binary1 = testCommandBinary(10);
        final List<int> binary2 = testCommandBinary(20);
        final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
          binary1.sublist(0, 5),
          binary1.sublist(5, 15),
          binary1.sublist(15) + binary2.sublist(0, 13),
          binary2.sublist(13),
        ]);
        final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
        final Stream<DaemonMessage> outputStream = converter.convertedStream;
        final List<DaemonMessage> outputs = await outputStream.toList();
        expect(outputs, hasLength(2));
        expect(outputs[0].data, testCommand(10));
        expect(outputs[0].binary, null);
        expect(outputs[1].data, testCommand(20));
        expect(outputs[1].binary, null);
      },
    );

    testWithoutContext(
      'can parse multiple messages even when they are combined in a single packet',
      () async {
        final List<int> binary1 = testCommandBinary(10);
        final List<int> binary2 = testCommandBinary(20);
        final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
          binary1 + binary2,
        ]);
        final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
        final Stream<DaemonMessage> outputStream = converter.convertedStream;
        final List<DaemonMessage> outputs = await outputStream.toList();
        expect(outputs, hasLength(2));
        expect(outputs[0].data, testCommand(10));
        expect(outputs[0].binary, null);
        expect(outputs[1].data, testCommand(20));
        expect(outputs[1].binary, null);
      },
    );

    testWithoutContext('can parse a single message with binary stream', () async {
      final List<int> binary = <int>[1, 2, 3, 4, 5];
      final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
        testCommandBinary(10, binary.length),
        binary,
      ]);
      final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
      final Stream<DaemonMessage> outputStream = converter.convertedStream;
      final List<_DaemonMessageAndBinary> allOutputs = await _readAllBinaries(outputStream);
      expect(allOutputs, hasLength(1));
      expect(allOutputs[0].message.data, testCommand(10, binary.length));
      expect(allOutputs[0].binary, binary);
    });

    testWithoutContext(
      'can parse a single message with binary stream when messages are combined in a single packet',
      () async {
        final List<int> binary = <int>[1, 2, 3, 4, 5];
        final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
          testCommandBinary(10, binary.length) + binary,
        ]);
        final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
        final Stream<DaemonMessage> outputStream = converter.convertedStream;
        final List<_DaemonMessageAndBinary> allOutputs = await _readAllBinaries(outputStream);
        expect(allOutputs, hasLength(1));
        expect(allOutputs[0].message.data, testCommand(10, binary.length));
        expect(allOutputs[0].binary, binary);
      },
    );

    testWithoutContext('can parse multiple messages with binary stream', () async {
      final List<int> binary1 = <int>[1, 2, 3, 4, 5];
      final List<int> binary2 = <int>[6, 7, 8, 9, 10, 11, 12];
      final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
        testCommandBinary(10, binary1.length),
        binary1,
        testCommandBinary(20, binary2.length),
        binary2,
      ]);
      final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
      final Stream<DaemonMessage> outputStream = converter.convertedStream;
      final List<_DaemonMessageAndBinary> allOutputs = await _readAllBinaries(outputStream);
      expect(allOutputs, hasLength(2));
      expect(allOutputs[0].message.data, testCommand(10, binary1.length));
      expect(allOutputs[0].binary, binary1);
      expect(allOutputs[1].message.data, testCommand(20, binary2.length));
      expect(allOutputs[1].binary, binary2);
    });

    testWithoutContext(
      'can parse multiple messages with binary stream when messages are split',
      () async {
        final List<int> binary1 = <int>[1, 2, 3, 4, 5];
        final List<int> message1 = testCommandBinary(10, binary1.length);
        final List<int> binary2 = <int>[6, 7, 8, 9, 10, 11, 12];
        final List<int> message2 = testCommandBinary(20, binary2.length);
        final Stream<List<int>> inputStream = Stream<List<int>>.fromIterable(<List<int>>[
          message1.sublist(0, 10),
          message1.sublist(10) + binary1 + message2.sublist(0, 5),
          message2.sublist(5) + binary2.sublist(0, 3),
          binary2.sublist(3, 5),
          binary2.sublist(5),
        ]);
        final DaemonInputStreamConverter converter = DaemonInputStreamConverter(inputStream);
        final Stream<DaemonMessage> outputStream = converter.convertedStream;
        final List<_DaemonMessageAndBinary> allOutputs = await _readAllBinaries(outputStream);
        expect(allOutputs, hasLength(2));
        expect(allOutputs[0].message.data, testCommand(10, binary1.length));
        expect(allOutputs[0].binary, binary1);
        expect(allOutputs[1].message.data, testCommand(20, binary2.length));
        expect(allOutputs[1].binary, binary2);
      },
    );
  });

  group('DaemonStreams', () {
    final Map<String, Object?> testCommand = <String, Object?>{'id': 100, 'method': 'test'};
    late StreamController<List<int>> inputStream;
    late StreamController<List<int>> outputStream;
    late DaemonStreams daemonStreams;
    setUp(() {
      inputStream = StreamController<List<int>>();
      outputStream = StreamController<List<int>>();
      daemonStreams = DaemonStreams(inputStream.stream, outputStream.sink, logger: bufferLogger);
    });

    testWithoutContext('parses the message received on the stream', () async {
      inputStream.add(utf8.encode('[${jsonEncode(testCommand)}]\n'));
      final DaemonMessage command = await daemonStreams.inputStream.first;
      expect(command.data, testCommand);
      expect(command.binary, null);
    });

    testWithoutContext('sends the encoded message through the sink', () async {
      daemonStreams.send(testCommand);
      final List<int> commands = await outputStream.stream.first;
      expect(commands, utf8.encode('[${jsonEncode(testCommand)}]\n'));
    });

    testWithoutContext('dispose closes the sink', () async {
      await daemonStreams.dispose();
      expect(outputStream.isClosed, true);
    });

    testWithoutContext('handles sending to a closed sink', () async {
      // Unless the stream is listened to, the call to .close() will never
      // complete
      outputStream.stream.listen((List<int> _) {});
      await outputStream.sink.close();
      daemonStreams.send(testCommand);
      expect(
        bufferLogger.errorText,
        contains(
          'Failed to write daemon command response: Bad state: Cannot add event after closing',
        ),
      );
    });
  });
}

class _DaemonMessageAndBinary {
  _DaemonMessageAndBinary(this.message, this.binary);
  final DaemonMessage message;
  final List<int>? binary;
}

Future<List<_DaemonMessageAndBinary>> _readAllBinaries(Stream<DaemonMessage> inputStream) async {
  final StreamIterator<DaemonMessage> iterator = StreamIterator<DaemonMessage>(inputStream);
  final List<_DaemonMessageAndBinary> outputs = <_DaemonMessageAndBinary>[];
  while (await iterator.moveNext()) {
    List<int>? binary;
    if (iterator.current.binary != null) {
      binary = await iterator.current.binary!.reduce((List<int> a, List<int> b) => a + b);
    }
    outputs.add(_DaemonMessageAndBinary(iterator.current, binary));
  }
  return outputs;
}
