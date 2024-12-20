// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:isolate';

import 'package:flutter/services.dart';

import 'pair.dart';
import 'test_step.dart';

class ExtendedStandardMessageCodec extends StandardMessageCodec {
  const ExtendedStandardMessageCodec();

  static const int _dateTime = 128;
  static const int _pair = 129;

  @override
  void writeValue(WriteBuffer buffer, dynamic value) {
    if (value is DateTime) {
      buffer.putUint8(_dateTime);
      buffer.putInt64(value.millisecondsSinceEpoch);
    } else if (value is Pair) {
      buffer.putUint8(_pair);
      writeValue(buffer, value.left);
      writeValue(buffer, value.right);
    } else {
      super.writeValue(buffer, value);
    }
  }

  @override
  dynamic readValueOfType(int type, ReadBuffer buffer) {
    return switch (type) {
      _dateTime => DateTime.fromMillisecondsSinceEpoch(buffer.getInt64()),
      _pair => Pair(readValue(buffer), readValue(buffer)),
      _ => super.readValueOfType(type, buffer),
    };
  }
}

Future<TestStepResult> basicBinaryHandshake(ByteData? message) async {
  const BasicMessageChannel<ByteData?> channel = BasicMessageChannel<ByteData?>(
    'binary-msg',
    BinaryCodec(),
  );
  return _basicMessageHandshake<ByteData?>('Binary >${toString(message)}<', channel, message);
}

Future<TestStepResult> basicStringHandshake(String? message) async {
  const BasicMessageChannel<String?> channel = BasicMessageChannel<String?>(
    'string-msg',
    StringCodec(),
  );
  return _basicMessageHandshake<String?>('String >$message<', channel, message);
}

Future<TestStepResult> basicJsonHandshake(dynamic message) async {
  const BasicMessageChannel<dynamic> channel = BasicMessageChannel<dynamic>(
    'json-msg',
    JSONMessageCodec(),
  );
  return _basicMessageHandshake<dynamic>('JSON >$message<', channel, message);
}

Future<TestStepResult> basicStandardHandshake(dynamic message) async {
  const BasicMessageChannel<dynamic> channel = BasicMessageChannel<dynamic>(
    'std-msg',
    ExtendedStandardMessageCodec(),
  );
  return _basicMessageHandshake<dynamic>('Standard >${toString(message)}<', channel, message);
}

Future<void> _basicBackgroundStandardEchoMain(List<Object> args) async {
  final SendPort sendPort = args[2] as SendPort;
  final Object message = args[1];
  final String name = 'Background Echo >${toString(message)}<';
  const String description = 'Uses a platform channel from a background isolate.';
  try {
    BackgroundIsolateBinaryMessenger.ensureInitialized(args[0] as RootIsolateToken);
    const BasicMessageChannel<dynamic> channel = BasicMessageChannel<dynamic>(
      'std-echo',
      ExtendedStandardMessageCodec(),
    );
    final Object response = await channel.send(message) as Object;

    final TestStatus testStatus =
        TestStepResult.deepEquals(message, response) ? TestStatus.ok : TestStatus.failed;
    sendPort.send(TestStepResult(name, description, testStatus));
  } catch (ex) {
    sendPort.send(TestStepResult(name, description, TestStatus.failed, error: ex.toString()));
  }
}

Future<TestStepResult> basicBackgroundStandardEcho(Object message) async {
  final ReceivePort receivePort = ReceivePort();
  Isolate.spawn(_basicBackgroundStandardEchoMain, <Object>[
    ServicesBinding.rootIsolateToken!,
    message,
    receivePort.sendPort,
  ]);
  return await receivePort.first as TestStepResult;
}

Future<TestStepResult> basicBinaryMessageToUnknownChannel() async {
  const BasicMessageChannel<ByteData?> channel = BasicMessageChannel<ByteData?>(
    'binary-unknown',
    BinaryCodec(),
  );
  return _basicMessageToUnknownChannel<ByteData>('Binary', channel);
}

Future<TestStepResult> basicStringMessageToUnknownChannel() async {
  const BasicMessageChannel<String?> channel = BasicMessageChannel<String?>(
    'string-unknown',
    StringCodec(),
  );
  return _basicMessageToUnknownChannel<String>('String', channel);
}

Future<TestStepResult> basicJsonMessageToUnknownChannel() async {
  const BasicMessageChannel<dynamic> channel = BasicMessageChannel<dynamic>(
    'json-unknown',
    JSONMessageCodec(),
  );
  return _basicMessageToUnknownChannel<dynamic>('JSON', channel);
}

Future<TestStepResult> basicStandardMessageToUnknownChannel() async {
  const BasicMessageChannel<dynamic> channel = BasicMessageChannel<dynamic>(
    'std-unknown',
    ExtendedStandardMessageCodec(),
  );
  return _basicMessageToUnknownChannel<dynamic>('Standard', channel);
}

/// Sends the specified message to the platform, doing a
/// receive message/send reply/receive reply echo handshake initiated by the
/// platform, then expecting a reply echo to the original message.
///
/// Fails, if an error occurs, or if any message seen is not deeply equal to
/// the original message.
Future<TestStepResult> _basicMessageHandshake<T>(
  String description,
  BasicMessageChannel<T?> channel,
  T message,
) async {
  final List<dynamic> received = <dynamic>[];
  channel.setMessageHandler((T? message) async {
    received.add(message);
    return message;
  });
  dynamic messageEcho = nothing;
  dynamic error = nothing;
  try {
    messageEcho = await channel.send(message);
  } catch (e) {
    error = e;
  }
  return resultOfHandshake(
    'Basic message handshake',
    description,
    message,
    received,
    messageEcho,
    error,
  );
}

/// Sends a message on a channel that no one listens on.
Future<TestStepResult> _basicMessageToUnknownChannel<T>(
  String description,
  BasicMessageChannel<T?> channel,
) async {
  dynamic messageEcho = nothing;
  dynamic error = nothing;
  try {
    messageEcho = await channel.send(null);
  } catch (e) {
    error = e;
  }
  return resultOfHandshake(
    'Message on unknown channel',
    description,
    null,
    <dynamic>[null, null],
    messageEcho,
    error,
  );
}

String toString(dynamic message) {
  if (message is ByteData) {
    return message.buffer.asUint8List(message.offsetInBytes, message.lengthInBytes).toString();
  } else {
    return '$message';
  }
}
