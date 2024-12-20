// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/material.dart';

import 'pair.dart';

enum TestStatus { ok, pending, failed, complete }

typedef TestStep = Future<TestStepResult> Function();

const String nothing = '-';

/// Result of a test step checking a nested communication handshake
/// between the Flutter app and the platform:
///
/// - The Flutter app sends a message to the platform.
/// - The platform, on receipt, echos the message back to Flutter in a separate message.
/// - The Flutter app records the incoming message echo and replies.
/// - The platform, on receipt of reply, echos the reply back to Flutter in a separate message.
/// - The Flutter app records the incoming reply echo.
/// - The platform finally replies to the original message with another echo.
class TestStepResult {
  const TestStepResult(
    this.name,
    this.description,
    this.status, {
    this.messageSent = nothing,
    this.messageEcho = nothing,
    this.messageReceived = nothing,
    this.replyEcho = nothing,
    this.error = nothing,
  });

  factory TestStepResult.fromSnapshot(AsyncSnapshot<TestStepResult> snapshot) {
    return switch (snapshot.connectionState) {
      ConnectionState.none => const TestStepResult('Not started', nothing, TestStatus.ok),
      ConnectionState.waiting => const TestStepResult('Executing', nothing, TestStatus.pending),
      ConnectionState.done => snapshot.data ?? snapshot.error! as TestStepResult,
      ConnectionState.active => throw 'Unsupported state: ConnectionState.active',
    };
  }

  final String name;
  final String description;
  final TestStatus status;
  final dynamic messageSent;
  final dynamic messageEcho;
  final dynamic messageReceived;
  final dynamic replyEcho;
  final dynamic error;

  static const TextStyle bold = TextStyle(fontWeight: FontWeight.bold);
  static const TestStepResult complete = TestStepResult(
    'Test complete',
    nothing,
    TestStatus.complete,
  );

  Widget asWidget(BuildContext context) {
    return ListView(
      children: <Widget>[
        Text('Step: $name', style: bold),
        Text(description),
        const Text(' '),
        Text('Msg sent: ${_toString(messageSent)}'),
        Text('Msg rvcd: ${_toString(messageReceived)}'),
        Text('Reply echo: ${_toString(replyEcho)}'),
        Text('Msg echo: ${_toString(messageEcho)}'),
        Text('Error: ${_toString(error)}'),
        const Text(' '),
        Text(
          status.name,
          key: ValueKey<String>(status == TestStatus.pending ? 'nostatus' : 'status'),
          style: bold,
        ),
      ],
    );
  }

  static bool deepEquals(dynamic a, dynamic b) => _deepEquals(a, b);

  @override
  String toString() {
    return 'TestStepResult($status)';
  }
}

Future<TestStepResult> resultOfHandshake(
  String name,
  String description,
  dynamic message,
  List<dynamic> received,
  dynamic messageEcho,
  dynamic error,
) async {
  assert(message != nothing);
  while (received.length < 2) {
    received.add(nothing);
  }
  TestStatus status;
  if (!_deepEquals(messageEcho, message) ||
      received.length != 2 ||
      !_deepEquals(received[0], message) ||
      !_deepEquals(received[1], message)) {
    status = TestStatus.failed;
  } else if (error != nothing) {
    status = TestStatus.failed;
  } else {
    status = TestStatus.ok;
  }
  return TestStepResult(
    name,
    description,
    status,
    messageSent: message,
    messageEcho: messageEcho,
    messageReceived: received[0],
    replyEcho: received[1],
    error: error,
  );
}

String _toString(dynamic message) {
  if (message is ByteData) {
    return message.buffer.asUint8List(message.offsetInBytes, message.lengthInBytes).toString();
  } else {
    return '$message';
  }
}

bool _deepEquals(dynamic a, dynamic b) {
  if (a == b) {
    return true;
  }
  if (a is double && a.isNaN) {
    return b is double && b.isNaN;
  }
  if (a is ByteData) {
    return b is ByteData && _deepEqualsByteData(a, b);
  }
  if (a is List) {
    return b is List && _deepEqualsList(a, b);
  }
  if (a is Map) {
    return b is Map && _deepEqualsMap(a, b);
  }
  if (a is Pair) {
    return b is Pair && _deepEqualsPair(a, b);
  }
  return false;
}

bool _deepEqualsByteData(ByteData a, ByteData b) {
  return _deepEqualsList(
    a.buffer.asUint8List(a.offsetInBytes, a.lengthInBytes),
    b.buffer.asUint8List(b.offsetInBytes, b.lengthInBytes),
  );
}

bool _deepEqualsList(List<dynamic> a, List<dynamic> b) {
  if (a.length != b.length) {
    return false;
  }
  for (int i = 0; i < a.length; i++) {
    if (!_deepEquals(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

bool _deepEqualsMap(Map<dynamic, dynamic> a, Map<dynamic, dynamic> b) {
  if (a.length != b.length) {
    return false;
  }
  for (final dynamic key in a.keys) {
    if (!b.containsKey(key) || !_deepEquals(a[key], b[key])) {
      return false;
    }
  }
  return true;
}

bool _deepEqualsPair(Pair a, Pair b) {
  return _deepEquals(a.left, b.left) && _deepEquals(a.right, b.right);
}
