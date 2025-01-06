// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:isolate';

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:platform/platform.dart';

final Matcher throwsRemoteError = throwsA(isA<RemoteError>());

int test1(int value) {
  return value + 1;
}

int test2(int value) {
  throw 2;
}

int test3(int value) {
  Isolate.exit();
}

int test4(int value) {
  Isolate.current.kill();

  return value + 1;
}

int test5(int value) {
  Isolate.current.kill(priority: Isolate.immediate);

  return value + 1;
}

Future<int> test1Async(int value) async {
  return value + 1;
}

Future<int> test2Async(int value) async {
  throw 2;
}

Future<int> test3Async(int value) async {
  Isolate.exit();
}

Future<int> test4Async(int value) async {
  Isolate.current.kill();

  return value + 1;
}

Future<int> test5Async(int value) async {
  Isolate.current.kill(priority: Isolate.immediate);

  return value + 1;
}

Future<int> test1CallCompute(int value) {
  return compute(test1, value);
}

Future<int> test2CallCompute(int value) {
  return compute(test2, value);
}

Future<int> test3CallCompute(int value) {
  return compute(test3, value);
}

Future<int> test4CallCompute(int value) {
  return compute(test4, value);
}

Future<int> test5CallCompute(int value) {
  return compute(test5, value);
}

Future<void> expectFileSuccessfullyCompletes(String filename) async {
  // Run a Dart script that calls compute().
  // The Dart process will terminate only if the script exits cleanly with
  // all isolate ports closed.
  const FileSystem fs = LocalFileSystem();
  const Platform platform = LocalPlatform();
  final String flutterRoot = platform.environment['FLUTTER_ROOT']!;
  final String dartPath = fs.path.join(flutterRoot, 'bin', 'cache', 'dart-sdk', 'bin', 'dart');
  final String scriptPath = fs.path.join(
    flutterRoot,
    'packages',
    'flutter',
    'test',
    'foundation',
    filename,
  );

  // Enable asserts to also catch potentially invalid assertions.
  final ProcessResult result = await Process.run(dartPath, <String>[
    'run',
    '--enable-asserts',
    scriptPath,
  ]);
  expect(result.exitCode, 0);
}

class ComputeTestSubject {
  ComputeTestSubject(this.base, [this.additional]);

  final int base;
  final dynamic additional;

  int method(int x) {
    return base * x;
  }

  static int staticMethod(int square) {
    return square * square;
  }
}

Future<int> computeStaticMethod(int square) {
  return compute(ComputeTestSubject.staticMethod, square);
}

Future<int> computeClosure(int square) {
  return compute((_) => square * square, null);
}

Future<int> computeInvalidClosure(int square) {
  final ReceivePort r = ReceivePort();

  return compute((_) {
    r.sendPort.send('Computing!');

    return square * square;
  }, null);
}

Future<int> computeInstanceMethod(int square) {
  final ComputeTestSubject subject = ComputeTestSubject(square);
  return compute(subject.method, square);
}

Future<int> computeInvalidInstanceMethod(int square) {
  final ComputeTestSubject subject = ComputeTestSubject(square, ReceivePort());
  expect(subject.additional, isA<ReceivePort>());
  return compute(subject.method, square);
}

dynamic testInvalidResponse(int square) {
  final ReceivePort r = ReceivePort();
  try {
    return r;
  } finally {
    r.close();
  }
}

dynamic testInvalidError(int square) {
  final ReceivePort r = ReceivePort();
  try {
    throw r;
  } finally {
    r.close();
  }
}

String? testDebugName(void _) {
  return Isolate.current.debugName;
}

int? testReturnNull(void _) {
  return null;
}

void main() {
  test('compute()', () async {
    expect(await compute(test1, 0), 1);
    expect(compute(test2, 0), throwsA(2));
    expect(compute(test3, 0), throwsRemoteError);
    expect(await compute(test4, 0), 1);
    expect(compute(test5, 0), throwsRemoteError);

    expect(await compute(test1Async, 0), 1);
    expect(compute(test2Async, 0), throwsA(2));
    expect(compute(test3Async, 0), throwsRemoteError);
    expect(await compute(test4Async, 0), 1);
    expect(compute(test5Async, 0), throwsRemoteError);

    expect(await compute(test1CallCompute, 0), 1);
    expect(compute(test2CallCompute, 0), throwsA(2));
    expect(compute(test3CallCompute, 0), throwsRemoteError);
    expect(await compute(test4CallCompute, 0), 1);
    expect(compute(test5CallCompute, 0), throwsRemoteError);

    expect(compute(testInvalidResponse, 0), throwsRemoteError);
    expect(compute(testInvalidError, 0), throwsRemoteError);

    expect(await computeStaticMethod(10), 100);
    expect(await computeClosure(10), 100);
    expect(computeInvalidClosure(10), throwsArgumentError);
    expect(await computeInstanceMethod(10), 100);
    expect(computeInvalidInstanceMethod(10), throwsArgumentError);

    expect(await compute(testDebugName, null, debugLabel: 'debug_name'), 'debug_name');
    expect(await compute(testReturnNull, null), null);
  }, skip: kIsWeb); // [intended] isn't supported on the web.

  group('compute() closes all ports', () {
    test('with valid message', () async {
      await expectFileSuccessfullyCompletes('_compute_caller.dart');
    });
    test('with invalid message', () async {
      await expectFileSuccessfullyCompletes('_compute_caller_invalid_message.dart');
    });
    test('with valid error', () async {
      await expectFileSuccessfullyCompletes('_compute_caller.dart');
    });
    test('with invalid error', () async {
      await expectFileSuccessfullyCompletes('_compute_caller_invalid_message.dart');
    });
  }, skip: kIsWeb); // [intended] isn't supported on the web.
}
