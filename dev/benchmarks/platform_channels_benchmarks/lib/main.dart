// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/foundation.dart' show kDebugMode;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:microbenchmarks/common.dart';

List<Object?> _makeTestBuffer(int size) {
  return <Object?>[
    for (int i = 0; i < size; i++)
      switch (i % 9) {
        0 => 1,
        1 => math.pow(2, 65),
        2 => 1234.0,
        3 => null,
        4 => <int>[1234],
        5 => <String, int>{'hello': 1234},
        6 => 'this is a test',
        7 => true,
        _ => Uint8List(64),
      },
  ];
}

Future<double> _runBasicStandardSmall(BasicMessageChannel<Object?> basicStandard, int count) async {
  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < count; ++i) {
    await basicStandard.send(1234);
  }
  watch.stop();
  return watch.elapsedMicroseconds / count;
}

class _Counter {
  int count = 0;
}

void _runBasicStandardParallelRecurse(
  BasicMessageChannel<Object?> basicStandard,
  _Counter counter,
  int count,
  Completer<int> completer,
  Object? payload,
) {
  counter.count += 1;
  if (counter.count == count) {
    completer.complete(counter.count);
  } else if (counter.count < count) {
    basicStandard.send(payload).then((Object? result) {
      _runBasicStandardParallelRecurse(basicStandard, counter, count, completer, payload);
    });
  }
}

Future<double> _runBasicStandardParallel(
  BasicMessageChannel<Object?> basicStandard,
  int count,
  Object? payload,
  int parallel,
) async {
  final Stopwatch watch = Stopwatch();
  final Completer<int> completer = Completer<int>();
  final _Counter counter = _Counter();
  watch.start();
  for (int i = 0; i < parallel; ++i) {
    basicStandard.send(payload).then((Object? result) {
      _runBasicStandardParallelRecurse(basicStandard, counter, count, completer, payload);
    });
  }
  await completer.future;
  watch.stop();
  return watch.elapsedMicroseconds / count;
}

Future<double> _runBasicStandardLarge(
  BasicMessageChannel<Object?> basicStandard,
  List<Object?> largeBuffer,
  int count,
) async {
  int size = 0;
  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < count; ++i) {
    final List<Object?>? result = await basicStandard.send(largeBuffer) as List<Object?>?;
    // This check should be tiny compared to the actual channel send/receive.
    size += (result == null) ? 0 : result.length;
  }
  watch.stop();

  if (size != largeBuffer.length * count) {
    throw Exception("There is an error with the echo channel, the results don't add up: $size");
  }

  return watch.elapsedMicroseconds / count;
}

Future<double> _runBasicBinary(
  BasicMessageChannel<ByteData> basicBinary,
  ByteData buffer,
  int count,
) async {
  int size = 0;
  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < count; ++i) {
    final ByteData? result = await basicBinary.send(buffer);
    // This check should be tiny compared to the actual channel send/receive.
    size += (result == null) ? 0 : result.lengthInBytes;
  }
  watch.stop();
  if (size != buffer.lengthInBytes * count) {
    throw Exception("There is an error with the echo channel, the results don't add up: $size");
  }

  return watch.elapsedMicroseconds / count;
}

Future<void> _runTest({
  required Future<double> Function(int) test,
  required BasicMessageChannel<Object?> resetChannel,
  required BenchmarkResultPrinter printer,
  required String description,
  required String name,
  required int numMessages,
}) async {
  print('running $name');
  resetChannel.send(true);
  // Prime test.
  await test(1);
  printer.addResult(
    description: description,
    value: await test(numMessages),
    unit: 'µs',
    name: name,
  );
}

Future<void> _runTests() async {
  if (kDebugMode) {
    throw Exception("Must be run in profile mode! Use 'flutter run --profile'.");
  }

  const BasicMessageChannel<Object?> resetChannel = BasicMessageChannel<Object?>(
    'dev.flutter.echo.reset',
    StandardMessageCodec(),
  );
  const BasicMessageChannel<Object?> basicStandard = BasicMessageChannel<Object?>(
    'dev.flutter.echo.basic.standard',
    StandardMessageCodec(),
  );
  const BasicMessageChannel<ByteData> basicBinary = BasicMessageChannel<ByteData>(
    'dev.flutter.echo.basic.binary',
    BinaryCodec(),
  );

  /// WARNING: Don't change the following line of code, it will invalidate
  /// `Large` tests. Instead make a different test. The size of largeBuffer
  /// serialized is 14214 bytes.
  final List<Object?> largeBuffer = _makeTestBuffer(1000);
  final ByteData largeBufferBytes = const StandardMessageCodec().encodeMessage(largeBuffer)!;
  final ByteData oneMB = ByteData(1024 * 1024);

  const int numMessages = 2500;

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  await _runTest(
    test: (int x) => _runBasicStandardSmall(basicStandard, x),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/StandardMessageCodec/Flutter->Host/Small',
    name: 'platform_channel_basic_standard_2host_small',
    numMessages: numMessages,
  );
  await _runTest(
    test: (int x) => _runBasicStandardLarge(basicStandard, largeBuffer, x),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/StandardMessageCodec/Flutter->Host/Large',
    name: 'platform_channel_basic_standard_2host_large',
    numMessages: numMessages,
  );
  await _runTest(
    test: (int x) => _runBasicBinary(basicBinary, largeBufferBytes, x),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/BinaryCodec/Flutter->Host/Large',
    name: 'platform_channel_basic_binary_2host_large',
    numMessages: numMessages,
  );
  await _runTest(
    test: (int x) => _runBasicBinary(basicBinary, oneMB, x),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/BinaryCodec/Flutter->Host/1MB',
    name: 'platform_channel_basic_binary_2host_1MB',
    numMessages: numMessages,
  );
  await _runTest(
    test: (int x) => _runBasicStandardParallel(basicStandard, x, 1234, 3),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/StandardMessageCodec/Flutter->Host/SmallParallel3',
    name: 'platform_channel_basic_standard_2host_small_parallel_3',
    numMessages: numMessages,
  );
  // Background platform channels aren't yet implemented for iOS.
  const BasicMessageChannel<Object?> backgroundStandard = BasicMessageChannel<Object?>(
    'dev.flutter.echo.background.standard',
    StandardMessageCodec(),
  );
  await _runTest(
    test: (int x) => _runBasicStandardSmall(backgroundStandard, x),
    resetChannel: resetChannel,
    printer: printer,
    description: 'BasicMessageChannel/StandardMessageCodec/Flutter->Host (background)/Small',
    name: 'platform_channel_basic_standard_2hostbackground_small',
    numMessages: numMessages,
  );
  await _runTest(
    test: (int x) => _runBasicStandardParallel(backgroundStandard, x, 1234, 3),
    resetChannel: resetChannel,
    printer: printer,
    description:
        'BasicMessageChannel/StandardMessageCodec/Flutter->Host (background)/SmallParallel3',
    name: 'platform_channel_basic_standard_2hostbackground_small_parallel_3',
    numMessages: numMessages,
  );
  printer.printToStdout();
  // Signal the end of our benchmark
  print('\n\n╡ ••• Done ••• ╞\n\n');
}

class _BenchmarkWidget extends StatefulWidget {
  const _BenchmarkWidget(this.tests);

  final Future<void> Function() tests;

  @override
  _BenchmarkWidgetState createState() => _BenchmarkWidgetState();
}

class _BenchmarkWidgetState extends State<_BenchmarkWidget> {
  @override
  void initState() {
    widget.tests();
    super.initState();
  }

  @override
  Widget build(BuildContext context) => Container();
}

void main() {
  runApp(const _BenchmarkWidget(_runTests));
}
