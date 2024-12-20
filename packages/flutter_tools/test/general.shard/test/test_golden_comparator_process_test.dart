// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/test/test_golden_comparator.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

void main() {
  group('Test that TestGoldenComparatorProcess', () {
    late File imageFile;
    late Uri goldenKey;
    late File imageFile2;
    late Uri goldenKey2;
    late FakeProcess Function(String) createFakeProcess;

    setUpAll(() {
      imageFile = globals.fs.file('test_image_file');
      goldenKey = Uri.parse('file://golden_key');
      imageFile2 = globals.fs.file('second_test_image_file');
      goldenKey2 = Uri.parse('file://second_golden_key');
      createFakeProcess =
          (String stdout) =>
              FakeProcess(exitCode: Future<int>.value(0), stdout: stdoutFromString(stdout));
    });

    testWithoutContext('can pass data', () async {
      final Map<String, dynamic> expectedResponse = <String, dynamic>{
        'success': true,
        'message': 'some message',
      };

      final FakeProcess mockProcess = createFakeProcess('${jsonEncode(expectedResponse)}\n');
      final MemoryIOSink ioSink = mockProcess.stdin as MemoryIOSink;

      final TestGoldenComparatorProcess process = TestGoldenComparatorProcess(
        mockProcess,
        logger: BufferLogger.test(),
      );
      process.sendCommand(imageFile, goldenKey, false);

      final Map<String, dynamic> response = await process.getResponse();
      final String stringToStdin = ioSink.getAndClear();

      expect(response, expectedResponse);
      expect(
        stringToStdin,
        '{"imageFile":"test_image_file","key":"file://golden_key/","update":false}\n',
      );
    });

    testWithoutContext('can handle multiple requests', () async {
      final Map<String, dynamic> expectedResponse1 = <String, dynamic>{
        'success': true,
        'message': 'some message',
      };
      final Map<String, dynamic> expectedResponse2 = <String, dynamic>{
        'success': false,
        'message': 'some other message',
      };

      final FakeProcess mockProcess = createFakeProcess(
        '${jsonEncode(expectedResponse1)}\n${jsonEncode(expectedResponse2)}\n',
      );
      final MemoryIOSink ioSink = mockProcess.stdin as MemoryIOSink;

      final TestGoldenComparatorProcess process = TestGoldenComparatorProcess(
        mockProcess,
        logger: BufferLogger.test(),
      );
      process.sendCommand(imageFile, goldenKey, false);

      final Map<String, dynamic> response1 = await process.getResponse();

      process.sendCommand(imageFile2, goldenKey2, true);

      final Map<String, dynamic> response2 = await process.getResponse();
      final String stringToStdin = ioSink.getAndClear();

      expect(response1, expectedResponse1);
      expect(response2, expectedResponse2);
      expect(
        stringToStdin,
        '{"imageFile":"test_image_file","key":"file://golden_key/","update":false}\n{"imageFile":"second_test_image_file","key":"file://second_golden_key/","update":true}\n',
      );
    });

    testWithoutContext('ignores anything that does not look like JSON', () async {
      final Map<String, dynamic> expectedResponse = <String, dynamic>{
        'success': true,
        'message': 'some message',
      };

      final FakeProcess mockProcess = createFakeProcess('''
Some random data including {} curly bracket
  {} curly bracket that is not on the beginning of the line
${jsonEncode(expectedResponse)}
{"success": false}
Other JSON data after the initial data
''');
      final MemoryIOSink ioSink = mockProcess.stdin as MemoryIOSink;

      final TestGoldenComparatorProcess process = TestGoldenComparatorProcess(
        mockProcess,
        logger: BufferLogger.test(),
      );
      process.sendCommand(imageFile, goldenKey, false);

      final Map<String, dynamic> response = await process.getResponse();
      final String stringToStdin = ioSink.getAndClear();

      expect(response, expectedResponse);
      expect(
        stringToStdin,
        '{"imageFile":"test_image_file","key":"file://golden_key/","update":false}\n',
      );
    });
  });
}

Stream<List<int>> stdoutFromString(String string) =>
    Stream<List<int>>.fromIterable(<List<int>>[utf8.encode(string)]);
