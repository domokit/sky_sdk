// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/test/test_compiler.dart';
import 'package:flutter_tools/src/test/test_golden_comparator.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  final Uri testUri1 = Uri(scheme: 'file', path: 'test_file_1');
  final Uri testUri2 = Uri(scheme: 'file', path: 'test_file_2');
  final Uri goldenKey1 = Uri(path: 'golden_key_1');
  final Uri goldenKey2 = Uri(path: 'golden_key_2');
  final Uint8List imageBytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);

  late FileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
  });

  FakeCommand fakeFluterTester(
    String pathToBinTool, {
    required String stdout,
    required Uri mainUri,
    Map<String, String>? environment,
    Completer<void>? waitUntil,
  }) {
    return FakeCommand(
      command: <String>[
        pathToBinTool,
        '--disable-vm-service',
        '--non-interactive',
        'path_to_compiler_output.dill',
      ],
      stdout: stdout,
      environment: environment,
      completer: waitUntil,
    );
  }

  testWithoutContext('should succeed when a golden-file comparison matched', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester('flutter_tester', stdout: _encodeStdout(success: true), mainUri: testUri1),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenComparison result = await comparator.compare(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenComparisonDone(matched: true));
  });

  testWithoutContext('should succeed when a golden-file comparison does not match', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: false),
          mainUri: testUri1,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenComparison result = await comparator.compare(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenComparisonDone(matched: false));
  });

  testWithoutContext('should return an error when a golden-file comparison errors', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: false, message: 'Did a bad'),
          mainUri: testUri1,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenComparison result = await comparator.compare(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenComparisonError(error: 'Did a bad'));
  });

  testWithoutContext('should succeed when a golden-file update completes', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester('flutter_tester', stdout: _encodeStdout(success: true), mainUri: testUri1),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenUpdate result = await comparator.update(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenUpdateDone());
  });

  testWithoutContext('should error when a golden-file update errors', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: false, message: 'Did a bad'),
          mainUri: testUri1,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenUpdate result = await comparator.update(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenUpdateError(error: 'Did a bad'));
  });

  testWithoutContext('provides environment variables to the process', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: true),
          environment: <String, String>{'THE_ANSWER': '42'},
          mainUri: testUri1,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
      environment: <String, String>{'THE_ANSWER': '42'},
    );

    final TestGoldenUpdate result = await comparator.update(testUri1, imageBytes, goldenKey1);
    expect(result, const TestGoldenUpdateDone());
  });

  testWithoutContext('reuses the process for the same test file', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: <String>[
            _encodeStdout(success: false, message: '1 Did a bad'),
            _encodeStdout(success: false, message: '2 Did a bad'),
          ].join('\n'),
          mainUri: testUri1,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenComparison result1 = await comparator.compare(testUri1, imageBytes, goldenKey1);
    expect(result1, const TestGoldenComparisonError(error: '1 Did a bad'));

    final TestGoldenComparison result2 = await comparator.compare(testUri1, imageBytes, goldenKey2);
    expect(result2, const TestGoldenComparisonError(error: '2 Did a bad'));
  });

  testWithoutContext('does not reuse the process for different test file', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.list(<FakeCommand>[
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: false, message: '1 Did a bad'),
          mainUri: testUri1,
        ),
        fakeFluterTester(
          'flutter_tester',
          stdout: _encodeStdout(success: false, message: '2 Did a bad'),
          mainUri: testUri2,
        ),
      ]),
      fileSystem: fileSystem,
      logger: logger,
    );

    final TestGoldenComparison result1 = await comparator.compare(testUri1, imageBytes, goldenKey1);
    expect(result1, const TestGoldenComparisonError(error: '1 Did a bad'));

    final TestGoldenComparison result2 = await comparator.compare(testUri2, imageBytes, goldenKey2);
    expect(result2, const TestGoldenComparisonError(error: '2 Did a bad'));
  });

  testWithoutContext('deletes the temporary directory when closed', () async {
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: _FakeTestCompiler.new,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.empty(),
      fileSystem: fileSystem,
      logger: logger,
    );

    expect(fileSystem.systemTempDirectory.listSync(recursive: true), isNotEmpty);
    await comparator.close();
    expect(fileSystem.systemTempDirectory.listSync(recursive: true), isEmpty);
  });

  testWithoutContext('disposes the test compiler when closed', () async {
    final _FakeTestCompiler testCompiler = _FakeTestCompiler();
    final TestGoldenComparator comparator = TestGoldenComparator(
      compilerFactory: () => testCompiler,
      flutterTesterBinPath: 'flutter_tester',
      processManager: FakeProcessManager.empty(),
      fileSystem: fileSystem,
      logger: logger,
    );

    expect(testCompiler.disposed, false);
    await comparator.close();
    expect(testCompiler.disposed, true);
  });
}

String _encodeStdout({required bool success, String? message}) {
  return jsonEncode(<String, Object?>{'success': success, if (message != null) 'message': message});
}

final class _FakeTestCompiler extends Fake implements TestCompiler {
  bool disposed = false;

  @override
  Future<TestCompilerResult> compile(Uri mainDart) async {
    return TestCompilerComplete(outputPath: 'path_to_compiler_output.dill', mainUri: mainDart);
  }

  @override
  Future<void> dispose() async {
    disposed = true;
  }
}
