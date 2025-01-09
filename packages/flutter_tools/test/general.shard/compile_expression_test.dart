// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:package_config/package_config.dart';
import 'package:process/process.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

void main() {
  late FakeProcessManager processManager;
  late ResidentCompiler generator;
  late MemoryIOSink frontendServerStdIn;
  late StreamController<String> stdErrStreamController;
  late BufferLogger testLogger;
  late MemoryFileSystem fileSystem;

  setUp(() {
    testLogger = BufferLogger.test();
    processManager = FakeProcessManager();
    frontendServerStdIn = MemoryIOSink();
    fileSystem = MemoryFileSystem.test();
    generator = ResidentCompiler(
      'sdkroot',
      buildMode: BuildMode.debug,
      artifacts: Artifacts.test(),
      processManager: processManager,
      logger: testLogger,
      platform: FakePlatform(),
      fileSystem: fileSystem,
    );

    stdErrStreamController = StreamController<String>();
    processManager.process.stdin = frontendServerStdIn;
    processManager.process.stderr = stdErrStreamController.stream.transform(utf8.encoder);
  });

  testWithoutContext('compile expression fails if not previously compiled', () async {
    final CompilerOutput? result = await generator.compileExpression(
      '2+2',
      null,
      null,
      null,
      null,
      null,
      null,
      null,
      null,
      false,
    );

    expect(result, isNull);
  });

  testWithoutContext('compile expression can compile single expression', () async {
    final Completer<List<int>> compileResponseCompleter = Completer<List<int>>();
    final Completer<List<int>> compileExpressionResponseCompleter = Completer<List<int>>();
    fileSystem.file('/path/to/main.dart.dill')
      ..createSync(recursive: true)
      ..writeAsBytesSync(<int>[1, 2, 3, 4]);

    processManager.process.stdout = Stream<List<int>>.fromFutures(<Future<List<int>>>[
      compileResponseCompleter.future,
      compileExpressionResponseCompleter.future,
    ]);
    compileResponseCompleter.complete(
      Future<List<int>>.value(
        utf8.encode('result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n'),
      ),
    );

    await generator
        .recompile(
          Uri.file('/path/to/main.dart'),
          null,
          /* invalidatedFiles */
          outputPath: '/build/',
          packageConfig: PackageConfig.empty,
          projectRootPath: '',
          fs: fileSystem,
        )
        .then((CompilerOutput? output) {
          expect(frontendServerStdIn.getAndClear(), 'compile file:///path/to/main.dart\n');
          expect(testLogger.errorText, equals('line1\nline2\n'));
          expect(output!.outputFilename, equals('/path/to/main.dart.dill'));

          compileExpressionResponseCompleter.complete(
            Future<List<int>>.value(
              utf8.encode(
                'result def\nline1\nline2\ndef\ndef /path/to/main.dart.dill.incremental 0\n',
              ),
            ),
          );
          generator
              .compileExpression('2+2', null, null, null, null, null, null, null, null, false)
              .then((CompilerOutput? outputExpression) {
                expect(outputExpression, isNotNull);
                expect(outputExpression!.expressionData, <int>[1, 2, 3, 4]);
              });
        });
  });

  testWithoutContext('compile expressions without awaiting', () async {
    final Completer<List<int>> compileResponseCompleter = Completer<List<int>>();
    final Completer<List<int>> compileExpressionResponseCompleter1 = Completer<List<int>>();
    final Completer<List<int>> compileExpressionResponseCompleter2 = Completer<List<int>>();

    processManager.process.stdout = Stream<List<int>>.fromFutures(<Future<List<int>>>[
      compileResponseCompleter.future,
      compileExpressionResponseCompleter1.future,
      compileExpressionResponseCompleter2.future,
    ]);

    // The test manages timing via completers.
    unawaited(
      generator
          .recompile(
            Uri.parse('/path/to/main.dart'),
            null,
            /* invalidatedFiles */
            outputPath: '/build/',
            packageConfig: PackageConfig.empty,
            projectRootPath: '',
            fs: MemoryFileSystem(),
          )
          .then((CompilerOutput? outputCompile) {
            expect(testLogger.errorText, equals('line1\nline2\n'));
            expect(outputCompile!.outputFilename, equals('/path/to/main.dart.dill'));

            fileSystem.file('/path/to/main.dart.dill.incremental')
              ..createSync(recursive: true)
              ..writeAsBytesSync(<int>[0, 1, 2, 3]);
            compileExpressionResponseCompleter1.complete(
              Future<List<int>>.value(
                utf8.encode(
                  'result def\nline1\nline2\ndef /path/to/main.dart.dill.incremental 0\n',
                ),
              ),
            );
          }),
    );

    // The test manages timing via completers.
    final Completer<bool> lastExpressionCompleted = Completer<bool>();
    unawaited(
      generator
          .compileExpression('0+1', null, null, null, null, null, null, null, null, false)
          .then((CompilerOutput? outputExpression) {
            expect(outputExpression, isNotNull);
            expect(outputExpression!.expressionData, <int>[0, 1, 2, 3]);

            fileSystem.file('/path/to/main.dart.dill.incremental')
              ..createSync(recursive: true)
              ..writeAsBytesSync(<int>[4, 5, 6, 7]);
            compileExpressionResponseCompleter2.complete(
              Future<List<int>>.value(
                utf8.encode(
                  'result def\nline1\nline2\ndef /path/to/main.dart.dill.incremental 0\n',
                ),
              ),
            );
          }),
    );

    // The test manages timing via completers.
    unawaited(
      generator
          .compileExpression('1+1', null, null, null, null, null, null, null, null, false)
          .then((CompilerOutput? outputExpression) {
            expect(outputExpression, isNotNull);
            expect(outputExpression!.expressionData, <int>[4, 5, 6, 7]);
            lastExpressionCompleted.complete(true);
          }),
    );

    compileResponseCompleter.complete(
      Future<List<int>>.value(
        utf8.encode('result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n'),
      ),
    );

    expect(await lastExpressionCompleted.future, isTrue);
  });
}

class FakeProcess extends Fake implements Process {
  @override
  Stream<List<int>> stdout = const Stream<List<int>>.empty();

  @override
  Stream<List<int>> stderr = const Stream<List<int>>.empty();

  @override
  IOSink stdin = IOSink(StreamController<List<int>>().sink);

  @override
  Future<int> get exitCode => Completer<int>().future;
}

class FakeProcessManager extends Fake implements ProcessManager {
  final FakeProcess process = FakeProcess();

  @override
  bool canRun(dynamic executable, {String? workingDirectory}) {
    return true;
  }

  @override
  Future<Process> start(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    ProcessStartMode mode = ProcessStartMode.normal,
  }) async {
    return process;
  }
}
