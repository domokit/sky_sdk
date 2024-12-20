// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/async_guard.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:package_config/package_config.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

void main() {
  late ResidentCompiler generator;
  late ResidentCompiler generatorWithScheme;
  late ResidentCompiler generatorWithPlatformDillAndLibrariesSpec;
  late MemoryIOSink frontendServerStdIn;
  late BufferLogger testLogger;
  late StdoutHandler generatorStdoutHandler;
  late StdoutHandler generatorWithSchemeStdoutHandler;
  late FakeProcessManager fakeProcessManager;

  const List<String> frontendServerCommand = <String>[
    'Artifact.engineDartAotRuntime',
    'Artifact.frontendServerSnapshotForEngineDartSdk',
    '--sdk-root',
    'sdkroot/',
    '--incremental',
    '--target=flutter',
    '--experimental-emit-debug-metadata',
    '--output-dill',
    '/build/',
    '-Ddart.vm.profile=false',
    '-Ddart.vm.product=false',
    '--enable-asserts',
    '--track-widget-creation',
  ];

  setUp(() {
    testLogger = BufferLogger.test();
    frontendServerStdIn = MemoryIOSink();

    fakeProcessManager = FakeProcessManager.empty();
    generatorStdoutHandler = StdoutHandler(logger: testLogger, fileSystem: MemoryFileSystem.test());
    generatorWithSchemeStdoutHandler = StdoutHandler(
      logger: testLogger,
      fileSystem: MemoryFileSystem.test(),
    );
    generator = DefaultResidentCompiler(
      'sdkroot',
      buildMode: BuildMode.debug,
      logger: testLogger,
      processManager: fakeProcessManager,
      artifacts: Artifacts.test(),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
      stdoutHandler: generatorStdoutHandler,
    );
    generatorWithScheme = DefaultResidentCompiler(
      'sdkroot',
      buildMode: BuildMode.debug,
      logger: testLogger,
      processManager: fakeProcessManager,
      artifacts: Artifacts.test(),
      platform: FakePlatform(),
      fileSystemRoots: <String>['/foo/bar/fizz'],
      fileSystemScheme: 'scheme',
      fileSystem: MemoryFileSystem.test(),
      stdoutHandler: generatorWithSchemeStdoutHandler,
    );
    generatorWithPlatformDillAndLibrariesSpec = DefaultResidentCompiler(
      'sdkroot',
      buildMode: BuildMode.debug,
      logger: testLogger,
      processManager: fakeProcessManager,
      artifacts: Artifacts.test(),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
      stdoutHandler: generatorStdoutHandler,
      platformDill: '/foo/platform.dill',
      librariesSpec: '/bar/libraries.json',
    );
  });

  testWithoutContext('incremental compile single dart compile', () async {
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[...frontendServerCommand, '--verbosity=error'],
        stdout: 'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
      ),
    );

    final CompilerOutput? output = await generator.recompile(
      Uri.parse('/path/to/main.dart'),
      null /* invalidatedFiles */,
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile /path/to/main.dart\n');
    expect(testLogger.errorText, equals('line1\nline2\n'));
    expect(output?.outputFilename, equals('/path/to/main.dart.dill'));
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testWithoutContext('incremental compile single dart compile with filesystem scheme', () async {
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[
          ...frontendServerCommand,
          '--filesystem-root',
          '/foo/bar/fizz',
          '--filesystem-scheme',
          'scheme',
          '--verbosity=error',
        ],
        stdout: 'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
      ),
    );

    final CompilerOutput? output = await generatorWithScheme.recompile(
      Uri.parse('file:///foo/bar/fizz/main.dart'),
      null /* invalidatedFiles */,
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile scheme:///main.dart\n');
    expect(testLogger.errorText, equals('line1\nline2\n'));
    expect(output?.outputFilename, equals('/path/to/main.dart.dill'));
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testWithoutContext('incremental compile single dart compile abnormally terminates', () async {
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[...frontendServerCommand, '--verbosity=error'],
        stdin: frontendServerStdIn,
      ),
    );

    expect(
      asyncGuard(
        () => generator.recompile(
          Uri.parse('/path/to/main.dart'),
          null,
          /* invalidatedFiles */
          outputPath: '/build/',
          packageConfig: PackageConfig.empty,
          fs: MemoryFileSystem(),
          projectRootPath: '',
        ),
      ),
      throwsToolExit(),
    );
  });

  testWithoutContext(
    'incremental compile single dart compile abnormally terminates via exitCode',
    () async {
      fakeProcessManager.addCommand(
        FakeCommand(
          command: const <String>[...frontendServerCommand, '--verbosity=error'],
          stdin: frontendServerStdIn,
          exitCode: 1,
        ),
      );

      expect(
        asyncGuard(
          () => generator.recompile(
            Uri.parse('/path/to/main.dart'),
            null,
            /* invalidatedFiles */
            outputPath: '/build/',
            packageConfig: PackageConfig.empty,
            fs: MemoryFileSystem(),
            projectRootPath: '',
          ),
        ),
        throwsToolExit(message: 'the Dart compiler exited unexpectedly.'),
      );
    },
  );

  testWithoutContext('incremental compile and recompile', () async {
    final Completer<void> completer = Completer<void>();
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[...frontendServerCommand, '--verbosity=error'],
        stdout: 'result abc\nline0\nline1\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
        completer: completer,
      ),
    );

    await generator.recompile(
      Uri.parse('/path/to/main.dart'),
      null,
      /* invalidatedFiles */
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      projectRootPath: '',
      fs: MemoryFileSystem(),
    );
    expect(frontendServerStdIn.getAndClear(), 'compile /path/to/main.dart\n');

    // No accept or reject commands should be issued until we
    // send recompile request.
    await _accept(generator, frontendServerStdIn, '');
    await _reject(generatorStdoutHandler, generator, frontendServerStdIn, '', '');

    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
    );

    await _accept(generator, frontendServerStdIn, r'^accept\n$');

    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
    );
    // No sources returned from reject command.
    await _reject(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nabc\n',
      r'^reject\n$',
    );
    completer.complete();
    expect(frontendServerStdIn.getAndClear(), isEmpty);
    expect(
      testLogger.errorText,
      equals(
        'line0\nline1\n'
        'line1\nline2\n'
        'line1\nline2\n',
      ),
    );
  });

  testWithoutContext('incremental compile and recompile with filesystem scheme', () async {
    final Completer<void> completer = Completer<void>();
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[
          ...frontendServerCommand,
          '--filesystem-root',
          '/foo/bar/fizz',
          '--filesystem-scheme',
          'scheme',
          '--verbosity=error',
        ],
        stdout: 'result abc\nline0\nline1\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
        completer: completer,
      ),
    );
    await generatorWithScheme.recompile(
      Uri.parse('file:///foo/bar/fizz/main.dart'),
      null,
      /* invalidatedFiles */
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile scheme:///main.dart\n');

    // No accept or reject commands should be issued until we
    // send recompile request.
    await _accept(generatorWithScheme, frontendServerStdIn, '');
    await _reject(
      generatorWithSchemeStdoutHandler,
      generatorWithScheme,
      frontendServerStdIn,
      '',
      '',
    );

    await _recompile(
      generatorWithSchemeStdoutHandler,
      generatorWithScheme,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
      mainUri: Uri.parse('file:///foo/bar/fizz/main.dart'),
      expectedMainUri: 'scheme:///main.dart',
    );

    await _accept(generatorWithScheme, frontendServerStdIn, r'^accept\n$');

    await _recompile(
      generatorWithSchemeStdoutHandler,
      generatorWithScheme,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
      mainUri: Uri.parse('file:///foo/bar/fizz/main.dart'),
      expectedMainUri: 'scheme:///main.dart',
    );
    // No sources returned from reject command.
    await _reject(
      generatorWithSchemeStdoutHandler,
      generatorWithScheme,
      frontendServerStdIn,
      'result abc\nabc\n',
      r'^reject\n$',
    );
    completer.complete();
    expect(frontendServerStdIn.getAndClear(), isEmpty);
    expect(
      testLogger.errorText,
      equals(
        'line0\nline1\n'
        'line1\nline2\n'
        'line1\nline2\n',
      ),
    );
  });

  testWithoutContext(
    'incremental compile and recompile non-entrypoint file with filesystem scheme',
    () async {
      final Uri mainUri = Uri.parse('file:///foo/bar/fizz/main.dart');
      const String expectedMainUri = 'scheme:///main.dart';
      final List<Uri> updatedUris = <Uri>[mainUri, Uri.parse('file:///foo/bar/fizz/other.dart')];
      const List<String> expectedUpdatedUris = <String>[expectedMainUri, 'scheme:///other.dart'];

      final Completer<void> completer = Completer<void>();
      fakeProcessManager.addCommand(
        FakeCommand(
          command: const <String>[
            ...frontendServerCommand,
            '--filesystem-root',
            '/foo/bar/fizz',
            '--filesystem-scheme',
            'scheme',
            '--verbosity=error',
          ],
          stdout: 'result abc\nline0\nline1\nabc\nabc /path/to/main.dart.dill 0',
          stdin: frontendServerStdIn,
          completer: completer,
        ),
      );
      await generatorWithScheme.recompile(
        Uri.parse('file:///foo/bar/fizz/main.dart'),
        null,
        /* invalidatedFiles */
        outputPath: '/build/',
        packageConfig: PackageConfig.empty,
        fs: MemoryFileSystem(),
        projectRootPath: '',
      );
      expect(frontendServerStdIn.getAndClear(), 'compile scheme:///main.dart\n');

      // No accept or reject commands should be issued until we
      // send recompile request.
      await _accept(generatorWithScheme, frontendServerStdIn, '');
      await _reject(
        generatorWithSchemeStdoutHandler,
        generatorWithScheme,
        frontendServerStdIn,
        '',
        '',
      );

      await _recompile(
        generatorWithSchemeStdoutHandler,
        generatorWithScheme,
        frontendServerStdIn,
        'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
        mainUri: mainUri,
        expectedMainUri: expectedMainUri,
        updatedUris: updatedUris,
        expectedUpdatedUris: expectedUpdatedUris,
      );

      await _accept(generatorWithScheme, frontendServerStdIn, r'^accept\n$');

      await _recompile(
        generatorWithSchemeStdoutHandler,
        generatorWithScheme,
        frontendServerStdIn,
        'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
        mainUri: mainUri,
        expectedMainUri: expectedMainUri,
        updatedUris: updatedUris,
        expectedUpdatedUris: expectedUpdatedUris,
      );
      // No sources returned from reject command.
      await _reject(
        generatorWithSchemeStdoutHandler,
        generatorWithScheme,
        frontendServerStdIn,
        'result abc\nabc\n',
        r'^reject\n$',
      );
      completer.complete();
      expect(frontendServerStdIn.getAndClear(), isEmpty);
      expect(
        testLogger.errorText,
        equals(
          'line0\nline1\n'
          'line1\nline2\n'
          'line1\nline2\n',
        ),
      );
    },
  );

  testWithoutContext('incremental compile can suppress errors', () async {
    final Completer<void> completer = Completer<void>();
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[...frontendServerCommand, '--verbosity=error'],
        stdout: 'result abc\nline0\nline1\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
        completer: completer,
      ),
    );

    await generator.recompile(
      Uri.parse('/path/to/main.dart'),
      <Uri>[],
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile /path/to/main.dart\n');

    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
    );

    await _accept(generator, frontendServerStdIn, r'^accept\n$');

    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
      suppressErrors: true,
    );

    completer.complete();
    expect(frontendServerStdIn.getAndClear(), isEmpty);

    // Compiler message is not printed with suppressErrors: true above.
    expect(testLogger.errorText, isNot(equals('line1\nline2\n')));
    expect(testLogger.traceText, contains('line1\nline2\n'));
  });

  testWithoutContext('incremental compile and recompile twice', () async {
    final Completer<void> completer = Completer<void>();
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[...frontendServerCommand, '--verbosity=error'],
        stdout: 'result abc\nline0\nline1\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
        completer: completer,
      ),
    );
    await generator.recompile(
      Uri.parse('/path/to/main.dart'),
      null /* invalidatedFiles */,
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile /path/to/main.dart\n');

    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0\n',
    );
    await _recompile(
      generatorStdoutHandler,
      generator,
      frontendServerStdIn,
      'result abc\nline2\nline3\nabc\nabc /path/to/main.dart.dill 0\n',
    );

    completer.complete();
    expect(frontendServerStdIn.getAndClear(), isEmpty);
    expect(
      testLogger.errorText,
      equals(
        'line0\nline1\n'
        'line1\nline2\n'
        'line2\nline3\n',
      ),
    );
  });

  testWithoutContext('incremental compile with dartPluginRegistrant', () async {
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[
          ...frontendServerCommand,
          '--filesystem-root',
          '/foo/bar/fizz',
          '--filesystem-scheme',
          'scheme',
          '--source',
          'some/dir/plugin_registrant.dart',
          '--source',
          'package:flutter/src/dart_plugin_registrant.dart',
          '-Dflutter.dart_plugin_registrant=some/dir/plugin_registrant.dart',
          '--verbosity=error',
        ],
        stdout: 'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
      ),
    );

    final MemoryFileSystem fs = MemoryFileSystem();
    final File dartPluginRegistrant = fs.file('some/dir/plugin_registrant.dart')
      ..createSync(recursive: true);
    final CompilerOutput? output = await generatorWithScheme.recompile(
      Uri.parse('file:///foo/bar/fizz/main.dart'),
      null /* invalidatedFiles */,
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: fs,
      projectRootPath: '',
      checkDartPluginRegistry: true,
      dartPluginRegistrant: dartPluginRegistrant,
    );
    expect(frontendServerStdIn.getAndClear(), 'compile scheme:///main.dart\n');
    expect(testLogger.errorText, equals('line1\nline2\n'));
    expect(output?.outputFilename, equals('/path/to/main.dart.dill'));
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testWithoutContext('compile does not pass libraries-spec when using a platform dill', () async {
    fakeProcessManager.addCommand(
      FakeCommand(
        command: const <String>[
          ...frontendServerCommand,
          '--platform',
          '/foo/platform.dill',
          '--verbosity=error',
        ],
        stdout: 'result abc\nline1\nline2\nabc\nabc /path/to/main.dart.dill 0',
        stdin: frontendServerStdIn,
      ),
    );

    final CompilerOutput? output = await generatorWithPlatformDillAndLibrariesSpec.recompile(
      Uri.parse('/path/to/main.dart'),
      null /* invalidatedFiles */,
      outputPath: '/build/',
      packageConfig: PackageConfig.empty,
      fs: MemoryFileSystem(),
      projectRootPath: '',
    );
    expect(frontendServerStdIn.getAndClear(), 'compile /path/to/main.dart\n');
    expect(testLogger.errorText, equals('line1\nline2\n'));
    expect(output?.outputFilename, equals('/path/to/main.dart.dill'));
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });
}

Future<void> _recompile(
  StdoutHandler stdoutHandler,
  ResidentCompiler generator,
  MemoryIOSink frontendServerStdIn,
  String mockCompilerOutput, {
  bool suppressErrors = false,
  Uri? mainUri,
  String expectedMainUri = '/path/to/main.dart',
  List<Uri>? updatedUris,
  List<String>? expectedUpdatedUris,
}) async {
  mainUri ??= Uri.parse('/path/to/main.dart');
  updatedUris ??= <Uri>[mainUri];
  expectedUpdatedUris ??= <String>[expectedMainUri];

  final Future<CompilerOutput?> recompileFuture = generator.recompile(
    mainUri,
    updatedUris,
    outputPath: '/build/',
    packageConfig: PackageConfig.empty,
    suppressErrors: suppressErrors,
    fs: MemoryFileSystem(),
    projectRootPath: '',
  );

  // Put content into the output stream after generator.recompile gets
  // going few lines below, resets completer.
  scheduleMicrotask(() {
    LineSplitter.split(mockCompilerOutput).forEach(stdoutHandler.handler);
  });
  final CompilerOutput? output = await recompileFuture;
  expect(output?.outputFilename, equals('/path/to/main.dart.dill'));
  final String commands = frontendServerStdIn.getAndClear();
  final RegExp whitespace = RegExp(r'\s+');
  final List<String> parts = commands.split(whitespace);

  // Test that uuid matches at beginning and end.
  expect(parts[2], equals(parts[3 + updatedUris.length]));
  expect(parts[1], equals(expectedMainUri));
  for (int i = 0; i < expectedUpdatedUris.length; i++) {
    expect(parts[3 + i], equals(expectedUpdatedUris[i]));
  }
}

Future<void> _accept(
  ResidentCompiler generator,
  MemoryIOSink frontendServerStdIn,
  String expected,
) async {
  generator.accept();
  final String commands = frontendServerStdIn.getAndClear();
  final RegExp re = RegExp(expected);
  expect(commands, matches(re));
}

Future<void> _reject(
  StdoutHandler stdoutHandler,
  ResidentCompiler generator,
  MemoryIOSink frontendServerStdIn,
  String mockCompilerOutput,
  String expected,
) async {
  // Put content into the output stream after generator.recompile gets
  // going few lines below, resets completer.
  final Future<CompilerOutput?> rejectFuture = generator.reject();
  scheduleMicrotask(() {
    LineSplitter.split(mockCompilerOutput).forEach(stdoutHandler.handler);
  });
  final CompilerOutput? output = await rejectFuture;
  expect(output, isNull);

  final String commands = frontendServerStdIn.getAndClear();
  final RegExp re = RegExp(expected);
  expect(commands, matches(re));
}
