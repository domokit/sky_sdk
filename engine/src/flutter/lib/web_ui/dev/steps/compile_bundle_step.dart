// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:path/path.dart' as pathlib;
import 'package:pool/pool.dart';

import '../environment.dart';
import '../exceptions.dart';
import '../felt_config.dart';
import '../pipeline.dart';
import '../utils.dart'
    show AnsiColors, FilePath, ProcessManager, cleanup, getBundleBuildDirectory, startProcess;

/// Compiles a web test bundle into web_ui/build/test_bundles/<bundle-name>.
class CompileBundleStep implements PipelineStep {
  CompileBundleStep({required this.bundle, required this.isVerbose, this.testFiles});

  final TestBundle bundle;
  final bool isVerbose;
  final Set<FilePath>? testFiles;

  // Maximum number of concurrent compile processes to use.
  static final int _compileConcurrency = int.parse(
    io.Platform.environment['FELT_COMPILE_CONCURRENCY'] ?? '8',
  );
  final Pool compilePool = Pool(_compileConcurrency);

  @override
  String get description => 'compile_bundle';

  @override
  bool get isSafeToInterrupt => true;

  @override
  Future<void> interrupt() async {
    await cleanup();
  }

  io.Directory get testSetDirectory =>
      io.Directory(pathlib.join(environment.webUiTestDir.path, bundle.testSet.directory));

  io.Directory get outputBundleDirectory => getBundleBuildDirectory(bundle);

  List<FilePath> _findTestFiles() {
    final io.Directory testDirectory = testSetDirectory;
    if (!testDirectory.existsSync()) {
      throw ToolExit(
        'Test directory "${testDirectory.path}" for bundle ${bundle.name.ansiMagenta} does not exist.',
      );
    }
    return testDirectory
        .listSync(recursive: true)
        .whereType<io.File>()
        .where((io.File f) => f.path.endsWith('_test.dart'))
        .map<FilePath>(
          (io.File f) =>
              FilePath.fromWebUi(pathlib.relative(f.path, from: environment.webUiRootDir.path)),
        )
        .toList();
  }

  TestCompiler _createCompiler(CompileConfiguration config) {
    switch (config.compiler) {
      case Compiler.dart2js:
        return Dart2JSCompiler(
          testSetDirectory,
          outputBundleDirectory,
          renderer: config.renderer,
          isVerbose: isVerbose,
        );
      case Compiler.dart2wasm:
        return Dart2WasmCompiler(
          testSetDirectory,
          outputBundleDirectory,
          renderer: config.renderer,
          isVerbose: isVerbose,
        );
    }
  }

  @override
  Future<void> run() async {
    print('Compiling test bundle ${bundle.name.ansiMagenta}...');
    final List<FilePath> allTests = _findTestFiles();
    final List<TestCompiler> compilers =
        bundle.compileConfigs
            .map((CompileConfiguration config) => _createCompiler(config))
            .toList();
    final Stopwatch stopwatch = Stopwatch()..start();
    final String testSetDirectoryPath = testSetDirectory.path;

    // Clear out old bundle compilations, if they exist
    if (outputBundleDirectory.existsSync()) {
      outputBundleDirectory.deleteSync(recursive: true);
    }

    final List<Future<MapEntry<String, CompileResult>>> pendingResults =
        <Future<MapEntry<String, CompileResult>>>[];
    for (final TestCompiler compiler in compilers) {
      for (final FilePath testFile in allTests) {
        final String relativePath = pathlib.relative(testFile.absolute, from: testSetDirectoryPath);
        final Future<MapEntry<String, CompileResult>> result = compilePool.withResource(() async {
          if (testFiles != null && !testFiles!.contains(testFile)) {
            return MapEntry<String, CompileResult>(relativePath, CompileResult.filtered);
          }
          final bool success = await compiler.compileTest(testFile);
          const int maxTestNameLength = 80;
          final String truncatedPath =
              relativePath.length > maxTestNameLength
                  ? relativePath.replaceRange(maxTestNameLength - 3, relativePath.length, '...')
                  : relativePath;
          final String expandedPath = truncatedPath.padRight(maxTestNameLength);
          io.stdout.write('\r  ${success ? expandedPath.ansiGreen : expandedPath.ansiRed}');
          return success
              ? MapEntry<String, CompileResult>(relativePath, CompileResult.success)
              : MapEntry<String, CompileResult>(relativePath, CompileResult.compilationFailure);
        });
        pendingResults.add(result);
      }
    }
    final Map<String, CompileResult> results = Map<String, CompileResult>.fromEntries(
      await Future.wait(pendingResults),
    );
    stopwatch.stop();

    final String resultsJson = const JsonEncoder.withIndent('  ').convert(<String, dynamic>{
      'name': bundle.name,
      'directory': bundle.testSet.directory,
      'builds':
          bundle.compileConfigs
              .map(
                (CompileConfiguration config) => <String, dynamic>{
                  'compiler': config.compiler.name,
                  'renderer': config.renderer.name,
                },
              )
              .toList(),
      'compileTimeInMs': stopwatch.elapsedMilliseconds,
      'results': results.map((String k, CompileResult v) => MapEntry<String, String>(k, v.name)),
    });
    final io.File outputResultsFile = io.File(
      pathlib.join(outputBundleDirectory.path, 'results.json'),
    );
    outputResultsFile.writeAsStringSync(resultsJson);
    final List<String> failedFiles = <String>[];
    results.forEach((String fileName, CompileResult result) {
      if (result == CompileResult.compilationFailure) {
        failedFiles.add(fileName);
      }
    });
    if (failedFiles.isEmpty) {
      print(
        '\rCompleted compilation of ${bundle.name.ansiMagenta} in ${stopwatch.elapsedMilliseconds}ms.'
            .padRight(82),
      );
    } else {
      print(
        '\rThe bundle ${bundle.name.ansiMagenta} compiled with some failures in ${stopwatch.elapsedMilliseconds}ms.',
      );
      print('Compilation failures:');
      for (final String fileName in failedFiles) {
        print('  $fileName');
      }
      throw ToolExit('Failed to compile ${bundle.name.ansiMagenta}.');
    }
  }
}

enum CompileResult { success, compilationFailure, filtered }

abstract class TestCompiler {
  TestCompiler(
    this.inputTestSetDirectory,
    this.outputTestBundleDirectory, {
    required this.renderer,
    required this.isVerbose,
  });

  final io.Directory inputTestSetDirectory;
  final io.Directory outputTestBundleDirectory;
  final Renderer renderer;
  final bool isVerbose;

  Future<bool> compileTest(FilePath input);
}

class Dart2JSCompiler extends TestCompiler {
  Dart2JSCompiler(
    super.inputTestSetDirectory,
    super.outputTestBundleDirectory, {
    required super.renderer,
    required super.isVerbose,
  });

  @override
  Future<bool> compileTest(FilePath input) async {
    final String relativePath = pathlib.relative(input.absolute, from: inputTestSetDirectory.path);

    final String targetFileName = pathlib.join(
      outputTestBundleDirectory.path,
      '$relativePath.browser_test.dart.js',
    );

    final io.Directory outputDirectory = io.File(targetFileName).parent;
    if (!outputDirectory.existsSync()) {
      outputDirectory.createSync(recursive: true);
    }

    final List<String> arguments = <String>[
      'compile',
      'js',
      '--no-minify',
      '--disable-inlining',
      '--enable-asserts',

      // We do not want to auto-select a renderer in tests. As of today, tests
      // are designed to run in one specific mode. So instead, we specify the
      // renderer explicitly.
      '-DFLUTTER_WEB_AUTO_DETECT=false',
      '-DFLUTTER_WEB_USE_SKIA=${renderer == Renderer.canvaskit}',
      '-DFLUTTER_WEB_USE_SKWASM=${renderer == Renderer.skwasm}',

      '-O2',
      '-o',
      targetFileName, // target path.
      relativePath, // current path.
    ];

    final ProcessManager process = await startProcess(
      environment.dartExecutable,
      arguments,
      workingDirectory: inputTestSetDirectory.path,
      failureIsSuccess: true,
      evalOutput: !isVerbose,
    );
    final int exitCode = await process.wait();
    if (exitCode != 0) {
      io.stderr.writeln(
        'ERROR: Failed to compile test $input. '
        'Dart2js exited with exit code $exitCode',
      );
      return false;
    } else {
      return true;
    }
  }
}

class Dart2WasmCompiler extends TestCompiler {
  Dart2WasmCompiler(
    super.inputTestSetDirectory,
    super.outputTestBundleDirectory, {
    required super.renderer,
    required super.isVerbose,
  });

  @override
  Future<bool> compileTest(FilePath input) async {
    final String relativePath = pathlib.relative(input.absolute, from: inputTestSetDirectory.path);

    final String targetFileName = pathlib.join(
      outputTestBundleDirectory.path,
      '$relativePath.browser_test.dart.wasm',
    );

    final io.Directory outputDirectory = io.File(targetFileName).parent;
    if (!outputDirectory.existsSync()) {
      outputDirectory.createSync(recursive: true);
    }

    final List<String> arguments = <String>[
      environment.dart2wasmSnapshotPath,

      '--libraries-spec=${environment.dartSdkDir.path}/lib/libraries.json',
      '--enable-asserts',
      '--enable-experimental-wasm-interop',

      // We do not want to auto-select a renderer in tests. As of today, tests
      // are designed to run in one specific mode. So instead, we specify the
      // renderer explicitly.
      '-DFLUTTER_WEB_AUTO_DETECT=false',
      '-DFLUTTER_WEB_USE_SKIA=${renderer == Renderer.canvaskit}',
      '-DFLUTTER_WEB_USE_SKWASM=${renderer == Renderer.skwasm}',

      if (renderer == Renderer.skwasm) ...<String>[
        '--import-shared-memory',
        '--shared-memory-max-pages=32768',
      ],

      relativePath, // current path.
      targetFileName, // target path.
    ];

    final ProcessManager process = await startProcess(
      environment.dartAotRuntimePath,
      arguments,
      workingDirectory: inputTestSetDirectory.path,
      failureIsSuccess: true,
      evalOutput: !isVerbose,
    );
    final int exitCode = await process.wait();

    if (exitCode != 0) {
      io.stderr.writeln(
        'ERROR: Failed to compile test $input. '
        'dart2wasm exited with exit code $exitCode',
      );
      return false;
    } else {
      return true;
    }
  }
}
