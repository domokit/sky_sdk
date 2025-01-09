// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

const String _dartBin = 'bin/cache/dart-sdk/bin/dart';

void main() {
  setUpAll(() {
    Cache.flutterRoot = '';
  });

  testWithoutContext('throws a tool exit if pub cannot be run', () async {
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final BufferLogger logger = BufferLogger.test();
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();

    processManager.excludedExecutables.add(_dartBin);
    fileSystem.file('pubspec.yaml').createSync();

    final Pub pub = Pub.test(
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
      usage: TestUsage(),
      platform: FakePlatform(),
      botDetector: const FakeBotDetector(false),
      stdio: FakeStdio(),
    );

    await expectLater(
      () => pub.deps(FlutterProject.fromDirectoryTest(fileSystem.currentDirectory)),
      throwsToolExit(message: 'Your Flutter SDK download may be corrupt'),
    );
  });

  testWithoutContext('fails on non-zero exit code', () async {
    final BufferLogger logger = BufferLogger.test();
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final ProcessManager processManager = _dartPubDepsFails(
      'Bad thing',
      project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
      exitCode: 1,
    );

    final Pub pub = Pub.test(
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
      usage: TestUsage(),
      platform: FakePlatform(),
      botDetector: const FakeBotDetector(false),
      stdio: FakeStdio(),
    );

    await expectLater(
      () => pub.deps(FlutterProject.fromDirectoryTest(fileSystem.currentDirectory)),
      throwsA(
        isA<StateError>().having(
          (StateError e) => e.message,
          'message',
          contains('dart pub --suppress-analytics deps --json failed'),
        ),
      ),
    );
  });

  testWithoutContext('fails on non-parseable JSON', () async {
    final BufferLogger logger = BufferLogger.test();
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final ProcessManager processManager = _dartPubDepsReturns(
      'Not JSON haha!',
      project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
    );

    final Pub pub = Pub.test(
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
      usage: TestUsage(),
      platform: FakePlatform(),
      botDetector: const FakeBotDetector(false),
      stdio: FakeStdio(),
    );

    await expectLater(
      () => pub.deps(FlutterProject.fromDirectoryTest(fileSystem.currentDirectory)),
      throwsA(
        isA<StateError>().having(
          (StateError e) => e.message,
          'message',
          contains('dart pub --suppress-analytics deps --json had unexpected output'),
        ),
      ),
    );
  });

  testWithoutContext('fails on unexpected JSON type', () async {
    final BufferLogger logger = BufferLogger.test();
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final ProcessManager processManager = _dartPubDepsReturns(
      '[]',
      project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
    );

    final Pub pub = Pub.test(
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
      usage: TestUsage(),
      platform: FakePlatform(),
      botDetector: const FakeBotDetector(false),
      stdio: FakeStdio(),
    );

    await expectLater(
      () => pub.deps(FlutterProject.fromDirectoryTest(fileSystem.currentDirectory)),
      throwsA(
        isA<StateError>().having(
          (StateError e) => e.message,
          'message',
          contains('Not a JSON object'),
        ),
      ),
    );
  });
}

ProcessManager _dartPubDepsReturns(String dartPubDepsOutput, {required FlutterProject project}) {
  return FakeProcessManager.list(<FakeCommand>[
    FakeCommand(
      command: const <String>[_dartBin, 'pub', '--suppress-analytics', 'deps', '--json'],
      stdout: dartPubDepsOutput,
      workingDirectory: project.directory.path,
    ),
  ]);
}

ProcessManager _dartPubDepsFails(
  String dartPubDepsError, {
  required FlutterProject project,
  required int exitCode,
}) {
  return FakeProcessManager.list(<FakeCommand>[
    FakeCommand(
      command: const <String>[_dartBin, 'pub', '--suppress-analytics', 'deps', '--json'],
      exitCode: exitCode,
      stderr: dartPubDepsError,
      workingDirectory: project.directory.path,
    ),
  ]);
}
