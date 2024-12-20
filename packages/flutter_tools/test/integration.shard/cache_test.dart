// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show ProcessSignal;

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:process/process.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';
import 'test_utils.dart';

final String dart = fileSystem.path.join(
  getFlutterRoot(),
  'bin',
  platform.isWindows ? 'dart.bat' : 'dart',
);

void main() {
  group('Cache.lock', () {
    // Windows locking is too flaky for this to work reliably.
    if (platform.isWindows) {
      return;
    }
    testWithoutContext('should log a message to stderr when lock is not acquired', () async {
      final String? oldRoot = Cache.flutterRoot;
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('cache_test.');
      final BufferLogger logger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences(),
      );
      logger.fatalWarnings = true;
      Process? process;
      try {
        Cache.flutterRoot = tempDir.absolute.path;
        final Cache cache = Cache.test(
          fileSystem: fileSystem,
          processManager: FakeProcessManager.any(),
          logger: logger,
        );
        final File cacheFile = fileSystem.file(
          fileSystem.path.join(Cache.flutterRoot!, 'bin', 'cache', 'lockfile'),
        )..createSync(recursive: true);
        final File script = fileSystem.file(
          fileSystem.path.join(Cache.flutterRoot!, 'bin', 'cache', 'test_lock.dart'),
        );
        script.writeAsStringSync(r'''
import 'dart:async';
import 'dart:io';

Future<void> main(List<String> args) async {
  File file = File(args[0]);
  final RandomAccessFile lock = file.openSync(mode: FileMode.write);
  while (true) {
    try {
      lock.lockSync();
      break;
    } on FileSystemException {}
  }
  await Future<void>.delayed(const Duration(seconds: 1));
  exit(0);
}
''');
        // Locks are per-process, so we have to launch a separate process to
        // test out cache locking.
        process = await const LocalProcessManager().start(<String>[
          dart,
          script.absolute.path,
          cacheFile.absolute.path,
        ]);
        // Wait for the script to lock the test cache file before checking to
        // see that the cache is unable to.
        bool locked = false;
        while (!locked) {
          // Give the script a chance to try for the lock much more often.
          await Future<void>.delayed(const Duration(milliseconds: 100));
          final RandomAccessFile lock = cacheFile.openSync(mode: FileMode.write);
          try {
            // If we can lock it, unlock immediately to give the script a
            // chance.
            lock.lockSync();
            lock.unlockSync();
          } on FileSystemException {
            // If we can't lock it, then the child script succeeded in locking
            // it, and we can now test.
            locked = true;
            break;
          }
        }
        // Finally, test that the cache cannot lock a locked file. This should
        // print a message if it can't lock the file.
        await cache.lock();
      } finally {
        // Just to keep from leaving the process hanging around.
        process?.kill(io.ProcessSignal.sighup);
        tryToDelete(tempDir);
        Cache.flutterRoot = oldRoot;
      }
      expect(logger.statusText, isEmpty);
      expect(logger.errorText, isEmpty);
      expect(
        logger.warningText,
        equals('Waiting for another flutter command to release the startup lock...\n'),
      );
      expect(logger.hadErrorOutput, isFalse);
      // Should still be false, since the particular "Waiting..." message above
      // aims to avoid triggering failure as a fatal warning.
      expect(logger.hadWarningOutput, isFalse);
    });
    testWithoutContext('should log a warning message for unknown version ', () async {
      final String? oldRoot = Cache.flutterRoot;
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('cache_test.');
      final BufferLogger logger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences(),
      );
      logger.fatalWarnings = true;
      try {
        Cache.flutterRoot = tempDir.absolute.path;
        final Cache cache = Cache.test(
          fileSystem: fileSystem,
          processManager: FakeProcessManager.any(),
          logger: logger,
        );
        final FakeVersionlessArtifact artifact = FakeVersionlessArtifact(cache);
        cache.registerArtifact(artifact);
        await artifact.update(
          FakeArtifactUpdater(),
          logger,
          fileSystem,
          FakeOperatingSystemUtils(),
        );
      } finally {
        tryToDelete(tempDir);
        Cache.flutterRoot = oldRoot;
      }
      expect(logger.statusText, isEmpty);
      expect(
        logger.warningText,
        equals(
          'No known version for the artifact name "fake". '
          'Flutter can continue, but the artifact may be re-downloaded on '
          'subsequent invocations until the problem is resolved.\n',
        ),
      );
      expect(logger.hadErrorOutput, isFalse);
      expect(logger.hadWarningOutput, isTrue);
    });
  });

  testWithoutContext('Dart SDK target arch matches host arch', () async {
    if (platform.isWindows) {
      return;
    }
    final ProcessResult dartResult = await const LocalProcessManager().run(<String>[
      dart,
      '--version',
    ]);
    // Parse 'arch' out of a string like '... "os_arch"\n'.
    final String dartTargetArch =
        (dartResult.stdout as String).trim().split(' ').last.replaceAll('"', '').split('_')[1];
    final ProcessResult unameResult = await const LocalProcessManager().run(<String>[
      'uname',
      '-m',
    ]);
    final String unameArch = (unameResult.stdout as String)
        .trim()
        .replaceAll('aarch64', 'arm64')
        .replaceAll('x86_64', 'x64');
    expect(dartTargetArch, equals(unameArch));
  });
}

class FakeArtifactUpdater extends Fake implements ArtifactUpdater {
  void Function(String, Uri, Directory)? onDownloadZipArchive;
  void Function(String, Uri, Directory)? onDownloadZipTarball;

  @override
  Future<void> downloadZippedTarball(String message, Uri url, Directory location) async {
    onDownloadZipTarball?.call(message, url, location);
  }

  @override
  Future<void> downloadZipArchive(String message, Uri url, Directory location) async {
    onDownloadZipArchive?.call(message, url, location);
  }

  @override
  void removeDownloadedFiles() {}
}

class FakeVersionlessArtifact extends CachedArtifact {
  FakeVersionlessArtifact(Cache cache) : super('fake', cache, DevelopmentArtifact.universal);

  @override
  String? get version => null;

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {}
}
