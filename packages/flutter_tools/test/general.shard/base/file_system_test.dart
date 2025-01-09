// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

class LocalFileSystemFake extends LocalFileSystem {
  LocalFileSystemFake.test({required super.signals}) : super.test();

  @override
  Directory get superSystemTempDirectory => directory('/does_not_exist');
}

void main() {
  group('fsUtils', () {
    late MemoryFileSystem fs;
    late FileSystemUtils fsUtils;

    setUp(() {
      fs = MemoryFileSystem.test();
      fsUtils = FileSystemUtils(fileSystem: fs, platform: FakePlatform());
    });

    testWithoutContext('getUniqueFile creates a unique file name', () async {
      final File fileA = fsUtils.getUniqueFile(fs.currentDirectory, 'foo', 'json')..createSync();
      final File fileB = fsUtils.getUniqueFile(fs.currentDirectory, 'foo', 'json');

      expect(fileA.path, '/foo_01.json');
      expect(fileB.path, '/foo_02.json');
    });

    testWithoutContext('getUniqueDirectory creates a unique directory name', () async {
      final Directory directoryA = fsUtils.getUniqueDirectory(fs.currentDirectory, 'foo')
        ..createSync();
      final Directory directoryB = fsUtils.getUniqueDirectory(fs.currentDirectory, 'foo');

      expect(directoryA.path, '/foo_01');
      expect(directoryB.path, '/foo_02');
    });
  });

  group('copyDirectorySync', () {
    /// Test file_systems.copyDirectorySync() using MemoryFileSystem.
    /// Copies between 2 instances of file systems which is also supported by copyDirectorySync().
    testWithoutContext('test directory copy', () async {
      final MemoryFileSystem sourceMemoryFs = MemoryFileSystem.test();
      const String sourcePath = '/some/origin';
      final Directory sourceDirectory = await sourceMemoryFs
          .directory(sourcePath)
          .create(recursive: true);
      sourceMemoryFs.currentDirectory = sourcePath;
      final File sourceFile1 = sourceMemoryFs.file('some_file.txt')..writeAsStringSync('bleh');
      final DateTime writeTime = sourceFile1.lastModifiedSync();
      sourceMemoryFs.file('sub_dir/another_file.txt').createSync(recursive: true);
      sourceMemoryFs.directory('empty_directory').createSync();

      // Copy to another memory file system instance.
      final MemoryFileSystem targetMemoryFs = MemoryFileSystem.test();
      const String targetPath = '/some/non-existent/target';
      final Directory targetDirectory = targetMemoryFs.directory(targetPath);

      copyDirectory(sourceDirectory, targetDirectory);

      expect(targetDirectory.existsSync(), true);
      targetMemoryFs.currentDirectory = targetPath;
      expect(targetMemoryFs.directory('empty_directory').existsSync(), true);
      expect(targetMemoryFs.file('sub_dir/another_file.txt').existsSync(), true);
      expect(targetMemoryFs.file('some_file.txt').readAsStringSync(), 'bleh');

      // Assert that the copy operation hasn't modified the original file in some way.
      expect(sourceMemoryFs.file('some_file.txt').lastModifiedSync(), writeTime);
      // There's still 3 things in the original directory as there were initially.
      expect(sourceMemoryFs.directory(sourcePath).listSync().length, 3);
    });

    testWithoutContext('test directory copy with followLinks: true', () async {
      final Signals signals = Signals.test();
      final LocalFileSystem fileSystem = LocalFileSystem.test(signals: signals);
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync(
        'flutter_copy_directory.',
      );
      try {
        final String sourcePath = io.Platform.isWindows ? r'some\origin' : 'some/origin';
        final Directory sourceDirectory = tempDir.childDirectory(sourcePath)
          ..createSync(recursive: true);
        final File sourceFile1 = sourceDirectory.childFile('some_file.txt')
          ..writeAsStringSync('file 1');
        sourceDirectory.childLink('absolute_linked.txt').createSync(sourceFile1.absolute.path);
        final DateTime writeTime = sourceFile1.lastModifiedSync();
        final Directory sourceSubDirectory = sourceDirectory
          .childDirectory('dir1')
          .childDirectory('dir2')..createSync(recursive: true);
        sourceSubDirectory.childFile('another_file.txt').writeAsStringSync('file 2');
        final String subdirectorySourcePath = io.Platform.isWindows ? r'dir1\dir2' : 'dir1/dir2';
        sourceDirectory.childLink('relative_linked_sub_dir').createSync(subdirectorySourcePath);
        sourceDirectory.childDirectory('empty_directory').createSync(recursive: true);

        final String targetPath =
            io.Platform.isWindows ? r'some\non-existent\target' : 'some/non-existent/target';
        final Directory targetDirectory = tempDir.childDirectory(targetPath);

        copyDirectory(sourceDirectory, targetDirectory);

        expect(targetDirectory.existsSync(), true);
        expect(targetDirectory.childFile('some_file.txt').existsSync(), true);
        expect(targetDirectory.childFile('some_file.txt').readAsStringSync(), 'file 1');
        expect(targetDirectory.childFile('absolute_linked.txt').readAsStringSync(), 'file 1');
        expect(targetDirectory.childLink('absolute_linked.txt').existsSync(), false);
        expect(targetDirectory.childDirectory('dir1').childDirectory('dir2').existsSync(), true);
        expect(
          targetDirectory
              .childDirectory('dir1')
              .childDirectory('dir2')
              .childFile('another_file.txt')
              .existsSync(),
          true,
        );
        expect(
          targetDirectory
              .childDirectory('dir1')
              .childDirectory('dir2')
              .childFile('another_file.txt')
              .readAsStringSync(),
          'file 2',
        );
        expect(targetDirectory.childDirectory('relative_linked_sub_dir').existsSync(), true);
        expect(targetDirectory.childLink('relative_linked_sub_dir').existsSync(), false);
        expect(
          targetDirectory
              .childDirectory('relative_linked_sub_dir')
              .childFile('another_file.txt')
              .existsSync(),
          true,
        );
        expect(
          targetDirectory
              .childDirectory('relative_linked_sub_dir')
              .childFile('another_file.txt')
              .readAsStringSync(),
          'file 2',
        );
        expect(targetDirectory.childDirectory('empty_directory').existsSync(), true);

        // Assert that the copy operation hasn't modified the original file in some way.
        expect(sourceDirectory.childFile('some_file.txt').lastModifiedSync(), writeTime);
        // There's still 5 things in the original directory as there were initially.
        expect(sourceDirectory.listSync().length, 5);
      } finally {
        tryToDelete(tempDir);
      }
    });

    testWithoutContext('test directory copy with followLinks: false', () async {
      final Signals signals = Signals.test();
      final LocalFileSystem fileSystem = LocalFileSystem.test(signals: signals);
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync(
        'flutter_copy_directory.',
      );
      try {
        final String sourcePath = io.Platform.isWindows ? r'some\origin' : 'some/origin';
        final Directory sourceDirectory = tempDir.childDirectory(sourcePath)
          ..createSync(recursive: true);
        final File sourceFile1 = sourceDirectory.childFile('some_file.txt')
          ..writeAsStringSync('file 1');
        sourceDirectory.childLink('absolute_linked.txt').createSync(sourceFile1.absolute.path);
        final DateTime writeTime = sourceFile1.lastModifiedSync();
        final Directory sourceSubDirectory = sourceDirectory
          .childDirectory('dir1')
          .childDirectory('dir2')..createSync(recursive: true);
        sourceSubDirectory.childFile('another_file.txt').writeAsStringSync('file 2');
        final String subdirectorySourcePath = io.Platform.isWindows ? r'dir1\dir2' : 'dir1/dir2';
        sourceDirectory.childLink('relative_linked_sub_dir').createSync(subdirectorySourcePath);
        sourceDirectory.childDirectory('empty_directory').createSync(recursive: true);

        final String targetPath =
            io.Platform.isWindows ? r'some\non-existent\target' : 'some/non-existent/target';
        final Directory targetDirectory = tempDir.childDirectory(targetPath);

        copyDirectory(sourceDirectory, targetDirectory, followLinks: false);

        expect(targetDirectory.existsSync(), true);
        expect(targetDirectory.childFile('some_file.txt').existsSync(), true);
        expect(targetDirectory.childFile('some_file.txt').readAsStringSync(), 'file 1');
        expect(targetDirectory.childFile('absolute_linked.txt').readAsStringSync(), 'file 1');
        expect(targetDirectory.childLink('absolute_linked.txt').existsSync(), true);
        expect(
          targetDirectory.childLink('absolute_linked.txt').targetSync(),
          sourceFile1.absolute.path,
        );
        expect(targetDirectory.childDirectory('dir1').childDirectory('dir2').existsSync(), true);
        expect(
          targetDirectory
              .childDirectory('dir1')
              .childDirectory('dir2')
              .childFile('another_file.txt')
              .existsSync(),
          true,
        );
        expect(
          targetDirectory
              .childDirectory('dir1')
              .childDirectory('dir2')
              .childFile('another_file.txt')
              .readAsStringSync(),
          'file 2',
        );
        expect(targetDirectory.childDirectory('relative_linked_sub_dir').existsSync(), true);
        expect(targetDirectory.childLink('relative_linked_sub_dir').existsSync(), true);
        expect(
          targetDirectory.childLink('relative_linked_sub_dir').targetSync(),
          subdirectorySourcePath,
        );
        expect(
          targetDirectory
              .childDirectory('relative_linked_sub_dir')
              .childFile('another_file.txt')
              .existsSync(),
          true,
        );
        expect(
          targetDirectory
              .childDirectory('relative_linked_sub_dir')
              .childFile('another_file.txt')
              .readAsStringSync(),
          'file 2',
        );
        expect(targetDirectory.childDirectory('empty_directory').existsSync(), true);

        // Assert that the copy operation hasn't modified the original file in some way.
        expect(sourceDirectory.childFile('some_file.txt').lastModifiedSync(), writeTime);
        // There's still 5 things in the original directory as there were initially.
        expect(sourceDirectory.listSync().length, 5);
      } finally {
        tryToDelete(tempDir);
      }
    });

    testWithoutContext('Skip files if shouldCopyFile returns false', () {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final Directory origin = fileSystem.directory('/origin');
      origin.createSync();
      fileSystem.file(fileSystem.path.join('origin', 'a.txt')).writeAsStringSync('irrelevant');
      fileSystem.directory('/origin/nested').createSync();
      fileSystem
          .file(fileSystem.path.join('origin', 'nested', 'a.txt'))
          .writeAsStringSync('irrelevant');
      fileSystem
          .file(fileSystem.path.join('origin', 'nested', 'b.txt'))
          .writeAsStringSync('irrelevant');

      final Directory destination = fileSystem.directory('/destination');
      copyDirectory(
        origin,
        destination,
        shouldCopyFile: (File origin, File dest) {
          return origin.basename == 'b.txt';
        },
      );

      expect(destination.existsSync(), isTrue);
      expect(destination.childDirectory('nested').existsSync(), isTrue);
      expect(destination.childDirectory('nested').childFile('b.txt').existsSync(), isTrue);

      expect(destination.childFile('a.txt').existsSync(), isFalse);
      expect(destination.childDirectory('nested').childFile('a.txt').existsSync(), isFalse);
    });

    testWithoutContext('Skip directories if shouldCopyDirectory returns false', () {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final Directory origin = fileSystem.directory('/origin');
      origin.createSync();
      fileSystem.file(fileSystem.path.join('origin', 'a.txt')).writeAsStringSync('irrelevant');
      fileSystem.directory('/origin/nested').createSync();
      fileSystem
          .file(fileSystem.path.join('origin', 'nested', 'a.txt'))
          .writeAsStringSync('irrelevant');
      fileSystem
          .file(fileSystem.path.join('origin', 'nested', 'b.txt'))
          .writeAsStringSync('irrelevant');

      final Directory destination = fileSystem.directory('/destination');
      copyDirectory(
        origin,
        destination,
        shouldCopyDirectory: (Directory directory) {
          return !directory.path.endsWith('nested');
        },
      );

      expect(destination, exists);
      expect(destination.childDirectory('nested'), isNot(exists));
      expect(destination.childDirectory('nested').childFile('b.txt'), isNot(exists));
    });
  });

  group('escapePath', () {
    testWithoutContext('on Windows', () {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final FileSystemUtils fsUtils = FileSystemUtils(
        fileSystem: fileSystem,
        platform: FakePlatform(operatingSystem: 'windows'),
      );
      expect(fsUtils.escapePath(r'C:\foo\bar\cool.dart'), r'C:\\foo\\bar\\cool.dart');
      expect(fsUtils.escapePath(r'foo\bar\cool.dart'), r'foo\\bar\\cool.dart');
      expect(fsUtils.escapePath('C:/foo/bar/cool.dart'), 'C:/foo/bar/cool.dart');
    });

    testWithoutContext('on Linux', () {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final FileSystemUtils fsUtils = FileSystemUtils(
        fileSystem: fileSystem,
        platform: FakePlatform(),
      );
      expect(fsUtils.escapePath('/foo/bar/cool.dart'), '/foo/bar/cool.dart');
      expect(fsUtils.escapePath('foo/bar/cool.dart'), 'foo/bar/cool.dart');
      expect(fsUtils.escapePath(r'foo\cool.dart'), r'foo\cool.dart');
    });
  });

  group('LocalFileSystem', () {
    late FakeProcessSignal fakeSignal;
    late ProcessSignal signalUnderTest;

    setUp(() {
      fakeSignal = FakeProcessSignal();
      signalUnderTest = ProcessSignal(fakeSignal);
    });

    testWithoutContext('runs shutdown hooks', () async {
      final Signals signals = Signals.test();
      final LocalFileSystem localFileSystem = LocalFileSystem.test(signals: signals);
      final Directory temp = localFileSystem.systemTempDirectory;

      expect(temp.existsSync(), isTrue);
      expect(localFileSystem.shutdownHooks.registeredHooks, hasLength(1));
      final BufferLogger logger = BufferLogger.test();
      await localFileSystem.shutdownHooks.runShutdownHooks(logger);
      expect(temp.existsSync(), isFalse);
      expect(logger.traceText, contains('Running 1 shutdown hook'));
    });

    testWithoutContext('deletes system temp entry on a fatal signal', () async {
      final Completer<void> completer = Completer<void>();
      final Signals signals = Signals.test();
      final LocalFileSystem localFileSystem = LocalFileSystem.test(
        signals: signals,
        fatalSignals: <ProcessSignal>[signalUnderTest],
      );
      final Directory temp = localFileSystem.systemTempDirectory;

      signals.addHandler(signalUnderTest, (ProcessSignal s) {
        completer.complete();
      });

      expect(temp.existsSync(), isTrue);

      fakeSignal.controller.add(fakeSignal);
      await completer.future;

      expect(temp.existsSync(), isFalse);
    });

    testWithoutContext('throwToolExit when temp not found', () async {
      final Signals signals = Signals.test();
      final LocalFileSystemFake localFileSystem = LocalFileSystemFake.test(signals: signals);

      try {
        localFileSystem.systemTempDirectory;
        fail('expected tool exit');
      } on ToolExit catch (e) {
        expect(
          e.message,
          'Your system temp directory (/does_not_exist) does not exist. '
          'Did you set an invalid override in your environment? '
          'See issue https://github.com/flutter/flutter/issues/74042 for more context.',
        );
      }
    });
  });
}

class FakeProcessSignal extends Fake implements io.ProcessSignal {
  final StreamController<io.ProcessSignal> controller = StreamController<io.ProcessSignal>();

  @override
  Stream<io.ProcessSignal> watch() => controller.stream;
}
