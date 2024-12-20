// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/windows/application_package.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  group('PrebuiltWindowsApp', () {
    late FakeOperatingSystemUtils os;
    late FileSystem fileSystem;
    late BufferLogger logger;

    final Map<Type, Generator> overrides = <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      OperatingSystemUtils: () => os,
      Logger: () => logger,
    };

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      os = FakeOperatingSystemUtils();
      logger = BufferLogger.test();
    });

    testUsingContext('Error on non-existing exe file', () {
      final PrebuiltWindowsApp? windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('not_existing.exe')) as PrebuiltWindowsApp?;

      expect(windowsApp, isNull);
      expect(logger.errorText, contains('File "not_existing.exe" does not exist.'));
    }, overrides: overrides);

    testUsingContext('Success on exe file', () {
      fileSystem.file('file.exe').createSync();
      final PrebuiltWindowsApp windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('file.exe'))! as PrebuiltWindowsApp;

      expect(windowsApp.name, 'file.exe');
    }, overrides: overrides);

    testUsingContext('Error on non-existing zip file', () {
      final PrebuiltWindowsApp? windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('not_existing.zip')) as PrebuiltWindowsApp?;

      expect(windowsApp, isNull);
      expect(logger.errorText, contains('File "not_existing.zip" does not exist.'));
    }, overrides: overrides);

    testUsingContext('Bad zipped app, no payload dir', () {
      fileSystem.file('app.zip').createSync();
      final PrebuiltWindowsApp? windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('app.zip')) as PrebuiltWindowsApp?;

      expect(windowsApp, isNull);
      expect(logger.errorText, contains('Cannot find .exe files in the zip archive.'));
    }, overrides: overrides);

    testUsingContext('Bad zipped app, two .exe files', () {
      fileSystem.file('app.zip').createSync();
      os.unzipOverride = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.zip') {
          return;
        }
        final String exePath1 = fileSystem.path.join(targetDirectory.path, 'app1.exe');
        final String exePath2 = fileSystem.path.join(targetDirectory.path, 'app2.exe');
        fileSystem.directory(exePath1).createSync(recursive: true);
        fileSystem.directory(exePath2).createSync(recursive: true);
      };
      final PrebuiltWindowsApp? windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('app.zip')) as PrebuiltWindowsApp?;

      expect(windowsApp, isNull);
      expect(logger.errorText, contains('Archive "app.zip" contains more than one .exe files.'));
    }, overrides: overrides);

    testUsingContext('Success with zipped app', () {
      fileSystem.file('app.zip').createSync();
      String? exePath;
      os.unzipOverride = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.zip') {
          return;
        }
        exePath = fileSystem.path.join(targetDirectory.path, 'app.exe');
        fileSystem.directory(exePath).createSync(recursive: true);
      };
      final PrebuiltWindowsApp windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('app.zip'))! as PrebuiltWindowsApp;

      expect(logger.errorText, isEmpty);
      expect(windowsApp.name, exePath);
      expect(windowsApp.applicationPackage.path, 'app.zip');
    }, overrides: overrides);

    testUsingContext('Error on unknown file type', () {
      fileSystem.file('not_existing.app').createSync();
      final PrebuiltWindowsApp? windowsApp =
          WindowsApp.fromPrebuiltApp(fileSystem.file('not_existing.app')) as PrebuiltWindowsApp?;

      expect(windowsApp, isNull);
      expect(logger.errorText, contains('Unknown windows application type.'));
    }, overrides: overrides);
  });
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils();

  void Function(File, Directory)? unzipOverride;

  @override
  void unzip(File file, Directory targetDirectory) {
    unzipOverride?.call(file, targetDirectory);
  }
}
