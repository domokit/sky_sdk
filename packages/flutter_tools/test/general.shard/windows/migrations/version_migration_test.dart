// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cmake_project.dart';
import 'package:flutter_tools/src/windows/migrations/version_migration.dart';
import 'package:test/fake.dart';

import '../../../src/common.dart';

void main() {
  group('Windows Flutter version migration', () {
    late MemoryFileSystem memoryFileSystem;
    late BufferLogger testLogger;
    late FakeWindowsProject mockProject;
    late File cmakeFile;
    late File resourceFile;

    setUp(() {
      memoryFileSystem = MemoryFileSystem.test();
      cmakeFile = memoryFileSystem.file('CMakeLists.txt');
      resourceFile = memoryFileSystem.file('Runner.rc');

      testLogger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences.test(),
      );

      mockProject = FakeWindowsProject(cmakeFile, resourceFile);
    });

    testWithoutContext('skipped if CMake file is missing', () async {
      const String resourceFileContents = 'Hello world';

      resourceFile.writeAsStringSync(resourceFileContents);
      final VersionMigration migration = VersionMigration(mockProject, testLogger);
      await migration.migrate();
      expect(cmakeFile.existsSync(), isFalse);
      expect(resourceFile.existsSync(), isTrue);

      expect(
        testLogger.traceText,
        contains('windows/runner/CMakeLists.txt file not found, skipping version migration'),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if resource file is missing', () async {
      const String cmakeFileContents = 'Hello world';

      cmakeFile.writeAsStringSync(cmakeFileContents);
      final VersionMigration migration = VersionMigration(mockProject, testLogger);
      await migration.migrate();
      expect(cmakeFile.existsSync(), isTrue);
      expect(resourceFile.existsSync(), isFalse);

      expect(
        testLogger.traceText,
        contains('windows/runner/Runner.rc file not found, skipping version migration'),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if nothing to migrate', () async {
      const String cmakeFileContents = 'Nothing to migrate';
      const String resourceFileContents = 'Nothing to migrate';

      cmakeFile.writeAsStringSync(cmakeFileContents);
      resourceFile.writeAsStringSync(resourceFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();
      final DateTime resourceUpdatedAt = resourceFile.lastModifiedSync();

      final VersionMigration versionMigration = VersionMigration(mockProject, testLogger);
      await versionMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);
      expect(resourceFile.lastModifiedSync(), resourceUpdatedAt);
      expect(resourceFile.readAsStringSync(), resourceFileContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated', () async {
      const String cmakeFileContents =
          '# Apply the standard set of build settings. This can be removed for applications\n'
          '# that need different build settings.\n'
          'apply_standard_settings(\${BINARY_NAME})\n'
          '\n'
          '# Add preprocessor definitions for the build version.\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION=\\"\${FLUTTER_VERSION}\\"")\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MAJOR=\${FLUTTER_VERSION_MAJOR}")\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MINOR=\${FLUTTER_VERSION_MINOR}")\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_PATCH=\${FLUTTER_VERSION_PATCH}")\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_BUILD=\${FLUTTER_VERSION_BUILD}")\n'
          '\n'
          '# Disable Windows macros that collide with C++ standard library functions.\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\n';
      const String resourceFileContents =
          '#if defined(FLUTTER_VERSION_MAJOR) && defined(FLUTTER_VERSION_MINOR) && defined(FLUTTER_VERSION_PATCH) && defined(FLUTTER_VERSION_BUILD)\n'
          '#define VERSION_AS_NUMBER FLUTTER_VERSION_MAJOR,FLUTTER_VERSION_MINOR,FLUTTER_VERSION_PATCH,FLUTTER_VERSION_BUILD\n'
          '#else\n'
          '#define VERSION_AS_NUMBER 1,0,0,0\n'
          '#endif\n'
          '\n'
          '#if defined(FLUTTER_VERSION)\n'
          '#define VERSION_AS_STRING FLUTTER_VERSION\n'
          '#else\n'
          '#define VERSION_AS_STRING "1.0.0"\n'
          '#endif\n';

      cmakeFile.writeAsStringSync(cmakeFileContents);
      resourceFile.writeAsStringSync(resourceFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();
      final DateTime resourceUpdatedAt = resourceFile.lastModifiedSync();

      final VersionMigration versionMigration = VersionMigration(mockProject, testLogger);
      await versionMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);
      expect(resourceFile.lastModifiedSync(), resourceUpdatedAt);
      expect(resourceFile.readAsStringSync(), resourceFileContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated (CRLF)', () async {
      const String cmakeFileContents =
          '# Apply the standard set of build settings. This can be removed for applications\r\n'
          '# that need different build settings.\r\n'
          'apply_standard_settings(\${BINARY_NAME})\r\n'
          '\r\n'
          '# Add preprocessor definitions for the build version.\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION=\\"\${FLUTTER_VERSION}\\"")\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MAJOR=\${FLUTTER_VERSION_MAJOR}")\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MINOR=\${FLUTTER_VERSION_MINOR}")\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_PATCH=\${FLUTTER_VERSION_PATCH}")\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_BUILD=\${FLUTTER_VERSION_BUILD}")\r\n'
          '\r\n'
          '# Disable Windows macros that collide with C++ standard library functions.\r\n'
          'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\r\n';
      const String resourceFileContents =
          '#if defined(FLUTTER_VERSION_MAJOR) && defined(FLUTTER_VERSION_MINOR) && defined(FLUTTER_VERSION_PATCH) && defined(FLUTTER_VERSION_BUILD)\r\n'
          '#define VERSION_AS_NUMBER FLUTTER_VERSION_MAJOR,FLUTTER_VERSION_MINOR,FLUTTER_VERSION_PATCH,FLUTTER_VERSION_BUILD\r\n'
          '#else\r\n'
          '#define VERSION_AS_NUMBER 1,0,0,0\r\n'
          '#endif\r\n'
          '\r\n'
          '#if defined(FLUTTER_VERSION)\r\n'
          '#define VERSION_AS_STRING FLUTTER_VERSION\r\n'
          '#else\r\n'
          '#define VERSION_AS_STRING "1.0.0"\r\n'
          '#endif\r\n';

      cmakeFile.writeAsStringSync(cmakeFileContents);
      resourceFile.writeAsStringSync(resourceFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();
      final DateTime resourceUpdatedAt = resourceFile.lastModifiedSync();

      final VersionMigration versionMigration = VersionMigration(mockProject, testLogger);
      await versionMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);
      expect(resourceFile.lastModifiedSync(), resourceUpdatedAt);
      expect(resourceFile.readAsStringSync(), resourceFileContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('migrates project to set version information', () async {
      cmakeFile.writeAsStringSync(
        '# Apply the standard set of build settings. This can be removed for applications\n'
        '# that need different build settings.\n'
        'apply_standard_settings(\${BINARY_NAME})\n'
        '\n'
        '# Disable Windows macros that collide with C++ standard library functions.\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\n',
      );
      resourceFile.writeAsStringSync(
        '#ifdef FLUTTER_BUILD_NUMBER\n'
        '#define VERSION_AS_NUMBER FLUTTER_BUILD_NUMBER\n'
        '#else\n'
        '#define VERSION_AS_NUMBER 1,0,0\n'
        '#endif\n'
        '\n'
        '#ifdef FLUTTER_BUILD_NAME\n'
        '#define VERSION_AS_STRING #FLUTTER_BUILD_NAME\n'
        '#else\n'
        '#define VERSION_AS_STRING "1.0.0"\n'
        '#endif\n',
      );

      final VersionMigration versionMigration = VersionMigration(mockProject, testLogger);
      await versionMigration.migrate();

      expect(
        cmakeFile.readAsStringSync(),
        '# Apply the standard set of build settings. This can be removed for applications\n'
        '# that need different build settings.\n'
        'apply_standard_settings(\${BINARY_NAME})\n'
        '\n'
        '# Add preprocessor definitions for the build version.\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION=\\"\${FLUTTER_VERSION}\\"")\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MAJOR=\${FLUTTER_VERSION_MAJOR}")\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MINOR=\${FLUTTER_VERSION_MINOR}")\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_PATCH=\${FLUTTER_VERSION_PATCH}")\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_BUILD=\${FLUTTER_VERSION_BUILD}")\n'
        '\n'
        '# Disable Windows macros that collide with C++ standard library functions.\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\n',
      );
      expect(
        resourceFile.readAsStringSync(),
        '#if defined(FLUTTER_VERSION_MAJOR) && defined(FLUTTER_VERSION_MINOR) && defined(FLUTTER_VERSION_PATCH) && defined(FLUTTER_VERSION_BUILD)\n'
        '#define VERSION_AS_NUMBER FLUTTER_VERSION_MAJOR,FLUTTER_VERSION_MINOR,FLUTTER_VERSION_PATCH,FLUTTER_VERSION_BUILD\n'
        '#else\n'
        '#define VERSION_AS_NUMBER 1,0,0,0\n'
        '#endif\n'
        '\n'
        '#if defined(FLUTTER_VERSION)\n'
        '#define VERSION_AS_STRING FLUTTER_VERSION\n'
        '#else\n'
        '#define VERSION_AS_STRING "1.0.0"\n'
        '#endif\n',
      );

      expect(
        testLogger.statusText,
        contains('windows/runner/CMakeLists.txt does not define version information, updating.'),
      );
      expect(
        testLogger.statusText,
        contains('windows/runner/Runner.rc does not use Flutter version information, updating.'),
      );
    });

    testWithoutContext('migrates project to set version information (CRLF)', () async {
      cmakeFile.writeAsStringSync(
        '# Apply the standard set of build settings. This can be removed for applications\r\n'
        '# that need different build settings.\r\n'
        'apply_standard_settings(\${BINARY_NAME})\r\n'
        '\r\n'
        '# Disable Windows macros that collide with C++ standard library functions.\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\r\n',
      );
      resourceFile.writeAsStringSync(
        '#ifdef FLUTTER_BUILD_NUMBER\r\n'
        '#define VERSION_AS_NUMBER FLUTTER_BUILD_NUMBER\r\n'
        '#else\r\n'
        '#define VERSION_AS_NUMBER 1,0,0\r\n'
        '#endif\r\n'
        '\r\n'
        '#ifdef FLUTTER_BUILD_NAME\r\n'
        '#define VERSION_AS_STRING #FLUTTER_BUILD_NAME\r\n'
        '#else\r\n'
        '#define VERSION_AS_STRING "1.0.0"\r\n'
        '#endif\r\n',
      );

      final VersionMigration versionMigration = VersionMigration(mockProject, testLogger);
      await versionMigration.migrate();

      expect(
        cmakeFile.readAsStringSync(),
        '# Apply the standard set of build settings. This can be removed for applications\r\n'
        '# that need different build settings.\r\n'
        'apply_standard_settings(\${BINARY_NAME})\r\n'
        '\r\n'
        '# Add preprocessor definitions for the build version.\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION=\\"\${FLUTTER_VERSION}\\"")\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MAJOR=\${FLUTTER_VERSION_MAJOR}")\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MINOR=\${FLUTTER_VERSION_MINOR}")\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_PATCH=\${FLUTTER_VERSION_PATCH}")\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "FLUTTER_VERSION_BUILD=\${FLUTTER_VERSION_BUILD}")\r\n'
        '\r\n'
        '# Disable Windows macros that collide with C++ standard library functions.\r\n'
        'target_compile_definitions(\${BINARY_NAME} PRIVATE "NOMINMAX")\r\n',
      );
      expect(
        resourceFile.readAsStringSync(),
        '#if defined(FLUTTER_VERSION_MAJOR) && defined(FLUTTER_VERSION_MINOR) && defined(FLUTTER_VERSION_PATCH) && defined(FLUTTER_VERSION_BUILD)\r\n'
        '#define VERSION_AS_NUMBER FLUTTER_VERSION_MAJOR,FLUTTER_VERSION_MINOR,FLUTTER_VERSION_PATCH,FLUTTER_VERSION_BUILD\r\n'
        '#else\r\n'
        '#define VERSION_AS_NUMBER 1,0,0,0\r\n'
        '#endif\r\n'
        '\r\n'
        '#if defined(FLUTTER_VERSION)\r\n'
        '#define VERSION_AS_STRING FLUTTER_VERSION\r\n'
        '#else\r\n'
        '#define VERSION_AS_STRING "1.0.0"\r\n'
        '#endif\r\n',
      );

      expect(
        testLogger.statusText,
        contains('windows/runner/CMakeLists.txt does not define version information, updating.'),
      );
      expect(
        testLogger.statusText,
        contains('windows/runner/Runner.rc does not use Flutter version information, updating.'),
      );
    });
  });
}

class FakeWindowsProject extends Fake implements WindowsProject {
  FakeWindowsProject(this.runnerCmakeFile, this.runnerResourceFile);

  @override
  final File runnerCmakeFile;

  @override
  final File runnerResourceFile;
}
