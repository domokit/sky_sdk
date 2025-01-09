// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cmake_project.dart';
import 'package:flutter_tools/src/windows/migrations/build_architecture_migration.dart';
import 'package:test/fake.dart';

import '../../../src/common.dart';

void main() {
  group('Windows Flutter build architecture migration', () {
    late MemoryFileSystem memoryFileSystem;
    late BufferLogger testLogger;
    late FakeWindowsProject mockProject;
    late File cmakeFile;
    late Directory buildDirectory;

    setUp(() {
      memoryFileSystem = MemoryFileSystem.test();
      cmakeFile = memoryFileSystem.file('CMakeLists.txt');
      buildDirectory = memoryFileSystem.directory('x64');

      testLogger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences.test(),
      );

      mockProject = FakeWindowsProject(cmakeFile);
    });

    testWithoutContext('delete old runner directory', () async {
      buildDirectory.createSync();
      final Directory oldRunnerDirectory = buildDirectory.parent.childDirectory('runner');
      oldRunnerDirectory.createSync();
      final File executable = oldRunnerDirectory.childFile('program.exe');
      executable.createSync();
      expect(oldRunnerDirectory.existsSync(), isTrue);

      final BuildArchitectureMigration migration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await migration.migrate();

      expect(oldRunnerDirectory.existsSync(), isFalse);
      expect(
        testLogger.traceText,
        contains(
          'Deleting previous build folder ./runner.\n'
          'New binaries can be found in x64/runner.\n',
        ),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if CMake file is missing', () async {
      final BuildArchitectureMigration migration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await migration.migrate();
      expect(cmakeFile.existsSync(), isFalse);

      expect(
        testLogger.traceText,
        contains(
          'windows/flutter/CMakeLists.txt file not found, skipping build architecture migration',
        ),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if nothing to migrate', () async {
      const String cmakeFileContents = 'Nothing to migrate';

      cmakeFile.writeAsStringSync(cmakeFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();

      final BuildArchitectureMigration buildArchitectureMigration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await buildArchitectureMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated', () async {
      const String cmakeFileContents =
          '# TODO: Move the rest of this into files in ephemeral. See\n'
          '# https://github.com/flutter/flutter/issues/57146.\n'
          'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\n'
          '\n'
          '# Set fallback configurations for older versions of the flutter tool.\n'
          'if (NOT DEFINED FLUTTER_TARGET_PLATFORM)\n'
          '  set(FLUTTER_TARGET_PLATFORM "windows-x64")\n'
          'endif()\n'
          '\n'
          '# === Flutter Library ===\n'
          '...\n'
          'add_custom_command(\n'
          '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\n'
          '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\n'
          '    \${CPP_WRAPPER_SOURCES_APP}\n'
          '    \${PHONY_OUTPUT}\n'
          '  COMMAND \${CMAKE_COMMAND} -E env\n'
          '    \${FLUTTER_TOOL_ENVIRONMENT}\n'
          '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\n'
          '      \${FLUTTER_TARGET_PLATFORM} \$<CONFIG>\n'
          '  VERBATIM\n'
          ')\n';

      cmakeFile.writeAsStringSync(cmakeFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();

      final BuildArchitectureMigration buildArchitectureMigration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await buildArchitectureMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated (CRLF)', () async {
      const String cmakeFileContents =
          '# TODO: Move the rest of this into files in ephemeral. See\r\n'
          '# https://github.com/flutter/flutter/issues/57146.\r\n'
          'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\r\n'
          '\r\n'
          '# Set fallback configurations for older versions of the flutter tool.\r\n'
          'if (NOT DEFINED FLUTTER_TARGET_PLATFORM)\r\n'
          '  set(FLUTTER_TARGET_PLATFORM "windows-x64")\r\n'
          'endif()\r\n'
          '\r\n'
          '# === Flutter Library ===\r\n'
          '...\r\n'
          'add_custom_command(\r\n'
          '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\r\n'
          '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\r\n'
          '    \${CPP_WRAPPER_SOURCES_APP}\r\n'
          '    \${PHONY_OUTPUT}\r\n'
          '  COMMAND \${CMAKE_COMMAND} -E env\r\n'
          '    \${FLUTTER_TOOL_ENVIRONMENT}\r\n'
          '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\r\n'
          '      \${FLUTTER_TARGET_PLATFORM} \$<CONFIG>\r\n'
          '  VERBATIM\r\n'
          ')\r\n';

      cmakeFile.writeAsStringSync(cmakeFileContents);

      final DateTime cmakeUpdatedAt = cmakeFile.lastModifiedSync();

      final BuildArchitectureMigration buildArchitectureMigration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await buildArchitectureMigration.migrate();

      expect(cmakeFile.lastModifiedSync(), cmakeUpdatedAt);
      expect(cmakeFile.readAsStringSync(), cmakeFileContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('migrates project to set the target platform', () async {
      cmakeFile.writeAsStringSync(
        '# TODO: Move the rest of this into files in ephemeral. See\n'
        '# https://github.com/flutter/flutter/issues/57146.\n'
        'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\n'
        '\n'
        '# === Flutter Library ===\n'
        '...\n'
        'add_custom_command(\n'
        '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\n'
        '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\n'
        '    \${CPP_WRAPPER_SOURCES_APP}\n'
        '    \${PHONY_OUTPUT}\n'
        '  COMMAND \${CMAKE_COMMAND} -E env\n'
        '    \${FLUTTER_TOOL_ENVIRONMENT}\n'
        '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\n'
        '      windows-x64 \$<CONFIG>\n'
        '  VERBATIM\n'
        ')\n',
      );
      final BuildArchitectureMigration buildArchitectureMigration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await buildArchitectureMigration.migrate();

      expect(
        cmakeFile.readAsStringSync(),
        '# TODO: Move the rest of this into files in ephemeral. See\n'
        '# https://github.com/flutter/flutter/issues/57146.\n'
        'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\n'
        '\n'
        '# Set fallback configurations for older versions of the flutter tool.\n'
        'if (NOT DEFINED FLUTTER_TARGET_PLATFORM)\n'
        '  set(FLUTTER_TARGET_PLATFORM "windows-x64")\n'
        'endif()\n'
        '\n'
        '# === Flutter Library ===\n'
        '...\n'
        'add_custom_command(\n'
        '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\n'
        '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\n'
        '    \${CPP_WRAPPER_SOURCES_APP}\n'
        '    \${PHONY_OUTPUT}\n'
        '  COMMAND \${CMAKE_COMMAND} -E env\n'
        '    \${FLUTTER_TOOL_ENVIRONMENT}\n'
        '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\n'
        '      \${FLUTTER_TARGET_PLATFORM} \$<CONFIG>\n'
        '  VERBATIM\n'
        ')\n',
      );

      expect(
        testLogger.statusText,
        contains('windows/flutter/CMakeLists.txt does not use FLUTTER_TARGET_PLATFORM, updating.'),
      );
    });

    testWithoutContext('migrates project to set the target platform (CRLF)', () async {
      cmakeFile.writeAsStringSync(
        '# TODO: Move the rest of this into files in ephemeral. See\r\n'
        '# https://github.com/flutter/flutter/issues/57146.\r\n'
        'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\r\n'
        '\r\n'
        '# === Flutter Library ===\r\n'
        '...\r\n'
        'add_custom_command(\r\n'
        '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\r\n'
        '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\r\n'
        '    \${CPP_WRAPPER_SOURCES_APP}\r\n'
        '    \${PHONY_OUTPUT}\r\n'
        '  COMMAND \${CMAKE_COMMAND} -E env\r\n'
        '    \${FLUTTER_TOOL_ENVIRONMENT}\r\n'
        '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\r\n'
        '      windows-x64 \$<CONFIG>\r\n'
        '  VERBATIM\r\n'
        ')\r\n',
      );

      final BuildArchitectureMigration buildArchitectureMigration = BuildArchitectureMigration(
        mockProject,
        buildDirectory,
        testLogger,
      );
      await buildArchitectureMigration.migrate();

      expect(
        cmakeFile.readAsStringSync(),
        '# TODO: Move the rest of this into files in ephemeral. See\r\n'
        '# https://github.com/flutter/flutter/issues/57146.\r\n'
        'set(WRAPPER_ROOT "\${EPHEMERAL_DIR}/cpp_client_wrapper")\r\n'
        '\r\n'
        '# Set fallback configurations for older versions of the flutter tool.\r\n'
        'if (NOT DEFINED FLUTTER_TARGET_PLATFORM)\r\n'
        '  set(FLUTTER_TARGET_PLATFORM "windows-x64")\r\n'
        'endif()\r\n'
        '\r\n'
        '# === Flutter Library ===\r\n'
        '...\r\n'
        'add_custom_command(\r\n'
        '  OUTPUT \${FLUTTER_LIBRARY} \${FLUTTER_LIBRARY_HEADERS}\r\n'
        '    \${CPP_WRAPPER_SOURCES_CORE} \${CPP_WRAPPER_SOURCES_PLUGIN}\r\n'
        '    \${CPP_WRAPPER_SOURCES_APP}\r\n'
        '    \${PHONY_OUTPUT}\r\n'
        '  COMMAND \${CMAKE_COMMAND} -E env\r\n'
        '    \${FLUTTER_TOOL_ENVIRONMENT}\r\n'
        '    "\${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"\r\n'
        '      \${FLUTTER_TARGET_PLATFORM} \$<CONFIG>\r\n'
        '  VERBATIM\r\n'
        ')\r\n',
      );

      expect(
        testLogger.statusText,
        contains('windows/flutter/CMakeLists.txt does not use FLUTTER_TARGET_PLATFORM, updating.'),
      );
    });
  });
}

class FakeWindowsProject extends Fake implements WindowsProject {
  FakeWindowsProject(this.managedCmakeFile);

  @override
  final File managedCmakeFile;
}
