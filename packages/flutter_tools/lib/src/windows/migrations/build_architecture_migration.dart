// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../cmake_project.dart';
import 'utils.dart';

const String _cmakeFileToolBackendBefore = r'''
add_custom_command(
  OUTPUT ${FLUTTER_LIBRARY} ${FLUTTER_LIBRARY_HEADERS}
    ${CPP_WRAPPER_SOURCES_CORE} ${CPP_WRAPPER_SOURCES_PLUGIN}
    ${CPP_WRAPPER_SOURCES_APP}
    ${PHONY_OUTPUT}
  COMMAND ${CMAKE_COMMAND} -E env
    ${FLUTTER_TOOL_ENVIRONMENT}
    "${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"
      windows-x64 $<CONFIG>
  VERBATIM
)
''';

const String _cmakeFileToolBackendAfter = r'''
add_custom_command(
  OUTPUT ${FLUTTER_LIBRARY} ${FLUTTER_LIBRARY_HEADERS}
    ${CPP_WRAPPER_SOURCES_CORE} ${CPP_WRAPPER_SOURCES_PLUGIN}
    ${CPP_WRAPPER_SOURCES_APP}
    ${PHONY_OUTPUT}
  COMMAND ${CMAKE_COMMAND} -E env
    ${FLUTTER_TOOL_ENVIRONMENT}
    "${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.bat"
      ${FLUTTER_TARGET_PLATFORM} $<CONFIG>
  VERBATIM
)
''';

const String _cmakeFileTargetPlatformBefore = r'''
# TODO: Move the rest of this into files in ephemeral. See
# https://github.com/flutter/flutter/issues/57146.
set(WRAPPER_ROOT "${EPHEMERAL_DIR}/cpp_client_wrapper")

# === Flutter Library ===
''';

const String _cmakeFileTargetPlatformAfter = r'''
# TODO: Move the rest of this into files in ephemeral. See
# https://github.com/flutter/flutter/issues/57146.
set(WRAPPER_ROOT "${EPHEMERAL_DIR}/cpp_client_wrapper")

# Set fallback configurations for older versions of the flutter tool.
if (NOT DEFINED FLUTTER_TARGET_PLATFORM)
  set(FLUTTER_TARGET_PLATFORM "windows-x64")
endif()

# === Flutter Library ===
''';

/// Migrates Windows build to target specific architecture.
/// In more, it deletes old runner folder
class BuildArchitectureMigration extends ProjectMigrator {
  BuildArchitectureMigration(WindowsProject project, Directory buildDirectory, super.logger)
    : _cmakeFile = project.managedCmakeFile,
      _buildDirectory = buildDirectory;

  final File _cmakeFile;
  final Directory _buildDirectory;

  @override
  Future<void> migrate() async {
    final Directory oldRunnerDirectory = _buildDirectory.parent.childDirectory('runner');
    if (oldRunnerDirectory.existsSync()) {
      logger.printTrace('''
Deleting previous build folder ${oldRunnerDirectory.path}.
New binaries can be found in ${_buildDirectory.childDirectory('runner').path}.
''');
      try {
        oldRunnerDirectory.deleteSync(recursive: true);
      } on FileSystemException catch (error) {
        logger.printError(
          'Failed to remove ${oldRunnerDirectory.path}: $error. '
          'A program may still be using a file in the directory or the directory itself. '
          'To find and stop such a program, see: '
          'https://superuser.com/questions/1333118/cant-delete-empty-folder-because-it-is-used',
        );
      }
    }

    // Skip this migration if the affected file does not exist. This indicates
    // the app has done non-trivial changes to its runner and this migration
    // might not work as expected if applied.
    if (!_cmakeFile.existsSync()) {
      logger.printTrace('''
windows/flutter/CMakeLists.txt file not found, skipping build architecture migration.

This indicates non-trivial changes have been made to the "windows" folder.
If needed, you can reset it by deleting the "windows" folder and then using the
"flutter create --platforms=windows ." command.
''');
      return;
    }

    // Migrate the windows/flutter/CMakeLists.txt file.
    final String originalCmakeContents = _cmakeFile.readAsStringSync();
    final String cmakeContentsWithTargetPlatform = replaceFirst(
      originalCmakeContents,
      _cmakeFileTargetPlatformBefore,
      _cmakeFileTargetPlatformAfter,
    );
    final String newCmakeContents = replaceFirst(
      cmakeContentsWithTargetPlatform,
      _cmakeFileToolBackendBefore,
      _cmakeFileToolBackendAfter,
    );
    if (originalCmakeContents != newCmakeContents) {
      logger.printStatus(
        'windows/flutter/CMakeLists.txt does not use FLUTTER_TARGET_PLATFORM, updating.',
      );
      _cmakeFile.writeAsStringSync(newCmakeContents);
    }
  }
}
