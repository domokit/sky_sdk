// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/project_migrator.dart';
import '../cmake_project.dart';

// CMake's add_custom_command() should use VERBATIM to handle escaping of spaces
// and special characters correctly.
// See https://github.com/flutter/flutter/issues/67270.
class CmakeCustomCommandMigration extends ProjectMigrator {
  CmakeCustomCommandMigration(CmakeBasedProject project, super.logger)
    : _cmakeFile = project.managedCmakeFile;

  final File _cmakeFile;

  @override
  Future<void> migrate() async {
    if (!_cmakeFile.existsSync()) {
      logger.printTrace(
        'CMake project not found, skipping add_custom_command() VERBATIM migration',
      );
      return;
    }

    final String originalProjectContents = _cmakeFile.readAsStringSync();
    // Example:
    //
    // add_custom_command(
    //   OUTPUT ${FLUTTER_LIBRARY} ${FLUTTER_LIBRARY_HEADERS}
    //     ${CMAKE_CURRENT_BINARY_DIR}/_phony_
    //   COMMAND ${CMAKE_COMMAND} -E env
    //     ${FLUTTER_TOOL_ENVIRONMENT}
    //     "${FLUTTER_ROOT}/packages/flutter_tools/bin/tool_backend.sh"
    //       linux-x64 ${CMAKE_BUILD_TYPE}
    // )

    // Match the whole add_custom_command() and append VERBATIM unless it
    // already exists.
    final RegExp addCustomCommand = RegExp(
      r'add_custom_command\(\s*(.*?)\s*\)',
      multiLine: true,
      dotAll: true,
    );

    String newProjectContents = originalProjectContents;

    final Iterable<RegExpMatch> matches = addCustomCommand.allMatches(originalProjectContents);

    for (final RegExpMatch match in matches) {
      final String? addCustomCommandOriginal = match.group(1);
      if (addCustomCommandOriginal != null && !addCustomCommandOriginal.contains('VERBATIM')) {
        final String addCustomCommandReplacement = '$addCustomCommandOriginal\n  VERBATIM';
        newProjectContents = newProjectContents.replaceAll(
          addCustomCommandOriginal,
          addCustomCommandReplacement,
        );
      }

      // CMake's add_custom_command() should add FLUTTER_TARGET_PLATFORM to support multi-architecture.
      // However, developers would get the following warning every time if we do nothing.
      // ------------------------------
      // CMake Warning:
      //   Manually-specified variables were not used by the project:
      //    FLUTTER_TARGET_PLATFORM
      // ------------------------------
      if (addCustomCommandOriginal?.contains('linux-x64') ?? false) {
        newProjectContents = newProjectContents.replaceAll(
          'linux-x64',
          r'${FLUTTER_TARGET_PLATFORM}',
        );
      }
    }
    if (originalProjectContents != newProjectContents) {
      logger.printStatus(
        'add_custom_command() missing VERBATIM or FLUTTER_TARGET_PLATFORM, updating.',
      );
      _cmakeFile.writeAsStringSync(newProjectContents);
    }
  }
}
