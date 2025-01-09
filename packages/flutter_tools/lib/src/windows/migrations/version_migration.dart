// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../cmake_project.dart';
import 'utils.dart';

const String _cmakeFileBefore = r'''
# Apply the standard set of build settings. This can be removed for applications
# that need different build settings.
apply_standard_settings(${BINARY_NAME})

# Disable Windows macros that collide with C++ standard library functions.
target_compile_definitions(${BINARY_NAME} PRIVATE "NOMINMAX")
''';
const String _cmakeFileAfter = r'''
# Apply the standard set of build settings. This can be removed for applications
# that need different build settings.
apply_standard_settings(${BINARY_NAME})

# Add preprocessor definitions for the build version.
target_compile_definitions(${BINARY_NAME} PRIVATE "FLUTTER_VERSION=\"${FLUTTER_VERSION}\"")
target_compile_definitions(${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MAJOR=${FLUTTER_VERSION_MAJOR}")
target_compile_definitions(${BINARY_NAME} PRIVATE "FLUTTER_VERSION_MINOR=${FLUTTER_VERSION_MINOR}")
target_compile_definitions(${BINARY_NAME} PRIVATE "FLUTTER_VERSION_PATCH=${FLUTTER_VERSION_PATCH}")
target_compile_definitions(${BINARY_NAME} PRIVATE "FLUTTER_VERSION_BUILD=${FLUTTER_VERSION_BUILD}")

# Disable Windows macros that collide with C++ standard library functions.
target_compile_definitions(${BINARY_NAME} PRIVATE "NOMINMAX")
''';

const String _resourceFileBefore = '''
#ifdef FLUTTER_BUILD_NUMBER
#define VERSION_AS_NUMBER FLUTTER_BUILD_NUMBER
#else
#define VERSION_AS_NUMBER 1,0,0
#endif

#ifdef FLUTTER_BUILD_NAME
#define VERSION_AS_STRING #FLUTTER_BUILD_NAME
#else
#define VERSION_AS_STRING "1.0.0"
#endif
''';
const String _resourceFileAfter = '''
#if defined(FLUTTER_VERSION_MAJOR) && defined(FLUTTER_VERSION_MINOR) && defined(FLUTTER_VERSION_PATCH) && defined(FLUTTER_VERSION_BUILD)
#define VERSION_AS_NUMBER FLUTTER_VERSION_MAJOR,FLUTTER_VERSION_MINOR,FLUTTER_VERSION_PATCH,FLUTTER_VERSION_BUILD
#else
#define VERSION_AS_NUMBER 1,0,0,0
#endif

#if defined(FLUTTER_VERSION)
#define VERSION_AS_STRING FLUTTER_VERSION
#else
#define VERSION_AS_STRING "1.0.0"
#endif
''';

/// Migrates Windows apps to set Flutter's version information.
///
/// See https://github.com/flutter/flutter/issues/73652.
class VersionMigration extends ProjectMigrator {
  VersionMigration(WindowsProject project, super.logger)
    : _cmakeFile = project.runnerCmakeFile,
      _resourceFile = project.runnerResourceFile;

  final File _cmakeFile;
  final File _resourceFile;

  @override
  Future<void> migrate() async {
    // Skip this migration if the affected files do not exist. This indicates
    // the app has done non-trivial changes to its runner and this migration
    // might not work as expected if applied.
    if (!_cmakeFile.existsSync()) {
      logger.printTrace('''
windows/runner/CMakeLists.txt file not found, skipping version migration.

This indicates non-trivial changes have been made to the Windows runner in the
"windows" folder. If needed, you can reset the Windows runner by deleting the
"windows" folder and then using the "flutter create --platforms=windows ." command.
''');
      return;
    }

    if (!_resourceFile.existsSync()) {
      logger.printTrace('''
windows/runner/Runner.rc file not found, skipping version migration.

This indicates non-trivial changes have been made to the Windows runner in the
"windows" folder. If needed, you can reset the Windows runner by deleting the
"windows" folder and then using the "flutter create --platforms=windows ." command.
''');
      return;
    }

    // Migrate the windows/runner/CMakeLists.txt file.
    final String originalCmakeContents = _cmakeFile.readAsStringSync();
    final String newCmakeContents = replaceFirst(
      originalCmakeContents,
      _cmakeFileBefore,
      _cmakeFileAfter,
    );
    if (originalCmakeContents != newCmakeContents) {
      logger.printStatus(
        'windows/runner/CMakeLists.txt does not define version information, updating.',
      );
      _cmakeFile.writeAsStringSync(newCmakeContents);
    }

    // Migrate the windows/runner/Runner.rc file.
    final String originalResourceFileContents = _resourceFile.readAsStringSync();
    final String newResourceFileContents = replaceFirst(
      originalResourceFileContents,
      _resourceFileBefore,
      _resourceFileAfter,
    );
    if (originalResourceFileContents != newResourceFileContents) {
      logger.printStatus(
        'windows/runner/Runner.rc does not use Flutter version information, updating.',
      );
      _resourceFile.writeAsStringSync(newResourceFileContents);
    }
  }
}
