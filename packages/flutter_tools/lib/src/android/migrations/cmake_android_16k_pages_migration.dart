// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../project.dart';

/// Adds the snippet to the CMake file to compile for Android 15.
///
/// Location of CMakeLists.txt is the src/CMakeLists.txt in the plugin
/// created with --template plugin_ffi.
///
/// ```cmake
/// if (ANDROID)
///   # Support Android 15 16k page size.
///   target_link_options({{projectName}} PRIVATE "-Wl,-z,max-page-size=16384")
/// endif()
/// ```
class CmakeAndroid16kPagesMigration extends ProjectMigrator {
  CmakeAndroid16kPagesMigration(AndroidProject project, super.logger) : _project = project;

  final AndroidProject _project;

  @override
  Future<void> migrate() async {
    // If the migrator is run in the example directory, navigate to the
    // plugin directory which contains src/CMakeLists.txt.
    final File cmakeLists = _project.parent.directory.parent
        .childDirectory('src/')
        .childFile('CMakeLists.txt');

    if (!cmakeLists.existsSync()) {
      logger.printTrace(
        'CMake project not found, skipping support Android 15 16k page size migration.',
      );
      return;
    }

    final String original = cmakeLists.readAsStringSync();

    if (original.contains('-Wl,-z,max-page-size=16384')) {
      // Link flags already present.
      return;
    }

    final RegExp regex = RegExp(r'target_compile_definitions\(([^ ]*) PUBLIC DART_SHARED_LIB\)');
    final String? projectName = regex.firstMatch(original)?.group(1);
    const String before = '''
 PUBLIC DART_SHARED_LIB)
''';

    /// Relevant template: templates/plugin_ffi/src.tmpl/CMakeLists.txt.tmpl
    final String linkerFlags = '''

if (ANDROID)
  # Support Android 15 16k page size.
  target_link_options($projectName PRIVATE "-Wl,-z,max-page-size=16384")
endif()
''';

    final String updated = original.replaceFirst(before, '$before$linkerFlags');

    if (original != updated) {
      logger.printStatus('CMake missing support Android 15 16k page size, updating.');
      cmakeLists.writeAsStringSync(updated);
    }
  }
}
