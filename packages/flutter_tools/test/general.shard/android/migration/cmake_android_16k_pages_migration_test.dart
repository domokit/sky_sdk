// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/migrations/cmake_android_16k_pages_migration.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../../src/common.dart';

const String _sampleCmakeListsTxtUnmigrated = r'''
# The Flutter tooling requires that developers have CMake 3.10 or later
# installed. You should not increase this version, as doing so will cause
# the plugin to fail to compile for some customers of the plugin.
cmake_minimum_required(VERSION 3.10)

project(my_plugin_library VERSION 0.0.1 LANGUAGES C)

add_library(my_plugin SHARED
  "my_plugin.c"
)

set_target_properties(my_plugin PROPERTIES
  PUBLIC_HEADER my_plugin.h
  OUTPUT_NAME "my_plugin"
)

target_compile_definitions(my_plugin PUBLIC DART_SHARED_LIB)
''';

const String _sampleCmakeListsTxtMigrated = r'''
# The Flutter tooling requires that developers have CMake 3.10 or later
# installed. You should not increase this version, as doing so will cause
# the plugin to fail to compile for some customers of the plugin.
cmake_minimum_required(VERSION 3.10)

project(my_plugin_library VERSION 0.0.1 LANGUAGES C)

add_library(my_plugin SHARED
  "my_plugin.c"
)

set_target_properties(my_plugin PROPERTIES
  PUBLIC_HEADER my_plugin.h
  OUTPUT_NAME "my_plugin"
)

target_compile_definitions(my_plugin PUBLIC DART_SHARED_LIB)

if (ANDROID)
  # Support Android 15 16k page size.
  target_link_options(my_plugin PRIVATE "-Wl,-z,max-page-size=16384")
endif()
''';

void main() {
  group('Android migration', () {
    group('CMake file', () {
      late MemoryFileSystem memoryFileSystem;
      late File cmakeFile;
      late BufferLogger bufferLogger;
      late FakeAndroidProject project;
      late CmakeAndroid16kPagesMigration migration;

      setUp(() {
        memoryFileSystem = MemoryFileSystem.test();
        final Directory pluginDir = memoryFileSystem.currentDirectory;
        final Directory exampleDir = pluginDir.childDirectory('example');
        exampleDir.createSync();
        final Directory androidDir = exampleDir.childDirectory('android');
        androidDir.createSync();
        final Directory srcDir = pluginDir.childDirectory('src');
        srcDir.createSync();
        cmakeFile = srcDir.childFile('CMakeLists.txt');
        cmakeFile.writeAsString(_sampleCmakeListsTxtMigrated);

        bufferLogger = BufferLogger.test();
        project = FakeAndroidProject(parent: FakeFlutterProject(directory: exampleDir));
        migration = CmakeAndroid16kPagesMigration(project, bufferLogger);
      });

      testWithoutContext('do nothing when files missing', () async {
        cmakeFile.deleteSync();
        await migration.migrate();
        expect(
          bufferLogger.traceText,
          contains('CMake project not found, skipping support Android 15 16k page size migration.'),
        );
      });

      testWithoutContext('migrate', () async {
        cmakeFile.writeAsStringSync(_sampleCmakeListsTxtUnmigrated);
        await migration.migrate();
        expect(cmakeFile.readAsStringSync(), _sampleCmakeListsTxtMigrated);
      });

      testWithoutContext('do nothing when already migrated', () async {
        expect(cmakeFile.readAsStringSync(), _sampleCmakeListsTxtMigrated);
        await migration.migrate();
        expect(cmakeFile.readAsStringSync(), _sampleCmakeListsTxtMigrated);
      });
    });
  });
}

class FakeAndroidProject extends Fake implements AndroidProject {
  FakeAndroidProject({required this.parent});

  @override
  FlutterProject parent;
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({required this.directory});

  @override
  final Directory directory;
}
