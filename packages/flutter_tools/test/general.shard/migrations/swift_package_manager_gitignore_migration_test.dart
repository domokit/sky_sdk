// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/migrations/swift_package_manager_gitignore_migration.dart';

import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  group('Swift Package Manager .gitignore migration', () {
    late MemoryFileSystem memoryFileSystem;
    late BufferLogger testLogger;
    late FakeFlutterProject mockProject;
    late File gitignoreFile;

    setUp(() {
      memoryFileSystem = MemoryFileSystem.test();
      gitignoreFile = memoryFileSystem.file('.gitignore');

      testLogger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences.test(),
      );

      mockProject = FakeFlutterProject(fileSystem: memoryFileSystem);
    });

    testWithoutContext('skipped if .gitignore file is missing', () {
      final SwiftPackageManagerGitignoreMigration migration = SwiftPackageManagerGitignoreMigration(
        mockProject,
        testLogger,
      );
      migration.migrate();
      expect(gitignoreFile.existsSync(), isFalse);

      expect(
        testLogger.traceText,
        contains('.gitignore file not found, skipping Swift Package Manager .gitignore migration.'),
      );
      expect(testLogger.warningText, isEmpty);
    });

    testWithoutContext('skipped if nothing to migrate', () {
      const String gitignoreFileContents = 'Nothing to migrate';

      gitignoreFile.writeAsStringSync(gitignoreFileContents);

      final DateTime updatedAt = gitignoreFile.lastModifiedSync();

      final SwiftPackageManagerGitignoreMigration migration = SwiftPackageManagerGitignoreMigration(
        mockProject,
        testLogger,
      );
      migration.migrate();

      expect(gitignoreFile.lastModifiedSync(), updatedAt);
      expect(gitignoreFile.readAsStringSync(), gitignoreFileContents);
      expect(testLogger.warningText, isEmpty);
    });

    testWithoutContext('skipped if already migrated', () {
      const String gitignoreFileContents = '''
.build/
.swiftpm/
''';

      gitignoreFile.writeAsStringSync(gitignoreFileContents);

      final DateTime updatedAt = gitignoreFile.lastModifiedSync();

      final SwiftPackageManagerGitignoreMigration migration = SwiftPackageManagerGitignoreMigration(
        mockProject,
        testLogger,
      );
      migration.migrate();

      expect(gitignoreFile.lastModifiedSync(), updatedAt);
      expect(gitignoreFile.readAsStringSync(), gitignoreFileContents);
      expect(testLogger.warningText, isEmpty);
    });

    testWithoutContext('migrates project to ignore Swift Package Manager build directories', () {
      gitignoreFile.writeAsStringSync(
        '.DS_Store\n'
        '.atom/\n'
        '.buildlog/\n'
        '.history\n'
        '.svn/\n'
        'migrate_working_dir/\n',
      );

      final SwiftPackageManagerGitignoreMigration migration = SwiftPackageManagerGitignoreMigration(
        mockProject,
        testLogger,
      );
      migration.migrate();

      expect(
        gitignoreFile.readAsStringSync(),
        '.DS_Store\n'
        '.atom/\n'
        '.build/\n'
        '.buildlog/\n'
        '.history\n'
        '.svn/\n'
        '.swiftpm/\n'
        'migrate_working_dir/\n',
      );

      expect(
        testLogger.warningText,
        contains('.gitignore does not ignore Swift Package Manager build directories, updating.'),
      );
    });
  });
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({required MemoryFileSystem fileSystem})
    : gitignoreFile = fileSystem.file('.gitignore');

  @override
  File gitignoreFile;
}
