// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cmake_project.dart';
import 'package:flutter_tools/src/windows/migrations/show_window_migration.dart';
import 'package:test/fake.dart';

import '../../../src/common.dart';

void main() {
  group('Windows Flutter show window migration', () {
    late MemoryFileSystem memoryFileSystem;
    late BufferLogger testLogger;
    late FakeWindowsProject mockProject;
    late File flutterWindowFile;

    setUp(() {
      memoryFileSystem = MemoryFileSystem.test();
      flutterWindowFile = memoryFileSystem.file('flutter_window.cpp');

      testLogger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences.test(),
      );

      mockProject = FakeWindowsProject(flutterWindowFile);
    });

    testWithoutContext('skipped if Flutter window file is missing', () async {
      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(flutterWindowFile.existsSync(), isFalse);

      expect(
        testLogger.traceText,
        contains(
          'windows/runner/flutter_window.cpp file not found, '
          'skipping show window migration',
        ),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if nothing to migrate', () async {
      const String flutterWindowContents = 'Nothing to migrate';

      flutterWindowFile.writeAsStringSync(flutterWindowContents);

      final DateTime updatedAt = flutterWindowFile.lastModifiedSync();
      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(flutterWindowFile.lastModifiedSync(), updatedAt);
      expect(flutterWindowFile.readAsStringSync(), flutterWindowContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated', () async {
      const String flutterWindowContents =
          '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\n'
          '    this->Show();\n'
          '  });\n'
          '\n'
          '  // Flutter can complete the first frame before the "show window" callback is\n'
          '  // registered. The following call ensures a frame is pending to ensure the\n'
          "  // window is shown. It is a no-op if the first frame hasn't completed yet.\n"
          '  flutter_controller_->ForceRedraw();\n'
          '\n'
          '  return true;\n';

      flutterWindowFile.writeAsStringSync(flutterWindowContents);

      final DateTime updatedAt = flutterWindowFile.lastModifiedSync();
      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(flutterWindowFile.lastModifiedSync(), updatedAt);
      expect(flutterWindowFile.readAsStringSync(), flutterWindowContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('skipped if already migrated (CRLF)', () async {
      const String flutterWindowContents =
          '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\r\n'
          '    this->Show();\r\n'
          '  });\r\n'
          '\r\n'
          '  // Flutter can complete the first frame before the "show window" callback is\r\n'
          '  // registered. The following call ensures a frame is pending to ensure the\r\n'
          "  // window is shown. It is a no-op if the first frame hasn't completed yet.\r\n"
          '  flutter_controller_->ForceRedraw();\r\n'
          '\r\n'
          '  return true;\r\n';

      flutterWindowFile.writeAsStringSync(flutterWindowContents);

      final DateTime updatedAt = flutterWindowFile.lastModifiedSync();
      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(flutterWindowFile.lastModifiedSync(), updatedAt);
      expect(flutterWindowFile.readAsStringSync(), flutterWindowContents);

      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('migrates project to ensure window is shown', () async {
      flutterWindowFile.writeAsStringSync(
        '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\n'
        '    this->Show();\n'
        '  });\n'
        '\n'
        '  return true;\n',
      );

      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(
        flutterWindowFile.readAsStringSync(),
        '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\n'
        '    this->Show();\n'
        '  });\n'
        '\n'
        '  // Flutter can complete the first frame before the "show window" callback is\n'
        '  // registered. The following call ensures a frame is pending to ensure the\n'
        "  // window is shown. It is a no-op if the first frame hasn't completed yet.\n"
        '  flutter_controller_->ForceRedraw();\n'
        '\n'
        '  return true;\n',
      );

      expect(
        testLogger.statusText,
        contains(
          'windows/runner/flutter_window.cpp does not ensure the show window callback is called, updating.',
        ),
      );
    });

    testWithoutContext('migrates project to ensure window is shown (CRLF)', () async {
      flutterWindowFile.writeAsStringSync(
        '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\r\n'
        '    this->Show();\r\n'
        '  });\r\n'
        '\r\n'
        '  return true;\r\n',
      );

      final ShowWindowMigration migration = ShowWindowMigration(mockProject, testLogger);
      await migration.migrate();

      expect(
        flutterWindowFile.readAsStringSync(),
        '  flutter_controller_->engine()->SetNextFrameCallback([&]() {\r\n'
        '    this->Show();\r\n'
        '  });\r\n'
        '\r\n'
        '  // Flutter can complete the first frame before the "show window" callback is\r\n'
        '  // registered. The following call ensures a frame is pending to ensure the\r\n'
        "  // window is shown. It is a no-op if the first frame hasn't completed yet.\r\n"
        '  flutter_controller_->ForceRedraw();\r\n'
        '\r\n'
        '  return true;\r\n',
      );

      expect(
        testLogger.statusText,
        contains(
          'windows/runner/flutter_window.cpp does not ensure the show window callback is called, updating.',
        ),
      );
    });
  });
}

class FakeWindowsProject extends Fake implements WindowsProject {
  FakeWindowsProject(this.runnerFlutterWindowFile);

  @override
  final File runnerFlutterWindowFile;
}
