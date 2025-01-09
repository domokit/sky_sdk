// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show IOOverrides;

import 'package:args/command_runner.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/commands/widget_preview.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';
import 'utils/project_testing_utils.dart';

void main() {
  late Directory tempDir;
  late LoggingProcessManager loggingProcessManager;
  late FakeStdio mockStdio;

  setUp(() {
    loggingProcessManager = LoggingProcessManager();
    tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_create_test.');
    mockStdio = FakeStdio();
  });

  tearDown(() {
    tryToDelete(tempDir);
  });

  Future<String> createRootProject() async {
    return createProject(tempDir, arguments: <String>['--pub']);
  }

  Future<void> runWidgetPreviewCommand(List<String> arguments) async {
    final CommandRunner<void> runner = createTestCommandRunner(WidgetPreviewCommand());
    await runner.run(<String>['widget-preview', ...arguments]);
  }

  Future<void> startWidgetPreview({
    required String? rootProjectPath,
    List<String>? arguments,
  }) async {
    await runWidgetPreviewCommand(<String>[
      'start',
      ...?arguments,
      if (rootProjectPath != null) rootProjectPath,
    ]);
    expect(
      globals.fs
          .directory(rootProjectPath ?? globals.fs.currentDirectory.path)
          .childDirectory('.dart_tool')
          .childDirectory('widget_preview_scaffold'),
      exists,
    );
  }

  Future<void> cleanWidgetPreview({required String rootProjectPath}) async {
    await runWidgetPreviewCommand(<String>['clean', rootProjectPath]);
    expect(
      globals.fs
          .directory(rootProjectPath)
          .childDirectory('.dart_tool')
          .childDirectory('widget_preview_scaffold'),
      isNot(exists),
    );
  }

  group('flutter widget-preview', () {
    group('start exits if', () {
      testUsingContext('given an invalid directory', () async {
        try {
          await runWidgetPreviewCommand(<String>['start', 'foo']);
          fail('Successfully executed with multiple project paths');
        } on ToolExit catch (e) {
          expect(e.message, contains('Could not find foo'));
        }
      });

      testUsingContext('more than one project directory is provided', () async {
        try {
          await runWidgetPreviewCommand(<String>['start', tempDir.path, tempDir.path]);
          fail('Successfully executed with multiple project paths');
        } on ToolExit catch (e) {
          expect(e.message, contains('Only one directory should be provided.'));
        }
      });

      testUsingContext('run outside of a Flutter project directory', () async {
        try {
          await startWidgetPreview(rootProjectPath: tempDir.path);
          fail('Successfully executed outside of a Flutter project directory');
        } on ToolExit catch (e) {
          expect(e.message, contains('${tempDir.path} is not a valid Flutter project.'));
        }
      });
    });

    testUsingContext(
      'start creates .dart_tool/widget_preview_scaffold',
      () async {
        final String rootProjectPath = await createRootProject();
        await startWidgetPreview(rootProjectPath: rootProjectPath);
      },
      overrides: <Type, Generator>{
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'start creates .dart_tool/widget_preview_scaffold in the CWD',
      () async {
        final String rootProjectPath = await createRootProject();
        await io.IOOverrides.runZoned<Future<void>>(() async {
          // Try to execute using the CWD.
          await startWidgetPreview(rootProjectPath: null);
        }, getCurrentDirectory: () => globals.fs.directory(rootProjectPath));
      },
      overrides: <Type, Generator>{
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'clean deletes .dart_tool/widget_preview_scaffold',
      () async {
        final String rootProjectPath = await createRootProject();
        await startWidgetPreview(rootProjectPath: rootProjectPath);
        await cleanWidgetPreview(rootProjectPath: rootProjectPath);
      },
      overrides: <Type, Generator>{
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'invokes pub in online and offline modes',
      () async {
        // Run pub online first in order to populate the pub cache.
        final String rootProjectPath = await createRootProject();
        loggingProcessManager.clear();

        final RegExp dartCommand = RegExp(r'dart-sdk[\\/]bin[\\/]dart');

        await startWidgetPreview(rootProjectPath: rootProjectPath);
        expect(
          loggingProcessManager.commands,
          contains(
            predicate(
              (List<String> c) =>
                  dartCommand.hasMatch(c[0]) && c[1].contains('pub') && !c.contains('--offline'),
            ),
          ),
        );

        await cleanWidgetPreview(rootProjectPath: rootProjectPath);

        // Run pub offline.
        loggingProcessManager.clear();
        await startWidgetPreview(
          rootProjectPath: rootProjectPath,
          arguments: <String>['--pub', '--offline'],
        );

        expect(
          loggingProcessManager.commands,
          contains(
            predicate(
              (List<String> c) =>
                  dartCommand.hasMatch(c[0]) && c[1].contains('pub') && c.contains('--offline'),
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => loggingProcessManager,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );
  });
}
