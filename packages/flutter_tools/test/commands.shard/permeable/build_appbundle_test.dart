// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_builder.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_appbundle.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/android_common.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart' show FakeFlutterVersion;
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();

  group('analytics', () {
    late Directory tempDir;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    testUsingContext('indicate the default target platforms', () async {
      final String projectPath = await createProject(tempDir,
          arguments: <String>['--no-pub', '--template=app']);

      await runBuildAppBundleCommand(projectPath);

      expect(
        fakeAnalytics.sentEvents,
        contains(Event.commandUsageValues(
          workflow: 'appbundle',
          commandHasTerminal: false,
          buildAppBundleTargetPlatform: 'android-arm,android-arm64,android-x64',
          buildAppBundleBuildMode: 'release',
        )),
      );
    }, overrides: <Type, Generator>{
      AndroidBuilder: () => FakeAndroidBuilder(),
      Analytics: () => fakeAnalytics,
    });

    testUsingContext('alias aab', () async {
      final BuildAppBundleCommand command =
          BuildAppBundleCommand(logger: BufferLogger.test());
      expect(command.aliases, contains('aab'));
    });

    testUsingContext('build type', () async {
      final String projectPath = await createProject(tempDir,
          arguments: <String>['--no-pub', '--template=app']);

      await runBuildAppBundleCommand(projectPath);
      expect(
        fakeAnalytics.sentEvents,
        contains(Event.commandUsageValues(
          workflow: 'appbundle',
          commandHasTerminal: false,
          buildAppBundleTargetPlatform: 'android-arm,android-arm64,android-x64',
          buildAppBundleBuildMode: 'release',
        )),
      );

      fakeAnalytics.sentEvents.clear();
      await runBuildAppBundleCommand(projectPath, arguments: <String>['--release']);
      expect(
        fakeAnalytics.sentEvents,
        contains(Event.commandUsageValues(
          workflow: 'appbundle',
          commandHasTerminal: false,
          buildAppBundleTargetPlatform: 'android-arm,android-arm64,android-x64',
          buildAppBundleBuildMode: 'release',
        )),
      );

      fakeAnalytics.sentEvents.clear();
      await runBuildAppBundleCommand(projectPath, arguments: <String>['--debug']);
      expect(
        fakeAnalytics.sentEvents,
        contains(Event.commandUsageValues(
          workflow: 'appbundle',
          commandHasTerminal: false,
          buildAppBundleTargetPlatform: 'android-arm,android-arm64,android-x64',
          buildAppBundleBuildMode: 'debug',
        )),
      );

      fakeAnalytics.sentEvents.clear();
      await runBuildAppBundleCommand(projectPath, arguments: <String>['--profile']);

      expect(
        fakeAnalytics.sentEvents,
        contains(Event.commandUsageValues(
          workflow: 'appbundle',
          commandHasTerminal: false,
          buildAppBundleTargetPlatform: 'android-arm,android-arm64,android-x64',
          buildAppBundleBuildMode: 'profile',
        )),
      );
    }, overrides: <Type, Generator>{
      AndroidBuilder: () => FakeAndroidBuilder(),
      Analytics: () => fakeAnalytics,
    });

    testUsingContext('logs success', () async {
      final String projectPath = await createProject(tempDir,
          arguments: <String>['--no-pub', '--template=app']);

      await runBuildAppBundleCommand(projectPath);

      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.flutterCommandResult(
            commandPath: 'create',
            result: 'success',
            commandHasTerminal: false,
            maxRss: globals.processInfo.maxRss,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      AndroidBuilder: () => FakeAndroidBuilder(),
      Analytics: () => fakeAnalytics,
    });

    testUsingContext('use of the deferred components feature sends a build info event indicating so', () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>[
          '--empty',
          '--no-pub',
          '--template=app',
        ],
      );

      // Add deferred manifest.
      final File pubspec = globals.localFileSystem
        .directory(projectPath)
        .childFile('pubspec.yaml');
      final String modifiedContents = pubspec
        .readAsStringSync()
        .replaceAll('flutter:', 'flutter:\n  deferred-components:');
      pubspec.writeAsStringSync(modifiedContents);
      printOnFailure(pubspec.readAsStringSync());

      final Directory oldCwd = globals.localFileSystem.currentDirectory;
      try {
        globals.localFileSystem.currentDirectory = globals.localFileSystem.directory(projectPath);
        await runBuildAppBundleCommand(projectPath);
      } finally {
        globals.localFileSystem.currentDirectory = oldCwd;
      }

      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.flutterBuildInfo(
            label: 'build-appbundle-deferred-components',
            buildType: 'android',
          ),
        ),
      );
    }, overrides: <Type, Generator>{
      AndroidBuilder: () => FakeAndroidBuilder(),
      Analytics: () => fakeAnalytics,
      ProcessInfo: () => processInfo,
    });
  });

  group('Gradle', () {
    late Directory tempDir;
    late FakeProcessManager processManager;
    late FakeAndroidSdk androidSdk;
    late FakeAnalytics analytics;

    setUp(() {
      analytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      processManager = FakeProcessManager.any();
      androidSdk = FakeAndroidSdk(globals.fs.directory('irrelevant'));
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    group('AndroidSdk', () {
      testUsingContext('throws throwsToolExit if AndroidSdk is null', () async {
        final String projectPath = await createProject(tempDir,
            arguments: <String>['--no-pub', '--template=app']);

        await expectLater(() async {
          await runBuildAppBundleCommand(
            projectPath,
            arguments: <String>['--no-pub'],
          );
        }, throwsToolExit(
          message: 'No Android SDK found. Try setting the ANDROID_HOME environment variable',
        ));
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => null,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        ProcessManager: () => processManager,
      });
    });

    testUsingContext("reports when the app isn't using AndroidX", () async {
      final String projectPath = await createProject(tempDir,
          arguments: <String>['--no-pub', '--template=app']);
      // Simulate a non-androidx project.
      tempDir
        .childDirectory('flutter_project')
        .childDirectory('android')
        .childFile('gradle.properties')
        .writeAsStringSync('android.useAndroidX=false');

      // The command throws a [ToolExit] because it expects an AAB in the file system.
      await expectLater(() async {
        await runBuildAppBundleCommand(
          projectPath,
        );
      }, throwsToolExit());

      expect(
        testLogger.statusText,
        containsIgnoringWhitespace("Your app isn't using AndroidX"),
      );
      expect(
        testLogger.statusText,
        containsIgnoringWhitespace(
        'To avoid potential build failures, you can quickly migrate your app by '
        'following the steps on https://goo.gl/CP92wY'
        ),
      );

      expect(analytics.sentEvents, contains(
        Event.flutterBuildInfo(
          label: 'app-not-using-android-x',
          buildType: 'gradle',
        ),
      ));
    },
    overrides: <Type, Generator>{
      AndroidSdk: () => androidSdk,
      FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
      ProcessManager: () => processManager,
      Analytics: () => analytics,
    });

    testUsingContext('reports when the app is using AndroidX', () async {
      final String projectPath = await createProject(tempDir,
          arguments: <String>['--no-pub', '--template=app']);

      // The command throws a [ToolExit] because it expects an AAB in the file system.
      await expectLater(() async {
        await runBuildAppBundleCommand(
          projectPath,
        );
      }, throwsToolExit());

      expect(
        testLogger.statusText,
        isNot(containsIgnoringWhitespace("Your app isn't using AndroidX")),
      );
      expect(
        testLogger.statusText,
        isNot(
          containsIgnoringWhitespace(
            'To avoid potential build failures, you can quickly migrate your app by '
            'following the steps on https://goo.gl/CP92wY'),
        )
      );

      expect(analytics.sentEvents, contains(
        Event.flutterBuildInfo(
          label: 'app-using-android-x',
          buildType: 'gradle',
        ),
      ));
    },
    overrides: <Type, Generator>{
      AndroidSdk: () => androidSdk,
      FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
      ProcessManager: () => processManager,
      Analytics: () => analytics,
    });
  });
}

Future<BuildAppBundleCommand> runBuildAppBundleCommand(
  String target, {
  List<String>? arguments,
}) async {
  final BuildAppBundleCommand command = BuildAppBundleCommand(logger: BufferLogger.test());
  final CommandRunner<void> runner = createTestCommandRunner(command);
  await runner.run(<String>[
    'appbundle',
    ...?arguments,
    '--no-pub',
    globals.fs.path.join(target, 'lib', 'main.dart'),
  ]);
  return command;
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  FakeAndroidSdk(this.directory);

  @override
  final Directory directory;
}
