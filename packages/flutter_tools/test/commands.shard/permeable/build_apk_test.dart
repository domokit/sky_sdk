// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_builder.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_apk.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/testing.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/android_common.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart' show FakeFlutterVersion;
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();

  group('Usage', () {
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

    testUsingContext(
      'indicate the default target platforms',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app'],
        );

        // Without buildMode flag.
        await runBuildApkCommand(projectPath);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm,android-arm64,android-x64',
              buildApkBuildMode: 'release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(projectPath, arguments: <String>['--debug']);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm,android-arm64,android-x86,android-x64',
              buildApkBuildMode: 'debug',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(projectPath, arguments: <String>['--jit-release']);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm,android-arm64,android-x86,android-x64',
              buildApkBuildMode: 'jit_release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(projectPath, arguments: <String>['--profile']);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm,android-arm64,android-x64',
              buildApkBuildMode: 'profile',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(projectPath, arguments: <String>['--release']);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm,android-arm64,android-x64',
              buildApkBuildMode: 'release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => FakeAndroidBuilder(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'Each build mode respects --target-platform',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app'],
        );

        // Without buildMode flag.
        await runBuildApkCommand(projectPath, arguments: <String>['--target-platform=android-arm']);
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm',
              buildApkBuildMode: 'release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(
          projectPath,
          arguments: <String>['--debug', '--target-platform=android-arm'],
        );
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm',
              buildApkBuildMode: 'debug',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(
          projectPath,
          arguments: <String>['--release', '--target-platform=android-arm'],
        );
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm',
              buildApkBuildMode: 'release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(
          projectPath,
          arguments: <String>['--profile', '--target-platform=android-arm'],
        );
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm',
              buildApkBuildMode: 'profile',
              buildApkSplitPerAbi: false,
            ),
          ),
        );

        await runBuildApkCommand(
          projectPath,
          arguments: <String>['--jit-release', '--target-platform=android-arm'],
        );
        expect(
          fakeAnalytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'apk',
              commandHasTerminal: false,
              buildApkTargetPlatform: 'android-arm',
              buildApkBuildMode: 'jit_release',
              buildApkSplitPerAbi: false,
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => FakeAndroidBuilder(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext('split per abi', () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>['--no-pub', '--template=app'],
      );

      final BuildApkCommand commandWithFlag = await runBuildApkCommand(
        projectPath,
        arguments: <String>['--split-per-abi'],
      );

      expect(
        (await commandWithFlag.unifiedAnalyticsUsageValues('run')).eventData['buildApkSplitPerAbi'],
        isTrue,
      );

      final BuildApkCommand commandWithoutFlag = await runBuildApkCommand(projectPath);
      expect(
        (await commandWithoutFlag.unifiedAnalyticsUsageValues(
          'run',
        )).eventData['buildApkSplitPerAbi'],
        isFalse,
      );
    }, overrides: <Type, Generator>{AndroidBuilder: () => FakeAndroidBuilder()});

    testUsingContext(
      'build type',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app'],
        );

        final BuildApkCommand defaultBuildCommand = await runBuildApkCommand(projectPath);
        final Event defaultBuildCommandUsageValues = await defaultBuildCommand
            .unifiedAnalyticsUsageValues('build');
        expect(defaultBuildCommandUsageValues.eventData['buildApkBuildMode'], 'release');

        final BuildApkCommand releaseBuildCommand = await runBuildApkCommand(
          projectPath,
          arguments: <String>['--release'],
        );
        final Event releaseBuildCommandUsageValues = await releaseBuildCommand
            .unifiedAnalyticsUsageValues('build');
        expect(releaseBuildCommandUsageValues.eventData['buildApkBuildMode'], 'release');

        final BuildApkCommand debugBuildCommand = await runBuildApkCommand(
          projectPath,
          arguments: <String>['--debug'],
        );
        final Event debugBuildCommandUsageValues = await debugBuildCommand
            .unifiedAnalyticsUsageValues('build');
        expect(debugBuildCommandUsageValues.eventData['buildApkBuildMode'], 'debug');

        final BuildApkCommand profileBuildCommand = await runBuildApkCommand(
          projectPath,
          arguments: <String>['--profile'],
        );
        final Event profileBuildCommandUsageValues = await profileBuildCommand
            .unifiedAnalyticsUsageValues('build');
        expect(profileBuildCommandUsageValues.eventData['buildApkBuildMode'], 'profile');

        fakeAnalytics.sentEvents.clear();
        await runBuildApkCommand(projectPath, arguments: <String>['--profile']);
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => FakeAndroidBuilder(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'logs success',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app'],
        );

        await runBuildApkCommand(projectPath);

        final Iterable<Event> successEvent = fakeAnalytics.sentEvents.where(
          (Event e) =>
              e.eventName == DashEvent.flutterCommandResult &&
              e.eventData['commandPath'] == 'create' &&
              e.eventData['result'] == 'success',
        );
        expect(successEvent, isNotEmpty, reason: 'Tool should send create success event');
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => FakeAndroidBuilder(),
        Analytics: () => fakeAnalytics,
      },
    );

    group('Impeller AndroidManifest.xml setting', () {
      // Adds a key-value `<meta-data>` pair to the `<application>` tag in the
      // corresponding `AndroidManifest.xml` file, right before the closing
      // `</application>` tag.
      void writeManifestMetadata({
        required String projectPath,
        required String name,
        required String value,
      }) {
        final String manifestPath = globals.fs.path.join(
          projectPath,
          'android',
          'app',
          'src',
          'main',
          'AndroidManifest.xml',
        );

        // It would be unnecessarily complicated to parse this XML file and
        // insert the key-value pair, so we just insert it right before the
        // closing </application> tag.
        final String oldManifest = globals.fs.file(manifestPath).readAsStringSync();
        final String newManifest = oldManifest.replaceFirst(
          '</application>',
          '    <meta-data\n'
              '        android:name="$name"\n'
              '        android:value="$value" />\n'
              '    </application>',
        );
        globals.fs.file(manifestPath).writeAsStringSync(newManifest);
      }

      testUsingContext(
        'a default APK build reports Impeller as enabled',
        () async {
          final String projectPath = await createProject(
            tempDir,
            arguments: <String>['--no-pub', '--template=app', '--platform=android'],
          );

          await runBuildApkCommand(projectPath);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-impeller-enabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => FakeAndroidBuilder(),
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        },
      );

      testUsingContext(
        'EnableImpeller="true" reports an enabled event',
        () async {
          final String projectPath = await createProject(
            tempDir,
            arguments: <String>['--no-pub', '--template=app', '--platform=android'],
          );

          writeManifestMetadata(
            projectPath: projectPath,
            name: 'io.flutter.embedding.android.EnableImpeller',
            value: 'true',
          );

          await runBuildApkCommand(projectPath);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-impeller-enabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => FakeAndroidBuilder(),
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        },
      );

      testUsingContext(
        'EnableImpeller="false" reports an disabled event',
        () async {
          final String projectPath = await createProject(
            tempDir,
            arguments: <String>['--no-pub', '--template=app', '--platform=android'],
          );

          writeManifestMetadata(
            projectPath: projectPath,
            name: 'io.flutter.embedding.android.EnableImpeller',
            value: 'false',
          );

          await runBuildApkCommand(projectPath);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-impeller-disabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => FakeAndroidBuilder(),
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        },
      );
    });
  });

  group('Gradle', () {
    late Directory tempDir;
    late FakeProcessManager processManager;
    late String gradlew;
    late AndroidSdk mockAndroidSdk;
    late FakeAnalytics analytics;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      gradlew = globals.fs.path.join(
        tempDir.path,
        'flutter_project',
        'android',
        globals.platform.isWindows ? 'gradlew.bat' : 'gradlew',
      );
      processManager = FakeProcessManager.empty();
      mockAndroidSdk = FakeAndroidSdk(globals.fs.directory('irrelevant'));
      analytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    group('AndroidSdk', () {
      testUsingContext(
        'throws throwsToolExit if AndroidSdk is null',
        () async {
          final String projectPath = await createProject(
            tempDir,
            arguments: <String>['--no-pub', '--template=app', '--platform=android'],
          );

          await expectLater(
            () => runBuildApkCommand(projectPath, arguments: <String>['--no-pub']),
            throwsToolExit(
              message: 'No Android SDK found. Try setting the ANDROID_HOME environment variable',
            ),
          );
        },
        overrides: <Type, Generator>{
          AndroidSdk: () => null,
          Java: () => null,
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
          ProcessManager: () => processManager,
          AndroidStudio: () => FakeAndroidStudio(),
        },
      );
    });

    testUsingContext(
      'shrinking is enabled by default on release mode',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
            exitCode: 1,
          ),
        );

        await expectLater(
          () => runBuildApkCommand(projectPath),
          throwsToolExit(message: 'Gradle task assembleRelease failed with exit code 1'),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        Java: () => null,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        ProcessManager: () => processManager,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );

    testUsingContext(
      '--split-debug-info is enabled when an output directory is provided',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Psplit-debug-info=${tempDir.path}',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
            exitCode: 1,
          ),
        );

        await expectLater(
          () => runBuildApkCommand(
            projectPath,
            arguments: <String>['--split-debug-info=${tempDir.path}'],
          ),
          throwsToolExit(message: 'Gradle task assembleRelease failed with exit code 1'),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        Java: () => null,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        ProcessManager: () => processManager,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );

    testUsingContext(
      '--extra-front-end-options are provided to gradle project',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Pextra-front-end-options=foo,bar',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
            exitCode: 1,
          ),
        );

        await expectLater(
          () => runBuildApkCommand(
            projectPath,
            arguments: <String>['--extra-front-end-options=foo', '--extra-front-end-options=bar'],
          ),
          throwsToolExit(message: 'Gradle task assembleRelease failed with exit code 1'),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        Java: () => null,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        ProcessManager: () => processManager,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );

    testUsingContext(
      'shrinking is disabled when --no-shrink is passed',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
            exitCode: 1,
          ),
        );

        await expectLater(
          () => runBuildApkCommand(projectPath, arguments: <String>['--no-shrink']),
          throwsToolExit(message: 'Gradle task assembleRelease failed with exit code 1'),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        Java: () => null,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        ProcessManager: () => processManager,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );

    testUsingContext(
      "reports when the app isn't using AndroidX",
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        // Simulate a non-androidx project.
        tempDir
            .childDirectory('flutter_project')
            .childDirectory('android')
            .childFile('gradle.properties')
            .writeAsStringSync('android.useAndroidX=false');
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
          ),
        );

        // The command throws a [ToolExit] because it expects an APK in the file system.
        await expectLater(() => runBuildApkCommand(projectPath), throwsToolExit());

        expect(
          testLogger.statusText,
          allOf(
            containsIgnoringWhitespace("Your app isn't using AndroidX"),
            containsIgnoringWhitespace(
              'To avoid potential build failures, you can quickly migrate your app by '
              'following the steps on https://goo.gl/CP92wY',
            ),
          ),
        );

        expect(
          analytics.sentEvents,
          contains(Event.flutterBuildInfo(label: 'app-not-using-android-x', buildType: 'gradle')),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        Java: () => null,
        ProcessManager: () => processManager,
        Analytics: () => analytics,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );

    testUsingContext(
      'reports when the app is using AndroidX',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=app', '--platform=android'],
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-q',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              '-Ptarget=${globals.fs.path.join(tempDir.path, 'flutter_project', 'lib', 'main.dart')}',
              '-Pbase-application-name=android.app.Application',
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              '-Pdart-obfuscation=false',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              'assembleRelease',
            ],
          ),
        );

        // The command throws a [ToolExit] because it expects an APK in the file system.
        await expectLater(() => runBuildApkCommand(projectPath), throwsToolExit());

        expect(
          testLogger.statusText,
          allOf(
            isNot(contains("[!] Your app isn't using AndroidX")),
            isNot(
              contains(
                'To avoid potential build failures, you can quickly migrate your app by '
                'following the steps on https://goo.gl/CP92wY',
              ),
            ),
          ),
        );

        expect(
          analytics.sentEvents,
          contains(Event.flutterBuildInfo(label: 'app-using-android-x', buildType: 'gradle')),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        AndroidSdk: () => mockAndroidSdk,
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        Java: () => null,
        ProcessManager: () => processManager,
        Analytics: () => analytics,
        AndroidStudio: () => FakeAndroidStudio(),
      },
    );
  });
}

Future<BuildApkCommand> runBuildApkCommand(String target, {List<String>? arguments}) async {
  final BuildApkCommand command = BuildApkCommand(logger: BufferLogger.test());
  final CommandRunner<void> runner = createTestCommandRunner(command);
  await runner.run(<String>[
    'apk',
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

class FakeAndroidStudio extends Fake implements AndroidStudio {
  @override
  String get javaPath => 'java';

  @override
  Version get version => Version(2021, 3, 1);
}
