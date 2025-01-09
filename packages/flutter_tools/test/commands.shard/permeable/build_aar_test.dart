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
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_aar.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/android_common.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart' hide FakeFlutterProjectFactory;
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();

  Future<BuildAarCommand> runCommandIn(String target, {List<String>? arguments}) async {
    final BuildAarCommand command = BuildAarCommand(
      androidSdk: FakeAndroidSdk(),
      fileSystem: globals.fs,
      logger: BufferLogger.test(),
      verboseHelp: false,
    );
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>['aar', '--no-pub', ...?arguments, target]);
    return command;
  }

  group('Usage', () {
    late Directory tempDir;
    late TestUsage testUsage;

    setUp(() {
      testUsage = TestUsage();
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    testUsingContext(
      'indicate that project is a module',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        final BuildAarCommand command = await runCommandIn(projectPath);
        expect((await command.usageValues).commandBuildAarProjectType, 'module');
      },
      overrides: <Type, Generator>{AndroidBuilder: () => FakeAndroidBuilder()},
    );

    testUsingContext(
      'indicate the target platform',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        final BuildAarCommand command = await runCommandIn(
          projectPath,
          arguments: <String>['--target-platform=android-arm'],
        );
        expect((await command.usageValues).commandBuildAarTargetPlatform, 'android-arm');
      },
      overrides: <Type, Generator>{AndroidBuilder: () => FakeAndroidBuilder()},
    );

    testUsingContext(
      'logs success',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        await runCommandIn(projectPath, arguments: <String>['--target-platform=android-arm']);

        expect(
          testUsage.events,
          contains(const TestUsageEvent('tool-command-result', 'aar', label: 'success')),
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => FakeAndroidBuilder(),
        Usage: () => testUsage,
      },
    );
  });

  group('flag parsing', () {
    late Directory tempDir;
    late FakeAndroidBuilder fakeAndroidBuilder;

    setUp(() {
      fakeAndroidBuilder = FakeAndroidBuilder();
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_build_aar_test.');
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    testUsingContext('defaults', () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>['--no-pub', '--template=module'],
      );
      await runCommandIn(projectPath);

      expect(fakeAndroidBuilder.buildNumber, '1.0');
      expect(fakeAndroidBuilder.androidBuildInfo.length, 3);

      final List<BuildMode> buildModes = <BuildMode>[];
      for (final AndroidBuildInfo androidBuildInfo in fakeAndroidBuilder.androidBuildInfo) {
        final BuildInfo buildInfo = androidBuildInfo.buildInfo;
        buildModes.add(buildInfo.mode);
        if (buildInfo.mode.isPrecompiled) {
          expect(buildInfo.treeShakeIcons, isTrue);
          expect(buildInfo.trackWidgetCreation, isTrue);
        } else {
          expect(buildInfo.treeShakeIcons, isFalse);
          expect(buildInfo.trackWidgetCreation, isTrue);
        }
        expect(buildInfo.flavor, isNull);
        expect(buildInfo.splitDebugInfoPath, isNull);
        expect(buildInfo.dartObfuscation, isFalse);
        expect(androidBuildInfo.targetArchs, <AndroidArch>[
          AndroidArch.armeabi_v7a,
          AndroidArch.arm64_v8a,
          AndroidArch.x86_64,
        ]);
      }
      expect(buildModes.length, 3);
      expect(
        buildModes,
        containsAll(<BuildMode>[BuildMode.debug, BuildMode.profile, BuildMode.release]),
      );
    }, overrides: <Type, Generator>{AndroidBuilder: () => fakeAndroidBuilder});

    testUsingContext('parses flags', () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>['--no-pub', '--template=module'],
      );
      await runCommandIn(
        projectPath,
        arguments: <String>[
          '--no-debug',
          '--no-profile',
          '--target-platform',
          'android-x86',
          '--tree-shake-icons',
          '--flavor',
          'free',
          '--build-number',
          '200',
          '--split-debug-info',
          '/project-name/v1.2.3/',
          '--obfuscate',
          '--dart-define=foo=bar',
        ],
      );

      expect(fakeAndroidBuilder.buildNumber, '200');

      final AndroidBuildInfo androidBuildInfo = fakeAndroidBuilder.androidBuildInfo.single;
      expect(androidBuildInfo.targetArchs, <AndroidArch>[AndroidArch.x86]);

      final BuildInfo buildInfo = androidBuildInfo.buildInfo;
      expect(buildInfo.mode, BuildMode.release);
      expect(buildInfo.treeShakeIcons, isTrue);
      expect(buildInfo.flavor, 'free');
      expect(buildInfo.splitDebugInfoPath, '/project-name/v1.2.3/');
      expect(buildInfo.dartObfuscation, isTrue);
      expect(buildInfo.dartDefines.contains('foo=bar'), isTrue);
      expect(buildInfo.nullSafetyMode, NullSafetyMode.sound);
    }, overrides: <Type, Generator>{AndroidBuilder: () => fakeAndroidBuilder});
  });

  group('Gradle', () {
    late Directory tempDir;
    late AndroidSdk mockAndroidSdk;
    late String gradlew;
    late FakeProcessManager processManager;
    late String flutterRoot;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
      mockAndroidSdk = FakeAndroidSdk();
      gradlew = globals.fs.path.join(
        tempDir.path,
        'flutter_project',
        '.android',
        globals.platform.isWindows ? 'gradlew.bat' : 'gradlew',
      );
      processManager = FakeProcessManager.empty();
      flutterRoot = getFlutterRoot();
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
            arguments: <String>['--no-pub', '--template=module'],
          );

          await expectLater(
            () async {
              await runBuildAarCommand(projectPath, null, arguments: <String>['--no-pub']);
            },
            throwsToolExit(
              message: 'No Android SDK found. Try setting the ANDROID_HOME environment variable',
            ),
          );
        },
        overrides: <Type, Generator>{
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );
    });

    group('throws ToolExit', () {
      testUsingContext('main.dart not found', () async {
        await expectLater(() async {
          await runBuildAarCommand(
            'missing_project',
            mockAndroidSdk,
            arguments: <String>['--no-pub'],
          );
        }, throwsToolExit(message: 'main.dart does not exist'));
      });

      testUsingContext('flutter project not valid', () async {
        await expectLater(() async {
          await runCommandIn(tempDir.path, arguments: <String>['--no-pub']);
        }, throwsToolExit(message: 'is not a valid flutter project'));
      });
    });

    testUsingContext(
      'support ExtraDartFlagOptions',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        processManager.addCommand(
          FakeCommand(
            command: <String>[
              gradlew,
              '-I=${globals.fs.path.join(flutterRoot, 'packages', 'flutter_tools', 'gradle', 'aar_init_script.gradle')}',
              '-Pflutter-root=$flutterRoot',
              '-Poutput-dir=${globals.fs.path.join(tempDir.path, 'flutter_project', 'build', 'host')}',
              '-Pis-plugin=false',
              '-PbuildNumber=1.0',
              '-q',
              '-Ptarget=${globals.fs.path.join('lib', 'main.dart')}',
              '-Pdart-obfuscation=false',
              '-Pextra-front-end-options=foo,bar',
              '-Ptrack-widget-creation=true',
              '-Ptree-shake-icons=true',
              '-Ptarget-platform=android-arm,android-arm64,android-x64',
              'assembleAarRelease',
            ],
            exitCode: 1,
          ),
        );

        await expectLater(
          () => runBuildAarCommand(
            projectPath,
            mockAndroidSdk,
            arguments: <String>[
              '--no-debug',
              '--no-profile',
              '--extra-front-end-options=foo',
              '--extra-front-end-options=bar',
            ],
          ),
          throwsToolExit(message: 'Gradle task assembleAarRelease failed with exit code 1'),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        Java: () => null,
        ProcessManager: () => processManager,
        FeatureFlags: () => TestFeatureFlags(isIOSEnabled: false),
        AndroidStudio: () => FakeAndroidStudio(),
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
          '.android',
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
        'a default AAR build reports Impeller as enabled',
        () async {
          final String projectPath = await createProject(
            tempDir,
            arguments: <String>['--no-pub', '--template=module'],
          );

          await runBuildAarCommand(projectPath, mockAndroidSdk);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-enabled', buildType: 'android'),
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
            arguments: <String>['--no-pub', '--template=module'],
          );
          final FlutterProject project = FlutterProject.fromDirectory(
            globals.fs.directory(projectPath),
          );
          await project.android.ensureReadyForPlatformSpecificTooling();

          writeManifestMetadata(
            projectPath: projectPath,
            name: 'io.flutter.embedding.android.EnableImpeller',
            value: 'true',
          );

          await runBuildAarCommand(projectPath, mockAndroidSdk);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-enabled', buildType: 'android'),
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
            arguments: <String>['--no-pub', '--template=module'],
          );
          final FlutterProject project = FlutterProject.fromDirectory(
            globals.fs.directory(projectPath),
          );
          await project.android.ensureReadyForPlatformSpecificTooling();

          writeManifestMetadata(
            projectPath: projectPath,
            name: 'io.flutter.embedding.android.EnableImpeller',
            value: 'false',
          );

          await runBuildAarCommand(projectPath, mockAndroidSdk);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-disabled', buildType: 'android'),
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
}

Future<BuildAarCommand> runBuildAarCommand(
  String target,
  AndroidSdk? androidSdk, {
  List<String>? arguments,
}) async {
  final BuildAarCommand command = BuildAarCommand(
    androidSdk: androidSdk,
    fileSystem: globals.fs,
    logger: BufferLogger.test(),
    verboseHelp: false,
  );
  final CommandRunner<void> runner = createTestCommandRunner(command);
  await runner.run(<String>[
    'aar',
    '--no-pub',
    ...?arguments,
    globals.fs.path.join(target, 'lib', 'main.dart'),
  ]);
  return command;
}

class FakeAndroidBuilder extends Fake implements AndroidBuilder {
  late FlutterProject project;
  late Set<AndroidBuildInfo> androidBuildInfo;
  late String target;
  String? outputDirectoryPath;
  late String buildNumber;

  @override
  Future<void> buildAar({
    required FlutterProject project,
    required Set<AndroidBuildInfo> androidBuildInfo,
    required String target,
    String? outputDirectoryPath,
    required String buildNumber,
  }) async {
    this.project = project;
    this.androidBuildInfo = androidBuildInfo;
    this.target = target;
    this.outputDirectoryPath = outputDirectoryPath;
    this.buildNumber = buildNumber;
  }
}

class FakeAndroidSdk extends Fake implements AndroidSdk {}

class FakeAndroidStudio extends Fake implements AndroidStudio {
  @override
  String get javaPath => 'java';
}
