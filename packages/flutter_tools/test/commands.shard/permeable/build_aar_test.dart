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
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/testing.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/android_common.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart' hide FakeFlutterProjectFactory;
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();

  /// Runs the equivalent of `flutter build aar`.
  ///
  /// If [arguments] are provided, they are appended to the end, i.e.:
  /// ```sh
  /// flutter build aar [arguments]
  /// ```
  ///
  /// If [androidSdk] is provided, it is used, otherwise defaults to [FakeAndroidSdk].
  Future<BuildAarCommand> runBuildAar(
    String target, {
    AndroidSdk? androidSdk = const _FakeAndroidSdk(),
    List<String>? arguments,
  }) async {
    final BuildAarCommand command = BuildAarCommand(
      androidSdk: androidSdk,
      fileSystem: globals.fs,
      logger: BufferLogger.test(),
      verboseHelp: false,
    );
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>['aar', ...?arguments, target]);
    return command;
  }

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  group('Usage', () {
    late Directory tempDir;
    late FakeAnalytics analytics;

    setUp(() {
      analytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
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

        await runBuildAar(projectPath, arguments: <String>['--no-pub']);
        expect(
          analytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'aar',
              buildAarProjectType: 'module',
              buildAarTargetPlatform: 'android-arm,android-arm64,android-x64',
              commandHasTerminal: false,
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
        Analytics: () => analytics,
      },
    );

    testUsingContext(
      'indicate the target platform',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        await runBuildAar(
          projectPath,
          arguments: <String>['--no-pub', '--target-platform=android-arm'],
        );
        expect(
          analytics.sentEvents,
          contains(
            Event.commandUsageValues(
              workflow: 'aar',
              buildAarProjectType: 'module',
              buildAarTargetPlatform: 'android-arm',
              commandHasTerminal: false,
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
        Analytics: () => analytics,
      },
    );

    // Regression test for https://github.com/flutter/flutter/issues/162649.
    testUsingContext(
      'triggers builds even with --pub',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        await runBuildAar(
          projectPath,
          arguments: <String>['--target-platform=android-arm'],
          // If we use --no-pub, it bypasses validation that occurs only on a
          // build with --pub, which as a consequence means that we aren't
          // testing every code branch.
        );
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
        Analytics: () => analytics,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: () => FakePubWithPrimedDeps(allowGet: true),
      },
    );

    testUsingContext(
      'logs success',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );

        await runBuildAar(
          projectPath,
          arguments: <String>['--no-pub', '--target-platform=android-arm'],
        );

        final Iterable<Event> successEvent = analytics.sentEvents.where(
          (Event e) =>
              e.eventName == DashEvent.flutterCommandResult &&
              e.eventData['commandPath'] == 'create' &&
              e.eventData['result'] == 'success',
        );
        expect(successEvent, isNotEmpty, reason: 'Tool should send create success event');
      },
      overrides: <Type, Generator>{
        AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
        Analytics: () => analytics,
      },
    );
  });

  group('flag parsing', () {
    late Directory tempDir;
    late _CapturingFakeAndroidBuilder fakeAndroidBuilder;

    setUp(() {
      fakeAndroidBuilder = _CapturingFakeAndroidBuilder();
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
      await runBuildAar(projectPath, arguments: <String>['--no-pub']);

      expect(
        fakeAndroidBuilder.capturedBuildAarCalls,
        hasLength(1),
        reason: 'A single call to buildAar was expected.',
      );
      final Invocation buildAarCall = fakeAndroidBuilder.capturedBuildAarCalls.single;
      expect(buildAarCall.namedArguments[#buildNumber], '1.0');

      final List<BuildMode> buildModes = <BuildMode>[];
      for (final AndroidBuildInfo androidBuildInfo
          in buildAarCall.namedArguments[#androidBuildInfo] as Set<AndroidBuildInfo>) {
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
      expect(buildModes, hasLength(3));
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
      await runBuildAar(
        projectPath,
        arguments: <String>[
          '--no-pub',
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

      expect(
        fakeAndroidBuilder.capturedBuildAarCalls,
        hasLength(1),
        reason: 'A single call to buildAar was expected.',
      );
      final Invocation buildAarCall = fakeAndroidBuilder.capturedBuildAarCalls.single;
      expect(buildAarCall.namedArguments[#buildNumber], '200');

      final AndroidBuildInfo androidBuildInfo =
          (buildAarCall.namedArguments[#androidBuildInfo] as Set<AndroidBuildInfo>).single;
      expect(androidBuildInfo.targetArchs, <AndroidArch>[AndroidArch.x86]);

      final BuildInfo buildInfo = androidBuildInfo.buildInfo;
      expect(buildInfo.mode, BuildMode.release);
      expect(buildInfo.treeShakeIcons, isTrue);
      expect(buildInfo.flavor, 'free');
      expect(buildInfo.splitDebugInfoPath, '/project-name/v1.2.3/');
      expect(buildInfo.dartObfuscation, isTrue);
      expect(buildInfo.dartDefines.contains('foo=bar'), isTrue);
    }, overrides: <Type, Generator>{AndroidBuilder: () => fakeAndroidBuilder});
  });

  group('Gradle', () {
    late Directory tempDir;
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
              await runBuildAar(projectPath, androidSdk: null, arguments: <String>['--no-pub']);
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
          await runBuildAar(
            'missing_project',
            arguments: <String>[
              '--no-pub',
              globals.fs.path.join('missing_project', 'lib', 'main.dart'),
            ],
          );
        }, throwsToolExit(message: 'main.dart does not exist'));
      });

      testUsingContext('flutter project not valid', () async {
        await expectLater(() async {
          await runBuildAar(tempDir.path, arguments: <String>['--no-pub']);
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
              '-Pdart-defines=RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
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
          () => runBuildAar(
            projectPath,
            arguments: <String>[
              '--no-pub',
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
        AndroidStudio: () => _FakeAndroidStudio(),
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

          await runBuildAar(projectPath, arguments: <String>['--no-pub']);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-enabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
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

          await runBuildAar(projectPath, arguments: <String>['--no-pub']);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-enabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
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

          await runBuildAar(projectPath, arguments: <String>['--no-pub']);

          expect(
            fakeAnalytics.sentEvents,
            contains(
              Event.flutterBuildInfo(label: 'manifest-aar-impeller-disabled', buildType: 'android'),
            ),
          );
        },
        overrides: <Type, Generator>{
          Analytics: () => fakeAnalytics,
          AndroidBuilder: () => _CapturingFakeAndroidBuilder(),
          FlutterProjectFactory: () => FakeFlutterProjectFactory(tempDir),
        },
      );
    });
  });
}

/// A fake implementation of [AndroidBuilder] that allows [buildAar] calls.
///
/// Calls to [buildAar] are stored as [capturedBuildAarCalls], other calls are rejected.
final class _CapturingFakeAndroidBuilder extends Fake implements AndroidBuilder {
  final List<Invocation> capturedBuildAarCalls = <Invocation>[];

  @override
  Object? noSuchMethod(Invocation invocation) {
    if (invocation.memberName != #buildAar) {
      return super.noSuchMethod(invocation);
    }
    capturedBuildAarCalls.add(invocation);
    return Future<void>.value();
  }
}

final class _FakeAndroidSdk with Fake implements AndroidSdk {
  const _FakeAndroidSdk();
}

final class _FakeAndroidStudio extends Fake implements AndroidStudio {
  @override
  String get javaPath => 'java';
}
