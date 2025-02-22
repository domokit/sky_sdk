// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/bundle.dart';
import 'package:flutter_tools/src/bundle_builder.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_bundle.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:meta/meta.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();
  late Directory tempDir;
  late FakeBundleBuilder fakeBundleBuilder;
  final FileSystemStyle fileSystemStyle =
      globals.fs.path.separator == '/' ? FileSystemStyle.posix : FileSystemStyle.windows;
  late FakeAnalytics fakeAnalytics;

  MemoryFileSystem fsFactory() {
    return MemoryFileSystem.test(style: fileSystemStyle);
  }

  setUp(() {
    tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');

    fakeBundleBuilder = FakeBundleBuilder();
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fsFactory(),
      fakeFlutterVersion: FakeFlutterVersion(),
    );
  });

  tearDown(() {
    tryToDelete(tempDir);
  });

  Future<BuildBundleCommand> runCommandIn(String projectPath, {List<String>? arguments}) async {
    final BuildBundleCommand command = BuildBundleCommand(
      logger: BufferLogger.test(),
      bundleBuilder: fakeBundleBuilder,
    );
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>[
      'bundle',
      ...?arguments,
      '--target=$projectPath/lib/main.dart',
      '--no-pub',
    ]);
    return command;
  }

  testUsingContext(
    'bundle getUsage indicate that project is a module',
    () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>['--no-pub', '--template=module'],
      );

      await runCommandIn(projectPath);

      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.commandUsageValues(
            workflow: 'bundle',
            commandHasTerminal: false,
            buildBundleTargetPlatform: 'android-arm',
            buildBundleIsModule: true,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{Analytics: () => fakeAnalytics},
  );

  testUsingContext(
    'bundle getUsage indicate that project is not a module',
    () async {
      final String projectPath = await createProject(
        tempDir,
        arguments: <String>['--no-pub', '--template=app'],
      );

      await runCommandIn(projectPath);

      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.commandUsageValues(
            workflow: 'bundle',
            commandHasTerminal: false,
            buildBundleTargetPlatform: 'android-arm',
            buildBundleIsModule: false,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{Analytics: () => fakeAnalytics},
  );

  testUsingContext('bundle getUsage indicate the target platform', () async {
    final String projectPath = await createProject(
      tempDir,
      arguments: <String>['--no-pub', '--template=app'],
    );

    final BuildBundleCommand command = await runCommandIn(projectPath);

    expect(
      (await command.unifiedAnalyticsUsageValues('bundle')).eventData['buildBundleTargetPlatform'],
      'android-arm',
    );
  });

  testUsingContext(
    'bundle fails to build for Windows if feature is disabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync(recursive: true);
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      expect(
        () => runner.run(<String>['bundle', '--no-pub', '--target-platform=windows-x64']),
        throwsToolExit(message: 'Windows is not a supported target platform.'),
      );
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'bundle fails to build for Linux if feature is disabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      expect(
        () => runner.run(<String>['bundle', '--no-pub', '--target-platform=linux-x64']),
        throwsToolExit(message: 'Linux is not a supported target platform.'),
      );
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'bundle fails to build for macOS if feature is disabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      expect(
        () => runner.run(<String>['bundle', '--no-pub', '--target-platform=darwin']),
        throwsToolExit(message: 'macOS is not a supported target platform.'),
      );
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'bundle --tree-shake-icons fails',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      expect(
        () => runner.run(<String>['bundle', '--no-pub', '--release', '--tree-shake-icons']),
        throwsToolExit(message: 'tree-shake-icons'),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'bundle can build for Windows if feature is enabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>['bundle', '--no-pub', '--target-platform=windows-x64']);
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'bundle can build for Linux if feature is enabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>['bundle', '--no-pub', '--target-platform=linux-x64']);
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
    },
  );

  testUsingContext(
    'bundle can build for macOS if feature is enabled',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>['bundle', '--no-pub', '--target-platform=darwin']);
    },
    overrides: <Type, Generator>{
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'passes track widget creation through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();

      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--track-widget-creation',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes dart-define through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--dart-define=foo=bar',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kDartDefines:
                  'Zm9vPWJhcg==,RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes filesystem-scheme through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--filesystem-scheme=org-dartlang-root2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes filesystem-roots through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--filesystem-root=test1,test2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kFileSystemRoots: 'test1,test2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes extra frontend-options through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--extra-front-end-options=--testflag,--testflag2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kExtraFrontEndOptions: '--testflag,--testflag2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes extra gen_snapshot-options through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--debug',
        '--target-platform=android-arm',
        '--extra-gen-snapshot-options=--testflag,--testflag2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'debug',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kExtraGenSnapshotOptions: '--testflag,--testflag2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes profile options through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--profile',
        '--dart-define=foo=bar',
        '--target-platform=android-arm',
        '--track-widget-creation',
        '--filesystem-scheme=org-dartlang-root',
        '--filesystem-root=test1,test2',
        '--extra-gen-snapshot-options=--testflag,--testflag2',
        '--extra-front-end-options=--testflagFront,--testflagFront2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'profile',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'Zm9vPWJhcg==,RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kFileSystemRoots: 'test1,test2',
              kExtraGenSnapshotOptions: '--testflag,--testflag2',
              kExtraFrontEndOptions: '--testflagFront,--testflagFront2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'passes release options through',
    () async {
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file('pubspec.yaml').createSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildBundleCommand(logger: BufferLogger.test()),
      );

      await runner.run(<String>[
        'bundle',
        '--no-pub',
        '--release',
        '--dart-define=foo=bar',
        '--target-platform=android-arm',
        '--track-widget-creation',
        '--filesystem-scheme=org-dartlang-root',
        '--filesystem-root=test1,test2',
        '--extra-gen-snapshot-options=--testflag,--testflag2',
        '--extra-front-end-options=--testflagFront,--testflagFront2',
      ]);
    },
    overrides: <Type, Generator>{
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              kBuildMode: 'release',
              kTargetPlatform: 'android-arm',
              kTargetFile: globals.fs.path.join('lib', 'main.dart'),
              kDartDefines:
                  'Zm9vPWJhcg==,RkxVVFRFUl9WRVJTSU9OPTAuMC4w,RkxVVFRFUl9DSEFOTkVMPW1hc3Rlcg==,RkxVVFRFUl9HSVRfVVJMPWh0dHBzOi8vZ2l0aHViLmNvbS9mbHV0dGVyL2ZsdXR0ZXIuZ2l0,RkxVVFRFUl9GUkFNRVdPUktfUkVWSVNJT049MTExMTE=,RkxVVFRFUl9FTkdJTkVfUkVWSVNJT049YWJjZGU=,RkxVVFRFUl9EQVJUX1ZFUlNJT049MTI=',
              kTrackWidgetCreation: 'true',
              kFileSystemScheme: 'org-dartlang-root',
              kFileSystemRoots: 'test1,test2',
              kExtraGenSnapshotOptions: '--testflag,--testflag2',
              kExtraFrontEndOptions: '--testflagFront,--testflagFront2',
              kIconTreeShakerFlag: 'false',
              kDeferredComponents: 'false',
              kDartObfuscation: 'false',
              kNativeAssets: 'false',
            });
          }),
      FileSystem: fsFactory,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );
}

class FakeBundleBuilder extends Fake implements BundleBuilder {
  @override
  Future<void> build({
    required TargetPlatform platform,
    required BuildInfo buildInfo,
    FlutterProject? project,
    String? mainPath,
    String manifestPath = defaultManifestPath,
    String? applicationKernelFilePath,
    String? depfilePath,
    String? assetDirPath,
    bool buildNativeAssets = true,
    @visibleForTesting BuildSystem? buildSystem,
  }) async {}
}
