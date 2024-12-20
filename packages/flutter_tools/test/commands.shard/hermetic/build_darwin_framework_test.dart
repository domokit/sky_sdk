// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_ios_framework.dart';
import 'package:flutter_tools/src/commands/build_macos_framework.dart';
import 'package:flutter_tools/src/version.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';

void main() {
  late MemoryFileSystem memoryFileSystem;
  late Directory outputDirectory;
  late FakePlatform fakePlatform;

  setUpAll(() {
    Cache.disableLocking();
  });

  const String storageBaseUrl = 'https://fake.googleapis.com';
  setUp(() {
    memoryFileSystem = MemoryFileSystem.test();
    fakePlatform = FakePlatform(
      operatingSystem: 'macos',
      environment: <String, String>{'FLUTTER_STORAGE_BASE_URL': storageBaseUrl},
    );

    outputDirectory = memoryFileSystem.systemTempDirectory
      .createTempSync('flutter_build_framework_test_output.')
      .childDirectory('Debug')..createSync();
  });

  group('build ios-framework', () {
    group('podspec', () {
      const String engineRevision = '0123456789abcdef';
      late Cache cache;

      setUp(() {
        final Directory rootOverride = memoryFileSystem.directory('cache');
        cache = Cache.test(
          rootOverride: rootOverride,
          platform: fakePlatform,
          fileSystem: memoryFileSystem,
          processManager: FakeProcessManager.any(),
        );
        rootOverride.childDirectory('bin').childDirectory('internal').childFile('engine.version')
          ..createSync(recursive: true)
          ..writeAsStringSync(engineRevision);
      });

      testUsingContext(
        'version unknown',
        () async {
          const String frameworkVersion = '0.0.0-unknown';
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            frameworkVersion: frameworkVersion,
          );

          final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(
              message:
                  '--cocoapods is only supported on the beta or stable channel. Detected version is $frameworkVersion',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'throws when not on a released version',
        () async {
          const String frameworkVersion = 'v1.13.10+hotfix-pre.2';
          const GitTagVersion gitTagVersion = GitTagVersion(
            x: 1,
            y: 13,
            z: 10,
            hotfix: 13,
            commits: 2,
          );
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            gitTagVersion: gitTagVersion,
            frameworkVersion: frameworkVersion,
          );

          final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(
              message:
                  '--cocoapods is only supported on the beta or stable channel. Detected version is $frameworkVersion',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'throws when license not found',
        () async {
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            gitTagVersion: const GitTagVersion(x: 1, y: 13, z: 10, hotfix: 13, commits: 0),
          );

          final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(message: 'Could not find license'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      group('is created', () {
        const String frameworkVersion = 'v1.13.11+hotfix.13';
        const String licenseText = 'This is the license!';

        setUp(() {
          // cache.getLicenseFile() relies on the flutter root being set.
          Cache.flutterRoot ??= getFlutterRoot();
          cache.getLicenseFile()
            ..createSync(recursive: true)
            ..writeAsStringSync(licenseText);
        });

        group('on master channel', () {
          testUsingContext(
            'created when forced',
            () async {
              const GitTagVersion gitTagVersion = GitTagVersion(
                x: 1,
                y: 13,
                z: 11,
                hotfix: 13,
                commits: 100,
              );
              final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
                gitTagVersion: gitTagVersion,
                frameworkVersion: frameworkVersion,
              );

              final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory, force: true);

              final File expectedPodspec = outputDirectory.childFile('Flutter.podspec');
              expect(expectedPodspec.existsSync(), isTrue);
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );
        });

        group('not on master channel', () {
          late FakeFlutterVersion fakeFlutterVersion;
          setUp(() {
            const GitTagVersion gitTagVersion = GitTagVersion(
              x: 1,
              y: 13,
              z: 11,
              hotfix: 13,
              commits: 0,
            );
            fakeFlutterVersion = FakeFlutterVersion(
              gitTagVersion: gitTagVersion,
              frameworkVersion: frameworkVersion,
            );
          });

          testUsingContext(
            'contains license and version',
            () async {
              final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('Flutter.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(podspecContents, contains("'1.13.1113'"));
              expect(podspecContents, contains('# $frameworkVersion'));
              expect(podspecContents, contains(licenseText));
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'debug URL',
            () async {
              final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('Flutter.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/ios/artifacts.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'profile URL',
            () async {
              final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.profile, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('Flutter.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/ios-profile/artifacts.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'release URL',
            () async {
              final BuildIOSFrameworkCommand command = BuildIOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.release, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('Flutter.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/ios-release/artifacts.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );
        });
      });
    });
  });

  group('build macos-framework', () {
    group('podspec', () {
      const String engineRevision = '0123456789abcdef';
      late Cache cache;

      setUp(() {
        final Directory rootOverride = memoryFileSystem.directory('cache');
        cache = Cache.test(
          rootOverride: rootOverride,
          platform: fakePlatform,
          fileSystem: memoryFileSystem,
          processManager: FakeProcessManager.any(),
        );
        rootOverride.childDirectory('bin').childDirectory('internal').childFile('engine.version')
          ..createSync(recursive: true)
          ..writeAsStringSync(engineRevision);
      });

      testUsingContext(
        'version unknown',
        () async {
          const String frameworkVersion = '0.0.0-unknown';
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            frameworkVersion: frameworkVersion,
          );

          final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(
              message:
                  '--cocoapods is only supported on the beta or stable channel. Detected version is $frameworkVersion',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'throws when not on a released version',
        () async {
          const String frameworkVersion = 'v1.13.10+hotfix-pre.2';
          const GitTagVersion gitTagVersion = GitTagVersion(
            x: 1,
            y: 13,
            z: 10,
            hotfix: 13,
            commits: 2,
          );
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            gitTagVersion: gitTagVersion,
            frameworkVersion: frameworkVersion,
          );

          final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(
              message:
                  '--cocoapods is only supported on the beta or stable channel. Detected version is $frameworkVersion',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'throws when license not found',
        () async {
          final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
            gitTagVersion: const GitTagVersion(x: 1, y: 13, z: 10, hotfix: 13, commits: 0),
          );

          final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
            logger: BufferLogger.test(),
            buildSystem: TestBuildSystem.all(BuildResult(success: true)),
            platform: fakePlatform,
            flutterVersion: fakeFlutterVersion,
            cache: cache,
            verboseHelp: false,
          );

          expect(
            () => command.produceFlutterPodspec(BuildMode.debug, outputDirectory),
            throwsToolExit(message: 'Could not find license'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => memoryFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      group('is created', () {
        const String frameworkVersion = 'v1.13.11+hotfix.13';
        const String licenseText = 'This is the license!';

        setUp(() {
          // cache.getLicenseFile() relies on the flutter root being set.
          Cache.flutterRoot ??= getFlutterRoot();
          cache.getLicenseFile()
            ..createSync(recursive: true)
            ..writeAsStringSync(licenseText);
        });

        group('on master channel', () {
          testUsingContext(
            'created when forced',
            () async {
              const GitTagVersion gitTagVersion = GitTagVersion(
                x: 1,
                y: 13,
                z: 11,
                hotfix: 13,
                commits: 100,
              );
              final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion(
                gitTagVersion: gitTagVersion,
                frameworkVersion: frameworkVersion,
              );

              final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory, force: true);

              final File expectedPodspec = outputDirectory.childFile('FlutterMacOS.podspec');
              expect(expectedPodspec.existsSync(), isTrue);
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );
        });

        group('not on master channel', () {
          late FakeFlutterVersion fakeFlutterVersion;
          setUp(() {
            const GitTagVersion gitTagVersion = GitTagVersion(
              x: 1,
              y: 13,
              z: 11,
              hotfix: 13,
              commits: 0,
            );
            fakeFlutterVersion = FakeFlutterVersion(
              gitTagVersion: gitTagVersion,
              frameworkVersion: frameworkVersion,
            );
          });

          testUsingContext(
            'contains license and version',
            () async {
              final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('FlutterMacOS.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(podspecContents, contains("'1.13.1113'"));
              expect(podspecContents, contains('# $frameworkVersion'));
              expect(podspecContents, contains(licenseText));
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'debug URL',
            () async {
              final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.debug, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('FlutterMacOS.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/darwin-x64/FlutterMacOS.framework.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'profile URL',
            () async {
              final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.profile, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('FlutterMacOS.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/darwin-x64-profile/FlutterMacOS.framework.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );

          testUsingContext(
            'release URL',
            () async {
              final BuildMacOSFrameworkCommand command = BuildMacOSFrameworkCommand(
                logger: BufferLogger.test(),
                buildSystem: TestBuildSystem.all(BuildResult(success: true)),
                platform: fakePlatform,
                flutterVersion: fakeFlutterVersion,
                cache: cache,
                verboseHelp: false,
              );
              command.produceFlutterPodspec(BuildMode.release, outputDirectory);

              final File expectedPodspec = outputDirectory.childFile('FlutterMacOS.podspec');
              final String podspecContents = expectedPodspec.readAsStringSync();
              expect(
                podspecContents,
                contains(
                  "'$storageBaseUrl/flutter_infra_release/flutter/$engineRevision/darwin-x64-release/FlutterMacOS.framework.zip'",
                ),
              );
            },
            overrides: <Type, Generator>{
              FileSystem: () => memoryFileSystem,
              ProcessManager: () => FakeProcessManager.any(),
            },
          );
        });
      });
    });
  });

  group('XCFrameworks', () {
    late MemoryFileSystem fileSystem;
    late FakeProcessManager fakeProcessManager;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      fakeProcessManager = FakeProcessManager.empty();
    });

    testWithoutContext('created', () async {
      final Directory frameworkA = fileSystem.directory('FrameworkA.framework')..createSync();
      final Directory frameworkB = fileSystem.directory('FrameworkB.framework')..createSync();
      final Directory output = fileSystem.directory('output');

      fakeProcessManager.addCommand(
        FakeCommand(
          command: <String>[
            'xcrun',
            'xcodebuild',
            '-create-xcframework',
            '-framework',
            frameworkA.path,
            '-framework',
            frameworkB.path,
            '-output',
            output.childDirectory('Combine.xcframework').path,
          ],
        ),
      );
      await BuildFrameworkCommand.produceXCFramework(
        <Directory>[frameworkA, frameworkB],
        'Combine',
        output,
        fakeProcessManager,
      );
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    testWithoutContext('created with symbols', () async {
      final Directory parentA = fileSystem.directory('FrameworkA')..createSync();
      final File dSYMA = parentA.childFile('FrameworkA.framework.dSYM')..createSync();
      final Directory frameworkA = parentA.childDirectory('FrameworkA.framework')..createSync();
      // Flutter.framework.dSYM should be correctly filtered out.
      parentA.childFile('Flutter.framework.dSYM').createSync();

      final Directory parentB = fileSystem.directory('FrameworkB')..createSync();
      final File dSYMB = parentB.childFile('FrameworkB.framework.dSYM')..createSync();
      final Directory frameworkB = parentB.childDirectory('FrameworkB.framework')..createSync();
      final Directory output = fileSystem.directory('output');

      fakeProcessManager.addCommand(
        FakeCommand(
          command: <String>[
            'xcrun',
            'xcodebuild',
            '-create-xcframework',
            '-framework',
            frameworkA.path,
            '-debug-symbols',
            dSYMA.path,
            '-framework',
            frameworkB.path,
            '-debug-symbols',
            dSYMB.path,
            '-output',
            output.childDirectory('Combine.xcframework').path,
          ],
        ),
      );
      await BuildFrameworkCommand.produceXCFramework(
        <Directory>[frameworkA, frameworkB],
        'Combine',
        output,
        fakeProcessManager,
      );
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });
  });
}
