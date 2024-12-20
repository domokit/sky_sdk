// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late BufferLogger logger;
  late MemoryFileSystem fs;
  late Artifacts artifacts;
  late FakeProcessManager processManager;
  late Platform platform;
  late Cache cache;
  late FakeAnalytics fakeAnalytics;

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fs = MemoryFileSystem.test();
    final Directory flutterRoot = fs.directory('flutter');
    Cache.flutterRoot = flutterRoot.path;
    artifacts = Artifacts.test(fileSystem: fs);
    logger = BufferLogger.test();
    platform = FakePlatform(environment: const <String, String>{'PATH': ''});
    processManager = FakeProcessManager.empty();
    cache = Cache.test(rootOverride: flutterRoot, logger: logger, processManager: processManager);
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fs,
      fakeFlutterVersion: FakeFlutterVersion(),
    );
  });

  testUsingContext(
    'will not build an AAR for a plugin',
    () async {
      fs.file('pubspec.yaml').writeAsStringSync('''
name: foo_bar

flutter:
  plugin:
    platforms:
      some_platform:
        null
''');

      final BuildCommand command = BuildCommand(
        androidSdk: FakeAndroidSdk(),
        artifacts: artifacts,
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fs,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        processUtils: ProcessUtils(logger: logger, processManager: processManager),
      );

      expect(
        createTestCommandRunner(command).run(const <String>['build', 'aar', '--no-pub']),
        throwsToolExit(message: 'AARs can only be built from modules'),
      );
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      Cache: () => cache,
      FileSystem: () => fs,
      Platform: () => platform,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'will build an AAR for a module',
    () async {
      fs.file('pubspec.yaml').writeAsStringSync('''
name: foo_bar

flutter:
  module:
    foo: bar
''');
      final Directory dotAndroidDir = fs.directory('.android')..createSync(recursive: true);
      dotAndroidDir.childFile('gradlew').createSync();

      processManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['chmod', '755', 'flutter/bin/cache/artifacts']),
        const FakeCommand(command: <String>['which', 'java']),
        ...<String>['Debug', 'Profile', 'Release'].map(
          (String buildMode) => FakeCommand(
            command: <Pattern>[
              '/.android/gradlew',
              '-I=/flutter/packages/flutter_tools/gradle/aar_init_script.gradle',
              ...List<RegExp>.filled(4, RegExp(r'-P[a-zA-Z-]+=.*')),
              '-q',
              ...List<RegExp>.filled(5, RegExp(r'-P[a-zA-Z-]+=.*')),
              'assembleAar$buildMode',
            ],
            onRun: (_) => fs.directory('/build/host/outputs/repo').createSync(recursive: true),
          ),
        ),
      ]);

      cache.getArtifactDirectory('gradle_wrapper').createSync(recursive: true);

      final BuildCommand command = BuildCommand(
        androidSdk: FakeAndroidSdk(),
        artifacts: artifacts,
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fs,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        processUtils: ProcessUtils(logger: logger, processManager: processManager),
      );

      await createTestCommandRunner(command).run(const <String>['build', 'aar', '--no-pub']);
      expect(processManager, hasNoRemainingExpectations);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.commandUsageValues(
            workflow: 'build/aar',
            commandHasTerminal: false,
            buildAarProjectType: 'module',
            buildAarTargetPlatform: 'android-arm,android-arm64,android-x64',
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      Platform: () => platform,
      ProcessManager: () => processManager,
      Analytics: () => fakeAnalytics,
    },
  );
}
