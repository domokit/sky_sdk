// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/android_workflow.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/commands/config.dart';
import 'package:flutter_tools/src/commands/doctor.dart';
import 'package:flutter_tools/src/doctor.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';
import 'package:usage/usage_io.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fakes.dart';
import '../src/test_build_system.dart';
import '../src/test_flutter_command_runner.dart';

void main() {
  setUpAll(() {
    Cache.disableLocking();
  });

  group('analytics', () {
    late Directory tempDir;
    late Config testConfig;
    late FileSystem fs;

    const String flutterRoot = '/path/to/flutter';

    setUp(() {
      Cache.flutterRoot = flutterRoot;
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_analytics_test.');
      testConfig = Config.test();
      fs = MemoryFileSystem.test();
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    // Ensure we don't send anything when analytics is disabled.
    testUsingContext(
      "doesn't send when disabled",
      () async {
        int count = 0;
        globals.flutterUsage.onSend.listen((Map<String, dynamic> data) => count++);

        final FlutterCommand command = FakeFlutterCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        globals.flutterUsage.enabled = false;
        await runner.run(<String>['fake']);
        expect(count, 0);

        globals.flutterUsage.enabled = true;
        await runner.run(<String>['fake']);
        // LogToFileAnalytics isFirstRun is hardcoded to false
        // so this usage will never act like the first run
        // (which would not send usage).
        expect(count, 4);

        count = 0;
        globals.flutterUsage.enabled = false;
        await runner.run(<String>['fake']);

        expect(count, 0);
      },
      overrides: <Type, Generator>{
        FlutterVersion: () => FakeFlutterVersion(),
        Usage:
            () => Usage(
              configDirOverride: tempDir.path,
              logFile: tempDir.childFile('analytics.log').path,
              runningOnBot: true,
            ),
      },
    );

    // Ensure we don't send for the 'flutter config' command.
    testUsingContext(
      "config doesn't send",
      () async {
        int count = 0;
        globals.flutterUsage.onSend.listen((Map<String, dynamic> data) => count++);

        globals.flutterUsage.enabled = false;
        final ConfigCommand command = ConfigCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['config']);
        expect(count, 0);

        globals.flutterUsage.enabled = true;
        await runner.run(<String>['config']);

        expect(count, 0);
      },
      overrides: <Type, Generator>{
        FlutterVersion: () => FakeFlutterVersion(),
        Usage:
            () => Usage(
              configDirOverride: tempDir.path,
              logFile: tempDir.childFile('analytics.log').path,
              runningOnBot: true,
            ),
      },
    );

    testUsingContext(
      'Usage records one feature in experiment setting',
      () async {
        testConfig.setValue(flutterWebFeature.configSetting!, true);
        final Usage usage = Usage(runningOnBot: true);
        usage.sendCommand('test');

        final String featuresKey = CustomDimensionsEnum.enabledFlutterFeatures.cdKey;

        expect(globals.fs.file('test').readAsStringSync(), contains('$featuresKey: enable-web'));
      },
      overrides: <Type, Generator>{
        FlutterVersion: () => FakeFlutterVersion(),
        Config: () => testConfig,
        Platform:
            () => FakePlatform(environment: <String, String>{'FLUTTER_ANALYTICS_LOG_FILE': 'test'}),
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Usage records multiple features in experiment setting',
      () async {
        testConfig.setValue(flutterWebFeature.configSetting!, true);
        testConfig.setValue(flutterLinuxDesktopFeature.configSetting!, true);
        testConfig.setValue(flutterMacOSDesktopFeature.configSetting!, true);
        final Usage usage = Usage(runningOnBot: true);
        usage.sendCommand('test');

        final String featuresKey = CustomDimensionsEnum.enabledFlutterFeatures.cdKey;

        expect(
          globals.fs.file('test').readAsStringSync(),
          contains('$featuresKey: enable-web,enable-linux-desktop,enable-macos-desktop'),
        );
      },
      overrides: <Type, Generator>{
        FlutterVersion: () => FakeFlutterVersion(),
        Config: () => testConfig,
        Platform:
            () => FakePlatform(environment: <String, String>{'FLUTTER_ANALYTICS_LOG_FILE': 'test'}),
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('analytics with fakes', () {
    late MemoryFileSystem memoryFileSystem;
    late FakeStdio fakeStdio;
    late TestUsage testUsage;
    late FakeClock fakeClock;
    late FakeDoctor doctor;
    late FakeAndroidStudio androidStudio;
    late ProcessManager processManager;
    late BufferLogger logger;
    late ProcessUtils processUtils;

    setUp(() {
      memoryFileSystem = MemoryFileSystem.test();
      fakeStdio = FakeStdio();
      testUsage = TestUsage();
      fakeClock = FakeClock();
      doctor = FakeDoctor();
      androidStudio = FakeAndroidStudio();
      processManager = FakeProcessManager.empty();
      logger = BufferLogger.test();
      processUtils = ProcessUtils(logger: logger, processManager: processManager);
    });

    testUsingContext(
      'flutter commands send timing events',
      () async {
        fakeClock.times = <int>[1000, 2000];
        doctor.diagnoseSucceeds = true;
        final DoctorCommand command = DoctorCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['doctor']);

        expect(
          testUsage.timings,
          contains(
            const TestTimingEvent(
              'flutter',
              'doctor',
              Duration(milliseconds: 1000),
              label: 'success',
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidStudio: () => androidStudio,
        SystemClock: () => fakeClock,
        Doctor: () => doctor,
        Usage: () => testUsage,
      },
    );

    testUsingContext(
      'doctor fail sends warning',
      () async {
        fakeClock.times = <int>[1000, 2000];
        doctor.diagnoseSucceeds = false;
        final DoctorCommand command = DoctorCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['doctor']);

        expect(
          testUsage.timings,
          contains(
            const TestTimingEvent(
              'flutter',
              'doctor',
              Duration(milliseconds: 1000),
              label: 'warning',
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        AndroidStudio: () => androidStudio,
        SystemClock: () => fakeClock,
        Doctor: () => doctor,
        Usage: () => testUsage,
      },
    );

    testUsingContext('single command usage path', () async {
      final FlutterCommand doctorCommand = DoctorCommand();

      expect(await doctorCommand.usagePath, 'doctor');
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext('compound command usage path', () async {
      final BuildCommand buildCommand = BuildCommand(
        artifacts: Artifacts.test(fileSystem: memoryFileSystem),
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: memoryFileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      final FlutterCommand buildApkCommand = buildCommand.subcommands['apk']! as FlutterCommand;

      expect(await buildApkCommand.usagePath, 'build/apk');
    });

    testUsingContext(
      'command sends localtime',
      () async {
        const int kMillis = 1000;
        fakeClock.times = <int>[kMillis];
        // Since FLUTTER_ANALYTICS_LOG_FILE is set in the environment, analytics
        // will be written to a file.
        final Usage usage = Usage(versionOverride: 'test', runningOnBot: true);
        usage.suppressAnalytics = false;
        usage.enabled = true;

        usage.sendCommand('test');

        final String log = globals.fs.file('analytics.log').readAsStringSync();
        final DateTime dateTime = DateTime.fromMillisecondsSinceEpoch(kMillis);

        expect(log.contains(formatDateTime(dateTime)), isTrue);
      },
      overrides: <Type, Generator>{
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => processManager,
        SystemClock: () => fakeClock,
        Platform:
            () => FakePlatform(
              environment: <String, String>{'FLUTTER_ANALYTICS_LOG_FILE': 'analytics.log'},
            ),
        Stdio: () => fakeStdio,
      },
    );

    testUsingContext(
      'event sends localtime',
      () async {
        const int kMillis = 1000;
        fakeClock.times = <int>[kMillis];
        // Since FLUTTER_ANALYTICS_LOG_FILE is set in the environment, analytics
        // will be written to a file.
        final Usage usage = Usage(versionOverride: 'test', runningOnBot: true);
        usage.suppressAnalytics = false;
        usage.enabled = true;

        usage.sendEvent('test', 'test');

        final String log = globals.fs.file('analytics.log').readAsStringSync();
        final DateTime dateTime = DateTime.fromMillisecondsSinceEpoch(kMillis);

        expect(log.contains(formatDateTime(dateTime)), isTrue);
      },
      overrides: <Type, Generator>{
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => processManager,
        SystemClock: () => fakeClock,
        Platform:
            () => FakePlatform(
              environment: <String, String>{'FLUTTER_ANALYTICS_LOG_FILE': 'analytics.log'},
            ),
        Stdio: () => fakeStdio,
      },
    );
  });

  group('analytics bots', () {
    late Directory tempDir;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_analytics_bots_test.');
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    testUsingContext(
      "don't send on bots with unknown version",
      () async {
        int count = 0;
        globals.flutterUsage.onSend.listen((Map<String, dynamic> data) => count++);
        await createTestCommandRunner().run(<String>['--version']);

        expect(count, 0);
      },
      overrides: <Type, Generator>{
        Usage:
            () => Usage(
              settingsName: 'flutter_bot_test',
              versionOverride: 'dev/unknown',
              configDirOverride: tempDir.path,
              runningOnBot: false,
            ),
      },
    );

    testUsingContext(
      "don't send on bots even when opted in",
      () async {
        int count = 0;
        globals.flutterUsage.onSend.listen((Map<String, dynamic> data) => count++);
        globals.flutterUsage.enabled = true;
        await createTestCommandRunner().run(<String>['--version']);

        expect(count, 0);
      },
      overrides: <Type, Generator>{
        Usage:
            () => Usage(
              settingsName: 'flutter_bot_test',
              versionOverride: 'dev/unknown',
              configDirOverride: tempDir.path,
              runningOnBot: false,
            ),
      },
    );

    testUsingContext('Uses AnalyticsMock when .flutter cannot be created', () async {
      final Usage usage = Usage(
        settingsName: 'flutter_bot_test',
        versionOverride: 'dev/known',
        configDirOverride: tempDir.path,
        analyticsIOFactory: throwingAnalyticsIOFactory,
        runningOnBot: false,
      );
      final AnalyticsMock analyticsMock = AnalyticsMock();

      expect(usage.clientId, analyticsMock.clientId);
      expect(usage.suppressAnalytics, isTrue);
    });
  });
}

Analytics throwingAnalyticsIOFactory(
  String trackingId,
  String applicationName,
  String applicationVersion, {
  String? analyticsUrl,
  Directory? documentDirectory,
}) {
  throw const FileSystemException('Could not create file');
}

class FakeFlutterCommand extends FlutterCommand {
  @override
  String get description => 'A fake command';

  @override
  String get name => 'fake';

  @override
  Future<FlutterCommandResult> runCommand() async {
    return FlutterCommandResult.success();
  }
}

class FakeDoctor extends Fake implements Doctor {
  bool diagnoseSucceeds = false;

  @override
  Future<bool> diagnose({
    bool androidLicenses = false,
    bool verbose = true,
    bool showColor = true,
    AndroidLicenseValidator? androidLicenseValidator,
    bool showPii = true,
    List<ValidatorTask>? startedValidatorTasks,
    bool sendEvent = true,
    FlutterVersion? version,
  }) async {
    return diagnoseSucceeds;
  }
}

class FakeClock extends Fake implements SystemClock {
  List<int> times = <int>[];

  @override
  DateTime now() {
    return DateTime.fromMillisecondsSinceEpoch(times.removeAt(0));
  }
}
