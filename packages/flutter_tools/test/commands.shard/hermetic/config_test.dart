// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/config.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart' as fakes;
import '../../src/test_flutter_command_runner.dart';

void main() {
  late Java fakeJava;
  late FakeAndroidStudio fakeAndroidStudio;
  late FakeAndroidSdk fakeAndroidSdk;
  late FakeFlutterVersion fakeFlutterVersion;
  late TestUsage testUsage;
  late FakeAnalytics fakeAnalytics;

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fakeJava = fakes.FakeJava();
    fakeAndroidStudio = FakeAndroidStudio();
    fakeAndroidSdk = FakeAndroidSdk();
    fakeFlutterVersion = FakeFlutterVersion();
    testUsage = TestUsage();
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: MemoryFileSystem.test(),
      fakeFlutterVersion: fakes.FakeFlutterVersion(),
    );
  });

  void verifyNoAnalytics() {
    expect(testUsage.commands, isEmpty);
    expect(testUsage.events, isEmpty);
    expect(testUsage.timings, isEmpty);
    expect(fakeAnalytics.sentEvents, isEmpty);
  }

  group('config', () {
    testUsingContext('prints all settings with --list', () async {
      final ConfigCommand configCommand = ConfigCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);
      await commandRunner.run(<String>['config', '--list']);
      expect(
        testLogger.statusText,
        'All Settings:\n'
        '${allFeatures.where((Feature e) => e.configSetting != null).map((Feature e) => '  ${e.configSetting}: (Not set)').join('\n')}'
        '\n\n',
      );
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext('throws error on excess arguments', () {
      final ConfigCommand configCommand = ConfigCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

      expect(
        () => commandRunner.run(<String>[
          'config',
          '--android-studio-dir=/opt/My',
          'Android',
          'Studio',
        ]),
        throwsToolExit(),
      );
      verifyNoAnalytics();
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext(
      'machine flag',
      () async {
        final ConfigCommand command = ConfigCommand();
        await command.handleMachine();

        expect(testLogger.statusText, isNotEmpty);
        final dynamic jsonObject = json.decode(testLogger.statusText);
        expect(jsonObject, const TypeMatcher<Map<String, dynamic>>());
        if (jsonObject is Map<String, dynamic>) {
          expect(jsonObject['android-studio-dir'], fakeAndroidStudio.directory);
          expect(jsonObject['android-sdk'], fakeAndroidSdk.directory.path);
          expect(jsonObject['jdk-dir'], fakeJava.javaHome);
        }
        verifyNoAnalytics();
      },
      overrides: <Type, Generator>{
        AndroidStudio: () => fakeAndroidStudio,
        AndroidSdk: () => fakeAndroidSdk,
        Java: () => fakeJava,
        Usage: () => testUsage,
      },
    );

    testUsingContext('Can set build-dir', () async {
      final ConfigCommand configCommand = ConfigCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

      await commandRunner.run(<String>['config', '--build-dir=foo']);

      expect(getBuildDirectory(), 'foo');
      verifyNoAnalytics();
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext('throws error on absolute path to build-dir', () async {
      final ConfigCommand configCommand = ConfigCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

      expect(() => commandRunner.run(<String>['config', '--build-dir=/foo']), throwsToolExit());
      verifyNoAnalytics();
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext(
      'allows setting and removing feature flags',
      () async {
        final ConfigCommand configCommand = ConfigCommand();
        final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

        await commandRunner.run(<String>[
          'config',
          '--enable-android',
          '--enable-ios',
          '--enable-web',
          '--enable-linux-desktop',
          '--enable-windows-desktop',
          '--enable-macos-desktop',
        ]);

        expect(globals.config.getValue('enable-android'), true);
        expect(globals.config.getValue('enable-ios'), true);
        expect(globals.config.getValue('enable-web'), true);
        expect(globals.config.getValue('enable-linux-desktop'), true);
        expect(globals.config.getValue('enable-windows-desktop'), true);
        expect(globals.config.getValue('enable-macos-desktop'), true);

        await commandRunner.run(<String>['config', '--clear-features']);

        expect(globals.config.getValue('enable-android'), null);
        expect(globals.config.getValue('enable-ios'), null);
        expect(globals.config.getValue('enable-web'), null);
        expect(globals.config.getValue('enable-linux-desktop'), null);
        expect(globals.config.getValue('enable-windows-desktop'), null);
        expect(globals.config.getValue('enable-macos-desktop'), null);

        await commandRunner.run(<String>[
          'config',
          '--no-enable-android',
          '--no-enable-ios',
          '--no-enable-web',
          '--no-enable-linux-desktop',
          '--no-enable-windows-desktop',
          '--no-enable-macos-desktop',
        ]);

        expect(globals.config.getValue('enable-android'), false);
        expect(globals.config.getValue('enable-ios'), false);
        expect(globals.config.getValue('enable-web'), false);
        expect(globals.config.getValue('enable-linux-desktop'), false);
        expect(globals.config.getValue('enable-windows-desktop'), false);
        expect(globals.config.getValue('enable-macos-desktop'), false);
        verifyNoAnalytics();
      },
      overrides: <Type, Generator>{
        AndroidStudio: () => fakeAndroidStudio,
        AndroidSdk: () => fakeAndroidSdk,
        Usage: () => testUsage,
      },
    );

    testUsingContext('warns the user to reload IDE', () async {
      final ConfigCommand configCommand = ConfigCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

      await commandRunner.run(<String>['config', '--enable-web']);

      expect(
        testLogger.statusText,
        containsIgnoringWhitespace('You may need to restart any open editors'),
      );
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext(
      'displays which config settings are available on stable',
      () async {
        fakeFlutterVersion.channel = 'stable';
        final ConfigCommand configCommand = ConfigCommand();
        final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

        await commandRunner.run(<String>[
          'config',
          '--enable-web',
          '--enable-linux-desktop',
          '--enable-windows-desktop',
          '--enable-macos-desktop',
        ]);

        await commandRunner.run(<String>['config', '--list']);

        expect(testLogger.statusText, containsIgnoringWhitespace('enable-web: true'));
        expect(testLogger.statusText, containsIgnoringWhitespace('enable-linux-desktop: true'));
        expect(testLogger.statusText, containsIgnoringWhitespace('enable-windows-desktop: true'));
        expect(testLogger.statusText, containsIgnoringWhitespace('enable-macos-desktop: true'));
        verifyNoAnalytics();
      },
      overrides: <Type, Generator>{
        AndroidStudio: () => fakeAndroidStudio,
        AndroidSdk: () => fakeAndroidSdk,
        FlutterVersion: () => fakeFlutterVersion,
        Usage: () => testUsage,
      },
    );

    testUsingContext(
      'no-analytics flag flips usage flag and sends event',
      () async {
        final ConfigCommand configCommand = ConfigCommand();
        final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

        expect(testUsage.enabled, true);
        await commandRunner.run(<String>['config', '--no-analytics']);

        expect(testUsage.enabled, false);

        // Verify that we flushed the analytics queue.
        expect(testUsage.ensureAnalyticsSentCalls, 1);

        // Verify that we only send the analytics disable event, and no other
        // info.
        expect(
          testUsage.events,
          equals(<TestUsageEvent>[const TestUsageEvent('analytics', 'enabled', label: 'false')]),
        );
        expect(testUsage.commands, isEmpty);
        expect(testUsage.timings, isEmpty);
        expect(fakeAnalytics.sentEvents, isEmpty);
      },
      overrides: <Type, Generator>{Usage: () => testUsage},
    );

    testUsingContext(
      'analytics flag flips usage flag and sends event',
      () async {
        final ConfigCommand configCommand = ConfigCommand();
        final CommandRunner<void> commandRunner = createTestCommandRunner(configCommand);

        await commandRunner.run(<String>['config', '--analytics']);

        expect(testUsage.enabled, true);

        // Verify that we only send the analytics enable event, and no other
        // info.
        expect(
          testUsage.events,
          equals(<TestUsageEvent>[const TestUsageEvent('analytics', 'enabled', label: 'true')]),
        );
        expect(testUsage.commands, isEmpty);
        expect(testUsage.timings, isEmpty);
        expect(fakeAnalytics.sentEvents, isEmpty);
      },
      overrides: <Type, Generator>{Usage: () => testUsage},
    );

    testUsingContext('analytics reported with help usages', () async {
      final ConfigCommand configCommand = ConfigCommand();
      createTestCommandRunner(configCommand);

      testUsage.suppressAnalytics = true;
      expect(
        configCommand.usage,
        containsIgnoringWhitespace('Analytics reporting is currently disabled'),
      );

      testUsage.suppressAnalytics = false;
      expect(
        configCommand.usage,
        containsIgnoringWhitespace('Analytics reporting is currently enabled'),
      );
    }, overrides: <Type, Generator>{Usage: () => testUsage});
  });
}

class FakeAndroidStudio extends Fake implements AndroidStudio, Comparable<AndroidStudio> {
  @override
  String get directory => 'path/to/android/studio';

  @override
  String? get javaPath => 'path/to/android/studio/jbr';
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  Directory get directory => globals.fs.directory('path/to/android/sdk');
}

class FakeFlutterVersion extends Fake implements FlutterVersion {
  @override
  late String channel;

  @override
  void ensureVersionFile() {}

  @override
  Future<void> checkFlutterVersionFreshness() async {}
}
