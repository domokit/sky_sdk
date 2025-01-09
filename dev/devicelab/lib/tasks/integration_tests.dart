// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/talkback.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

TaskFunction createChannelsIntegrationTest() {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/channels',
    'integration_test/main_test.dart',
  ).call;
}

TaskFunction createPlatformInteractionTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/platform_interaction',
    'lib/main.dart',
  ).call;
}

TaskFunction createFlavorsTest({Map<String, String>? environment}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/flavors',
    'lib/main.dart',
    extraOptions: <String>['--flavor', 'paid'],
    environment: environment,
  ).call;
}

TaskFunction createIntegrationTestFlavorsTest({Map<String, String>? environment}) {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/flavors',
    'integration_test/integration_test.dart',
    extraOptions: <String>['--flavor', 'paid'],
    environment: environment,
  ).call;
}

TaskFunction createExternalTexturesFrameRateIntegrationTest({
  List<String> extraOptions = const <String>[],
}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/external_textures',
    'lib/frame_rate_main.dart',
    extraOptions: extraOptions,
  ).call;
}

TaskFunction createPlatformChannelSampleTest({String? deviceIdOverride}) {
  return DriverTest(
    '${flutterDirectory.path}/examples/platform_channel',
    'test_driver/button_tap.dart',
    deviceIdOverride: deviceIdOverride,
  ).call;
}

TaskFunction createPlatformChannelSwiftSampleTest() {
  return DriverTest(
    '${flutterDirectory.path}/examples/platform_channel_swift',
    'test_driver/button_tap.dart',
  ).call;
}

TaskFunction createEmbeddedAndroidViewsIntegrationTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/android_views',
    'lib/main.dart',
  ).call;
}

TaskFunction createHybridAndroidViewsIntegrationTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/hybrid_android_views',
    'lib/main.dart',
  ).call;
}

TaskFunction createAndroidSemanticsIntegrationTest() {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/android_semantics_testing',
    'integration_test/main_test.dart',
    withTalkBack: true,
  ).call;
}

TaskFunction createIOSPlatformViewTests() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ios_platform_view_tests',
    'lib/main.dart',
    extraOptions: <String>['--dart-define=ENABLE_DRIVER_EXTENSION=true'],
  ).call;
}

TaskFunction createEndToEndKeyboardTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/keyboard_resize.dart',
  ).call;
}

TaskFunction createEndToEndFrameNumberTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/frame_number.dart',
  ).call;
}

TaskFunction createEndToEndDriverTest({Map<String, String>? environment}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/driver.dart',
    environment: environment,
  ).call;
}

TaskFunction createEndToEndScreenshotTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/screenshot.dart',
  ).call;
}

TaskFunction createEndToEndKeyboardTextfieldTest() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/keyboard_textfield.dart',
  ).call;
}

TaskFunction createSolidColorTest({required bool enableImpeller}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/solid_color.dart',
    extraOptions: <String>[if (enableImpeller) '--enable-impeller'],
  ).call;
}

TaskFunction dartDefinesTask() {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/defines.dart',
    extraOptions: <String>[
      '--dart-define=test.valueA=Example,A',
      '--dart-define=test.valueB=Value',
    ],
  ).call;
}

TaskFunction createEndToEndIntegrationTest() {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'integration_test/integration_test.dart',
  ).call;
}

TaskFunction createSpellCheckIntegrationTest() {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/spell_check',
    'integration_test/integration_test.dart',
  ).call;
}

TaskFunction createWindowsStartupDriverTest({String? deviceIdOverride}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/windows_startup_test',
    'lib/main.dart',
    deviceIdOverride: deviceIdOverride,
  ).call;
}

TaskFunction createWideGamutTest() {
  return IntegrationTest(
    '${flutterDirectory.path}/dev/integration_tests/wide_gamut_test',
    'integration_test/app_test.dart',
    createPlatforms: <String>['ios'],
  ).call;
}

class DriverTest {
  DriverTest(
    this.testDirectory,
    this.testTarget, {
    this.extraOptions = const <String>[],
    this.deviceIdOverride,
    this.environment,
  });

  final String testDirectory;
  final String testTarget;
  final List<String> extraOptions;
  final String? deviceIdOverride;
  final Map<String, String>? environment;

  Future<TaskResult> call() {
    return inDirectory<TaskResult>(testDirectory, () async {
      String deviceId;
      if (deviceIdOverride != null) {
        deviceId = deviceIdOverride!;
      } else {
        final Device device = await devices.workingDevice;
        await device.unlock();
        deviceId = device.deviceId;
      }
      await flutter('packages', options: <String>['get']);

      final List<String> options = <String>[
        '--no-android-gradle-daemon',
        '-v',
        '-t',
        testTarget,
        '-d',
        deviceId,
        ...extraOptions,
      ];
      await flutter('drive', options: options, environment: environment);

      return TaskResult.success(null);
    });
  }
}

class IntegrationTest {
  IntegrationTest(
    this.testDirectory,
    this.testTarget, {
    this.extraOptions = const <String>[],
    this.createPlatforms = const <String>[],
    this.withTalkBack = false,
    this.environment,
  });

  final String testDirectory;
  final String testTarget;
  final List<String> extraOptions;
  final List<String> createPlatforms;
  final bool withTalkBack;
  final Map<String, String>? environment;

  Future<TaskResult> call() {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;
      await device.unlock();
      final String deviceId = device.deviceId;
      await flutter('packages', options: <String>['get']);

      if (createPlatforms.isNotEmpty) {
        await flutter(
          'create',
          options: <String>['--platforms', createPlatforms.join(','), '--no-overwrite', '.'],
        );
      }

      if (withTalkBack) {
        if (device is! AndroidDevice) {
          return TaskResult.failure(
            'A test that enables TalkBack can only be run on Android devices',
          );
        }
        await enableTalkBack();
      }

      final List<String> options = <String>['-v', '-d', deviceId, testTarget, ...extraOptions];
      await flutter('test', options: options, environment: environment);

      if (withTalkBack) {
        await disableTalkBack();
      }

      return TaskResult.success(null);
    });
  }
}
