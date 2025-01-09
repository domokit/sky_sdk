// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_features.dart';

import '../src/common.dart';
import '../src/fakes.dart';

void main() {
  group('Features', () {
    late Config testConfig;
    late FakePlatform platform;
    late FlutterFeatureFlags featureFlags;

    setUp(() {
      testConfig = Config.test();
      platform = FakePlatform(environment: <String, String>{});

      for (final Feature feature in allConfigurableFeatures) {
        testConfig.setValue(feature.configSetting!, false);
      }

      featureFlags = FlutterFeatureFlags(
        flutterVersion: FakeFlutterVersion(),
        config: testConfig,
        platform: platform,
      );
    });

    FeatureFlags createFlags(String channel) {
      return FlutterFeatureFlags(
        flutterVersion: FakeFlutterVersion(branch: channel),
        config: testConfig,
        platform: platform,
      );
    }

    testWithoutContext('setting has safe defaults', () {
      const FeatureChannelSetting featureSetting = FeatureChannelSetting();

      expect(featureSetting.available, false);
      expect(featureSetting.enabledByDefault, false);
    });

    testWithoutContext('has safe defaults', () {
      const Feature feature = Feature(name: 'example');

      expect(feature.name, 'example');
      expect(feature.environmentOverride, null);
      expect(feature.configSetting, null);
    });

    testWithoutContext('retrieves the correct setting for each branch', () {
      const FeatureChannelSetting masterSetting = FeatureChannelSetting(available: true);
      const FeatureChannelSetting betaSetting = FeatureChannelSetting(available: true);
      const FeatureChannelSetting stableSetting = FeatureChannelSetting(available: true);
      const Feature feature = Feature(
        name: 'example',
        master: masterSetting,
        beta: betaSetting,
        stable: stableSetting,
      );

      expect(feature.getSettingForChannel('master'), masterSetting);
      expect(feature.getSettingForChannel('beta'), betaSetting);
      expect(feature.getSettingForChannel('stable'), stableSetting);
      expect(feature.getSettingForChannel('unknown'), masterSetting);
    });

    testWithoutContext('env variables are only enabled with "true" string', () {
      platform.environment = <String, String>{'FLUTTER_WEB': 'hello'};

      expect(featureFlags.isWebEnabled, false);

      platform.environment = <String, String>{'FLUTTER_WEB': 'true'};

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web help string', () {
      expect(flutterWebFeature.generateHelpMessage(), 'Enable or disable Flutter for web.');
    });

    testWithoutContext('Flutter macOS desktop help string', () {
      expect(
        flutterMacOSDesktopFeature.generateHelpMessage(),
        'Enable or disable support for desktop on macOS.',
      );
    });

    testWithoutContext('Flutter Linux desktop help string', () {
      expect(
        flutterLinuxDesktopFeature.generateHelpMessage(),
        'Enable or disable support for desktop on Linux.',
      );
    });

    testWithoutContext('Flutter Windows desktop help string', () {
      expect(
        flutterWindowsDesktopFeature.generateHelpMessage(),
        'Enable or disable support for desktop on Windows.',
      );
    });

    testWithoutContext('help string on multiple channels', () {
      const Feature testWithoutContextFeature = Feature(
        name: 'example',
        master: FeatureChannelSetting(available: true),
        beta: FeatureChannelSetting(available: true),
        stable: FeatureChannelSetting(available: true),
        configSetting: 'foo',
      );

      expect(testWithoutContextFeature.generateHelpMessage(), 'Enable or disable example.');
    });

    /// Flutter Web

    testWithoutContext('Flutter web off by default on master', () {
      final FeatureFlags featureFlags = createFlags('master');

      expect(featureFlags.isWebEnabled, false);
    });

    testWithoutContext('Flutter web enabled with config on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      testConfig.setValue('enable-web', true);

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web enabled with environment variable on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      platform.environment = <String, String>{'FLUTTER_WEB': 'true'};

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web off by default on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');

      expect(featureFlags.isWebEnabled, false);
    });

    testWithoutContext('Flutter web enabled with config on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      testConfig.setValue('enable-web', true);

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web not enabled with environment variable on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      platform.environment = <String, String>{'FLUTTER_WEB': 'true'};

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web on by default on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      testConfig.removeValue('enable-web');

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web enabled with config on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      testConfig.setValue('enable-web', true);

      expect(featureFlags.isWebEnabled, true);
    });

    testWithoutContext('Flutter web not enabled with environment variable on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      platform.environment = <String, String>{'FLUTTER_WEB': 'enabled'};

      expect(featureFlags.isWebEnabled, false);
    });

    /// Flutter macOS desktop.

    testWithoutContext('Flutter macos desktop off by default on master', () {
      final FeatureFlags featureFlags = createFlags('master');

      expect(featureFlags.isMacOSEnabled, false);
    });

    testWithoutContext('Flutter macos desktop enabled with config on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      testConfig.setValue('enable-macos-desktop', true);

      expect(featureFlags.isMacOSEnabled, true);
    });

    testWithoutContext('Flutter macos desktop enabled with environment variable on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      platform.environment = <String, String>{'FLUTTER_MACOS': 'true'};

      expect(featureFlags.isMacOSEnabled, true);
    });

    testWithoutContext('Flutter macos desktop off by default on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');

      expect(featureFlags.isMacOSEnabled, false);
    });

    testWithoutContext('Flutter macos desktop enabled with config on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      testConfig.setValue('enable-macos-desktop', true);

      expect(featureFlags.isMacOSEnabled, true);
    });

    testWithoutContext('Flutter macos desktop enabled with environment variable on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      platform.environment = <String, String>{'FLUTTER_MACOS': 'true'};

      expect(featureFlags.isMacOSEnabled, true);
    });

    testWithoutContext('Flutter macos desktop off by default on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');

      expect(featureFlags.isMacOSEnabled, false);
    });

    testWithoutContext('Flutter macos desktop enabled with config on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      testConfig.setValue('enable-macos-desktop', true);

      expect(featureFlags.isMacOSEnabled, true);
    });

    testWithoutContext('Flutter macos desktop enabled with environment variable on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      platform.environment = <String, String>{'FLUTTER_MACOS': 'true'};

      expect(featureFlags.isMacOSEnabled, true);
    });

    /// Flutter Linux Desktop
    testWithoutContext('Flutter linux desktop off by default on master', () {
      final FeatureFlags featureFlags = createFlags('stable');

      expect(featureFlags.isLinuxEnabled, false);
    });

    testWithoutContext('Flutter linux desktop enabled with config on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      testConfig.setValue('enable-linux-desktop', true);

      expect(featureFlags.isLinuxEnabled, true);
    });

    testWithoutContext('Flutter linux desktop enabled with environment variable on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      platform.environment = <String, String>{'FLUTTER_LINUX': 'true'};

      expect(featureFlags.isLinuxEnabled, true);
    });

    testWithoutContext('Flutter linux desktop off by default on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');

      expect(featureFlags.isLinuxEnabled, false);
    });

    testWithoutContext('Flutter linux desktop enabled with config on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      testConfig.setValue('enable-linux-desktop', true);

      expect(featureFlags.isLinuxEnabled, true);
    });

    testWithoutContext('Flutter linux desktop enabled with environment variable on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      platform.environment = <String, String>{'FLUTTER_LINUX': 'true'};

      expect(featureFlags.isLinuxEnabled, true);
    });

    testWithoutContext('Flutter linux desktop off by default on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');

      expect(featureFlags.isLinuxEnabled, false);
    });

    testWithoutContext('Flutter linux desktop enabled with config on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      testConfig.setValue('enable-linux-desktop', true);

      expect(featureFlags.isLinuxEnabled, true);
    });

    testWithoutContext('Flutter linux desktop enabled with environment variable on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      platform.environment = <String, String>{'FLUTTER_LINUX': 'true'};

      expect(featureFlags.isLinuxEnabled, true);
    });

    /// Flutter Windows desktop.
    testWithoutContext('Flutter Windows desktop off by default on master', () {
      final FeatureFlags featureFlags = createFlags('master');

      expect(featureFlags.isWindowsEnabled, false);
    });

    testWithoutContext('Flutter Windows desktop enabled with config on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      testConfig.setValue('enable-windows-desktop', true);

      expect(featureFlags.isWindowsEnabled, true);
    });

    testWithoutContext('Flutter Windows desktop enabled with environment variable on master', () {
      final FeatureFlags featureFlags = createFlags('master');
      platform.environment = <String, String>{'FLUTTER_WINDOWS': 'true'};

      expect(featureFlags.isWindowsEnabled, true);
    });

    testWithoutContext('Flutter Windows desktop off by default on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');

      expect(featureFlags.isWindowsEnabled, false);
    });

    testWithoutContext('Flutter Windows desktop enabled with config on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      testConfig.setValue('enable-windows-desktop', true);

      expect(featureFlags.isWindowsEnabled, true);
    });

    testWithoutContext('Flutter Windows desktop enabled with environment variable on beta', () {
      final FeatureFlags featureFlags = createFlags('beta');
      platform.environment = <String, String>{'FLUTTER_WINDOWS': 'true'};

      expect(featureFlags.isWindowsEnabled, true);
    });

    testWithoutContext('Flutter Windows desktop off by default on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');

      expect(featureFlags.isWindowsEnabled, false);
    });

    testWithoutContext('Flutter Windows desktop enabled with config on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      testConfig.setValue('enable-windows-desktop', true);

      expect(featureFlags.isWindowsEnabled, true);
    });

    testWithoutContext('Flutter Windows desktop enabled with environment variable on stable', () {
      final FeatureFlags featureFlags = createFlags('stable');
      platform.environment = <String, String>{'FLUTTER_WINDOWS': 'true'};

      expect(featureFlags.isWindowsEnabled, true);
    });

    for (final Feature feature in <Feature>[
      flutterWindowsDesktopFeature,
      flutterMacOSDesktopFeature,
      flutterLinuxDesktopFeature,
    ]) {
      test('${feature.name} available and enabled by default on master', () {
        expect(feature.master.enabledByDefault, true);
        expect(feature.master.available, true);
      });
      test('${feature.name} available and enabled by default on beta', () {
        expect(feature.beta.enabledByDefault, true);
        expect(feature.beta.available, true);
      });
      test('${feature.name} available and enabled by default on stable', () {
        expect(feature.stable.enabledByDefault, true);
        expect(feature.stable.available, true);
      });
    }

    // Custom devices on all channels
    for (final String channel in <String>['master', 'beta', 'stable']) {
      testWithoutContext('Custom devices are enabled with flag on $channel', () {
        final FeatureFlags featureFlags = createFlags(channel);
        testConfig.setValue('enable-custom-devices', true);
        expect(featureFlags.areCustomDevicesEnabled, true);
      });

      testWithoutContext('Custom devices are enabled with environment variable on $channel', () {
        final FeatureFlags featureFlags = createFlags(channel);
        platform.environment = <String, String>{'FLUTTER_CUSTOM_DEVICES': 'true'};
        expect(featureFlags.areCustomDevicesEnabled, true);
      });
    }

    test('${nativeAssets.name} availability and default enabled', () {
      expect(nativeAssets.master.enabledByDefault, false);
      expect(nativeAssets.master.available, true);
      expect(nativeAssets.beta.enabledByDefault, false);
      expect(nativeAssets.beta.available, false);
      expect(nativeAssets.stable.enabledByDefault, false);
      expect(nativeAssets.stable.available, false);
    });

    group('Swift Package Manager feature', () {
      test('availability and default enabled', () {
        expect(swiftPackageManager.master.enabledByDefault, false);
        expect(swiftPackageManager.master.available, true);
        expect(swiftPackageManager.beta.enabledByDefault, false);
        expect(swiftPackageManager.beta.available, true);
        expect(swiftPackageManager.stable.enabledByDefault, false);
        expect(swiftPackageManager.stable.available, true);
      });

      test('can be enabled', () {
        platform.environment = <String, String>{'FLUTTER_SWIFT_PACKAGE_MANAGER': 'true'};

        expect(featureFlags.isSwiftPackageManagerEnabled, isTrue);
      });
    });
  });
}
