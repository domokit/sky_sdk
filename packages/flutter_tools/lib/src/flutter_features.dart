// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/config.dart';
import 'base/platform.dart';
import 'features.dart';
import 'version.dart';

class FlutterFeatureFlags implements FeatureFlags {
  FlutterFeatureFlags({
    required FlutterVersion flutterVersion,
    required Config config,
    required Platform platform,
  }) : _flutterVersion = flutterVersion,
       _config = config,
       _platform = platform;

  final FlutterVersion _flutterVersion;
  final Config _config;
  final Platform _platform;

  @override
  bool get isLinuxEnabled => isEnabled(flutterLinuxDesktopFeature);

  @override
  bool get isMacOSEnabled => isEnabled(flutterMacOSDesktopFeature);

  @override
  bool get isWebEnabled => isEnabled(flutterWebFeature);

  @override
  bool get isWindowsEnabled => isEnabled(flutterWindowsDesktopFeature);

  @override
  bool get isAndroidEnabled => isEnabled(flutterAndroidFeature);

  @override
  bool get isIOSEnabled => isEnabled(flutterIOSFeature);

  @override
  bool get isFuchsiaEnabled => isEnabled(flutterFuchsiaFeature);

  @override
  bool get areCustomDevicesEnabled => isEnabled(flutterCustomDevicesFeature);

  @override
  bool get isCliAnimationEnabled {
    if (_platform.environment['TERM'] == 'dumb') {
      return false;
    }
    return isEnabled(cliAnimation);
  }

  @override
  bool get isNativeAssetsEnabled => isEnabled(nativeAssets);

  @override
  bool get isPreviewDeviceEnabled => isEnabled(previewDevice);

  @override
  bool get isSwiftPackageManagerEnabled => isEnabled(swiftPackageManager);

  @override
  bool get isSwiftPackageManagerMigrationEnabled {
    if (!isEnabled(swiftPackageManager)) {
      return false;
    }

    return isEnabled(swiftPackageManagerMigration);
  }

  @override
  bool get isExplicitPackageDependenciesEnabled => isEnabled(explicitPackageDependencies);

  @override
  bool isEnabled(Feature feature) {
    final String currentChannel = _flutterVersion.channel;
    final FeatureChannelSetting featureSetting = feature.getSettingForChannel(currentChannel);
    if (!featureSetting.available) {
      return false;
    }
    bool isEnabled = featureSetting.enabledByDefault;
    if (feature.configSetting != null) {
      final bool? configOverride = _config.getValue(feature.configSetting!) as bool?;
      if (configOverride != null) {
        isEnabled = configOverride;
      }
    }
    if (feature.environmentOverride != null) {
      if (_platform.environment[feature.environmentOverride]?.toLowerCase() == 'true') {
        isEnabled = true;
      }
    }
    return isEnabled;
  }
}
