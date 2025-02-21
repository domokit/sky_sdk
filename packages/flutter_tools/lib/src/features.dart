// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/context.dart';

/// The current [FeatureFlags] implementation.
FeatureFlags get featureFlags => context.get<FeatureFlags>()!;

/// The interface used to determine if a particular [Feature] is enabled.
///
/// The rest of the tools code should use this class instead of looking up
/// features directly. To facilitate rolls to google3 and other clients, all
/// flags should be provided with a default implementation here. Clients that
/// use this class should extent instead of implement, so that new flags are
/// picked up automatically.
abstract class FeatureFlags {
  /// const constructor so that subclasses can be const.
  const FeatureFlags();

  /// Whether flutter desktop for linux is enabled.
  bool get isLinuxEnabled => false;

  /// Whether flutter desktop for macOS is enabled.
  bool get isMacOSEnabled => false;

  /// Whether flutter web is enabled.
  bool get isWebEnabled => false;

  /// Whether flutter desktop for Windows is enabled.
  bool get isWindowsEnabled => false;

  /// Whether android is enabled.
  bool get isAndroidEnabled => true;

  /// Whether iOS is enabled.
  bool get isIOSEnabled => true;

  /// Whether fuchsia is enabled.
  bool get isFuchsiaEnabled => true;

  /// Whether custom devices are enabled.
  bool get areCustomDevicesEnabled => false;

  /// Whether animations are used in the command line interface.
  bool get isCliAnimationEnabled => true;

  /// Whether native assets compilation and bundling is enabled.
  bool get isNativeAssetsEnabled => false;

  /// Whether Swift Package Manager dependency management is enabled.
  bool get isSwiftPackageManagerEnabled => false;

  /// Whether explicit package dependency management is enabled.
  bool get isExplicitPackageDependenciesEnabled => false;

  /// Whether a particular feature is enabled for the current channel.
  ///
  /// Prefer using one of the specific getters above instead of this API.
  bool isEnabled(Feature feature);
}

/// All current Flutter feature flags.
const List<Feature> allFeatures = <Feature>[
  flutterWebFeature,
  flutterLinuxDesktopFeature,
  flutterMacOSDesktopFeature,
  flutterWindowsDesktopFeature,
  flutterAndroidFeature,
  flutterIOSFeature,
  flutterFuchsiaFeature,
  flutterCustomDevicesFeature,
  cliAnimation,
  nativeAssets,
  swiftPackageManager,
  explicitPackageDependencies,
];

/// All current Flutter feature flags that can be configured.
///
/// [Feature.configSetting] is not `null`.
Iterable<Feature> get allConfigurableFeatures =>
    allFeatures.where((Feature feature) => feature.configSetting != null);

/// The [Feature] for flutter web.
const Feature flutterWebFeature = Feature.fullyEnabled(
  name: 'Flutter for web',
  configSetting: 'enable-web',
  environmentOverride: 'FLUTTER_WEB',
);

/// The [Feature] for macOS desktop.
const Feature flutterMacOSDesktopFeature = Feature.fullyEnabled(
  name: 'support for desktop on macOS',
  configSetting: 'enable-macos-desktop',
  environmentOverride: 'FLUTTER_MACOS',
);

/// The [Feature] for Linux desktop.
const Feature flutterLinuxDesktopFeature = Feature.fullyEnabled(
  name: 'support for desktop on Linux',
  configSetting: 'enable-linux-desktop',
  environmentOverride: 'FLUTTER_LINUX',
);

/// The [Feature] for Windows desktop.
const Feature flutterWindowsDesktopFeature = Feature.fullyEnabled(
  name: 'support for desktop on Windows',
  configSetting: 'enable-windows-desktop',
  environmentOverride: 'FLUTTER_WINDOWS',
);

/// The [Feature] for Android devices.
const Feature flutterAndroidFeature = Feature.fullyEnabled(
  name: 'Flutter for Android',
  configSetting: 'enable-android',
);

/// The [Feature] for iOS devices.
const Feature flutterIOSFeature = Feature.fullyEnabled(
  name: 'Flutter for iOS',
  configSetting: 'enable-ios',
);

/// The [Feature] for Fuchsia support.
const Feature flutterFuchsiaFeature = Feature(
  name: 'Flutter for Fuchsia',
  configSetting: 'enable-fuchsia',
  environmentOverride: 'FLUTTER_FUCHSIA',
  master: FeatureChannelSetting(available: true),
);

const Feature flutterCustomDevicesFeature = Feature(
  name: 'early support for custom device types',
  configSetting: 'enable-custom-devices',
  environmentOverride: 'FLUTTER_CUSTOM_DEVICES',
  master: FeatureChannelSetting(available: true),
  beta: FeatureChannelSetting(available: true),
  stable: FeatureChannelSetting(available: true),
);

/// The [Feature] for CLI animations.
///
/// The TERM environment variable set to "dumb" turns this off.
const Feature cliAnimation = Feature.fullyEnabled(
  name: 'animations in the command line interface',
  configSetting: 'cli-animations',
);

/// Enable native assets compilation and bundling.
const Feature nativeAssets = Feature(
  name: 'native assets compilation and bundling',
  configSetting: 'enable-native-assets',
  environmentOverride: 'FLUTTER_NATIVE_ASSETS',
  master: FeatureChannelSetting(available: true),
);

/// Enable Swift Package Manager as a darwin dependency manager.
const Feature swiftPackageManager = Feature(
  name: 'support for Swift Package Manager for iOS and macOS',
  configSetting: 'enable-swift-package-manager',
  environmentOverride: 'FLUTTER_SWIFT_PACKAGE_MANAGER',
  master: FeatureChannelSetting(available: true),
  beta: FeatureChannelSetting(available: true),
  stable: FeatureChannelSetting(available: true),
);

/// Enable explicit resolution and generation of package dependencies.
const Feature explicitPackageDependencies = Feature.fullyEnabled(
  name: 'support for dev_dependency plugins',
  configSetting: 'explicit-package-dependencies',
  extraHelpText:
      'Plugins that are resolved as result of being in "dev_dependencies" of a '
      'package are not included in release builds of an app. By enabling this '
      'feature, the synthetic "package:flutter_gen" can no longer be generated '
      'and the legacy ".flutter-plugins" tool artifact is no longer generated.\n'
      '\n'
      'See also:\n'
      '* https://flutter.dev/to/flutter-plugins-configuration.\n'
      '* https://flutter.dev/to/flutter-gen-deprecation.',
);

/// A [Feature] is a process for conditionally enabling tool features.
///
/// All settings are optional, and if not provided will generally default to
/// a "safe" value, such as being off.
///
/// The top level feature settings can be provided to apply to all channels.
/// Otherwise, more specific settings take precedence over higher level
/// settings.
class Feature {
  /// Creates a [Feature].
  const Feature({
    required this.name,
    this.environmentOverride,
    this.configSetting,
    this.extraHelpText,
    this.master = const FeatureChannelSetting(),
    this.beta = const FeatureChannelSetting(),
    this.stable = const FeatureChannelSetting(),
  });

  /// Creates a [Feature] that is fully enabled across channels.
  const Feature.fullyEnabled({
    required this.name,
    this.environmentOverride,
    this.configSetting,
    this.extraHelpText,
  }) : master = const FeatureChannelSetting(available: true, enabledByDefault: true),
       beta = const FeatureChannelSetting(available: true, enabledByDefault: true),
       stable = const FeatureChannelSetting(available: true, enabledByDefault: true);

  /// The user visible name for this feature.
  final String name;

  /// The settings for the master branch and other unknown channels.
  final FeatureChannelSetting master;

  /// The settings for the beta branch.
  final FeatureChannelSetting beta;

  /// The settings for the stable branch.
  final FeatureChannelSetting stable;

  /// The name of an environment variable that can override the setting.
  ///
  /// The environment variable needs to be set to the value 'true'. This is
  /// only intended for usage by CI and not as an advertised method to enable
  /// a feature.
  ///
  /// If not provided, defaults to `null` meaning there is no override.
  final String? environmentOverride;

  /// The name of a setting that can be used to enable this feature.
  ///
  /// If not provided, defaults to `null` meaning there is no config setting.
  final String? configSetting;

  /// Additional text to add to the end of the help message.
  ///
  /// If not provided, defaults to `null` meaning there is no additional text.
  final String? extraHelpText;

  /// A help message for the `flutter config` command, or null if unsupported.
  String? generateHelpMessage() {
    if (configSetting == null) {
      return null;
    }
    final StringBuffer buffer = StringBuffer('Enable or disable $name.');
    final List<String> channels = <String>[
      if (master.available) 'master',
      if (beta.available) 'beta',
      if (stable.available) 'stable',
    ];
    // Add channel info for settings only on some channels.
    if (channels.length == 1) {
      buffer.write('\nThis setting applies only to the ${channels.single} channel.');
    } else if (channels.length == 2) {
      buffer.write('\nThis setting applies only to the ${channels.join(' and ')} channels.');
    }
    if (extraHelpText != null) {
      buffer.write(' $extraHelpText');
    }
    return buffer.toString();
  }

  /// Retrieve the correct setting for the provided `channel`.
  FeatureChannelSetting getSettingForChannel(String channel) {
    return switch (channel) {
      'stable' => stable,
      'beta' => beta,
      'master' || _ => master,
    };
  }
}

/// A description of the conditions to enable a feature for a particular channel.
class FeatureChannelSetting {
  const FeatureChannelSetting({this.available = false, this.enabledByDefault = false});

  /// Whether the feature is available on this channel.
  ///
  /// If not provided, defaults to `false`. This implies that the feature
  /// cannot be enabled even by the settings below.
  final bool available;

  /// Whether the feature is enabled by default.
  ///
  /// If not provided, defaults to `false`.
  final bool enabledByDefault;
}
