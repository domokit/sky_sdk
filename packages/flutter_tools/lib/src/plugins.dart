// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:pub_semver/pub_semver.dart';
import 'package:yaml/yaml.dart';

import 'base/common.dart';
import 'base/file_system.dart';
import 'platform_plugins.dart';

class Plugin {
  Plugin({
    required this.name,
    required this.path,
    required this.platforms,
    required this.defaultPackagePlatforms,
    required this.pluginDartClassPlatforms,
    this.flutterConstraint,
    required this.dependencies,
    required this.isDirectDependency,
    required this.isDevDependency,
    this.implementsPackage,
  });

  /// Parses [Plugin] specification from the provided pluginYaml.
  ///
  /// This currently supports two formats. Legacy and Multi-platform.
  ///
  /// Example of the deprecated Legacy format.
  ///
  ///     flutter:
  ///      plugin:
  ///        androidPackage: io.flutter.plugins.sample
  ///        iosPrefix: FLT
  ///        pluginClass: SamplePlugin
  ///
  /// Example Multi-platform format.
  ///
  ///     flutter:
  ///      plugin:
  ///        platforms:
  ///          android:
  ///            package: io.flutter.plugins.sample
  ///            pluginClass: SamplePlugin
  ///          ios:
  ///            # A plugin implemented through method channels.
  ///            pluginClass: SamplePlugin
  ///          linux:
  ///            # A plugin implemented purely in Dart code.
  ///            dartPluginClass: SamplePlugin
  ///            # Optional field to determine file containing dartPluginClass.
  ///            # This file will be used in imports in generated files, e.g.:
  ///            #   import 'package:{{pluginName}}/{{dartFileName}}'
  ///            # instead of default:
  ///            #    import 'package:{{pluginName}}/{{pluginName}}.dart'
  ///            dartFileName: src/sample_plugin.dart
  ///          macos:
  ///            # A plugin implemented with `dart:ffi`.
  ///            ffiPlugin: true
  ///          windows:
  ///            # A plugin using platform-specific Dart and method channels.
  ///            dartPluginClass: SamplePlugin
  ///            pluginClass: SamplePlugin
  factory Plugin.fromYaml(
    String name,
    String path,
    YamlMap? pluginYaml,
    VersionConstraint? flutterConstraint,
    List<String> dependencies, {
    required FileSystem fileSystem,
    required bool isDevDependency,
    Set<String>? appDependencies,
  }) {
    final List<String> errors = validatePluginYaml(pluginYaml);
    if (errors.isNotEmpty) {
      throwToolExit('Invalid plugin specification $name.\n${errors.join('\n')}');
    }
    if (pluginYaml != null && pluginYaml['platforms'] != null) {
      return Plugin._fromMultiPlatformYaml(
        name,
        path,
        pluginYaml,
        flutterConstraint,
        dependencies,
        fileSystem,
        isDevDependency: isDevDependency,
        appDependencies != null && appDependencies.contains(name),
      );
    }
    return Plugin._fromLegacyYaml(
      name,
      path,
      pluginYaml,
      flutterConstraint,
      dependencies,
      fileSystem,
      isDevDependency: isDevDependency,
      appDependencies != null && appDependencies.contains(name),
    );
  }

  factory Plugin._fromMultiPlatformYaml(
    String name,
    String path,
    YamlMap pluginYaml,
    VersionConstraint? flutterConstraint,
    List<String> dependencies,
    FileSystem fileSystem,
    bool isDirectDependency, {
    required bool isDevDependency,
  }) {
    assert(pluginYaml['platforms'] != null, 'Invalid multi-platform plugin specification $name.');
    final YamlMap platformsYaml = pluginYaml['platforms'] as YamlMap;

    assert(
      _validateMultiPlatformYaml(platformsYaml).isEmpty,
      'Invalid multi-platform plugin specification $name.',
    );

    final Map<String, PluginPlatform> platforms = <String, PluginPlatform>{};

    if (_providesImplementationForPlatform(platformsYaml, AndroidPlugin.kConfigKey)) {
      platforms[AndroidPlugin.kConfigKey] = AndroidPlugin.fromYaml(
        name,
        platformsYaml[AndroidPlugin.kConfigKey] as YamlMap,
        path,
        fileSystem,
      );
    }

    if (_providesImplementationForPlatform(platformsYaml, IOSPlugin.kConfigKey)) {
      platforms[IOSPlugin.kConfigKey] = IOSPlugin.fromYaml(
        name,
        platformsYaml[IOSPlugin.kConfigKey] as YamlMap,
      );
    }

    if (_providesImplementationForPlatform(platformsYaml, LinuxPlugin.kConfigKey)) {
      platforms[LinuxPlugin.kConfigKey] = LinuxPlugin.fromYaml(
        name,
        platformsYaml[LinuxPlugin.kConfigKey] as YamlMap,
      );
    }

    if (_providesImplementationForPlatform(platformsYaml, MacOSPlugin.kConfigKey)) {
      platforms[MacOSPlugin.kConfigKey] = MacOSPlugin.fromYaml(
        name,
        platformsYaml[MacOSPlugin.kConfigKey] as YamlMap,
      );
    }

    if (_providesImplementationForPlatform(platformsYaml, WebPlugin.kConfigKey)) {
      platforms[WebPlugin.kConfigKey] = WebPlugin.fromYaml(
        name,
        platformsYaml[WebPlugin.kConfigKey] as YamlMap,
      );
    }

    if (_providesImplementationForPlatform(platformsYaml, WindowsPlugin.kConfigKey)) {
      platforms[WindowsPlugin.kConfigKey] = WindowsPlugin.fromYaml(
        name,
        platformsYaml[WindowsPlugin.kConfigKey] as YamlMap,
      );
    }

    // TODO(stuartmorgan): Consider merging web into this common handling; the
    //  fact that its implementation of Dart-only plugins and default packages
    //  are separate is legacy.
    final List<String> sharedHandlingPlatforms = <String>[
      AndroidPlugin.kConfigKey,
      IOSPlugin.kConfigKey,
      LinuxPlugin.kConfigKey,
      MacOSPlugin.kConfigKey,
      WindowsPlugin.kConfigKey,
    ];
    final Map<String, String> defaultPackages = <String, String>{};
    final Map<String, DartPluginClassAndFilePair> dartPluginClasses =
        <String, DartPluginClassAndFilePair>{};
    for (final String platform in sharedHandlingPlatforms) {
      final String? defaultPackage = _getDefaultPackageForPlatform(platformsYaml, platform);
      if (defaultPackage != null) {
        defaultPackages[platform] = defaultPackage;
      }
      final DartPluginClassAndFilePair? dartPair = _getPluginDartClassForPlatform(
        platformsYaml,
        platformKey: platform,
        pluginName: name,
      );
      if (dartPair != null) {
        dartPluginClasses[platform] = dartPair;
      }
    }

    return Plugin(
      name: name,
      path: path,
      platforms: platforms,
      defaultPackagePlatforms: defaultPackages,
      pluginDartClassPlatforms: dartPluginClasses,
      flutterConstraint: flutterConstraint,
      dependencies: dependencies,
      isDirectDependency: isDirectDependency,
      implementsPackage: pluginYaml['implements'] != null ? pluginYaml['implements'] as String : '',
      isDevDependency: isDevDependency,
    );
  }

  factory Plugin._fromLegacyYaml(
    String name,
    String path,
    dynamic pluginYaml,
    VersionConstraint? flutterConstraint,
    List<String> dependencies,
    FileSystem fileSystem,
    bool isDirectDependency, {
    required bool isDevDependency,
  }) {
    final Map<String, PluginPlatform> platforms = <String, PluginPlatform>{};
    final String? pluginClass = (pluginYaml as Map<dynamic, dynamic>)['pluginClass'] as String?;
    if (pluginClass != null) {
      final String? androidPackage = pluginYaml['androidPackage'] as String?;
      if (androidPackage != null) {
        platforms[AndroidPlugin.kConfigKey] = AndroidPlugin(
          name: name,
          package: androidPackage,
          pluginClass: pluginClass,
          pluginPath: path,
          fileSystem: fileSystem,
        );
      }

      final String iosPrefix = pluginYaml['iosPrefix'] as String? ?? '';
      platforms[IOSPlugin.kConfigKey] = IOSPlugin(
        name: name,
        classPrefix: iosPrefix,
        pluginClass: pluginClass,
      );
    }
    return Plugin(
      name: name,
      path: path,
      platforms: platforms,
      defaultPackagePlatforms: <String, String>{},
      pluginDartClassPlatforms: <String, DartPluginClassAndFilePair>{},
      flutterConstraint: flutterConstraint,
      dependencies: dependencies,
      isDirectDependency: isDirectDependency,
      isDevDependency: isDevDependency,
    );
  }

  /// Create a YamlMap that represents the supported platforms.
  ///
  /// For example, if the `platforms` contains 'ios' and 'android', the return map looks like:
  ///
  ///     android:
  ///       package: io.flutter.plugins.sample
  ///       pluginClass: SamplePlugin
  ///     ios:
  ///       pluginClass: SamplePlugin
  static YamlMap createPlatformsYamlMap(
    List<String> platforms,
    String pluginClass,
    String androidPackage,
  ) {
    final Map<String, dynamic> map = <String, dynamic>{};
    for (final String platform in platforms) {
      map[platform] = <String, String>{
        'pluginClass': pluginClass,
        ...platform == 'android' ? <String, String>{'package': androidPackage} : <String, String>{},
      };
    }
    return YamlMap.wrap(map);
  }

  static List<String> validatePluginYaml(YamlMap? yaml) {
    if (yaml == null) {
      return <String>['Invalid "plugin" specification.'];
    }

    final bool usesOldPluginFormat = const <String>{
      'androidPackage',
      'iosPrefix',
      'pluginClass',
    }.any(yaml.containsKey);

    final bool usesNewPluginFormat = yaml.containsKey('platforms');

    if (usesOldPluginFormat && usesNewPluginFormat) {
      const String errorMessage =
          'The flutter.plugin.platforms key cannot be used in combination with the old '
          'flutter.plugin.{androidPackage,iosPrefix,pluginClass} keys. '
          'See: https://flutter.dev/to/pubspec-plugin-platforms';
      return <String>[errorMessage];
    }

    if (!usesOldPluginFormat && !usesNewPluginFormat) {
      const String errorMessage =
          'Cannot find the `flutter.plugin.platforms` key in the `pubspec.yaml` file. '
          'An instruction to format the `pubspec.yaml` can be found here: '
          'https://flutter.dev/to/pubspec-plugin-platforms';
      return <String>[errorMessage];
    }

    if (usesNewPluginFormat) {
      if (yaml['platforms'] != null && yaml['platforms'] is! YamlMap) {
        const String errorMessage =
            'flutter.plugin.platforms should be a map with the platform name as the key';
        return <String>[errorMessage];
      }
      return _validateMultiPlatformYaml(yaml['platforms'] as YamlMap?);
    } else {
      return _validateLegacyYaml(yaml);
    }
  }

  static List<String> _validateMultiPlatformYaml(YamlMap? yaml) {
    bool isInvalid(String key, bool Function(YamlMap) validate) {
      if (!yaml!.containsKey(key)) {
        return false;
      }
      final dynamic yamlValue = yaml[key];
      if (yamlValue is! YamlMap) {
        return true;
      }
      if (yamlValue.containsKey('default_package')) {
        return false;
      }
      return !validate(yamlValue);
    }

    if (yaml == null) {
      return <String>['Invalid "platforms" specification.'];
    }
    return <String>[
      if (isInvalid(AndroidPlugin.kConfigKey, AndroidPlugin.validate))
        'Invalid "android" plugin specification.',
      if (isInvalid(IOSPlugin.kConfigKey, IOSPlugin.validate))
        'Invalid "ios" plugin specification.',
      if (isInvalid(LinuxPlugin.kConfigKey, LinuxPlugin.validate))
        'Invalid "linux" plugin specification.',
      if (isInvalid(MacOSPlugin.kConfigKey, MacOSPlugin.validate))
        'Invalid "macos" plugin specification.',
      if (isInvalid(WindowsPlugin.kConfigKey, WindowsPlugin.validate))
        'Invalid "windows" plugin specification.',
    ];
  }

  static List<String> _validateLegacyYaml(YamlMap yaml) {
    return <String>[
      if (yaml['androidPackage'] is! String?)
        'The "androidPackage" must either be null or a string.',
      if (yaml['iosPrefix'] is! String?) 'The "iosPrefix" must either be null or a string.',
      if (yaml['pluginClass'] is! String?) 'The "pluginClass" must either be null or a string.',
    ];
  }

  static bool _supportsPlatform(YamlMap platformsYaml, String platformKey) {
    if (!platformsYaml.containsKey(platformKey)) {
      return false;
    }
    if (platformsYaml[platformKey] is YamlMap) {
      return true;
    }
    return false;
  }

  static String? _getDefaultPackageForPlatform(YamlMap platformsYaml, String platformKey) {
    if (!_supportsPlatform(platformsYaml, platformKey)) {
      return null;
    }
    if ((platformsYaml[platformKey] as YamlMap).containsKey(kDefaultPackage)) {
      return (platformsYaml[platformKey] as YamlMap)[kDefaultPackage] as String;
    }
    return null;
  }

  static DartPluginClassAndFilePair? _getPluginDartClassForPlatform(
    YamlMap platformsYaml, {
    required String platformKey,
    required String pluginName,
  }) {
    if (!_supportsPlatform(platformsYaml, platformKey)) {
      return null;
    }
    if ((platformsYaml[platformKey] as YamlMap).containsKey(kDartPluginClass)) {
      final String dartClass = (platformsYaml[platformKey] as YamlMap)[kDartPluginClass] as String;
      final String dartFileName =
          (platformsYaml[platformKey] as YamlMap)[kDartFileName] as String? ?? '$pluginName.dart';
      return (dartClass: dartClass, dartFileName: dartFileName);
    }
    return null;
  }

  static bool _providesImplementationForPlatform(YamlMap platformsYaml, String platformKey) {
    if (!_supportsPlatform(platformsYaml, platformKey)) {
      return false;
    }
    if ((platformsYaml[platformKey] as YamlMap).containsKey(kDefaultPackage)) {
      return false;
    }
    return true;
  }

  final String name;
  final String path;

  /// The name of the interface package that this plugin implements.
  /// If [null], this plugin doesn't implement an interface.
  final String? implementsPackage;

  /// The required version of Flutter, if specified.
  final VersionConstraint? flutterConstraint;

  /// The name of the packages this plugin depends on.
  final List<String> dependencies;

  /// This is a mapping from platform config key to the plugin platform spec.
  final Map<String, PluginPlatform> platforms;

  /// This is a mapping from platform config key to the default package implementation.
  final Map<String, String> defaultPackagePlatforms;

  /// This is a mapping from platform config key to the Dart plugin class for the given platform.
  final Map<String, DartPluginClassAndFilePair> pluginDartClassPlatforms;

  /// Whether this plugin is a direct dependency of the app.
  /// If [false], the plugin is a dependency of another plugin.
  final bool isDirectDependency;

  /// Whether this plugin is exclusively used as a dev dependency of the app.
  ///
  /// If [false], the plugin is either:
  /// - _Not_ a dev dependency
  /// - _Not_ a dev dependency of some dependency that itself is not a dev
  ///   dependency
  ///
  /// Dev dependencies are intended to be stripped out in release builds.
  final bool isDevDependency;

  /// Expected path to the plugin's Package.swift. Returns null if the plugin
  /// does not support the [platform] or the [platform] is not iOS or macOS.
  String? pluginSwiftPackageManifestPath(FileSystem fileSystem, String platform) {
    final String? platformDirectoryName = _darwinPluginDirectoryName(platform);
    if (platformDirectoryName == null) {
      return null;
    }
    return fileSystem.path.join(path, platformDirectoryName, name, 'Package.swift');
  }

  /// Expected path to the plugin's podspec. Returns null if the plugin does
  /// not support the [platform] or the [platform] is not iOS or macOS.
  String? pluginPodspecPath(FileSystem fileSystem, String platform) {
    final String? platformDirectoryName = _darwinPluginDirectoryName(platform);
    if (platformDirectoryName == null) {
      return null;
    }
    return fileSystem.path.join(path, platformDirectoryName, '$name.podspec');
  }

  String? _darwinPluginDirectoryName(String platform) {
    final PluginPlatform? platformPlugin = platforms[platform];
    if (platformPlugin == null ||
        (platform != IOSPlugin.kConfigKey && platform != MacOSPlugin.kConfigKey)) {
      return null;
    }

    // iOS and macOS code can be shared in "darwin" directory, otherwise
    // respectively in "ios" or "macos" directories.
    if (platformPlugin is DarwinPlugin && (platformPlugin as DarwinPlugin).sharedDarwinSource) {
      return 'darwin';
    }
    return platform;
  }
}

/// Metadata associated with the resolution of a platform interface of a plugin.
class PluginInterfaceResolution {
  PluginInterfaceResolution({required this.plugin, required this.platform});

  /// The plugin.
  final Plugin plugin;
  // The name of the platform that this plugin implements.
  final String platform;

  Map<String, String> toMap() {
    return <String, String>{
      'pluginName': plugin.name,
      'platform': platform,
      'dartClass': plugin.pluginDartClassPlatforms[platform]?.dartClass ?? '',
      'dartFileName': plugin.pluginDartClassPlatforms[platform]?.dartFileName ?? '',
    };
  }

  @override
  String toString() {
    return '<PluginInterfaceResolution ${plugin.name} for $platform>';
  }
}

/// A record representing pair of dartPluginClass and dartFileName used as metadata
/// in [PluginInterfaceResolution].
///
/// The [dartClass] and [dartFileName] fields are guaranteed to be non-null:
/// - record should be created only if dartClassName exists in plugin configuration.
/// - dartFileName either taken from configuration, or, if absent, should be
/// constructed from plugin name.
/// See also:
/// - [PluginInterfaceResolution], which uses this record to create Map with metadata.
typedef DartPluginClassAndFilePair = ({String dartClass, String dartFileName});
