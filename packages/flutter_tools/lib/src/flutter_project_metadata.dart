// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:yaml/yaml.dart';

import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'features.dart';
import 'project.dart';
import 'template.dart';
import 'version.dart';

/// The result of parsing `--template=` for `flutter create` and related commands.
@immutable
sealed class ParsedFlutterTemplateType implements CliEnum {
  static const List<ParsedFlutterTemplateType> _values = <ParsedFlutterTemplateType>[
    ...FlutterTemplateType.values,
    ...RemovedFlutterTemplateType.values,
  ];

  /// Parses and returns a [ParsedFlutterTemplateType], if any, for [cliName].
  ///
  /// If no match was found `null` is returned.
  static ParsedFlutterTemplateType? fromCliName(String cliName) {
    for (final ParsedFlutterTemplateType type in _values) {
      if (cliName == type.cliName) {
        return type;
      }
    }
    return null;
  }

  /// Returns template types that are enabled based on the current [featureFlags].
  static List<ParsedFlutterTemplateType> enabledValues(FeatureFlags featureFlags) {
    return _values.toList()..retainWhere((ParsedFlutterTemplateType templateType) {
      return templateType.isEnabled(featureFlags);
    });
  }

  /// Whether the flag is enabled based on a flag being set.
  bool isEnabled(FeatureFlags featureFlags) => true;
}

/// A [ParsedFlutterTemplateType] that is no longer operable.
///
/// The CLI can give a hint to a developer that the [cliName] _did_ use to exist,
/// but does no longer, and provides [helpText] for other resources to use instead.
enum RemovedFlutterTemplateType implements ParsedFlutterTemplateType {
  skeleton(
    helpText:
        'Formerly generated a list view / detail view Flutter application that '
        'followed some community best practices. For up to date resources, see '
        'https://flutter.github.io/samples, https://docs.flutter.dev/codelabs, '
        'and community resources such as https://flutter-builder.app/.',
  );

  const RemovedFlutterTemplateType({required this.helpText});

  @override
  bool isEnabled(FeatureFlags featureFlags) => true;

  @override
  final String helpText;

  @override
  String get cliName => snakeCase(name);
}

/// The result of parsing a recognized `--template` for `flutter create` and related commands.
enum FlutterTemplateType implements ParsedFlutterTemplateType {
  /// The default project with the user-managed host code.
  ///
  /// It is different than the "module" template in that it exposes and doesn't
  /// manage the platform code.
  app(helpText: '(default) Generate a Flutter application.'),

  /// A project that has managed platform host code.
  ///
  /// It is an application with ephemeral .ios and .android directories that can be updated automatically.
  module(
    helpText:
        'Generate a project to add a Flutter module to an existing Android or iOS application.',
  ),

  /// A Flutter Dart package project.
  ///
  /// It doesn't have any native components, only Dart.
  package(helpText: 'Generate a shareable Flutter project containing modular Dart code.'),

  /// A Dart package project with external builds for native components.
  packageFfi(
    helpText:
        'Generate a shareable Dart/Flutter project containing an API '
        'in Dart code with a platform-specific implementation through dart:ffi for Android, iOS, '
        'Linux, macOS, and Windows.',
  ),

  /// A native plugin project.
  plugin(
    helpText:
        'Generate a shareable Flutter project containing an API '
        'in Dart code with a platform-specific implementation through method channels for Android, iOS, '
        'Linux, macOS, Windows, web, or any combination of these.',
  ),

  /// This is an FFI native plugin project.
  pluginFfi(
    helpText:
        'Generate a shareable Flutter project containing an API '
        'in Dart code with a platform-specific implementation through dart:ffi for Android, iOS, '
        'Linux, macOS, Windows, or any combination of these.',
  );

  const FlutterTemplateType({required this.helpText});

  @override
  bool isEnabled(FeatureFlags featureFlags) {
    return switch (this) {
      FlutterTemplateType.packageFfi => featureFlags.isNativeAssetsEnabled,
      _ => true,
    };
  }

  @override
  final String helpText;

  @override
  String get cliName => snakeCase(name);
}

/// Verifies the expected yaml keys are present in the file.
bool _validateMetadataMap(YamlMap map, Map<String, Type> validations, Logger logger) {
  bool isValid = true;
  for (final MapEntry<String, Object> entry in validations.entries) {
    if (!map.keys.contains(entry.key)) {
      isValid = false;
      logger.printTrace('The key `${entry.key}` was not found');
      break;
    }
    final Object? metadataValue = map[entry.key];
    if (metadataValue.runtimeType != entry.value) {
      isValid = false;
      logger.printTrace(
        'The value of key `${entry.key}` in .metadata was expected to be ${entry.value} but was ${metadataValue.runtimeType}',
      );
      break;
    }
  }
  return isValid;
}

/// A wrapper around the `.metadata` file.
class FlutterProjectMetadata {
  /// Creates a MigrateConfig by parsing an existing .migrate_config yaml file.
  FlutterProjectMetadata(this.file, Logger logger)
    : _logger = logger,
      migrateConfig = MigrateConfig() {
    if (!file.existsSync()) {
      _logger.printTrace('No .metadata file found at ${file.path}.');
      // Create a default empty metadata.
      return;
    }
    Object? yamlRoot;
    try {
      yamlRoot = loadYaml(file.readAsStringSync());
    } on YamlException {
      // Handled in _validate below.
    }
    if (yamlRoot is! YamlMap) {
      _logger.printTrace('.metadata file at ${file.path} was empty or malformed.');
      return;
    }
    if (_validateMetadataMap(yamlRoot, <String, Type>{'version': YamlMap}, _logger)) {
      final Object? versionYamlMap = yamlRoot['version'];
      if (versionYamlMap is YamlMap &&
          _validateMetadataMap(versionYamlMap, <String, Type>{
            'revision': String,
            'channel': String,
          }, _logger)) {
        _versionRevision = versionYamlMap['revision'] as String?;
        _versionChannel = versionYamlMap['channel'] as String?;
      }
    }
    if (_validateMetadataMap(yamlRoot, <String, Type>{'project_type': String}, _logger)) {
      final ParsedFlutterTemplateType? templateType = ParsedFlutterTemplateType.fromCliName(
        yamlRoot['project_type'] as String,
      );
      _projectType = switch (templateType) {
        RemovedFlutterTemplateType() || null => null,
        FlutterTemplateType() => templateType,
      };
    }
    final Object? migrationYaml = yamlRoot['migration'];
    if (migrationYaml is YamlMap) {
      migrateConfig.parseYaml(migrationYaml, _logger);
    }
  }

  /// Creates a FlutterProjectMetadata by explicitly providing all values.
  FlutterProjectMetadata.explicit({
    required this.file,
    required String? versionRevision,
    required String? versionChannel,
    required FlutterTemplateType? projectType,
    required this.migrateConfig,
    required Logger logger,
  }) : _logger = logger,
       _versionChannel = versionChannel,
       _versionRevision = versionRevision,
       _projectType = projectType;

  /// The name of the config file.
  static const String kFileName = '.metadata';

  String? _versionRevision;
  String? get versionRevision => _versionRevision;

  String? _versionChannel;
  String? get versionChannel => _versionChannel;

  FlutterTemplateType? _projectType;
  FlutterTemplateType? get projectType => _projectType;

  /// Metadata and configuration for the migrate command.
  MigrateConfig migrateConfig;

  final Logger _logger;

  final File file;

  /// Writes the .migrate_config file in the provided project directory's platform subdirectory.
  ///
  /// We write the file manually instead of with a template because this
  /// needs to be able to write the .migrate_config file into legacy apps.
  void writeFile({File? outputFile}) {
    outputFile = outputFile ?? file;
    outputFile
      ..createSync(recursive: true)
      ..writeAsStringSync(toString(), flush: true);
  }

  @override
  String toString() {
    return '''
# This file tracks properties of this Flutter project.
# Used by Flutter tool to assess capabilities and perform upgrades etc.
#
# This file should be version controlled and should not be manually edited.

version:
  revision: ${escapeYamlString(_versionRevision ?? '')}
  channel: ${escapeYamlString(_versionChannel ?? kUserBranch)}

project_type: ${projectType == null ? '' : projectType!.cliName}
${migrateConfig.getOutputFileString()}''';
  }

  void populate({
    List<SupportedPlatform>? platforms,
    required Directory projectDirectory,
    String? currentRevision,
    String? createRevision,
    bool create = true,
    bool update = true,
    required Logger logger,
  }) {
    migrateConfig.populate(
      platforms: platforms,
      projectDirectory: projectDirectory,
      currentRevision: currentRevision,
      createRevision: createRevision,
      create: create,
      update: update,
      logger: logger,
    );
  }

  /// Finds the fallback revision to use when no base revision is found in the migrate config.
  String getFallbackBaseRevision(Logger logger, FlutterVersion flutterVersion) {
    // Use the .metadata file if it exists.
    return versionRevision ?? flutterVersion.frameworkRevision;
  }
}

/// Represents the migrate command metadata section of a .metadata file.
///
/// This file tracks the flutter sdk git hashes of the last successful migration ('base') and
/// the version the project was created with.
///
/// Each platform tracks a different set of revisions because flutter create can be
/// used to add support for new platforms, so the base and create revision may not always be the same.
class MigrateConfig {
  MigrateConfig({
    Map<SupportedPlatform, MigratePlatformConfig>? platformConfigs,
    this.unmanagedFiles = kDefaultUnmanagedFiles,
  }) : platformConfigs = platformConfigs ?? <SupportedPlatform, MigratePlatformConfig>{};

  /// A mapping of the files that are unmanaged by default for each platform.
  static const List<String> kDefaultUnmanagedFiles = <String>[
    'lib/main.dart',
    'ios/Runner.xcodeproj/project.pbxproj',
  ];

  /// The metadata for each platform supported by the project.
  final Map<SupportedPlatform, MigratePlatformConfig> platformConfigs;

  /// A list of paths relative to this file the migrate tool should ignore.
  ///
  /// These files are typically user-owned files that should not be changed.
  List<String> unmanagedFiles;

  bool get isEmpty =>
      platformConfigs.isEmpty &&
      (unmanagedFiles.isEmpty || unmanagedFiles == kDefaultUnmanagedFiles);

  /// Parses the project for all supported platforms and populates the [MigrateConfig]
  /// to reflect the project.
  void populate({
    List<SupportedPlatform>? platforms,
    required Directory projectDirectory,
    String? currentRevision,
    String? createRevision,
    bool create = true,
    bool update = true,
    required Logger logger,
  }) {
    final FlutterProject flutterProject = FlutterProject.fromDirectory(projectDirectory);
    platforms ??= flutterProject.getSupportedPlatforms(includeRoot: true);

    for (final SupportedPlatform platform in platforms) {
      if (platformConfigs.containsKey(platform)) {
        if (update) {
          platformConfigs[platform]!.baseRevision = currentRevision;
        }
      } else {
        if (create) {
          platformConfigs[platform] = MigratePlatformConfig(
            platform: platform,
            createRevision: createRevision,
            baseRevision: currentRevision,
          );
        }
      }
    }
  }

  /// Returns the string that should be written to the .metadata file.
  String getOutputFileString() {
    String unmanagedFilesString = '';
    for (final String path in unmanagedFiles) {
      unmanagedFilesString += "\n    - '$path'";
    }

    String platformsString = '';
    for (final MapEntry<SupportedPlatform, MigratePlatformConfig> entry
        in platformConfigs.entries) {
      platformsString +=
          '\n    - platform: ${entry.key.toString().split('.').last}\n      create_revision: ${entry.value.createRevision == null ? 'null' : "${entry.value.createRevision}"}\n      base_revision: ${entry.value.baseRevision == null ? 'null' : "${entry.value.baseRevision}"}';
    }

    return isEmpty
        ? ''
        : '''

# Tracks metadata for the flutter migrate command
migration:
  platforms:$platformsString

  # User provided section

  # List of Local paths (relative to this file) that should be
  # ignored by the migrate tool.
  #
  # Files that are not part of the templates will be ignored by default.
  unmanaged_files:$unmanagedFilesString
''';
  }

  /// Parses and validates the `migration` section of the .metadata file.
  void parseYaml(YamlMap map, Logger logger) {
    final Object? platformsYaml = map['platforms'];
    if (_validateMetadataMap(map, <String, Type>{'platforms': YamlList}, logger)) {
      if (platformsYaml is YamlList && platformsYaml.isNotEmpty) {
        for (final YamlMap platformYamlMap in platformsYaml.whereType<YamlMap>()) {
          if (_validateMetadataMap(platformYamlMap, <String, Type>{
            'platform': String,
            'create_revision': String,
            'base_revision': String,
          }, logger)) {
            final SupportedPlatform platformValue = SupportedPlatform.values.firstWhere(
              (SupportedPlatform val) =>
                  val.toString() == 'SupportedPlatform.${platformYamlMap['platform'] as String}',
            );
            platformConfigs[platformValue] = MigratePlatformConfig(
              platform: platformValue,
              createRevision: platformYamlMap['create_revision'] as String?,
              baseRevision: platformYamlMap['base_revision'] as String?,
            );
          } else {
            // malformed platform entry
            continue;
          }
        }
      }
    }
    if (_validateMetadataMap(map, <String, Type>{'unmanaged_files': YamlList}, logger)) {
      final Object? unmanagedFilesYaml = map['unmanaged_files'];
      if (unmanagedFilesYaml is YamlList && unmanagedFilesYaml.isNotEmpty) {
        unmanagedFiles = List<String>.from(unmanagedFilesYaml.value.cast<String>());
      }
    }
  }
}

/// Holds the revisions for a single platform for use by the flutter migrate command.
class MigratePlatformConfig {
  MigratePlatformConfig({required this.platform, this.createRevision, this.baseRevision});

  /// The platform this config describes.
  SupportedPlatform platform;

  /// The Flutter SDK revision this platform was created by.
  ///
  /// Null if the initial create git revision is unknown.
  final String? createRevision;

  /// The Flutter SDK revision this platform was last migrated by.
  ///
  /// Null if the project was never migrated or the revision is unknown.
  String? baseRevision;

  bool equals(MigratePlatformConfig other) {
    return platform == other.platform &&
        createRevision == other.createRevision &&
        baseRevision == other.baseRevision;
  }
}
