// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:yaml/yaml.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/utils.dart';
import '../build_system/build_system.dart';
import '../build_system/build_targets.dart';
import '../features.dart';

/// Generates the `package:flutter_gen` synthetic package.
///
/// If the package has been configured *not* to use synthetic packages, this
/// method is a NO-OP and returns `false`.
///
/// Returns `true` if the package was generated, or `false` it was not.
Future<bool> generateLocalizationsSyntheticPackage({
  required Environment environment,
  required BuildSystem buildSystem,
  required BuildTargets buildTargets,
}) async {
  final FileSystem fileSystem = environment.fileSystem;
  final File l10nYamlFile = fileSystem.file(
    fileSystem.path.join(environment.projectDir.path, 'l10n.yaml'),
  );

  // If pubspec.yaml has generate:true and if l10n.yaml exists in the
  // root project directory, check to see if a synthetic package should
  // be generated for gen_l10n.
  if (!l10nYamlFile.existsSync()) {
    return false;
  }

  final YamlNode yamlNode = loadYamlNode(l10nYamlFile.readAsStringSync());
  if (yamlNode.value != null && yamlNode is! YamlMap) {
    throwToolExit('Expected ${l10nYamlFile.path} to contain a map, instead was $yamlNode');
  }

  // If an l10n.yaml file exists and is not empty, attempt to parse settings in
  // it.
  if (yamlNode.value != null) {
    final YamlMap yamlMap = yamlNode as YamlMap;
    final Object? value = yamlMap['synthetic-package'];
    if (value is! bool && value != null) {
      throwToolExit('Expected "synthetic-package" to have a bool value, instead was "$value"');
    }

    // Generate gen_l10n synthetic package only if synthetic-package: true or
    // synthetic-package is null.
    final bool? isSyntheticL10nPackage = value as bool?;
    if (isSyntheticL10nPackage == false ||
        isSyntheticL10nPackage == null && featureFlags.isExplicitPackageDependenciesEnabled) {
      return false;
    }
  } else if (featureFlags.isExplicitPackageDependenciesEnabled) {
    // synthetic-packages: true was not set and it is no longer the default.
    return false;
  }

  if (featureFlags.isExplicitPackageDependenciesEnabled) {
    throwToolExit(
      'Cannot generate a synthetic package when explicit-package-dependencies is enabled.\n'
      '\n'
      'Synthetic package output (package:flutter_gen) is deprecated: '
      'https://flutter.dev/to/flutter-gen-deprecation. If you are seeing this '
      'message either you have provided explicit-package-dependencies, or it '
      'is the default value (see flutter config --help).',
    );
  }

  // Log a warning: synthetic-package: true (or implicit true) is deprecated.
  environment.logger.printWarning(
    'Synthetic package output (package:flutter_gen) is deprecated: '
    'https://flutter.dev/to/flutter-gen-deprecation. In a future release, '
    'synthetic-package will default to `false` and will later be removed '
    'entirely.',
  );

  final BuildResult result = await buildSystem.build(
    buildTargets.generateLocalizationsTarget,
    environment,
  );

  if (result.hasException) {
    throwToolExit(
      'Generating synthetic localizations package failed with ${result.exceptions.length} ${pluralize('error', result.exceptions.length)}:'
      '\n\n'
      '${result.exceptions.values.map<Object?>((ExceptionMeasurement e) => e.exception).join('\n\n')}',
    );
  }

  return true;
}
