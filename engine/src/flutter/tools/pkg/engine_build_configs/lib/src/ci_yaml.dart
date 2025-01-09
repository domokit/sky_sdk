// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:yaml/yaml.dart' as y;

// This file contains classes for parsing information about CI configuration
// from the .ci.yaml file at the root of the flutter/engine repository.
// The meanings of the sections and fields are documented at:
//
// https://github.com/flutter/cocoon/blob/main/CI_YAML.md
//
// The classes here don't parse every possible field, but rather only those that
// are useful for working locally in the engine repo.

const String _targetsField = 'targets';
const String _nameField = 'name';
const String _recipeField = 'recipe';
const String _propertiesField = 'properties';
const String _configNameField = 'config_name';

/// A class containing the information deserialized from the .ci.yaml file.
///
/// The file contains three sections. "enabled_branches", "platform_properties",
/// and "targets". The "enabled_branches" section is not meaningful when working
/// locally. The configurations listed in the "targets" section inherit
/// properties listed in the "platform_properties" section depending on their
/// names. The configurations listed in the "targets" section are the names,
/// recipes, build configs, etc. of the builders in CI.
class CiConfig {
  /// Builds a [CiConfig] instance from parsed yaml data.
  ///
  /// If the yaml was malformed, then `CiConfig.valid` will be false, and
  /// `CiConfig.error` will be populated with an informative error message.
  /// Otherwise, `CiConfig.ciTargets` will contain a mapping from target name
  /// to [CiTarget] instance.
  factory CiConfig.fromYaml(y.YamlNode yaml) {
    if (yaml is! y.YamlMap) {
      final String error = yaml.span.message('Expected a map');
      return CiConfig._error(error);
    }
    final y.YamlMap ymap = yaml;
    final y.YamlNode? targetsNode = ymap.nodes[_targetsField];
    if (targetsNode == null) {
      final String error = ymap.span.message('Expected a "$_targetsField" key');
      return CiConfig._error(error);
    }
    if (targetsNode is! y.YamlList) {
      final String error = targetsNode.span.message('Expected "$_targetsField" to be a list.');
      return CiConfig._error(error);
    }
    final y.YamlList targetsList = targetsNode;

    final Map<String, CiTarget> result = <String, CiTarget>{};
    for (final y.YamlNode yamlTarget in targetsList.nodes) {
      final CiTarget target = CiTarget.fromYaml(yamlTarget);
      if (!target.valid) {
        return CiConfig._error(target.error);
      }
      result[target.name] = target;
    }

    return CiConfig._(ciTargets: result);
  }

  CiConfig._({required this.ciTargets}) : error = null;

  CiConfig._error(this.error) : ciTargets = <String, CiTarget>{};

  /// Information about CI builder configurations, which .ci.yaml calls
  /// "targets".
  final Map<String, CiTarget> ciTargets;

  /// An error message when this instance is invalid.
  final String? error;

  /// Whether this is a valid instance.
  late final bool valid = error == null;
}

/// Information about the configuration of a builder on CI, which .ci.yaml
/// calls a "target".
class CiTarget {
  /// Builds a [CiTarget] from parsed yaml data.
  ///
  /// If the yaml was malformed then `CiTarget.valid` is false and
  /// `CiTarget.error` contains a useful error message. Otherwise, the other
  /// fields contain information about the target.
  factory CiTarget.fromYaml(y.YamlNode yaml) {
    if (yaml is! y.YamlMap) {
      final String error = yaml.span.message('Expected a map.');
      return CiTarget._error(error);
    }
    final y.YamlMap targetMap = yaml;
    final String? name = _stringOfNode(targetMap.nodes[_nameField]);
    if (name == null) {
      final String error = targetMap.span.message(
        'Expected map to contain a string value for key "$_nameField".',
      );
      return CiTarget._error(error);
    }

    final String? recipe = _stringOfNode(targetMap.nodes[_recipeField]);
    if (recipe == null) {
      final String error = targetMap.span.message(
        'Expected map to contain a string value for key "$_recipeField".',
      );
      return CiTarget._error(error);
    }

    final y.YamlNode? propertiesNode = targetMap.nodes[_propertiesField];
    if (propertiesNode == null) {
      final String error = targetMap.span.message(
        'Expected map to contain a string value for key "$_propertiesField".',
      );
      return CiTarget._error(error);
    }
    final CiTargetProperties properties = CiTargetProperties.fromYaml(propertiesNode);
    if (!properties.valid) {
      return CiTarget._error(properties.error);
    }

    return CiTarget._(name: name, recipe: recipe, properties: properties);
  }

  CiTarget._({required this.name, required this.recipe, required this.properties}) : error = null;

  CiTarget._error(this.error)
    : name = '',
      recipe = '',
      properties = CiTargetProperties._error('Invalid');

  /// The name of the builder in CI.
  final String name;

  /// The CI recipe used to run the build.
  final String recipe;

  /// The properties of the build or builder.
  final CiTargetProperties properties;

  /// An error message when this instance is invalid.
  final String? error;

  /// Whether this is a valid instance.
  late final bool valid = error == null;
}

/// Various properties of a [CiTarget].
class CiTargetProperties {
  /// Builds a [CiTargetProperties] instance from parsed yaml data.
  ///
  /// If the yaml was malformed then `CiTargetProperties.valid` is false and
  /// `CiTargetProperties.error` contains a useful error message. Otherwise, the
  /// other fields contain information about the target properties.
  factory CiTargetProperties.fromYaml(y.YamlNode yaml) {
    if (yaml is! y.YamlMap) {
      final String error = yaml.span.message('Expected "$_propertiesField" to be a map.');
      return CiTargetProperties._error(error);
    }
    final y.YamlMap propertiesMap = yaml;
    final String? configName = _stringOfNode(propertiesMap.nodes[_configNameField]);
    return CiTargetProperties._(configName: configName ?? '');
  }

  CiTargetProperties._({required this.configName}) : error = null;

  CiTargetProperties._error(this.error) : configName = '';

  /// The name of the build configuration. If the containing [CiTarget] instance
  /// is using the engine_v2 recipes, then this name is the same as the name
  /// of the build config json file under ci/builders.
  final String configName;

  /// An error message when this instance is invalid.
  final String? error;

  /// Whether this is a valid instance.
  late final bool valid = error == null;
}

String? _stringOfNode(y.YamlNode? stringNode) {
  if (stringNode == null) {
    return null;
  }
  if (stringNode is! y.YamlScalar) {
    return null;
  }
  final y.YamlScalar stringScalar = stringNode;
  if (stringScalar.value is! String) {
    return null;
  }
  return stringScalar.value as String;
}
