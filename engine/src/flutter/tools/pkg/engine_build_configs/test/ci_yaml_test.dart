// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:engine_build_configs/src/ci_yaml.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:source_span/source_span.dart';
import 'package:test/test.dart';
import 'package:yaml/yaml.dart' as y;

void main() {
  y.yamlWarningCallback = (String message, [SourceSpan? span]) {};

  // Find the engine repo.
  final Engine engine;
  try {
    engine = Engine.findWithin();
  } catch (e) {
    io.stderr.writeln(e);
    io.exitCode = 1;
    return;
  }

  final String ciYamlPath = path.join(engine.flutterDir.path, '.ci.yaml');
  final String realCiYaml = io.File(ciYamlPath).readAsStringSync();

  test('Can load the real .ci.yaml file', () {
    final y.YamlNode yamlNode = y.loadYamlNode(realCiYaml, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    if (!config.valid) {
      io.stderr.writeln(config.error);
    }
    expect(config.valid, isTrue);
  });

  test('Parses all supported fields', () {
    const String yamlData = '''
targets:
  - name: Linux linux_build
    recipe: engine_v2/engine_v2
    properties:
      config_name: linux_build
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    if (!config.valid) {
      io.stderr.writeln(config.error);
    }
    expect(config.valid, isTrue);
    expect(config.ciTargets.entries.isNotEmpty, isTrue);
    expect(config.ciTargets['Linux linux_build'], isNotNull);
    expect(config.ciTargets['Linux linux_build']!.valid, isTrue);
    expect(config.ciTargets['Linux linux_build']!.name, equals('Linux linux_build'));
    expect(config.ciTargets['Linux linux_build']!.recipe, equals('engine_v2/engine_v2'));
    expect(config.ciTargets['Linux linux_build']!.properties.valid, isTrue);
    expect(config.ciTargets['Linux linux_build']!.properties.configName, equals('linux_build'));
  });

  test('Invalid when targets is malformed', () {
    const String yamlData = '''
targets: 4
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isFalse);
    expect(config.error, contains('Expected "targets" to be a list.'));
  });

  test('Invalid when a target is malformed', () {
    const String yamlData = '''
targets:
  - name: 4
    recipe: engine_v2/engine_v2
    properties:
      config_name: linux_build
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isFalse);
    expect(config.error, contains('Expected map to contain a string value for key "name".'));
  });

  test('Invalid when a recipe is malformed', () {
    const String yamlData = '''
targets:
  - name: Linux linux_build
    recipe: 4
    properties:
      config_name: linux_build
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isFalse);
    expect(config.error, contains('Expected map to contain a string value for key "recipe".'));
  });

  test('Invalid when a properties list is malformed', () {
    const String yamlData = '''
targets:
  - name: Linux linux_build
    recipe: engine_v2/engine_v2
    properties: 4
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isFalse);
    expect(config.error, contains('Expected "properties" to be a map.'));
  });

  test('Still valid when a config_name is not present', () {
    const String yamlData = '''
targets:
  - name: Linux linux_build
    recipe: engine_v2/engine_v2
    properties:
      field: value
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isTrue);
  });

  test('Invalid when any target is malformed', () {
    const String yamlData = '''
targets:
  - name: Linux linux_build
    recipe: engine_v2/engine_v2
    properties:
      config_name: linux_build

  - name: 4
    recipe: engine_v2/engine_v2
    properties:
      config_name: linux_build
''';
    final y.YamlNode yamlNode = y.loadYamlNode(yamlData, sourceUrl: Uri.file(ciYamlPath));
    final CiConfig config = CiConfig.fromYaml(yamlNode);
    expect(config.valid, isFalse);
    expect(config.error, contains('Expected map to contain a string value for key "name".'));
  });
}
