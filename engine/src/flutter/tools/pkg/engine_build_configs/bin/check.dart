// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_build_configs/src/ci_yaml.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';
import 'package:source_span/source_span.dart';
import 'package:yaml/yaml.dart' as y;

// Usage:
// $ dart bin/check.dart
//
// Or, for more options:
// $ dart bin/check.dart --help

final _argParser =
    ArgParser()
      ..addFlag('verbose', abbr: 'v', help: 'Enable noisier diagnostic output', negatable: false)
      ..addFlag('help', abbr: 'h', help: 'Output usage information.', negatable: false)
      ..addOption(
        'engine-src-path',
        valueHelp: '/path/to/engine/src',
        defaultsTo: Engine.tryFindWithin()?.srcDir.path,
      );

void main(List<String> args) {
  run(
    args,
    stderr: io.stderr,
    stdout: io.stdout,
    platform: const LocalPlatform(),
    setExitCode: (exitCode) {
      io.exitCode = exitCode;
    },
  );
}

@visibleForTesting
void run(
  Iterable<String> args, {
  required Platform platform,
  required StringSink stderr,
  required StringSink stdout,
  required void Function(int) setExitCode,
}) {
  y.yamlWarningCallback = (String message, [SourceSpan? span]) {};

  final argResults = _argParser.parse(args);
  if (argResults.flag('help')) {
    stdout.writeln(_argParser.usage);
    return;
  }

  final verbose = argResults.flag('verbose');
  void debugPrint(String output) {
    if (!verbose) {
      return;
    }
    stderr.writeln(output);
  }

  void indentedPrint(Iterable<String> errors) {
    for (final error in errors) {
      stderr.writeln('  $error');
    }
  }

  final supportsEmojis = !platform.isWindows || platform.environment.containsKey('WT_SESSION');
  final symbolSuccess = supportsEmojis ? '✅' : '✓';
  final symbolFailure = supportsEmojis ? '❌' : 'X';
  void statusPrint(String describe, {required bool success}) {
    stderr.writeln('${success ? symbolSuccess : symbolFailure} $describe');
    if (!success) {
      setExitCode(1);
    }
  }

  final engine = Engine.fromSrcPath(argResults.option('engine-src-path')!);
  debugPrint('Initializing from ${p.relative(engine.srcDir.path)}');

  // Find and parse the engine build configs.
  final io.Directory buildConfigsDir = io.Directory(
    p.join(engine.flutterDir.path, 'ci', 'builders'),
  );
  final BuildConfigLoader loader = BuildConfigLoader(buildConfigsDir: buildConfigsDir);

  // Treat it as an error if no build configs were found. The caller likely
  // expected to find some.
  final Map<String, BuilderConfig> configs = loader.configs;

  // We can't make further progress if we didn't find any configurations.
  statusPrint(
    'Loaded build configs under ${p.relative(buildConfigsDir.path)}',
    success: configs.isNotEmpty && loader.errors.isEmpty,
  );
  if (configs.isEmpty) {
    return;
  }
  indentedPrint(loader.errors);

  // Find and parse the .ci.yaml configuration (for the engine).
  final CiConfig? ciConfig;
  {
    final String ciYamlPath = p.join(engine.flutterDir.path, '.ci.yaml');
    final String realCiYaml = io.File(ciYamlPath).readAsStringSync();
    final y.YamlNode yamlNode = y.loadYamlNode(realCiYaml, sourceUrl: Uri.file(ciYamlPath));
    final loadedConfig = CiConfig.fromYaml(yamlNode);

    statusPrint('.ci.yaml at ${p.relative(ciYamlPath)} is valid', success: loadedConfig.valid);
    if (!loadedConfig.valid) {
      indentedPrint([loadedConfig.error!]);
      ciConfig = null;
    } else {
      ciConfig = loadedConfig;
    }
  }

  // Check the parsed build configs for validity.
  final List<String> invalidErrors = checkForInvalidConfigs(configs);
  statusPrint('All configuration files are valid', success: invalidErrors.isEmpty);
  indentedPrint(invalidErrors);

  // We require all builds within a builder config to be uniquely named.
  final List<String> duplicateErrors = checkForDuplicateConfigs(configs);
  statusPrint('All builds within a builder are uniquely named', success: duplicateErrors.isEmpty);
  indentedPrint(duplicateErrors);

  // We require all builds to be named in a way that is understood by et.
  final List<String> buildNameErrors = checkForInvalidBuildNames(configs);
  statusPrint('All build names must have a conforming prefix', success: buildNameErrors.isEmpty);
  indentedPrint(buildNameErrors);

  // If we have a successfully parsed .ci.yaml, perform additional checks.
  if (ciConfig == null) {
    return;
  }

  // We require that targets that have `properties: release_build: "true"`:
  // (1) Each sub-build produces artifacts (`archives: [...]`)
  // (2) Each sub-build does not have `tests: [ ... ]`
  final buildConventionErrors = <String>[];
  for (final MapEntry(key: _, value: target) in ciConfig.ciTargets.entries) {
    final config = loader.configs[target.properties.configName];
    if (target.properties.configName == null) {
      // * builder_cache targets do not have configuration files.
      debugPrint('  Skipping ${target.name}: No configuration file found');
      continue;
    }

    // This would fail above during the general loading.
    if (config == null) {
      throw StateError('Unreachable');
    }

    final configConventionErrors = <String>[];
    if (target.properties.isReleaseBuilder) {
      // If there is a global generators step, assume artifacts are uploaded from the generators.
      if (config.generators.isNotEmpty) {
        debugPrint('  Skipping ${target.name}: Has "generators": [ ... ] which could do anything');
        continue;
      }
      // Check each build: it must have "archives: [ ... ]" and NOT "tests: [ ... ]"
      for (final build in config.builds) {
        if (build.archives.isEmpty) {
          configConventionErrors.add('${build.name}: Does not have "archives: [ ... ]"');
        }
        if (build.archives.any((e) => e.includePaths.isEmpty)) {
          configConventionErrors.add(
            '${build.name}: Has an archive with an empty "include_paths": []',
          );
        }
        if (build.tests.isNotEmpty) {
          configConventionErrors.add('${build.name}: Includes "tests: [ ... ]"');
        }
      }
    }

    if (configConventionErrors.isNotEmpty) {
      buildConventionErrors.add(
        '${p.basename(config.path)} (${target.name}, release_build = ${target.properties.isReleaseBuilder}):',
      );
      buildConventionErrors.addAll(configConventionErrors.map((e) => '  $e'));
    }
  }
  statusPrint(
    'All builder files conform to release_build standards',
    success: buildConventionErrors.isEmpty,
  );
  indentedPrint(buildConventionErrors);
}

// This check ensures that all the json files were deserialized without errors.
List<String> checkForInvalidConfigs(Map<String, BuilderConfig> configs) {
  final List<String> errors = <String>[];
  for (final String name in configs.keys) {
    final BuilderConfig buildConfig = configs[name]!;
    final List<String> buildConfigErrors = buildConfig.check(name);
    if (buildConfigErrors.isNotEmpty) {
      errors.add('Errors in ${buildConfig.path}:');
    }
    for (final String error in buildConfigErrors) {
      errors.add('    $error');
    }
  }
  return errors;
}

// Thjs check ensures that json files do not contain builds with duplicate
// names.
List<String> checkForDuplicateConfigs(Map<String, BuilderConfig> configs) {
  final List<String> errors = <String>[];
  final Map<String, Set<String>> builderBuildSet = <String, Set<String>>{};
  _forEachBuild(configs, (String name, BuilderConfig config, Build build) {
    final Set<String> builds = builderBuildSet.putIfAbsent(name, () => <String>{});
    if (builds.contains(build.name)) {
      errors.add('${build.name} is duplicated in $name\n');
    } else {
      builds.add(build.name);
    }
  });
  return errors;
}

// This check ensures that builds are named in a way that is understood by
// `et`.
List<String> checkForInvalidBuildNames(Map<String, BuilderConfig> configs) {
  final List<String> errors = <String>[];

  // In local_engine.json, allowed OS names are linux, macos, and windows.
  final List<String> osNames =
      <String>[
        Platform.linux,
        Platform.macOS,
        Platform.windows,
      ].expand((String s) => <String>['$s/', '$s\\']).toList();

  // In all other build json files, allowed prefix names are ci and web_tests.
  final List<String> ciNames =
      <String>['ci', 'web_tests'].expand((String s) => <String>['$s/', '$s\\']).toList();

  _forEachBuild(configs, (String name, BuilderConfig config, Build build) {
    final List<String> goodPrefixes = name.contains('local_engine') ? osNames : ciNames;
    if (!goodPrefixes.any(build.name.startsWith)) {
      errors.add(
        '${build.name} in $name must start with one of '
        '{${goodPrefixes.join(', ')}}',
      );
    }
  });
  return errors;
}

void _forEachBuild(
  Map<String, BuilderConfig> configs,
  void Function(String configName, BuilderConfig config, Build build) fn,
) {
  for (final String builderName in configs.keys) {
    final BuilderConfig builderConfig = configs[builderName]!;
    for (final Build build in builderConfig.builds) {
      fn(builderName, builderConfig, build);
    }
  }
}
