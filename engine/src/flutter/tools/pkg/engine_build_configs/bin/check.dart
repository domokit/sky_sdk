// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show Directory, exitCode, stderr;

import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';

// Usage:
// $ dart bin/check.dart [/path/to/engine/src]

void main(List<String> args) {
  final String? engineSrcPath;
  if (args.isNotEmpty) {
    engineSrcPath = args[0];
  } else {
    engineSrcPath = null;
  }

  // Find the engine repo.
  final Engine engine;
  try {
    engine = Engine.findWithin(engineSrcPath);
  } catch (e) {
    io.stderr.writeln(e);
    io.exitCode = 1;
    return;
  }

  // Find and parse the engine build configs.
  final io.Directory buildConfigsDir = io.Directory(
    p.join(engine.flutterDir.path, 'ci', 'builders'),
  );
  final BuildConfigLoader loader = BuildConfigLoader(buildConfigsDir: buildConfigsDir);

  // Treat it as an error if no build configs were found. The caller likely
  // expected to find some.
  final Map<String, BuilderConfig> configs = loader.configs;
  if (configs.isEmpty) {
    io.stderr.writeln('Error: No build configs found under ${buildConfigsDir.path}');
    io.exitCode = 1;
    return;
  }
  if (loader.errors.isNotEmpty) {
    loader.errors.forEach(io.stderr.writeln);
    io.exitCode = 1;
  }

  // Check the parsed build configs for validity.
  final List<String> invalidErrors = checkForInvalidConfigs(configs);
  if (invalidErrors.isNotEmpty) {
    invalidErrors.forEach(io.stderr.writeln);
    io.exitCode = 1;
  }

  // We require all builds within a builder config to be uniquely named.
  final List<String> duplicateErrors = checkForDuplicateConfigs(configs);
  if (duplicateErrors.isNotEmpty) {
    duplicateErrors.forEach(io.stderr.writeln);
    io.exitCode = 1;
  }

  // We require all builds to be named in a way that is understood by et.
  final List<String> buildNameErrors = checkForInvalidBuildNames(configs);
  if (buildNameErrors.isNotEmpty) {
    buildNameErrors.forEach(io.stderr.writeln);
    io.exitCode = 1;
  }
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
