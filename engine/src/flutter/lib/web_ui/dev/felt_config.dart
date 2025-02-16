// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'package:yaml/yaml.dart';

enum Compiler { dart2js, dart2wasm }

enum Renderer { canvaskit, skwasm }

class CompileConfiguration {
  CompileConfiguration(this.name, this.compiler, this.renderer);

  final String name;
  final Compiler compiler;
  final Renderer renderer;
}

class TestSet {
  TestSet(this.name, this.directory);

  final String name;
  final String directory;
}

class TestBundle {
  TestBundle(this.name, this.testSet, this.compileConfigs);

  final String name;
  final TestSet testSet;
  final List<CompileConfiguration> compileConfigs;
}

enum CanvasKitVariant { full, chromium }

enum BrowserName { chrome, edge, firefox, safari }

class RunConfiguration {
  RunConfiguration(
    this.name,
    this.browser,
    this.variant,
    this.crossOriginIsolated,
    this.forceSingleThreadedSkwasm,
  );

  final String name;
  final BrowserName browser;
  final CanvasKitVariant? variant;
  final bool crossOriginIsolated;
  final bool forceSingleThreadedSkwasm;
}

class ArtifactDependencies {
  ArtifactDependencies({
    required this.canvasKit,
    required this.canvasKitChromium,
    required this.skwasm,
  });

  ArtifactDependencies.none() : canvasKit = false, canvasKitChromium = false, skwasm = false;
  final bool canvasKit;
  final bool canvasKitChromium;
  final bool skwasm;

  ArtifactDependencies operator |(ArtifactDependencies other) {
    return ArtifactDependencies(
      canvasKit: canvasKit || other.canvasKit,
      canvasKitChromium: canvasKitChromium || other.canvasKitChromium,
      skwasm: skwasm || other.skwasm,
    );
  }

  ArtifactDependencies operator &(ArtifactDependencies other) {
    return ArtifactDependencies(
      canvasKit: canvasKit && other.canvasKit,
      canvasKitChromium: canvasKitChromium && other.canvasKitChromium,
      skwasm: skwasm && other.skwasm,
    );
  }
}

class TestSuite {
  TestSuite(this.name, this.testBundle, this.runConfig, this.artifactDependencies);

  String name;
  TestBundle testBundle;
  RunConfiguration runConfig;
  ArtifactDependencies artifactDependencies;
}

class FeltConfig {
  FeltConfig(
    this.compileConfigs,
    this.testSets,
    this.testBundles,
    this.runConfigs,
    this.testSuites,
  );

  factory FeltConfig.fromFile(String filePath) {
    final io.File configFile = io.File(filePath);
    final YamlMap yaml = loadYaml(configFile.readAsStringSync()) as YamlMap;

    final List<CompileConfiguration> compileConfigs = <CompileConfiguration>[];
    final Map<String, CompileConfiguration> compileConfigsByName = <String, CompileConfiguration>{};
    for (final dynamic node in yaml['compile-configs'] as YamlList) {
      final YamlMap configYaml = node as YamlMap;
      final String name = configYaml['name'] as String;
      final Compiler compiler = Compiler.values.byName(configYaml['compiler'] as String);
      final Renderer renderer = Renderer.values.byName(configYaml['renderer'] as String);
      final CompileConfiguration config = CompileConfiguration(name, compiler, renderer);
      compileConfigs.add(config);
      if (compileConfigsByName.containsKey(name)) {
        throw AssertionError('Duplicate compile config name: $name');
      }
      compileConfigsByName[name] = config;
    }

    final List<TestSet> testSets = <TestSet>[];
    final Map<String, TestSet> testSetsByName = <String, TestSet>{};
    for (final dynamic node in yaml['test-sets'] as YamlList) {
      final YamlMap testSetYaml = node as YamlMap;
      final String name = testSetYaml['name'] as String;
      final String directory = testSetYaml['directory'] as String;
      final TestSet testSet = TestSet(name, directory);
      testSets.add(testSet);
      if (testSetsByName.containsKey(name)) {
        throw AssertionError('Duplicate test set name: $name');
      }
      testSetsByName[name] = testSet;
    }

    final List<TestBundle> testBundles = <TestBundle>[];
    final Map<String, TestBundle> testBundlesByName = <String, TestBundle>{};
    for (final dynamic node in yaml['test-bundles'] as YamlList) {
      final YamlMap testBundleYaml = node as YamlMap;
      final String name = testBundleYaml['name'] as String;
      final String testSetName = testBundleYaml['test-set'] as String;
      final TestSet? testSet = testSetsByName[testSetName];
      if (testSet == null) {
        throw AssertionError(
          'Test set not found with name: `$testSetName` (referenced by test bundle: `$name`)',
        );
      }
      final dynamic compileConfigsValue = testBundleYaml['compile-configs'];
      final List<CompileConfiguration> compileConfigs;
      if (compileConfigsValue is String) {
        compileConfigs = <CompileConfiguration>[compileConfigsByName[compileConfigsValue]!];
      } else {
        compileConfigs =
            (compileConfigsValue as List<dynamic>)
                .map((dynamic configName) => compileConfigsByName[configName as String]!)
                .toList();
      }
      final TestBundle bundle = TestBundle(name, testSet, compileConfigs);
      testBundles.add(bundle);
      if (testBundlesByName.containsKey(name)) {
        throw AssertionError('Duplicate test bundle name: $name');
      }
      testBundlesByName[name] = bundle;
    }

    final List<RunConfiguration> runConfigs = <RunConfiguration>[];
    final Map<String, RunConfiguration> runConfigsByName = <String, RunConfiguration>{};
    for (final dynamic node in yaml['run-configs'] as YamlList) {
      final YamlMap runConfigYaml = node as YamlMap;
      final String name = runConfigYaml['name'] as String;
      final BrowserName browser = BrowserName.values.byName(runConfigYaml['browser'] as String);
      final dynamic variantNode = runConfigYaml['canvaskit-variant'];
      final CanvasKitVariant? variant =
          variantNode == null ? null : CanvasKitVariant.values.byName(variantNode as String);
      final bool crossOriginIsolated = runConfigYaml['cross-origin-isolated'] as bool? ?? false;
      final bool forceSingleThreadedSkwasm =
          runConfigYaml['force-single-threaded-skwasm'] as bool? ?? false;
      final RunConfiguration runConfig = RunConfiguration(
        name,
        browser,
        variant,
        crossOriginIsolated,
        forceSingleThreadedSkwasm,
      );
      runConfigs.add(runConfig);
      if (runConfigsByName.containsKey(name)) {
        throw AssertionError('Duplicate run config name: $name');
      }
      runConfigsByName[name] = runConfig;
    }

    final List<TestSuite> testSuites = <TestSuite>[];
    for (final dynamic node in yaml['test-suites'] as YamlList) {
      final YamlMap testSuiteYaml = node as YamlMap;
      final String name = testSuiteYaml['name'] as String;
      final String testBundleName = testSuiteYaml['test-bundle'] as String;
      final TestBundle? bundle = testBundlesByName[testBundleName];
      if (bundle == null) {
        throw AssertionError(
          'Test bundle not found with name: `$testBundleName` (referenced by test suite: `$name`)',
        );
      }
      final String runConfigName = testSuiteYaml['run-config'] as String;
      final RunConfiguration? runConfig = runConfigsByName[runConfigName];
      if (runConfig == null) {
        throw AssertionError(
          'Run config not found with name: `$runConfigName` (referenced by test suite: `$name`)',
        );
      }
      bool canvasKit = false;
      bool canvasKitChromium = false;
      bool skwasm = false;
      final dynamic depsNode = testSuiteYaml['artifact-deps'];
      if (depsNode != null) {
        for (final dynamic dep in depsNode as YamlList) {
          switch (dep as String) {
            case 'canvaskit':
              if (canvasKit) {
                throw AssertionError('Artifact dep $dep listed twice in suite $name.');
              }
              canvasKit = true;
            case 'canvaskit_chromium':
              if (canvasKitChromium) {
                throw AssertionError('Artifact dep $dep listed twice in suite $name.');
              }
              canvasKitChromium = true;
            case 'skwasm':
              if (skwasm) {
                throw AssertionError('Artifact dep $dep listed twice in suite $name.');
              }
              skwasm = true;
            default:
              throw AssertionError('Unrecognized artifact dependency: $dep');
          }
        }
      }
      final ArtifactDependencies artifactDeps = ArtifactDependencies(
        canvasKit: canvasKit,
        canvasKitChromium: canvasKitChromium,
        skwasm: skwasm,
      );
      final TestSuite suite = TestSuite(name, bundle, runConfig, artifactDeps);
      testSuites.add(suite);
    }
    return FeltConfig(compileConfigs, testSets, testBundles, runConfigs, testSuites);
  }

  List<CompileConfiguration> compileConfigs;
  List<TestSet> testSets;
  List<TestBundle> testBundles;
  List<RunConfiguration> runConfigs;
  List<TestSuite> testSuites;
}
