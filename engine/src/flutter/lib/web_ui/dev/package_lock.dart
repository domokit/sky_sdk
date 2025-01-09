// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:path/path.dart' as path;
import 'package:yaml/yaml.dart';

import 'environment.dart';

/// Returns the browser configuration based on the `package_lock.yaml` file in
/// the current engine workspace.
final PackageLock packageLock = PackageLock();

/// Provides access to the contents of the `package_lock.yaml` file.
class PackageLock {
  factory PackageLock() {
    final io.File lockFile = io.File(
      path.join(environment.webUiRootDir.path, 'dev', 'package_lock.yaml'),
    );
    final YamlMap yaml = loadYaml(lockFile.readAsStringSync()) as YamlMap;
    return PackageLock._fromYaml(yaml);
  }

  PackageLock._fromYaml(YamlMap yaml)
    : chromeLock = ChromeLock._fromYaml(yaml['chrome'] as YamlMap),
      firefoxLock = FirefoxLock._fromYaml(yaml['firefox'] as YamlMap),
      edgeLock = EdgeLock._fromYaml(yaml['edge'] as YamlMap),
      esbuildLock = EsbuildLock._fromYaml(yaml['esbuild'] as YamlMap);

  final ChromeLock chromeLock;
  final FirefoxLock firefoxLock;
  final EdgeLock edgeLock;
  final EsbuildLock esbuildLock;
}

class ChromeLock {
  ChromeLock._fromYaml(YamlMap yaml) : version = yaml['version'] as String;

  /// The full version of Chromium represented by this lock. E.g: '119.0.6045.9'
  final String version;
}

class FirefoxLock {
  FirefoxLock._fromYaml(YamlMap yaml) : version = yaml['version'] as String;

  final String version;
}

class EdgeLock {
  EdgeLock._fromYaml(YamlMap yaml) : launcherVersion = yaml['launcher_version'] as String;

  final String launcherVersion;
}

class EsbuildLock {
  EsbuildLock._fromYaml(YamlMap yaml) : version = yaml['version'] as String;

  final String version;
}
