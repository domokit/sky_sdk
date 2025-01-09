// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:isolate';
import 'dart:typed_data';

import 'package:package_config/package_config.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';

/// Loads the package configuration of the current isolate.
Future<PackageConfig> currentPackageConfig() async {
  return loadPackageConfigUri(Isolate.packageConfigSync!);
}

/// Locates the `.dart_tool/package_config.json` relevant to [dir].
///
/// Searches [dir] and all parent directories.
///
/// Returns `null` if no package_config.json was found.
// TODO(sigurdm): Only call this once per run - and read in from BuildInfo.
File? findPackageConfigFile(Directory dir) {
  final FileSystem fileSystem = dir.fileSystem;

  Directory candidateDir = fileSystem.directory(dir.path).absolute;
  while (true) {
    final File candidatePackageConfigFile = candidateDir
        .childDirectory('.dart_tool')
        .childFile('package_config.json');
    if (candidatePackageConfigFile.existsSync()) {
      return candidatePackageConfigFile;
    }
    final Directory parentDir = candidateDir.parent;
    if (fileSystem.identicalSync(parentDir.path, candidateDir.path)) {
      return null;
    }
    candidateDir = parentDir;
  }
}

/// Locates the `.dart_tool/package_config.json` relevant to [dir].
///
/// Like [findPackageConfigFile] but returns
/// `$dir/.dart_tool/package_config.json` if no package config could be found.
File findPackageConfigFileOrDefault(Directory dir) {
  return findPackageConfigFile(dir) ??
      dir.childDirectory('.dart_tool').childFile('package_config.json');
}

/// Load the package configuration from [file] or throws a [ToolExit]
/// if the operation would fail.
///
/// If [throwOnError] is false, in the event of an error an empty package
/// config is returned.
Future<PackageConfig> loadPackageConfigWithLogging(
  File file, {
  required Logger logger,
  bool throwOnError = true,
}) async {
  final FileSystem fileSystem = file.fileSystem;
  bool didError = false;
  final PackageConfig result = await loadPackageConfigUri(
    file.absolute.uri,
    loader: (Uri uri) async {
      final File configFile = fileSystem.file(uri);
      if (!configFile.existsSync()) {
        return null;
      }
      return Future<Uint8List>.value(configFile.readAsBytesSync());
    },
    onError: (dynamic error) {
      if (!throwOnError) {
        return;
      }
      logger.printTrace(error.toString());
      String message = '${file.path} does not exist.';
      final String pubspecPath = fileSystem.path.absolute(
        fileSystem.path.dirname(file.path),
        'pubspec.yaml',
      );
      if (fileSystem.isFileSync(pubspecPath)) {
        message += '\nDid you run "flutter pub get" in this directory?';
      } else {
        message += '\nDid you run this command from the same directory as your pubspec.yaml file?';
      }
      logger.printError(message);
      didError = true;
    },
  );
  if (didError) {
    throwToolExit('');
  }
  return result;
}
