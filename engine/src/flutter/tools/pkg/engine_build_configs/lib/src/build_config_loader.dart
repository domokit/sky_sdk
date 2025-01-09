// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' as convert;
import 'dart:io' as io show Directory, File;

import 'package:path/path.dart' as p;

import 'build_config.dart';

/// This is a utility class for reading all of the build configurations from
/// a subdirectory of the engine repo. After building an instance of this class,
/// the build configurations can be accessed on the [configs] getter.
class BuildConfigLoader {
  BuildConfigLoader({required this.buildConfigsDir});

  /// Any errors encountered while parsing and loading the build config files
  /// are accumulated in this list as strings. It should be checked for errors
  /// after the first access to the [configs] getter.
  final List<String> errors = <String>[];

  /// The directory where the engine's build config .json files exist.
  final io.Directory buildConfigsDir;

  /// Walks [buildConfigsDir] looking for .json files, which it attempts to
  /// parse as engine build configs. JSON parsing errors during this process
  /// are added as strings to the [errors] list. That last should be checked
  /// for errors after accessing this getter.
  ///
  /// The [BuilderConfig]s given by this getter should be further checked for
  /// validity by calling `BuildConfig.check()` on each one. See
  /// `bin/check.dart` for an example.
  late final Map<String, BuilderConfig> configs = () {
    return _parseAllBuildConfigs(buildConfigsDir);
  }();

  Map<String, BuilderConfig> _parseAllBuildConfigs(io.Directory dir) {
    final Map<String, BuilderConfig> result = <String, BuilderConfig>{};
    if (!dir.existsSync()) {
      errors.add('${buildConfigsDir.path} does not exist.');
      return result;
    }
    final List<io.File> jsonFiles =
        dir
            .listSync(recursive: true)
            .whereType<io.File>()
            .where((io.File f) => f.path.endsWith('.json'))
            .toList();
    for (final io.File jsonFile in jsonFiles) {
      final String basename = p.basename(jsonFile.path);
      final String name = basename.substring(0, basename.length - 5);
      final String jsonData = jsonFile.readAsStringSync();
      final dynamic maybeJson;
      try {
        maybeJson = convert.jsonDecode(jsonData);
      } on FormatException catch (e) {
        errors.add('While parsing ${jsonFile.path}:\n$e');
        continue;
      }
      if (maybeJson is! Map<String, Object?>) {
        errors.add('${jsonFile.path} did not contain a json map.');
        continue;
      }
      result[name] = BuilderConfig.fromJson(path: jsonFile.path, map: maybeJson);
    }
    return result;
  }
}
