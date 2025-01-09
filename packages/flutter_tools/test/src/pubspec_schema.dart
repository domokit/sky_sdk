// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:yaml/yaml.dart';

import 'common.dart';

/// Check if the pubspec.yaml file under the `projectDir` is valid for a plugin project.
void validatePubspecForPlugin({
  required String projectDir,
  String? pluginClass,
  bool ffiPlugin = false,
  required List<String> expectedPlatforms,
  List<String> unexpectedPlatforms = const <String>[],
  String? androidIdentifier,
  String? webFileName,
}) {
  assert(pluginClass != null || ffiPlugin);
  final FlutterManifest manifest =
      FlutterManifest.createFromPath(
        '$projectDir/pubspec.yaml',
        fileSystem: globals.fs,
        logger: globals.logger,
      )!;
  final YamlMap platformMaps = YamlMap.wrap(manifest.supportedPlatforms!);
  for (final String platform in expectedPlatforms) {
    expect(platformMaps[platform], isNotNull);
    final YamlMap platformMap = platformMaps[platform]! as YamlMap;
    if (pluginClass != null) {
      expect(platformMap['pluginClass'], pluginClass);
    }
    if (platform == 'android') {
      expect(platformMap['package'], androidIdentifier);
    }
    if (platform == 'web') {
      expect(platformMap['fileName'], webFileName);
    }
  }
  for (final String platform in unexpectedPlatforms) {
    expect(platformMaps[platform], isNull);
  }
}
