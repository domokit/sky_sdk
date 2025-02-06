// Copyright 2025 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'dart:convert';
import 'package:file/file.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

/// Writes a `.dart_tool/package_config.json` file at [directory].
///
/// If directory is not specified, it will be `globals.fs.currentDirectory`;
///
/// It will contain a package entry for [mainLibName] with `rootUri` at
/// [directory].
///
/// [otherLibs] maps other package names to their `rootUri` relative to `directory`.
///
/// Returns the `File` Object representing the package config.
File writePackageConfigFile({
  Directory? directory,
  String mainLibName = 'my_app',
  Map<String, String> packages = const <String, String>{},
}) {
  directory ??= globals.fs.currentDirectory;
  return directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync(
      json.encode(<String, Object?>{
        'packages': <Object>[
          <String, Object?>{'name': mainLibName, 'rootUri': '../', 'packageUri': 'lib/'},
          ...packages.entries.map(
            (MapEntry<String, String> entry) => <String, Object?>{
              'name': entry.key,
              'rootUri': Uri.parse('../').resolve(entry.value).toString(),
              'packageUri': 'lib/',
              'languageVersion': '3.7',
            },
          ),
        ],
        'configVersion': 2,
      }),
    );
}
