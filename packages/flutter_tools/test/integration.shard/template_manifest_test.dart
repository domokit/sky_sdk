// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/convert.dart';
import '../src/common.dart';
import 'test_utils.dart';

/// Checks that all active template files are defined in the template_manifest.json file.
void main() {
  testWithoutContext('Check template manifest is up to date', () {
    final Map<String, Object?> manifest =
        json.decode(fileSystem.file('templates/template_manifest.json').readAsStringSync())
            as Map<String, Object?>;
    final Set<Uri> declaredFileList = Set<Uri>.from(
      (manifest['files']! as List<Object?>).cast<String>().map<Uri>(fileSystem.path.toUri),
    );

    final Set<Uri> activeTemplateList =
        fileSystem
            .directory('templates')
            .listSync(recursive: true)
            .whereType<File>()
            .where(
              (File file) =>
                  fileSystem.path.basename(file.path) != 'template_manifest.json' &&
                  fileSystem.path.basename(file.path) != 'README.md' &&
                  fileSystem.path.basename(file.path) != '.DS_Store',
            )
            .map((File file) => file.uri)
            .toSet();

    final Set<Uri> difference = activeTemplateList.difference(declaredFileList);

    expect(difference, isEmpty, reason: 'manifest and template directory should be in-sync');
  });
}
