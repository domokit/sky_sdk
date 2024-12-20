// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/isolated/mustache_template.dart';
import 'package:flutter_tools/src/template.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('kotlin reserved keywords', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    final Directory rootDir = fileSystem.currentDirectory;
    final Directory templateSource = rootDir.childDirectory('src');
    final Directory imageSourceDir = templateSource;
    final Directory destination = rootDir.childDirectory('dest');

    const String outputClass = 'SomeClass.kt';

    final File sourceFile = templateSource.childFile('$outputClass.tmpl');

    templateSource.createSync();
    sourceFile.writeAsStringSync('package {{androidIdentifier}};');

    final Template template = Template(
      templateSource,
      imageSourceDir,
      fileSystem: fileSystem,
      logger: logger,
      templateRenderer: const MustacheTemplateRenderer(),
    );

    final Map<String, Object> context = <String, Object>{'androidIdentifier': 'is.in.when.there'};
    template.render(destination, context);

    final File destinationFile = destination.childFile(outputClass);
    expect(destinationFile.readAsStringSync(), equals('package `is`.`in`.`when`.there;'));
  });
}
