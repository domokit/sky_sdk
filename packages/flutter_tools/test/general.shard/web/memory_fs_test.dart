// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/web/memory_fs.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('correctly parses source, source map, metadata, manifest files', () {
    final MemoryFileSystem fileSystem = MemoryFileSystem();
    final File source = fileSystem.file('source')..writeAsStringSync('main() {}');
    final File sourcemap = fileSystem.file('sourcemap')..writeAsStringSync('{}');
    final File metadata = fileSystem.file('metadata')..writeAsStringSync('{}');
    final File manifest = fileSystem.file('manifest')..writeAsStringSync(
      json.encode(<String, Object>{
        '/foo.js': <String, Object>{
          'code': <int>[0, source.lengthSync()],
          'sourcemap': <int>[0, 2],
          'metadata': <int>[0, 2],
        },
      }),
    );
    final WebMemoryFS webMemoryFS = WebMemoryFS();
    webMemoryFS.write(source, manifest, sourcemap, metadata);

    expect(utf8.decode(webMemoryFS.files['foo.js']!), 'main() {}');
    expect(utf8.decode(webMemoryFS.sourcemaps['foo.js.map']!), '{}');
    expect(utf8.decode(webMemoryFS.metadataFiles['foo.js.metadata']!), '{}');
    expect(webMemoryFS.mergedMetadata, '{}');
  });
}
