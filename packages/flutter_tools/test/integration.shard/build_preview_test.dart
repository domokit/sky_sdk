// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';

import '../src/common.dart';
import '../src/context.dart';
import 'test_utils.dart';

void main() {
  Cache.disableLocking();

  late Directory tempDir;
  final FileSystem fs = LocalFileSystemBlockingSetCurrentDirectory();
  final File previewBin = fs
      .directory(getFlutterRoot())
      .childDirectory('bin')
      .childDirectory('cache')
      .childDirectory('artifacts')
      .childDirectory('flutter_preview')
      .childFile('flutter_preview.exe');

  setUp(() {
    tempDir = fs.systemTempDirectory.createTempSync('flutter_tools_preview_integration_test.');
  });

  tearDown(() {
    tryToDelete(tempDir);
    tryToDelete(previewBin);
  });

  testUsingContext(
    'flutter build _preview creates preview device',
    () async {
      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'build',
        '_preview',
        '--verbose',
      ]);

      expect(result, const ProcessResultMatcher());
      expect(previewBin, exists);
    },
    // [intended] Flutter Preview only supported on Windows currently
    skip: !const LocalPlatform().isWindows,
  );
}
