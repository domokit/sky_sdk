// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../src/common.dart';
import '../src/context.dart';
import '../src/test_flutter_command_runner.dart';
import 'test_utils.dart';

void main() {
  group('pass analyze template:', () {
    final List<String> templates = <String>['app', 'module', 'package', 'plugin', 'plugin_ffi'];
    late Directory tempDir;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync(
        'flutter_tools_analyze_all_template.',
      );
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    for (final String template in templates) {
      testUsingContext('analysis for $template', () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template', template],
        );
        final ProcessResult result = await globals.processManager.run(<String>[
          'flutter',
          'analyze',
        ], workingDirectory: projectPath);

        expect(result, const ProcessResultMatcher());
      });
    }
  });
}
