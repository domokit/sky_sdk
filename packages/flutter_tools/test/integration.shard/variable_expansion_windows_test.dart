// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/84270 .
  testWithoutContext(
    'dart command will expand variables on windows',
    () async {
      final ProcessResult result = await processManager.run(<String>[
        fileSystem.path.join(getFlutterRoot(), 'bin', 'dart'),
        fileSystem.path.join(
          getFlutterRoot(),
          'packages',
          'flutter_tools',
          'test',
          'integration.shard',
          'variable_expansion_windows.dart',
        ),
        '"^(?!Golden).+"',
      ]);
      expect(result.stdout, contains('args: ["(?!Golden).+"]'));
    },
    // https://github.com/flutter/flutter/issues/87934
    skip: 'Reverted in https://github.com/flutter/flutter/pull/86000',
  );
}
