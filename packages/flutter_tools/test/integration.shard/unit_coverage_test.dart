// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:process/process.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('unit_coverage_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('Can parse and output summaries for code coverage', () async {
    final File coverageFile = tempDir.childFile('info.lcov')..writeAsStringSync('''
SF:lib/src/artifacts.dart
DA:15,10
DA:17,7
DA:19,7
DA:20,7
DA:22,7
DA:324,10
DA:724,0
DA:727,2
DA:729,3
DA:732,0
DA:737,1
LF:443
LH:292
end_of_record
SF:lib/src/base/common.dart
DA:13,7
DA:14,7
DA:22,7
DA:27,3
DA:28,6
DA:40,7
LF:6
LH:6
end_of_record
''');

    final String dartScript = fileSystem.path.join(getFlutterRoot(), 'bin', 'dart');
    final String coverageScript = fileSystem.path.join(
      getFlutterRoot(),
      'packages',
      'flutter_tools',
      'tool',
      'unit_coverage.dart',
    );
    final ProcessResult result = await const LocalProcessManager().run(<String>[
      dartScript,
      coverageScript,
      coverageFile.path,
    ]);

    // May contain other output if building flutter tool.
    expect(
      result.stdout.toString().split('\n'),
      containsAll(<Matcher>[
        contains('lib/src/artifacts.dart: 81.82%'),
        contains('lib/src/base/common.dart: 100.00%'),
      ]),
    );
  });
}
