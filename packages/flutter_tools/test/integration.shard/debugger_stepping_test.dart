// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';

import '../src/common.dart';
import 'test_data/stepping_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('debugger_stepping_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('can step over statements', () async {
    final SteppingProject project = SteppingProject();
    await project.setUpIn(tempDir);

    final FlutterRunTestDriver flutter = FlutterRunTestDriver(tempDir);

    await flutter.run(withDebugger: true, startPaused: true);
    await flutter.addBreakpoint(project.breakpointUri, project.breakpointLine);
    await flutter.resume(waitForNextPause: true); // Now we should be on the breakpoint.

    expect((await flutter.getSourceLocation())?.line, equals(project.breakpointLine));

    // Issue 5 steps, ensuring that we end up on the annotated lines each time.
    for (int i = 1; i <= project.numberOfSteps; i += 1) {
      await flutter.stepOverOrOverAsyncSuspension();
      final SourcePosition? location = await flutter.getSourceLocation();
      final int? actualLine = location?.line;

      // Get the line we're expected to stop at by searching for the comment
      // within the source code.
      final int expectedLine = project.lineForStep(i);

      expect(
        actualLine,
        equals(expectedLine),
        reason: 'After $i steps, debugger should stop at $expectedLine but stopped at $actualLine',
      );
    }

    await flutter.stop();
  });
}
