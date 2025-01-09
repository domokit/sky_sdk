// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:build_bucket_golden_scraper/build_bucket_golden_scraper.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

void main() {
  test('parses command-line arguments', () {
    // Create a fake engine directory.
    final io.Directory buildRoot = io.Directory.systemTemp.createTempSync(
      'build_bucket_golden_scraper_test_engine',
    );
    final io.Directory srcDir = io.Directory(p.join(buildRoot.path, 'src'))..createSync();
    io.Directory(p.join(srcDir.path, 'flutter')).createSync();

    // Does not run, but just parses.
    try {
      final BuildBucketGoldenScraper scraper = BuildBucketGoldenScraper.fromCommandLine(<String>[
        '--dry-run',
        '--engine-src-path',
        srcDir.path,
        'https://ci.chromium.org/raw/buildbucket/v1/builders/flutter/flutter-linux/builder:linux_bare',
      ]);

      expect(scraper.dryRun, isTrue);
      expect(scraper.engine.srcDir.path, srcDir.path);
      expect(
        scraper.pathOrUrl,
        'https://ci.chromium.org/raw/buildbucket/v1/builders/flutter/flutter-linux/builder:linux_bare',
      );
    } finally {
      buildRoot.deleteSync(recursive: true);
    }
  });

  test('finds diffs', () async {
    // Create a fake engine directory.
    final io.Directory buildRoot = io.Directory.systemTemp.createTempSync(
      'build_bucket_golden_scraper_test_engine',
    );
    final io.Directory srcDir = io.Directory(p.join(buildRoot.path, 'src'))..createSync();
    io.Directory(p.join(srcDir.path, 'flutter', 'docs')).createSync(recursive: true);

    // Create an empty logo in docs/flutter_logo.png.
    final io.File logo = io.File(p.join(srcDir.path, 'flutter', 'docs', 'flutter_logo.png'))
      ..createSync();

    // Create a fake log file.
    try {
      final io.File logFile = io.File(p.join(buildRoot.path, 'log.txt'))..createSync();
      logFile.writeAsStringSync('''
See also the base64 encoded /b/s/w/ir/cache/builder/src/flutter/docs/flutter_logo_new.png:
iVBORw0KGgoAAAANSUhEUgAAADcAAAA3CAYAAACo29JGAAAAAXNSR0IArs4c6QAAAAlwSFlzAAALEwAACxMBAJqcGAAAA6hpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IlhNUCBDb3JlIDUuNC4wIj4KICAgPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4KICAgICAgPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIKICAgICAgICAgICAgeG1sbnM6eG1wPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvIgogICAgICAgICAgICB4bWxuczp0aWZmPSJodHRwOi8vbnMuYWRvYmUuY29tL3RpZmYvMS4wLyIKICAgICAgICAgICAgeG1sbnM6ZXhpZj0iaHR0cDovL25zLmFkb2JlLmNvbS9leGlmLzEuMC8iPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxOS0wOC0yN1QyMDowODo0NzwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6Q3JlYXRvclRvb2w+UGl4ZWxtYXRvciAzLjguNTwveG1wOkNyZWF0b3JUb29sPgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpDb21wcmVzc2lvbj4wPC90aWZmOkNvbXByZXNzaW9uPgogICAgICAgICA8dGlmZjpSZXNvbHV0aW9uVW5pdD4yPC90aWZmOlJlc29sdXRpb25Vbml0PgogICAgICAgICA8dGlmZjpZUmVzb2x1dGlvbj43MjwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6WFJlc29sdXRpb24+NzI8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDxleGlmOlBpeGVsWERpbWVuc2lvbj41NTwvZXhpZjpQaXhlbFhEaW1lbnNpb24+CiAgICAgICAgIDxleGlmOkNvbG9yU3BhY2U+MTwvZXhpZjpDb2xvclNwYWNl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
''');

      // Tell the runner to extract diffs.
      final StringBuffer outSink = StringBuffer();
      final BuildBucketGoldenScraper scraper = BuildBucketGoldenScraper(
        pathOrUrl: logFile.path,
        engineSrcPath: srcDir.path,
        outSink: outSink,
      );

      // Run.
      final int result = await scraper.run();
      expect(result, 0);

      // Check the output.
      expect(outSink.toString(), contains('Wrote 1 golden file changes:'));
      expect(outSink.toString(), contains('docs/flutter_logo.png'));
      expect(logo.readAsBytesSync().length, 4257);
    } finally {
      buildRoot.deleteSync(recursive: true);
    }
  });
}
