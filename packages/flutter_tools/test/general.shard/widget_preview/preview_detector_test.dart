// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/widget_preview/preview_detector.dart';
import 'package:test/test.dart';

import '../../src/common.dart';
import '../../src/context.dart';

Directory createBasicProjectStructure(FileSystem fs) {
  return fs.systemTempDirectory.createTempSync('root');
}

PreviewPath addPreviewContainingFile(Directory projectRoot, List<String> path) {
  final File file =
      projectRoot.childDirectory('lib').childFile(path.join(const LocalPlatform().pathSeparator))
        ..createSync(recursive: true)
        ..writeAsStringSync(previewContainingFileContents);
  return (path: file.path, uri: file.uri);
}

PreviewPath addNonPreviewContainingFile(Directory projectRoot, List<String> path) {
  final File file =
      projectRoot.childDirectory('lib').childFile(path.join(const LocalPlatform().pathSeparator))
        ..createSync(recursive: true)
        ..writeAsStringSync(nonPreviewContainingFileContents);
  return (path: file.path, uri: file.uri);
}

void main() {
  group('$PreviewDetector', () {
    // Note: we don't use a MemoryFileSystem since we don't have a way to
    // provide it to package:analyzer APIs without writing a significant amount
    // of wrapper logic.
    late LocalFileSystem fs;
    late Logger logger;
    late PreviewDetector previewDetector;
    late Directory projectRoot;
    void Function(PreviewMapping)? onChangeDetected;

    void onChangeDetectedRoot(PreviewMapping mapping) {
      onChangeDetected!(mapping);
    }

    setUp(() {
      fs = LocalFileSystem.test(signals: Signals.test());
      projectRoot = createBasicProjectStructure(fs);
      logger = BufferLogger.test();
      previewDetector = PreviewDetector(
        logger: logger,
        fs: fs,
        onChangeDetected: onChangeDetectedRoot,
      );
    });

    tearDown(() async {
      await previewDetector.dispose();
      projectRoot.deleteSync(recursive: true);
      onChangeDetected = null;
    });

    testUsingContext('can detect previews in existing files', () async {
      final List<PreviewPath> previewFiles = <PreviewPath>[
        addPreviewContainingFile(projectRoot, <String>['foo.dart']),
        addPreviewContainingFile(projectRoot, <String>['src', 'bar.dart']),
      ];
      addNonPreviewContainingFile(projectRoot, <String>['baz.dart']);
      final PreviewMapping mapping = previewDetector.findPreviewFunctions(projectRoot);
      expect(mapping.keys.toSet(), previewFiles.toSet());
    });

    testUsingContext('can detect previews in updated files', () async {
      // Create two files with existing previews and one without.
      final PreviewMapping expectedInitialMapping = <PreviewPath, List<String>>{
        addPreviewContainingFile(projectRoot, <String>['foo.dart']): <String>['previews'],
        addPreviewContainingFile(projectRoot, <String>['src', 'bar.dart']): <String>['previews'],
      };
      final PreviewPath nonPreviewContainingFile = addNonPreviewContainingFile(
        projectRoot,
        <String>['baz.dart'],
      );

      Completer<void> completer = Completer<void>();
      onChangeDetected = (PreviewMapping updated) {
        // The new preview in baz.dart should be included in the preview mapping.
        expect(updated, <PreviewPath, List<String>>{
          ...expectedInitialMapping,
          nonPreviewContainingFile: <String>['previews'],
        });
        completer.complete();
      };
      // Initialize the file watcher.
      final PreviewMapping initialPreviews = await previewDetector.initialize(projectRoot);
      expect(initialPreviews, expectedInitialMapping);

      // Update the file without an existing preview to include a preview and ensure it triggers
      // the preview detector.
      addPreviewContainingFile(projectRoot, <String>['baz.dart']);
      await completer.future;

      completer = Completer<void>();
      onChangeDetected = (PreviewMapping updated) {
        // The removed preview in baz.dart should not longer be included in the preview mapping.
        expect(updated, expectedInitialMapping);
        completer.complete();
      };

      // Update the file with an existing preview to remove the preview and ensure it triggers
      // the preview detector.
      addNonPreviewContainingFile(projectRoot, <String>['baz.dart']);
      await completer.future;
    });
  });
}

const String previewContainingFileContents = '''
@Preview()
// This isn't necessarily valid code. We're just looking for the annotation
WidgetPreview previews() => WidgetPreview();
''';

const String nonPreviewContainingFileContents = '''
String foo() => 'bar';
''';
