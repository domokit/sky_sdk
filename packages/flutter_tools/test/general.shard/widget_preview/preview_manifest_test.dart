// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/widget_preview/preview_manifest.dart';
import 'package:test/test.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  group('$PreviewManifest', () {
    late FlutterProject rootProject;
    late PreviewManifest previewManifest;
    late Logger logger;

    // The version really doesn't matter, just the format.
    const String kFakeSDKVersion = '2.1.0-dev.8.0.flutter-4312ae32';

    setUp(() {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FlutterManifest manifest = FlutterManifest.empty(logger: BufferLogger.test());
      final Directory projectDir = fs.currentDirectory.childDirectory('project')..createSync();
      projectDir.childDirectory('lib/src').createSync(recursive: true);
      rootProject = FlutterProject(projectDir, manifest, manifest);
      logger = BufferLogger.test();
      previewManifest = PreviewManifest(
        logger: logger,
        rootProject: rootProject,
        fs: fs,
        cache: Cache.test(
          processManager: FakeProcessManager.any(),
          fileSystem: fs,
          platform: FakePlatform(version: kFakeSDKVersion),
        ),
      );
    });

    testUsingContext('generates a valid manifest', () async {
      previewManifest.generate();
      final PreviewManifestContents manifest =
          json.decode(
                rootProject.widgetPreviewScaffold
                    .childFile(PreviewManifest.previewManifestPath)
                    .readAsStringSync(),
              )
              as PreviewManifestContents;

      expect(manifest.containsKey(PreviewManifest.kPubspecHash), true);
      expect(manifest.containsKey(PreviewManifest.kManifestVersion), true);
      expect(manifest.containsKey(PreviewManifest.kSdkVersion), true);
    });

    testUsingContext('identifies widget preview scaffold project needs to be generated', () {
      // The widget preview scaffold directory doesn't exist, so we should know that we need to
      // generate the project.
      expect(previewManifest.shouldGenerateProject(), true);

      // Populate the manifest. For this test, this has the side effect of creating the widget
      // preview scaffold project directory as well.
      previewManifest.generate();

      // The widget preview scaffold project directory should exist as well as the newly generated
      // preview manifest.
      expect(previewManifest.shouldGenerateProject(), false);

      // Simulate changing the SDK version and verify that we should regenerate the project.
      final PreviewManifest modified = previewManifest.copyWith(
        cache: Cache.test(
          processManager: FakeProcessManager.any(),
          platform: FakePlatform(version: '${kFakeSDKVersion}foo'),
        ),
      );

      const String sdkMismatchMessage =
          'The existing Widget Preview Scaffold was generated with Dart SDK '
          'version 2.1.0 (build 2.1.0-dev.8.0 4312ae32), which does not match the current Dart '
          'SDK version (2.1.0 (build 2.1.0-dev.8.0 4312ae32foo)). Regenerating Widget Preview '
          'Scaffold.\n';
      expect(modified.shouldGenerateProject(), true);
      expect((modified.logger as BufferLogger).statusText, contains(sdkMismatchMessage));
    });

    testUsingContext('identifies root project pubspec has changed', () {
      // The widget preview scaffold directory doesn't exist, so we should know that we need to
      // generate the project.
      expect(previewManifest.shouldGenerateProject(), true);

      // Populate the manifest. For this test, this has the side effect of creating the widget
      // preview scaffold project directory as well.
      previewManifest.generate();

      // The widget preview scaffold project directory should exist as well as the newly generated
      // preview manifest.
      expect(previewManifest.shouldGenerateProject(), false);

      // Simulate changing the root project's pubspec.yaml and verify that we should regenerate
      // the widget preview scaffold's pubspec.yaml.
      rootProject.replacePubspec(
        rootProject.manifest.copyWith(logger: logger, shaders: <Uri>[Uri(host: 'Random')]),
      );
      expect(previewManifest.shouldRegeneratePubspec(), true);

      // Update the manifest to include the hash for the updated pubspec.yaml and verify that we
      // no longer need to regenerate the pubspec.
      previewManifest.updatePubspecHash();
      expect(previewManifest.shouldRegeneratePubspec(), false);
    });
  });
}
