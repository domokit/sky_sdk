// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:unified_analytics/unified_analytics.dart';

import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

// Xcode 11.4 requires linked and embedded frameworks to contain all targeted architectures before build phases are run.
// This caused issues switching between a real device and simulator due to architecture mismatch.
// Remove the linking and embedding logic from the Xcode project to give the tool more control over these.
class RemoveFrameworkLinkAndEmbeddingMigration extends ProjectMigrator {
  RemoveFrameworkLinkAndEmbeddingMigration(IosProject project, super.logger, Analytics analytics)
    : _xcodeProjectInfoFile = project.xcodeProjectInfoFile,
      _analytics = analytics;

  final File _xcodeProjectInfoFile;
  final Analytics _analytics;

  @override
  Future<void> migrate() async {
    if (!_xcodeProjectInfoFile.existsSync()) {
      logger.printTrace('Xcode project not found, skipping framework link and embedding migration');
      return;
    }

    processFileLines(_xcodeProjectInfoFile);
  }

  @override
  String? migrateLine(String line) {
    // App.framework Frameworks reference.
    // isa = PBXFrameworksBuildPhase;
    // files = (
    //    3B80C3941E831B6300D905FE /* App.framework in Frameworks */,
    if (line.contains('3B80C3941E831B6300D905FE')) {
      return null;
    }

    // App.framework Embed Framework reference (build phase to embed framework).
    // 3B80C3951E831B6300D905FE /* App.framework in Embed Frameworks */,
    if (line.contains('3B80C3951E831B6300D905FE') || line.contains('741F496821356857001E2961')) {
      // Ephemeral add-to-app variant.
      return null;
    }

    // App.framework project file reference (seen in Xcode navigator pane).
    // isa = PBXGroup;
    // children = (
    //	 3B80C3931E831B6300D905FE /* App.framework */,
    if (line.contains('3B80C3931E831B6300D905FE') || line.contains('741F496521356807001E2961')) {
      // Ephemeral add-to-app variant.
      return null;
    }

    // Flutter.framework Frameworks reference.
    // isa = PBXFrameworksBuildPhase;
    // files = (
    //   9705A1C61CF904A100538489 /* Flutter.framework in Frameworks */,
    if (line.contains('9705A1C61CF904A100538489')) {
      return null;
    }

    // Flutter.framework Embed Framework reference (build phase to embed framework).
    // 9705A1C71CF904A300538489 /* Flutter.framework in Embed Frameworks */,
    if (line.contains('9705A1C71CF904A300538489') || line.contains('741F496221355F47001E2961')) {
      // Ephemeral add-to-app variant.
      return null;
    }

    // Flutter.framework project file reference (seen in Xcode navigator pane).
    // isa = PBXGroup;
    // children = (
    //	 9740EEBA1CF902C7004384FC /* Flutter.framework */,
    if (line.contains('9740EEBA1CF902C7004384FC') || line.contains('741F495E21355F27001E2961')) {
      // Ephemeral add-to-app variant.
      return null;
    }

    // Embed and thin frameworks in a script instead of using Xcode's link / embed build phases.
    const String thinBinaryScript = r'xcode_backend.sh\" thin';
    if (line.contains(thinBinaryScript) && !line.contains(' embed')) {
      return line.replaceFirst(thinBinaryScript, r'xcode_backend.sh\" embed_and_thin');
    }

    if (line.contains('/* App.framework ') || line.contains('/* Flutter.framework ')) {
      // Print scary message.
      _analytics.send(
        Event.appleUsageEvent(
          workflow: 'ios-migration',
          parameter: 'remove-frameworks',
          result: 'failure',
        ),
      );
      throwToolExit(
        'Your Xcode project requires migration. See https://docs.flutter.dev/ios-project-migration for details.',
      );
    }

    return line;
  }
}
