// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:unified_analytics/unified_analytics.dart';

import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

// Remove the linking and embedding logic from the Xcode project to give the tool more control over these.
class RemoveMacOSFrameworkLinkAndEmbeddingMigration extends ProjectMigrator {
  RemoveMacOSFrameworkLinkAndEmbeddingMigration(
    MacOSProject project,
    super.logger,
    Analytics analytics,
  ) : _xcodeProjectInfoFile = project.xcodeProjectInfoFile,
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
    //     D73912F022F37F9E000D13A0 /* App.framework in Frameworks */,
    if (line.contains('D73912F022F37F9E000D13A0')) {
      return null;
    }

    // App.framework Embed Framework reference (build phase to embed framework).
    // D73912F222F3801D000D13A0 /* App.framework in Bundle Framework */,
    if (line.contains('D73912F222F3801D000D13A0')) {
      return null;
    }

    // App.framework project file reference (seen in Xcode navigator pane).
    // isa = PBXGroup;
    // children = (
    //	  D73912EF22F37F9E000D13A0 /* App.framework */,
    if (line.contains('D73912EF22F37F9E000D13A0')) {
      return null;
    }

    // FlutterMacOS.framework Frameworks reference.
    // isa = PBXFrameworksBuildPhase;
    // files = (
    //   33D1A10422148B71006C7A3E /* FlutterMacOS.framework in Frameworks */,
    if (line.contains('33D1A10422148B71006C7A3E')) {
      return null;
    }

    // FlutterMacOS.framework Embed Framework reference (build phase to embed framework).
    // 33D1A10522148B93006C7A3E /* FlutterMacOS.framework in Bundle Framework */,
    if (line.contains('33D1A10522148B93006C7A3E')) {
      return null;
    }

    // FlutterMacOS.framework project file reference (seen in Xcode navigator pane).
    // isa = PBXGroup;
    // children = (
    //	 33D1A10322148B71006C7A3E /* FlutterMacOS.framework */,
    if (line.contains('33D1A10322148B71006C7A3E')) {
      return null;
    }

    // Embed frameworks in a script instead of using Xcode's link / embed build phases.
    const String thinBinaryScript = r'/Flutter/ephemeral/.app_filename';
    if (line.contains(thinBinaryScript) && !line.contains(' embed')) {
      return line.replaceFirst(
        thinBinaryScript,
        r'/Flutter/ephemeral/.app_filename && \"$FLUTTER_ROOT\"/packages/flutter_tools/bin/macos_assemble.sh embed',
      );
    }

    if (line.contains('/* App.framework ') || line.contains('/* FlutterMacOS.framework ')) {
      _analytics.send(
        Event.appleUsageEvent(
          workflow: 'macos-migration',
          parameter: 'remove-frameworks',
          result: 'failure',
        ),
      );
      throwToolExit('Your Xcode project requires migration.');
    }

    return line;
  }
}
