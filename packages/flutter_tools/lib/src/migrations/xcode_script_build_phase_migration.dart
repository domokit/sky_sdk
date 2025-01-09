// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/project_migrator.dart';
import '../xcode_project.dart';

// Migrate Xcode build phases that build and embed the Flutter and
// compiled dart frameworks.
class XcodeScriptBuildPhaseMigration extends ProjectMigrator {
  XcodeScriptBuildPhaseMigration(XcodeBasedProject project, super.logger)
    : _xcodeProjectInfoFile = project.xcodeProjectInfoFile;

  final File _xcodeProjectInfoFile;

  @override
  Future<void> migrate() async {
    if (!_xcodeProjectInfoFile.existsSync()) {
      logger.printTrace(
        'Xcode project not found, skipping script build phase dependency analysis removal.',
      );
      return;
    }

    final String originalProjectContents = _xcodeProjectInfoFile.readAsStringSync();

    // Uncheck "Based on dependency analysis" which causes a warning in Xcode 14.
    // Unchecking sets "alwaysOutOfDate = 1" in the Xcode project file.

    // Example:
    // 3B06AD1E1E4923F5004D2608 /* Thin Binary */ = {
    //   isa = PBXShellScriptBuildPhase;
    //   buildActionMask = 2147483647;

    final List<String> scriptIdentifierLinesToMigrate = <String>[
      '3B06AD1E1E4923F5004D2608 /* Thin Binary */', // iOS template
      '9740EEB61CF901F6004384FC /* Run Script */', // iOS template
      '3399D490228B24CF009A79C7 /* ShellScript */', // macOS Runner target (not Flutter Assemble)
    ];

    String newProjectContents = originalProjectContents;
    for (final String scriptIdentifierLine in scriptIdentifierLinesToMigrate) {
      final String scriptBuildPhaseOriginal = '''
		$scriptIdentifierLine = {
			isa = PBXShellScriptBuildPhase;
			buildActionMask = 2147483647;
''';
      final String scriptBuildPhaseReplacement = '''
		$scriptIdentifierLine = {
			isa = PBXShellScriptBuildPhase;
			alwaysOutOfDate = 1;
			buildActionMask = 2147483647;
''';
      newProjectContents = newProjectContents.replaceAll(
        scriptBuildPhaseOriginal,
        scriptBuildPhaseReplacement,
      );
    }
    if (originalProjectContents != newProjectContents) {
      logger.printStatus('Removing script build phase dependency analysis.');
      _xcodeProjectInfoFile.writeAsStringSync(newProjectContents);
    }
  }
}
