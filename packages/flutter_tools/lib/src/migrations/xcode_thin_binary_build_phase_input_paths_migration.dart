// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/project_migrator.dart';
import '../xcode_project.dart';

// Migrate Xcode Thin Binary build phase to depend on Info.plist from build directory
// as an input file to ensure it has been created before inserting the NSBonjourServices key
// to avoid an mDNS error.
class XcodeThinBinaryBuildPhaseInputPathsMigration extends ProjectMigrator {
  XcodeThinBinaryBuildPhaseInputPathsMigration(XcodeBasedProject project, super.logger)
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

    // Add Info.plist from build directory as an input file to Thin Binary build phase.
    // Path for the Info.plist is ${TARGET_BUILD_DIR}/\${INFOPLIST_PATH}

    // Example:
    // 3B06AD1E1E4923F5004D2608 /* Thin Binary */ = {
    //   isa = PBXShellScriptBuildPhase;
    //   alwaysOutOfDate = 1;
    //   buildActionMask = 2147483647;
    //   files = (
    // 	 );
    // 	 inputPaths = (
    // 	 );

    String newProjectContents = originalProjectContents;
    const String thinBinaryBuildPhaseOriginal = '''
		3B06AD1E1E4923F5004D2608 /* Thin Binary */ = {
			isa = PBXShellScriptBuildPhase;
			alwaysOutOfDate = 1;
			buildActionMask = 2147483647;
			files = (
			);
			inputPaths = (
			);
''';

    const String thinBinaryBuildPhaseReplacement = r'''
		3B06AD1E1E4923F5004D2608 /* Thin Binary */ = {
			isa = PBXShellScriptBuildPhase;
			alwaysOutOfDate = 1;
			buildActionMask = 2147483647;
			files = (
			);
			inputPaths = (
				"${TARGET_BUILD_DIR}/${INFOPLIST_PATH}",
			);
''';

    newProjectContents = newProjectContents.replaceAll(
      thinBinaryBuildPhaseOriginal,
      thinBinaryBuildPhaseReplacement,
    );
    if (originalProjectContents != newProjectContents) {
      logger.printStatus('Adding input path to Thin Binary build phase.');
      _xcodeProjectInfoFile.writeAsStringSync(newProjectContents);
    }
  }
}
