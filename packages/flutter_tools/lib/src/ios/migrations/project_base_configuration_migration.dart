// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

// The Runner target should inherit its build configuration from Generated.xcconfig.
// However the top-level Runner project should not inherit any build configuration so
// the Flutter build settings do not stomp on non-Flutter targets.
class ProjectBaseConfigurationMigration extends ProjectMigrator {
  ProjectBaseConfigurationMigration(IosProject project, super.logger)
    : _xcodeProjectInfoFile = project.xcodeProjectInfoFile;

  final File _xcodeProjectInfoFile;

  @override
  Future<void> migrate() async {
    if (!_xcodeProjectInfoFile.existsSync()) {
      logger.printTrace(
        'Xcode project not found, skipping Runner project build settings and configuration migration',
      );
      return;
    }

    final String originalProjectContents = _xcodeProjectInfoFile.readAsStringSync();
    // Example:
    //
    // 		97C146E91CF9000F007C117D /* Build configuration list for PBXProject "Runner" */ = {
    //			isa = XCConfigurationList;
    //			buildConfigurations = (
    //				97C147031CF9000F007C1171 /* Debug */,
    //				97C147041CF9000F007C1171 /* Release */,
    //				2436755321828D23008C7051 /* Profile */,
    //			);
    final RegExp projectBuildConfigurationList = RegExp(
      r'\/\* Build configuration list for PBXProject "Runner" \*\/ = {\s*isa = XCConfigurationList;\s*buildConfigurations = \(\s*(.*) \/\* Debug \*\/,\s*(.*) \/\* Release \*\/,\s*(.*) \/\* Profile \*\/,',
      multiLine: true,
    );

    final RegExpMatch? match = projectBuildConfigurationList.firstMatch(originalProjectContents);

    // If the PBXProject "Runner" build configuration identifiers can't be parsed, default to the generated template identifiers.
    final String debugIdentifier = match?.group(1) ?? '97C147031CF9000F007C117D';
    final String releaseIdentifier = match?.group(2) ?? '97C147041CF9000F007C117D';
    final String profileIdentifier = match?.group(3) ?? '249021D3217E4FDB00AE95B9';

    // Debug
    final String debugBaseConfigurationOriginal = '''
		$debugIdentifier /* Debug */ = {
			isa = XCBuildConfiguration;
			baseConfigurationReference = 9740EEB21CF90195004384FC /* Debug.xcconfig */;
''';
    final String debugBaseConfigurationReplacement = '''
		$debugIdentifier /* Debug */ = {
			isa = XCBuildConfiguration;
''';
    String newProjectContents = originalProjectContents.replaceAll(
      debugBaseConfigurationOriginal,
      debugBaseConfigurationReplacement,
    );

    // Profile
    final String profileBaseConfigurationOriginal = '''
		$profileIdentifier /* Profile */ = {
			isa = XCBuildConfiguration;
			baseConfigurationReference = 7AFA3C8E1D35360C0083082E /* Release.xcconfig */;
''';
    final String profileBaseConfigurationReplacement = '''
		$profileIdentifier /* Profile */ = {
			isa = XCBuildConfiguration;
''';
    newProjectContents = newProjectContents.replaceAll(
      profileBaseConfigurationOriginal,
      profileBaseConfigurationReplacement,
    );

    // Release
    final String releaseBaseConfigurationOriginal = '''
		$releaseIdentifier /* Release */ = {
			isa = XCBuildConfiguration;
			baseConfigurationReference = 7AFA3C8E1D35360C0083082E /* Release.xcconfig */;
''';
    final String releaseBaseConfigurationReplacement = '''
		$releaseIdentifier /* Release */ = {
			isa = XCBuildConfiguration;
''';

    newProjectContents = newProjectContents.replaceAll(
      releaseBaseConfigurationOriginal,
      releaseBaseConfigurationReplacement,
    );
    if (originalProjectContents != newProjectContents) {
      logger.printStatus('Project base configurations detected, removing.');
      _xcodeProjectInfoFile.writeAsStringSync(newProjectContents);
    }
  }
}
