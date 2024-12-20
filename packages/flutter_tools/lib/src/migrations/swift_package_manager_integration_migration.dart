// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:xml/xml.dart';

import '../base/common.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/project_migrator.dart';
import '../build_info.dart';
import '../convert.dart';
import '../features.dart';
import '../ios/plist_parser.dart';
import '../ios/xcodeproj.dart';
import '../project.dart';

/// Swift Package Manager integration requires changes to the Xcode project's
/// project.pbxproj and xcscheme. This class handles making those changes.
class SwiftPackageManagerIntegrationMigration extends ProjectMigrator {
  SwiftPackageManagerIntegrationMigration(
    XcodeBasedProject project,
    SupportedPlatform platform,
    BuildInfo buildInfo, {
    required XcodeProjectInterpreter xcodeProjectInterpreter,
    required Logger logger,
    required FileSystem fileSystem,
    required PlistParser plistParser,
    required FeatureFlags features,
  }) : _xcodeProject = project,
       _platform = platform,
       _buildInfo = buildInfo,
       _xcodeProjectInfoFile = project.xcodeProjectInfoFile,
       _xcodeProjectInterpreter = xcodeProjectInterpreter,
       _fileSystem = fileSystem,
       _plistParser = plistParser,
       _features = features,
       super(logger);

  final XcodeBasedProject _xcodeProject;
  final SupportedPlatform _platform;
  final BuildInfo _buildInfo;
  final XcodeProjectInterpreter _xcodeProjectInterpreter;
  final FileSystem _fileSystem;
  final File _xcodeProjectInfoFile;
  final PlistParser _plistParser;
  final FeatureFlags _features;

  /// New identifier for FlutterGeneratedPluginSwiftPackage PBXBuildFile.
  static const String _flutterPluginsSwiftPackageBuildFileIdentifier = '78A318202AECB46A00862997';

  /// New identifier for FlutterGeneratedPluginSwiftPackage XCLocalSwiftPackageReference.
  static const String _localFlutterPluginsSwiftPackageReferenceIdentifier =
      '781AD8BC2B33823900A9FFBB';

  /// New identifier for FlutterGeneratedPluginSwiftPackage XCSwiftPackageProductDependency.
  static const String _flutterPluginsSwiftPackageProductDependencyIdentifier =
      '78A3181F2AECB46A00862997';

  /// Existing iOS identifier for Runner PBXFrameworksBuildPhase.
  static const String _iosRunnerFrameworksBuildPhaseIdentifier = '97C146EB1CF9000F007C117D';

  /// Existing macOS identifier for Runner PBXFrameworksBuildPhase.
  static const String _macosRunnerFrameworksBuildPhaseIdentifier = '33CC10EA2044A3C60003C045';

  /// Existing iOS identifier for Runner PBXNativeTarget.
  static const String _iosRunnerNativeTargetIdentifier = '97C146ED1CF9000F007C117D';

  /// Existing macOS identifier for Runner PBXNativeTarget.
  static const String _macosRunnerNativeTargetIdentifier = '33CC10EC2044A3C60003C045';

  /// Existing iOS identifier for Runner PBXProject.
  static const String _iosProjectIdentifier = '97C146E61CF9000F007C117D';

  /// Existing macOS identifier for Runner PBXProject.
  static const String _macosProjectIdentifier = '33CC10E52044A3C60003C045';

  File get backupProjectSettings =>
      _fileSystem.directory(_xcodeProjectInfoFile.parent).childFile('project.pbxproj.backup');

  String get _runnerFrameworksBuildPhaseIdentifier {
    return _platform == SupportedPlatform.ios
        ? _iosRunnerFrameworksBuildPhaseIdentifier
        : _macosRunnerFrameworksBuildPhaseIdentifier;
  }

  String get _runnerNativeTargetIdentifier {
    return _platform == SupportedPlatform.ios
        ? _iosRunnerNativeTargetIdentifier
        : _macosRunnerNativeTargetIdentifier;
  }

  String get _projectIdentifier {
    return _platform == SupportedPlatform.ios ? _iosProjectIdentifier : _macosProjectIdentifier;
  }

  void restoreFromBackup(SchemeInfo? schemeInfo) {
    if (backupProjectSettings.existsSync()) {
      logger.printTrace('Restoring project settings from backup file...');
      backupProjectSettings.copySync(_xcodeProject.xcodeProjectInfoFile.path);
    }
    schemeInfo?.backupSchemeFile?.copySync(schemeInfo.schemeFile.path);
  }

  /// Add Swift Package Manager integration to Xcode project's project.pbxproj
  /// and Runner.xcscheme.
  ///
  /// If migration fails or project.pbxproj or Runner.xcscheme becomes invalid,
  /// will revert any changes made and throw an error.
  @override
  Future<void> migrate() async {
    if (!_features.isSwiftPackageManagerEnabled) {
      logger.printTrace(
        'The Swift Package Manager feature is off. '
        'Skipping the migration that adds Swift Package Manager integration...',
      );
      return;
    }

    if (!_xcodeProject.flutterPluginSwiftPackageManifest.existsSync()) {
      logger.printTrace(
        'The tool did not generate a Swift package. '
        "This can happen if the project doesn't have any plugins. "
        'Skipping the migration that adds Swift Package Manager integration...',
      );
      return;
    }

    Status? migrationStatus;
    SchemeInfo? schemeInfo;
    try {
      if (!_xcodeProjectInfoFile.existsSync()) {
        throw Exception('Xcode project not found.');
      }

      schemeInfo = await _getSchemeFile();

      // Check for specific strings in the xcscheme and pbxproj to see if the
      // project has been already migrated, whether automatically or manually.
      final bool isSchemeMigrated = _isSchemeMigrated(schemeInfo);
      final bool isPbxprojMigrated = _xcodeProject.flutterPluginSwiftPackageInProjectSettings;
      if (isSchemeMigrated && isPbxprojMigrated) {
        return;
      }

      migrationStatus = logger.startProgress('Adding Swift Package Manager integration...');

      if (isSchemeMigrated) {
        logger.printTrace('${schemeInfo.schemeFile.basename} already migrated. Skipping...');
      } else {
        _migrateScheme(schemeInfo);
      }
      if (isPbxprojMigrated) {
        logger.printTrace('${_xcodeProjectInfoFile.basename} already migrated. Skipping...');
      } else {
        _migratePbxproj();
      }

      logger.printTrace('Validating project settings...');

      // Re-parse the project settings to check for syntax errors.
      final ParsedProjectInfo updatedInfo = _parsePbxproj();

      // If pbxproj was not already migrated, verify settings were set correctly.
      if (!isPbxprojMigrated) {
        if (!_isPbxprojMigratedCorrectly(updatedInfo, logErrorIfNotMigrated: true)) {
          throw Exception('Settings were not updated correctly.');
        }
      }

      // Get the project info to make sure it compiles with xcodebuild
      await _xcodeProjectInterpreter.getInfo(_xcodeProject.hostAppRoot.path);
    } on Exception catch (e) {
      restoreFromBackup(schemeInfo);
      // TODO(vashworth): Add link to instructions on how to manually integrate
      // once available on website.
      throwToolExit(
        'An error occurred when adding Swift Package Manager integration:\n'
        '  $e\n\n'
        'Swift Package Manager is currently an experimental feature, please file a bug at\n'
        '  https://github.com/flutter/flutter/issues/new?template=1_activation.yml \n'
        'Consider including a copy of the following files in your bug report:\n'
        '  ${_platform.name}/Runner.xcodeproj/project.pbxproj\n'
        '  ${_platform.name}/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme '
        '(or the scheme for the flavor used)\n\n'
        'To avoid this failure, disable Flutter Swift Package Manager integration for the project\n'
        'by adding the following in the project\'s pubspec.yaml under the "flutter" section:\n'
        '  "disable-swift-package-manager: true"\n'
        'Alternatively, disable Flutter Swift Package Manager integration globally with the\n'
        'following command:\n'
        '  "flutter config --no-enable-swift-package-manager"\n',
      );
    } finally {
      ErrorHandlingFileSystem.deleteIfExists(backupProjectSettings);
      if (schemeInfo?.backupSchemeFile != null) {
        ErrorHandlingFileSystem.deleteIfExists(schemeInfo!.backupSchemeFile!);
      }
      migrationStatus?.stop();
    }
  }

  Future<SchemeInfo> _getSchemeFile() async {
    final XcodeProjectInfo? projectInfo = await _xcodeProject.projectInfo();
    if (projectInfo == null) {
      throw Exception('Unable to get Xcode project info.');
    }
    if (_xcodeProject.xcodeWorkspace == null) {
      throw Exception('Xcode workspace not found.');
    }
    final String? scheme = projectInfo.schemeFor(_buildInfo);
    if (scheme == null) {
      projectInfo.reportFlavorNotFoundAndExit();
    }

    final File schemeFile = _xcodeProject.xcodeProjectSchemeFile(scheme: scheme);
    if (!schemeFile.existsSync()) {
      throw Exception('Unable to get scheme file for $scheme.');
    }

    final String schemeContent = schemeFile.readAsStringSync();
    return SchemeInfo(schemeName: scheme, schemeFile: schemeFile, schemeContent: schemeContent);
  }

  bool _isSchemeMigrated(SchemeInfo schemeInfo) {
    if (schemeInfo.schemeContent.contains('Run Prepare Flutter Framework Script')) {
      return true;
    }
    return false;
  }

  void _migrateScheme(SchemeInfo schemeInfo) {
    final File schemeFile = schemeInfo.schemeFile;
    final String schemeContent = schemeInfo.schemeContent;

    // The scheme should have a BuildableReference already in it with a
    // BlueprintIdentifier matching the Runner Native Target. Copy from it
    // since BuildableName, BlueprintName, ReferencedContainer may have been
    // changed from "Runner". Ensures the expected attributes are found.
    // Example:
    // <BuildableReference
    //     BuildableIdentifier = "primary"
    //     BlueprintIdentifier = "97C146ED1CF9000F007C117D"
    //     BuildableName = "Runner.app"
    //     BlueprintName = "Runner"
    //     ReferencedContainer = "container:Runner.xcodeproj">
    // </BuildableReference>
    final List<String> schemeLines = LineSplitter.split(schemeContent).toList();
    final int index = schemeLines.indexWhere(
      (String line) => line.contains('BlueprintIdentifier = "$_runnerNativeTargetIdentifier"'),
    );
    if (index == -1 || index + 3 >= schemeLines.length) {
      throw Exception(
        'Failed to parse ${schemeFile.basename}: Could not find BuildableReference '
        'for ${_xcodeProject.hostAppProjectName}.',
      );
    }

    final String buildableName = schemeLines[index + 1].trim();
    if (!buildableName.contains('BuildableName')) {
      throw Exception('Failed to parse ${schemeFile.basename}: Could not find BuildableName.');
    }

    final String blueprintName = schemeLines[index + 2].trim();
    if (!blueprintName.contains('BlueprintName')) {
      throw Exception('Failed to parse ${schemeFile.basename}: Could not find BlueprintName.');
    }

    final String referencedContainer = schemeLines[index + 3].trim();
    if (!referencedContainer.contains('ReferencedContainer')) {
      throw Exception(
        'Failed to parse ${schemeFile.basename}: Could not find ReferencedContainer.',
      );
    }

    schemeInfo.backupSchemeFile = schemeFile.parent.childFile('${schemeFile.basename}.backup');
    schemeFile.copySync(schemeInfo.backupSchemeFile!.path);

    final String scriptText;
    if (_platform == SupportedPlatform.ios) {
      scriptText =
          r'scriptText = "/bin/sh &quot;$FLUTTER_ROOT/packages/flutter_tools/bin/xcode_backend.sh&quot; prepare&#10;">';
    } else {
      scriptText =
          r'scriptText = "&quot;$FLUTTER_ROOT&quot;/packages/flutter_tools/bin/macos_assemble.sh prepare&#10;">';
    }

    String newContent = '''
         <ExecutionAction
            ActionType = "Xcode.IDEStandardExecutionActionsCore.ExecutionActionType.ShellScriptAction">
            <ActionContent
               title = "Run Prepare Flutter Framework Script"
               $scriptText
               <EnvironmentBuildable>
                  <BuildableReference
                     BuildableIdentifier = "primary"
                     BlueprintIdentifier = "$_runnerNativeTargetIdentifier"
                     $buildableName
                     $blueprintName
                     $referencedContainer
                  </BuildableReference>
               </EnvironmentBuildable>
            </ActionContent>
         </ExecutionAction>''';
    String newScheme = schemeContent;
    if (schemeContent.contains('PreActions')) {
      newScheme = schemeContent.replaceFirst('<PreActions>', '<PreActions>\n$newContent');
    } else {
      newContent = '''
      <PreActions>
$newContent
      </PreActions>
''';
      final String? buildActionEntries =
          schemeLines.where((String line) => line.contains('<BuildActionEntries>')).firstOrNull;
      if (buildActionEntries == null) {
        throw Exception(
          'Failed to parse ${schemeFile.basename}: Could not find BuildActionEntries.',
        );
      } else {
        newScheme = schemeContent.replaceFirst(
          buildActionEntries,
          '$newContent$buildActionEntries',
        );
      }
    }

    schemeFile.writeAsStringSync(newScheme);
    try {
      XmlDocument.parse(newScheme);
    } on XmlException catch (exception) {
      throw Exception(
        'Failed to parse ${schemeFile.basename}: Invalid xml: $newScheme\n$exception',
      );
    }
  }

  /// Parses the project.pbxproj into [ParsedProjectInfo]. Will throw an
  /// exception if it fails to parse.
  ParsedProjectInfo _parsePbxproj() {
    final String? results = _plistParser.plistJsonContent(_xcodeProjectInfoFile.path);
    if (results == null) {
      throw Exception('Failed to parse project settings.');
    }

    try {
      final Object decodeResult = json.decode(results) as Object;
      if (decodeResult is! Map<String, Object?>) {
        throw Exception('project.pbxproj returned unexpected JSON response: $results');
      }
      return ParsedProjectInfo.fromJson(decodeResult);
    } on FormatException {
      throw Exception('project.pbxproj returned non-JSON response: $results');
    }
  }

  /// Checks if all sections have been migrated. If [logErrorIfNotMigrated] is
  /// true, will log an error for each section that is not migrated.
  bool _isPbxprojMigratedCorrectly(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool buildFilesMigrated = _isBuildFilesMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    final bool frameworksBuildPhaseMigrated = _isFrameworksBuildPhaseMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    final bool nativeTargetsMigrated = _isNativeTargetMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    final bool projectObjectMigrated = _isProjectObjectMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    final bool localSwiftPackageMigrated = _isLocalSwiftPackageProductDependencyMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    final bool swiftPackageMigrated = _isSwiftPackageProductDependencyMigrated(
      projectInfo,
      logErrorIfNotMigrated: logErrorIfNotMigrated,
    );
    return buildFilesMigrated &&
        frameworksBuildPhaseMigrated &&
        nativeTargetsMigrated &&
        projectObjectMigrated &&
        localSwiftPackageMigrated &&
        swiftPackageMigrated;
  }

  void _migratePbxproj() {
    final String originalProjectContents = _xcodeProjectInfoFile.readAsStringSync();

    _ensureNewIdentifiersNotUsed(originalProjectContents);

    // Parse project.pbxproj into JSON
    final ParsedProjectInfo parsedInfo = _parsePbxproj();

    List<String> lines = LineSplitter.split(originalProjectContents).toList();
    lines = _migrateBuildFile(lines, parsedInfo);
    lines = _migrateFrameworksBuildPhase(lines, parsedInfo);
    lines = _migrateNativeTarget(lines, parsedInfo);
    lines = _migrateProjectObject(lines, parsedInfo);
    lines = _migrateLocalPackageProductDependencies(lines, parsedInfo);
    lines = _migratePackageProductDependencies(lines, parsedInfo);

    final String newProjectContents = '${lines.join('\n')}\n';

    if (originalProjectContents != newProjectContents) {
      logger.printTrace('Updating project settings...');
      _xcodeProjectInfoFile.copySync(backupProjectSettings.path);
      _xcodeProjectInfoFile.writeAsStringSync(newProjectContents);
    }
  }

  void _ensureNewIdentifiersNotUsed(String originalProjectContents) {
    if (originalProjectContents.contains(_flutterPluginsSwiftPackageBuildFileIdentifier)) {
      throw Exception('Duplicate id found for PBXBuildFile.');
    }
    if (originalProjectContents.contains(_flutterPluginsSwiftPackageProductDependencyIdentifier)) {
      throw Exception('Duplicate id found for XCSwiftPackageProductDependency.');
    }
    if (originalProjectContents.contains(_localFlutterPluginsSwiftPackageReferenceIdentifier)) {
      throw Exception('Duplicate id found for XCLocalSwiftPackageReference.');
    }
  }

  bool _isBuildFilesMigrated(ParsedProjectInfo projectInfo, {bool logErrorIfNotMigrated = false}) {
    final bool migrated = projectInfo.buildFileIdentifiers.contains(
      _flutterPluginsSwiftPackageBuildFileIdentifier,
    );
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError('PBXBuildFile was not migrated or was migrated incorrectly.');
    }
    return migrated;
  }

  List<String> _migrateBuildFile(List<String> lines, ParsedProjectInfo projectInfo) {
    if (_isBuildFilesMigrated(projectInfo)) {
      logger.printTrace('PBXBuildFile already migrated. Skipping...');
      return lines;
    }

    const String newContent =
        '		$_flutterPluginsSwiftPackageBuildFileIdentifier /* FlutterGeneratedPluginSwiftPackage in Frameworks */ = {isa = PBXBuildFile; productRef = $_flutterPluginsSwiftPackageProductDependencyIdentifier /* FlutterGeneratedPluginSwiftPackage */; };';

    final (int _, int endSectionIndex) = _sectionRange('PBXBuildFile', lines);

    lines.insert(endSectionIndex, newContent);
    return lines;
  }

  bool _isFrameworksBuildPhaseMigrated(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool migrated =
        projectInfo.frameworksBuildPhases
            .where(
              (ParsedProjectFrameworksBuildPhase phase) =>
                  phase.identifier == _runnerFrameworksBuildPhaseIdentifier &&
                  phase.files != null &&
                  phase.files!.contains(_flutterPluginsSwiftPackageBuildFileIdentifier),
            )
            .toList()
            .isNotEmpty;
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError('PBXFrameworksBuildPhase was not migrated or was migrated incorrectly.');
    }
    return migrated;
  }

  List<String> _migrateFrameworksBuildPhase(List<String> lines, ParsedProjectInfo projectInfo) {
    if (_isFrameworksBuildPhaseMigrated(projectInfo)) {
      logger.printTrace('PBXFrameworksBuildPhase already migrated. Skipping...');
      return lines;
    }

    final (int startSectionIndex, int endSectionIndex) = _sectionRange(
      'PBXFrameworksBuildPhase',
      lines,
    );

    // Find index where Frameworks Build Phase for the Runner target begins.
    final int runnerFrameworksPhaseStartIndex = lines.indexWhere(
      (String line) =>
          line.trim().startsWith('$_runnerFrameworksBuildPhaseIdentifier /* Frameworks */ = {'),
      startSectionIndex,
    );
    if (runnerFrameworksPhaseStartIndex == -1 ||
        runnerFrameworksPhaseStartIndex > endSectionIndex) {
      throw Exception(
        'Unable to find PBXFrameworksBuildPhase for ${_xcodeProject.hostAppProjectName} target.',
      );
    }

    // Get the Frameworks Build Phase for the Runner target from the parsed
    // project info.
    final ParsedProjectFrameworksBuildPhase? runnerFrameworksPhase =
        projectInfo.frameworksBuildPhases
            .where(
              (ParsedProjectFrameworksBuildPhase phase) =>
                  phase.identifier == _runnerFrameworksBuildPhaseIdentifier,
            )
            .toList()
            .firstOrNull;
    if (runnerFrameworksPhase == null) {
      throw Exception(
        'Unable to find parsed PBXFrameworksBuildPhase for ${_xcodeProject.hostAppProjectName} target.',
      );
    }

    if (runnerFrameworksPhase.files == null) {
      // If files is null, the files field is missing and must be added.
      const String newContent = '''
			files = (
				$_flutterPluginsSwiftPackageBuildFileIdentifier /* FlutterGeneratedPluginSwiftPackage in Frameworks */,
			);''';
      lines.insert(runnerFrameworksPhaseStartIndex + 1, newContent);
    } else {
      // Find the files field within the Frameworks PBXFrameworksBuildPhase for the Runner target.
      final int startFilesIndex = lines.indexWhere(
        (String line) => line.trim().contains('files = ('),
        runnerFrameworksPhaseStartIndex,
      );
      if (startFilesIndex == -1 || startFilesIndex > endSectionIndex) {
        throw Exception(
          'Unable to files for PBXFrameworksBuildPhase ${_xcodeProject.hostAppProjectName} target.',
        );
      }
      const String newContent =
          '				$_flutterPluginsSwiftPackageBuildFileIdentifier /* FlutterGeneratedPluginSwiftPackage in Frameworks */,';
      lines.insert(startFilesIndex + 1, newContent);
    }

    return lines;
  }

  bool _isNativeTargetMigrated(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool migrated =
        projectInfo.nativeTargets
            .where(
              (ParsedNativeTarget target) =>
                  target.identifier == _runnerNativeTargetIdentifier &&
                  target.packageProductDependencies != null &&
                  target.packageProductDependencies!.contains(
                    _flutterPluginsSwiftPackageProductDependencyIdentifier,
                  ),
            )
            .toList()
            .isNotEmpty;
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError('PBXNativeTarget was not migrated or was migrated incorrectly.');
    }
    return migrated;
  }

  List<String> _migrateNativeTarget(List<String> lines, ParsedProjectInfo projectInfo) {
    if (_isNativeTargetMigrated(projectInfo)) {
      logger.printTrace('PBXNativeTarget already migrated. Skipping...');
      return lines;
    }

    final (int startSectionIndex, int endSectionIndex) = _sectionRange('PBXNativeTarget', lines);

    // Find index where Native Target for the Runner target begins.
    final ParsedNativeTarget? runnerNativeTarget =
        projectInfo.nativeTargets
            .where(
              (ParsedNativeTarget target) => target.identifier == _runnerNativeTargetIdentifier,
            )
            .firstOrNull;
    if (runnerNativeTarget == null) {
      throw Exception(
        'Unable to find parsed PBXNativeTarget for ${_xcodeProject.hostAppProjectName} target.',
      );
    }
    final String subsectionLineStart =
        runnerNativeTarget.name != null
            ? '$_runnerNativeTargetIdentifier /* ${runnerNativeTarget.name} */ = {'
            : _runnerNativeTargetIdentifier;
    final int runnerNativeTargetStartIndex = lines.indexWhere(
      (String line) => line.trim().startsWith(subsectionLineStart),
      startSectionIndex,
    );
    if (runnerNativeTargetStartIndex == -1 || runnerNativeTargetStartIndex > endSectionIndex) {
      throw Exception(
        'Unable to find PBXNativeTarget for ${_xcodeProject.hostAppProjectName} target.',
      );
    }

    if (runnerNativeTarget.packageProductDependencies == null) {
      // If packageProductDependencies is null, the packageProductDependencies field is missing and must be added.
      const List<String> newContent = <String>[
        '			packageProductDependencies = (',
        '				$_flutterPluginsSwiftPackageProductDependencyIdentifier /* FlutterGeneratedPluginSwiftPackage */,',
        '			);',
      ];
      lines.insertAll(runnerNativeTargetStartIndex + 1, newContent);
    } else {
      // Find the packageProductDependencies field within the Native Target for the Runner target.
      final int packageProductDependenciesIndex = lines.indexWhere(
        (String line) => line.trim().contains('packageProductDependencies'),
        runnerNativeTargetStartIndex,
      );
      if (packageProductDependenciesIndex == -1 ||
          packageProductDependenciesIndex > endSectionIndex) {
        throw Exception(
          'Unable to find packageProductDependencies for ${_xcodeProject.hostAppProjectName} PBXNativeTarget.',
        );
      }
      const String newContent =
          '				$_flutterPluginsSwiftPackageProductDependencyIdentifier /* FlutterGeneratedPluginSwiftPackage */,';
      lines.insert(packageProductDependenciesIndex + 1, newContent);
    }
    return lines;
  }

  bool _isProjectObjectMigrated(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool migrated =
        projectInfo.projects
            .where(
              (ParsedProject target) =>
                  target.identifier == _projectIdentifier &&
                  target.packageReferences != null &&
                  target.packageReferences!.contains(
                    _localFlutterPluginsSwiftPackageReferenceIdentifier,
                  ),
            )
            .toList()
            .isNotEmpty;
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError('PBXProject was not migrated or was migrated incorrectly.');
    }
    return migrated;
  }

  List<String> _migrateProjectObject(List<String> lines, ParsedProjectInfo projectInfo) {
    if (_isProjectObjectMigrated(projectInfo)) {
      logger.printTrace('PBXProject already migrated. Skipping...');
      return lines;
    }

    final (int startSectionIndex, int endSectionIndex) = _sectionRange('PBXProject', lines);

    // Find index where Runner Project begins.
    final int projectStartIndex = lines.indexWhere(
      (String line) => line.trim().startsWith('$_projectIdentifier /* Project object */ = {'),
      startSectionIndex,
    );
    if (projectStartIndex == -1 || projectStartIndex > endSectionIndex) {
      throw Exception('Unable to find PBXProject for ${_xcodeProject.hostAppProjectName}.');
    }

    // Get the Runner project from the parsed project info.
    final ParsedProject? projectObject =
        projectInfo.projects
            .where((ParsedProject project) => project.identifier == _projectIdentifier)
            .toList()
            .firstOrNull;
    if (projectObject == null) {
      throw Exception('Unable to find parsed PBXProject for ${_xcodeProject.hostAppProjectName}.');
    }

    if (projectObject.packageReferences == null) {
      // If packageReferences is null, the packageReferences field is missing and must be added.
      const List<String> newContent = <String>[
        '			packageReferences = (',
        '				$_localFlutterPluginsSwiftPackageReferenceIdentifier /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */,',
        '			);',
      ];
      lines.insertAll(projectStartIndex + 1, newContent);
    } else {
      // Find the packageReferences field within the Runner project.
      final int packageReferencesIndex = lines.indexWhere(
        (String line) => line.trim().contains('packageReferences'),
        projectStartIndex,
      );
      if (packageReferencesIndex == -1 || packageReferencesIndex > endSectionIndex) {
        throw Exception(
          'Unable to find packageReferences for ${_xcodeProject.hostAppProjectName} PBXProject.',
        );
      }
      const String newContent =
          '				$_localFlutterPluginsSwiftPackageReferenceIdentifier /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */,';
      lines.insert(packageReferencesIndex + 1, newContent);
    }
    return lines;
  }

  bool _isLocalSwiftPackageProductDependencyMigrated(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool migrated = projectInfo.localSwiftPackageProductDependencies.contains(
      _localFlutterPluginsSwiftPackageReferenceIdentifier,
    );
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError(
        'XCLocalSwiftPackageReference was not migrated or was migrated incorrectly.',
      );
    }
    return migrated;
  }

  List<String> _migrateLocalPackageProductDependencies(
    List<String> lines,
    ParsedProjectInfo projectInfo,
  ) {
    if (_isLocalSwiftPackageProductDependencyMigrated(projectInfo)) {
      logger.printTrace('XCLocalSwiftPackageReference already migrated. Skipping...');
      return lines;
    }

    final (int startSectionIndex, int endSectionIndex) = _sectionRange(
      'XCLocalSwiftPackageReference',
      lines,
      throwIfMissing: false,
    );

    if (startSectionIndex == -1) {
      // There isn't a XCLocalSwiftPackageReference section yet, so add it
      final List<String> newContent = <String>[
        '/* Begin XCLocalSwiftPackageReference section */',
        '		$_localFlutterPluginsSwiftPackageReferenceIdentifier /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */ = {',
        '			isa = XCLocalSwiftPackageReference;',
        '			relativePath = Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage;',
        '		};',
        '/* End XCLocalSwiftPackageReference section */',
      ];

      final int index = lines.lastIndexWhere((String line) => line.trim().startsWith('/* End'));
      if (index == -1) {
        throw Exception('Unable to find any sections.');
      }
      lines.insertAll(index + 1, newContent);

      return lines;
    }

    final List<String> newContent = <String>[
      '		$_localFlutterPluginsSwiftPackageReferenceIdentifier /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */ = {',
      '			isa = XCLocalSwiftPackageReference;',
      '			relativePath = Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage;',
      '		};',
    ];

    lines.insertAll(endSectionIndex, newContent);

    return lines;
  }

  bool _isSwiftPackageProductDependencyMigrated(
    ParsedProjectInfo projectInfo, {
    bool logErrorIfNotMigrated = false,
  }) {
    final bool migrated = projectInfo.swiftPackageProductDependencies.contains(
      _flutterPluginsSwiftPackageProductDependencyIdentifier,
    );
    if (logErrorIfNotMigrated && !migrated) {
      logger.printError(
        'XCSwiftPackageProductDependency was not migrated or was migrated incorrectly.',
      );
    }
    return migrated;
  }

  List<String> _migratePackageProductDependencies(
    List<String> lines,
    ParsedProjectInfo projectInfo,
  ) {
    if (_isSwiftPackageProductDependencyMigrated(projectInfo)) {
      logger.printTrace('XCSwiftPackageProductDependency already migrated. Skipping...');
      return lines;
    }

    final (int startSectionIndex, int endSectionIndex) = _sectionRange(
      'XCSwiftPackageProductDependency',
      lines,
      throwIfMissing: false,
    );

    if (startSectionIndex == -1) {
      // There isn't a XCSwiftPackageProductDependency section yet, so add it
      final List<String> newContent = <String>[
        '/* Begin XCSwiftPackageProductDependency section */',
        '		$_flutterPluginsSwiftPackageProductDependencyIdentifier /* FlutterGeneratedPluginSwiftPackage */ = {',
        '			isa = XCSwiftPackageProductDependency;',
        '			productName = FlutterGeneratedPluginSwiftPackage;',
        '		};',
        '/* End XCSwiftPackageProductDependency section */',
      ];

      final int index = lines.lastIndexWhere((String line) => line.trim().startsWith('/* End'));
      if (index == -1) {
        throw Exception('Unable to find any sections.');
      }
      lines.insertAll(index + 1, newContent);

      return lines;
    }

    final List<String> newContent = <String>[
      '		$_flutterPluginsSwiftPackageProductDependencyIdentifier /* FlutterGeneratedPluginSwiftPackage */ = {',
      '			isa = XCSwiftPackageProductDependency;',
      '			productName = FlutterGeneratedPluginSwiftPackage;',
      '		};',
    ];

    lines.insertAll(endSectionIndex, newContent);

    return lines;
  }

  (int, int) _sectionRange(String sectionName, List<String> lines, {bool throwIfMissing = true}) {
    final int startSectionIndex = lines.indexOf('/* Begin $sectionName section */');
    if (throwIfMissing && startSectionIndex == -1) {
      throw Exception('Unable to find beginning of $sectionName section.');
    }
    final int endSectionIndex = lines.indexOf('/* End $sectionName section */');
    if (throwIfMissing && endSectionIndex == -1) {
      throw Exception('Unable to find end of $sectionName section.');
    }
    if (throwIfMissing && startSectionIndex > endSectionIndex) {
      throw Exception('Found the end of $sectionName section before the beginning.');
    }
    return (startSectionIndex, endSectionIndex);
  }
}

class SchemeInfo {
  SchemeInfo({required this.schemeName, required this.schemeFile, required this.schemeContent});

  final String schemeName;
  final File schemeFile;
  final String schemeContent;
  File? backupSchemeFile;
}

/// Representation of data parsed from Xcode project's project.pbxproj.
class ParsedProjectInfo {
  ParsedProjectInfo._({
    required this.buildFileIdentifiers,
    required this.fileReferenceIdentifiers,
    required this.parsedGroups,
    required this.frameworksBuildPhases,
    required this.nativeTargets,
    required this.projects,
    required this.swiftPackageProductDependencies,
    required this.localSwiftPackageProductDependencies,
  });

  factory ParsedProjectInfo.fromJson(Map<String, Object?> data) {
    final List<String> buildFiles = <String>[];
    final List<String> references = <String>[];
    final List<ParsedProjectGroup> groups = <ParsedProjectGroup>[];
    final List<ParsedProjectFrameworksBuildPhase> buildPhases =
        <ParsedProjectFrameworksBuildPhase>[];
    final List<ParsedNativeTarget> native = <ParsedNativeTarget>[];
    final List<ParsedProject> project = <ParsedProject>[];
    final List<String> parsedSwiftPackageProductDependencies = <String>[];
    final List<String> parsedLocalSwiftPackageProductDependencies = <String>[];

    if (data case {'objects': final Map<String, Object?> values}) {
      for (final String key in values.keys) {
        if (values[key] case final Map<String, Object?> details) {
          switch (details['isa']) {
            case 'PBXBuildFile':
              buildFiles.add(key);
            case 'PBXFileReference':
              references.add(key);
            case 'PBXGroup':
              groups.add(ParsedProjectGroup.fromJson(key, details));
            case 'PBXFrameworksBuildPhase':
              buildPhases.add(ParsedProjectFrameworksBuildPhase.fromJson(key, details));
            case 'PBXNativeTarget':
              native.add(ParsedNativeTarget.fromJson(key, details));
            case 'PBXProject':
              project.add(ParsedProject.fromJson(key, details));
            case 'XCSwiftPackageProductDependency':
              parsedSwiftPackageProductDependencies.add(key);
            case 'XCLocalSwiftPackageReference':
              parsedLocalSwiftPackageProductDependencies.add(key);
          }
        }
      }
    }

    return ParsedProjectInfo._(
      buildFileIdentifiers: buildFiles,
      fileReferenceIdentifiers: references,
      parsedGroups: groups,
      frameworksBuildPhases: buildPhases,
      nativeTargets: native,
      projects: project,
      swiftPackageProductDependencies: parsedSwiftPackageProductDependencies,
      localSwiftPackageProductDependencies: parsedLocalSwiftPackageProductDependencies,
    );
  }

  /// List of identifiers under PBXBuildFile section.
  List<String> buildFileIdentifiers;

  /// List of identifiers under PBXFileReference section.
  List<String> fileReferenceIdentifiers;

  /// List of [ParsedProjectGroup] items under PBXGroup section.
  List<ParsedProjectGroup> parsedGroups;

  /// List of [ParsedProjectFrameworksBuildPhase] items under PBXFrameworksBuildPhase section.
  List<ParsedProjectFrameworksBuildPhase> frameworksBuildPhases;

  /// List of [ParsedNativeTarget] items under PBXNativeTarget section.
  List<ParsedNativeTarget> nativeTargets;

  /// List of [ParsedProject] items under PBXProject section.
  List<ParsedProject> projects;

  /// List of identifiers under XCSwiftPackageProductDependency section.
  List<String> swiftPackageProductDependencies;

  /// List of identifiers under XCLocalSwiftPackageReference section.
  /// Introduced in Xcode 15.
  List<String> localSwiftPackageProductDependencies;
}

/// Representation of data parsed from PBXGroup section in Xcode project's project.pbxproj.
class ParsedProjectGroup {
  ParsedProjectGroup.fromJson(this.identifier, Map<String, Object?> data)
    : children = switch (data['children']) {
        final List<Object?> children => children.whereType<String>().toList(),
        _ => null,
      },
      name = switch (data) {
        {'name': final String name} => name,
        {'path': final String path} => path,
        _ => null,
      };

  final String identifier;
  final List<String>? children;
  final String? name;
}

/// Representation of data parsed from PBXFrameworksBuildPhase section in Xcode
/// project's project.pbxproj.
class ParsedProjectFrameworksBuildPhase {
  ParsedProjectFrameworksBuildPhase.fromJson(this.identifier, Map<String, Object?> data)
    : files = switch (data['files']) {
        final List<Object?> files => files.whereType<String>().toList(),
        _ => null,
      };

  final String identifier;
  final List<String>? files;
}

/// Representation of data parsed from PBXNativeTarget section in Xcode project's
/// project.pbxproj.
class ParsedNativeTarget {
  ParsedNativeTarget.fromJson(this.identifier, this.data)
    : name = switch (data) {
        {'name': final String name} => name,
        _ => null,
      },
      packageProductDependencies = switch (data['packageProductDependencies']) {
        final List<Object?> dependencies => dependencies.whereType<String>().toList(),
        _ => null,
      };

  final Map<String, Object?> data;
  final String identifier;
  final String? name;
  final List<String>? packageProductDependencies;
}

/// Representation of data parsed from PBXProject section in Xcode project's
/// project.pbxproj.
class ParsedProject {
  ParsedProject.fromJson(this.identifier, this.data)
    : packageReferences = switch (data['packageReferences']) {
        final List<Object?> references => references.whereType<String>().toList(),
        _ => null,
      };

  final Map<String, Object?> data;
  final String identifier;
  final List<String>? packageReferences;
}
