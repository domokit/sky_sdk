// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/utils.dart';
import 'base/version.dart';
import 'build_info.dart';
import 'bundle.dart' as bundle;
import 'convert.dart';
import 'features.dart';
import 'flutter_plugins.dart';
import 'globals.dart' as globals;
import 'ios/code_signing.dart';
import 'ios/plist_parser.dart';
import 'ios/xcode_build_settings.dart' as xcode;
import 'ios/xcodeproj.dart';
import 'macos/xcode.dart';
import 'platform_plugins.dart';
import 'project.dart';
import 'template.dart';

/// Represents an Xcode-based sub-project.
///
/// This defines interfaces common to iOS and macOS projects.
abstract class XcodeBasedProject extends FlutterProjectPlatform {
  static const String _defaultHostAppName = 'Runner';

  /// The Xcode workspace (.xcworkspace directory) of the host app.
  Directory? get xcodeWorkspace {
    if (!hostAppRoot.existsSync()) {
      return null;
    }
    return _xcodeDirectoryWithExtension('.xcworkspace');
  }

  /// The project name (.xcodeproj basename) of the host app.
  late final String hostAppProjectName = () {
    if (!hostAppRoot.existsSync()) {
      return _defaultHostAppName;
    }
    final Directory? xcodeProjectDirectory = _xcodeDirectoryWithExtension('.xcodeproj');
    return xcodeProjectDirectory != null
        ? xcodeProjectDirectory.fileSystem.path.basenameWithoutExtension(xcodeProjectDirectory.path)
        : _defaultHostAppName;
  }();

  Directory? _xcodeDirectoryWithExtension(String extension) {
    final List<FileSystemEntity> contents = hostAppRoot.listSync();
    for (final FileSystemEntity entity in contents) {
      if (globals.fs.path.extension(entity.path) == extension &&
          !globals.fs.path.basename(entity.path).startsWith('.')) {
        return hostAppRoot.childDirectory(entity.basename);
      }
    }
    return null;
  }

  /// The parent of this project.
  FlutterProject get parent;

  Directory get hostAppRoot;

  /// The default 'Info.plist' file of the host app. The developer can change this location in Xcode.
  File get defaultHostInfoPlist =>
      hostAppRoot.childDirectory(_defaultHostAppName).childFile('Info.plist');

  /// The Xcode project (.xcodeproj directory) of the host app.
  Directory get xcodeProject => hostAppRoot.childDirectory('$hostAppProjectName.xcodeproj');

  /// The 'project.pbxproj' file of [xcodeProject].
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  /// The 'Runner.xcscheme' file of [xcodeProject].
  File xcodeProjectSchemeFile({String? scheme}) {
    final String schemeName = scheme ?? 'Runner';
    return xcodeProject
        .childDirectory('xcshareddata')
        .childDirectory('xcschemes')
        .childFile('$schemeName.xcscheme');
  }

  File get xcodeProjectWorkspaceData =>
      xcodeProject.childDirectory('project.xcworkspace').childFile('contents.xcworkspacedata');

  /// Xcode workspace shared data directory for the host app.
  Directory? get xcodeWorkspaceSharedData => xcodeWorkspace?.childDirectory('xcshareddata');

  /// Xcode workspace shared workspace settings file for the host app.
  File? get xcodeWorkspaceSharedSettings =>
      xcodeWorkspaceSharedData?.childFile('WorkspaceSettings.xcsettings');

  /// Contains definitions for FLUTTER_ROOT, LOCAL_ENGINE, and more flags for
  /// the Xcode build.
  File get generatedXcodePropertiesFile;

  /// The Flutter-managed Xcode config file for [mode].
  File xcodeConfigFor(String mode);

  /// The script that exports environment variables needed for Flutter tools.
  /// Can be run first in a Xcode Script build phase to make FLUTTER_ROOT,
  /// LOCAL_ENGINE, and other Flutter variables available to any flutter
  /// tooling (`flutter build`, etc) to convert into flags.
  File get generatedEnvironmentVariableExportScript;

  /// The CocoaPods 'Podfile'.
  File get podfile => hostAppRoot.childFile('Podfile');

  /// The CocoaPods 'Podfile.lock'.
  File get podfileLock => hostAppRoot.childFile('Podfile.lock');

  /// The CocoaPods 'Manifest.lock'.
  File get podManifestLock => hostAppRoot.childDirectory('Pods').childFile('Manifest.lock');

  /// The CocoaPods generated 'Pods-Runner-frameworks.sh'.
  File get podRunnerFrameworksScript =>
      podRunnerTargetSupportFiles.childFile('Pods-Runner-frameworks.sh');

  /// The CocoaPods generated directory 'Pods-Runner'.
  Directory get podRunnerTargetSupportFiles => hostAppRoot
      .childDirectory('Pods')
      .childDirectory('Target Support Files')
      .childDirectory('Pods-Runner');

  /// The directory in the project that is managed by Flutter. As much as
  /// possible, files that are edited by Flutter tooling after initial project
  /// creation should live here.
  Directory get managedDirectory => hostAppRoot.childDirectory('Flutter');

  /// The subdirectory of [managedDirectory] that contains files that are
  /// generated on the fly. All generated files that are not intended to be
  /// checked in should live here.
  Directory get ephemeralDirectory => managedDirectory.childDirectory('ephemeral');

  /// The Flutter generated directory for the Swift Package handling plugin
  /// dependencies.
  Directory get flutterPluginSwiftPackageDirectory => ephemeralDirectory
      .childDirectory('Packages')
      .childDirectory('FlutterGeneratedPluginSwiftPackage');

  /// The Flutter generated Swift Package manifest (Package.swift) for plugin
  /// dependencies.
  File get flutterPluginSwiftPackageManifest =>
      flutterPluginSwiftPackageDirectory.childFile('Package.swift');

  /// Checks if FlutterGeneratedPluginSwiftPackage has been added to the
  /// project's build settings by checking the contents of the pbxproj.
  bool get flutterPluginSwiftPackageInProjectSettings {
    return xcodeProjectInfoFile.existsSync() &&
        xcodeProjectInfoFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage');
  }

  /// True if this project doesn't have Swift Package Manager disabled in the
  /// pubspec, has either an iOS or macOS platform implementation, is not a
  /// module project, Xcode is 15 or greater, and the Swift Package Manager
  /// feature is enabled.
  bool get usesSwiftPackageManager {
    if (!featureFlags.isSwiftPackageManagerEnabled) {
      return false;
    }

    // The project can disable Swift Package Manager in its pubspec.yaml.
    if (parent.manifest.disabledSwiftPackageManager) {
      return false;
    }

    // TODO(loic-sharma): Support Swift Package Manager in add-to-app modules.
    // https://github.com/flutter/flutter/issues/146957
    if (parent.isModule) {
      return false;
    }

    if (!existsSync()) {
      return false;
    }

    // Swift Package Manager requires Xcode 15 or greater.
    final Xcode? xcode = globals.xcode;
    final Version? xcodeVersion = xcode?.currentVersion;
    if (xcodeVersion == null || xcodeVersion.major < 15) {
      return false;
    }

    return true;
  }

  Future<XcodeProjectInfo?> projectInfo() async {
    final XcodeProjectInterpreter? xcodeProjectInterpreter = globals.xcodeProjectInterpreter;
    if (!xcodeProject.existsSync() ||
        xcodeProjectInterpreter == null ||
        !xcodeProjectInterpreter.isInstalled) {
      return null;
    }
    return _projectInfo ??= await xcodeProjectInterpreter.getInfo(hostAppRoot.path);
  }

  XcodeProjectInfo? _projectInfo;

  /// The build settings for the host app of this project, as a detached map.
  ///
  /// Returns null, if Xcode tooling is unavailable.
  Future<Map<String, String>?> buildSettingsForBuildInfo(
    BuildInfo? buildInfo, {
    String? scheme,
    String? configuration,
    String? target,
    EnvironmentType environmentType = EnvironmentType.physical,
    String? deviceId,
    bool isWatch = false,
  }) async {
    if (!existsSync()) {
      return null;
    }
    final XcodeProjectInfo? info = await projectInfo();
    if (info == null) {
      return null;
    }

    scheme ??= info.schemeFor(buildInfo);
    if (scheme == null) {
      info.reportFlavorNotFoundAndExit();
    }

    configuration ??= (await projectInfo())?.buildConfigurationFor(buildInfo, scheme);

    final XcodeSdk sdk = switch ((environmentType, this)) {
      (EnvironmentType.physical, _) when isWatch => XcodeSdk.WatchOS,
      (EnvironmentType.simulator, _) when isWatch => XcodeSdk.WatchSimulator,
      (EnvironmentType.physical, IosProject _) => XcodeSdk.IPhoneOS,
      (EnvironmentType.simulator, IosProject _) => XcodeSdk.IPhoneSimulator,
      (EnvironmentType.physical, MacOSProject _) => XcodeSdk.MacOSX,
      (_, _) => throw ArgumentError('Unsupported SDK'),
    };

    return _buildSettingsForXcodeProjectBuildContext(
      XcodeProjectBuildContext(
        scheme: scheme,
        configuration: configuration,
        sdk: sdk,
        target: target,
        deviceId: deviceId,
      ),
    );
  }

  Future<Map<String, String>?> _buildSettingsForXcodeProjectBuildContext(
    XcodeProjectBuildContext buildContext,
  ) async {
    if (!existsSync()) {
      return null;
    }
    final Map<String, String>? currentBuildSettings = _buildSettingsByBuildContext[buildContext];
    if (currentBuildSettings == null) {
      final Map<String, String>? calculatedBuildSettings = await _xcodeProjectBuildSettings(
        buildContext,
      );
      if (calculatedBuildSettings != null) {
        _buildSettingsByBuildContext[buildContext] = calculatedBuildSettings;
      }
    }
    return _buildSettingsByBuildContext[buildContext];
  }

  final Map<XcodeProjectBuildContext, Map<String, String>> _buildSettingsByBuildContext =
      <XcodeProjectBuildContext, Map<String, String>>{};

  Future<Map<String, String>?> _xcodeProjectBuildSettings(
    XcodeProjectBuildContext buildContext,
  ) async {
    final XcodeProjectInterpreter? xcodeProjectInterpreter = globals.xcodeProjectInterpreter;
    if (xcodeProjectInterpreter == null || !xcodeProjectInterpreter.isInstalled) {
      return null;
    }

    final Map<String, String> buildSettings = await xcodeProjectInterpreter.getBuildSettings(
      xcodeProject.path,
      buildContext: buildContext,
    );
    if (buildSettings.isNotEmpty) {
      // No timeouts, flakes, or errors.
      return buildSettings;
    }
    return null;
  }
}

/// Represents the iOS sub-project of a Flutter project.
///
/// Instances will reflect the contents of the `ios/` sub-folder of
/// Flutter applications and the `.ios/` sub-folder of Flutter module projects.
class IosProject extends XcodeBasedProject {
  IosProject.fromFlutter(this.parent);

  @override
  final FlutterProject parent;

  @override
  String get pluginConfigKey => IOSPlugin.kConfigKey;

  // build setting keys
  static const String kProductBundleIdKey = 'PRODUCT_BUNDLE_IDENTIFIER';
  static const String kTeamIdKey = 'DEVELOPMENT_TEAM';
  static const String kEntitlementFilePathKey = 'CODE_SIGN_ENTITLEMENTS';
  static const String kProductNameKey = 'PRODUCT_NAME';

  static final RegExp _productBundleIdPattern = RegExp(
    '^\\s*$kProductBundleIdKey\\s*=\\s*(["\']?)(.*?)\\1;\\s*\$',
  );
  static const String _kProductBundleIdVariable = '\$($kProductBundleIdKey)';

  // The string starts with `applinks:` and ignores the query param which starts with `?`.
  static final RegExp _associatedDomainPattern = RegExp(r'^applinks:([^?]+)');

  Directory get ephemeralModuleDirectory => parent.directory.childDirectory('.ios');
  Directory get _editableDirectory => parent.directory.childDirectory('ios');

  /// This parent folder of `Runner.xcodeproj`.
  @override
  Directory get hostAppRoot {
    if (!isModule || _editableDirectory.existsSync()) {
      return _editableDirectory;
    }
    return ephemeralModuleDirectory;
  }

  /// The root directory of the iOS wrapping of Flutter and plugins. This is the
  /// parent of the `Flutter/` folder into which Flutter artifacts are written
  /// during build.
  ///
  /// This is the same as [hostAppRoot] except when the project is
  /// a Flutter module with an editable host app.
  Directory get _flutterLibRoot => isModule ? ephemeralModuleDirectory : _editableDirectory;

  /// True, if the parent Flutter project is a module project.
  bool get isModule => parent.isModule;

  /// Whether the Flutter application has an iOS project.
  bool get exists => hostAppRoot.existsSync();

  @override
  Directory get managedDirectory => _flutterLibRoot.childDirectory('Flutter');

  @override
  File xcodeConfigFor(String mode) => managedDirectory.childFile('$mode.xcconfig');

  @override
  File get generatedEnvironmentVariableExportScript =>
      managedDirectory.childFile('flutter_export_environment.sh');

  File get appFrameworkInfoPlist => managedDirectory.childFile('AppFrameworkInfo.plist');

  /// The 'AppDelegate.swift' file of the host app. This file might not exist if the app project uses Objective-C.
  File get appDelegateSwift =>
      _editableDirectory.childDirectory('Runner').childFile('AppDelegate.swift');

  File get infoPlist => _editableDirectory.childDirectory('Runner').childFile('Info.plist');

  Directory get symlinks => _flutterLibRoot.childDirectory('.symlinks');

  /// True if the app project uses Swift.
  bool get isSwift => appDelegateSwift.existsSync();

  /// Do all plugins support arm64 simulators to run natively on an ARM Mac?
  Future<bool> pluginsSupportArmSimulator() async {
    final Directory podXcodeProject = hostAppRoot
        .childDirectory('Pods')
        .childDirectory('Pods.xcodeproj');
    if (!podXcodeProject.existsSync()) {
      // No plugins.
      return true;
    }

    final XcodeProjectInterpreter? xcodeProjectInterpreter = globals.xcodeProjectInterpreter;
    if (xcodeProjectInterpreter == null) {
      // Xcode isn't installed, don't try to check.
      return false;
    }
    final String? buildSettings = await xcodeProjectInterpreter.pluginsBuildSettingsOutput(
      podXcodeProject,
    );

    // See if any plugins or their dependencies exclude arm64 simulators
    // as a valid architecture, usually because a binary is missing that slice.
    // Example: EXCLUDED_ARCHS = arm64 i386
    // NOT: EXCLUDED_ARCHS = i386
    return buildSettings != null && !buildSettings.contains(RegExp('EXCLUDED_ARCHS.*arm64'));
  }

  @override
  bool existsSync() {
    return parent.isModule || _editableDirectory.existsSync();
  }

  /// Outputs universal link related project settings of the iOS sub-project into
  /// a json file.
  ///
  /// The return future will resolve to string path to the output file.
  Future<String> outputsUniversalLinkSettings({
    required String configuration,
    required String target,
  }) async {
    final XcodeProjectBuildContext context = XcodeProjectBuildContext(
      configuration: configuration,
      target: target,
    );
    final File file = await parent.buildDirectory
        .childDirectory('deeplink_data')
        .childFile('universal-link-settings-$configuration-$target.json')
        .create(recursive: true);

    await file.writeAsString(
      jsonEncode(<String, Object?>{
        'bundleIdentifier': await _productBundleIdentifierWithBuildContext(context),
        'teamIdentifier': await _getTeamIdentifier(context),
        'associatedDomains': await _getAssociatedDomains(context),
      }),
    );
    return file.absolute.path;
  }

  /// The product bundle identifier of the host app, or null if not set or if
  /// iOS tooling needed to read it is not installed.
  Future<String?> productBundleIdentifier(BuildInfo? buildInfo) async {
    if (!existsSync()) {
      return null;
    }

    XcodeProjectBuildContext? buildContext;
    final XcodeProjectInfo? info = await projectInfo();
    if (info != null) {
      final String? scheme = info.schemeFor(buildInfo);
      if (scheme == null) {
        info.reportFlavorNotFoundAndExit();
      }
      final String? configuration = info.buildConfigurationFor(buildInfo, scheme);
      buildContext = XcodeProjectBuildContext(configuration: configuration, scheme: scheme);
    }
    return _productBundleIdentifierWithBuildContext(buildContext);
  }

  Future<String?> _productBundleIdentifierWithBuildContext(
    XcodeProjectBuildContext? buildContext,
  ) async {
    if (!existsSync()) {
      return null;
    }
    if (_productBundleIdentifiers.containsKey(buildContext)) {
      return _productBundleIdentifiers[buildContext];
    }
    return _productBundleIdentifiers[buildContext] = await _parseProductBundleIdentifier(
      buildContext,
    );
  }

  final Map<XcodeProjectBuildContext?, String?> _productBundleIdentifiers =
      <XcodeProjectBuildContext?, String?>{};

  Future<String?> _parseProductBundleIdentifier(XcodeProjectBuildContext? buildContext) async {
    String? fromPlist;
    final File defaultInfoPlist = defaultHostInfoPlist;
    // Users can change the location of the Info.plist.
    // Try parsing the default, first.
    if (defaultInfoPlist.existsSync()) {
      try {
        fromPlist = globals.plistParser.getValueFromFile<String>(
          defaultHostInfoPlist.path,
          PlistParser.kCFBundleIdentifierKey,
        );
      } on FileNotFoundException {
        // iOS tooling not found; likely not running OSX; let [fromPlist] be null
      }
      if (fromPlist != null && !fromPlist.contains(r'$')) {
        // Info.plist has no build variables in product bundle ID.
        return fromPlist;
      }
    }
    if (buildContext == null) {
      // Getting build settings to evaluate info.Plist requires a context.
      return null;
    }

    final Map<String, String>? allBuildSettings = await _buildSettingsForXcodeProjectBuildContext(
      buildContext,
    );
    if (allBuildSettings != null) {
      if (fromPlist != null) {
        // Perform variable substitution using build settings.
        return substituteXcodeVariables(fromPlist, allBuildSettings);
      }
      return allBuildSettings[kProductBundleIdKey];
    }

    // On non-macOS platforms, parse the first PRODUCT_BUNDLE_IDENTIFIER from
    // the project file. This can return the wrong bundle identifier if additional
    // bundles have been added to the project and are found first, like frameworks
    // or companion watchOS projects. However, on non-macOS platforms this is
    // only used for display purposes and to regenerate organization names, so
    // best-effort is probably fine.
    final String? fromPbxproj = firstMatchInFile(
      xcodeProjectInfoFile,
      _productBundleIdPattern,
    )?.group(2);
    if (fromPbxproj != null && (fromPlist == null || fromPlist == _kProductBundleIdVariable)) {
      return fromPbxproj;
    }
    return null;
  }

  Future<String?> _getTeamIdentifier(XcodeProjectBuildContext buildContext) async {
    final Map<String, String>? buildSettings = await _buildSettingsForXcodeProjectBuildContext(
      buildContext,
    );
    return buildSettings?[kTeamIdKey];
  }

  Future<List<String>> _getAssociatedDomains(XcodeProjectBuildContext buildContext) async {
    final Map<String, String>? buildSettings = await _buildSettingsForXcodeProjectBuildContext(
      buildContext,
    );
    if (buildSettings != null) {
      final String? entitlementPath = buildSettings[kEntitlementFilePathKey];
      if (entitlementPath != null) {
        final File entitlement = hostAppRoot.childFile(entitlementPath);
        if (entitlement.existsSync()) {
          final List<String>? domains =
              globals.plistParser
                  .getValueFromFile<List<Object>>(
                    entitlement.path,
                    PlistParser.kAssociatedDomainsKey,
                  )
                  ?.cast<String>();

          if (domains != null) {
            return <String>[
              for (final String domain in domains)
                if (_associatedDomainPattern.firstMatch(domain) case final RegExpMatch match)
                  match.group(1)!,
            ];
          }
        }
      }
    }
    return const <String>[];
  }

  /// The product name of the app, `My App`.
  Future<String?> productName(BuildInfo? buildInfo) async {
    if (!existsSync()) {
      return null;
    }
    return _productName ??= await _parseProductName(buildInfo);
  }

  String? _productName;

  Future<String> _parseProductName(BuildInfo? buildInfo) async {
    // The product name and bundle name are derived from the display name, which the user
    // is instructed to change in Xcode as part of deploying to the App Store.
    // https://flutter.dev/to/xcode-name-config
    // The only source of truth for the name is Xcode's interpretation of the build settings.
    String? productName;
    if (globals.xcodeProjectInterpreter?.isInstalled ?? false) {
      final Map<String, String>? xcodeBuildSettings = await buildSettingsForBuildInfo(buildInfo);
      if (xcodeBuildSettings != null) {
        productName = xcodeBuildSettings[kProductNameKey];
      }
    }
    if (productName == null) {
      globals.printTrace('$kProductNameKey not present, defaulting to $hostAppProjectName');
    }
    return productName ?? XcodeBasedProject._defaultHostAppName;
  }

  Future<void> ensureReadyForPlatformSpecificTooling() async {
    await _regenerateFromTemplateIfNeeded();
    if (!_flutterLibRoot.existsSync()) {
      return;
    }
    await _updateGeneratedXcodeConfigIfNeeded();
  }

  /// Check if one the [targets] of the project is a watchOS companion app target.
  Future<bool> containsWatchCompanion({
    required XcodeProjectInfo projectInfo,
    required BuildInfo buildInfo,
    String? deviceId,
  }) async {
    final String? bundleIdentifier = await productBundleIdentifier(buildInfo);
    // A bundle identifier is required for a companion app.
    if (bundleIdentifier == null) {
      return false;
    }
    for (final String target in projectInfo.targets) {
      // Create Info.plist file of the target.
      final File infoFile = hostAppRoot.childDirectory(target).childFile('Info.plist');
      // In older versions of Xcode, if the target was a watchOS companion app,
      // the Info.plist file of the target contained the key WKCompanionAppBundleIdentifier.
      if (infoFile.existsSync()) {
        final String? fromPlist = globals.plistParser.getValueFromFile<String>(
          infoFile.path,
          'WKCompanionAppBundleIdentifier',
        );
        if (bundleIdentifier == fromPlist) {
          return true;
        }

        // The key WKCompanionAppBundleIdentifier might contain an xcode variable
        // that needs to be substituted before comparing it with bundle id
        if (fromPlist != null && fromPlist.contains(r'$')) {
          final Map<String, String>? allBuildSettings = await buildSettingsForBuildInfo(
            buildInfo,
            deviceId: deviceId,
          );
          if (allBuildSettings != null) {
            final String substitutedVariable = substituteXcodeVariables(
              fromPlist,
              allBuildSettings,
            );
            if (substitutedVariable == bundleIdentifier) {
              return true;
            }
          }
        }
      }
    }

    // If key not found in Info.plist above, do more expensive check of build settings.
    // In newer versions of Xcode, the build settings of the watchOS companion
    // app's scheme should contain the key INFOPLIST_KEY_WKCompanionAppBundleIdentifier.
    final bool watchIdentifierFound = xcodeProjectInfoFile.readAsStringSync().contains(
      'WKCompanionAppBundleIdentifier',
    );
    if (!watchIdentifierFound) {
      return false;
    }

    final String? defaultScheme = projectInfo.schemeFor(buildInfo);
    if (defaultScheme == null) {
      projectInfo.reportFlavorNotFoundAndExit();
    }
    for (final String scheme in projectInfo.schemes) {
      // the default scheme should not be a watch scheme, so skip it
      if (scheme == defaultScheme) {
        continue;
      }
      final Map<String, String>? allBuildSettings = await buildSettingsForBuildInfo(
        buildInfo,
        deviceId: deviceId,
        scheme: scheme,
        isWatch: true,
      );
      if (allBuildSettings != null) {
        final String? fromBuild = allBuildSettings['INFOPLIST_KEY_WKCompanionAppBundleIdentifier'];
        if (bundleIdentifier == fromBuild) {
          return true;
        }
        if (fromBuild != null && fromBuild.contains(r'$')) {
          final String substitutedVariable = substituteXcodeVariables(fromBuild, allBuildSettings);
          if (substitutedVariable == bundleIdentifier) {
            return true;
          }
        }
      }
    }
    return false;
  }

  Future<void> _updateGeneratedXcodeConfigIfNeeded() async {
    if (globals.cache.isOlderThanToolsStamp(generatedXcodePropertiesFile)) {
      await xcode.updateGeneratedXcodeProperties(
        project: parent,
        buildInfo: BuildInfo.dummy,
        targetOverride: bundle.defaultMainPath,
      );
    }
  }

  Future<void> _regenerateFromTemplateIfNeeded() async {
    if (!isModule) {
      return;
    }
    final bool pubspecChanged = globals.fsUtils.isOlderThanReference(
      entity: ephemeralModuleDirectory,
      referenceFile: parent.pubspecFile,
    );
    final bool toolingChanged = globals.cache.isOlderThanToolsStamp(ephemeralModuleDirectory);
    if (!pubspecChanged && !toolingChanged) {
      return;
    }

    ErrorHandlingFileSystem.deleteIfExists(ephemeralModuleDirectory, recursive: true);
    await _overwriteFromTemplate(
      globals.fs.path.join('module', 'ios', 'library'),
      ephemeralModuleDirectory,
    );
    // Add ephemeral host app, if a editable host app does not already exist.
    if (!_editableDirectory.existsSync()) {
      await _overwriteFromTemplate(
        globals.fs.path.join('module', 'ios', 'host_app_ephemeral'),
        ephemeralModuleDirectory,
      );
      if (hasPlugins(parent)) {
        await _overwriteFromTemplate(
          globals.fs.path.join('module', 'ios', 'host_app_ephemeral_cocoapods'),
          ephemeralModuleDirectory,
        );
      }
    }
  }

  @override
  File get generatedXcodePropertiesFile =>
      _flutterLibRoot.childDirectory('Flutter').childFile('Generated.xcconfig');

  /// No longer compiled to this location.
  ///
  /// Used only for "flutter clean" to remove old references.
  Directory get deprecatedCompiledDartFramework =>
      _flutterLibRoot.childDirectory('Flutter').childDirectory('App.framework');

  /// No longer copied to this location.
  ///
  /// Used only for "flutter clean" to remove old references.
  Directory get deprecatedProjectFlutterFramework =>
      _flutterLibRoot.childDirectory('Flutter').childDirectory('Flutter.framework');

  /// Used only for "flutter clean" to remove old references.
  File get flutterPodspec => _flutterLibRoot.childDirectory('Flutter').childFile('Flutter.podspec');

  Directory get pluginRegistrantHost {
    return isModule
        ? _flutterLibRoot.childDirectory('Flutter').childDirectory('FlutterPluginRegistrant')
        : hostAppRoot.childDirectory(XcodeBasedProject._defaultHostAppName);
  }

  File get pluginRegistrantHeader {
    final Directory registryDirectory =
        isModule ? pluginRegistrantHost.childDirectory('Classes') : pluginRegistrantHost;
    return registryDirectory.childFile('GeneratedPluginRegistrant.h');
  }

  File get pluginRegistrantImplementation {
    final Directory registryDirectory =
        isModule ? pluginRegistrantHost.childDirectory('Classes') : pluginRegistrantHost;
    return registryDirectory.childFile('GeneratedPluginRegistrant.m');
  }

  Future<void> _overwriteFromTemplate(String path, Directory target) async {
    final Template template = await Template.fromName(
      path,
      fileSystem: globals.fs,
      templateManifest: null,
      logger: globals.logger,
      templateRenderer: globals.templateRenderer,
    );
    final String iosBundleIdentifier =
        parent.manifest.iosBundleIdentifier ?? 'com.example.${parent.manifest.appName}';

    final String? iosDevelopmentTeam = await getCodeSigningIdentityDevelopmentTeam(
      processManager: globals.processManager,
      platform: globals.platform,
      logger: globals.logger,
      config: globals.config,
      terminal: globals.terminal,
    );

    final String projectName = parent.manifest.appName;

    // The dart project_name is in snake_case, this variable is the Title Case of the Project Name.
    final String titleCaseProjectName = snakeCaseToTitleCase(projectName);

    template.render(target, <String, Object>{
      'ios': true,
      'projectName': projectName,
      'titleCaseProjectName': titleCaseProjectName,
      'iosIdentifier': iosBundleIdentifier,
      'hasIosDevelopmentTeam': iosDevelopmentTeam != null && iosDevelopmentTeam.isNotEmpty,
      'iosDevelopmentTeam': iosDevelopmentTeam ?? '',
    }, printStatusWhenWriting: false);
  }
}

/// The macOS sub project.
class MacOSProject extends XcodeBasedProject {
  MacOSProject.fromFlutter(this.parent);

  @override
  final FlutterProject parent;

  @override
  String get pluginConfigKey => MacOSPlugin.kConfigKey;

  @override
  bool existsSync() => hostAppRoot.existsSync();

  @override
  Directory get hostAppRoot => parent.directory.childDirectory('macos');

  /// The xcfilelist used to track the inputs for the Flutter script phase in
  /// the Xcode build.
  File get inputFileList => ephemeralDirectory.childFile('FlutterInputs.xcfilelist');

  /// The xcfilelist used to track the outputs for the Flutter script phase in
  /// the Xcode build.
  File get outputFileList => ephemeralDirectory.childFile('FlutterOutputs.xcfilelist');

  @override
  File get generatedXcodePropertiesFile =>
      ephemeralDirectory.childFile('Flutter-Generated.xcconfig');

  File get pluginRegistrantImplementation =>
      managedDirectory.childFile('GeneratedPluginRegistrant.swift');

  /// The 'AppDelegate.swift' file of the host app. This file might not exist if the app project uses Objective-C.
  File get appDelegateSwift => hostAppRoot.childDirectory('Runner').childFile('AppDelegate.swift');

  @override
  File xcodeConfigFor(String mode) => managedDirectory.childFile('Flutter-$mode.xcconfig');

  @override
  File get generatedEnvironmentVariableExportScript =>
      ephemeralDirectory.childFile('flutter_export_environment.sh');

  /// The file where the Xcode build will write the name of the built app.
  ///
  /// Ideally this will be replaced in the future with inspection of the Runner
  /// scheme's target.
  File get nameFile => ephemeralDirectory.childFile('.app_filename');

  Future<void> ensureReadyForPlatformSpecificTooling() async {
    // TODO(stuartmorgan): Add create-from-template logic here.
    await _updateGeneratedXcodeConfigIfNeeded();
  }

  Future<void> _updateGeneratedXcodeConfigIfNeeded() async {
    if (globals.cache.isOlderThanToolsStamp(generatedXcodePropertiesFile)) {
      await xcode.updateGeneratedXcodeProperties(
        project: parent,
        buildInfo: BuildInfo.dummy,
        useMacOSConfig: true,
      );
    }
  }
}
