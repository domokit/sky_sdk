// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:xml/xml.dart';
import 'package:yaml/yaml.dart';
import 'package:yaml_edit/yaml_edit.dart';

import '../src/convert.dart';
import 'android/android_builder.dart';
import 'android/gradle_utils.dart' as gradle;
import 'base/common.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'base/version.dart';
import 'bundle.dart' as bundle;
import 'cmake_project.dart';
import 'dart/package_map.dart';
import 'features.dart';
import 'flutter_manifest.dart';
import 'flutter_plugins.dart';
import 'globals.dart' as globals;
import 'platform_plugins.dart';
import 'project_validator_result.dart';
import 'template.dart';
import 'xcode_project.dart';

export 'cmake_project.dart';
export 'xcode_project.dart';

/// Enum for each officially supported platform.
enum SupportedPlatform {
  android(name: 'android'),
  ios(name: 'ios'),
  linux(name: 'linux'),
  macos(name: 'macos'),
  web(name: 'web'),
  windows(name: 'windows'),
  fuchsia(name: 'fuchsia'),
  root(name: 'root'); // Special platform to represent the root project directory

  const SupportedPlatform({required this.name});

  final String name;
}

class FlutterProjectFactory {
  FlutterProjectFactory({required Logger logger, required FileSystem fileSystem})
    : _logger = logger,
      _fileSystem = fileSystem;

  final Logger _logger;
  final FileSystem _fileSystem;

  @visibleForTesting
  final Map<String, FlutterProject> projects = <String, FlutterProject>{};

  /// Returns a [FlutterProject] view of the given directory or a ToolExit error,
  /// if `pubspec.yaml` or `example/pubspec.yaml` is invalid.
  FlutterProject fromDirectory(Directory directory) {
    return projects.putIfAbsent(directory.path, () {
      final FlutterManifest manifest = FlutterProject._readManifest(
        directory.childFile(bundle.defaultManifestPath).path,
        logger: _logger,
        fileSystem: _fileSystem,
      );
      final FlutterManifest exampleManifest = FlutterProject._readManifest(
        FlutterProject._exampleDirectory(directory).childFile(bundle.defaultManifestPath).path,
        logger: _logger,
        fileSystem: _fileSystem,
      );
      return FlutterProject(directory, manifest, exampleManifest);
    });
  }
}

/// Represents the contents of a Flutter project at the specified [directory].
///
/// [FlutterManifest] information is read from `pubspec.yaml` and
/// `example/pubspec.yaml` files on construction of a [FlutterProject] instance.
/// The constructed instance carries an immutable snapshot representation of the
/// presence and content of those files. Accordingly, [FlutterProject] instances
/// should be discarded upon changes to the `pubspec.yaml` files, but can be
/// used across changes to other files, as no other file-level information is
/// cached.
class FlutterProject {
  @visibleForTesting
  FlutterProject(this.directory, this._manifest, this._exampleManifest);

  /// Returns a [FlutterProject] view of the given directory or a ToolExit error,
  /// if `pubspec.yaml` or `example/pubspec.yaml` is invalid.
  static FlutterProject fromDirectory(Directory directory) =>
      globals.projectFactory.fromDirectory(directory);

  /// Returns a [FlutterProject] view of the current directory or a ToolExit error,
  /// if `pubspec.yaml` or `example/pubspec.yaml` is invalid.
  static FlutterProject current() =>
      globals.projectFactory.fromDirectory(globals.fs.currentDirectory);

  /// Create a [FlutterProject] and bypass the project caching.
  @visibleForTesting
  static FlutterProject fromDirectoryTest(Directory directory, [Logger? logger]) {
    final FileSystem fileSystem = directory.fileSystem;
    logger ??= BufferLogger.test();
    final FlutterManifest manifest = FlutterProject._readManifest(
      directory.childFile(bundle.defaultManifestPath).path,
      logger: logger,
      fileSystem: fileSystem,
    );
    final FlutterManifest exampleManifest = FlutterProject._readManifest(
      FlutterProject._exampleDirectory(directory).childFile(bundle.defaultManifestPath).path,
      logger: logger,
      fileSystem: fileSystem,
    );
    return FlutterProject(directory, manifest, exampleManifest);
  }

  /// The location of this project.
  final Directory directory;

  /// The location of the build folder.
  Directory get buildDirectory => directory.childDirectory('build');

  /// The manifest of this project.
  FlutterManifest get manifest => _manifest;
  late FlutterManifest _manifest;

  /// The manifest of the example sub-project of this project.
  final FlutterManifest _exampleManifest;

  /// The set of organization names found in this project as
  /// part of iOS product bundle identifier, Android application ID, or
  /// Gradle group ID.
  Future<Set<String>> get organizationNames async {
    final List<String> candidates = <String>[];

    if (ios.existsSync()) {
      // Don't require iOS build info, this method is only
      // used during create as best-effort, use the
      // default target bundle identifier.
      try {
        final String? bundleIdentifier = await ios.productBundleIdentifier(null);
        if (bundleIdentifier != null) {
          candidates.add(bundleIdentifier);
        }
      } on ToolExit {
        // It's possible that while parsing the build info for the ios project
        // that the bundleIdentifier can't be resolve. However, we would like
        // skip parsing that id in favor of searching in other place. We can
        // consider a tool exit in this case to be non fatal for the program.
      }
    }
    if (android.existsSync()) {
      final String? applicationId = android.applicationId;
      final String? group = android.group;
      candidates.addAll(<String>[
        if (applicationId != null) applicationId,
        if (group != null) group,
      ]);
    }
    if (example.android.existsSync()) {
      final String? applicationId = example.android.applicationId;
      if (applicationId != null) {
        candidates.add(applicationId);
      }
    }
    if (example.ios.existsSync()) {
      final String? bundleIdentifier = await example.ios.productBundleIdentifier(null);
      if (bundleIdentifier != null) {
        candidates.add(bundleIdentifier);
      }
    }
    return Set<String>.of(
      candidates.map<String?>(_organizationNameFromPackageName).whereType<String>(),
    );
  }

  String? _organizationNameFromPackageName(String packageName) {
    if (0 <= packageName.lastIndexOf('.')) {
      return packageName.substring(0, packageName.lastIndexOf('.'));
    }
    return null;
  }

  /// The iOS sub project of this project.
  late final IosProject ios = IosProject.fromFlutter(this);

  /// The Android sub project of this project.
  late final AndroidProject android = AndroidProject._(this);

  /// The web sub project of this project.
  late final WebProject web = WebProject._(this);

  /// The MacOS sub project of this project.
  late final MacOSProject macos = MacOSProject.fromFlutter(this);

  /// The Linux sub project of this project.
  late final LinuxProject linux = LinuxProject.fromFlutter(this);

  /// The Windows sub project of this project.
  late final WindowsProject windows = WindowsProject.fromFlutter(this);

  /// The Fuchsia sub project of this project.
  late final FuchsiaProject fuchsia = FuchsiaProject._(this);

  /// The `pubspec.yaml` file of this project.
  File get pubspecFile => directory.childFile('pubspec.yaml');

  /// The `.metadata` file of this project.
  File get metadataFile => directory.childFile('.metadata');

  /// The `.flutter-plugins` file of this project.
  File get flutterPluginsFile => directory.childFile('.flutter-plugins');

  /// The `.flutter-plugins-dependencies` file of this project,
  /// which contains the dependencies each plugin depends on.
  File get flutterPluginsDependenciesFile => directory.childFile('.flutter-plugins-dependencies');

  /// The `.gitignore` file of this project.
  File get gitignoreFile => directory.childFile('.gitignore');

  File get packageConfig => findPackageConfigFileOrDefault(directory);

  /// The `.dart-tool` directory of this project.
  Directory get dartTool => directory.childDirectory('.dart_tool');

  /// The location of the generated scaffolding project for hosting widget
  /// previews from this project.
  Directory get widgetPreviewScaffold => dartTool.childDirectory('widget_preview_scaffold');

  /// The directory containing the generated code for this project.
  Directory get generated => directory.absolute
      .childDirectory('.dart_tool')
      .childDirectory('build')
      .childDirectory('generated')
      .childDirectory(manifest.appName);

  /// The generated Dart plugin registrant for non-web platforms.
  File get dartPluginRegistrant =>
      dartTool.childDirectory('flutter_build').childFile('dart_plugin_registrant.dart');

  /// The example sub-project of this project.
  FlutterProject get example => FlutterProject(
    _exampleDirectory(directory),
    _exampleManifest,
    FlutterManifest.empty(logger: globals.logger),
  );

  /// The generated scaffolding project for hosting widget previews from this
  /// project.
  late final FlutterProject widgetPreviewScaffoldProject = FlutterProject.fromDirectory(
    widgetPreviewScaffold,
  );

  /// True if this project is a Flutter module project.
  bool get isModule => manifest.isModule;

  /// True if this project is a Flutter plugin project.
  bool get isPlugin => manifest.isPlugin;

  /// True if the Flutter project is using the AndroidX support library.
  bool get usesAndroidX => manifest.usesAndroidX;

  /// True if this project has an example application.
  bool get hasExampleApp => _exampleDirectory(directory).existsSync();

  /// Returns a list of platform names that are supported by the project.
  List<SupportedPlatform> getSupportedPlatforms({bool includeRoot = false}) {
    return <SupportedPlatform>[
      if (includeRoot) SupportedPlatform.root,
      if (android.existsSync()) SupportedPlatform.android,
      if (ios.exists) SupportedPlatform.ios,
      if (web.existsSync()) SupportedPlatform.web,
      if (macos.existsSync()) SupportedPlatform.macos,
      if (linux.existsSync()) SupportedPlatform.linux,
      if (windows.existsSync()) SupportedPlatform.windows,
      if (fuchsia.existsSync()) SupportedPlatform.fuchsia,
    ];
  }

  /// The directory that will contain the example if an example exists.
  static Directory _exampleDirectory(Directory directory) => directory.childDirectory('example');

  /// Reads and validates the `pubspec.yaml` file at [path], asynchronously
  /// returning a [FlutterManifest] representation of the contents.
  ///
  /// Completes with an empty [FlutterManifest], if the file does not exist.
  /// Completes with a ToolExit on validation error.
  static FlutterManifest _readManifest(
    String path, {
    required Logger logger,
    required FileSystem fileSystem,
  }) {
    FlutterManifest? manifest;
    try {
      manifest = FlutterManifest.createFromPath(path, logger: logger, fileSystem: fileSystem);
    } on YamlException catch (e) {
      logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
      logger.printError('$e');
    } on FormatException catch (e) {
      logger.printError('Error detected while parsing pubspec.yaml:', emphasis: true);
      logger.printError('$e');
    } on FileSystemException catch (e) {
      logger.printError('Error detected while reading pubspec.yaml:', emphasis: true);
      logger.printError('$e');
    }
    if (manifest == null) {
      throwToolExit('Please correct the pubspec.yaml file at $path');
    }
    return manifest;
  }

  /// Replaces the content of [pubspecFile] with the contents of [updated] and
  /// sets [manifest] to the [updated] manifest.
  void replacePubspec(FlutterManifest updated) {
    final YamlMap updatedPubspecContents = updated.toYaml();
    final YamlEditor editor = YamlEditor('');
    editor.update(const <String>[], updatedPubspecContents);
    pubspecFile.writeAsStringSync(editor.toString());
    _manifest = updated;
  }

  /// Reapplies template files and regenerates project files and plugin
  /// registrants for app and module projects only.
  ///
  /// Will not create project platform directories if they do not already exist.
  ///
  /// If [allowedPlugins] is non-null, all plugins with method channels in the
  /// project's pubspec.yaml will be validated to be in that set, or else a
  /// [ToolExit] will be thrown.
  Future<void> regeneratePlatformSpecificTooling({
    DeprecationBehavior deprecationBehavior = DeprecationBehavior.none,
    Iterable<String>? allowedPlugins,
  }) async {
    return ensureReadyForPlatformSpecificTooling(
      androidPlatform: android.existsSync(),
      iosPlatform: ios.existsSync(),
      // TODO(stuartmorgan): Revisit the conditions here once the plans for handling
      // desktop in existing projects are in place.
      linuxPlatform: featureFlags.isLinuxEnabled && linux.existsSync(),
      macOSPlatform: featureFlags.isMacOSEnabled && macos.existsSync(),
      windowsPlatform: featureFlags.isWindowsEnabled && windows.existsSync(),
      webPlatform: featureFlags.isWebEnabled && web.existsSync(),
      deprecationBehavior: deprecationBehavior,
      allowedPlugins: allowedPlugins,
    );
  }

  /// Applies template files and generates project files and plugin
  /// registrants for app and module projects only for the specified platforms.
  Future<void> ensureReadyForPlatformSpecificTooling({
    bool androidPlatform = false,
    bool iosPlatform = false,
    bool linuxPlatform = false,
    bool macOSPlatform = false,
    bool windowsPlatform = false,
    bool webPlatform = false,
    DeprecationBehavior deprecationBehavior = DeprecationBehavior.none,
    Iterable<String>? allowedPlugins,
  }) async {
    if (!directory.existsSync() || isPlugin) {
      return;
    }
    await refreshPluginsList(this, iosPlatform: iosPlatform, macOSPlatform: macOSPlatform);
    if (androidPlatform) {
      await android.ensureReadyForPlatformSpecificTooling(deprecationBehavior: deprecationBehavior);
    }
    if (iosPlatform) {
      await ios.ensureReadyForPlatformSpecificTooling();
    }
    if (linuxPlatform) {
      await linux.ensureReadyForPlatformSpecificTooling();
    }
    if (macOSPlatform) {
      await macos.ensureReadyForPlatformSpecificTooling();
    }
    if (windowsPlatform) {
      await windows.ensureReadyForPlatformSpecificTooling();
    }
    if (webPlatform) {
      await web.ensureReadyForPlatformSpecificTooling();
    }
    await injectPlugins(
      this,
      androidPlatform: androidPlatform,
      iosPlatform: iosPlatform,
      linuxPlatform: linuxPlatform,
      macOSPlatform: macOSPlatform,
      windowsPlatform: windowsPlatform,
      allowedPlugins: allowedPlugins,
    );
  }

  void checkForDeprecation({DeprecationBehavior deprecationBehavior = DeprecationBehavior.none}) {
    if (android.existsSync() && pubspecFile.existsSync()) {
      android.checkForDeprecation(deprecationBehavior: deprecationBehavior);
    }
  }

  /// Returns a json encoded string containing the [appName], [version], and [buildNumber] that is used to generate version.json
  String getVersionInfo() {
    final String? buildName = manifest.buildName;
    final String? buildNumber = manifest.buildNumber;
    final Map<String, String> versionFileJson = <String, String>{
      'app_name': manifest.appName,
      if (buildName != null) 'version': buildName,
      if (buildNumber != null) 'build_number': buildNumber,
      'package_name': manifest.appName,
    };
    return jsonEncode(versionFileJson);
  }
}

/// Base class for projects per platform.
abstract class FlutterProjectPlatform {
  /// Plugin's platform config key, e.g., "macos", "ios".
  String get pluginConfigKey;

  /// Whether the platform exists in the project.
  bool existsSync();
}

/// Represents the Android sub-project of a Flutter project.
///
/// Instances will reflect the contents of the `android/` sub-folder of
/// Flutter applications and the `.android/` sub-folder of Flutter module projects.
class AndroidProject extends FlutterProjectPlatform {
  AndroidProject._(this.parent);

  // User facing string when java/gradle/agp versions are compatible.
  @visibleForTesting
  static const String validJavaGradleAgpString = 'compatible java/gradle/agp';

  // User facing link that describes compatibility between gradle and
  // android gradle plugin.
  static const String gradleAgpCompatUrl =
      'https://developer.android.com/studio/releases/gradle-plugin#updating-gradle';

  // User facing link that describes compatibility between java and the first
  // version of gradle to support it.
  static const String javaGradleCompatUrl =
      'https://docs.gradle.org/current/userguide/compatibility.html#java';

  // User facing link that describes instructions for downloading
  // the latest version of Android Studio.
  static const String installAndroidStudioUrl = 'https://developer.android.com/studio/install';

  /// The parent of this project.
  final FlutterProject parent;

  @override
  String get pluginConfigKey => AndroidPlugin.kConfigKey;

  static final RegExp _androidNamespacePattern = RegExp(
    'android {[\\S\\s]+namespace\\s*=?\\s*[\'"](.+)[\'"]',
  );
  static final RegExp _applicationIdPattern = RegExp(
    '^\\s*applicationId\\s*=?\\s*[\'"](.*)[\'"]\\s*\$',
  );
  static final RegExp _imperativeKotlinPluginPattern = RegExp(
    '^\\s*apply plugin\\:\\s+[\'"]kotlin-android[\'"]\\s*\$',
  );

  /// Examples of strings that this regex matches:
  /// - `id "kotlin-android"`
  /// - `id("kotlin-android")`
  /// - `id ( "kotlin-android" ) `
  /// - `id "org.jetbrains.kotlin.android"`
  /// - `id("org.jetbrains.kotlin.android")`
  /// - `id ( "org.jetbrains.kotlin.android" )`
  static final List<RegExp> _declarativeKotlinPluginPatterns = <RegExp>[
    RegExp('^\\s*id\\s*\\(?\\s*[\'"]kotlin-android[\'"]\\s*\\)?\\s*\$'),
    RegExp('^\\s*id\\s*\\(?\\s*[\'"]org.jetbrains.kotlin.android[\'"]\\s*\\)?\\s*\$'),
  ];

  /// Pattern used to find the assignment of the "group" property in Gradle.
  /// Expected example: `group "dev.flutter.plugin"`
  /// Regex is used in both Groovy and Kotlin Gradle files.
  static final RegExp _groupPattern = RegExp('^\\s*group\\s*=?\\s*[\'"](.*)[\'"]\\s*\$');

  /// The Gradle root directory of the Android host app. This is the directory
  /// containing the `app/` subdirectory and the `settings.gradle` file that
  /// includes it in the overall Gradle project.
  Directory get hostAppGradleRoot {
    if (!isModule || _editableHostAppDirectory.existsSync()) {
      return _editableHostAppDirectory;
    }
    return ephemeralDirectory;
  }

  /// The Gradle root directory of the Android wrapping of Flutter and plugins.
  /// This is the same as [hostAppGradleRoot] except when the project is
  /// a Flutter module with an editable host app.
  Directory get _flutterLibGradleRoot => isModule ? ephemeralDirectory : _editableHostAppDirectory;

  Directory get ephemeralDirectory => parent.directory.childDirectory('.android');
  Directory get _editableHostAppDirectory => parent.directory.childDirectory('android');

  /// True if the parent Flutter project is a module.
  bool get isModule => parent.isModule;

  /// True if the parent Flutter project is a plugin.
  bool get isPlugin => parent.isPlugin;

  /// True if the Flutter project is using the AndroidX support library.
  bool get usesAndroidX => parent.usesAndroidX;

  /// Returns true if the current version of the Gradle plugin is supported.
  late final bool isSupportedVersion = _computeSupportedVersion();

  /// Gets all build variants of this project.
  Future<List<String>> getBuildVariants() async {
    if (!existsSync() || androidBuilder == null) {
      return const <String>[];
    }
    return androidBuilder!.getBuildVariants(project: parent);
  }

  /// Outputs app link related settings into a json file.
  ///
  /// The return future resolves to the path of the json file.
  ///
  /// The future resolves to null if it fails to retrieve app link settings.
  Future<String> outputsAppLinkSettings({required String variant}) async {
    if (!existsSync() || androidBuilder == null) {
      throwToolExit('Target directory $hostAppGradleRoot is not an Android project');
    }
    return androidBuilder!.outputsAppLinkSettings(variant, project: parent);
  }

  bool _computeSupportedVersion() {
    final FileSystem fileSystem = hostAppGradleRoot.fileSystem;
    final File plugin = hostAppGradleRoot.childFile(
      fileSystem.path.join('buildSrc', 'src', 'main', 'groovy', 'FlutterPlugin.groovy'),
    );
    if (plugin.existsSync()) {
      return false;
    }
    try {
      for (final String line in appGradleFile.readAsLinesSync()) {
        // This syntax corresponds to applying the Flutter Gradle Plugin with a
        // script.
        // See https://docs.gradle.org/current/userguide/plugins.html#sec:script_plugins.
        final bool fileBasedApply = line.contains(RegExp(r'apply from: .*/flutter.gradle'));

        // This syntax corresponds to applying the Flutter Gradle Plugin using
        // the declarative "plugins {}" block after including it in the
        // pluginManagement block of the settings.gradle file.
        // See https://docs.gradle.org/current/userguide/composite_builds.html#included_plugin_builds,
        // as well as the settings.gradle and build.gradle templates.
        final bool declarativeApply = line.contains(
          RegExp(r'dev\.flutter\.(?:(?:flutter-gradle-plugin)|(?:`flutter-gradle-plugin`))'),
        );

        // This case allows for flutter run/build to work for modules. It does
        // not guarantee the Flutter Gradle Plugin is applied.
        final bool managed = line.contains(RegExp('def flutterPluginVersion = [\'"]managed[\'"]'));
        if (fileBasedApply || declarativeApply || managed) {
          return true;
        }
      }
    } on FileSystemException {
      return false;
    }
    return false;
  }

  /// True, if the app project is using Kotlin.
  bool get isKotlin {
    final bool imperativeMatch =
        firstMatchInFile(appGradleFile, _imperativeKotlinPluginPattern) != null;
    final bool declarativeMatch = _declarativeKotlinPluginPatterns.any((RegExp pattern) {
      return (firstMatchInFile(appGradleFile, pattern) != null);
    });
    return imperativeMatch || declarativeMatch;
  }

  /// Gets top-level Gradle build file.
  /// See https://developer.android.com/build#top-level.
  ///
  /// The file must exist and it must be written in either Groovy (build.gradle)
  /// or Kotlin (build.gradle.kts).
  File get hostAppGradleFile {
    return getGroovyOrKotlin(hostAppGradleRoot, 'build.gradle');
  }

  /// Gets the project root level Gradle settings file.
  ///
  /// The file must exist and it must be written in either Groovy (build.gradle)
  /// or Kotlin (build.gradle.kts).
  File get settingsGradleFile {
    return getGroovyOrKotlin(hostAppGradleRoot, 'settings.gradle');
  }

  File getGroovyOrKotlin(Directory directory, String baseFilename) {
    final File groovyFile = directory.childFile(baseFilename);
    final File kotlinFile = directory.childFile('$baseFilename.kts');

    if (groovyFile.existsSync()) {
      // We mimic Gradle's behavior of preferring Groovy over Kotlin when both files exist.
      return groovyFile;
    }
    if (kotlinFile.existsSync()) {
      return kotlinFile;
    }

    // TODO(bartekpacia): An exception should be thrown when neither
    // the Groovy or Kotlin file exists, instead of falling back to the
    // Groovy file. See #141180.
    return groovyFile;
  }

  /// Gets the module-level build.gradle file.
  /// See https://developer.android.com/build#module-level.
  ///
  /// The file must exist and it must be written in either Groovy (build.gradle)
  /// or Kotlin (build.gradle.kts).
  File get appGradleFile {
    final Directory appDir = hostAppGradleRoot.childDirectory('app');
    return getGroovyOrKotlin(appDir, 'build.gradle');
  }

  File get appManifestFile {
    if (isUsingGradle) {
      return hostAppGradleRoot
          .childDirectory('app')
          .childDirectory('src')
          .childDirectory('main')
          .childFile('AndroidManifest.xml');
    }

    return hostAppGradleRoot.childFile('AndroidManifest.xml');
  }

  File get gradleAppOutV1File => gradleAppOutV1Directory.childFile('app-debug.apk');

  Directory get gradleAppOutV1Directory {
    return globals.fs.directory(
      globals.fs.path.join(hostAppGradleRoot.path, 'app', 'build', 'outputs', 'apk'),
    );
  }

  /// Whether the current flutter project has an Android sub-project.
  @override
  bool existsSync() {
    return parent.isModule || _editableHostAppDirectory.existsSync();
  }

  /// Check if the versions of Java, Gradle and AGP are compatible.
  ///
  /// This is expected to be called from
  /// flutter_tools/lib/src/project_validator.dart.
  Future<ProjectValidatorResult> validateJavaAndGradleAgpVersions() async {
    // Constructing ProjectValidatorResult happens here and not in
    // flutter_tools/lib/src/project_validator.dart because of the additional
    // Complexity of variable status values and error string formatting.
    const String visibleName = 'Java/Gradle/Android Gradle Plugin';
    final CompatibilityResult validJavaGradleAgpVersions = await hasValidJavaGradleAgpVersions();

    return ProjectValidatorResult(
      name: visibleName,
      value: validJavaGradleAgpVersions.description,
      status:
          validJavaGradleAgpVersions.success
              ? StatusProjectValidator.success
              : StatusProjectValidator.error,
    );
  }

  /// Ensures Java SDK is compatible with the project's Gradle version and
  /// the project's Gradle version is compatible with the AGP version used
  /// in build.gradle.
  Future<CompatibilityResult> hasValidJavaGradleAgpVersions() async {
    final String? gradleVersion = await gradle.getGradleVersion(
      hostAppGradleRoot,
      globals.logger,
      globals.processManager,
    );
    final String? agpVersion = gradle.getAgpVersion(hostAppGradleRoot, globals.logger);
    final String? javaVersion = versionToParsableString(globals.java?.version);

    // Assume valid configuration.
    String description = validJavaGradleAgpString;

    final bool compatibleGradleAgp = gradle.validateGradleAndAgp(
      globals.logger,
      gradleV: gradleVersion,
      agpV: agpVersion,
    );

    final bool compatibleJavaGradle = gradle.validateJavaAndGradle(
      globals.logger,
      javaV: javaVersion,
      gradleV: gradleVersion,
    );

    // Begin description formatting.
    if (!compatibleGradleAgp) {
      final String gradleDescription =
          agpVersion != null
              ? 'Update Gradle to at least "${gradle.getGradleVersionFor(agpVersion)}".'
              : '';
      description = '''
Incompatible Gradle/AGP versions. \n
Gradle Version: $gradleVersion, AGP Version: $agpVersion
$gradleDescription\n
See the link below for more information:
$gradleAgpCompatUrl
''';
    }
    if (!compatibleJavaGradle) {
      // Should contain the agp error (if present) but not the valid String.
      description = '''
${compatibleGradleAgp ? '' : description}
Incompatible Java/Gradle versions.
Java Version: $javaVersion, Gradle Version: $gradleVersion\n
See the link below for more information:
$javaGradleCompatUrl
''';
    }
    return CompatibilityResult(compatibleJavaGradle && compatibleGradleAgp, description);
  }

  bool get isUsingGradle {
    return hostAppGradleFile.existsSync();
  }

  String? get applicationId {
    return firstMatchInFile(appGradleFile, _applicationIdPattern)?.group(1);
  }

  /// Get the namespace for newer Android projects,
  /// which replaces the `package` attribute in the Manifest.xml.
  String? get namespace {
    try {
      // firstMatchInFile() reads per line but `_androidNamespacePattern` matches a multiline pattern.
      return _androidNamespacePattern.firstMatch(appGradleFile.readAsStringSync())?.group(1);
    } on FileSystemException {
      return null;
    }
  }

  String? get group {
    return firstMatchInFile(hostAppGradleFile, _groupPattern)?.group(1);
  }

  /// The build directory where the Android artifacts are placed.
  Directory get buildDirectory {
    return parent.buildDirectory;
  }

  Future<void> ensureReadyForPlatformSpecificTooling({
    DeprecationBehavior deprecationBehavior = DeprecationBehavior.none,
  }) async {
    if (isModule && _shouldRegenerateFromTemplate()) {
      await _regenerateLibrary();
      // Add ephemeral host app, if an editable host app does not already exist.
      if (!_editableHostAppDirectory.existsSync()) {
        await _overwriteFromTemplate(
          globals.fs.path.join('module', 'android', 'host_app_common'),
          ephemeralDirectory,
        );
        await _overwriteFromTemplate(
          globals.fs.path.join('module', 'android', 'host_app_ephemeral'),
          ephemeralDirectory,
        );
      }
    }
    if (!hostAppGradleRoot.existsSync()) {
      return;
    }
    gradle.updateLocalProperties(project: parent, requireAndroidSdk: false);
  }

  bool _shouldRegenerateFromTemplate() {
    return globals.fsUtils.isOlderThanReference(
          entity: ephemeralDirectory,
          referenceFile: parent.pubspecFile,
        ) ||
        globals.cache.isOlderThanToolsStamp(ephemeralDirectory);
  }

  File get localPropertiesFile => _flutterLibGradleRoot.childFile('local.properties');

  Directory get pluginRegistrantHost =>
      _flutterLibGradleRoot.childDirectory(isModule ? 'Flutter' : 'app');

  Future<void> _regenerateLibrary() async {
    ErrorHandlingFileSystem.deleteIfExists(ephemeralDirectory, recursive: true);
    await _overwriteFromTemplate(
      globals.fs.path.join('module', 'android', 'library_new_embedding'),
      ephemeralDirectory,
    );
    await _overwriteFromTemplate(
      globals.fs.path.join('module', 'android', 'gradle'),
      ephemeralDirectory,
    );
    globals.gradleUtils?.injectGradleWrapperIfNeeded(ephemeralDirectory);
  }

  Future<void> _overwriteFromTemplate(String path, Directory target) async {
    final Template template = await Template.fromName(
      path,
      fileSystem: globals.fs,
      templateManifest: null,
      logger: globals.logger,
      templateRenderer: globals.templateRenderer,
    );
    final String androidIdentifier =
        parent.manifest.androidPackage ?? 'com.example.${parent.manifest.appName}';
    template.render(target, <String, Object>{
      'android': true,
      'projectName': parent.manifest.appName,
      'androidIdentifier': androidIdentifier,
      'androidX': usesAndroidX,
      'agpVersion': gradle.templateAndroidGradlePluginVersion,
      'agpVersionForModule': gradle.templateAndroidGradlePluginVersionForModule,
      'kotlinVersion': gradle.templateKotlinGradlePluginVersion,
      'gradleVersion': gradle.templateDefaultGradleVersion,
      'compileSdkVersion': gradle.compileSdkVersion,
      'minSdkVersion': gradle.minSdkVersion,
      'ndkVersion': gradle.ndkVersion,
      'targetSdkVersion': gradle.targetSdkVersion,
    }, printStatusWhenWriting: false);
  }

  void checkForDeprecation({DeprecationBehavior deprecationBehavior = DeprecationBehavior.none}) {
    if (deprecationBehavior == DeprecationBehavior.none) {
      return;
    }
    final AndroidEmbeddingVersionResult result = computeEmbeddingVersion();
    if (result.version != AndroidEmbeddingVersion.v1) {
      return;
    }
    // The v1 android embedding has been deleted.
    throwToolExit('Build failed due to use of deleted Android v1 embedding.', exitCode: 1);
  }

  AndroidEmbeddingVersion getEmbeddingVersion() {
    final AndroidEmbeddingVersion androidEmbeddingVersion = computeEmbeddingVersion().version;
    if (androidEmbeddingVersion == AndroidEmbeddingVersion.v1) {
      throwToolExit('Build failed due to use of deleted Android v1 embedding.', exitCode: 1);
    }

    return androidEmbeddingVersion;
  }

  AndroidEmbeddingVersionResult computeEmbeddingVersion() {
    if (isModule) {
      // A module type's Android project is used in add-to-app scenarios and
      // only supports the V2 embedding.
      return AndroidEmbeddingVersionResult(AndroidEmbeddingVersion.v2, 'Is add-to-app module');
    }
    if (isPlugin) {
      // Plugins do not use an appManifest, so we stop here.
      //
      // TODO(garyq): This method does not currently check for code references to
      // the v1 embedding, we should check for this once removal is further along.
      return AndroidEmbeddingVersionResult(AndroidEmbeddingVersion.v2, 'Is plugin');
    }
    if (!appManifestFile.existsSync()) {
      return AndroidEmbeddingVersionResult(
        AndroidEmbeddingVersion.v1,
        'No `${appManifestFile.absolute.path}` file',
      );
    }
    XmlDocument document;
    try {
      document = XmlDocument.parse(appManifestFile.readAsStringSync());
    } on XmlException {
      throwToolExit(
        'Error parsing $appManifestFile '
        'Please ensure that the android manifest is a valid XML document and try again.',
      );
    } on FileSystemException {
      throwToolExit(
        'Error reading $appManifestFile even though it exists. '
        'Please ensure that you have read permission to this file and try again.',
      );
    }
    for (final XmlElement application in document.findAllElements('application')) {
      final String? applicationName = application.getAttribute('android:name');
      if (applicationName == 'io.flutter.app.FlutterApplication') {
        return AndroidEmbeddingVersionResult(
          AndroidEmbeddingVersion.v1,
          '${appManifestFile.absolute.path} uses `android:name="io.flutter.app.FlutterApplication"`',
        );
      }
    }
    for (final XmlElement metaData in document.findAllElements('meta-data')) {
      final String? name = metaData.getAttribute('android:name');
      // External code checks for this string to identify flutter android apps.
      // See cl/667760684 as an example.
      if (name == 'flutterEmbedding') {
        final String? embeddingVersionString = metaData.getAttribute('android:value');
        if (embeddingVersionString == '1') {
          return AndroidEmbeddingVersionResult(
            AndroidEmbeddingVersion.v1,
            '${appManifestFile.absolute.path} `<meta-data android:name="flutterEmbedding"` has value 1',
          );
        }
        if (embeddingVersionString == '2') {
          return AndroidEmbeddingVersionResult(
            AndroidEmbeddingVersion.v2,
            '${appManifestFile.absolute.path} `<meta-data android:name="flutterEmbedding"` has value 2',
          );
        }
      }
    }
    return AndroidEmbeddingVersionResult(
      AndroidEmbeddingVersion.v1,
      'No `<meta-data android:name="flutterEmbedding" android:value="2"/>` in ${appManifestFile.absolute.path}',
    );
  }

  static const bool _impellerEnabledByDefault = true;

  /// Returns the `io.flutter.embedding.android.EnableImpeller` manifest value.
  ///
  /// If there is no manifest file, or the key is not present, returns `false`.
  bool computeImpellerEnabled() {
    if (!appManifestFile.existsSync()) {
      return _impellerEnabledByDefault;
    }
    final XmlDocument document;
    try {
      document = XmlDocument.parse(appManifestFile.readAsStringSync());
    } on XmlException {
      throwToolExit(
        'Error parsing $appManifestFile '
        'Please ensure that the android manifest is a valid XML document and try again.',
      );
    } on FileSystemException {
      throwToolExit(
        'Error reading $appManifestFile even though it exists. '
        'Please ensure that you have read permission to this file and try again.',
      );
    }
    for (final XmlElement metaData in document.findAllElements('meta-data')) {
      final String? name = metaData.getAttribute('android:name');
      if (name == 'io.flutter.embedding.android.EnableImpeller') {
        final String? value = metaData.getAttribute('android:value');
        if (value == 'true') {
          return true;
        }
        if (value == 'false') {
          return false;
        }
      }
    }
    return _impellerEnabledByDefault;
  }
}

/// Iteration of the embedding Java API in the engine used by the Android project.
enum AndroidEmbeddingVersion {
  /// V1 APIs based on io.flutter.app.FlutterActivity.
  v1,

  /// V2 APIs based on io.flutter.embedding.android.FlutterActivity.
  v2,
}

/// Data class that holds the results of checking for embedding version.
///
/// This class includes the reason why a particular embedding was selected.
class AndroidEmbeddingVersionResult {
  AndroidEmbeddingVersionResult(this.version, this.reason);

  /// The embedding version.
  AndroidEmbeddingVersion version;

  /// The reason why the embedding version was selected.
  String reason;
}

// What the tool should do when encountering deprecated API in applications.
enum DeprecationBehavior {
  // The command being run does not care about deprecation status.
  none,
  // The command should continue and ignore the deprecation warning.
  ignore,
  // The command should exit the tool.
  exit,
}

/// Represents the web sub-project of a Flutter project.
class WebProject extends FlutterProjectPlatform {
  WebProject._(this.parent);

  final FlutterProject parent;

  @override
  String get pluginConfigKey => WebPlugin.kConfigKey;

  /// Whether this flutter project has a web sub-project.
  @override
  bool existsSync() {
    return parent.directory.childDirectory('web').existsSync() && indexFile.existsSync();
  }

  /// The 'lib' directory for the application.
  Directory get libDirectory => parent.directory.childDirectory('lib');

  /// The directory containing additional files for the application.
  Directory get directory => parent.directory.childDirectory('web');

  /// The html file used to host the flutter web application.
  File get indexFile => parent.directory.childDirectory('web').childFile('index.html');

  /// The .dart_tool/dartpad directory
  Directory get dartpadToolDirectory =>
      parent.directory.childDirectory('.dart_tool').childDirectory('dartpad');

  Future<void> ensureReadyForPlatformSpecificTooling() async {
    /// Create .dart_tool/dartpad/web_plugin_registrant.dart.
    /// See: https://github.com/dart-lang/dart-services/pull/874
    await injectBuildTimePluginFilesForWebPlatform(parent, destination: dartpadToolDirectory);
  }
}

/// The Fuchsia sub project.
class FuchsiaProject {
  FuchsiaProject._(this.project);

  final FlutterProject project;

  Directory? _editableHostAppDirectory;
  Directory get editableHostAppDirectory =>
      _editableHostAppDirectory ??= project.directory.childDirectory('fuchsia');

  bool existsSync() => editableHostAppDirectory.existsSync();

  Directory? _meta;
  Directory get meta => _meta ??= editableHostAppDirectory.childDirectory('meta');
}

// Combines success and a description into one object that can be returned
// together.
@visibleForTesting
class CompatibilityResult {
  CompatibilityResult(this.success, this.description);
  final bool success;
  final String description;
}

/// Converts a [Version] to a string that can be parsed by [Version.parse].
String? versionToParsableString(Version? version) {
  if (version == null) {
    return null;
  }

  return '${version.major}.${version.minor}.${version.patch}';
}
