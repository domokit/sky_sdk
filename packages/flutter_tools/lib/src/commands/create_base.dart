// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:uuid/uuid.dart';
import 'package:yaml/yaml.dart';

import '../android/android.dart' as android_common;
import '../android/android_workflow.dart';
import '../android/gradle_utils.dart' as gradle;
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../build_system/build_system.dart';
import '../cache.dart';
import '../convert.dart';
import '../dart/generate_synthetic_packages.dart';
import '../flutter_project_metadata.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../runner/flutter_command.dart';
import '../template.dart';

const List<String> _kAvailablePlatforms = <String>[
  'ios',
  'android',
  'windows',
  'linux',
  'macos',
  'web',
];

/// A list of all possible create platforms, even those that may not be enabled
/// with the current config.
const List<String> kAllCreatePlatforms = <String>[
  'ios',
  'android',
  'windows',
  'linux',
  'macos',
  'web',
];

const String _kDefaultPlatformArgumentHelp =
    '(required) The platforms supported by this project. '
    'Platform folders (e.g. android/) will be generated in the target project. '
    'Adding desktop platforms requires the corresponding desktop config setting to be enabled.';

/// Common behavior for `flutter create` and `flutter widget-preview start` commands.
mixin CreateBase on FlutterCommand {
  /// Pattern for a Windows file system drive (e.g. "D:").
  ///
  /// `dart:io` does not recognize strings matching this pattern as absolute
  /// paths, as they have no top level back-slash; however, users often specify
  /// this
  @visibleForTesting
  @protected
  static final RegExp kWindowsDrivePattern = RegExp(r'^[a-zA-Z]:$');

  /// The output directory of the command.
  @protected
  @visibleForTesting
  Directory get projectDir {
    final String argProjectDir = argResults!.rest.first;
    if (globals.platform.isWindows && kWindowsDrivePattern.hasMatch(argProjectDir)) {
      throwToolExit(
        'You attempted to create a flutter project at the path "$argProjectDir", which is the name of a drive. This '
        'is usually a mistake--you probably want to specify a containing directory, like "$argProjectDir\\app_name". '
        'If you really want it at the drive root, re-run the command with the root directory after the drive, like '
        '"$argProjectDir\\".',
      );
    }
    return globals.fs.directory(argResults!.rest.first);
  }

  /// The normalized absolute path of [projectDir].
  @protected
  String get projectDirPath {
    return globals.fs.path.normalize(projectDir.absolute.path);
  }

  @protected
  bool get shouldCallPubGet {
    return boolArg('pub');
  }

  @protected
  bool get offline {
    return boolArg('offline');
  }

  /// Adds `--pub` and `--offline` options.
  @protected
  void addPubOptions() {
    argParser
      ..addFlag(
        'pub',
        defaultsTo: true,
        help: 'Whether to run "flutter pub get" after the project has been created.',
      )
      ..addFlag(
        'offline',
        help:
            'When "flutter pub get" is run by the create command, this indicates '
            'whether to run it in offline mode or not. In offline mode, it will need to '
            'have all dependencies already available in the pub cache to succeed.',
      );
  }

  /// Adds a `--platforms` argument.
  ///
  /// The help message of the argument is replaced with `customHelp` if `customHelp` is not null.
  @protected
  void addPlatformsOptions({String? customHelp}) {
    argParser.addMultiOption(
      'platforms',
      help: customHelp ?? _kDefaultPlatformArgumentHelp,
      aliases: <String>['platform'],
      defaultsTo: <String>[..._kAvailablePlatforms],
      allowed: <String>[..._kAvailablePlatforms],
    );
  }

  /// Throw with exit code 2 if the output directory is invalid.
  @protected
  void validateOutputDirectoryArg() {
    final List<String>? rest = argResults?.rest;
    if (rest == null || rest.isEmpty) {
      throwToolExit('No option specified for the output directory.\n$usage', exitCode: 2);
    }

    if (rest.length > 1) {
      String message = 'Multiple output directories specified.';
      for (final String arg in rest) {
        if (arg.startsWith('-')) {
          message += '\nTry moving $arg to be immediately following $name';
          break;
        }
      }
      throwToolExit(message, exitCode: 2);
    }
  }

  /// Gets the flutter root directory.
  @protected
  String get flutterRoot => Cache.flutterRoot!;

  /// Determines the project type in an existing flutter project.
  ///
  /// If it has a .metadata file with the project_type in it, use that.
  /// If it has an android dir and an android/app dir, it's a legacy app
  /// If it has an ios dir and an ios/Flutter dir, it's a legacy app
  /// Otherwise, we don't presume to know what type of project it could be, since
  /// many of the files could be missing, and we can't really tell definitively.
  ///
  /// Throws assertion if [projectDir] does not exist or empty.
  /// Returns null if no project type can be determined.
  @protected
  FlutterProjectType? determineTemplateType() {
    assert(projectDir.existsSync() && projectDir.listSync().isNotEmpty);
    final File metadataFile = globals.fs.file(
      globals.fs.path.join(projectDir.absolute.path, '.metadata'),
    );
    final FlutterProjectMetadata projectMetadata = FlutterProjectMetadata(
      metadataFile,
      globals.logger,
    );
    final FlutterProjectType? projectType = projectMetadata.projectType;
    if (projectType != null) {
      return projectType;
    }

    bool exists(List<String> path) {
      return globals.fs
          .directory(globals.fs.path.joinAll(<String>[projectDir.absolute.path, ...path]))
          .existsSync();
    }

    // There either wasn't any metadata, or it didn't contain the project type,
    // so try and figure out what type of project it is from the existing
    // directory structure.
    if (exists(<String>['android', 'app']) ||
        exists(<String>['ios', 'Runner']) ||
        exists(<String>['ios', 'Flutter'])) {
      return FlutterProjectType.app;
    }
    // Since we can't really be definitive on nearly-empty directories, err on
    // the side of prudence and just say we don't know.
    return null;
  }

  /// Determines the organization.
  ///
  /// If `--org` is specified in the command, returns that directly.
  /// If `--org` is not specified, returns the organization from the existing project.
  @protected
  Future<String> getOrganization() async {
    String? organization = stringArg('org');
    if (!argResults!.wasParsed('org')) {
      final FlutterProject project = FlutterProject.fromDirectory(projectDir);
      final Set<String> existingOrganizations = await project.organizationNames;
      if (existingOrganizations.length == 1) {
        organization = existingOrganizations.first;
      } else if (existingOrganizations.length > 1) {
        throwToolExit(
          'Ambiguous organization in existing files: $existingOrganizations. '
          'The --org command line argument must be specified to recreate project.',
        );
      }
    }
    if (organization == null) {
      throwToolExit('The --org command line argument must be specified to create a project.');
    }
    return organization;
  }

  /// Throws with exit 2 if the project directory is illegal.
  @protected
  void validateProjectDir({bool overwrite = false}) {
    if (globals.fs.path.isWithin(flutterRoot, projectDirPath)) {
      // Make exception for dev and examples to facilitate example project development.
      final String examplesDirectory = globals.fs.path.join(flutterRoot, 'examples');
      final String devDirectory = globals.fs.path.join(flutterRoot, 'dev');
      if (!globals.fs.path.isWithin(examplesDirectory, projectDirPath) &&
          !globals.fs.path.isWithin(devDirectory, projectDirPath)) {
        throwToolExit(
          'Cannot create a project within the Flutter SDK. '
          "Target directory '$projectDirPath' is within the Flutter SDK at '$flutterRoot'.",
          exitCode: 2,
        );
      }
    }

    // If the destination directory is actually a file, then we refuse to
    // overwrite, on the theory that the user probably didn't expect it to exist.
    if (globals.fs.isFileSync(projectDirPath)) {
      final String message =
          "Invalid project name: '$projectDirPath' - refers to an existing file.";
      throwToolExit(
        overwrite ? '$message Refusing to overwrite a file with a directory.' : message,
        exitCode: 2,
      );
    }

    if (overwrite) {
      return;
    }

    final FileSystemEntityType type = globals.fs.typeSync(projectDirPath);

    // ignore: exhaustive_cases, https://github.com/dart-lang/linter/issues/3017
    switch (type) {
      case FileSystemEntityType.file:
        // Do not overwrite files.
        throwToolExit("Invalid project name: '$projectDirPath' - file exists.", exitCode: 2);
      case FileSystemEntityType.link:
        // Do not overwrite links.
        throwToolExit("Invalid project name: '$projectDirPath' - refers to a link.", exitCode: 2);
      case FileSystemEntityType.directory:
      case FileSystemEntityType.notFound:
        break;
    }
  }

  /// Gets the project name.
  ///
  /// If the `--project-name` is not specified explicitly,
  /// the `name` field from the pubspec.yaml file is used.
  ///
  /// If the pubspec.yaml file does not exist,
  /// the current directory path name is used.
  @protected
  String get projectName {
    String? projectName = stringArg('project-name');

    if (projectName == null) {
      final File pubspec = globals.fs.directory(projectDirPath).childFile('pubspec.yaml');

      if (pubspec.existsSync()) {
        final String pubspecContents = pubspec.readAsStringSync();

        try {
          final Object? pubspecYaml = loadYaml(pubspecContents);

          if (pubspecYaml is YamlMap) {
            final Object? pubspecName = pubspecYaml['name'];

            if (pubspecName is String) {
              projectName = pubspecName;
            }
          }
        } on YamlException {
          // If the pubspec is malformed, fallback to using the directory name.
        }
      }

      final String projectDirName = globals.fs.path.basename(projectDirPath);

      projectName ??= projectDirName;
    }

    if (!boolArg('skip-name-checks')) {
      final String? error = _validateProjectName(projectName);
      if (error != null) {
        throwToolExit(error);
      }
    }
    return projectName;
  }

  /// Creates a template to use for [renderTemplate].
  @protected
  Map<String, Object?> createTemplateContext({
    required String organization,
    required String projectName,
    required String titleCaseProjectName,
    String? projectDescription,
    String? androidLanguage,
    String? iosDevelopmentTeam,
    String? iosLanguage,
    required String flutterRoot,
    required String dartSdkVersionBounds,
    String? agpVersion,
    String? kotlinVersion,
    String? gradleVersion,
    bool withPlatformChannelPluginHook = false,
    bool withSwiftPackageManager = false,
    bool withFfiPluginHook = false,
    bool withFfiPackage = false,
    bool withEmptyMain = false,
    bool ios = false,
    bool android = false,
    bool web = false,
    bool linux = false,
    bool macos = false,
    bool windows = false,
    bool implementationTests = false,
  }) {
    final String pluginDartClass = _createPluginClassName(projectName);
    final String pluginClass =
        pluginDartClass.endsWith('Plugin') ? pluginDartClass : '${pluginDartClass}Plugin';
    final String pluginClassSnakeCase = snakeCase(pluginClass);
    final String pluginClassCapitalSnakeCase = pluginClassSnakeCase.toUpperCase();
    final String pluginClassLowerCamelCase =
        pluginClass[0].toLowerCase() + pluginClass.substring(1);
    final String appleIdentifier = createUTIIdentifier(organization, projectName);
    final String androidIdentifier = createAndroidIdentifier(organization, projectName);
    final String windowsIdentifier = createWindowsIdentifier(organization, projectName);
    // Linux uses the same scheme as the Android identifier.
    // https://developer.gnome.org/gio/stable/GApplication.html#g-application-id-is-valid
    final String linuxIdentifier = androidIdentifier;

    return <String, Object?>{
      'organization': organization,
      'projectName': projectName,
      'titleCaseProjectName': titleCaseProjectName,
      'androidIdentifier': androidIdentifier,
      'iosIdentifier': appleIdentifier,
      'macosIdentifier': appleIdentifier,
      'linuxIdentifier': linuxIdentifier,
      'windowsIdentifier': windowsIdentifier,
      'description': projectDescription,
      'dartSdk': '$flutterRoot/bin/cache/dart-sdk',
      'androidMinApiLevel': android_common.minApiLevel,
      'androidSdkVersion': kAndroidSdkMinVersion,
      'pluginClass': pluginClass,
      'pluginClassSnakeCase': pluginClassSnakeCase,
      'pluginClassLowerCamelCase': pluginClassLowerCamelCase,
      'pluginClassCapitalSnakeCase': pluginClassCapitalSnakeCase,
      'pluginDartClass': pluginDartClass,
      'pluginProjectUUID': const Uuid().v4().toUpperCase(),
      'withFfi': withFfiPluginHook || withFfiPackage,
      'withFfiPackage': withFfiPackage,
      'withFfiPluginHook': withFfiPluginHook,
      'withPlatformChannelPluginHook': withPlatformChannelPluginHook,
      'withSwiftPackageManager': withSwiftPackageManager,
      'withPluginHook': withFfiPluginHook || withFfiPackage || withPlatformChannelPluginHook,
      'withEmptyMain': withEmptyMain,
      'androidLanguage': androidLanguage,
      'iosLanguage': iosLanguage,
      'hasIosDevelopmentTeam': iosDevelopmentTeam != null && iosDevelopmentTeam.isNotEmpty,
      'iosDevelopmentTeam': iosDevelopmentTeam ?? '',
      'flutterRevision': escapeYamlString(globals.flutterVersion.frameworkRevision),
      'flutterChannel': escapeYamlString(globals.flutterVersion.getBranchName()), // may contain PII
      'ios': ios,
      'android': android,
      'web': web,
      'linux': linux,
      'macos': macos,
      'windows': windows,
      'year': DateTime.now().year,
      'dartSdkVersionBounds': dartSdkVersionBounds,
      'implementationTests': implementationTests,
      'agpVersion': agpVersion,
      'agpVersionForModule': gradle.templateAndroidGradlePluginVersionForModule,
      'kotlinVersion': kotlinVersion,
      'gradleVersion': gradleVersion,
      'compileSdkVersion': gradle.compileSdkVersion,
      'minSdkVersion': gradle.minSdkVersion,
      'ndkVersion': gradle.ndkVersion,
      'targetSdkVersion': gradle.targetSdkVersion,
    };
  }

  /// Renders the template, generate files into `directory`.
  ///
  /// `templateName` should match one of directory names under flutter_tools/template/.
  /// If `overwrite` is true, overwrites existing files, `overwrite` defaults to `false`.
  @protected
  Future<int> renderTemplate(
    String templateName,
    Directory directory,
    Map<String, Object?> context, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
  }) async {
    final Template template = await Template.fromName(
      templateName,
      fileSystem: globals.fs,
      logger: globals.logger,
      templateRenderer: globals.templateRenderer,
      templateManifest: _templateManifest,
    );
    return template.render(
      directory,
      context,
      overwriteExisting: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );
  }

  /// Merges named templates into a single template, output to `directory`.
  ///
  /// `names` should match directory names under flutter_tools/template/.
  ///
  /// If `overwrite` is true, overwrites existing files, `overwrite` defaults to `false`.
  @protected
  Future<int> renderMerged(
    List<String> names,
    Directory directory,
    Map<String, Object?> context, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
  }) async {
    final Template template = await Template.merged(
      names,
      directory,
      fileSystem: globals.fs,
      logger: globals.logger,
      templateRenderer: globals.templateRenderer,
      templateManifest: _templateManifest,
    );
    return template.render(
      directory,
      context,
      overwriteExisting: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );
  }

  /// Generate application project in the `directory` using `templateContext`.
  ///
  /// If `overwrite` is true, overwrites existing files, `overwrite` defaults to `false`.
  @protected
  Future<int> generateApp(
    List<String> templateNames,
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool pluginExampleApp = false,
    bool printStatusWhenWriting = true,
    bool generateMetadata = true,
    FlutterProjectType? projectType,
  }) async {
    int generatedCount = 0;
    generatedCount += await renderMerged(
      <String>[...templateNames, 'app_shared'],
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );
    final FlutterProject project = FlutterProject.fromDirectory(directory);
    if (templateContext['android'] == true) {
      generatedCount += _injectGradleWrapper(project);
    }

    final bool androidPlatform = templateContext['android'] as bool? ?? false;
    final bool iosPlatform = templateContext['ios'] as bool? ?? false;
    final bool linuxPlatform = templateContext['linux'] as bool? ?? false;
    final bool macOSPlatform = templateContext['macos'] as bool? ?? false;
    final bool windowsPlatform = templateContext['windows'] as bool? ?? false;
    final bool webPlatform = templateContext['web'] as bool? ?? false;

    if (shouldCallPubGet) {
      final Environment environment = Environment(
        artifacts: globals.artifacts!,
        logger: globals.logger,
        cacheDir: globals.cache.getRoot(),
        engineVersion: globals.flutterVersion.engineRevision,
        fileSystem: globals.fs,
        flutterRootDir: globals.fs.directory(Cache.flutterRoot),
        outputDir: globals.fs.directory(getBuildDirectory()),
        processManager: globals.processManager,
        platform: globals.platform,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        projectDir: project.directory,
        packageConfigPath: packageConfigPath(),
        generateDartPluginRegistry: true,
      );

      // Generate the l10n synthetic package that will be injected into the
      // package_config in the call to pub.get() below.
      await generateLocalizationsSyntheticPackage(
        environment: environment,
        buildSystem: globals.buildSystem,
        buildTargets: globals.buildTargets,
      );
    }
    final List<SupportedPlatform> platformsForMigrateConfig = <SupportedPlatform>[
      SupportedPlatform.root,
    ];
    if (androidPlatform) {
      gradle.updateLocalProperties(project: project, requireAndroidSdk: false);
      platformsForMigrateConfig.add(SupportedPlatform.android);
    }
    if (iosPlatform) {
      platformsForMigrateConfig.add(SupportedPlatform.ios);
    }
    if (linuxPlatform) {
      platformsForMigrateConfig.add(SupportedPlatform.linux);
    }
    if (macOSPlatform) {
      platformsForMigrateConfig.add(SupportedPlatform.macos);
    }
    if (webPlatform) {
      platformsForMigrateConfig.add(SupportedPlatform.web);
    }
    if (windowsPlatform) {
      platformsForMigrateConfig.add(SupportedPlatform.windows);
    }
    if (templateContext['fuchsia'] == true) {
      platformsForMigrateConfig.add(SupportedPlatform.fuchsia);
    }
    if (generateMetadata) {
      final File metadataFile = globals.fs.file(
        globals.fs.path.join(projectDir.absolute.path, '.metadata'),
      );
      final FlutterProjectMetadata metadata = FlutterProjectMetadata.explicit(
        file: metadataFile,
        versionRevision: globals.flutterVersion.frameworkRevision,
        versionChannel: globals.flutterVersion.getBranchName(), // may contain PII
        projectType: projectType,
        migrateConfig: MigrateConfig(),
        logger: globals.logger,
      );
      metadata.populate(
        platforms: platformsForMigrateConfig,
        projectDirectory: directory,
        update: false,
        currentRevision:
            stringArg('initial-create-revision') ?? globals.flutterVersion.frameworkRevision,
        createRevision: globals.flutterVersion.frameworkRevision,
        logger: globals.logger,
      );
      metadata.writeFile();
    }

    return generatedCount;
  }

  /// Creates an android identifier.
  ///
  /// Android application ID is specified in: https://developer.android.com/studio/build/application-id
  /// All characters must be alphanumeric or an underscore [a-zA-Z0-9_].
  static String createAndroidIdentifier(String organization, String name) {
    String tmpIdentifier = '$organization.$name';
    final RegExp disallowed = RegExp(r'[^\w\.]');
    tmpIdentifier = tmpIdentifier.replaceAll(disallowed, '');

    // It must have at least two segments (one or more dots).
    final List<String> segments =
        tmpIdentifier.split('.').where((String segment) => segment.isNotEmpty).toList();
    while (segments.length < 2) {
      segments.add('untitled');
    }

    // Each segment must start with a letter.
    final RegExp segmentPatternRegex = RegExp(r'^[a-zA-Z][\w]*$');
    final List<String> prefixedSegments =
        segments.map((String segment) {
          if (!segmentPatternRegex.hasMatch(segment)) {
            return 'u$segment';
          }
          return segment;
        }).toList();
    return prefixedSegments.join('.');
  }

  /// Creates a Windows package name.
  ///
  /// Package names must be a globally unique, commonly a GUID.
  static String createWindowsIdentifier(String organization, String name) {
    return const Uuid().v4().toUpperCase();
  }

  String _createPluginClassName(String name) {
    final String camelizedName = camelCase(name);
    return camelizedName[0].toUpperCase() + camelizedName.substring(1);
  }

  /// Create a UTI (https://en.wikipedia.org/wiki/Uniform_Type_Identifier) from a base name
  static String createUTIIdentifier(String organization, String name) {
    name = camelCase(name);
    String tmpIdentifier = '$organization.$name';
    final RegExp disallowed = RegExp(r'[^a-zA-Z0-9\-\.\u0080-\uffff]+');
    tmpIdentifier = tmpIdentifier.replaceAll(disallowed, '');

    // It must have at least two segments (one or more dots).
    final List<String> segments =
        tmpIdentifier.split('.').where((String segment) => segment.isNotEmpty).toList();
    while (segments.length < 2) {
      segments.add('untitled');
    }

    return segments.join('.');
  }

  late final Set<Uri> _templateManifest = _computeTemplateManifest();
  Set<Uri> _computeTemplateManifest() {
    final String flutterToolsAbsolutePath = globals.fs.path.join(
      Cache.flutterRoot!,
      'packages',
      'flutter_tools',
    );
    final String manifestPath = globals.fs.path.join(
      flutterToolsAbsolutePath,
      'templates',
      'template_manifest.json',
    );
    final String manifestFileContents;
    try {
      manifestFileContents = globals.fs.file(manifestPath).readAsStringSync();
    } on FileSystemException catch (e) {
      throwToolExit(
        'Unable to read the template manifest at path "$manifestPath".\n'
        'Make sure that your user account has sufficient permissions to read this file.\n'
        'Exception details: $e',
      );
    }
    final Map<String, Object?> manifest = json.decode(manifestFileContents) as Map<String, Object?>;
    return Set<Uri>.from(
      (manifest['files']! as List<Object?>).cast<String>().map<Uri>(
        (String path) => Uri.file(globals.fs.path.join(flutterToolsAbsolutePath, path)),
      ),
    );
  }

  int _injectGradleWrapper(FlutterProject project) {
    int filesCreated = 0;
    copyDirectory(
      globals.cache.getArtifactDirectory('gradle_wrapper'),
      project.android.hostAppGradleRoot,
      onFileCopied: (File sourceFile, File destinationFile) {
        filesCreated++;
        final String modes = sourceFile.statSync().modeString();
        if (modes.contains('x')) {
          globals.os.makeExecutable(destinationFile);
        }
      },
    );
    return filesCreated;
  }
}

// A valid Dart identifier that can be used for a package, i.e. no
// capital letters.
// https://dart.dev/language#important-concepts
final RegExp _identifierRegExp = RegExp('[a-z_][a-z0-9_]*');

// non-contextual dart keywords.
// https://dart.dev/language/keywords
const Set<String> _keywords = <String>{
  'abstract',
  'as',
  'assert',
  'async',
  'await',
  'break',
  'case',
  'catch',
  'class',
  'const',
  'continue',
  'covariant',
  'default',
  'deferred',
  'do',
  'dynamic',
  'else',
  'enum',
  'export',
  'extends',
  'extension',
  'external',
  'factory',
  'false',
  'final',
  'finally',
  'for',
  'function',
  'get',
  'hide',
  'if',
  'implements',
  'import',
  'in',
  'inout',
  'interface',
  'is',
  'late',
  'library',
  'mixin',
  'native',
  'new',
  'null',
  'of',
  'on',
  'operator',
  'out',
  'part',
  'patch',
  'required',
  'rethrow',
  'return',
  'set',
  'show',
  'source',
  'static',
  'super',
  'switch',
  'sync',
  'this',
  'throw',
  'true',
  'try',
  'typedef',
  'var',
  'void',
  'while',
  'with',
  'yield',
};

const Set<String> _packageDependencies = <String>{'collection', 'flutter', 'flutter_test', 'meta'};

/// Whether [name] is a valid Pub package.
@visibleForTesting
bool isValidPackageName(String name) {
  final Match? match = _identifierRegExp.matchAsPrefix(name);
  return match != null && match.end == name.length && !_keywords.contains(name);
}

/// Returns a potential valid name from the given [name].
///
/// If a valid name cannot be found, returns `null`.
@visibleForTesting
String? potentialValidPackageName(String name) {
  String newName = name.toLowerCase();
  if (newName.startsWith(RegExp(r'[0-9]'))) {
    newName = '_$newName';
  }
  newName = newName.replaceAll('-', '_');
  if (isValidPackageName(newName)) {
    return newName;
  } else {
    return null;
  }
}

// Return null if the project name is legal. Return a validation message if
// we should disallow the project name.
String? _validateProjectName(String projectName) {
  if (!isValidPackageName(projectName)) {
    final String? potentialValidName = potentialValidPackageName(projectName);
    return '"$projectName" is not a valid Dart package name.'
        '${potentialValidName != null ? ' Try "$potentialValidName" instead.' : ''}\n'
        '\n'
        'The name should consist of lowercase words separated by underscores, "like_this". '
        'Use only basic Latin letters and Arabic digits: [a-z0-9_], and '
        'ensure the name is a valid Dart identifier '
        '(i.e. it does not start with a digit and is not a reserved word).\n'
        '\n'
        'See https://dart.dev/tools/pub/pubspec#name for more information.';
  }
  if (_packageDependencies.contains(projectName)) {
    return "Invalid project name: '$projectName' - this will conflict with Flutter "
        'package dependencies.';
  }
  return null;
}
