// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../android/gradle_utils.dart' as gradle;
import '../base/common.dart';
import '../base/context.dart';
import '../base/file_system.dart';
import '../base/net.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../base/version.dart';
import '../base/version_range.dart';
import '../convert.dart';
import '../dart/pub.dart';
import '../features.dart';
import '../flutter_manifest.dart';
import '../flutter_project_metadata.dart';
import '../globals.dart' as globals;
import '../ios/code_signing.dart';
import '../macos/swift_package_manager.dart';
import '../macos/swift_packages.dart';
import '../project.dart';
import '../reporting/reporting.dart';
import '../runner/flutter_command.dart';
import 'create_base.dart';

const String kPlatformHelp =
    'The platforms supported by this project. '
    'Platform folders (e.g. android/) will be generated in the target project. '
    'This argument only works when "--template" is set to app or plugin. '
    'When adding platforms to a plugin project, the pubspec.yaml will be updated with the requested platform. '
    'Adding desktop platforms requires the corresponding desktop config setting to be enabled.';

class CreateCommand extends FlutterCommand with CreateBase {
  CreateCommand({bool verboseHelp = false}) {
    addPubOptions();
    argParser.addFlag(
      'with-driver-test',
      help:
          '(deprecated) Historically, this added a flutter_driver dependency and generated a '
          'sample "flutter drive" test. Now it does nothing. Consider using the '
          '"integration_test" package: https://pub.dev/packages/integration_test',
      hide: !verboseHelp,
    );
    argParser.addFlag('overwrite', help: 'When performing operations, overwrite existing files.');
    argParser.addOption(
      'description',
      defaultsTo: 'A new Flutter project.',
      help:
          'The description to use for your new Flutter project. This string ends up in the pubspec.yaml file.',
    );
    argParser.addOption(
      'org',
      defaultsTo: 'com.example',
      help:
          'The organization responsible for your new Flutter project, in reverse domain name notation. '
          'This string is used in Java package names and as prefix in the iOS bundle identifier.',
    );
    argParser.addOption(
      'project-name',
      help:
          'The project name for this new Flutter project. This must be a valid dart package name.',
    );
    argParser.addOption(
      'ios-language',
      abbr: 'i',
      defaultsTo: 'swift',
      allowed: <String>['objc', 'swift'],
      help:
          '(deprecated) The language to use for iOS-specific code, either Swift (recommended) or Objective-C (legacy).',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'android-language',
      abbr: 'a',
      defaultsTo: 'kotlin',
      allowed: <String>['java', 'kotlin'],
      help:
          'The language to use for Android-specific code, either Kotlin (recommended) or Java (legacy).',
    );
    argParser.addFlag(
      'skip-name-checks',
      help:
          'Allow the creation of applications and plugins with invalid names. '
          'This is only intended to enable testing of the tool itself.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'implementation-tests',
      help:
          'Include implementation tests that verify the template functions correctly. '
          'This is only intended to enable testing of the tool itself.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'initial-create-revision',
      help:
          'The Flutter SDK git commit hash to store in .migrate_config. This parameter is used by the tool '
          'internally and should generally not be used manually.',
      hide: !verboseHelp,
    );
    addPlatformsOptions(customHelp: kPlatformHelp);

    final List<ParsedFlutterTemplateType> enabledTemplates =
        ParsedFlutterTemplateType.enabledValues(featureFlags);
    argParser.addOption(
      'template',
      abbr: 't',
      allowed: enabledTemplates.map((ParsedFlutterTemplateType t) => t.cliName),
      help: 'Specify the type of project to create.',
      valueHelp: 'type',
      allowedHelp: CliEnum.allowedHelp(enabledTemplates),
    );
    argParser.addOption(
      'sample',
      abbr: 's',
      help:
          'Specifies the Flutter code sample to use as the "main.dart" for an application. Implies '
          '"--template=app". The value should be the sample ID of the desired sample from the API '
          'documentation website (https://api.flutter.dev/). An example can be found at: '
          'https://api.flutter.dev/flutter/widgets/SingleChildScrollView-class.html',
      valueHelp: 'id',
    );
    argParser.addFlag(
      'empty',
      abbr: 'e',
      help:
          'Specifies creating using an application template with a main.dart that is minimal, '
          'including no comments, as a starting point for a new application. Implies "--template=app".',
    );
    argParser.addOption(
      'list-samples',
      help:
          'Specifies a JSON output file for a listing of Flutter code samples '
          'that can be created with "--sample".',
      valueHelp: 'path',
    );
  }

  @override
  final String name = 'create';

  @override
  final String description =
      'Create a new Flutter project.\n\n'
      'If run on a project that already exists, this will repair the project, recreating any files that are missing.';

  @override
  String get category => FlutterCommandCategory.project;

  @override
  String get invocation => '${runner?.executableName} $name <output directory>';

  @override
  Future<CustomDimensions> get usageValues async {
    return CustomDimensions(
      commandCreateProjectType: stringArg('template'),
      commandCreateAndroidLanguage: stringArg('android-language'),
      commandCreateIosLanguage: stringArg('ios-language'),
    );
  }

  @override
  Future<Event> unifiedAnalyticsUsageValues(String commandPath) async => Event.commandUsageValues(
    workflow: commandPath,
    commandHasTerminal: hasTerminal,
    createProjectType: stringArg('template'),
    createAndroidLanguage: stringArg('android-language'),
    createIosLanguage: stringArg('ios-language'),
  );

  // Lazy-initialize the net utilities with values from the context.
  late final Net _net = Net(
    httpClientFactory: context.get<HttpClientFactory>(),
    logger: globals.logger,
    platform: globals.platform,
  );

  /// The hostname for the Flutter docs for the current channel.
  String get _snippetsHost =>
      globals.flutterVersion.channel == 'stable' ? 'api.flutter.dev' : 'main-api.flutter.dev';

  Future<String?> _fetchSampleFromServer(String sampleId) async {
    // Sanity check the sampleId
    if (sampleId.contains(RegExp(r'[^-\w\.]'))) {
      throwToolExit(
        'Sample ID "$sampleId" contains invalid characters. Check the ID in the '
        'documentation and try again.',
      );
    }

    final Uri snippetsUri = Uri.https(_snippetsHost, 'snippets/$sampleId.dart');
    final List<int>? data = await _net.fetchUrl(snippetsUri);
    if (data == null || data.isEmpty) {
      return null;
    }
    return utf8.decode(data);
  }

  /// Fetches the samples index file from the Flutter docs website.
  Future<String?> _fetchSamplesIndexFromServer() async {
    final Uri snippetsUri = Uri.https(_snippetsHost, 'snippets/index.json');
    final List<int>? data = await _net.fetchUrl(snippetsUri, maxAttempts: 2);
    if (data == null || data.isEmpty) {
      return null;
    }
    return utf8.decode(data);
  }

  /// Fetches the samples index file from the server and writes it to
  /// [outputFilePath].
  Future<void> _writeSamplesJson(String outputFilePath) async {
    try {
      final File outputFile = globals.fs.file(outputFilePath);
      if (outputFile.existsSync()) {
        throwToolExit('File "$outputFilePath" already exists', exitCode: 1);
      }
      final String? samplesJson = await _fetchSamplesIndexFromServer();
      if (samplesJson == null) {
        throwToolExit('Unable to download samples', exitCode: 2);
      } else {
        outputFile.writeAsStringSync(samplesJson);
        globals.printStatus('Wrote samples JSON to "$outputFilePath"');
      }
    } on Exception catch (e) {
      throwToolExit('Failed to write samples JSON to "$outputFilePath": $e', exitCode: 2);
    }
  }

  FlutterTemplateType _getProjectType(Directory projectDir) {
    FlutterTemplateType? template;
    FlutterTemplateType? detectedProjectType;
    final bool metadataExists = projectDir.absolute.childFile('.metadata').existsSync();
    final String? templateArgument = stringArg('template');
    if (templateArgument != null) {
      final ParsedFlutterTemplateType? parsedTemplate = ParsedFlutterTemplateType.fromCliName(
        templateArgument,
      );
      switch (parsedTemplate) {
        case RemovedFlutterTemplateType():
          throwToolExit(
            'The template ${parsedTemplate.cliName} is no longer available. For '
            'your convenience the former help text is repeated below with context '
            'about the removal and other possible resources:\n\n'
            '${parsedTemplate.helpText}',
          );
        case FlutterTemplateType():
          template = parsedTemplate;
        case null:
          break;
      }
    }
    // If the project directory exists and isn't empty, then try to determine the template
    // type from the project directory.
    if (projectDir.existsSync() && projectDir.listSync().isNotEmpty) {
      detectedProjectType = determineTemplateType();
      if (detectedProjectType == null && metadataExists) {
        // We can only be definitive that this is the wrong type if the .metadata file
        // exists and contains a type that we don't understand, or doesn't contain a type.
        throwToolExit(
          'Sorry, unable to detect the type of project to recreate. '
          'Try creating a fresh project and migrating your existing code to '
          'the new project manually.',
        );
      }
    }
    template ??= detectedProjectType ?? FlutterTemplateType.app;
    if (detectedProjectType != null && template != detectedProjectType && metadataExists) {
      // We can only be definitive that this is the wrong type if the .metadata file
      // exists and contains a type that doesn't match.
      throwToolExit(
        "The requested template type '${template.cliName}' doesn't match the "
        "existing template type of '${detectedProjectType.cliName}'.",
      );
    }
    return template;
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final String? listSamples = stringArg('list-samples');
    if (listSamples != null) {
      // _writeSamplesJson can potentially be long-lived.
      await _writeSamplesJson(listSamples);
      return FlutterCommandResult.success();
    }

    if (argResults!.wasParsed('empty') && argResults!.wasParsed('sample')) {
      throwToolExit('Only one of --empty or --sample may be specified, not both.', exitCode: 2);
    }

    validateOutputDirectoryArg();
    String? sampleCode;
    final String? sampleArgument = stringArg('sample');
    final bool emptyArgument = boolArg('empty');
    final FlutterTemplateType template = _getProjectType(projectDir);
    if (sampleArgument != null) {
      if (template != FlutterTemplateType.app) {
        throwToolExit(
          'Cannot specify --sample with a project type other than '
          '"${FlutterTemplateType.app.cliName}"',
        );
      }
      // Fetch the sample from the server.
      sampleCode = await _fetchSampleFromServer(sampleArgument);
    }
    if (emptyArgument && template != FlutterTemplateType.app) {
      throwToolExit('The --empty flag is only supported for the app template.');
    }

    final bool generateModule = template == FlutterTemplateType.module;
    final bool generateMethodChannelsPlugin = template == FlutterTemplateType.plugin;
    final bool generateFfiPackage = template == FlutterTemplateType.packageFfi;
    final bool generateFfiPlugin = template == FlutterTemplateType.pluginFfi;
    final bool generateFfi = generateFfiPlugin || generateFfiPackage;
    final bool generatePackage = template == FlutterTemplateType.package;

    final List<String> platforms = stringsArg('platforms');
    // `--platforms` does not support module or package.
    if (argResults!.wasParsed('platforms') &&
        (generateModule || generatePackage || generateFfiPackage)) {
      final String template = generateModule ? 'module' : 'package';
      throwToolExit(
        'The "--platforms" argument is not supported in $template template.',
        exitCode: 2,
      );
    } else if (platforms.isEmpty) {
      throwToolExit('Must specify at least one platform using --platforms', exitCode: 2);
    } else if (generateFfiPlugin &&
        argResults!.wasParsed('platforms') &&
        platforms.contains('web')) {
      throwToolExit('The web platform is not supported in plugin_ffi template.', exitCode: 2);
    } else if (generateFfi && argResults!.wasParsed('ios-language')) {
      throwToolExit(
        'The "ios-language" option is not supported with the ${template.cliName} '
        'template: the language will always be C or C++.',
        exitCode: 2,
      );
    } else if (generateFfi && argResults!.wasParsed('android-language')) {
      throwToolExit(
        'The "android-language" option is not supported with the ${template.cliName} '
        'template: the language will always be C or C++.',
        exitCode: 2,
      );
    } else if (argResults!.wasParsed('ios-language')) {
      globals.printWarning(
        'The "ios-language" option is deprecated and will be removed in a future Flutter release.',
      );
      if (stringArg('ios-language') == 'objc') {
        globals.printWarning(
          'Please comment in https://github.com/flutter/flutter/issues/148586 describing your use-case for using Objective-C instead of Swift.',
        );
      }
    }

    final String organization = await getOrganization();

    final bool overwrite = boolArg('overwrite');
    validateProjectDir(overwrite: overwrite);

    if (boolArg('with-driver-test')) {
      globals.printWarning(
        'The "--with-driver-test" argument has been deprecated and will no longer add a flutter '
        'driver template. Instead, learn how to use package:integration_test by '
        'visiting https://pub.dev/packages/integration_test .',
      );
    }

    final String dartSdk = globals.cache.dartSdkBuild;
    final bool includeIos;
    final bool includeAndroid;
    final bool includeWeb;
    final bool includeLinux;
    final bool includeMacos;
    final bool includeWindows;
    if (template == FlutterTemplateType.module) {
      // The module template only supports iOS and Android.
      includeIos = true;
      includeAndroid = true;
      includeWeb = false;
      includeLinux = false;
      includeMacos = false;
      includeWindows = false;
    } else if (template == FlutterTemplateType.package) {
      // The package template does not supports any platform.
      includeIos = false;
      includeAndroid = false;
      includeWeb = false;
      includeLinux = false;
      includeMacos = false;
      includeWindows = false;
    } else {
      includeIos = featureFlags.isIOSEnabled && platforms.contains('ios');
      includeAndroid = featureFlags.isAndroidEnabled && platforms.contains('android');
      includeWeb = featureFlags.isWebEnabled && platforms.contains('web');
      includeLinux = featureFlags.isLinuxEnabled && platforms.contains('linux');
      includeMacos = featureFlags.isMacOSEnabled && platforms.contains('macos');
      includeWindows = featureFlags.isWindowsEnabled && platforms.contains('windows');
    }

    String? developmentTeam;
    if (includeIos) {
      developmentTeam = await getCodeSigningIdentityDevelopmentTeam(
        processManager: globals.processManager,
        platform: globals.platform,
        logger: globals.logger,
        config: globals.config,
        terminal: globals.terminal,
      );
    }

    // The dart project_name is in snake_case, this variable is the Title Case of the Project Name.
    final String titleCaseProjectName = snakeCaseToTitleCase(projectName);

    final Map<String, Object?> templateContext = createTemplateContext(
      organization: organization,
      projectName: projectName,
      titleCaseProjectName: titleCaseProjectName,
      projectDescription: stringArg('description'),
      flutterRoot: flutterRoot,
      withPlatformChannelPluginHook: generateMethodChannelsPlugin,
      withSwiftPackageManager: featureFlags.isSwiftPackageManagerEnabled,
      withFfiPluginHook: generateFfiPlugin,
      withFfiPackage: generateFfiPackage,
      withEmptyMain: emptyArgument,
      androidLanguage: stringArg('android-language'),
      iosLanguage: stringArg('ios-language'),
      iosDevelopmentTeam: developmentTeam,
      ios: includeIos,
      android: includeAndroid,
      web: includeWeb,
      linux: includeLinux,
      macos: includeMacos,
      windows: includeWindows,
      dartSdkVersionBounds: '^$dartSdk',
      implementationTests: boolArg('implementation-tests'),
      agpVersion: gradle.templateAndroidGradlePluginVersion,
      kotlinVersion: gradle.templateKotlinGradlePluginVersion,
      gradleVersion: gradle.templateDefaultGradleVersion,
    );

    final String relativeDirPath = globals.fs.path.relative(projectDirPath);
    final bool creatingNewProject = !projectDir.existsSync() || projectDir.listSync().isEmpty;
    if (creatingNewProject) {
      globals.printStatus('Creating project $relativeDirPath...');
    } else {
      if (sampleCode != null && !overwrite) {
        throwToolExit(
          'Will not overwrite existing project in $relativeDirPath: '
          'must specify --overwrite for samples to overwrite.',
        );
      }
      globals.printStatus('Recreating project $relativeDirPath...');
    }

    final Directory relativeDir = globals.fs.directory(projectDirPath);
    int generatedFileCount = 0;
    final PubContext pubContext;
    switch (template) {
      case FlutterTemplateType.app:
        final bool skipWidgetTestsGeneration = sampleCode != null || emptyArgument;

        generatedFileCount += await generateApp(
          <String>['app', if (!skipWidgetTestsGeneration) 'app_test_widget'],
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
          projectType: template,
        );
        pubContext = PubContext.create;
      case FlutterTemplateType.module:
        generatedFileCount += await _generateModule(
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
        );
        pubContext = PubContext.create;
      case FlutterTemplateType.package:
        generatedFileCount += await _generatePackage(
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
        );
        pubContext = PubContext.createPackage;
      case FlutterTemplateType.plugin:
        generatedFileCount += await _generateMethodChannelPlugin(
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
          projectType: template,
        );
        pubContext = PubContext.createPlugin;
      case FlutterTemplateType.pluginFfi:
        generatedFileCount += await _generateFfiPlugin(
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
          projectType: template,
        );
        pubContext = PubContext.createPlugin;
      case FlutterTemplateType.packageFfi:
        generatedFileCount += await _generateFfiPackage(
          relativeDir,
          templateContext,
          overwrite: overwrite,
          printStatusWhenWriting: !creatingNewProject,
          projectType: template,
        );
        pubContext = PubContext.createPackage;
    }

    if (shouldCallPubGet) {
      final FlutterProject project = FlutterProject.fromDirectory(relativeDir);
      await pub.get(
        context: pubContext,
        project: project,
        offline: offline,
        outputMode: PubOutputMode.summaryOnly,
      );
      // Setting `includeIos` etc to false as with FlutterProjectType.package
      // causes the example sub directory to not get os sub directories.
      // This will lead to `flutter build ios` to fail in the example.
      // TODO(dacoharkes): Uncouple the app and parent project platforms. https://github.com/flutter/flutter/issues/133874
      // Then this if can be removed.
      if (!generateFfiPackage) {
        await project.ensureReadyForPlatformSpecificTooling(
          androidPlatform: includeAndroid,
          iosPlatform: includeIos,
          linuxPlatform: includeLinux,
          macOSPlatform: includeMacos,
          windowsPlatform: includeWindows,
          webPlatform: includeWeb,
        );
      }
    }
    if (sampleCode != null) {
      _applySample(relativeDir, sampleCode);
    }
    globals.printStatus('Wrote $generatedFileCount files.');
    globals.printStatus('\nAll done!');
    final String application =
        '${emptyArgument ? 'empty ' : ''}${sampleCode != null ? 'sample ' : ''}application';
    if (generatePackage) {
      final String relativeMainPath = globals.fs.path.normalize(
        globals.fs.path.join(relativeDirPath, 'lib', '${templateContext['projectName']}.dart'),
      );
      globals.printStatus('Your package code is in $relativeMainPath');
    } else if (generateModule) {
      final String relativeMainPath = globals.fs.path.normalize(
        globals.fs.path.join(relativeDirPath, 'lib', 'main.dart'),
      );
      globals.printStatus('Your module code is in $relativeMainPath.');
    } else if (generateMethodChannelsPlugin || generateFfiPlugin) {
      final String relativePluginPath = globals.fs.path.normalize(
        globals.fs.path.relative(projectDirPath),
      );
      final List<String> requestedPlatforms = _getUserRequestedPlatforms();
      final String platformsString = requestedPlatforms.join(', ');
      _printPluginDirectoryLocationMessage(relativePluginPath, projectName, platformsString);
      if (!creatingNewProject && requestedPlatforms.isNotEmpty) {
        _printPluginUpdatePubspecMessage(relativePluginPath, platformsString);
      } else if (_getSupportedPlatformsInPlugin(projectDir).isEmpty) {
        _printNoPluginMessage();
      }

      final List<String> platformsToWarn = _getPlatformWarningList(requestedPlatforms);
      if (platformsToWarn.isNotEmpty) {
        _printWarningDisabledPlatform(platformsToWarn);
      }
      final String template = generateMethodChannelsPlugin ? 'plugin' : 'plugin_ffi';
      _printPluginAddPlatformMessage(relativePluginPath, template);
    } else {
      // Tell the user the next steps.
      final FlutterProject project = FlutterProject.fromDirectory(
        globals.fs.directory(projectDirPath),
      );
      final FlutterProject app = project.hasExampleApp ? project.example : project;
      final String relativeAppPath = globals.fs.path.normalize(
        globals.fs.path.relative(app.directory.path),
      );
      final String relativeAppMain = globals.fs.path.join(relativeAppPath, 'lib', 'main.dart');
      final List<String> requestedPlatforms = _getUserRequestedPlatforms();

      // Let them know a summary of the state of their tooling.
      globals.printStatus('''
You can find general documentation for Flutter at: https://docs.flutter.dev/
Detailed API documentation is available at: https://api.flutter.dev/
If you prefer video documentation, consider: https://www.youtube.com/c/flutterdev

In order to run your $application, type:

  \$ cd $relativeAppPath
  \$ flutter run

Your $application code is in $relativeAppMain.
''');
      // Show warning if any selected platform is not enabled
      final List<String> platformsToWarn = _getPlatformWarningList(requestedPlatforms);
      if (platformsToWarn.isNotEmpty) {
        _printWarningDisabledPlatform(platformsToWarn);
      }
    }

    // Show warning for Java/AGP or Java/Gradle incompatibility if building for
    // Android and Java version has been detected.
    if (includeAndroid && globals.java?.version != null) {
      _printIncompatibleJavaAgpGradleVersionsWarning(
        javaVersion: versionToParsableString(globals.java?.version)!,
        templateGradleVersion: templateContext['gradleVersion']! as String,
        templateAgpVersion: templateContext['agpVersion']! as String,
        templateAgpVersionForModule: templateContext['agpVersionForModule']! as String,
        projectType: template,
        projectDirPath: projectDirPath,
      );
    }

    return FlutterCommandResult.success();
  }

  Future<int> _generateModule(
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
  }) async {
    int generatedCount = 0;
    final String? description =
        argResults!.wasParsed('description')
            ? stringArg('description')
            : 'A new Flutter module project.';
    templateContext['description'] = description;
    generatedCount += await renderTemplate(
      globals.fs.path.join('module', 'common'),
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );
    return generatedCount;
  }

  Future<int> _generatePackage(
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
  }) async {
    int generatedCount = 0;
    final String? description =
        argResults!.wasParsed('description')
            ? stringArg('description')
            : 'A new Flutter package project.';
    templateContext['description'] = description;
    generatedCount += await renderTemplate(
      'package',
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );
    return generatedCount;
  }

  Future<int> _generateMethodChannelPlugin(
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
    required FlutterTemplateType projectType,
  }) async {
    // Plugins only add a platform if it was requested explicitly by the user.
    if (!argResults!.wasParsed('platforms')) {
      for (final String platform in kAllCreatePlatforms) {
        templateContext[platform] = false;
      }
    }
    final List<String> platformsToAdd = _getSupportedPlatformsFromTemplateContext(templateContext);

    final List<String> existingPlatforms = _getSupportedPlatformsInPlugin(directory);
    for (final String existingPlatform in existingPlatforms) {
      // re-generate files for existing platforms
      templateContext[existingPlatform] = true;
    }

    final bool willAddPlatforms = platformsToAdd.isNotEmpty;
    templateContext['no_platforms'] = !willAddPlatforms;
    int generatedCount = 0;
    final String? description =
        argResults!.wasParsed('description')
            ? stringArg('description')
            : 'A new Flutter plugin project.';
    templateContext['description'] = description;

    final String? projectName = templateContext['projectName'] as String?;
    final List<String> templates = <String>['plugin', 'plugin_shared'];
    if ((templateContext['ios'] == true || templateContext['macos'] == true) &&
        featureFlags.isSwiftPackageManagerEnabled) {
      templates.add('plugin_swift_package_manager');
      templateContext['swiftLibraryName'] = projectName?.replaceAll('_', '-');
      templateContext['swiftToolsVersion'] = minimumSwiftToolchainVersion;
      templateContext['iosSupportedPlatform'] =
          SwiftPackageManager.iosSwiftPackageSupportedPlatform.format();
      templateContext['macosSupportedPlatform'] =
          SwiftPackageManager.macosSwiftPackageSupportedPlatform.format();
    } else {
      templates.add('plugin_cocoapods');
    }

    generatedCount += await renderMerged(
      templates,
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );

    final FlutterProject project = FlutterProject.fromDirectory(directory);
    final bool generateAndroid = templateContext['android'] == true;
    if (generateAndroid) {
      gradle.updateLocalProperties(project: project, requireAndroidSdk: false);
    }

    final String organization =
        templateContext['organization']! as String; // Required to make the context.
    final String? androidPluginIdentifier = templateContext['androidIdentifier'] as String?;
    final String exampleProjectName = '${projectName}_example';
    templateContext['projectName'] = exampleProjectName;
    templateContext['androidIdentifier'] = CreateBase.createAndroidIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['iosIdentifier'] = CreateBase.createUTIIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['macosIdentifier'] = CreateBase.createUTIIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['windowsIdentifier'] = CreateBase.createWindowsIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['description'] = 'Demonstrates how to use the $projectName plugin.';
    templateContext['pluginProjectName'] = projectName;
    templateContext['androidPluginIdentifier'] = androidPluginIdentifier;

    generatedCount += await generateApp(
      <String>['app', 'app_test_widget', 'app_integration_test'],
      project.example.directory,
      templateContext,
      overwrite: overwrite,
      pluginExampleApp: true,
      printStatusWhenWriting: printStatusWhenWriting,
      projectType: projectType,
    );
    return generatedCount;
  }

  Future<int> _generateFfiPlugin(
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
    required FlutterTemplateType projectType,
  }) async {
    // Plugins only add a platform if it was requested explicitly by the user.
    if (!argResults!.wasParsed('platforms')) {
      for (final String platform in kAllCreatePlatforms) {
        templateContext[platform] = false;
      }
    }
    final List<String> platformsToAdd = _getSupportedPlatformsFromTemplateContext(templateContext);

    final List<String> existingPlatforms = _getSupportedPlatformsInPlugin(directory);
    for (final String existingPlatform in existingPlatforms) {
      // re-generate files for existing platforms
      templateContext[existingPlatform] = true;
    }

    final bool willAddPlatforms = platformsToAdd.isNotEmpty;
    templateContext['no_platforms'] = !willAddPlatforms;
    int generatedCount = 0;
    final String? description =
        argResults!.wasParsed('description')
            ? stringArg('description')
            : 'A new Flutter FFI plugin project.';
    templateContext['description'] = description;
    generatedCount += await renderMerged(
      <String>['plugin_ffi', 'plugin_shared'],
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );

    final FlutterProject project = FlutterProject.fromDirectory(directory);
    final bool generateAndroid = templateContext['android'] == true;
    if (generateAndroid) {
      gradle.updateLocalProperties(project: project, requireAndroidSdk: false);
    }

    final String? projectName = templateContext['projectName'] as String?;
    final String organization =
        templateContext['organization']! as String; // Required to make the context.
    final String? androidPluginIdentifier = templateContext['androidIdentifier'] as String?;
    final String exampleProjectName = '${projectName}_example';
    templateContext['projectName'] = exampleProjectName;
    templateContext['androidIdentifier'] = CreateBase.createAndroidIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['iosIdentifier'] = CreateBase.createUTIIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['macosIdentifier'] = CreateBase.createUTIIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['windowsIdentifier'] = CreateBase.createWindowsIdentifier(
      organization,
      exampleProjectName,
    );
    templateContext['description'] = 'Demonstrates how to use the $projectName plugin.';
    templateContext['pluginProjectName'] = projectName;
    templateContext['androidPluginIdentifier'] = androidPluginIdentifier;

    generatedCount += await generateApp(
      <String>['app'],
      project.example.directory,
      templateContext,
      overwrite: overwrite,
      pluginExampleApp: true,
      printStatusWhenWriting: printStatusWhenWriting,
      projectType: projectType,
    );
    return generatedCount;
  }

  Future<int> _generateFfiPackage(
    Directory directory,
    Map<String, Object?> templateContext, {
    bool overwrite = false,
    bool printStatusWhenWriting = true,
    required FlutterTemplateType projectType,
  }) async {
    int generatedCount = 0;
    final String? description =
        argResults!.wasParsed('description')
            ? stringArg('description')
            : 'A new Dart FFI package project.';
    templateContext['description'] = description;
    generatedCount += await renderMerged(
      <String>['package_ffi'],
      directory,
      templateContext,
      overwrite: overwrite,
      printStatusWhenWriting: printStatusWhenWriting,
    );

    final FlutterProject project = FlutterProject.fromDirectory(directory);

    final String? projectName = templateContext['projectName'] as String?;
    final String exampleProjectName = '${projectName}_example';
    templateContext['projectName'] = exampleProjectName;
    templateContext['description'] = 'Demonstrates how to use the $projectName package.';
    templateContext['pluginProjectName'] = projectName;

    generatedCount += await generateApp(
      <String>['app'],
      project.example.directory,
      templateContext,
      overwrite: overwrite,
      pluginExampleApp: true,
      printStatusWhenWriting: printStatusWhenWriting,
      projectType: projectType,
    );
    return generatedCount;
  }

  // Takes an application template and replaces the main.dart with one from the
  // documentation website in sampleCode. Returns the difference in the number
  // of files after applying the sample, since it also deletes the application's
  // test directory (since the template's test doesn't apply to the sample).
  void _applySample(Directory directory, String sampleCode) {
    final File mainDartFile = directory.childDirectory('lib').childFile('main.dart');
    mainDartFile.createSync(recursive: true);
    mainDartFile.writeAsStringSync(sampleCode);
  }

  List<String> _getSupportedPlatformsFromTemplateContext(Map<String, Object?> templateContext) {
    return <String>[
      for (final String platform in kAllCreatePlatforms)
        if (templateContext[platform] == true) platform,
    ];
  }

  // Returns a list of platforms that are explicitly requested by user via `--platforms`.
  List<String> _getUserRequestedPlatforms() {
    if (!argResults!.wasParsed('platforms')) {
      return <String>[];
    }
    return stringsArg('platforms');
  }
}

// Determine what platforms are supported based on generated files.
List<String> _getSupportedPlatformsInPlugin(Directory projectDir) {
  final String pubspecPath = globals.fs.path.join(projectDir.absolute.path, 'pubspec.yaml');
  final FlutterManifest? manifest = FlutterManifest.createFromPath(
    pubspecPath,
    fileSystem: globals.fs,
    logger: globals.logger,
  );
  final Map<String, Object?>? validSupportedPlatforms = manifest?.validSupportedPlatforms;
  final List<String> platforms =
      validSupportedPlatforms == null ? <String>[] : validSupportedPlatforms.keys.toList();
  return platforms;
}

void _printPluginDirectoryLocationMessage(
  String pluginPath,
  String projectName,
  String platformsString,
) {
  final String relativePluginMain = globals.fs.path.join(pluginPath, 'lib', '$projectName.dart');
  final String relativeExampleMain = globals.fs.path.join(
    pluginPath,
    'example',
    'lib',
    'main.dart',
  );
  globals.printStatus('''

Your plugin code is in $relativePluginMain.

Your example app code is in $relativeExampleMain.

''');
  if (platformsString.isNotEmpty) {
    globals.printStatus('''
Host platform code is in the $platformsString directories under $pluginPath.
To edit platform code in an IDE see https://flutter.dev/to/edit-plugins.

    ''');
  }
}

void _printPluginUpdatePubspecMessage(String pluginPath, String platformsString) {
  globals.printStatus(
    '''
You need to update $pluginPath/pubspec.yaml to support $platformsString.
''',
    emphasis: true,
    color: TerminalColor.red,
  );
}

void _printNoPluginMessage() {
  globals.printError('''
You've created a plugin project that doesn't yet support any platforms.
''');
}

void _printPluginAddPlatformMessage(String pluginPath, String template) {
  globals.printStatus('''
To add platforms, run `flutter create -t $template --platforms <platforms> .` under $pluginPath.
For more information, see https://flutter.dev/to/pubspec-plugin-platforms.

''');
}

// returns a list disabled, but requested platforms
List<String> _getPlatformWarningList(List<String> requestedPlatforms) {
  final List<String> platformsToWarn = <String>[
    if (requestedPlatforms.contains('web') && !featureFlags.isWebEnabled) 'web',
    if (requestedPlatforms.contains('macos') && !featureFlags.isMacOSEnabled) 'macos',
    if (requestedPlatforms.contains('windows') && !featureFlags.isWindowsEnabled) 'windows',
    if (requestedPlatforms.contains('linux') && !featureFlags.isLinuxEnabled) 'linux',
  ];

  return platformsToWarn;
}

void _printWarningDisabledPlatform(List<String> platforms) {
  final List<String> desktop = <String>[];
  final List<String> web = <String>[];

  for (final String platform in platforms) {
    switch (platform) {
      case 'web':
        web.add(platform);
      case 'macos' || 'windows' || 'linux':
        desktop.add(platform);
    }
  }

  if (desktop.isNotEmpty) {
    final String platforms = desktop.length > 1 ? 'platforms' : 'platform';
    final String verb = desktop.length > 1 ? 'are' : 'is';

    globals.printStatus('''
The desktop $platforms: ${desktop.join(', ')} $verb currently not supported on your local environment.
For more details, see: https://flutter.dev/to/add-desktop-support
''');
  }
  if (web.isNotEmpty) {
    globals.printStatus('''
The web is currently not supported on your local environment.
For more details, see: https://flutter.dev/to/add-web-support
''');
  }
}

// Prints a warning if the specified Java version conflicts with either the
// template Gradle or AGP version.
//
// Assumes the specified templateGradleVersion and templateAgpVersion are
// compatible, meaning that the Java version may only conflict with one of the
// template Gradle or AGP versions.
void _printIncompatibleJavaAgpGradleVersionsWarning({
  required String javaVersion,
  required String templateGradleVersion,
  required String templateAgpVersion,
  required String templateAgpVersionForModule,
  required FlutterTemplateType projectType,
  required String projectDirPath,
}) {
  // Determine if the Java version specified conflicts with the template Gradle or AGP version.
  final bool javaGradleVersionsCompatible = gradle.validateJavaAndGradle(
    globals.logger,
    javaV: javaVersion,
    gradleV: templateGradleVersion,
  );
  bool javaAgpVersionsCompatible = gradle.validateJavaAndAgp(
    globals.logger,
    javaV: javaVersion,
    agpV: templateAgpVersion,
  );
  String relevantTemplateAgpVersion = templateAgpVersion;

  if (projectType == FlutterTemplateType.module &&
      Version.parse(templateAgpVersion)! < Version.parse(templateAgpVersionForModule)!) {
    // If a module is being created, make sure to check for Java/AGP compatibility between the highest used version of AGP in the module template.
    javaAgpVersionsCompatible = gradle.validateJavaAndAgp(
      globals.logger,
      javaV: javaVersion,
      agpV: templateAgpVersionForModule,
    );
    relevantTemplateAgpVersion = templateAgpVersionForModule;
  }

  if (javaGradleVersionsCompatible && javaAgpVersionsCompatible) {
    return;
  }

  // Determine header of warning with recommended fix of re-configuring Java version.
  final String incompatibleVersionsAndRecommendedOptionMessage =
      getIncompatibleJavaGradleAgpMessageHeader(
        javaGradleVersionsCompatible,
        templateGradleVersion,
        relevantTemplateAgpVersion,
        projectType.cliName,
      );

  if (!javaGradleVersionsCompatible) {
    if (projectType == FlutterTemplateType.plugin || projectType == FlutterTemplateType.pluginFfi) {
      // Only impacted files could be in sample code.
      return;
    }

    // Gradle template version incompatible with Java version.
    final gradle.JavaGradleCompat? validCompatibleGradleVersionRange = gradle
        .getValidGradleVersionRangeForJavaVersion(globals.logger, javaV: javaVersion);
    final String compatibleGradleVersionMessage =
        validCompatibleGradleVersionRange == null
            ? ''
            : ' (compatible Gradle version range: ${validCompatibleGradleVersionRange.gradleMin} - ${validCompatibleGradleVersionRange.gradleMax})';

    globals.printWarning('''
$incompatibleVersionsAndRecommendedOptionMessage

Alternatively, to continue using your configured Java version, update the Gradle
version specified in the following file to a compatible Gradle version$compatibleGradleVersionMessage:
${_getGradleWrapperPropertiesFilePath(projectType, projectDirPath)}

You may also update the Gradle version used by running
`./gradlew wrapper --gradle-version=<COMPATIBLE_GRADLE_VERSION>`.

See
https://docs.gradle.org/current/userguide/compatibility.html#java for details
on compatible Java/Gradle versions, and see
https://docs.gradle.org/current/userguide/gradle_wrapper.html#sec:upgrading_wrapper
for more details on using the Gradle Wrapper command to update the Gradle version
used.
''', emphasis: true);
    return;
  }

  // AGP template version incompatible with Java version.
  final gradle.JavaAgpCompat? minimumCompatibleAgpVersion = gradle
      .getMinimumAgpVersionForJavaVersion(globals.logger, javaV: javaVersion);
  final String compatibleAgpVersionMessage =
      minimumCompatibleAgpVersion == null
          ? ''
          : ' (minimum compatible AGP version: ${minimumCompatibleAgpVersion.agpMin})';
  final String gradleBuildFilePaths =
      '    ${_getBuildGradleConfigurationFilePaths(projectType, projectDirPath)!.join('\n    - ')}';

  globals.printWarning('''
$incompatibleVersionsAndRecommendedOptionMessage

Alternatively, to continue using your current Java version, update the AGP
version in the following file(s) to a compatible version$compatibleAgpVersionMessage:
$gradleBuildFilePaths

For details on compatible Java and AGP versions, see
https://developer.android.com/build/releases/gradle-plugin
''', emphasis: true);
}

// Returns incompatible Java/template Gradle/template AGP message header based
// on incompatibility and project type.
@visibleForTesting
String getIncompatibleJavaGradleAgpMessageHeader(
  bool javaGradleVersionsCompatible,
  String templateGradleVersion,
  String templateAgpVersion,
  String projectType,
) {
  final String incompatibleDependency =
      javaGradleVersionsCompatible ? 'Android Gradle Plugin (AGP)' : 'Gradle';
  final String incompatibleDependencyVersion =
      javaGradleVersionsCompatible
          ? 'AGP version $templateAgpVersion'
          : 'Gradle version $templateGradleVersion';
  final VersionRange validJavaRange = gradle.getJavaVersionFor(
    gradleV: templateGradleVersion,
    agpV: templateAgpVersion,
  );
  // validJavaRange should have non-null versionMin and versionMax since it based on our template AGP and Gradle versions.
  final String validJavaRangeMessage =
      '(Java ${validJavaRange.versionMin!} <= compatible Java version < Java ${validJavaRange.versionMax!})';

  return '''
The configured version of Java detected may conflict with the $incompatibleDependency version in your new Flutter $projectType.

To keep the default AGP version $incompatibleDependencyVersion, download a compatible Java version
(Java 17 <= $validJavaRangeMessage Java version < Java 21). Configure this Java version
globally for Flutter by running:

  flutter config --jdk-dir=<JDK_DIRECTORY>
''';
}

// Returns path of the gradle-wrapper.properties file for the specified
// generated project type.
String? _getGradleWrapperPropertiesFilePath(
  FlutterTemplateType projectType,
  String projectDirPath,
) {
  String gradleWrapperPropertiesFilePath = '';
  switch (projectType) {
    case FlutterTemplateType.app:
      gradleWrapperPropertiesFilePath = globals.fs.path.join(
        projectDirPath,
        'android/gradle/wrapper/gradle-wrapper.properties',
      );
    case FlutterTemplateType.module:
      gradleWrapperPropertiesFilePath = globals.fs.path.join(
        projectDirPath,
        '.android/gradle/wrapper/gradle-wrapper.properties',
      );
    case FlutterTemplateType.plugin:
    case FlutterTemplateType.pluginFfi:
    case FlutterTemplateType.package:
    case FlutterTemplateType.packageFfi:
      // TODO(camsim99): Add relevant file path for packageFfi when Android is supported.
      // No gradle-wrapper.properties files not part of sample code that
      // can be determined.
      return null;
  }
  return gradleWrapperPropertiesFilePath;
}

// Returns the path(s) of the build.gradle file(s) for the specified generated
// project type.
List<String>? _getBuildGradleConfigurationFilePaths(
  FlutterTemplateType projectType,
  String projectDirPath,
) {
  final List<String> buildGradleConfigurationFilePaths = <String>[];
  switch (projectType) {
    case FlutterTemplateType.app:
    case FlutterTemplateType.pluginFfi:
      buildGradleConfigurationFilePaths.add(
        globals.fs.path.join(projectDirPath, 'android/build.gradle'),
      );
    case FlutterTemplateType.module:
      const String moduleBuildGradleFilePath = '.android/build.gradle';
      const String moduleAppBuildGradleFlePath = '.android/app/build.gradle';
      const String moduleFlutterBuildGradleFilePath = '.android/Flutter/build.gradle';
      buildGradleConfigurationFilePaths.addAll(<String>[
        globals.fs.path.join(projectDirPath, moduleBuildGradleFilePath),
        globals.fs.path.join(projectDirPath, moduleAppBuildGradleFlePath),
        globals.fs.path.join(projectDirPath, moduleFlutterBuildGradleFilePath),
      ]);
    case FlutterTemplateType.plugin:
      buildGradleConfigurationFilePaths.add(
        globals.fs.path.join(projectDirPath, 'android/app/build.gradle'),
      );
    case FlutterTemplateType.package:
    case FlutterTemplateType.packageFfi:
      // TODO(camsim99): Add any relevant file paths for packageFfi when Android is supported.
      // No build.gradle file because there is no platform-specific implementation.
      return null;
  }
  return buildGradleConfigurationFilePaths;
}
