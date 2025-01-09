// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:crypto/crypto.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:xml/xml.dart';

import '../artifacts.dart';
import '../base/analyze_size.dart';
import '../base/common.dart';
import '../base/deferred_component.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/net.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/project_migrator.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../cache.dart';
import '../convert.dart';
import '../flutter_manifest.dart';
import '../globals.dart' as globals;
import '../project.dart';
import 'android_builder.dart';
import 'android_studio.dart';
import 'gradle_errors.dart';
import 'gradle_utils.dart';
import 'java.dart';
import 'migrations/android_studio_java_gradle_conflict_migration.dart';
import 'migrations/cmake_android_16k_pages_migration.dart';
import 'migrations/min_sdk_version_migration.dart';
import 'migrations/multidex_removal_migration.dart';
import 'migrations/top_level_gradle_build_file_migration.dart';

/// The regex to grab variant names from printBuildVariants gradle task
///
/// The task is defined in flutter/packages/flutter_tools/gradle/src/main/groovy/flutter.groovy
///
/// The expected output from the task should be similar to:
///
/// BuildVariant: debug
/// BuildVariant: release
/// BuildVariant: profile
final RegExp _kBuildVariantRegex = RegExp('^BuildVariant: (?<$_kBuildVariantRegexGroupName>.*)\$');
const String _kBuildVariantRegexGroupName = 'variant';
const String _kBuildVariantTaskName = 'printBuildVariants';

typedef _OutputParser = void Function(String line);

String _getOutputAppLinkSettingsTaskFor(String buildVariant) {
  return _taskForBuildVariant('output', buildVariant, 'AppLinkSettings');
}

/// The directory where the APK artifact is generated.
Directory getApkDirectory(FlutterProject project) {
  return project.isModule
      ? project.android.buildDirectory
          .childDirectory('host')
          .childDirectory('outputs')
          .childDirectory('apk')
      : project.android.buildDirectory
          .childDirectory('app')
          .childDirectory('outputs')
          .childDirectory('flutter-apk');
}

/// The directory where the app bundle artifact is generated.
@visibleForTesting
Directory getBundleDirectory(FlutterProject project) {
  return project.isModule
      ? project.android.buildDirectory
          .childDirectory('host')
          .childDirectory('outputs')
          .childDirectory('bundle')
      : project.android.buildDirectory
          .childDirectory('app')
          .childDirectory('outputs')
          .childDirectory('bundle');
}

/// The directory where the repo is generated.
/// Only applicable to AARs.
Directory getRepoDirectory(Directory buildDirectory) {
  return buildDirectory.childDirectory('outputs').childDirectory('repo');
}

/// Returns the name of Gradle task that starts with [prefix].
String _taskFor(String prefix, BuildInfo buildInfo) {
  final String buildType = camelCase(buildInfo.modeName);
  final String productFlavor = buildInfo.flavor ?? '';
  return _taskForBuildVariant(prefix, '$productFlavor${sentenceCase(buildType)}');
}

String _taskForBuildVariant(String prefix, String buildVariant, [String suffix = '']) {
  return '$prefix${sentenceCase(buildVariant)}$suffix';
}

/// Returns the task to build an APK.
@visibleForTesting
String getAssembleTaskFor(BuildInfo buildInfo) {
  return _taskFor('assemble', buildInfo);
}

/// Returns the task to build an AAB.
@visibleForTesting
String getBundleTaskFor(BuildInfo buildInfo) {
  return _taskFor('bundle', buildInfo);
}

/// Returns the task to build an AAR.
@visibleForTesting
String getAarTaskFor(BuildInfo buildInfo) {
  return _taskFor('assembleAar', buildInfo);
}

@visibleForTesting
const String androidX86DeprecationWarning =
    'Support for Android x86 targets will be removed in the next stable release after 3.27. '
    'See https://github.com/flutter/flutter/issues/157543 for details.';

/// Returns the output APK file names for a given [AndroidBuildInfo].
///
/// For example, when [splitPerAbi] is true, multiple APKs are created.
Iterable<String> _apkFilesFor(AndroidBuildInfo androidBuildInfo) {
  final String buildType = camelCase(androidBuildInfo.buildInfo.modeName);
  final String productFlavor = androidBuildInfo.buildInfo.lowerCasedFlavor ?? '';
  final String flavorString = productFlavor.isEmpty ? '' : '-$productFlavor';
  if (androidBuildInfo.splitPerAbi) {
    return androidBuildInfo.targetArchs.map<String>((AndroidArch arch) {
      final String abi = arch.archName;
      return 'app$flavorString-$abi-$buildType.apk';
    });
  }
  return <String>['app$flavorString-$buildType.apk'];
}

// The maximum time to wait before the tool retries a Gradle build.
const Duration kMaxRetryTime = Duration(seconds: 10);

/// An implementation of the [AndroidBuilder] that delegates to gradle.
class AndroidGradleBuilder implements AndroidBuilder {
  AndroidGradleBuilder({
    required Java? java,
    required Logger logger,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required Artifacts artifacts,
    required Analytics analytics,
    required GradleUtils gradleUtils,
    required Platform platform,
    required AndroidStudio? androidStudio,
  }) : _java = java,
       _logger = logger,
       _fileSystem = fileSystem,
       _artifacts = artifacts,
       _analytics = analytics,
       _gradleUtils = gradleUtils,
       _androidStudio = androidStudio,
       _fileSystemUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform),
       _processUtils = ProcessUtils(logger: logger, processManager: processManager);

  final Java? _java;
  final Logger _logger;
  final ProcessUtils _processUtils;
  final FileSystem _fileSystem;
  final Artifacts _artifacts;
  final Analytics _analytics;
  final GradleUtils _gradleUtils;
  final FileSystemUtils _fileSystemUtils;
  final AndroidStudio? _androidStudio;

  /// Builds the AAR and POM files for the current Flutter module or plugin.
  @override
  Future<void> buildAar({
    required FlutterProject project,
    required Set<AndroidBuildInfo> androidBuildInfo,
    required String target,
    String? outputDirectoryPath,
    required String buildNumber,
  }) async {
    Directory outputDirectory = _fileSystem.directory(
      outputDirectoryPath ?? project.android.buildDirectory,
    );
    if (project.isModule) {
      // Module projects artifacts are located in `build/host`.
      outputDirectory = outputDirectory.childDirectory('host');
    }

    final bool containsX86Targets =
        androidBuildInfo.where((AndroidBuildInfo info) => info.containsX86Target).isNotEmpty;
    if (containsX86Targets) {
      _logger.printWarning(androidX86DeprecationWarning);
    }
    for (final AndroidBuildInfo androidBuildInfo in androidBuildInfo) {
      await buildGradleAar(
        project: project,
        androidBuildInfo: androidBuildInfo,
        target: target,
        outputDirectory: outputDirectory,
        buildNumber: buildNumber,
      );
    }
    printHowToConsumeAar(
      buildModes:
          androidBuildInfo.map<String>((AndroidBuildInfo androidBuildInfo) {
            return androidBuildInfo.buildInfo.modeName;
          }).toSet(),
      androidPackage: project.manifest.androidPackage,
      repoDirectory: getRepoDirectory(outputDirectory),
      buildNumber: buildNumber,
      logger: _logger,
      fileSystem: _fileSystem,
    );
  }

  /// Builds the APK.
  @override
  Future<void> buildApk({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    bool configOnly = false,
  }) async {
    await buildGradleApp(
      project: project,
      androidBuildInfo: androidBuildInfo,
      target: target,
      isBuildingBundle: false,
      localGradleErrors: gradleErrors,
      configOnly: configOnly,
      maxRetries: 1,
    );
  }

  /// Builds the App Bundle.
  @override
  Future<void> buildAab({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    bool validateDeferredComponents = true,
    bool deferredComponentsEnabled = false,
    bool configOnly = false,
  }) async {
    await buildGradleApp(
      project: project,
      androidBuildInfo: androidBuildInfo,
      target: target,
      isBuildingBundle: true,
      localGradleErrors: gradleErrors,
      validateDeferredComponents: validateDeferredComponents,
      deferredComponentsEnabled: deferredComponentsEnabled,
      configOnly: configOnly,
      maxRetries: 1,
    );
  }

  Future<int> _runGradleTask(
    String taskName, {
    List<String> options = const <String>[],
    required FlutterProject project,
    required List<GradleHandledError> localGradleErrors,
    required String gradleExecutablePath,
    int retry = 0,
    VoidCallback? preRunTask,
    VoidCallback? postRunTask,
    int? maxRetries,
    _OutputParser? outputParser,
  }) async {
    final bool usesAndroidX = isAppUsingAndroidX(project.android.hostAppGradleRoot);
    if (usesAndroidX) {
      _analytics.send(Event.flutterBuildInfo(label: 'app-using-android-x', buildType: 'gradle'));
    } else if (!usesAndroidX) {
      _analytics.send(
        Event.flutterBuildInfo(label: 'app-not-using-android-x', buildType: 'gradle'),
      );

      _logger.printStatus(
        "${_logger.terminal.warningMark} Your app isn't using AndroidX.",
        emphasis: true,
      );
      _logger.printStatus(
        'To avoid potential build failures, you can quickly migrate your app '
        'by following the steps on https://goo.gl/CP92wY .',
        indent: 4,
      );
    }

    GradleHandledError? detectedGradleError;
    String? detectedGradleErrorLine;
    String? consumeLog(String line) {
      if (outputParser != null) {
        outputParser(line);
      }
      // The log lines that trigger incompatibleKotlinVersionHandler don't
      // always indicate an error, and there are times that that handler
      // covers up a more important error handler. Uniquely set it to be
      // the lowest priority handler by allowing it to be overridden.
      if (detectedGradleError != null && detectedGradleError != incompatibleKotlinVersionHandler) {
        // Pipe stdout/stderr from Gradle.
        return line;
      }
      for (final GradleHandledError gradleError in localGradleErrors) {
        if (gradleError.test(line)) {
          detectedGradleErrorLine = line;
          detectedGradleError = gradleError;
          // The first error match wins.
          break;
        }
      }
      // Pipe stdout/stderr from Gradle.
      return line;
    }

    final Status status = _logger.startProgress("Running Gradle task '$taskName'...");
    final List<String> command = <String>[
      gradleExecutablePath,
      ...options, // suppresses gradle output.
      taskName,
    ];
    preRunTask?.call();

    int exitCode = 1;
    try {
      exitCode = await _processUtils.stream(
        command,
        workingDirectory: project.android.hostAppGradleRoot.path,
        allowReentrantFlutter: true,
        environment: _java?.environment,
        mapFunction: consumeLog,
      );
    } on ProcessException catch (exception) {
      consumeLog(exception.toString());
      // Rethrow the exception if the error isn't handled by any of the
      // `localGradleErrors`.
      if (detectedGradleError == null) {
        rethrow;
      }
    } finally {
      status.stop();
    }
    postRunTask?.call();
    if (exitCode != 0) {
      if (detectedGradleError == null) {
        _analytics.send(
          Event.flutterBuildInfo(label: 'gradle-unknown-failure', buildType: 'gradle'),
        );
        return exitCode;
      }
      final GradleBuildStatus status = await detectedGradleError!.handler(
        line: detectedGradleErrorLine!,
        project: project,
        usesAndroidX: usesAndroidX,
      );

      if (maxRetries == null || retry < maxRetries) {
        switch (status) {
          case GradleBuildStatus.retry:
            // Use binary exponential backoff before retriggering the build.
            // The expected wait times are: 100ms, 200ms, 400ms, and so on...
            final int waitTime = min(pow(2, retry).toInt() * 100, kMaxRetryTime.inMicroseconds);
            retry += 1;
            _logger.printStatus('Retrying Gradle Build: #$retry, wait time: ${waitTime}ms');
            await Future<void>.delayed(Duration(milliseconds: waitTime));
            exitCode = await _runGradleTask(
              taskName,
              options: options,
              preRunTask: preRunTask,
              postRunTask: postRunTask,
              localGradleErrors: localGradleErrors,
              gradleExecutablePath: gradleExecutablePath,
              retry: retry,
              project: project,
              maxRetries: maxRetries,
            );
            if (exitCode == 0) {
              final String successEventLabel = 'gradle-${detectedGradleError!.eventLabel}-success';
              _analytics.send(
                Event.flutterBuildInfo(label: successEventLabel, buildType: 'gradle'),
              );
              return exitCode;
            }
          case GradleBuildStatus.exit:
          // Continue and throw tool exit.
        }
      }
      final String usageLabel = 'gradle-${detectedGradleError!.eventLabel}-failure';
      _analytics.send(Event.flutterBuildInfo(label: usageLabel, buildType: 'gradle'));
    }

    return exitCode;
  }

  /// Builds an app.
  ///
  /// * [project] is typically [FlutterProject.current()].
  /// * [androidBuildInfo] is the build configuration.
  /// * [target] is the target dart entry point. Typically, `lib/main.dart`.
  /// * If [isBuildingBundle] is `true`, then the output artifact is an `*.aab`,
  ///   otherwise the output artifact is an `*.apk`.
  /// * [maxRetries] If not `null`, this is the max number of build retries in case a retry is triggered.
  Future<void> buildGradleApp({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    required bool isBuildingBundle,
    required List<GradleHandledError> localGradleErrors,
    required bool configOnly,
    bool validateDeferredComponents = true,
    bool deferredComponentsEnabled = false,
    int retry = 0,
    @visibleForTesting int? maxRetries,
  }) async {
    if (androidBuildInfo.containsX86Target) {
      _logger.printWarning(androidX86DeprecationWarning);
    }
    if (!project.android.isSupportedVersion) {
      _exitWithUnsupportedProjectMessage(_logger.terminal, _analytics);
    }

    final List<ProjectMigrator> migrators = <ProjectMigrator>[
      TopLevelGradleBuildFileMigration(project.android, _logger),
      AndroidStudioJavaGradleConflictMigration(
        _logger,
        project: project.android,
        androidStudio: _androidStudio,
        java: globals.java,
      ),
      MinSdkVersionMigration(project.android, _logger),
      MultidexRemovalMigration(project.android, _logger),
      CmakeAndroid16kPagesMigration(project.android, _logger),
    ];

    final ProjectMigration migration = ProjectMigration(migrators);
    await migration.run();

    // The default Gradle script reads the version name and number
    // from the local.properties file.
    updateLocalProperties(project: project, buildInfo: androidBuildInfo.buildInfo);

    final List<String> options = <String>[];

    final String gradleExecutablePath = _gradleUtils.getExecutable(project);

    // All automatically created files should exist.
    if (configOnly) {
      _logger.printStatus('Config complete.');
      return;
    }

    // Assembly work starts here.
    final BuildInfo buildInfo = androidBuildInfo.buildInfo;
    final String assembleTask =
        isBuildingBundle ? getBundleTaskFor(buildInfo) : getAssembleTaskFor(buildInfo);

    if (_logger.isVerbose) {
      options.add('--full-stacktrace');
      options.add('--info');
      options.add('-Pverbose=true');
    } else {
      options.add('-q');
    }
    if (!buildInfo.androidGradleDaemon) {
      options.add('--no-daemon');
    }
    if (buildInfo.androidSkipBuildDependencyValidation) {
      options.add('-PskipDependencyChecks=true');
    }
    final LocalEngineInfo? localEngineInfo = _artifacts.localEngineInfo;
    if (localEngineInfo != null) {
      final Directory localEngineRepo = _getLocalEngineRepo(
        engineOutPath: localEngineInfo.targetOutPath,
        androidBuildInfo: androidBuildInfo,
        fileSystem: _fileSystem,
      );
      _logger.printTrace(
        'Using local engine: ${localEngineInfo.targetOutPath}\n'
        'Local Maven repo: ${localEngineRepo.path}',
      );
      options.add('-Plocal-engine-repo=${localEngineRepo.path}');
      options.add('-Plocal-engine-build-mode=${buildInfo.modeName}');
      options.add('-Plocal-engine-out=${localEngineInfo.targetOutPath}');
      options.add('-Plocal-engine-host-out=${localEngineInfo.hostOutPath}');
      options.add(
        '-Ptarget-platform=${_getTargetPlatformByLocalEnginePath(localEngineInfo.targetOutPath)}',
      );
    } else if (androidBuildInfo.targetArchs.isNotEmpty) {
      final String targetPlatforms = androidBuildInfo.targetArchs
          .map((AndroidArch e) => e.platformName)
          .join(',');
      options.add('-Ptarget-platform=$targetPlatforms');
    }
    options.add('-Ptarget=$target');
    // If using v1 embedding, we want to use FlutterApplication as the base app.
    final String baseApplicationName =
        project.android.getEmbeddingVersion() == AndroidEmbeddingVersion.v2
            ? 'android.app.Application'
            : 'io.flutter.app.FlutterApplication';
    options.add('-Pbase-application-name=$baseApplicationName');
    final List<DeferredComponent>? deferredComponents = project.manifest.deferredComponents;
    if (deferredComponents != null) {
      if (deferredComponentsEnabled) {
        options.add('-Pdeferred-components=true');
        androidBuildInfo.buildInfo.dartDefines.add(
          'validate-deferred-components=$validateDeferredComponents',
        );
      }
      // Pass in deferred components regardless of building split aot to satisfy
      // android dynamic features registry in build.gradle.
      final List<String> componentNames = <String>[];
      for (final DeferredComponent component in deferredComponents) {
        componentNames.add(component.name);
      }
      if (componentNames.isNotEmpty) {
        options.add('-Pdeferred-component-names=${componentNames.join(',')}');
        // Multi-apk applications cannot use shrinking. This is only relevant when using
        // android dynamic feature modules.
        _logger.printStatus(
          'Shrinking has been disabled for this build due to deferred components. Shrinking is '
          'not available for multi-apk applications. This limitation is expected to be removed '
          'when Gradle plugin 4.2+ is available in Flutter.',
          color: TerminalColor.yellow,
        );
        options.add('-Pshrink=false');
      }
    }
    options.addAll(androidBuildInfo.buildInfo.toGradleConfig());
    if (buildInfo.fileSystemRoots.isNotEmpty) {
      options.add('-Pfilesystem-roots=${buildInfo.fileSystemRoots.join('|')}');
    }
    if (buildInfo.fileSystemScheme != null) {
      options.add('-Pfilesystem-scheme=${buildInfo.fileSystemScheme}');
    }
    if (androidBuildInfo.splitPerAbi) {
      options.add('-Psplit-per-abi=true');
    }
    if (androidBuildInfo.fastStart) {
      options.add('-Pfast-start=true');
    }
    late Stopwatch sw;
    final int exitCode = await _runGradleTask(
      assembleTask,
      preRunTask: () {
        sw = Stopwatch()..start();
      },
      postRunTask: () {
        final Duration elapsedDuration = sw.elapsed;
        _analytics.send(
          Event.timing(
            workflow: 'build',
            variableName: 'gradle',
            elapsedMilliseconds: elapsedDuration.inMilliseconds,
          ),
        );
      },
      options: options,
      project: project,
      maxRetries: maxRetries,
      localGradleErrors: localGradleErrors,
      gradleExecutablePath: gradleExecutablePath,
    );

    if (exitCode != 0) {
      throwToolExit(
        'Gradle task $assembleTask failed with exit code $exitCode',
        exitCode: exitCode,
      );
    }

    if (isBuildingBundle) {
      final File bundleFile = findBundleFile(project, buildInfo, _logger, _analytics);
      final String appSize =
          (buildInfo.mode == BuildMode.debug)
              ? '' // Don't display the size when building a debug variant.
              : ' (${getSizeAsPlatformMB(bundleFile.lengthSync())})';

      if (buildInfo.codeSizeDirectory != null) {
        await _performCodeSizeAnalysis('aab', bundleFile, androidBuildInfo);
      }

      _logger.printStatus(
        '${_logger.terminal.successMark} '
        'Built ${_fileSystem.path.relative(bundleFile.path)}$appSize',
        color: TerminalColor.green,
      );
      return;
    }
    // Gradle produced APKs.
    final Iterable<String> apkFilesPaths =
        project.isModule
            ? findApkFilesModule(project, androidBuildInfo, _logger, _analytics)
            : listApkPaths(androidBuildInfo);
    final Directory apkDirectory = getApkDirectory(project);

    // Generate sha1 for every generated APKs.
    for (final File apkFile in apkFilesPaths.map(apkDirectory.childFile)) {
      if (!apkFile.existsSync()) {
        _exitWithExpectedFileNotFound(
          project: project,
          fileExtension: '.apk',
          logger: _logger,
          analytics: _analytics,
        );
      }

      final String filename = apkFile.basename;
      _logger.printTrace('Calculate SHA1: $apkDirectory/$filename');
      final File apkShaFile = apkDirectory.childFile('$filename.sha1');
      apkShaFile.writeAsStringSync(_calculateSha(apkFile));

      final String appSize =
          (buildInfo.mode == BuildMode.debug)
              ? '' // Don't display the size when building a debug variant.
              : ' (${getSizeAsPlatformMB(apkFile.lengthSync())})';
      _logger.printStatus(
        '${_logger.terminal.successMark} '
        'Built ${_fileSystem.path.relative(apkFile.path)}$appSize',
        color: TerminalColor.green,
      );

      if (buildInfo.codeSizeDirectory != null) {
        await _performCodeSizeAnalysis('apk', apkFile, androidBuildInfo);
      }
    }
  }

  Future<void> _performCodeSizeAnalysis(
    String kind,
    File zipFile,
    AndroidBuildInfo androidBuildInfo,
  ) async {
    final SizeAnalyzer sizeAnalyzer = SizeAnalyzer(
      fileSystem: _fileSystem,
      logger: _logger,
      analytics: _analytics,
    );
    final String archName = androidBuildInfo.targetArchs.single.archName;
    final BuildInfo buildInfo = androidBuildInfo.buildInfo;
    final File aotSnapshot = _fileSystem
        .directory(buildInfo.codeSizeDirectory)
        .childFile('snapshot.$archName.json');
    final File precompilerTrace = _fileSystem
        .directory(buildInfo.codeSizeDirectory)
        .childFile('trace.$archName.json');
    final Map<String, Object?> output = await sizeAnalyzer.analyzeZipSizeAndAotSnapshot(
      zipFile: zipFile,
      aotSnapshot: aotSnapshot,
      precompilerTrace: precompilerTrace,
      kind: kind,
    );
    final File outputFile = _fileSystemUtils.getUniqueFile(
      _fileSystem.directory(_fileSystemUtils.homeDirPath).childDirectory('.flutter-devtools'),
      '$kind-code-size-analysis',
      'json',
    )..writeAsStringSync(jsonEncode(output));
    // This message is used as a sentinel in analyze_apk_size_test.dart
    _logger.printStatus(
      'A summary of your ${kind.toUpperCase()} analysis can be found at: ${outputFile.path}',
    );

    _logger.printStatus(
      '\nTo analyze your app size in Dart DevTools, run the following command:\n'
      'dart devtools --appSizeBase=${outputFile.path}',
    );
  }

  /// Builds AAR and POM files.
  ///
  /// * [project] is typically [FlutterProject.current()].
  /// * [androidBuildInfo] is the build configuration.
  /// * [outputDir] is the destination of the artifacts,
  /// * [buildNumber] is the build number of the output aar,
  Future<void> buildGradleAar({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    required Directory outputDirectory,
    required String buildNumber,
  }) async {
    final FlutterManifest manifest = project.manifest;
    if (!manifest.isModule) {
      throwToolExit('AARs can only be built for module projects.');
    }

    final BuildInfo buildInfo = androidBuildInfo.buildInfo;
    final String aarTask = getAarTaskFor(buildInfo);
    final Status status = _logger.startProgress("Running Gradle task '$aarTask'...");

    final String flutterRoot = _fileSystem.path.absolute(Cache.flutterRoot!);
    final String initScript = _fileSystem.path.join(
      flutterRoot,
      'packages',
      'flutter_tools',
      'gradle',
      'aar_init_script.gradle',
    );
    final List<String> command = <String>[
      _gradleUtils.getExecutable(project),
      '-I=$initScript',
      '-Pflutter-root=$flutterRoot',
      '-Poutput-dir=${outputDirectory.path}',
      '-Pis-plugin=${manifest.isPlugin}',
      '-PbuildNumber=$buildNumber',
    ];
    if (_logger.isVerbose) {
      command.add('--full-stacktrace');
      command.add('--info');
      command.add('-Pverbose=true');
    } else {
      command.add('-q');
    }
    if (!buildInfo.androidGradleDaemon) {
      command.add('--no-daemon');
    }

    if (target.isNotEmpty) {
      command.add('-Ptarget=$target');
    }
    command.addAll(androidBuildInfo.buildInfo.toGradleConfig());
    if (buildInfo.dartObfuscation && buildInfo.mode != BuildMode.release) {
      _logger.printStatus(
        'Dart obfuscation is not supported in ${sentenceCase(buildInfo.friendlyModeName)}'
        ' mode, building as un-obfuscated.',
      );
    }

    final LocalEngineInfo? localEngineInfo = _artifacts.localEngineInfo;
    if (localEngineInfo != null) {
      final Directory localEngineRepo = _getLocalEngineRepo(
        engineOutPath: localEngineInfo.targetOutPath,
        androidBuildInfo: androidBuildInfo,
        fileSystem: _fileSystem,
      );
      _logger.printTrace(
        'Using local engine: ${localEngineInfo.targetOutPath}\n'
        'Local Maven repo: ${localEngineRepo.path}',
      );
      command.add('-Plocal-engine-repo=${localEngineRepo.path}');
      command.add('-Plocal-engine-build-mode=${buildInfo.modeName}');
      command.add('-Plocal-engine-out=${localEngineInfo.targetOutPath}');
      command.add('-Plocal-engine-host-out=${localEngineInfo.hostOutPath}');

      // Copy the local engine repo in the output directory.
      try {
        copyDirectory(localEngineRepo, getRepoDirectory(outputDirectory));
      } on FileSystemException catch (error, st) {
        throwToolExit(
          'Failed to copy the local engine ${localEngineRepo.path} repo '
          'in ${outputDirectory.path}: $error, $st',
        );
      }
      command.add(
        '-Ptarget-platform=${_getTargetPlatformByLocalEnginePath(localEngineInfo.targetOutPath)}',
      );
    } else if (androidBuildInfo.targetArchs.isNotEmpty) {
      final String targetPlatforms = androidBuildInfo.targetArchs
          .map((AndroidArch e) => e.platformName)
          .join(',');
      command.add('-Ptarget-platform=$targetPlatforms');
    }

    command.add(aarTask);

    final Stopwatch sw = Stopwatch()..start();
    RunResult result;
    try {
      result = await _processUtils.run(
        command,
        workingDirectory: project.android.hostAppGradleRoot.path,
        allowReentrantFlutter: true,
        environment: _java?.environment,
      );
    } finally {
      status.stop();
    }
    final Duration elapsedDuration = sw.elapsed;
    _analytics.send(
      Event.timing(
        workflow: 'build',
        variableName: 'gradle-aar',
        elapsedMilliseconds: elapsedDuration.inMilliseconds,
      ),
    );

    if (result.exitCode != 0) {
      _logger.printStatus(result.stdout, wrap: false);
      _logger.printError(result.stderr, wrap: false);
      throwToolExit(
        'Gradle task $aarTask failed with exit code ${result.exitCode}.',
        exitCode: result.exitCode,
      );
    }
    final Directory repoDirectory = getRepoDirectory(outputDirectory);
    if (!repoDirectory.existsSync()) {
      _logger.printStatus(result.stdout, wrap: false);
      _logger.printError(result.stderr, wrap: false);
      throwToolExit('Gradle task $aarTask failed to produce $repoDirectory.', exitCode: exitCode);
    }
    _logger.printStatus(
      '${_logger.terminal.successMark} '
      'Built ${_fileSystem.path.relative(repoDirectory.path)}',
      color: TerminalColor.green,
    );
  }

  @override
  Future<List<String>> getBuildVariants({required FlutterProject project}) async {
    late Stopwatch sw;
    int exitCode = 1;
    final List<String> results = <String>[];

    try {
      exitCode = await _runGradleTask(
        _kBuildVariantTaskName,
        preRunTask: () {
          sw = Stopwatch()..start();
        },
        postRunTask: () {
          final Duration elapsedDuration = sw.elapsed;
          _analytics.send(
            Event.timing(
              workflow: 'print',
              variableName: 'android build variants',
              elapsedMilliseconds: elapsedDuration.inMilliseconds,
            ),
          );
        },
        options: const <String>['-q'],
        project: project,
        localGradleErrors: gradleErrors,
        gradleExecutablePath: _gradleUtils.getExecutable(project),
        outputParser: (String line) {
          if (_kBuildVariantRegex.firstMatch(line) case final RegExpMatch match) {
            results.add(match.namedGroup(_kBuildVariantRegexGroupName)!);
          }
        },
      );
    } on Error catch (error) {
      _logger.printError(error.toString());
    }

    if (exitCode != 0) {
      return const <String>[];
    }

    return results;
  }

  @override
  Future<String> outputsAppLinkSettings(
    String buildVariant, {
    required FlutterProject project,
  }) async {
    final String taskName = _getOutputAppLinkSettingsTaskFor(buildVariant);
    final Directory directory = await project.buildDirectory
        .childDirectory('deeplink_data')
        .create(recursive: true);
    final String outputPath = globals.fs.path.join(
      directory.absolute.path,
      'app-link-settings-$buildVariant.json',
    );
    late Stopwatch sw;
    int exitCode = 1;
    try {
      exitCode = await _runGradleTask(
        taskName,
        preRunTask: () {
          sw = Stopwatch()..start();
        },
        postRunTask: () {
          final Duration elapsedDuration = sw.elapsed;
          _analytics.send(
            Event.timing(
              workflow: 'outputs',
              variableName: 'app link settings',
              elapsedMilliseconds: elapsedDuration.inMilliseconds,
            ),
          );
        },
        options: <String>['-q', '-PoutputPath=$outputPath'],
        project: project,
        localGradleErrors: gradleErrors,
        gradleExecutablePath: _gradleUtils.getExecutable(project),
      );
    } on Error catch (error) {
      _logger.printError(error.toString());
    }

    if (exitCode != 0) {
      throwToolExit('Gradle task $taskName failed with exit code $exitCode');
    }
    return outputPath;
  }
}

/// Prints how to consume the AAR from a host app.
void printHowToConsumeAar({
  required Set<String> buildModes,
  String? androidPackage = 'unknown',
  required Directory repoDirectory,
  required Logger logger,
  required FileSystem fileSystem,
  String? buildNumber,
}) {
  assert(buildModes.isNotEmpty);
  buildNumber ??= '1.0';

  logger.printStatus('\nConsuming the Module', emphasis: true);
  logger.printStatus('''
  1. Open ${fileSystem.path.join('<host>', 'app', 'build.gradle')}
  2. Ensure you have the repositories configured, otherwise add them:

      String storageUrl = System.env.$kFlutterStorageBaseUrl ?: "https://storage.googleapis.com"
      repositories {
        maven {
            url '${repoDirectory.path}'
        }
        maven {
            url "\$storageUrl/download.flutter.io"
        }
      }

  3. Make the host app depend on the Flutter module:

    dependencies {''');

  for (final String buildMode in buildModes) {
    logger.printStatus("""
      ${buildMode}Implementation '$androidPackage:flutter_$buildMode:$buildNumber'""");
  }

  logger.printStatus('''
    }
''');

  if (buildModes.contains('profile')) {
    logger.printStatus('''

  4. Add the `profile` build type:

    android {
      buildTypes {
        profile {
          initWith debug
        }
      }
    }
''');
  }

  logger.printStatus('To learn more, visit https://flutter.dev/to/integrate-android-archive');
}

String _hex(List<int> bytes) {
  final StringBuffer result = StringBuffer();
  for (final int part in bytes) {
    result.write('${part < 16 ? '0' : ''}${part.toRadixString(16)}');
  }
  return result.toString();
}

String _calculateSha(File file) {
  final List<int> bytes = file.readAsBytesSync();
  return _hex(sha1.convert(bytes).bytes);
}

void _exitWithUnsupportedProjectMessage(Terminal terminal, Analytics analytics) {
  analytics.send(
    Event.flutterBuildInfo(
      label: 'unsupported-project',
      buildType: 'gradle',
      error: 'gradle-plugin',
    ),
  );

  throwToolExit(
    '${terminal.warningMark} Your app is using an unsupported Gradle project. '
    'To fix this problem, create a new project by running `flutter create -t app <app-directory>` '
    'and then move the dart code, assets and pubspec.yaml to the new project.',
  );
}

/// Returns [true] if the current app uses AndroidX.
// TODO(egarciad): https://github.com/flutter/flutter/issues/40800
// Remove `FlutterManifest.usesAndroidX` and provide a unified `AndroidProject.usesAndroidX`.
bool isAppUsingAndroidX(Directory androidDirectory) {
  final File properties = androidDirectory.childFile('gradle.properties');
  if (!properties.existsSync()) {
    return false;
  }
  return properties.readAsStringSync().contains('android.useAndroidX=true');
}

/// Returns the APK files for a given [FlutterProject] and [AndroidBuildInfo].
@visibleForTesting
Iterable<String> findApkFilesModule(
  FlutterProject project,
  AndroidBuildInfo androidBuildInfo,
  Logger logger,
  Analytics analytics,
) {
  final Iterable<String> apkFileNames = _apkFilesFor(androidBuildInfo);
  final Directory apkDirectory = getApkDirectory(project);
  final Iterable<File> apks = apkFileNames.expand<File>((String apkFileName) {
    File apkFile = apkDirectory.childFile(apkFileName);
    if (apkFile.existsSync()) {
      return <File>[apkFile];
    }
    final BuildInfo buildInfo = androidBuildInfo.buildInfo;
    final String modeName = camelCase(buildInfo.modeName);
    apkFile = apkDirectory.childDirectory(modeName).childFile(apkFileName);
    if (apkFile.existsSync()) {
      return <File>[apkFile];
    }
    final String? flavor = buildInfo.flavor;
    if (flavor != null) {
      // Android Studio Gradle plugin v3 adds flavor to path.
      apkFile = apkDirectory.childDirectory(flavor).childDirectory(modeName).childFile(apkFileName);
      if (apkFile.existsSync()) {
        return <File>[apkFile];
      }
    }
    return const <File>[];
  });
  if (apks.isEmpty) {
    _exitWithExpectedFileNotFound(
      project: project,
      fileExtension: '.apk',
      logger: logger,
      analytics: analytics,
    );
  }
  return apks.map((File file) => file.path);
}

/// Returns the APK files for a given [FlutterProject] and [AndroidBuildInfo].
///
/// The flutter.gradle plugin will copy APK outputs into:
/// `$buildDir/app/outputs/flutter-apk/app-<abi>-<flavor-flag>-<build-mode-flag>.apk`
@visibleForTesting
Iterable<String> listApkPaths(AndroidBuildInfo androidBuildInfo) {
  final String buildType = camelCase(androidBuildInfo.buildInfo.modeName);
  final List<String> apkPartialName = <String>[
    if (androidBuildInfo.buildInfo.flavor?.isNotEmpty ?? false)
      androidBuildInfo.buildInfo.lowerCasedFlavor!,
    '$buildType.apk',
  ];
  if (androidBuildInfo.splitPerAbi) {
    return <String>[
      for (final AndroidArch androidArch in androidBuildInfo.targetArchs)
        <String>['app', androidArch.archName, ...apkPartialName].join('-'),
    ];
  }
  return <String>[
    <String>['app', ...apkPartialName].join('-'),
  ];
}

@visibleForTesting
File findBundleFile(
  FlutterProject project,
  BuildInfo buildInfo,
  Logger logger,
  Analytics analytics,
) {
  final List<File> fileCandidates = <File>[
    getBundleDirectory(project).childDirectory(camelCase(buildInfo.modeName)).childFile('app.aab'),
    getBundleDirectory(
      project,
    ).childDirectory(camelCase(buildInfo.modeName)).childFile('app-${buildInfo.modeName}.aab'),
  ];
  if (buildInfo.flavor != null) {
    // The Android Gradle plugin 3.0.0 adds the flavor name to the path.
    // For example: In release mode, if the flavor name is `foo_bar`, then
    // the directory name is `foo_barRelease`.
    fileCandidates.add(
      getBundleDirectory(project)
          .childDirectory('${buildInfo.lowerCasedFlavor}${camelCase('_${buildInfo.modeName}')}')
          .childFile('app.aab'),
    );

    // The Android Gradle plugin 3.5.0 adds the flavor name to file name.
    // For example: In release mode, if the flavor name is `foo_bar`, then
    // the file name is `app-foo_bar-release.aab`.
    fileCandidates.add(
      getBundleDirectory(project)
          .childDirectory('${buildInfo.lowerCasedFlavor}${camelCase('_${buildInfo.modeName}')}')
          .childFile('app-${buildInfo.lowerCasedFlavor}-${buildInfo.modeName}.aab'),
    );

    // The Android Gradle plugin 4.1.0 does only lowercase the first character of flavor name.
    fileCandidates.add(
      getBundleDirectory(project)
          .childDirectory('${buildInfo.uncapitalizedFlavor}${camelCase('_${buildInfo.modeName}')}')
          .childFile('app-${buildInfo.uncapitalizedFlavor}-${buildInfo.modeName}.aab'),
    );

    // The Android Gradle plugin uses kebab-case and lowercases the first character of the flavor name
    // when multiple flavor dimensions are used:
    // e.g.
    // flavorDimensions "dimension1","dimension2"
    // productFlavors {
    //   foo {
    //     dimension "dimension1"
    //   }
    //   bar {
    //     dimension "dimension2"
    //   }
    // }
    fileCandidates.add(
      getBundleDirectory(project)
          .childDirectory('${buildInfo.uncapitalizedFlavor}${camelCase('_${buildInfo.modeName}')}')
          .childFile('app-${kebabCase(buildInfo.uncapitalizedFlavor!)}-${buildInfo.modeName}.aab'),
    );
  }
  for (final File bundleFile in fileCandidates) {
    if (bundleFile.existsSync()) {
      return bundleFile;
    }
  }
  _exitWithExpectedFileNotFound(
    project: project,
    fileExtension: '.aab',
    logger: logger,
    analytics: analytics,
  );
}

/// Throws a [ToolExit] exception and logs the event.
Never _exitWithExpectedFileNotFound({
  required FlutterProject project,
  required String fileExtension,
  required Logger logger,
  required Analytics analytics,
}) {
  final String androidGradlePluginVersion = getGradleVersionForAndroidPlugin(
    project.android.hostAppGradleRoot,
    logger,
  );
  final String gradleBuildSettings =
      'androidGradlePluginVersion: $androidGradlePluginVersion, '
      'fileExtension: $fileExtension';

  analytics.send(
    Event.flutterBuildInfo(
      label: 'gradle-expected-file-not-found',
      buildType: 'gradle',
      settings: gradleBuildSettings,
    ),
  );

  throwToolExit(
    'Gradle build failed to produce an $fileExtension file. '
    "It's likely that this file was generated under ${project.android.buildDirectory.path}, "
    "but the tool couldn't find it.",
  );
}

void _createSymlink(String targetPath, String linkPath, FileSystem fileSystem) {
  final File targetFile = fileSystem.file(targetPath);
  if (!targetFile.existsSync()) {
    throwToolExit("The file $targetPath wasn't found in the local engine out directory.");
  }
  final File linkFile = fileSystem.file(linkPath);
  final Link symlink = linkFile.parent.childLink(linkFile.basename);
  try {
    symlink.createSync(targetPath, recursive: true);
  } on FileSystemException catch (exception) {
    throwToolExit('Failed to create the symlink $linkPath->$targetPath: $exception');
  }
}

String _getLocalArtifactVersion(String pomPath, FileSystem fileSystem) {
  final File pomFile = fileSystem.file(pomPath);
  if (!pomFile.existsSync()) {
    throwToolExit("The file $pomPath wasn't found in the local engine out directory.");
  }
  XmlDocument document;
  try {
    document = XmlDocument.parse(pomFile.readAsStringSync());
  } on XmlException {
    throwToolExit('Error parsing $pomPath. Please ensure that this is a valid XML document.');
  } on FileSystemException {
    throwToolExit(
      'Error reading $pomPath. Please ensure that you have read permission to this '
      'file and try again.',
    );
  }
  final Iterable<XmlElement> project = document.findElements('project');
  assert(project.isNotEmpty);
  for (final XmlElement versionElement in document.findAllElements('version')) {
    if (versionElement.parent == project.first) {
      return versionElement.innerText;
    }
  }
  throwToolExit('Error while parsing the <version> element from $pomPath');
}

/// Returns the local Maven repository for a local engine build.
/// For example, if the engine is built locally at <home>/engine/src/out/android_release_unopt
/// This method generates symlinks in the temp directory to the engine artifacts
/// following the convention specified on https://maven.apache.org/pom.html#Repositories
Directory _getLocalEngineRepo({
  required String engineOutPath,
  required AndroidBuildInfo androidBuildInfo,
  required FileSystem fileSystem,
}) {
  final String abi = _getAbiByLocalEnginePath(engineOutPath);
  final Directory localEngineRepo = fileSystem.systemTempDirectory.createTempSync(
    'flutter_tool_local_engine_repo.',
  );
  final String buildMode = androidBuildInfo.buildInfo.modeName;
  final String artifactVersion = _getLocalArtifactVersion(
    fileSystem.path.join(engineOutPath, 'flutter_embedding_$buildMode.pom'),
    fileSystem,
  );
  for (final String artifact in const <String>['pom', 'jar']) {
    // The Android embedding artifacts.
    _createSymlink(
      fileSystem.path.join(engineOutPath, 'flutter_embedding_$buildMode.$artifact'),
      fileSystem.path.join(
        localEngineRepo.path,
        'io',
        'flutter',
        'flutter_embedding_$buildMode',
        artifactVersion,
        'flutter_embedding_$buildMode-$artifactVersion.$artifact',
      ),
      fileSystem,
    );
    // The engine artifacts (libflutter.so).
    _createSymlink(
      fileSystem.path.join(engineOutPath, '${abi}_$buildMode.$artifact'),
      fileSystem.path.join(
        localEngineRepo.path,
        'io',
        'flutter',
        '${abi}_$buildMode',
        artifactVersion,
        '${abi}_$buildMode-$artifactVersion.$artifact',
      ),
      fileSystem,
    );
  }
  for (final String artifact in <String>['flutter_embedding_$buildMode', '${abi}_$buildMode']) {
    _createSymlink(
      fileSystem.path.join(engineOutPath, '$artifact.maven-metadata.xml'),
      fileSystem.path.join(localEngineRepo.path, 'io', 'flutter', artifact, 'maven-metadata.xml'),
      fileSystem,
    );
  }
  return localEngineRepo;
}

String _getAbiByLocalEnginePath(String engineOutPath) {
  String result = 'armeabi_v7a';
  if (engineOutPath.contains('x86')) {
    result = 'x86';
  } else if (engineOutPath.contains('x64')) {
    result = 'x86_64';
  } else if (engineOutPath.contains('arm64')) {
    result = 'arm64_v8a';
  }
  return result;
}

String _getTargetPlatformByLocalEnginePath(String engineOutPath) {
  String result = 'android-arm';
  if (engineOutPath.contains('x86')) {
    result = 'android-x86';
  } else if (engineOutPath.contains('x64')) {
    result = 'android-x64';
  } else if (engineOutPath.contains('arm64')) {
    result = 'android-arm64';
  }
  return result;
}
