// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../base/common.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/project_migrator.dart';
import '../base/version.dart';
import '../build_info.dart';
import '../cache.dart';
import '../ios/xcodeproj.dart';
import '../migrations/cocoapods_script_symlink.dart';
import '../migrations/cocoapods_toolchain_directory_migration.dart';
import '../project.dart';

const String noCocoaPodsConsequence = '''
  CocoaPods is a package manager for iOS or macOS platform code.
  Without CocoaPods, plugins will not work on iOS or macOS.
  For more info, see https://flutter.dev/to/platform-plugins''';

const String unknownCocoaPodsConsequence = '''
  Flutter is unable to determine the installed CocoaPods's version.
  Ensure that the output of 'pod --version' contains only digits and . to be recognized by Flutter.''';

const String brokenCocoaPodsConsequence = '''
  You appear to have CocoaPods installed but it is not working.
  This can happen if the version of Ruby that CocoaPods was installed with is different from the one being used to invoke it.
  This can usually be fixed by re-installing CocoaPods.''';

const String outOfDateFrameworksPodfileConsequence = '''
  This can cause a mismatched version of Flutter to be embedded in your app, which may result in App Store submission rejection or crashes.
  If you have local Podfile edits you would like to keep, see https://github.com/flutter/flutter/issues/24641 for instructions.''';

const String outOfDatePluginsPodfileConsequence = '''
  This can cause issues if your application depends on plugins that do not support iOS or macOS.
  See https://flutter.dev/to/pubspec-plugin-platforms for details.
  If you have local Podfile edits you would like to keep, see https://github.com/flutter/flutter/issues/45197 for instructions.''';

const String cocoaPodsInstallInstructions =
    'see https://guides.cocoapods.org/using/getting-started.html#installation';

const String cocoaPodsUpdateInstructions =
    'see https://guides.cocoapods.org/using/getting-started.html#updating-cocoapods';

const String podfileIosMigrationInstructions = '''
  rm ios/Podfile''';

const String podfileMacOSMigrationInstructions = '''
  rm macos/Podfile''';

/// Result of evaluating the CocoaPods installation.
enum CocoaPodsStatus {
  /// iOS plugins will not work, installation required.
  notInstalled,

  /// iOS plugins might not work, upgrade recommended.
  unknownVersion,

  /// iOS plugins will not work, upgrade required.
  belowMinimumVersion,

  /// iOS plugins may not work in certain situations (Swift, static libraries),
  /// upgrade recommended.
  belowRecommendedVersion,

  /// Everything should be fine.
  recommended,

  /// iOS plugins will not work, re-install required.
  brokenInstall,
}

const Version cocoaPodsMinimumVersion = Version.withText(1, 10, 0, '1.10.0');
const Version cocoaPodsRecommendedVersion = Version.withText(1, 16, 2, '1.16.2');

/// Cocoapods is a dependency management solution for iOS and macOS applications.
///
/// Cocoapods is generally installed via ruby gems and interacted with via
/// the `pod` CLI command.
///
/// See also:
///   * https://cocoapods.org/ - the cocoapods website.
///   * https://flutter.dev/to/macos-ios-setup - instructions for
///     installing iOS/macOS dependencies.
class CocoaPods {
  CocoaPods({
    required FileSystem fileSystem,
    required ProcessManager processManager,
    required XcodeProjectInterpreter xcodeProjectInterpreter,
    required Logger logger,
    required Platform platform,
    required Analytics analytics,
  }) : _fileSystem = fileSystem,
       _processManager = processManager,
       _xcodeProjectInterpreter = xcodeProjectInterpreter,
       _logger = logger,
       _analytics = analytics,
       _processUtils = ProcessUtils(processManager: processManager, logger: logger),
       _operatingSystemUtils = OperatingSystemUtils(
         fileSystem: fileSystem,
         logger: logger,
         platform: platform,
         processManager: processManager,
       );

  final FileSystem _fileSystem;
  final ProcessManager _processManager;
  final ProcessUtils _processUtils;
  final OperatingSystemUtils _operatingSystemUtils;
  final XcodeProjectInterpreter _xcodeProjectInterpreter;
  final Logger _logger;
  final Analytics _analytics;

  Future<String?>? _versionText;

  Future<bool> get isInstalled => _processUtils.exitsHappy(<String>['which', 'pod']);

  Future<String?> get cocoaPodsVersionText {
    _versionText ??= _processUtils
        .run(<String>['pod', '--version'], environment: <String, String>{'LANG': 'en_US.UTF-8'})
        .then<String?>((RunResult result) {
          return result.exitCode == 0 ? result.stdout.trim() : null;
        }, onError: (dynamic _) => null);
    return _versionText!;
  }

  Future<CocoaPodsStatus> get evaluateCocoaPodsInstallation async {
    if (!(await isInstalled)) {
      return CocoaPodsStatus.notInstalled;
    }
    final String? versionText = await cocoaPodsVersionText;
    if (versionText == null) {
      return CocoaPodsStatus.brokenInstall;
    }
    try {
      final Version? installedVersion = Version.parse(versionText);
      if (installedVersion == null) {
        return CocoaPodsStatus.unknownVersion;
      }
      if (installedVersion < cocoaPodsMinimumVersion) {
        return CocoaPodsStatus.belowMinimumVersion;
      }
      if (installedVersion < cocoaPodsRecommendedVersion) {
        return CocoaPodsStatus.belowRecommendedVersion;
      }
      return CocoaPodsStatus.recommended;
    } on FormatException {
      return CocoaPodsStatus.notInstalled;
    }
  }

  Future<bool> processPods({
    required XcodeBasedProject xcodeProject,
    required BuildMode buildMode,
    bool dependenciesChanged = true,
  }) async {
    if (!xcodeProject.podfile.existsSync()) {
      // Swift Package Manager doesn't need Podfile, so don't error.
      if (xcodeProject.usesSwiftPackageManager) {
        return false;
      }
      throwToolExit('Podfile missing');
    }
    _warnIfPodfileOutOfDate(xcodeProject);
    bool podsProcessed = false;
    if (_shouldRunPodInstall(xcodeProject, dependenciesChanged)) {
      if (!await _checkPodCondition()) {
        throwToolExit('CocoaPods not installed or not in valid state.');
      }
      await _runPodInstall(xcodeProject, buildMode);

      // This migrator works around a CocoaPods bug, and should be run after `pod install` is run.
      final ProjectMigration postPodMigration = ProjectMigration(<ProjectMigrator>[
        CocoaPodsScriptReadlink(xcodeProject, _xcodeProjectInterpreter, _logger),
        CocoaPodsToolchainDirectoryMigration(xcodeProject, _xcodeProjectInterpreter, _logger),
      ]);
      await postPodMigration.run();

      podsProcessed = true;
    }
    return podsProcessed;
  }

  /// Make sure the CocoaPods tools are in the right states.
  Future<bool> _checkPodCondition() async {
    final CocoaPodsStatus installation = await evaluateCocoaPodsInstallation;
    switch (installation) {
      case CocoaPodsStatus.notInstalled:
        _logger.printWarning(
          'Warning: CocoaPods not installed. Skipping pod install.\n'
          '$noCocoaPodsConsequence\n'
          'For installation instructions, $cocoaPodsInstallInstructions\n',
          emphasis: true,
        );
        return false;
      case CocoaPodsStatus.brokenInstall:
        _logger.printWarning(
          'Warning: CocoaPods is installed but broken. Skipping pod install.\n'
          '$brokenCocoaPodsConsequence\n'
          'For re-installation instructions, $cocoaPodsInstallInstructions\n',
          emphasis: true,
        );
        return false;
      case CocoaPodsStatus.unknownVersion:
        _logger.printWarning(
          'Warning: Unknown CocoaPods version installed.\n'
          '$unknownCocoaPodsConsequence\n'
          'To update CocoaPods, $cocoaPodsUpdateInstructions\n',
          emphasis: true,
        );
      case CocoaPodsStatus.belowMinimumVersion:
        _logger.printWarning(
          'Warning: CocoaPods minimum required version $cocoaPodsMinimumVersion or greater not installed. Skipping pod install.\n'
          '$noCocoaPodsConsequence\n'
          'To update CocoaPods, $cocoaPodsUpdateInstructions\n',
          emphasis: true,
        );
        return false;
      case CocoaPodsStatus.belowRecommendedVersion:
        _logger.printWarning(
          'Warning: CocoaPods recommended version $cocoaPodsRecommendedVersion or greater not installed.\n'
          'Pods handling may fail on some projects involving plugins.\n'
          'To update CocoaPods, $cocoaPodsUpdateInstructions\n',
          emphasis: true,
        );
      case CocoaPodsStatus.recommended:
        break;
    }

    return true;
  }

  /// Ensures the given Xcode-based sub-project of a parent Flutter project
  /// contains a suitable `Podfile` and that its `Flutter/Xxx.xcconfig` files
  /// include pods configuration.
  Future<void> setupPodfile(XcodeBasedProject xcodeProject) async {
    if (!_xcodeProjectInterpreter.isInstalled) {
      // Don't do anything for iOS when host platform doesn't support it.
      return;
    }
    final Directory runnerProject = xcodeProject.xcodeProject;
    if (!runnerProject.existsSync()) {
      return;
    }
    final File podfile = xcodeProject.podfile;
    if (podfile.existsSync()) {
      addPodsDependencyToFlutterXcconfig(xcodeProject);
      return;
    }
    final File podfileTemplate = await getPodfileTemplate(xcodeProject, runnerProject);
    podfileTemplate.copySync(podfile.path);
    addPodsDependencyToFlutterXcconfig(xcodeProject);
  }

  Future<File> getPodfileTemplate(XcodeBasedProject xcodeProject, Directory runnerProject) async {
    String podfileTemplateName;
    if (xcodeProject is MacOSProject) {
      podfileTemplateName = 'Podfile-macos';
    } else {
      final bool isSwift = (await _xcodeProjectInterpreter.getBuildSettings(
        runnerProject.path,
        buildContext: const XcodeProjectBuildContext(),
      )).containsKey('SWIFT_VERSION');
      podfileTemplateName = isSwift ? 'Podfile-ios-swift' : 'Podfile-ios-objc';
    }
    return _fileSystem.file(
      _fileSystem.path.join(
        Cache.flutterRoot!,
        'packages',
        'flutter_tools',
        'templates',
        'cocoapods',
        podfileTemplateName,
      ),
    );
  }

  /// Ensures all `Flutter/Xxx.xcconfig` files for the given Xcode-based
  /// sub-project of a parent Flutter project include pods configuration.
  void addPodsDependencyToFlutterXcconfig(XcodeBasedProject xcodeProject) {
    _addPodsDependencyToFlutterXcconfig(xcodeProject, 'Debug');
    _addPodsDependencyToFlutterXcconfig(xcodeProject, 'Release');
  }

  String includePodsXcconfig(String mode) {
    return 'Pods/Target Support Files/Pods-Runner/Pods-Runner.${mode.toLowerCase()}.xcconfig';
  }

  bool xcconfigIncludesPods(File xcodeConfig) {
    if (xcodeConfig.existsSync()) {
      final String content = xcodeConfig.readAsStringSync();
      return content.contains('Pods/Target Support Files/Pods-');
    }
    return false;
  }

  void _addPodsDependencyToFlutterXcconfig(XcodeBasedProject xcodeProject, String mode) {
    final File file = xcodeProject.xcodeConfigFor(mode);
    if (file.existsSync()) {
      final String content = file.readAsStringSync();
      final String includeFile = includePodsXcconfig(mode);
      final String include = '#include? "$includeFile"';
      if (!content.contains('Pods/Target Support Files/Pods-')) {
        file.writeAsStringSync('$include\n$content', flush: true);
      }
    }
  }

  /// Ensures that pod install is deemed needed on next check.
  void invalidatePodInstallOutput(XcodeBasedProject xcodeProject) {
    final File manifestLock = xcodeProject.podManifestLock;
    ErrorHandlingFileSystem.deleteIfExists(manifestLock);
  }

  // Check if you need to run pod install.
  // The pod install will run if any of below is true.
  // 1. Flutter dependencies have changed
  // 2. Podfile.lock doesn't exist or is older than Podfile
  // 3. Pods/Manifest.lock doesn't exist (It is deleted when plugins change)
  // 4. Podfile.lock doesn't match Pods/Manifest.lock.
  bool _shouldRunPodInstall(XcodeBasedProject xcodeProject, bool dependenciesChanged) {
    if (dependenciesChanged) {
      return true;
    }

    final File podfileFile = xcodeProject.podfile;
    final File podfileLockFile = xcodeProject.podfileLock;
    final File manifestLockFile = xcodeProject.podManifestLock;

    return !podfileLockFile.existsSync() ||
        !manifestLockFile.existsSync() ||
        podfileLockFile.statSync().modified.isBefore(podfileFile.statSync().modified) ||
        podfileLockFile.readAsStringSync() != manifestLockFile.readAsStringSync();
  }

  Future<void> _runPodInstall(XcodeBasedProject xcodeProject, BuildMode buildMode) async {
    final Status status = _logger.startProgress('Running pod install...');
    final ProcessResult result = await _processManager.run(
      <String>['pod', 'install', '--verbose'],
      workingDirectory: _fileSystem.path.dirname(xcodeProject.podfile.path),
      environment: <String, String>{
        // See https://github.com/flutter/flutter/issues/10873.
        // CocoaPods analytics adds a lot of latency.
        'COCOAPODS_DISABLE_STATS': 'true',
        'LANG': 'en_US.UTF-8',
      },
    );
    status.stop();
    if (_logger.isVerbose || result.exitCode != 0) {
      final String stdout = result.stdout as String;
      if (stdout.isNotEmpty) {
        _logger.printStatus("CocoaPods' output:\n↳");
        _logger.printStatus(stdout, indent: 4);
      }
      final String stderr = result.stderr as String;
      if (stderr.isNotEmpty) {
        _logger.printStatus('Error output from CocoaPods:\n↳');
        _logger.printStatus(stderr, indent: 4);
      }
    }

    if (result.exitCode != 0) {
      invalidatePodInstallOutput(xcodeProject);
      _diagnosePodInstallFailure(result, xcodeProject);
      throwToolExit('Error running pod install');
    } else if (xcodeProject.podfileLock.existsSync()) {
      // Even if the Podfile.lock didn't change, update its modified date to now
      // so Podfile.lock is newer than Podfile.
      _processManager.runSync(<String>[
        'touch',
        xcodeProject.podfileLock.path,
      ], workingDirectory: _fileSystem.path.dirname(xcodeProject.podfile.path));
    }
  }

  void _diagnosePodInstallFailure(ProcessResult result, XcodeBasedProject xcodeProject) {
    final Object? stdout = result.stdout;
    final Object? stderr = result.stderr;
    if (stdout is! String || stderr is! String) {
      return;
    }
    if (stdout.contains('out-of-date source repos')) {
      _logger.printError(
        "Error: CocoaPods's specs repository is too out-of-date to satisfy dependencies.\n"
        'To update the CocoaPods specs, run:\n'
        '  pod repo update\n',
        emphasis: true,
      );
    } else if ((_isFfiX86Error(stdout) || _isFfiX86Error(stderr)) &&
        _operatingSystemUtils.hostPlatform == HostPlatform.darwin_arm64) {
      // https://github.com/flutter/flutter/issues/70796
      _analytics.send(Event.appleUsageEvent(workflow: 'pod-install-failure', parameter: 'arm-ffi'));
      _logger.printError(
        'Error: To set up CocoaPods for ARM macOS, run:\n'
        '  sudo gem uninstall ffi && sudo gem install ffi -- --enable-libffi-alloc\n',
        emphasis: true,
      );
    } else if (stdout.contains('required a higher minimum deployment target')) {
      final ({String failingPod, String sourcePlugin, String podPluginSubdir})? podInfo =
          _parseMinDeploymentFailureInfo(stdout);
      if (podInfo != null) {
        final Directory symlinksDir;
        final String podPlatformString;
        final String platformName;
        final String docsLink;
        if (xcodeProject is IosProject) {
          symlinksDir = xcodeProject.symlinks;
          podPlatformString = 'ios';
          platformName = 'iOS';
          docsLink = 'https://flutter.dev/to/ios-deploy';
        } else if (xcodeProject is MacOSProject) {
          symlinksDir = xcodeProject.ephemeralDirectory.childDirectory('.symlinks');
          podPlatformString = 'osx';
          platformName = 'macOS';
          docsLink = 'https://flutter.dev/to/macos-deploy';
        } else {
          return;
        }

        final String sourcePlugin = podInfo.sourcePlugin;
        // If the plugin's podfile has set its own minimum version correctly
        // based on the requirements of its dependencies the failing pod should
        // be the plugin itself, but if not they may be different (e.g., if
        // a plugin says its minimum iOS version is 11, but depends on a pod
        // with a minimum version of 12, then building for 11 will report that
        // pod as failing.)
        if (podInfo.failingPod == podInfo.sourcePlugin) {
          final File podspec = symlinksDir
              .childDirectory('plugins')
              .childDirectory(sourcePlugin)
              .childDirectory(podInfo.podPluginSubdir)
              .childFile('$sourcePlugin.podspec');
          final String? minDeploymentVersion = _findPodspecMinDeploymentVersion(
            podspec,
            podPlatformString,
          );
          if (minDeploymentVersion != null) {
            _logger.printError(
              'Error: The plugin "$sourcePlugin" requires a higher minimum '
              '$platformName deployment version than your application is targeting.\n'
              "To build, increase your application's deployment target to at "
              'least $minDeploymentVersion as described at $docsLink',
              emphasis: true,
            );
          } else {
            // If for some reason the min version can't be parsed out, provide
            // a less specific error message that still describes the problem,
            // but also requests filing a Flutter issue so the parsing in
            // _findPodspecMinDeploymentVersion can be improved.
            _logger.printError(
              'Error: The plugin "$sourcePlugin" requires a higher minimum '
              '$platformName deployment version than your application is targeting.\n'
              "To build, increase your application's deployment target as "
              'described at $docsLink\n\n'
              'The minimum required version for "$sourcePlugin" could not be '
              'determined. Please file an issue at '
              'https://github.com/flutter/flutter/issues about this error message.',
              emphasis: true,
            );
          }
        } else {
          // In theory this could find the failing pod's spec and parse out its
          // minimum deployment version, but finding that spec would add a lot
          // of complexity to handle a case that plugin authors should not
          // create, so this just provides the actionable step of following up
          // with the plugin developer.
          _logger.printError(
            'Error: The pod "${podInfo.failingPod}" required by the plugin '
            '"$sourcePlugin" requires a higher minimum $platformName deployment '
            "version than the plugin's reported minimum version.\n"
            'To build, remove the plugin "$sourcePlugin", or contact the plugin\'s '
            'developers for assistance.',
            emphasis: true,
          );
        }
      }
    } else if (stdout.contains('unknown ISA `PBXFileSystemSynchronizedRootGroup`')) {
      // CocoaPods must be at least version 1.16.2 to handle synchronized
      // groups/folders https://github.com/CocoaPods/CocoaPods/issues/12456
      _logger.printError(
        'Error: Your Cocoapods might be out-of-date and unable to support synchronized groups/folders. '
        'Please update to a minimum version of 1.16.2 and try again.',
        emphasis: true,
      );
    }
  }

  ({String failingPod, String sourcePlugin, String podPluginSubdir})?
  _parseMinDeploymentFailureInfo(String podInstallOutput) {
    final RegExp sourceLine = RegExp(r'\(from `.*\.symlinks/plugins/([^/]+)/([^/]+)`\)');
    final RegExp dependencyLine = RegExp(
      r'Specs satisfying the `([^ ]+).*` dependency were found, '
      'but they required a higher minimum deployment target',
    );
    final RegExpMatch? sourceMatch = sourceLine.firstMatch(podInstallOutput);
    final RegExpMatch? dependencyMatch = dependencyLine.firstMatch(podInstallOutput);
    if (sourceMatch == null || dependencyMatch == null) {
      return null;
    }
    return (
      failingPod: dependencyMatch.group(1)!,
      sourcePlugin: sourceMatch.group(1)!,
      podPluginSubdir: sourceMatch.group(2)!,
    );
  }

  String? _findPodspecMinDeploymentVersion(File podspec, String platformString) {
    if (!podspec.existsSync()) {
      return null;
    }
    // There are two ways the deployment target can be specified; see
    // https://guides.cocoapods.org/syntax/podspec.html#group_platform
    final RegExp platformPattern = RegExp(
      // Example: spec.platform = :osx, '10.8'
      // where "spec" is an arbitrary variable name.
      r'^\s*[a-zA-Z_]+\.platform\s*=\s*'
      ':$platformString'
      r'''\s*,\s*["']([^"']+)["']''',
      multiLine: true,
    );
    final RegExp deploymentTargetPlatform = RegExp(
      // Example: spec.osx.deployment_target = '10.8'
      // where "spec" is an arbitrary variable name.
      r'^\s*[a-zA-Z_]+\.'
      '$platformString\\.deployment_target'
      r'''\s*=\s*["']([^"']+)["']''',
      multiLine: true,
    );
    final String podspecContents = podspec.readAsStringSync();
    final RegExpMatch? match =
        platformPattern.firstMatch(podspecContents) ??
        deploymentTargetPlatform.firstMatch(podspecContents);
    return match?.group(1);
  }

  bool _isFfiX86Error(String error) {
    return error.contains('ffi_c.bundle') || error.contains('/ffi/');
  }

  void _warnIfPodfileOutOfDate(XcodeBasedProject xcodeProject) {
    final bool isIos = xcodeProject is IosProject;
    if (isIos) {
      // Previously, the Podfile created a symlink to the cached artifacts engine framework
      // and installed the Flutter pod from that path. This could get out of sync with the copy
      // of the Flutter engine that was copied to ios/Flutter by the xcode_backend script.
      // It was possible for the symlink to point to a Debug version of the engine when the
      // Xcode build configuration was Release, which caused App Store submission rejections.
      //
      // Warn the user if they are still symlinking to the framework.
      final Link flutterSymlink = _fileSystem.link(
        _fileSystem.path.join(xcodeProject.symlinks.path, 'flutter'),
      );
      if (flutterSymlink.existsSync()) {
        throwToolExit(
          'Warning: Podfile is out of date\n'
          '$outOfDateFrameworksPodfileConsequence\n'
          'To regenerate the Podfile, run:\n'
          '$podfileIosMigrationInstructions\n',
        );
      }
    }
    // Most of the pod and plugin parsing logic was moved from the Podfile
    // into the tool's podhelper.rb script. If the Podfile still references
    // the old parsed .flutter-plugins file, prompt the regeneration. Old line was:
    // plugin_pods = parse_KV_file('../.flutter-plugins')
    if (xcodeProject.podfile.existsSync() &&
        xcodeProject.podfile.readAsStringSync().contains(".flutter-plugins'")) {
      const String warning =
          'Warning: Podfile is out of date\n'
          '$outOfDatePluginsPodfileConsequence\n'
          'To regenerate the Podfile, run:\n';
      if (isIos) {
        throwToolExit('$warning\n$podfileIosMigrationInstructions\n');
      } else {
        // The old macOS Podfile will work until `.flutter-plugins` is removed.
        // Warn instead of exit.
        _logger.printWarning('$warning\n$podfileMacOSMigrationInstructions\n', emphasis: true);
      }
    }
  }
}
