// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/user_messages.dart';
import '../base/version.dart';
import '../build_info.dart';
import '../cache.dart';
import '../ios/xcodeproj.dart';

Version get xcodeRequiredVersion => Version(14, null, null);

/// Diverging this number from the minimum required version will provide a doctor
/// warning, not error, that users should upgrade Xcode.
Version get xcodeRecommendedVersion => Version(15, null, null);

/// SDK name passed to `xcrun --sdk`. Corresponds to undocumented Xcode
/// SUPPORTED_PLATFORMS values.
///
/// Usage: xcrun [options] <tool name> ... arguments ...
/// ...
/// --sdk <sdk name>            find the tool for the given SDK name.
String getSDKNameForIOSEnvironmentType(EnvironmentType environmentType) {
  return (environmentType == EnvironmentType.simulator) ? 'iphonesimulator' : 'iphoneos';
}

/// A utility class for interacting with Xcode command line tools.
class Xcode {
  Xcode({
    required Platform platform,
    required ProcessManager processManager,
    required Logger logger,
    required FileSystem fileSystem,
    required XcodeProjectInterpreter xcodeProjectInterpreter,
    required UserMessages userMessages,
    String? flutterRoot,
  }) : _platform = platform,
       _fileSystem = fileSystem,
       _xcodeProjectInterpreter = xcodeProjectInterpreter,
       _userMessage = userMessages,
       _flutterRoot = flutterRoot,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       _logger = logger;

  /// Create an [Xcode] for testing.
  ///
  /// Defaults to a memory file system, fake platform,
  /// buffer logger, and test [XcodeProjectInterpreter].
  @visibleForTesting
  factory Xcode.test({
    required ProcessManager processManager,
    XcodeProjectInterpreter? xcodeProjectInterpreter,
    Platform? platform,
    FileSystem? fileSystem,
    String? flutterRoot,
    Logger? logger,
  }) {
    platform ??= FakePlatform(operatingSystem: 'macos', environment: <String, String>{});
    logger ??= BufferLogger.test();
    return Xcode(
      platform: platform,
      processManager: processManager,
      fileSystem: fileSystem ?? MemoryFileSystem.test(),
      userMessages: UserMessages(),
      flutterRoot: flutterRoot,
      logger: logger,
      xcodeProjectInterpreter:
          xcodeProjectInterpreter ?? XcodeProjectInterpreter.test(processManager: processManager),
    );
  }

  final Platform _platform;
  final ProcessUtils _processUtils;
  final FileSystem _fileSystem;
  final XcodeProjectInterpreter _xcodeProjectInterpreter;
  final UserMessages _userMessage;
  final String? _flutterRoot;
  final Logger _logger;

  bool get isInstalledAndMeetsVersionCheck =>
      _platform.isMacOS && isInstalled && isRequiredVersionSatisfactory;

  String? _xcodeSelectPath;
  String? get xcodeSelectPath {
    if (_xcodeSelectPath == null) {
      try {
        _xcodeSelectPath =
            _processUtils.runSync(<String>['/usr/bin/xcode-select', '--print-path']).stdout.trim();
      } on ProcessException {
        // Ignored, return null below.
      } on ArgumentError {
        // Ignored, return null below.
      }
    }
    return _xcodeSelectPath;
  }

  String get xcodeAppPath {
    // If the Xcode Select Path is /Applications/Xcode.app/Contents/Developer,
    // the path to Xcode App is /Applications/Xcode.app

    final String? pathToXcode = xcodeSelectPath;
    if (pathToXcode == null || pathToXcode.isEmpty) {
      throwToolExit(_userMessage.xcodeMissing);
    }
    final int index = pathToXcode.indexOf('.app');
    if (index == -1) {
      throwToolExit(_userMessage.xcodeMissing);
    }
    return pathToXcode.substring(0, index + 4);
  }

  /// Path to script to automate debugging through Xcode. Used in xcode_debug.dart.
  /// Located in this file to make it easily overrideable in google3.
  String get xcodeAutomationScriptPath {
    final String flutterRoot = _flutterRoot ?? Cache.flutterRoot!;
    final String flutterToolsAbsolutePath = _fileSystem.path.join(
      flutterRoot,
      'packages',
      'flutter_tools',
    );

    final String filePath = '$flutterToolsAbsolutePath/bin/xcode_debug.js';
    if (!_fileSystem.file(filePath).existsSync()) {
      throwToolExit('Unable to find Xcode automation script at $filePath');
    }
    return filePath;
  }

  bool get isInstalled => _xcodeProjectInterpreter.isInstalled;

  Version? get currentVersion => _xcodeProjectInterpreter.version;

  String? get buildVersion => _xcodeProjectInterpreter.build;

  String? get versionText => _xcodeProjectInterpreter.versionText;

  bool? _eulaSigned;

  /// Has the EULA been signed?
  bool get eulaSigned {
    if (_eulaSigned == null) {
      try {
        final RunResult result = _processUtils.runSync(<String>[...xcrunCommand(), 'clang']);
        if (result.stdout.contains('license')) {
          _eulaSigned = false;
        } else if (result.stderr.contains('license')) {
          _eulaSigned = false;
        } else {
          _eulaSigned = true;
        }
      } on ProcessException {
        _eulaSigned = false;
      }
    }
    return _eulaSigned ?? false;
  }

  bool? _isSimctlInstalled;

  /// Verifies that simctl is installed by trying to run it.
  bool get isSimctlInstalled {
    // This command will error if additional components need to be installed in
    // xcode 9.2 and above.
    _isSimctlInstalled ??= _processUtils.exitsHappySync(<String>[
      ...xcrunCommand(),
      'simctl',
      'list',
      'devices',
      'booted',
    ]);
    return _isSimctlInstalled ?? false;
  }

  bool? _isDevicectlInstalled;

  /// Verifies that `devicectl` is installed by checking Xcode version and trying
  /// to run it. `devicectl` is made available in Xcode 15.
  bool get isDevicectlInstalled {
    if (_isDevicectlInstalled == null) {
      if (currentVersion == null || currentVersion!.major < 15) {
        _isDevicectlInstalled = false;
        return _isDevicectlInstalled!;
      }
      _isDevicectlInstalled = _processUtils.exitsHappySync(<String>[
        ...xcrunCommand(),
        'devicectl',
        '--version',
      ]);
    }
    return _isDevicectlInstalled ?? false;
  }

  bool get isRequiredVersionSatisfactory {
    final Version? version = currentVersion;
    if (version == null) {
      return false;
    }
    return version >= xcodeRequiredVersion;
  }

  bool get isRecommendedVersionSatisfactory {
    final Version? version = currentVersion;
    if (version == null) {
      return false;
    }
    return version >= xcodeRecommendedVersion;
  }

  /// See [XcodeProjectInterpreter.xcrunCommand].
  List<String> xcrunCommand() => _xcodeProjectInterpreter.xcrunCommand();

  Future<RunResult> cc(List<String> args) => _run('cc', args);

  Future<RunResult> clang(List<String> args) => _run('clang', args);

  Future<RunResult> dsymutil(List<String> args) => _run('dsymutil', args);

  Future<RunResult> strip(List<String> args) => _run('strip', args);

  Future<RunResult> _run(String command, List<String> args) {
    return _processUtils.run(<String>[...xcrunCommand(), command, ...args], throwOnError: true);
  }

  Future<String> sdkLocation(EnvironmentType environmentType) async {
    final RunResult runResult = await _processUtils.run(<String>[
      ...xcrunCommand(),
      '--sdk',
      getSDKNameForIOSEnvironmentType(environmentType),
      '--show-sdk-path',
    ]);
    if (runResult.exitCode != 0) {
      throwToolExit('Could not find SDK location: ${runResult.stderr}');
    }
    return runResult.stdout.trim();
  }

  String? getSimulatorPath() {
    final String? selectPath = xcodeSelectPath;
    if (selectPath == null) {
      return null;
    }
    final String appPath = _fileSystem.path.join(selectPath, 'Applications', 'Simulator.app');
    return _fileSystem.directory(appPath).existsSync() ? appPath : null;
  }

  /// Gets the version number of the platform for the selected SDK.
  Future<Version?> sdkPlatformVersion(EnvironmentType environmentType) async {
    final RunResult runResult = await _processUtils.run(<String>[
      ...xcrunCommand(),
      '--sdk',
      getSDKNameForIOSEnvironmentType(environmentType),
      '--show-sdk-platform-version',
    ]);
    if (runResult.exitCode != 0) {
      _logger.printError('Could not find SDK Platform Version: ${runResult.stderr}');
      return null;
    }
    final String versionString = runResult.stdout.trim();
    return Version.parse(versionString);
  }
}

EnvironmentType? environmentTypeFromSdkroot(String sdkroot, FileSystem fileSystem) {
  // iPhoneSimulator.sdk or iPhoneOS.sdk
  final String sdkName = fileSystem.path.basename(sdkroot).toLowerCase();
  if (sdkName.contains('iphone')) {
    return sdkName.contains('simulator') ? EnvironmentType.simulator : EnvironmentType.physical;
  }
  assert(false);
  return null;
}
