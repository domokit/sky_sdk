// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/config.dart';
import '../base/file_system.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/version.dart';
import '../convert.dart';
import '../globals.dart' as globals;
import 'java.dart';

// ANDROID_SDK_ROOT is deprecated.
// See https://developer.android.com/studio/command-line/variables.html#envar
const String kAndroidSdkRoot = 'ANDROID_SDK_ROOT';
const String kAndroidHome = 'ANDROID_HOME';

// No official environment variable for the NDK root is documented:
// https://developer.android.com/tools/variables#envar
// The follow three seem to be most commonly used.
const String kAndroidNdkHome = 'ANDROID_NDK_HOME';
const String kAndroidNdkPath = 'ANDROID_NDK_PATH';
const String kAndroidNdkRoot = 'ANDROID_NDK_ROOT';

final RegExp _numberedAndroidPlatformRe = RegExp(r'^android-([0-9]+)$');
final RegExp _sdkVersionRe = RegExp(r'^ro.build.version.sdk=([0-9]+)$');

// Android SDK layout:

// $ANDROID_HOME/platform-tools/adb

// $ANDROID_HOME/build-tools/19.1.0/aapt, dx, zipalign
// $ANDROID_HOME/build-tools/22.0.1/aapt
// $ANDROID_HOME/build-tools/23.0.2/aapt
// $ANDROID_HOME/build-tools/24.0.0-preview/aapt
// $ANDROID_HOME/build-tools/25.0.2/apksigner

// $ANDROID_HOME/platforms/android-22/android.jar
// $ANDROID_HOME/platforms/android-23/android.jar
// $ANDROID_HOME/platforms/android-N/android.jar
class AndroidSdk {
  AndroidSdk(this.directory, {Java? java, FileSystem? fileSystem}) : _java = java {
    reinitialize(fileSystem: fileSystem);
  }

  /// The Android SDK root directory.
  final Directory directory;

  final Java? _java;

  List<AndroidSdkVersion> _sdkVersions = <AndroidSdkVersion>[];
  AndroidSdkVersion? _latestVersion;

  /// Whether the `cmdline-tools` directory exists in the Android SDK.
  ///
  /// This is required to use the newest SDK manager which only works with
  /// the newer JDK.
  bool get cmdlineToolsAvailable => directory.childDirectory('cmdline-tools').existsSync();

  /// Whether the `platform-tools` or `cmdline-tools` directory exists in the Android SDK.
  ///
  /// It is possible to have an Android SDK folder that is missing this with
  /// the expectation that it will be downloaded later, e.g. by gradle or the
  /// sdkmanager. The [licensesAvailable] property should be used to determine
  /// whether the licenses are at least possibly accepted.
  bool get platformToolsAvailable =>
      cmdlineToolsAvailable || directory.childDirectory('platform-tools').existsSync();

  /// Whether the `licenses` directory exists in the Android SDK.
  ///
  /// The existence of this folder normally indicates that the SDK licenses have
  /// been accepted, e.g. via the sdkmanager, Android Studio, or by copying them
  /// from another workstation such as in CI scenarios. If these files are valid
  /// gradle or the sdkmanager will be able to download and use other parts of
  /// the SDK on demand.
  bool get licensesAvailable => directory.childDirectory('licenses').existsSync();

  static AndroidSdk? locateAndroidSdk() {
    String? findAndroidHomeDir() {
      String? androidHomeDir;
      if (globals.config.containsKey('android-sdk')) {
        androidHomeDir = globals.config.getValue('android-sdk') as String?;
      } else if (globals.platform.environment.containsKey(kAndroidHome)) {
        androidHomeDir = globals.platform.environment[kAndroidHome];
      } else if (globals.platform.environment.containsKey(kAndroidSdkRoot)) {
        androidHomeDir = globals.platform.environment[kAndroidSdkRoot];
      } else if (globals.platform.isLinux) {
        if (globals.fsUtils.homeDirPath != null) {
          androidHomeDir = globals.fs.path.join(globals.fsUtils.homeDirPath!, 'Android', 'Sdk');
        }
      } else if (globals.platform.isMacOS) {
        if (globals.fsUtils.homeDirPath != null) {
          androidHomeDir = globals.fs.path.join(
            globals.fsUtils.homeDirPath!,
            'Library',
            'Android',
            'sdk',
          );
        }
      } else if (globals.platform.isWindows) {
        if (globals.fsUtils.homeDirPath != null) {
          androidHomeDir = globals.fs.path.join(
            globals.fsUtils.homeDirPath!,
            'AppData',
            'Local',
            'Android',
            'sdk',
          );
        }
      }

      if (androidHomeDir != null) {
        if (validSdkDirectory(androidHomeDir)) {
          return androidHomeDir;
        }
        if (validSdkDirectory(globals.fs.path.join(androidHomeDir, 'sdk'))) {
          return globals.fs.path.join(androidHomeDir, 'sdk');
        }
      }

      // in build-tools/$version/aapt
      final List<File> aaptBins = globals.os.whichAll('aapt');
      for (File aaptBin in aaptBins) {
        // Make sure we're using the aapt from the SDK.
        aaptBin = globals.fs.file(aaptBin.resolveSymbolicLinksSync());
        final String dir = aaptBin.parent.parent.parent.path;
        if (validSdkDirectory(dir)) {
          return dir;
        }
      }

      // in platform-tools/adb
      final List<File> adbBins = globals.os.whichAll('adb');
      for (File adbBin in adbBins) {
        // Make sure we're using the adb from the SDK.
        adbBin = globals.fs.file(adbBin.resolveSymbolicLinksSync());
        final String dir = adbBin.parent.parent.path;
        if (validSdkDirectory(dir)) {
          return dir;
        }
      }

      return null;
    }

    final String? androidHomeDir = findAndroidHomeDir();
    if (androidHomeDir == null) {
      // No dice.
      globals.printTrace('Unable to locate an Android SDK.');
      return null;
    }

    return AndroidSdk(globals.fs.directory(androidHomeDir));
  }

  static bool validSdkDirectory(String dir) {
    return sdkDirectoryHasLicenses(dir) || sdkDirectoryHasPlatformTools(dir);
  }

  static bool sdkDirectoryHasPlatformTools(String dir) {
    return globals.fs.isDirectorySync(globals.fs.path.join(dir, 'platform-tools'));
  }

  static bool sdkDirectoryHasLicenses(String dir) {
    return globals.fs.isDirectorySync(globals.fs.path.join(dir, 'licenses'));
  }

  List<AndroidSdkVersion> get sdkVersions => _sdkVersions;

  AndroidSdkVersion? get latestVersion => _latestVersion;

  late final String? adbPath = getPlatformToolsPath(globals.platform.isWindows ? 'adb.exe' : 'adb');

  String? get emulatorPath => getEmulatorPath();

  String? get avdManagerPath => getAvdManagerPath();

  /// Locate the path for storing AVD emulator images. Returns null if none found.
  String? getAvdPath() {
    final String? avdHome = globals.platform.environment['ANDROID_AVD_HOME'];
    final String? home = globals.platform.environment['HOME'];
    final List<String> searchPaths = <String>[
      if (avdHome != null) avdHome,
      if (home != null) globals.fs.path.join(home, '.android', 'avd'),
    ];

    if (globals.platform.isWindows) {
      final String? homeDrive = globals.platform.environment['HOMEDRIVE'];
      final String? homePath = globals.platform.environment['HOMEPATH'];

      if (homeDrive != null && homePath != null) {
        // Can't use path.join for HOMEDRIVE/HOMEPATH
        // https://github.com/dart-lang/path/issues/37
        final String home = homeDrive + homePath;
        searchPaths.add(globals.fs.path.join(home, '.android', 'avd'));
      }
    }

    for (final String searchPath in searchPaths) {
      if (globals.fs.directory(searchPath).existsSync()) {
        return searchPath;
      }
    }
    return null;
  }

  Directory get _platformsDir => directory.childDirectory('platforms');

  Iterable<Directory> get _platforms {
    Iterable<Directory> platforms = <Directory>[];
    if (_platformsDir.existsSync()) {
      platforms = _platformsDir.listSync().whereType<Directory>();
    }
    return platforms;
  }

  /// Validate the Android SDK. This returns an empty list if there are no
  /// issues; otherwise, it returns a list of issues found.
  List<String> validateSdkWellFormed() {
    if (adbPath == null || !globals.processManager.canRun(adbPath)) {
      return <String>['Android SDK file not found: ${adbPath ?? 'adb'}.'];
    }

    if (sdkVersions.isEmpty || latestVersion == null) {
      final StringBuffer msg = StringBuffer(
        'No valid Android SDK platforms found in ${_platformsDir.path}.',
      );
      if (_platforms.isEmpty) {
        msg.write(' Directory was empty.');
      } else {
        msg.write(' Candidates were:\n');
        msg.write(_platforms.map((Directory dir) => '  - ${dir.basename}').join('\n'));
      }
      return <String>[msg.toString()];
    }

    return latestVersion!.validateSdkWellFormed();
  }

  String? getPlatformToolsPath(String binaryName) {
    final File cmdlineToolsBinary = directory.childDirectory('cmdline-tools').childFile(binaryName);
    if (cmdlineToolsBinary.existsSync()) {
      return cmdlineToolsBinary.path;
    }
    final File platformToolBinary = directory
        .childDirectory('platform-tools')
        .childFile(binaryName);
    if (platformToolBinary.existsSync()) {
      return platformToolBinary.path;
    }
    return null;
  }

  String? getEmulatorPath() {
    final String binaryName = globals.platform.isWindows ? 'emulator.exe' : 'emulator';
    // Emulator now lives inside "emulator" but used to live inside "tools" so
    // try both.
    final List<String> searchFolders = <String>['emulator', 'tools'];
    for (final String folder in searchFolders) {
      final File file = directory.childDirectory(folder).childFile(binaryName);
      if (file.existsSync()) {
        return file.path;
      }
    }
    return null;
  }

  String? getCmdlineToolsPath(String binaryName, {bool skipOldTools = false}) {
    // First look for the latest version of the command-line tools
    final File cmdlineToolsLatestBinary = directory
        .childDirectory('cmdline-tools')
        .childDirectory('latest')
        .childDirectory('bin')
        .childFile(binaryName);
    if (cmdlineToolsLatestBinary.existsSync()) {
      return cmdlineToolsLatestBinary.path;
    }

    // Next look for the highest version of the command-line tools
    final Directory cmdlineToolsDir = directory.childDirectory('cmdline-tools');
    if (cmdlineToolsDir.existsSync()) {
      final List<Version> cmdlineTools =
          cmdlineToolsDir
              .listSync()
              .whereType<Directory>()
              .map((Directory subDirectory) {
                try {
                  return Version.parse(subDirectory.basename);
                } on Exception {
                  return null;
                }
              })
              .whereType<Version>()
              .toList();
      cmdlineTools.sort();

      for (final Version cmdlineToolsVersion in cmdlineTools.reversed) {
        final File cmdlineToolsBinary = directory
            .childDirectory('cmdline-tools')
            .childDirectory(cmdlineToolsVersion.toString())
            .childDirectory('bin')
            .childFile(binaryName);
        if (cmdlineToolsBinary.existsSync()) {
          return cmdlineToolsBinary.path;
        }
      }
    }
    if (skipOldTools) {
      return null;
    }

    // Finally fallback to the old SDK tools
    final File toolsBinary = directory
        .childDirectory('tools')
        .childDirectory('bin')
        .childFile(binaryName);
    if (toolsBinary.existsSync()) {
      return toolsBinary.path;
    }

    return null;
  }

  String? getAvdManagerPath() =>
      getCmdlineToolsPath(globals.platform.isWindows ? 'avdmanager.bat' : 'avdmanager');

  /// From https://developer.android.com/ndk/guides/other_build_systems.
  static const Map<String, String> _llvmHostDirectoryName = <String, String>{
    'macos': 'darwin-x86_64',
    'linux': 'linux-x86_64',
    'windows': 'windows-x86_64',
  };

  /// Locates the binary path for an NDK binary.
  ///
  /// The order of resolution is as follows:
  ///
  /// 1. If [globals.config] defines an `'android-ndk'` use that.
  /// 2. If the environment variable `ANDROID_NDK_HOME` is defined, use that.
  /// 3. If the environment variable `ANDROID_NDK_PATH` is defined, use that.
  /// 4. If the environment variable `ANDROID_NDK_ROOT` is defined, use that.
  /// 5. Look for the default install location inside the Android SDK:
  ///    [directory]/ndk/\<version\>/. If multiple versions exist, use the
  ///    newest.
  String? getNdkBinaryPath(String binaryName, {Platform? platform, Config? config}) {
    platform ??= globals.platform;
    config ??= globals.config;
    Directory? findAndroidNdkHomeDir() {
      String? androidNdkHomeDir;
      if (config!.containsKey('android-ndk')) {
        androidNdkHomeDir = config.getValue('android-ndk') as String?;
      } else if (platform!.environment.containsKey(kAndroidNdkHome)) {
        androidNdkHomeDir = platform.environment[kAndroidNdkHome];
      } else if (platform.environment.containsKey(kAndroidNdkPath)) {
        androidNdkHomeDir = platform.environment[kAndroidNdkPath];
      } else if (platform.environment.containsKey(kAndroidNdkRoot)) {
        androidNdkHomeDir = platform.environment[kAndroidNdkRoot];
      }
      if (androidNdkHomeDir != null) {
        return directory.fileSystem.directory(androidNdkHomeDir);
      }

      // Look for the default install location of the NDK inside the Android
      // SDK when installed through `sdkmanager` or Android studio.
      final Directory ndk = directory.childDirectory('ndk');
      if (!ndk.existsSync()) {
        return null;
      }
      final List<Version> ndkVersions =
          ndk
              .listSync()
              .map((FileSystemEntity entity) {
                try {
                  return Version.parse(entity.basename);
                } on Exception {
                  return null;
                }
              })
              .whereType<Version>()
              .toList()
            // Use latest NDK first.
            ..sort((Version a, Version b) => -a.compareTo(b));
      if (ndkVersions.isEmpty) {
        return null;
      }
      return ndk.childDirectory(ndkVersions.first.toString());
    }

    final Directory? androidNdkHomeDir = findAndroidNdkHomeDir();
    if (androidNdkHomeDir == null) {
      return null;
    }
    final File executable = androidNdkHomeDir
        .childDirectory('toolchains')
        .childDirectory('llvm')
        .childDirectory('prebuilt')
        .childDirectory(_llvmHostDirectoryName[platform.operatingSystem]!)
        .childDirectory('bin')
        .childFile(binaryName);
    if (executable.existsSync()) {
      // LLVM missing in this NDK version.
      return executable.path;
    }
    return null;
  }

  String? getNdkClangPath({Platform? platform, Config? config}) {
    platform ??= globals.platform;
    return getNdkBinaryPath(
      platform.isWindows ? 'clang.exe' : 'clang',
      platform: platform,
      config: config,
    );
  }

  String? getNdkArPath({Platform? platform, Config? config}) {
    platform ??= globals.platform;
    return getNdkBinaryPath(
      platform.isWindows ? 'llvm-ar.exe' : 'llvm-ar',
      platform: platform,
      config: config,
    );
  }

  String? getNdkLdPath({Platform? platform, Config? config}) {
    platform ??= globals.platform;
    return getNdkBinaryPath(
      platform.isWindows ? 'ld.lld.exe' : 'ld.lld',
      platform: platform,
      config: config,
    );
  }

  /// Sets up various paths used internally.
  ///
  /// This method should be called in a case where the tooling may have updated
  /// SDK artifacts, such as after running a gradle build.
  void reinitialize({FileSystem? fileSystem}) {
    List<Version> buildTools = <Version>[]; // 19.1.0, 22.0.1, ...

    final Directory buildToolsDir = directory.childDirectory('build-tools');
    if (buildToolsDir.existsSync()) {
      buildTools =
          buildToolsDir
              .listSync()
              .map((FileSystemEntity entity) {
                try {
                  return Version.parse(entity.basename);
                } on Exception {
                  return null;
                }
              })
              .whereType<Version>()
              .toList();
    }

    // Match up platforms with the best corresponding build-tools.
    _sdkVersions =
        _platforms
            .map<AndroidSdkVersion?>((Directory platformDir) {
              final String platformName = platformDir.basename;
              int platformVersion;

              try {
                final Match? numberedVersion = _numberedAndroidPlatformRe.firstMatch(platformName);
                if (numberedVersion != null) {
                  platformVersion = int.parse(numberedVersion.group(1)!);
                } else {
                  final String buildProps = platformDir.childFile('build.prop').readAsStringSync();
                  final Iterable<Match> versionMatches =
                      const LineSplitter()
                          .convert(buildProps)
                          .map<RegExpMatch?>(_sdkVersionRe.firstMatch)
                          .whereType<Match>();

                  if (versionMatches.isEmpty) {
                    return null;
                  }

                  final String? versionString = versionMatches.first.group(1);
                  if (versionString == null) {
                    return null;
                  }
                  platformVersion = int.parse(versionString);
                }
              } on Exception {
                return null;
              }

              Version? buildToolsVersion = Version.primary(
                buildTools.where((Version version) {
                  return version.major == platformVersion;
                }).toList(),
              );

              buildToolsVersion ??= Version.primary(buildTools);

              if (buildToolsVersion == null) {
                return null;
              }

              return AndroidSdkVersion._(
                this,
                sdkLevel: platformVersion,
                platformName: platformName,
                buildToolsVersion: buildToolsVersion,
                fileSystem: fileSystem ?? globals.fs,
              );
            })
            .whereType<AndroidSdkVersion>()
            .toList();

    _sdkVersions.sort();

    _latestVersion = _sdkVersions.isEmpty ? null : _sdkVersions.last;
  }

  /// Returns the filesystem path of the Android SDK manager tool.
  String? get sdkManagerPath {
    final String executable = globals.platform.isWindows ? 'sdkmanager.bat' : 'sdkmanager';
    return getCmdlineToolsPath(executable, skipOldTools: true);
  }

  /// Returns the version of the Android SDK manager tool or null if not found.
  String? get sdkManagerVersion {
    if (sdkManagerPath == null || !globals.processManager.canRun(sdkManagerPath)) {
      throwToolExit(
        'Android sdkmanager not found. Update to the latest Android SDK and ensure that '
        'the cmdline-tools are installed to resolve this.',
      );
    }
    final RunResult result = globals.processUtils.runSync(<String>[
      sdkManagerPath!,
      '--version',
    ], environment: _java?.environment);
    if (result.exitCode != 0) {
      globals.printTrace(
        'sdkmanager --version failed: exitCode: ${result.exitCode} stdout: ${result.stdout} stderr: ${result.stderr}',
      );
      return null;
    }
    return result.stdout.trim();
  }

  @override
  String toString() => 'AndroidSdk: $directory';
}

class AndroidSdkVersion implements Comparable<AndroidSdkVersion> {
  AndroidSdkVersion._(
    this.sdk, {
    required this.sdkLevel,
    required this.platformName,
    required this.buildToolsVersion,
    required FileSystem fileSystem,
  }) : _fileSystem = fileSystem;

  final AndroidSdk sdk;
  final int sdkLevel;
  final String platformName;
  final Version buildToolsVersion;

  final FileSystem _fileSystem;

  String get buildToolsVersionName => buildToolsVersion.toString();

  String get androidJarPath => getPlatformsPath('android.jar');

  /// Return the path to the android application package tool.
  ///
  /// This is used to dump the xml in order to launch built android applications.
  ///
  /// See also:
  ///   * [AndroidApk.fromApk], which depends on this to determine application identifiers.
  String get aaptPath => getBuildToolsPath('aapt');

  List<String> validateSdkWellFormed() {
    final String? existsAndroidJarPath = _exists(androidJarPath);
    if (existsAndroidJarPath != null) {
      return <String>[existsAndroidJarPath];
    }

    final String? canRunAaptPath = _canRun(aaptPath);
    if (canRunAaptPath != null) {
      return <String>[canRunAaptPath];
    }

    return <String>[];
  }

  String getPlatformsPath(String itemName) {
    return sdk.directory
        .childDirectory('platforms')
        .childDirectory(platformName)
        .childFile(itemName)
        .path;
  }

  String getBuildToolsPath(String binaryName) {
    return sdk.directory
        .childDirectory('build-tools')
        .childDirectory(buildToolsVersionName)
        .childFile(binaryName)
        .path;
  }

  @override
  int compareTo(AndroidSdkVersion other) => sdkLevel - other.sdkLevel;

  @override
  String toString() =>
      '[${sdk.directory}, SDK version $sdkLevel, build-tools $buildToolsVersionName]';

  String? _exists(String path) {
    if (!_fileSystem.isFileSync(path)) {
      return 'Android SDK file not found: $path.';
    }
    return null;
  }

  String? _canRun(String path) {
    if (!globals.processManager.canRun(path)) {
      return 'Android SDK file not found: $path.';
    }
    return null;
  }
}
