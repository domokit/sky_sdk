// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';

import 'android/java.dart';
import 'base/common.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/os.dart' show OperatingSystemUtils;
import 'base/platform.dart';
import 'base/process.dart';
import 'cache.dart';
import 'dart/package_map.dart';
import 'dart/pub.dart';
import 'globals.dart' as globals;
import 'project.dart';

/// An implementation of the [Cache] which provides all of Flutter's default artifacts.
class FlutterCache extends Cache {
  /// [rootOverride] is configurable for testing.
  /// [artifacts] is configurable for testing.
  FlutterCache({
    required Logger logger,
    required super.fileSystem,
    required Platform platform,
    required super.osUtils,
    required FlutterProjectFactory projectFactory,
  }) : super(logger: logger, platform: platform, artifacts: <ArtifactSet>[]) {
    registerArtifact(MaterialFonts(this));
    registerArtifact(GradleWrapper(this));
    registerArtifact(AndroidGenSnapshotArtifacts(this, platform: platform));
    registerArtifact(AndroidInternalBuildArtifacts(this));
    registerArtifact(IOSEngineArtifacts(this, platform: platform));
    registerArtifact(FlutterWebSdk(this));
    registerArtifact(LegacyCanvasKitRemover(this));
    registerArtifact(FlutterSdk(this, platform: platform));
    registerArtifact(WindowsEngineArtifacts(this, platform: platform));
    registerArtifact(MacOSEngineArtifacts(this, platform: platform));
    registerArtifact(LinuxEngineArtifacts(this, platform: platform));
    registerArtifact(LinuxFuchsiaSDKArtifacts(this, platform: platform));
    registerArtifact(MacOSFuchsiaSDKArtifacts(this, platform: platform));
    registerArtifact(FlutterRunnerSDKArtifacts(this, platform: platform));
    registerArtifact(FlutterRunnerDebugSymbols(this, platform: platform));
    for (final String artifactName in IosUsbArtifacts.artifactNames) {
      registerArtifact(IosUsbArtifacts(artifactName, this, platform: platform));
    }
    registerArtifact(FontSubsetArtifacts(this, platform: platform));
    registerArtifact(
      PubDependencies(
        logger: logger,
        // flutter root and pub must be lazily initialized to avoid accessing
        // before the version is determined.
        flutterRoot: () => Cache.flutterRoot!,
        pub: () => pub,
        projectFactory: projectFactory,
      ),
    );
  }
}

/// Ensures that the source files for all of the dependencies for the
/// flutter_tool are present.
///
/// This does not handle cases where the source files are modified or the
/// directory contents are incomplete.
class PubDependencies extends ArtifactSet {
  PubDependencies({
    // Needs to be lazy to avoid reading from the cache before the root is initialized.
    required String Function() flutterRoot,
    required Logger logger,
    required Pub Function() pub,
    required FlutterProjectFactory projectFactory,
  }) : _logger = logger,
       _flutterRoot = flutterRoot,
       _pub = pub,
       _projectFactory = projectFactory,
       super(DevelopmentArtifact.universal);

  final String Function() _flutterRoot;
  final Logger _logger;
  final Pub Function() _pub;
  final FlutterProjectFactory _projectFactory;

  @override
  Future<bool> isUpToDate(FileSystem fileSystem) async {
    final File toolPackageConfig = fileSystem.file(
      fileSystem.path.join(
        _flutterRoot(),
        'packages',
        'flutter_tools',
        '.dart_tool',
        'package_config.json',
      ),
    );
    if (!toolPackageConfig.existsSync()) {
      return false;
    }
    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      toolPackageConfig,
      logger: _logger,
      throwOnError: false,
    );
    if (packageConfig == PackageConfig.empty) {
      return false;
    }
    for (final Package package in packageConfig.packages) {
      if (!fileSystem.directory(package.root).childFile('pubspec.yaml').existsSync()) {
        return false;
      }
    }
    return true;
  }

  @override
  String get name => 'pub_dependencies';

  @override
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  }) async {
    await _pub().get(
      context: PubContext.pubGet,
      project: _projectFactory.fromDirectory(
        fileSystem.directory(fileSystem.path.join(_flutterRoot(), 'packages', 'flutter_tools')),
      ),
      offline: offline,
      outputMode: PubOutputMode.failuresOnly,
    );
  }
}

/// A cached artifact containing fonts used for Material Design.
class MaterialFonts extends CachedArtifact {
  MaterialFonts(Cache cache) : super('material_fonts', cache, DevelopmentArtifact.universal);

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    final Uri archiveUri = _toStorageUri(version!);
    return artifactUpdater.downloadZipArchive(
      'Downloading Material fonts...',
      archiveUri,
      location,
    );
  }

  Uri _toStorageUri(String path) => Uri.parse('${cache.storageBaseUrl}/$path');
}

/// A cached artifact containing the web dart:ui sources, platform dill files,
/// and libraries.json.
///
/// This SDK references code within the regular Dart sdk to reduce download size.
class FlutterWebSdk extends CachedArtifact {
  FlutterWebSdk(Cache cache) : super('flutter_web_sdk', cache, DevelopmentArtifact.web);

  @override
  Directory get location => cache.getWebSdkDirectory();

  @override
  String? get version => cache.getVersionFor('engine');

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    final Uri url = Uri.parse(
      '${cache.storageBaseUrl}/flutter_infra_release/flutter/$version/flutter-web-sdk.zip',
    );
    ErrorHandlingFileSystem.deleteIfExists(location, recursive: true);
    await artifactUpdater.downloadZipArchive('Downloading Web SDK...', url, location);
  }
}

// In previous builds, CanvasKit artifacts were stored in a different location
// than they are now. Leaving those old artifacts in the cache confuses the
// in-memory filesystem that the web runner uses, so this artifact will evict
// them from our cache if they are there.
class LegacyCanvasKitRemover extends ArtifactSet {
  LegacyCanvasKitRemover(this.cache) : super(DevelopmentArtifact.web);

  final Cache cache;

  @override
  String get name => 'legacy_canvaskit_remover';

  Directory _getLegacyCanvasKitDirectory(FileSystem fileSystem) =>
      fileSystem.directory(fileSystem.path.join(cache.getRoot().path, 'canvaskit'));

  @override
  Future<bool> isUpToDate(FileSystem fileSystem) async =>
      !(await _getLegacyCanvasKitDirectory(fileSystem).exists());

  @override
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  }) => _getLegacyCanvasKitDirectory(fileSystem).delete(recursive: true);
}

/// A cached artifact containing the dart:ui source code.
class FlutterSdk extends EngineCachedArtifact {
  FlutterSdk(Cache cache, {required Platform platform})
    : _platform = platform,
      super('flutter_sdk', cache, DevelopmentArtifact.universal);

  final Platform _platform;

  @override
  List<String> getPackageDirs() => const <String>['sky_engine', 'flutter_gpu'];

  @override
  List<List<String>> getBinaryDirs() {
    // Linux and Windows both support arm64 and x64.
    final String arch = cache.getHostPlatformArchName();
    return <List<String>>[
      <String>['common', 'flutter_patched_sdk.zip'],
      <String>['common', 'flutter_patched_sdk_product.zip'],
      if (cache.includeAllPlatforms) ...<List<String>>[
        <String>['windows-$arch', 'windows-$arch/artifacts.zip'],
        <String>['linux-$arch', 'linux-$arch/artifacts.zip'],
        <String>['darwin-x64', 'darwin-$arch/artifacts.zip'],
      ] else if (_platform.isWindows)
        <String>['windows-$arch', 'windows-$arch/artifacts.zip']
      else if (_platform.isMacOS)
        <String>['darwin-x64', 'darwin-$arch/artifacts.zip']
      else if (_platform.isLinux)
        <String>['linux-$arch', 'linux-$arch/artifacts.zip'],
    ];
  }

  @override
  List<String> getLicenseDirs() => const <String>[];
}

class MacOSEngineArtifacts extends EngineCachedArtifact {
  MacOSEngineArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('macos-sdk', cache, DevelopmentArtifact.macOS);

  final Platform _platform;

  @override
  List<String> getPackageDirs() => const <String>[];

  @override
  List<List<String>> getBinaryDirs() {
    if (_platform.isMacOS || ignorePlatformFiltering) {
      return _macOSDesktopBinaryDirs;
    }
    return const <List<String>>[];
  }

  @override
  List<String> getLicenseDirs() => const <String>[];
}

/// Artifacts required for desktop Windows builds.
class WindowsEngineArtifacts extends EngineCachedArtifact {
  WindowsEngineArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('windows-sdk', cache, DevelopmentArtifact.windows);

  final Platform _platform;

  @override
  List<String> getPackageDirs() => const <String>[];

  @override
  List<List<String>> getBinaryDirs() {
    if (_platform.isWindows || ignorePlatformFiltering) {
      final String arch = cache.getHostPlatformArchName();
      return _getWindowsDesktopBinaryDirs(arch);
    }
    return const <List<String>>[];
  }

  @override
  List<String> getLicenseDirs() => const <String>[];
}

/// Artifacts required for desktop Linux builds.
class LinuxEngineArtifacts extends EngineCachedArtifact {
  LinuxEngineArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('linux-sdk', cache, DevelopmentArtifact.linux);

  final Platform _platform;

  @override
  List<String> getPackageDirs() => const <String>[];

  @override
  List<List<String>> getBinaryDirs() {
    if (_platform.isLinux || ignorePlatformFiltering) {
      final String arch = cache.getHostPlatformArchName();
      return <List<String>>[
        <String>['linux-$arch', 'linux-$arch-debug/linux-$arch-flutter-gtk.zip'],
        <String>['linux-$arch-profile', 'linux-$arch-profile/linux-$arch-flutter-gtk.zip'],
        <String>['linux-$arch-release', 'linux-$arch-release/linux-$arch-flutter-gtk.zip'],
      ];
    }
    return const <List<String>>[];
  }

  @override
  List<String> getLicenseDirs() => const <String>[];
}

/// The artifact used to generate snapshots for Android builds.
class AndroidGenSnapshotArtifacts extends EngineCachedArtifact {
  AndroidGenSnapshotArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('android-sdk', cache, DevelopmentArtifact.androidGenSnapshot);

  final Platform _platform;

  @override
  List<String> getPackageDirs() => const <String>[];

  @override
  List<List<String>> getBinaryDirs() {
    return <List<String>>[
      if (cache.includeAllPlatforms) ...<List<String>>[
        ..._osxBinaryDirs,
        ..._linuxBinaryDirs,
        ..._windowsBinaryDirs,
        ..._dartSdks,
      ] else if (_platform.isWindows)
        ..._windowsBinaryDirs
      else if (_platform.isMacOS)
        ..._osxBinaryDirs
      else if (_platform.isLinux)
        ..._linuxBinaryDirs,
    ];
  }

  @override
  List<String> getLicenseDirs() {
    return <String>[];
  }
}

/// A cached artifact containing the Maven dependencies used to build Android projects.
///
/// This is a no-op if the android SDK is not available.
///
/// Set [Java] to `null` to indicate that no Java/JDK installation could be found.
class AndroidMavenArtifacts extends ArtifactSet {
  AndroidMavenArtifacts(this.cache, {required Java? java, required Platform platform})
    : _java = java,
      _platform = platform,
      super(DevelopmentArtifact.androidMaven);

  final Java? _java;
  final Platform _platform;
  final Cache cache;

  @override
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  }) async {
    // TODO(andrewkolos): Should this really be no-op if the Android SDK
    // is unavailable? https://github.com/flutter/flutter/issues/127848
    if (globals.androidSdk == null) {
      return;
    }
    final Directory tempDir = cache.getRoot().createTempSync('flutter_gradle_wrapper.');
    globals.gradleUtils?.injectGradleWrapperIfNeeded(tempDir);

    final Status status = logger.startProgress('Downloading Android Maven dependencies...');
    final File gradle = tempDir.childFile(_platform.isWindows ? 'gradlew.bat' : 'gradlew');
    try {
      final String gradleExecutable = gradle.absolute.path;
      final String flutterSdk = globals.fsUtils.escapePath(Cache.flutterRoot!);
      final RunResult processResult = await globals.processUtils.run(<String>[
        gradleExecutable,
        '-b',
        globals.fs.path.join(
          flutterSdk,
          'packages',
          'flutter_tools',
          'gradle',
          'resolve_dependencies.gradle',
        ),
        '--project-cache-dir',
        tempDir.path,
        'resolveDependencies',
      ], environment: _java?.environment);
      if (processResult.exitCode != 0) {
        logger.printError('Failed to download the Android dependencies');
      }
    } finally {
      status.stop();
      tempDir.deleteSync(recursive: true);
      globals.androidSdk?.reinitialize();
    }
  }

  @override
  Future<bool> isUpToDate(FileSystem fileSystem) async {
    // The dependencies are downloaded and cached by Gradle.
    // The tool doesn't know if the dependencies are already cached at this point.
    // Therefore, call Gradle to figure this out.
    return false;
  }

  @override
  String get name => 'android-maven-artifacts';
}

/// Artifacts used for internal builds. The flutter tool builds Android projects
/// using the artifacts cached by [AndroidMavenArtifacts].
class AndroidInternalBuildArtifacts extends EngineCachedArtifact {
  AndroidInternalBuildArtifacts(Cache cache)
    : super('android-internal-build-artifacts', cache, DevelopmentArtifact.androidInternalBuild);

  @override
  List<String> getPackageDirs() => const <String>[];

  @override
  List<List<String>> getBinaryDirs() {
    return _androidBinaryDirs;
  }

  @override
  List<String> getLicenseDirs() {
    return <String>[];
  }
}

class IOSEngineArtifacts extends EngineCachedArtifact {
  IOSEngineArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('ios-sdk', cache, DevelopmentArtifact.iOS);

  final Platform _platform;

  @override
  List<List<String>> getBinaryDirs() {
    return <List<String>>[if (_platform.isMacOS || ignorePlatformFiltering) ..._iosBinaryDirs];
  }

  @override
  List<String> getLicenseDirs() {
    if (_platform.isMacOS || ignorePlatformFiltering) {
      return const <String>['ios', 'ios-profile', 'ios-release'];
    }
    return const <String>[];
  }

  @override
  List<String> getPackageDirs() {
    return <String>[];
  }
}

/// A cached artifact containing Gradle Wrapper scripts and binaries.
///
/// While this is only required for Android, we need to always download it due
/// the ensurePlatformSpecificTooling logic.
class GradleWrapper extends CachedArtifact {
  GradleWrapper(Cache cache) : super('gradle_wrapper', cache, DevelopmentArtifact.universal);

  List<String> get _gradleScripts => <String>['gradlew', 'gradlew.bat'];

  Uri _toStorageUri(String path) => Uri.parse('${cache.storageBaseUrl}/$path');

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    final Uri archiveUri = _toStorageUri(version!);
    await artifactUpdater.downloadZippedTarball(
      'Downloading Gradle Wrapper...',
      archiveUri,
      location,
    );
    // Delete property file, allowing templates to provide it.
    // Remove NOTICE file. Should not be part of the template.
    final File propertiesFile = fileSystem.file(
      fileSystem.path.join(location.path, 'gradle', 'wrapper', 'gradle-wrapper.properties'),
    );
    final File noticeFile = fileSystem.file(fileSystem.path.join(location.path, 'NOTICE'));
    ErrorHandlingFileSystem.deleteIfExists(propertiesFile);
    ErrorHandlingFileSystem.deleteIfExists(noticeFile);
  }

  @override
  bool isUpToDateInner(FileSystem fileSystem) {
    final String gradleWrapper = fileSystem.path.join('gradle', 'wrapper', 'gradle-wrapper.jar');
    final Directory wrapperDir = cache.getCacheDir(
      fileSystem.path.join('artifacts', 'gradle_wrapper'),
    );
    if (!fileSystem.directory(wrapperDir).existsSync()) {
      return false;
    }
    for (final String scriptName in _gradleScripts) {
      final File scriptFile = fileSystem.file(fileSystem.path.join(wrapperDir.path, scriptName));
      if (!scriptFile.existsSync()) {
        return false;
      }
    }
    final File gradleWrapperJar = fileSystem.file(
      fileSystem.path.join(wrapperDir.path, gradleWrapper),
    );
    if (!gradleWrapperJar.existsSync()) {
      return false;
    }
    return true;
  }
}

/// Common functionality for pulling Fuchsia SDKs.
abstract class _FuchsiaSDKArtifacts extends CachedArtifact {
  _FuchsiaSDKArtifacts(Cache cache, String platform)
    : _path = 'fuchsia/sdk/core/$platform-amd64',
      super('fuchsia-$platform', cache, DevelopmentArtifact.fuchsia);

  final String _path;

  @override
  Directory get location => cache.getArtifactDirectory('fuchsia');

  Future<void> _doUpdate(ArtifactUpdater artifactUpdater) {
    final String url = '${cache.cipdBaseUrl}/$_path/+/$version';
    return artifactUpdater.downloadZipArchive(
      'Downloading package fuchsia SDK...',
      Uri.parse(url),
      location,
    );
  }
}

/// The pre-built flutter runner for Fuchsia development.
class FlutterRunnerSDKArtifacts extends CachedArtifact {
  FlutterRunnerSDKArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super('flutter_runner', cache, DevelopmentArtifact.flutterRunner);

  final Platform _platform;

  @override
  Directory get location => cache.getArtifactDirectory('flutter_runner');

  @override
  String? get version => cache.getVersionFor('engine');

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    if (!_platform.isLinux && !_platform.isMacOS) {
      return;
    }
    final String url = '${cache.cipdBaseUrl}/flutter/fuchsia/+/git_revision:$version';
    await artifactUpdater.downloadZipArchive(
      'Downloading package flutter runner...',
      Uri.parse(url),
      location,
    );
  }
}

/// Implementations of this class can resolve URLs for packages that are versioned.
///
/// See also [CipdArchiveResolver].
abstract class VersionedPackageResolver {
  const VersionedPackageResolver();

  /// Returns the URL for the artifact.
  String resolveUrl(String packageName, String version);
}

/// Resolves the CIPD archive URL for a given package and version.
class CipdArchiveResolver extends VersionedPackageResolver {
  const CipdArchiveResolver(this.cache);

  final Cache cache;

  @override
  String resolveUrl(String packageName, String version) {
    return '${cache.cipdBaseUrl}/flutter/$packageName/+/git_revision:$version';
  }
}

/// The debug symbols for flutter runner for Fuchsia development.
class FlutterRunnerDebugSymbols extends CachedArtifact {
  FlutterRunnerDebugSymbols(
    Cache cache, {
    required Platform platform,
    VersionedPackageResolver? packageResolver,
  }) : _platform = platform,
       packageResolver = packageResolver ?? CipdArchiveResolver(cache),
       super('flutter_runner_debug_symbols', cache, DevelopmentArtifact.flutterRunner);

  final VersionedPackageResolver packageResolver;
  final Platform _platform;

  @override
  Directory get location => cache.getArtifactDirectory(name);

  @override
  String? get version => cache.getVersionFor('engine');

  Future<void> _downloadDebugSymbols(String targetArch, ArtifactUpdater artifactUpdater) async {
    final String packageName = 'fuchsia-debug-symbols-$targetArch';
    final String url = packageResolver.resolveUrl(packageName, version!);
    await artifactUpdater.downloadZipArchive(
      'Downloading debug symbols for flutter runner - arch:$targetArch...',
      Uri.parse(url),
      location,
    );
  }

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    if (!_platform.isLinux && !_platform.isMacOS) {
      return;
    }
    await _downloadDebugSymbols('x64', artifactUpdater);
    await _downloadDebugSymbols('arm64', artifactUpdater);
  }
}

/// The Fuchsia core SDK for Linux.
class LinuxFuchsiaSDKArtifacts extends _FuchsiaSDKArtifacts {
  LinuxFuchsiaSDKArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super(cache, 'linux');

  final Platform _platform;

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    if (!_platform.isLinux) {
      return;
    }
    return _doUpdate(artifactUpdater);
  }
}

/// The Fuchsia core SDK for MacOS.
class MacOSFuchsiaSDKArtifacts extends _FuchsiaSDKArtifacts {
  MacOSFuchsiaSDKArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super(cache, 'mac');

  final Platform _platform;

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    if (!_platform.isMacOS) {
      return;
    }
    return _doUpdate(artifactUpdater);
  }
}

/// Cached artifacts for font subsetting.
class FontSubsetArtifacts extends EngineCachedArtifact {
  FontSubsetArtifacts(Cache cache, {required Platform platform})
    : _platform = platform,
      super(artifactName, cache, DevelopmentArtifact.universal);

  final Platform _platform;

  static const String artifactName = 'font-subset';

  @override
  List<List<String>> getBinaryDirs() {
    // Linux and Windows both support arm64 and x64.
    final String arch = cache.getHostPlatformArchName();
    final Map<String, List<String>> artifacts = <String, List<String>>{
      'macos': <String>['darwin-x64', 'darwin-$arch/$artifactName.zip'],
      'linux': <String>['linux-$arch', 'linux-$arch/$artifactName.zip'],
      'windows': <String>['windows-$arch', 'windows-$arch/$artifactName.zip'],
    };
    if (cache.includeAllPlatforms) {
      return artifacts.values.toList();
    } else {
      final List<String>? binaryDirs = artifacts[_platform.operatingSystem];
      if (binaryDirs == null) {
        throwToolExit('Unsupported operating system: ${_platform.operatingSystem}');
      }
      return <List<String>>[binaryDirs];
    }
  }

  @override
  List<String> getLicenseDirs() => const <String>[];

  @override
  List<String> getPackageDirs() => const <String>[];
}

/// Cached iOS/USB binary artifacts.
class IosUsbArtifacts extends CachedArtifact {
  IosUsbArtifacts(String name, Cache cache, {required Platform platform})
    : _platform = platform,
      super(name, cache, DevelopmentArtifact.universal);

  final Platform _platform;

  static const List<String> artifactNames = <String>[
    'libimobiledevice',
    'usbmuxd',
    'libplist',
    'openssl',
    'ios-deploy',
  ];

  // For unknown reasons, users are getting into bad states where libimobiledevice is
  // downloaded but some executables are missing from the zip. The names here are
  // used for additional download checks below, so we can re-download if they are
  // missing.
  static const Map<String, List<String>> _kExecutables = <String, List<String>>{
    'libimobiledevice': <String>['idevicescreenshot', 'idevicesyslog'],
    'usbmuxd': <String>['iproxy'],
  };

  @override
  Map<String, String> get environment => <String, String>{
    'DYLD_LIBRARY_PATH': cache.getArtifactDirectory(name).path,
  };

  @override
  bool isUpToDateInner(FileSystem fileSystem) {
    final List<String>? executables = _kExecutables[name];
    if (executables == null) {
      return true;
    }
    for (final String executable in executables) {
      if (!location.childFile(executable).existsSync()) {
        return false;
      }
    }
    return true;
  }

  @override
  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  ) async {
    if (!_platform.isMacOS && !ignorePlatformFiltering) {
      return;
    }
    if (location.existsSync()) {
      location.deleteSync(recursive: true);
    }
    await artifactUpdater.downloadZipArchive('Downloading $name...', archiveUri, location);
  }

  @visibleForTesting
  Uri get archiveUri => Uri.parse(
    '${cache.realmlessStorageBaseUrl}/flutter_infra_release/'
    'ios-usb-dependencies${cache.useUnsignedMacBinaries ? '/unsigned' : ''}'
    '/$name/$version/$name.zip',
  );
}

// TODO(zanderso): upload debug desktop artifacts to host-debug and
// remove from existing host folder.
// https://github.com/flutter/flutter/issues/38935

List<List<String>> _getWindowsDesktopBinaryDirs(String arch) {
  return <List<String>>[
    <String>['windows-$arch', 'windows-$arch-debug/windows-$arch-flutter.zip'],
    <String>['windows-$arch', 'windows-$arch/flutter-cpp-client-wrapper.zip'],
    <String>['windows-$arch-profile', 'windows-$arch-profile/windows-$arch-flutter.zip'],
    <String>['windows-$arch-release', 'windows-$arch-release/windows-$arch-flutter.zip'],
  ];
}

const List<List<String>> _macOSDesktopBinaryDirs = <List<String>>[
  <String>['darwin-x64', 'darwin-x64/framework.zip'],
  <String>['darwin-x64', 'darwin-x64/gen_snapshot.zip'],
  <String>['darwin-x64-profile', 'darwin-x64-profile/framework.zip'],
  <String>['darwin-x64-profile', 'darwin-x64-profile/artifacts.zip'],
  <String>['darwin-x64-profile', 'darwin-x64-profile/gen_snapshot.zip'],
  <String>['darwin-x64-release', 'darwin-x64-release/framework.zip'],
  <String>['darwin-x64-release', 'darwin-x64-release/artifacts.zip'],
  <String>['darwin-x64-release', 'darwin-x64-release/gen_snapshot.zip'],
];

const List<List<String>> _osxBinaryDirs = <List<String>>[
  <String>['android-arm-profile/darwin-x64', 'android-arm-profile/darwin-x64.zip'],
  <String>['android-arm-release/darwin-x64', 'android-arm-release/darwin-x64.zip'],
  <String>['android-arm64-profile/darwin-x64', 'android-arm64-profile/darwin-x64.zip'],
  <String>['android-arm64-release/darwin-x64', 'android-arm64-release/darwin-x64.zip'],
  <String>['android-x64-profile/darwin-x64', 'android-x64-profile/darwin-x64.zip'],
  <String>['android-x64-release/darwin-x64', 'android-x64-release/darwin-x64.zip'],
];

const List<List<String>> _linuxBinaryDirs = <List<String>>[
  <String>['android-arm-profile/linux-x64', 'android-arm-profile/linux-x64.zip'],
  <String>['android-arm-release/linux-x64', 'android-arm-release/linux-x64.zip'],
  <String>['android-arm64-profile/linux-x64', 'android-arm64-profile/linux-x64.zip'],
  <String>['android-arm64-release/linux-x64', 'android-arm64-release/linux-x64.zip'],
  <String>['android-x64-profile/linux-x64', 'android-x64-profile/linux-x64.zip'],
  <String>['android-x64-release/linux-x64', 'android-x64-release/linux-x64.zip'],
];

const List<List<String>> _windowsBinaryDirs = <List<String>>[
  <String>['android-arm-profile/windows-x64', 'android-arm-profile/windows-x64.zip'],
  <String>['android-arm-release/windows-x64', 'android-arm-release/windows-x64.zip'],
  <String>['android-arm64-profile/windows-x64', 'android-arm64-profile/windows-x64.zip'],
  <String>['android-arm64-release/windows-x64', 'android-arm64-release/windows-x64.zip'],
  <String>['android-x64-profile/windows-x64', 'android-x64-profile/windows-x64.zip'],
  <String>['android-x64-release/windows-x64', 'android-x64-release/windows-x64.zip'],
];

const List<List<String>> _iosBinaryDirs = <List<String>>[
  <String>['ios', 'ios/artifacts.zip'],
  <String>['ios-profile', 'ios-profile/artifacts.zip'],
  <String>['ios-release', 'ios-release/artifacts.zip'],
];

const List<List<String>> _androidBinaryDirs = <List<String>>[
  <String>['android-x86', 'android-x86/artifacts.zip'],
  <String>['android-x64', 'android-x64/artifacts.zip'],
  <String>['android-arm', 'android-arm/artifacts.zip'],
  <String>['android-arm-profile', 'android-arm-profile/artifacts.zip'],
  <String>['android-arm-release', 'android-arm-release/artifacts.zip'],
  <String>['android-arm64', 'android-arm64/artifacts.zip'],
  <String>['android-arm64-profile', 'android-arm64-profile/artifacts.zip'],
  <String>['android-arm64-release', 'android-arm64-release/artifacts.zip'],
  <String>['android-x64-profile', 'android-x64-profile/artifacts.zip'],
  <String>['android-x64-release', 'android-x64-release/artifacts.zip'],
];

const List<List<String>> _dartSdks = <List<String>>[
  <String>['darwin-x64', 'dart-sdk-darwin-x64.zip'],
  <String>['linux-x64', 'dart-sdk-linux-x64.zip'],
  <String>['windows-x64', 'dart-sdk-windows-x64.zip'],
];
