// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:crypto/crypto.dart';
import 'package:file/memory.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import 'base/common.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/io.dart'
    show
        HttpClient,
        HttpClientRequest,
        HttpClientResponse,
        HttpHeaders,
        HttpStatus,
        SocketException;
import 'base/logger.dart';
import 'base/net.dart';
import 'base/os.dart' show OperatingSystemUtils;
import 'base/platform.dart';
import 'base/terminal.dart';
import 'base/user_messages.dart';
import 'convert.dart';
import 'features.dart';

const String kFlutterRootEnvironmentVariableName =
    'FLUTTER_ROOT'; // should point to //flutter/ (root of flutter/flutter repo)
const String kFlutterEngineEnvironmentVariableName =
    'FLUTTER_ENGINE'; // should point to //engine/src/ (root of flutter/engine repo)
const String kSnapshotFileName = 'flutter_tools.snapshot'; // in //flutter/bin/cache/
const String kFlutterToolsScriptFileName =
    'flutter_tools.dart'; // in //flutter/packages/flutter_tools/bin/
const String kFlutterEnginePackageName = 'sky_engine';

/// A tag for a set of development artifacts that need to be cached.
class DevelopmentArtifact {
  const DevelopmentArtifact._(this.name, {this.feature});

  /// The name of the artifact.
  ///
  /// This should match the flag name in precache.dart.
  final String name;

  /// A feature to control the visibility of this artifact.
  final Feature? feature;

  /// Artifacts required for Android development.
  static const DevelopmentArtifact androidGenSnapshot = DevelopmentArtifact._(
    'android_gen_snapshot',
    feature: flutterAndroidFeature,
  );
  static const DevelopmentArtifact androidMaven = DevelopmentArtifact._(
    'android_maven',
    feature: flutterAndroidFeature,
  );

  // Artifacts used for internal builds.
  static const DevelopmentArtifact androidInternalBuild = DevelopmentArtifact._(
    'android_internal_build',
    feature: flutterAndroidFeature,
  );

  /// Artifacts required for iOS development.
  static const DevelopmentArtifact iOS = DevelopmentArtifact._('ios', feature: flutterIOSFeature);

  /// Artifacts required for web development.
  static const DevelopmentArtifact web = DevelopmentArtifact._('web', feature: flutterWebFeature);

  /// Artifacts required for desktop macOS.
  static const DevelopmentArtifact macOS = DevelopmentArtifact._(
    'macos',
    feature: flutterMacOSDesktopFeature,
  );

  /// Artifacts required for desktop Windows.
  static const DevelopmentArtifact windows = DevelopmentArtifact._(
    'windows',
    feature: flutterWindowsDesktopFeature,
  );

  /// Artifacts required for desktop Linux.
  static const DevelopmentArtifact linux = DevelopmentArtifact._(
    'linux',
    feature: flutterLinuxDesktopFeature,
  );

  /// Artifacts required for Fuchsia.
  static const DevelopmentArtifact fuchsia = DevelopmentArtifact._(
    'fuchsia',
    feature: flutterFuchsiaFeature,
  );

  /// Artifacts required for the Flutter Runner.
  static const DevelopmentArtifact flutterRunner = DevelopmentArtifact._(
    'flutter_runner',
    feature: flutterFuchsiaFeature,
  );

  /// Artifacts required for any development platform.
  ///
  /// This does not need to be explicitly returned from requiredArtifacts as
  /// it will always be downloaded.
  static const DevelopmentArtifact universal = DevelopmentArtifact._('universal');

  /// The values of DevelopmentArtifacts.
  static final List<DevelopmentArtifact> values = <DevelopmentArtifact>[
    androidGenSnapshot,
    androidMaven,
    androidInternalBuild,
    iOS,
    web,
    macOS,
    windows,
    linux,
    fuchsia,
    universal,
    flutterRunner,
  ];

  @override
  String toString() => 'Artifact($name)';
}

/// A wrapper around the `bin/cache/` directory.
///
/// This does not provide any artifacts by default. See [FlutterCache] for the default
/// artifact set.
///
/// ## Artifact mirrors
///
/// Some environments cannot reach the Google Cloud Storage buckets and CIPD due
/// to regional or corporate policies.
///
/// To enable Flutter users in these environments, the Flutter tool supports
/// custom artifact mirrors that the administrators of such environments may
/// provide. To use an artifact mirror, the user defines the [kFlutterStorageBaseUrl]
/// (`FLUTTER_STORAGE_BASE_URL`) environment variable that points to the mirror.
/// Flutter tool reads this variable and uses it instead of the default URLs.
///
/// For more details on specific URLs used to download artifacts, see
/// [storageBaseUrl] and [cipdBaseUrl].
class Cache {
  /// [rootOverride] is configurable for testing.
  /// [artifacts] is configurable for testing.
  Cache({
    @protected Directory? rootOverride,
    @protected List<ArtifactSet>? artifacts,
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
    required OperatingSystemUtils osUtils,
  }) : _rootOverride = rootOverride,
       _logger = logger,
       _fileSystem = fileSystem,
       _platform = platform,
       _osUtils = osUtils,
       _net = Net(logger: logger, platform: platform),
       _fsUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform),
       _artifacts = artifacts ?? <ArtifactSet>[];

  /// Create a [Cache] for testing.
  ///
  /// Defaults to a memory file system, fake platform,
  /// buffer logger, and no accessible artifacts.
  /// By default, the root cache directory path is "cache".
  factory Cache.test({
    Directory? rootOverride,
    List<ArtifactSet>? artifacts,
    Logger? logger,
    FileSystem? fileSystem,
    Platform? platform,
    required ProcessManager processManager,
  }) {
    if (rootOverride?.fileSystem != null &&
        fileSystem != null &&
        rootOverride!.fileSystem != fileSystem) {
      throw ArgumentError(
        'If rootOverride and fileSystem are both non-null, '
            'rootOverride.fileSystem must be the same as fileSystem.',
        'fileSystem',
      );
    }
    fileSystem ??= rootOverride?.fileSystem ?? MemoryFileSystem.test();
    platform ??= FakePlatform(environment: <String, String>{});
    logger ??= BufferLogger.test();
    return Cache(
      rootOverride: rootOverride ?? fileSystem.currentDirectory,
      artifacts: artifacts ?? <ArtifactSet>[],
      logger: logger,
      fileSystem: fileSystem,
      platform: platform,
      osUtils: OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
      ),
    );
  }

  final Logger _logger;
  final Platform _platform;
  final FileSystem _fileSystem;
  final OperatingSystemUtils _osUtils;
  final Directory? _rootOverride;
  final List<ArtifactSet> _artifacts;
  final Net _net;
  final FileSystemUtils _fsUtils;

  late final ArtifactUpdater _artifactUpdater = _createUpdater();

  @visibleForTesting
  @protected
  void registerArtifact(ArtifactSet artifactSet) {
    _artifacts.add(artifactSet);
  }

  /// This has to be lazy because it requires FLUTTER_ROOT to be initialized.
  ArtifactUpdater _createUpdater() {
    return ArtifactUpdater(
      operatingSystemUtils: _osUtils,
      logger: _logger,
      fileSystem: _fileSystem,
      tempStorage: getDownloadDir(),
      platform: _platform,
      httpClient: HttpClient(),
      allowedBaseUrls: <String>[storageBaseUrl, realmlessStorageBaseUrl, cipdBaseUrl],
    );
  }

  static const List<String> _hostsBlockedInChina = <String>[
    'storage.googleapis.com',
    'chrome-infra-packages.appspot.com',
  ];

  // Initialized by FlutterCommandRunner on startup.
  // Explore making this field lazy to catch non-initialized access.
  static String? flutterRoot;

  /// Determine the absolute and normalized path for the root of the current
  /// Flutter checkout.
  ///
  /// This method has a series of fallbacks for determining the repo location. The
  /// first success will immediately return the root without further checks.
  ///
  /// The order of these tests is:
  ///   1. FLUTTER_ROOT environment variable contains the path.
  ///   2. Platform script is a data URI scheme, returning `../..` to support
  ///      tests run from `packages/flutter_tools`.
  ///   3. Platform script is package URI scheme, returning the grandgrandparent
  ///      directory of the package config file location from
  ///      `packages/flutter_tools/.dart_tool/package_config.json`.
  ///   4. Platform script file path is the snapshot path generated by `bin/flutter`,
  ///      returning the grandparent directory from `bin/cache`.
  ///   5. Platform script file name is the entrypoint in `packages/flutter_tools/bin/flutter_tools.dart`,
  ///      returning the 4th parent directory.
  ///   6. The current directory
  ///
  /// If an exception is thrown during any of these checks, an error message is
  /// printed and `.` is returned by default (6).
  static String defaultFlutterRoot({
    required Platform platform,
    required FileSystem fileSystem,
    required UserMessages userMessages,
  }) {
    String normalize(String path) {
      return fileSystem.path.normalize(fileSystem.path.absolute(path));
    }

    if (platform.environment.containsKey(kFlutterRootEnvironmentVariableName)) {
      return normalize(platform.environment[kFlutterRootEnvironmentVariableName]!);
    }
    try {
      if (platform.script.scheme == 'data') {
        return normalize('../..'); // The tool is running as a test.
      }
      final String Function(String) dirname = fileSystem.path.dirname;

      if (platform.script.scheme == 'package') {
        final String packageConfigPath = Uri.parse(
          platform.packageConfig!,
        ).toFilePath(windows: platform.isWindows);
        return normalize(dirname(dirname(dirname(dirname(packageConfigPath)))));
      }

      if (platform.script.scheme == 'file') {
        final String script = platform.script.toFilePath(windows: platform.isWindows);
        if (fileSystem.path.basename(script) == kSnapshotFileName) {
          return normalize(dirname(dirname(fileSystem.path.dirname(script))));
        }
        if (fileSystem.path.basename(script) == kFlutterToolsScriptFileName) {
          return normalize(dirname(dirname(dirname(dirname(script)))));
        }
      }
    } on Exception catch (error) {
      // There is currently no logger attached since this is computed at startup.
      // ignore: avoid_print
      print(userMessages.runnerNoRoot('$error'));
    }
    return normalize('.');
  }

  // Whether to cache artifacts for all platforms. Defaults to only caching
  // artifacts for the current platform.
  bool includeAllPlatforms = false;

  // Names of artifacts which should be cached even if they would normally
  // be filtered out for the current platform.
  Set<String>? platformOverrideArtifacts;

  // Whether to cache the unsigned mac binaries. Defaults to caching the signed binaries.
  bool useUnsignedMacBinaries = false;

  // Whether the warning printed when a custom artifact URL is used is fatal.
  bool fatalStorageWarning = true;

  static RandomAccessFile? _lock;
  static bool _lockEnabled = true;

  /// Turn off the [lock]/[releaseLock] mechanism.
  ///
  /// This is used by the tests since they run simultaneously and all in one
  /// process and so it would be a mess if they had to use the lock.
  @visibleForTesting
  static void disableLocking() {
    _lockEnabled = false;
  }

  /// Turn on the [lock]/[releaseLock] mechanism.
  ///
  /// This is used by the tests.
  @visibleForTesting
  static void enableLocking() {
    _lockEnabled = true;
  }

  /// Check if lock acquired, skipping FLUTTER_ALREADY_LOCKED reentrant checks.
  ///
  /// This is used by the tests.
  @visibleForTesting
  static bool isLocked() {
    return _lock != null;
  }

  /// Lock the cache directory.
  ///
  /// This happens while required artifacts are updated
  /// (see [FlutterCommandRunner.runCommand]).
  ///
  /// This uses normal POSIX flock semantics.
  Future<void> lock() async {
    if (!_lockEnabled) {
      return;
    }
    assert(_lock == null);
    final File lockFile = _fileSystem.file(
      _fileSystem.path.join(flutterRoot!, 'bin', 'cache', 'lockfile'),
    );
    try {
      _lock = lockFile.openSync(mode: FileMode.write);
    } on FileSystemException catch (e) {
      _logger.printError('Failed to open or create the artifact cache lockfile: "$e"');
      _logger.printError('Please ensure you have permissions to create or open ${lockFile.path}');
      throwToolExit('Failed to open or create the lockfile');
    }
    bool locked = false;
    bool printed = false;
    while (!locked) {
      try {
        _lock!.lockSync();
        locked = true;
      } on FileSystemException {
        if (!printed) {
          _logger.printTrace(
            'Waiting to be able to obtain lock of Flutter binary artifacts directory: ${_lock!.path}',
          );
          // This needs to go to stderr to avoid cluttering up stdout if a
          // parent process is collecting stdout (e.g. when calling "flutter
          // version --machine"). It's not really a "warning" though, so print it
          // in grey. Also, make sure that it isn't counted as a warning for
          // Logger.warningsAreFatal.
          _logger.printWarning(
            'Waiting for another flutter command to release the startup lock...',
            color: TerminalColor.grey,
            fatal: false,
          );
          printed = true;
        }
        await Future<void>.delayed(const Duration(milliseconds: 50));
      }
    }
  }

  /// Releases the lock.
  ///
  /// This happens automatically on startup (see [FlutterCommand.verifyThenRunCommand])
  /// after the command's required artifacts are updated.
  void releaseLock() {
    if (!_lockEnabled || _lock == null) {
      return;
    }
    _lock!.closeSync();
    _lock = null;
  }

  /// Checks if the current process owns the lock for the cache directory at
  /// this very moment; throws a [StateError] if it doesn't.
  void checkLockAcquired() {
    if (_lockEnabled &&
        _lock == null &&
        _platform.environment['FLUTTER_ALREADY_LOCKED'] != 'true') {
      throw StateError(
        'The current process does not own the lock for the cache directory. This is a bug in Flutter CLI tools.',
      );
    }
  }

  String get devToolsVersion {
    if (_devToolsVersion == null) {
      const String devToolsDirPath = 'dart-sdk/bin/resources/devtools';
      final Directory devToolsDir = getCacheDir(devToolsDirPath, shouldCreate: false);
      if (!devToolsDir.existsSync()) {
        throw Exception('Could not find directory at ${devToolsDir.path}');
      }
      final String versionFilePath = '${devToolsDir.path}/version.json';
      final File versionFile = _fileSystem.file(versionFilePath);
      if (!versionFile.existsSync()) {
        throw Exception('Could not find file at $versionFilePath');
      }
      final dynamic data = jsonDecode(versionFile.readAsStringSync());
      if (data is! Map<String, Object?>) {
        throw Exception(
          "Expected object of type 'Map<String, Object?>' but got one of type '${data.runtimeType}'",
        );
      }
      final Object? version = data['version'];
      if (version == null) {
        throw Exception('Could not parse DevTools version from $version');
      }
      if (version is! String) {
        throw Exception(
          "Could not parse DevTools version. Expected object of type 'String', but got one of type '${version.runtimeType}'",
        );
      }
      return _devToolsVersion = version;
    }
    return _devToolsVersion!;
  }

  String? _devToolsVersion;

  /// The current version of Dart used to build Flutter and run the tool.
  String get dartSdkVersion {
    if (_dartSdkVersion == null) {
      // Make the version string more customer-friendly.
      // Changes '2.1.0-dev.8.0.flutter-4312ae32' to '2.1.0 (build 2.1.0-dev.8.0 4312ae32)'
      final String justVersion = _platform.version.split(' ')[0];
      _dartSdkVersion = justVersion.replaceFirstMapped(RegExp(r'(\d+\.\d+\.\d+)(.+)'), (
        Match match,
      ) {
        final String noFlutter = match[2]!.replaceAll('.flutter-', ' ');
        return '${match[1]} (build ${match[1]}$noFlutter)';
      });
    }
    return _dartSdkVersion!;
  }

  String? _dartSdkVersion;

  /// The current version of Dart used to build Flutter and run the tool.
  String get dartSdkBuild {
    if (_dartSdkBuild == null) {
      // Make the version string more customer-friendly.
      // Changes '2.1.0-dev.8.0.flutter-4312ae32' to '2.1.0 (build 2.1.0-dev.8.0 4312ae32)'
      final String justVersion = _platform.version.split(' ')[0];
      _dartSdkBuild = justVersion.replaceFirstMapped(RegExp(r'(\d+\.\d+\.\d+)(.+)'), (Match match) {
        final String noFlutter = match[2]!.replaceAll('.flutter-', ' ');
        return '${match[1]}$noFlutter';
      });
    }
    return _dartSdkBuild!;
  }

  String? _dartSdkBuild;

  /// The current version of the Flutter engine the flutter tool will download.
  String get engineRevision {
    _engineRevision ??= getVersionFor('engine');
    if (_engineRevision == null) {
      throwToolExit('Could not determine engine revision.');
    }
    return _engineRevision!;
  }

  String? _engineRevision;

  /// The "realm" for the storage URL.
  ///
  /// For production artifacts from Engine post-submit and release builds,
  /// this string will be empty, and the `storageBaseUrl` will be unmodified.
  /// When non-empty, this string will be appended to the `storageBaseUrl` after
  /// a '/'. For artifacts generated by Engine presubmits, the realm should be
  /// "flutter_archives_v2".
  String get storageRealm {
    _storageRealm ??= getRealmFor('engine');
    if (_storageRealm == null) {
      throwToolExit('Could not determine engine realm.');
    }
    return _storageRealm!;
  }

  String? _storageRealm;

  /// The base for URLs that store Flutter engine artifacts that are fetched
  /// during the installation of the Flutter SDK.
  ///
  /// By default the base URL is https://storage.googleapis.com. However, if
  /// `FLUTTER_STORAGE_BASE_URL` environment variable ([kFlutterStorageBaseUrl])
  /// is provided, the environment variable value is returned instead.
  ///
  /// See also:
  ///
  ///  * [cipdBaseUrl], which determines how CIPD artifacts are fetched.
  ///  * [Cache] class-level dartdocs that explain how artifact mirrors work.
  String get storageBaseUrl {
    String? overrideUrl = _platform.environment[kFlutterStorageBaseUrl];
    if (overrideUrl == null) {
      return storageRealm.isEmpty
          ? 'https://storage.googleapis.com'
          : 'https://storage.googleapis.com/$storageRealm';
    }
    // verify that this is a valid URI.
    overrideUrl = storageRealm.isEmpty ? overrideUrl : '$overrideUrl/$storageRealm';
    try {
      Uri.parse(overrideUrl);
    } on FormatException catch (err) {
      throwToolExit('"$kFlutterStorageBaseUrl" contains an invalid URL:\n$err');
    }
    _maybeWarnAboutStorageOverride(overrideUrl);
    return overrideUrl;
  }

  String get realmlessStorageBaseUrl {
    return storageRealm.isEmpty ? storageBaseUrl : storageBaseUrl.replaceAll('/$storageRealm', '');
  }

  /// The base for URLs that store Flutter engine artifacts in CIPD.
  ///
  /// For some platforms, such as Web and Fuchsia, CIPD artifacts are fetched
  /// during the installation of the Flutter SDK, in addition to those fetched
  /// from [storageBaseUrl].
  ///
  /// By default the base URL is https://chrome-infra-packages.appspot.com/dl.
  /// However, if `FLUTTER_STORAGE_BASE_URL` environment variable is provided
  /// ([kFlutterStorageBaseUrl]), then the following value is used:
  ///
  ///     FLUTTER_STORAGE_BASE_URL/flutter_infra_release/cipd
  ///
  /// See also:
  ///
  ///  * [storageBaseUrl], which determines how engine artifacts stored in the
  ///    Google Cloud Storage buckets are fetched.
  ///  * https://chromium.googlesource.com/infra/luci/luci-go/+/refs/heads/main/cipd,
  ///    which contains information about CIPD.
  ///  * [Cache] class-level dartdocs that explain how artifact mirrors work.
  String get cipdBaseUrl {
    final String? overrideUrl = _platform.environment[kFlutterStorageBaseUrl];
    if (overrideUrl == null) {
      return 'https://chrome-infra-packages.appspot.com/dl';
    }

    final Uri original;
    try {
      original = Uri.parse(overrideUrl);
    } on FormatException catch (err) {
      throwToolExit('"$kFlutterStorageBaseUrl" contains an invalid URL:\n$err');
    }

    final String cipdOverride =
        original
            .replace(
              pathSegments: <String>[...original.pathSegments, 'flutter_infra_release', 'cipd'],
            )
            .toString();
    return cipdOverride;
  }

  bool _hasWarnedAboutStorageOverride = false;

  void _maybeWarnAboutStorageOverride(String overrideUrl) {
    if (_hasWarnedAboutStorageOverride) {
      return;
    }
    _logger.printWarning(
      'Flutter assets will be downloaded from $overrideUrl. Make sure you trust this source!',
      emphasis: true,
      fatal: false,
    );
    _hasWarnedAboutStorageOverride = true;
  }

  /// Return the top-level directory in the cache; this is `bin/cache`.
  Directory getRoot() {
    return _fileSystem.directory(
      _fileSystem.path.join(_rootOverride?.path ?? flutterRoot!, 'bin', 'cache'),
    );
  }

  String getHostPlatformArchName() {
    return _osUtils.hostPlatform.platformName;
  }

  /// Return a directory in the cache dir. For `pkg`, this will return `bin/cache/pkg`.
  ///
  /// When [shouldCreate] is true, the cache directory at [name] will be created
  /// if it does not already exist.
  Directory getCacheDir(String name, {bool shouldCreate = true}) {
    final Directory dir = _fileSystem.directory(_fileSystem.path.join(getRoot().path, name));
    if (!dir.existsSync() && shouldCreate) {
      dir.createSync(recursive: true);
      _osUtils.chmod(dir, '755');
    }
    return dir;
  }

  /// Return the top-level directory for artifact downloads.
  Directory getDownloadDir() => getCacheDir('downloads');

  /// Return the top-level mutable directory in the cache; this is `bin/cache/artifacts`.
  Directory getCacheArtifacts() => getCacheDir('artifacts');

  /// Location of LICENSE file.
  File getLicenseFile() => _fileSystem.file(_fileSystem.path.join(flutterRoot!, 'LICENSE'));

  /// Get a named directory from with the cache's artifact directory; for example,
  /// `material_fonts` would return `bin/cache/artifacts/material_fonts`.
  Directory getArtifactDirectory(String name) {
    return getCacheArtifacts().childDirectory(name);
  }

  MapEntry<String, String> get dyLdLibEntry {
    if (_dyLdLibEntry != null) {
      return _dyLdLibEntry!;
    }
    final List<String> paths = <String>[];
    for (final ArtifactSet artifact in _artifacts) {
      final Map<String, String> env = artifact.environment;
      if (!env.containsKey('DYLD_LIBRARY_PATH')) {
        continue;
      }
      final String path = env['DYLD_LIBRARY_PATH']!;
      if (path.isEmpty) {
        continue;
      }
      paths.add(path);
    }
    _dyLdLibEntry = MapEntry<String, String>('DYLD_LIBRARY_PATH', paths.join(':'));
    return _dyLdLibEntry!;
  }

  MapEntry<String, String>? _dyLdLibEntry;

  /// The web sdk has to be co-located with the dart-sdk so that they can share source
  /// code.
  Directory getWebSdkDirectory() {
    return getRoot().childDirectory('flutter_web_sdk');
  }

  String? getVersionFor(String artifactName) {
    final File versionFile = _fileSystem.file(
      _fileSystem.path.join(
        _rootOverride?.path ?? flutterRoot!,
        'bin',
        'internal',
        '$artifactName.version',
      ),
    );
    return versionFile.existsSync() ? versionFile.readAsStringSync().trim() : null;
  }

  String? getRealmFor(String artifactName) {
    final File realmFile = _fileSystem.file(
      _fileSystem.path.join(
        _rootOverride?.path ?? flutterRoot!,
        'bin',
        'internal',
        '$artifactName.realm',
      ),
    );
    return realmFile.existsSync() ? realmFile.readAsStringSync().trim() : '';
  }

  /// Delete all stamp files maintained by the cache.
  void clearStampFiles() {
    try {
      getStampFileFor('flutter_tools').deleteSync();
      for (final ArtifactSet artifact in _artifacts) {
        final File file = getStampFileFor(artifact.stampName);
        ErrorHandlingFileSystem.deleteIfExists(file);
      }
    } on FileSystemException catch (err) {
      _logger.printWarning('Failed to delete some stamp files: $err');
    }
  }

  /// Read the stamp for [artifactName].
  ///
  /// If the file is missing or cannot be parsed, returns `null`.
  String? getStampFor(String artifactName) {
    final File stampFile = getStampFileFor(artifactName);
    if (!stampFile.existsSync()) {
      return null;
    }
    try {
      return stampFile.readAsStringSync().trim();
    } on FileSystemException {
      return null;
    }
  }

  void setStampFor(String artifactName, String version) {
    getStampFileFor(artifactName).writeAsStringSync(version);
  }

  File getStampFileFor(String artifactName) {
    return _fileSystem.file(_fileSystem.path.join(getRoot().path, '$artifactName.stamp'));
  }

  /// Returns `true` if either [entity] is older than the tools stamp or if
  /// [entity] doesn't exist.
  bool isOlderThanToolsStamp(FileSystemEntity entity) {
    final File flutterToolsStamp = getStampFileFor('flutter_tools');
    return _fsUtils.isOlderThanReference(entity: entity, referenceFile: flutterToolsStamp);
  }

  Future<bool> isUpToDate() async {
    for (final ArtifactSet artifact in _artifacts) {
      if (!await artifact.isUpToDate(_fileSystem)) {
        return false;
      }
    }
    return true;
  }

  /// Update the cache to contain all `requiredArtifacts`.
  Future<void> updateAll(Set<DevelopmentArtifact> requiredArtifacts, {bool offline = false}) async {
    if (!_lockEnabled) {
      return;
    }
    for (final ArtifactSet artifact in _artifacts) {
      if (!requiredArtifacts.contains(artifact.developmentArtifact)) {
        _logger.printTrace('Artifact $artifact is not required, skipping update.');
        continue;
      }
      if (await artifact.isUpToDate(_fileSystem)) {
        continue;
      }
      try {
        await artifact.update(_artifactUpdater, _logger, _fileSystem, _osUtils, offline: offline);
      } on SocketException catch (e) {
        if (_hostsBlockedInChina.contains(e.address?.host)) {
          _logger.printError(
            'Failed to retrieve Flutter tool dependencies: ${e.message}.\n'
            "If you're in China, please see this page: "
            'https://flutter.dev/to/china-setup',
            emphasis: true,
          );
        }
        rethrow;
      }
    }
  }

  Future<bool> areRemoteArtifactsAvailable({
    String? engineVersion,
    bool includeAllPlatforms = true,
  }) async {
    final bool includeAllPlatformsState = this.includeAllPlatforms;
    bool allAvailable = true;
    this.includeAllPlatforms = includeAllPlatforms;
    for (final ArtifactSet cachedArtifact in _artifacts) {
      if (cachedArtifact is EngineCachedArtifact) {
        allAvailable &= await cachedArtifact.checkForArtifacts(engineVersion);
      }
    }
    this.includeAllPlatforms = includeAllPlatformsState;
    return allAvailable;
  }

  Future<bool> doesRemoteExist(String message, Uri url) async {
    final Status status = _logger.startProgress(message);
    bool exists;
    try {
      exists = await _net.doesRemoteFileExist(url);
    } finally {
      status.stop();
    }
    return exists;
  }
}

/// Representation of a set of artifacts used by the tool.
abstract class ArtifactSet {
  ArtifactSet(this.developmentArtifact);

  /// The development artifact.
  final DevelopmentArtifact developmentArtifact;

  /// [true] if the artifact is up to date.
  Future<bool> isUpToDate(FileSystem fileSystem);

  /// The environment variables (if any) required to consume the artifacts.
  Map<String, String> get environment {
    return const <String, String>{};
  }

  /// Updates the artifact.
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  });

  /// The canonical name of the artifact.
  String get name;

  // The name of the stamp file. Defaults to the same as the
  // artifact name.
  String get stampName => name;
}

/// An artifact set managed by the cache.
abstract class CachedArtifact extends ArtifactSet {
  CachedArtifact(this.name, this.cache, DevelopmentArtifact developmentArtifact)
    : super(developmentArtifact);

  final Cache cache;

  @override
  final String name;

  @override
  String get stampName => name;

  Directory get location => cache.getArtifactDirectory(name);

  String? get version => cache.getVersionFor(name);

  // Whether or not to bypass normal platform filtering for this artifact.
  bool get ignorePlatformFiltering {
    return cache.includeAllPlatforms ||
        (cache.platformOverrideArtifacts != null &&
            cache.platformOverrideArtifacts!.contains(developmentArtifact.name));
  }

  @override
  Future<bool> isUpToDate(FileSystem fileSystem) async {
    if (!location.existsSync()) {
      return false;
    }
    if (version != cache.getStampFor(stampName)) {
      return false;
    }
    return isUpToDateInner(fileSystem);
  }

  @override
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  }) async {
    if (!location.existsSync()) {
      try {
        location.createSync(recursive: true);
      } on FileSystemException catch (err) {
        logger.printError(err.toString());
        throwToolExit(
          'Failed to create directory for flutter cache at ${location.path}. '
          'Flutter may be missing permissions in its cache directory.',
        );
      }
    }
    await updateInner(artifactUpdater, fileSystem, operatingSystemUtils);
    try {
      if (version == null) {
        logger.printWarning(
          'No known version for the artifact name "$name". '
          'Flutter can continue, but the artifact may be re-downloaded on '
          'subsequent invocations until the problem is resolved.',
        );
      } else {
        cache.setStampFor(stampName, version!);
      }
    } on FileSystemException catch (err) {
      logger.printWarning(
        'The new artifact "$name" was downloaded, but Flutter failed to update '
        'its stamp file, receiving the error "$err". '
        'Flutter can continue, but the artifact may be re-downloaded on '
        'subsequent invocations until the problem is resolved.',
      );
    }
    artifactUpdater.removeDownloadedFiles();
  }

  /// Hook method for extra checks for being up-to-date.
  bool isUpToDateInner(FileSystem fileSystem) => true;

  Future<void> updateInner(
    ArtifactUpdater artifactUpdater,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils,
  );
}

abstract class EngineCachedArtifact extends CachedArtifact {
  EngineCachedArtifact(this.stampName, Cache cache, DevelopmentArtifact developmentArtifact)
    : super('engine', cache, developmentArtifact);

  @override
  final String stampName;

  /// Return a list of (directory path, download URL path) tuples.
  List<List<String>> getBinaryDirs();

  /// A list of cache directory paths to which the LICENSE file should be copied.
  List<String> getLicenseDirs();

  /// A list of the dart package directories to download.
  List<String> getPackageDirs();

  @override
  bool isUpToDateInner(FileSystem fileSystem) {
    final Directory pkgDir = cache.getCacheDir('pkg');
    for (final String pkgName in getPackageDirs()) {
      final String pkgPath = fileSystem.path.join(pkgDir.path, pkgName);
      if (!fileSystem.directory(pkgPath).existsSync()) {
        return false;
      }
    }

    for (final List<String> toolsDir in getBinaryDirs()) {
      final Directory dir = fileSystem.directory(fileSystem.path.join(location.path, toolsDir[0]));
      if (!dir.existsSync()) {
        return false;
      }
    }

    for (final String licenseDir in getLicenseDirs()) {
      final File file = fileSystem.file(fileSystem.path.join(location.path, licenseDir, 'LICENSE'));
      if (!file.existsSync()) {
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
    final String url = '${cache.storageBaseUrl}/flutter_infra_release/flutter/$version/';

    final Directory pkgDir = cache.getCacheDir('pkg');
    for (final String pkgName in getPackageDirs()) {
      await artifactUpdater.downloadZipArchive(
        'Downloading package $pkgName...',
        Uri.parse('$url$pkgName.zip'),
        pkgDir,
      );
    }

    for (final List<String> toolsDir in getBinaryDirs()) {
      final String cacheDir = toolsDir[0];
      final String urlPath = toolsDir[1];
      final Directory dir = fileSystem.directory(fileSystem.path.join(location.path, cacheDir));

      // Avoid printing things like 'Downloading linux-x64 tools...' multiple times.
      final String friendlyName = urlPath.replaceAll('/artifacts.zip', '').replaceAll('.zip', '');
      await artifactUpdater.downloadZipArchive(
        'Downloading $friendlyName tools...',
        Uri.parse(url + urlPath),
        dir,
      );

      _makeFilesExecutable(dir, operatingSystemUtils);
    }

    final File licenseSource = cache.getLicenseFile();
    for (final String licenseDir in getLicenseDirs()) {
      final String licenseDestinationPath = fileSystem.path.join(
        location.path,
        licenseDir,
        'LICENSE',
      );
      await licenseSource.copy(licenseDestinationPath);
    }
  }

  Future<bool> checkForArtifacts(String? engineVersion) async {
    engineVersion ??= version;
    final String url = '${cache.storageBaseUrl}/flutter_infra_release/flutter/$engineVersion/';

    bool exists = false;
    for (final String pkgName in getPackageDirs()) {
      exists = await cache.doesRemoteExist(
        'Checking package $pkgName is available...',
        Uri.parse('$url$pkgName.zip'),
      );
      if (!exists) {
        return false;
      }
    }

    for (final List<String> toolsDir in getBinaryDirs()) {
      final String cacheDir = toolsDir[0];
      final String urlPath = toolsDir[1];
      exists = await cache.doesRemoteExist(
        'Checking $cacheDir tools are available...',
        Uri.parse(url + urlPath),
      );
      if (!exists) {
        return false;
      }
    }
    return true;
  }

  void _makeFilesExecutable(Directory dir, OperatingSystemUtils operatingSystemUtils) {
    operatingSystemUtils.chmod(dir, 'a+r,a+x');
    for (final File file in dir.listSync(recursive: true).whereType<File>()) {
      final FileStat stat = file.statSync();
      final bool isUserExecutable = ((stat.mode >> 6) & 0x1) == 1;
      if (file.basename == 'flutter_tester' || isUserExecutable) {
        // Make the file readable and executable by all users.
        operatingSystemUtils.chmod(file, 'a+r,a+x');
      }
    }
  }
}

/// An API for downloading and un-archiving artifacts, such as engine binaries or
/// additional source code.
class ArtifactUpdater {
  ArtifactUpdater({
    required OperatingSystemUtils operatingSystemUtils,
    required Logger logger,
    required FileSystem fileSystem,
    required Directory tempStorage,
    required HttpClient httpClient,
    required Platform platform,
    required List<String> allowedBaseUrls,
  }) : _operatingSystemUtils = operatingSystemUtils,
       _httpClient = httpClient,
       _logger = logger,
       _fileSystem = fileSystem,
       _tempStorage = tempStorage,
       _platform = platform,
       _allowedBaseUrls = allowedBaseUrls;

  /// The number of times the artifact updater will repeat the artifact download loop.
  static const int _kRetryCount = 2;

  final Logger _logger;
  final OperatingSystemUtils _operatingSystemUtils;
  final FileSystem _fileSystem;
  final Directory _tempStorage;
  final HttpClient _httpClient;
  final Platform _platform;

  /// Artifacts should only be downloaded from URLs that use one of these
  /// prefixes.
  ///
  /// [ArtifactUpdater] will issue a warning if an attempt to download from a
  /// non-compliant URL is made.
  final List<String> _allowedBaseUrls;

  /// Keep track of the files we've downloaded for this execution so we
  /// can delete them after completion. We don't delete them right after
  /// extraction in case [update] is interrupted, so we can restart without
  /// starting from scratch.
  @visibleForTesting
  final List<File> downloadedFiles = <File>[];

  /// These filenames, should they exist after extracting an archive, should be deleted.
  static const Set<String> _denylistedBasenames = <String>{
    'entitlements.txt',
    'without_entitlements.txt',
    'unsigned_binaries.txt',
  };
  void _removeDenylistedFiles(Directory directory) {
    for (final FileSystemEntity entity in directory.listSync(recursive: true)) {
      if (entity is! File) {
        continue;
      }
      if (_denylistedBasenames.contains(entity.basename)) {
        entity.deleteSync();
      }
    }
  }

  /// Download a zip archive from the given [url] and unzip it to [location].
  Future<void> downloadZipArchive(String message, Uri url, Directory location) {
    return _downloadArchive(message, url, location, _operatingSystemUtils.unzip);
  }

  /// Download a gzipped tarball from the given [url] and unpack it to [location].
  Future<void> downloadZippedTarball(String message, Uri url, Directory location) {
    return _downloadArchive(message, url, location, _operatingSystemUtils.unpack);
  }

  /// Download an archive from the given [url] and unzip it to [location].
  Future<void> _downloadArchive(
    String message,
    Uri url,
    Directory location,
    void Function(File, Directory) extractor,
  ) async {
    final String downloadPath = flattenNameSubdirs(url, _fileSystem);
    final File tempFile = _createDownloadFile(downloadPath);
    Status status;
    int retries = _kRetryCount;

    while (retries > 0) {
      status = _logger.startProgress(message);
      try {
        _ensureExists(tempFile.parent);
        if (tempFile.existsSync()) {
          tempFile.deleteSync();
        }
        await _download(url, tempFile, status);

        if (!tempFile.existsSync()) {
          throw Exception('Did not find downloaded file ${tempFile.path}');
        }
      } on Exception catch (err) {
        _logger.printTrace(err.toString());
        retries -= 1;
        if (retries == 0) {
          throwToolExit(
            'Failed to download $url. Ensure you have network connectivity and then try again.\n$err',
          );
        }
        continue;
      } on ArgumentError catch (error) {
        final String? overrideUrl = _platform.environment[kFlutterStorageBaseUrl];
        if (overrideUrl != null && url.toString().contains(overrideUrl)) {
          _logger.printError(error.toString());
          throwToolExit(
            'The value of $kFlutterStorageBaseUrl ($overrideUrl) could not be '
            'parsed as a valid url. Please see https://flutter.dev/to/use-mirror-site '
            'for an example of how to use it.\n'
            'Full URL: $url',
            exitCode: kNetworkProblemExitCode,
          );
        }
        // This error should not be hit if there was not a storage URL override, allow the
        // tool to crash.
        rethrow;
      } finally {
        status.stop();
      }

      /// Unzipping multiple file into a directory will not remove old files
      /// from previous versions that are not present in the new bundle.
      final Directory destination = location.childDirectory(
        tempFile.fileSystem.path.basenameWithoutExtension(tempFile.path),
      );
      try {
        ErrorHandlingFileSystem.deleteIfExists(destination, recursive: true);
      } on FileSystemException catch (error) {
        // Error that indicates another program has this file open and that it
        // cannot be deleted. For the cache, this is either the analyzer reading
        // the sky_engine package or a running flutter_tester device.
        const int kSharingViolation = 32;
        if (_platform.isWindows && error.osError?.errorCode == kSharingViolation) {
          throwToolExit(
            'Failed to delete ${destination.path} because the local file/directory is in use '
            'by another process. Try closing any running IDEs or editors and trying '
            'again',
          );
        }
      }
      _ensureExists(location);

      try {
        extractor(tempFile, location);
      } on Exception catch (err) {
        retries -= 1;
        if (retries == 0) {
          throwToolExit(
            'Flutter could not download and/or extract $url. Ensure you have '
            'network connectivity and all of the required dependencies listed at '
            'https://flutter.dev/setup.\nThe original exception was: $err.',
          );
        }
        _deleteIgnoringErrors(tempFile);
        continue;
      }
      _removeDenylistedFiles(location);
      return;
    }
  }

  /// Download bytes from [url], throwing non-200 responses as an exception.
  ///
  /// Validates that the md5 of the content bytes matches the provided
  /// `x-goog-hash` header, if present. This header should contain an md5 hash
  /// if the download source is Google cloud storage.
  ///
  /// See also:
  ///   * https://cloud.google.com/storage/docs/xml-api/reference-headers#xgooghash
  Future<void> _download(Uri url, File file, Status status) async {
    final bool isAllowedUrl = _allowedBaseUrls.any(
      (String baseUrl) => url.toString().startsWith(baseUrl),
    );

    // In tests make this a hard failure.
    assert(
      isAllowedUrl,
      'URL not allowed: $url\n'
      'Allowed URLs must be based on one of: ${_allowedBaseUrls.join(', ')}',
    );

    // In production, issue a warning but allow the download to proceed.
    if (!isAllowedUrl) {
      status.pause();
      _logger.printWarning(
        'Downloading an artifact that may not be reachable in some environments (e.g. firewalled environments): $url\n'
        'This should not have happened. This is likely a Flutter SDK bug. Please file an issue at https://github.com/flutter/flutter/issues/new?template=1_activation.yml',
      );
      status.resume();
    }

    final HttpClientRequest request = await _httpClient.getUrl(url);
    final HttpClientResponse response = await request.close();
    if (response.statusCode != HttpStatus.ok) {
      throw Exception(response.statusCode);
    }

    final String? md5Hash = _expectedMd5(response.headers);
    ByteConversionSink? inputSink;
    late StreamController<Digest> digests;
    if (md5Hash != null) {
      _logger.printTrace('Content $url md5 hash: $md5Hash');
      digests = StreamController<Digest>();
      inputSink = md5.startChunkedConversion(digests);
    }
    final RandomAccessFile randomAccessFile = file.openSync(mode: FileMode.writeOnly);
    await response.forEach((List<int> chunk) {
      inputSink?.add(chunk);
      randomAccessFile.writeFromSync(chunk);
    });
    randomAccessFile.closeSync();
    if (inputSink != null) {
      inputSink.close();
      final Digest digest = await digests.stream.last;
      final String rawDigest = base64.encode(digest.bytes);
      if (rawDigest != md5Hash) {
        throw Exception(
          'Expected $url to have md5 checksum $md5Hash, but was $rawDigest. This '
          'may indicate a problem with your connection to the Flutter backend servers. '
          'Please re-try the download after confirming that your network connection is '
          'stable.',
        );
      }
    }
  }

  String? _expectedMd5(HttpHeaders httpHeaders) {
    final List<String>? values = httpHeaders['x-goog-hash'];
    if (values == null) {
      return null;
    }
    String? rawMd5Hash;
    for (final String value in values) {
      if (value.startsWith('md5=')) {
        rawMd5Hash = value;
        break;
      }
    }
    if (rawMd5Hash == null) {
      return null;
    }
    final List<String> segments = rawMd5Hash.split('md5=');
    if (segments.length < 2) {
      return null;
    }
    final String md5Hash = segments[1];
    if (md5Hash.isEmpty) {
      return null;
    }
    return md5Hash;
  }

  /// Create a temporary file and invoke [onTemporaryFile] with the file as
  /// argument, then add the temporary file to the [downloadedFiles].
  File _createDownloadFile(String name) {
    final File tempFile = _fileSystem.file(_fileSystem.path.join(_tempStorage.path, name));
    downloadedFiles.add(tempFile);
    return tempFile;
  }

  /// Create the given [directory] and parents, as necessary.
  void _ensureExists(Directory directory) {
    if (!directory.existsSync()) {
      directory.createSync(recursive: true);
    }
  }

  /// Clear any zip/gzip files downloaded.
  void removeDownloadedFiles() {
    for (final File file in downloadedFiles) {
      if (!file.existsSync()) {
        continue;
      }
      try {
        file.deleteSync();
      } on FileSystemException catch (e) {
        _logger.printWarning('Failed to delete "${file.path}". Please delete manually. $e');
        continue;
      }
      for (
        Directory directory = file.parent;
        directory.absolute.path != _tempStorage.absolute.path;
        directory = directory.parent
      ) {
        // Handle race condition when the directory is deleted before this step
        if (!directory.existsSync()) {
          break;
        }
        if (directory.listSync().isNotEmpty) {
          break;
        }
        _deleteIgnoringErrors(directory);
      }
    }
  }

  static void _deleteIgnoringErrors(FileSystemEntity entity) {
    if (!entity.existsSync()) {
      return;
    }
    try {
      entity.deleteSync();
    } on FileSystemException {
      // Ignore errors.
    }
  }
}

@visibleForTesting
String flattenNameSubdirs(Uri url, FileSystem fileSystem) {
  final List<String> pieces = <String>[url.host, ...url.pathSegments];
  final Iterable<String> convertedPieces = pieces.map<String>(_flattenNameNoSubdirs);
  return fileSystem.path.joinAll(convertedPieces);
}

/// Given a name containing slashes, colons, and backslashes, expand it into
/// something that doesn't.
String _flattenNameNoSubdirs(String fileName) {
  final List<int> replacedCodeUnits = <int>[
    for (final int codeUnit in fileName.codeUnits)
      ..._flattenNameSubstitutions[codeUnit] ?? <int>[codeUnit],
  ];
  return String.fromCharCodes(replacedCodeUnits);
}

// Many characters are problematic in filenames, especially on Windows.
final Map<int, List<int>> _flattenNameSubstitutions = <int, List<int>>{
  r'@'.codeUnitAt(0): '@@'.codeUnits,
  r'/'.codeUnitAt(0): '@s@'.codeUnits,
  r'\'.codeUnitAt(0): '@bs@'.codeUnits,
  r':'.codeUnitAt(0): '@c@'.codeUnits,
  r'%'.codeUnitAt(0): '@per@'.codeUnits,
  r'*'.codeUnitAt(0): '@ast@'.codeUnits,
  r'<'.codeUnitAt(0): '@lt@'.codeUnits,
  r'>'.codeUnitAt(0): '@gt@'.codeUnits,
  r'"'.codeUnitAt(0): '@q@'.codeUnits,
  r'|'.codeUnitAt(0): '@pip@'.codeUnits,
  r'?'.codeUnitAt(0): '@ques@'.codeUnits,
};
