// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'base/common.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'base/process.dart';
import 'base/time.dart';
import 'cache.dart';
import 'convert.dart';
import 'globals.dart' as globals;

/// The default version when a version could not be determined.
const String kUnknownFrameworkVersion = '0.0.0-unknown';

/// A git shortcut for the branch that is being tracked by the current one.
///
/// See `man gitrevisions` for more information.
const String kGitTrackingUpstream = '@{upstream}';

/// Replacement name when the branch is user-specific.
const String kUserBranch = '[user-branch]';

/// This maps old branch names to the names of branches that replaced them.
///
/// For example, in 2021 we deprecated the "dev" channel and transitioned "dev"
/// users to the "beta" channel.
const Map<String, String> kObsoleteBranches = <String, String>{'dev': 'beta'};

/// The names of each channel/branch in order of increasing stability.
enum Channel { master, main, beta, stable }

// Beware: Keep order in accordance with stability
const Set<String> kOfficialChannels = <String>{'master', 'main', 'beta', 'stable'};

const Map<String, String> kChannelDescriptions = <String, String>{
  'master': 'latest development branch, for contributors',
  'main': 'latest development branch, follows master channel',
  'beta': 'updated monthly, recommended for experienced users',
  'stable': 'updated quarterly, for new users and for production app releases',
};

const Set<String> kDevelopmentChannels = <String>{'master', 'main'};

/// Retrieve a human-readable name for a given [channel].
///
/// Requires [kOfficialChannels] to be correctly ordered.
String getNameForChannel(Channel channel) {
  return kOfficialChannels.elementAt(channel.index);
}

/// Retrieve the [Channel] representation for a string [name].
///
/// Returns `null` if [name] is not in the list of official channels, according
/// to [kOfficialChannels].
Channel? getChannelForName(String name) {
  if (kOfficialChannels.contains(name)) {
    return Channel.values[kOfficialChannels.toList().indexOf(name)];
  }
  return null;
}

abstract class FlutterVersion {
  /// Parses the Flutter version from currently available tags in the local
  /// repo.
  factory FlutterVersion({
    SystemClock clock = const SystemClock(),
    required FileSystem fs,
    required String flutterRoot,
    @protected bool fetchTags = false,
  }) {
    final File versionFile = getVersionFile(fs, flutterRoot);

    if (!fetchTags && versionFile.existsSync()) {
      final _FlutterVersionFromFile? version = _FlutterVersionFromFile.tryParseFromFile(
        versionFile,
        flutterRoot: flutterRoot,
      );
      if (version != null) {
        return version;
      }
    }

    // if we are fetching tags, ignore cached versionFile
    if (fetchTags && versionFile.existsSync()) {
      versionFile.deleteSync();
      final File legacyVersionFile = fs.file(fs.path.join(flutterRoot, 'version'));
      if (legacyVersionFile.existsSync()) {
        legacyVersionFile.deleteSync();
      }
    }

    final String frameworkRevision = _runGit(
      gitLog(<String>['-n', '1', '--pretty=format:%H']).join(' '),
      globals.processUtils,
      flutterRoot,
    );

    return FlutterVersion.fromRevision(
      clock: clock,
      frameworkRevision: frameworkRevision,
      fs: fs,
      flutterRoot: flutterRoot,
      fetchTags: fetchTags,
    );
  }

  FlutterVersion._({required SystemClock clock, required this.flutterRoot, required this.fs})
    : _clock = clock;

  factory FlutterVersion.fromRevision({
    SystemClock clock = const SystemClock(),
    required String flutterRoot,
    required String frameworkRevision,
    required FileSystem fs,
    bool fetchTags = false,
  }) {
    final GitTagVersion gitTagVersion = GitTagVersion.determine(
      globals.processUtils,
      globals.platform,
      gitRef: frameworkRevision,
      workingDirectory: flutterRoot,
      fetchTags: fetchTags,
    );
    final String frameworkVersion = gitTagVersion.frameworkVersionFor(frameworkRevision);
    return _FlutterVersionGit._(
      clock: clock,
      flutterRoot: flutterRoot,
      frameworkRevision: frameworkRevision,
      frameworkVersion: frameworkVersion,
      gitTagVersion: gitTagVersion,
      fs: fs,
    );
  }

  /// Ensure the latest git tags are fetched and recalculate [FlutterVersion].
  ///
  /// This is only required when not on beta or stable and we need to calculate
  /// the current version relative to upstream tags.
  ///
  /// This is a method and not a factory constructor so that test classes can
  /// override it.
  FlutterVersion fetchTagsAndGetVersion({SystemClock clock = const SystemClock()}) {
    // We don't need to fetch tags on beta and stable to calculate the version,
    // we should already exactly be on a tag that was pushed when this release
    // was published.
    if (channel != 'master' && channel != 'main') {
      return this;
    }
    return FlutterVersion(clock: clock, flutterRoot: flutterRoot, fs: fs, fetchTags: true);
  }

  final FileSystem fs;

  final SystemClock _clock;

  String? get repositoryUrl;

  GitTagVersion get gitTagVersion;

  /// The channel is the upstream branch.
  ///
  /// `master`, `dev`, `beta`, `stable`; or old ones, like `alpha`, `hackathon`, ...
  String get channel;

  /// The SHA describing the commit being used for the SDK and tools provide in `flutter/flutter`.
  ///
  /// The _exception_ is the _engine artifacts_, which are downloaded separately as [engineRevision].
  String get frameworkRevision;

  /// The shorter Git commit SHA of [frameworkRevion].
  String get frameworkRevisionShort => _shortGitRevision(frameworkRevision);
  String get frameworkVersion;

  String get devToolsVersion;
  String get dartSdkVersion;

  /// The SHA describing the commit being used for the engine artifacts, which are compiled from the `engine/` sub-directory.
  ///
  /// When using a standard release build, or master channel, [engineRevision] will be identical to [frameworkRevision] since
  /// the monorepository merge (as of 2025); however when modifying the framework (or engine) locally, or using a flag such
  /// as `FLUTTER_PREBUILT_ENGINE_VERSION=...`, the engine SHA will be _different_ than the [frameworkRevision].
  String get engineRevision;

  /// The shorter Git commit SHA of [engineRevision].
  String get engineRevisionShort => _shortGitRevision(engineRevision);

  // This is static as it is called from a constructor.
  static File getVersionFile(FileSystem fs, String flutterRoot) {
    return fs.file(fs.path.join(flutterRoot, 'bin', 'cache', 'flutter.version.json'));
  }

  final String flutterRoot;

  String _getTimeSinceCommit({String? revision}) {
    return _runGit(
      FlutterVersion.gitLog(<String>[
        '-n',
        '1',
        '--pretty=format:%ar',
        if (revision != null) revision,
      ]).join(' '),
      globals.processUtils,
      flutterRoot,
    );
  }

  // TODO(fujino): calculate this relative to frameworkCommitDate for
  // _FlutterVersionFromFile so we don't need a git call.
  late final String frameworkAge = _getTimeSinceCommit();
  late final String engineAge = _getTimeSinceCommit(revision: engineRevision);

  void ensureVersionFile();

  @override
  String toString() {
    final String versionText =
        frameworkVersion == kUnknownFrameworkVersion ? '' : ' $frameworkVersion';
    final String flutterText =
        'Flutter$versionText • channel $channel • ${repositoryUrl ?? 'unknown source'}';
    final String frameworkText =
        'Framework • revision $frameworkRevisionShort ($frameworkAge) • $frameworkCommitDate';
    String engineText = 'Engine • revision $engineRevisionShort ($engineAge)';
    if (engineCommitDate != null) {
      engineText = '$engineText • $engineCommitDate';
    }

    final String toolsText = 'Tools • Dart $dartSdkVersion • DevTools $devToolsVersion';

    // Flutter 1.10.2-pre.69 • channel master • https://github.com/flutter/flutter.git
    // Framework • revision 340c158f32 (85 minutes ago) • 2018-10-26 11:27:22 -0400
    // Engine • revision 9c46333e14 (96 minutes ago) • 2018-10-26 11:16:22 -0400
    // Tools • Dart 2.1.0 (build 2.1.0-dev.8.0 bf26f760b1)

    return '$flutterText\n$frameworkText\n$engineText\n$toolsText';
  }

  Map<String, Object> toJson() => <String, Object>{
    'frameworkVersion': frameworkVersion,
    'channel': channel,
    'repositoryUrl': repositoryUrl ?? 'unknown source',
    'frameworkRevision': frameworkRevision,
    'frameworkCommitDate': frameworkCommitDate,
    'engineRevision': engineRevision,
    if (engineCommitDate != null) 'engineCommitDate': engineCommitDate!,
    'dartSdkVersion': dartSdkVersion,
    'devToolsVersion': devToolsVersion,
    'flutterVersion': frameworkVersion,
  };

  /// A date String describing the [frameworkRevision] commit.
  ///
  /// If a git command fails, this will return a placeholder date.
  String get frameworkCommitDate;

  /// A date String describing the [engineRevision] commit.
  ///
  /// If a git command fails, this will return a placeholder date.
  ///
  /// If no date was recorded ([engineCommitDate] is a newly stored field),
  /// the date is omitted, and left `null`.
  String? get engineCommitDate;

  /// Checks if the currently installed version of Flutter is up-to-date, and
  /// warns the user if it isn't.
  ///
  /// This function must run while [Cache.lock] is acquired because it reads and
  /// writes shared cache files.
  Future<void> checkFlutterVersionFreshness() async {
    // Don't perform update checks if we're not on an official channel.
    if (!kOfficialChannels.contains(channel)) {
      return;
    }
    // Don't perform the update check if the tracking remote is not standard.
    if (VersionUpstreamValidator(version: this, platform: globals.platform).run() != null) {
      return;
    }
    DateTime localFrameworkCommitDate;
    try {
      // Don't perform the update check if fetching the latest local commit failed.
      localFrameworkCommitDate = DateTime.parse(_gitCommitDate(workingDirectory: flutterRoot));
    } on VersionCheckError {
      return;
    } on FormatException {
      return;
    }
    final DateTime? latestFlutterCommitDate = await _getLatestAvailableFlutterDate();

    return VersionFreshnessValidator(
      version: this,
      clock: _clock,
      localFrameworkCommitDate: localFrameworkCommitDate,
      latestFlutterCommitDate: latestFlutterCommitDate,
      logger: globals.logger,
      cache: globals.cache,
      pauseTime: VersionFreshnessValidator.timeToPauseToLetUserReadTheMessage,
    ).run();
  }

  /// Gets the release date of the latest available Flutter version.
  ///
  /// This method sends a server request if it's been more than
  /// [checkAgeConsideredUpToDate] since the last version check.
  ///
  /// Returns null if the cached version is out-of-date or missing, and we are
  /// unable to reach the server to get the latest version.
  Future<DateTime?> _getLatestAvailableFlutterDate() async {
    globals.cache.checkLockAcquired();
    final VersionCheckStamp versionCheckStamp = await VersionCheckStamp.load(
      globals.cache,
      globals.logger,
    );

    final DateTime now = _clock.now();
    if (versionCheckStamp.lastTimeVersionWasChecked != null) {
      final Duration timeSinceLastCheck = now.difference(
        versionCheckStamp.lastTimeVersionWasChecked!,
      );

      // Don't ping the server too often. Return cached value if it's fresh.
      if (timeSinceLastCheck < VersionFreshnessValidator.checkAgeConsideredUpToDate) {
        return versionCheckStamp.lastKnownRemoteVersion;
      }
    }

    // Cache is empty or it's been a while since the last server ping. Ping the server.
    try {
      final DateTime remoteFrameworkCommitDate = DateTime.parse(
        await fetchRemoteFrameworkCommitDate(),
      );
      await versionCheckStamp.store(
        newTimeVersionWasChecked: now,
        newKnownRemoteVersion: remoteFrameworkCommitDate,
      );
      return remoteFrameworkCommitDate;
    } on VersionCheckError catch (error) {
      // This happens when any of the git commands fails, which can happen when
      // there's no Internet connectivity. Remote version check is best effort
      // only. We do not prevent the command from running when it fails.
      globals.printTrace('Failed to check Flutter version in the remote repository: $error');
      // Still update the timestamp to avoid us hitting the server on every single
      // command if for some reason we cannot connect (eg. we may be offline).
      await versionCheckStamp.store(newTimeVersionWasChecked: now);
      return null;
    }
  }

  /// The date of the latest framework commit in the remote repository.
  ///
  /// Throws [VersionCheckError] if a git command fails, for example, when the
  /// remote git repository is not reachable due to a network issue.
  static Future<String> fetchRemoteFrameworkCommitDate() async {
    try {
      // Fetch upstream branch's commit and tags
      await _run(<String>['git', 'fetch', '--tags']);
      return _gitCommitDate(gitRef: kGitTrackingUpstream, workingDirectory: Cache.flutterRoot);
    } on VersionCheckError catch (error) {
      globals.printError(error.message);
      rethrow;
    }
  }

  /// Return a short string for the version (e.g. `master/0.0.59-pre.92`, `scroll_refactor/a76bc8e22b`).
  String getVersionString({bool redactUnknownBranches = false}) {
    if (frameworkVersion != kUnknownFrameworkVersion) {
      return '${getBranchName(redactUnknownBranches: redactUnknownBranches)}/$frameworkVersion';
    }
    return '${getBranchName(redactUnknownBranches: redactUnknownBranches)}/$frameworkRevisionShort';
  }

  /// The name of the local branch.
  ///
  /// Use getBranchName() to read this.
  String? _branch;

  /// Return the branch name.
  ///
  /// If [redactUnknownBranches] is true and the branch is unknown,
  /// the branch name will be returned as `'[user-branch]'` ([kUserBranch]).
  String getBranchName({bool redactUnknownBranches = false}) {
    _branch ??= () {
      final String branch = _runGit(
        'git symbolic-ref --short HEAD',
        globals.processUtils,
        flutterRoot,
      );
      return branch == 'HEAD' ? '' : branch;
    }();
    if (redactUnknownBranches || _branch!.isEmpty) {
      // Only return the branch names we know about; arbitrary branch names might contain PII.
      if (!kOfficialChannels.contains(_branch) && !kObsoleteBranches.containsKey(_branch)) {
        return kUserBranch;
      }
    }
    return _branch!;
  }

  /// Reset the version freshness information by removing the stamp file.
  ///
  /// New version freshness information will be regenerated when
  /// [checkFlutterVersionFreshness] is called after this. This is typically
  /// used when switching channels so that stale information from another
  /// channel doesn't linger.
  static Future<void> resetFlutterVersionFreshnessCheck() async {
    try {
      await globals.cache.getStampFileFor(VersionCheckStamp.flutterVersionCheckStampFile).delete();
    } on FileSystemException {
      // Ignore, since we don't mind if the file didn't exist in the first place.
    }
  }

  /// log.showSignature=false is a user setting and it will break things,
  /// so we want to disable it for every git log call. This is a convenience
  /// wrapper that does that.
  @visibleForTesting
  static List<String> gitLog(List<String> args) {
    return <String>['git', '-c', 'log.showSignature=false', 'log'] + args;
  }
}

// The date of the given commit hash as [gitRef]. If no hash is specified,
// then it is the HEAD of the current local branch.
//
// If lenient is true, and the git command fails, a placeholder date is
// returned. Otherwise, the VersionCheckError exception is propagated.
String _gitCommitDate({
  String gitRef = 'HEAD',
  bool lenient = false,
  required String? workingDirectory,
}) {
  final List<String> args = FlutterVersion.gitLog(<String>[
    gitRef,
    '-n',
    '1',
    '--pretty=format:%ad',
    '--date=iso',
  ]);
  try {
    // Don't plumb 'lenient' through directly so that we can print an error
    // if something goes wrong.
    return _runSync(args, lenient: false, workingDirectory: workingDirectory);
  } on VersionCheckError catch (e) {
    if (lenient) {
      final DateTime dummyDate = DateTime.fromMillisecondsSinceEpoch(0);
      globals.printError(
        'Failed to find the latest git commit date: $e\n'
        'Returning $dummyDate instead.',
      );
      // Return something that DateTime.parse() can parse.
      return dummyDate.toString();
    } else {
      rethrow;
    }
  }
}

class _FlutterVersionFromFile extends FlutterVersion {
  _FlutterVersionFromFile._({
    required super.clock,
    required this.frameworkVersion,
    required this.channel,
    required this.repositoryUrl,
    required this.frameworkRevision,
    required this.frameworkCommitDate,
    required this.engineRevision,
    required this.engineCommitDate,
    required this.dartSdkVersion,
    required this.devToolsVersion,
    required this.gitTagVersion,
    required super.flutterRoot,
    required super.fs,
  }) : super._();

  static _FlutterVersionFromFile? tryParseFromFile(
    File jsonFile, {
    required String flutterRoot,
    SystemClock clock = const SystemClock(),
  }) {
    try {
      final String jsonContents = jsonFile.readAsStringSync();
      final Map<String, Object?> manifest = jsonDecode(jsonContents) as Map<String, Object?>;

      return _FlutterVersionFromFile._(
        clock: clock,
        frameworkVersion: manifest['frameworkVersion']! as String,
        channel: manifest['channel']! as String,
        repositoryUrl: manifest['repositoryUrl']! as String,
        frameworkRevision: manifest['frameworkRevision']! as String,
        frameworkCommitDate: manifest['frameworkCommitDate']! as String,
        engineRevision: manifest['engineRevision']! as String,
        engineCommitDate: manifest['engineCommitDate'] as String?,
        dartSdkVersion: manifest['dartSdkVersion']! as String,
        devToolsVersion: manifest['devToolsVersion']! as String,
        gitTagVersion: GitTagVersion.parse(manifest['flutterVersion']! as String),
        flutterRoot: flutterRoot,
        fs: jsonFile.fileSystem,
      );
      // ignore: avoid_catches_without_on_clauses
    } catch (err) {
      globals.printTrace('Failed to parse ${jsonFile.path} with $err');
      try {
        jsonFile.deleteSync();
      } on FileSystemException {
        globals.printTrace('Failed to delete ${jsonFile.path}');
      }
      // Returning null means fallback to git implementation.
      return null;
    }
  }

  @override
  final GitTagVersion gitTagVersion;

  @override
  final String frameworkVersion;

  @override
  final String channel;

  @override
  String getBranchName({bool redactUnknownBranches = false}) => channel;

  @override
  final String repositoryUrl;

  @override
  final String frameworkRevision;

  @override
  final String frameworkCommitDate;

  @override
  final String? engineCommitDate;

  @override
  final String engineRevision;

  @override
  final String dartSdkVersion;

  @override
  final String devToolsVersion;

  @override
  void ensureVersionFile() {
    _ensureLegacyVersionFile(fs: fs, flutterRoot: flutterRoot, frameworkVersion: frameworkVersion);
  }
}

class _FlutterVersionGit extends FlutterVersion {
  _FlutterVersionGit._({
    required super.clock,
    required super.flutterRoot,
    required this.frameworkRevision,
    required this.frameworkVersion,
    required this.gitTagVersion,
    required super.fs,
  }) : super._();

  @override
  final GitTagVersion gitTagVersion;

  @override
  final String frameworkRevision;

  @override
  String get frameworkCommitDate => _gitCommitDate(lenient: true, workingDirectory: flutterRoot);

  // This uses 'late final' instead of 'String get' because unlike frameworkCommitDate, it is
  // operating based on a 'gitRef: ...', which we can assume to be immutable in the context of
  // this invocation (possibly HEAD could change, but gitRef should not).
  @override
  late final String engineCommitDate = _gitCommitDate(
    gitRef: engineRevision,
    lenient: true,
    workingDirectory: flutterRoot,
  );

  String? _repositoryUrl;
  @override
  String? get repositoryUrl {
    if (_repositoryUrl == null) {
      final String gitChannel = _runGit(
        'git rev-parse --abbrev-ref --symbolic $kGitTrackingUpstream',
        globals.processUtils,
        flutterRoot,
      );
      final int slash = gitChannel.indexOf('/');
      if (slash != -1) {
        final String remote = gitChannel.substring(0, slash);
        _repositoryUrl = _runGit(
          'git ls-remote --get-url $remote',
          globals.processUtils,
          flutterRoot,
        );
      }
    }
    return _repositoryUrl;
  }

  @override
  String get devToolsVersion => globals.cache.devToolsVersion;

  @override
  String get dartSdkVersion => globals.cache.dartSdkVersion;

  @override
  String get engineRevision => globals.cache.engineRevision;

  @override
  final String frameworkVersion;

  /// The channel is the current branch if we recognize it, or "[user-branch]" (kUserBranch).
  /// `master`, `beta`, `stable`; or old ones, like `alpha`, `hackathon`, `dev`, ...
  @override
  String get channel {
    final String channel = getBranchName(redactUnknownBranches: true);
    assert(
      kOfficialChannels.contains(channel) ||
          kObsoleteBranches.containsKey(channel) ||
          channel == kUserBranch,
      'Potential PII leak in channel name: "$channel"',
    );
    return channel;
  }

  @override
  void ensureVersionFile() {
    _ensureLegacyVersionFile(fs: fs, flutterRoot: flutterRoot, frameworkVersion: frameworkVersion);

    const JsonEncoder encoder = JsonEncoder.withIndent('  ');
    final File newVersionFile = FlutterVersion.getVersionFile(fs, flutterRoot);
    if (!newVersionFile.existsSync()) {
      newVersionFile.writeAsStringSync(encoder.convert(toJson()));
    }
  }
}

void _ensureLegacyVersionFile({
  required FileSystem fs,
  required String flutterRoot,
  required String frameworkVersion,
}) {
  final File legacyVersionFile = fs.file(fs.path.join(flutterRoot, 'version'));
  if (!legacyVersionFile.existsSync()) {
    legacyVersionFile.writeAsStringSync(frameworkVersion);
  }
}

/// Checks if the provided [version] is tracking a standard remote.
///
/// A "standard remote" is one having the same url as(in order of precedence):
///  * The value of `FLUTTER_GIT_URL` environment variable.
///  * The HTTPS or SSH url of the Flutter repository as provided by GitHub.
///
/// To initiate the validation check, call [run].
///
/// This prevents the tool to check for version freshness from the standard
/// remote but fetch updates from the upstream of current branch/channel, both
/// of which can be different.
///
/// This also prevents unnecessary freshness check from a forked repo unless the
/// user explicitly configures the environment to do so.
class VersionUpstreamValidator {
  VersionUpstreamValidator({required this.version, required this.platform});

  final Platform platform;
  final FlutterVersion version;

  /// Performs the validation against the tracking remote of the [version].
  ///
  /// Returns [VersionCheckError] if the tracking remote is not standard.
  VersionCheckError? run() {
    final String? flutterGit = platform.environment['FLUTTER_GIT_URL'];
    final String? repositoryUrl = version.repositoryUrl;

    if (repositoryUrl == null) {
      return VersionCheckError(
        'The tool could not determine the remote upstream which is being '
        'tracked by the SDK.',
      );
    }

    // Strip `.git` suffix before comparing the remotes
    final List<String> sanitizedStandardRemotes =
        <String>[
          // If `FLUTTER_GIT_URL` is set, use that as standard remote.
          if (flutterGit != null)
            flutterGit
          // Else use the predefined standard remotes.
          else
            ..._standardRemotes,
        ].map((String remote) => stripDotGit(remote)).toList();

    final String sanitizedRepositoryUrl = stripDotGit(repositoryUrl);

    if (!sanitizedStandardRemotes.contains(sanitizedRepositoryUrl)) {
      if (flutterGit != null) {
        // If `FLUTTER_GIT_URL` is set, inform to either remove the
        // `FLUTTER_GIT_URL` environment variable or set it to the current
        // tracking remote.
        return VersionCheckError(
          'The Flutter SDK is tracking "$repositoryUrl" but "FLUTTER_GIT_URL" '
          'is set to "$flutterGit".\n'
          'Either remove "FLUTTER_GIT_URL" from the environment or set it to '
          '"$repositoryUrl". '
          'If this is intentional, it is recommended to use "git" directly to '
          'manage the SDK.',
        );
      }
      // If `FLUTTER_GIT_URL` is unset, inform to set the environment variable.
      return VersionCheckError(
        'The Flutter SDK is tracking a non-standard remote "$repositoryUrl".\n'
        'Set the environment variable "FLUTTER_GIT_URL" to '
        '"$repositoryUrl". '
        'If this is intentional, it is recommended to use "git" directly to '
        'manage the SDK.',
      );
    }
    return null;
  }

  // The predefined list of remotes that are considered to be standard.
  static final List<String> _standardRemotes = <String>[
    'https://github.com/flutter/flutter.git',
    'git@github.com:flutter/flutter.git',
    'ssh://git@github.com/flutter/flutter.git',
  ];

  // Strips ".git" suffix from a given string, preferably an url.
  // For example, changes 'https://github.com/flutter/flutter.git' to 'https://github.com/flutter/flutter'.
  // URLs without ".git" suffix will remain unaffected.
  static final RegExp _patternUrlDotGit = RegExp(r'(.*)(\.git)$');
  static String stripDotGit(String url) {
    return _patternUrlDotGit.firstMatch(url)?.group(1)! ?? url;
  }
}

/// Contains data and load/save logic pertaining to Flutter version checks.
@visibleForTesting
class VersionCheckStamp {
  const VersionCheckStamp({
    this.lastTimeVersionWasChecked,
    this.lastKnownRemoteVersion,
    this.lastTimeWarningWasPrinted,
  });

  final DateTime? lastTimeVersionWasChecked;
  final DateTime? lastKnownRemoteVersion;
  final DateTime? lastTimeWarningWasPrinted;

  /// The prefix of the stamp file where we cache Flutter version check data.
  @visibleForTesting
  static const String flutterVersionCheckStampFile = 'flutter_version_check';

  static Future<VersionCheckStamp> load(Cache cache, Logger logger) async {
    final String? versionCheckStamp = cache.getStampFor(flutterVersionCheckStampFile);

    if (versionCheckStamp != null) {
      // Attempt to parse stamp JSON.
      try {
        final dynamic jsonObject = json.decode(versionCheckStamp);
        if (jsonObject is Map<String, dynamic>) {
          return fromJson(jsonObject);
        } else {
          logger.printTrace('Warning: expected version stamp to be a Map but found: $jsonObject');
        }
      } on Exception catch (error, stackTrace) {
        // Do not crash if JSON is malformed.
        logger.printTrace('${error.runtimeType}: $error\n$stackTrace');
      }
    }

    // Stamp is missing or is malformed.
    return const VersionCheckStamp();
  }

  static VersionCheckStamp fromJson(Map<String, dynamic> jsonObject) {
    DateTime? readDateTime(String property) {
      return jsonObject.containsKey(property)
          ? DateTime.parse(jsonObject[property] as String)
          : null;
    }

    return VersionCheckStamp(
      lastTimeVersionWasChecked: readDateTime('lastTimeVersionWasChecked'),
      lastKnownRemoteVersion: readDateTime('lastKnownRemoteVersion'),
      lastTimeWarningWasPrinted: readDateTime('lastTimeWarningWasPrinted'),
    );
  }

  Future<void> store({
    DateTime? newTimeVersionWasChecked,
    DateTime? newKnownRemoteVersion,
    DateTime? newTimeWarningWasPrinted,
    Cache? cache,
  }) async {
    final Map<String, String> jsonData = toJson();

    if (newTimeVersionWasChecked != null) {
      jsonData['lastTimeVersionWasChecked'] = '$newTimeVersionWasChecked';
    }

    if (newKnownRemoteVersion != null) {
      jsonData['lastKnownRemoteVersion'] = '$newKnownRemoteVersion';
    }

    if (newTimeWarningWasPrinted != null) {
      jsonData['lastTimeWarningWasPrinted'] = '$newTimeWarningWasPrinted';
    }

    const JsonEncoder prettyJsonEncoder = JsonEncoder.withIndent('  ');
    (cache ?? globals.cache).setStampFor(
      flutterVersionCheckStampFile,
      prettyJsonEncoder.convert(jsonData),
    );
  }

  Map<String, String> toJson({
    DateTime? updateTimeVersionWasChecked,
    DateTime? updateKnownRemoteVersion,
    DateTime? updateTimeWarningWasPrinted,
  }) {
    updateTimeVersionWasChecked = updateTimeVersionWasChecked ?? lastTimeVersionWasChecked;
    updateKnownRemoteVersion = updateKnownRemoteVersion ?? lastKnownRemoteVersion;
    updateTimeWarningWasPrinted = updateTimeWarningWasPrinted ?? lastTimeWarningWasPrinted;

    final Map<String, String> jsonData = <String, String>{};

    if (updateTimeVersionWasChecked != null) {
      jsonData['lastTimeVersionWasChecked'] = '$updateTimeVersionWasChecked';
    }

    if (updateKnownRemoteVersion != null) {
      jsonData['lastKnownRemoteVersion'] = '$updateKnownRemoteVersion';
    }

    if (updateTimeWarningWasPrinted != null) {
      jsonData['lastTimeWarningWasPrinted'] = '$updateTimeWarningWasPrinted';
    }

    return jsonData;
  }
}

/// Thrown when we fail to check Flutter version.
///
/// This can happen when we attempt to `git fetch` but there is no network, or
/// when the installation is not git-based (e.g. a user clones the repo but
/// then removes .git).
class VersionCheckError implements Exception {
  VersionCheckError(this.message);

  final String message;

  @override
  String toString() => '$VersionCheckError: $message';
}

/// Runs [command] and returns the standard output as a string.
///
/// If [lenient] is true and the command fails, returns an empty string.
/// Otherwise, throws a [ToolExit] exception.
String _runSync(List<String> command, {bool lenient = true, required String? workingDirectory}) {
  final ProcessResult results = globals.processManager.runSync(
    command,
    workingDirectory: workingDirectory,
  );

  if (results.exitCode == 0) {
    return (results.stdout as String).trim();
  }

  if (!lenient) {
    throw VersionCheckError(
      'Command exited with code ${results.exitCode}: ${command.join(' ')}\n'
      'Standard out: ${results.stdout}\n'
      'Standard error: ${results.stderr}',
    );
  }

  return '';
}

String _runGit(String command, ProcessUtils processUtils, String? workingDirectory) {
  return processUtils.runSync(command.split(' '), workingDirectory: workingDirectory).stdout.trim();
}

/// Runs [command] in the root of the Flutter installation and returns the
/// standard output as a string.
///
/// If the command fails, throws a [ToolExit] exception.
Future<String> _run(List<String> command) async {
  final ProcessResult results = await globals.processManager.run(
    command,
    workingDirectory: Cache.flutterRoot,
  );

  if (results.exitCode == 0) {
    return (results.stdout as String).trim();
  }

  throw VersionCheckError(
    'Command exited with code ${results.exitCode}: ${command.join(' ')}\n'
    'Standard error: ${results.stderr}',
  );
}

String _shortGitRevision(String? revision) {
  if (revision == null) {
    return '';
  }
  return revision.length > 10 ? revision.substring(0, 10) : revision;
}

/// Version of Flutter SDK parsed from Git.
class GitTagVersion {
  const GitTagVersion({
    this.x,
    this.y,
    this.z,
    this.hotfix,
    this.devVersion,
    this.devPatch,
    this.commits,
    this.hash,
    this.gitTag,
  });
  const GitTagVersion.unknown()
    : x = null,
      y = null,
      z = null,
      hotfix = null,
      commits = 0,
      devVersion = null,
      devPatch = null,
      hash = '',
      gitTag = '';

  /// The X in vX.Y.Z.
  final int? x;

  /// The Y in vX.Y.Z.
  final int? y;

  /// The Z in vX.Y.Z.
  final int? z;

  /// the F in vX.Y.Z+hotfix.F.
  final int? hotfix;

  /// Number of commits since the vX.Y.Z tag.
  final int? commits;

  /// The git hash (or an abbreviation thereof) for this commit.
  final String? hash;

  /// The N in X.Y.Z-dev.N.M.
  final int? devVersion;

  /// The M in X.Y.Z-dev.N.M.
  final int? devPatch;

  /// The git tag that is this version's closest ancestor.
  final String? gitTag;

  static GitTagVersion determine(
    ProcessUtils processUtils,
    Platform platform, {
    String? workingDirectory,
    bool fetchTags = false,
    String gitRef = 'HEAD',
  }) {
    if (fetchTags) {
      final String channel = _runGit(
        'git symbolic-ref --short HEAD',
        processUtils,
        workingDirectory,
      );
      if (!kDevelopmentChannels.contains(channel) && kOfficialChannels.contains(channel)) {
        globals.printTrace('Skipping request to fetchTags - on well known channel $channel.');
      } else {
        final String flutterGit =
            platform.environment['FLUTTER_GIT_URL'] ?? 'https://github.com/flutter/flutter.git';
        _runGit('git fetch $flutterGit --tags -f', processUtils, workingDirectory);
      }
    }
    // find all tags attached to the given [gitRef]
    final List<String> tags = _runGit(
      'git tag --points-at $gitRef',
      processUtils,
      workingDirectory,
    ).trim().split('\n');

    // Check first for a stable tag
    final RegExp stableTagPattern = RegExp(r'^\d+\.\d+\.\d+$');
    for (final String tag in tags) {
      if (stableTagPattern.hasMatch(tag.trim())) {
        return parse(tag);
      }
    }
    // Next check for a dev tag
    final RegExp devTagPattern = RegExp(r'^\d+\.\d+\.\d+-\d+\.\d+\.pre$');
    for (final String tag in tags) {
      if (devTagPattern.hasMatch(tag.trim())) {
        return parse(tag);
      }
    }

    // If we're not currently on a tag, use git describe to find the most
    // recent tag and number of commits past.
    return parse(
      _runGit('git describe --match *.*.* --long --tags $gitRef', processUtils, workingDirectory),
    );
  }

  /// Parse a version string.
  ///
  /// The version string can either be an exact release tag (e.g. '1.2.3' for
  /// stable or 1.2.3-4.5.pre for a dev) or the output of `git describe` (e.g.
  /// for commit abc123 that is 6 commits after tag 1.2.3-4.5.pre, git would
  /// return '1.2.3-4.5.pre-6-gabc123').
  static GitTagVersion parseVersion(String version) {
    final RegExp versionPattern = RegExp(
      r'^(\d+)\.(\d+)\.(\d+)(-\d+\.\d+\.pre)?(?:-(\d+)-g([a-f0-9]+))?$',
    );
    final Match? match = versionPattern.firstMatch(version.trim());
    if (match == null) {
      return const GitTagVersion.unknown();
    }

    final List<String?> matchGroups = match.groups(<int>[1, 2, 3, 4, 5, 6]);
    final int? x = matchGroups[0] == null ? null : int.tryParse(matchGroups[0]!);
    final int? y = matchGroups[1] == null ? null : int.tryParse(matchGroups[1]!);
    final int? z = matchGroups[2] == null ? null : int.tryParse(matchGroups[2]!);
    final String? devString = matchGroups[3];
    int? devVersion, devPatch;
    if (devString != null) {
      final Match? devMatch = RegExp(r'^-(\d+)\.(\d+)\.pre$').firstMatch(devString);
      final List<String?>? devGroups = devMatch?.groups(<int>[1, 2]);
      devVersion = devGroups?[0] == null ? null : int.tryParse(devGroups![0]!);
      devPatch = devGroups?[1] == null ? null : int.tryParse(devGroups![1]!);
    }
    // count of commits past last tagged version
    final int? commits = matchGroups[4] == null ? 0 : int.tryParse(matchGroups[4]!);
    final String hash = matchGroups[5] ?? '';

    return GitTagVersion(
      x: x,
      y: y,
      z: z,
      devVersion: devVersion,
      devPatch: devPatch,
      commits: commits,
      hash: hash,
      gitTag: '$x.$y.$z${devString ?? ''}', // e.g. 1.2.3-4.5.pre
    );
  }

  static GitTagVersion parse(String version) {
    GitTagVersion gitTagVersion;

    gitTagVersion = parseVersion(version);
    if (gitTagVersion != const GitTagVersion.unknown()) {
      return gitTagVersion;
    }
    globals.printTrace('Could not interpret results of "git describe": $version');
    return const GitTagVersion.unknown();
  }

  String frameworkVersionFor(String revision) {
    if (x == null || y == null || z == null || (hash != null && !revision.startsWith(hash!))) {
      return kUnknownFrameworkVersion;
    }
    if (commits == 0 && gitTag != null) {
      return gitTag!;
    }
    if (hotfix != null) {
      // This is an unexpected state where untagged commits exist past a hotfix
      return '$x.$y.$z+hotfix.${hotfix! + 1}.pre.$commits';
    }
    if (devPatch != null && devVersion != null) {
      // The next tag that will contain this commit will be the next candidate
      // branch, which will increment the devVersion.
      return '$x.$y.0-${devVersion! + 1}.0.pre.$commits';
    }
    return '$x.$y.${z! + 1}-0.0.pre.$commits';
  }
}

enum VersionCheckResult {
  /// Unable to check whether a new version is available, possibly due to
  /// a connectivity issue.
  unknown,

  /// The current version is up to date.
  versionIsCurrent,

  /// A newer version is available.
  newVersionAvailable,
}

/// Determine whether or not the provided [version] is "fresh" and notify the user if appropriate.
///
/// To initiate the validation check, call [run].
///
/// We do not want to check with the upstream git remote for newer commits on
/// every tool invocation, as this would significantly slow down running tool
/// commands. Thus, the tool writes to the [VersionCheckStamp] every time that
/// it actually has fetched commits from upstream, and this validator only
/// checks again if it has been more than [checkAgeConsideredUpToDate] since the
/// last fetch.
///
/// We do not want to notify users with "reasonably" fresh versions about new
/// releases. The method [versionAgeConsideredUpToDate] defines a different
/// duration of freshness for each channel. If [localFrameworkCommitDate] is
/// newer than this duration, then we do not show the warning.
///
/// We do not want to annoy users who intentionally disregard the warning and
/// choose not to upgrade. Thus, we only show the message if it has been more
/// than [maxTimeSinceLastWarning] since the last time the user saw the warning.
class VersionFreshnessValidator {
  VersionFreshnessValidator({
    required this.version,
    required this.localFrameworkCommitDate,
    required this.clock,
    required this.cache,
    required this.logger,
    this.latestFlutterCommitDate,
    this.pauseTime = Duration.zero,
  });

  final FlutterVersion version;
  final DateTime localFrameworkCommitDate;
  final SystemClock clock;
  final Cache cache;
  final Logger logger;
  final Duration pauseTime;
  final DateTime? latestFlutterCommitDate;

  late final DateTime now = clock.now();
  late final Duration frameworkAge = now.difference(localFrameworkCommitDate);

  /// The amount of time we wait before pinging the server to check for the
  /// availability of a newer version of Flutter.
  @visibleForTesting
  static const Duration checkAgeConsideredUpToDate = Duration(days: 3);

  /// The amount of time we wait between issuing a warning.
  ///
  /// This is to avoid annoying users who are unable to upgrade right away.
  @visibleForTesting
  static const Duration maxTimeSinceLastWarning = Duration(days: 1);

  /// The amount of time we pause for to let the user read the message about
  /// outdated Flutter installation.
  ///
  /// This can be customized in tests to speed them up.
  @visibleForTesting
  static Duration timeToPauseToLetUserReadTheMessage = const Duration(seconds: 2);

  // We show a warning if either we know there is a new remote version, or we
  // couldn't tell but the local version is outdated.
  @visibleForTesting
  bool canShowWarning(VersionCheckResult remoteVersionStatus) {
    final bool installationSeemsOutdated =
        frameworkAge > versionAgeConsideredUpToDate(version.channel);
    if (remoteVersionStatus == VersionCheckResult.newVersionAvailable) {
      return true;
    }
    if (!installationSeemsOutdated) {
      return false;
    }
    return remoteVersionStatus == VersionCheckResult.unknown;
  }

  /// We warn the user if the age of their Flutter installation is greater than
  /// this duration. The durations are slightly longer than the expected release
  /// cadence for each channel, to give the user a grace period before they get
  /// notified.
  ///
  /// For example, for the beta channel, this is set to eight weeks because
  /// beta releases happen approximately every month.
  @visibleForTesting
  static Duration versionAgeConsideredUpToDate(String channel) {
    return switch (channel) {
      'stable' => const Duration(days: 365 ~/ 2), // Six months
      'beta' => const Duration(days: 7 * 8), // Eight weeks
      _ => const Duration(days: 7 * 3), // Three weeks
    };
  }

  /// Execute validations and print warning to [logger] if necessary.
  Future<void> run() async {
    // Get whether there's a newer version on the remote. This only goes
    // to the server if we haven't checked recently so won't happen on every
    // command.
    final VersionCheckResult remoteVersionStatus;

    if (latestFlutterCommitDate == null) {
      remoteVersionStatus = VersionCheckResult.unknown;
    } else {
      if (latestFlutterCommitDate!.isAfter(localFrameworkCommitDate)) {
        remoteVersionStatus = VersionCheckResult.newVersionAvailable;
      } else {
        remoteVersionStatus = VersionCheckResult.versionIsCurrent;
      }
    }

    // Do not load the stamp before the above server check as it may modify the stamp file.
    final VersionCheckStamp stamp = await VersionCheckStamp.load(cache, logger);
    final DateTime lastTimeWarningWasPrinted =
        stamp.lastTimeWarningWasPrinted ?? clock.ago(maxTimeSinceLastWarning * 2);
    final bool beenAWhileSinceWarningWasPrinted =
        now.difference(lastTimeWarningWasPrinted) > maxTimeSinceLastWarning;
    if (!beenAWhileSinceWarningWasPrinted) {
      return;
    }

    final bool canShowWarningResult = canShowWarning(remoteVersionStatus);

    if (!canShowWarningResult) {
      return;
    }

    // By this point, we should show the update message
    final String updateMessage;
    switch (remoteVersionStatus) {
      case VersionCheckResult.newVersionAvailable:
        updateMessage = _newVersionAvailableMessage;
      case VersionCheckResult.versionIsCurrent:
      case VersionCheckResult.unknown:
        updateMessage = versionOutOfDateMessage(frameworkAge);
    }

    logger.printBox(updateMessage);
    await Future.wait<void>(<Future<void>>[
      stamp.store(newTimeWarningWasPrinted: now, cache: cache),
      Future<void>.delayed(pauseTime),
    ]);
  }
}

@visibleForTesting
String versionOutOfDateMessage(Duration frameworkAge) {
  return '''
WARNING: your installation of Flutter is ${frameworkAge.inDays} days old.

To update to the latest version, run "flutter upgrade".''';
}

const String _newVersionAvailableMessage = '''
A new version of Flutter is available!

To update to the latest version, run "flutter upgrade".''';
