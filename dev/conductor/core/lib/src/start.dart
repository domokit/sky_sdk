// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:fixnum/fixnum.dart';
import 'package:meta/meta.dart';
import 'package:platform/platform.dart';
import 'package:process/process.dart';

import 'context.dart';
import 'git.dart';
import 'globals.dart';
import 'proto/conductor_state.pb.dart' as pb;
import 'proto/conductor_state.pbenum.dart';
import 'repository.dart';
import 'state.dart' as state_import;
import 'stdio.dart';
import 'version.dart';

const String kCandidateOption = 'candidate-branch';
const String kDartRevisionOption = 'dart-revision';
const String kEngineUpstreamOption = 'engine-upstream';
const String kFrameworkMirrorOption = 'framework-mirror';
const String kFrameworkUpstreamOption = 'framework-upstream';
const String kEngineMirrorOption = 'engine-mirror';
const String kReleaseOption = 'release-channel';
const String kStateOption = 'state-file';
const String kVersionOverrideOption = 'version-override';
const String kGithubUsernameOption = 'github-username';

/// Command to print the status of the current Flutter release.
///
/// This command has many required options which the user must provide
/// via command line arguments (or optionally environment variables).
///
/// This command is the one with the worst user experience (as the user has to
/// carefully type out many different options into their terminal) and the one
/// that would benefit the most from a GUI frontend. This command will
/// optionally read its options from an environment variable to facilitate a workflow
/// in which configuration is provided by editing a bash script that sets environment
/// variables and then invokes the conductor tool.
class StartCommand extends Command<void> {
  StartCommand({required this.checkouts, required this.conductorVersion})
    : platform = checkouts.platform,
      processManager = checkouts.processManager,
      fileSystem = checkouts.fileSystem,
      stdio = checkouts.stdio {
    final String defaultPath = state_import.defaultStateFilePath(platform);
    argParser.addOption(
      kCandidateOption,
      help: 'The candidate branch the release will be based on.',
    );
    argParser.addOption(
      kReleaseOption,
      help: 'The target release channel for the release.',
      allowed: kBaseReleaseChannels,
    );
    argParser.addOption(kFrameworkMirrorOption, help: 'Configurable Framework repo mirror remote.');
    argParser.addOption(
      kFrameworkUpstreamOption,
      defaultsTo: FrameworkRepository.defaultUpstream,
      help: 'Configurable Framework repo upstream remote. Primarily for testing.',
      hide: true,
    );
    argParser.addOption(
      kEngineUpstreamOption,
      defaultsTo: EngineRepository.defaultUpstream,
      help: 'Configurable Engine repo upstream remote. Primarily for testing.',
      hide: true,
    );
    argParser.addOption(
      kStateOption,
      defaultsTo: defaultPath,
      help: 'Path to persistent state file. Defaults to $defaultPath',
    );
    argParser.addOption(kDartRevisionOption, help: 'New Dart revision to cherrypick.');
    argParser.addFlag(
      kForceFlag,
      abbr: 'f',
      help: 'Override all validations of the command line inputs.',
    );
    argParser.addOption(
      kVersionOverrideOption,
      help:
          'Explicitly set the desired version. This should only be used if '
          'the version computed by the tool is not correct.',
    );
    argParser.addOption(kGithubUsernameOption, help: 'Github username');
  }

  final Checkouts checkouts;

  final String conductorVersion;
  final FileSystem fileSystem;
  final Platform platform;
  final ProcessManager processManager;
  final Stdio stdio;

  @override
  String get name => 'start';

  @override
  String get description => 'Initialize a new Flutter release.';

  @visibleForTesting
  StartContext createContext(ArgResults argumentResults) {
    final String frameworkUpstream =
        getValueFromEnvOrArgs(kFrameworkUpstreamOption, argumentResults, platform.environment)!;
    final String githubUsername =
        getValueFromEnvOrArgs(kGithubUsernameOption, argumentResults, platform.environment)!;
    final String frameworkMirror =
        getValueFromEnvOrArgs(
          kFrameworkMirrorOption,
          argumentResults,
          platform.environment,
          allowNull: true,
        ) ??
        'git@github.com:$githubUsername/flutter.git';
    final String engineUpstream =
        getValueFromEnvOrArgs(kEngineUpstreamOption, argumentResults, platform.environment)!;
    final String engineMirror = 'git@github.com:$githubUsername/engine.git';
    final String candidateBranch =
        getValueFromEnvOrArgs(kCandidateOption, argumentResults, platform.environment)!;
    final String releaseChannel =
        getValueFromEnvOrArgs(kReleaseOption, argumentResults, platform.environment)!;
    final String? dartRevision = getValueFromEnvOrArgs(
      kDartRevisionOption,
      argumentResults,
      platform.environment,
      allowNull: true,
    );
    final bool force = getBoolFromEnvOrArgs(kForceFlag, argumentResults, platform.environment);
    final File stateFile = checkouts.fileSystem.file(
      getValueFromEnvOrArgs(kStateOption, argumentResults, platform.environment),
    );
    final String? versionOverrideString = getValueFromEnvOrArgs(
      kVersionOverrideOption,
      argumentResults,
      platform.environment,
      allowNull: true,
    );
    Version? versionOverride;
    if (versionOverrideString != null) {
      versionOverride = Version.fromString(versionOverrideString);
    }

    return StartContext(
      candidateBranch: candidateBranch,
      checkouts: checkouts,
      dartRevision: dartRevision,
      engineMirror: engineMirror,
      engineUpstream: engineUpstream,
      conductorVersion: conductorVersion,
      frameworkMirror: frameworkMirror,
      frameworkUpstream: frameworkUpstream,
      processManager: processManager,
      releaseChannel: releaseChannel,
      stateFile: stateFile,
      force: force,
      versionOverride: versionOverride,
      githubUsername: githubUsername,
    );
  }

  @override
  Future<void> run() async {
    final ArgResults argumentResults = argResults!;
    if (!platform.isMacOS && !platform.isLinux) {
      throw ConductorException('Error! This tool is only supported on macOS and Linux');
    }

    return createContext(argumentResults).run();
  }
}

/// Context for starting a new release.
///
/// This is a frontend-agnostic implementation.
class StartContext extends Context {
  StartContext({
    required this.candidateBranch,
    required this.dartRevision,
    required this.engineMirror,
    required this.engineUpstream,
    required this.frameworkMirror,
    required this.frameworkUpstream,
    required this.conductorVersion,
    required this.processManager,
    required this.releaseChannel,
    required this.githubUsername,
    required super.checkouts,
    required super.stateFile,
    this.force = false,
    this.versionOverride,
  }) : git = Git(processManager),
       engine = EngineRepository(
         checkouts,
         initialRef: 'upstream/$candidateBranch',
         upstreamRemote: Remote.upstream(engineUpstream),
         mirrorRemote: Remote.mirror(engineMirror),
       ),
       framework = FrameworkRepository(
         checkouts,
         initialRef: 'upstream/$candidateBranch',
         upstreamRemote: Remote.upstream(frameworkUpstream),
         mirrorRemote: Remote.mirror(frameworkMirror),
       );

  final String candidateBranch;
  final String? dartRevision;
  final String engineMirror;
  final String engineUpstream;
  final String frameworkMirror;
  final String frameworkUpstream;
  final String conductorVersion;
  final Git git;
  final ProcessManager processManager;
  final String releaseChannel;
  final Version? versionOverride;
  final String githubUsername;

  /// If validations should be overridden.
  final bool force;

  final EngineRepository engine;
  final FrameworkRepository framework;

  /// Determine which part of the version to increment in the next release.
  ///
  /// If [atBranchPoint] is true, then this is a [ReleaseType.BETA_INITIAL].
  @visibleForTesting
  ReleaseType computeReleaseType(Version lastVersion, bool atBranchPoint) {
    if (atBranchPoint) {
      return ReleaseType.BETA_INITIAL;
    }
    if (releaseChannel != 'stable') {
      return ReleaseType.BETA_HOTFIX;
    }

    return switch (lastVersion.type) {
      VersionType.stable => ReleaseType.STABLE_HOTFIX,
      VersionType.development || VersionType.gitDescribe || VersionType.latest =>
        ReleaseType.STABLE_INITIAL,
    };
  }

  Future<void> run() async {
    if (stateFile.existsSync()) {
      throw ConductorException(
        'Error! A persistent state file already found at ${stateFile.path}.\n\n'
        'Run `conductor clean` to cancel a previous release.',
      );
    }
    if (!releaseCandidateBranchRegex.hasMatch(candidateBranch)) {
      throw ConductorException(
        'Invalid release candidate branch "$candidateBranch". Text should '
        'match the regex pattern /${releaseCandidateBranchRegex.pattern}/.',
      );
    }

    final Int64 unixDate = Int64(DateTime.now().millisecondsSinceEpoch);
    final pb.ConductorState state = pb.ConductorState();

    state.releaseChannel = releaseChannel;
    state.createdDate = unixDate;
    state.lastUpdatedDate = unixDate;

    // Create a new branch so that we don't accidentally push to upstream
    // candidateBranch.
    final String workingBranchName = 'cherrypicks-$candidateBranch';
    await engine.newBranch(workingBranchName);

    if (dartRevision != null && dartRevision!.isNotEmpty) {
      await engine.updateDartRevision(dartRevision!);
      await engine.commit('Update Dart SDK to $dartRevision', addFirst: true);
    }

    final String engineHead = await engine.reverseParse('HEAD');
    state.engine =
        (pb.Repository.create()
          ..candidateBranch = candidateBranch
          ..workingBranch = workingBranchName
          ..startingGitHead = engineHead
          ..currentGitHead = engineHead
          ..checkoutPath = (await engine.checkoutDirectory).path
          ..upstream =
              (pb.Remote.create()
                ..name = 'upstream'
                ..url = engine.upstreamRemote.url)
          ..mirror =
              (pb.Remote.create()
                ..name = 'mirror'
                ..url = engine.mirrorRemote!.url));
    if (dartRevision != null && dartRevision!.isNotEmpty) {
      state.engine.dartRevision = dartRevision!;
    }

    await framework.newBranch(workingBranchName);

    // Get framework version
    final Version lastVersion = Version.fromString(
      await framework.getFullTag(framework.upstreamRemote.name, candidateBranch, exact: false),
    );

    final String frameworkHead = await framework.reverseParse('HEAD');
    final String branchPoint = await framework.branchPoint(
      '${framework.upstreamRemote.name}/$candidateBranch',
      '${framework.upstreamRemote.name}/${FrameworkRepository.defaultBranch}',
    );
    final bool atBranchPoint = branchPoint == frameworkHead;

    final ReleaseType releaseType = computeReleaseType(lastVersion, atBranchPoint);
    state.releaseType = releaseType;

    try {
      lastVersion.ensureValid(candidateBranch, releaseType);
    } on ConductorException catch (e) {
      // Let the user know, but resume execution
      stdio.printError(e.message);
    }

    Version nextVersion;
    if (versionOverride != null) {
      nextVersion = versionOverride!;
    } else {
      nextVersion = calculateNextVersion(lastVersion, releaseType);
      nextVersion = await ensureBranchPointTagged(
        branchPoint: branchPoint,
        requestedVersion: nextVersion,
        framework: framework,
      );
    }

    state.releaseVersion = nextVersion.toString();

    state.framework =
        (pb.Repository.create()
          ..candidateBranch = candidateBranch
          ..workingBranch = workingBranchName
          ..startingGitHead = frameworkHead
          ..currentGitHead = frameworkHead
          ..checkoutPath = (await framework.checkoutDirectory).path
          ..upstream =
              (pb.Remote.create()
                ..name = 'upstream'
                ..url = framework.upstreamRemote.url)
          ..mirror =
              (pb.Remote.create()
                ..name = 'mirror'
                ..url = framework.mirrorRemote!.url));

    state.currentPhase = ReleasePhase.APPLY_ENGINE_CHERRYPICKS;

    state.conductorVersion = conductorVersion;

    stdio.printTrace('Writing state to file ${stateFile.path}...');

    updateState(state, stdio.logs);

    stdio.printStatus(state_import.presentState(state));
  }

  /// Determine this release's version number from the [lastVersion] and the [incrementLetter].
  Version calculateNextVersion(Version lastVersion, ReleaseType releaseType) {
    return switch (releaseType) {
      ReleaseType.STABLE_INITIAL => Version(
        x: lastVersion.x,
        y: lastVersion.y,
        z: 0,
        type: VersionType.stable,
      ),
      ReleaseType.STABLE_HOTFIX => Version.increment(lastVersion, 'z'),
      ReleaseType.BETA_INITIAL => Version.fromCandidateBranch(candidateBranch),
      ReleaseType.BETA_HOTFIX || _ => Version.increment(lastVersion, 'n'),
    };
  }

  /// Ensures the branch point [candidateBranch] and `master` has a version tag.
  ///
  /// This is necessary for version reporting for users on the `master` channel
  /// to be correct.
  Future<Version> ensureBranchPointTagged({
    required Version requestedVersion,
    required String branchPoint,
    required FrameworkRepository framework,
  }) async {
    if (await framework.isCommitTagged(branchPoint)) {
      // The branch point is tagged, no work to be done
      return requestedVersion;
    }
    if (requestedVersion.n != 0) {
      stdio.printError(
        'Tried to tag the branch point, however the target version is '
        '$requestedVersion, which does not have n == 0!',
      );
      return requestedVersion;
    }

    final bool response = await prompt(
      'About to tag the release candidate branch branchpoint of $branchPoint '
      'as $requestedVersion and push it to ${framework.upstreamRemote.url}. '
      'Is this correct?',
    );

    if (!response) {
      throw ConductorException('Aborting command.');
    }

    stdio.printStatus('Applying the tag $requestedVersion at the branch point $branchPoint');

    await framework.tag(branchPoint, requestedVersion.toString(), frameworkUpstream);
    final Version nextVersion = Version.increment(requestedVersion, 'n');
    stdio.printStatus('The actual release will be version $nextVersion.');
    return nextVersion;
  }
}
