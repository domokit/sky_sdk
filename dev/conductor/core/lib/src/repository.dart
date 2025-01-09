// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:platform/platform.dart';
import 'package:process/process.dart';

import './git.dart';
import './globals.dart';
import './stdio.dart';
import './version.dart';

/// Allowed git remote names.
enum RemoteName { upstream, mirror }

class Remote {
  const Remote({required RemoteName name, required this.url}) : _name = name, assert(url != '');

  const Remote.mirror(String url) : this(name: RemoteName.mirror, url: url);
  const Remote.upstream(String url) : this(name: RemoteName.upstream, url: url);

  final RemoteName _name;

  /// The name of the remote.
  String get name => switch (_name) {
    RemoteName.upstream => 'upstream',
    RemoteName.mirror => 'mirror',
  };

  /// The URL of the remote.
  final String url;
}

/// A source code repository.
///
/// This class is an abstraction over a git
/// repository on the local disk. Ideally this abstraction would hide from
/// the outside libraries what git calls were needed to either read or update
/// data in the underlying repository. In practice, most of the bugs in the
/// conductor codebase are related to the git calls made from this and its
/// subclasses.
///
/// Two factors that make this code more complicated than it would otherwise
/// need to be are:
/// 1. That any particular invocation of the conductor may or may not already
/// have the git checkout present on disk, depending on what commands were
/// previously run; and
/// 2. The need to provide overrides for integration tests (in particular
/// the ability to mark a [Repository] instance as a [localUpstream] made
/// integration tests more hermetic, at the cost of complexity in the
/// implementation).
///
/// The only way to simplify the first factor would be to change the behavior of
/// the conductor tool to be a long-lived dart process that keeps all of its
/// state in memory and blocks on user input. This would add the constraint that
/// the user would need to keep the process running for the duration of a
/// release, which could potentially take multiple days and users could not
/// manually change the state of the release process (via editing the JSON
/// config file). However, these may be reasonable trade-offs to make the
/// codebase simpler and easier to reason about.
///
/// The way to simplify the second factor would be to not put any special
/// handling in this library for integration tests. This would make integration
/// tests more difficult/less hermetic, but the production code more reliable.
/// This is probably the right trade-off to make, as the integration tests were
/// still not hermetic or reliable, and the main integration test was ultimately
/// deleted in #84354.
abstract class Repository {
  Repository({
    required this.name,
    required this.upstreamRemote,
    required this.processManager,
    required this.stdio,
    required this.platform,
    required this.fileSystem,
    required this.parentDirectory,
    required this.requiredLocalBranches,
    this.initialRef,
    this.localUpstream = false,
    this.previousCheckoutLocation,
    this.mirrorRemote,
  }) : git = Git(processManager),
       assert(upstreamRemote.url.isNotEmpty);

  final String name;
  final Remote upstreamRemote;

  /// Branches that must exist locally in this [Repository].
  ///
  /// If this [Repository] is used as a local upstream for another, the
  /// downstream may try to fetch these branches, and git will fail if they do
  /// not exist.
  final List<String> requiredLocalBranches;

  /// Remote for user's mirror.
  ///
  /// This value can be null, in which case attempting to access it will lead to
  /// a [ConductorException].
  final Remote? mirrorRemote;

  /// The initial ref (branch or commit name) to check out.
  final String? initialRef;
  final Git git;
  final ProcessManager processManager;
  final Stdio stdio;
  final Platform platform;
  final FileSystem fileSystem;
  final Directory parentDirectory;

  /// If the repository will be used as an upstream for a test repo.
  final bool localUpstream;

  Directory? _checkoutDirectory;
  String? previousCheckoutLocation;

  /// Directory for the repository checkout.
  ///
  /// Since cloning a repository takes a long time, we do not ensure it is
  /// cloned on the filesystem until this getter is accessed.
  Future<Directory> get checkoutDirectory async {
    if (_checkoutDirectory != null) {
      return _checkoutDirectory!;
    }
    if (previousCheckoutLocation != null) {
      _checkoutDirectory = fileSystem.directory(previousCheckoutLocation);
      if (!_checkoutDirectory!.existsSync()) {
        throw ConductorException(
          'Provided previousCheckoutLocation $previousCheckoutLocation does not exist on disk!',
        );
      }
      if (initialRef != null) {
        assert(initialRef != '');
        await git.run(
          <String>['fetch', upstreamRemote.name],
          'Fetch ${upstreamRemote.name} to ensure we have latest refs',
          workingDirectory: _checkoutDirectory!.path,
        );
        // If [initialRef] is a remote ref, the checkout will be left in a detached HEAD state.
        await git.run(
          <String>['checkout', initialRef!],
          'Checking out initialRef $initialRef',
          workingDirectory: _checkoutDirectory!.path,
        );
      }

      return _checkoutDirectory!;
    }

    _checkoutDirectory = parentDirectory.childDirectory(name);
    await lazilyInitialize(_checkoutDirectory!);

    return _checkoutDirectory!;
  }

  /// RegExp pattern to parse the output of git ls-remote.
  ///
  /// Git output looks like:
  ///
  /// 35185330c6af3a435f615ee8ac2fed8b8bb7d9d4        refs/heads/95159-squash
  /// 6f60a1e7b2f3d2c2460c9dc20fe54d0e9654b131        refs/heads/add-debug-trace
  /// c1436c42c0f3f98808ae767e390c3407787f1a67        refs/heads/add-recipe-field
  /// 4d44dca340603e25d4918c6ef070821181202e69        refs/heads/add-release-channel
  ///
  /// We are interested in capturing what comes after 'refs/heads/'.
  static final RegExp _lsRemotePattern = RegExp(r'.*\s+refs\/heads\/([^\s]+)$');

  /// Parse git ls-remote --heads and return branch names.
  Future<List<String>> listRemoteBranches(String remote) async {
    final String output = await git.getOutput(
      <String>['ls-remote', '--heads', remote],
      'get remote branches',
      workingDirectory: (await checkoutDirectory).path,
    );

    return <String>[
      for (final String line in output.split('\n'))
        if (_lsRemotePattern.firstMatch(line) case final RegExpMatch match) match.group(1)!,
    ];
  }

  /// Ensure the repository is cloned to disk and initialized with proper state.
  Future<void> lazilyInitialize(Directory checkoutDirectory) async {
    if (checkoutDirectory.existsSync()) {
      stdio.printTrace('Deleting $name from ${checkoutDirectory.path}...');
      checkoutDirectory.deleteSync(recursive: true);
    }

    stdio.printTrace('Cloning $name from ${upstreamRemote.url} to ${checkoutDirectory.path}...');
    await git.run(
      <String>[
        'clone',
        '--origin',
        upstreamRemote.name,
        '--',
        upstreamRemote.url,
        checkoutDirectory.path,
      ],
      'Cloning $name repo',
      workingDirectory: parentDirectory.path,
    );
    if (mirrorRemote != null) {
      await git.run(
        <String>['remote', 'add', mirrorRemote!.name, mirrorRemote!.url],
        'Adding remote ${mirrorRemote!.url} as ${mirrorRemote!.name}',
        workingDirectory: checkoutDirectory.path,
      );
      await git.run(
        <String>['fetch', mirrorRemote!.name],
        'Fetching git remote ${mirrorRemote!.name}',
        workingDirectory: checkoutDirectory.path,
      );
    }
    if (localUpstream) {
      // These branches must exist locally for the repo that depends on it
      // to fetch and push to.
      for (final String channel in requiredLocalBranches) {
        await git.run(
          <String>['checkout', channel, '--'],
          'check out branch $channel locally',
          workingDirectory: checkoutDirectory.path,
        );
      }
    }

    if (initialRef != null) {
      await git.run(
        <String>['checkout', initialRef!],
        'Checking out initialRef $initialRef',
        workingDirectory: checkoutDirectory.path,
      );
    }
    final String revision = await reverseParse('HEAD');
    stdio.printTrace('Repository $name is checked out at revision "$revision".');
  }

  /// The URL of the remote named [remoteName].
  Future<String> remoteUrl(String remoteName) async {
    return git.getOutput(
      <String>['remote', 'get-url', remoteName],
      'verify the URL of the $remoteName remote',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Get the working tree status.
  ///
  /// Calls `git status --porcelain` which should output in a stable format
  /// across git versions.
  Future<String> gitStatus() async {
    return git.getOutput(
      <String>['status', '--porcelain'],
      'check that the git checkout is clean',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Verify the repository's git checkout is clean.
  Future<bool> gitCheckoutClean() async {
    return (await gitStatus()).isEmpty;
  }

  /// Return the revision for the branch point between two refs.
  Future<String> branchPoint(String firstRef, String secondRef) async {
    return (await git.getOutput(
      <String>['merge-base', firstRef, secondRef],
      'determine the merge base between $firstRef and $secondRef',
      workingDirectory: (await checkoutDirectory).path,
    )).trim();
  }

  /// Fetch all branches and associated commits and tags from [remoteName].
  Future<void> fetch(String remoteName) async {
    await git.run(
      <String>['fetch', remoteName, '--tags'],
      'fetch $remoteName --tags',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Create (and checkout) a new branch based on the current HEAD.
  ///
  /// Runs `git checkout -b $branchName`.
  Future<void> newBranch(String branchName) async {
    await git.run(
      <String>['checkout', '-b', branchName],
      'create & checkout new branch $branchName',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Check out the given ref.
  Future<void> checkout(String ref) async {
    await git.run(
      <String>['checkout', ref],
      'checkout ref',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Obtain the version tag at the tip of a release branch.
  Future<String> getFullTag(String remoteName, String branchName, {bool exact = true}) async {
    // includes both stable (e.g. 1.2.3) and dev tags (e.g. 1.2.3-4.5.pre)
    const String glob = '*.*.*';
    // describe the latest dev release
    final String ref = 'refs/remotes/$remoteName/$branchName';
    return git.getOutput(
      <String>['describe', '--match', glob, if (exact) '--exact-match', '--tags', ref],
      'obtain last released version number',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Tag [commit] and push the tag to the remote.
  Future<void> tag(String commit, String tagName, String remote) async {
    assert(commit.isNotEmpty);
    assert(tagName.isNotEmpty);
    assert(remote.isNotEmpty);
    stdio.printStatus('About to tag commit $commit as $tagName...');
    await git.run(
      <String>['tag', tagName, commit],
      'tag the commit with the version label',
      workingDirectory: (await checkoutDirectory).path,
    );
    stdio.printStatus('Tagging successful.');
    stdio.printStatus('About to push $tagName to remote $remote...');
    await git.run(
      <String>['push', remote, tagName],
      'publish the tag to the repo',
      workingDirectory: (await checkoutDirectory).path,
    );
    stdio.printStatus('Tag push successful.');
  }

  /// List commits in reverse chronological order.
  Future<List<String>> revList(List<String> args) async {
    return (await git.getOutput(
      <String>['rev-list', ...args],
      'rev-list with args ${args.join(' ')}',
      workingDirectory: (await checkoutDirectory).path,
    )).trim().split('\n');
  }

  /// Look up the commit for [ref].
  Future<String> reverseParse(String ref) async {
    final String revisionHash = await git.getOutput(
      <String>['rev-parse', ref],
      'look up the commit for the ref $ref',
      workingDirectory: (await checkoutDirectory).path,
    );
    assert(revisionHash.isNotEmpty);
    return revisionHash;
  }

  /// Determines if one ref is an ancestor for another.
  Future<bool> isAncestor(String possibleAncestor, String possibleDescendant) async {
    final io.ProcessResult result = await git.run(
      <String>['merge-base', '--is-ancestor', possibleDescendant, possibleAncestor],
      'verify $possibleAncestor is a direct ancestor of $possibleDescendant.',
      allowNonZeroExitCode: true,
      workingDirectory: (await checkoutDirectory).path,
    );
    return result.exitCode == 0;
  }

  /// Determines if a given commit has a tag.
  Future<bool> isCommitTagged(String commit) async {
    final io.ProcessResult result = await git.run(
      <String>['describe', '--exact-match', '--tags', commit],
      'verify $commit is already tagged',
      allowNonZeroExitCode: true,
      workingDirectory: (await checkoutDirectory).path,
    );
    return result.exitCode == 0;
  }

  /// Resets repository HEAD to [ref].
  Future<void> reset(String ref) async {
    await git.run(
      <String>['reset', ref, '--hard'],
      'reset to $ref',
      workingDirectory: (await checkoutDirectory).path,
    );
  }

  /// Push [commit] to the release channel [branch].
  Future<void> pushRef({
    required String fromRef,
    required String remote,
    required String toRef,
    bool force = false,
    bool dryRun = false,
  }) async {
    final List<String> args = <String>['push', if (force) '--force', remote, '$fromRef:$toRef'];
    final String command = <String>['git', ...args].join(' ');
    if (dryRun) {
      stdio.printStatus('About to execute command: `$command`');
    } else {
      await git.run(
        args,
        'update the release branch with the commit',
        workingDirectory: (await checkoutDirectory).path,
      );
      stdio.printStatus('Executed command: `$command`');
    }
  }

  Future<String> commit(String message, {bool addFirst = false, String? author}) async {
    final bool hasChanges =
        (await git.getOutput(
          <String>['status', '--porcelain'],
          'check for uncommitted changes',
          workingDirectory: (await checkoutDirectory).path,
        )).trim().isNotEmpty;
    if (!hasChanges) {
      throw ConductorException('Tried to commit with message $message but no changes were present');
    }
    if (addFirst) {
      await git.run(
        <String>['add', '--all'],
        'add all changes to the index',
        workingDirectory: (await checkoutDirectory).path,
      );
    }
    String? authorArg;
    if (author != null) {
      if (author.contains('"')) {
        throw FormatException('Commit author cannot contain character \'"\', received $author');
      }
      // verify [author] matches git author convention, e.g. "Jane Doe <jane.doe@email.com>"
      if (!RegExp(r'.+<.*>').hasMatch(author)) {
        throw FormatException('Commit author appears malformed: "$author"');
      }
      authorArg = '--author="$author"';
    }
    final List<String> commitCmd = <String>[
      'commit',
      '--message',
      message,
      if (authorArg != null) authorArg,
    ];
    stdio.printTrace('Executing git $commitCmd...');
    final io.ProcessResult commitResult = await git.run(
      commitCmd,
      'commit changes',
      workingDirectory: (await checkoutDirectory).path,
    );
    final String stdout = commitResult.stdout as String;
    if (stdout.isNotEmpty) {
      stdio.printTrace(stdout);
    }
    final String stderr = commitResult.stderr as String;
    if (stderr.isNotEmpty) {
      stdio.printTrace(stderr);
    }

    return reverseParse('HEAD');
  }

  /// Create an empty commit and return the revision.
  @visibleForTesting
  Future<String> authorEmptyCommit([String message = 'An empty commit']) async {
    await git.run(
      <String>[
        '-c',
        'user.name=Conductor',
        '-c',
        'user.email=conductor@flutter.dev',
        'commit',
        '--allow-empty',
        '-m',
        "'$message'",
      ],
      'create an empty commit',
      workingDirectory: (await checkoutDirectory).path,
    );
    return reverseParse('HEAD');
  }

  /// Create a new clone of the current repository.
  ///
  /// The returned repository will inherit all properties from this one, except
  /// for the upstream, which will be the path to this repository on disk.
  ///
  /// This method is for testing purposes.
  @visibleForTesting
  Future<Repository> cloneRepository(String cloneName);
}

class FrameworkRepository extends Repository {
  FrameworkRepository(
    this.checkouts, {
    super.name = 'framework',
    super.upstreamRemote = const Remote.upstream(FrameworkRepository.defaultUpstream),
    super.localUpstream,
    super.previousCheckoutLocation,
    String super.initialRef = FrameworkRepository.defaultBranch,
    super.mirrorRemote,
    List<String>? additionalRequiredLocalBranches,
  }) : super(
         fileSystem: checkouts.fileSystem,
         parentDirectory: checkouts.directory,
         platform: checkouts.platform,
         processManager: checkouts.processManager,
         stdio: checkouts.stdio,
         requiredLocalBranches: <String>[...?additionalRequiredLocalBranches, ...kReleaseChannels],
       );

  /// A [FrameworkRepository] with the host conductor's repo set as upstream.
  ///
  /// This is useful when testing a commit that has not been merged upstream
  /// yet.
  factory FrameworkRepository.localRepoAsUpstream(
    Checkouts checkouts, {
    String name = 'framework',
    String? previousCheckoutLocation,
    String initialRef = FrameworkRepository.defaultBranch,
    required String upstreamPath,
  }) {
    return FrameworkRepository(
      checkouts,
      name: name,
      upstreamRemote: Remote.upstream('file://$upstreamPath/'),
      previousCheckoutLocation: previousCheckoutLocation,
      initialRef: initialRef,
    );
  }

  final Checkouts checkouts;
  static const String defaultUpstream = 'git@github.com:flutter/flutter.git';
  static const String defaultBranch = 'master';

  Future<String> get cacheDirectory async {
    return fileSystem.path.join((await checkoutDirectory).path, 'bin', 'cache');
  }

  @override
  Future<FrameworkRepository> cloneRepository(String? cloneName) async {
    assert(localUpstream);
    cloneName ??= 'clone-of-$name';
    return FrameworkRepository(
      checkouts,
      name: cloneName,
      upstreamRemote: Remote.upstream('file://${(await checkoutDirectory).path}/'),
    );
  }

  Future<void> _ensureToolReady() async {
    final File toolsStamp = fileSystem
        .directory(await cacheDirectory)
        .childFile('flutter_tools.stamp');
    if (toolsStamp.existsSync()) {
      final String toolsStampHash = toolsStamp.readAsStringSync().trim();
      final String repoHeadHash = await reverseParse('HEAD');
      if (toolsStampHash == repoHeadHash) {
        return;
      }
    }

    stdio.printTrace('Building tool...');
    // Build tool
    await processManager.run(<String>[
      fileSystem.path.join((await checkoutDirectory).path, 'bin', 'flutter'),
      'help',
    ]);
  }

  Future<io.ProcessResult> runFlutter(List<String> args) async {
    await _ensureToolReady();
    final String workingDirectory = (await checkoutDirectory).path;
    return processManager.run(<String>[
      fileSystem.path.join(workingDirectory, 'bin', 'flutter'),
      ...args,
    ], workingDirectory: workingDirectory);
  }

  Future<void> streamDart(List<String> args, {String? workingDirectory}) async {
    final String repoWorkingDirectory = (await checkoutDirectory).path;

    await _streamProcess(<String>[
      fileSystem.path.join(repoWorkingDirectory, 'bin', 'dart'),
      ...args,
    ], workingDirectory: workingDirectory ?? repoWorkingDirectory);
  }

  Future<io.Process> streamFlutter(
    List<String> args, {
    void Function(String)? stdoutCallback,
    void Function(String)? stderrCallback,
  }) async {
    final String workingDirectory = (await checkoutDirectory).path;

    return _streamProcess(<String>[
      fileSystem.path.join(workingDirectory, 'bin', 'flutter'),
      ...args,
    ], workingDirectory: workingDirectory);
  }

  Future<io.Process> _streamProcess(
    List<String> cmd, {
    void Function(String)? stdoutCallback,
    void Function(String)? stderrCallback,
    String? workingDirectory,
  }) async {
    stdio.printTrace('Executing $cmd...');
    final io.Process process = await processManager.start(cmd, workingDirectory: workingDirectory);
    final StreamSubscription<String> stdoutSub = process.stdout
        .transform(utf8.decoder)
        .transform(const LineSplitter())
        .listen(stdoutCallback ?? stdio.printTrace);
    final StreamSubscription<String> stderrSub = process.stderr
        .transform(utf8.decoder)
        .transform(const LineSplitter())
        .listen(stderrCallback ?? stdio.printError);
    await Future.wait<void>(<Future<void>>[stdoutSub.asFuture<void>(), stderrSub.asFuture<void>()]);
    unawaited(stdoutSub.cancel());
    unawaited(stderrSub.cancel());

    final int exitCode = await process.exitCode;
    if (exitCode != 0) {
      throw io.ProcessException(cmd.first, cmd.sublist(1), 'Process failed', exitCode);
    }
    return process;
  }

  @override
  Future<void> checkout(String ref) async {
    await super.checkout(ref);
    // The tool will overwrite old cached artifacts, but not delete unused
    // artifacts from a previous version. Thus, delete the entire cache and
    // re-populate.
    final Directory cache = fileSystem.directory(await cacheDirectory);
    if (cache.existsSync()) {
      stdio.printTrace('Deleting cache...');
      cache.deleteSync(recursive: true);
    }
    await _ensureToolReady();
  }

  Future<Version> flutterVersion() async {
    // Check version
    final io.ProcessResult result = await runFlutter(<String>['--version', '--machine']);
    final Map<String, dynamic> versionJson =
        jsonDecode(stdoutToString(result.stdout)) as Map<String, dynamic>;
    return Version.fromString(versionJson['frameworkVersion'] as String);
  }

  /// Create a release candidate branch version file.
  ///
  /// This file allows for easily traversing what candidate branch was used
  /// from a release channel.
  ///
  /// Returns [true] if the version file was updated and a commit is needed.
  Future<bool> updateCandidateBranchVersion(
    String branch, {
    @visibleForTesting File? versionFile,
  }) async {
    assert(branch.isNotEmpty);
    versionFile ??= (await checkoutDirectory)
        .childDirectory('bin')
        .childDirectory('internal')
        .childFile('release-candidate-branch.version');
    if (versionFile.existsSync()) {
      final String oldCandidateBranch = versionFile.readAsStringSync();
      if (oldCandidateBranch.trim() == branch.trim()) {
        stdio.printTrace(
          'Tried to update the candidate branch but version file is already up to date at: $branch',
        );
        return false;
      }
    }
    stdio.printStatus('Create ${versionFile.path} containing $branch');
    versionFile.writeAsStringSync(
      // Version files have trailing newlines
      '${branch.trim()}\n',
      flush: true,
    );
    return true;
  }

  /// Update this framework's engine version file.
  ///
  /// Returns [true] if the version file was updated and a commit is needed.
  Future<bool> updateEngineRevision(
    String newEngine, {
    @visibleForTesting File? engineVersionFile,
  }) async {
    assert(newEngine.isNotEmpty);
    engineVersionFile ??= (await checkoutDirectory)
        .childDirectory('bin')
        .childDirectory('internal')
        .childFile('engine.version');
    assert(engineVersionFile.existsSync());
    final String oldEngine = engineVersionFile.readAsStringSync();
    if (oldEngine.trim() == newEngine.trim()) {
      stdio.printTrace(
        'Tried to update the engine revision but version file is already up to date at: $newEngine',
      );
      return false;
    }
    stdio.printStatus('Updating engine revision from $oldEngine to $newEngine');
    engineVersionFile.writeAsStringSync(
      // Version files have trailing newlines
      '${newEngine.trim()}\n',
      flush: true,
    );
    return true;
  }
}

/// A wrapper around the host repository that is executing the conductor.
///
/// [Repository] methods that mutate the underlying repository will throw a
/// [ConductorException].
class HostFrameworkRepository extends FrameworkRepository {
  HostFrameworkRepository({
    required Checkouts checkouts,
    String name = 'host-framework',
    required String upstreamPath,
  }) : super(
         checkouts,
         name: name,
         upstreamRemote: Remote.upstream('file://$upstreamPath/'),
         localUpstream: false,
       ) {
    _checkoutDirectory = checkouts.fileSystem.directory(upstreamPath);
  }

  @override
  Future<Directory> get checkoutDirectory async => _checkoutDirectory!;

  @override
  Future<void> newBranch(String branchName) async {
    throw ConductorException('newBranch not implemented for the host repository');
  }

  @override
  Future<void> checkout(String ref) async {
    throw ConductorException('checkout not implemented for the host repository');
  }

  @override
  Future<String> reset(String ref) async {
    throw ConductorException('reset not implemented for the host repository');
  }

  @override
  Future<void> tag(String commit, String tagName, String remote) async {
    throw ConductorException('tag not implemented for the host repository');
  }

  void updateChannel(
    String commit,
    String remote,
    String branch, {
    bool force = false,
    bool dryRun = false,
  }) {
    throw ConductorException('updateChannel not implemented for the host repository');
  }

  @override
  Future<String> authorEmptyCommit([String message = 'An empty commit']) async {
    throw ConductorException('authorEmptyCommit not implemented for the host repository');
  }
}

class EngineRepository extends Repository {
  EngineRepository(
    this.checkouts, {
    super.name = 'engine',
    String super.initialRef = EngineRepository.defaultBranch,
    super.upstreamRemote = const Remote.upstream(EngineRepository.defaultUpstream),
    super.localUpstream,
    super.previousCheckoutLocation,
    super.mirrorRemote,
    List<String>? additionalRequiredLocalBranches,
  }) : super(
         fileSystem: checkouts.fileSystem,
         parentDirectory: checkouts.directory,
         platform: checkouts.platform,
         processManager: checkouts.processManager,
         stdio: checkouts.stdio,
         requiredLocalBranches: additionalRequiredLocalBranches ?? const <String>[],
       );

  final Checkouts checkouts;

  static const String defaultUpstream = 'git@github.com:flutter/engine.git';
  static const String defaultBranch = 'main';

  /// Update the `dart_revision` entry in the DEPS file.
  Future<void> updateDartRevision(String newRevision, {@visibleForTesting File? depsFile}) async {
    assert(newRevision.length == 40);
    depsFile ??= (await checkoutDirectory).childFile('DEPS');
    final String fileContent = depsFile.readAsStringSync();
    final RegExp dartPattern = RegExp("[ ]+'dart_revision': '([a-z0-9]{40})',");
    final Iterable<RegExpMatch> allMatches = dartPattern.allMatches(fileContent);
    if (allMatches.length != 1) {
      throw ConductorException(
        'Unexpected content in the DEPS file at ${depsFile.path}\n'
        'Expected to find pattern ${dartPattern.pattern} 1 times, but got '
        '${allMatches.length}.',
      );
    }
    final String updatedFileContent = fileContent.replaceFirst(
      dartPattern,
      "  'dart_revision': '$newRevision',",
    );

    depsFile.writeAsStringSync(updatedFileContent, flush: true);
  }

  @override
  Future<Repository> cloneRepository(String? cloneName) async {
    assert(localUpstream);
    cloneName ??= 'clone-of-$name';
    return EngineRepository(
      checkouts,
      name: cloneName,
      upstreamRemote: Remote.upstream('file://${(await checkoutDirectory).path}/'),
    );
  }
}

/// An enum of all the repositories that the Conductor supports.
enum RepositoryType { framework, engine }

class Checkouts {
  Checkouts({
    required this.fileSystem,
    required this.platform,
    required this.processManager,
    required this.stdio,
    required Directory parentDirectory,
    String directoryName = 'flutter_conductor_checkouts',
  }) : directory = parentDirectory.childDirectory(directoryName) {
    if (!directory.existsSync()) {
      directory.createSync(recursive: true);
    }
  }

  final Directory directory;
  final FileSystem fileSystem;
  final Platform platform;
  final ProcessManager processManager;
  final Stdio stdio;
}
