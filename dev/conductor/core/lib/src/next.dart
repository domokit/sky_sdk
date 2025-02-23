// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/file.dart' show File;
import 'package:meta/meta.dart' show visibleForTesting;

import 'context.dart';
import 'git.dart';
import 'globals.dart';
import 'proto/conductor_state.pb.dart' as pb;
import 'proto/conductor_state.pbenum.dart';
import 'repository.dart';
import 'state.dart' as state_import;

const String kStateOption = 'state-file';
const String kYesFlag = 'yes';

/// Command to proceed from one [pb.ReleasePhase] to the next.
///
/// After `conductor start`, the rest of the release steps are initiated by the
/// user via `conductor next`. Thus this command's behavior is conditional upon
/// which phase of the release the user is currently in. This is implemented
/// with a switch case statement.
class NextCommand extends Command<void> {
  NextCommand({required this.checkouts}) {
    final String defaultPath = state_import.defaultStateFilePath(checkouts.platform);
    argParser.addOption(
      kStateOption,
      defaultsTo: defaultPath,
      help: 'Path to persistent state file. Defaults to $defaultPath',
    );
    argParser.addFlag(
      kYesFlag,
      help: 'Auto-accept any confirmation prompts.',
      hide: true, // primarily for integration testing
    );
    argParser.addFlag(kForceFlag, help: 'Force push when updating remote git branches.');
  }

  final Checkouts checkouts;

  @override
  String get name => 'next';

  @override
  String get description => 'Proceed to the next release phase.';

  @override
  Future<void> run() async {
    final File stateFile = checkouts.fileSystem.file(argResults![kStateOption]);
    if (!stateFile.existsSync()) {
      throw ConductorException('No persistent state file found at ${stateFile.path}.');
    }
    final pb.ConductorState state = state_import.readStateFromFile(stateFile);

    await NextContext(
      autoAccept: argResults![kYesFlag] as bool,
      checkouts: checkouts,
      force: argResults![kForceFlag] as bool,
      stateFile: stateFile,
    ).run(state);
  }
}

/// Utility class for proceeding to the next step in a release.
///
/// Any calls to functions that cause side effects are wrapped in methods to
/// allow overriding in unit tests.
class NextContext extends Context {
  const NextContext({
    required this.autoAccept,
    required this.force,
    required super.checkouts,
    required super.stateFile,
  });

  final bool autoAccept;
  final bool force;

  Future<void> run(pb.ConductorState state) async {
    const List<CherrypickState> finishedStates = <CherrypickState>[
      CherrypickState.COMPLETED,
      CherrypickState.ABANDONED,
    ];
    switch (state.currentPhase) {
      case pb.ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS:
        final Remote upstream = Remote.upstream(state.framework.upstream.url);
        final FrameworkRepository framework = FrameworkRepository(
          checkouts,
          initialRef: state.framework.workingBranch,
          upstreamRemote: upstream,
          previousCheckoutLocation: state.framework.checkoutPath,
        );
        stdio.printStatus('Writing candidate branch...');
        final bool needsCommit = await framework.updateCandidateBranchVersion(
          state.framework.candidateBranch,
        );
        if (needsCommit) {
          final String revision = await framework.commit(
            'Create candidate branch version ${state.framework.candidateBranch} for ${state.releaseChannel}',
            addFirst: true,
          );
          // append to list of cherrypicks so we know a PR is required
          state.framework.cherrypicks.add(
            pb.Cherrypick.create()
              ..appliedRevision = revision
              ..state = pb.CherrypickState.COMPLETED,
          );
        }

        final List<pb.Cherrypick> unappliedCherrypicks = <pb.Cherrypick>[
          for (final pb.Cherrypick cherrypick in state.framework.cherrypicks)
            if (!finishedStates.contains(cherrypick.state)) cherrypick,
        ];

        if (state.framework.cherrypicks.isEmpty) {
          stdio.printStatus(
            'This release has no framework cherrypicks. However, a framework PR is still\n'
            'required to roll engine cherrypicks.',
          );
        } else if (unappliedCherrypicks.isEmpty) {
          stdio.printStatus('All framework cherrypicks were auto-applied by the conductor.');
        } else {
          if (unappliedCherrypicks.length == 1) {
            stdio.printStatus(
              'There was ${unappliedCherrypicks.length} cherrypick that was not auto-applied.',
            );
          } else {
            stdio.printStatus(
              'There were ${unappliedCherrypicks.length} cherrypicks that were not auto-applied.',
            );
          }
          stdio.printStatus(
            'These must be applied manually in the directory '
            '${state.framework.checkoutPath} before proceeding.\n',
          );
        }

        if (!autoAccept) {
          final bool response = await prompt(
            'Are you ready to push your framework branch to the repository '
            '${state.framework.mirror.url}?',
          );
          if (!response) {
            stdio.printError('Aborting command.');
            updateState(state, stdio.logs);
            return;
          }
        }

        await pushWorkingBranch(framework, state.framework);
      case pb.ReleasePhase.UPDATE_ENGINE_VERSION:
        final Remote upstream = Remote.upstream(state.framework.upstream.url);
        final FrameworkRepository framework = FrameworkRepository(
          checkouts,
          initialRef: state.framework.workingBranch,
          upstreamRemote: upstream,
          previousCheckoutLocation: state.framework.checkoutPath,
        );
        final String rev = await framework.reverseParse('HEAD');
        final File engineVersionFile = (await framework.checkoutDirectory)
            .childDirectory('bin')
            .childDirectory('internal')
            .childFile('engine.version');

        engineVersionFile.writeAsStringSync(rev);

        // Must force add since it is gitignored
        await framework.git.run(
          const <String>['add', 'bin/internal/engine.version', '--force'],
          'adding engine.version file',
          workingDirectory: (await framework.checkoutDirectory).path,
        );
        final String revision = await framework.commit(
          'Create engine.version file pointing to $rev',
        );
        // append to list of cherrypicks so we know a PR is required
        state.framework.cherrypicks.add(
          pb.Cherrypick.create()
            ..appliedRevision = revision
            ..state = pb.CherrypickState.COMPLETED,
        );

        if (!autoAccept) {
          final bool response = await prompt(
            'Are you ready to push your framework branch to the repository '
            '${state.framework.mirror.url}?',
          );
          if (!response) {
            stdio.printError('Aborting command.');
            updateState(state, stdio.logs);
            return;
          }
        }

        await pushWorkingBranch(framework, state.framework);
      case pb.ReleasePhase.PUBLISH_VERSION:
        final String command = '''
          tool-proxy-cli --tool_proxy=/abns/dart-eng-tool-proxy/prod-dart-eng-tool-proxy-tool-proxy.annealed-tool-proxy \\
          --block_on_mpa -I flutter_release \\
          :git_branch ${state.framework.candidateBranch} \\
          :release_channel ${state.releaseChannel} \\
          :tag ${state.releaseVersion} \\
          :force false
        ''';
        stdio.printStatus('Please ensure that you have merged your framework PR');
        stdio.printStatus('and post-submit CI has finished successfully.\n');
        stdio.printStatus('Run the following command, and ask a Googler');
        stdio.printStatus('to review the request\n\n$command');
      case pb.ReleasePhase.VERIFY_RELEASE:
        stdio.printStatus(
          'The current status of packaging builds can be seen at:\n'
          '\t$kLuciPackagingConsoleLink',
        );
        if (!autoAccept) {
          final bool response = await prompt(
            'Have all packaging builds finished successfully and post release announcements been completed?',
          );
          if (!response) {
            stdio.printError('Aborting command.');
            updateState(state, stdio.logs);
            return;
          }
        }
      case pb.ReleasePhase.RELEASE_COMPLETED:
        throw ConductorException('This release is finished.');
    }
    final ReleasePhase nextPhase = state_import.getNextPhase(state.currentPhase);
    stdio.printStatus('\nUpdating phase from ${state.currentPhase} to $nextPhase...\n');
    state.currentPhase = nextPhase;
    stdio.printStatus(state_import.phaseInstructions(state));

    updateState(state, stdio.logs);
  }

  /// Push the working branch to the user's mirror.
  ///
  /// [repository] represents the actual Git repository on disk, and is used to
  /// call `git push`, while [pbRepository] represents the user-specified
  /// configuration for the repository, and is used to read the name of the
  /// working branch and the mirror's remote name.
  ///
  /// May throw either a [ConductorException] if the user already has a branch
  /// of the same name on their mirror, or a [GitException] for any other
  /// failures from the underlying git process call.
  @visibleForTesting
  Future<void> pushWorkingBranch(Repository repository, pb.Repository pbRepository) async {
    try {
      await repository.pushRef(
        fromRef: 'HEAD',
        // Explicitly create new branch
        toRef: 'refs/heads/${pbRepository.workingBranch}',
        remote: pbRepository.mirror.name,
        force: force,
      );
    } on GitException catch (exception) {
      if (exception.type == GitExceptionType.PushRejected && !force) {
        throw ConductorException(
          'Push failed because the working branch named '
          '${pbRepository.workingBranch} already exists on your mirror. '
          'Re-run this command with --force to overwrite the remote branch.\n'
          '${exception.message}',
        );
      }
      rethrow;
    }
  }
}
