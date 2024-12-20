// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show jsonDecode;

import 'package:args/command_runner.dart';
import 'package:conductor_core/src/proto/conductor_state.pb.dart' as pb;
import 'package:conductor_core/src/proto/conductor_state.pbenum.dart';
import 'package:conductor_core/src/repository.dart';
import 'package:conductor_core/src/start.dart';
import 'package:conductor_core/src/state.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:platform/platform.dart';

import './common.dart';

void main() {
  group(
    'start command',
    () {
      const String branchPointRevision = '5131a6e5e0c50b8b7b2906cd58dab8746d6450be';
      const String flutterRoot = '/flutter';
      const String checkoutsParentDirectory = '$flutterRoot/dev/tools/';
      const String githubUsername = 'user';
      const String frameworkMirror = 'git@github.com:$githubUsername/flutter.git';
      const String engineMirror = 'git@github.com:$githubUsername/engine.git';
      const String candidateBranch = 'flutter-1.2-candidate.3';
      const String releaseChannel = 'beta';
      const String revision = 'abcd1234';
      const String conductorVersion = 'deadbeef';
      late Checkouts checkouts;
      late MemoryFileSystem fileSystem;
      late FakePlatform platform;
      late TestStdio stdio;
      late FakeProcessManager processManager;

      setUp(() {
        stdio = TestStdio();
        fileSystem = MemoryFileSystem.test();
      });

      CommandRunner<void> createRunner({
        Map<String, String>? environment,
        String? operatingSystem,
        List<FakeCommand>? commands,
      }) {
        operatingSystem ??= const LocalPlatform().operatingSystem;
        final String pathSeparator = operatingSystem == 'windows' ? r'\' : '/';
        environment ??= <String, String>{'HOME': '/path/to/user/home'};
        final Directory homeDir = fileSystem.directory(environment['HOME']);
        // Tool assumes this exists
        homeDir.createSync(recursive: true);
        platform = FakePlatform(
          environment: environment,
          operatingSystem: operatingSystem,
          pathSeparator: pathSeparator,
        );
        processManager = FakeProcessManager.list(commands ?? <FakeCommand>[]);
        checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );
        final StartCommand command = StartCommand(
          checkouts: checkouts,
          conductorVersion: conductorVersion,
        );
        return CommandRunner<void>('codesign-test', '')..addCommand(command);
      }

      group('start arg parser', () {
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        late StartCommand startCommand;
        setUp(() {
          final String operatingSystem = const LocalPlatform().operatingSystem;
          final Map<String, String> environment = <String, String>{'HOME': '/path/to/user/home'};
          final Directory homeDir = fileSystem.directory(environment['HOME']);
          // Tool assumes this exists
          homeDir.createSync(recursive: true);
          platform = FakePlatform(
            environment: environment,
            operatingSystem: operatingSystem,
            pathSeparator: '/',
          );
          processManager = FakeProcessManager.list(<FakeCommand>[]);
          checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          startCommand = StartCommand(checkouts: checkouts, conductorVersion: conductorVersion);
        });

        test('default args', () async {
          final List<String> args = <String>[
            'start',
            '--$kCandidateOption',
            candidateBranch,
            '--$kReleaseOption',
            'stable',
            '--$kStateOption',
            '/path/to/statefile.json',
            '--$kDartRevisionOption',
            nextDartRevision,
            '--$kGithubUsernameOption',
            githubUsername,
          ];
          final StartContext context = startCommand.createContext(
            startCommand.argParser.parse(args),
          );
          expect(context.frameworkUpstream, FrameworkRepository.defaultUpstream);
          expect(context.frameworkMirror, contains(githubUsername));
          expect(context.frameworkMirror, contains('/flutter.git'));
          expect(context.engineUpstream, EngineRepository.defaultUpstream);
        });

        test('overridden mirror', () async {
          const String customFrameworkMirror = 'git@github.com:$githubUsername/flutter-work.git';
          final List<String> args = <String>[
            'start',
            '--$kCandidateOption',
            candidateBranch,
            '--$kReleaseOption',
            'stable',
            '--$kStateOption',
            '/path/to/statefile.json',
            '--$kDartRevisionOption',
            nextDartRevision,
            '--$kGithubUsernameOption',
            githubUsername,
            '--$kFrameworkMirrorOption',
            customFrameworkMirror,
          ];
          final StartContext context = startCommand.createContext(
            startCommand.argParser.parse(args),
          );
          expect(context.frameworkMirror, customFrameworkMirror);
        });
      });

      test('throws exception if run from Windows', () async {
        final CommandRunner<void> runner = createRunner(
          commands: <FakeCommand>[
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision),
          ],
          operatingSystem: 'windows',
        );
        await expectLater(
          () async => runner.run(<String>[
            'start',
            '--$kCandidateOption',
            candidateBranch,
            '--$kReleaseOption',
            'beta',
            '--$kStateOption',
            '/path/to/statefile.json',
          ]),
          throwsExceptionWith('Error! This tool is only supported on macOS and Linux'),
        );
      });

      test('throws if provided an invalid --$kVersionOverrideOption', () async {
        final CommandRunner<void> runner = createRunner();

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );

        await expectLater(
          () async => runner.run(<String>[
            'start',
            '--$kCandidateOption',
            candidateBranch,
            '--$kReleaseOption',
            releaseChannel,
            '--$kStateOption',
            stateFilePath,
            '--$kVersionOverrideOption',
            'an invalid version string',
            '--$kGithubUsernameOption',
            githubUsername,
          ]),
          throwsExceptionWith('an invalid version string cannot be parsed'),
        );
      });

      test('creates state file if provided correct inputs', () async {
        stdio.stdin.add('y'); // accept prompt from ensureBranchPointTagged()
        const String revision2 = 'def789';
        const String revision3 = '123abc';
        const String previousDartRevision = '171876a4e6cf56ee6da1f97d203926bd7afda7ef';
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        const String previousVersion = '1.2.0-1.0.pre';
        // This is what this release will be
        const String nextVersion = '1.2.0-1.1.pre';
        const String candidateBranch = 'flutter-1.2-candidate.1';

        final Directory engine = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('engine');

        final File depsFile = engine.childFile('DEPS');

        final List<FakeCommand> engineCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              EngineRepository.defaultUpstream,
              engine.path,
            ],
            onRun: (_) {
              // Create the DEPS file which the tool will update
              engine.createSync(recursive: true);
              depsFile.writeAsStringSync(generateMockDeps(previousDartRevision));
            },
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', engineMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>['git', 'status', '--porcelain'],
            stdout: 'MM path/to/DEPS',
          ),
          const FakeCommand(command: <String>['git', 'add', '--all']),
          const FakeCommand(
            command: <String>['git', 'commit', '--message', 'Update Dart SDK to $nextDartRevision'],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
        ];

        final List<FakeCommand> frameworkCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              FrameworkRepository.defaultUpstream,
              fileSystem.path.join(
                checkoutsParentDirectory,
                'flutter_conductor_checkouts',
                'framework',
              ),
            ],
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', frameworkMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>[
              'git',
              'describe',
              '--match',
              '*.*.*',
              '--tags',
              'refs/remotes/upstream/$candidateBranch',
            ],
            stdout: '$previousVersion-42-gabc123',
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'merge-base', 'upstream/$candidateBranch', 'upstream/master'],
            stdout: branchPointRevision,
          ),
          // check if commit is tagged, zero exit code means it is tagged
          const FakeCommand(
            command: <String>['git', 'describe', '--exact-match', '--tags', branchPointRevision],
          ),
        ];

        final CommandRunner<void> runner = createRunner(
          commands: <FakeCommand>[...engineCommands, ...frameworkCommands],
        );

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );

        await runner.run(<String>[
          'start',
          '--$kCandidateOption',
          candidateBranch,
          '--$kReleaseOption',
          releaseChannel,
          '--$kStateOption',
          stateFilePath,
          '--$kDartRevisionOption',
          nextDartRevision,
          '--$kGithubUsernameOption',
          githubUsername,
        ]);

        final File stateFile = fileSystem.file(stateFilePath);

        final pb.ConductorState state = pb.ConductorState();
        state.mergeFromProto3Json(jsonDecode(stateFile.readAsStringSync()));

        expect(state.releaseType, ReleaseType.BETA_HOTFIX);
        expect(
          stdio.error,
          isNot(contains('Tried to tag the branch point, however the target version')),
        );
        expect(processManager, hasNoRemainingExpectations);
        expect(state.isInitialized(), true);
        expect(state.releaseChannel, releaseChannel);
        expect(state.releaseVersion, nextVersion);
        expect(state.engine.candidateBranch, candidateBranch);
        expect(state.engine.startingGitHead, revision2);
        expect(state.engine.dartRevision, nextDartRevision);
        expect(state.engine.upstream.url, 'git@github.com:flutter/engine.git');
        expect(state.framework.candidateBranch, candidateBranch);
        expect(state.framework.startingGitHead, revision3);
        expect(state.framework.upstream.url, 'git@github.com:flutter/flutter.git');
        expect(state.currentPhase, ReleasePhase.APPLY_ENGINE_CHERRYPICKS);
        expect(state.conductorVersion, conductorVersion);
      });

      test('uses --$kVersionOverrideOption', () async {
        stdio.stdin.add('y'); // accept prompt from ensureBranchPointTagged()
        const String revision2 = 'def789';
        const String revision3 = '123abc';
        const String previousDartRevision = '171876a4e6cf56ee6da1f97d203926bd7afda7ef';
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        const String previousVersion = '1.2.0-1.0.pre';
        const String candidateBranch = 'flutter-1.2-candidate.1';
        const String versionOverride = '42.0.0-42.0.pre';

        final Directory engine = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('engine');

        final File depsFile = engine.childFile('DEPS');

        final List<FakeCommand> engineCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              EngineRepository.defaultUpstream,
              engine.path,
            ],
            onRun: (_) {
              // Create the DEPS file which the tool will update
              engine.createSync(recursive: true);
              depsFile.writeAsStringSync(generateMockDeps(previousDartRevision));
            },
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', engineMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>['git', 'status', '--porcelain'],
            stdout: 'MM path/to/DEPS',
          ),
          const FakeCommand(command: <String>['git', 'add', '--all']),
          const FakeCommand(
            command: <String>['git', 'commit', '--message', 'Update Dart SDK to $nextDartRevision'],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
        ];

        final List<FakeCommand> frameworkCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              FrameworkRepository.defaultUpstream,
              fileSystem.path.join(
                checkoutsParentDirectory,
                'flutter_conductor_checkouts',
                'framework',
              ),
            ],
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', frameworkMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>[
              'git',
              'describe',
              '--match',
              '*.*.*',
              '--tags',
              'refs/remotes/upstream/$candidateBranch',
            ],
            stdout: '$previousVersion-42-gabc123',
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'merge-base', 'upstream/$candidateBranch', 'upstream/master'],
            stdout: branchPointRevision,
          ),
        ];

        final CommandRunner<void> runner = createRunner(
          commands: <FakeCommand>[...engineCommands, ...frameworkCommands],
        );

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );

        await runner.run(<String>[
          'start',
          '--$kCandidateOption',
          candidateBranch,
          '--$kReleaseOption',
          releaseChannel,
          '--$kStateOption',
          stateFilePath,
          '--$kDartRevisionOption',
          nextDartRevision,
          '--$kVersionOverrideOption',
          versionOverride,
          '--$kGithubUsernameOption',
          githubUsername,
        ]);

        final File stateFile = fileSystem.file(stateFilePath);

        final pb.ConductorState state = pb.ConductorState();
        state.mergeFromProto3Json(jsonDecode(stateFile.readAsStringSync()));

        expect(processManager, hasNoRemainingExpectations);
        expect(state.releaseVersion, versionOverride);
      });

      test('logs to STDERR but does not fail on an unexpected candidate branch', () async {
        stdio.stdin.add('y'); // accept prompt from ensureBranchPointTagged()
        const String revision2 = 'def789';
        const String revision3 = '123abc';
        const String previousDartRevision = '171876a4e6cf56ee6da1f97d203926bd7afda7ef';
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        // This is significantly behind the candidate branch name
        const String previousVersion = '0.9.0-1.0.pre';
        // This is what this release will be
        const String nextVersion = '0.9.0-1.1.pre';

        final Directory engine = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('engine');

        final File depsFile = engine.childFile('DEPS');

        final List<FakeCommand> engineCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              EngineRepository.defaultUpstream,
              engine.path,
            ],
            onRun: (_) {
              // Create the DEPS file which the tool will update
              engine.createSync(recursive: true);
              depsFile.writeAsStringSync(generateMockDeps(previousDartRevision));
            },
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', engineMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>['git', 'status', '--porcelain'],
            stdout: 'MM path/to/DEPS',
          ),
          const FakeCommand(command: <String>['git', 'add', '--all']),
          const FakeCommand(
            command: <String>['git', 'commit', '--message', 'Update Dart SDK to $nextDartRevision'],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
        ];

        final List<FakeCommand> frameworkCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              FrameworkRepository.defaultUpstream,
              fileSystem.path.join(
                checkoutsParentDirectory,
                'flutter_conductor_checkouts',
                'framework',
              ),
            ],
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', frameworkMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>[
              'git',
              'describe',
              '--match',
              '*.*.*',
              '--tags',
              'refs/remotes/upstream/$candidateBranch',
            ],
            stdout: '$previousVersion-42-gabc123',
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'merge-base', 'upstream/$candidateBranch', 'upstream/master'],
            stdout: branchPointRevision,
          ),
          // check if commit is tagged, 0 exit code means it is tagged
          const FakeCommand(
            command: <String>['git', 'describe', '--exact-match', '--tags', branchPointRevision],
          ),
        ];

        final CommandRunner<void> runner = createRunner(
          commands: <FakeCommand>[...engineCommands, ...frameworkCommands],
        );

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );

        await runner.run(<String>[
          'start',
          '--$kCandidateOption',
          candidateBranch,
          '--$kReleaseOption',
          releaseChannel,
          '--$kStateOption',
          stateFilePath,
          '--$kDartRevisionOption',
          nextDartRevision,
          '--$kGithubUsernameOption',
          githubUsername,
        ]);

        final File stateFile = fileSystem.file(stateFilePath);

        final pb.ConductorState state = pb.ConductorState();
        state.mergeFromProto3Json(jsonDecode(stateFile.readAsStringSync()));

        expect(stdio.error, isNot(contains('Tried to tag the branch point, however')));
        expect(processManager, hasNoRemainingExpectations);
        expect(state.isInitialized(), true);
        expect(state.releaseChannel, releaseChannel);
        expect(state.releaseVersion, nextVersion);
        expect(state.engine.candidateBranch, candidateBranch);
        expect(state.engine.startingGitHead, revision2);
        expect(state.engine.dartRevision, nextDartRevision);
        expect(state.engine.upstream.url, 'git@github.com:flutter/engine.git');
        expect(state.framework.candidateBranch, candidateBranch);
        expect(state.framework.startingGitHead, revision3);
        expect(state.framework.upstream.url, 'git@github.com:flutter/flutter.git');
        expect(state.currentPhase, ReleasePhase.APPLY_ENGINE_CHERRYPICKS);
        expect(state.conductorVersion, conductorVersion);
        expect(state.releaseType, ReleaseType.BETA_HOTFIX);
        expect(
          stdio.error,
          contains(
            'Parsed version $previousVersion.42 has a different x value than candidate branch $candidateBranch',
          ),
        );
      });

      test('can convert from dev style version to stable version', () async {
        const String revision2 = 'def789';
        const String revision3 = '123abc';
        const String previousDartRevision = '171876a4e6cf56ee6da1f97d203926bd7afda7ef';
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        const String previousVersion = '1.2.0-3.0.pre';
        const String nextVersion = '1.2.0';

        final Directory engine = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('engine');

        final File depsFile = engine.childFile('DEPS');

        final List<FakeCommand> engineCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              EngineRepository.defaultUpstream,
              engine.path,
            ],
            onRun: (_) {
              // Create the DEPS file which the tool will update
              engine.createSync(recursive: true);
              depsFile.writeAsStringSync(generateMockDeps(previousDartRevision));
            },
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', engineMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>['git', 'status', '--porcelain'],
            stdout: 'MM path/to/DEPS',
          ),
          const FakeCommand(command: <String>['git', 'add', '--all']),
          const FakeCommand(
            command: <String>['git', 'commit', '--message', 'Update Dart SDK to $nextDartRevision'],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
        ];

        final List<FakeCommand> frameworkCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              FrameworkRepository.defaultUpstream,
              fileSystem.path.join(
                checkoutsParentDirectory,
                'flutter_conductor_checkouts',
                'framework',
              ),
            ],
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', frameworkMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>[
              'git',
              'describe',
              '--match',
              '*.*.*',
              '--tags',
              'refs/remotes/upstream/$candidateBranch',
            ],
            stdout: '$previousVersion-42-gabc123',
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'merge-base', 'upstream/$candidateBranch', 'upstream/master'],
            stdout: branchPointRevision,
          ),
          // check if commit is tagged, 0 exit code thus it is tagged
          const FakeCommand(
            command: <String>['git', 'describe', '--exact-match', '--tags', branchPointRevision],
          ),
        ];

        final CommandRunner<void> runner = createRunner(
          commands: <FakeCommand>[...engineCommands, ...frameworkCommands],
        );

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );

        await runner.run(<String>[
          'start',
          '--$kCandidateOption',
          candidateBranch,
          '--$kReleaseOption',
          'stable',
          '--$kStateOption',
          stateFilePath,
          '--$kDartRevisionOption',
          nextDartRevision,
          '--$kGithubUsernameOption',
          githubUsername,
        ]);

        final File stateFile = fileSystem.file(stateFilePath);

        final pb.ConductorState state = pb.ConductorState();
        state.mergeFromProto3Json(jsonDecode(stateFile.readAsStringSync()));

        expect(processManager.hasRemainingExpectations, false);
        expect(state.isInitialized(), true);
        expect(state.releaseChannel, 'stable');
        expect(state.releaseVersion, nextVersion);
        expect(state.engine.candidateBranch, candidateBranch);
        expect(state.engine.startingGitHead, revision2);
        expect(state.engine.dartRevision, nextDartRevision);
        expect(state.framework.candidateBranch, candidateBranch);
        expect(state.framework.startingGitHead, revision3);
        expect(state.currentPhase, ReleasePhase.APPLY_ENGINE_CHERRYPICKS);
        expect(state.conductorVersion, conductorVersion);
        expect(state.releaseType, ReleaseType.STABLE_INITIAL);
      });
      test('StartContext gets engine and framework checkout directories after run', () async {
        stdio.stdin.add('y');
        const String revision2 = 'def789';
        const String revision3 = '123abc';
        const String branchPointRevision = 'deadbeef';
        const String previousDartRevision = '171876a4e6cf56ee6da1f97d203926bd7afda7ef';
        const String nextDartRevision = 'f6c91128be6b77aef8351e1e3a9d07c85bc2e46e';
        const String previousVersion = '1.2.0-1.0.pre';
        // This is a git tag applied to the branch point, not an actual release
        const String branchPointTag = '1.2.0-3.0.pre';

        final Directory engine = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('engine');

        final Directory framework = fileSystem
            .directory(checkoutsParentDirectory)
            .childDirectory('flutter_conductor_checkouts')
            .childDirectory('framework');

        final File depsFile = engine.childFile('DEPS');

        final List<FakeCommand> engineCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              EngineRepository.defaultUpstream,
              engine.path,
            ],
            onRun: (_) {
              // Create the DEPS file which the tool will update
              engine.createSync(recursive: true);
              depsFile.writeAsStringSync(generateMockDeps(previousDartRevision));
            },
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', engineMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>['git', 'status', '--porcelain'],
            stdout: 'MM path/to/DEPS',
          ),
          const FakeCommand(command: <String>['git', 'add', '--all']),
          const FakeCommand(
            command: <String>['git', 'commit', '--message', 'Update Dart SDK to $nextDartRevision'],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision2),
        ];

        final List<FakeCommand> frameworkCommands = <FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              FrameworkRepository.defaultUpstream,
              framework.path,
            ],
          ),
          const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', frameworkMirror]),
          const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
          const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
          const FakeCommand(
            command: <String>['git', 'checkout', '-b', 'cherrypicks-$candidateBranch'],
          ),
          const FakeCommand(
            command: <String>[
              'git',
              'describe',
              '--match',
              '*.*.*',
              '--tags',
              'refs/remotes/upstream/$candidateBranch',
            ],
            stdout: '$previousVersion-42-gabc123',
          ),
          // HEAD and branch point are same
          const FakeCommand(
            command: <String>['git', 'rev-parse', 'HEAD'],
            stdout: branchPointRevision,
          ),
          const FakeCommand(
            command: <String>['git', 'merge-base', 'upstream/$candidateBranch', 'upstream/master'],
            stdout: branchPointRevision,
          ),
          // check if commit is tagged
          const FakeCommand(
            command: <String>['git', 'describe', '--exact-match', '--tags', branchPointRevision],
            // non-zero exit code means branch point is NOT tagged
            exitCode: 128,
          ),
          const FakeCommand(command: <String>['git', 'tag', branchPointTag, branchPointRevision]),
          const FakeCommand(
            command: <String>['git', 'push', FrameworkRepository.defaultUpstream, branchPointTag],
          ),
        ];

        final String operatingSystem = const LocalPlatform().operatingSystem;
        final Map<String, String> environment = <String, String>{'HOME': '/path/to/user/home'};
        final Directory homeDir = fileSystem.directory(environment['HOME']);
        // Tool assumes this exists
        homeDir.createSync(recursive: true);
        platform = FakePlatform(environment: environment, operatingSystem: operatingSystem);

        final String stateFilePath = fileSystem.path.join(
          platform.environment['HOME']!,
          kStateFileName,
        );
        final File stateFile = fileSystem.file(stateFilePath);

        processManager = FakeProcessManager.list(<FakeCommand>[
          ...engineCommands,
          ...frameworkCommands,
        ]);
        checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );

        final StartContext startContext = StartContext(
          candidateBranch: candidateBranch,
          checkouts: checkouts,
          dartRevision: nextDartRevision,
          engineMirror: engineMirror,
          engineUpstream: EngineRepository.defaultUpstream,
          frameworkMirror: frameworkMirror,
          frameworkUpstream: FrameworkRepository.defaultUpstream,
          releaseChannel: releaseChannel,
          processManager: processManager,
          conductorVersion: conductorVersion,
          githubUsername: githubUsername,
          stateFile: stateFile,
        );

        await startContext.run();

        final pb.ConductorState state = pb.ConductorState();
        state.mergeFromProto3Json(jsonDecode(stateFile.readAsStringSync()));

        expect((await startContext.engine.checkoutDirectory).path, equals(engine.path));
        expect((await startContext.framework.checkoutDirectory).path, equals(framework.path));
        expect(state.releaseType, ReleaseType.BETA_INITIAL);
        expect(processManager, hasNoRemainingExpectations);
      });
    },
    onPlatform: <String, dynamic>{
      'windows': const Skip('Flutter Conductor only supported on macos/linux'),
    },
  );
}

String generateMockDeps(String dartRevision) {
  return '''
vars = {
  'chromium_git': 'https://chromium.googlesource.com',
  'swiftshader_git': 'https://swiftshader.googlesource.com',
  'dart_git': 'https://dart.googlesource.com',
  'flutter_git': 'https://flutter.googlesource.com',
  'fuchsia_git': 'https://fuchsia.googlesource.com',
  'github_git': 'https://github.com',
  'skia_git': 'https://skia.googlesource.com',
  'ocmock_git': 'https://github.com/erikdoe/ocmock.git',
  'skia_revision': '4e9d5e2bdf04c58bc0bff57be7171e469e5d7175',

  'dart_revision': '$dartRevision',
  'dart_boringssl_gen_rev': '7322fc15cc065d8d2957fccce6b62a509dc4d641',
}''';
}
