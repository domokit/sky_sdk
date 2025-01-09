// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:conductor_core/src/git.dart';
import 'package:conductor_core/src/globals.dart';
import 'package:conductor_core/src/next.dart';
import 'package:conductor_core/src/proto/conductor_state.pb.dart' as pb;
import 'package:conductor_core/src/proto/conductor_state.pbenum.dart' show ReleasePhase;
import 'package:conductor_core/src/repository.dart';
import 'package:conductor_core/src/state.dart';
import 'package:conductor_core/src/stdio.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:platform/platform.dart';

import './common.dart';

void main() {
  const String flutterRoot = '/flutter';
  const String checkoutsParentDirectory = '$flutterRoot/dev/conductor';
  const String candidateBranch = 'flutter-1.2-candidate.3';
  const String workingBranch = 'cherrypicks-$candidateBranch';
  const String remoteUrl = 'https://github.com/org/repo.git';
  const String revision1 = 'd3af60d18e01fcb36e0c0fa06c8502e4935ed095';
  const String revision2 = 'f99555c1e1392bf2a8135056b9446680c2af4ddf';
  const String revision3 = 'ffffffffffffffffffffffffffffffffffffffff';
  const String revision4 = '280e23318a0d8341415c66aa32581352a421d974';
  const String releaseVersion = '1.2.0-3.0.pre';
  const String releaseChannel = 'beta';
  const String stateFile = '/state-file.json';
  final String localPathSeparator = const LocalPlatform().pathSeparator;
  final String localOperatingSystem = const LocalPlatform().operatingSystem;

  group(
    'next command',
    () {
      late MemoryFileSystem fileSystem;
      late TestStdio stdio;

      setUp(() {
        stdio = TestStdio();
        fileSystem = MemoryFileSystem.test();
      });

      CommandRunner<void> createRunner({required Checkouts checkouts}) {
        final NextCommand command = NextCommand(checkouts: checkouts);
        return CommandRunner<void>('codesign-test', '')..addCommand(command);
      }

      test('throws if no state file found', () async {
        final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[]);
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{
            'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
          },
          operatingSystem: localOperatingSystem,
          pathSeparator: localPathSeparator,
        );
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory)
            ..createSync(recursive: true),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );
        final CommandRunner<void> runner = createRunner(checkouts: checkouts);
        expect(
          () async => runner.run(<String>['next', '--$kStateOption', stateFile]),
          throwsExceptionWith('No persistent state file found at $stateFile'),
        );
      });

      group('APPLY_ENGINE_CHERRYPICKS to VERIFY_ENGINE_CI', () {
        test('confirms to stdout when all engine cherrypicks were auto-applied', () async {
          stdio.stdin.add('n');
          final File ciYaml = fileSystem.file('$checkoutsParentDirectory/engine/.ci.yaml')
            ..createSync(recursive: true);
          _initializeCiYamlFile(ciYaml);
          final FakeProcessManager processManager = FakeProcessManager.empty();
          final FakePlatform platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
          final pb.ConductorState state =
              (pb.ConductorState.create()
                ..releaseChannel = releaseChannel
                ..engine =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..cherrypicks.add(
                        pb.Cherrypick.create()
                          ..trunkRevision = 'abc123'
                          ..state = pb.CherrypickState.COMPLETED,
                      )
                      ..checkoutPath = fileSystem.path.join(checkoutsParentDirectory, 'engine')
                      ..workingBranch = workingBranch
                      ..upstream =
                          (pb.Remote.create()
                            ..name = 'upstream'
                            ..url = remoteUrl)
                      ..mirror =
                          (pb.Remote.create()
                            ..name = 'mirror'
                            ..url = remoteUrl))
                ..currentPhase = ReleasePhase.APPLY_ENGINE_CHERRYPICKS);
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          expect(processManager, hasNoRemainingExpectations);
          expect(
            stdio.stdout,
            contains('All engine cherrypicks have been auto-applied by the conductor'),
          );
        });

        test('updates lastPhase if user responds yes', () async {
          const String remoteUrl = 'https://github.com/org/repo.git';
          const String releaseChannel = 'beta';
          stdio.stdin.add('y');
          final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
            const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
            FakeCommand(
              command: const <String>['git', 'checkout', workingBranch],
              onRun: (_) {
                final File file = fileSystem.file('$checkoutsParentDirectory/engine/.ci.yaml')
                  ..createSync(recursive: true);
                _initializeCiYamlFile(file);
              },
            ),
            const FakeCommand(
              command: <String>['git', 'push', 'mirror', 'HEAD:refs/heads/$workingBranch'],
            ),
          ]);
          final FakePlatform platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
          final pb.ConductorState state =
              (pb.ConductorState.create()
                ..currentPhase = ReleasePhase.APPLY_ENGINE_CHERRYPICKS
                ..engine =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..checkoutPath = fileSystem.path.join(checkoutsParentDirectory, 'engine')
                      ..cherrypicks.add(
                        pb.Cherrypick.create()
                          ..trunkRevision = revision2
                          ..state = pb.CherrypickState.PENDING,
                      )
                      ..workingBranch = workingBranch
                      ..upstream =
                          (pb.Remote.create()
                            ..name = 'upstream'
                            ..url = remoteUrl)
                      ..mirror =
                          (pb.Remote.create()
                            ..name = 'mirror'
                            ..url = remoteUrl))
                ..releaseChannel = releaseChannel
                ..releaseVersion = releaseVersion);
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          // engine dir is expected to already exist
          fileSystem
              .directory(checkoutsParentDirectory)
              .childDirectory('engine')
              .createSync(recursive: true);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(processManager, hasNoRemainingExpectations);
          expect(
            stdio.stdout,
            contains(
              'You must now open a pull request at https://github.com/flutter/engine/compare/flutter-1.2-candidate.3...org:cherrypicks-flutter-1.2-candidate.3?expand=1',
            ),
          );
          expect(
            stdio.stdout,
            contains(
              'Are you ready to push your engine branch to the repository $remoteUrl? (y/n) ',
            ),
          );
          expect(finalState.currentPhase, ReleasePhase.VERIFY_ENGINE_CI);
          expect(stdio.error, isEmpty);
        });
      });

      group('VERIFY_ENGINE_CI to APPLY_FRAMEWORK_CHERRYPICKS', () {
        late pb.ConductorState state;
        late FakeProcessManager processManager;
        late FakePlatform platform;

        setUp(() {
          state =
              (pb.ConductorState.create()
                ..releaseChannel = releaseChannel
                ..engine =
                    (pb.Repository.create()
                      ..cherrypicks.add(
                        pb.Cherrypick.create()
                          ..trunkRevision = 'abc123'
                          ..state = pb.CherrypickState.PENDING,
                      )
                      ..candidateBranch = 'flutter-1.0-candidate.0')
                ..currentPhase = ReleasePhase.VERIFY_ENGINE_CI);

          processManager = FakeProcessManager.empty();

          platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
        });

        test('does not update currentPhase if user responds no', () async {
          stdio.stdin.add('n');
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(processManager, hasNoRemainingExpectations);
          expect(stdio.stdout, contains('Has CI passed for the engine PR?'));
          expect(finalState.currentPhase, ReleasePhase.VERIFY_ENGINE_CI);
          expect(stdio.error.contains('Aborting command.'), true);
        });

        test('updates currentPhase if user responds yes', () async {
          stdio.stdin.add('y');
          final FakePlatform platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(processManager, hasNoRemainingExpectations);
          expect(stdio.stdout, contains('Has CI passed for the engine PR?'));
          expect(finalState.currentPhase, ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS);
        });
      });

      group('APPLY_FRAMEWORK_CHERRYPICKS to PUBLISH_VERSION', () {
        const String mirrorRemoteUrl = 'https://github.com/org/repo.git';
        const String upstreamRemoteUrl = 'https://github.com/mirror/repo.git';
        const String engineUpstreamRemoteUrl = 'https://github.com/mirror/engine.git';
        const String frameworkCheckoutPath = '$checkoutsParentDirectory/framework';
        const String engineCheckoutPath = '$checkoutsParentDirectory/engine';
        const String oldEngineVersion = '000000001';
        const String frameworkCherrypick = '431ae69b4dd2dd48f7ba0153671e0311014c958b';
        late FakeProcessManager processManager;
        late FakePlatform platform;
        late pb.ConductorState state;

        setUp(() {
          processManager = FakeProcessManager.empty();
          platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
          state =
              (pb.ConductorState.create()
                ..releaseChannel = releaseChannel
                ..releaseVersion = releaseVersion
                ..framework =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..checkoutPath = frameworkCheckoutPath
                      ..cherrypicks.add(
                        pb.Cherrypick.create()
                          ..trunkRevision = frameworkCherrypick
                          ..state = pb.CherrypickState.PENDING,
                      )
                      ..mirror =
                          (pb.Remote.create()
                            ..name = 'mirror'
                            ..url = mirrorRemoteUrl)
                      ..upstream =
                          (pb.Remote.create()
                            ..name = 'upstream'
                            ..url = upstreamRemoteUrl)
                      ..workingBranch = workingBranch)
                ..engine =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..checkoutPath = engineCheckoutPath
                      ..dartRevision = 'cdef0123'
                      ..workingBranch = workingBranch
                      ..upstream =
                          (pb.Remote.create()
                            ..name = 'upstream'
                            ..url = engineUpstreamRemoteUrl))
                ..currentPhase = ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS);
          // create engine repo
          fileSystem.directory(engineCheckoutPath).createSync(recursive: true);
          // create framework repo
          final Directory frameworkDir = fileSystem.directory(frameworkCheckoutPath);
          final File engineRevisionFile = frameworkDir
              .childDirectory('bin')
              .childDirectory('internal')
              .childFile('engine.version');
          engineRevisionFile.createSync(recursive: true);
          engineRevisionFile.writeAsStringSync(oldEngineVersion, flush: true);
        });

        test(
          'with no dart, engine or framework cherrypicks, updates engine revision if version mismatch',
          () async {
            stdio.stdin.add('n');
            processManager.addCommands(<FakeCommand>[
              const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
              // we want merged upstream commit, not local working commit
              const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision1),
              const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
              FakeCommand(
                command: const <String>['git', 'checkout', workingBranch],
                onRun: (_) {
                  final File file = fileSystem.file('$checkoutsParentDirectory/framework/.ci.yaml')
                    ..createSync();
                  _initializeCiYamlFile(file);
                },
              ),
              const FakeCommand(
                command: <String>['git', 'status', '--porcelain'],
                stdout: 'MM bin/internal/release-candidate-branch.version',
              ),
              const FakeCommand(command: <String>['git', 'add', '--all']),
              const FakeCommand(
                command: <String>[
                  'git',
                  'commit',
                  '--message',
                  'Create candidate branch version $candidateBranch for $releaseChannel',
                ],
              ),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
              const FakeCommand(
                command: <String>['git', 'status', '--porcelain'],
                stdout: 'MM bin/internal/engine.version',
              ),
              const FakeCommand(command: <String>['git', 'add', '--all']),
              const FakeCommand(
                command: <String>[
                  'git',
                  'commit',
                  '--message',
                  'Update Engine revision to $revision1 for $releaseChannel release $releaseVersion',
                ],
              ),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision4),
            ]);
            final pb.ConductorState state =
                (pb.ConductorState.create()
                  ..releaseChannel = releaseChannel
                  ..releaseVersion = releaseVersion
                  ..currentPhase = ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS
                  ..framework =
                      (pb.Repository.create()
                        ..candidateBranch = candidateBranch
                        ..checkoutPath = frameworkCheckoutPath
                        ..mirror =
                            (pb.Remote.create()
                              ..name = 'mirror'
                              ..url = mirrorRemoteUrl)
                        ..upstream =
                            (pb.Remote.create()
                              ..name = 'upstream'
                              ..url = upstreamRemoteUrl)
                        ..workingBranch = workingBranch)
                  ..engine =
                      (pb.Repository.create()
                        ..candidateBranch = candidateBranch
                        ..checkoutPath = engineCheckoutPath
                        ..upstream =
                            (pb.Remote.create()
                              ..name = 'upstream'
                              ..url = engineUpstreamRemoteUrl)
                        ..currentGitHead = revision1));
            writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
            final Checkouts checkouts = Checkouts(
              fileSystem: fileSystem,
              parentDirectory: fileSystem.directory(checkoutsParentDirectory)
                ..createSync(recursive: true),
              platform: platform,
              processManager: processManager,
              stdio: stdio,
            );
            final CommandRunner<void> runner = createRunner(checkouts: checkouts);
            await runner.run(<String>['next', '--$kStateOption', stateFile]);

            expect(processManager, hasNoRemainingExpectations);
            expect(
              stdio.stdout,
              contains('release-candidate-branch.version containing $candidateBranch'),
            );
            expect(
              stdio.stdout,
              contains('Updating engine revision from $oldEngineVersion to $revision1'),
            );
            expect(stdio.stdout, contains('Are you ready to push your framework branch'));
          },
        );

        test(
          'with no engine cherrypicks but a dart revision update, updates engine revision',
          () async {
            stdio.stdin.add('n');
            processManager.addCommands(<FakeCommand>[
              const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
              // we want merged upstream commit, not local working commit
              const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision1),
              const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
              FakeCommand(
                command: const <String>['git', 'checkout', workingBranch],
                onRun: (_) {
                  final File file = fileSystem.file('$checkoutsParentDirectory/framework/.ci.yaml')
                    ..createSync();
                  _initializeCiYamlFile(file);
                },
              ),
              const FakeCommand(
                command: <String>['git', 'status', '--porcelain'],
                stdout: 'MM bin/internal/release-candidate-branch.version',
              ),
              const FakeCommand(command: <String>['git', 'add', '--all']),
              const FakeCommand(
                command: <String>[
                  'git',
                  'commit',
                  '--message',
                  'Create candidate branch version $candidateBranch for $releaseChannel',
                ],
              ),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
              const FakeCommand(
                command: <String>['git', 'status', '--porcelain'],
                stdout: 'MM bin/internal/engine.version',
              ),
              const FakeCommand(command: <String>['git', 'add', '--all']),
              const FakeCommand(
                command: <String>[
                  'git',
                  'commit',
                  '--message',
                  'Update Engine revision to $revision1 for $releaseChannel release $releaseVersion',
                ],
              ),
              const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision4),
            ]);
            final pb.ConductorState state =
                (pb.ConductorState.create()
                  ..releaseChannel = releaseChannel
                  ..releaseVersion = releaseVersion
                  ..currentPhase = ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS
                  ..framework =
                      (pb.Repository.create()
                        ..candidateBranch = candidateBranch
                        ..checkoutPath = frameworkCheckoutPath
                        ..mirror =
                            (pb.Remote.create()
                              ..name = 'mirror'
                              ..url = mirrorRemoteUrl)
                        ..upstream =
                            (pb.Remote.create()
                              ..name = 'upstream'
                              ..url = upstreamRemoteUrl)
                        ..workingBranch = workingBranch)
                  ..engine =
                      (pb.Repository.create()
                        ..candidateBranch = candidateBranch
                        ..checkoutPath = engineCheckoutPath
                        ..upstream =
                            (pb.Remote.create()
                              ..name = 'upstream'
                              ..url = engineUpstreamRemoteUrl)
                        ..dartRevision = 'abc123'));
            writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
            final Checkouts checkouts = Checkouts(
              fileSystem: fileSystem,
              parentDirectory: fileSystem.directory(checkoutsParentDirectory)
                ..createSync(recursive: true),
              platform: platform,
              processManager: processManager,
              stdio: stdio,
            );
            final CommandRunner<void> runner = createRunner(checkouts: checkouts);
            await runner.run(<String>['next', '--$kStateOption', stateFile]);

            expect(processManager, hasNoRemainingExpectations);
            expect(
              stdio.stdout,
              contains('release-candidate-branch.version containing $candidateBranch'),
            );
            expect(
              stdio.stdout,
              contains('Updating engine revision from $oldEngineVersion to $revision1'),
            );
            expect(stdio.stdout, contains('Are you ready to push your framework branch'));
          },
        );

        test('does not update state.currentPhase if user responds no', () async {
          stdio.stdin.add('n');
          processManager.addCommands(<FakeCommand>[
            const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
            // we want merged upstream commit, not local working commit
            FakeCommand(
              command: const <String>['git', 'checkout', 'upstream/$candidateBranch'],
              onRun: (_) {
                final File file = fileSystem.file('$checkoutsParentDirectory/framework/.ci.yaml')
                  ..createSync();
                _initializeCiYamlFile(file);
              },
            ),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision1),
            const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
            const FakeCommand(command: <String>['git', 'checkout', workingBranch]),
            const FakeCommand(
              command: <String>['git', 'status', '--porcelain'],
              stdout: 'MM bin/internal/release-candidate-branch.version',
            ),
            const FakeCommand(command: <String>['git', 'add', '--all']),
            const FakeCommand(
              command: <String>[
                'git',
                'commit',
                '--message',
                'Create candidate branch version $candidateBranch for $releaseChannel',
              ],
            ),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
            const FakeCommand(
              command: <String>['git', 'status', '--porcelain'],
              stdout: 'MM bin/internal/engine.version',
            ),
            const FakeCommand(command: <String>['git', 'add', '--all']),
            const FakeCommand(
              command: <String>[
                'git',
                'commit',
                '--message',
                'Update Engine revision to $revision1 for $releaseChannel release $releaseVersion',
              ],
            ),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision4),
          ]);
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(
            stdio.stdout,
            contains(
              'Are you ready to push your framework branch to the repository $mirrorRemoteUrl? (y/n) ',
            ),
          );
          expect(stdio.error, contains('Aborting command.'));
          expect(finalState.currentPhase, ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS);
        });

        test('updates state.currentPhase if user responds yes', () async {
          stdio.stdin.add('y');
          processManager.addCommands(<FakeCommand>[
            // Engine repo
            const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
            // we want merged upstream commit, not local working commit
            const FakeCommand(command: <String>['git', 'checkout', 'upstream/$candidateBranch']),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision1),
            // Framework repo
            const FakeCommand(command: <String>['git', 'fetch', 'upstream']),
            FakeCommand(
              command: const <String>['git', 'checkout', workingBranch],
              onRun: (_) {
                final File file = fileSystem.file('$checkoutsParentDirectory/framework/.ci.yaml')
                  ..createSync();
                _initializeCiYamlFile(file);
              },
            ),
            const FakeCommand(
              command: <String>['git', 'status', '--porcelain'],
              stdout: 'MM bin/internal/release-candidate-branch.version',
            ),
            const FakeCommand(command: <String>['git', 'add', '--all']),
            const FakeCommand(
              command: <String>[
                'git',
                'commit',
                '--message',
                'Create candidate branch version $candidateBranch for $releaseChannel',
              ],
            ),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision3),
            const FakeCommand(
              command: <String>['git', 'status', '--porcelain'],
              stdout: 'MM bin/internal/engine.version',
            ),
            const FakeCommand(command: <String>['git', 'add', '--all']),
            const FakeCommand(
              command: <String>[
                'git',
                'commit',
                '--message',
                'Update Engine revision to $revision1 for $releaseChannel release $releaseVersion',
              ],
            ),
            const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision4),
            const FakeCommand(
              command: <String>['git', 'push', 'mirror', 'HEAD:refs/heads/$workingBranch'],
            ),
          ]);
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(finalState.currentPhase, ReleasePhase.PUBLISH_VERSION);
          expect(
            stdio.stdout,
            contains('Rolling new engine hash $revision1 to framework checkout...'),
          );
          expect(stdio.stdout, contains('There was 1 cherrypick that was not auto-applied'));
          expect(
            stdio.stdout,
            contains(
              'Are you ready to push your framework branch to the repository $mirrorRemoteUrl? (y/n)',
            ),
          );
          expect(
            stdio.stdout,
            contains('Executed command: `git push mirror HEAD:refs/heads/$workingBranch`'),
          );
          expect(stdio.error, isEmpty);
        });
      });

      group('PUBLISH_VERSION to VERIFY_RELEASE', () {
        const String releaseVersion = '1.2.0-3.0.pre';
        late pb.ConductorState state;

        setUp(() {
          state =
              (pb.ConductorState.create()
                ..releaseChannel = releaseChannel
                ..currentPhase = ReleasePhase.PUBLISH_VERSION
                ..framework =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..upstream = (pb.Remote.create()..url = FrameworkRepository.defaultUpstream))
                ..engine =
                    (pb.Repository.create()
                      ..candidateBranch = candidateBranch
                      ..upstream = (pb.Remote.create()..url = EngineRepository.defaultUpstream))
                ..releaseVersion = releaseVersion);
        });

        test('gives push command and updates state.currentPhase', () async {
          stdio.stdin.add('y');
          final FakeProcessManager processManager = FakeProcessManager.empty();
          final FakePlatform platform = FakePlatform(
            environment: <String, String>{
              'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
            },
            operatingSystem: localOperatingSystem,
            pathSeparator: localPathSeparator,
          );
          writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
          final Checkouts checkouts = Checkouts(
            fileSystem: fileSystem,
            parentDirectory: fileSystem.directory(checkoutsParentDirectory)
              ..createSync(recursive: true),
            platform: platform,
            processManager: processManager,
            stdio: stdio,
          );
          final CommandRunner<void> runner = createRunner(checkouts: checkouts);
          await runner.run(<String>['next', '--$kStateOption', stateFile]);

          final pb.ConductorState finalState = readStateFromFile(fileSystem.file(stateFile));

          expect(processManager, hasNoRemainingExpectations);
          expect(finalState.currentPhase, ReleasePhase.VERIFY_RELEASE);
          expect(stdio.stdout, contains('Run the following command, and ask a Googler'));
          expect(stdio.stdout, contains(':tag $releaseVersion'));
          expect(stdio.stdout, contains(':git_branch ${state.framework.candidateBranch}'));
          expect(stdio.stdout, contains(':release_channel ${state.releaseChannel}'));
          expect(finalState.logs, stdio.logs);
        });
      });

      test('throws exception if state.currentPhase is RELEASE_COMPLETED', () async {
        final FakeProcessManager processManager = FakeProcessManager.empty();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{
            'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
          },
          operatingSystem: localOperatingSystem,
          pathSeparator: localPathSeparator,
        );
        final pb.ConductorState state =
            pb.ConductorState.create()..currentPhase = ReleasePhase.RELEASE_COMPLETED;
        writeStateToFile(fileSystem.file(stateFile), state, <String>[]);
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory)
            ..createSync(recursive: true),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );
        final CommandRunner<void> runner = createRunner(checkouts: checkouts);
        expect(
          () async => runner.run(<String>['next', '--$kStateOption', stateFile]),
          throwsExceptionWith('This release is finished.'),
        );
      });
    },
    onPlatform: <String, dynamic>{
      'windows': const Skip('Flutter Conductor only supported on macos/linux'),
    },
  );

  group('prompt', () {
    test('can be overridden for different frontend implementations', () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Stdio stdio = _UnimplementedStdio.instance;
      final Checkouts checkouts = Checkouts(
        fileSystem: fileSystem,
        parentDirectory: fileSystem.directory('/'),
        platform: FakePlatform(),
        processManager: FakeProcessManager.empty(),
        stdio: stdio,
      );
      final _TestNextContext context = _TestNextContext(
        checkouts: checkouts,
        stateFile: fileSystem.file('/statefile.json'),
      );

      final bool response = await context.prompt('A prompt that will immediately be agreed to');
      expect(response, true);
    });

    test('throws if user inputs character that is not "y" or "n"', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TestStdio stdio = TestStdio(stdin: <String>['x'], verbose: true);
      final Checkouts checkouts = Checkouts(
        fileSystem: fileSystem,
        parentDirectory: fileSystem.directory('/'),
        platform: FakePlatform(),
        processManager: FakeProcessManager.empty(),
        stdio: stdio,
      );
      final NextContext context = NextContext(
        autoAccept: false,
        force: false,
        checkouts: checkouts,
        stateFile: fileSystem.file('/statefile.json'),
      );

      expect(
        () => context.prompt('Asking a question?'),
        throwsExceptionWith('Unknown user input (expected "y" or "n")'),
      );
    });
  });

  group('.pushWorkingBranch()', () {
    late MemoryFileSystem fileSystem;
    late TestStdio stdio;
    late Platform platform;

    setUp(() {
      stdio = TestStdio();
      fileSystem = MemoryFileSystem.test();
      platform = FakePlatform();
    });

    test(
      'catches GitException if the push was rejected and instead throws a helpful ConductorException',
      () async {
        const String gitPushErrorMessage = '''
 To github.com:user/engine.git

  ! [rejected]            HEAD -> cherrypicks-flutter-2.8-candidate.3 (non-fast-forward)
 error: failed to push some refs to 'github.com:user/engine.git'
 hint: Updates were rejected because the tip of your current branch is behind
 hint: its remote counterpart. Integrate the remote changes (e.g.
 hint: 'git pull ...') before pushing again.
 hint: See the 'Note about fast-forwards' in 'git push --help' for details.
''';
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory)
            ..createSync(recursive: true),
          platform: platform,
          processManager: FakeProcessManager.empty(),
          stdio: stdio,
        );
        final Repository testRepository = _TestRepository.fromCheckouts(checkouts);
        final pb.Repository testPbRepository = pb.Repository();
        (checkouts.processManager as FakeProcessManager).addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'git',
              'clone',
              '--origin',
              'upstream',
              '--',
              testRepository.upstreamRemote.url,
              '/flutter/dev/conductor/flutter_conductor_checkouts/test-repo/test-repo',
            ],
          ),
          const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: revision1),
          FakeCommand(
            command: const <String>['git', 'push', '', 'HEAD:refs/heads/'],
            exception: GitException(gitPushErrorMessage, <String>[
              'git',
              'push',
              '--force',
              '',
              'HEAD:refs/heads/',
            ]),
          ),
        ]);
        final NextContext nextContext = NextContext(
          autoAccept: false,
          checkouts: checkouts,
          force: false,
          stateFile: fileSystem.file(stateFile),
        );

        expect(
          () => nextContext.pushWorkingBranch(testRepository, testPbRepository),
          throwsA(
            isA<ConductorException>().having(
              (ConductorException exception) => exception.message,
              'has correct message',
              contains('Re-run this command with --force to overwrite the remote branch'),
            ),
          ),
        );
      },
    );
  });
}

/// A [Stdio] that will throw an exception if any of its methods are called.
class _UnimplementedStdio extends Fake implements Stdio {
  _UnimplementedStdio();

  static final _UnimplementedStdio instance = _UnimplementedStdio();
}

class _TestRepository extends Repository {
  _TestRepository.fromCheckouts(Checkouts checkouts, [String name = 'test-repo'])
    : super(
        fileSystem: checkouts.fileSystem,
        parentDirectory: checkouts.directory.childDirectory(name),
        platform: checkouts.platform,
        processManager: checkouts.processManager,
        name: name,
        requiredLocalBranches: <String>[],
        stdio: checkouts.stdio,
        upstreamRemote: const Remote.upstream('git@github.com:upstream/repo.git'),
      );

  @override
  Future<_TestRepository> cloneRepository(String? cloneName) async {
    throw Exception('Unimplemented!');
  }
}

class _TestNextContext extends NextContext {
  const _TestNextContext({required super.stateFile, required super.checkouts})
    : super(autoAccept: false, force: false);

  @override
  Future<bool> prompt(String message) {
    // always say yes
    return Future<bool>.value(true);
  }
}

void _initializeCiYamlFile(File file, {List<String>? enabledBranches}) {
  enabledBranches ??= <String>['master', 'beta', 'stable'];
  file.createSync(recursive: true);
  final StringBuffer buffer = StringBuffer('enabled_branches:\n');
  for (final String branch in enabledBranches) {
    buffer.writeln('  - $branch');
  }
  buffer.writeln('''

platform_properties:
  linux:
    properties:
    caches: ["name":"openjdk","path":"java"]

targets:
  - name: Linux analyze
    recipe: flutter/flutter
    timeout: 60
    properties:
      tags: >
        ["framework","hostonly"]
      validation: analyze
      validation_name: Analyze
    scheduler: luci
''');
  file.writeAsStringSync(buffer.toString());
}
