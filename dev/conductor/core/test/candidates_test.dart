// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:conductor_core/src/candidates.dart';
import 'package:conductor_core/src/repository.dart';
import 'package:file/memory.dart';
import 'package:platform/platform.dart';

import './common.dart';

void main() {
  group(
    'candidates command',
    () {
      const String flutterRoot = '/flutter';
      const String flutterBinPath = '$flutterRoot/bin/flutter';
      const String checkoutsParentDirectory = '$flutterRoot/dev/tools/';
      const String remoteName = 'origin';

      late MemoryFileSystem fileSystem;
      late FakePlatform platform;
      late TestStdio stdio;
      late FakeProcessManager processManager;
      final String operatingSystem = const LocalPlatform().operatingSystem;

      setUp(() {
        stdio = TestStdio();
        fileSystem = MemoryFileSystem.test();
      });

      CommandRunner<void> createRunner({required Checkouts checkouts}) {
        final CandidatesCommand command = CandidatesCommand(
          checkouts: checkouts,
          flutterRoot: fileSystem.directory(flutterRoot),
        );
        return CommandRunner<void>('clean-test', '')..addCommand(command);
      }

      test('prints only branches from targeted remote', () async {
        const String currentVersion = '1.2.3';
        const String branch = 'flutter-1.3-candidate.0';

        processManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['git', 'fetch', remoteName]),
          const FakeCommand(command: <String>[flutterBinPath, 'help']),
          const FakeCommand(
            command: <String>[flutterBinPath, '--version', '--machine'],
            stdout: '{"frameworkVersion": "$currentVersion"}',
          ),
          FakeCommand(
            command: const <String>[
              'git',
              'branch',
              '--no-color',
              '--remotes',
              '--list',
              '$remoteName/*',
            ],
            stdout: <String>[
              'other-remote/flutter-5.0-candidate.0',
              '$remoteName/$branch',
            ].join('\n'),
          ),
        ]);
        final String pathSeparator = operatingSystem == 'windows' ? r'\' : '/';

        platform = FakePlatform(
          environment: <String, String>{
            'HOME': <String>['path', 'to', 'home'].join(pathSeparator),
          },
          pathSeparator: pathSeparator,
        );
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );

        final CommandRunner<void> runner = createRunner(checkouts: checkouts);
        await runner.run(<String>['candidates', '--$kRemote', remoteName]);
        expect(stdio.stdout.contains('currentVersion = $currentVersion'), true);
        expect(stdio.stdout.contains(branch), true);
        expect(stdio.stdout.contains('flutter-5.0-candidate.0'), false);
      });

      test('does not print branches older or equal to current version', () async {
        const String currentVersion = '2.3.0-13.0.pre.48';
        const String newBranch = 'flutter-2.4-candidate.0';
        const String oldBranch = 'flutter-1.0-candidate.0';
        const String currentBranch = 'flutter-2.3-candidate.13';

        processManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['git', 'fetch', remoteName]),
          const FakeCommand(command: <String>[flutterBinPath, 'help']),
          const FakeCommand(
            command: <String>[flutterBinPath, '--version', '--machine'],
            stdout: '{"frameworkVersion": "$currentVersion"}',
          ),
          FakeCommand(
            command: const <String>[
              'git',
              'branch',
              '--no-color',
              '--remotes',
              '--list',
              '$remoteName/*',
            ],
            stdout: <String>[
              '$remoteName/$oldBranch',
              '$remoteName/$currentBranch',
              '$remoteName/$newBranch',
            ].join('\n'),
          ),
        ]);
        final String pathSeparator = operatingSystem == 'windows' ? r'\' : '/';

        platform = FakePlatform(
          environment: <String, String>{
            'HOME': <String>['path', 'to', 'home'].join(pathSeparator),
          },
          pathSeparator: pathSeparator,
        );
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );

        final CommandRunner<void> runner = createRunner(checkouts: checkouts);
        await runner.run(<String>['candidates', '--$kRemote', remoteName]);
        expect(stdio.stdout.contains('currentVersion = $currentVersion'), true);
        expect(stdio.stdout.contains(newBranch), true);
        expect(stdio.stdout.contains(oldBranch), false);
        expect(stdio.stdout.contains(currentBranch), false);
      });
    },
    onPlatform: <String, dynamic>{
      'windows': const Skip('Flutter Conductor only supported on macos/linux'),
    },
  );
}
