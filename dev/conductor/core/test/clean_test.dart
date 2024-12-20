// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:conductor_core/src/clean.dart';
import 'package:conductor_core/src/repository.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:platform/platform.dart';

import './common.dart';

void main() {
  group(
    'clean command',
    () {
      const String flutterRoot = '/flutter';
      const String checkoutsParentDirectory = '$flutterRoot/dev/tools/';
      const String stateFilePath = '/state-file.json';

      late MemoryFileSystem fileSystem;
      late FakePlatform platform;
      late TestStdio stdio;
      late FakeProcessManager processManager;
      late CommandRunner<void> runner;

      setUp(() {
        stdio = TestStdio();
        fileSystem = MemoryFileSystem.test();
        final String operatingSystem = const LocalPlatform().operatingSystem;
        final String pathSeparator = operatingSystem == 'windows' ? r'\' : '/';

        processManager = FakeProcessManager.empty();
        platform = FakePlatform(
          environment: <String, String>{'HOME': '/path/to/user/home'},
          pathSeparator: pathSeparator,
        );
        final Checkouts checkouts = Checkouts(
          fileSystem: fileSystem,
          parentDirectory: fileSystem.directory(checkoutsParentDirectory),
          platform: platform,
          processManager: processManager,
          stdio: stdio,
        );
        final CleanCommand command = CleanCommand(checkouts: checkouts);
        runner = CommandRunner<void>('clean-test', '')..addCommand(command);
      });

      test('throws if no state file found', () async {
        await expectLater(
          () async =>
              runner.run(<String>['clean', '--$kStateOption', stateFilePath, '--$kYesFlag']),
          throwsExceptionWith('No persistent state file found at $stateFilePath'),
        );
      });

      test('deletes an empty state file', () async {
        final File stateFile = fileSystem.file(stateFilePath);
        stateFile.writeAsStringSync('');

        await runner.run(<String>['clean', '--$kStateOption', stateFile.path, '--$kYesFlag']);

        expect(stateFile.existsSync(), false);
      });

      test('deletes a state file with content', () async {
        final File stateFile = fileSystem.file(stateFilePath);
        stateFile.writeAsStringSync('{status: pending}');

        await runner.run(<String>['clean', '--$kStateOption', stateFile.path, '--$kYesFlag']);

        expect(stateFile.existsSync(), false);
      });
    },
    onPlatform: <String, dynamic>{
      'windows': const Skip('Flutter Conductor only supported on macos/linux'),
    },
  );
}
