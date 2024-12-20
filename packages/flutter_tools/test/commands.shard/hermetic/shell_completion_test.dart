// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/shell_completion.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  group('shell_completion', () {
    late FakeStdio fakeStdio;

    setUp(() {
      Cache.disableLocking();
      fakeStdio = FakeStdio()..stdout.terminalColumns = 80;
    });

    testUsingContext(
      'generates bash initialization script to stdout',
      () async {
        final ShellCompletionCommand command = ShellCompletionCommand();
        await createTestCommandRunner(command).run(<String>['bash-completion']);
        expect(fakeStdio.writtenToStdout.length, equals(1));
        expect(fakeStdio.writtenToStdout.first, contains('__flutter_completion'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Stdio: () => fakeStdio,
      },
    );

    testUsingContext(
      'generates bash initialization script to stdout with arg',
      () async {
        final ShellCompletionCommand command = ShellCompletionCommand();
        await createTestCommandRunner(command).run(<String>['bash-completion', '-']);
        expect(fakeStdio.writtenToStdout.length, equals(1));
        expect(fakeStdio.writtenToStdout.first, contains('__flutter_completion'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Stdio: () => fakeStdio,
      },
    );

    testUsingContext(
      'generates bash initialization script to output file',
      () async {
        final ShellCompletionCommand command = ShellCompletionCommand();
        const String outputFile = 'bash-setup.sh';
        await createTestCommandRunner(command).run(<String>['bash-completion', outputFile]);
        expect(globals.fs.isFileSync(outputFile), isTrue);
        expect(globals.fs.file(outputFile).readAsStringSync(), contains('__flutter_completion'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Stdio: () => fakeStdio,
      },
    );

    testUsingContext(
      "won't overwrite existing output file ",
      () async {
        final ShellCompletionCommand command = ShellCompletionCommand();
        const String outputFile = 'bash-setup.sh';
        globals.fs.file(outputFile).createSync();
        await expectLater(
          () => createTestCommandRunner(command).run(<String>['bash-completion', outputFile]),
          throwsA(
            isA<ToolExit>()
                .having((ToolExit error) => error.exitCode, 'exitCode', anyOf(isNull, 1))
                .having((ToolExit error) => error.message, 'message', contains('Use --overwrite')),
          ),
        );
        expect(globals.fs.isFileSync(outputFile), isTrue);
        expect(globals.fs.file(outputFile).readAsStringSync(), isEmpty);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Stdio: () => fakeStdio,
      },
    );

    testUsingContext(
      'will overwrite existing output file if given --overwrite',
      () async {
        final ShellCompletionCommand command = ShellCompletionCommand();
        const String outputFile = 'bash-setup.sh';
        globals.fs.file(outputFile).createSync();
        await createTestCommandRunner(
          command,
        ).run(<String>['bash-completion', '--overwrite', outputFile]);
        expect(globals.fs.isFileSync(outputFile), isTrue);
        expect(globals.fs.file(outputFile).readAsStringSync(), contains('__flutter_completion'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Stdio: () => fakeStdio,
      },
    );
  });
}
