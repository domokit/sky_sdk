// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/channel.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/version.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart' show FakeFlutterVersion;
import '../src/test_flutter_command_runner.dart';

void main() {
  group('channel', () {
    late FakeProcessManager fakeProcessManager;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
    });

    setUpAll(() {
      Cache.disableLocking();
    });

    Future<void> simpleChannelTest(List<String> args) async {
      fakeProcessManager.addCommands(const <FakeCommand>[
        FakeCommand(
          command: <String>['git', 'branch', '-r'],
          stdout:
              '  origin/branch-1\n'
              '  origin/branch-2\n'
              '  origin/master\n'
              '  origin/main\n'
              '  origin/stable\n'
              '  origin/beta',
        ),
      ]);
      final ChannelCommand command = ChannelCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(args);
      expect(testLogger.errorText, hasLength(0));
      // The bots may return an empty list of channels (network hiccup?)
      // and when run locally the list of branches might be different
      // so we check for the header text rather than any specific channel name.
      expect(testLogger.statusText, containsIgnoringWhitespace('Flutter channels:'));
    }

    testUsingContext(
      'usage (--help) explains how to use channel',
      () async {
        final ChannelCommand command = ChannelCommand();

        // Required because otherwise command.usage fails as it is not hooked up.
        createTestCommandRunner(command);

        // TODO(matanlurey): https://github.com/flutter/flutter/issues/158532
        //
        // <Command>.usage is checked instead of log output because by default
        // every command emits usage directly to stdout (via print) instead of
        // to the interfaces provided. It would be a much larger refactor to
        // change how every command works:
        expect(
          command.usage,
          stringContainsInOrder(<String>[
            'List or switch Flutter channels',
            'Common commands:',
            'List Flutter channels',
            "Switch to Flutter's main channel.",
          ]),
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.empty(),
        FileSystem: () => MemoryFileSystem.test(),
      },
    );

    testUsingContext(
      'list',
      () async {
        await simpleChannelTest(<String>['channel']);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
      },
    );

    testUsingContext(
      'verbose list',
      () async {
        await simpleChannelTest(<String>['channel', '-v']);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
      },
    );

    testUsingContext(
      'sorted by stability',
      () async {
        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/beta\n'
                'origin/master\n'
                'origin/main\n'
                'origin/stable\n',
          ),
        );

        await runner.run(<String>['channel']);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        expect(
          testLogger.statusText,
          'Flutter channels:\n'
          '* master (latest development branch, for contributors)\n'
          '  main (latest development branch, follows master channel)\n'
          '  beta (updated monthly, recommended for experienced users)\n'
          '  stable (updated quarterly, for new users and for production app releases)\n',
        );

        // clear buffer for next process
        testLogger.clear();

        // Extra branches.
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/beta\n'
                'origin/master\n'
                'origin/dependabot/bundler\n'
                'origin/main\n'
                'origin/v1.4.5-hotfixes\n'
                'origin/stable\n',
          ),
        );

        await runner.run(<String>['channel']);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        expect(
          testLogger.statusText,
          'Flutter channels:\n'
          '* master (latest development branch, for contributors)\n'
          '  main (latest development branch, follows master channel)\n'
          '  beta (updated monthly, recommended for experienced users)\n'
          '  stable (updated quarterly, for new users and for production app releases)\n',
        );

        // clear buffer for next process
        testLogger.clear();

        // Missing branches.
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/master\n'
                'origin/dependabot/bundler\n'
                'origin/v1.4.5-hotfixes\n'
                'origin/stable\n'
                'origin/beta\n',
          ),
        );

        await runner.run(<String>['channel']);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        // check if available official channels are in order of stability
        int prev = -1;
        int next = -1;
        for (final String branch in kOfficialChannels) {
          next = testLogger.statusText.indexOf(branch);
          if (next != -1) {
            expect(prev < next, isTrue);
            prev = next;
          }
        }
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
      },
    );

    testUsingContext(
      'ignores lines with unexpected output',
      () async {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/beta\n'
                'origin/stable\n'
                'upstream/beta\n'
                'upstream/stable\n'
                'foo',
          ),
        );

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel']);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        expect(
          testLogger.statusText,
          'Flutter channels:\n'
          '* beta (updated monthly, recommended for experienced users)\n'
          '  stable (updated quarterly, for new users and for production app releases)\n',
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
        FlutterVersion: () => FakeFlutterVersion(branch: 'beta'),
      },
    );

    testUsingContext(
      'handles custom branches',
      () async {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/beta\n'
                'origin/stable\n'
                'origin/foo',
          ),
        );

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel']);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        expect(
          testLogger.statusText,
          'Flutter channels:\n'
          '  beta (updated monthly, recommended for experienced users)\n'
          '  stable (updated quarterly, for new users and for production app releases)\n'
          '* foo\n'
          '\n'
          'Currently not on an official channel.\n',
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
        FlutterVersion: () => FakeFlutterVersion(branch: 'foo'),
      },
    );

    testUsingContext(
      'removes duplicates',
      () async {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['git', 'branch', '-r'],
            stdout:
                'origin/beta\n'
                'origin/stable\n'
                'upstream/beta\n'
                'upstream/stable\n',
          ),
        );

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel']);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(testLogger.errorText, hasLength(0));
        expect(
          testLogger.statusText,
          'Flutter channels:\n'
          '* beta (updated monthly, recommended for experienced users)\n'
          '  stable (updated quarterly, for new users and for production app releases)\n',
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => MemoryFileSystem.test(),
        FlutterVersion: () => FakeFlutterVersion(branch: 'beta'),
      },
    );

    testUsingContext(
      'can switch channels',
      () async {
        fakeProcessManager.addCommands(const <FakeCommand>[
          FakeCommand(command: <String>['git', 'fetch']),
          FakeCommand(
            command: <String>['git', 'show-ref', '--verify', '--quiet', 'refs/heads/beta'],
          ),
          FakeCommand(command: <String>['git', 'checkout', 'beta', '--']),
          FakeCommand(
            command: <String>['bin/flutter', '--no-color', '--no-version-check', 'precache'],
          ),
        ]);

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel', 'beta']);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(
          testLogger.statusText,
          containsIgnoringWhitespace("Switching to flutter channel 'beta'..."),
        );
        expect(testLogger.errorText, hasLength(0));

        fakeProcessManager.addCommands(const <FakeCommand>[
          FakeCommand(command: <String>['git', 'fetch']),
          FakeCommand(
            command: <String>['git', 'show-ref', '--verify', '--quiet', 'refs/heads/stable'],
          ),
          FakeCommand(command: <String>['git', 'checkout', 'stable', '--']),
          FakeCommand(
            command: <String>['bin/flutter', '--no-color', '--no-version-check', 'precache'],
          ),
        ]);

        await runner.run(<String>['channel', 'stable']);

        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => fakeProcessManager,
      },
    );

    testUsingContext(
      'switching channels prompts to run flutter upgrade',
      () async {
        fakeProcessManager.addCommands(const <FakeCommand>[
          FakeCommand(command: <String>['git', 'fetch']),
          FakeCommand(
            command: <String>['git', 'show-ref', '--verify', '--quiet', 'refs/heads/beta'],
          ),
          FakeCommand(command: <String>['git', 'checkout', 'beta', '--']),
          FakeCommand(
            command: <String>['bin/flutter', '--no-color', '--no-version-check', 'precache'],
          ),
        ]);

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel', 'beta']);

        expect(
          testLogger.statusText,
          containsIgnoringWhitespace("Successfully switched to flutter channel 'beta'."),
        );
        expect(
          testLogger.statusText,
          containsIgnoringWhitespace(
            "To ensure that you're on the latest build "
            "from this channel, run 'flutter upgrade'",
          ),
        );
        expect(testLogger.errorText, hasLength(0));
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => fakeProcessManager,
      },
    );

    // This verifies that bug https://github.com/flutter/flutter/issues/21134
    // doesn't return.
    testUsingContext(
      'removes version stamp file when switching channels',
      () async {
        fakeProcessManager.addCommands(const <FakeCommand>[
          FakeCommand(command: <String>['git', 'fetch']),
          FakeCommand(
            command: <String>['git', 'show-ref', '--verify', '--quiet', 'refs/heads/beta'],
          ),
          FakeCommand(command: <String>['git', 'checkout', 'beta', '--']),
          FakeCommand(
            command: <String>['bin/flutter', '--no-color', '--no-version-check', 'precache'],
          ),
        ]);

        final File versionCheckFile = globals.cache.getStampFileFor(
          VersionCheckStamp.flutterVersionCheckStampFile,
        );

        /// Create a bogus "leftover" version check file to make sure it gets
        /// removed when the channel changes. The content doesn't matter.
        versionCheckFile.createSync(recursive: true);
        versionCheckFile.writeAsStringSync('''
        {
          "lastTimeVersionWasChecked": "2151-08-29 10:17:30.763802",
          "lastKnownRemoteVersion": "2151-09-26 15:56:19.000Z"
        }
      ''');

        final ChannelCommand command = ChannelCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['channel', 'beta']);

        expect(testLogger.statusText, isNot(contains('A new version of Flutter')));
        expect(testLogger.errorText, hasLength(0));
        expect(versionCheckFile.existsSync(), isFalse);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => fakeProcessManager,
      },
    );
  });
}
