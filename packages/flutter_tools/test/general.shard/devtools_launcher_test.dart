// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/devtools_launcher.dart';
import 'package:flutter_tools/src/resident_runner.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

void main() {
  Cache.flutterRoot = '';

  (BufferLogger, Artifacts) getTestState() => (BufferLogger.test(), Artifacts.test());

  testWithoutContext('DevtoolsLauncher launches DevTools from the SDK and saves the URI', () async {
    final (BufferLogger logger, Artifacts artifacts) = getTestState();
    final Completer<void> completer = Completer<void>();
    final DevtoolsLauncher launcher = DevtoolsServerLauncher(
      artifacts: artifacts,
      logger: logger,
      botDetector: const FakeBotDetector(false),
      processManager: FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>[
            'Artifact.engineDartBinary',
            'devtools',
            '--no-launch-browser',
            // TODO(bkonyi): does this need to be removed?
            // '--print-dtd',
          ],
          stdout: 'Serving DevTools at http://127.0.0.1:9100.\n',
          completer: completer,
        ),
      ]),
    );

    expect(launcher.dtdUri, isNull);
    expect(launcher.printDtdUri, false);
    final DevToolsServerAddress? address = await launcher.serve();
    expect(address?.host, '127.0.0.1');
    expect(address?.port, 9100);
    expect(launcher.dtdUri, isNull);
    expect(launcher.printDtdUri, false);
  });

  // TODO(bkonyi): this test can be removed when DevTools is served from DDS.
  testWithoutContext('DevtoolsLauncher saves the Dart Tooling Daemon uri', () async {
    final (BufferLogger logger, Artifacts artifacts) = getTestState();
    final Completer<void> completer = Completer<void>();
    final DevtoolsLauncher launcher = DevtoolsServerLauncher(
      artifacts: artifacts,
      logger: logger,
      botDetector: const FakeBotDetector(false),
      processManager: FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>[
            'Artifact.engineDartBinary',
            'devtools',
            '--no-launch-browser',
            '--print-dtd',
          ],
          stdout: '''
Serving the Dart Tooling Daemon at ws://127.0.0.1:53449/
Serving DevTools at http://127.0.0.1:9100.
''',
          completer: completer,
        ),
      ]),
    )..printDtdUri = true;

    expect(launcher.dtdUri, isNull);
    expect(launcher.printDtdUri, true);
    final DevToolsServerAddress? address = await launcher.serve();
    expect(address?.host, '127.0.0.1');
    expect(address?.port, 9100);
    expect(launcher.dtdUri?.toString(), 'ws://127.0.0.1:53449/');
    expect(launcher.printDtdUri, true);
  });

  testWithoutContext(
    'DevtoolsLauncher does not launch a new DevTools instance if one is already active',
    () async {
      final (BufferLogger logger, Artifacts artifacts) = getTestState();
      final Completer<void> completer = Completer<void>();
      final DevtoolsLauncher launcher = DevtoolsServerLauncher(
        artifacts: artifacts,
        logger: logger,
        botDetector: const FakeBotDetector(false),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: const <String>[
              'Artifact.engineDartBinary',
              'devtools',
              '--no-launch-browser',
              // '--print-dtd',
            ],
            stdout: 'Serving DevTools at http://127.0.0.1:9100.\n',
            completer: completer,
          ),
        ]),
      );

      DevToolsServerAddress? address = await launcher.serve();
      expect(address?.host, '127.0.0.1');
      expect(address?.port, 9100);

      // Call `serve` again and verify that the already-running server is returned.
      address = await launcher.serve();
      expect(address?.host, '127.0.0.1');
      expect(address?.port, 9100);
    },
  );

  testWithoutContext('DevtoolsLauncher can launch devtools with a memory profile', () async {
    final (BufferLogger logger, Artifacts artifacts) = getTestState();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>[
          'Artifact.engineDartBinary',
          'devtools',
          '--no-launch-browser',
          // '--print-dtd',
          '--vm-uri=localhost:8181/abcdefg',
          '--profile-memory=foo',
        ],
        stdout: 'Serving DevTools at http://127.0.0.1:9100.\n',
      ),
    ]);
    final DevtoolsLauncher launcher = DevtoolsServerLauncher(
      artifacts: artifacts,
      logger: logger,
      botDetector: const FakeBotDetector(false),
      processManager: processManager,
    );

    await launcher.launch(
      Uri.parse('localhost:8181/abcdefg'),
      additionalArguments: <String>['--profile-memory=foo'],
    );

    expect(launcher.processStart, completes);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext(
    'DevtoolsLauncher prints error if exception is thrown during launch',
    () async {
      final (BufferLogger logger, Artifacts artifacts) = getTestState();
      final DevtoolsLauncher launcher = DevtoolsServerLauncher(
        artifacts: artifacts,
        logger: logger,
        botDetector: const FakeBotDetector(false),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>[
              'Artifact.engineDartBinary',
              'devtools',
              '--no-launch-browser',
              // '--print-dtd',
              '--vm-uri=http://127.0.0.1:1234/abcdefg',
            ],
            exception: ProcessException('pub', <String>[]),
          ),
        ]),
      );

      await launcher.launch(Uri.parse('http://127.0.0.1:1234/abcdefg'));

      expect(logger.errorText, contains('Failed to launch DevTools: ProcessException'));
    },
  );

  testWithoutContext('DevtoolsLauncher handles failure of DevTools process on a bot', () async {
    final (BufferLogger logger, Artifacts artifacts) = getTestState();
    final Completer<void> completer = Completer<void>();
    final DevtoolsServerLauncher launcher = DevtoolsServerLauncher(
      artifacts: artifacts,
      logger: logger,
      botDetector: const FakeBotDetector(true),
      processManager: FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>[
            'Artifact.engineDartBinary',
            'devtools',
            '--no-launch-browser',
            // '--print-dtd',
          ],
          stdout: 'Serving DevTools at http://127.0.0.1:9100.\n',
          completer: completer,
          exitCode: 255,
        ),
      ]),
    );

    await launcher.launch(null);
    completer.complete();
    expect(launcher.devToolsProcessExit, throwsToolExit());
  });
}
