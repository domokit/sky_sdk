// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;

import 'package:conductor_core/conductor_core.dart';
import 'package:conductor_core/packages_autoroller.dart';
import 'package:conductor_core/src/validate_checkout_post_gradle_regeneration.dart';
import 'package:file/memory.dart';
import 'package:path/path.dart' show Context, Style;
import 'package:platform/platform.dart';

import '../bin/packages_autoroller.dart' show run;
import 'common.dart';

void main() {
  const String flutterRoot = '/flutter';
  const String checkoutsParentDirectory = '$flutterRoot/dev/conductor';
  const String githubClient = 'gh';
  const String token = '0123456789abcdef';
  const String orgName = 'flutter-roller';
  const String mirrorUrl = 'https://githost.com/flutter-roller/flutter.git';
  final String localPathSeparator = const LocalPlatform().pathSeparator;
  final String localOperatingSystem = const LocalPlatform().operatingSystem;
  late MemoryFileSystem fileSystem;
  late TestStdio stdio;
  late FrameworkRepository framework;
  late PackageAutoroller autoroller;
  late FakeProcessManager processManager;

  setUp(() {
    stdio = TestStdio();
    fileSystem = MemoryFileSystem.test();
    processManager = FakeProcessManager.empty();
    final FakePlatform platform = FakePlatform(
      environment: <String, String>{
        'HOME': <String>['path', 'to', 'home'].join(localPathSeparator),
      },
      operatingSystem: localOperatingSystem,
      pathSeparator: localPathSeparator,
    );
    final Checkouts checkouts = Checkouts(
      fileSystem: fileSystem,
      parentDirectory: fileSystem.directory(checkoutsParentDirectory)..createSync(recursive: true),
      platform: platform,
      processManager: processManager,
      stdio: stdio,
    );
    framework = FrameworkRepository(checkouts, mirrorRemote: const Remote.mirror(mirrorUrl));

    autoroller = PackageAutoroller(
      githubClient: githubClient,
      token: token,
      framework: framework,
      orgName: orgName,
      processManager: processManager,
      stdio: stdio,
      githubUsername: 'flutter-pub-roller-bot',
    );
  });

  test('GitHub token is redacted from exceptions while pushing', () async {
    final StreamController<List<int>> controller = StreamController<List<int>>();
    processManager.addCommands(<FakeCommand>[
      FakeCommand(
        command: const <String>[
          'gh',
          'auth',
          'login',
          '--hostname',
          'github.com',
          '--git-protocol',
          'https',
          '--with-token',
        ],
        stdin: io.IOSink(controller.sink),
      ),
      const FakeCommand(
        command: <String>[
          'git',
          'clone',
          '--origin',
          'upstream',
          '--',
          FrameworkRepository.defaultUpstream,
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework',
        ],
      ),
      const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', mirrorUrl]),
      const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', FrameworkRepository.defaultBranch]),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: 'deadbeef'),
      const FakeCommand(command: <String>['git', 'ls-remote', '--heads', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', '-b', 'packages-autoroller-branch-1']),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/flutter',
          'help',
        ],
      ),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/flutter',
          '--verbose',
          'update-packages',
          '--force-upgrade',
        ],
      ),
      const FakeCommand(
        command: <String>['git', 'status', '--porcelain'],
        stdout: '''
 M packages/foo/pubspec.yaml
 M packages/bar/pubspec.yaml
 M dev/integration_tests/test_foo/pubspec.yaml
''',
      ),
      const FakeCommand(command: <String>['git', 'add', '--all']),
      const FakeCommand(
        command: <String>[
          'git',
          'commit',
          '--message',
          'roll packages',
          '--author="fluttergithubbot <fluttergithubbot@google.com>"',
        ],
      ),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: '000deadbeef'),
      const FakeCommand(
        command: <String>[
          'git',
          'push',
          'https://$token@github.com/$orgName/flutter.git',
          'packages-autoroller-branch-1:packages-autoroller-branch-1',
        ],
        exitCode: 1,
        stderr: 'Authentication error!',
      ),
    ]);
    await expectLater(
      () async {
        final Future<void> rollFuture = autoroller.roll();
        await controller.stream.drain<Object?>();
        await rollFuture;
      },
      throwsA(
        isA<Exception>().having(
          (Exception exc) => exc.toString(),
          'message',
          isNot(contains(token)),
        ),
      ),
    );
  });

  test('Does not attempt to roll if bot already has an open PR', () async {
    final StreamController<List<int>> controller = StreamController<List<int>>();
    processManager.addCommands(<FakeCommand>[
      FakeCommand(
        command: const <String>[
          'gh',
          'auth',
          'login',
          '--hostname',
          'github.com',
          '--git-protocol',
          'https',
          '--with-token',
        ],
        stdin: io.IOSink(controller.sink),
      ),
      const FakeCommand(
        command: <String>[
          'gh',
          'pr',
          'list',
          '--author',
          'flutter-pub-roller-bot',
          '--repo',
          'flutter/flutter',
          '--state',
          'open',
          '--search',
          'Roll pub packages',
          '--json',
          'number',
          // Non empty array means there are open PRs by the bot with the tool label
          // We expect no further commands to be run
        ],
        stdout: '[{"number": 123}]',
      ),
    ]);
    final Future<void> rollFuture = autoroller.roll();
    await controller.stream.drain<Object?>();
    await rollFuture;
    expect(processManager, hasNoRemainingExpectations);
    expect(stdio.stdout, contains('flutter-pub-roller-bot already has open tool PRs'));
    expect(stdio.stdout, contains(r'[{number: 123}]'));
  });

  test('Does not commit or create a PR if no changes were made', () async {
    final StreamController<List<int>> controller = StreamController<List<int>>();
    processManager.addCommands(<FakeCommand>[
      FakeCommand(
        command: const <String>[
          'gh',
          'auth',
          'login',
          '--hostname',
          'github.com',
          '--git-protocol',
          'https',
          '--with-token',
        ],
        stdin: io.IOSink(controller.sink),
      ),
      const FakeCommand(
        command: <String>[
          'gh',
          'pr',
          'list',
          '--author',
          'flutter-pub-roller-bot',
          '--repo',
          'flutter/flutter',
          '--state',
          'open',
          '--search',
          'Roll pub packages',
          '--json',
          'number',
          // Returns empty array, as there are no other open roll PRs from the bot
        ],
        stdout: '[]',
      ),
      const FakeCommand(
        command: <String>[
          'git',
          'clone',
          '--origin',
          'upstream',
          '--',
          FrameworkRepository.defaultUpstream,
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework',
        ],
      ),
      const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', mirrorUrl]),
      const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', FrameworkRepository.defaultBranch]),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: 'deadbeef'),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/dart',
          'pub',
          'get',
        ],
        workingDirectory:
            '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/dev/tools',
      ),
      FakeCommand(
        command: const <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/dart',
          'compile',
          'exe',
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/dev/tools/bin/generate_gradle_lockfiles.dart',
          '-o',
          '/.tmp_rand0/rand0/generate_gradle_lockfiles',
        ],
        onRun: (_) => fileSystem.file('/.tmp_rand0/rand0/generate_gradle_lockfiles').createSync(),
      ),
      const FakeCommand(
        command: <String>['chmod', '+x', '/.tmp_rand0/rand0/generate_gradle_lockfiles'],
      ),
      const FakeCommand(command: <String>['git', 'ls-remote', '--heads', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', '-b', 'packages-autoroller-branch-1']),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/flutter',
          '--verbose',
          'update-packages',
          '--force-upgrade',
        ],
      ),
      // Because there is no stdout to git status, the script should exit cleanly here
      const FakeCommand(command: <String>['git', 'status', '--porcelain']),
    ]);
    final Future<void> rollFuture = autoroller.roll();
    await controller.stream.drain<Object?>();
    await rollFuture;
    expect(processManager, hasNoRemainingExpectations);
  });

  test('can roll with correct inputs', () async {
    final StreamController<List<int>> controller = StreamController<List<int>>();
    processManager.addCommands(<FakeCommand>[
      FakeCommand(
        command: const <String>[
          'gh',
          'auth',
          'login',
          '--hostname',
          'github.com',
          '--git-protocol',
          'https',
          '--with-token',
        ],
        stdin: io.IOSink(controller.sink),
      ),
      const FakeCommand(
        command: <String>[
          'gh',
          'pr',
          'list',
          '--author',
          'flutter-pub-roller-bot',
          '--repo',
          'flutter/flutter',
          '--state',
          'open',
          '--search',
          'Roll pub packages',
          '--json',
          'number',
          // Returns empty array, as there are no other open roll PRs from the bot
        ],
        stdout: '[]',
      ),
      const FakeCommand(
        command: <String>[
          'git',
          'clone',
          '--origin',
          'upstream',
          '--',
          FrameworkRepository.defaultUpstream,
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework',
        ],
      ),
      const FakeCommand(command: <String>['git', 'remote', 'add', 'mirror', mirrorUrl]),
      const FakeCommand(command: <String>['git', 'fetch', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', FrameworkRepository.defaultBranch]),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: 'deadbeef'),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/dart',
          'pub',
          'get',
        ],
        workingDirectory:
            '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/dev/tools',
      ),
      FakeCommand(
        command: const <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/dart',
          'compile',
          'exe',
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/dev/tools/bin/generate_gradle_lockfiles.dart',
          '-o',
          '/.tmp_rand0/rand0/generate_gradle_lockfiles',
        ],
        onRun: (_) => fileSystem.file('/.tmp_rand0/rand0/generate_gradle_lockfiles').createSync(),
      ),
      const FakeCommand(
        command: <String>['chmod', '+x', '/.tmp_rand0/rand0/generate_gradle_lockfiles'],
      ),
      const FakeCommand(command: <String>['git', 'ls-remote', '--heads', 'mirror']),
      const FakeCommand(command: <String>['git', 'checkout', '-b', 'packages-autoroller-branch-1']),
      const FakeCommand(
        command: <String>[
          '$checkoutsParentDirectory/flutter_conductor_checkouts/framework/bin/flutter',
          '--verbose',
          'update-packages',
          '--force-upgrade',
        ],
      ),
      const FakeCommand(
        command: <String>['git', 'status', '--porcelain'],
        stdout: '''
 M packages/foo/pubspec.yaml
 M packages/bar/pubspec.yaml
 M dev/integration_tests/test_foo/pubspec.yaml
''',
      ),
      const FakeCommand(
        command: <String>['git', 'status', '--porcelain'],
        stdout: '''
 M packages/foo/pubspec.yaml
 M packages/bar/pubspec.yaml
 M dev/integration_tests/test_foo/pubspec.yaml
''',
      ),
      const FakeCommand(command: <String>['git', 'add', '--all']),
      const FakeCommand(
        command: <String>[
          'git',
          'commit',
          '--message',
          'roll packages',
          '--author="flutter-pub-roller-bot <flutter-pub-roller-bot@google.com>"',
        ],
      ),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: '000deadbeef'),
      const FakeCommand(
        command: <String>[
          '/.tmp_rand0/rand0/generate_gradle_lockfiles',
          '--no-gradle-generation',
          '--no-exclusion',
        ],
      ),
      const FakeCommand(
        command: <String>['git', 'status', '--porcelain'],
        stdout: '''
 M dev/integration_tests/ui/android/project-app.lockfile
 M examples/image_list/android/project-app.lockfile
''',
      ),
      const FakeCommand(
        command: <String>['git', 'status', '--porcelain'],
        stdout: '''
 M dev/integration_tests/ui/android/project-app.lockfile
 M examples/image_list/android/project-app.lockfile
''',
      ),
      const FakeCommand(command: <String>['git', 'add', '--all']),
      const FakeCommand(
        command: <String>[
          'git',
          'commit',
          '--message',
          'Re-generate Gradle lockfiles',
          '--author="flutter-pub-roller-bot <flutter-pub-roller-bot@google.com>"',
        ],
      ),
      const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: '234deadbeef'),
      const FakeCommand(
        command: <String>[
          'git',
          'push',
          'https://$token@github.com/$orgName/flutter.git',
          'packages-autoroller-branch-1:packages-autoroller-branch-1',
        ],
      ),
      const FakeCommand(
        command: <String>[
          'gh',
          'pr',
          'create',
          '--title',
          'Roll pub packages',
          '--body',
          'This PR was generated by `flutter update-packages --force-upgrade`.',
          '--head',
          'flutter-roller:packages-autoroller-branch-1',
          '--base',
          FrameworkRepository.defaultBranch,
          '--label',
          'tool',
          '--label',
          'autosubmit',
        ],
      ),
      const FakeCommand(command: <String>['gh', 'auth', 'logout', '--hostname', 'github.com']),
    ]);
    final Future<void> rollFuture = autoroller.roll();
    final String givenToken = await controller.stream.transform(const Utf8Decoder()).join();
    expect(givenToken.trim(), token);
    await rollFuture;
    expect(processManager, hasNoRemainingExpectations);
  });

  group('command argument validations', () {
    const String tokenPath = '/path/to/token';

    test('validates that file exists at --token option', () async {
      await expectLater(
        () => run(<String>['--token', tokenPath], fs: fileSystem, processManager: processManager),
        throwsA(
          isA<ArgumentError>().having(
            (ArgumentError err) => err.message,
            'message',
            contains('Provided token path $tokenPath but no file exists at'),
          ),
        ),
      );
      expect(processManager, hasNoRemainingExpectations);
    });

    test('validates that the token file is not empty', () async {
      fileSystem.file(tokenPath)
        ..createSync(recursive: true)
        ..writeAsStringSync('');
      await expectLater(
        () => run(<String>['--token', tokenPath], fs: fileSystem, processManager: processManager),
        throwsA(
          isA<ArgumentError>().having(
            (ArgumentError err) => err.message,
            'message',
            contains('Tried to read a GitHub access token from file $tokenPath but it was empty'),
          ),
        ),
      );
      expect(processManager, hasNoRemainingExpectations);
    });
  });

  test('VerboseStdio logger can filter out confidential pattern', () async {
    const String token = 'secret';
    const String replacement = 'replacement';
    final VerboseStdio stdio = VerboseStdio(
      stdin: _NoOpStdin(),
      stderr: _NoOpStdout(),
      stdout: _NoOpStdout(),
      filter: (String msg) => msg.replaceAll(token, replacement),
    );
    stdio.printStatus('Hello');
    expect(stdio.logs.last, '[status] Hello');

    stdio.printStatus('Using $token');
    expect(stdio.logs.last, '[status] Using $replacement');

    stdio.printWarning('Using $token');
    expect(stdio.logs.last, '[warning] Using $replacement');

    stdio.printError('Using $token');
    expect(stdio.logs.last, '[error] Using $replacement');

    stdio.printTrace('Using $token');
    expect(stdio.logs.last, '[trace] Using $replacement');
  });

  group('CheckoutStatePostGradleRegeneration', () {
    final Context ctx = Context(style: Style.posix);

    test('empty input returns NoDiff', () {
      expect(CheckoutStatePostGradleRegeneration('', ctx), const NoDiff());
    });

    test('only *.lockfile changes returns OnlyLockfileChanges', () {
      expect(
        CheckoutStatePostGradleRegeneration('''
 A dev/benchmarks/test_apps/stocks/android/buildscript-gradle.lockfile
 M dev/integration_tests/ui/android/project-app.lockfile
 M examples/image_list/android/project-app.lockfile
''', ctx),
        const OnlyLockfileChanges(),
      );
    });

    test('if a *.zip file is added returns NonLockfileChanges', () {
      const String pathToZip = 'dev/benchmarks/test_apps/stocks/android/very-large-archive.zip';
      CheckoutStatePostGradleRegeneration result = CheckoutStatePostGradleRegeneration('''
 A dev/benchmarks/test_apps/stocks/android/buildscript-gradle.lockfile
 A $pathToZip
 M dev/integration_tests/ui/android/project-app.lockfile
 M examples/image_list/android/project-app.lockfile
''', ctx);
      expect(result, isA<NonLockfileChanges>());
      result = result as NonLockfileChanges;
      expect(result.changes, hasLength(1));
      expect(result.changes.single, pathToZip);
    });

    test('if it contains a line not matching the regex returns MalformedLine', () {
      const String malformedLine = 'New Git Output.';
      CheckoutStatePostGradleRegeneration result = CheckoutStatePostGradleRegeneration('''
$malformedLine
 A dev/benchmarks/test_apps/stocks/android/buildscript-gradle.lockfile
 M dev/integration_tests/ui/android/project-app.lockfile
 M examples/image_list/android/project-app.lockfile
''', ctx);
      expect(result, isA<MalformedLine>());
      result = result as MalformedLine;
      expect(result.line, malformedLine);
    });
  });
}

class _NoOpStdin extends Fake implements io.Stdin {}

class _NoOpStdout extends Fake implements io.Stdout {
  @override
  void writeln([Object? object]) {}
}
