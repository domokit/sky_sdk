// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/runner.dart' as runner;
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/bot_detector.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart' as io;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/net.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/reporting/crash_reporting.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_http_client.dart';
import '../../src/fakes.dart';

const String kCustomBugInstructions = 'These are instructions to report with a custom bug tracker.';

void main() {
  group('runner (crash reporting)', () {
    int? firstExitCode;
    late MemoryFileSystem fileSystem;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      // Instead of exiting with dart:io exit(), this causes an exception to
      // be thrown, which we catch with the onError callback in the zone below.
      //
      // Tests might trigger exit() multiple times. In real life, exit() would
      // cause the VM to terminate immediately, so only the first one matters.
      firstExitCode = null;
      io.setExitFunctionForTests((int exitCode) {
        firstExitCode ??= exitCode;

        // TODO(jamesderlin): Ideally only the first call to exit() would be
        // honored and subsequent calls would be no-ops, but existing tests
        // rely on all calls to throw.
        throw Exception('test exit');
      });

      Cache.disableLocking();
      fileSystem = MemoryFileSystem.test();

      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: fileSystem,
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    tearDown(() {
      io.restoreExitFunction();
      Cache.enableLocking();
    });

    testUsingContext(
      'error handling crash report (synchronous crash)',
      () async {
        final Completer<void> completer = Completer<void>();
        // runner.run() asynchronously calls the exit function set above, so we
        // catch it in a zone.
        unawaited(
          runZoned<Future<void>?>(
            () {
              unawaited(
                runner.run(
                  <String>['crash'],
                  () => <FlutterCommand>[CrashingFlutterCommand()],
                  // This flutterVersion disables crash reporting.
                  flutterVersion: '[user-branch]/',
                  reportCrashes: true,
                  shutdownHooks: ShutdownHooks(),
                ),
              );
              return null;
            },
            onError: (Object error, StackTrace stack) {
              expect(firstExitCode, isNotNull);
              expect(firstExitCode, isNot(0));
              expect(error.toString(), 'Exception: test exit');
              completer.complete();
            },
          ),
        );
        await completer.future;

        // This is the main check of this test.
        //
        // We are checking that, even though crash reporting failed with an
        // exception on the first attempt, the second attempt tries to report the
        // *original* crash, and not the crash from the first crash report
        // attempt.
        expect(fakeAnalytics.sentEvents, contains(Event.exception(exception: '_Exception')));
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              environment: <String, String>{
                'FLUTTER_ANALYTICS_LOG_FILE': 'test',
                'FLUTTER_ROOT': '/',
              },
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Artifacts: () => Artifacts.test(),
        HttpClientFactory: () => () => FakeHttpClient.any(),
        Analytics: () => fakeAnalytics,
      },
    );

    // This Completer completes when CrashingFlutterCommand.runCommand
    // completes, but ideally we'd want it to complete when execution resumes
    // runner.run. Currently the distinction does not matter, but if it ever
    // does, this test might fail to catch a regression of
    // https://github.com/flutter/flutter/issues/56406.
    final Completer<void> commandCompleter = Completer<void>();
    testUsingContext(
      'error handling crash report (asynchronous crash)',
      () async {
        final Completer<void> completer = Completer<void>();
        // runner.run() asynchronously calls the exit function set above, so we
        // catch it in a zone.
        unawaited(
          runZoned<Future<void>?>(
            () {
              unawaited(
                runner.run(
                  <String>['crash'],
                  () => <FlutterCommand>[
                    CrashingFlutterCommand(asyncCrash: true, completer: commandCompleter),
                  ],
                  // This flutterVersion disables crash reporting.
                  flutterVersion: '[user-branch]/',
                  reportCrashes: true,
                  shutdownHooks: ShutdownHooks(),
                ),
              );
              return null;
            },
            onError: (Object error, StackTrace stack) {
              expect(firstExitCode, isNotNull);
              expect(firstExitCode, isNot(0));
              expect(error.toString(), 'Exception: test exit');
              completer.complete();
            },
          ),
        );
        await completer.future;
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              environment: <String, String>{
                'FLUTTER_ANALYTICS_LOG_FILE': 'test',
                'FLUTTER_ROOT': '/',
              },
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        CrashReporter: () => WaitingCrashReporter(commandCompleter.future),
        Artifacts: () => Artifacts.test(),
        HttpClientFactory: () => () => FakeHttpClient.any(),
      },
    );

    testUsingContext(
      'create local report',
      () async {
        // Since crash reporting calls the doctor, which checks for the devtools
        // version file in the cache, write a version file to the memory fs.
        Cache.flutterRoot = '/path/to/flutter';
        final Directory devtoolsDir = globals.fs.directory(
          '${Cache.flutterRoot}/bin/cache/dart-sdk/bin/resources/devtools',
        )..createSync(recursive: true);
        devtoolsDir.childFile('version.json').writeAsStringSync('{"version": "1.2.3"}');

        final Completer<void> completer = Completer<void>();
        // runner.run() asynchronously calls the exit function set above, so we
        // catch it in a zone.
        unawaited(
          runZoned<Future<void>?>(
            () {
              unawaited(
                runner.run(
                  <String>['crash'],
                  () => <FlutterCommand>[CrashingFlutterCommand()],
                  // This flutterVersion disables crash reporting.
                  flutterVersion: '[user-branch]/',
                  reportCrashes: true,
                  shutdownHooks: ShutdownHooks(),
                ),
              );
              return null;
            },
            onError: (Object error, StackTrace stack) {
              expect(firstExitCode, isNotNull);
              expect(firstExitCode, isNot(0));
              expect(error.toString(), 'Exception: test exit');
              completer.complete();
            },
          ),
        );
        await completer.future;

        final String errorText = testLogger.errorText;
        expect(
          errorText,
          containsIgnoringWhitespace(
            'Oops; flutter has exited unexpectedly: "Exception: an exception % --".\n',
          ),
        );

        final File log = globals.fs.file('/flutter_01.log');
        final String logContents = log.readAsStringSync();
        expect(logContents, contains(kCustomBugInstructions));
        expect(logContents, contains('flutter crash'));
        expect(logContents, contains('Exception: an exception % --'));
        expect(logContents, contains('CrashingFlutterCommand.runCommand'));
        expect(logContents, contains('[!] Flutter'));

        final CrashDetails sentDetails = (globals.crashReporter! as WaitingCrashReporter)._details;
        expect(sentDetails.command, 'flutter crash');
        expect(sentDetails.error.toString(), 'Exception: an exception % --');
        expect(sentDetails.stackTrace.toString(), contains('CrashingFlutterCommand.runCommand'));
        expect(await sentDetails.doctorText.text, contains('[!] Flutter'));
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              environment: <String, String>{
                'FLUTTER_ANALYTICS_LOG_FILE': 'test',
                'FLUTTER_ROOT': '/',
              },
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        UserMessages: () => CustomBugInstructions(),
        Artifacts: () => Artifacts.test(),
        CrashReporter: () => WaitingCrashReporter(Future<void>.value()),
        HttpClientFactory: () => () => FakeHttpClient.any(),
      },
    );

    group('in directory without permission', () {
      setUp(() {
        bool inTestSetup = true;
        fileSystem = MemoryFileSystem(
          opHandle: (String context, FileSystemOp operation) {
            if (inTestSetup) {
              // Allow all operations during test setup.
              return;
            }
            const Set<FileSystemOp> disallowedOperations = <FileSystemOp>{
              FileSystemOp.create,
              FileSystemOp.delete,
              FileSystemOp.copy,
              FileSystemOp.write,
            };
            // Make current_directory not writable.
            if (context.startsWith('/current_directory') &&
                disallowedOperations.contains(operation)) {
              throw FileSystemException(
                'No permission, context = $context, operation = $operation',
              );
            }
          },
        );
        final Directory currentDirectory = fileSystem.directory('/current_directory');
        currentDirectory.createSync();
        fileSystem.currentDirectory = currentDirectory;
        inTestSetup = false;
      });
      testUsingContext(
        'create local report in temporary directory',
        () async {
          // Since crash reporting calls the doctor, which checks for the devtools
          // version file in the cache, write a version file to the memory fs.
          Cache.flutterRoot = '/path/to/flutter';
          final Directory devtoolsDir = globals.fs.directory(
            '${Cache.flutterRoot}/bin/cache/dart-sdk/bin/resources/devtools',
          )..createSync(recursive: true);
          devtoolsDir.childFile('version.json').writeAsStringSync('{"version": "1.2.3"}');

          final Completer<void> completer = Completer<void>();
          // runner.run() asynchronously calls the exit function set above, so we
          // catch it in a zone.
          unawaited(
            runZoned<Future<void>?>(
              () {
                unawaited(
                  runner.run(
                    <String>['crash'],
                    () => <FlutterCommand>[CrashingFlutterCommand()],
                    // This flutterVersion disables crash reporting.
                    flutterVersion: '[user-branch]/',
                    reportCrashes: true,
                    shutdownHooks: ShutdownHooks(),
                  ),
                );
                return null;
              },
              onError: (Object error, StackTrace stack) {
                expect(firstExitCode, isNotNull);
                expect(firstExitCode, isNot(0));
                expect(error.toString(), 'Exception: test exit');
                completer.complete();
              },
            ),
          );
          await completer.future;

          final String errorText = testLogger.errorText;
          expect(
            errorText,
            containsIgnoringWhitespace(
              'Oops; flutter has exited unexpectedly: "Exception: an exception % --".\n',
            ),
          );

          final File log = globals.fs.systemTempDirectory.childFile('flutter_01.log');
          final String logContents = log.readAsStringSync();
          expect(logContents, contains(kCustomBugInstructions));
          expect(logContents, contains('flutter crash'));
          expect(logContents, contains('Exception: an exception % --'));
          expect(logContents, contains('CrashingFlutterCommand.runCommand'));
          expect(logContents, contains('[!] Flutter'));

          final CrashDetails sentDetails =
              (globals.crashReporter! as WaitingCrashReporter)._details;
          expect(sentDetails.command, 'flutter crash');
          expect(sentDetails.error.toString(), 'Exception: an exception % --');
          expect(sentDetails.stackTrace.toString(), contains('CrashingFlutterCommand.runCommand'));
          expect(await sentDetails.doctorText.text, contains('[!] Flutter'));
        },
        overrides: <Type, Generator>{
          Platform:
              () => FakePlatform(
                environment: <String, String>{
                  'FLUTTER_ANALYTICS_LOG_FILE': 'test',
                  'FLUTTER_ROOT': '/',
                },
              ),
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          UserMessages: () => CustomBugInstructions(),
          Artifacts: () => Artifacts.test(),
          CrashReporter: () => WaitingCrashReporter(Future<void>.value()),
          HttpClientFactory: () => () => FakeHttpClient.any(),
        },
      );
    });
  });

  group('runner', () {
    late MemoryFileSystem fs;

    setUp(() {
      io.setExitFunctionForTests((int exitCode) {});

      fs = MemoryFileSystem.test();

      Cache.disableLocking();
    });

    tearDown(() {
      io.restoreExitFunction();
      Cache.enableLocking();
    });

    testUsingContext(
      "catches ProcessException calling git because it's not available",
      () async {
        final _GitNotFoundFlutterCommand command = _GitNotFoundFlutterCommand();

        await runner.run(
          <String>[command.name],
          () => <FlutterCommand>[command],
          // This flutterVersion disables crash reporting.
          flutterVersion: '[user-branch]/',
          reportCrashes: false,
          shutdownHooks: ShutdownHooks(),
        );

        expect(
          (globals.logger as BufferLogger).errorText,
          'Failed to find "git" in the search path.\n'
          '\n'
          'An error was encountered when trying to run git.\n'
          "Please ensure git is installed and available in your system's search path. "
          'See https://docs.flutter.dev/get-started/install for instructions on installing git for your platform.\n',
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        Artifacts: () => Artifacts.test(),
        ProcessManager: () => FakeProcessManager.any()..excludedExecutables.add('git'),
      },
    );

    testUsingContext(
      'handles ProcessException calling git when ProcessManager.canRun fails',
      () async {
        final _GitNotFoundFlutterCommand command = _GitNotFoundFlutterCommand();

        await runner.run(
          <String>[command.name],
          () => <FlutterCommand>[command],
          // This flutterVersion disables crash reporting.
          flutterVersion: '[user-branch]/',
          reportCrashes: false,
          shutdownHooks: ShutdownHooks(),
        );

        expect(
          (globals.logger as BufferLogger).errorText,
          'Failed to find "git" in the search path.\n'
          '\n'
          'An error was encountered when trying to run git.\n'
          "Please ensure git is installed and available in your system's search path. "
          'See https://docs.flutter.dev/get-started/install for instructions on installing git for your platform.\n',
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        Artifacts: () => Artifacts.test(),
        ProcessManager: () => _ErrorOnCanRunFakeProcessManager(),
      },
    );

    testUsingContext(
      'do not print welcome on bots',
      () async {
        await runner.run(
          <String>['--version', '--machine'],
          () => <FlutterCommand>[],
          // This flutterVersion disables crash reporting.
          flutterVersion: '[user-branch]/',
          shutdownHooks: ShutdownHooks(),
        );

        expect(
          (globals.logger as BufferLogger).traceText,
          isNot(contains('Showed analytics consent message.')),
        );
      },
      overrides: <Type, Generator>{
        Logger: () => BufferLogger.test(),
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        BotDetector: () => const FakeBotDetector(true),
      },
    );
  });

  group('unified_analytics', () {
    late FakeAnalytics fakeAnalytics;
    late MemoryFileSystem fs;

    setUp(() {
      fs = MemoryFileSystem.test();

      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: fs,
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    testUsingContext(
      'runner disable telemetry with flag',
      () async {
        io.setExitFunctionForTests((int exitCode) {});

        expect(globals.analytics.telemetryEnabled, true);

        await runner.run(
          <String>['--disable-analytics'],
          () => <FlutterCommand>[],
          // This flutterVersion disables crash reporting.
          flutterVersion: '[user-branch]/',
          shutdownHooks: ShutdownHooks(),
        );

        expect(globals.analytics.telemetryEnabled, false);
      },
      overrides: <Type, Generator>{
        Analytics: () => fakeAnalytics,
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      '--enable-analytics and --disable-analytics enables/disables telemetry',
      () async {
        io.setExitFunctionForTests((int exitCode) {});

        expect(globals.analytics.telemetryEnabled, true);

        await runner.run(
          <String>['--disable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect(globals.analytics.telemetryEnabled, false);

        await runner.run(
          <String>['--enable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect(globals.analytics.telemetryEnabled, true);
      },
      overrides: <Type, Generator>{
        Analytics: () => fakeAnalytics,
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      '--enable-analytics and --disable-analytics send an event when telemetry is enabled/disabled',
      () async {
        io.setExitFunctionForTests((int exitCode) {});
        await globals.analytics.setTelemetry(true);

        await runner.run(
          <String>['--disable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect((globals.analytics as FakeAnalytics).sentEvents, <Event>[
          Event.analyticsCollectionEnabled(status: false),
        ]);

        (globals.analytics as FakeAnalytics).sentEvents.clear();
        expect(globals.analytics.telemetryEnabled, false);
        await runner.run(
          <String>['--enable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect((globals.analytics as FakeAnalytics).sentEvents, <Event>[
          Event.analyticsCollectionEnabled(status: true),
        ]);
      },
      overrides: <Type, Generator>{
        Analytics: () => fakeAnalytics,
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      '--enable-analytics and --disable-analytics do not send an event when telemetry is already enabled/disabled',
      () async {
        io.setExitFunctionForTests((int exitCode) {});

        await globals.analytics.setTelemetry(false);
        await runner.run(
          <String>['--disable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect((globals.analytics as FakeAnalytics).sentEvents, isEmpty);

        await globals.analytics.setTelemetry(true);
        await runner.run(
          <String>['--enable-analytics'],
          () => <FlutterCommand>[],
          shutdownHooks: ShutdownHooks(),
        );

        expect((globals.analytics as FakeAnalytics).sentEvents, isEmpty);
      },
      overrides: <Type, Generator>{
        Analytics: () => fakeAnalytics,
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'throw error when both flags passed',
      () async {
        io.setExitFunctionForTests((int exitCode) {});

        expect(globals.analytics.telemetryEnabled, true);

        final int exitCode = await runner.run(
          <String>['--disable-analytics', '--enable-analytics'],
          () => <FlutterCommand>[],
          // This flutterVersion disables crash reporting.
          flutterVersion: '[user-branch]/',
          shutdownHooks: ShutdownHooks(),
        );

        expect(exitCode, 1, reason: 'Should return 1 due to conflicting options for telemetry');
        expect(
          globals.analytics.telemetryEnabled,
          true,
          reason: 'Should not have changed from initialization',
        );
      },
      overrides: <Type, Generator>{
        Analytics: () => fakeAnalytics,
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });
}

class CrashingFlutterCommand extends FlutterCommand {
  CrashingFlutterCommand({bool asyncCrash = false, Completer<void>? completer})
    : _asyncCrash = asyncCrash,
      _completer = completer;

  final bool _asyncCrash;
  final Completer<void>? _completer;

  @override
  String get description => '';

  @override
  String get name => 'crash';

  @override
  Future<FlutterCommandResult> runCommand() async {
    final Exception error = Exception('an exception % --'); // Test URL encoding.
    if (!_asyncCrash) {
      throw error;
    }

    final Completer<void> completer = Completer<void>();
    Timer.run(() {
      completer.complete();
      throw error;
    });

    await completer.future;
    _completer!.complete();

    return FlutterCommandResult.success();
  }
}

class _GitNotFoundFlutterCommand extends FlutterCommand {
  @override
  String get description => '';

  @override
  String get name => 'git-not-found';

  @override
  Future<FlutterCommandResult> runCommand() {
    throw const io.ProcessException('git', <String>[
      'log',
    ], 'Failed to find "git" in the search path.');
  }
}

class CustomBugInstructions extends UserMessages {
  @override
  String get flutterToolBugInstructions => kCustomBugInstructions;
}

/// A fake [CrashReporter] that waits for a [Future] to complete.
///
/// Used to exacerbate a race between the success and failure paths of
/// [runner.run]. See https://github.com/flutter/flutter/issues/56406.
class WaitingCrashReporter implements CrashReporter {
  WaitingCrashReporter(Future<void> future) : _future = future;

  final Future<void> _future;
  late CrashDetails _details;

  @override
  Future<void> informUser(CrashDetails details, File crashFile) {
    _details = details;
    return _future;
  }
}

class _ErrorOnCanRunFakeProcessManager extends Fake implements FakeProcessManager {
  final FakeProcessManager delegate = FakeProcessManager.any();
  @override
  bool canRun(dynamic executable, {String? workingDirectory}) {
    if (executable == 'git') {
      throw Exception("oh no, we couldn't check for git!");
    }
    return delegate.canRun(executable, workingDirectory: workingDirectory);
  }
}
