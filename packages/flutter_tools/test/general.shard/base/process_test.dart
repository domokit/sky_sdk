// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/terminal.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  group('process exceptions', () {
    late FakeProcessManager fakeProcessManager;
    late ProcessUtils processUtils;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      processUtils = ProcessUtils(processManager: fakeProcessManager, logger: BufferLogger.test());
    });

    testWithoutContext(
      'runAsync throwOnError: exceptions should be ProcessException objects',
      () async {
        fakeProcessManager.addCommand(const FakeCommand(command: <String>['false'], exitCode: 1));

        expect(
          () async => processUtils.run(<String>['false'], throwOnError: true),
          throwsProcessException(message: 'Process exited abnormally with exit code 1'),
        );
      },
    );
  });

  group('shutdownHooks', () {
    testWithoutContext('runInExpectedOrder', () async {
      int i = 1;
      int? cleanup;

      final ShutdownHooks shutdownHooks = ShutdownHooks();

      shutdownHooks.addShutdownHook(() async {
        cleanup = i++;
      });

      await shutdownHooks.runShutdownHooks(BufferLogger.test());

      expect(cleanup, 1);
    });
  });

  group('output formatting', () {
    late FakeProcessManager processManager;
    late ProcessUtils processUtils;
    late BufferLogger logger;

    setUp(() {
      processManager = FakeProcessManager.empty();
      logger = BufferLogger.test();
      processUtils = ProcessUtils(processManager: processManager, logger: logger);
    });

    testWithoutContext('Command output is not wrapped.', () async {
      final List<String> testString = <String>['0123456789' * 10];
      processManager.addCommand(
        FakeCommand(
          command: const <String>['command'],
          stdout: testString.join(),
          stderr: testString.join(),
        ),
      );

      await processUtils.stream(<String>['command']);

      expect(logger.statusText, equals('${testString[0]}\n'));
      expect(logger.errorText, equals('${testString[0]}\n'));
    });

    testWithoutContext('Command output is filtered by mapFunction', () async {
      processManager.addCommand(
        const FakeCommand(
          command: <String>['command'],
          stdout: 'match\nno match',
          stderr: 'match\nno match',
        ),
      );

      await processUtils.stream(
        <String>['command'],
        mapFunction: (String line) {
          if (line == 'match') {
            return line;
          }
          return null;
        },
      );

      expect(logger.statusText, equals('match\n'));
      expect(logger.errorText, equals('match\n'));
    });
  });

  group('run', () {
    late FakeProcessManager fakeProcessManager;
    late ProcessUtils processUtils;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      processUtils = ProcessUtils(processManager: fakeProcessManager, logger: BufferLogger.test());
    });

    testWithoutContext(' succeeds on success', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['whoohoo']));
      expect((await processUtils.run(<String>['whoohoo'])).exitCode, 0);
    });

    testWithoutContext(' fails on failure', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['boohoo'], exitCode: 1));
      expect((await processUtils.run(<String>['boohoo'])).exitCode, 1);
    });

    testWithoutContext(' throws on failure with throwOnError', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['kaboom'], exitCode: 1));
      expect(
        () => processUtils.run(<String>['kaboom'], throwOnError: true),
        throwsProcessException(),
      );
    });

    testWithoutContext(' does not throw on allowed Failures', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['kaboom'], exitCode: 1));
      expect(
        (await processUtils.run(
          <String>['kaboom'],
          throwOnError: true,
          allowedFailures: (int c) => c == 1,
        )).exitCode,
        1,
      );
    });

    testWithoutContext(' throws on disallowed failure', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['kaboom'], exitCode: 2));
      expect(
        () => processUtils.run(
          <String>['kaboom'],
          throwOnError: true,
          allowedFailures: (int c) => c == 1,
        ),
        throwsProcessException(),
      );
    });
  });

  group('runSync', () {
    late FakeProcessManager fakeProcessManager;
    late ProcessUtils processUtils;
    late BufferLogger testLogger;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      testLogger = BufferLogger(
        terminal: AnsiTerminal(stdio: FakeStdio(), platform: FakePlatform()),
        outputPreferences: OutputPreferences(wrapText: true, wrapColumn: 40),
      );
      processUtils = ProcessUtils(processManager: fakeProcessManager, logger: testLogger);
    });

    testWithoutContext(' succeeds on success', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['whoohoo']));
      expect(processUtils.runSync(<String>['whoohoo']).exitCode, 0);
    });

    testWithoutContext(' fails on failure', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['boohoo'], exitCode: 1));
      expect(processUtils.runSync(<String>['boohoo']).exitCode, 1);
    });

    testWithoutContext('throws on failure with throwOnError', () async {
      const String stderr = 'Something went wrong.';
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['kaboom'], exitCode: 1, stderr: stderr),
      );
      expect(
        () => processUtils.runSync(<String>['kaboom'], throwOnError: true),
        throwsA(
          isA<ProcessException>().having(
            (ProcessException error) => error.message,
            'message',
            isNot(contains(stderr)),
          ),
        ),
      );
    });

    testWithoutContext(
      'throws with stderr in exception on failure with verboseExceptions',
      () async {
        const String stderr = 'Something went wrong.';
        fakeProcessManager.addCommand(
          const FakeCommand(command: <String>['verybad'], exitCode: 1, stderr: stderr),
        );
        expect(
          () => processUtils.runSync(
            <String>['verybad'],
            throwOnError: true,
            verboseExceptions: true,
          ),
          throwsProcessException(message: stderr),
        );
      },
    );

    testWithoutContext(' does not throw on allowed Failures', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['kaboom'], exitCode: 1));
      expect(
        processUtils
            .runSync(<String>['kaboom'], throwOnError: true, allowedFailures: (int c) => c == 1)
            .exitCode,
        1,
      );
    });

    testWithoutContext(' throws on disallowed failure', () async {
      fakeProcessManager.addCommand(const FakeCommand(command: <String>['kaboom'], exitCode: 2));
      expect(
        () => processUtils.runSync(
          <String>['kaboom'],
          throwOnError: true,
          allowedFailures: (int c) => c == 1,
        ),
        throwsProcessException(),
      );
    });

    testWithoutContext(' prints stdout and stderr to trace on success', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['whoohoo'], stdout: 'stdout', stderr: 'stderr'),
      );
      expect(processUtils.runSync(<String>['whoohoo']).exitCode, 0);
      expect(testLogger.traceText, contains('stdout'));
      expect(testLogger.traceText, contains('stderr'));
    });

    testWithoutContext(
      ' prints stdout to status and stderr to error on failure with throwOnError',
      () async {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['kaboom'],
            exitCode: 1,
            stdout: 'stdout',
            stderr: 'stderr',
          ),
        );
        expect(
          () => processUtils.runSync(<String>['kaboom'], throwOnError: true),
          throwsProcessException(),
        );
        expect(testLogger.statusText, contains('stdout'));
        expect(testLogger.errorText, contains('stderr'));
      },
    );

    testWithoutContext(' does not print stdout with hideStdout', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['whoohoo'], stdout: 'stdout', stderr: 'stderr'),
      );
      expect(processUtils.runSync(<String>['whoohoo'], hideStdout: true).exitCode, 0);
      expect(testLogger.traceText.contains('stdout'), isFalse);
      expect(testLogger.traceText, contains('stderr'));
    });
  });

  group('exitsHappySync', () {
    late FakeProcessManager processManager;
    late ProcessUtils processUtils;

    setUp(() {
      processManager = FakeProcessManager.empty();
      processUtils = ProcessUtils(processManager: processManager, logger: BufferLogger.test());
    });

    testWithoutContext('succeeds on success', () async {
      processManager.addCommand(const FakeCommand(command: <String>['whoohoo']));

      expect(processUtils.exitsHappySync(<String>['whoohoo']), isTrue);
    });

    testWithoutContext('fails on failure', () async {
      processManager.addCommand(const FakeCommand(command: <String>['boohoo'], exitCode: 1));

      expect(processUtils.exitsHappySync(<String>['boohoo']), isFalse);
    });

    testWithoutContext('catches Exception and returns false', () {
      processManager.addCommand(
        const FakeCommand(
          command: <String>['boohoo'],
          exception: ProcessException('Process failed', <String>[]),
        ),
      );

      expect(processUtils.exitsHappySync(<String>['boohoo']), isFalse);
    });

    testWithoutContext('does not throw Exception and returns false if binary cannot run', () {
      processManager.excludedExecutables.add('nonesuch');

      expect(processUtils.exitsHappySync(<String>['nonesuch']), isFalse);
    });

    testWithoutContext('does not catch ArgumentError', () async {
      processManager.addCommand(
        FakeCommand(command: const <String>['invalid'], exception: ArgumentError('Bad input')),
      );

      expect(() => processUtils.exitsHappySync(<String>['invalid']), throwsArgumentError);
    });
  });

  group('exitsHappy', () {
    late FakeProcessManager processManager;
    late ProcessUtils processUtils;

    setUp(() {
      processManager = FakeProcessManager.empty();
      processUtils = ProcessUtils(processManager: processManager, logger: BufferLogger.test());
    });

    testWithoutContext('succeeds on success', () async {
      processManager.addCommand(const FakeCommand(command: <String>['whoohoo']));

      expect(await processUtils.exitsHappy(<String>['whoohoo']), isTrue);
    });

    testWithoutContext('fails on failure', () async {
      processManager.addCommand(const FakeCommand(command: <String>['boohoo'], exitCode: 1));

      expect(await processUtils.exitsHappy(<String>['boohoo']), isFalse);
    });

    testWithoutContext('catches Exception and returns false', () async {
      processManager.addCommand(
        const FakeCommand(
          command: <String>['boohoo'],
          exception: ProcessException('Process failed', <String>[]),
        ),
      );

      expect(await processUtils.exitsHappy(<String>['boohoo']), isFalse);
    });

    testWithoutContext('does not throw Exception and returns false if binary cannot run', () async {
      processManager.excludedExecutables.add('nonesuch');

      expect(await processUtils.exitsHappy(<String>['nonesuch']), isFalse);
    });

    testWithoutContext('does not catch ArgumentError', () async {
      processManager.addCommand(
        FakeCommand(command: const <String>['invalid'], exception: ArgumentError('Bad input')),
      );

      expect(() async => processUtils.exitsHappy(<String>['invalid']), throwsArgumentError);
    });
  });

  group('writeToStdinGuarded', () {
    testWithoutContext('handles any error thrown by stdin.flush', () async {
      final _ThrowsOnFlushIOSink stdin = _ThrowsOnFlushIOSink();
      Object? errorPassedToCallback;

      await ProcessUtils.writeToStdinGuarded(
        stdin: stdin,
        content: 'message to stdin',
        onError: (Object error, StackTrace stackTrace) {
          errorPassedToCallback = error;
        },
      );

      expect(
        errorPassedToCallback,
        isNotNull,
        reason: 'onError callback should have been invoked.',
      );

      expect(errorPassedToCallback, const TypeMatcher<SocketException>());
    });
  });
}

class _ThrowsOnFlushIOSink extends MemoryIOSink {
  @override
  Future<Object?> flush() async {
    throw const SocketException('Write failed', osError: OSError('Broken pipe', 32));
  }
}
