// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/executable.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/commands/daemon.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

final Platform _kNoAnsiPlatform = FakePlatform();
final String red = RegExp.escape(AnsiTerminal.red);
final String bold = RegExp.escape(AnsiTerminal.bold);
final String resetBold = RegExp.escape(AnsiTerminal.resetBold);
final String resetColor = RegExp.escape(AnsiTerminal.resetColor);

void main() {
  testWithoutContext('correct logger instance is created', () {
    final LoggerFactory loggerFactory = LoggerFactory(
      terminal: Terminal.test(),
      stdio: FakeStdio(),
      outputPreferences: OutputPreferences.test(),
    );

    expect(
      loggerFactory.createLogger(
        verbose: false,
        prefixedErrors: false,
        machine: false,
        daemon: false,
        windows: false,
      ),
      isA<StdoutLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: false,
        prefixedErrors: false,
        machine: false,
        daemon: false,
        windows: true,
      ),
      isA<WindowsStdoutLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: true,
        prefixedErrors: false,
        machine: false,
        daemon: false,
        windows: true,
      ),
      isA<VerboseLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: true,
        prefixedErrors: false,
        machine: false,
        daemon: false,
        windows: false,
      ),
      isA<VerboseLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: false,
        prefixedErrors: true,
        machine: false,
        daemon: false,
        windows: false,
      ),
      isA<PrefixedErrorLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: false,
        prefixedErrors: false,
        machine: false,
        daemon: true,
        windows: false,
      ),
      isA<NotifyingLogger>(),
    );
    expect(
      loggerFactory.createLogger(
        verbose: false,
        prefixedErrors: false,
        machine: true,
        daemon: false,
        windows: false,
      ),
      isA<AppRunLogger>(),
    );
  });

  testWithoutContext(
    'WindowsStdoutLogger rewrites emojis when terminal does not support emoji',
    () {
      final FakeStdio stdio = FakeStdio();
      final WindowsStdoutLogger logger = WindowsStdoutLogger(
        outputPreferences: OutputPreferences.test(),
        stdio: stdio,
        terminal: Terminal.test(),
      );

      logger.printStatus('🔥🖼️✗✓🔨💪✏️');

      expect(stdio.writtenToStdout, <String>['X√\n']);
    },
  );

  testWithoutContext(
    'WindowsStdoutLogger does not rewrite emojis when terminal does support emoji',
    () {
      final FakeStdio stdio = FakeStdio();
      final WindowsStdoutLogger logger = WindowsStdoutLogger(
        outputPreferences: OutputPreferences.test(),
        stdio: stdio,
        terminal: Terminal.test(supportsColor: true, supportsEmoji: true),
      );

      logger.printStatus('🔥🖼️✗✓🔨💪✏️');

      expect(stdio.writtenToStdout, <String>['🔥🖼️✗✓🔨💪✏️\n']);
    },
  );

  testWithoutContext('DelegatingLogger delegates', () {
    final FakeLogger fakeLogger = FakeLogger();
    final DelegatingLogger delegatingLogger = DelegatingLogger(fakeLogger);

    expect(() => delegatingLogger.quiet, _throwsInvocationFor(() => fakeLogger.quiet));

    expect(
      () => delegatingLogger.quiet = true,
      _throwsInvocationFor(() => fakeLogger.quiet = true),
    );

    expect(() => delegatingLogger.hasTerminal, _throwsInvocationFor(() => fakeLogger.hasTerminal));

    expect(() => delegatingLogger.isVerbose, _throwsInvocationFor(() => fakeLogger.isVerbose));

    const String message = 'message';
    final StackTrace stackTrace = StackTrace.current;
    const bool emphasis = true;
    const TerminalColor color = TerminalColor.cyan;
    const int indent = 88;
    const int hangingIndent = 52;
    const bool wrap = true;
    const bool newline = true;
    expect(
      () => delegatingLogger.printError(
        message,
        stackTrace: stackTrace,
        emphasis: emphasis,
        color: color,
        indent: indent,
        hangingIndent: hangingIndent,
        wrap: wrap,
      ),
      _throwsInvocationFor(
        () => fakeLogger.printError(
          message,
          stackTrace: stackTrace,
          emphasis: emphasis,
          color: color,
          indent: indent,
          hangingIndent: hangingIndent,
          wrap: wrap,
        ),
      ),
    );

    expect(
      () => delegatingLogger.printStatus(
        message,
        emphasis: emphasis,
        color: color,
        newline: newline,
        indent: indent,
        hangingIndent: hangingIndent,
        wrap: wrap,
      ),
      _throwsInvocationFor(
        () => fakeLogger.printStatus(
          message,
          emphasis: emphasis,
          color: color,
          newline: newline,
          indent: indent,
          hangingIndent: hangingIndent,
          wrap: wrap,
        ),
      ),
    );

    expect(
      () => delegatingLogger.printTrace(message),
      _throwsInvocationFor(() => fakeLogger.printTrace(message)),
    );

    final Map<String, dynamic> eventArgs = <String, dynamic>{};
    expect(
      () => delegatingLogger.sendEvent(message, eventArgs),
      _throwsInvocationFor(() => fakeLogger.sendEvent(message, eventArgs)),
    );

    const String progressId = 'progressId';
    const int progressIndicatorPadding = kDefaultStatusPadding * 2;
    expect(
      () => delegatingLogger.startProgress(
        message,
        progressId: progressId,
        progressIndicatorPadding: progressIndicatorPadding,
      ),
      _throwsInvocationFor(
        () => fakeLogger.startProgress(
          message,
          progressId: progressId,
          progressIndicatorPadding: progressIndicatorPadding,
        ),
      ),
    );

    expect(
      () => delegatingLogger.supportsColor,
      _throwsInvocationFor(() => fakeLogger.supportsColor),
    );

    expect(() => delegatingLogger.clear(), _throwsInvocationFor(() => fakeLogger.clear()));
  });

  testWithoutContext('asLogger finds the correct delegate', () async {
    final FakeLogger fakeLogger = FakeLogger();
    final VerboseLogger verboseLogger = VerboseLogger(fakeLogger);
    final NotifyingLogger notifyingLogger = NotifyingLogger(verbose: true, parent: verboseLogger);
    expect(asLogger<Logger>(notifyingLogger), notifyingLogger);
    expect(asLogger<NotifyingLogger>(notifyingLogger), notifyingLogger);
    expect(asLogger<VerboseLogger>(notifyingLogger), verboseLogger);
    expect(asLogger<FakeLogger>(notifyingLogger), fakeLogger);

    expect(() => asLogger<AppRunLogger>(notifyingLogger), throwsStateError);
  });

  group('AppContext', () {
    late FakeStopwatch fakeStopWatch;

    setUp(() {
      fakeStopWatch = FakeStopwatch();
    });

    testWithoutContext('error', () async {
      final BufferLogger mockLogger = BufferLogger.test(
        outputPreferences: OutputPreferences.test(),
      );
      final VerboseLogger verboseLogger = VerboseLogger(
        mockLogger,
        stopwatchFactory: FakeStopwatchFactory(stopwatch: fakeStopWatch),
      );

      verboseLogger.printStatus('Hey Hey Hey Hey');
      verboseLogger.printTrace('Oooh, I do I do I do');
      final StackTrace stackTrace = StackTrace.current;
      verboseLogger.printError('Helpless!', stackTrace: stackTrace);

      expect(
        mockLogger.statusText,
        matches(
          r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] Hey Hey Hey Hey\n'
          r'\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] Oooh, I do I do I do\n$',
        ),
      );
      expect(mockLogger.traceText, '');
      expect(mockLogger.errorText, matches(r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] Helpless!\n'));
      final String lastLine = LineSplitter.split(stackTrace.toString()).toList().last;
      expect(mockLogger.errorText, endsWith('$lastLine\n\n'));
    });

    testWithoutContext('ANSI colored errors', () async {
      final BufferLogger mockLogger = BufferLogger(
        terminal: AnsiTerminal(
          stdio: FakeStdio(),
          platform: FakePlatform(stdoutSupportsAnsi: true),
        ),
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      final VerboseLogger verboseLogger = VerboseLogger(
        mockLogger,
        stopwatchFactory: FakeStopwatchFactory(stopwatch: fakeStopWatch),
      );

      verboseLogger.printStatus('Hey Hey Hey Hey');
      verboseLogger.printTrace('Oooh, I do I do I do');
      verboseLogger.printError('Helpless!');

      expect(
        mockLogger.statusText,
        matches(
          r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] '
          '${bold}Hey Hey Hey Hey$resetBold'
          r'\n\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] Oooh, I do I do I do\n$',
        ),
      );
      expect(mockLogger.traceText, '');
      expect(
        mockLogger.errorText,
        matches(
          '^$red'
          r'\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] '
          '${bold}Helpless!$resetBold$resetColor'
          r'\n$',
        ),
      );
    });

    testWithoutContext('printBox', () {
      final BufferLogger mockLogger = BufferLogger(
        terminal: AnsiTerminal(
          stdio: FakeStdio(),
          platform: FakePlatform(stdoutSupportsAnsi: true),
        ),
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      final VerboseLogger verboseLogger = VerboseLogger(
        mockLogger,
        stopwatchFactory: FakeStopwatchFactory(stopwatch: fakeStopWatch),
      );

      verboseLogger.printBox('This is the box message', title: 'Sample title');

      expect(
        mockLogger.statusText,
        contains(
          '[        ] \x1B[1m\x1B[22m\n'
          '\x1B[1m           ┌─ Sample title ──────────┐\x1B[22m\n'
          '\x1B[1m           │ This is the box message │\x1B[22m\n'
          '\x1B[1m           └─────────────────────────┘\x1B[22m\n'
          '\x1B[1m           \x1B[22m\n',
        ),
      );
    });
  });

  testWithoutContext('Logger does not throw when stdio write throws synchronously', () async {
    final FakeStdout stdout = FakeStdout(syncError: true);
    final FakeStdout stderr = FakeStdout(syncError: true);
    final Stdio stdio = Stdio.test(stdout: stdout, stderr: stderr);
    final Logger logger = StdoutLogger(
      terminal: AnsiTerminal(stdio: stdio, platform: _kNoAnsiPlatform),
      stdio: stdio,
      outputPreferences: OutputPreferences.test(),
    );

    logger.printStatus('message');
    logger.printError('error message');
  });

  testWithoutContext('Logger does not throw when stdio write throws asynchronously', () async {
    final FakeStdout stdout = FakeStdout(syncError: false);
    final FakeStdout stderr = FakeStdout(syncError: false);
    final Stdio stdio = Stdio.test(stdout: stdout, stderr: stderr);
    final Logger logger = StdoutLogger(
      terminal: AnsiTerminal(stdio: stdio, platform: _kNoAnsiPlatform),
      stdio: stdio,
      outputPreferences: OutputPreferences.test(),
    );
    logger.printStatus('message');
    logger.printError('error message');

    await stdout.done;
    await stderr.done;
  });

  testWithoutContext('Logger does not throw when stdio completes done with an error', () async {
    final FakeStdout stdout = FakeStdout(syncError: false, completeWithError: true);
    final FakeStdout stderr = FakeStdout(syncError: false, completeWithError: true);
    final Stdio stdio = Stdio.test(stdout: stdout, stderr: stderr);
    final Logger logger = StdoutLogger(
      terminal: AnsiTerminal(stdio: stdio, platform: _kNoAnsiPlatform),
      stdio: stdio,
      outputPreferences: OutputPreferences.test(),
    );
    logger.printStatus('message');
    logger.printError('error message');

    expect(() async => stdout.done, throwsException);
    expect(() async => stderr.done, throwsException);
  });

  group('Spinners', () {
    late FakeStdio mockStdio;
    late FakeStopwatch mockStopwatch;
    late FakeStopwatchFactory stopwatchFactory;
    late int called;
    final List<Platform> testPlatforms = <Platform>[
      FakePlatform(environment: <String, String>{}, executableArguments: <String>[]),
      FakePlatform(
        operatingSystem: 'macos',
        environment: <String, String>{},
        executableArguments: <String>[],
      ),
      FakePlatform(
        operatingSystem: 'windows',
        environment: <String, String>{},
        executableArguments: <String>[],
      ),
      FakePlatform(
        operatingSystem: 'windows',
        environment: <String, String>{'WT_SESSION': ''},
        executableArguments: <String>[],
      ),
      FakePlatform(
        operatingSystem: 'fuchsia',
        environment: <String, String>{},
        executableArguments: <String>[],
      ),
    ];
    final RegExp secondDigits = RegExp(r'[0-9,.]*[0-9]m?s');

    setUp(() {
      mockStopwatch = FakeStopwatch();
      mockStdio = FakeStdio();
      called = 0;
      stopwatchFactory = FakeStopwatchFactory(stopwatch: mockStopwatch);
    });

    List<String> outputStdout() => mockStdio.writtenToStdout.join().split('\n');
    List<String> outputStderr() => mockStdio.writtenToStderr.join().split('\n');

    void doWhileAsync(FakeAsync time, bool Function() doThis) {
      do {
        mockStopwatch.elapsed += const Duration(milliseconds: 1);
        time.elapse(const Duration(milliseconds: 1));
      } while (doThis());
    }

    for (final Platform testPlatform in testPlatforms) {
      group('(${testPlatform.operatingSystem})', () {
        late Platform platform;
        late Platform ansiPlatform;
        late AnsiTerminal terminal;
        late AnsiTerminal coloredTerminal;
        late SpinnerStatus spinnerStatus;

        setUp(() {
          platform = FakePlatform();
          ansiPlatform = FakePlatform(stdoutSupportsAnsi: true);

          terminal = AnsiTerminal(stdio: mockStdio, platform: platform);
          coloredTerminal = AnsiTerminal(stdio: mockStdio, platform: ansiPlatform);

          spinnerStatus = SpinnerStatus(
            message: 'Hello world',
            padding: 20,
            onFinish: () => called += 1,
            stdio: mockStdio,
            stopwatch: stopwatchFactory.createStopwatch(),
            terminal: terminal,
          );
        });

        testWithoutContext('AnonymousSpinnerStatus works (1)', () async {
          bool done = false;
          mockStopwatch = FakeStopwatch();
          FakeAsync().run((FakeAsync time) {
            final AnonymousSpinnerStatus spinner = AnonymousSpinnerStatus(
              stdio: mockStdio,
              stopwatch: mockStopwatch,
              terminal: terminal,
            )..start();
            doWhileAsync(time, () => spinner.ticks < 10);
            List<String> lines = outputStdout();
            expect(
              lines[0],
              startsWith(
                terminal.supportsEmoji
                    ? '⣽\b⣻\b⢿\b⡿\b⣟\b⣯\b⣷\b⣾\b⣽\b⣻'
                    : '\\\b|\b/\b-\b\\\b|\b/\b-',
              ),
            );
            expect(lines[0].endsWith('\n'), isFalse);
            expect(lines.length, equals(1));

            spinner.stop();
            lines = outputStdout();

            expect(lines[0], endsWith('\b \b'));
            expect(lines.length, equals(1));

            // Verify that stopping or canceling multiple times throws.
            expect(spinner.stop, throwsAssertionError);
            expect(spinner.cancel, throwsAssertionError);
            done = true;
          });
          expect(done, isTrue);
        });

        testWithoutContext(
          'AnonymousSpinnerStatus logs warning after timeout without color support',
          () async {
            mockStopwatch = FakeStopwatch();
            const String warningMessage = 'a warning message.';
            final bool done = FakeAsync().run<bool>((FakeAsync time) {
              final AnonymousSpinnerStatus spinner = AnonymousSpinnerStatus(
                stdio: mockStdio,
                stopwatch: mockStopwatch,
                terminal: terminal,
                slowWarningCallback: () => warningMessage,
                warningColor: TerminalColor.red,
                timeout: const Duration(milliseconds: 100),
              )..start();
              // must be greater than the spinner timer duration
              const Duration timeLapse = Duration(milliseconds: 101);
              mockStopwatch.elapsed += timeLapse;
              time.elapse(timeLapse);

              List<String> lines = outputStdout();
              expect(lines.join().contains(RegExp(red)), isFalse);
              expect(lines.join(), '⣽\ba warning message.⣻');

              spinner.stop();
              lines = outputStdout();
              return true;
            });
            expect(done, isTrue);
          },
        );

        testWithoutContext(
          'AnonymousSpinnerStatus logs warning after timeout with color support',
          () async {
            mockStopwatch = FakeStopwatch();
            const String warningMessage = 'a warning message.';
            final bool done = FakeAsync().run<bool>((FakeAsync time) {
              final AnonymousSpinnerStatus spinner = AnonymousSpinnerStatus(
                stdio: mockStdio,
                stopwatch: mockStopwatch,
                terminal: coloredTerminal,
                slowWarningCallback: () => warningMessage,
                warningColor: TerminalColor.red,
                timeout: const Duration(milliseconds: 100),
              )..start();
              // must be greater than the spinner timer duration
              const Duration timeLapse = Duration(milliseconds: 101);
              mockStopwatch.elapsed += timeLapse;
              time.elapse(timeLapse);

              List<String> lines = outputStdout();
              expect(lines.join().contains(RegExp(red)), isTrue);
              expect(
                lines.join(),
                '⣽\b${AnsiTerminal.red}a warning message.${AnsiTerminal.resetColor}⣻',
              );
              expect(lines.join(), matches('$red$warningMessage$resetColor'));

              spinner.stop();
              lines = outputStdout();
              return true;
            });
            expect(done, isTrue);
          },
        );

        testWithoutContext('Stdout startProgress on colored terminal', () async {
          final Logger logger = StdoutLogger(
            terminal: coloredTerminal,
            stdio: mockStdio,
            outputPreferences: OutputPreferences.test(showColor: true),
            stopwatchFactory: stopwatchFactory,
          );
          final Status status = logger.startProgress(
            'Hello',
            progressIndicatorPadding: 20, // this minus the "Hello" equals the 15 below.
          );
          expect(outputStderr().length, equals(1));
          expect(outputStderr().first, isEmpty);
          // the 4 below is the margin that is always included between the message and the time.
          // the 8 below is the space left for the time.
          expect(
            outputStdout().join('\n'),
            matches(terminal.supportsEmoji ? r'^Hello {15} {4} {8}⣽$' : r'^Hello {15} {4} {8}\\$'),
          );
          mockStopwatch.elapsed = const Duration(seconds: 4, milliseconds: 100);
          status.stop();
          expect(
            outputStdout().join('\n'),
            matches(
              terminal.supportsEmoji
                  ? r'^Hello {15} {4} {8}⣽[\b] [\b]{8}[\d, ]{4}[\d]\.[\d]s[\n]$'
                  : r'^Hello {15} {4} {8}\\[\b] [\b]{8}[\d, ]{4}[\d]\.[\d]s[\n]$',
            ),
          );
        });

        testWithoutContext('Stdout startProgress on colored terminal pauses', () async {
          bool done = false;
          FakeAsync().run((FakeAsync time) {
            mockStopwatch.elapsed = const Duration(seconds: 5);
            final Logger logger = StdoutLogger(
              terminal: coloredTerminal,
              stdio: mockStdio,
              outputPreferences: OutputPreferences.test(showColor: true),
              stopwatchFactory: stopwatchFactory,
            );
            const String message = "Knock Knock, Who's There";
            final Status status = logger.startProgress(
              message,
              progressIndicatorPadding: 10, // ignored
            );
            logger.printStatus('Rude Interrupting Cow');
            status.stop();
            final String a = terminal.supportsEmoji ? '⣽' : r'\';
            final String b = terminal.supportsEmoji ? '⣻' : '|';
            const String blankLine = '\r\x1B[K';
            expect(
              outputStdout().join('\n'),
              '$message' // initial message
              '${" " * 4}${" " * 8}' // margin (4) and space for the time at the end (8)
              '$a' // first tick
              '$blankLine' // clearing the line
              'Rude Interrupting Cow\n' // message
              '$message' // message restoration
              '${" " * 4}${" " * 8}' // margin (4) and space for the time at the end (8)
              '$b' // second tick
              // ignore: missing_whitespace_between_adjacent_strings
              '\b \b' // backspace the tick, wipe the tick, backspace the wipe
              '\b\b\b\b\b\b\b' // backspace the space for the time
              '    5.0s\n', // replacing it with the time
            );
            done = true;
          });
          expect(done, isTrue);
        });

        testWithoutContext('Stdout startProgress on non-colored terminal pauses', () async {
          bool done = false;
          FakeAsync().run((FakeAsync time) {
            mockStopwatch.elapsed = const Duration(seconds: 5);
            final Logger logger = StdoutLogger(
              terminal: terminal,
              stdio: mockStdio,
              outputPreferences: OutputPreferences.test(showColor: true),
              stopwatchFactory: stopwatchFactory,
            );
            const String message = "Knock Knock, Who's There";
            final Status status = logger.startProgress(
              message,
              progressIndicatorPadding: 10, // ignored
            );
            logger.printStatus('Rude Interrupting Cow');
            status.stop();
            expect(
              outputStdout().join('\n'),
              '$message' // initial message
              '     ' // margin
              '\n' // clearing the line
              'Rude Interrupting Cow\n' // message
              '$message         5.0s\n', // message restoration
            );
            done = true;
          });
          expect(done, isTrue);
        });

        testWithoutContext('SpinnerStatus works when canceled', () async {
          bool done = false;
          FakeAsync().run((FakeAsync time) {
            spinnerStatus.start();
            mockStopwatch.elapsed = const Duration(seconds: 1);
            doWhileAsync(time, () => spinnerStatus.ticks < 10);
            List<String> lines = outputStdout();

            expect(
              lines[0],
              startsWith(
                terminal.supportsEmoji
                    ? 'Hello world                     ⣽\b⣻\b⢿\b⡿\b⣟\b⣯\b⣷\b⣾\b⣽\b⣻'
                    : 'Hello world                     \\\b|\b/\b-\b\\\b|\b/\b-\b\\\b|',
              ),
            );
            expect(lines.length, equals(1));
            expect(lines[0].endsWith('\n'), isFalse);

            // Verify a cancel does _not_ print the time and prints a newline.
            spinnerStatus.cancel();
            lines = outputStdout();
            final List<Match> matches = secondDigits.allMatches(lines[0]).toList();
            expect(matches, isEmpty);
            final String leading = terminal.supportsEmoji ? '⣻' : '|';

            expect(lines[0], endsWith('$leading\b \b'));
            expect(called, equals(1));
            expect(lines.length, equals(2));
            expect(lines[1], equals(''));

            // Verify that stopping or canceling multiple times throws.
            expect(spinnerStatus.cancel, throwsAssertionError);
            expect(spinnerStatus.stop, throwsAssertionError);
            done = true;
          });
          expect(done, isTrue);
        });

        testWithoutContext('SpinnerStatus works when stopped', () async {
          bool done = false;
          FakeAsync().run((FakeAsync time) {
            spinnerStatus.start();
            mockStopwatch.elapsed = const Duration(seconds: 1);
            doWhileAsync(time, () => spinnerStatus.ticks < 10);
            List<String> lines = outputStdout();

            expect(lines, hasLength(1));
            expect(
              lines[0],
              terminal.supportsEmoji
                  ? 'Hello world                     ⣽\b⣻\b⢿\b⡿\b⣟\b⣯\b⣷\b⣾\b⣽\b⣻'
                  : 'Hello world                     \\\b|\b/\b-\b\\\b|\b/\b-\b\\\b|',
            );

            // Verify a stop prints the time.
            spinnerStatus.stop();
            lines = outputStdout();
            expect(lines, hasLength(2));
            expect(
              lines[0],
              matches(
                terminal.supportsEmoji
                    ? r'Hello world                     ⣽[\b]⣻[\b]⢿[\b]⡿[\b]⣟[\b]⣯[\b]⣷[\b]⣾[\b]⣽[\b]⣻[\b] [\b]{8}[\d., ]{5}[\d]ms$'
                    : r'Hello world                     \\[\b]|[\b]/[\b]-[\b]\\[\b]|[\b]/[\b]-[\b]\\[\b]|[\b] [\b]{8}[\d., ]{5}[\d]ms$',
              ),
            );
            expect(lines[1], isEmpty);
            final List<Match> times = secondDigits.allMatches(lines[0]).toList();
            expect(times, isNotNull);
            expect(times, hasLength(1));
            final Match match = times.single;

            expect(lines[0], endsWith(match.group(0)!));
            expect(called, equals(1));
            expect(lines.length, equals(2));
            expect(lines[1], equals(''));

            // Verify that stopping or canceling multiple times throws.
            expect(spinnerStatus.stop, throwsAssertionError);
            expect(spinnerStatus.cancel, throwsAssertionError);
            done = true;
          });
          expect(done, isTrue);
        });
      });
    }
  });

  group('Output format', () {
    late FakeStdio fakeStdio;
    late SummaryStatus summaryStatus;
    late int called;

    setUp(() {
      fakeStdio = FakeStdio();
      called = 0;
      summaryStatus = SummaryStatus(
        message: 'Hello world',
        padding: 20,
        onFinish: () => called++,
        stdio: fakeStdio,
        stopwatch: FakeStopwatch(),
      );
    });

    List<String> outputStdout() => fakeStdio.writtenToStdout.join().split('\n');
    List<String> outputStderr() => fakeStdio.writtenToStderr.join().split('\n');

    testWithoutContext('Error logs are wrapped', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printError('0123456789' * 15);
      final List<String> lines = outputStderr();

      expect(outputStdout().length, equals(1));
      expect(outputStdout().first, isEmpty);
      expect(lines[0], equals('0123456789' * 4));
      expect(lines[1], equals('0123456789' * 4));
      expect(lines[2], equals('0123456789' * 4));
      expect(lines[3], equals('0123456789' * 3));
    });

    testWithoutContext('AppRunLogger writes plain text statuses when no app is active', () async {
      final BufferLogger buffer = BufferLogger.test();
      final AppRunLogger logger = AppRunLogger(parent: buffer);

      logger.startProgress('Test status...').stop();

      expect(buffer.statusText.trim(), equals('Test status...'));
    });

    testWithoutContext('Error logs are wrapped and can be indented.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printError('0123456789' * 15, indent: 5);
      final List<String> lines = outputStderr();

      expect(outputStdout().length, equals(1));
      expect(outputStdout().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('     01234567890123456789012345678901234'));
      expect(lines[1], equals('     56789012345678901234567890123456789'));
      expect(lines[2], equals('     01234567890123456789012345678901234'));
      expect(lines[3], equals('     56789012345678901234567890123456789'));
      expect(lines[4], equals('     0123456789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Error logs are wrapped and can have hanging indent.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printError('0123456789' * 15, hangingIndent: 5);
      final List<String> lines = outputStderr();

      expect(outputStdout().length, equals(1));
      expect(outputStdout().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('0123456789012345678901234567890123456789'));
      expect(lines[1], equals('     01234567890123456789012345678901234'));
      expect(lines[2], equals('     56789012345678901234567890123456789'));
      expect(lines[3], equals('     01234567890123456789012345678901234'));
      expect(lines[4], equals('     56789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Error logs are wrapped, indented, and can have hanging indent.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printError('0123456789' * 15, indent: 4, hangingIndent: 5);
      final List<String> lines = outputStderr();

      expect(outputStdout().length, equals(1));
      expect(outputStdout().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('    012345678901234567890123456789012345'));
      expect(lines[1], equals('         6789012345678901234567890123456'));
      expect(lines[2], equals('         7890123456789012345678901234567'));
      expect(lines[3], equals('         8901234567890123456789012345678'));
      expect(lines[4], equals('         901234567890123456789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Stdout logs are wrapped', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printStatus('0123456789' * 15);
      final List<String> lines = outputStdout();

      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      expect(lines[0], equals('0123456789' * 4));
      expect(lines[1], equals('0123456789' * 4));
      expect(lines[2], equals('0123456789' * 4));
      expect(lines[3], equals('0123456789' * 3));
    });

    testWithoutContext('Stdout logs are wrapped and can be indented.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printStatus('0123456789' * 15, indent: 5);
      final List<String> lines = outputStdout();

      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('     01234567890123456789012345678901234'));
      expect(lines[1], equals('     56789012345678901234567890123456789'));
      expect(lines[2], equals('     01234567890123456789012345678901234'));
      expect(lines[3], equals('     56789012345678901234567890123456789'));
      expect(lines[4], equals('     0123456789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Stdout logs are wrapped and can have hanging indent.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printStatus('0123456789' * 15, hangingIndent: 5);
      final List<String> lines = outputStdout();

      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('0123456789012345678901234567890123456789'));
      expect(lines[1], equals('     01234567890123456789012345678901234'));
      expect(lines[2], equals('     56789012345678901234567890123456789'));
      expect(lines[3], equals('     01234567890123456789012345678901234'));
      expect(lines[4], equals('     56789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Stdout logs are wrapped, indented, and can have hanging indent.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.printStatus('0123456789' * 15, indent: 4, hangingIndent: 5);
      final List<String> lines = outputStdout();

      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      expect(lines.length, equals(6));
      expect(lines[0], equals('    012345678901234567890123456789012345'));
      expect(lines[1], equals('         6789012345678901234567890123456'));
      expect(lines[2], equals('         7890123456789012345678901234567'));
      expect(lines[3], equals('         8901234567890123456789012345678'));
      expect(lines[4], equals('         901234567890123456789'));
      expect(lines[5], isEmpty);
    });

    testWithoutContext('Error logs are red', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform(stdoutSupportsAnsi: true)),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      logger.printError('Pants on fire!');
      final List<String> lines = outputStderr();

      expect(outputStdout().length, equals(1));
      expect(outputStdout().first, isEmpty);
      expect(lines[0], equals('${AnsiTerminal.red}Pants on fire!${AnsiTerminal.resetColor}'));
    });

    testWithoutContext('Stdout logs are not colored', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      logger.printStatus('All good.');

      final List<String> lines = outputStdout();
      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      expect(lines[0], equals('All good.'));
    });

    testWithoutContext('Stdout printBox puts content inside a box', () {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      logger.printBox('Hello world', title: 'Test title');
      final String stdout = fakeStdio.writtenToStdout.join();
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test title ┐\n'
          '│ Hello world │\n'
          '└─────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox does not require title', () {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      logger.printBox('Hello world');
      final String stdout = fakeStdio.writtenToStdout.join();
      expect(
        stdout,
        contains(
          '\n'
          '┌─────────────┐\n'
          '│ Hello world │\n'
          '└─────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox handles new lines', () {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      logger.printBox('Hello world\nThis is a new line', title: 'Test title');
      final String stdout = fakeStdio.writtenToStdout.join();
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test title ───────┐\n'
          '│ Hello world        │\n'
          '│ This is a new line │\n'
          '└────────────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox handles content with ANSI escape characters', () {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true),
      );
      const String bold = '\u001B[1m';
      const String clear = '\u001B[2J\u001B[H';
      logger.printBox('${bold}Hello world$clear', title: 'Test title');
      final String stdout = fakeStdio.writtenToStdout.join();
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test title ┐\n'
          '│ ${bold}Hello world$clear │\n'
          '└─────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox handles column limit', () {
      const int columnLimit = 14;
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true, wrapColumn: columnLimit),
      );
      logger.printBox('This line is longer than $columnLimit characters', title: 'Test');
      final String stdout = fakeStdio.writtenToStdout.join();
      final List<String> stdoutLines = stdout.split('\n');

      expect(stdoutLines.length, greaterThan(1));
      expect(stdoutLines[1].length, equals(columnLimit));
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test ─────┐\n'
          '│ This line  │\n'
          '│ is longer  │\n'
          '│ than 14    │\n'
          '│ characters │\n'
          '└────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox handles column limit and respects new lines', () {
      const int columnLimit = 14;
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true, wrapColumn: columnLimit),
      );
      logger.printBox('This\nline is longer than\n\n$columnLimit characters', title: 'Test');
      final String stdout = fakeStdio.writtenToStdout.join();
      final List<String> stdoutLines = stdout.split('\n');

      expect(stdoutLines.length, greaterThan(1));
      expect(stdoutLines[1].length, equals(columnLimit));
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test ─────┐\n'
          '│ This       │\n'
          '│ line is    │\n'
          '│ longer     │\n'
          '│ than       │\n'
          '│            │\n'
          '│ 14         │\n'
          '│ characters │\n'
          '└────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout printBox breaks long words that exceed the column limit', () {
      const int columnLimit = 14;
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: FakePlatform()),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(showColor: true, wrapColumn: columnLimit),
      );
      logger.printBox('Thiswordislongerthan${columnLimit}characters', title: 'Test');
      final String stdout = fakeStdio.writtenToStdout.join();
      final List<String> stdoutLines = stdout.split('\n');

      expect(stdoutLines.length, greaterThan(1));
      expect(stdoutLines[1].length, equals(columnLimit));
      expect(
        stdout,
        contains(
          '\n'
          '┌─ Test ─────┐\n'
          '│ Thiswordis │\n'
          '│ longerthan │\n'
          '│ 14characte │\n'
          '│ rs         │\n'
          '└────────────┘\n',
        ),
      );
    });

    testWithoutContext('Stdout startProgress on non-color terminal', () async {
      final FakeStopwatch fakeStopwatch = FakeStopwatch();
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(),
        stopwatchFactory: FakeStopwatchFactory(stopwatch: fakeStopwatch),
      );
      final Status status = logger.startProgress(
        'Hello',
        progressIndicatorPadding: 20, // this minus the "Hello" equals the 15 below.
      );
      expect(outputStderr().length, equals(1));
      expect(outputStderr().first, isEmpty);
      // the 5 below is the margin that is always included between the message and the time.
      expect(outputStdout().join('\n'), matches(r'^Hello {15} {5}$'));

      fakeStopwatch.elapsed = const Duration(seconds: 4, milliseconds: 123);
      status.stop();

      expect(outputStdout(), <String>['Hello                        4.1s', '']);
    });

    testWithoutContext('SummaryStatus works when canceled', () async {
      final SummaryStatus summaryStatus = SummaryStatus(
        message: 'Hello world',
        padding: 20,
        onFinish: () => called++,
        stdio: fakeStdio,
        stopwatch: FakeStopwatch(),
      );
      summaryStatus.start();
      final List<String> lines = outputStdout();
      expect(lines[0], startsWith('Hello world              '));
      expect(lines.length, equals(1));
      expect(lines[0].endsWith('\n'), isFalse);

      // Verify a cancel does _not_ print the time and prints a newline.
      summaryStatus.cancel();
      expect(outputStdout(), <String>['Hello world              ', '']);

      // Verify that stopping or canceling multiple times throws.
      expect(summaryStatus.cancel, throwsAssertionError);
      expect(summaryStatus.stop, throwsAssertionError);
    });

    testWithoutContext('SummaryStatus works when stopped', () async {
      summaryStatus.start();
      final List<String> lines = outputStdout();
      expect(lines[0], startsWith('Hello world              '));
      expect(lines.length, equals(1));

      // Verify a stop prints the time.
      summaryStatus.stop();
      expect(outputStdout(), <String>['Hello world                   0ms', '']);

      // Verify that stopping or canceling multiple times throws.
      expect(summaryStatus.stop, throwsAssertionError);
      expect(summaryStatus.cancel, throwsAssertionError);
    });

    testWithoutContext('sequential startProgress calls with StdoutLogger', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        stdio: fakeStdio,
        outputPreferences: OutputPreferences.test(),
      );
      logger.startProgress('AAA').stop();
      logger.startProgress('BBB').stop();
      final List<String> output = outputStdout();

      expect(output.length, equals(3));

      // There's 61 spaces at the start: 59 (padding default) - 3 (length of AAA) + 5 (margin).
      // Then there's a left-padded "0ms" 8 characters wide, so 5 spaces then "0ms"
      // (except sometimes it's randomly slow so we handle up to "99,999ms").
      expect(output[0], matches(RegExp(r'AAA[ ]{61}[\d, ]{5}[\d]ms')));
      expect(output[1], matches(RegExp(r'BBB[ ]{61}[\d, ]{5}[\d]ms')));
    });

    testWithoutContext(
      'sequential startProgress calls with VerboseLogger and StdoutLogger',
      () async {
        final Logger logger = VerboseLogger(
          StdoutLogger(
            terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
            stdio: fakeStdio,
            outputPreferences: OutputPreferences.test(),
          ),
          stopwatchFactory: FakeStopwatchFactory(),
        );
        logger.startProgress('AAA').stop();
        logger.startProgress('BBB').stop();

        expect(outputStdout(), <Matcher>[
          matches(r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] AAA$'),
          matches(r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] AAA \(completed.*\)$'),
          matches(r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] BBB$'),
          matches(r'^\[ (?: {0,2}\+[0-9]{1,4} ms|       )\] BBB \(completed.*\)$'),
          matches(r'^$'),
        ]);
      },
    );

    testWithoutContext('sequential startProgress calls with BufferLogger', () async {
      final BufferLogger logger = BufferLogger(
        terminal: AnsiTerminal(stdio: fakeStdio, platform: _kNoAnsiPlatform),
        outputPreferences: OutputPreferences.test(),
      );
      logger.startProgress('AAA').stop();
      logger.startProgress('BBB').stop();

      expect(logger.statusText, 'AAA\nBBB\n');
    });

    testWithoutContext('BufferLogger prints status, trace, error', () async {
      final BufferLogger mockLogger = BufferLogger.test(
        outputPreferences: OutputPreferences.test(),
      );

      mockLogger.printStatus('Hey Hey Hey Hey');
      mockLogger.printTrace('Oooh, I do I do I do');
      final StackTrace stackTrace = StackTrace.current;
      mockLogger.printError('Helpless!', stackTrace: stackTrace);

      expect(mockLogger.statusText, 'Hey Hey Hey Hey\n');
      expect(mockLogger.traceText, 'Oooh, I do I do I do\n');
      expect(mockLogger.errorText, 'Helpless!\n$stackTrace\n');
    });

    testWithoutContext('Animations are disabled when, uh, disabled.', () async {
      final Logger logger = StdoutLogger(
        terminal: AnsiTerminal(
          stdio: fakeStdio,
          platform: _kNoAnsiPlatform,
          defaultCliAnimationEnabled: false,
        ),
        stdio: fakeStdio,
        stopwatchFactory: FakeStopwatchFactory(stopwatch: FakeStopwatch()),
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
      );
      logger.startProgress('po').stop();
      expect(outputStderr(), <String>['']);
      expect(outputStdout(), <String>[
        'po                                                                   0ms',
        '',
      ]);
      logger.startProgress('ta')
        ..pause()
        ..resume()
        ..stop();
      expect(outputStderr(), <String>['']);
      expect(outputStdout(), <String>[
        'po                                                                   0ms',
        'ta                                                              ',
        'ta                                                                   0ms',
        '',
      ]);
      logger.startSpinner()
        ..pause()
        ..resume()
        ..stop();
      expect(outputStderr(), <String>['']);
      expect(outputStdout(), <String>[
        'po                                                                   0ms',
        'ta                                                              ',
        'ta                                                                   0ms',
        '',
      ]);
    });
  });
}

/// A fake [Logger] that throws the [Invocation] for any method call.
class FakeLogger implements Logger {
  @override
  dynamic noSuchMethod(Invocation invocation) => throw invocation; // ignore: only_throw_errors
}

/// Returns the [Invocation] thrown from a call to [FakeLogger].
Invocation _invocationFor(dynamic Function() fakeCall) {
  try {
    fakeCall();
  } on Invocation catch (invocation) {
    return invocation;
  }
  throw UnsupportedError(
    '_invocationFor can be used only with Fake objects '
    'that throw Invocations',
  );
}

/// Returns a [Matcher] that matches against an expected [Invocation].
Matcher _matchesInvocation(Invocation expected) {
  return const TypeMatcher<Invocation>()
      // Compare Symbol strings instead of comparing Symbols directly for a nicer failure message.
      .having(
        (Invocation actual) => actual.memberName.toString(),
        'memberName',
        expected.memberName.toString(),
      )
      .having((Invocation actual) => actual.isGetter, 'isGetter', expected.isGetter)
      .having((Invocation actual) => actual.isSetter, 'isSetter', expected.isSetter)
      .having((Invocation actual) => actual.isMethod, 'isMethod', expected.isMethod)
      .having((Invocation actual) => actual.typeArguments, 'typeArguments', expected.typeArguments)
      .having(
        (Invocation actual) => actual.positionalArguments,
        'positionalArguments',
        expected.positionalArguments,
      )
      .having(
        (Invocation actual) => actual.namedArguments,
        'namedArguments',
        expected.namedArguments,
      );
}

/// Returns a [Matcher] that matches against an [Invocation] thrown from a call
/// to [FakeLogger].
Matcher _throwsInvocationFor(dynamic Function() fakeCall) =>
    throwsA(_matchesInvocation(_invocationFor(fakeCall)));

class FakeStdout extends Fake implements Stdout {
  FakeStdout({required this.syncError, this.completeWithError = false});

  final bool syncError;
  final bool completeWithError;
  final Completer<void> _completer = Completer<void>();

  @override
  void write(Object? object) {
    if (syncError) {
      throw Exception('Error!');
    }
    Zone.current.runUnaryGuarded<void>((_) {
      if (completeWithError) {
        _completer.completeError(Exception('Some pipe error'));
      } else {
        _completer.complete();
        throw Exception('Error!');
      }
    }, null);
  }

  @override
  Future<void> get done => _completer.future;
}
