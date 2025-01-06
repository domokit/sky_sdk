// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter_tools/src/base/platform.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../src/common.dart';
import 'test_utils.dart' show flutterBin;

const ProcessManager processManager = LocalProcessManager();
final String flutterRoot = getFlutterRoot();

void debugPrint(String message) {
  // This is called to intentionally print debugging output when a test is
  // either taking too long or has failed.
  // ignore: avoid_print
  print(message);
}

typedef LineHandler = String? Function(String line);

abstract class Transition {
  const Transition({this.handler, this.logging});

  /// Callback that is invoked when the transition matches.
  ///
  /// This should not throw, even if the test is failing. (For example, don't use "expect"
  /// in these callbacks.) Throwing here would prevent the [runFlutter] function from running
  /// to completion, which would leave zombie `flutter` processes around.
  final LineHandler? handler;

  /// Whether to enable or disable logging when this transition is matched.
  ///
  /// The default value, null, leaves the logging state unaffected.
  final bool? logging;

  bool matches(String line);

  @protected
  bool lineMatchesPattern(String line, Pattern pattern, bool contains) {
    if (pattern is RegExp) {
      // Ideally this would also distinguish between "contains" and "equals"
      // operation.
      return line.contains(pattern);
    }
    return contains ? line.contains(pattern) : line == pattern;
  }

  @protected
  String describe(Pattern pattern, bool contains) {
    if (pattern is RegExp) {
      return '/${pattern.pattern}/';
    }
    return contains ? '"...$pattern..."' : '"$pattern"';
  }
}

class Barrier extends Transition {
  Barrier(this.pattern, {super.handler, super.logging}) : contains = false;
  Barrier.contains(this.pattern, {super.handler, super.logging}) : contains = true;

  final Pattern pattern;
  final bool contains;

  @override
  bool matches(String line) => lineMatchesPattern(line, pattern, contains);

  @override
  String toString() => describe(pattern, contains);
}

class Multiple extends Transition {
  Multiple(List<Pattern> patterns, {super.handler, super.logging})
    : _originalPatterns = patterns,
      patterns = patterns.toList(),
      contains = false;
  Multiple.contains(List<Pattern> patterns, {super.handler, super.logging})
    : _originalPatterns = patterns,
      patterns = patterns.toList(),
      contains = true;

  final List<Pattern> _originalPatterns;
  final List<Pattern> patterns;
  final bool contains;

  @override
  bool matches(String line) {
    for (int index = 0; index < patterns.length; index += 1) {
      if (lineMatchesPattern(line, patterns[index], contains)) {
        patterns.removeAt(index);
        break;
      }
    }
    return patterns.isEmpty;
  }

  @override
  String toString() {
    String describe(Pattern pattern) => super.describe(pattern, contains);
    if (patterns.isEmpty) {
      return '${_originalPatterns.map(describe).join(', ')} (all matched)';
    }
    return '${_originalPatterns.map(describe).join(', ')} (matched ${_originalPatterns.length - patterns.length} so far)';
  }
}

class LogLine {
  const LogLine(this.channel, this.stamp, this.message);
  final String channel;
  final String stamp;
  final String message;

  bool get couldBeCrash => message.contains('Oops; flutter has exited unexpectedly:');

  @override
  String toString() => '$stamp $channel: $message';

  void printClearly() {
    debugPrint('$stamp $channel: ${clarify(message)}');
  }

  static String clarify(String line) {
    return line.runes
        .map<String>(
          (int rune) => switch (rune) {
            >= 0x20 && <= 0x7F => String.fromCharCode(rune),
            0x00 => '<NUL>',
            0x07 => '<BEL>',
            0x08 => '<TAB>',
            0x09 => '<BS>',
            0x0A => '<LF>',
            0x0D => '<CR>',
            _ => '<${rune.toRadixString(16).padLeft(rune <= 0xFF
                ? 2
                : rune <= 0xFFFF
                ? 4
                : 5, '0')}>',
          },
        )
        .join();
  }
}

class ProcessTestResult {
  const ProcessTestResult(this.exitCode, this.logs);
  final int exitCode;
  final List<LogLine> logs;

  List<String> get stdout {
    return logs
        .where((LogLine log) => log.channel == 'stdout')
        .map<String>((LogLine log) => log.message)
        .toList();
  }

  List<String> get stderr {
    return logs
        .where((LogLine log) => log.channel == 'stderr')
        .map<String>((LogLine log) => log.message)
        .toList();
  }

  @override
  String toString() => 'exit code $exitCode\nlogs:\n  ${logs.join('\n  ')}\n';
}

Future<ProcessTestResult> runFlutter(
  List<String> arguments,
  String workingDirectory,
  List<Transition> transitions, {
  bool debug = false,
  bool logging = true,
  Duration expectedMaxDuration = const Duration(
    minutes: 10,
  ), // must be less than test timeout of 15 minutes! See ../../dart_test.yaml.
}) async {
  const LocalPlatform platform = LocalPlatform();
  final Stopwatch clock = Stopwatch()..start();
  final Process process = await processManager.start(<String>[
    // In a container with no X display, use the virtual framebuffer.
    if (platform.isLinux && (platform.environment['DISPLAY'] ?? '').isEmpty) '/usr/bin/xvfb-run',
    flutterBin,
    ...arguments,
  ], workingDirectory: workingDirectory);
  final List<LogLine> logs = <LogLine>[];
  int nextTransition = 0;
  void describeStatus() {
    if (transitions.isNotEmpty) {
      debugPrint('Expected state transitions:');
      for (int index = 0; index < transitions.length; index += 1) {
        debugPrint(
          '${index.toString().padLeft(5)} '
          '${index < nextTransition
              ? 'ALREADY MATCHED '
              : index == nextTransition
              ? 'NOW WAITING FOR>'
              : '                '} ${transitions[index]}',
        );
      }
    }
    if (logs.isEmpty) {
      debugPrint(
        'So far nothing has been logged${debug ? "" : "; use debug:true to print all output"}.',
      );
    } else {
      debugPrint(
        'Log${debug ? "" : " (only contains logged lines; use debug:true to print all output)"}:',
      );
      for (final LogLine log in logs) {
        log.printClearly();
      }
    }
  }

  bool streamingLogs = false;
  Timer? timeout;
  void processTimeout() {
    if (!streamingLogs) {
      streamingLogs = true;
      if (!debug) {
        debugPrint('Test is taking a long time (${clock.elapsed.inSeconds} seconds so far).');
      }
      describeStatus();
      debugPrint('(streaming all logs from this point on...)');
    } else {
      debugPrint('(taking a long time...)');
    }
  }

  String stamp() => '[${(clock.elapsed.inMilliseconds / 1000.0).toStringAsFixed(1).padLeft(5)}s]';
  void processStdout(String line) {
    final LogLine log = LogLine('stdout', stamp(), line);
    if (logging) {
      logs.add(log);
    }
    if (streamingLogs) {
      log.printClearly();
    }
    if (nextTransition < transitions.length && transitions[nextTransition].matches(line)) {
      if (streamingLogs) {
        debugPrint('(matched ${transitions[nextTransition]})');
      }
      if (transitions[nextTransition].logging != null) {
        if (!logging && transitions[nextTransition].logging!) {
          logs.add(log);
        }
        logging = transitions[nextTransition].logging!;
        if (streamingLogs) {
          if (logging) {
            debugPrint('(enabled logging)');
          } else {
            debugPrint('(disabled logging)');
          }
        }
      }
      if (transitions[nextTransition].handler != null) {
        final String? command = transitions[nextTransition].handler!(line);
        if (command != null) {
          final LogLine inLog = LogLine('stdin', stamp(), command);
          logs.add(inLog);
          if (streamingLogs) {
            inLog.printClearly();
          }
          process.stdin.write(command);
        }
      }
      nextTransition += 1;
      timeout?.cancel();
      timeout = Timer(
        expectedMaxDuration ~/ 5,
        processTimeout,
      ); // This is not a failure timeout, just when to start logging verbosely to help debugging.
    }
  }

  void processStderr(String line) {
    final LogLine log = LogLine('stdout', stamp(), line);
    logs.add(log);
    if (streamingLogs) {
      log.printClearly();
    }
  }

  if (debug) {
    processTimeout();
  } else {
    timeout = Timer(
      expectedMaxDuration ~/ 2,
      processTimeout,
    ); // This is not a failure timeout, just when to start logging verbosely to help debugging.
  }
  process.stdout
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen(processStdout);
  process.stderr
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen(processStderr);
  unawaited(
    process.exitCode
        .timeout(
          expectedMaxDuration,
          onTimeout: () {
            // This is a failure timeout, must not be short.
            debugPrint(
              '${stamp()} (process is not quitting, trying to send a "q" just in case that helps)',
            );
            debugPrint('(a functional test should never reach this point)');
            final LogLine inLog = LogLine('stdin', stamp(), 'q');
            logs.add(inLog);
            if (streamingLogs) {
              inLog.printClearly();
            }
            process.stdin.write('q');
            return -1; // discarded
          },
        )
        .then(
          (int i) => i,
          onError: (Object error) {
            // ignore errors here, they will be reported on the next line
            return -1; // discarded
          },
        ),
  );
  final int exitCode = await process.exitCode;
  if (streamingLogs) {
    debugPrint('${stamp()} (process terminated with exit code $exitCode)');
  }
  timeout?.cancel();
  if (nextTransition < transitions.length) {
    debugPrint('The subprocess terminated before all the expected transitions had been matched.');
    if (logs.any((LogLine line) => line.couldBeCrash)) {
      debugPrint('The subprocess may in fact have crashed. Check the stderr logs below.');
    }
    debugPrint('The transition that we were hoping to see next but that we never saw was:');
    debugPrint(
      '${nextTransition.toString().padLeft(5)} NOW WAITING FOR> ${transitions[nextTransition]}',
    );
    if (!streamingLogs) {
      describeStatus();
      debugPrint('(process terminated with exit code $exitCode)');
    }
    throw TestFailure('Missed some expected transitions.');
  }
  if (streamingLogs) {
    debugPrint('${stamp()} (completed execution successfully!)');
  }
  return ProcessTestResult(exitCode, logs);
}

const int progressMessageWidth = 64;
