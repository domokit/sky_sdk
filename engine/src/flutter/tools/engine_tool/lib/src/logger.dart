// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async' show Timer, runZoned;
import 'dart:io' as io show IOSink, stderr, stdout;

import 'package:logging/logging.dart' as log;
import 'package:meta/meta.dart';

// Part of the public API, so it's nicer to provide the symbols directly.
export 'package:logging/logging.dart' show LogRecord;

// This is where a flutter_tool style progress spinner, color output,
// ascii art, terminal control for clearing lines or the whole screen, etc.
// can go. We can just add more methods to Logger using the flutter_tool's
// Logger as a guide:
//
// https://github.com/flutter/flutter/blob/c530276f7806c77da2541c518a0e103c9bb44f10/packages/flutter_tools/lib/src/base/logger.dart#L422

/// A simplified wrapper around the [Logger] from package:logging.
///
/// The default log level is [Logger.status]. A --quiet flag might change it to
/// [Logger.warning] or [Logger.error]. A --verbose flag might change it to
/// [Logger.info].
///
/// Log messages at [Logger.warning] and higher will be written to stderr, and
/// to stdout otherwise. [Logger.test] records all log messages to a buffer,
/// which can be inspected by unit tetss.
class Logger {
  /// Constructs a logger for use in the tool.
  Logger({log.Level level = statusLevel}) : _logger = log.Logger.detached('et'), _test = false {
    _logger.level = level;
    _logger.onRecord.listen(_handler);
    _setupIoSink(io.stderr);
    _setupIoSink(io.stdout);
  }

  /// Constructs a logger that invokes a [callback] for each log message.
  @visibleForTesting
  Logger.test(void Function(log.LogRecord) onLog, {log.Level level = statusLevel})
    : _logger = log.Logger.detached('et'),
      _test = true {
    _logger.level = level;
    _logger.onRecord.listen(onLog);
  }

  /// The logging level for error messages. These go to stderr.
  static const log.Level errorLevel = log.Level('ERROR', 100);

  /// The logging level for warning messages. These go to stderr.
  static const log.Level warningLevel = log.Level('WARNING', 75);

  /// The logging level for normal status messages. These go to stdout.
  static const log.Level statusLevel = log.Level('STATUS', 25);

  /// The logging level for verbose informational messages. These go to stdout.
  static const log.Level infoLevel = log.Level('INFO', 10);

  static void _handler(log.LogRecord r) {
    final io.IOSink sink = r.level >= warningLevel ? io.stderr : io.stdout;
    final String prefix = r.level >= warningLevel ? '[${r.time}] ${r.level}: ' : '';
    _ioSinkWrite(sink, '$prefix${r.message}');
  }

  // Status of the global io.stderr and io.stdout is shared across all
  // Logger instances.
  static bool _stdioDone = false;

  // stdout and stderr might already be closed, and when not already closed,
  // writing can still fail by throwing either a sync or async exception.
  // This function handles all three cases.
  static void _ioSinkWrite(io.IOSink sink, String message) {
    if (_stdioDone) {
      return;
    }
    runZoned<void>(
      () {
        try {
          sink.write(message);
        } catch (_) {
          _stdioDone = true;
        }
      },
      onError: (Object e, StackTrace s) {
        _stdioDone = true;
      },
    );
  }

  static void _setupIoSink(io.IOSink sink) {
    sink.done.then(
      (void _) {
        _stdioDone = true;
      },
      onError: (Object err, StackTrace st) {
        _stdioDone = true;
      },
    );
  }

  final log.Logger _logger;
  final bool _test;

  Spinner? _status;

  /// Get the current logging level.
  log.Level get level => _logger.level;

  /// Record a log message level [Logger.error] and throw a FatalError.
  /// This should only be called when the program has entered an impossible
  /// to recover from state or when something isn't implemented yet.
  Never fatal(Object? message, {int indent = 0, bool newline = true, bool fit = false}) {
    _emitLog(errorLevel, message, indent, newline, fit);
    throw FatalError(_formatMessage(message, indent, newline, fit));
  }

  /// Record a log message at level [Logger.error].
  void error(Object? message, {int indent = 0, bool newline = true, bool fit = false}) {
    _emitLog(errorLevel, message, indent, newline, fit);
  }

  /// Record a log message at level [Logger.warning].
  void warning(Object? message, {int indent = 0, bool newline = true, bool fit = false}) {
    _emitLog(warningLevel, message, indent, newline, fit);
  }

  /// Record a log message at level [Logger.warning].
  void status(Object? message, {int indent = 0, bool newline = true, bool fit = false}) {
    _emitLog(statusLevel, message, indent, newline, fit);
  }

  /// Record a log message at level [Logger.info].
  void info(Object? message, {int indent = 0, bool newline = true, bool fit = false}) {
    _emitLog(infoLevel, message, indent, newline, fit);
  }

  /// Functionally ends and starts a new line.
  ///
  /// How that is done depends on the terminal capabilities:
  ///
  /// - If we are not in a terminal, just write a newline.
  /// - If we are in a a terminal, any spinners are temporarily paused, the
  ///   current line is cleared, and spinners are resumed. If ANSI escapes are
  ///   supported, the cursor is moved to the start of the line and the line is
  ///   cleared. Otherwise, the line is cleared by writing spaces to the width
  ///   of the terminal, then moving the cursor back to the start of the line.
  void clearLine() {
    if (!io.stdout.hasTerminal || _test) {
      // Just write a newline if we're not in a terminal.
      _ioSinkWrite(io.stdout, '\n');
      return;
    }
    _status?.pause();
    _emitClearLine();
    _status?.resume();
  }

  /// Starts printing a progress spinner.
  @useResult
  Spinner startSpinner({void Function()? onFinish}) {
    void finishCallback() {
      onFinish?.call();
      _status = null;
    }

    _status =
        io.stdout.hasTerminal && !_test
            ? FlutterSpinner(onFinish: finishCallback)
            : Spinner(onFinish: finishCallback);
    _status!.start();
    return _status!;
  }

  static void _emitClearLine() {
    if (io.stdout.supportsAnsiEscapes) {
      // Go to start of the line and clear the line.
      _ioSinkWrite(io.stdout, '\r\x1B[K');
      return;
    }
    final int width = io.stdout.terminalColumns;
    final String backspaces = '\b' * width;
    final String spaces = ' ' * width;
    _ioSinkWrite(io.stdout, '$backspaces$spaces$backspaces');
  }

  String _formatMessage(Object? message, int indent, bool newline, bool fit) {
    String m = '${' ' * indent}$message${newline ? '\n' : ''}';
    if (fit && io.stdout.hasTerminal) {
      m = fitToWidth(m, io.stdout.terminalColumns);
    }
    return m;
  }

  void _emitLog(log.Level level, Object? message, int indent, bool newline, bool fit) {
    final String m = _formatMessage(message, indent, newline, fit);
    _status?.pause();
    _logger.log(level, m);
    _status?.resume();
  }

  /// Shorten a string such that its length will be `w` by replacing
  /// enough characters in the middle with '...'. Trailing whitespace will not
  /// be preserved or counted against 'w', but if the input ends with a newline,
  /// then the output will end with a newline that is not counted against 'w'.
  /// That is, if the input string ends with a newline, the output string will
  /// have length up to (w + 1) and end with a newline.
  ///
  /// If w <= 0, the result will be the empty string.
  /// If w <= 3, the result will be a string containing w '.'s.
  /// If there are a different number of non-'...' characters to the right and
  /// left of '...' in the result, then the right will have one more than the
  /// left.
  @visibleForTesting
  static String fitToWidth(String s, int w) {
    // Preserve a trailing newline if needed.
    final String maybeNewline = s.endsWith('\n') ? '\n' : '';
    if (w <= 0) {
      return maybeNewline;
    }
    if (w <= 3) {
      return '${'.' * w}$maybeNewline';
    }

    // But remove trailing whitespace before removing the middle of the string.
    s = s.trimRight();
    if (s.length <= w) {
      return '$s$maybeNewline';
    }

    // remove (s.length + 3 - w) characters from the middle of `s` and
    // replace them with '...'.
    final int diff = (s.length + 3) - w;
    final int leftEnd = (s.length - diff) ~/ 2;
    final int rightStart = (s.length + diff) ~/ 2;
    s = s.replaceRange(leftEnd, rightStart, '...');
    return s + maybeNewline;
  }
}

/// A base class for progress spinners, and a no-op implementation that prints
/// nothing.
class Spinner {
  /// Creates a progress spinner. If supplied the `onDone` callback will be
  /// called when `finish()` is called.
  Spinner({this.onFinish});

  /// The callback called when `finish()` is called.
  final void Function()? onFinish;

  /// Starts the spinner animation.
  void start() {}

  /// Pauses the spinner animation. That is, this call causes printing to the
  /// terminal to stop.
  void pause() {}

  /// Resumes the animation at the same from where `pause()` was called.
  void resume() {}

  /// Ends an animation, calling the `onFinish` callback if one was provided.
  void finish() {
    onFinish?.call();
  }
}

/// A [Spinner] implementation that prints an animated "Flutter" banner.
class FlutterSpinner extends Spinner {
  // ignore: public_member_api_docs
  FlutterSpinner({super.onFinish});

  /// The frames of the animation.
  static const String frames = '⢸⡯⠭⠅⢸⣇⣀⡀⢸⣇⣸⡇⠈⢹⡏⠁⠈⢹⡏⠁⢸⣯⣭⡅⢸⡯⢕⡂⠀⠀';

  static final List<String> _flutterAnimation =
      frames.runes.map<String>((int scalar) => String.fromCharCode(scalar)).toList();

  Timer? _timer;
  int _ticks = 0;
  int _lastAnimationFrameLength = 0;

  @override
  void start() {
    _startSpinner();
  }

  void _startSpinner() {
    _timer = Timer.periodic(const Duration(milliseconds: 100), _callback);
    _callback(_timer!);
  }

  void _callback(Timer timer) {
    Logger._ioSinkWrite(io.stdout, '\b' * _lastAnimationFrameLength);
    _ticks += 1;
    final String newFrame = _currentAnimationFrame;
    _lastAnimationFrameLength = newFrame.runes.length;
    Logger._ioSinkWrite(io.stdout, newFrame);
  }

  String get _currentAnimationFrame {
    return _flutterAnimation[_ticks % _flutterAnimation.length];
  }

  @override
  void pause() {
    Logger._emitClearLine();
    _lastAnimationFrameLength = 0;
    _timer?.cancel();
  }

  @override
  void resume() {
    _startSpinner();
  }

  @override
  void finish() {
    _timer?.cancel();
    _timer = null;
    Logger._emitClearLine();
    _lastAnimationFrameLength = 0;
    if (onFinish != null) {
      onFinish!();
    }
  }
}

/// FatalErrors are thrown when a fatal error has occurred.
final class FatalError extends Error {
  /// Constructs a FatalError with a message.
  FatalError(this._message);

  final String _message;

  @override
  String toString() => _message;
}
