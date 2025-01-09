// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math';

import 'package:meta/meta.dart';

import '../convert.dart';
import 'common.dart';
import 'io.dart';
import 'terminal.dart' show OutputPreferences, Terminal, TerminalColor;
import 'utils.dart';

const int kDefaultStatusPadding = 59;

/// A factory for generating [Stopwatch] instances for [Status] instances.
class StopwatchFactory {
  /// const constructor so that subclasses may be const.
  const StopwatchFactory();

  /// Create a new [Stopwatch] instance.
  ///
  /// The optional [name] parameter is useful in tests when there are multiple
  /// instances being created.
  Stopwatch createStopwatch([String name = '']) => Stopwatch();
}

typedef VoidCallback = void Function();

abstract class Logger {
  /// Whether or not this logger should print [printTrace] messages.
  bool get isVerbose => false;

  /// If true, silences the logger output.
  bool quiet = false;

  /// If true, this logger supports ANSI sequences and animations are enabled.
  bool get supportsColor;

  /// If true, this logger is connected to a terminal.
  bool get hasTerminal;

  /// If true, then [printError] has been called at least once for this logger
  /// since the last time it was set to false.
  bool hadErrorOutput = false;

  /// If true, then [printWarning] has been called at least once with its
  /// "fatal" argument true for this logger
  /// since the last time it was reset to false.
  bool hadWarningOutput = false;

  /// Causes [checkForFatalLogs] to call [throwToolExit] when it is called if
  /// [hadWarningOutput] is true.
  bool fatalWarnings = false;

  /// Returns the terminal attached to this logger.
  Terminal get terminal;

  OutputPreferences get _outputPreferences;

  /// Display an error `message` to the user. Commands should use this if they
  /// fail in some way. Errors are typically followed shortly by a call to
  /// [throwToolExit] to terminate the run.
  ///
  /// The `message` argument is printed to the stderr in [TerminalColor.red] by
  /// default.
  ///
  /// The `stackTrace` argument is the stack trace that will be printed if
  /// supplied.
  ///
  /// The `emphasis` argument will cause the output message be printed in bold text.
  ///
  /// The `color` argument will print the message in the supplied color instead
  /// of the default of red. Colors will not be printed if the output terminal
  /// doesn't support them.
  ///
  /// The `indent` argument specifies the number of spaces to indent the overall
  /// message. If wrapping is enabled in [outputPreferences], then the wrapped
  /// lines will be indented as well.
  ///
  /// If `hangingIndent` is specified, then any wrapped lines will be indented
  /// by this much more than the first line, if wrapping is enabled in
  /// [outputPreferences].
  ///
  /// If `wrap` is specified, then it overrides the
  /// `outputPreferences.wrapText` setting.
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  });

  /// Display a warning `message` to the user. Commands should use this if they
  /// important information to convey to the user that is not fatal.
  ///
  /// The `message` argument is printed to the stderr in [TerminalColor.cyan] by
  /// default.
  ///
  /// The `emphasis` argument will cause the output message be printed in bold text.
  ///
  /// The `color` argument will print the message in the supplied color instead
  /// of the default of cyan. Colors will not be printed if the output terminal
  /// doesn't support them.
  ///
  /// The `indent` argument specifies the number of spaces to indent the overall
  /// message. If wrapping is enabled in [outputPreferences], then the wrapped
  /// lines will be indented as well.
  ///
  /// If `hangingIndent` is specified, then any wrapped lines will be indented
  /// by this much more than the first line, if wrapping is enabled in
  /// [outputPreferences].
  ///
  /// If `wrap` is specified, then it overrides the
  /// `outputPreferences.wrapText` setting.
  void printWarning(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  });

  /// Display normal output of the command. This should be used for things like
  /// progress messages, success messages, or just normal command output.
  ///
  /// The `message` argument is printed to the stdout.
  ///
  /// The `stackTrace` argument is the stack trace that will be printed if
  /// supplied.
  ///
  /// If the `emphasis` argument is true, it will cause the output message be
  /// printed in bold text. Defaults to false.
  ///
  /// The `color` argument will print the message in the supplied color instead
  /// of the default of red. Colors will not be printed if the output terminal
  /// doesn't support them.
  ///
  /// If `newline` is true, then a newline will be added after printing the
  /// status. Defaults to true.
  ///
  /// The `indent` argument specifies the number of spaces to indent the overall
  /// message. If wrapping is enabled in [outputPreferences], then the wrapped
  /// lines will be indented as well.
  ///
  /// If `hangingIndent` is specified, then any wrapped lines will be indented
  /// by this much more than the first line, if wrapping is enabled in
  /// [outputPreferences].
  ///
  /// If `wrap` is specified, then it overrides the
  /// `outputPreferences.wrapText` setting.
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  });

  /// Display the [message] inside a box.
  ///
  /// For example, this is the generated output:
  ///
  ///   ┌─ [title] ─┐
  ///   │ [message] │
  ///   └───────────┘
  ///
  /// If a terminal is attached, the lines in [message] are automatically wrapped based on
  /// the available columns.
  ///
  /// Use this utility only to highlight a message in the logs.
  ///
  /// This is particularly useful when the message can be easily missed because of clutter
  /// generated by other commands invoked by the tool.
  ///
  /// One common use case is to provide actionable steps in a Flutter app when a Gradle
  /// error is printed.
  ///
  /// In the future, this output can be integrated with an IDE like VS Code to display a
  /// notification, and allow the user to trigger an action. e.g. run a migration.
  void printBox(String message, {String? title});

  /// Use this for verbose tracing output. Users can turn this output on in order
  /// to help diagnose issues with the toolchain or with their setup.
  void printTrace(String message);

  /// Start an indeterminate progress display.
  ///
  /// The `message` argument is the message to display to the user.
  ///
  /// The `progressId` argument provides an ID that can be used to identify
  /// this type of progress (e.g. `hot.reload`, `hot.restart`).
  ///
  /// The `progressIndicatorPadding` can optionally be used to specify the width
  /// of the space into which the `message` is placed before the progress
  /// indicator, if any. It is ignored if the message is longer.
  Status startProgress(
    String message, {
    String? progressId,
    int progressIndicatorPadding = kDefaultStatusPadding,
  });

  /// A [SilentStatus] or an [AnonymousSpinnerStatus] (depending on whether the
  /// terminal is fancy enough), already started.
  Status startSpinner({
    VoidCallback? onFinish,
    Duration? timeout,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
  });

  /// Send an event to be emitted.
  ///
  /// Only surfaces a value in machine modes, Loggers may ignore this message in
  /// non-machine modes.
  void sendEvent(String name, [Map<String, dynamic>? args]) {}

  /// Clears all output.
  void clear();

  /// If [fatalWarnings] is set, causes the logger to check if
  /// [hadWarningOutput] is true, and then to call [throwToolExit] if so.
  ///
  /// The [fatalWarnings] flag can be set from the command line with the
  /// "--fatal-warnings" option on commands that support it.
  void checkForFatalLogs() {
    if (fatalWarnings && (hadWarningOutput || hadErrorOutput)) {
      throwToolExit(
        'Logger received ${hadErrorOutput ? 'error' : 'warning'} output '
        'during the run, and "--fatal-warnings" is enabled.',
      );
    }
  }
}

/// A [Logger] that forwards all methods to another logger.
///
/// Classes can derive from this to add functionality to an existing [Logger].
class DelegatingLogger implements Logger {
  @visibleForTesting
  @protected
  DelegatingLogger(this._delegate);

  final Logger _delegate;

  @override
  bool get quiet => _delegate.quiet;

  @override
  set quiet(bool value) => _delegate.quiet = value;

  @override
  bool get hasTerminal => _delegate.hasTerminal;

  @override
  Terminal get terminal => _delegate.terminal;

  @override
  OutputPreferences get _outputPreferences => _delegate._outputPreferences;

  @override
  bool get isVerbose => _delegate.isVerbose;

  @override
  bool get hadErrorOutput => _delegate.hadErrorOutput;

  @override
  set hadErrorOutput(bool value) => _delegate.hadErrorOutput = value;

  @override
  bool get hadWarningOutput => _delegate.hadWarningOutput;

  @override
  set hadWarningOutput(bool value) => _delegate.hadWarningOutput = value;

  @override
  bool get fatalWarnings => _delegate.fatalWarnings;

  @override
  set fatalWarnings(bool value) => _delegate.fatalWarnings = value;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _delegate.printError(
      message,
      stackTrace: stackTrace,
      emphasis: emphasis,
      color: color,
      indent: indent,
      hangingIndent: hangingIndent,
      wrap: wrap,
    );
  }

  @override
  void printWarning(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    _delegate.printWarning(
      message,
      emphasis: emphasis,
      color: color,
      indent: indent,
      hangingIndent: hangingIndent,
      wrap: wrap,
      fatal: fatal,
    );
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _delegate.printStatus(
      message,
      emphasis: emphasis,
      color: color,
      newline: newline,
      indent: indent,
      hangingIndent: hangingIndent,
      wrap: wrap,
    );
  }

  @override
  void printBox(String message, {String? title}) {
    _delegate.printBox(message, title: title);
  }

  @override
  void printTrace(String message) {
    _delegate.printTrace(message);
  }

  @override
  void sendEvent(String name, [Map<String, dynamic>? args]) {
    _delegate.sendEvent(name, args);
  }

  @override
  Status startProgress(
    String message, {
    String? progressId,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    return _delegate.startProgress(
      message,
      progressId: progressId,
      progressIndicatorPadding: progressIndicatorPadding,
    );
  }

  @override
  Status startSpinner({
    VoidCallback? onFinish,
    Duration? timeout,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
  }) {
    return _delegate.startSpinner(
      onFinish: onFinish,
      timeout: timeout,
      slowWarningCallback: slowWarningCallback,
      warningColor: warningColor,
    );
  }

  @override
  bool get supportsColor => _delegate.supportsColor;

  @override
  void clear() => _delegate.clear();

  @override
  void checkForFatalLogs() => _delegate.checkForFatalLogs();
}

/// If [logger] is a [DelegatingLogger], walks the delegate chain and returns
/// the first delegate with the matching type.
///
/// Throws a [StateError] if no matching delegate is found.
@override
T asLogger<T extends Logger>(Logger logger) {
  final Logger original = logger;
  while (true) {
    if (logger is T) {
      return logger;
    } else if (logger is DelegatingLogger) {
      logger = logger._delegate;
    } else {
      throw StateError('$original has no ancestor delegate of type $T');
    }
  }
}

class StdoutLogger extends Logger {
  StdoutLogger({
    required this.terminal,
    required Stdio stdio,
    required OutputPreferences outputPreferences,
    StopwatchFactory stopwatchFactory = const StopwatchFactory(),
  }) : _stdio = stdio,
       _outputPreferences = outputPreferences,
       _stopwatchFactory = stopwatchFactory;

  @override
  final Terminal terminal;
  @override
  final OutputPreferences _outputPreferences;
  final Stdio _stdio;
  final StopwatchFactory _stopwatchFactory;

  Status? _status;

  @override
  bool get isVerbose => false;

  @override
  bool get supportsColor => terminal.supportsColor && terminal.isCliAnimationEnabled;

  @override
  bool get hasTerminal => _stdio.stdinHasTerminal;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    hadErrorOutput = true;
    _status?.pause();
    message = wrapText(
      message,
      indent: indent,
      hangingIndent: hangingIndent,
      shouldWrap: wrap ?? _outputPreferences.wrapText,
      columnWidth: _outputPreferences.wrapColumn,
    );
    if (emphasis ?? false) {
      message = terminal.bolden(message);
    }
    message = terminal.color(message, color ?? TerminalColor.red);
    writeToStdErr('$message\n');
    if (stackTrace != null) {
      writeToStdErr('$stackTrace\n');
    }
    _status?.resume();
  }

  @override
  void printWarning(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    hadWarningOutput = hadWarningOutput || fatal;
    _status?.pause();
    message = wrapText(
      message,
      indent: indent,
      hangingIndent: hangingIndent,
      shouldWrap: wrap ?? _outputPreferences.wrapText,
      columnWidth: _outputPreferences.wrapColumn,
    );
    if (emphasis ?? false) {
      message = terminal.bolden(message);
    }
    message = terminal.color(message, color ?? TerminalColor.cyan);
    writeToStdErr('$message\n');
    _status?.resume();
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _status?.pause();
    message = wrapText(
      message,
      indent: indent,
      hangingIndent: hangingIndent,
      shouldWrap: wrap ?? _outputPreferences.wrapText,
      columnWidth: _outputPreferences.wrapColumn,
    );
    if (emphasis ?? false) {
      message = terminal.bolden(message);
    }
    if (color != null) {
      message = terminal.color(message, color);
    }
    if (newline ?? true) {
      message = '$message\n';
    }
    writeToStdOut(message);
    _status?.resume();
  }

  @override
  void printBox(String message, {String? title}) {
    _status?.pause();
    _generateBox(
      title: title,
      message: message,
      wrapColumn: _outputPreferences.wrapColumn,
      terminal: terminal,
      write: writeToStdOut,
    );
    _status?.resume();
  }

  @protected
  void writeToStdOut(String message) => _stdio.stdoutWrite(message);

  @protected
  void writeToStdErr(String message) => _stdio.stderrWrite(message);

  @override
  void printTrace(String message) {}

  @override
  Status startProgress(
    String message, {
    String? progressId,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    if (_status != null) {
      // Ignore nested progresses; return a no-op status object.
      return SilentStatus(stopwatch: _stopwatchFactory.createStopwatch())..start();
    }
    if (supportsColor) {
      _status = SpinnerStatus(
        message: message,
        padding: progressIndicatorPadding,
        onFinish: _clearStatus,
        stdio: _stdio,
        stopwatch: _stopwatchFactory.createStopwatch(),
        terminal: terminal,
      )..start();
    } else {
      _status = SummaryStatus(
        message: message,
        padding: progressIndicatorPadding,
        onFinish: _clearStatus,
        stdio: _stdio,
        stopwatch: _stopwatchFactory.createStopwatch(),
      )..start();
    }
    return _status!;
  }

  @override
  Status startSpinner({
    VoidCallback? onFinish,
    Duration? timeout,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
  }) {
    if (_status != null || !supportsColor) {
      return SilentStatus(onFinish: onFinish, stopwatch: _stopwatchFactory.createStopwatch())
        ..start();
    }
    _status = AnonymousSpinnerStatus(
      onFinish: () {
        if (onFinish != null) {
          onFinish();
        }
        _clearStatus();
      },
      stdio: _stdio,
      stopwatch: _stopwatchFactory.createStopwatch(),
      terminal: terminal,
      timeout: timeout,
      slowWarningCallback: slowWarningCallback,
      warningColor: warningColor,
    )..start();
    return _status!;
  }

  void _clearStatus() {
    _status = null;
  }

  @override
  void sendEvent(String name, [Map<String, dynamic>? args]) {}

  @override
  void clear() {
    _status?.pause();
    writeToStdOut('${terminal.clearScreen()}\n');
    _status?.resume();
  }
}

typedef _Writer = void Function(String message);

/// Wraps the message in a box, and writes the bytes by calling [write].
///
///  Example output:
///
///   ┌─ [title] ─┐
///   │ [message] │
///   └───────────┘
///
/// When [title] is provided, the box will have a title above it.
///
/// The box width never exceeds [wrapColumn].
///
/// If [wrapColumn] is not provided, the default value is 100.
void _generateBox({
  required String message,
  required int wrapColumn,
  required _Writer write,
  required Terminal terminal,
  String? title,
}) {
  const int kPaddingLeftRight = 1;
  const int kEdges = 2;

  final int maxTextWidthPerLine = wrapColumn - kEdges - kPaddingLeftRight * 2;
  final List<String> lines = wrapText(
    message,
    shouldWrap: true,
    columnWidth: maxTextWidthPerLine,
  ).split('\n');
  final List<int> lineWidth = lines.map((String line) => _getColumnSize(line)).toList();
  final int maxColumnSize = lineWidth.reduce((int currLen, int maxLen) => max(currLen, maxLen));
  final int textWidth = min(maxColumnSize, maxTextWidthPerLine);
  final int textWithPaddingWidth = textWidth + kPaddingLeftRight * 2;

  write('\n');

  // Write `┌─ [title] ─┐`.
  write('┌');
  write('─');
  if (title == null) {
    write('─' * (textWithPaddingWidth - 1));
  } else {
    write(' ${terminal.bolden(title)} ');
    write('─' * (textWithPaddingWidth - title.length - 3));
  }
  write('┐');
  write('\n');

  // Write `│ [message] │`.
  for (int lineIdx = 0; lineIdx < lines.length; lineIdx++) {
    write('│');
    write(' ' * kPaddingLeftRight);
    write(lines[lineIdx]);
    final int remainingSpacesToEnd = textWidth - lineWidth[lineIdx];
    write(' ' * (remainingSpacesToEnd + kPaddingLeftRight));
    write('│');
    write('\n');
  }

  // Write `└───────────┘`.
  write('└');
  write('─' * textWithPaddingWidth);
  write('┘');
  write('\n');
}

final RegExp _ansiEscapePattern = RegExp('\x1B\\[[\x30-\x3F]*[\x20-\x2F]*[\x40-\x7E]');

int _getColumnSize(String line) {
  // Remove ANSI escape characters from the string.
  return line.replaceAll(_ansiEscapePattern, '').length;
}

/// A [StdoutLogger] which replaces Unicode characters that cannot be printed to
/// the Windows console with alternative symbols.
///
/// By default, Windows uses either "Consolas" or "Lucida Console" as fonts to
/// render text in the console. Both fonts only have a limited character set.
/// Unicode characters, that are not available in either of the two default
/// fonts, should be replaced by this class with printable symbols. Otherwise,
/// they will show up as the unrepresentable character symbol '�'.
class WindowsStdoutLogger extends StdoutLogger {
  WindowsStdoutLogger({
    required super.terminal,
    required super.stdio,
    required super.outputPreferences,
    super.stopwatchFactory,
  });

  @override
  void writeToStdOut(String message) {
    final String windowsMessage =
        terminal.supportsEmoji
            ? message
            : message
                .replaceAll('🔥', '')
                .replaceAll('🖼️', '')
                .replaceAll('✗', 'X')
                .replaceAll('✓', '√')
                .replaceAll('🔨', '')
                .replaceAll('💪', '')
                .replaceAll('⚠️', '!')
                .replaceAll('✏️', '');
    _stdio.stdoutWrite(windowsMessage);
  }
}

class BufferLogger extends Logger {
  BufferLogger({
    required this.terminal,
    required OutputPreferences outputPreferences,
    StopwatchFactory stopwatchFactory = const StopwatchFactory(),
    bool verbose = false,
  }) : _outputPreferences = outputPreferences,
       _stopwatchFactory = stopwatchFactory,
       _verbose = verbose;

  /// Create a [BufferLogger] with test preferences.
  BufferLogger.test({
    Terminal? terminal,
    OutputPreferences? outputPreferences,
    bool verbose = false,
  }) : terminal = terminal ?? Terminal.test(),
       _outputPreferences = outputPreferences ?? OutputPreferences.test(),
       _stopwatchFactory = const StopwatchFactory(),
       _verbose = verbose;

  @override
  final OutputPreferences _outputPreferences;

  @override
  final Terminal terminal;

  final StopwatchFactory _stopwatchFactory;

  final bool _verbose;

  @override
  bool get isVerbose => _verbose;

  @override
  bool get supportsColor => terminal.supportsColor && terminal.isCliAnimationEnabled;

  final StringBuffer _error = StringBuffer();
  final StringBuffer _warning = StringBuffer();
  final StringBuffer _status = StringBuffer();
  final StringBuffer _trace = StringBuffer();
  final StringBuffer _events = StringBuffer();

  String get errorText => _error.toString();
  String get warningText => _warning.toString();
  String get statusText => _status.toString();
  String get traceText => _trace.toString();
  String get eventText => _events.toString();

  @override
  bool get hasTerminal => false;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    hadErrorOutput = true;
    final StringBuffer errorMessage = StringBuffer();
    errorMessage.write(message);
    if (stackTrace != null) {
      errorMessage.writeln();
      errorMessage.write(stackTrace);
    }
    _error.writeln(
      terminal.color(
        wrapText(
          errorMessage.toString(),
          indent: indent,
          hangingIndent: hangingIndent,
          shouldWrap: wrap ?? _outputPreferences.wrapText,
          columnWidth: _outputPreferences.wrapColumn,
        ),
        color ?? TerminalColor.red,
      ),
    );
  }

  @override
  void printWarning(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    hadWarningOutput = hadWarningOutput || fatal;
    _warning.writeln(
      terminal.color(
        wrapText(
          message,
          indent: indent,
          hangingIndent: hangingIndent,
          shouldWrap: wrap ?? _outputPreferences.wrapText,
          columnWidth: _outputPreferences.wrapColumn,
        ),
        color ?? TerminalColor.cyan,
      ),
    );
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    if (newline ?? true) {
      _status.writeln(
        wrapText(
          message,
          indent: indent,
          hangingIndent: hangingIndent,
          shouldWrap: wrap ?? _outputPreferences.wrapText,
          columnWidth: _outputPreferences.wrapColumn,
        ),
      );
    } else {
      _status.write(
        wrapText(
          message,
          indent: indent,
          hangingIndent: hangingIndent,
          shouldWrap: wrap ?? _outputPreferences.wrapText,
          columnWidth: _outputPreferences.wrapColumn,
        ),
      );
    }
  }

  @override
  void printBox(String message, {String? title}) {
    _generateBox(
      title: title,
      message: message,
      wrapColumn: _outputPreferences.wrapColumn,
      terminal: terminal,
      write: _status.write,
    );
  }

  @override
  void printTrace(String message) => _trace.writeln(message);

  @override
  Status startProgress(
    String message, {
    String? progressId,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    printStatus(message);
    return SilentStatus(stopwatch: _stopwatchFactory.createStopwatch())..start();
  }

  @override
  Status startSpinner({
    VoidCallback? onFinish,
    Duration? timeout,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
  }) {
    return SilentStatus(stopwatch: _stopwatchFactory.createStopwatch(), onFinish: onFinish)
      ..start();
  }

  @override
  void clear() {
    _error.clear();
    _status.clear();
    _trace.clear();
    _events.clear();
  }

  @override
  void sendEvent(String name, [Map<String, dynamic>? args]) {
    _events.write(json.encode(<String, Object?>{'name': name, 'args': args}));
  }
}

class VerboseLogger extends DelegatingLogger {
  VerboseLogger(super.parent, {StopwatchFactory stopwatchFactory = const StopwatchFactory()})
    : _stopwatch = stopwatchFactory.createStopwatch(),
      _stopwatchFactory = stopwatchFactory {
    _stopwatch.start();
  }

  final Stopwatch _stopwatch;

  final StopwatchFactory _stopwatchFactory;

  @override
  bool get isVerbose => true;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    hadErrorOutput = true;
    _emit(
      _LogType.error,
      wrapText(
        message,
        indent: indent,
        hangingIndent: hangingIndent,
        shouldWrap: wrap ?? _outputPreferences.wrapText,
        columnWidth: _outputPreferences.wrapColumn,
      ),
      stackTrace,
    );
  }

  @override
  void printWarning(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    hadWarningOutput = true;
    _emit(
      _LogType.warning,
      wrapText(
        message,
        indent: indent,
        hangingIndent: hangingIndent,
        shouldWrap: wrap ?? _outputPreferences.wrapText,
        columnWidth: _outputPreferences.wrapColumn,
      ),
      stackTrace,
    );
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _emit(
      _LogType.status,
      wrapText(
        message,
        indent: indent,
        hangingIndent: hangingIndent,
        shouldWrap: wrap ?? _outputPreferences.wrapText,
        columnWidth: _outputPreferences.wrapColumn,
      ),
    );
  }

  @override
  void printBox(String message, {String? title}) {
    String composedMessage = '';
    _generateBox(
      title: title,
      message: message,
      wrapColumn: _outputPreferences.wrapColumn,
      terminal: terminal,
      write: (String line) {
        composedMessage += line;
      },
    );
    _emit(_LogType.status, composedMessage);
  }

  @override
  void printTrace(String message) {
    _emit(_LogType.trace, message);
  }

  @override
  Status startProgress(
    String message, {
    String? progressId,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    printStatus(message);
    final Stopwatch timer = _stopwatchFactory.createStopwatch()..start();
    return SilentStatus(
      // This is intentionally a different stopwatch than above.
      stopwatch: _stopwatchFactory.createStopwatch(),
      onFinish: () {
        String time;
        if (timer.elapsed.inSeconds > 2) {
          time = getElapsedAsSeconds(timer.elapsed);
        } else {
          time = getElapsedAsMilliseconds(timer.elapsed);
        }
        printTrace('$message (completed in $time)');
      },
    )..start();
  }

  void _emit(_LogType type, String message, [StackTrace? stackTrace]) {
    if (message.trim().isEmpty) {
      return;
    }

    final int millis = _stopwatch.elapsedMilliseconds;
    _stopwatch.reset();

    String prefix;
    const int prefixWidth = 8;
    if (millis == 0) {
      prefix = ''.padLeft(prefixWidth);
    } else {
      prefix = '+$millis ms'.padLeft(prefixWidth);
      if (millis >= 100) {
        prefix = terminal.bolden(prefix);
      }
    }
    prefix = '[$prefix] ';

    final String indent = ''.padLeft(prefix.length);
    final String indentMessage = message.replaceAll('\n', '\n$indent');

    switch (type) {
      case _LogType.error:
        super.printError(prefix + terminal.bolden(indentMessage));
        if (stackTrace != null) {
          super.printError(indent + stackTrace.toString().replaceAll('\n', '\n$indent'));
        }
      case _LogType.warning:
        super.printWarning(prefix + terminal.bolden(indentMessage));
      case _LogType.status:
        super.printStatus(prefix + terminal.bolden(indentMessage));
      case _LogType.trace:
        // This seems wrong, since there is a 'printTrace' to call on the
        // superclass, but it's actually the entire point of this logger: to
        // make things more verbose than they normally would be.
        super.printStatus(prefix + indentMessage);
    }
  }

  @override
  void sendEvent(String name, [Map<String, dynamic>? args]) {}
}

class PrefixedErrorLogger extends DelegatingLogger {
  PrefixedErrorLogger(super.parent);

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    hadErrorOutput = true;
    if (message.trim().isNotEmpty) {
      message = 'ERROR: $message';
    }
    super.printError(
      message,
      stackTrace: stackTrace,
      emphasis: emphasis,
      color: color,
      indent: indent,
      hangingIndent: hangingIndent,
      wrap: wrap,
    );
  }
}

enum _LogType { error, warning, status, trace }

typedef SlowWarningCallback = String Function();

/// A [Status] class begins when start is called, and may produce progress
/// information asynchronously.
///
/// The [SilentStatus] class never has any output.
///
/// The [SpinnerStatus] subclass shows a message with a spinner, and replaces it
/// with timing information when stopped. When canceled, the information isn't
/// shown. In either case, a newline is printed.
///
/// The [AnonymousSpinnerStatus] subclass just shows a spinner.
///
/// The [SummaryStatus] subclass shows only a static message (without an
/// indicator), then updates it when the operation ends.
///
/// Generally, consider `logger.startProgress` instead of directly creating
/// a [Status] or one of its subclasses.
abstract class Status {
  Status({VoidCallback? onFinish, required Stopwatch stopwatch, Duration? timeout})
    : _timeout = timeout,
      _onFinish = onFinish,
      _stopwatch = stopwatch;

  final VoidCallback? _onFinish;
  final Duration? _timeout;

  final Stopwatch _stopwatch;

  String get _elapsedTime {
    if (_stopwatch.elapsed.inSeconds > 2) {
      return getElapsedAsSeconds(_stopwatch.elapsed);
    }
    return getElapsedAsMilliseconds(_stopwatch.elapsed);
  }

  bool get seemsSlow => _timeout != null && _stopwatch.elapsed > _timeout;

  /// Call to start spinning.
  void start() {
    assert(!_stopwatch.isRunning);
    _stopwatch.start();
  }

  /// Call to stop spinning after success.
  void stop() {
    finish();
  }

  /// Call to cancel the spinner after failure or cancellation.
  void cancel() {
    finish();
  }

  /// Call to clear the current line but not end the progress.
  void pause() {}

  /// Call to resume after a pause.
  void resume() {}

  @protected
  void finish() {
    assert(_stopwatch.isRunning);
    _stopwatch.stop();
    _onFinish?.call();
  }
}

/// A [Status] that shows nothing.
class SilentStatus extends Status {
  SilentStatus({required super.stopwatch, super.onFinish});

  @override
  void finish() {
    _onFinish?.call();
  }
}

const int _kTimePadding = 8; // should fit "99,999ms"

/// Constructor writes [message] to [stdout]. On [cancel] or [stop], will call
/// [onFinish]. On [stop], will additionally print out summary information.
class SummaryStatus extends Status {
  SummaryStatus({
    String message = '',
    required super.stopwatch,
    int padding = kDefaultStatusPadding,
    super.onFinish,
    required Stdio stdio,
  }) : _padding = padding,
       _message = message,
       _stdio = stdio;

  final String _message;
  final int _padding;
  final Stdio _stdio;

  bool _messageShowingOnCurrentLine = false;

  @override
  void start() {
    _printMessage();
    super.start();
  }

  void _writeToStdOut(String message) => _stdio.stdoutWrite(message);

  void _printMessage() {
    assert(!_messageShowingOnCurrentLine);
    _writeToStdOut('${_message.padRight(_padding)}     ');
    _messageShowingOnCurrentLine = true;
  }

  @override
  void stop() {
    if (!_messageShowingOnCurrentLine) {
      _printMessage();
    }
    super.stop();
    assert(_messageShowingOnCurrentLine);
    _writeToStdOut(_elapsedTime.padLeft(_kTimePadding));
    _writeToStdOut('\n');
  }

  @override
  void cancel() {
    super.cancel();
    if (_messageShowingOnCurrentLine) {
      _writeToStdOut('\n');
    }
  }

  @override
  void pause() {
    super.pause();
    if (_messageShowingOnCurrentLine) {
      _writeToStdOut('\n');
      _messageShowingOnCurrentLine = false;
    }
  }
}

/// A kind of animated [Status] that has no message.
///
/// Call [pause] before outputting any text while this is running.
class AnonymousSpinnerStatus extends Status {
  AnonymousSpinnerStatus({
    super.onFinish,
    required super.stopwatch,
    required Stdio stdio,
    required Terminal terminal,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
    super.timeout,
  }) : _warningColor = warningColor,
       _slowWarningCallback = slowWarningCallback,
       _stdio = stdio,
       _terminal = terminal,
       _animation = _selectAnimation(terminal);

  final Stdio _stdio;
  final Terminal _terminal;
  String _slowWarning = '';
  final SlowWarningCallback? _slowWarningCallback;
  final TerminalColor? _warningColor;

  static const String _backspaceChar = '\b';
  static const String _clearChar = ' ';

  static const List<String> _emojiAnimations = <String>[
    '⣾⣽⣻⢿⡿⣟⣯⣷', // counter-clockwise
    '⣾⣷⣯⣟⡿⢿⣻⣽', // clockwise
    '⣾⣷⣯⣟⡿⢿⣻⣽⣷⣾⣽⣻⢿⡿⣟⣯⣷', // bouncing clockwise and counter-clockwise
    '⣾⣷⣯⣽⣻⣟⡿⢿⣻⣟⣯⣽', // snaking
    '⣾⣽⣻⢿⣿⣷⣯⣟⡿⣿', // alternating rain
    '⣀⣠⣤⣦⣶⣾⣿⡿⠿⠻⠛⠋⠉⠙⠛⠟⠿⢿⣿⣷⣶⣴⣤⣄', // crawl up and down, large
    '⠙⠚⠖⠦⢤⣠⣄⡤⠴⠲⠓⠋', // crawl up and down, small
    '⣀⡠⠤⠔⠒⠊⠉⠑⠒⠢⠤⢄', // crawl up and down, tiny
    '⡀⣄⣦⢷⠻⠙⠈⠀⠁⠋⠟⡾⣴⣠⢀⠀', // slide up and down
    '⠙⠸⢰⣠⣄⡆⠇⠋', // clockwise line
    '⠁⠈⠐⠠⢀⡀⠄⠂', // clockwise dot
    '⢇⢣⢱⡸⡜⡎', // vertical wobble up
    '⡇⡎⡜⡸⢸⢱⢣⢇', // vertical wobble down
    '⡀⣀⣐⣒⣖⣶⣾⣿⢿⠿⠯⠭⠩⠉⠁⠀', // swirl
    '⠁⠐⠄⢀⢈⢂⢠⣀⣁⣐⣄⣌⣆⣤⣥⣴⣼⣶⣷⣿⣾⣶⣦⣤⣠⣀⡀⠀⠀', // snowing and melting
    '⠁⠋⠞⡴⣠⢀⠀⠈⠙⠻⢷⣦⣄⡀⠀⠉⠛⠲⢤⢀⠀', // falling water
    '⠄⡢⢑⠈⠀⢀⣠⣤⡶⠞⠋⠁⠀⠈⠙⠳⣆⡀⠀⠆⡷⣹⢈⠀⠐⠪⢅⡀⠀', // fireworks
    '⠐⢐⢒⣒⣲⣶⣷⣿⡿⡷⡧⠧⠇⠃⠁⠀⡀⡠⡡⡱⣱⣳⣷⣿⢿⢯⢧⠧⠣⠃⠂⠀⠈⠨⠸⠺⡺⡾⡿⣿⡿⡷⡗⡇⡅⡄⠄⠀⡀⡐⣐⣒⣓⣳⣻⣿⣾⣼⡼⡸⡘⡈⠈⠀', // fade
    '⢸⡯⠭⠅⢸⣇⣀⡀⢸⣇⣸⡇⠈⢹⡏⠁⠈⢹⡏⠁⢸⣯⣭⡅⢸⡯⢕⡂⠀⠀', // text crawl
  ];

  static const List<String> _asciiAnimations = <String>[r'-\|/'];

  static List<String> _selectAnimation(Terminal terminal) {
    final List<String> animations = terminal.supportsEmoji ? _emojiAnimations : _asciiAnimations;
    return animations[terminal.preferredStyle % animations.length].runes
        .map<String>((int scalar) => String.fromCharCode(scalar))
        .toList();
  }

  final List<String> _animation;

  Timer? _timer;

  int _ticks = 0;
  @visibleForTesting
  int get ticks => _ticks;

  int _lastAnimationFrameLength = 0;
  bool _timedOut = false;

  String get _currentAnimationFrame => _animation[ticks % _animation.length];
  int get _currentLineLength => _lastAnimationFrameLength + _slowWarning.length;

  void _writeToStdOut(String message) => _stdio.stdoutWrite(message);

  void _clear(int length) {
    _writeToStdOut(
      '${_backspaceChar * length}'
      '${_clearChar * length}'
      '${_backspaceChar * length}',
    );
  }

  @override
  void start() {
    super.start();
    assert(_timer == null);
    _startSpinner();
  }

  void _startSpinner() {
    _timer = Timer.periodic(const Duration(milliseconds: 100), _callback);
    _callback(_timer!);
  }

  void _callback(Timer timer) {
    assert(_timer == timer);
    assert(timer.isActive);
    _writeToStdOut(_backspaceChar * _lastAnimationFrameLength);
    _ticks += 1;
    if (seemsSlow) {
      if (!_timedOut) {
        _timedOut = true;
        if (_currentLineLength > _lastAnimationFrameLength) {
          _clear(_currentLineLength - _lastAnimationFrameLength);
        }
      }
      final SlowWarningCallback? callback = _slowWarningCallback;
      if (_slowWarning.isEmpty && callback != null) {
        final TerminalColor? color = _warningColor;
        if (color != null) {
          _slowWarning = _terminal.color(callback(), color);
        } else {
          _slowWarning = callback();
        }

        _writeToStdOut(_slowWarning);
      }
    }
    final String newFrame = _currentAnimationFrame;
    _lastAnimationFrameLength = newFrame.runes.length;
    _writeToStdOut(newFrame);
  }

  @override
  void pause() {
    assert(_timer != null);
    assert(_timer!.isActive);
    if (_terminal.supportsColor) {
      _writeToStdOut('\r\x1B[K'); // go to start of line and clear line
    } else {
      _clear(_currentLineLength);
    }
    _lastAnimationFrameLength = 0;
    _timer?.cancel();
  }

  @override
  void resume() {
    assert(_timer != null);
    assert(!_timer!.isActive);
    _startSpinner();
  }

  @override
  void finish() {
    assert(_timer != null);
    assert(_timer!.isActive);
    _timer?.cancel();
    _timer = null;
    _clear(_lastAnimationFrameLength + _slowWarning.length);
    _slowWarning = '';
    _lastAnimationFrameLength = 0;
    super.finish();
  }
}

/// An animated version of [Status].
///
/// The constructor writes [message] to [stdout] with padding, then starts an
/// indeterminate progress indicator animation.
///
/// On [cancel] or [stop], will call [onFinish]. On [stop], will
/// additionally print out summary information.
///
/// Call [pause] before outputting any text while this is running.
class SpinnerStatus extends AnonymousSpinnerStatus {
  SpinnerStatus({
    required String message,
    int padding = kDefaultStatusPadding,
    super.onFinish,
    required super.stopwatch,
    required super.stdio,
    required super.terminal,
  }) : _padding = padding,
       _message = message;

  final String _message;
  final int _padding;

  static final String _margin = AnonymousSpinnerStatus._clearChar * (5 + _kTimePadding - 1);

  int _totalMessageLength = 0;

  @override
  int get _currentLineLength => _totalMessageLength + super._currentLineLength;

  @override
  void start() {
    _printStatus();
    super.start();
  }

  void _printStatus() {
    final String line = '${_message.padRight(_padding)}$_margin';
    _totalMessageLength = line.length;
    _writeToStdOut(line);
  }

  @override
  void pause() {
    super.pause();
    _totalMessageLength = 0;
  }

  @override
  void resume() {
    _printStatus();
    super.resume();
  }

  @override
  void stop() {
    super.stop(); // calls finish, which clears the spinner
    assert(_totalMessageLength > _kTimePadding);
    _writeToStdOut(AnonymousSpinnerStatus._backspaceChar * (_kTimePadding - 1));
    _writeToStdOut(_elapsedTime.padLeft(_kTimePadding));
    _writeToStdOut('\n');
  }

  @override
  void cancel() {
    super.cancel(); // calls finish, which clears the spinner
    assert(_totalMessageLength > 0);
    _writeToStdOut('\n');
  }
}
