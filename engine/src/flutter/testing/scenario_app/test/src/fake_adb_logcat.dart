import '../../bin/utils/adb_logcat_filtering.dart';

/// Simulates the output of `adb logcat`, i.e. for testing.
///
/// ## Example
///
/// ```dart
/// final FakeAdbLogcat logcat = FakeAdbLogcat();
/// final FakeAdbProcess process = logcat.withProcess();
/// process.info('ActivityManager', 'Force stopping dev.flutter.scenarios appid=10226 user=0: start instr');
/// // ...
/// final List<String> logLines = logcat.drain();
/// // ...
/// ```
final class FakeAdbLogcat {
  final List<String> _lines = <String>[];
  final Map<int, FakeAdbProcess> _processById = <int, FakeAdbProcess>{};

  /// The current date and time.
  DateTime _now = DateTime.now();

  /// Returns the date and time for the next log line.
  ///
  /// Time is progressed by 1 second each time this method is called.
  DateTime _progressTime({Duration by = const Duration(seconds: 1)}) {
    _now = _now.add(by);
    return _now;
  }

  /// `02-22 13:54:39.839`
  static String _formatTime(DateTime time) {
    return '${time.month.toString().padLeft(2, '0')}-'
        '${time.day.toString().padLeft(2, '0')} '
        '${time.hour.toString().padLeft(2, '0')}:'
        '${time.minute.toString().padLeft(2, '0')}:'
        '${time.second.toString().padLeft(2, '0')}.'
        '${time.millisecond.toString().padLeft(3, '0')}';
  }

  void _write({
    required int processId,
    required int threadId,
    required String severity,
    required String tag,
    required String message,
  }) {
    final DateTime time = _progressTime();
    final String line = '${_formatTime(time)}   $processId  $threadId $severity $tag: $message';
    assert(AdbLogLine.tryParse(line) != null, 'Invalid log line: $line');
    _lines.add(line);
  }

  /// Drains the stored log lines and returns them.
  List<String> drain() {
    final List<String> result = List<String>.from(_lines);
    _lines.clear();
    return result;
  }

  /// Creates a new process writing to this logcat.
  ///
  /// Optionally specify a [processId] to use for the process, otherwise a
  /// simple default is used (sequential numbers starting from 1000).
  FakeAdbProcess process({int? processId}) {
    processId ??= 1000 + _processById.length;
    return _processById.putIfAbsent(processId, () => _createProcess(processId: processId!));
  }

  FakeAdbProcess _createProcess({required int processId}) {
    return FakeAdbProcess._(this, processId: processId);
  }
}

/// A stateful fixture that represents a fake process writing to `adb logcat`.
///
/// See [FakeAdbLogcat.process] for how to create this fixture.
final class FakeAdbProcess {
  const FakeAdbProcess._(this._logcat, {required this.processId});

  final FakeAdbLogcat _logcat;

  /// The process ID of this process.
  final int processId;

  /// Writes a debug log message.
  void debug(String tag, String message, {int threadId = 1}) {
    _logcat._write(
      processId: processId,
      threadId: threadId,
      severity: 'D',
      tag: tag,
      message: message,
    );
  }

  /// Writes an info log message.
  void info(String tag, String message, {int threadId = 1}) {
    _logcat._write(
      processId: processId,
      threadId: threadId,
      severity: 'I',
      tag: tag,
      message: message,
    );
  }

  /// Writes a warning log message.
  void warning(String tag, String message, {int threadId = 1}) {
    _logcat._write(
      processId: processId,
      threadId: threadId,
      severity: 'W',
      tag: tag,
      message: message,
    );
  }

  /// Writes an error log message.
  void error(String tag, String message, {int threadId = 1}) {
    _logcat._write(
      processId: processId,
      threadId: threadId,
      severity: 'E',
      tag: tag,
      message: message,
    );
  }

  /// Writes a fatal log message.
  void fatal(String tag, String message, {int threadId = 1}) {
    _logcat._write(
      processId: processId,
      threadId: threadId,
      severity: 'F',
      tag: tag,
      message: message,
    );
  }
}
