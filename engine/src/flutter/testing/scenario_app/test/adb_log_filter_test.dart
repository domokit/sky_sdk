import 'package:test/test.dart';

import '../bin/utils/adb_logcat_filtering.dart';
import 'src/fake_adb_logcat.dart';

void main() {
  /// Simulates the filtering of logcat output [lines].
  Iterable<String> filter(Iterable<String> lines, {int? filterProcessId}) {
    if (lines.isEmpty) {
      throw StateError('No log lines to filter. This is unexpected.');
    }
    return lines.where((String line) {
      final AdbLogLine? logLine = AdbLogLine.tryParse(line);
      if (logLine == null) {
        throw StateError('Invalid log line: $line');
      }
      final bool isVerbose = logLine.isVerbose(filterProcessId: filterProcessId?.toString());
      return !isVerbose;
    });
  }

  test('should always retain fatal logs', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    process.fatal('Something', 'A bad thing happened');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, hasLength(1));
    expect(filtered.first, contains('Something: A bad thing happened'));
  });

  test('should never retain debug logs', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    final String tag = AdbLogLine.knownNoiseTags.first;
    process.debug(tag, 'A debug message');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, isEmpty);
  });

  test('should never retain logs from known "noise" tags', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    final String tag = AdbLogLine.knownNoiseTags.first;
    process.info(tag, 'Flutter flutter flutter');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, isEmpty);
  });

  test('should always retain logs from known "useful" tags', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    final String tag = AdbLogLine.knownUsefulTags.first;
    process.info(tag, 'A useful message');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, hasLength(1));
    expect(filtered.first, contains('$tag: A useful message'));
  });

  test('if a process ID is passed, retain the log', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    process.info('SomeTag', 'A message');

    final Iterable<String> filtered = filter(logcat.drain(), filterProcessId: process.processId);
    expect(filtered, hasLength(1));
    expect(filtered.first, contains('SomeTag: A message'));
  });

  test('even if a process ID passed, retain logs containing "flutter"', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    process.info('SomeTag', 'A message with flutter');

    final Iterable<String> filtered = filter(logcat.drain(), filterProcessId: process.processId);
    expect(filtered, hasLength(1));
    expect(filtered.first, contains('SomeTag: A message with flutter'));
  });

  test('should retain E-level flags from known "useful" error tags', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();
    final String tag = AdbLogLine.knownUsefulErrorTags.first;
    process.error(tag, 'An error message');
    process.info(tag, 'An info message');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, hasLength(1));
    expect(filtered.first, contains('$tag: An error message'));
  });

  test('should filter out error logs from unimportant processes', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();
    final FakeAdbProcess process = logcat.process();

    // I hate this one.
    const String tag = 'gs.intelligence';
    process.error(tag, 'No package ID ff found for resource ID 0xffffffff.');

    final Iterable<String> filtered = filter(logcat.drain());
    expect(filtered, isEmpty);
  });

  test('should filter the flutter-launched process, not just any process', () {
    final FakeAdbLogcat logcat = FakeAdbLogcat();

    final FakeAdbProcess device = logcat.process();
    final FakeAdbProcess unrelated = logcat.process();
    final FakeAdbProcess flutter = logcat.process();

    device.info(
      AdbLogLine.activityManagerTag,
      'Start proc ${unrelated.processId}:com.example.unrelated',
    );

    List<String> rawLines = logcat.drain();
    expect(rawLines, hasLength(1));
    AdbLogLine parsedLogLine = AdbLogLine.tryParse(rawLines.single)!;
    expect(parsedLogLine.tryParseProcess(), isNull);

    device.info(
      AdbLogLine.activityManagerTag,
      'Start proc ${flutter.processId}:${AdbLogLine.flutterProcessName}',
    );

    rawLines = logcat.drain();
    expect(rawLines, hasLength(1));
    parsedLogLine = AdbLogLine.tryParse(rawLines.single)!;
    expect(parsedLogLine.tryParseProcess(), '${flutter.processId}');
  });
}
