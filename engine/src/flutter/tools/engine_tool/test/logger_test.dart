// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_tool/src/logger.dart';
import 'package:logging/logging.dart' as log;
import 'package:test/test.dart';

void main() {
  List<String> stringsFromLogs(List<log.LogRecord> logs) {
    return logs.map((log.LogRecord r) => r.message).toList();
  }

  test('Setting the level works', () {
    final Logger logger = Logger.test((_) {}, level: Logger.infoLevel);
    expect(logger.level, equals(Logger.infoLevel));
  });

  test('error messages are recorded at the default log level', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.error('Error');
    expect(stringsFromLogs(testLogs), equals(<String>['Error\n']));
  });

  test('warning messages are recorded at the default log level', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.warning('Warning');
    expect(stringsFromLogs(testLogs), equals(<String>['Warning\n']));
  });

  test('status messages are recorded at the default log level', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.status('Status');
    expect(stringsFromLogs(testLogs), equals(<String>['Status\n']));
  });

  test('info messages are not recorded at the default log level', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.info('info');
    expect(stringsFromLogs(testLogs), equals(<String>[]));
  });

  test('info messages are recorded at the infoLevel log level', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add, level: Logger.infoLevel);
    logger.info('info');
    expect(stringsFromLogs(testLogs), equals(<String>['info\n']));
  });

  test('indent indents the message', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.status('Status', indent: 1);
    expect(stringsFromLogs(testLogs), equals(<String>[' Status\n']));
  });

  test('newlines in error() can be disabled', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.error('Error', newline: false);
    expect(stringsFromLogs(testLogs), equals(<String>['Error']));
  });

  test('newlines in warning() can be disabled', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.warning('Warning', newline: false);
    expect(stringsFromLogs(testLogs), equals(<String>['Warning']));
  });

  test('newlines in status() can be disabled', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    logger.status('Status', newline: false);
    expect(stringsFromLogs(testLogs), equals(<String>['Status']));
  });

  test('newlines in info() can be disabled', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add, level: Logger.infoLevel);
    logger.info('info', newline: false);
    expect(stringsFromLogs(testLogs), equals(<String>['info']));
  });

  test('fatal throws exception', () {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add, level: Logger.infoLevel);
    bool caught = false;
    try {
      logger.fatal('test', newline: false);
    } on FatalError catch (_) {
      caught = true;
    }
    expect(caught, equals(true));
    expect(stringsFromLogs(testLogs), equals(<String>['test']));
  });

  test('fitToWidth', () {
    expect(Logger.fitToWidth('hello', 0), equals(''));
    expect(Logger.fitToWidth('hello', 1), equals('.'));
    expect(Logger.fitToWidth('hello', 2), equals('..'));
    expect(Logger.fitToWidth('hello', 3), equals('...'));
    expect(Logger.fitToWidth('hello', 4), equals('...o'));
    expect(Logger.fitToWidth('hello', 5), equals('hello'));

    expect(Logger.fitToWidth('foobar', 5), equals('f...r'));

    expect(Logger.fitToWidth('foobarb', 5), equals('f...b'));
    expect(Logger.fitToWidth('foobarb', 6), equals('f...rb'));

    expect(Logger.fitToWidth('foobarba', 5), equals('f...a'));
    expect(Logger.fitToWidth('foobarba', 6), equals('f...ba'));
    expect(Logger.fitToWidth('foobarba', 7), equals('fo...ba'));

    expect(Logger.fitToWidth('hello\n', 0), equals('\n'));
    expect(Logger.fitToWidth('hello\n', 1), equals('.\n'));
    expect(Logger.fitToWidth('hello\n', 2), equals('..\n'));
    expect(Logger.fitToWidth('hello\n', 3), equals('...\n'));
    expect(Logger.fitToWidth('hello\n', 4), equals('...o\n'));
    expect(Logger.fitToWidth('hello\n', 5), equals('hello\n'));

    expect(Logger.fitToWidth('foobar\n', 5), equals('f...r\n'));

    expect(Logger.fitToWidth('foobarb\n', 5), equals('f...b\n'));
    expect(Logger.fitToWidth('foobarb\n', 6), equals('f...rb\n'));

    expect(Logger.fitToWidth('foobarba\n', 5), equals('f...a\n'));
    expect(Logger.fitToWidth('foobarba\n', 6), equals('f...ba\n'));
    expect(Logger.fitToWidth('foobarba\n', 7), equals('fo...ba\n'));
  });

  test('Spinner calls onFinish callback', () {
    final Logger logger = Logger.test((_) {});
    bool called = false;
    final Spinner spinner = logger.startSpinner(
      onFinish: () {
        called = true;
      },
    );
    spinner.finish();
    expect(called, isTrue);
  });
}
