// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:test_api/backend.dart';
/// @docImport 'package:test_api/scaffolding.dart';
library;

import 'dart:async';

import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';
import 'package:meta/meta.dart';
import 'package:test_api/scaffolding.dart' show Timeout;
import 'package:test_api/src/backend/declarer.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/group.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/group_entry.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/invoker.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/live_test.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/message.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/runtime.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/state.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/suite.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/suite_platform.dart'; // ignore: implementation_imports
import 'package:test_api/src/backend/test.dart'; // ignore: implementation_imports

export 'package:test_api/fake.dart' show Fake;

Declarer? _localDeclarer;
Declarer get _declarer {
  final Declarer? declarer = Zone.current[#test.declarer] as Declarer?;
  if (declarer != null) {
    return declarer;
  }
  // If no declarer is defined, this test is being run via `flutter run -t test_file.dart`.
  if (_localDeclarer == null) {
    _localDeclarer = Declarer();
    Future<void>(() {
      Invoker.guard<Future<void>>(() async {
        final _Reporter reporter = _Reporter(color: false); // disable color when run directly.
        // ignore: recursive_getters, this self-call is safe since it will just fetch the declarer instance
        final Group group = _declarer.build();
        final Suite suite = Suite(group, SuitePlatform(Runtime.vm));
        await _runGroup(suite, group, <Group>[], reporter);
        reporter._onDone();
      });
    });
  }
  return _localDeclarer!;
}

Future<void> _runGroup(
  Suite suiteConfig,
  Group group,
  List<Group> parents,
  _Reporter reporter,
) async {
  parents.add(group);
  try {
    final bool skipGroup = group.metadata.skip;
    bool setUpAllSucceeded = true;
    if (!skipGroup && group.setUpAll != null) {
      final LiveTest liveTest = group.setUpAll!.load(suiteConfig, groups: parents);
      await _runLiveTest(suiteConfig, liveTest, reporter, countSuccess: false);
      setUpAllSucceeded = liveTest.state.result.isPassing;
    }
    if (setUpAllSucceeded) {
      for (final GroupEntry entry in group.entries) {
        if (entry is Group) {
          await _runGroup(suiteConfig, entry, parents, reporter);
        } else if (entry.metadata.skip) {
          await _runSkippedTest(suiteConfig, entry as Test, parents, reporter);
        } else {
          final Test test = entry as Test;
          await _runLiveTest(suiteConfig, test.load(suiteConfig, groups: parents), reporter);
        }
      }
    }
    // Even if we're closed or setUpAll failed, we want to run all the
    // teardowns to ensure that any state is properly cleaned up.
    if (!skipGroup && group.tearDownAll != null) {
      final LiveTest liveTest = group.tearDownAll!.load(suiteConfig, groups: parents);
      await _runLiveTest(suiteConfig, liveTest, reporter, countSuccess: false);
    }
  } finally {
    parents.remove(group);
  }
}

Future<void> _runLiveTest(
  Suite suiteConfig,
  LiveTest liveTest,
  _Reporter reporter, {
  bool countSuccess = true,
}) async {
  reporter._onTestStarted(liveTest);
  // Schedule a microtask to ensure that [onTestStarted] fires before the
  // first [LiveTest.onStateChange] event.
  await Future<void>.microtask(liveTest.run);
  // Once the test finishes, use await null to do a coarse-grained event
  // loop pump to avoid starving non-microtask events.
  await null;
  final bool isSuccess = liveTest.state.result.isPassing;
  if (isSuccess) {
    reporter.passed.add(liveTest);
  } else {
    reporter.failed.add(liveTest);
  }
}

Future<void> _runSkippedTest(
  Suite suiteConfig,
  Test test,
  List<Group> parents,
  _Reporter reporter,
) async {
  final LocalTest skipped = LocalTest(test.name, test.metadata, () {}, trace: test.trace);
  if (skipped.metadata.skipReason != null) {
    reporter.log('Skip: ${skipped.metadata.skipReason}');
  }
  final LiveTest liveTest = skipped.load(suiteConfig);
  reporter._onTestStarted(liveTest);
  reporter.skipped.add(skipped);
}

// TODO(nweiz): This and other top-level functions should throw exceptions if
// they're called after the declarer has finished declaring.
/// Creates a new test case with the given description (converted to a string)
/// and body.
///
/// The description will be added to the descriptions of any surrounding
/// [group]s. If [testOn] is passed, it's parsed as a [platform selector][]; the
/// test will only be run on matching platforms.
///
/// [platform selector]: https://github.com/dart-lang/test/tree/master/pkgs/test#platform-selectors
///
/// If [timeout] is passed, it's used to modify or replace the default timeout
/// of 30 seconds. Timeout modifications take precedence in suite-group-test
/// order, so [timeout] will also modify any timeouts set on the group or suite.
///
/// If [skip] is a String or `true`, the test is skipped. If it's a String, it
/// should explain why the test is skipped; this reason will be printed instead
/// of running the test.
///
/// If [tags] is passed, it declares user-defined tags that are applied to the
/// test. These tags can be used to select or skip the test on the command line,
/// or to do bulk test configuration. All tags should be declared in the
/// [package configuration file][configuring tags]. The parameter can be an
/// [Iterable] of tag names, or a [String] representing a single tag.
///
/// If [retry] is passed, the test will be retried the provided number of times
/// before being marked as a failure.
///
/// [configuring tags]: https://github.com/dart-lang/test/blob/44d6cb196f34a93a975ed5f3cb76afcc3a7b39b0/doc/package_config.md#configuring-tags
///
/// [onPlatform] allows tests to be configured on a platform-by-platform
/// basis. It's a map from strings that are parsed as [PlatformSelector]s to
/// annotation classes: [Timeout], [Skip], or lists of those. These
/// annotations apply only on the given platforms. For example:
///
///     test('potentially slow test', () {
///       // ...
///     }, onPlatform: {
///       // This test is especially slow on Windows.
///       'windows': Timeout.factor(2),
///       'browser': [
///         Skip('add browser support'),
///         // This will be slow on browsers once it works on them.
///         Timeout.factor(2)
///       ]
///     });
///
/// If multiple platforms match, the annotations apply in order as through
/// they were in nested groups.
@isTest
void test(
  Object description,
  dynamic Function() body, {
  String? testOn,
  Timeout? timeout,
  dynamic skip,
  dynamic tags,
  Map<String, dynamic>? onPlatform,
  int? retry,
}) {
  _maybeConfigureTearDownForTestFile();
  _declarer.test(
    description.toString(),
    body,
    testOn: testOn,
    timeout: timeout,
    skip: skip,
    onPlatform: onPlatform,
    tags: tags,
    retry: retry,
  );
}

/// Creates a group of tests.
///
/// A group's description (converted to a string) is included in the descriptions
/// of any tests or sub-groups it contains. [setUp] and [tearDown] are also scoped
/// to the containing group.
///
/// If `skip` is a String or `true`, the group is skipped. If it's a String, it
/// should explain why the group is skipped; this reason will be printed instead
/// of running the group's tests.
@isTestGroup
void group(Object description, void Function() body, {dynamic skip, int? retry}) {
  _maybeConfigureTearDownForTestFile();
  _declarer.group(description.toString(), body, skip: skip, retry: retry);
}

/// Registers a function to be run before tests.
///
/// This function will be called before each test is run. The `body` may be
/// asynchronous; if so, it must return a [Future].
///
/// If this is called within a test group, it applies only to tests in that
/// group. The `body` will be run after any set-up callbacks in parent groups or
/// at the top level.
///
/// Each callback at the top level or in a given group will be run in the order
/// they were declared.
void setUp(dynamic Function() body) {
  _maybeConfigureTearDownForTestFile();
  _declarer.setUp(body);
}

/// Registers a function to be run after tests.
///
/// This function will be called after each test is run. The `body` may be
/// asynchronous; if so, it must return a [Future].
///
/// If this is called within a test group, it applies only to tests in that
/// group. The `body` will be run before any tear-down callbacks in parent
/// groups or at the top level.
///
/// Each callback at the top level or in a given group will be run in the
/// reverse of the order they were declared.
///
/// See also [addTearDown], which adds tear-downs to a running test.
void tearDown(dynamic Function() body) {
  _maybeConfigureTearDownForTestFile();
  _declarer.tearDown(body);
}

/// Registers a function to be run once before all tests.
///
/// The `body` may be asynchronous; if so, it must return a [Future].
///
/// If this is called within a test group, The `body` will run before all tests
/// in that group. It will be run after any [setUpAll] callbacks in parent
/// groups or at the top level. It won't be run if none of the tests in the
/// group are run.
///
/// **Note**: This function makes it very easy to accidentally introduce hidden
/// dependencies between tests that should be isolated. In general, you should
/// prefer [setUp], and only use [setUpAll] if the callback is prohibitively
/// slow.
void setUpAll(dynamic Function() body) {
  _maybeConfigureTearDownForTestFile();
  _declarer.setUpAll(body);
}

/// Registers a function to be run once after all tests.
///
/// If this is called within a test group, `body` will run after all tests
/// in that group. It will be run before any [tearDownAll] callbacks in parent
/// groups or at the top level. It won't be run if none of the tests in the
/// group are run.
///
/// **Note**: This function makes it very easy to accidentally introduce hidden
/// dependencies between tests that should be isolated. In general, you should
/// prefer [tearDown], and only use [tearDownAll] if the callback is
/// prohibitively slow.
void tearDownAll(dynamic Function() body) {
  _maybeConfigureTearDownForTestFile();
  _declarer.tearDownAll(body);
}

bool _isTearDownForTestFileConfigured = false;

/// If needed, configures `tearDownAll` after all user defined `tearDownAll` in the test file.
///
/// This function should be invoked in all functions, that may be invoked by user in the test file,
/// to be invoked before any other `tearDownAll`.
void _maybeConfigureTearDownForTestFile() {
  if (_isTearDownForTestFileConfigured || !_shouldConfigureTearDownForTestFile()) {
    return;
  }
  _declarer.tearDownAll(_tearDownForTestFile);
  _isTearDownForTestFileConfigured = true;
}

/// Returns true if tear down for the test file needs to be configured.
bool _shouldConfigureTearDownForTestFile() {
  return LeakTesting.enabled;
}

/// Tear down that should happen after all user defined tear down.
Future<void> _tearDownForTestFile() async {
  await maybeTearDownLeakTrackingForAll();
}

/// A reporter that prints each test on its own line.
///
/// This is currently used in place of `CompactReporter` by `lib/test.dart`,
/// which can't transitively import `dart:io` but still needs access to a runner
/// so that test files can be run directly. This means that until issue 6943 is
/// fixed, this must not import `dart:io`.
class _Reporter {
  _Reporter({bool color = true, bool printPath = true})
    : _printPath = printPath,
      _green = color ? '\u001b[32m' : '',
      _red = color ? '\u001b[31m' : '',
      _yellow = color ? '\u001b[33m' : '',
      _bold = color ? '\u001b[1m' : '',
      _noColor = color ? '\u001b[0m' : '';

  final List<LiveTest> passed = <LiveTest>[];
  final List<LiveTest> failed = <LiveTest>[];
  final List<Test> skipped = <Test>[];

  /// The terminal escape for green text, or the empty string if this is Windows
  /// or not outputting to a terminal.
  final String _green;

  /// The terminal escape for red text, or the empty string if this is Windows
  /// or not outputting to a terminal.
  final String _red;

  /// The terminal escape for yellow text, or the empty string if this is
  /// Windows or not outputting to a terminal.
  final String _yellow;

  /// The terminal escape for bold text, or the empty string if this is
  /// Windows or not outputting to a terminal.
  final String _bold;

  /// The terminal escape for removing test coloring, or the empty string if
  /// this is Windows or not outputting to a terminal.
  final String _noColor;

  /// Whether the path to each test's suite should be printed.
  final bool _printPath;

  /// A stopwatch that tracks the duration of the full run.
  final Stopwatch _stopwatch = Stopwatch(); // flutter_ignore: stopwatch (see analyze.dart)
  // Ignore context: Used for logging of actual test runs, outside of FakeAsync.

  /// The size of `_engine.passed` last time a progress notification was
  /// printed.
  int? _lastProgressPassed;

  /// The size of `_engine.skipped` last time a progress notification was
  /// printed.
  int? _lastProgressSkipped;

  /// The size of `_engine.failed` last time a progress notification was
  /// printed.
  int? _lastProgressFailed;

  /// The message printed for the last progress notification.
  String? _lastProgressMessage;

  /// The suffix added to the last progress notification.
  String? _lastProgressSuffix;

  /// The set of all subscriptions to various streams.
  final Set<StreamSubscription<void>> _subscriptions = <StreamSubscription<void>>{};

  /// A callback called when the engine begins running [liveTest].
  void _onTestStarted(LiveTest liveTest) {
    if (!_stopwatch.isRunning) {
      _stopwatch.start();
    }
    _progressLine(_description(liveTest));
    _subscriptions.add(
      liveTest.onStateChange.listen((State state) => _onStateChange(liveTest, state)),
    );
    _subscriptions.add(
      liveTest.onError.listen(
        (AsyncError error) => _onError(liveTest, error.error, error.stackTrace),
      ),
    );
    _subscriptions.add(
      liveTest.onMessage.listen((Message message) {
        _progressLine(_description(liveTest));
        String text = message.text;
        if (message.type == MessageType.skip) {
          text = '  $_yellow$text$_noColor';
        }
        log(text);
      }),
    );
  }

  /// A callback called when [liveTest]'s state becomes [state].
  void _onStateChange(LiveTest liveTest, State state) {
    if (state.status != Status.complete) {
      return;
    }
  }

  /// A callback called when [liveTest] throws [error].
  void _onError(LiveTest liveTest, Object error, StackTrace stackTrace) {
    if (liveTest.state.status != Status.complete) {
      return;
    }
    _progressLine(_description(liveTest), suffix: ' $_bold$_red[E]$_noColor');
    log(_indent(error.toString()));
    log(_indent('$stackTrace'));
  }

  /// A callback called when the engine is finished running tests.
  void _onDone() {
    final bool success = failed.isEmpty;
    if (!success) {
      _progressLine('Some tests failed.', color: _red);
    } else if (passed.isEmpty) {
      _progressLine('All tests skipped.');
    } else {
      _progressLine('All tests passed!');
    }
  }

  /// Prints a line representing the current state of the tests.
  ///
  /// [message] goes after the progress report. If [color] is passed, it's used
  /// as the color for [message]. If [suffix] is passed, it's added to the end
  /// of [message].
  void _progressLine(String message, {String? color, String? suffix}) {
    // Print nothing if nothing has changed since the last progress line.
    if (passed.length == _lastProgressPassed &&
        skipped.length == _lastProgressSkipped &&
        failed.length == _lastProgressFailed &&
        message == _lastProgressMessage &&
        // Don't re-print just because a suffix was removed.
        (suffix == null || suffix == _lastProgressSuffix)) {
      return;
    }
    _lastProgressPassed = passed.length;
    _lastProgressSkipped = skipped.length;
    _lastProgressFailed = failed.length;
    _lastProgressMessage = message;
    _lastProgressSuffix = suffix;

    if (suffix != null) {
      message += suffix;
    }
    color ??= '';
    final Duration duration = _stopwatch.elapsed;
    final StringBuffer buffer = StringBuffer();

    // \r moves back to the beginning of the current line.
    buffer.write('${_timeString(duration)} ');
    buffer.write(_green);
    buffer.write('+');
    buffer.write(passed.length);
    buffer.write(_noColor);

    if (skipped.isNotEmpty) {
      buffer.write(_yellow);
      buffer.write(' ~');
      buffer.write(skipped.length);
      buffer.write(_noColor);
    }

    if (failed.isNotEmpty) {
      buffer.write(_red);
      buffer.write(' -');
      buffer.write(failed.length);
      buffer.write(_noColor);
    }

    buffer.write(': ');
    buffer.write(color);
    buffer.write(message);
    buffer.write(_noColor);

    log(buffer.toString());
  }

  /// Returns a representation of [duration] as `MM:SS`.
  String _timeString(Duration duration) {
    final String minutes = duration.inMinutes.toString().padLeft(2, '0');
    final String seconds = (duration.inSeconds % 60).toString().padLeft(2, '0');
    return '$minutes:$seconds';
  }

  /// Returns a description of [liveTest].
  ///
  /// This differs from the test's own description in that it may also include
  /// the suite's name.
  String _description(LiveTest liveTest) {
    String name = liveTest.test.name;
    if (_printPath && liveTest.suite.path != null) {
      name = '${liveTest.suite.path}: $name';
    }
    return name;
  }

  /// Print the message to the console.
  void log(String message) {
    // We centralize all the prints in this file through this one method so that
    // in principle we can reroute the output easily should we need to.
    print(message); // ignore: avoid_print
  }
}

String _indent(String string, {int? size, String? first}) {
  size ??= first == null ? 2 : first.length;
  return _prefixLines(string, ' ' * size, first: first);
}

String _prefixLines(String text, String prefix, {String? first, String? last, String? single}) {
  first ??= prefix;
  last ??= prefix;
  single ??= first;
  final List<String> lines = text.split('\n');
  if (lines.length == 1) {
    return '$single$text';
  }
  final StringBuffer buffer = StringBuffer('$first${lines.first}\n');
  // Write out all but the first and last lines with [prefix].
  for (final String line in lines.skip(1).take(lines.length - 2)) {
    buffer.writeln('$prefix$line');
  }
  buffer.write('$last${lines.last}');
  return buffer.toString();
}
