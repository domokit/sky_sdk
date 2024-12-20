// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:core' hide print;
import 'dart:io' as system show exit;
import 'dart:io' hide exit;
import 'dart:math' as math;

import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/source/line_info.dart';
import 'package:collection/collection.dart';
import 'package:file/file.dart' as fs;
import 'package:file/local.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

import 'run_command.dart';
import 'tool_subsharding.dart';

typedef ShardRunner = Future<void> Function();

/// A function used to validate the output of a test.
///
/// If the output matches expectations, the function shall return null.
///
/// If the output does not match expectations, the function shall return an
/// appropriate error message.
typedef OutputChecker = String? Function(CommandResult);

const Duration _quietTimeout = Duration(
  minutes: 10,
); // how long the output should be hidden between calls to printProgress before just being verbose

// If running from LUCI set to False.
final bool isLuci = Platform.environment['LUCI_CI'] == 'True';
final bool hasColor = stdout.supportsAnsiEscapes && !isLuci;
final bool _isRandomizationOff =
    bool.tryParse(Platform.environment['TEST_RANDOMIZATION_OFF'] ?? '') ?? false;

final String bold = hasColor ? '\x1B[1m' : ''; // shard titles
final String red = hasColor ? '\x1B[31m' : ''; // errors
final String green = hasColor ? '\x1B[32m' : ''; // section titles, commands
final String yellow =
    hasColor
        ? '\x1B[33m'
        : ''; // indications that a test was skipped (usually renders orange or brown)
final String cyan = hasColor ? '\x1B[36m' : ''; // paths
final String reverse = hasColor ? '\x1B[7m' : ''; // clocks
final String gray =
    hasColor ? '\x1B[30m' : ''; // subtle decorative items (usually renders as dark gray)
final String white = hasColor ? '\x1B[37m' : ''; // last log line (usually renders as light gray)
final String reset = hasColor ? '\x1B[0m' : '';

final String exe = Platform.isWindows ? '.exe' : '';
final String bat = Platform.isWindows ? '.bat' : '';
final String flutterRoot = path.dirname(path.dirname(path.dirname(path.fromUri(Platform.script))));
final String flutter = path.join(flutterRoot, 'bin', 'flutter$bat');
final String dart = path.join(flutterRoot, 'bin', 'cache', 'dart-sdk', 'bin', 'dart$exe');
final String pubCache = path.join(flutterRoot, '.pub-cache');
final String engineVersionFile = path.join(flutterRoot, 'bin', 'internal', 'engine.version');
final String luciBotId = Platform.environment['SWARMING_BOT_ID'] ?? '';
final bool runningInDartHHHBot =
    luciBotId.startsWith('luci-dart-') || luciBotId.startsWith('dart-tests-');

const String kShardKey = 'SHARD';
const String kSubshardKey = 'SUBSHARD';
const String kTestHarnessShardName = 'test_harness_tests';
const String CIRRUS_TASK_NAME = 'CIRRUS_TASK_NAME';

/// Environment variables to override the local engine when running `pub test`,
/// if such flags are provided to `test.dart`.
final Map<String, String> localEngineEnv = <String, String>{};

/// The arguments to pass to `flutter test` (typically the local engine
/// configuration) -- prefilled with  the arguments passed to test.dart.
final List<String> flutterTestArgs = <String>[];

/// Whether execution should be simulated for debugging purposes.
///
/// When `true`, calls to [runCommand] print to [io.stdout] instead of running
/// the process. This is useful for determining what an invocation of `test.dart`
/// _might_ due if not invoked with `--dry-run`, or otherwise determine what the
/// different test shards and sub-shards are configured as.
bool get dryRun => _dryRun ?? false;

/// Switches [dryRun] to `true`.
///
/// Expected to be called at most once during execution of a process.
void enableDryRun() {
  if (_dryRun != null) {
    throw StateError('Should only be called at most once');
  }
  _dryRun = true;
}

bool? _dryRun;

const int kESC = 0x1B;
const int kOpenSquareBracket = 0x5B;
const int kCSIParameterRangeStart = 0x30;
const int kCSIParameterRangeEnd = 0x3F;
const int kCSIIntermediateRangeStart = 0x20;
const int kCSIIntermediateRangeEnd = 0x2F;
const int kCSIFinalRangeStart = 0x40;
const int kCSIFinalRangeEnd = 0x7E;

String get redLine {
  if (hasColor) {
    return '$red${'━' * stdout.terminalColumns}$reset';
  }
  return '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
}

String get clock {
  final DateTime now = DateTime.now();
  return '$reverse▌'
      '${now.hour.toString().padLeft(2, "0")}:'
      '${now.minute.toString().padLeft(2, "0")}:'
      '${now.second.toString().padLeft(2, "0")}'
      '▐$reset';
}

String prettyPrintDuration(Duration duration) {
  String result = '';
  final int minutes = duration.inMinutes;
  if (minutes > 0) {
    result += '${minutes}min ';
  }
  final int seconds = duration.inSeconds - minutes * 60;
  final int milliseconds = duration.inMilliseconds - (seconds * 1000 + minutes * 60 * 1000);
  result += '$seconds.${milliseconds.toString().padLeft(3, "0")}s';
  return result;
}

typedef PrintCallback = void Function(Object? line);
typedef VoidCallback = void Function();

// Allow print() to be overridden, for tests.
//
// Files that import this library should not import `print` from dart:core
// and should not use dart:io's `stdout` or `stderr`.
//
// By default this hides log lines between `printProgress` calls unless a
// timeout expires or anything calls `foundError`.
//
// Also used to implement `--verbose` in test.dart.
PrintCallback print = _printQuietly;

// Called by foundError and used to implement `--abort-on-error` in test.dart.
VoidCallback? onError;

bool get hasError => _hasError;
bool _hasError = false;

List<List<String>> _errorMessages = <List<String>>[];

final List<String> _pendingLogs = <String>[];
Timer? _hideTimer; // When this is null, the output is verbose.

void foundError(List<String> messages) {
  if (dryRun) {
    printProgress(messages.join('\n'));
    return;
  }
  assert(messages.isNotEmpty);
  // Make the error message easy to notice in the logs by
  // wrapping it in a red box.
  final int width = math.max(15, (hasColor ? stdout.terminalColumns : 80) - 1);
  final String title = 'ERROR #${_errorMessages.length + 1}';
  print('$red╔═╡$bold$title$reset$red╞═${"═" * (width - 4 - title.length)}');
  for (final String message in messages.expand((String line) => line.split('\n'))) {
    print('$red║$reset $message');
  }
  print('$red╚${"═" * width}');
  // Normally, "print" actually prints to the log. To make the errors visible,
  // and to include useful context, print the entire log up to this point, and
  // clear it. Subsequent messages will continue to not be logged until there is
  // another error.
  _pendingLogs.forEach(_printLoudly);
  _pendingLogs.clear();
  _errorMessages.add(messages);
  _hasError = true;
  onError?.call();
}

@visibleForTesting
void resetErrorStatus() {
  _hasError = false;
  _errorMessages.clear();
  _pendingLogs.clear();
  _hideTimer?.cancel();
  _hideTimer = null;
}

Never reportSuccessAndExit(String message) {
  _hideTimer?.cancel();
  _hideTimer = null;
  print('$clock $message$reset');
  system.exit(0);
}

Never reportErrorsAndExit(String message) {
  _hideTimer?.cancel();
  _hideTimer = null;
  print('$clock $message$reset');
  print(redLine);
  print('${red}The error messages reported above are repeated here:$reset');
  final bool printSeparators = _errorMessages.any((List<String> messages) => messages.length > 1);
  if (printSeparators) {
    print('  -- This line intentionally left blank --  ');
  }
  for (int index = 0; index < _errorMessages.length * 2 - 1; index += 1) {
    if (index.isEven) {
      _errorMessages[index ~/ 2].forEach(print);
    } else if (printSeparators) {
      print('  -- This line intentionally left blank --  ');
    }
  }
  print(redLine);
  print('You may find the errors by searching for "╡ERROR #" in the logs.');
  system.exit(1);
}

void printProgress(String message) {
  _pendingLogs.clear();
  _hideTimer?.cancel();
  _hideTimer = null;
  print('$clock $message$reset');
  if (hasColor) {
    // This sets up a timer to switch to verbose mode when the tests take too long,
    // so that if a test hangs we can see the logs.
    // (This is only supported with a color terminal. When the terminal doesn't
    // support colors, the scripts just print everything verbosely, that way in
    // CI there's nothing hidden.)
    _hideTimer = Timer(_quietTimeout, () {
      _hideTimer = null;
      _pendingLogs.forEach(_printLoudly);
      _pendingLogs.clear();
    });
  }
}

final Pattern _lineBreak = RegExp(r'[\r\n]');

void _printQuietly(Object? message) {
  // The point of this function is to avoid printing its output unless the timer
  // has gone off in which case the function assumes verbose mode is active and
  // prints everything. To show that progress is still happening though, rather
  // than showing nothing at all, it instead shows the last line of output and
  // keeps overwriting it. To do this in color mode, carefully measures the line
  // of text ignoring color codes, which is what the parser below does.
  if (_hideTimer != null) {
    _pendingLogs.add(message.toString());
    String line = '$message'.trimRight();
    final int start = line.lastIndexOf(_lineBreak) + 1;
    int index = start;
    int length = 0;
    while (index < line.length && length < stdout.terminalColumns) {
      if (line.codeUnitAt(index) == kESC) {
        // 0x1B
        index += 1;
        if (index < line.length && line.codeUnitAt(index) == kOpenSquareBracket) {
          // 0x5B, [
          // That was the start of a CSI sequence.
          index += 1;
          while (index < line.length &&
              line.codeUnitAt(index) >= kCSIParameterRangeStart &&
              line.codeUnitAt(index) <= kCSIParameterRangeEnd) {
            // 0x30..0x3F
            index += 1; // ...parameter bytes...
          }
          while (index < line.length &&
              line.codeUnitAt(index) >= kCSIIntermediateRangeStart &&
              line.codeUnitAt(index) <= kCSIIntermediateRangeEnd) {
            // 0x20..0x2F
            index += 1; // ...intermediate bytes...
          }
          if (index < line.length &&
              line.codeUnitAt(index) >= kCSIFinalRangeStart &&
              line.codeUnitAt(index) <= kCSIFinalRangeEnd) {
            // 0x40..0x7E
            index += 1; // ...final byte.
          }
        }
      } else {
        index += 1;
        length += 1;
      }
    }
    line = line.substring(start, index);
    if (line.isNotEmpty) {
      stdout.write('\r\x1B[2K$white$line$reset');
    }
  } else {
    _printLoudly('$message');
  }
}

void _printLoudly(String message) {
  if (hasColor) {
    // Overwrite the last line written by _printQuietly.
    stdout.writeln('\r\x1B[2K$reset${message.trimRight()}');
  } else {
    stdout.writeln(message);
  }
}

// THE FOLLOWING CODE IS A VIOLATION OF OUR STYLE GUIDE
// BECAUSE IT INTRODUCES A VERY FLAKY RACE CONDITION
// https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md#never-check-if-a-port-is-available-before-using-it-never-add-timeouts-and-other-race-conditions
// DO NOT USE THE FOLLOWING FUNCTIONS
// DO NOT WRITE CODE LIKE THE FOLLOWING FUNCTIONS
// https://github.com/flutter/flutter/issues/109474

int _portCounter = 8080;

/// Finds the next available local port.
Future<int> findAvailablePortAndPossiblyCauseFlakyTests() async {
  while (!await _isPortAvailable(_portCounter)) {
    _portCounter += 1;
  }
  return _portCounter++;
}

Future<bool> _isPortAvailable(int port) async {
  try {
    final RawSocket socket = await RawSocket.connect('localhost', port);
    socket.shutdown(SocketDirection.both);
    await socket.close();
    return false;
  } on SocketException {
    return true;
  }
}

String locationInFile(ResolvedUnitResult unit, AstNode node, String workingDirectory) {
  return '${path.relative(path.relative(unit.path, from: workingDirectory))}:${unit.lineInfo.getLocation(node.offset).lineNumber}';
}

/// Whether the given [AstNode] within the `compilationUnit` is under the effect
/// of an inline ignore directive described by `ignoreDirectivePattern`.
///
/// The `compilationUnit` parameter is the parsed dart file containing the given
/// [AstNode]. The `ignoreDirectivePattern` is a [Pattern] that should precisely
/// match the ignore directive of interest (including the slashes, example:
/// `// flutter_ignore: deprecation_syntax`).
///
/// The implementation assumes the `ignoreDirectivePattern` matches no more than
/// one line. It searches for the given `ignoreDirectivePattern` in the
/// `compilationUnit`, that either starts the line above the given `node`, or
/// appears after `node` but on the same line, such that the ignore directive
/// works the same way as dart's "ignore" comment: it can either be added above
/// or after the line that needs to be exemped.
bool hasInlineIgnore(
  AstNode node,
  ParseStringResult compilationUnit,
  Pattern ignoreDirectivePattern,
) {
  final LineInfo lineInfo = compilationUnit.lineInfo;
  // In case the node has multiple lines, match from its start offset.
  final String textAfterNode = compilationUnit.content.substring(
    node.offset,
    // This assumes every line ends with a newline character (including the last
    // line) and the new line character is not included to match the given pattern.
    lineInfo.getOffsetOfLineAfter(node.offset) - 1,
  );
  if (textAfterNode.contains(ignoreDirectivePattern)) {
    return true;
  }
  // The lineNumber getter uses one-based index while everything else uses zero-based index.
  final int lineNumber = lineInfo.getLocation(node.offset).lineNumber - 1;
  if (lineNumber <= 0) {
    return false;
  }
  return compilationUnit.content
      .substring(
        lineInfo.getOffsetOfLine(lineNumber - 1),
        lineInfo.getOffsetOfLine(lineNumber) - 1, // Excludes LF, see the comment above.
      )
      .trimLeft()
      .contains(ignoreDirectivePattern);
}

// The seed used to shuffle tests. If not passed with
// --test-randomize-ordering-seed=<seed> on the command line, it will be set the
// first time it is accessed. Pass zero to turn off shuffling.
String? _shuffleSeed;

set shuffleSeed(String? newSeed) {
  _shuffleSeed = newSeed;
}

String get shuffleSeed {
  if (_shuffleSeed != null) {
    return _shuffleSeed!;
  }
  // Attempt to load from the command-line argument
  final String? seedArg = Platform.environment['--test-randomize-ordering-seed'];
  if (seedArg != null) {
    return seedArg;
  }
  // Fallback to the original time-based seed generation
  final DateTime seedTime = DateTime.now().toUtc().subtract(const Duration(hours: 7));
  _shuffleSeed = '${seedTime.year * 10000 + seedTime.month * 100 + seedTime.day}';
  return _shuffleSeed!;
}

// TODO(sigmund): includeLocalEngineEnv should default to true. Currently we
// only enable it on flutter-web test because some test suites do not work
// properly when overriding the local engine (for example, because some platform
// dependent targets are only built on some engines).
// See https://github.com/flutter/flutter/issues/72368
Future<void> runDartTest(
  String workingDirectory, {
  List<String>? testPaths,
  bool enableFlutterToolAsserts = true,
  bool useBuildRunner = false,
  String? coverage,
  bool forceSingleCore = false,
  Duration? perTestTimeout,
  bool includeLocalEngineEnv = false,
  bool ensurePrecompiledTool = true,
  bool shuffleTests = true,
  bool collectMetrics = false,
  List<String>? tags,
  bool runSkipped = false,
}) async {
  int? cpus;
  final String? cpuVariable = Platform.environment['CPU']; // CPU is set in cirrus.yml
  if (cpuVariable != null) {
    cpus = int.tryParse(cpuVariable, radix: 10);
    if (cpus == null) {
      foundError(<String>[
        '${red}The CPU environment variable, if set, must be set to the integer number of available cores.$reset',
        'Actual value: "$cpuVariable"',
      ]);
      return;
    }
  } else {
    cpus = 2; // Don't default to 1, otherwise we won't catch race conditions.
  }
  // Integration tests that depend on external processes like chrome
  // can get stuck if there are multiple instances running at once.
  if (forceSingleCore) {
    cpus = 1;
  }

  const LocalFileSystem fileSystem = LocalFileSystem();
  final String suffix = DateTime.now().microsecondsSinceEpoch.toString();
  final File metricFile = fileSystem.systemTempDirectory.childFile('metrics_$suffix.json');
  final List<String> args = <String>[
    'run',
    'test',
    '--reporter=expanded',
    '--file-reporter=json:${metricFile.path}',
    if (shuffleTests) '--test-randomize-ordering-seed=$shuffleSeed',
    '-j$cpus',
    if (!hasColor) '--no-color',
    if (coverage != null) '--coverage=$coverage',
    if (perTestTimeout != null) '--timeout=${perTestTimeout.inMilliseconds}ms',
    if (runSkipped) '--run-skipped',
    if (tags != null) ...tags.map((String t) => '--tags=$t'),
    if (testPaths != null)
      for (final String testPath in testPaths) testPath,
  ];
  final Map<String, String> environment = <String, String>{
    'FLUTTER_ROOT': flutterRoot,
    if (includeLocalEngineEnv) ...localEngineEnv,
    if (Directory(pubCache).existsSync()) 'PUB_CACHE': pubCache,
  };
  if (enableFlutterToolAsserts) {
    adjustEnvironmentToEnableFlutterAsserts(environment);
  }
  if (ensurePrecompiledTool) {
    // We rerun the `flutter` tool here just to make sure that it is compiled
    // before tests run, because the tests might time out if they have to rebuild
    // the tool themselves.
    await runCommand(flutter, <String>['--version'], environment: environment);
  }
  await runCommand(
    dart,
    args,
    workingDirectory: workingDirectory,
    environment: environment,
    removeLine: useBuildRunner ? (String line) => line.startsWith('[INFO]') : null,
  );

  if (dryRun) {
    return;
  }

  final TestFileReporterResults test = TestFileReporterResults.fromFile(
    metricFile,
  ); // --file-reporter name
  final File info = fileSystem.file(path.join(flutterRoot, 'error.log'));
  info.writeAsStringSync(json.encode(test.errors));

  if (collectMetrics) {
    try {
      final List<String> testList = <String>[];
      final Map<int, TestSpecs> allTestSpecs = test.allTestSpecs;
      for (final TestSpecs testSpecs in allTestSpecs.values) {
        testList.add(testSpecs.toJson());
      }
      if (testList.isNotEmpty) {
        final String testJson = json.encode(testList);
        final File testResults = fileSystem.file(path.join(flutterRoot, 'test_results.json'));
        testResults.writeAsStringSync(testJson);
      }
    } on fs.FileSystemException catch (e) {
      print('Failed to generate metrics: $e');
    }
  }

  // metriciFile is a transitional file that needs to be deleted once it is parsed.
  // TODO(godofredoc): Ensure metricFile is parsed and aggregated before deleting.
  // https://github.com/flutter/flutter/issues/146003
  metricFile.deleteSync();
}

Future<void> runFlutterTest(
  String workingDirectory, {
  String? script,
  bool expectFailure = false,
  bool printOutput = true,
  OutputChecker? outputChecker,
  List<String> options = const <String>[],
  Map<String, String>? environment,
  List<String> tests = const <String>[],
  bool shuffleTests = true,
  bool fatalWarnings = true,
}) async {
  assert(
    !printOutput || outputChecker == null,
    'Output either can be printed or checked but not both',
  );

  final List<String> tags = <String>[];
  // Recipe-configured reduced test shards will only execute tests with the
  // appropriate tag.
  if (Platform.environment['REDUCED_TEST_SET'] == 'True') {
    tags.addAll(<String>['-t', 'reduced-test-set']);
  }

  const LocalFileSystem fileSystem = LocalFileSystem();
  final String suffix = DateTime.now().microsecondsSinceEpoch.toString();
  final File metricFile = fileSystem.systemTempDirectory.childFile('metrics_$suffix.json');
  final List<String> args = <String>[
    'test',
    '--reporter=expanded',
    '--file-reporter=json:${metricFile.path}',
    if (shuffleTests && !_isRandomizationOff) '--test-randomize-ordering-seed=$shuffleSeed',
    if (fatalWarnings) '--fatal-warnings',
    ...options,
    ...tags,
    ...flutterTestArgs,
  ];

  if (script != null) {
    final String fullScriptPath = path.join(workingDirectory, script);
    if (!FileSystemEntity.isFileSync(fullScriptPath)) {
      foundError(<String>[
        '${red}Could not find test$reset: $green$fullScriptPath$reset',
        'Working directory: $cyan$workingDirectory$reset',
        'Script: $green$script$reset',
        if (!printOutput) 'This is one of the tests that does not normally print output.',
      ]);
      return;
    }
    args.add(script);
  }

  args.addAll(tests);

  final OutputMode outputMode =
      outputChecker == null && printOutput ? OutputMode.print : OutputMode.capture;

  final CommandResult result = await runCommand(
    flutter,
    args,
    workingDirectory: workingDirectory,
    expectNonZeroExit: expectFailure,
    outputMode: outputMode,
    environment: environment,
  );

  // metriciFile is a transitional file that needs to be deleted once it is parsed.
  // TODO(godofredoc): Ensure metricFile is parsed and aggregated before deleting.
  // https://github.com/flutter/flutter/issues/146003
  if (!dryRun) {
    metricFile.deleteSync();
  }

  if (outputChecker != null) {
    final String? message = outputChecker(result);
    if (message != null) {
      foundError(<String>[message]);
    }
  }
}

/// This will force the next run of the Flutter tool (if it uses the provided
/// environment) to have asserts enabled, by setting an environment variable.
void adjustEnvironmentToEnableFlutterAsserts(Map<String, String> environment) {
  // If an existing env variable exists append to it, but only if
  // it doesn't appear to already include enable-asserts.
  String toolsArgs = Platform.environment['FLUTTER_TOOL_ARGS'] ?? '';
  if (!toolsArgs.contains('--enable-asserts')) {
    toolsArgs += ' --enable-asserts';
  }
  environment['FLUTTER_TOOL_ARGS'] = toolsArgs.trim();
}

Future<void> selectShard(Map<String, ShardRunner> shards) =>
    _runFromList(shards, kShardKey, 'shard', 0);
Future<void> selectSubshard(Map<String, ShardRunner> subshards) =>
    _runFromList(subshards, kSubshardKey, 'subshard', 1);

Future<void> runShardRunnerIndexOfTotalSubshard(List<ShardRunner> tests) async {
  final List<ShardRunner> sublist = selectIndexOfTotalSubshard<ShardRunner>(tests);
  for (final ShardRunner test in sublist) {
    await test();
  }
}

/// Parse (one-)index/total-named subshards from environment variable SUBSHARD
/// and equally distribute [tests] between them.
/// The format of SUBSHARD is "{index}_{total number of shards}".
/// The scheduler can change the number of total shards without needing an additional
/// commit in this repository.
///
/// Examples:
/// 1_3
/// 2_3
/// 3_3
List<T> selectIndexOfTotalSubshard<T>(List<T> tests, {String subshardKey = kSubshardKey}) {
  // Example: "1_3" means the first (one-indexed) shard of three total shards.
  final String? subshardName = Platform.environment[subshardKey];
  if (subshardName == null) {
    print('$kSubshardKey environment variable is missing, skipping sharding');
    return tests;
  }
  printProgress('$bold$subshardKey=$subshardName$reset');

  final RegExp pattern = RegExp(r'^(\d+)_(\d+)$');
  final Match? match = pattern.firstMatch(subshardName);
  if (match == null || match.groupCount != 2) {
    foundError(<String>[
      '${red}Invalid subshard name "$subshardName". Expected format "[int]_[int]" ex. "1_3"',
    ]);
    throw Exception('Invalid subshard name: $subshardName');
  }
  // One-indexed.
  final int index = int.parse(match.group(1)!);
  final int total = int.parse(match.group(2)!);
  if (index > total) {
    foundError(<String>[
      '${red}Invalid subshard name "$subshardName". Index number must be greater or equal to total.',
    ]);
    return <T>[];
  }

  final (int start, int end) = selectTestsForSubShard(
    testCount: tests.length,
    subShardIndex: index,
    subShardCount: total,
  );
  print('Selecting subshard $index of $total (tests ${start + 1}-$end of ${tests.length})');
  return tests.sublist(start, end);
}

/// Finds the interval of tests that a subshard is responsible for testing.
@visibleForTesting
(int start, int end) selectTestsForSubShard({
  required int testCount,
  required int subShardIndex,
  required int subShardCount,
}) {
  // While there exists a closed formula figuring out the range of tests the
  // subshard is responsible for, modeling this as a simulation of distributing
  // items equally into buckets is more intuitive.
  //
  // A bucket represents how many tests a subshard should be allocated.
  final List<int> buckets = List<int>.filled(subShardCount, 0);
  // First, allocate an equal number of items to each bucket.
  for (int i = 0; i < buckets.length; i++) {
    buckets[i] = (testCount / subShardCount).floor();
  }
  // For the N leftover items, put one into each of the first N buckets.
  final int remainingItems = testCount % buckets.length;
  for (int i = 0; i < remainingItems; i++) {
    buckets[i] += 1;
  }

  // Lastly, compute the indices of the items in buckets[index].
  // We derive this from the toal number items in previous buckets and the number
  // of items in this bucket.
  final int numberOfItemsInPreviousBuckets =
      subShardIndex == 0 ? 0 : buckets.sublist(0, subShardIndex - 1).sum;
  final int start = numberOfItemsInPreviousBuckets;
  final int end = start + buckets[subShardIndex - 1];

  return (start, end);
}

Future<void> _runFromList(
  Map<String, ShardRunner> items,
  String key,
  String name,
  int positionInTaskName,
) async {
  try {
    String? item = Platform.environment[key];
    if (item == null && Platform.environment.containsKey(CIRRUS_TASK_NAME)) {
      final List<String> parts = Platform.environment[CIRRUS_TASK_NAME]!.split('-');
      assert(positionInTaskName < parts.length);
      item = parts[positionInTaskName];
    }
    if (item == null) {
      for (final String currentItem in items.keys) {
        printProgress('$bold$key=$currentItem$reset');
        await items[currentItem]!();
      }
    } else {
      printProgress('$bold$key=$item$reset');
      if (!items.containsKey(item)) {
        foundError(<String>[
          '${red}Invalid $name: $item$reset',
          'The available ${name}s are: ${items.keys.join(", ")}',
        ]);
        return;
      }
      await items[item]!();
    }
  } catch (_) {
    if (!dryRun) {
      rethrow;
    }
  }
}

/// Checks the given file's contents to determine if they match the allowed
/// pattern for version strings.
///
/// Returns null if the contents are good. Returns a string if they are bad.
/// The string is an error message.
Future<String?> verifyVersion(File file) async {
  final RegExp pattern = RegExp(r'^(\d+)\.(\d+)\.(\d+)((-\d+\.\d+)?\.pre(\.\d+)?)?$');
  if (!file.existsSync()) {
    return 'The version logic failed to create the Flutter version file.';
  }
  final String version = await file.readAsString();
  if (version == '0.0.0-unknown') {
    return 'The version logic failed to determine the Flutter version.';
  }
  if (!version.contains(pattern)) {
    return 'The version logic generated an invalid version string: "$version".';
  }
  return null;
}
