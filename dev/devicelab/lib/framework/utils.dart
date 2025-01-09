// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;

import 'package:path/path.dart' as path;
import 'package:process/process.dart';
import 'package:stack_trace/stack_trace.dart';

import 'devices.dart';
import 'host_agent.dart';
import 'task_result.dart';

/// Virtual current working directory, which affect functions, such as [exec].
String cwd = Directory.current.path;

/// The local engine to use for [flutter] and [evalFlutter], if any.
///
/// This is set as an environment variable when running the task, see runTask in runner.dart.
String? get localEngineFromEnv {
  const bool isDefined = bool.hasEnvironment('localEngine');
  return isDefined ? const String.fromEnvironment('localEngine') : null;
}

/// The local engine host to use for [flutter] and [evalFlutter], if any.
///
/// This is set as an environment variable when running the task, see runTask in runner.dart.
String? get localEngineHostFromEnv {
  const bool isDefined = bool.hasEnvironment('localEngineHost');
  return isDefined ? const String.fromEnvironment('localEngineHost') : null;
}

/// The local engine source path to use if a local engine is used for [flutter]
/// and [evalFlutter].
///
/// This is set as an environment variable when running the task, see runTask in runner.dart.
String? get localEngineSrcPathFromEnv {
  const bool isDefined = bool.hasEnvironment('localEngineSrcPath');
  return isDefined ? const String.fromEnvironment('localEngineSrcPath') : null;
}

/// The local Web SDK to use for [flutter] and [evalFlutter], if any.
///
/// This is set as an environment variable when running the task, see runTask in runner.dart.
String? get localWebSdkFromEnv {
  const bool isDefined = bool.hasEnvironment('localWebSdk');
  return isDefined ? const String.fromEnvironment('localWebSdk') : null;
}

List<ProcessInfo> _runningProcesses = <ProcessInfo>[];
ProcessManager _processManager = const LocalProcessManager();

class ProcessInfo {
  ProcessInfo(this.command, this.process);

  final DateTime startTime = DateTime.now();
  final String command;
  final Process process;

  @override
  String toString() {
    return '''
  command: $command
  started: $startTime
  pid    : ${process.pid}
'''.trim();
  }
}

/// Result of a health check for a specific parameter.
class HealthCheckResult {
  HealthCheckResult.success([this.details]) : succeeded = true;
  HealthCheckResult.failure(this.details) : succeeded = false;
  HealthCheckResult.error(dynamic error, dynamic stackTrace)
    : succeeded = false,
      details = 'ERROR: $error${stackTrace != null ? '\n$stackTrace' : ''}';

  final bool succeeded;
  final String? details;

  @override
  String toString() {
    final StringBuffer buf = StringBuffer(succeeded ? 'succeeded' : 'failed');
    if (details != null && details!.trim().isNotEmpty) {
      buf.writeln();
      // Indent details by 4 spaces
      for (final String line in details!.trim().split('\n')) {
        buf.writeln('    $line');
      }
    }
    return '$buf';
  }
}

class BuildFailedError extends Error {
  BuildFailedError(this.message);

  final String message;

  @override
  String toString() => message;
}

void fail(String message) {
  throw BuildFailedError(message);
}

// Remove the given file or directory.
void rm(FileSystemEntity entity, {bool recursive = false}) {
  if (entity.existsSync()) {
    // This should not be necessary, but it turns out that
    // on Windows it's common for deletions to fail due to
    // bogus (we think) "access denied" errors.
    try {
      entity.deleteSync(recursive: recursive);
    } on FileSystemException catch (error) {
      print('Failed to delete ${entity.path}: $error');
    }
  }
}

/// Remove recursively.
void rmTree(FileSystemEntity entity) {
  rm(entity, recursive: true);
}

List<FileSystemEntity> ls(Directory directory) => directory.listSync();

Directory dir(String path) => Directory(path);

File file(String path) => File(path);

void copy(File sourceFile, Directory targetDirectory, {String? name}) {
  final File target = file(path.join(targetDirectory.path, name ?? path.basename(sourceFile.path)));
  target.writeAsBytesSync(sourceFile.readAsBytesSync());
}

void recursiveCopy(Directory source, Directory target) {
  if (!target.existsSync()) {
    target.createSync();
  }

  for (final FileSystemEntity entity in source.listSync(followLinks: false)) {
    final String name = path.basename(entity.path);
    if (entity is Directory && !entity.path.contains('.dart_tool')) {
      recursiveCopy(entity, Directory(path.join(target.path, name)));
    } else if (entity is File) {
      final File dest = File(path.join(target.path, name));
      dest.writeAsBytesSync(entity.readAsBytesSync());
      // Preserve executable bit
      final String modes = entity.statSync().modeString();
      if (modes.contains('x')) {
        makeExecutable(dest);
      }
    }
  }
}

FileSystemEntity move(FileSystemEntity whatToMove, {required Directory to, String? name}) {
  return whatToMove.renameSync(path.join(to.path, name ?? path.basename(whatToMove.path)));
}

/// Equivalent of `chmod a+x file`
void makeExecutable(File file) {
  // Windows files do not have an executable bit
  if (Platform.isWindows) {
    return;
  }
  final ProcessResult result = _processManager.runSync(<String>['chmod', 'a+x', file.path]);

  if (result.exitCode != 0) {
    throw FileSystemException(
      'Error making ${file.path} executable.\n'
      '${result.stderr}',
      file.path,
    );
  }
}

/// Equivalent of `mkdir directory`.
void mkdir(Directory directory) {
  directory.createSync();
}

/// Equivalent of `mkdir -p directory`.
void mkdirs(Directory directory) {
  directory.createSync(recursive: true);
}

bool exists(FileSystemEntity entity) => entity.existsSync();

void section(String title) {
  String output;
  if (Platform.isWindows) {
    // Windows doesn't cope well with characters produced for *nix systems, so
    // just output the title with no decoration.
    output = title;
  } else {
    title = '╡ ••• $title ••• ╞';
    final String line = '═' * math.max((80 - title.length) ~/ 2, 2);
    output = '$line$title$line';
    if (output.length == 79) {
      output += '═';
    }
  }
  print('\n\n$output\n');
}

Future<String> getDartVersion() async {
  // The Dart VM returns the version text to stderr.
  final ProcessResult result = _processManager.runSync(<String>[dartBin, '--version']);
  String version = (result.stderr as String).trim();

  // Convert:
  //   Dart VM version: 1.17.0-dev.2.0 (Tue May  3 12:14:52 2016) on "macos_x64"
  // to:
  //   1.17.0-dev.2.0
  if (version.contains('(')) {
    version = version.substring(0, version.indexOf('(')).trim();
  }
  if (version.contains(':')) {
    version = version.substring(version.indexOf(':') + 1).trim();
  }

  return version.replaceAll('"', "'");
}

Future<String?> getCurrentFlutterRepoCommit() {
  if (!dir('${flutterDirectory.path}/.git').existsSync()) {
    return Future<String?>.value();
  }

  return inDirectory<String>(flutterDirectory, () {
    return eval('git', <String>['rev-parse', 'HEAD']);
  });
}

Future<DateTime> getFlutterRepoCommitTimestamp(String commit) {
  // git show -s --format=%at 4b546df7f0b3858aaaa56c4079e5be1ba91fbb65
  return inDirectory<DateTime>(flutterDirectory, () async {
    final String unixTimestamp = await eval('git', <String>['show', '-s', '--format=%at', commit]);
    final int secondsSinceEpoch = int.parse(unixTimestamp);
    return DateTime.fromMillisecondsSinceEpoch(secondsSinceEpoch * 1000);
  });
}

/// Starts a subprocess.
///
/// The first argument is the full path to the executable to run.
///
/// The second argument is the list of arguments to provide on the command line.
/// This argument can be null, indicating no arguments (same as the empty list).
///
/// The `environment` argument can be provided to configure environment variables
/// that will be made available to the subprocess. The `BOT` environment variable
/// is always set and overrides any value provided in the `environment` argument.
/// The `isBot` argument controls the value of the `BOT` variable. It will either
/// be "true", if `isBot` is true (the default), or "false" if it is false.
///
/// The `BOT` variable is in particular used by the `flutter` tool to determine
/// how verbose to be and whether to enable analytics by default.
///
/// The working directory can be provided using the `workingDirectory` argument.
/// By default it will default to the current working directory (see [cwd]).
///
/// Information regarding the execution of the subprocess is printed to the
/// console.
///
/// The actual process executes asynchronously. A handle to the subprocess is
/// returned in the form of a [Future] that completes to a [Process] object.
Future<Process> startProcess(
  String executable,
  List<String>? arguments, {
  Map<String, String>? environment,
  bool isBot =
      true, // set to false to pretend not to be on a bot (e.g. to test user-facing outputs)
  String? workingDirectory,
}) async {
  final String command = '$executable ${arguments?.join(" ") ?? ""}';
  final String finalWorkingDirectory = workingDirectory ?? cwd;
  final Map<String, String> newEnvironment = Map<String, String>.from(
    environment ?? <String, String>{},
  );
  newEnvironment['BOT'] = isBot ? 'true' : 'false';
  newEnvironment['LANG'] = 'en_US.UTF-8';
  print('Executing "$command" in "$finalWorkingDirectory" with environment $newEnvironment');

  final Process process = await _processManager.start(
    <String>[executable, ...?arguments],
    environment: newEnvironment,
    workingDirectory: finalWorkingDirectory,
  );
  final ProcessInfo processInfo = ProcessInfo(command, process);
  _runningProcesses.add(processInfo);

  unawaited(
    process.exitCode.then<void>((int exitCode) {
      _runningProcesses.remove(processInfo);
    }),
  );

  return process;
}

Future<void> forceQuitRunningProcesses() async {
  if (_runningProcesses.isEmpty) {
    return;
  }

  // Give normally quitting processes a chance to report their exit code.
  await Future<void>.delayed(const Duration(seconds: 1));

  // Whatever's left, kill it.
  for (final ProcessInfo p in _runningProcesses) {
    print('Force-quitting process:\n$p');
    if (!p.process.kill()) {
      print('Failed to force quit process.');
    }
  }
  _runningProcesses.clear();
}

/// Executes a command and returns its exit code.
Future<int> exec(
  String executable,
  List<String> arguments, {
  Map<String, String>? environment,
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  String? workingDirectory,
  StringBuffer? output, // if not null, the stdout will be written here
  StringBuffer? stderr, // if not null, the stderr will be written here
}) async {
  return _execute(
    executable,
    arguments,
    environment: environment,
    canFail: canFail,
    workingDirectory: workingDirectory,
    output: output,
    stderr: stderr,
  );
}

Future<int> _execute(
  String executable,
  List<String> arguments, {
  Map<String, String>? environment,
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  String? workingDirectory,
  StringBuffer? output, // if not null, the stdout will be written here
  StringBuffer? stderr, // if not null, the stderr will be written here
  bool printStdout = true,
  bool printStderr = true,
}) async {
  final Process process = await startProcess(
    executable,
    arguments,
    environment: environment,
    workingDirectory: workingDirectory,
  );
  await forwardStandardStreams(
    process,
    output: output,
    stderr: stderr,
    printStdout: printStdout,
    printStderr: printStderr,
  );
  final int exitCode = await process.exitCode;

  if (exitCode != 0 && !canFail) {
    fail('Executable "$executable" failed with exit code $exitCode.');
  }

  return exitCode;
}

/// Forwards standard out and standard error from [process] to this process'
/// respective outputs. Also writes stdout to [output] and stderr to [stderr]
/// if they are not null.
///
/// Returns a future that completes when both out and error streams a closed.
Future<void> forwardStandardStreams(
  Process process, {
  StringBuffer? output,
  StringBuffer? stderr,
  bool printStdout = true,
  bool printStderr = true,
}) {
  final Completer<void> stdoutDone = Completer<void>();
  final Completer<void> stderrDone = Completer<void>();
  process.stdout
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen(
        (String line) {
          if (printStdout) {
            print('stdout: $line');
          }
          output?.writeln(line);
        },
        onDone: () {
          stdoutDone.complete();
        },
      );
  process.stderr
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen(
        (String line) {
          if (printStderr) {
            print('stderr: $line');
          }
          stderr?.writeln(line);
        },
        onDone: () {
          stderrDone.complete();
        },
      );

  return Future.wait<void>(<Future<void>>[stdoutDone.future, stderrDone.future]);
}

/// Executes a command and returns its standard output as a String.
///
/// For logging purposes, the command's output is also printed out by default.
Future<String> eval(
  String executable,
  List<String> arguments, {
  Map<String, String>? environment,
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  String? workingDirectory,
  StringBuffer? stdout, // if not null, the stdout will be written here
  StringBuffer? stderr, // if not null, the stderr will be written here
  bool printStdout = true,
  bool printStderr = true,
}) async {
  final StringBuffer output = stdout ?? StringBuffer();
  await _execute(
    executable,
    arguments,
    environment: environment,
    canFail: canFail,
    workingDirectory: workingDirectory,
    output: output,
    stderr: stderr,
    printStdout: printStdout,
    printStderr: printStderr,
  );
  return output.toString().trimRight();
}

List<String> _flutterCommandArgs(
  String command,
  List<String> options, {
  bool driveWithDds = false,
}) {
  // Commands support the --device-timeout flag.
  final Set<String> supportedDeviceTimeoutCommands = <String>{
    'attach',
    'devices',
    'drive',
    'install',
    'logs',
    'run',
    'screenshot',
  };
  final String? localEngine = localEngineFromEnv;
  final String? localEngineHost = localEngineHostFromEnv;
  final String? localEngineSrcPath = localEngineSrcPathFromEnv;
  final String? localWebSdk = localWebSdkFromEnv;
  final bool pubOrPackagesCommand = command.startsWith('packages') || command.startsWith('pub');
  return <String>[
    command,
    if (deviceOperatingSystem == DeviceOperatingSystem.ios &&
        supportedDeviceTimeoutCommands.contains(command)) ...<String>['--device-timeout', '5'],

    // DDS should generally be disabled for flutter drive in CI.
    // See https://github.com/flutter/flutter/issues/152684.
    if (command == 'drive' && !driveWithDds) '--no-dds',

    if (command == 'drive' && hostAgent.dumpDirectory != null) ...<String>[
      '--screenshot',
      hostAgent.dumpDirectory!.path,
    ],
    if (localEngine != null) ...<String>['--local-engine', localEngine],
    if (localEngineHost != null) ...<String>['--local-engine-host', localEngineHost],
    if (localEngineSrcPath != null) ...<String>['--local-engine-src-path', localEngineSrcPath],
    if (localWebSdk != null) ...<String>['--local-web-sdk', localWebSdk],
    ...options,
    // Use CI flag when running devicelab tests, except for `packages`/`pub` commands.
    // `packages`/`pub` commands effectively runs the `pub` tool, which does not have
    // the same allowed args.
    if (!pubOrPackagesCommand) '--ci',
    if (!pubOrPackagesCommand && hostAgent.dumpDirectory != null)
      '--debug-logs-dir=${hostAgent.dumpDirectory!.path}',
  ];
}

/// Runs the flutter `command`, and returns the exit code.
/// If `canFail` is `false`, the future completes with an error.
Future<int> flutter(
  String command, {
  List<String> options = const <String>[],
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  bool driveWithDds = false, // `flutter drive` tests should generally have dds disabled.
  // The exception is tests that also exercise DevTools, such as
  // DevToolsMemoryTest in perf_tests.dart.
  Map<String, String>? environment,
  String? workingDirectory,
  StringBuffer? output, // if not null, the stdout will be written here
  StringBuffer? stderr, // if not null, the stderr will be written here
}) async {
  final List<String> args = _flutterCommandArgs(command, options, driveWithDds: driveWithDds);
  final int exitCode = await exec(
    path.join(flutterDirectory.path, 'bin', 'flutter'),
    args,
    canFail: canFail,
    environment: environment,
    workingDirectory: workingDirectory,
    output: output,
    stderr: stderr,
  );

  if (exitCode != 0 && !canFail) {
    await _flutterScreenshot(workingDirectory: workingDirectory);
  }
  return exitCode;
}

/// Starts a Flutter subprocess.
///
/// The first argument is the flutter command to run.
///
/// The second argument is the list of arguments to provide on the command line.
/// This argument can be null, indicating no arguments (same as the empty list).
///
/// The `environment` argument can be provided to configure environment variables
/// that will be made available to the subprocess. The `BOT` environment variable
/// is always set and overrides any value provided in the `environment` argument.
/// The `isBot` argument controls the value of the `BOT` variable. It will either
/// be "true", if `isBot` is true (the default), or "false" if it is false.
///
/// The `isBot` argument controls whether the `BOT` environment variable is set
/// to `true` or `false` and is used by the `flutter` tool to determine how
/// verbose to be and whether to enable analytics by default.
///
/// Information regarding the execution of the subprocess is printed to the
/// console.
///
/// The actual process executes asynchronously. A handle to the subprocess is
/// returned in the form of a [Future] that completes to a [Process] object.
Future<Process> startFlutter(
  String command, {
  List<String> options = const <String>[],
  Map<String, String> environment = const <String, String>{},
  bool isBot =
      true, // set to false to pretend not to be on a bot (e.g. to test user-facing outputs)
  String? workingDirectory,
}) async {
  final List<String> args = _flutterCommandArgs(command, options);
  final Process process = await startProcess(
    path.join(flutterDirectory.path, 'bin', 'flutter'),
    args,
    environment: environment,
    isBot: isBot,
    workingDirectory: workingDirectory,
  );

  unawaited(
    process.exitCode.then<void>((int exitCode) async {
      if (exitCode != 0) {
        await _flutterScreenshot(workingDirectory: workingDirectory);
      }
    }),
  );
  return process;
}

/// Runs a `flutter` command and returns the standard output as a string.
Future<String> evalFlutter(
  String command, {
  List<String> options = const <String>[],
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  Map<String, String>? environment,
  StringBuffer? stderr, // if not null, the stderr will be written here.
  String? workingDirectory,
}) {
  final List<String> args = _flutterCommandArgs(command, options);
  return eval(
    path.join(flutterDirectory.path, 'bin', 'flutter'),
    args,
    canFail: canFail,
    environment: environment,
    stderr: stderr,
    workingDirectory: workingDirectory,
  );
}

Future<ProcessResult> executeFlutter(
  String command, {
  List<String> options = const <String>[],
  bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
}) async {
  final List<String> args = _flutterCommandArgs(command, options);
  final ProcessResult processResult = await _processManager.run(<String>[
    path.join(flutterDirectory.path, 'bin', 'flutter'),
    ...args,
  ], workingDirectory: cwd);

  if (processResult.exitCode != 0 && !canFail) {
    await _flutterScreenshot();
  }
  return processResult;
}

Future<void> _flutterScreenshot({String? workingDirectory}) async {
  try {
    final Directory? dumpDirectory = hostAgent.dumpDirectory;
    if (dumpDirectory == null) {
      return;
    }
    // On command failure try uploading screenshot of failing command.
    final String screenshotPath = path.join(
      dumpDirectory.path,
      'device-screenshot-${DateTime.now().toLocal().toIso8601String()}.png',
    );

    final String deviceId = (await devices.workingDevice).deviceId;
    print('Taking screenshot of working device $deviceId at $screenshotPath');
    final List<String> args = _flutterCommandArgs('screenshot', <String>[
      '--out',
      screenshotPath,
      '-d',
      deviceId,
    ]);
    final ProcessResult screenshot = await _processManager.run(<String>[
      path.join(flutterDirectory.path, 'bin', 'flutter'),
      ...args,
    ], workingDirectory: workingDirectory ?? cwd);

    if (screenshot.exitCode != 0) {
      print('Failed to take screenshot. Continuing.');
    }
  } catch (exception) {
    print('Failed to take screenshot. Continuing.\n$exception');
  }
}

String get dartBin => path.join(flutterDirectory.path, 'bin', 'cache', 'dart-sdk', 'bin', 'dart');

String get pubBin => path.join(flutterDirectory.path, 'bin', 'cache', 'dart-sdk', 'bin', 'pub');

Future<int> dart(List<String> args) => exec(dartBin, args);

/// Returns a future that completes with a path suitable for JAVA_HOME
/// or with null, if Java cannot be found.
Future<String?> findJavaHome() async {
  if (_javaHome == null) {
    final Iterable<String> hits = grep(
      'Java binary at: ',
      from: await evalFlutter('doctor', options: <String>['-v']),
    );
    if (hits.isEmpty) {
      return null;
    }
    final String javaBinary = hits.first.split(': ').last;
    // javaBinary == /some/path/to/java/home/bin/java
    _javaHome = path.dirname(path.dirname(javaBinary));
  }
  return _javaHome;
}

String? _javaHome;

Future<T> inDirectory<T>(dynamic directory, Future<T> Function() action) async {
  final String previousCwd = cwd;
  try {
    cd(directory);
    return await action();
  } finally {
    cd(previousCwd);
  }
}

void cd(dynamic directory) {
  Directory d;
  if (directory is String) {
    cwd = directory;
    d = dir(directory);
  } else if (directory is Directory) {
    cwd = directory.path;
    d = directory;
  } else {
    throw FileSystemException(
      'Unsupported directory type ${directory.runtimeType}',
      directory.toString(),
    );
  }

  if (!d.existsSync()) {
    throw FileSystemException('Cannot cd into directory that does not exist', d.toString());
  }
}

Directory get flutterDirectory => Directory.current.parent.parent;

Directory get openpayDirectory => Directory(requireEnvVar('OPENPAY_CHECKOUT_PATH'));

String requireEnvVar(String name) {
  final String? value = Platform.environment[name];

  if (value == null) {
    fail('$name environment variable is missing. Quitting.');
  }

  return value!;
}

T requireConfigProperty<T>(Map<String, dynamic> map, String propertyName) {
  if (!map.containsKey(propertyName)) {
    fail('Configuration property not found: $propertyName');
  }
  final T result = map[propertyName] as T;
  return result;
}

String jsonEncode(dynamic data) {
  final String jsonValue = const JsonEncoder.withIndent('  ').convert(data);
  return '$jsonValue\n';
}

/// Splits [from] into lines and selects those that contain [pattern].
Iterable<String> grep(Pattern pattern, {required String from}) {
  return from.split('\n').where((String line) {
    return line.contains(pattern);
  });
}

/// Captures asynchronous stack traces thrown by [callback].
///
/// This is a convenience wrapper around [Chain] optimized for use with
/// `async`/`await`.
///
/// Example:
///
///     try {
///       await captureAsyncStacks(() { /* async things */ });
///     } catch (error, chain) {
///
///     }
Future<void> runAndCaptureAsyncStacks(Future<void> Function() callback) {
  final Completer<void> completer = Completer<void>();
  Chain.capture(() async {
    await callback();
    completer.complete();
  }, onError: completer.completeError);
  return completer.future;
}

bool canRun(String path) => _processManager.canRun(path);

final RegExp _obsRegExp = RegExp('A Dart VM Service .* is available at: ');
final RegExp _obsPortRegExp = RegExp(r'(\S+:(\d+)/\S*)$');
final RegExp _obsUriRegExp = RegExp(r'((http|//)[a-zA-Z0-9:/=_\-\.\[\]]+)');

/// Tries to extract a port from the string.
///
/// The `prefix`, if specified, is a regular expression pattern and must not contain groups.
/// `prefix` defaults to the RegExp: `A Dart VM Service .* is available at: `.
int? parseServicePort(String line, {Pattern? prefix}) {
  prefix ??= _obsRegExp;
  final Iterable<Match> matchesIter = prefix.allMatches(line);
  if (matchesIter.isEmpty) {
    return null;
  }
  final Match prefixMatch = matchesIter.first;
  final List<Match> matches = _obsPortRegExp.allMatches(line, prefixMatch.end).toList();
  return matches.isEmpty ? null : int.parse(matches[0].group(2)!);
}

/// Tries to extract a URL from the string.
///
/// The `prefix`, if specified, is a regular expression pattern and must not contain groups.
/// `prefix` defaults to the RegExp: `A Dart VM Service .* is available at: `.
Uri? parseServiceUri(String line, {Pattern? prefix}) {
  prefix ??= _obsRegExp;
  final Iterable<Match> matchesIter = prefix.allMatches(line);
  if (matchesIter.isEmpty) {
    return null;
  }
  final Match prefixMatch = matchesIter.first;
  final List<Match> matches = _obsUriRegExp.allMatches(line, prefixMatch.end).toList();
  return matches.isEmpty ? null : Uri.parse(matches[0].group(0)!);
}

/// Checks that the file exists, otherwise throws a [FileSystemException].
void checkFileExists(String file) {
  if (!exists(File(file))) {
    throw FileSystemException('Expected file to exist.', file);
  }
}

/// Checks that the file does not exists, otherwise throws a [FileSystemException].
void checkFileNotExists(String file) {
  if (exists(File(file))) {
    throw FileSystemException('Expected file to not exist.', file);
  }
}

/// Checks that the directory exists, otherwise throws a [FileSystemException].
void checkDirectoryExists(String directory) {
  if (!exists(Directory(directory))) {
    throw FileSystemException('Expected directory to exist.', directory);
  }
}

/// Checks that the directory does not exist, otherwise throws a [FileSystemException].
void checkDirectoryNotExists(String directory) {
  if (exists(Directory(directory))) {
    throw FileSystemException('Expected directory to not exist.', directory);
  }
}

/// Checks that the symlink exists, otherwise throws a [FileSystemException].
void checkSymlinkExists(String file) {
  if (!exists(Link(file))) {
    throw FileSystemException('Expected symlink to exist.', file);
  }
}

/// Check that `collection` contains all entries in `values`.
void checkCollectionContains<T>(Iterable<T> values, Iterable<T> collection) {
  for (final T value in values) {
    if (!collection.contains(value)) {
      throw TaskResult.failure('Expected to find `$value` in `$collection`.');
    }
  }
}

/// Check that `collection` does not contain any entries in `values`
void checkCollectionDoesNotContain<T>(Iterable<T> values, Iterable<T> collection) {
  for (final T value in values) {
    if (collection.contains(value)) {
      throw TaskResult.failure('Did not expect to find `$value` in `$collection`.');
    }
  }
}

/// Checks that the contents of a [File] at `filePath` contains the specified
/// [Pattern]s, otherwise throws a [TaskResult].
void checkFileContains(List<Pattern> patterns, String filePath) {
  final String fileContent = File(filePath).readAsStringSync();
  for (final Pattern pattern in patterns) {
    if (!fileContent.contains(pattern)) {
      throw TaskResult.failure(
        'Expected to find `$pattern` in `$filePath` '
        'instead it found:\n$fileContent',
      );
    }
  }
}

/// Clones a git repository.
///
/// Removes the directory [path], then clones the git repository
/// specified by [repo] to the directory [path].
Future<int> gitClone({required String path, required String repo}) async {
  rmTree(Directory(path));

  await Directory(path).create(recursive: true);

  return inDirectory<int>(path, () => exec('git', <String>['clone', repo]));
}

/// Call [fn] retrying so long as [retryIf] return `true` for the exception
/// thrown and [maxAttempts] has not been reached.
///
/// If no [retryIf] function is given this will retry any for any [Exception]
/// thrown. To retry on an [Error], the error must be caught and _rethrown_
/// as an [Exception].
///
/// Waits a constant duration of [delayDuration] between every retry attempt.
Future<T> retry<T>(
  FutureOr<T> Function() fn, {
  FutureOr<bool> Function(Exception)? retryIf,
  int maxAttempts = 5,
  Duration delayDuration = const Duration(seconds: 3),
}) async {
  int attempt = 0;
  while (true) {
    attempt++; // first invocation is the first attempt
    try {
      return await fn();
    } on Exception catch (e) {
      if (attempt >= maxAttempts || (retryIf != null && !(await retryIf(e)))) {
        rethrow;
      }
    }

    // Sleep for a delay
    await Future<void>.delayed(delayDuration);
  }
}

Future<void> createFfiPackage(String name, Directory parent) async {
  await inDirectory(parent, () async {
    await flutter(
      'create',
      options: <String>[
        '--no-pub',
        '--org',
        'io.flutter.devicelab',
        '--template=package_ffi',
        name,
      ],
    );
    await _pinDependencies(File(path.join(parent.path, name, 'pubspec.yaml')));
    await _pinDependencies(File(path.join(parent.path, name, 'example', 'pubspec.yaml')));
  });
}

Future<void> _pinDependencies(File pubspecFile) async {
  final String oldPubspec = await pubspecFile.readAsString();
  final String newPubspec = oldPubspec.replaceAll(': ^', ': ');
  await pubspecFile.writeAsString(newPubspec);
}
