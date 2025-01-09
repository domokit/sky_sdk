// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert' show LineSplitter, utf8;
import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

import 'common.dart';
import 'environment.dart';
import 'felt_config.dart';

enum RuntimeMode { debug, profile, release }

class FilePath {
  FilePath.fromCwd(String relativePath) : _absolutePath = path.absolute(relativePath);
  FilePath.fromWebUi(String relativePath)
    : _absolutePath = path.join(environment.webUiRootDir.path, relativePath);
  FilePath.fromTestSet(TestSet testSet, String relativePath)
    : _absolutePath = path.join(getTestSetDirectory(testSet).path, relativePath);

  final String _absolutePath;

  String get absolute => _absolutePath;
  String get relativeToCwd => path.relative(_absolutePath);
  String get relativeToWebUi => path.relative(_absolutePath, from: environment.webUiRootDir.path);

  @override
  bool operator ==(Object other) {
    return other is FilePath && other._absolutePath == _absolutePath;
  }

  @override
  int get hashCode => _absolutePath.hashCode;

  @override
  String toString() => _absolutePath;
}

/// Runs [executable] merging its output into the current process' standard out and standard error.
Future<int> runProcess(
  String executable,
  List<String> arguments, {
  String? workingDirectory,
  bool failureIsSuccess = false,
  Map<String, String> environment = const <String, String>{},
}) async {
  final ProcessManager manager = await startProcess(
    executable,
    arguments,
    workingDirectory: workingDirectory,
    failureIsSuccess: failureIsSuccess,
    environment: environment,
  );
  return manager.wait();
}

/// Runs the process and returns its standard output as a string.
///
/// Standard error output is ignored (use [ProcessManager.evalStderr] for that).
///
/// Throws an exception if the process exited with a non-zero exit code.
Future<String> evalProcess(
  String executable,
  List<String> arguments, {
  String? workingDirectory,
  Map<String, String> environment = const <String, String>{},
}) async {
  final ProcessManager manager = await startProcess(
    executable,
    arguments,
    workingDirectory: workingDirectory,
    environment: environment,
    evalOutput: true,
  );
  return manager.evalStdout();
}

/// Starts a process using the [executable], passing it [arguments].
///
/// Returns a process manager that decorates the process with extra
/// functionality. See [ProcessManager] for what it can do.
///
/// If [workingDirectory] is not null makes it the current working directory of
/// the process. Otherwise, the process inherits this processes working
/// directory.
///
/// If [failureIsSuccess] is set to true, the returned [ProcessManager] treats
/// non-zero exit codes as success, and zero exit code as failure.
///
/// If [evalOutput] is set to true, collects and decodes the process' standard
/// streams into in-memory strings.
Future<ProcessManager> startProcess(
  String executable,
  List<String> arguments, {
  String? workingDirectory,
  bool failureIsSuccess = false,
  bool evalOutput = false,
  Map<String, String> environment = const <String, String>{},
}) async {
  final io.Process process = await io.Process.start(
    executable,
    arguments,
    workingDirectory: workingDirectory,
    // Running the process in a system shell for Windows. Otherwise
    // the process is not able to get Dart from path.
    runInShell: io.Platform.isWindows,
    // When [evalOutput] is false, we don't need to intercept the stdout of the
    // sub-process. In this case, it's better to run the sub-process in the
    // `inheritStdio` mode which lets it print directly to the terminal.
    // This allows sub-processes such as `ninja` to use all kinds of terminal
    // features like printing colors, printing progress on the same line, etc.
    mode: evalOutput ? io.ProcessStartMode.normal : io.ProcessStartMode.inheritStdio,
    environment: environment,
  );
  processesToCleanUp.add(process);

  return ProcessManager._(
    executable: executable,
    arguments: arguments,
    workingDirectory: workingDirectory,
    process: process,
    evalOutput: evalOutput,
    failureIsSuccess: failureIsSuccess,
  );
}

/// Manages a process running outside `felt`.
class ProcessManager {
  /// Creates a process manager that manages [process].
  ProcessManager._({
    required this.executable,
    required this.arguments,
    required this.workingDirectory,
    required this.process,
    required bool evalOutput,
    required bool failureIsSuccess,
  }) : _evalOutput = evalOutput,
       _failureIsSuccess = failureIsSuccess {
    if (_evalOutput) {
      _forwardStream(process.stdout, _stdout);
      _forwardStream(process.stderr, _stderr);
    }
  }

  /// The executable, from which the process was spawned.
  final String executable;

  /// The arguments passed to the prcess.
  final List<String> arguments;

  /// The current working directory (CWD) of the child process.
  ///
  /// If null, the child process inherits `felt`'s CWD.
  final String? workingDirectory;

  /// The process being managed by this manager.
  final io.Process process;

  /// Whether the standard output and standard error should be decoded into
  /// strings while running the process.
  final bool _evalOutput;

  /// Whether non-zero exit code is considered successful completion of the
  /// process.
  ///
  /// See also [wait].
  final bool _failureIsSuccess;

  final StringBuffer _stdout = StringBuffer();
  final StringBuffer _stderr = StringBuffer();

  void _forwardStream(Stream<List<int>> stream, StringSink buffer) {
    stream.transform(utf8.decoder).transform(const LineSplitter()).listen(buffer.writeln);
  }

  /// Waits for the [process] to exit. Returns the exit code.
  ///
  /// The returned future completes successfully if:
  ///
  ///  * [failureIsSuccess] is false and the process exited with exit code 0.
  ///  * [failureIsSuccess] is true and the process exited with a non-zero exit code.
  ///
  /// In all other cicumstances the future completes with an error.
  Future<int> wait() async {
    final int exitCode = await process.exitCode;
    if (!_failureIsSuccess && exitCode != 0) {
      _throwProcessException(description: 'Sub-process failed.', exitCode: exitCode);
    }
    return exitCode;
  }

  /// If [evalOutput] is true, wait for the process to finish then returns the
  /// decoded standard streams.
  Future<ProcessOutput> eval() async {
    if (!_evalOutput) {
      kill();
      _throwProcessException(
        description:
            'Cannot eval process output. The process was launched '
            'with `evalOutput` set to false.',
      );
    }
    final int exitCode = await wait();
    return ProcessOutput(
      exitCode: exitCode,
      stdout: _stdout.toString(),
      stderr: _stderr.toString(),
    );
  }

  /// A convenience method on top of [eval] that only extracts standard output.
  Future<String> evalStdout() async {
    return (await eval()).stdout;
  }

  /// A convenience method on top of [eval] that only extracts standard error.
  Future<String> evalStderr() async {
    return (await eval()).stderr;
  }

  Never _throwProcessException({required String description, int? exitCode}) {
    throw ProcessException(
      description: description,
      executable: executable,
      arguments: arguments,
      workingDirectory: workingDirectory,
      exitCode: exitCode,
    );
  }

  /// Kills the [process] by sending it the [signal].
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    return process.kill(signal);
  }
}

/// Stringified standard output and standard error streams from a process.
class ProcessOutput {
  ProcessOutput({required this.exitCode, required this.stdout, required this.stderr});

  /// The exit code of the process.
  final int exitCode;

  /// Standard output of the process decoded as a string.
  final String stdout;

  /// Standard error of the process decoded as a string.
  final String stderr;
}

/// An exception related to an attempt to spawn a sub-process.
@immutable
class ProcessException implements Exception {
  const ProcessException({
    required this.description,
    required this.executable,
    required this.arguments,
    required this.workingDirectory,
    this.exitCode,
  });

  final String description;
  final String executable;
  final List<String> arguments;
  final String? workingDirectory;

  /// The exit code of the process.
  ///
  /// The value is null if the exception is thrown before the process exits.
  /// For example, this can happen on invalid attempts to start a process, or
  /// when a process is stuck and is unable to exit.
  final int? exitCode;

  @override
  String toString() {
    final StringBuffer message = StringBuffer();
    message
      ..writeln(description)
      ..writeln('Command: $executable ${arguments.join(' ')}')
      ..writeln('Working directory: ${workingDirectory ?? io.Directory.current.path}');
    if (exitCode != null) {
      message.writeln('Exit code: $exitCode');
    }
    return '$message';
  }
}

/// Adds utility methods
mixin ArgUtils<T> on Command<T> {
  /// Extracts a boolean argument from [argResults].
  bool boolArg(String name) => argResults![name] as bool;

  /// Extracts a string argument from [argResults].
  String stringArg(String name) => argResults![name] as String;
}

io.Directory getBuildDirectoryForRuntimeMode(RuntimeMode runtimeMode) => switch (runtimeMode) {
  RuntimeMode.debug => environment.wasmDebugUnoptOutDir,
  RuntimeMode.profile => environment.wasmProfileOutDir,
  RuntimeMode.release => environment.wasmReleaseOutDir,
};

/// There might be proccesses started during the tests.
///
/// Use this list to store those Processes, for cleaning up before shutdown.
final List<io.Process> processesToCleanUp = <io.Process>[];

/// There might be temporary directories created during the tests.
///
/// Use this list to store those directories and for deleteing them before
/// shutdown.
final List<io.Directory> temporaryDirectories = <io.Directory>[];

typedef AsyncCallback = Future<void> Function();

/// There might be additional cleanup needs to be done after the tools ran.
///
/// Add these operations here to make sure that they will run before felt
/// exit.
final List<AsyncCallback> cleanupCallbacks = <AsyncCallback>[];

/// Cleanup the remaning processes, close open browsers, delete temp files.
Future<void> cleanup() async {
  // Cleanup remaining processes if any.
  if (processesToCleanUp.isNotEmpty) {
    for (final io.Process process in processesToCleanUp) {
      process.kill();
    }
  }
  // Delete temporary directories.
  if (temporaryDirectories.isNotEmpty) {
    for (final io.Directory directory in temporaryDirectories) {
      if (!directory.existsSync()) {
        directory.deleteSync(recursive: true);
      }
    }
  }

  for (final AsyncCallback callback in cleanupCallbacks) {
    await callback();
  }
}

io.Directory getTestSetDirectory(TestSet testSet) {
  return io.Directory(path.join(environment.webUiTestDir.path, testSet.directory));
}

io.Directory getBundleBuildDirectory(TestBundle bundle) {
  return io.Directory(path.join(environment.webUiBuildDir.path, 'test_bundles', bundle.name));
}

io.Directory getSkiaGoldDirectoryForSuite(TestSuite suite) {
  return io.Directory(path.join(environment.webUiSkiaGoldDirectory.path, suite.name));
}

extension AnsiColors on String {
  static bool shouldEscape = () {
    if (isLuci) {
      // Produce clean output on LUCI.
      return false;
    }
    return io.stdout.hasTerminal && io.stdout.supportsAnsiEscapes;
  }();

  static const String _noColorCode = '\u001b[39m';

  String _wrapText(String prefix, String suffix) => shouldEscape ? '$prefix$this$suffix' : this;

  String _colorText(String colorCode) => _wrapText(colorCode, _noColorCode);

  String get ansiBlack => _colorText('\u001b[30m');
  String get ansiRed => _colorText('\u001b[31m');
  String get ansiGreen => _colorText('\u001b[32m');
  String get ansiYellow => _colorText('\u001b[33m');
  String get ansiBlue => _colorText('\u001b[34m');
  String get ansiMagenta => _colorText('\u001b[35m');
  String get ansiCyan => _colorText('\u001b[36m');
  String get ansiWhite => _colorText('\u001b[37m');

  String get ansiBold => _wrapText('\u001b[1m', '\u001b[0m');
}
