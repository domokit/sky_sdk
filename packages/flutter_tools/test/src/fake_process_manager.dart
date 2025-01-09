// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io show Process, ProcessResult, ProcessSignal, ProcessStartMode, systemEncoding;

import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import 'test_wrapper.dart';

export 'package:process/process.dart' show ProcessManager;

typedef VoidCallback = void Function();

/// A command for [FakeProcessManager].
@immutable
class FakeCommand {
  const FakeCommand({
    required this.command,
    this.workingDirectory,
    this.environment,
    this.encoding,
    this.duration = Duration.zero,
    this.onRun,
    this.exitCode = 0,
    this.stdout = '',
    this.stderr = '',
    this.completer,
    this.stdin,
    this.exception,
    this.outputFollowsExit = false,
    this.processStartMode,
  });

  /// The exact commands that must be matched for this [FakeCommand] to be
  /// considered correct.
  final List<Pattern> command;

  /// The exact working directory that must be matched for this [FakeCommand] to
  /// be considered correct.
  ///
  /// If this is null, the working directory is ignored.
  final String? workingDirectory;

  /// The environment that must be matched for this [FakeCommand] to be considered correct.
  ///
  /// If this is null, then the environment is ignored.
  ///
  /// Otherwise, each key in this environment must be present and must have a
  /// value that matches the one given here for the [FakeCommand] to match.
  final Map<String, String>? environment;

  /// The stdout and stderr encoding that must be matched for this [FakeCommand]
  /// to be considered correct.
  ///
  /// If this is null, then the encodings are ignored.
  final Encoding? encoding;

  /// The time to allow to elapse before returning the [exitCode], if this command
  /// is "executed".
  ///
  /// If you set this to a non-zero time, you should use a [FakeAsync] zone,
  /// otherwise the test will be artificially slow.
  final Duration duration;

  /// A callback that is run after [duration] expires but before the [exitCode]
  /// (and output) are passed back.
  ///
  /// The callback will be provided the full command that matched this instance.
  /// This can be useful in the rare scenario where the full command cannot be known
  /// ahead of time (i.e. when one or more instances of [RegExp] are used to
  /// match the command). For example, the command may contain one or more
  /// randomly-generated elements, such as a temporary directory path.
  final void Function(List<String> command)? onRun;

  /// The process' exit code.
  ///
  /// To simulate a never-ending process, set [duration] to a value greater than
  /// 15 minutes (the timeout for our tests).
  ///
  /// To simulate a crash, subtract the crash signal number from 256. For example,
  /// SIGPIPE (-13) is 243.
  final int exitCode;

  /// The output to simulate on stdout. This will be encoded as UTF-8 and
  /// returned in one go.
  final String stdout;

  /// The output to simulate on stderr. This will be encoded as UTF-8 and
  /// returned in one go.
  final String stderr;

  /// If provided, allows the command completion to be blocked until the future
  /// resolves.
  final Completer<void>? completer;

  /// An optional stdin sink that will be exposed through the resulting
  /// [FakeProcess].
  final IOSink? stdin;

  /// If provided, this exception will be thrown when the fake command is run.
  final Object? exception;

  /// When true, stdout and stderr will only be emitted after the `exitCode`
  /// [Future] on [io.Process] completes.
  final bool outputFollowsExit;

  final io.ProcessStartMode? processStartMode;

  void _matches(
    List<String> command,
    String? workingDirectory,
    Map<String, String>? environment,
    Encoding? encoding,
    io.ProcessStartMode? mode,
  ) {
    final List<dynamic> matchers =
        this.command.map((Pattern x) => x is String ? x : matches(x)).toList();
    expect(command, matchers);
    if (processStartMode != null) {
      expect(mode, processStartMode);
    }
    if (this.workingDirectory != null) {
      expect(workingDirectory, this.workingDirectory);
    }
    if (this.environment != null) {
      expect(environment, this.environment);
    }
    if (this.encoding != null) {
      expect(encoding, this.encoding);
    }
  }
}

/// A fake process for use with [FakeProcessManager].
///
/// The process delays exit until both [duration] (if specified) has elapsed
/// and [completer] (if specified) has completed.
///
/// When [outputFollowsExit] is specified, bytes are streamed to [stderr] and
/// [stdout] after the process exits.
@visibleForTesting
class FakeProcess implements io.Process {
  FakeProcess({
    int exitCode = 0,
    Duration duration = Duration.zero,
    this.pid = 1234,
    List<int> stderr = const <int>[],
    IOSink? stdin,
    List<int> stdout = const <int>[],
    Completer<void>? completer,
    bool outputFollowsExit = false,
  }) : _exitCode = exitCode,
       exitCode = Future<void>.delayed(duration).then((void value) {
         if (completer != null) {
           return completer.future.then((void _) => exitCode);
         }
         return exitCode;
       }),
       _stderr = stderr,
       stdin =
           stdin ??
           IOSink(
             StreamController<List<int>>()
               ..stream.listen((_) {})
               ..sink,
           ),
       _stdout = stdout,
       _completer = completer {
    if (_stderr.isEmpty) {
      this.stderr = const Stream<List<int>>.empty();
    } else if (outputFollowsExit) {
      // Wait for the process to exit before emitting stderr.
      this.stderr = Stream<List<int>>.fromFuture(
        this.exitCode.then((_) {
          // Return a Future so stderr isn't immediately available to those who
          // await exitCode, but is available asynchronously later.
          return Future<List<int>>(() => _stderr);
        }),
      );
    } else {
      this.stderr = Stream<List<int>>.value(_stderr);
    }

    if (_stdout.isEmpty) {
      this.stdout = const Stream<List<int>>.empty();
    } else if (outputFollowsExit) {
      // Wait for the process to exit before emitting stdout.
      this.stdout = Stream<List<int>>.fromFuture(
        this.exitCode.then((_) {
          // Return a Future so stdout isn't immediately available to those who
          // await exitCode, but is available asynchronously later.
          return Future<List<int>>(() => _stdout);
        }),
      );
    } else {
      this.stdout = Stream<List<int>>.value(_stdout);
    }
  }

  /// The process exit code.
  final int _exitCode;

  /// When specified, blocks process exit until completed.
  final Completer<void>? _completer;

  @override
  final Future<int> exitCode;

  @override
  final int pid;

  /// The raw byte content of stderr.
  final List<int> _stderr;

  @override
  late final Stream<List<int>> stderr;

  @override
  final IOSink stdin;

  @override
  late final Stream<List<int>> stdout;

  /// The raw byte content of stdout.
  final List<int> _stdout;

  /// The list of [kill] signals this process received so far.
  @visibleForTesting
  List<io.ProcessSignal> get signals => _signals;
  final List<io.ProcessSignal> _signals = <io.ProcessSignal>[];

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    _signals.add(signal);

    // Killing a fake process has no effect.
    return true;
  }
}

abstract class FakeProcessManager implements ProcessManager {
  /// A fake [ProcessManager] which responds to all commands as if they had run
  /// instantaneously with an exit code of 0 and no output.
  factory FakeProcessManager.any() = _FakeAnyProcessManager;

  /// A fake [ProcessManager] which responds to particular commands with
  /// particular results.
  ///
  /// On creation, pass in a list of [FakeCommand] objects. When the
  /// [ProcessManager] methods such as [start] are invoked, the next
  /// [FakeCommand] must match (otherwise the test fails); its settings are used
  /// to simulate the result of running that command.
  ///
  /// If no command is found, then one is implied which immediately returns exit
  /// code 0 with no output.
  ///
  /// There is no logic to ensure that all the listed commands are run. Use
  /// [FakeCommand.onRun] to set a flag, or specify a sentinel command as your
  /// last command and verify its execution is successful, to ensure that all
  /// the specified commands are actually called.
  factory FakeProcessManager.list(List<FakeCommand> commands) = _SequenceProcessManager;
  factory FakeProcessManager.empty() => _SequenceProcessManager(<FakeCommand>[]);

  FakeProcessManager._();

  /// Adds a new [FakeCommand] to the current process manager.
  ///
  /// This can be used to configure test expectations after the [ProcessManager] has been
  /// provided to another interface.
  ///
  /// This is a no-op on [FakeProcessManager.any].
  void addCommand(FakeCommand command);

  /// Add multiple [FakeCommand] to the current process manager.
  void addCommands(Iterable<FakeCommand> commands) {
    commands.forEach(addCommand);
  }

  final Map<int, FakeProcess> _fakeRunningProcesses = <int, FakeProcess>{};

  /// Whether this fake has more [FakeCommand]s that are expected to run.
  ///
  /// This is always `true` for [FakeProcessManager.any].
  bool get hasRemainingExpectations;

  /// The expected [FakeCommand]s that have not yet run.
  List<FakeCommand> get _remainingExpectations;

  @protected
  FakeCommand findCommand(
    List<String> command,
    String? workingDirectory,
    Map<String, String>? environment,
    Encoding? encoding,
    io.ProcessStartMode? mode,
  );

  int _pid = 9999;

  FakeProcess _runCommand(
    List<String> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    Encoding? encoding,
    io.ProcessStartMode? mode,
  }) {
    _pid += 1;
    final FakeCommand fakeCommand = findCommand(
      command,
      workingDirectory,
      environment,
      encoding,
      mode,
    );
    if (fakeCommand.exception != null) {
      assert(fakeCommand.exception is Exception || fakeCommand.exception is Error);
      throw fakeCommand.exception!; // ignore: only_throw_errors
    }
    if (fakeCommand.onRun != null) {
      fakeCommand.onRun!(command);
    }
    return FakeProcess(
      duration: fakeCommand.duration,
      exitCode: fakeCommand.exitCode,
      pid: _pid,
      stderr: encoding?.encode(fakeCommand.stderr) ?? fakeCommand.stderr.codeUnits,
      stdin: fakeCommand.stdin,
      stdout: encoding?.encode(fakeCommand.stdout) ?? fakeCommand.stdout.codeUnits,
      completer: fakeCommand.completer,
      outputFollowsExit: fakeCommand.outputFollowsExit,
    );
  }

  @override
  Future<io.Process> start(
    List<dynamic> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true, // ignored
    bool runInShell = false, // ignored
    io.ProcessStartMode mode = io.ProcessStartMode.normal,
  }) {
    final FakeProcess process = _runCommand(
      command.cast<String>(),
      workingDirectory: workingDirectory,
      environment: environment,
      encoding: io.systemEncoding,
      mode: mode,
    );
    if (process._completer != null) {
      _fakeRunningProcesses[process.pid] = process;
      process.exitCode.whenComplete(() {
        _fakeRunningProcesses.remove(process.pid);
      });
    }
    return Future<io.Process>.value(process);
  }

  @override
  Future<io.ProcessResult> run(
    List<dynamic> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true, // ignored
    bool runInShell = false, // ignored
    Encoding? stdoutEncoding = io.systemEncoding,
    Encoding? stderrEncoding = io.systemEncoding,
  }) async {
    final FakeProcess process = _runCommand(
      command.cast<String>(),
      workingDirectory: workingDirectory,
      environment: environment,
      encoding: stdoutEncoding,
    );
    await process.exitCode;
    return io.ProcessResult(
      process.pid,
      process._exitCode,
      stdoutEncoding == null ? process._stdout : await stdoutEncoding.decodeStream(process.stdout),
      stderrEncoding == null ? process._stderr : await stderrEncoding.decodeStream(process.stderr),
    );
  }

  @override
  io.ProcessResult runSync(
    List<dynamic> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true, // ignored
    bool runInShell = false, // ignored
    Encoding? stdoutEncoding = io.systemEncoding,
    Encoding? stderrEncoding = io.systemEncoding,
  }) {
    final FakeProcess process = _runCommand(
      command.cast<String>(),
      workingDirectory: workingDirectory,
      environment: environment,
      encoding: stdoutEncoding,
    );
    return io.ProcessResult(
      process.pid,
      process._exitCode,
      stdoutEncoding == null ? process._stdout : stdoutEncoding.decode(process._stdout),
      stderrEncoding == null ? process._stderr : stderrEncoding.decode(process._stderr),
    );
  }

  /// Returns false if executable in [excludedExecutables].
  @override
  bool canRun(dynamic executable, {String? workingDirectory}) =>
      !excludedExecutables.contains(executable);

  Set<String> excludedExecutables = <String>{};

  @override
  bool killPid(int pid, [io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    // Killing a fake process has no effect unless it has an attached completer.
    final FakeProcess? fakeProcess = _fakeRunningProcesses[pid];
    if (fakeProcess == null) {
      return false;
    }
    fakeProcess.kill(signal);
    if (fakeProcess._completer != null) {
      fakeProcess._completer.complete();
    }
    return true;
  }
}

class _FakeAnyProcessManager extends FakeProcessManager {
  _FakeAnyProcessManager() : super._();

  @override
  FakeCommand findCommand(
    List<String> command,
    String? workingDirectory,
    Map<String, String>? environment,
    Encoding? encoding,
    io.ProcessStartMode? mode,
  ) {
    return FakeCommand(
      command: command,
      workingDirectory: workingDirectory,
      environment: environment,
      encoding: encoding,
      processStartMode: mode,
    );
  }

  @override
  void addCommand(FakeCommand command) {}

  @override
  bool get hasRemainingExpectations => true;

  @override
  List<FakeCommand> get _remainingExpectations => <FakeCommand>[];
}

class _SequenceProcessManager extends FakeProcessManager {
  _SequenceProcessManager(this._commands) : super._();

  final List<FakeCommand> _commands;

  @override
  FakeCommand findCommand(
    List<String> command,
    String? workingDirectory,
    Map<String, String>? environment,
    Encoding? encoding,
    io.ProcessStartMode? mode,
  ) {
    expect(
      _commands,
      isNotEmpty,
      reason:
          'ProcessManager was told to execute $command (in $workingDirectory) '
          'but the FakeProcessManager.list expected no more processes.',
    );
    _commands.first._matches(command, workingDirectory, environment, encoding, mode);
    return _commands.removeAt(0);
  }

  @override
  void addCommand(FakeCommand command) {
    _commands.add(command);
  }

  @override
  bool get hasRemainingExpectations => _commands.isNotEmpty;

  @override
  List<FakeCommand> get _remainingExpectations => _commands;
}

/// Matcher that successfully matches against a [FakeProcessManager] with
/// no remaining expectations ([item.hasRemainingExpectations] returns false).
const Matcher hasNoRemainingExpectations = _HasNoRemainingExpectations();

class _HasNoRemainingExpectations extends Matcher {
  const _HasNoRemainingExpectations();

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) =>
      item is FakeProcessManager && !item.hasRemainingExpectations;

  @override
  Description describe(Description description) =>
      description.add('a fake process manager with no remaining expectations');

  @override
  Description describeMismatch(
    dynamic item,
    Description description,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    final FakeProcessManager fakeProcessManager = item as FakeProcessManager;
    return description.add(
      'has remaining expectations:\n${fakeProcessManager._remainingExpectations.map((FakeCommand command) => command.command).join('\n')}',
    );
  }
}
