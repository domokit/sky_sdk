// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:developer';
import 'dart:io';
import 'dart:isolate';

import 'package:logging/logging.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';
import 'package:stack_trace/stack_trace.dart';

import 'devices.dart';
import 'host_agent.dart';
import 'running_processes.dart';
import 'task_result.dart';
import 'utils.dart';

/// Identifiers for devices that should never be rebooted.
final Set<String> noRebootForbidList = <String>{
  '822ef7958bba573829d85eef4df6cbdd86593730', // 32bit iPhone requires manual intervention on reboot.
};

/// The maximum number of test runs before a device must be rebooted.
///
/// This number was chosen arbitrarily.
const int maximumRuns = 30;

/// Represents a unit of work performed in the CI environment that can
/// succeed, fail and be retried independently of others.
typedef TaskFunction = Future<TaskResult> Function();

bool _isTaskRegistered = false;

/// Registers a [task] to run, returns the result when it is complete.
///
/// The task does not run immediately but waits for the request via the
/// VM service protocol to run it.
///
/// It is OK for a [task] to perform many things. However, only one task can be
/// registered per Dart VM.
///
/// If no `processManager` is provided, a default [LocalProcessManager] is created
/// for the task.
Future<TaskResult> task(TaskFunction task, {ProcessManager? processManager}) async {
  if (_isTaskRegistered) {
    throw StateError('A task is already registered');
  }
  _isTaskRegistered = true;

  processManager ??= const LocalProcessManager();

  // TODO(ianh): allow overriding logging.
  Logger.root.level = Level.ALL;
  Logger.root.onRecord.listen((LogRecord rec) {
    print('${rec.level.name}: ${rec.time}: ${rec.message}');
  });

  final _TaskRunner runner = _TaskRunner(task, processManager);
  runner.keepVmAliveUntilTaskRunRequested();
  return runner.whenDone;
}

class _TaskRunner {
  _TaskRunner(this.task, this.processManager) {
    final String successResponse = json.encode(const <String, String>{'result': 'success'});

    registerExtension('ext.cocoonRunTask', (String method, Map<String, String> parameters) async {
      final Duration? taskTimeout =
          parameters.containsKey('timeoutInMinutes')
              ? Duration(minutes: int.parse(parameters['timeoutInMinutes']!))
              : null;
      final bool runFlutterConfig =
          parameters['runFlutterConfig'] !=
          'false'; // used by tests to avoid changing the configuration
      final bool runProcessCleanup = parameters['runProcessCleanup'] != 'false';
      final String? localEngine = parameters['localEngine'];
      final String? localEngineHost = parameters['localEngineHost'];
      final TaskResult result = await run(
        taskTimeout,
        runProcessCleanup: runProcessCleanup,
        runFlutterConfig: runFlutterConfig,
        localEngine: localEngine,
        localEngineHost: localEngineHost,
      );
      const Duration taskResultReceivedTimeout = Duration(seconds: 30);
      _taskResultReceivedTimeout = Timer(taskResultReceivedTimeout, () {
        logger.severe(
          'Task runner did not acknowledge task results in $taskResultReceivedTimeout.',
        );
        _closeKeepAlivePort();
        exitCode = 1;
      });
      return ServiceExtensionResponse.result(json.encode(result.toJson()));
    });
    registerExtension('ext.cocoonRunnerReady', (
      String method,
      Map<String, String> parameters,
    ) async {
      return ServiceExtensionResponse.result(successResponse);
    });
    registerExtension('ext.cocoonTaskResultReceived', (
      String method,
      Map<String, String> parameters,
    ) async {
      _closeKeepAlivePort();
      return ServiceExtensionResponse.result(successResponse);
    });
  }

  final TaskFunction task;
  final ProcessManager processManager;

  Future<Device?> _getWorkingDeviceIfAvailable() async {
    try {
      return await devices.workingDevice;
    } on DeviceException {
      return null;
    }
  }

  // TODO(ianh): workaround for https://github.com/dart-lang/sdk/issues/23797
  RawReceivePort? _keepAlivePort;
  Timer? _startTaskTimeout;
  Timer? _taskResultReceivedTimeout;
  bool _taskStarted = false;

  final Completer<TaskResult> _completer = Completer<TaskResult>();

  static final Logger logger = Logger('TaskRunner');

  /// Signals that this task runner finished running the task.
  Future<TaskResult> get whenDone => _completer.future;

  Future<TaskResult> run(
    Duration? taskTimeout, {
    bool runFlutterConfig = true,
    bool runProcessCleanup = true,
    required String? localEngine,
    required String? localEngineHost,
  }) async {
    try {
      _taskStarted = true;
      print('Running task with a timeout of $taskTimeout.');
      final String exe = Platform.isWindows ? '.exe' : '';
      late Set<RunningProcessInfo> beforeRunningDartInstances;
      if (runProcessCleanup) {
        section('Checking running Dart$exe processes');
        beforeRunningDartInstances = await getRunningProcesses(
          processName: 'dart$exe',
          processManager: processManager,
        );
        final Set<RunningProcessInfo> allProcesses = await getRunningProcesses(
          processManager: processManager,
        );
        beforeRunningDartInstances.forEach(print);
        for (final RunningProcessInfo info in allProcesses) {
          if (info.commandLine.contains('iproxy')) {
            print('[LEAK]: ${info.commandLine} ${info.creationDate} ${info.pid} ');
          }
        }
      }

      if (runFlutterConfig) {
        print('Enabling configs for macOS and Linux...');
        final int configResult = await exec(
          path.join(flutterDirectory.path, 'bin', 'flutter'),
          <String>[
            'config',
            '-v',
            '--enable-macos-desktop',
            '--enable-linux-desktop',
            if (localEngine != null) ...<String>['--local-engine', localEngine],
            if (localEngineHost != null) ...<String>['--local-engine-host', localEngineHost],
          ],
          canFail: true,
        );
        if (configResult != 0) {
          print('Failed to enable configuration, tasks may not run.');
        }
      }

      final Device? device = await _getWorkingDeviceIfAvailable();

      // Some tests assume the phone is in home
      await device?.home();

      late TaskResult result;
      IOSink? sink;
      try {
        if (device != null && device.canStreamLogs && hostAgent.dumpDirectory != null) {
          sink =
              File(path.join(hostAgent.dumpDirectory!.path, '${device.deviceId}.log')).openWrite();
          await device.startLoggingToSink(sink);
        }

        Future<TaskResult> futureResult = _performTask();
        if (taskTimeout != null) {
          futureResult = futureResult.timeout(taskTimeout);
        }

        result = await futureResult;
      } finally {
        if (device != null && device.canStreamLogs) {
          await device.stopLoggingToSink();
          await sink?.close();
        }
      }

      if (runProcessCleanup) {
        section('Terminating lingering Dart$exe processes after task...');
        final Set<RunningProcessInfo> afterRunningDartInstances = await getRunningProcesses(
          processName: 'dart$exe',
          processManager: processManager,
        );
        for (final RunningProcessInfo info in afterRunningDartInstances) {
          if (!beforeRunningDartInstances.contains(info)) {
            print('$info was leaked by this test.');
            if (result is TaskResultCheckProcesses) {
              result = TaskResult.failure('This test leaked dart processes');
            }
            if (await info.terminate(processManager: processManager)) {
              print('Killed process id ${info.pid}.');
            } else {
              print('Failed to kill process ${info.pid}.');
            }
          }
        }
      }
      _completer.complete(result);
      return result;
    } on TimeoutException catch (err, stackTrace) {
      print('Task timed out in framework.dart after $taskTimeout.');
      print(err);
      print(stackTrace);
      return TaskResult.failure('Task timed out after $taskTimeout');
    } finally {
      await checkForRebootRequired();
      await forceQuitRunningProcesses();
    }
  }

  Future<void> checkForRebootRequired() async {
    print('Checking for reboot');
    try {
      final Device device = await devices.workingDevice;
      if (noRebootForbidList.contains(device.deviceId)) {
        return;
      }
      final File rebootFile = _rebootFile();
      int runCount;
      if (rebootFile.existsSync()) {
        runCount = int.tryParse(rebootFile.readAsStringSync().trim()) ?? 0;
      } else {
        runCount = 0;
      }
      if (runCount < maximumRuns) {
        rebootFile
          ..createSync()
          ..writeAsStringSync((runCount + 1).toString());
        return;
      }
      rebootFile.deleteSync();
      print('rebooting');
      await device.reboot();
    } on TimeoutException {
      // Could not find device in order to reboot.
    } on DeviceException {
      // No attached device needed to reboot.
    }
  }

  /// Causes the Dart VM to stay alive until a request to run the task is
  /// received via the VM service protocol.
  void keepVmAliveUntilTaskRunRequested() {
    if (_taskStarted) {
      throw StateError('Task already started.');
    }

    // Merely creating this port object will cause the VM to stay alive and keep
    // the VM service server running until the port is disposed of.
    _keepAlivePort = RawReceivePort();

    // Timeout if nothing bothers to connect and ask us to run the task.
    const Duration taskStartTimeout = Duration(seconds: 60);
    _startTaskTimeout = Timer(taskStartTimeout, () {
      if (!_taskStarted) {
        logger.severe('Task did not start in $taskStartTimeout.');
        _closeKeepAlivePort();
        exitCode = 1;
      }
    });
  }

  /// Disables the keepalive port, allowing the VM to exit.
  void _closeKeepAlivePort() {
    _startTaskTimeout?.cancel();
    _taskResultReceivedTimeout?.cancel();
    _keepAlivePort?.close();
  }

  Future<TaskResult> _performTask() {
    final Completer<TaskResult> completer = Completer<TaskResult>();
    Chain.capture(
      () async {
        completer.complete(await task());
      },
      onError: (dynamic taskError, Chain taskErrorStack) {
        final String message = 'Task failed: $taskError';
        stderr
          ..writeln(message)
          ..writeln('\nStack trace:')
          ..writeln(taskErrorStack.terse);
        // IMPORTANT: We're completing the future _successfully_ but with a value
        // that indicates a task failure. This is intentional. At this point we
        // are catching errors coming from arbitrary (and untrustworthy) task
        // code. Our goal is to convert the failure into a readable message.
        // Propagating it further is not useful.
        if (!completer.isCompleted) {
          completer.complete(TaskResult.failure(message));
        }
      },
    );
    return completer.future;
  }
}

File _rebootFile() {
  if (Platform.isLinux || Platform.isMacOS) {
    return File(path.join(Platform.environment['HOME']!, '.reboot-count'));
  }
  if (!Platform.isWindows) {
    throw StateError('Unexpected platform ${Platform.operatingSystem}');
  }
  return File(path.join(Platform.environment['USERPROFILE']!, '.reboot-count'));
}
