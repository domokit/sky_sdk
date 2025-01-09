// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:meta/meta.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

import 'cocoon.dart';
import 'devices.dart';
import 'task_result.dart';
import 'utils.dart';

/// Run a list of tasks.
///
/// For each task, an auto rerun will be triggered when task fails.
///
/// If the task succeeds the first time, it will be recorded as successful.
///
/// If the task fails first, but gets passed in the end, the
/// test will be recorded as successful but with a flake flag.
///
/// If the task fails all reruns, it will be recorded as failed.
Future<void> runTasks(
  List<String> taskNames, {
  bool exitOnFirstTestFailure = false,
  // terminateStrayDartProcesses defaults to false so that tests don't have to specify it.
  // It is set based on the --terminate-stray-dart-processes command line argument in
  // normal execution, and that flag defaults to true.
  bool terminateStrayDartProcesses = false,
  bool silent = false,
  String? deviceId,
  String? gitBranch,
  String? localEngine,
  String? localEngineHost,
  String? localEngineSrcPath,
  String? luciBuilder,
  String? resultsPath,
  List<String>? taskArgs,
  bool useEmulator = false,
  @visibleForTesting Map<String, String>? isolateParams,
  @visibleForTesting void Function(String) print = print,
  @visibleForTesting List<String>? logs,
}) async {
  for (final String taskName in taskNames) {
    TaskResult result = TaskResult.success(null);
    int failureCount = 0;
    while (failureCount <= Cocoon.retryNumber) {
      result = await rerunTask(
        taskName,
        deviceId: deviceId,
        localEngine: localEngine,
        localEngineHost: localEngineHost,
        localEngineSrcPath: localEngineSrcPath,
        terminateStrayDartProcesses: terminateStrayDartProcesses,
        silent: silent,
        taskArgs: taskArgs,
        resultsPath: resultsPath,
        gitBranch: gitBranch,
        luciBuilder: luciBuilder,
        isolateParams: isolateParams,
        useEmulator: useEmulator,
      );

      if (!result.succeeded) {
        failureCount += 1;
        if (exitOnFirstTestFailure) {
          break;
        }
      } else {
        section('Flaky status for "$taskName"');
        if (failureCount > 0) {
          print(
            'Total ${failureCount + 1} executions: $failureCount failures and 1 false positive.',
          );
          print('flaky: true');
          // TODO(ianh): stop ignoring this failure. We should set exitCode=1, and quit
          // if exitOnFirstTestFailure is true.
        } else {
          print('Test passed on first attempt.');
          print('flaky: false');
        }
        break;
      }
    }

    if (!result.succeeded) {
      section('Flaky status for "$taskName"');
      print('Consistently failed across all $failureCount executions.');
      print('flaky: false');
      exitCode = 1;
      if (exitOnFirstTestFailure) {
        return;
      }
    }
  }
}

/// A rerun wrapper for `runTask`.
///
/// This separates reruns in separate sections.
Future<TaskResult> rerunTask(
  String taskName, {
  String? deviceId,
  String? localEngine,
  String? localEngineHost,
  String? localEngineSrcPath,
  bool terminateStrayDartProcesses = false,
  bool silent = false,
  List<String>? taskArgs,
  String? resultsPath,
  String? gitBranch,
  String? luciBuilder,
  bool useEmulator = false,
  @visibleForTesting Map<String, String>? isolateParams,
}) async {
  section('Running task "$taskName"');
  final TaskResult result = await runTask(
    taskName,
    deviceId: deviceId,
    localEngine: localEngine,
    localEngineHost: localEngineHost,
    localEngineSrcPath: localEngineSrcPath,
    terminateStrayDartProcesses: terminateStrayDartProcesses,
    silent: silent,
    taskArgs: taskArgs,
    isolateParams: isolateParams,
    useEmulator: useEmulator,
  );

  print('Task result:');
  print(const JsonEncoder.withIndent('  ').convert(result));
  section('Finished task "$taskName"');

  if (resultsPath != null) {
    final Cocoon cocoon = Cocoon();
    await cocoon.writeTaskResultToFile(
      builderName: luciBuilder,
      gitBranch: gitBranch,
      result: result,
      resultsPath: resultsPath,
    );
  }
  return result;
}

/// Runs a task in a separate Dart VM and collects the result using the VM
/// service protocol.
///
/// [taskName] is the name of the task. The corresponding task executable is
/// expected to be found under `bin/tasks`.
///
/// Running the task in [silent] mode will suppress standard output from task
/// processes and only print standard errors.
///
/// [taskArgs] are passed to the task executable for additional configuration.
Future<TaskResult> runTask(
  String taskName, {
  bool terminateStrayDartProcesses = false,
  bool silent = false,
  String? localEngine,
  String? localEngineHost,
  String? localWebSdk,
  String? localEngineSrcPath,
  String? deviceId,
  List<String>? taskArgs,
  bool useEmulator = false,
  @visibleForTesting Map<String, String>? isolateParams,
}) async {
  final String taskExecutable = 'bin/tasks/$taskName.dart';

  if (!file(taskExecutable).existsSync()) {
    print('Executable Dart file not found: $taskExecutable');
    exit(1);
  }

  if (useEmulator) {
    taskArgs ??= <String>[];
    taskArgs
      ..add('--android-emulator')
      ..add('--browser-name=android-chrome');
  }

  stdout.writeln('Starting process for task: [$taskName]');

  final Process runner = await startProcess(
    dartBin,
    <String>[
      '--enable-vm-service=0', // zero causes the system to choose a free port
      '--no-pause-isolates-on-exit',
      if (localEngine != null) '-DlocalEngine=$localEngine',
      if (localEngineHost != null) '-DlocalEngineHost=$localEngineHost',
      if (localWebSdk != null) '-DlocalWebSdk=$localWebSdk',
      if (localEngineSrcPath != null) '-DlocalEngineSrcPath=$localEngineSrcPath',
      taskExecutable,
      ...?taskArgs,
    ],
    environment: <String, String>{if (deviceId != null) DeviceIdEnvName: deviceId},
  );

  bool runnerFinished = false;

  unawaited(
    runner.exitCode.whenComplete(() {
      runnerFinished = true;
    }),
  );

  final Completer<Uri> uri = Completer<Uri>();

  final StreamSubscription<String> stdoutSub = runner.stdout
      .transform<String>(const Utf8Decoder())
      .transform<String>(const LineSplitter())
      .listen((String line) {
        if (!uri.isCompleted) {
          final Uri? serviceUri = parseServiceUri(
            line,
            prefix: RegExp('The Dart VM service is listening on '),
          );
          if (serviceUri != null) {
            uri.complete(serviceUri);
          }
        }
        if (!silent) {
          stdout.writeln('[${DateTime.now()}] [STDOUT] $line');
        }
      });

  final StreamSubscription<String> stderrSub = runner.stderr
      .transform<String>(const Utf8Decoder())
      .transform<String>(const LineSplitter())
      .listen((String line) {
        stderr.writeln('[${DateTime.now()}] [STDERR] $line');
      });

  try {
    final ConnectionResult result = await _connectToRunnerIsolate(await uri.future);
    print('[$taskName] Connected to VM server.');
    isolateParams =
        isolateParams == null ? <String, String>{} : Map<String, String>.of(isolateParams);
    isolateParams['runProcessCleanup'] = terminateStrayDartProcesses.toString();
    final VmService service = result.vmService;
    final String isolateId = result.isolate.id!;
    final Map<String, dynamic> taskResultJson =
        (await service.callServiceExtension(
          'ext.cocoonRunTask',
          args: isolateParams,
          isolateId: isolateId,
        )).json!;
    // Notify the task process that the task result has been received and it
    // can proceed to shutdown.
    await _acknowledgeTaskResultReceived(service: service, isolateId: isolateId);
    final TaskResult taskResult = TaskResult.fromJson(taskResultJson);
    final int exitCode = await runner.exitCode;
    print('[$taskName] Process terminated with exit code $exitCode.');
    return taskResult;
  } catch (error, stack) {
    print('[$taskName] Task runner system failed with exception!\n$error\n$stack');
    rethrow;
  } finally {
    if (!runnerFinished) {
      print('[$taskName] Terminating process...');
      runner.kill(ProcessSignal.sigkill);
    }
    await stdoutSub.cancel();
    await stderrSub.cancel();
  }
}

Future<ConnectionResult> _connectToRunnerIsolate(Uri vmServiceUri) async {
  final List<String> pathSegments = <String>[
    // Add authentication code.
    if (vmServiceUri.pathSegments.isNotEmpty) vmServiceUri.pathSegments[0],
    'ws',
  ];
  final String url = vmServiceUri.replace(scheme: 'ws', pathSegments: pathSegments).toString();
  final Stopwatch stopwatch = Stopwatch()..start();

  while (true) {
    try {
      // Look up the isolate.
      final VmService client = await vmServiceConnectUri(url);
      VM vm = await client.getVM();
      while (vm.isolates!.isEmpty) {
        await Future<void>.delayed(const Duration(seconds: 1));
        vm = await client.getVM();
      }
      final IsolateRef isolate = vm.isolates!.first;
      // Sanity check to ensure we're talking with the main isolate.
      final Response response = await client.callServiceExtension(
        'ext.cocoonRunnerReady',
        isolateId: isolate.id,
      );
      if (response.json!['result'] != 'success') {
        throw 'not ready yet';
      }
      return ConnectionResult(client, isolate);
    } catch (error) {
      if (stopwatch.elapsed > const Duration(seconds: 10)) {
        print(
          'VM service still not ready. It is possible the target has failed.\n'
          'Latest connection error:\n'
          '  $error\n'
          'Continuing to retry...\n',
        );
        stopwatch.reset();
      }
      await Future<void>.delayed(const Duration(milliseconds: 50));
    }
  }
}

Future<void> _acknowledgeTaskResultReceived({
  required VmService service,
  required String isolateId,
}) async {
  try {
    await service.callServiceExtension('ext.cocoonTaskResultReceived', isolateId: isolateId);
  } on RPCError {
    // The target VM may shutdown before the response is received.
  }
}

class ConnectionResult {
  ConnectionResult(this.vmService, this.isolate);

  final VmService vmService;
  final IsolateRef isolate;
}
