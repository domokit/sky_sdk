// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:path/path.dart' as path;
import 'package:watcher/watcher.dart';

import 'exceptions.dart';
import 'utils.dart';

/// Describes what [Pipeline] is currently doing.
enum PipelineStatus {
  /// The pipeline has not started yet.
  ///
  /// This is the initial state of the pipeline.
  idle,

  /// The pipeline is running build steps.
  started,

  /// The pipeline is stopping.
  stopping,

  /// The pipeline is not running anything because it has been interrupted.
  interrupted,

  /// The pipeline is not running anything because it encountered an error.
  error,

  /// The pipeline is not running anything because it finished all build steps successfully.
  done,
}

/// A step in the build pipeline.
abstract class PipelineStep {
  /// The name of this step.
  ///
  /// This value appears in logs, so it should be descriptive and human-readable.
  String get description;

  /// Whether it is safe to interrupt this step while it's running.
  bool get isSafeToInterrupt;

  /// Runs this step.
  ///
  /// The returned future is completed when the step is finished. The future
  /// completes with an error if the step failed.
  Future<void> run();

  /// Interrupts this step, if it's already running.
  ///
  /// [Pipeline] only calls this if [isSafeToInterrupt] returns true.
  Future<void> interrupt();
}

/// A helper class for implementing [PipelineStep] in terms of a process.
abstract class ProcessStep implements PipelineStep {
  ProcessManager? _process;
  bool _isInterrupted = false;

  /// Starts and returns the process that implements the logic of this pipeline
  /// step.
  Future<ProcessManager> createProcess();

  @override
  Future<void> interrupt() async {
    _isInterrupted = true;
    _process?.kill();
  }

  @override
  Future<void> run() async {
    final ProcessManager process = await createProcess();

    if (_isInterrupted) {
      // If the step was interrupted while creating the process, the
      // `interrupt` won't kill the process; it must be done here.
      process.kill();
      return;
    }

    _process = process;
    await process.wait();
    _process = null;
  }
}

class _PipelineStepFailure {
  _PipelineStepFailure(this.step, this.error);

  final PipelineStep step;
  final Object error;
}

/// Executes a sequence of asynchronous tasks, typically as part of a build/test
/// process.
///
/// The pipeline can be executed by calling [start] and stopped by calling
/// [stop].
///
/// When a pipeline is stopped, it switches to the [PipelineStatus.stopping]
/// state. If [PipelineStep.isSafeToInterrupt] is true, interrupts the currently
/// running step and skips the rest. Otherwise, waits until the current task
/// finishes and skips the rest.
class Pipeline {
  Pipeline({required this.steps});

  final Iterable<PipelineStep> steps;

  PipelineStep? _currentStep;
  Future<void>? _currentStepFuture;

  PipelineStatus get status => _status;
  PipelineStatus _status = PipelineStatus.idle;

  /// Runs the steps of the pipeline.
  ///
  /// Returns a future that resolves after all steps have been performed.
  ///
  /// If any steps fail, the pipeline attempts to continue to subsequent steps,
  /// but will fail at the end.
  ///
  /// The pipeline may be interrupted by calling [stop] before the future
  /// resolves.
  Future<void> run() async {
    _status = PipelineStatus.started;
    final List<_PipelineStepFailure> failures = <_PipelineStepFailure>[];
    for (final PipelineStep step in steps) {
      _currentStep = step;
      _currentStepFuture = step.run();
      try {
        await _currentStepFuture;
      } catch (e) {
        failures.add(_PipelineStepFailure(step, e));
      } finally {
        _currentStep = null;
      }
    }
    if (failures.isEmpty) {
      _status = PipelineStatus.done;
    } else {
      _status = PipelineStatus.error;
      print('Pipeline experienced the following failures:');
      for (final _PipelineStepFailure failure in failures) {
        print('  "${failure.step.description}": ${failure.error}');
      }
      throw ToolExit('Test pipeline failed.');
    }
  }

  /// Stops executing any more tasks in the pipeline.
  ///
  /// Tasks that are safe to interrupt (according to [PipelineStep.isSafeToInterrupt]),
  /// are interrupted. Otherwise, waits for the current step to finish before
  /// interrupting the pipeline.
  Future<void> stop() async {
    _status = PipelineStatus.stopping;
    final PipelineStep? step = _currentStep;
    if (step == null) {
      _status = PipelineStatus.interrupted;
      return;
    }
    if (step.isSafeToInterrupt) {
      print('Interrupting ${step.description}');
      await step.interrupt();
      _status = PipelineStatus.interrupted;
      return;
    }
    print('${step.description} cannot be interrupted. Waiting for it to complete.');
    await _currentStepFuture;
    _status = PipelineStatus.interrupted;
  }
}

/// Signature of functions to be called when a [WatchEvent] is received.
typedef WatchEventPredicate = bool Function(WatchEvent event);

/// Responsible for watching a directory [dir] and executing the given
/// [pipeline] whenever a change occurs in the directory.
///
/// The [ignore] callback can be used to customize the watching behavior to
/// ignore certain files.
class PipelineWatcher {
  PipelineWatcher({required this.dir, required this.pipeline, this.ignore})
    : watcher = DirectoryWatcher(dir);

  /// The path of the directory to watch for changes.
  final String dir;

  /// The pipeline to be executed when an event is fired by the watcher.
  final Pipeline pipeline;

  /// Used to watch a directory for any file system changes.
  final DirectoryWatcher watcher;

  /// A callback that determines whether to rerun the pipeline or not for a
  /// given [WatchEvent] instance.
  final WatchEventPredicate? ignore;

  /// Activates the watcher.
  Future<void> start() async {
    watcher.events.listen(_onEvent);

    // Listen to the `q` key stroke to stop the pipeline.
    print("Press 'q' to exit felt");

    // Key strokes should be reported immediately and one at a time rather than
    // wait for the user to hit ENTER and report the whole line. To achieve
    // that, echo mode and line mode must be disabled.
    io.stdin.echoMode = false;
    io.stdin.lineMode = false;

    // Reset these settings when the felt command is done.
    cleanupCallbacks.add(() async {
      io.stdin.echoMode = true;
      io.stdin.lineMode = true;
    });

    await io.stdin.firstWhere((List<int> event) {
      const int qKeyCode = 113;
      final bool qEntered = event.isNotEmpty && event.first == qKeyCode;
      return qEntered;
    });
    print('Stopping felt');
    await pipeline.stop();
  }

  int _pipelineRunCount = 0;
  Timer? _scheduledPipeline;

  void _onEvent(WatchEvent event) {
    if (ignore?.call(event) ?? false) {
      return;
    }

    final String relativePath = path.relative(event.path, from: dir);
    print('- [${event.type}] $relativePath');

    _pipelineRunCount++;
    _scheduledPipeline?.cancel();
    _scheduledPipeline = Timer(const Duration(milliseconds: 100), () {
      _scheduledPipeline = null;
      _runPipeline();
    });
  }

  Future<void> _runPipeline() async {
    if (pipeline.status == PipelineStatus.stopping) {
      // We are already trying to stop the pipeline. No need to do anything.
      return;
    }

    if (pipeline.status == PipelineStatus.started) {
      // If the pipeline already running, stop it before starting it again.
      await pipeline.stop();
    }

    final int runCount = _pipelineRunCount;
    try {
      await pipeline.run();
      _pipelineSucceeded(runCount);
    } catch (error, stackTrace) {
      // The error is printed but not rethrown. This is because in watch mode
      // failures are expected. The idea is that the developer corrects the
      // error, saves the file, and the pipeline reruns.
      _pipelineFailed(error, stackTrace);
    }
  }

  void _pipelineSucceeded(int pipelineRunCount) {
    if (pipelineRunCount == _pipelineRunCount) {
      print('*** Done! ***');
      print("Press 'q' to exit felt");
    }
  }

  void _pipelineFailed(Object error, StackTrace stackTrace) {
    print('felt command failed: $error');
    print(stackTrace);
  }
}
