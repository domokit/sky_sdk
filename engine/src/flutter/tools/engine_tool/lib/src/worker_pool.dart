// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:collection';

import 'environment.dart';

/// A WorkerTask.
abstract class WorkerTask {
  /// WorkerTask with a name.
  WorkerTask(this.name);

  /// Name of worker task.
  final String name;

  /// Run the task which is complete when the returned future completes.
  /// Returns true if successful.
  Future<bool> run();

  /// Returns the run time of this task.
  Duration get runTime {
    if (_startTime == null || _finishTime == null) {
      return Duration.zero;
    }
    return _finishTime!.difference(_startTime!);
  }

  // When did this task start running?
  DateTime? _startTime;
  // When did this task finish running?
  DateTime? _finishTime;
}

/// A pool of worker tasks that will run numWorkers tasks at a time
/// until all of the tasks are finished.
class WorkerPool {
  /// Construct a worker pool with a specific reporter and max concurrency
  /// limit.
  WorkerPool(this._environment, this._reporter, [this._maxConcurrency = 4]);

  final Environment _environment;
  final WorkerPoolProgressReporter _reporter;
  final int _maxConcurrency;

  late Completer<bool> _runCompleter;
  bool _anyFailed = false;
  final Set<WorkerTask> _running = <WorkerTask>{};
  final Set<WorkerTask> _pending = <WorkerTask>{};
  final Set<WorkerTask> _finished = <WorkerTask>{};

  /// Run all tasks in the pool. Report progress via reporter.
  /// Returns 0 on success and non-zero on failure.
  Future<bool> run(Set<WorkerTask> tasks) async {
    _environment.logger.info('Running ${tasks.length}');
    _runCompleter = Completer<bool>();
    _reporter.onRun(tasks);
    _pending.addAll(tasks);
    _runQueue();
    return _runCompleter.future;
  }

  /// Returns the current set of pending tasks.
  UnmodifiableSetView<WorkerTask> get pending {
    return UnmodifiableSetView<WorkerTask>(_pending);
  }

  /// Returns the current set of running tasks.
  UnmodifiableSetView<WorkerTask> get running {
    return UnmodifiableSetView<WorkerTask>(_running);
  }

  /// Returns the current set of finished tasks.
  UnmodifiableSetView<WorkerTask> get finished {
    return UnmodifiableSetView<WorkerTask>(_finished);
  }

  void _runQueue() {
    if (_pending.isEmpty && _running.isEmpty) {
      _reporter.onFinish();
      // Nothing left to do or running.
      _runCompleter.complete(!_anyFailed);
      return;
    }
    while (_running.length < _maxConcurrency && _pending.isNotEmpty) {
      final WorkerTask task = _pending.elementAt(0);
      _pending.remove(task);
      _runTask(task);
    }
  }

  Future<void> _runTask(WorkerTask task) async {
    task._startTime = DateTime.now();
    final Future<bool> result = task.run();

    _running.add(task);
    _reporter.onTaskStart(this, task);

    Object? err;
    late final bool r;
    try {
      r = await result;
    } catch (e) {
      err = e;
      r = false;
    }
    _anyFailed = _anyFailed || !r;
    task._finishTime = DateTime.now();

    _running.remove(task);
    _finished.add(task);
    _reporter.onTaskDone(this, task, err);

    // Kick the queue again.
    _runQueue();
  }
}

/// WorkerPoolProgressReporter can be used to monitor worker pool progress.
abstract class WorkerPoolProgressReporter {
  /// Invoked when [WorkerPool.run] is invoked.
  void onRun(Set<WorkerTask> tasks);

  /// Invoked right before [WorkerPool.run] is returned from.
  void onFinish();

  /// Invoked right after a task has been started.
  void onTaskStart(WorkerPool pool, WorkerTask task);

  /// Invoked right after a task has finished.
  void onTaskDone(WorkerPool pool, WorkerTask task, [Object? err]);
}

/// Useful for tests.
class NoopWorkerPoolProgressReporter implements WorkerPoolProgressReporter {
  @override
  void onRun(Set<WorkerTask> tasks) {}

  @override
  void onFinish() {}

  @override
  void onTaskStart(WorkerPool pool, WorkerTask task) {}

  @override
  void onTaskDone(WorkerPool pool, WorkerTask task, [Object? err]) {}
}
