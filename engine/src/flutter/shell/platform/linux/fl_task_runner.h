// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TASK_RUNNER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TASK_RUNNER_H_

#include <glib-object.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlTaskRunner, fl_task_runner, FL, TASK_RUNNER, GObject);

/**
 * fl_task_runner_new:
 * @engine: the #FlEngine owning the task runner.
 *
 * Creates new task runner instance.
 *
 * Returns: an #FlTaskRunner.
 */
FlTaskRunner* fl_task_runner_new(FlEngine* engine);

/**
 * fl_task_runner_post_flutter_task:
 * @task_runner: an #FlTaskRunner.
 * @task: Flutter task being scheduled
 * @target_time_nanos: absolute time in nanoseconds
 *
 * Posts a Flutter task to be executed on main thread. This function is thread
 * safe and may be called from any thread.
 */
void fl_task_runner_post_flutter_task(FlTaskRunner* task_runner,
                                      FlutterTask task,
                                      uint64_t target_time_nanos);

/**
 * fl_task_runner_post_callback:
 * @task_runner: an #FlTaskRunner.
 * @callback: callback to be scheduled
 * @data: data to be passed to the callback
 *
 * Schedules arbitrary callback to be executed on main thread. The callback
 * will be executed in next run loop turn. This function is thread
 * safe and may be called from any thread.
 */
void fl_task_runner_post_callback(FlTaskRunner* task_runner,
                                  void (*callback)(gpointer data),
                                  gpointer data);

/**
 * fl_task_runner_block_main_thread:
 * @task_runner: an #FlTaskRunner.
 *
 * Blocks main thread until fl_task_runner_release_main_thread is called.
 * While main thread is blocked tasks posted to #FlTaskRunner are executed as
 * usual.
 * Must be invoked on main thread.
 */
void fl_task_runner_block_main_thread(FlTaskRunner* task_runner);

/**
 * fl_task_runner_release_main_thread:
 * @task_runner: an #FlTaskRunner.
 *
 * Unblocks main thread. This will resume normal processing of main loop.
 * Can be invoked from any thread.
 */
void fl_task_runner_release_main_thread(FlTaskRunner* self);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TASK_RUNNER_H_
