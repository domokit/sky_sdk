// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_task_runner.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"

static constexpr int kMicrosecondsPerNanosecond = 1000;
static constexpr int kMillisecondsPerMicrosecond = 1000;

struct _FlTaskRunner {
  GObject parent_instance;

  GWeakRef engine;

  GMutex mutex;
  GCond cond;

  guint timeout_source_id;
  GList /*<FlTaskRunnerTask>*/* pending_tasks;
  gboolean blocking_main_thread;
};

typedef struct _FlTaskRunnerTask {
  // absolute time of task (based on g_get_monotonic_time)
  gint64 task_time_micros;
  FlutterTask task;
} FlTaskRunnerTask;

G_DEFINE_TYPE(FlTaskRunner, fl_task_runner, G_TYPE_OBJECT)

// Removes expired tasks from the task queue and executes them.
// The execution is performed with mutex unlocked.
static void fl_task_runner_process_expired_tasks_locked(FlTaskRunner* self) {
  GList* expired_tasks = nullptr;

  gint64 current_time = g_get_monotonic_time();

  GList* l = self->pending_tasks;
  while (l != nullptr) {
    FlTaskRunnerTask* task = static_cast<FlTaskRunnerTask*>(l->data);
    if (task->task_time_micros <= current_time) {
      GList* link = l;
      l = l->next;
      self->pending_tasks = g_list_remove_link(self->pending_tasks, link);
      expired_tasks = g_list_concat(expired_tasks, link);
    } else {
      l = l->next;
    }
  }

  g_mutex_unlock(&self->mutex);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine != nullptr) {
    l = expired_tasks;
    while (l != nullptr) {
      FlTaskRunnerTask* task = static_cast<FlTaskRunnerTask*>(l->data);
      fl_engine_execute_task(engine, &task->task);
      l = l->next;
    }
  }

  g_list_free_full(expired_tasks, g_free);

  g_mutex_lock(&self->mutex);
}

static void fl_task_runner_tasks_did_change_locked(FlTaskRunner* self);

// Invoked from a timeout source. Removes and executes expired tasks
// and reschedules timeout if needed.
static gboolean fl_task_runner_on_expired_timeout(gpointer data) {
  FlTaskRunner* self = FL_TASK_RUNNER(data);

  g_autoptr(GMutexLocker) locker = g_mutex_locker_new(&self->mutex);
  (void)locker;  // unused variable

  g_object_ref(self);

  self->timeout_source_id = 0;
  fl_task_runner_process_expired_tasks_locked(self);

  // reschedule timeout
  fl_task_runner_tasks_did_change_locked(self);

  g_object_unref(self);

  return FALSE;
}

// Returns the absolute time of next expired task (in microseconds, based on
// g_get_monotonic_time). If no task is scheduled returns G_MAXINT64.
static gint64 fl_task_runner_next_task_expiration_time_locked(
    FlTaskRunner* self) {
  gint64 min_time = G_MAXINT64;
  GList* l = self->pending_tasks;
  while (l != nullptr) {
    FlTaskRunnerTask* task = static_cast<FlTaskRunnerTask*>(l->data);
    min_time = MIN(min_time, task->task_time_micros);
    l = l->next;
  }
  return min_time;
}

static void fl_task_runner_tasks_did_change_locked(FlTaskRunner* self) {
  if (self->blocking_main_thread) {
    // Wake up blocked thread
    g_cond_signal(&self->cond);
  } else {
    // Reschedule timeout
    if (self->timeout_source_id != 0) {
      g_source_remove(self->timeout_source_id);
      self->timeout_source_id = 0;
    }
    gint64 min_time = fl_task_runner_next_task_expiration_time_locked(self);
    if (min_time != G_MAXINT64) {
      gint64 remaining = MAX(min_time - g_get_monotonic_time(), 0);
      self->timeout_source_id =
          g_timeout_add(remaining / kMillisecondsPerMicrosecond + 1,
                        fl_task_runner_on_expired_timeout, self);
    }
  }
}

void fl_task_runner_dispose(GObject* object) {
  FlTaskRunner* self = FL_TASK_RUNNER(object);

  // this should never happen because the task runner is retained while blocking
  // main thread
  g_assert(!self->blocking_main_thread);

  g_weak_ref_clear(&self->engine);
  g_mutex_clear(&self->mutex);
  g_cond_clear(&self->cond);

  g_list_free_full(self->pending_tasks, g_free);
  if (self->timeout_source_id != 0) {
    g_source_remove(self->timeout_source_id);
  }

  G_OBJECT_CLASS(fl_task_runner_parent_class)->dispose(object);
}

static void fl_task_runner_class_init(FlTaskRunnerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_task_runner_dispose;
}

static void fl_task_runner_init(FlTaskRunner* self) {
  g_mutex_init(&self->mutex);
  g_cond_init(&self->cond);
}

FlTaskRunner* fl_task_runner_new(FlEngine* engine) {
  FlTaskRunner* self =
      FL_TASK_RUNNER(g_object_new(fl_task_runner_get_type(), nullptr));
  g_weak_ref_init(&self->engine, G_OBJECT(engine));
  return self;
}

void fl_task_runner_post_task(FlTaskRunner* self,
                              FlutterTask task,
                              uint64_t target_time_nanos) {
  g_autoptr(GMutexLocker) locker = g_mutex_locker_new(&self->mutex);
  (void)locker;  // unused variable

  FlTaskRunnerTask* runner_task = g_new0(FlTaskRunnerTask, 1);
  runner_task->task = task;
  runner_task->task_time_micros =
      target_time_nanos / kMicrosecondsPerNanosecond;

  self->pending_tasks = g_list_append(self->pending_tasks, runner_task);
  fl_task_runner_tasks_did_change_locked(self);
}

void fl_task_runner_block_main_thread(FlTaskRunner* self) {
  g_autoptr(GMutexLocker) locker = g_mutex_locker_new(&self->mutex);
  (void)locker;  // unused variable

  g_return_if_fail(self->blocking_main_thread == FALSE);

  g_object_ref(self);

  self->blocking_main_thread = true;
  while (self->blocking_main_thread) {
    g_cond_wait_until(&self->cond, &self->mutex,
                      fl_task_runner_next_task_expiration_time_locked(self));
    fl_task_runner_process_expired_tasks_locked(self);
  }

  // Tasks might have changed in the meanwhile, reschedule timeout
  fl_task_runner_tasks_did_change_locked(self);

  g_object_unref(self);
}

void fl_task_runner_release_main_thread(FlTaskRunner* self) {
  g_autoptr(GMutexLocker) locker = g_mutex_locker_new(&self->mutex);
  (void)locker;  // unused variable

  g_return_if_fail(self->blocking_main_thread == TRUE);

  self->blocking_main_thread = FALSE;
  g_cond_signal(&self->cond);
}
