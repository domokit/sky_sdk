// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_TASK_RUNNER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_TASK_RUNNER_H_

#include <mutex>
#include <unordered_map>

#include "flutter/fml/macros.h"
#include "flutter/fml/task_runner.h"

namespace flutter {

//------------------------------------------------------------------------------
/// A task runner which delegates responsibility of task execution to an
/// embedder. This is done by managing a dispatch table to the embedder.
///
class EmbedderTaskRunner final : public fml::TaskRunner {
 public:
  //----------------------------------------------------------------------------
  /// @brief      A
  ///
  struct DispatchTable {
    //--------------------------------------------------------------------------
    /// Delegates responsibility of deferred task execution to the embedder.
    /// Once the embedder gets the task, it must call
    /// `EmbedderTaskRunner::PostTask` with the supplied `task_baton` on the
    /// correct thread after the tasks `target_time` point expires.
    ///
    std::function<void(EmbedderTaskRunner* task_runner,
                       uint64_t task_baton,
                       fml::TimePoint target_time)>
        post_task_callback;
    //--------------------------------------------------------------------------
    /// Asks the embedder if tasks posted to it on this task runner via the
    /// `post_task_callback` will be executed (after task expiry) on the calling
    /// thread.
    ///
    std::function<bool(void)> runs_task_on_current_thread_callback;

    //--------------------------------------------------------------------------
    /// Performs user-designated cleanup on destruction.
    std::function<void()> destruction_callback;
  };

  //----------------------------------------------------------------------------
  /// @brief      Create a task runner with a dispatch table for delegation of
  ///             task runner responsibility to the embedder. When embedders
  ///             specify task runner dispatch tables that service tasks on the
  ///             same thread, they also must ensure that their
  ///             `embedder_idetifier`s match. This allows the engine to
  ///             determine task runner equality without actually posting tasks
  ///             to the task runner.
  ///
  /// @param[in]  table                The task runner dispatch table.
  /// @param[in]  embedder_identifier  The embedder identifier
  ///
  EmbedderTaskRunner(DispatchTable table, size_t embedder_identifier);

  // |fml::TaskRunner|
  ~EmbedderTaskRunner() override;

  //----------------------------------------------------------------------------
  /// @brief      The unique identifier provided by the embedder for the task
  ///             runner. Embedders whose dispatch tables service tasks on the
  ///             same underlying OS thread must ensure that their identifiers
  ///             match. This allows the engine to determine task runner
  ///             equality without posting tasks on the thread.
  ///
  /// @return     The embedder identifier.
  ///
  size_t GetEmbedderIdentifier() const;

  bool PostTask(uint64_t baton);

 private:
  const size_t embedder_identifier_;
  DispatchTable dispatch_table_;
  std::mutex tasks_mutex_;
  uint64_t last_baton_ = 0;
  std::unordered_map<uint64_t, fml::closure> pending_tasks_;
  fml::TaskQueueId placeholder_id_;

  // |fml::TaskRunner|
  void PostTask(const fml::closure& task) override;

  // |fml::TaskRunner|
  void PostTaskForTime(const fml::closure& task,
                       fml::TimePoint target_time) override;

  // |fml::TaskRunner|
  void PostDelayedTask(const fml::closure& task, fml::TimeDelta delay) override;

  // |fml::TaskRunner|
  bool RunsTasksOnCurrentThread() override;

  // |fml::TaskRunner|
  fml::TaskQueueId GetTaskQueueId() override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTaskRunner);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_TASK_RUNNER_H_
