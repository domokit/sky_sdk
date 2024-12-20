// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_RASTER_THREAD_MERGER_H_
#define FLUTTER_FML_RASTER_THREAD_MERGER_H_

#include <condition_variable>
#include <mutex>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/message_loop_task_queues.h"
#include "flutter/fml/shared_thread_merger.h"

namespace fml {

class MessageLoopImpl;

enum class RasterThreadStatus {
  kRemainsMerged,
  kRemainsUnmerged,
  kUnmergedNow
};

/// This class is a client and proxy between the rasterizer and
/// |SharedThreadMerger|. The multiple |RasterThreadMerger| instances with same
/// owner_queue_id and same subsumed_queue_id share the same
/// |SharedThreadMerger| instance. Whether they share the same inner instance is
/// determined by |RasterThreadMerger::CreateOrShareThreadMerger| method.
class RasterThreadMerger
    : public fml::RefCountedThreadSafe<RasterThreadMerger> {
 public:
  // Merges the raster thread into platform thread for the duration of
  // the lease term. Lease is managed by the caller by either calling
  // |ExtendLeaseTo| or |DecrementLease|.
  // When the caller merges with a lease term of say 2. The threads
  // are going to remain merged until 2 invocations of |DecreaseLease|,
  // unless an |ExtendLeaseTo| gets called.
  //
  // If the task queues are the same, we consider them statically merged.
  // When task queues are statically merged this method becomes no-op.
  void MergeWithLease(size_t lease_term);

  // Gets the shared merger from current merger object
  const fml::RefPtr<SharedThreadMerger>& GetSharedRasterThreadMerger() const;

  /// Creates a new merger from parent, share the inside shared_merger member
  /// when the platform_queue_id and raster_queue_id are same, otherwise create
  /// a new shared_merger instance
  static fml::RefPtr<fml::RasterThreadMerger> CreateOrShareThreadMerger(
      const fml::RefPtr<fml::RasterThreadMerger>& parent_merger,
      TaskQueueId platform_id,
      TaskQueueId raster_id);

  // Un-merges the threads now if current caller is the last merged caller,
  // and it resets the lease term to 0, otherwise it will remove
  // the caller record and return. The multiple caller records were recorded
  // after |MergeWithLease| or |ExtendLeaseTo| method.
  //
  // Must be executed on the raster task runner.
  //
  // If the task queues are the same, we consider them statically merged.
  // When task queues are statically merged, we never unmerge them and
  // this method becomes no-op.
  void UnMergeNowIfLastOne();

  // If the task queues are the same, we consider them statically merged.
  // When task queues are statically merged this method becomes no-op.
  void ExtendLeaseTo(size_t lease_term);

  // Returns |RasterThreadStatus::kUnmergedNow| if this call resulted in
  // splitting the raster and platform threads. Reduces the lease term by 1.
  //
  // If the task queues are the same, we consider them statically merged.
  // When task queues are statically merged this method becomes no-op.
  RasterThreadStatus DecrementLease();

  // The method is locked by current instance, and asks the shared instance of
  // SharedThreadMerger and the merging state is determined by the
  // lease_term_ counter.
  bool IsMerged();

  // Waits until the threads are merged.
  //
  // Must run on the platform task runner.
  void WaitUntilMerged();

  // Returns true if the current thread owns rasterizing.
  // When the threads are merged, platform thread owns rasterizing.
  // When un-merged, raster thread owns rasterizing.
  bool IsOnRasterizingThread();

  // Returns true if the current thread is the platform thread.
  bool IsOnPlatformThread() const;

  // Enables the thread merger.
  void Enable();

  // Disables the thread merger. Once disabled, any call to
  // |MergeWithLease| or |UnMergeNowIfLastOne| results in a noop.
  void Disable();

  // Whether the thread merger is enabled. By default, the thread merger is
  // enabled. If false, calls to |MergeWithLease| or |UnMergeNowIfLastOne|
  // or |ExtendLeaseTo| or |DecrementLease| results in a noop.
  bool IsEnabled();

  // Registers a callback that can be used to clean up global state right after
  // the thread configuration has changed.
  //
  // For example, it can be used to clear the GL context so it can be used in
  // the next task from a different thread.
  void SetMergeUnmergeCallback(const fml::closure& callback);

 private:
  fml::TaskQueueId platform_queue_id_;
  fml::TaskQueueId gpu_queue_id_;

  RasterThreadMerger(fml::TaskQueueId platform_queue_id,
                     fml::TaskQueueId gpu_queue_id);
  RasterThreadMerger(fml::RefPtr<fml::SharedThreadMerger> shared_merger,
                     fml::TaskQueueId platform_queue_id,
                     fml::TaskQueueId gpu_queue_id);

  const fml::RefPtr<fml::SharedThreadMerger> shared_merger_;
  std::condition_variable merged_condition_;
  std::mutex mutex_;
  fml::closure merge_unmerge_callback_;

  bool IsMergedUnSafe() const;

  bool IsEnabledUnSafe() const;

  // The platform_queue_id and gpu_queue_id are exactly the same.
  // We consider the threads are always merged and cannot be unmerged.
  bool TaskQueuesAreSame() const;

  FML_FRIEND_REF_COUNTED_THREAD_SAFE(RasterThreadMerger);
  FML_FRIEND_MAKE_REF_COUNTED(RasterThreadMerger);
  FML_DISALLOW_COPY_AND_ASSIGN(RasterThreadMerger);
};

}  // namespace fml

#endif  // FLUTTER_FML_RASTER_THREAD_MERGER_H_
