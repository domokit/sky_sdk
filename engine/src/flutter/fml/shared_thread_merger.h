// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_SHARED_THREAD_MERGER_H_
#define FLUTTER_FML_SHARED_THREAD_MERGER_H_

#include <condition_variable>
#include <mutex>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/message_loop_task_queues.h"

namespace fml {

class RasterThreadMerger;

typedef void* RasterThreadMergerId;

/// Instance of this class is shared between multiple |RasterThreadMerger|
/// instances, Most of the callings from |RasterThreadMerger| will be redirected
/// to this class with one more caller parameter.
class SharedThreadMerger
    : public fml::RefCountedThreadSafe<SharedThreadMerger> {
 public:
  SharedThreadMerger(TaskQueueId owner, TaskQueueId subsumed);

  // It's called by |RasterThreadMerger::MergeWithLease|.
  // See the doc of |RasterThreadMerger::MergeWithLease|.
  bool MergeWithLease(RasterThreadMergerId caller, size_t lease_term);

  // It's called by |RasterThreadMerger::UnMergeNowIfLastOne|.
  // See the doc of |RasterThreadMerger::UnMergeNowIfLastOne|.
  bool UnMergeNowIfLastOne(RasterThreadMergerId caller);

  // It's called by |RasterThreadMerger::ExtendLeaseTo|.
  // See the doc of |RasterThreadMerger::ExtendLeaseTo|.
  void ExtendLeaseTo(RasterThreadMergerId caller, size_t lease_term);

  // It's called by |RasterThreadMerger::IsMergedUnSafe|.
  // See the doc of |RasterThreadMerger::IsMergedUnSafe|.
  bool IsMergedUnSafe() const;

  // It's called by |RasterThreadMerger::IsEnabledUnSafe|.
  // See the doc of |RasterThreadMerger::IsEnabledUnSafe|.
  bool IsEnabledUnSafe() const;

  // It's called by |RasterThreadMerger::Enable| or
  // |RasterThreadMerger::Disable|.
  void SetEnabledUnSafe(bool enabled);

  // It's called by |RasterThreadMerger::DecrementLease|.
  // See the doc of |RasterThreadMerger::DecrementLease|.
  bool DecrementLease(RasterThreadMergerId caller);

 private:
  fml::TaskQueueId owner_;
  fml::TaskQueueId subsumed_;
  fml::MessageLoopTaskQueues* task_queues_;
  std::mutex mutex_;
  bool enabled_ = false;

  /// The |MergeWithLease| or |ExtendLeaseTo| method will record the caller
  /// into this lease_term_by_caller_ map, |UnMergeNowIfLastOne|
  /// method will remove the caller from this lease_term_by_caller_.
  std::map<RasterThreadMergerId, std::atomic_size_t> lease_term_by_caller_;

  bool IsAllLeaseTermsZeroUnSafe() const;

  bool UnMergeNowUnSafe();

  FML_DISALLOW_COPY_AND_ASSIGN(SharedThreadMerger);
};

}  // namespace fml

#endif  // FLUTTER_FML_SHARED_THREAD_MERGER_H_
