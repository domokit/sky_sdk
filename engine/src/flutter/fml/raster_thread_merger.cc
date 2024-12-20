// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/raster_thread_merger.h"

#include <utility>

#include "flutter/fml/message_loop_impl.h"

namespace fml {

RasterThreadMerger::RasterThreadMerger(fml::TaskQueueId platform_queue_id,
                                       fml::TaskQueueId gpu_queue_id)
    : RasterThreadMerger(
          MakeRefCounted<SharedThreadMerger>(platform_queue_id, gpu_queue_id),
          platform_queue_id,
          gpu_queue_id) {}

RasterThreadMerger::RasterThreadMerger(
    fml::RefPtr<fml::SharedThreadMerger> shared_merger,
    fml::TaskQueueId platform_queue_id,
    fml::TaskQueueId gpu_queue_id)
    : platform_queue_id_(platform_queue_id),
      gpu_queue_id_(gpu_queue_id),
      shared_merger_(std::move(shared_merger)) {}

void RasterThreadMerger::SetMergeUnmergeCallback(const fml::closure& callback) {
  merge_unmerge_callback_ = callback;
}

const fml::RefPtr<fml::SharedThreadMerger>&
RasterThreadMerger::GetSharedRasterThreadMerger() const {
  return shared_merger_;
}

fml::RefPtr<fml::RasterThreadMerger>
RasterThreadMerger::CreateOrShareThreadMerger(
    const fml::RefPtr<fml::RasterThreadMerger>& parent_merger,
    TaskQueueId platform_id,
    TaskQueueId raster_id) {
  if (parent_merger && parent_merger->platform_queue_id_ == platform_id &&
      parent_merger->gpu_queue_id_ == raster_id) {
    auto shared_merger = parent_merger->GetSharedRasterThreadMerger();
    return fml::MakeRefCounted<RasterThreadMerger>(shared_merger, platform_id,
                                                   raster_id);
  } else {
    return fml::MakeRefCounted<RasterThreadMerger>(platform_id, raster_id);
  }
}

void RasterThreadMerger::MergeWithLease(size_t lease_term) {
  std::scoped_lock lock(mutex_);
  if (TaskQueuesAreSame()) {
    return;
  }
  if (!IsEnabledUnSafe()) {
    return;
  }
  FML_DCHECK(lease_term > 0) << "lease_term should be positive.";

  if (IsMergedUnSafe()) {
    merged_condition_.notify_one();
    return;
  }

  bool success = shared_merger_->MergeWithLease(this, lease_term);
  if (success && merge_unmerge_callback_ != nullptr) {
    merge_unmerge_callback_();
  }

  merged_condition_.notify_one();
}

void RasterThreadMerger::UnMergeNowIfLastOne() {
  std::scoped_lock lock(mutex_);

  if (TaskQueuesAreSame()) {
    return;
  }
  if (!IsEnabledUnSafe()) {
    return;
  }
  bool success = shared_merger_->UnMergeNowIfLastOne(this);
  if (success && merge_unmerge_callback_ != nullptr) {
    merge_unmerge_callback_();
  }
}

bool RasterThreadMerger::IsOnPlatformThread() const {
  return MessageLoop::GetCurrentTaskQueueId() == platform_queue_id_;
}

bool RasterThreadMerger::IsOnRasterizingThread() {
  std::scoped_lock lock(mutex_);

  if (IsMergedUnSafe()) {
    return IsOnPlatformThread();
  } else {
    return !IsOnPlatformThread();
  }
}

void RasterThreadMerger::ExtendLeaseTo(size_t lease_term) {
  FML_DCHECK(lease_term > 0) << "lease_term should be positive.";
  if (TaskQueuesAreSame()) {
    return;
  }
  std::scoped_lock lock(mutex_);
  if (!IsEnabledUnSafe()) {
    return;
  }
  shared_merger_->ExtendLeaseTo(this, lease_term);
}

bool RasterThreadMerger::IsMerged() {
  std::scoped_lock lock(mutex_);
  return IsMergedUnSafe();
}

void RasterThreadMerger::Enable() {
  std::scoped_lock lock(mutex_);
  shared_merger_->SetEnabledUnSafe(true);
}

void RasterThreadMerger::Disable() {
  std::scoped_lock lock(mutex_);
  shared_merger_->SetEnabledUnSafe(false);
}

bool RasterThreadMerger::IsEnabled() {
  std::scoped_lock lock(mutex_);
  return IsEnabledUnSafe();
}

bool RasterThreadMerger::IsEnabledUnSafe() const {
  return shared_merger_->IsEnabledUnSafe();
}

bool RasterThreadMerger::IsMergedUnSafe() const {
  return TaskQueuesAreSame() || shared_merger_->IsMergedUnSafe();
}

bool RasterThreadMerger::TaskQueuesAreSame() const {
  return platform_queue_id_ == gpu_queue_id_;
}

void RasterThreadMerger::WaitUntilMerged() {
  if (TaskQueuesAreSame()) {
    return;
  }
  FML_CHECK(IsOnPlatformThread());
  std::unique_lock<std::mutex> lock(mutex_);
  merged_condition_.wait(lock, [&] { return IsMergedUnSafe(); });
}

RasterThreadStatus RasterThreadMerger::DecrementLease() {
  if (TaskQueuesAreSame()) {
    return RasterThreadStatus::kRemainsMerged;
  }
  std::scoped_lock lock(mutex_);
  if (!IsMergedUnSafe()) {
    return RasterThreadStatus::kRemainsUnmerged;
  }
  if (!IsEnabledUnSafe()) {
    return RasterThreadStatus::kRemainsMerged;
  }
  bool unmerged_after_decrement = shared_merger_->DecrementLease(this);
  if (unmerged_after_decrement) {
    if (merge_unmerge_callback_ != nullptr) {
      merge_unmerge_callback_();
    }
    return RasterThreadStatus::kUnmergedNow;
  }

  return RasterThreadStatus::kRemainsMerged;
}

}  // namespace fml
