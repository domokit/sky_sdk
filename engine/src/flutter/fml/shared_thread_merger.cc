// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/shared_thread_merger.h"

#include <algorithm>
#include <set>

namespace fml {

SharedThreadMerger::SharedThreadMerger(fml::TaskQueueId owner,
                                       fml::TaskQueueId subsumed)
    : owner_(owner),
      subsumed_(subsumed),
      task_queues_(fml::MessageLoopTaskQueues::GetInstance()),
      enabled_(true) {}

bool SharedThreadMerger::MergeWithLease(RasterThreadMergerId caller,
                                        size_t lease_term) {
  FML_DCHECK(lease_term > 0) << "lease_term should be positive.";
  std::scoped_lock lock(mutex_);
  if (IsMergedUnSafe()) {
    return true;
  }
  bool success = task_queues_->Merge(owner_, subsumed_);
  FML_CHECK(success) << "Unable to merge the raster and platform threads.";
  // Save the lease term
  lease_term_by_caller_[caller] = lease_term;
  return success;
}

bool SharedThreadMerger::UnMergeNowUnSafe() {
  FML_CHECK(IsAllLeaseTermsZeroUnSafe())
      << "all lease term records must be zero before calling "
         "UnMergeNowUnSafe()";
  bool success = task_queues_->Unmerge(owner_, subsumed_);
  FML_CHECK(success) << "Unable to un-merge the raster and platform threads.";
  return success;
}

bool SharedThreadMerger::UnMergeNowIfLastOne(RasterThreadMergerId caller) {
  std::scoped_lock lock(mutex_);
  lease_term_by_caller_.erase(caller);
  if (lease_term_by_caller_.empty() || IsAllLeaseTermsZeroUnSafe()) {
    return UnMergeNowUnSafe();
  }
  return true;
}

bool SharedThreadMerger::DecrementLease(RasterThreadMergerId caller) {
  std::scoped_lock lock(mutex_);
  auto entry = lease_term_by_caller_.find(caller);
  bool exist = entry != lease_term_by_caller_.end();
  if (exist) {
    std::atomic_size_t& lease_term_ref = entry->second;
    FML_CHECK(lease_term_ref > 0)
        << "lease_term should always be positive when merged, lease_term="
        << lease_term_ref;
    lease_term_ref--;
  } else {
    FML_LOG(WARNING) << "The caller does not exist when calling "
                        "DecrementLease(), ignored. This may happens after "
                        "caller is erased in UnMergeNowIfLastOne(). caller="
                     << caller;
  }
  if (IsAllLeaseTermsZeroUnSafe()) {
    // Unmerge now because lease_term_ decreased to zero.
    UnMergeNowUnSafe();
    return true;
  }
  return false;
}

void SharedThreadMerger::ExtendLeaseTo(RasterThreadMergerId caller,
                                       size_t lease_term) {
  FML_DCHECK(lease_term > 0) << "lease_term should be positive.";
  std::scoped_lock lock(mutex_);
  FML_DCHECK(IsMergedUnSafe())
      << "should be merged state when calling this method";
  lease_term_by_caller_[caller] = lease_term;
}

bool SharedThreadMerger::IsMergedUnSafe() const {
  return !IsAllLeaseTermsZeroUnSafe();
}

bool SharedThreadMerger::IsEnabledUnSafe() const {
  return enabled_;
}

void SharedThreadMerger::SetEnabledUnSafe(bool enabled) {
  enabled_ = enabled;
}

bool SharedThreadMerger::IsAllLeaseTermsZeroUnSafe() const {
  return std::all_of(lease_term_by_caller_.begin(), lease_term_by_caller_.end(),
                     [&](const auto& item) { return item.second == 0; });
}

}  // namespace fml
