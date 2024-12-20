// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_storage.h"

namespace flutter {

static constexpr inline bool is_power_of_two(int value) {
  return (value & (value - 1)) == 0;
}

void DisplayListStorage::realloc(size_t count) {
  ptr_.reset(static_cast<uint8_t*>(std::realloc(ptr_.release(), count)));
  FML_CHECK(ptr_);
  allocated_ = count;
}

uint8_t* DisplayListStorage::allocate(size_t needed) {
  if (used_ + needed > allocated_) {
    static_assert(is_power_of_two(kDLPageSize),
                  "This math needs updating for non-pow2.");
    // Next greater multiple of DL_BUILDER_PAGE.
    size_t new_size = (used_ + needed + kDLPageSize) & ~(kDLPageSize - 1);
    size_t old_size = allocated_;
    realloc(new_size);
    FML_CHECK(ptr_.get());
    FML_CHECK(allocated_ == new_size);
    FML_CHECK(allocated_ >= old_size);
    FML_CHECK(used_ + needed <= allocated_);
    memset(ptr_.get() + used_, 0, allocated_ - old_size);
  }
  uint8_t* ret = ptr_.get() + used_;
  used_ += needed;
  FML_CHECK(used_ <= allocated_);
  return ret;
}

DisplayListStorage::DisplayListStorage(DisplayListStorage&& source) {
  ptr_ = std::move(source.ptr_);
  used_ = source.used_;
  allocated_ = source.allocated_;
  source.used_ = 0u;
  source.allocated_ = 0u;
}

void DisplayListStorage::reset() {
  ptr_.reset();
  used_ = 0u;
  allocated_ = 0u;
}

DisplayListStorage& DisplayListStorage::operator=(DisplayListStorage&& source) {
  ptr_ = std::move(source.ptr_);
  used_ = source.used_;
  allocated_ = source.allocated_;
  source.used_ = 0u;
  source.allocated_ = 0u;
  return *this;
}

}  // namespace flutter
