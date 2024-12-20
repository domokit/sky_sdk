// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/core/buffer_view.h"

namespace impeller {

BufferView::BufferView() : buffer_(nullptr), raw_buffer_(nullptr), range_({}) {}

BufferView::BufferView(DeviceBuffer* buffer, Range range)
    : buffer_(), raw_buffer_(buffer), range_(range) {}

BufferView::BufferView(std::shared_ptr<const DeviceBuffer> buffer, Range range)
    : buffer_(std::move(buffer)), raw_buffer_(nullptr), range_(range) {}

const DeviceBuffer* BufferView::GetBuffer() const {
  return raw_buffer_ ? raw_buffer_ : buffer_.get();
}

std::shared_ptr<const DeviceBuffer> BufferView::TakeBuffer() {
  if (buffer_) {
    raw_buffer_ = buffer_.get();
    return std::move(buffer_);
  } else {
    return nullptr;
  }
}

BufferView::operator bool() const {
  return buffer_ || raw_buffer_;
}

}  // namespace impeller
