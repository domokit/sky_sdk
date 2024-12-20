// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/stream_capture.h"

namespace flutter::testing {

StreamCapture::StreamCapture(std::ostream* ostream)
    : ostream_(ostream), old_buffer_(ostream_->rdbuf()) {
  ostream_->rdbuf(buffer_.rdbuf());
}

StreamCapture::~StreamCapture() {
  Stop();
}

void StreamCapture::Stop() {
  if (old_buffer_) {
    ostream_->rdbuf(old_buffer_);
    old_buffer_ = nullptr;
  }
}

std::string StreamCapture::GetOutput() const {
  return buffer_.str();
}

}  // namespace flutter::testing
