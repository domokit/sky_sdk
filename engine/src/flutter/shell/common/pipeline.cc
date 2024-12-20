// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/pipeline.h"

namespace flutter {

size_t GetNextPipelineTraceID() {
  static std::atomic_size_t PipelineLastTraceID = {0};
  return ++PipelineLastTraceID;
}

}  // namespace flutter
