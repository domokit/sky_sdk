// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SNAPSHOT_SURFACE_PRODUCER_H_
#define FLUTTER_SHELL_COMMON_SNAPSHOT_SURFACE_PRODUCER_H_

#include <memory>

#include "flutter/flow/surface.h"

namespace flutter {

class SnapshotSurfaceProducer {
 public:
  virtual ~SnapshotSurfaceProducer() = default;

  virtual std::unique_ptr<Surface> CreateSnapshotSurface() = 0;
};

}  // namespace flutter
#endif  // FLUTTER_SHELL_COMMON_SNAPSHOT_SURFACE_PRODUCER_H_
