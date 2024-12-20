// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface/snapshot_surface_producer.h"

namespace flutter {

AndroidSnapshotSurfaceProducer::AndroidSnapshotSurfaceProducer(
    AndroidSurface& android_surface)
    : android_surface_(android_surface) {}

std::unique_ptr<Surface>
AndroidSnapshotSurfaceProducer::CreateSnapshotSurface() {
  return android_surface_.CreateSnapshotSurface();
}

}  // namespace flutter
