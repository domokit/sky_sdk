// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_SNAPSHOT_SNAPSHOT_H_
#define FLUTTER_LIB_SNAPSHOT_SNAPSHOT_H_

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

extern "C" {
extern const uint8_t kDartVmSnapshotData[];
extern const uint8_t kDartVmSnapshotInstructions[];
extern const uint8_t kDartIsolateSnapshotData[];
extern const uint8_t kDartIsolateSnapshotInstructions[];
}

#endif  // FLUTTER_LIB_SNAPSHOT_SNAPSHOT_H_
