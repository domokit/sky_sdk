// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_EMBEDDER_SNAPSHOT_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_EMBEDDER_SNAPSHOT_H_

#include <cstdint>

namespace dart_runner {

extern uint8_t const* const vm_isolate_snapshot_buffer;
extern uint8_t const* const isolate_snapshot_buffer;

}  // namespace dart_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_EMBEDDER_SNAPSHOT_H_
