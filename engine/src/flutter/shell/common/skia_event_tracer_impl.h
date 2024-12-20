// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SKIA_EVENT_TRACER_IMPL_H_
#define FLUTTER_SHELL_COMMON_SKIA_EVENT_TRACER_IMPL_H_

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace flutter {

void InitSkiaEventTracer(
    bool enabled,
    const std::optional<std::vector<std::string>>& allowlist);

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SKIA_EVENT_TRACER_IMPL_H_
