// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_ANDROID_CPU_AFFINITY_H_
#define FLUTTER_FML_PLATFORM_ANDROID_CPU_AFFINITY_H_

#include "flutter/fml/cpu_affinity.h"

namespace fml {

/// @brief Android specific implementation of EfficiencyCoreCount.
std::optional<size_t> AndroidEfficiencyCoreCount();

/// @brief Android specific implementation of RequestAffinity.
bool AndroidRequestAffinity(CpuAffinity affinity);

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_ANDROID_CPU_AFFINITY_H_
