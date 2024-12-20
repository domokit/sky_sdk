// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/common/settings.h"

#include <sstream>

namespace flutter {

constexpr FrameTiming::Phase FrameTiming::kPhases[FrameTiming::kCount];

Settings::Settings() = default;

Settings::Settings(const Settings& other) = default;

Settings::~Settings() = default;

}  // namespace flutter
