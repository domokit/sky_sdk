// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TONIC_COMMON_LOG_H_
#define TONIC_COMMON_LOG_H_

#include <functional>

namespace tonic {

void Log(const char* format, ...);

void SetLogHandler(std::function<void(const char*)> handler);

}  // namespace tonic

#endif  // TONIC_COMMON_LOG_H_
