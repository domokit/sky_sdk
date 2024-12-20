// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_BACKTRACE_H_
#define FLUTTER_FML_BACKTRACE_H_

#include <string>

#include "flutter/fml/macros.h"

namespace fml {

// Retrieve the backtrace, for debugging.
//
// If the |offset| is 0, the backtrace is included caller function.
std::string BacktraceHere(size_t offset = 0);

void InstallCrashHandler();

bool IsCrashHandlingSupported();

}  // namespace fml

#endif  // FLUTTER_FML_BACKTRACE_H_
