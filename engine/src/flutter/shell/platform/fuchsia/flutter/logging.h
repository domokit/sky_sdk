// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_LOGGING_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_LOGGING_H_

#include "flutter/fml/logging.h"

namespace flutter_runner {

// Use to mark logs published via the syslog API.
#define LOG_TAG "flutter-runner"

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_LOGGING_H_
