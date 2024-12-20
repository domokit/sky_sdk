// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TASK_RUNNER_ADAPTER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TASK_RUNNER_ADAPTER_H_

#include <lib/async/dispatcher.h>

#include "flutter/fml/task_runner.h"

namespace flutter_runner {

fml::RefPtr<fml::TaskRunner> CreateFMLTaskRunner(
    async_dispatcher_t* dispatcher);

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TASK_RUNNER_ADAPTER_H_
