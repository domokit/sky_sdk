// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_POST_TASK_SYNC_H_
#define FLUTTER_TESTING_POST_TASK_SYNC_H_

#include "flutter/fml/task_runner.h"

namespace flutter::testing {

void PostTaskSync(const fml::RefPtr<fml::TaskRunner>& task_runner,
                  const std::function<void()>& function);

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_POST_TASK_SYNC_H_
