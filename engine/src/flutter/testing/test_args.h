// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_ARGS_H_
#define FLUTTER_TESTING_TEST_ARGS_H_

#include "flutter/fml/command_line.h"

namespace flutter::testing {

const fml::CommandLine& GetArgsForProcess();

void SetArgsForProcess(int argc, char** argv);

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_ARGS_H_
