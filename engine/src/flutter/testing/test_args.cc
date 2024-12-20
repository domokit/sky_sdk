// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_args.h"

namespace flutter::testing {

static fml::CommandLine gProcessArgs;

const fml::CommandLine& GetArgsForProcess() {
  return gProcessArgs;
}

void SetArgsForProcess(int argc, char** argv) {
  gProcessArgs = fml::CommandLineFromArgcArgv(argc, argv);
}

}  // namespace flutter::testing
