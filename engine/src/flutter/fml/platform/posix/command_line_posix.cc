// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/command_line.h"

namespace fml {

std::optional<CommandLine> CommandLineFromPlatform() {
  return std::nullopt;
}

}  // namespace fml
