// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_INCLUDE_DIR_H_
#define FLUTTER_IMPELLER_COMPILER_INCLUDE_DIR_H_

#include <memory>
#include <string>

#include "flutter/fml/unique_fd.h"

namespace impeller {
namespace compiler {

struct IncludeDir {
  std::shared_ptr<fml::UniqueFD> dir;
  std::string name;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_INCLUDE_DIR_H_
