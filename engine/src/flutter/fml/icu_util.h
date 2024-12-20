// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_ICU_UTIL_H_
#define FLUTTER_FML_ICU_UTIL_H_

#include <string>

#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"

namespace fml {
namespace icu {

void InitializeICU(const std::string& icu_data_path = "");

void InitializeICUFromMapping(std::unique_ptr<Mapping> mapping);

}  // namespace icu
}  // namespace fml

#endif  // FLUTTER_FML_ICU_UTIL_H_
