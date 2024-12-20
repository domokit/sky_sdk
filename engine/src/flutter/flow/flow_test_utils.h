// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_FLOW_TEST_UTILS_H_
#define FLUTTER_FLOW_FLOW_TEST_UTILS_H_

#include <string>

namespace flutter {

const std::string& GetGoldenDir();

void SetGoldenDir(const std::string& dir);

const std::string& GetFontFile();

void SetFontFile(const std::string& dir);

}  // namespace flutter

#endif  // FLUTTER_FLOW_FLOW_TEST_UTILS_H_
