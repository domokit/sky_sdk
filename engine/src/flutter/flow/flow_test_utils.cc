// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <string>

namespace flutter {

static std::string gGoldenDir;
static std::string gFontFile;

const std::string& GetGoldenDir() {
  return gGoldenDir;
}

void SetGoldenDir(const std::string& dir) {
  gGoldenDir = dir;
}

const std::string& GetFontFile() {
  return gFontFile;
}

void SetFontFile(const std::string& file) {
  gFontFile = file;
}

}  // namespace flutter
