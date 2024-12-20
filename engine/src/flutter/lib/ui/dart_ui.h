// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_DART_UI_H_
#define FLUTTER_LIB_UI_DART_UI_H_

#include "flutter/common/settings.h"
#include "flutter/fml/macros.h"

namespace flutter {

class DartUI {
 public:
  static void InitForIsolate(const Settings& settings);

 private:
  FML_DISALLOW_IMPLICIT_CONSTRUCTORS(DartUI);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_DART_UI_H_
