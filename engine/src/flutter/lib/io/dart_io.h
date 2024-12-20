// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_IO_DART_IO_H_
#define FLUTTER_LIB_IO_DART_IO_H_

#include <cstdint>
#include <string>

#include "flutter/fml/macros.h"

namespace flutter {

class DartIO {
 public:
  static void InitForIsolate(bool may_insecurely_connect_to_all_domains,
                             const std::string& domain_network_policy);

 private:
  FML_DISALLOW_IMPLICIT_CONSTRUCTORS(DartIO);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_IO_DART_IO_H_
