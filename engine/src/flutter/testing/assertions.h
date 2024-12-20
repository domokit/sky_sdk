// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_ASSERTIONS_H_
#define FLUTTER_TESTING_ASSERTIONS_H_

#include <type_traits>

namespace flutter::testing {

inline bool NumberNear(double a, double b) {
  static const double epsilon = 1e-3;
  return (a > (b - epsilon)) && (a < (b + epsilon));
}

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_ASSERTIONS_H_
