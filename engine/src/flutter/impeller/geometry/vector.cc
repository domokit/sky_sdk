// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vector.h"
#include <sstream>

namespace impeller {

std::string Vector3::ToString() const {
  std::stringstream stream;
  stream << "{" << x << ", " << y << ", " << z << "}";
  return stream.str();
}

std::string Vector4::ToString() const {
  std::stringstream stream;
  stream << "{" << x << ", " << y << ", " << z << ", " << w << "}";
  return stream.str();
}

}  // namespace impeller
