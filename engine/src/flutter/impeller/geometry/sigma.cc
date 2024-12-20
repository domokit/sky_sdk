// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/geometry/sigma.h"

#include <sstream>

namespace impeller {

Sigma::operator Radius() const {
  return Radius{sigma > 0.5f ? (sigma - 0.5f) * kKernelRadiusPerSigma : 0.0f};
}

Radius::operator Sigma() const {
  return Sigma{radius > 0 ? radius / kKernelRadiusPerSigma + 0.5f : 0.0f};
}

}  // namespace impeller
