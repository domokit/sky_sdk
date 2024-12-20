// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_surface_provider.h"

#include "flutter/display_list/testing/dl_test_surface_software.h"

namespace flutter::testing {

std::unique_ptr<DlSurfaceProvider> DlSurfaceProvider::CreateSoftware() {
  return std::make_unique<DlSoftwareSurfaceProvider>();
}

}  // namespace flutter::testing
