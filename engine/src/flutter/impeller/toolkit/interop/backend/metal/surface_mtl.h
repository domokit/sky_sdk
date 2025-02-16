// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_METAL_SURFACE_MTL_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_METAL_SURFACE_MTL_H_

#include "impeller/toolkit/interop/surface.h"

namespace impeller::interop {

class SurfaceMTL final : public Surface {
 public:
  SurfaceMTL(Context& context, void* metal_drawable);

  SurfaceMTL(Context& context, std::shared_ptr<impeller::Surface> surface);

  ~SurfaceMTL();

  SurfaceMTL(const SurfaceMTL&) = delete;

  SurfaceMTL& operator=(const SurfaceMTL&) = delete;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_METAL_SURFACE_MTL_H_
