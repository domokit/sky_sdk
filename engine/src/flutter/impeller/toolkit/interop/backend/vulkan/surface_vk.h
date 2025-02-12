// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SURFACE_VK_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SURFACE_VK_H_

#include "impeller/toolkit/interop/surface.h"

namespace impeller::interop {

class SurfaceVK final : public Surface {
 public:
  SurfaceVK(Context& context, std::shared_ptr<impeller::Surface> surface);

  ~SurfaceVK();

  SurfaceVK(const SurfaceVK&) = delete;

  SurfaceVK& operator=(const SurfaceVK&) = delete;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SURFACE_VK_H_
