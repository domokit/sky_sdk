// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_SURFACE_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_SURFACE_H_

#include <memory>

#include "impeller/renderer/surface.h"
#include "impeller/toolkit/interop/context.h"
#include "impeller/toolkit/interop/dl.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class Surface final
    : public Object<Surface, IMPELLER_INTERNAL_HANDLE_NAME(ImpellerSurface)> {
 public:
  static ScopedObject<Surface> WrapFBO(Context& context,
                                       uint64_t fbo,
                                       PixelFormat color_format,
                                       ISize size);

  explicit Surface(Context& context,
                   std::shared_ptr<impeller::Surface> surface);

  ~Surface() override;

  Surface(const Surface&) = delete;

  Surface& operator=(const Surface&) = delete;

  bool IsValid() const;

  bool DrawDisplayList(const DisplayList& dl) const;

 private:
  ScopedObject<Context> context_;
  std::shared_ptr<impeller::Surface> surface_;
  bool is_valid_ = false;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_SURFACE_H_
