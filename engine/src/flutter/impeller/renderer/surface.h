// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_SURFACE_H_
#define FLUTTER_IMPELLER_RENDERER_SURFACE_H_

#include "impeller/renderer/render_target.h"

namespace impeller {

class Surface {
 public:
  Surface();

  explicit Surface(const RenderTarget& target_desc);

  virtual ~Surface();

  const ISize& GetSize() const;

  bool IsValid() const;

  const RenderTarget& GetRenderTarget() const;

  virtual bool Present() const;

 private:
  RenderTarget desc_;
  ISize size_;

  bool is_valid_ = false;

  Surface(const Surface&) = delete;

  Surface& operator=(const Surface&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_SURFACE_H_
