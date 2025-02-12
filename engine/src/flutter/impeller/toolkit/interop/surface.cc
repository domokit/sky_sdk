// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/surface.h"

#include "impeller/base/validation.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/toolkit/interop/formats.h"

namespace impeller::interop {

Surface::Surface(Context& context, std::shared_ptr<impeller::Surface> surface)
    : context_(Ref(&context)), surface_(std::move(surface)) {
  is_valid_ =
      context_ && context_->IsValid() && surface_ && surface_->IsValid();
}

Surface::~Surface() = default;

bool Surface::IsValid() const {
  return is_valid_;
}

bool Surface::DrawDisplayList(const DisplayList& dl) const {
  if (!IsValid() || !dl.IsValid()) {
    return false;
  }

  auto display_list = dl.GetDisplayList();
  auto& content_context = context_->GetAiksContext().GetContentContext();
  auto render_target = surface_->GetRenderTarget();

  const auto cull_rect = IRect::MakeSize(surface_->GetSize());
  auto skia_cull_rect =
      SkIRect::MakeWH(cull_rect.GetWidth(), cull_rect.GetHeight());

  auto result = RenderToTarget(content_context, render_target, display_list,
                               skia_cull_rect, /*reset_host_buffer=*/true);
  context_->GetContext()->ResetThreadLocalState();
  return result;
}

bool Surface::Present() const {
  if (!IsValid()) {
    return false;
  }
  return surface_->Present();
}

}  // namespace impeller::interop
