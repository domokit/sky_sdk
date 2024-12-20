// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/anonymous_contents.h"

#include <memory>

namespace impeller {

std::shared_ptr<Contents> AnonymousContents::Make(RenderProc render_proc,
                                                  CoverageProc coverage_proc) {
  return std::shared_ptr<Contents>(
      new AnonymousContents(std::move(render_proc), std::move(coverage_proc)));
}

AnonymousContents::AnonymousContents(RenderProc render_proc,
                                     CoverageProc coverage_proc)
    : render_proc_(std::move(render_proc)),
      coverage_proc_(std::move(coverage_proc)) {}

AnonymousContents::~AnonymousContents() = default;

bool AnonymousContents::Render(const ContentContext& renderer,
                               const Entity& entity,
                               RenderPass& pass) const {
  return render_proc_(renderer, entity, pass);
}

std::optional<Rect> AnonymousContents::GetCoverage(const Entity& entity) const {
  return coverage_proc_(entity);
}

}  // namespace impeller
