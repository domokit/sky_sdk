// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_ANONYMOUS_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_ANONYMOUS_CONTENTS_H_

#include <memory>

#include "impeller/entity/contents/contents.h"

namespace impeller {

class AnonymousContents final : public Contents {
 public:
  static std::shared_ptr<Contents> Make(RenderProc render_proc,
                                        CoverageProc coverage_proc);

  // |Contents|
  ~AnonymousContents() override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

 private:
  RenderProc render_proc_;
  CoverageProc coverage_proc_;

  AnonymousContents(RenderProc render_proc, CoverageProc coverage_proc);

  AnonymousContents(const AnonymousContents&) = delete;

  AnonymousContents& operator=(const AnonymousContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_ANONYMOUS_CONTENTS_H_
