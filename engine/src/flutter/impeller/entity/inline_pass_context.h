// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_INLINE_PASS_CONTEXT_H_
#define FLUTTER_IMPELLER_ENTITY_INLINE_PASS_CONTEXT_H_

#include <cstdint>

#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity_pass_target.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

class InlinePassContext {
 public:
  InlinePassContext(const ContentContext& renderer,
                    EntityPassTarget& pass_target);

  ~InlinePassContext();

  bool IsValid() const;

  bool IsActive() const;

  std::shared_ptr<Texture> GetTexture();

  bool EndPass();

  EntityPassTarget& GetPassTarget() const;

  uint32_t GetPassCount() const;

  const std::shared_ptr<RenderPass>& GetRenderPass();

 private:
  const ContentContext& renderer_;
  EntityPassTarget& pass_target_;
  std::shared_ptr<CommandBuffer> command_buffer_;
  std::shared_ptr<RenderPass> pass_;
  uint32_t pass_count_ = 0;

  InlinePassContext(const InlinePassContext&) = delete;

  InlinePassContext& operator=(const InlinePassContext&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_INLINE_PASS_CONTEXT_H_
