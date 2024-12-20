// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_CONTEXT_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_CONTEXT_H_

#include <memory>

#include "impeller/entity/contents/content_context.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/render_target.h"
#include "impeller/typographer/typographer_context.h"

namespace impeller {

struct Picture;

class AiksContext {
 public:
  /// Construct a new AiksContext.
  ///
  /// @param context              The Impeller context that Aiks should use for
  ///                             allocating resources and executing device
  ///                             commands. Required.
  /// @param typographer_context  The text backend to use for rendering text. If
  ///                             `nullptr` is supplied, then attempting to draw
  ///                             text with Aiks will result in validation
  ///                             errors.
  /// @param render_target_allocator Injects a render target allocator or
  ///                                allocates its own if none is supplied.
  AiksContext(std::shared_ptr<Context> context,
              std::shared_ptr<TypographerContext> typographer_context,
              std::optional<std::shared_ptr<RenderTargetAllocator>>
                  render_target_allocator = std::nullopt);

  ~AiksContext();

  bool IsValid() const;

  std::shared_ptr<Context> GetContext() const;

  ContentContext& GetContentContext() const;

 private:
  std::shared_ptr<Context> context_;
  std::unique_ptr<ContentContext> content_context_;
  bool is_valid_ = false;

  AiksContext(const AiksContext&) = delete;

  AiksContext& operator=(const AiksContext&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_CONTEXT_H_
