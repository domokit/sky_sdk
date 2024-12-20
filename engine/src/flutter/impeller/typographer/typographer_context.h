// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_TYPOGRAPHER_CONTEXT_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_TYPOGRAPHER_CONTEXT_H_

#include <memory>

#include "impeller/renderer/context.h"
#include "impeller/typographer/glyph_atlas.h"
#include "impeller/typographer/text_frame.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      The graphics context necessary to render text.
///
///             This is necessary to create and reference resources related to
///             rendering text on the GPU.
///
///
class TypographerContext {
 public:
  virtual ~TypographerContext();

  virtual bool IsValid() const;

  virtual std::shared_ptr<GlyphAtlasContext> CreateGlyphAtlasContext(
      GlyphAtlas::Type type) const = 0;

  virtual std::shared_ptr<GlyphAtlas> CreateGlyphAtlas(
      Context& context,
      GlyphAtlas::Type type,
      HostBuffer& host_buffer,
      const std::shared_ptr<GlyphAtlasContext>& atlas_context,
      const std::vector<std::shared_ptr<TextFrame>>& text_frames) const = 0;

 protected:
  //----------------------------------------------------------------------------
  /// @brief      Create a new context to render text that talks to an
  ///             underlying graphics context.
  ///
  TypographerContext();

 private:
  bool is_valid_ = false;

  TypographerContext(const TypographerContext&) = delete;

  TypographerContext& operator=(const TypographerContext&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_TYPOGRAPHER_CONTEXT_H_
