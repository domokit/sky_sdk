// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_LAZY_GLYPH_ATLAS_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_LAZY_GLYPH_ATLAS_H_

#include "impeller/renderer/context.h"
#include "impeller/typographer/glyph_atlas.h"
#include "impeller/typographer/text_frame.h"
#include "impeller/typographer/typographer_context.h"

namespace impeller {

class LazyGlyphAtlas {
 public:
  explicit LazyGlyphAtlas(
      std::shared_ptr<TypographerContext> typographer_context);

  ~LazyGlyphAtlas();

  void AddTextFrame(const std::shared_ptr<TextFrame>& frame,
                    Scalar scale,
                    Point offset,
                    std::optional<GlyphProperties> properties);

  void ResetTextFrames();

  const std::shared_ptr<GlyphAtlas>& CreateOrGetGlyphAtlas(
      Context& context,
      HostBuffer& host_buffer,
      GlyphAtlas::Type type) const;

 private:
  std::shared_ptr<TypographerContext> typographer_context_;

  std::vector<std::shared_ptr<TextFrame>> alpha_text_frames_;
  std::vector<std::shared_ptr<TextFrame>> color_text_frames_;
  std::shared_ptr<GlyphAtlasContext> alpha_context_;
  std::shared_ptr<GlyphAtlasContext> color_context_;
  mutable std::shared_ptr<GlyphAtlas> alpha_atlas_;
  mutable std::shared_ptr<GlyphAtlas> color_atlas_;

  LazyGlyphAtlas(const LazyGlyphAtlas&) = delete;

  LazyGlyphAtlas& operator=(const LazyGlyphAtlas&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_LAZY_GLYPH_ATLAS_H_
