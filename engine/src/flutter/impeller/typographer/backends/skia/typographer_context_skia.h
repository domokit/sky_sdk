// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPOGRAPHER_CONTEXT_SKIA_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPOGRAPHER_CONTEXT_SKIA_H_

#include "impeller/typographer/typographer_context.h"

namespace impeller {

class TypographerContextSkia : public TypographerContext {
 public:
  static std::shared_ptr<TypographerContext> Make();

  TypographerContextSkia();

  ~TypographerContextSkia() override;

  // |TypographerContext|
  std::shared_ptr<GlyphAtlasContext> CreateGlyphAtlasContext(
      GlyphAtlas::Type type) const override;

  // |TypographerContext|
  std::shared_ptr<GlyphAtlas> CreateGlyphAtlas(
      Context& context,
      GlyphAtlas::Type type,
      HostBuffer& host_buffer,
      const std::shared_ptr<GlyphAtlasContext>& atlas_context,
      const std::vector<std::shared_ptr<TextFrame>>& text_frames)
      const override;

 private:
  static std::pair<std::vector<FontGlyphPair>, std::vector<Rect>>
  CollectNewGlyphs(const std::shared_ptr<GlyphAtlas>& atlas,
                   const std::vector<std::shared_ptr<TextFrame>>& text_frames);

  TypographerContextSkia(const TypographerContextSkia&) = delete;

  TypographerContextSkia& operator=(const TypographerContextSkia&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPOGRAPHER_CONTEXT_SKIA_H_
