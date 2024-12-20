// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/typographer/text_run.h"

namespace impeller {

TextRun::TextRun(const Font& font) : font_(font) {
  if (!font_.IsValid()) {
    return;
  }
  is_valid_ = true;
}

TextRun::TextRun(const Font& font, std::vector<GlyphPosition>& glyphs)
    : font_(font), glyphs_(std::move(glyphs)) {
  if (!font_.IsValid()) {
    return;
  }
  is_valid_ = true;
}

TextRun::~TextRun() = default;

bool TextRun::AddGlyph(Glyph glyph, Point position) {
  glyphs_.emplace_back(GlyphPosition{glyph, position});
  return true;
}

bool TextRun::IsValid() const {
  return is_valid_;
}

const std::vector<TextRun::GlyphPosition>& TextRun::GetGlyphPositions() const {
  return glyphs_;
}

size_t TextRun::GetGlyphCount() const {
  return glyphs_.size();
}

const Font& TextRun::GetFont() const {
  return font_;
}

}  // namespace impeller
