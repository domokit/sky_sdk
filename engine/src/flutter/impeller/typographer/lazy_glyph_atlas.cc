// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/typographer/lazy_glyph_atlas.h"

#include "fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/typographer/glyph_atlas.h"
#include "impeller/typographer/text_frame.h"
#include "impeller/typographer/typographer_context.h"

#include <memory>
#include <utility>

namespace impeller {

static const std::shared_ptr<GlyphAtlas> kNullGlyphAtlas = nullptr;

LazyGlyphAtlas::LazyGlyphAtlas(
    std::shared_ptr<TypographerContext> typographer_context)
    : typographer_context_(std::move(typographer_context)),
      alpha_context_(typographer_context_
                         ? typographer_context_->CreateGlyphAtlasContext(
                               GlyphAtlas::Type::kAlphaBitmap)
                         : nullptr),
      color_context_(typographer_context_
                         ? typographer_context_->CreateGlyphAtlasContext(
                               GlyphAtlas::Type::kColorBitmap)
                         : nullptr) {}

LazyGlyphAtlas::~LazyGlyphAtlas() = default;

void LazyGlyphAtlas::AddTextFrame(const std::shared_ptr<TextFrame>& frame,
                                  Scalar scale,
                                  Point offset,
                                  std::optional<GlyphProperties> properties) {
  frame->SetPerFrameData(scale, offset, properties);
  FML_DCHECK(alpha_atlas_ == nullptr && color_atlas_ == nullptr);
  if (frame->GetAtlasType() == GlyphAtlas::Type::kAlphaBitmap) {
    alpha_text_frames_.push_back(frame);
  } else {
    color_text_frames_.push_back(frame);
  }
}

void LazyGlyphAtlas::ResetTextFrames() {
  alpha_text_frames_.clear();
  color_text_frames_.clear();
  alpha_atlas_.reset();
  color_atlas_.reset();
}

const std::shared_ptr<GlyphAtlas>& LazyGlyphAtlas::CreateOrGetGlyphAtlas(
    Context& context,
    HostBuffer& host_buffer,
    GlyphAtlas::Type type) const {
  {
    if (type == GlyphAtlas::Type::kAlphaBitmap && alpha_atlas_) {
      return alpha_atlas_;
    }
    if (type == GlyphAtlas::Type::kColorBitmap && color_atlas_) {
      return color_atlas_;
    }
  }

  if (!typographer_context_) {
    VALIDATION_LOG << "Unable to render text because a TypographerContext has "
                      "not been set.";
    return kNullGlyphAtlas;
  }
  if (!typographer_context_->IsValid()) {
    VALIDATION_LOG
        << "Unable to render text because the TypographerContext is invalid.";
    return kNullGlyphAtlas;
  }

  auto& glyph_map = type == GlyphAtlas::Type::kAlphaBitmap ? alpha_text_frames_
                                                           : color_text_frames_;
  const std::shared_ptr<GlyphAtlasContext>& atlas_context =
      type == GlyphAtlas::Type::kAlphaBitmap ? alpha_context_ : color_context_;
  std::shared_ptr<GlyphAtlas> atlas = typographer_context_->CreateGlyphAtlas(
      context, type, host_buffer, atlas_context, glyph_map);
  if (!atlas || !atlas->IsValid()) {
    VALIDATION_LOG << "Could not create valid atlas.";
    return kNullGlyphAtlas;
  }
  if (type == GlyphAtlas::Type::kAlphaBitmap) {
    alpha_atlas_ = std::move(atlas);
    return alpha_atlas_;
  }
  if (type == GlyphAtlas::Type::kColorBitmap) {
    color_atlas_ = std::move(atlas);
    return color_atlas_;
  }
  FML_UNREACHABLE();
}

}  // namespace impeller
