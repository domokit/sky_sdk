// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXT_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXT_CONTENTS_H_

#include <memory>

#include "impeller/entity/contents/contents.h"
#include "impeller/geometry/color.h"
#include "impeller/typographer/font_glyph_pair.h"
#include "impeller/typographer/text_frame.h"

namespace impeller {

class LazyGlyphAtlas;
class Context;

class TextContents final : public Contents {
 public:
  TextContents();

  ~TextContents();

  void SetTextFrame(const std::shared_ptr<TextFrame>& frame);

  void SetColor(Color color);

  /// @brief Force the text color to apply to the rendered glyphs, even if those
  ///        glyphs are bitmaps.
  ///
  ///        This is used to ensure that mask blurs work correctly on emoji.
  void SetForceTextColor(bool value);

  /// Must be set after text frame.
  void SetTextProperties(Color color,
                         bool stroke,
                         Scalar stroke_width,
                         Cap stroke_cap,
                         Join stroke_join,
                         Scalar stroke_miter);

  Color GetColor() const;

  // |Contents|
  void SetInheritedOpacity(Scalar opacity) override;

  // The offset is only used for computing the subpixel glyph position.
  void SetOffset(Vector2 offset);

  std::optional<Rect> GetTextFrameBounds() const;

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  void SetScale(Scalar scale) { scale_ = scale; }

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

 private:
  std::optional<GlyphProperties> GetGlyphProperties() const;

  std::shared_ptr<TextFrame> frame_;
  Scalar scale_ = 1.0;
  Scalar inherited_opacity_ = 1.0;
  Vector2 offset_;
  bool force_text_color_ = false;
  Color color_;
  GlyphProperties properties_;

  TextContents(const TextContents&) = delete;

  TextContents& operator=(const TextContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXT_CONTENTS_H_
