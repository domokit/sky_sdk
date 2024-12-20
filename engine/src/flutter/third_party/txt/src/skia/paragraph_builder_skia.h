/*
 * Copyright 2019 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIB_TXT_SRC_PARAGRAPH_BUILDER_SKIA_H_
#define LIB_TXT_SRC_PARAGRAPH_BUILDER_SKIA_H_

#include "txt/paragraph_builder.h"

#include "flutter/display_list/dl_paint.h"
#include "third_party/skia/modules/skparagraph/include/ParagraphBuilder.h"

namespace txt {

//------------------------------------------------------------------------------
/// @brief      ParagraphBuilder implementation using Skia's text layout module.
///
/// @note       Despite the suffix "Skia", this class is not specific to Skia
///             and is also used with the Impeller backend.
class ParagraphBuilderSkia : public ParagraphBuilder {
 public:
  ParagraphBuilderSkia(const ParagraphStyle& style,
                       std::shared_ptr<FontCollection> font_collection,
                       const bool impeller_enabled);

  virtual ~ParagraphBuilderSkia();

  virtual void PushStyle(const TextStyle& style) override;
  virtual void Pop() override;
  virtual const TextStyle& PeekStyle() override;
  virtual void AddText(const std::u16string& text) override;
  virtual void AddText(const uint8_t* utf8_data, size_t byte_length) override;
  virtual void AddPlaceholder(PlaceholderRun& span) override;
  virtual std::unique_ptr<Paragraph> Build() override;

 private:
  friend class SkiaParagraphBuilderTests_ParagraphStrutStyle_Test;

  skia::textlayout::ParagraphPainter::PaintID CreatePaintID(
      const flutter::DlPaint& dl_paint);
  skia::textlayout::ParagraphStyle TxtToSkia(const ParagraphStyle& txt);
  skia::textlayout::TextStyle TxtToSkia(const TextStyle& txt);

  std::shared_ptr<skia::textlayout::ParagraphBuilder> builder_;
  TextStyle base_style_;

  /// @brief      Whether Impeller is enabled in the runtime.
  ///
  /// @note       As of the time of this writing, this is used to draw text
  ///             decorations (i.e. dashed and dotted lines) directly using the
  ///             `drawLine` API, because Impeller's path rendering does not
  ///             support dashed and dotted lines (but Skia's does).
  const bool impeller_enabled_;
  std::stack<TextStyle> txt_style_stack_;
  std::vector<flutter::DlPaint> dl_paints_;
};

}  // namespace txt

#endif  // LIB_TXT_SRC_PARAGRAPH_BUILDER_SKIA_H_
