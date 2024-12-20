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

#ifndef LIB_TXT_SRC_PARAGRAPH_SKIA_H_
#define LIB_TXT_SRC_PARAGRAPH_SKIA_H_

#include <optional>

#include "txt/paragraph.h"

#include "third_party/skia/modules/skparagraph/include/Paragraph.h"

namespace txt {

// Implementation of Paragraph based on Skia's text layout module.
class ParagraphSkia : public Paragraph {
 public:
  ParagraphSkia(std::unique_ptr<skia::textlayout::Paragraph> paragraph,
                std::vector<flutter::DlPaint>&& dl_paints,
                bool impeller_enabled);

  virtual ~ParagraphSkia() = default;

  double GetMaxWidth() override;

  double GetHeight() override;

  double GetLongestLine() override;

  double GetMinIntrinsicWidth() override;

  double GetMaxIntrinsicWidth() override;

  double GetAlphabeticBaseline() override;

  double GetIdeographicBaseline() override;

  std::vector<LineMetrics>& GetLineMetrics() override;

  bool GetLineMetricsAt(
      int lineNumber,
      skia::textlayout::LineMetrics* lineMetrics) const override;

  size_t GetNumberOfLines() const override;

  int GetLineNumberAt(size_t utf16Offset) const override;

  bool DidExceedMaxLines() override;

  void Layout(double width) override;

  bool Paint(flutter::DisplayListBuilder* builder, double x, double y) override;

  std::vector<TextBox> GetRectsForRange(
      size_t start,
      size_t end,
      RectHeightStyle rect_height_style,
      RectWidthStyle rect_width_style) override;

  std::vector<TextBox> GetRectsForPlaceholders() override;

  PositionWithAffinity GetGlyphPositionAtCoordinate(double dx,
                                                    double dy) override;

  bool GetGlyphInfoAt(
      unsigned offset,
      skia::textlayout::Paragraph::GlyphInfo* glyphInfo) const override;

  bool GetClosestGlyphInfoAtCoordinate(
      double dx,
      double dy,
      skia::textlayout::Paragraph::GlyphInfo* glyphInfo) const override;

  Range<size_t> GetWordBoundary(size_t offset) override;

 private:
  TextStyle SkiaToTxt(const skia::textlayout::TextStyle& skia);

  std::unique_ptr<skia::textlayout::Paragraph> paragraph_;
  std::vector<flutter::DlPaint> dl_paints_;
  std::optional<std::vector<LineMetrics>> line_metrics_;
  std::vector<TextStyle> line_metrics_styles_;
  const bool impeller_enabled_;
};

}  // namespace txt

#endif  // LIB_TXT_SRC_PARAGRAPH_SKIA_H_
