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

#include "paragraph_skia.h"

#include <algorithm>
#include <numeric>
#include "display_list/dl_paint.h"
#include "fml/logging.h"
#include "impeller/typographer/backends/skia/text_frame_skia.h"
#include "include/core/SkMatrix.h"
#include "third_party/skia/src/core/SkTextBlobPriv.h"  // nogncheck

namespace txt {

namespace skt = skia::textlayout;
using PaintID = skt::ParagraphPainter::PaintID;

using namespace flutter;

namespace {

// Convert SkFontStyle::Weight values (ranging from 100-900) to txt::FontWeight
// values (ranging from 0-8).
txt::FontWeight GetTxtFontWeight(int font_weight) {
  int txt_weight = (font_weight - 100) / 100;
  txt_weight = std::clamp(txt_weight, static_cast<int>(txt::FontWeight::w100),
                          static_cast<int>(txt::FontWeight::w900));
  return static_cast<txt::FontWeight>(txt_weight);
}

txt::FontStyle GetTxtFontStyle(SkFontStyle::Slant font_slant) {
  return font_slant == SkFontStyle::Slant::kUpright_Slant
             ? txt::FontStyle::normal
             : txt::FontStyle::italic;
}

class DisplayListParagraphPainter : public skt::ParagraphPainter {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Creates a |skt::ParagraphPainter| that draws to DisplayList.
  ///
  /// @param      builder  The display list builder.
  /// @param[in]  dl_paints The paints referenced by ID in the `drawX` methods.
  /// @param[in]  draw_path_effect  If true, draw path effects directly by
  ///                               drawing multiple lines instead of providing
  //                                a path effect to the paint.
  ///
  /// @note       Impeller does not (and will not) support path effects, but the
  ///             Skia backend does. That means that if we want to draw dashed
  ///             and dotted lines, we need to draw them directly using the
  ///             `drawLine` API instead of using a path effect.
  ///
  ///             See https://github.com/flutter/flutter/issues/126673. It
  ///             probably makes sense to eventually make this a compile-time
  ///             decision (i.e. with `#ifdef`) instead of a runtime option.
  DisplayListParagraphPainter(DisplayListBuilder* builder,
                              const std::vector<DlPaint>& dl_paints,
                              bool impeller_enabled)
      : builder_(builder),
        dl_paints_(dl_paints),
        impeller_enabled_(impeller_enabled) {}

  void drawTextBlob(const sk_sp<SkTextBlob>& blob,
                    SkScalar x,
                    SkScalar y,
                    const SkPaintOrID& paint) override {
    if (!blob) {
      return;
    }
    size_t paint_id = std::get<PaintID>(paint);
    FML_DCHECK(paint_id < dl_paints_.size());

#ifdef IMPELLER_SUPPORTS_RENDERING
    if (impeller_enabled_) {
      SkTextBlobRunIterator run(blob.get());
      if (ShouldRenderAsPath(dl_paints_[paint_id])) {
        auto path = skia::textlayout::Paragraph::GetPath(blob.get());
        // If there is no path, this is an emoji and should be drawn as is,
        // ignoring the color source.
        if (path.isEmpty()) {
          builder_->DrawTextFrame(impeller::MakeTextFrameFromTextBlobSkia(blob),
                                  x, y, dl_paints_[paint_id]);

          return;
        }

        auto transformed = path.makeTransform(SkMatrix::Translate(
            x + blob->bounds().left(), y + blob->bounds().top()));
        builder_->DrawPath(transformed, dl_paints_[paint_id]);
        return;
      }
      builder_->DrawTextFrame(impeller::MakeTextFrameFromTextBlobSkia(blob), x,
                              y, dl_paints_[paint_id]);
      return;
    }
#endif  // IMPELLER_SUPPORTS_RENDERING
    builder_->DrawTextBlob(blob, x, y, dl_paints_[paint_id]);
  }

  void drawTextShadow(const sk_sp<SkTextBlob>& blob,
                      SkScalar x,
                      SkScalar y,
                      SkColor color,
                      SkScalar blur_sigma) override {
    if (!blob) {
      return;
    }
    DlPaint paint;
    paint.setColor(DlColor(color));
    if (blur_sigma > 0.0) {
      DlBlurMaskFilter filter(DlBlurStyle::kNormal, blur_sigma, false);
      paint.setMaskFilter(&filter);
    }
    if (impeller_enabled_) {
      builder_->DrawTextFrame(impeller::MakeTextFrameFromTextBlobSkia(blob), x,
                              y, paint);
      return;
    }
    builder_->DrawTextBlob(blob, x, y, paint);
  }

  void drawRect(const SkRect& rect, const SkPaintOrID& paint) override {
    size_t paint_id = std::get<PaintID>(paint);
    FML_DCHECK(paint_id < dl_paints_.size());
    builder_->DrawRect(rect, dl_paints_[paint_id]);
  }

  void drawFilledRect(const SkRect& rect,
                      const DecorationStyle& decor_style) override {
    DlPaint paint = toDlPaint(decor_style, DlDrawStyle::kFill);
    builder_->DrawRect(rect, paint);
  }

  void drawPath(const SkPath& path,
                const DecorationStyle& decor_style) override {
    builder_->DrawPath(path, toDlPaint(decor_style));
  }

  void drawLine(SkScalar x0,
                SkScalar y0,
                SkScalar x1,
                SkScalar y1,
                const DecorationStyle& decor_style) override {
    auto dash_path_effect = decor_style.getDashPathEffect();
    auto paint = toDlPaint(decor_style);

    if (dash_path_effect) {
      builder_->DrawDashedLine(DlPoint(x0, y0), DlPoint(x1, y1),
                               dash_path_effect->fOnLength,
                               dash_path_effect->fOffLength, paint);
    } else {
      builder_->DrawLine(SkPoint::Make(x0, y0), SkPoint::Make(x1, y1), paint);
    }
  }

  void clipRect(const SkRect& rect) override {
    builder_->ClipRect(rect, DlCanvas::ClipOp::kIntersect, false);
  }

  void translate(SkScalar dx, SkScalar dy) override {
    builder_->Translate(dx, dy);
  }

  void save() override { builder_->Save(); }

  void restore() override { builder_->Restore(); }

 private:
  bool ShouldRenderAsPath(const DlPaint& paint) const {
    FML_DCHECK(impeller_enabled_);
    // Text with non-trivial color sources should be rendered as a path when
    // running on Impeller for correctness. These filters rely on having the
    // glyph coverage, whereas regular text is drawn as rectangular texture
    // samples.
    // If the text is stroked and the stroke width is large enough, use path
    // rendering anyway, as the fidelity problems won't be as noticable and
    // rendering will be faster as it avoids software rasterization. A stroke
    // width of four was chosen by eyeballing the point at which the path
    // text looks good enough, with some room for error.
    return paint.getColorSource() ||
           (paint.getDrawStyle() == DlDrawStyle::kStroke &&
            paint.getStrokeWidth() > 4);
  }

  DlPaint toDlPaint(const DecorationStyle& decor_style,
                    DlDrawStyle draw_style = DlDrawStyle::kStroke) {
    DlPaint paint;
    paint.setDrawStyle(draw_style);
    paint.setAntiAlias(true);
    paint.setColor(DlColor(decor_style.getColor()));
    paint.setStrokeWidth(decor_style.getStrokeWidth());
    return paint;
  }

  DisplayListBuilder* builder_;
  const std::vector<DlPaint>& dl_paints_;
  const bool impeller_enabled_;
};

}  // anonymous namespace

ParagraphSkia::ParagraphSkia(std::unique_ptr<skt::Paragraph> paragraph,
                             std::vector<flutter::DlPaint>&& dl_paints,
                             bool impeller_enabled)
    : paragraph_(std::move(paragraph)),
      dl_paints_(dl_paints),
      impeller_enabled_(impeller_enabled) {}

double ParagraphSkia::GetMaxWidth() {
  return SkScalarToDouble(paragraph_->getMaxWidth());
}

double ParagraphSkia::GetHeight() {
  return SkScalarToDouble(paragraph_->getHeight());
}

double ParagraphSkia::GetLongestLine() {
  return SkScalarToDouble(paragraph_->getLongestLine());
}

std::vector<LineMetrics>& ParagraphSkia::GetLineMetrics() {
  if (!line_metrics_) {
    std::vector<skt::LineMetrics> metrics;
    paragraph_->getLineMetrics(metrics);

    line_metrics_.emplace();
    line_metrics_styles_.reserve(
        std::accumulate(metrics.begin(), metrics.end(), 0,
                        [](const int a, const skt::LineMetrics& b) {
                          return a + b.fLineMetrics.size();
                        }));

    for (const skt::LineMetrics& skm : metrics) {
      LineMetrics& txtm = line_metrics_->emplace_back(
          skm.fStartIndex, skm.fEndIndex, skm.fEndExcludingWhitespaces,
          skm.fEndIncludingNewline, skm.fHardBreak);
      txtm.ascent = skm.fAscent;
      txtm.descent = skm.fDescent;
      txtm.unscaled_ascent = skm.fUnscaledAscent;
      txtm.height = skm.fHeight;
      txtm.width = skm.fWidth;
      txtm.left = skm.fLeft;
      txtm.baseline = skm.fBaseline;
      txtm.line_number = skm.fLineNumber;

      for (const auto& sk_iter : skm.fLineMetrics) {
        const skt::StyleMetrics& sk_style_metrics = sk_iter.second;
        line_metrics_styles_.push_back(SkiaToTxt(*sk_style_metrics.text_style));
        txtm.run_metrics.emplace(
            std::piecewise_construct, std::forward_as_tuple(sk_iter.first),
            std::forward_as_tuple(&line_metrics_styles_.back(),
                                  sk_style_metrics.font_metrics));
      }
    }
  }

  return line_metrics_.value();
}

bool ParagraphSkia::GetLineMetricsAt(int lineNumber,
                                     skt::LineMetrics* lineMetrics) const {
  return paragraph_->getLineMetricsAt(lineNumber, lineMetrics);
};

double ParagraphSkia::GetMinIntrinsicWidth() {
  return SkScalarToDouble(paragraph_->getMinIntrinsicWidth());
}

double ParagraphSkia::GetMaxIntrinsicWidth() {
  return SkScalarToDouble(paragraph_->getMaxIntrinsicWidth());
}

double ParagraphSkia::GetAlphabeticBaseline() {
  return SkScalarToDouble(paragraph_->getAlphabeticBaseline());
}

double ParagraphSkia::GetIdeographicBaseline() {
  return SkScalarToDouble(paragraph_->getIdeographicBaseline());
}

bool ParagraphSkia::DidExceedMaxLines() {
  return paragraph_->didExceedMaxLines();
}

void ParagraphSkia::Layout(double width) {
  line_metrics_.reset();
  line_metrics_styles_.clear();
  paragraph_->layout(width);
}

bool ParagraphSkia::Paint(DisplayListBuilder* builder, double x, double y) {
  DisplayListParagraphPainter painter(builder, dl_paints_, impeller_enabled_);
  paragraph_->paint(&painter, x, y);
  return true;
}

std::vector<Paragraph::TextBox> ParagraphSkia::GetRectsForRange(
    size_t start,
    size_t end,
    RectHeightStyle rect_height_style,
    RectWidthStyle rect_width_style) {
  std::vector<skt::TextBox> skia_boxes = paragraph_->getRectsForRange(
      start, end, static_cast<skt::RectHeightStyle>(rect_height_style),
      static_cast<skt::RectWidthStyle>(rect_width_style));

  std::vector<Paragraph::TextBox> boxes;
  for (const skt::TextBox& skia_box : skia_boxes) {
    boxes.emplace_back(skia_box.rect,
                       static_cast<TextDirection>(skia_box.direction));
  }

  return boxes;
}

std::vector<Paragraph::TextBox> ParagraphSkia::GetRectsForPlaceholders() {
  std::vector<skt::TextBox> skia_boxes = paragraph_->getRectsForPlaceholders();

  std::vector<Paragraph::TextBox> boxes;
  for (const skt::TextBox& skia_box : skia_boxes) {
    boxes.emplace_back(skia_box.rect,
                       static_cast<TextDirection>(skia_box.direction));
  }

  return boxes;
}

Paragraph::PositionWithAffinity ParagraphSkia::GetGlyphPositionAtCoordinate(
    double dx,
    double dy) {
  skt::PositionWithAffinity skia_pos =
      paragraph_->getGlyphPositionAtCoordinate(dx, dy);

  return ParagraphSkia::PositionWithAffinity(
      skia_pos.position, static_cast<Affinity>(skia_pos.affinity));
}

bool ParagraphSkia::GetGlyphInfoAt(
    unsigned offset,
    skia::textlayout::Paragraph::GlyphInfo* glyphInfo) const {
  return paragraph_->getGlyphInfoAtUTF16Offset(offset, glyphInfo);
}

bool ParagraphSkia::GetClosestGlyphInfoAtCoordinate(
    double dx,
    double dy,
    skia::textlayout::Paragraph::GlyphInfo* glyphInfo) const {
  return paragraph_->getClosestUTF16GlyphInfoAt(dx, dy, glyphInfo);
};

Paragraph::Range<size_t> ParagraphSkia::GetWordBoundary(size_t offset) {
  skt::SkRange<size_t> range = paragraph_->getWordBoundary(offset);
  return Paragraph::Range<size_t>(range.start, range.end);
}

size_t ParagraphSkia::GetNumberOfLines() const {
  return paragraph_->lineNumber();
}

int ParagraphSkia::GetLineNumberAt(size_t codeUnitIndex) const {
  return paragraph_->getLineNumberAtUTF16Offset(codeUnitIndex);
}

TextStyle ParagraphSkia::SkiaToTxt(const skt::TextStyle& skia) {
  TextStyle txt;

  txt.color = skia.getColor();
  txt.decoration = static_cast<TextDecoration>(skia.getDecorationType());
  txt.decoration_color = skia.getDecorationColor();
  txt.decoration_style =
      static_cast<TextDecorationStyle>(skia.getDecorationStyle());
  txt.decoration_thickness_multiplier =
      SkScalarToDouble(skia.getDecorationThicknessMultiplier());
  txt.font_weight = GetTxtFontWeight(skia.getFontStyle().weight());
  txt.font_style = GetTxtFontStyle(skia.getFontStyle().slant());

  txt.text_baseline = static_cast<TextBaseline>(skia.getTextBaseline());

  for (const SkString& font_family : skia.getFontFamilies()) {
    txt.font_families.emplace_back(font_family.c_str());
  }

  txt.font_size = SkScalarToDouble(skia.getFontSize());
  txt.letter_spacing = SkScalarToDouble(skia.getLetterSpacing());
  txt.word_spacing = SkScalarToDouble(skia.getWordSpacing());
  txt.height = SkScalarToDouble(skia.getHeight());

  txt.locale = skia.getLocale().c_str();
  if (skia.hasBackground()) {
    PaintID background_id = std::get<PaintID>(skia.getBackgroundPaintOrID());
    txt.background = dl_paints_[background_id];
  }
  if (skia.hasForeground()) {
    PaintID foreground_id = std::get<PaintID>(skia.getForegroundPaintOrID());
    txt.foreground = dl_paints_[foreground_id];
  }

  txt.text_shadows.clear();
  for (const skt::TextShadow& skia_shadow : skia.getShadows()) {
    txt::TextShadow shadow;
    shadow.offset = skia_shadow.fOffset;
    shadow.blur_sigma = skia_shadow.fBlurSigma;
    shadow.color = skia_shadow.fColor;
    txt.text_shadows.emplace_back(shadow);
  }

  return txt;
}

}  // namespace txt
