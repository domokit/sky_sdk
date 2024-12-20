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

#include "paragraph_builder_skia.h"
#include "paragraph_skia.h"

#include "third_party/skia/modules/skparagraph/include/ParagraphStyle.h"
#include "third_party/skia/modules/skparagraph/include/TextStyle.h"
#include "third_party/skia/modules/skunicode/include/SkUnicode_icu.h"
#include "txt/paragraph_style.h"

namespace skt = skia::textlayout;

namespace txt {

namespace {

// Convert txt::FontWeight values (ranging from 0-8) to SkFontStyle::Weight
// values (ranging from 100-900).
SkFontStyle::Weight GetSkFontStyleWeight(txt::FontWeight font_weight) {
  return static_cast<SkFontStyle::Weight>(static_cast<int>(font_weight) * 100 +
                                          100);
}

SkFontStyle MakeSkFontStyle(txt::FontWeight font_weight,
                            txt::FontStyle font_style) {
  return SkFontStyle(
      GetSkFontStyleWeight(font_weight), SkFontStyle::Width::kNormal_Width,
      font_style == txt::FontStyle::normal ? SkFontStyle::Slant::kUpright_Slant
                                           : SkFontStyle::Slant::kItalic_Slant);
}

}  // anonymous namespace

ParagraphBuilderSkia::ParagraphBuilderSkia(
    const ParagraphStyle& style,
    std::shared_ptr<FontCollection> font_collection,
    const bool impeller_enabled)
    : base_style_(style.GetTextStyle()), impeller_enabled_(impeller_enabled) {
  builder_ = skt::ParagraphBuilder::make(
      TxtToSkia(style), font_collection->CreateSktFontCollection(),
      SkUnicodes::ICU::Make());
}

ParagraphBuilderSkia::~ParagraphBuilderSkia() = default;

void ParagraphBuilderSkia::PushStyle(const TextStyle& style) {
  builder_->pushStyle(TxtToSkia(style));
  txt_style_stack_.push(style);
}

void ParagraphBuilderSkia::Pop() {
  builder_->pop();
  txt_style_stack_.pop();
}

const TextStyle& ParagraphBuilderSkia::PeekStyle() {
  return txt_style_stack_.empty() ? base_style_ : txt_style_stack_.top();
}

void ParagraphBuilderSkia::AddText(const std::u16string& text) {
  builder_->addText(text);
}

void ParagraphBuilderSkia::AddText(const uint8_t* utf8_data,
                                   size_t byte_length) {
  builder_->addText(reinterpret_cast<const char*>(utf8_data), byte_length);
}

void ParagraphBuilderSkia::AddPlaceholder(PlaceholderRun& span) {
  skt::PlaceholderStyle placeholder_style;
  placeholder_style.fHeight = span.height;
  placeholder_style.fWidth = span.width;
  placeholder_style.fBaseline = static_cast<skt::TextBaseline>(span.baseline);
  placeholder_style.fBaselineOffset = span.baseline_offset;
  placeholder_style.fAlignment =
      static_cast<skt::PlaceholderAlignment>(span.alignment);

  builder_->addPlaceholder(placeholder_style);
}

std::unique_ptr<Paragraph> ParagraphBuilderSkia::Build() {
  return std::make_unique<ParagraphSkia>(
      builder_->Build(), std::move(dl_paints_), impeller_enabled_);
}

skt::ParagraphPainter::PaintID ParagraphBuilderSkia::CreatePaintID(
    const flutter::DlPaint& dl_paint) {
  dl_paints_.push_back(dl_paint);
  return dl_paints_.size() - 1;
}

skt::ParagraphStyle ParagraphBuilderSkia::TxtToSkia(const ParagraphStyle& txt) {
  skt::ParagraphStyle skia;
  skt::TextStyle text_style;

  // Convert the default color of an SkParagraph text style into a DlPaint.
  flutter::DlPaint dl_paint;
  dl_paint.setColor(flutter::DlColor(text_style.getColor()));
  text_style.setForegroundPaintID(CreatePaintID(dl_paint));

  text_style.setFontStyle(MakeSkFontStyle(txt.font_weight, txt.font_style));
  text_style.setFontSize(SkDoubleToScalar(txt.font_size));
  text_style.setHeight(SkDoubleToScalar(txt.height));
  text_style.setHeightOverride(txt.has_height_override);
  text_style.setFontFamilies({SkString(txt.font_family.c_str())});
  text_style.setLocale(SkString(txt.locale.c_str()));
  skia.setTextStyle(text_style);

  skt::StrutStyle strut_style;
  strut_style.setFontStyle(
      MakeSkFontStyle(txt.strut_font_weight, txt.strut_font_style));
  strut_style.setFontSize(SkDoubleToScalar(txt.strut_font_size));
  strut_style.setHeight(SkDoubleToScalar(txt.strut_height));
  strut_style.setHeightOverride(txt.strut_has_height_override);
  strut_style.setHalfLeading(txt.strut_half_leading);

  std::vector<SkString> strut_fonts;
  std::transform(txt.strut_font_families.begin(), txt.strut_font_families.end(),
                 std::back_inserter(strut_fonts),
                 [](const std::string& f) { return SkString(f.c_str()); });
  strut_style.setFontFamilies(strut_fonts);
  strut_style.setLeading(txt.strut_leading);
  strut_style.setForceStrutHeight(txt.force_strut_height);
  strut_style.setStrutEnabled(txt.strut_enabled);
  skia.setStrutStyle(strut_style);

  skia.setTextAlign(static_cast<skt::TextAlign>(txt.text_align));
  skia.setTextDirection(static_cast<skt::TextDirection>(txt.text_direction));
  skia.setMaxLines(txt.max_lines);
  skia.setEllipsis(txt.ellipsis);
  skia.setTextHeightBehavior(
      static_cast<skt::TextHeightBehavior>(txt.text_height_behavior));

  skia.turnHintingOff();
  skia.setReplaceTabCharacters(true);
  skia.setApplyRoundingHack(false);

  return skia;
}

skt::TextStyle ParagraphBuilderSkia::TxtToSkia(const TextStyle& txt) {
  skt::TextStyle skia;

  skia.setColor(txt.color);
  skia.setDecoration(static_cast<skt::TextDecoration>(txt.decoration));
  skia.setDecorationColor(txt.decoration_color);
  skia.setDecorationStyle(
      static_cast<skt::TextDecorationStyle>(txt.decoration_style));
  skia.setDecorationThicknessMultiplier(
      SkDoubleToScalar(txt.decoration_thickness_multiplier));
  skia.setFontStyle(MakeSkFontStyle(txt.font_weight, txt.font_style));
  skia.setTextBaseline(static_cast<skt::TextBaseline>(txt.text_baseline));

  std::vector<SkString> skia_fonts;
  std::transform(txt.font_families.begin(), txt.font_families.end(),
                 std::back_inserter(skia_fonts),
                 [](const std::string& f) { return SkString(f.c_str()); });
  skia.setFontFamilies(skia_fonts);

  skia.setFontSize(SkDoubleToScalar(txt.font_size));
  skia.setLetterSpacing(SkDoubleToScalar(txt.letter_spacing));
  skia.setWordSpacing(SkDoubleToScalar(txt.word_spacing));
  skia.setHeight(SkDoubleToScalar(txt.height));
  skia.setHeightOverride(txt.has_height_override);
  skia.setHalfLeading(txt.half_leading);

  skia.setLocale(SkString(txt.locale.c_str()));
  if (txt.background.has_value()) {
    skia.setBackgroundPaintID(CreatePaintID(txt.background.value()));
  }
  if (txt.foreground.has_value()) {
    skia.setForegroundPaintID(CreatePaintID(txt.foreground.value()));
  } else {
    flutter::DlPaint dl_paint;
    dl_paint.setColor(flutter::DlColor(txt.color));
    skia.setForegroundPaintID(CreatePaintID(dl_paint));
  }

  skia.resetFontFeatures();
  for (const auto& ff : txt.font_features.GetFontFeatures()) {
    skia.addFontFeature(SkString(ff.first.c_str()), ff.second);
  }

  if (!txt.font_variations.GetAxisValues().empty()) {
    std::vector<SkFontArguments::VariationPosition::Coordinate> coordinates;
    for (const auto& it : txt.font_variations.GetAxisValues()) {
      const std::string& axis = it.first;
      if (axis.length() != 4) {
        continue;
      }
      coordinates.push_back({
          SkSetFourByteTag(axis[0], axis[1], axis[2], axis[3]),
          it.second,
      });
    }
    SkFontArguments::VariationPosition position = {
        coordinates.data(), static_cast<int>(coordinates.size())};
    skia.setFontArguments(
        SkFontArguments().setVariationDesignPosition(position));
  }

  skia.resetShadows();
  for (const txt::TextShadow& txt_shadow : txt.text_shadows) {
    skt::TextShadow shadow;
    shadow.fOffset = txt_shadow.offset;
    shadow.fBlurSigma = txt_shadow.blur_sigma;
    shadow.fColor = txt_shadow.color;
    skia.addShadow(shadow);
  }

  return skia;
}

}  // namespace txt
