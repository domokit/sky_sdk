/*
 * Copyright 2017 Google, Inc.
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

#include <sstream>

#include "flutter/fml/command_line.h"
#include "flutter/fml/logging.h"
#include "flutter/third_party/txt/tests/txt_test_utils.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"
#include "third_party/icu/source/common/unicode/unistr.h"
#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkColor.h"
#include "third_party/skia/modules/skparagraph/include/Paragraph.h"
#include "third_party/skia/modules/skparagraph/include/ParagraphBuilder.h"
#include "third_party/skia/modules/skparagraph/include/TypefaceFontProvider.h"
#include "third_party/skia/modules/skparagraph/utils/TestFontCollection.h"
#include "third_party/skia/modules/skunicode/include/SkUnicode_icu.h"

namespace sktxt = skia::textlayout;

class SkParagraphFixture : public benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State& state) {
    font_collection_ = sk_make_sp<sktxt::TestFontCollection>(txt::GetFontDir());

    bitmap_ = std::make_unique<SkBitmap>();
    bitmap_->allocN32Pixels(1000, 1000);
    canvas_ = std::make_unique<SkCanvas>(*bitmap_);
    canvas_->clear(SK_ColorWHITE);
  }

 protected:
  sk_sp<sktxt::TestFontCollection> font_collection_;
  std::unique_ptr<SkCanvas> canvas_;
  std::unique_ptr<SkBitmap> bitmap_;
};

BENCHMARK_F(SkParagraphFixture, ShortLayout)(benchmark::State& state) {
  const char* text = "Hello World";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(text);
  builder->pop();
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
}

BENCHMARK_F(SkParagraphFixture, LongLayout)(benchmark::State& state) {
  const char* text =
      "This is a very long sentence to test if the text will properly wrap "
      "around and go to the next line. Sometimes, short sentence. Longer "
      "sentences are okay too because they are necessary. Very short. "
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod "
      "tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim "
      "veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea "
      "commodo consequat. Duis aute irure dolor in reprehenderit in voluptate "
      "velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint "
      "occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
      "mollit anim id est laborum. "
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod "
      "tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim "
      "veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea "
      "commodo consequat. Duis aute irure dolor in reprehenderit in voluptate "
      "velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint "
      "occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
      "mollit anim id est laborum.";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(text);
  builder->pop();
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
}

BENCHMARK_F(SkParagraphFixture, JustifyLayout)(benchmark::State& state) {
  const char* text =
      "This is a very long sentence to test if the text will properly wrap "
      "around and go to the next line. Sometimes, short sentence. Longer "
      "sentences are okay too because they are necessary. Very short. "
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod "
      "tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim "
      "veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea "
      "commodo consequat. Duis aute irure dolor in reprehenderit in voluptate "
      "velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint "
      "occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
      "mollit anim id est laborum. "
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod "
      "tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim "
      "veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea "
      "commodo consequat. Duis aute irure dolor in reprehenderit in voluptate "
      "velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint "
      "occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
      "mollit anim id est laborum.";
  sktxt::ParagraphStyle paragraph_style;
  paragraph_style.setTextAlign(sktxt::TextAlign::kJustify);
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(text);
  builder->pop();
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
}

BENCHMARK_F(SkParagraphFixture, ManyStylesLayout)(benchmark::State& state) {
  const char* text = "-";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  for (int i = 0; i < 1000; ++i) {
    builder->pushStyle(text_style);
    builder->addText(text);
  }
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
}

BENCHMARK_DEFINE_F(SkParagraphFixture, TextBigO)(benchmark::State& state) {
  std::vector<uint16_t> text;
  for (uint16_t i = 0; i < state.range(0); ++i) {
    text.push_back(i % 5 == 0 ? ' ' : i);
  }
  std::u16string u16_text(text.data(), text.data() + text.size());
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(u16_text);
  builder->pop();
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK_REGISTER_F(SkParagraphFixture, TextBigO)
    ->RangeMultiplier(4)
    ->Range(1 << 6, 1 << 14)
    ->Complexity(benchmark::oN);

BENCHMARK_DEFINE_F(SkParagraphFixture, StylesBigO)(benchmark::State& state) {
  const char* text = "vry shrt ";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder = sktxt::ParagraphBuilder::make(
      paragraph_style,
      sk_make_sp<sktxt::TestFontCollection>(txt::GetFontDir()),
      SkUnicodes::ICU::Make());
  for (int i = 0; i < 1000; ++i) {
    builder->pushStyle(text_style);
    builder->addText(text);
  }
  auto paragraph = builder->Build();
  while (state.KeepRunning()) {
    paragraph->markDirty();
    paragraph->layout(300);
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK_REGISTER_F(SkParagraphFixture, StylesBigO)
    ->RangeMultiplier(4)
    ->Range(1 << 3, 1 << 12)
    ->Complexity(benchmark::oN);

BENCHMARK_F(SkParagraphFixture, PaintSimple)(benchmark::State& state) {
  const char* text = "This is a simple sentence to test drawing.";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(text);
  builder->pop();
  auto paragraph = builder->Build();
  paragraph->layout(300);
  int offset = 0;
  while (state.KeepRunning()) {
    paragraph->paint(canvas_.get(), offset % 700, 10);
    offset++;
  }
}

BENCHMARK_F(SkParagraphFixture, PaintLarge)(benchmark::State& state) {
  const char* text =
      "Hello world! This is a simple sentence to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.Hello world! This is a simple sentence "
      "to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.Hello world! This is a simple sentence "
      "to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.Hello world! This is a simple sentence "
      "to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.Hello world! This is a simple sentence "
      "to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.Hello world! This is a simple sentence "
      "to test drawing. Hello world! "
      "This is a simple sentence to test drawing. Hello world! This is a "
      "simple sentence to test drawing.";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  builder->pushStyle(text_style);
  builder->addText(text);
  builder->pop();
  auto paragraph = builder->Build();
  paragraph->layout(300);
  int offset = 0;
  while (state.KeepRunning()) {
    paragraph->paint(canvas_.get(), offset % 700, 10);
    offset++;
  }
}

BENCHMARK_F(SkParagraphFixture, PaintDecoration)(benchmark::State& state) {
  const char* text =
      "Hello world! This is a simple sentence to test drawing. Hello world! "
      "This is a simple sentence to test drawing.";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  text_style.setDecoration(static_cast<sktxt::TextDecoration>(
      sktxt::TextDecoration::kLineThrough | sktxt::TextDecoration::kOverline |
      sktxt::TextDecoration::kUnderline));
  auto builder =
      sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
  text_style.setDecorationStyle(sktxt::TextDecorationStyle::kSolid);
  builder->pushStyle(text_style);
  builder->addText(text);

  text_style.setDecorationStyle(sktxt::TextDecorationStyle::kDotted);
  builder->pushStyle(text_style);
  builder->addText(text);

  text_style.setDecorationStyle(sktxt::TextDecorationStyle::kWavy);
  builder->pushStyle(text_style);
  builder->addText(text);

  auto paragraph = builder->Build();
  paragraph->layout(300);
  int offset = 0;
  while (state.KeepRunning()) {
    paragraph->paint(canvas_.get(), offset % 700, 10);
    offset++;
  }
}

BENCHMARK_F(SkParagraphFixture, SimpleBuilder)(benchmark::State& state) {
  const char* text = "Hello World";
  sktxt::ParagraphStyle paragraph_style;
  sktxt::TextStyle text_style;
  text_style.setFontFamilies({SkString("Roboto")});
  text_style.setColor(SK_ColorBLACK);
  while (state.KeepRunning()) {
    auto builder =
        sktxt::ParagraphBuilder::make(paragraph_style, font_collection_, SkUnicodes::ICU::Make());
    builder->pushStyle(text_style);
    builder->addText(text);
    builder->pop();
    auto paragraph = builder->Build();
  }
}
