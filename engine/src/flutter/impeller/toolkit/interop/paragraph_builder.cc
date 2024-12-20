// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/paragraph_builder.h"

#include "flutter/third_party/txt/src/skia/paragraph_builder_skia.h"
#include "impeller/base/validation.h"
#include "impeller/toolkit/interop/paragraph.h"

namespace impeller::interop {

ParagraphBuilder::ParagraphBuilder(ScopedObject<TypographyContext> context)
    : context_(std::move(context)) {}

ParagraphBuilder::~ParagraphBuilder() = default;

bool ParagraphBuilder::IsValid() const {
  return !!context_;
}

void ParagraphBuilder::PushStyle(const ParagraphStyle& style) {
  GetBuilder(style.GetParagraphStyle())->PushStyle(style.CreateTextStyle());
}

void ParagraphBuilder::PopStyle() {
  GetBuilder()->Pop();
}

void ParagraphBuilder::AddText(const uint8_t* data, size_t byte_length) {
  GetBuilder()->AddText(data, byte_length);
}

ScopedObject<Paragraph> ParagraphBuilder::Build(Scalar width) const {
  auto txt_paragraph = GetBuilder()->Build();
  if (!txt_paragraph) {
    return nullptr;
  }
  txt_paragraph->Layout(width);
  return Create<Paragraph>(std::move(txt_paragraph));
}

const std::unique_ptr<txt::ParagraphBuilder>& ParagraphBuilder::GetBuilder(
    const txt::ParagraphStyle& style) const {
  if (lazy_builder_) {
    return lazy_builder_;
  }
  lazy_builder_ = std::make_unique<txt::ParagraphBuilderSkia>(
      style,                          //
      context_->GetFontCollection(),  //
      true                            // is impeller enabled
  );
  return lazy_builder_;
}

const std::unique_ptr<txt::ParagraphBuilder>& ParagraphBuilder::GetBuilder()
    const {
  static txt::ParagraphStyle kDefaultStyle;
  return GetBuilder(kDefaultStyle);
}

}  // namespace impeller::interop
