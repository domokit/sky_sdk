// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "../export.h"
#include "../wrappers.h"
#include "third_party/skia/modules/skparagraph/include/Paragraph.h"

const double kTextHeightNone = 0.0;

using namespace skia::textlayout;
using namespace Skwasm;

SKWASM_EXPORT TextStyle* textStyle_create() {
  auto style = new TextStyle();

  // Default color in flutter is black.
  style->setColor(SK_ColorBLACK);
  return style;
}

SKWASM_EXPORT TextStyle* textStyle_copy(TextStyle* style) {
  return new TextStyle(*style);
}

SKWASM_EXPORT void textStyle_dispose(TextStyle* style) {
  delete style;
}

SKWASM_EXPORT void textStyle_setColor(TextStyle* style, SkColor color) {
  style->setColor(color);
}

SKWASM_EXPORT void textStyle_setDecoration(TextStyle* style,
                                           TextDecoration decoration) {
  style->setDecoration(decoration);
}

SKWASM_EXPORT void textStyle_setDecorationColor(TextStyle* style,
                                                SkColor color) {
  style->setDecorationColor(color);
}

SKWASM_EXPORT void textStyle_setDecorationStyle(
    TextStyle* style,
    TextDecorationStyle decorationStyle) {
  style->setDecorationStyle(decorationStyle);
}

SKWASM_EXPORT void textStyle_setDecorationThickness(TextStyle* style,
                                                    SkScalar thickness) {
  style->setDecorationThicknessMultiplier(thickness);
}

SKWASM_EXPORT void textStyle_setFontStyle(TextStyle* style,
                                          int weight,
                                          SkFontStyle::Slant slant) {
  style->setFontStyle(SkFontStyle(weight, SkFontStyle::kNormal_Width, slant));
}

SKWASM_EXPORT void textStyle_setTextBaseline(TextStyle* style,
                                             TextBaseline baseline) {
  style->setTextBaseline(baseline);
}

SKWASM_EXPORT void textStyle_clearFontFamilies(TextStyle* style) {
  style->setFontFamilies({});
}

SKWASM_EXPORT void textStyle_addFontFamilies(TextStyle* style,
                                             SkString** fontFamilies,
                                             int count) {
  const std::vector<SkString>& currentFamilies = style->getFontFamilies();
  std::vector<SkString> newFamilies;
  newFamilies.reserve(currentFamilies.size() + count);
  for (int i = 0; i < count; i++) {
    newFamilies.push_back(*fontFamilies[i]);
  }
  for (const auto& family : currentFamilies) {
    newFamilies.push_back(family);
  }
  style->setFontFamilies(std::move(newFamilies));
}

SKWASM_EXPORT void textStyle_setFontSize(TextStyle* style, SkScalar size) {
  style->setFontSize(size);
}

SKWASM_EXPORT void textStyle_setLetterSpacing(TextStyle* style,
                                              SkScalar letterSpacing) {
  style->setLetterSpacing(letterSpacing);
}

SKWASM_EXPORT void textStyle_setWordSpacing(TextStyle* style,
                                            SkScalar wordSpacing) {
  style->setWordSpacing(wordSpacing);
}

SKWASM_EXPORT void textStyle_setHeight(TextStyle* style, SkScalar height) {
  style->setHeight(height);
  style->setHeightOverride(height != kTextHeightNone);
}

SKWASM_EXPORT void textStyle_setHalfLeading(TextStyle* style,
                                            bool halfLeading) {
  style->setHalfLeading(halfLeading);
}

SKWASM_EXPORT void textStyle_setLocale(TextStyle* style, SkString* locale) {
  style->setLocale(*locale);
}

SKWASM_EXPORT void textStyle_setBackground(TextStyle* style, SkPaint* paint) {
  style->setBackgroundColor(*paint);
}

SKWASM_EXPORT void textStyle_setForeground(TextStyle* style, SkPaint* paint) {
  style->setForegroundColor(*paint);
}

SKWASM_EXPORT void textStyle_addShadow(TextStyle* style,
                                       SkColor color,
                                       SkScalar offsetX,
                                       SkScalar offsetY,
                                       SkScalar blurSigma) {
  style->addShadow(TextShadow(color, {offsetX, offsetY}, blurSigma));
}

SKWASM_EXPORT void textStyle_addFontFeature(TextStyle* style,
                                            SkString* featureName,
                                            int value) {
  style->addFontFeature(*featureName, value);
}

SKWASM_EXPORT void textStyle_setFontVariations(TextStyle* style,
                                               SkFourByteTag* axes,
                                               float* values,
                                               int count) {
  std::vector<SkFontArguments::VariationPosition::Coordinate> coordinates;
  for (int i = 0; i < count; i++) {
    coordinates.push_back({axes[i], values[i]});
  }
  SkFontArguments::VariationPosition position = {
      coordinates.data(), static_cast<int>(coordinates.size())};
  style->setFontArguments(
      SkFontArguments().setVariationDesignPosition(position));
}
