// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "../export.h"
#include "../wrappers.h"
#include "third_party/skia/modules/skparagraph/include/Paragraph.h"

using namespace skia::textlayout;
using namespace Skwasm;

SKWASM_EXPORT ParagraphStyle* paragraphStyle_create() {
  auto style = new ParagraphStyle();

  // This is the default behavior in Flutter
  style->setReplaceTabCharacters(true);

  // Default text style has a black color
  TextStyle textStyle;
  textStyle.setColor(SK_ColorBLACK);
  style->setTextStyle(textStyle);

  return style;
}

SKWASM_EXPORT void paragraphStyle_dispose(ParagraphStyle* style) {
  delete style;
}

SKWASM_EXPORT void paragraphStyle_setTextAlign(ParagraphStyle* style,
                                               TextAlign align) {
  style->setTextAlign(align);
}

SKWASM_EXPORT void paragraphStyle_setTextDirection(ParagraphStyle* style,
                                                   TextDirection direction) {
  style->setTextDirection(direction);
}

SKWASM_EXPORT void paragraphStyle_setMaxLines(ParagraphStyle* style,
                                              size_t maxLines) {
  style->setMaxLines(maxLines);
}

SKWASM_EXPORT void paragraphStyle_setHeight(ParagraphStyle* style,
                                            SkScalar height) {
  style->setHeight(height);
}

SKWASM_EXPORT void paragraphStyle_setTextHeightBehavior(
    ParagraphStyle* style,
    bool applyHeightToFirstAscent,
    bool applyHeightToLastDescent) {
  TextHeightBehavior behavior;
  if (!applyHeightToFirstAscent && !applyHeightToLastDescent) {
    behavior = kDisableAll;
  } else if (!applyHeightToLastDescent) {
    behavior = kDisableLastDescent;
  } else if (!applyHeightToFirstAscent) {
    behavior = kDisableFirstAscent;
  } else {
    behavior = kAll;
  }
  style->setTextHeightBehavior(behavior);
}

SKWASM_EXPORT void paragraphStyle_setEllipsis(ParagraphStyle* style,
                                              SkString* ellipsis) {
  style->setEllipsis(*ellipsis);
}

SKWASM_EXPORT void paragraphStyle_setStrutStyle(ParagraphStyle* style,
                                                StrutStyle* strutStyle) {
  style->setStrutStyle(*strutStyle);
}

SKWASM_EXPORT void paragraphStyle_setTextStyle(ParagraphStyle* style,
                                               TextStyle* textStyle) {
  style->setTextStyle(*textStyle);
}

SKWASM_EXPORT void paragraphStyle_setApplyRoundingHack(ParagraphStyle* style,
                                                       bool applyRoundingHack) {
  style->setApplyRoundingHack(applyRoundingHack);
}
