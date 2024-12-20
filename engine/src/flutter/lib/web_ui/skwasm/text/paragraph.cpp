// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "third_party/skia/modules/skparagraph/include/Paragraph.h"
#include "../export.h"
#include "DartTypes.h"
#include "TextStyle.h"
#include "include/core/SkScalar.h"

using namespace skia::textlayout;

SKWASM_EXPORT void paragraph_dispose(Paragraph* paragraph) {
  delete paragraph;
}

SKWASM_EXPORT SkScalar paragraph_getWidth(Paragraph* paragraph) {
  return paragraph->getMaxWidth();
}

SKWASM_EXPORT SkScalar paragraph_getHeight(Paragraph* paragraph) {
  return paragraph->getHeight();
}

SKWASM_EXPORT SkScalar paragraph_getLongestLine(Paragraph* paragraph) {
  return paragraph->getLongestLine();
}

SKWASM_EXPORT SkScalar paragraph_getMinIntrinsicWidth(Paragraph* paragraph) {
  return paragraph->getMinIntrinsicWidth();
}

SKWASM_EXPORT SkScalar paragraph_getMaxIntrinsicWidth(Paragraph* paragraph) {
  return paragraph->getMaxIntrinsicWidth();
}

SKWASM_EXPORT SkScalar paragraph_getAlphabeticBaseline(Paragraph* paragraph) {
  return paragraph->getAlphabeticBaseline();
}

SKWASM_EXPORT SkScalar paragraph_getIdeographicBaseline(Paragraph* paragraph) {
  return paragraph->getIdeographicBaseline();
}

SKWASM_EXPORT bool paragraph_getDidExceedMaxLines(Paragraph* paragraph) {
  return paragraph->didExceedMaxLines();
}

SKWASM_EXPORT void paragraph_layout(Paragraph* paragraph, SkScalar width) {
  paragraph->layout(width);
}

SKWASM_EXPORT int32_t paragraph_getPositionForOffset(Paragraph* paragraph,
                                                     SkScalar offsetX,
                                                     SkScalar offsetY,
                                                     Affinity* outAffinity) {
  auto position = paragraph->getGlyphPositionAtCoordinate(offsetX, offsetY);
  if (outAffinity) {
    *outAffinity = position.affinity;
  }
  return position.position;
}

SKWASM_EXPORT bool paragraph_getClosestGlyphInfoAtCoordinate(
    Paragraph* paragraph,
    SkScalar offsetX,
    SkScalar offsetY,
    // Out parameters:
    SkRect* graphemeLayoutBounds,   // 1 SkRect
    size_t* graphemeCodeUnitRange,  // 2 size_ts: [start, end]
    bool* booleanFlags) {           // 1 boolean: isLTR
  Paragraph::GlyphInfo glyphInfo;
  if (!paragraph->getClosestUTF16GlyphInfoAt(offsetX, offsetY, &glyphInfo)) {
    return false;
  }
  // This is more verbose than memcpying the whole struct but ideally we don't
  // want to depend on the exact memory layout of the struct.
  std::memcpy(graphemeLayoutBounds, &glyphInfo.fGraphemeLayoutBounds,
              sizeof(SkRect));
  std::memcpy(graphemeCodeUnitRange, &glyphInfo.fGraphemeClusterTextRange,
              2 * sizeof(size_t));
  booleanFlags[0] =
      glyphInfo.fDirection == skia::textlayout::TextDirection::kLtr;
  return true;
}

SKWASM_EXPORT bool paragraph_getGlyphInfoAt(
    Paragraph* paragraph,
    size_t index,
    // Out parameters:
    SkRect* graphemeLayoutBounds,   // 1 SkRect
    size_t* graphemeCodeUnitRange,  // 2 size_ts: [start, end]
    bool* booleanFlags) {           // 1 boolean: isLTR
  Paragraph::GlyphInfo glyphInfo;
  if (!paragraph->getGlyphInfoAtUTF16Offset(index, &glyphInfo)) {
    return false;
  }
  std::memcpy(graphemeLayoutBounds, &glyphInfo.fGraphemeLayoutBounds,
              sizeof(SkRect));
  std::memcpy(graphemeCodeUnitRange, &glyphInfo.fGraphemeClusterTextRange,
              2 * sizeof(size_t));
  booleanFlags[0] =
      glyphInfo.fDirection == skia::textlayout::TextDirection::kLtr;
  return true;
}

SKWASM_EXPORT void paragraph_getWordBoundary(
    Paragraph* paragraph,
    unsigned int position,
    int32_t* outRange  // Two `int32_t`s, start and end
) {
  auto range = paragraph->getWordBoundary(position);
  outRange[0] = range.start;
  outRange[1] = range.end;
}

SKWASM_EXPORT size_t paragraph_getLineCount(Paragraph* paragraph) {
  return paragraph->lineNumber();
}

SKWASM_EXPORT int paragraph_getLineNumberAt(Paragraph* paragraph,
                                            size_t characterIndex) {
  return paragraph->getLineNumberAtUTF16Offset(characterIndex);
}

SKWASM_EXPORT LineMetrics* paragraph_getLineMetricsAtIndex(Paragraph* paragraph,
                                                           size_t lineNumber) {
  auto metrics = new LineMetrics();
  if (paragraph->getLineMetricsAt(lineNumber, metrics)) {
    return metrics;
  } else {
    delete metrics;
    return nullptr;
  }
}

struct TextBoxList {
  std::vector<TextBox> boxes;
};

SKWASM_EXPORT void textBoxList_dispose(TextBoxList* list) {
  delete list;
}

SKWASM_EXPORT size_t textBoxList_getLength(TextBoxList* list) {
  return list->boxes.size();
}

SKWASM_EXPORT TextDirection textBoxList_getBoxAtIndex(TextBoxList* list,
                                                      size_t index,
                                                      SkRect* outRect) {
  const auto& box = list->boxes[index];
  *outRect = box.rect;
  return box.direction;
}

SKWASM_EXPORT TextBoxList* paragraph_getBoxesForRange(
    Paragraph* paragraph,
    int start,
    int end,
    RectHeightStyle heightStyle,
    RectWidthStyle widthStyle) {
  return new TextBoxList{
      paragraph->getRectsForRange(start, end, heightStyle, widthStyle)};
}

SKWASM_EXPORT TextBoxList* paragraph_getBoxesForPlaceholders(
    Paragraph* paragraph) {
  return new TextBoxList{paragraph->getRectsForPlaceholders()};
}

// Returns a list of the code points that were unable to be rendered with the
// selected fonts. The list is deduplicated, so each code point in the output
// is unique.
// If `nullptr` is passed in for `outCodePoints`, we simply return the count
// of the code points.
// Note: This must be called after the paragraph has been laid out at least
// once in order to get valid data.
SKWASM_EXPORT int paragraph_getUnresolvedCodePoints(Paragraph* paragraph,
                                                    SkUnichar* outCodePoints,
                                                    int outLength) {
  if (!outCodePoints) {
    return paragraph->unresolvedCodepoints().size();
  }
  int outIndex = 0;
  for (SkUnichar character : paragraph->unresolvedCodepoints()) {
    if (outIndex < outLength) {
      outCodePoints[outIndex] = character;
      outIndex++;
    } else {
      break;
    }
  }
  return outIndex;
}
