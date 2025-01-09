// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawParagraph extends Opaque {}

typedef ParagraphHandle = Pointer<RawParagraph>;

final class RawTextBoxList extends Opaque {}

typedef TextBoxListHandle = Pointer<RawTextBoxList>;

final class RawUnicodePositionBuffer extends Opaque {}

typedef UnicodePositionBufferHandle = Pointer<RawUnicodePositionBuffer>;

final class RawLineBreakBuffer extends Opaque {}

typedef LineBreakBufferHandle = Pointer<RawLineBreakBuffer>;

final class LineBreak extends Struct {
  @Int32()
  external int position;

  @Int32()
  external int lineBreakType;
}

@Native<Void Function(ParagraphHandle)>(symbol: 'paragraph_dispose', isLeaf: true)
external void paragraphDispose(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getWidth', isLeaf: true)
external double paragraphGetWidth(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getHeight', isLeaf: true)
external double paragraphGetHeight(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getLongestLine', isLeaf: true)
external double paragraphGetLongestLine(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getMinIntrinsicWidth', isLeaf: true)
external double paragraphGetMinIntrinsicWidth(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getMaxIntrinsicWidth', isLeaf: true)
external double paragraphGetMaxIntrinsicWidth(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getAlphabeticBaseline', isLeaf: true)
external double paragraphGetAlphabeticBaseline(ParagraphHandle handle);

@Native<Float Function(ParagraphHandle)>(symbol: 'paragraph_getIdeographicBaseline', isLeaf: true)
external double paragraphGetIdeographicBaseline(ParagraphHandle handle);

@Native<Bool Function(ParagraphHandle)>(symbol: 'paragraph_getDidExceedMaxLines', isLeaf: true)
external bool paragraphGetDidExceedMaxLines(ParagraphHandle handle);

@Native<Void Function(ParagraphHandle, Float)>(symbol: 'paragraph_layout', isLeaf: true)
external void paragraphLayout(ParagraphHandle handle, double width);

@Native<Int32 Function(ParagraphHandle, Float, Float, Pointer<Int32>)>(
  symbol: 'paragraph_getPositionForOffset',
  isLeaf: true,
)
external int paragraphGetPositionForOffset(
  ParagraphHandle handle,
  double offsetX,
  double offsetY,
  Pointer<Int32> outAffinity,
);

@Native<Bool Function(ParagraphHandle, Float, Float, RawRect, Pointer<Uint32>, Pointer<Bool>)>(
  symbol: 'paragraph_getClosestGlyphInfoAtCoordinate',
)
external bool paragraphGetClosestGlyphInfoAtCoordinate(
  ParagraphHandle handle,
  double offsetX,
  double offsetY,
  RawRect graphemeLayoutBounds, // 4 floats, [LTRB]
  Pointer<Uint32> graphemeCodeUnitRange, // 2 `size_t`s, start and end.
  Pointer<Bool> booleanFlags, // 1 boolean, isLTR.
);

@Native<Bool Function(ParagraphHandle, Uint32, RawRect, Pointer<Uint32>, Pointer<Bool>)>(
  symbol: 'paragraph_getGlyphInfoAt',
)
external bool paragraphGetGlyphInfoAt(
  ParagraphHandle handle,
  int codeUnitOffset,
  RawRect graphemeLayoutBounds, // 4 floats, [LTRB]
  Pointer<Uint32> graphemeCodeUnitRange, // 2 `size_t`s, start and end.
  Pointer<Bool> booleanFlags, // 1 boolean, isLTR.
);

@Native<Void Function(ParagraphHandle, UnsignedInt, Pointer<Int32>)>(
  symbol: 'paragraph_getWordBoundary',
  isLeaf: true,
)
external void paragraphGetWordBoundary(
  ParagraphHandle handle,
  int position,
  Pointer<Int32> outRange, // Two `size_t`s, start and end
);

@Native<Size Function(ParagraphHandle)>(symbol: 'paragraph_getLineCount', isLeaf: true)
external int paragraphGetLineCount(ParagraphHandle handle);

@Native<Int Function(ParagraphHandle, Size)>(symbol: 'paragraph_getLineNumberAt', isLeaf: true)
external int paragraphGetLineNumberAt(ParagraphHandle handle, int characterIndex);

@Native<LineMetricsHandle Function(ParagraphHandle, Size)>(
  symbol: 'paragraph_getLineMetricsAtIndex',
  isLeaf: true,
)
external LineMetricsHandle paragraphGetLineMetricsAtIndex(ParagraphHandle handle, int index);

@Native<Void Function(TextBoxListHandle)>(symbol: 'textBoxList_dispose', isLeaf: true)
external void textBoxListDispose(TextBoxListHandle handle);

@Native<Size Function(TextBoxListHandle)>(symbol: 'textBoxList_getLength', isLeaf: true)
external int textBoxListGetLength(TextBoxListHandle handle);

@Native<Int Function(TextBoxListHandle, Size, RawRect)>(
  symbol: 'textBoxList_getBoxAtIndex',
  isLeaf: true,
)
external int textBoxListGetBoxAtIndex(TextBoxListHandle handle, int index, RawRect outRect);

@Native<TextBoxListHandle Function(ParagraphHandle, Int, Int, Int, Int)>(
  symbol: 'paragraph_getBoxesForRange',
  isLeaf: true,
)
external TextBoxListHandle paragraphGetBoxesForRange(
  ParagraphHandle handle,
  int start,
  int end,
  int heightStyle,
  int widthStyle,
);

@Native<TextBoxListHandle Function(ParagraphHandle)>(
  symbol: 'paragraph_getBoxesForPlaceholders',
  isLeaf: true,
)
external TextBoxListHandle paragraphGetBoxesForPlaceholders(ParagraphHandle handle);

// Returns a list of the code points that were unable to be rendered with the
// selected fonts. The list is deduplicated, so each code point in the output
// is unique.
// If `nullptr` is passed in for `outCodePoints`, we simply return the count
// of the code points.
// Note: This must be called after the paragraph has been laid out at least
// once in order to get valid data.
@Native<Int Function(ParagraphHandle, Pointer<Uint32>, Int)>(
  symbol: 'paragraph_getUnresolvedCodePoints',
  isLeaf: true,
)
external int paragraphGetUnresolvedCodePoints(
  ParagraphHandle handle,
  Pointer<Uint32> outCodePoints,
  int outLength,
);
