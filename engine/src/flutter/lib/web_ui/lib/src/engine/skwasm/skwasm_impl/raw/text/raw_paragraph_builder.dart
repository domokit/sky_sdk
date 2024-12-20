// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawParagraphBuilder extends Opaque {}

typedef ParagraphBuilderHandle = Pointer<RawParagraphBuilder>;

@Native<ParagraphBuilderHandle Function(ParagraphStyleHandle, FontCollectionHandle)>(
  symbol: 'paragraphBuilder_create',
  isLeaf: true,
)
external ParagraphBuilderHandle paragraphBuilderCreate(
  ParagraphStyleHandle styleHandle,
  FontCollectionHandle fontCollectionHandle,
);

@Native<Void Function(ParagraphBuilderHandle)>(symbol: 'paragraphBuilder_dispose', isLeaf: true)
external void paragraphBuilderDispose(ParagraphBuilderHandle handle);

@Native<Void Function(ParagraphBuilderHandle, Float, Float, Int, Float, Int)>(
  symbol: 'paragraphBuilder_addPlaceholder',
  isLeaf: true,
)
external void paragraphBuilderAddPlaceholder(
  ParagraphBuilderHandle handle,
  double width,
  double height,
  int alignment,
  double baslineOffset,
  int baseline,
);

@Native<Void Function(ParagraphBuilderHandle, SkString16Handle)>(
  symbol: 'paragraphBuilder_addText',
  isLeaf: true,
)
external void paragraphBuilderAddText(ParagraphBuilderHandle handle, SkString16Handle text);

@Native<Pointer<Uint8> Function(ParagraphBuilderHandle, Pointer<Uint32>)>(
  symbol: 'paragraphBuilder_getUtf8Text',
  isLeaf: true,
)
external Pointer<Uint8> paragraphBuilderGetUtf8Text(
  ParagraphBuilderHandle handle,
  Pointer<Uint32> outSize,
);

@Native<Void Function(ParagraphBuilderHandle, TextStyleHandle)>(
  symbol: 'paragraphBuilder_pushStyle',
  isLeaf: true,
)
external void paragraphBuilderPushStyle(ParagraphBuilderHandle handle, TextStyleHandle styleHandle);

@Native<Void Function(ParagraphBuilderHandle)>(symbol: 'paragraphBuilder_pop', isLeaf: true)
external void paragraphBuilderPop(ParagraphBuilderHandle handle);

@Native<ParagraphHandle Function(ParagraphBuilderHandle)>(
  symbol: 'paragraphBuilder_build',
  isLeaf: true,
)
external ParagraphHandle paragraphBuilderBuild(ParagraphBuilderHandle handle);

@Native<UnicodePositionBufferHandle Function(Size)>(
  symbol: 'unicodePositionBuffer_create',
  isLeaf: true,
)
external UnicodePositionBufferHandle unicodePositionBufferCreate(int size);

@Native<Pointer<Uint32> Function(UnicodePositionBufferHandle)>(
  symbol: 'unicodePositionBuffer_getDataPointer',
  isLeaf: true,
)
external Pointer<Uint32> unicodePositionBufferGetDataPointer(UnicodePositionBufferHandle handle);

@Native<Void Function(UnicodePositionBufferHandle)>(
  symbol: 'unicodePositionBuffer_free',
  isLeaf: true,
)
external void unicodePositionBufferFree(UnicodePositionBufferHandle handle);

@Native<LineBreakBufferHandle Function(Size)>(symbol: 'lineBreakBuffer_create', isLeaf: true)
external LineBreakBufferHandle lineBreakBufferCreate(int size);

@Native<Pointer<LineBreak> Function(LineBreakBufferHandle)>(
  symbol: 'lineBreakBuffer_getDataPointer',
  isLeaf: true,
)
external Pointer<LineBreak> lineBreakBufferGetDataPointer(LineBreakBufferHandle handle);

@Native<Void Function(LineBreakBufferHandle)>(symbol: 'lineBreakBuffer_free', isLeaf: true)
external void lineBreakBufferFree(LineBreakBufferHandle handle);

@Native<Void Function(ParagraphBuilderHandle, UnicodePositionBufferHandle)>(
  symbol: 'paragraphBuilder_setGraphemeBreaksUtf16',
  isLeaf: true,
)
external void paragraphBuilderSetGraphemeBreaksUtf16(
  ParagraphBuilderHandle handle,
  UnicodePositionBufferHandle positionBuffer,
);

@Native<Void Function(ParagraphBuilderHandle, UnicodePositionBufferHandle)>(
  symbol: 'paragraphBuilder_setWordBreaksUtf16',
  isLeaf: true,
)
external void paragraphBuilderSetWordBreaksUtf16(
  ParagraphBuilderHandle handle,
  UnicodePositionBufferHandle positionBuffer,
);

@Native<Void Function(ParagraphBuilderHandle, LineBreakBufferHandle)>(
  symbol: 'paragraphBuilder_setLineBreaksUtf16',
  isLeaf: true,
)
external void paragraphBuilderSetLineBreaksUtf16(
  ParagraphBuilderHandle handle,
  LineBreakBufferHandle positionBuffer,
);
