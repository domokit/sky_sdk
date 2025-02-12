// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
library;

import 'dart:typed_data';
import 'dart:ui' as ui show Codec, FrameInfo, Image, ImmutableBuffer;

import 'binding.dart';

/// Creates an image from a list of bytes.
///
/// This function attempts to interpret the given bytes an image. If successful,
/// the returned [Future] resolves to the decoded image. Otherwise, the [Future]
/// resolves to null.
///
/// If the image is animated, this returns the first frame. Consider
/// [instantiateImageCodec] if support for animated images is necessary.
///
/// This function differs from [decodeImageFromList] in that it defers to
/// [PaintingBinding.instantiateImageCodecWithSize], and therefore can be mocked
/// in tests.
Future<ui.Image> decodeImageFromList(Uint8List bytes) async {
  final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(bytes);
  final ui.Codec codec = await PaintingBinding.instance.instantiateImageCodecWithSize(buffer);
  final ui.FrameInfo frameInfo;
  try {
    frameInfo = await codec.getNextFrame();
  } finally {
    codec.dispose();
  }
  return frameInfo.image;
}
