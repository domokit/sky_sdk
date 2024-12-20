// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/painting.dart';
library;

import 'dart:ui';

/// Returns a [Codec] that throws on all member invocations.
Codec createNoOpCodec() => _NoOpCodec();

/// Function matching [DecoderBufferCallback] which returns a [Codec]
/// that throws on all member invocations.
Future<Codec> noOpDecoderBufferCallback(
  ImmutableBuffer buffer, {
  int? cacheWidth,
  int? cacheHeight,
  bool? allowUpscaling,
}) async => _NoOpCodec();

class _NoOpCodec implements Codec {
  @override
  void dispose() {}

  @override
  int get frameCount => throw UnimplementedError();

  @override
  Future<FrameInfo> getNextFrame() => throw UnimplementedError();

  @override
  int get repetitionCount => throw UnimplementedError();
}
