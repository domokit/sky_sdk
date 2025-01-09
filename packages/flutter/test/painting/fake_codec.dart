// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show Codec, FrameInfo, instantiateImageCodec;

import 'package:flutter/foundation.dart';

/// A [ui.Codec] implementation for testing that pre-fetches all the image
/// frames, and provides synchronous [getNextFrame] implementation.
///
/// This is useful for running in the test Zone, where it is tricky to receive
/// callbacks originating from the IO thread.
class FakeCodec implements ui.Codec {
  FakeCodec._(this._frameCount, this._repetitionCount, this._frameInfos);

  final int _frameCount;
  final int _repetitionCount;
  final List<ui.FrameInfo> _frameInfos;
  int _nextFrame = 0;
  int _numFramesAsked = 0;

  /// Creates a FakeCodec from encoded image data.
  ///
  /// Only call this method outside of the test zone.
  static Future<FakeCodec> fromData(Uint8List data) async {
    final ui.Codec codec = await ui.instantiateImageCodec(data);
    final int frameCount = codec.frameCount;
    final List<ui.FrameInfo> frameInfos = <ui.FrameInfo>[];
    for (int i = 0; i < frameCount; i += 1) {
      frameInfos.add(await codec.getNextFrame());
    }
    return FakeCodec._(frameCount, codec.repetitionCount, frameInfos);
  }

  @override
  int get frameCount => _frameCount;

  @override
  int get repetitionCount => _repetitionCount;

  int get numFramesAsked => _numFramesAsked;

  @override
  Future<ui.FrameInfo> getNextFrame() {
    _numFramesAsked += 1;
    final Future<ui.FrameInfo> result = Future<ui.FrameInfo>.value(_frameInfos[_nextFrame]);
    _nextFrame = (_nextFrame + 1) % _frameCount;
    return result;
  }

  @override
  void dispose() {}
}
