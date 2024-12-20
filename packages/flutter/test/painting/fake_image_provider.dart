// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show Codec;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

/// An image provider implementation for testing that is using a [ui.Codec]
/// that it was given at construction time (typically the job of real image
/// providers is to resolve some data and instantiate a [ui.Codec] from it).
class FakeImageProvider extends ImageProvider<FakeImageProvider> {
  const FakeImageProvider(this._codec, {this.scale = 1.0});

  final ui.Codec _codec;

  /// The scale to place in the [ImageInfo] object of the image.
  final double scale;

  @override
  Future<FakeImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<FakeImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(FakeImageProvider key, ImageDecoderCallback decode) {
    assert(key == this);
    return MultiFrameImageStreamCompleter(codec: SynchronousFuture<ui.Codec>(_codec), scale: scale);
  }
}
