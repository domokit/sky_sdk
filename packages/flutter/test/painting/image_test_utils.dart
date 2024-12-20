// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

class TestImageProvider extends ImageProvider<TestImageProvider> {
  TestImageProvider(this.testImage);

  final ui.Image testImage;

  final Completer<ImageInfo> _completer = Completer<ImageInfo>.sync();
  ImageConfiguration? configuration;
  int loadCallCount = 0;

  @override
  Future<TestImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<TestImageProvider>(this);
  }

  @override
  void resolveStreamForKey(
    ImageConfiguration config,
    ImageStream stream,
    TestImageProvider key,
    ImageErrorListener handleError,
  ) {
    configuration = config;
    super.resolveStreamForKey(config, stream, key, handleError);
  }

  @override
  ImageStreamCompleter loadBuffer(TestImageProvider key, DecoderBufferCallback decode) {
    throw UnsupportedError('Use ImageProvider.loadImage instead.');
  }

  @override
  ImageStreamCompleter loadImage(TestImageProvider key, ImageDecoderCallback decode) {
    loadCallCount += 1;
    return OneFrameImageStreamCompleter(_completer.future);
  }

  ImageInfo complete() {
    final ImageInfo imageInfo = ImageInfo(image: testImage);
    _completer.complete(imageInfo);
    return imageInfo;
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}
