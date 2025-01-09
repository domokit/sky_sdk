// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Uses image codecs supplied by the CanvasKit WASM bundle.
///
/// See also:
///
///  * `image_web_codecs.dart`, which uses the `ImageDecoder` supplied by the browser.
library image_wasm_codecs;

import 'dart:async';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// The CanvasKit implementation of [ui.Codec].
///
/// Wraps `SkAnimatedImage`.
class CkAnimatedImage implements ui.Codec {
  /// Decodes an image from a list of encoded bytes.
  CkAnimatedImage.decodeFromBytes(this._bytes, this.src, {this.targetWidth, this.targetHeight}) {
    final SkAnimatedImage skAnimatedImage = createSkAnimatedImage();
    _ref = UniqueRef<SkAnimatedImage>(this, skAnimatedImage, 'Codec');
  }

  late final UniqueRef<SkAnimatedImage> _ref;
  final String src;
  final Uint8List _bytes;
  int _frameCount = 0;
  int _repetitionCount = -1;

  final int? targetWidth;
  final int? targetHeight;

  SkAnimatedImage createSkAnimatedImage() {
    SkAnimatedImage? animatedImage = canvasKit.MakeAnimatedImageFromEncoded(_bytes);
    if (animatedImage == null) {
      throw ImageCodecException(
        'Failed to decode image data.\n'
        'Image source: $src',
      );
    }

    if (targetWidth != null || targetHeight != null) {
      if (animatedImage.getFrameCount() > 1) {
        printWarning('targetWidth and targetHeight for multi-frame images not supported');
      } else {
        animatedImage = _resizeAnimatedImage(animatedImage, targetWidth, targetHeight);
        if (animatedImage == null) {
          throw ImageCodecException(
            'Failed to decode re-sized image data.\n'
            'Image source: $src',
          );
        }
      }
    }

    _frameCount = animatedImage.getFrameCount().toInt();
    _repetitionCount = animatedImage.getRepetitionCount().toInt();

    return animatedImage;
  }

  SkAnimatedImage? _resizeAnimatedImage(
    SkAnimatedImage animatedImage,
    int? targetWidth,
    int? targetHeight,
  ) {
    final SkImage image = animatedImage.makeImageAtCurrentFrame();
    final CkImage ckImage = scaleImage(image, targetWidth, targetHeight);
    final Uint8List? resizedBytes = ckImage.skImage.encodeToBytes();

    if (resizedBytes == null) {
      throw ImageCodecException('Failed to re-size image');
    }

    final SkAnimatedImage? resizedAnimatedImage = canvasKit.MakeAnimatedImageFromEncoded(
      resizedBytes,
    );
    return resizedAnimatedImage;
  }

  bool _disposed = false;
  bool get debugDisposed => _disposed;

  bool _debugCheckIsNotDisposed() {
    assert(!_disposed, 'This image has been disposed.');
    return true;
  }

  @override
  void dispose() {
    assert(!_disposed, 'Cannot dispose a codec that has already been disposed.');
    _disposed = true;
    _ref.dispose();
  }

  @override
  int get frameCount {
    assert(_debugCheckIsNotDisposed());
    return _frameCount;
  }

  @override
  int get repetitionCount {
    assert(_debugCheckIsNotDisposed());
    return _repetitionCount;
  }

  @override
  Future<ui.FrameInfo> getNextFrame() {
    assert(_debugCheckIsNotDisposed());
    final SkAnimatedImage animatedImage = _ref.nativeObject;

    // SkAnimatedImage comes pre-initialized to point to the current frame (by
    // default the first frame, and, with some special resurrection logic in
    // `createDefault`, to a subsequent frame if resurrection happens in the
    // middle of animation). Flutter's `Codec` semantics is to initialize to
    // point to "just before the first frame", i.e. the first invocation of
    // `getNextFrame` returns the first frame. Therefore, we have to read the
    // current Skia frame, then advance SkAnimatedImage to the next frame, and
    // return the current frame.
    final ui.FrameInfo currentFrame = AnimatedImageFrameInfo(
      Duration(milliseconds: animatedImage.currentFrameDuration().toInt()),
      CkImage(animatedImage.makeImageAtCurrentFrame()),
    );

    animatedImage.decodeNextFrame();
    return Future<ui.FrameInfo>.value(currentFrame);
  }
}
