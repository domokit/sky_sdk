// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Image decoder backed by the browser's `ImageDecoder`.
abstract class BrowserImageDecoder implements ui.Codec {
  BrowserImageDecoder({
    required this.contentType,
    required this.dataSource,
    required this.debugSource,
  });

  final String contentType;
  final JSAny dataSource;
  final String debugSource;

  @override
  late int frameCount;

  @override
  late int repetitionCount;

  /// Whether this decoder has been disposed of.
  ///
  /// Once this turns true it stays true forever, and this decoder becomes
  /// unusable.
  bool _isDisposed = false;

  @override
  void dispose() {
    _isDisposed = true;

    // This releases all resources, including any currently running decoding work.
    _cachedWebDecoder?.close();
    _cachedWebDecoder = null;
  }

  /// The index of the frame that will be decoded on the next call of [getNextFrame];
  int _nextFrameIndex = 0;

  /// Creating a new decoder is expensive, so we cache the decoder for reuse.
  ///
  // TODO(jacksongardner): Evaluate whether this complexity is necessary.
  // See https://github.com/flutter/flutter/issues/127548
  ImageDecoder? _cachedWebDecoder;

  /// The underlying image decoder used to decode images.
  ///
  /// This value is volatile. It may be closed or become null any time.
  ///
  ///
  /// This is only meant to be used in tests.
  @visibleForTesting
  ImageDecoder? get debugCachedWebDecoder => _cachedWebDecoder;

  Future<void> initialize() async {
    _cachedWebDecoder = await _createWebDecoder();
  }

  Future<ImageDecoder> _createWebDecoder() async {
    try {
      final ImageDecoder webDecoder = ImageDecoder(
        ImageDecoderOptions(
          type: contentType.toJS,
          data: dataSource,

          // Flutter always uses premultiplied alpha when decoding.
          premultiplyAlpha: 'premultiply'.toJS,
          // "default" gives the browser the liberty to convert to display-appropriate
          // color space, typically SRGB, which is what we want.
          colorSpaceConversion: 'default'.toJS,

          // Flutter doesn't give the developer a way to customize this, so if this
          // is an animated image we should prefer the animated track.
          preferAnimation: true.toJS,
        ),
      );

      await promiseToFuture<void>(webDecoder.tracks.ready);

      // Flutter doesn't have an API for progressive loading of images, so we
      // wait until the image is fully decoded.
      // package:js bindings don't work with getters that return a Promise, which
      // is why js_util is used instead.
      await promiseToFuture<void>(getJsProperty(webDecoder, 'completed'));
      frameCount = webDecoder.tracks.selectedTrack!.frameCount.toInt();

      // We coerce the DOM's `repetitionCount` into an int by explicitly
      // handling `infinity`. Note: This will still throw if the DOM returns a
      // `NaN`.
      final double rawRepetitionCount = webDecoder.tracks.selectedTrack!.repetitionCount;
      repetitionCount = rawRepetitionCount == double.infinity ? -1 : rawRepetitionCount.toInt();

      return webDecoder;
    } catch (error) {
      if (domInstanceOfString(error, 'DOMException')) {
        if ((error as DomException).name == DomException.notSupported) {
          throw ImageCodecException(
            "Image file format ($contentType) is not supported by this browser's ImageDecoder API.\n"
            'Image source: $debugSource',
          );
        }
      }
      throw ImageCodecException(
        "Failed to decode image using the browser's ImageDecoder API.\n"
        'Image source: $debugSource\n'
        'Original browser error: $error',
      );
    }
  }

  @override
  Future<ui.FrameInfo> getNextFrame() async {
    if (_isDisposed) {
      throw ImageCodecException(
        'Cannot decode image. The image decoder has been disposed.\n'
        'Image source: $debugSource',
      );
    }
    final ImageDecoder? webDecoder = _cachedWebDecoder;
    if (webDecoder == null) {
      throw ImageCodecException(
        'Cannot decode image. The image decoder has not been initialized.\n'
        'Image source: $debugSource',
      );
    }

    final DecodeResult result = await promiseToFuture<DecodeResult>(
      webDecoder.decode(DecodeOptions(frameIndex: _nextFrameIndex.toJS)),
    );
    final VideoFrame frame = result.image;
    _nextFrameIndex = (_nextFrameIndex + 1) % frameCount;

    // Duration can be null if the image is not animated. However, Flutter
    // requires a non-null value. 0 indicates that the frame is meant to be
    // displayed indefinitely, which is fine for a static image.
    final Duration duration = Duration(microseconds: frame.duration?.toInt() ?? 0);
    final ui.Image image = generateImageFromVideoFrame(frame);
    return AnimatedImageFrameInfo(duration, image);
  }

  /// Creates a [ui.Image] from a [VideoFrame]. Implementers of this class
  /// should override this method to create a [ui.Image] that is appropriate
  /// for their associated renderer.
  ui.Image generateImageFromVideoFrame(VideoFrame frame);
}

/// Data for a single frame of an animated image.
class AnimatedImageFrameInfo implements ui.FrameInfo {
  AnimatedImageFrameInfo(this.duration, this.image);

  @override
  final Duration duration;

  @override
  final ui.Image image;
}

// Wraps another codec and resizes each output image.
class ResizingCodec implements ui.Codec {
  ResizingCodec(this.delegate, {this.targetWidth, this.targetHeight, this.allowUpscaling = true});

  final ui.Codec delegate;
  final int? targetWidth;
  final int? targetHeight;
  final bool allowUpscaling;

  @override
  void dispose() => delegate.dispose();

  @override
  int get frameCount => delegate.frameCount;

  @override
  Future<ui.FrameInfo> getNextFrame() async {
    final ui.FrameInfo frameInfo = await delegate.getNextFrame();
    return AnimatedImageFrameInfo(
      frameInfo.duration,
      scaleImage(
        frameInfo.image,
        targetWidth: targetWidth,
        targetHeight: targetHeight,
        allowUpscaling: allowUpscaling,
      ),
    );
  }

  ui.Image scaleImage(
    ui.Image image, {
    int? targetWidth,
    int? targetHeight,
    bool allowUpscaling = true,
  }) => scaleImageIfNeeded(
    image,
    targetWidth: targetWidth,
    targetHeight: targetHeight,
    allowUpscaling: allowUpscaling,
  );

  @override
  int get repetitionCount => delegate.repetitionCount;
}

BitmapSize? scaledImageSize(int width, int height, int? targetWidth, int? targetHeight) {
  if (targetWidth == width && targetHeight == height) {
    // Not scaled
    return null;
  }
  if (targetWidth == null) {
    if (targetHeight == null || targetHeight == height) {
      // Not scaled.
      return null;
    }
    targetWidth = (width * targetHeight / height).round();
  } else if (targetHeight == null) {
    if (targetWidth == width) {
      // Not scaled.
      return null;
    }
    targetHeight = (height * targetWidth / width).round();
  }
  return BitmapSize(targetWidth, targetHeight);
}

ui.Image scaleImageIfNeeded(
  ui.Image image, {
  int? targetWidth,
  int? targetHeight,
  bool allowUpscaling = true,
}) {
  final int width = image.width;
  final int height = image.height;
  final BitmapSize? scaledSize = scaledImageSize(width, height, targetWidth, targetHeight);
  if (scaledSize == null) {
    return image;
  }
  if (!allowUpscaling && (scaledSize.width > width || scaledSize.height > height)) {
    return image;
  }

  final ui.Rect outputRect = ui.Rect.fromLTWH(
    0,
    0,
    scaledSize.width.toDouble(),
    scaledSize.height.toDouble(),
  );
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder, outputRect);

  canvas.drawImageRect(
    image,
    ui.Rect.fromLTWH(0, 0, width.toDouble(), height.toDouble()),
    outputRect,
    ui.Paint(),
  );
  final ui.Picture picture = recorder.endRecording();
  final ui.Image finalImage = picture.toImageSync(scaledSize.width, scaledSize.height);
  picture.dispose();
  image.dispose();
  return finalImage;
}
