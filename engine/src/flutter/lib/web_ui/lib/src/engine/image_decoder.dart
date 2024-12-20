// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

Duration _kDefaultWebDecoderExpireDuration = const Duration(seconds: 3);
Duration _kWebDecoderExpireDuration = _kDefaultWebDecoderExpireDuration;

/// Overrides the inactivity duration after which the web decoder is closed.
///
/// This should only be used in tests.
void debugOverrideWebDecoderExpireDuration(Duration override) {
  _kWebDecoderExpireDuration = override;
}

/// Restores the web decoder inactivity expiry duration to its original value.
///
/// This should only be used in tests.
void debugRestoreWebDecoderExpireDuration() {
  _kWebDecoderExpireDuration = _kDefaultWebDecoderExpireDuration;
}

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

  void _debugCheckNotDisposed() {
    assert(!_isDisposed, 'Cannot use this image decoder. It has been disposed of.');
  }

  /// The index of the frame that will be decoded on the next call of [getNextFrame];
  int _nextFrameIndex = 0;

  /// Creating a new decoder is expensive, so we cache the decoder for reuse.
  ///
  /// This decoder is closed and the field is nulled out after some time of
  /// inactivity.
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

  final AlarmClock _cacheExpirationClock = AlarmClock(() => DateTime.now());

  Future<void> initialize() => _getOrCreateWebDecoder();

  Future<ImageDecoder> _getOrCreateWebDecoder() async {
    if (_cachedWebDecoder != null) {
      // Give the cached value some time for reuse, e.g. if the image is
      // currently animating.
      _cacheExpirationClock.datetime = DateTime.now().add(_kWebDecoderExpireDuration);
      return _cachedWebDecoder!;
    }

    // Null out the callback so the clock doesn't try to expire the decoder
    // while it's initializing. There's no way to tell how long the
    // initialization will take place. We just let it proceed at its own pace.
    _cacheExpirationClock.callback = null;
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
      _cachedWebDecoder = webDecoder;

      // Expire the decoder if it's not used for several seconds. If the image is
      // not animated, it could mean that the framework has cached the frame and
      // therefore doesn't need the decoder any more, or it could mean that the
      // widget is gone and it's time to collect resources associated with it.
      // If it's an animated image it means the animation has stopped, otherwise
      // we'd see calls to [getNextFrame] which would update the expiry date on
      // the decoder. If the animation is stopped for long enough, it's better
      // to collect resources. If and when the animation resumes, a new decoder
      // will be instantiated.
      _cacheExpirationClock.callback = () {
        _cachedWebDecoder?.close();
        _cachedWebDecoder = null;
        _cacheExpirationClock.callback = null;
      };
      _cacheExpirationClock.datetime = DateTime.now().add(_kWebDecoderExpireDuration);

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
    _debugCheckNotDisposed();
    final ImageDecoder webDecoder = await _getOrCreateWebDecoder();
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
  int get repetitionCount => delegate.frameCount;
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
