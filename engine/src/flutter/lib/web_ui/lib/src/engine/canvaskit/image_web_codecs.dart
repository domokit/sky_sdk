// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Uses the `ImageDecoder` class supplied by the browser.
//
// See also:
//
//  * `image_wasm_codecs.dart`, which uses codecs supplied by the CanvasKit WASM bundle.

import 'dart:async';
import 'dart:convert' show base64;
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Image decoder backed by the browser's `ImageDecoder`.
class CkBrowserImageDecoder extends BrowserImageDecoder {
  CkBrowserImageDecoder._({
    required super.contentType,
    required super.dataSource,
    required super.debugSource,
  });

  static Future<CkBrowserImageDecoder> create({
    required Uint8List data,
    required String contentType,
    required String debugSource,
  }) async {
    final CkBrowserImageDecoder decoder = CkBrowserImageDecoder._(
      contentType: contentType,
      dataSource: data.toJS,
      debugSource: debugSource,
    );

    // Call once to initialize the decoder and populate late fields.
    await decoder.initialize();
    return decoder;
  }

  @override
  ui.Image generateImageFromVideoFrame(VideoFrame frame) {
    final SkImage? skImage = canvasKit.MakeLazyImageFromTextureSourceWithInfo(
      frame,
      SkPartialImageInfo(
        alphaType: canvasKit.AlphaType.Premul,
        colorType: canvasKit.ColorType.RGBA_8888,
        colorSpace: SkColorSpaceSRGB,
        width: frame.displayWidth,
        height: frame.displayHeight,
      ),
    );
    if (skImage == null) {
      throw ImageCodecException(
        "Failed to create image from pixel data decoded using the browser's ImageDecoder.",
      );
    }

    return CkImage(skImage, imageSource: VideoFrameImageSource(frame));
  }
}

Future<ByteData> readPixelsFromVideoFrame(VideoFrame videoFrame, ui.ImageByteFormat format) async {
  if (format == ui.ImageByteFormat.png) {
    final Uint8List png = await encodeDomImageSourceAsPng(
      videoFrame,
      videoFrame.displayWidth.toInt(),
      videoFrame.displayHeight.toInt(),
    );
    return png.buffer.asByteData();
  }

  final ByteBuffer pixels = await readVideoFramePixelsUnmodified(videoFrame);

  // Check if the pixels are already in the right format and if so, return the
  // original pixels without modification.
  if (_shouldReadPixelsUnmodified(videoFrame, format)) {
    return pixels.asByteData();
  }

  // At this point we know we want to read unencoded pixels, and that the video
  // frame is _not_ using the same format as the requested one.
  final bool isBgrx = videoFrame.format == 'BGRX';
  final bool isBgrFrame = videoFrame.format == 'BGRA' || isBgrx;
  if (isBgrFrame) {
    if (format == ui.ImageByteFormat.rawStraightRgba || isBgrx) {
      _bgrToStraightRgba(pixels, isBgrx);
      return pixels.asByteData();
    } else if (format == ui.ImageByteFormat.rawRgba) {
      _bgrToRawRgba(pixels);
      return pixels.asByteData();
    }
  }

  // Last resort, just return the original pixels.
  return pixels.asByteData();
}

Future<ByteData> readPixelsFromDomImageSource(
  DomCanvasImageSource imageSource,
  ui.ImageByteFormat format,
  int width,
  int height,
) async {
  if (format == ui.ImageByteFormat.png) {
    final Uint8List png = await encodeDomImageSourceAsPng(imageSource, width, height);
    return png.buffer.asByteData();
  }

  final ByteBuffer pixels = readDomImageSourcePixelsUnmodified(imageSource, width, height);
  return pixels.asByteData();
}

/// Mutates the [pixels], converting them from BGRX/BGRA to RGBA.
void _bgrToStraightRgba(ByteBuffer pixels, bool isBgrx) {
  final Uint8List pixelBytes = pixels.asUint8List();
  for (int i = 0; i < pixelBytes.length; i += 4) {
    // It seems even in little-endian machines the BGR_ pixels are encoded as
    // big-endian, i.e. the blue byte is written into the lowest byte in the
    // memory address space.
    final int b = pixelBytes[i];
    final int r = pixelBytes[i + 2];

    // So far the codec has reported 255 for the X component, so there's no
    // special treatment for alpha. This may need to change if we ever face
    // codecs that do something different.
    pixelBytes[i] = r;
    pixelBytes[i + 2] = b;
    if (isBgrx) {
      pixelBytes[i + 3] = 255;
    }
  }
}

/// Based on Chromium's SetRGBAPremultiply.
@pragma('dart2js:tryInline')
int _premultiply(int value, int alpha) {
  if (alpha == 255) {
    return value;
  }
  const int kRoundFractionControl = 257 * 128;
  return (value * alpha * 257 + kRoundFractionControl) >> 16;
}

/// Mutates the [pixels], converting them from BGRX/BGRA to RGBA with
/// premultiplied alpha.
void _bgrToRawRgba(ByteBuffer pixels) {
  final Uint8List pixelBytes = pixels.asUint8List();
  for (int i = 0; i < pixelBytes.length; i += 4) {
    final int a = pixelBytes[i + 3];
    final int r = _premultiply(pixelBytes[i + 2], a);
    final int g = _premultiply(pixelBytes[i + 1], a);
    final int b = _premultiply(pixelBytes[i], a);

    pixelBytes[i] = r;
    pixelBytes[i + 1] = g;
    pixelBytes[i + 2] = b;
  }
}

bool _shouldReadPixelsUnmodified(VideoFrame videoFrame, ui.ImageByteFormat format) {
  if (format == ui.ImageByteFormat.rawUnmodified) {
    return true;
  }

  // Do not convert if the requested format is RGBA and the video frame is
  // encoded as either RGBA or RGBX.
  final bool isRgbFrame = videoFrame.format == 'RGBA' || videoFrame.format == 'RGBX';
  return format == ui.ImageByteFormat.rawStraightRgba && isRgbFrame;
}

Future<ByteBuffer> readVideoFramePixelsUnmodified(VideoFrame videoFrame) async {
  final int size = videoFrame.allocationSize().toInt();

  // In dart2wasm, Uint8List is not the same as a JS Uint8Array. So we
  // explicitly construct the JS object here.
  final JSUint8Array destination = createUint8ArrayFromLength(size);
  final JSPromise<JSAny?> copyPromise = videoFrame.copyTo(destination);
  await promiseToFuture<void>(copyPromise);

  // In dart2wasm, `toDart` incurs a copy here. On JS backends, this is a
  // no-op.
  return destination.toDart.buffer;
}

ByteBuffer readDomImageSourcePixelsUnmodified(
  DomCanvasImageSource imageSource,
  int width,
  int height,
) {
  final DomCanvasElement htmlCanvas = createDomCanvasElement(width: width, height: height);
  final DomCanvasRenderingContext2D ctx =
      htmlCanvas.getContext('2d')! as DomCanvasRenderingContext2D;
  ctx.drawImage(imageSource, 0, 0);
  final DomImageData imageData = ctx.getImageData(0, 0, width, height);
  // Resize the canvas to 0x0 to cause the browser to reclaim its memory
  // eagerly.
  htmlCanvas.width = 0;
  htmlCanvas.height = 0;
  return imageData.data.buffer;
}

Future<Uint8List> encodeDomImageSourceAsPng(
  DomCanvasImageSource imageSource,
  int width,
  int height,
) async {
  final DomCanvasElement canvas = createDomCanvasElement(width: width, height: height);
  final DomCanvasRenderingContext2D ctx = canvas.context2D;
  ctx.drawImage(imageSource, 0, 0);
  final String pngBase64 = canvas.toDataURL().substring('data:image/png;base64,'.length);
  // Resize the canvas to 0x0 to cause the browser to reclaim its memory
  // eagerly.
  canvas.width = 0;
  canvas.height = 0;
  return base64.decode(pngBase64);
}
