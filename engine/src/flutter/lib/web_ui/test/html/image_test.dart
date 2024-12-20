// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Matcher listEqual(List<int> source, {int tolerance = 0}) {
  return predicate((List<int> target) {
    if (source.length != target.length) {
      return false;
    }
    for (int i = 0; i < source.length; i += 1) {
      if ((source[i] - target[i]).abs() > tolerance) {
        return false;
      }
    }
    return true;
  }, source.toString());
}

// Converts `rawPixels` into a list of bytes that represent raw pixels in rgba8888.
//
// Each element of `rawPixels` represents a bytes in order 0xRRGGBBAA, with
// pixel order Left to right, then top to bottom.
Uint8List _pixelsToBytes(List<int> rawPixels) {
  return Uint8List.fromList(<int>[
    for (final int pixel in rawPixels) ...<int>[
      (pixel >> 24) & 0xff, // r
      (pixel >> 16) & 0xff, // g
      (pixel >> 8) & 0xff, // b
      (pixel >> 0) & 0xff, // a
    ],
  ]);
}

Future<Image> _encodeToHtmlThenDecode(
  Uint8List rawBytes,
  int width,
  int height, {
  PixelFormat pixelFormat = PixelFormat.rgba8888,
}) async {
  final ImageDescriptor descriptor = ImageDescriptor.raw(
    await ImmutableBuffer.fromUint8List(rawBytes),
    width: width,
    height: height,
    pixelFormat: pixelFormat,
  );
  return (await (await descriptor.instantiateCodec()).getNextFrame()).image;
}

// This utility function detects how the current Web engine decodes pixel data.
//
// The HTML renderer uses the BMP format to display pixel data, but it used to
// use a wrong implementation. The bug has been fixed, but the fix breaks apps
// that had to provide incorrect data to work around this issue. This function
// is used in the migration guide to assist libraries that would like to run on
// both pre- and post-patch engines by testing the current behavior on a single
// pixel, making use the fact that the patch fixes the pixel order.
//
// The `format` argument is used for testing. In the actual code it should be
// replaced by `PixelFormat.rgba8888`.
//
// See also:
//
//  * Patch: https://github.com/flutter/engine/pull/29448
//  * Migration guide: https://docs.flutter.dev/release/breaking-changes/raw-images-on-web-uses-correct-origin-and-colors
Future<bool> rawImageUsesCorrectBehavior(PixelFormat format) async {
  final ImageDescriptor descriptor = ImageDescriptor.raw(
    await ImmutableBuffer.fromUint8List(Uint8List.fromList(<int>[0xED, 0, 0, 0xFF])),
    width: 1,
    height: 1,
    pixelFormat: format,
  );
  final Image image = (await (await descriptor.instantiateCodec()).getNextFrame()).image;
  final Uint8List resultPixels = Uint8List.sublistView(
    (await image.toByteData(format: ImageByteFormat.rawStraightRgba))!,
  );
  return resultPixels[0] == 0xED;
}

Future<void> testMain() async {
  test('Correctly encodes an opaque image', () async {
    // A 2x2 testing image without transparency.
    final Image sourceImage = await _encodeToHtmlThenDecode(
      _pixelsToBytes(<int>[0xFF0102FF, 0x04FE05FF, 0x0708FDFF, 0x0A0B0C00]),
      2,
      2,
    );
    final Uint8List actualPixels = Uint8List.sublistView(
      (await sourceImage.toByteData(format: ImageByteFormat.rawStraightRgba))!,
    );
    // The `benchmarkPixels` is identical to `sourceImage` except for the fully
    // transparent last pixel, whose channels are turned 0.
    final Uint8List benchmarkPixels = _pixelsToBytes(<int>[
      0xFF0102FF,
      0x04FE05FF,
      0x0708FDFF,
      0x00000000,
    ]);
    expect(actualPixels, listEqual(benchmarkPixels));
  });

  test('Correctly encodes an opaque image in bgra8888', () async {
    // A 2x2 testing image without transparency.
    final Image sourceImage = await _encodeToHtmlThenDecode(
      _pixelsToBytes(<int>[0xFF0102FF, 0x04FE05FF, 0x0708FDFF, 0x0A0B0C00]),
      2,
      2,
      pixelFormat: PixelFormat.bgra8888,
    );
    final Uint8List actualPixels = Uint8List.sublistView(
      (await sourceImage.toByteData(format: ImageByteFormat.rawStraightRgba))!,
    );
    // The `benchmarkPixels` is the same as `sourceImage` except that the R and
    // G channels are swapped and the fully transparent last pixel is turned 0.
    final Uint8List benchmarkPixels = _pixelsToBytes(<int>[
      0x0201FFFF,
      0x05FE04FF,
      0xFD0807FF,
      0x00000000,
    ]);
    expect(actualPixels, listEqual(benchmarkPixels));
  });

  test('Correctly encodes a transparent image', () async {
    // A 2x2 testing image with transparency.
    final Image sourceImage = await _encodeToHtmlThenDecode(
      _pixelsToBytes(<int>[0xFF800006, 0xFF800080, 0xFF8000C0, 0xFF8000FF]),
      2,
      2,
    );
    final Image blueBackground = await _encodeToHtmlThenDecode(
      _pixelsToBytes(<int>[0x0000FFFF, 0x0000FFFF, 0x0000FFFF, 0x0000FFFF]),
      2,
      2,
    );
    // The standard way of testing the raw bytes of `sourceImage` is to draw
    // the image onto a canvas and fetch its data (see HtmlImage.toByteData).
    // But here, we draw an opaque background first before drawing the image,
    // and test if the blended result is expected.
    //
    // This is because, if we only draw the `sourceImage`, the resulting pixels
    // will be slightly off from the raw pixels. The reason is unknown, but
    // very likely because the canvas.getImageData introduces rounding errors
    // if any pixels are left semi-transparent, which might be caused by
    // converting to and from pre-multiplied values. See
    // https://github.com/flutter/flutter/issues/92958 .
    final DomCanvasElement canvas =
        createDomCanvasElement()
          ..width = 2
          ..height = 2;
    final DomCanvasRenderingContext2D ctx = canvas.context2D;
    ctx.drawImage((blueBackground as HtmlImage).imgElement, 0, 0);
    ctx.drawImage((sourceImage as HtmlImage).imgElement, 0, 0);

    final DomImageData imageData = ctx.getImageData(0, 0, 2, 2);
    final List<int> actualPixels = imageData.data;

    final Uint8List benchmarkPixels = _pixelsToBytes(<int>[
      0x0603F9FF,
      0x80407FFF,
      0xC0603FFF,
      0xFF8000FF,
    ]);
    expect(actualPixels, listEqual(benchmarkPixels, tolerance: 1));
  });

  test('The behavior detector works correctly', () async {
    expect(await rawImageUsesCorrectBehavior(PixelFormat.rgba8888), true);
    expect(await rawImageUsesCorrectBehavior(PixelFormat.bgra8888), false);
  });
}
