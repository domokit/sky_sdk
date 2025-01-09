// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

import 'package:path/path.dart' as path;
import 'package:test/test.dart';

import 'impeller_enabled.dart';

const int _kWidth = 10;
const int _kRadius = 2;

const Color _kBlack = Color.fromRGBO(0, 0, 0, 1.0);
const Color _kGreen = Color.fromRGBO(0, 255, 0, 1.0);

void main() {
  test('decodeImageFromPixels float32', () async {
    if (impellerEnabled) {
      print('Disabled on Impeller - https://github.com/flutter/flutter/issues/135702');
      return;
    }
    const int width = 2;
    const int height = 2;
    final Float32List pixels = Float32List(width * height * 4);
    final List<List<double>> pixels2d = <List<double>>[
      <double>[1, 0, 0, 1],
      <double>[0, 1, 0, 1],
      <double>[0, 0, 1, 1],
      <double>[1, 1, 1, 0],
    ];
    int offset = 0;
    for (final List<double> color in pixels2d) {
      pixels[offset + 0] = color[0];
      pixels[offset + 1] = color[1];
      pixels[offset + 2] = color[2];
      pixels[offset + 3] = color[3];
      offset += 4;
    }

    final Completer<Image> completer = Completer<Image>();
    decodeImageFromPixels(Uint8List.view(pixels.buffer), width, height, PixelFormat.rgbaFloat32, (
      Image result,
    ) {
      completer.complete(result);
    });

    final Image image = await completer.future;
    final ByteData data = (await image.toByteData(format: ImageByteFormat.rawStraightRgba))!;
    final Uint32List readPixels = Uint32List.view(data.buffer);
    expect(width * height, readPixels.length);
    expect(readPixels[0], 0xff0000ff);
    expect(readPixels[1], 0xff00ff00);
    expect(readPixels[2], 0xffff0000);
    expect(readPixels[3], 0x00ffffff);
  });

  test('Image.toByteData RGBA format works with simple image', () async {
    final Image image = await Square4x4Image.image;
    final ByteData data = (await image.toByteData())!;
    expect(Uint8List.view(data.buffer), Square4x4Image.bytes);
  });

  test('Image.toByteData RGBA format converts grayscale images', () async {
    final Image image = await GrayscaleImage.load();
    final ByteData data = (await image.toByteData())!;
    final Uint8List bytes = data.buffer.asUint8List();
    expect(bytes, hasLength(16));
    expect(bytes, GrayscaleImage.bytesAsRgba);
  });

  test('Image.toByteData RGBA format works with transparent image', () async {
    final Image image = await TransparentImage.load();
    final ByteData data = (await image.toByteData())!;
    final Uint8List bytes = data.buffer.asUint8List();
    expect(bytes, hasLength(64));
    expect(bytes, TransparentImage.bytesAsPremultipliedRgba);
  });

  test('Image.toByteData Straight RGBA format works with transparent image', () async {
    final Image image = await TransparentImage.load();
    final ByteData data = (await image.toByteData(format: ImageByteFormat.rawStraightRgba))!;
    final Uint8List bytes = data.buffer.asUint8List();
    expect(bytes, hasLength(64));
    expect(bytes, TransparentImage.bytesAsStraightRgba);
  });

  test('Image.toByteData Unmodified format works with simple image', () async {
    final Image image = await Square4x4Image.image;
    final ByteData data = (await image.toByteData(format: ImageByteFormat.rawUnmodified))!;
    expect(Uint8List.view(data.buffer), Square4x4Image.bytes);
  });

  test('Image.toByteData Unmodified format works with grayscale images', () async {
    if (impellerEnabled) {
      print('Disabled on Impeller - https://github.com/flutter/flutter/issues/135706');
      return;
    }
    final Image image = await GrayscaleImage.load();
    final ByteData data = (await image.toByteData(format: ImageByteFormat.rawUnmodified))!;
    final Uint8List bytes = data.buffer.asUint8List();
    expect(bytes, hasLength(4));
    expect(bytes, GrayscaleImage.bytesUnmodified);
  });

  test('Image.toByteData PNG format works with simple image', () async {
    if (impellerEnabled) {
      print('Disabled on Impeller - https://github.com/flutter/flutter/issues/135706');
      return;
    }
    final Image image = await Square4x4Image.image;
    final ByteData data = (await image.toByteData(format: ImageByteFormat.png))!;
    final List<int> expected = await readFile('square.png');
    expect(Uint8List.view(data.buffer), expected);
  });

  test('Image.toByteData ExtendedRGBA128', () async {
    final Image image = await Square4x4Image.image;
    final ByteData data = (await image.toByteData(format: ImageByteFormat.rawExtendedRgba128))!;
    expect(image.width, _kWidth);
    expect(image.height, _kWidth);
    expect(data.lengthInBytes, _kWidth * _kWidth * 4 * 4);
    // Top-left pixel should be black.
    final Float32List floats = Float32List.view(data.buffer);
    expect(floats[0], 0.0);
    expect(floats[1], 0.0);
    expect(floats[2], 0.0);
    expect(floats[3], 1.0);
    expect(image.colorSpace, ColorSpace.sRGB);
  });
}

class Square4x4Image {
  Square4x4Image._();

  static Future<Image> get image async {
    final double width = _kWidth.toDouble();
    final double radius = _kRadius.toDouble();
    final double innerWidth = (_kWidth - 2 * _kRadius).toDouble();

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, Rect.fromLTWH(0.0, 0.0, width, width));

    final Paint black =
        Paint()
          ..strokeWidth = 1.0
          ..color = _kBlack;
    final Paint green =
        Paint()
          ..strokeWidth = 1.0
          ..color = _kGreen;

    canvas.drawRect(Rect.fromLTWH(0.0, 0.0, width, width), black);
    canvas.drawRect(Rect.fromLTWH(radius, radius, innerWidth, innerWidth), green);
    return recorder.endRecording().toImage(_kWidth, _kWidth);
  }

  static List<int> get bytes {
    const int bytesPerChannel = 4;
    final List<int> result = List<int>.filled(_kWidth * _kWidth * bytesPerChannel, 0);

    void fillWithColor(Color color, int min, int max) {
      for (int i = min; i < max; i++) {
        for (int j = min; j < max; j++) {
          final int offset = i * bytesPerChannel + j * _kWidth * bytesPerChannel;
          result[offset] = color.red;
          result[offset + 1] = color.green;
          result[offset + 2] = color.blue;
          result[offset + 3] = color.alpha;
        }
      }
    }

    fillWithColor(_kBlack, 0, _kWidth);
    fillWithColor(_kGreen, _kRadius, _kWidth - _kRadius);

    return result;
  }
}

class GrayscaleImage {
  GrayscaleImage._();

  static Future<Image> load() async {
    final Uint8List bytes = await readFile('2x2.png');
    final Completer<Image> completer = Completer<Image>();
    decodeImageFromList(bytes, (Image image) => completer.complete(image));
    return completer.future;
  }

  static List<int> get bytesAsRgba {
    return <int>[255, 255, 255, 255, 127, 127, 127, 255, 127, 127, 127, 255, 0, 0, 0, 255];
  }

  static List<int> get bytesUnmodified => <int>[255, 127, 127, 0];
}

class TransparentImage {
  TransparentImage._();

  static Future<Image> load() async {
    final Uint8List bytes = await readFile('transparent_image.png');
    final Completer<Image> completer = Completer<Image>();
    decodeImageFromList(bytes, (Image image) => completer.complete(image));
    return completer.future;
  }

  static List<int> get bytesAsPremultipliedRgba {
    return <int>[
      //First raw, solid colors
      255, 0, 0, 255, // red
      0, 255, 0, 255, // green
      0, 0, 255, 255, // blue
      136, 136, 136, 255, // grey
      //Second raw, 50% transparent
      127, 0, 0, 127, // red
      0, 127, 0, 127, // green
      0, 0, 127, 127, // blue
      67, 67, 67, 127, // grey
      //Third raw, 25% transparent
      63, 0, 0, 63, // red
      0, 63, 0, 63, // green
      0, 0, 63, 63, // blue
      33, 33, 33, 63, // grey
      //Fourth raw, transparent
      0, 0, 0, 0, // red
      0, 0, 0, 0, // green
      0, 0, 0, 0, // blue
      0, 0, 0, 0, // grey
    ];
  }

  static List<int> get bytesAsStraightRgba {
    return <int>[
      //First raw, solid colors
      255, 0, 0, 255, // red
      0, 255, 0, 255, // green
      0, 0, 255, 255, // blue
      136, 136, 136, 255, // grey
      //Second raw, 50% transparent
      255, 0, 0, 127, // red
      0, 255, 0, 127, // green
      0, 0, 255, 127, // blue
      135, 135, 135, 127, // grey
      //Third raw, 25% transparent
      255, 0, 0, 63, // red
      0, 255, 0, 63, // green
      0, 0, 255, 63, // blue
      134, 134, 134, 63, // grey
      //Fourth raw, transparent
      0, 0, 0, 0, // red
      0, 0, 0, 0, // green
      0, 0, 0, 0, // blue
      0, 0, 0, 0, // grey
    ];
  }
}

Future<Uint8List> readFile(String fileName) async {
  final File file = File(path.join('flutter', 'testing', 'resources', fileName));
  return file.readAsBytes();
}
