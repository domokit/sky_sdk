// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:path/path.dart' as path;
import 'package:test/test.dart';

import 'impeller_enabled.dart';

void main() {
  test('Animation metadata', () async {
    Uint8List data = await _getSkiaResource('alphabetAnim.gif').readAsBytes();
    ui.Codec codec = await ui.instantiateImageCodec(data);
    expect(codec, isNotNull);
    expect(codec.frameCount, 13);
    expect(codec.repetitionCount, 0);
    codec.dispose();

    data = await _getSkiaResource('test640x479.gif').readAsBytes();
    codec = await ui.instantiateImageCodec(data);
    expect(codec.frameCount, 4);
    expect(codec.repetitionCount, -1);
  });

  test('Fails with invalid data', () async {
    final Uint8List data = Uint8List.fromList(<int>[1, 2, 3]);
    try {
      await ui.instantiateImageCodec(data);
      fail('exception not thrown');
    } on Exception catch (e) {
      expect(e.toString(), contains('Invalid image data'));
    }
  });

  test('getNextFrame fails with invalid data', () async {
    Uint8List data = await _getSkiaResource('flutter_logo.jpg').readAsBytes();
    data = Uint8List.view(data.buffer, 0, 4000);
    final ui.Codec codec = await ui.instantiateImageCodec(data);
    try {
      await codec.getNextFrame();
      fail('exception not thrown');
    } on Exception catch (e) {
      if (impellerEnabled) {
        expect(e.toString(), contains('Could not decompress image.'));
      } else {
        expect(e.toString(), contains('Codec failed'));
      }
    }
  });

  test('nextFrame', () async {
    final Uint8List data = await _getSkiaResource('test640x479.gif').readAsBytes();
    final ui.Codec codec = await ui.instantiateImageCodec(data);
    final List<List<int>> decodedFrameInfos = <List<int>>[];
    for (int i = 0; i < 5; i++) {
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      decodedFrameInfos.add(<int>[
        frameInfo.duration.inMilliseconds,
        frameInfo.image.width,
        frameInfo.image.height,
      ]);
    }
    expect(
      decodedFrameInfos,
      equals(<List<int>>[
        <int>[200, 640, 479],
        <int>[200, 640, 479],
        <int>[200, 640, 479],
        <int>[200, 640, 479],
        <int>[200, 640, 479],
      ]),
    );
  });

  test('non animated image', () async {
    final Uint8List data = await _getSkiaResource('baby_tux.png').readAsBytes();
    final ui.Codec codec = await ui.instantiateImageCodec(data);
    final List<List<int>> decodedFrameInfos = <List<int>>[];
    for (int i = 0; i < 2; i++) {
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      decodedFrameInfos.add(<int>[
        frameInfo.duration.inMilliseconds,
        frameInfo.image.width,
        frameInfo.image.height,
      ]);
    }
    expect(
      decodedFrameInfos,
      equals(<List<int>>[
        <int>[0, 240, 246],
        <int>[0, 240, 246],
      ]),
    );
  });

  test('with size', () async {
    final Uint8List data = await _getSkiaResource('baby_tux.png').readAsBytes();
    final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(data);
    final ui.Codec codec = await ui.instantiateImageCodecWithSize(
      buffer,
      getTargetSize: (int intrinsicWidth, int intrinsicHeight) {
        return ui.TargetImageSize(width: intrinsicWidth ~/ 2, height: intrinsicHeight ~/ 2);
      },
    );
    final List<List<int>> decodedFrameInfos = <List<int>>[];
    for (int i = 0; i < 2; i++) {
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      decodedFrameInfos.add(<int>[
        frameInfo.duration.inMilliseconds,
        frameInfo.image.width,
        frameInfo.image.height,
      ]);
    }
    expect(
      decodedFrameInfos,
      equals(<List<int>>[
        <int>[0, 120, 123],
        <int>[0, 120, 123],
      ]),
    );
  });

  test('disposed decoded image', () async {
    final Uint8List data = await _getSkiaResource('flutter_logo.jpg').readAsBytes();
    final ui.Codec codec = await ui.instantiateImageCodec(data);
    final ui.FrameInfo frameInfo = await codec.getNextFrame();
    expect(frameInfo.image, isNotNull);
    frameInfo.image.dispose();
    try {
      await codec.getNextFrame();
      fail('exception not thrown');
    } on Exception catch (e) {
      expect(e.toString(), contains('Decoded image has been disposed'));
    }
  });

  test('Animated gif can reuse across multiple frames', () async {
    // Regression test for b/271947267 and https://github.com/flutter/flutter/issues/122134

    final Uint8List data =
        File(
          path.join('flutter', 'lib', 'ui', 'fixtures', 'four_frame_with_reuse.gif'),
        ).readAsBytesSync();
    final ui.Codec codec = await ui.instantiateImageCodec(data);

    // Capture the final frame of animation. If we have not composited
    // correctly, it will be clipped strangely.
    late ui.FrameInfo frameInfo;
    for (int i = 0; i < 4; i++) {
      frameInfo = await codec.getNextFrame();
    }

    final ui.Image image = frameInfo.image;
    final ByteData imageData = (await image.toByteData(format: ui.ImageByteFormat.png))!;

    final String fileName =
        impellerEnabled
            ? 'impeller_four_frame_with_reuse_end.png'
            : 'four_frame_with_reuse_end.png';
    final Uint8List goldenData =
        File(path.join('flutter', 'lib', 'ui', 'fixtures', fileName)).readAsBytesSync();

    expect(imageData.buffer.asUint8List(), goldenData);
  });

  test('Animated webp can reuse across multiple frames', () async {
    // Regression test for https://github.com/flutter/flutter/issues/61150#issuecomment-679055858

    final Uint8List data =
        File(path.join('flutter', 'lib', 'ui', 'fixtures', 'heart.webp')).readAsBytesSync();
    final ui.Codec codec = await ui.instantiateImageCodec(data);

    // Capture the final frame of animation. If we have not composited
    // correctly, the hearts will be incorrectly repeated in the image.
    late ui.FrameInfo frameInfo;
    for (int i = 0; i < 69; i++) {
      frameInfo = await codec.getNextFrame();
    }

    final ui.Image image = frameInfo.image;
    final ByteData imageData = (await image.toByteData(format: ui.ImageByteFormat.png))!;

    final String fileName = impellerEnabled ? 'impeller_heart_end.png' : 'heart_end.png';

    final Uint8List goldenData =
        File(path.join('flutter', 'lib', 'ui', 'fixtures', fileName)).readAsBytesSync();

    expect(imageData.buffer.asUint8List(), goldenData);
  });

  test('Animated apng can reuse pre-pre-frame', () async {
    // https://github.com/flutter/engine/pull/42153

    final Uint8List data =
        File(
          path.join('flutter', 'lib', 'ui', 'fixtures', '2_dispose_op_restore_previous.apng'),
        ).readAsBytesSync();
    final ui.Codec codec = await ui.instantiateImageCodec(data);

    // Capture the 67,68,69 frames of animation and then compare the pixels.
    late ui.FrameInfo frameInfo;
    for (int i = 0; i < 70; i++) {
      frameInfo = await codec.getNextFrame();
      if (i >= 67) {
        final ui.Image image = frameInfo.image;
        final ByteData imageData = (await image.toByteData(format: ui.ImageByteFormat.png))!;

        final String fileName =
            impellerEnabled
                ? 'impeller_2_dispose_op_restore_previous.apng.$i.png'
                : '2_dispose_op_restore_previous.apng.$i.png';

        final Uint8List goldenData =
            File(path.join('flutter', 'lib', 'ui', 'fixtures', fileName)).readAsBytesSync();

        expect(imageData.buffer.asUint8List(), goldenData);
      }
    }
  });

  test('Animated apng alpha type handling', () async {
    final Uint8List data =
        File(
          path.join('flutter', 'lib', 'ui', 'fixtures', 'alpha_animated.apng'),
        ).readAsBytesSync();
    final ui.Codec codec = await ui.instantiateImageCodec(data);

    // The test image contains two frames of solid red.  The first has
    // alpha=0.2, and the second has alpha=0.6.
    ui.Image image = (await codec.getNextFrame()).image;
    ByteData imageData = (await image.toByteData())!;
    expect(imageData.getUint32(0), 0x33000033);
    image = (await codec.getNextFrame()).image;
    imageData = (await image.toByteData())!;
    expect(imageData.getUint32(0), 0x99000099);
  });

  test('Animated apng background color restore', () async {
    final Uint8List data =
        File(
          path.join('flutter', 'lib', 'ui', 'fixtures', 'dispose_op_background.apng'),
        ).readAsBytesSync();
    final ui.Codec codec = await ui.instantiateImageCodec(data);

    // First frame is solid red
    ui.Image image = (await codec.getNextFrame()).image;
    ByteData imageData = (await image.toByteData())!;
    expect(imageData.getUint32(0), 0xFF0000FF);

    // Third frame is blue in the lower right corner.
    await codec.getNextFrame();
    image = (await codec.getNextFrame()).image;
    imageData = (await image.toByteData())!;
    expect(imageData.getUint32(imageData.lengthInBytes - 4), 0x0000FFFF);

    // Fourth frame is transparent in the lower right corner
    image = (await codec.getNextFrame()).image;
    imageData = (await image.toByteData())!;
    expect(imageData.getUint32(imageData.lengthInBytes - 4), 0x00000000);
  });

  test('Animated apng frame decode does not crash with invalid destination region', () async {
    final Uint8List data =
        File(path.join('flutter', 'lib', 'ui', 'fixtures', 'out_of_bounds.apng')).readAsBytesSync();

    final ui.Codec codec = await ui.instantiateImageCodec(data);
    try {
      await codec.getNextFrame();
      fail('exception not thrown');
    } on Exception catch (e) {
      if (impellerEnabled) {
        expect(e.toString(), contains('Could not decompress image.'));
      } else {
        expect(e.toString(), contains('Codec failed'));
      }
    }
  });

  test(
    'Animated apng frame decode does not crash with invalid destination region and bounds wrapping',
    () async {
      final Uint8List data =
          File(
            path.join('flutter', 'lib', 'ui', 'fixtures', 'out_of_bounds_wrapping.apng'),
          ).readAsBytesSync();

      final ui.Codec codec = await ui.instantiateImageCodec(data);
      try {
        await codec.getNextFrame();
        fail('exception not thrown');
      } on Exception catch (e) {
        if (impellerEnabled) {
          expect(e.toString(), contains('Could not decompress image.'));
        } else {
          expect(e.toString(), contains('Codec failed'));
        }
      }
    },
  );
}

/// Returns a File handle to a file in the skia/resources directory.
File _getSkiaResource(String fileName) {
  // As Platform.script is not working for flutter_tester
  // (https://github.com/flutter/flutter/issues/12847), this is currently
  // assuming the curent working directory is engine/src.
  // This is fragile and should be changed once the Platform.script issue is
  // resolved.
  final String assetPath = path.join(
    'flutter',
    'third_party',
    'skia',
    'resources',
    'images',
    fileName,
  );
  return File(assetPath);
}
