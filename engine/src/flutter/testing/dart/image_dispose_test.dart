// Copyright 2019 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

import 'package:path/path.dart' as path;
import 'package:test/test.dart';

void main() {
  test('Handles are distinct', () async {
    final Uint8List bytes = await _readFile('2x2.png');
    final Codec codec = await instantiateImageCodec(bytes);
    final FrameInfo frame = await codec.getNextFrame();
    codec.dispose();

    expect(frame.image.width, 2);
    expect(frame.image.height, 2);
    final Image handle1 = frame.image.clone();
    expect(handle1.width, frame.image.width);
    expect(handle1.height, frame.image.height);

    final Image handle2 = handle1.clone();
    expect(handle1 != handle2, true);
    expect(handle1 != frame.image, true);
    expect(frame.image == frame.image, true);

    frame.image.dispose();
  });

  test('Canvas can paint image from handle and byte data from handle', () async {
    final Uint8List bytes = await _readFile('2x2.png');
    final Codec codec = await instantiateImageCodec(bytes);
    final FrameInfo frame = await codec.getNextFrame();
    codec.dispose();

    expect(frame.image.width, 2);
    expect(frame.image.height, 2);
    final Image handle1 = frame.image.clone();

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    const Rect rect = Rect.fromLTRB(0, 0, 2, 2);
    canvas.drawImage(handle1, Offset.zero, Paint());
    canvas.drawImageRect(handle1, rect, rect, Paint());
    canvas.drawImageNine(handle1, rect, rect, Paint());
    canvas.drawAtlas(handle1, <RSTransform>[], <Rect>[], <Color>[], BlendMode.src, rect, Paint());
    canvas.drawRawAtlas(
      handle1,
      Float32List(0),
      Float32List(0),
      Int32List(0),
      BlendMode.src,
      rect,
      Paint(),
    );

    final Picture picture = recorder.endRecording();

    final Image rasterizedImage = await picture.toImage(2, 2);
    final ByteData sourceData = (await frame.image.toByteData())!;
    final ByteData handleData = (await handle1.toByteData())!;
    final ByteData rasterizedData = (await rasterizedImage.toByteData())!;

    expect(sourceData.buffer.asUint8List(), equals(handleData.buffer.asUint8List()));
    expect(sourceData.buffer.asUint8List(), equals(rasterizedData.buffer.asUint8List()));
  });

  test('Records stack traces', () async {
    final Uint8List bytes = await _readFile('2x2.png');
    final Codec codec = await instantiateImageCodec(bytes);
    final FrameInfo frame = await codec.getNextFrame();
    codec.dispose();

    final Image handle1 = frame.image.clone();
    final Image handle2 = handle1.clone();

    List<StackTrace> stackTraces = frame.image.debugGetOpenHandleStackTraces()!;
    expect(stackTraces.length, 3);
    expect(stackTraces, equals(handle1.debugGetOpenHandleStackTraces()));

    handle1.dispose();
    stackTraces = frame.image.debugGetOpenHandleStackTraces()!;
    expect(stackTraces.length, 2);
    expect(stackTraces, equals(handle2.debugGetOpenHandleStackTraces()));

    handle2.dispose();
    stackTraces = frame.image.debugGetOpenHandleStackTraces()!;
    expect(stackTraces.length, 1);
    expect(stackTraces, equals(frame.image.debugGetOpenHandleStackTraces()));

    frame.image.dispose();
    expect(frame.image.debugGetOpenHandleStackTraces(), isEmpty);
  });

  test('Clones can be compared', () async {
    final Uint8List bytes = await _readFile('2x2.png');
    final Codec codec = await instantiateImageCodec(bytes);
    final FrameInfo frame = await codec.getNextFrame();
    codec.dispose();

    final Image handle1 = frame.image.clone();
    final Image handle2 = handle1.clone();

    expect(handle1.isCloneOf(handle2), true);
    expect(handle2.isCloneOf(handle1), true);
    expect(handle1.isCloneOf(frame.image), true);

    handle1.dispose();
    expect(handle1.isCloneOf(handle2), true);
    expect(handle2.isCloneOf(handle1), true);
    expect(handle1.isCloneOf(frame.image), true);

    final Codec codec2 = await instantiateImageCodec(bytes);
    final FrameInfo frame2 = await codec2.getNextFrame();
    codec2.dispose();

    expect(frame2.image.isCloneOf(frame.image), false);
  });

  test('debugDisposed works', () async {
    final Uint8List bytes = await _readFile('2x2.png');
    final Codec codec = await instantiateImageCodec(bytes);
    final FrameInfo frame = await codec.getNextFrame();
    codec.dispose();

    expect(frame.image.debugDisposed, false);

    frame.image.dispose();
    expect(frame.image.debugDisposed, true);
  });
}

Future<Uint8List> _readFile(String fileName) async {
  final File file = File(path.join('flutter', 'testing', 'resources', fileName));
  return file.readAsBytes();
}
