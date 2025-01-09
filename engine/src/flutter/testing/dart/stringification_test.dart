// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

final Uint8List imageData = Uint8List.fromList(<int>[
  // Small WebP file
  0x52,
  0x49,
  0x46,
  0x46,
  0x12,
  0x00,
  0x00,
  0x00,
  0x57,
  0x45,
  0x42,
  0x50,
  0x56,
  0x50,
  0x38,
  0x4c, // |RIFF....WEBPVP8L|
  0x06, 0x00, 0x00, 0x00, 0x2f, 0x41, 0x6c, 0x6f, 0x00, 0x6b, // |..../Alo.k|
]);

void main() {
  test('Stringification of native objects exposed in Dart', () async {
    expect(SemanticsUpdateBuilder().toString(), 'SemanticsUpdateBuilder');
    expect(SemanticsUpdateBuilder().build().toString(), 'SemanticsUpdate');
    expect(ParagraphBuilder(ParagraphStyle()).toString(), 'ParagraphBuilder');
    expect(ParagraphBuilder(ParagraphStyle()).build().toString(), 'Paragraph(dirty)');
    expect((await instantiateImageCodec(imageData)).toString(), 'Codec()');
    expect(Path().toString(), 'Path');
    final PictureRecorder recorder = PictureRecorder();
    expect(recorder.toString(), 'PictureRecorder(recording: false)');
    final Canvas canvas = Canvas(recorder);
    expect(recorder.toString(), 'PictureRecorder(recording: true)');
    expect(canvas.toString(), 'Canvas(recording: true)');
    final Picture picture = recorder.endRecording();
    expect(recorder.toString(), 'PictureRecorder(recording: false)');
    expect(canvas.toString(), 'Canvas(recording: false)');
    expect(picture.toString(), 'Picture');
    expect(
      ImageDescriptor.raw(
        await ImmutableBuffer.fromUint8List(Uint8List.fromList(<int>[0, 0, 0, 0])),
        width: 1,
        height: 1,
        pixelFormat: PixelFormat.rgba8888,
      ).toString(),
      'ImageDescriptor(width: 1, height: 1, bytes per pixel: 4)',
    );
    expect(SceneBuilder().toString(), 'SceneBuilder');
    expect(SceneBuilder().build().toString(), 'Scene');
  });
}
