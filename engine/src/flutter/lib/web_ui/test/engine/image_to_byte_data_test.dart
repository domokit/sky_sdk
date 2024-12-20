// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpImplicitView();

  Future<Image> createTestImageByColor(Color color) async {
    final EnginePictureRecorder recorder = EnginePictureRecorder();
    final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 2, 2));
    canvas.drawColor(color, BlendMode.srcOver);
    final Picture testPicture = recorder.endRecording();
    final Image testImage = await testPicture.toImage(2, 2);
    return testImage;
  }

  test('Picture.toImage().toByteData()', () async {
    final Image testImage = await createTestImageByColor(const Color(0xFFCCDD00));

    final ByteData bytes = (await testImage.toByteData())!;
    expect(bytes.buffer.asUint32List(), <int>[0xFF00DDCC, 0xFF00DDCC, 0xFF00DDCC, 0xFF00DDCC]);

    final ByteData pngBytes = (await testImage.toByteData(format: ImageByteFormat.png))!;

    // PNG-encoding is browser-specific, but the header is standard. We only
    // test the header.
    final List<int> pngHeader = <int>[137, 80, 78, 71, 13, 10, 26, 10];
    expect(pngBytes.buffer.asUint8List().sublist(0, pngHeader.length), pngHeader);
  });

  test('Image.toByteData(format: ImageByteFormat.rawStraightRgba)', () async {
    final Image testImage = await createTestImageByColor(const Color(0xAAFFFF00));

    final ByteData bytes = (await testImage.toByteData(format: ImageByteFormat.rawStraightRgba))!;
    expect(bytes.buffer.asUint32List(), <int>[0xAA00FFFF, 0xAA00FFFF, 0xAA00FFFF, 0xAA00FFFF]);
  });
}
